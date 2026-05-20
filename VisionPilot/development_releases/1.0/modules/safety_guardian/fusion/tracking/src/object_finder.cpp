#include <tracking/object_finder.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace visionpilot::tracking {

// ─── Lightweight YAML parser for the homography file ─────────────────────────
// Handles the structured format produced by OpenCV FileStorage or manual YAML:
//   H:
//     rows: 3
//     cols: 3
//     data:
//       - v0
//       - v1
//       ...
static cv::Mat load_homography_yaml(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("ObjectFinder: cannot open homography YAML: " + path);

    std::vector<double> data;
    bool in_data = false;

    std::string line;
    while (std::getline(f, line) && data.size() < 9) {
        // Detect the "data:" key
        if (line.find("data:") != std::string::npos) {
            in_data = true;
            // data might be on the same line as an inline sequence: "data: [a, b, ...]"
            auto lb = line.find('[');
            if (lb != std::string::npos) {
                auto rb = line.find(']', lb);
                std::string seq = line.substr(lb + 1, rb - lb - 1);
                std::replace(seq.begin(), seq.end(), ',', ' ');
                std::istringstream ss(seq);
                double v;
                while (ss >> v) data.push_back(v);
                break;
            }
            continue;
        }
        if (in_data) {
            auto dash = line.find('-');
            if (dash == std::string::npos) continue;
            try {
                data.push_back(std::stod(line.substr(dash + 1)));
            } catch (...) {}
        }
    }

    if (data.size() != 9)
        throw std::runtime_error(
            "ObjectFinder: homography YAML must contain exactly 9 values, got " +
            std::to_string(data.size()) + " in " + path);

    cv::Mat H64(3, 3, CV_64F, data.data());
    cv::Mat H32;
    H64.convertTo(H32, CV_32F);
    return H32.clone();
}

// ─── Construction ─────────────────────────────────────────────────────────────

ObjectFinder::ObjectFinder(const std::string& homography_yaml,
                           int image_w, int image_h, bool debug)
    : image_w_(image_w), image_h_(image_h), debug_(debug)
{
    H_ = load_homography_yaml(homography_yaml);
    printf("[ObjectFinder] Homography loaded from %s  image=%dx%d\n",
           homography_yaml.c_str(), image_w_, image_h_);
}

// ─── Homography helpers ───────────────────────────────────────────────────────

cv::Point2f ObjectFinder::image_to_world(const cv::Point2f& pt) const
{
    std::vector<cv::Point2f> src = {pt}, dst;
    cv::perspectiveTransform(src, dst, H_);
    return dst[0];
}

float ObjectFinder::distance(const cv::Point2f& wp) const
{
    return std::sqrt(wp.x * wp.x + wp.y * wp.y);
}

// ─── Data association ─────────────────────────────────────────────────────────

std::vector<std::pair<int,int>> ObjectFinder::associate(
    const std::vector<visionpilot::models::Detection>& dets) const
{
    std::vector<std::pair<int,int>> assoc;
    std::vector<bool> det_used(dets.size(), false);
    std::vector<bool> trk_used(prev_.size(), false);

    for (std::size_t di = 0; di < dets.size(); ++di) {
        if (!should_track(dets[di].class_id)) continue;

        cv::Rect det_bbox(
            static_cast<int>(dets[di].x1),
            static_cast<int>(dets[di].y1),
            static_cast<int>(dets[di].x2 - dets[di].x1),
            static_cast<int>(dets[di].y2 - dets[di].y1));

        int   best_ti = -1;
        float best_s  = 0.f;

        for (std::size_t ti = 0; ti < prev_.size(); ++ti) {
            if (trk_used[ti] || prev_[ti].class_id != dets[di].class_id) continue;
            float s = TrackingUtils::calculateMatchingScore(
                det_bbox, prev_[ti].bbox, image_w_, image_h_);
            if (s > match_threshold_ && s > best_s) {
                best_s = s; best_ti = static_cast<int>(ti);
            }
        }

        assoc.push_back({static_cast<int>(di), best_ti});
        det_used[di] = true;
        if (best_ti >= 0) trk_used[static_cast<std::size_t>(best_ti)] = true;
    }
    return assoc;
}

// ─── Main entry point ─────────────────────────────────────────────────────────

TrackingResult ObjectFinder::update_and_get_cipo(
    const std::vector<visionpilot::models::Detection>& dets,
    const cv::Mat& frame)
{
    auto now = std::chrono::steady_clock::now();
    cut_in_detected_ = false;
    kalman_reset_    = false;

    auto assoc = associate(dets);
    std::vector<TrackedObject> new_tracks;

    // ─ Process matched and new detections ─────────────────────────────────────
    for (const auto& [di, ti] : assoc) {
        const auto& det = dets[static_cast<std::size_t>(di)];

        cv::Rect bbox(
            static_cast<int>(det.x1),
            static_cast<int>(det.y1),
            static_cast<int>(det.x2 - det.x1),
            static_cast<int>(det.y2 - det.y1));

        cv::Point2f bc      = TrackingUtils::getBottomCenter(bbox);
        cv::Point2f world   = image_to_world(bc);
        float       meas_d  = distance(world);

        TrackedObject obj;

        if (ti >= 0) {
            // Update existing track
            obj = prev_[static_cast<std::size_t>(ti)];
            ++obj.frames_tracked;
            obj.frames_unmatched = 0;

            float dt = std::chrono::duration<float>(now - obj.last_update_time).count();
            obj.kalman.predict(dt);
            obj.kalman.update(meas_d);
            obj.distance_m  = obj.kalman.position();
            obj.velocity_ms = obj.kalman.velocity();
        } else {
            // New track
            obj.track_id       = next_id_++;
            obj.class_id       = det.class_id;
            obj.frames_tracked = 1;
            obj.frames_unmatched = 0;
            obj.kalman.initialize(meas_d);
            obj.distance_m  = meas_d;
            obj.velocity_ms = 0.f;
        }

        obj.bbox         = bbox;
        obj.confidence   = det.score;
        obj.last_update_time = now;
        new_tracks.push_back(obj);
    }

    // ─ Keep unmatched tracks alive for up to max_unmatched_ frames ────────────
    for (std::size_t ti = 0; ti < prev_.size(); ++ti) {
        bool matched = false;
        for (const auto& [di, mti] : assoc)
            if (mti == static_cast<int>(ti)) { matched = true; break; }

        if (!matched) {
            auto t = prev_[ti];
            if (++t.frames_unmatched <= max_unmatched_)
                new_tracks.push_back(t);
        }
    }

    tracks_ = new_tracks;
    prev_   = tracks_;

    // ─ CIPO selection ─────────────────────────────────────────────────────────
    // Find the closest object at each level, then take whichever is closer.
    auto find_closest = [&](int cls) -> int {
        int   idx = -1;
        float mn  = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const auto& o = tracks_[i];
            if (o.class_id == cls && o.distance_m > 0.f && o.distance_m < mn) {
                mn = o.distance_m; idx = static_cast<int>(i);
            }
        }
        return idx;
    };

    int ci1 = find_closest(1);
    int ci2 = find_closest(2);
    int ci  = -1;
    if (ci1 >= 0 && ci2 >= 0)
        ci = (tracks_[static_cast<std::size_t>(ci1)].distance_m <=
              tracks_[static_cast<std::size_t>(ci2)].distance_m) ? ci1 : ci2;
    else if (ci1 >= 0) ci = ci1;
    else               ci = ci2;

    CIPOInfo cipo;
    if (ci >= 0) {
        auto& mc       = tracks_[static_cast<std::size_t>(ci)];
        cipo.exists    = true;
        cipo.track_id  = mc.track_id;
        cipo.class_id  = mc.class_id;
        cipo.distance_m = mc.distance_m;

        // ─ CIPO history & cut-in detection ────────────────────────────────────
        CIPOSnapshot snap;
        snap.track_id   = mc.track_id;
        snap.class_id   = mc.class_id;
        snap.bbox       = mc.bbox;
        snap.distance_m = mc.distance_m;
        snap.velocity_ms = mc.velocity_ms;
        snap.timestamp  = mc.last_update_time;
        snap.frame_crop = FeatureMatchingUtils::extractSafeCrop(frame, mc.bbox);
        history_.push(snap);

        if (history_.did_change()) {
            const auto* prev_snap = history_.previous();
            const auto* curr_snap = history_.latest();

            if (prev_snap && curr_snap &&
                !prev_snap->frame_crop.empty() &&
                !curr_snap->frame_crop.empty()) {

                bool same = FeatureMatchingUtils::areSameObject(
                    prev_snap->frame_crop,
                    cv::Rect(0,0, prev_snap->frame_crop.cols, prev_snap->frame_crop.rows),
                    curr_snap->frame_crop,
                    cv::Rect(0,0, curr_snap->frame_crop.cols, curr_snap->frame_crop.rows),
                    feat_threshold_);

                if (same) {
                    // Model confused L1↔L2 — transfer Kalman state
                    for (auto& po : prev_) {
                        if (po.track_id == prev_snap->track_id) {
                            mc.kalman = po.kalman; break;
                        }
                    }
                } else {
                    // Real cut-in — reset Kalman
                    cut_in_detected_ = true;
                    kalman_reset_    = true;
                    mc.kalman.reset();
                    mc.kalman.initialize(mc.distance_m);
                    mc.velocity_ms = 0.f;
                }
            }
        }

        cipo.velocity_ms = mc.velocity_ms;

        if (debug_) {
            printf("[ObjectFinder] CIPO Track=%d class=%d dist=%.1fm vel=%.2fm/s\n",
                   cipo.track_id, cipo.class_id, cipo.distance_m, cipo.velocity_ms);
        }
    }

    TrackingResult r;
    r.tracked_objects = tracks_;
    r.cipo            = cipo;
    r.cut_in_detected = cut_in_detected_;
    r.kalman_reset    = kalman_reset_;
    return r;
}

}  // namespace visionpilot::tracking
