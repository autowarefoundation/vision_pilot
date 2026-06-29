#include <visualization/visualization.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>
#include <visualization/webrtc_stream.hpp>

#include "visualization/local_display.hpp"

namespace visualization {

namespace fs = std::filesystem;

// ─── Layout constants ─────────────────────────────────────────────────────────
static constexpr int   kNetW     = 1024;
static constexpr int   kNetH     = 512;
static constexpr int   kPathPts  = 64;
static constexpr float kDMax     = 150.f;
static constexpr int   kIconPx   = 80;   // icon display size (px)

// ─── Warped-pixel → world homography (copy of LongitudinalFusion H_) ──────────
// Input:  pixel (u, v) in the 1024×512 warped BEV image
// Output: world (x_fwd, y_lat) in metres; +x ahead, +y left of ego
static cv::Mat make_H_warp_to_world() {
    return (cv::Mat_<double>(3, 3) <<
         0.00209514907,  -0.000941721466, -9.24906396,
         0.00662758637,  -0.000352940531, -3.33396502,
         0.000120077371, -0.00411343505,   1.0);
}

// ─── Global state (single-threaded render loop — no mutex needed) ─────────────
static bool    g_h_ready        = false;
static cv::Mat g_H_warp2world;   // float32 3×3
static cv::Mat g_H_world2px;     // float32 3×3  (inverse of above)

static bool    g_ico_ready      = false;
static cv::Mat g_icon_brake;
static cv::Mat g_icon_collision;
static cv::Mat g_icon_rld;       // right lane departure  (BGRA, kIconPx²)
static cv::Mat g_icon_lld;       // left  lane departure  (BGRA, horizontally flipped)

// ─── Helper: alpha-composite BGRA icon centred at (cx, cy) onto BGR base ──────
static void paste_icon(cv::Mat& base, const cv::Mat& icon, int cx, int cy) {
    if (icon.empty() || base.empty()) return;

    const int x  = cx - icon.cols / 2;
    const int y  = cy - icon.rows / 2;
    const int x1 = std::max(x, 0);
    const int y1 = std::max(y, 0);
    const int x2 = std::min(x + icon.cols, base.cols);
    const int y2 = std::min(y + icon.rows, base.rows);
    if (x2 <= x1 || y2 <= y1) return;

    const cv::Rect src_rect(x1 - x, y1 - y, x2 - x1, y2 - y1);
    cv::Mat roi = base(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    cv::Mat src = icon(src_rect);

    if (src.channels() == 4) {
        std::vector<cv::Mat> ch;
        cv::split(src, ch);
        cv::Mat rgb, a_f, a3;
        cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, rgb);
        ch[3].convertTo(a_f, CV_32F, 1.0 / 255.0);
        cv::cvtColor(a_f, a3, cv::COLOR_GRAY2BGR);
        cv::Mat rf, bf;
        rgb.convertTo(rf, CV_32FC3, 1.0 / 255.0);
        roi.convertTo(bf, CV_32FC3, 1.0 / 255.0);
        cv::Mat blended = rf.mul(a3) + bf.mul(cv::Scalar(1.0, 1.0, 1.0) - a3);
        blended.convertTo(roi, CV_8UC3, 255.0);
    } else {
        src.copyTo(roi);
    }
}

// ─── Helper: semi-transparent filled rectangle ────────────────────────────────
static void fill_rect_alpha(cv::Mat& img, cv::Rect r, cv::Scalar color, double alpha) {
    r &= cv::Rect(0, 0, img.cols, img.rows);
    if (r.width <= 0 || r.height <= 0) return;
    cv::Mat roi = img(r);
    cv::Mat block(roi.size(), roi.type(), color);
    cv::addWeighted(block, alpha, roi, 1.0 - alpha, 0.0, roi);
}

// ─── Helper: icon loader ──────────────────────────────────────────────────────
static cv::Mat load_icon(const fs::path& p, int px) {
    if (!fs::exists(p)) return {};
    cv::Mat img = cv::imread(p.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) return {};
    cv::resize(img, img, cv::Size(px, px), 0, 0, cv::INTER_AREA);
    if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
    return img;
}

// ─── Helper: locate assets folder ────────────────────────────────────────────
static std::string resolve_icons_dir(const std::string& hint) {
    if (!hint.empty() && fs::is_directory(hint)) return hint;
    for (const char* c : {"../assets/icons", "assets/icons", "VisionPilot/assets/icons"}) {
        if (fs::is_directory(c)) return std::string(c);
    }
    return {};
}

// ─── Helper: pick gradient color pair from acceleration ───────────────────────
// Colors in BGR.  near = bottom of image (ego), far = vanishing point (horizon).
static void gradient_colors(double acc,
                             cv::Scalar& near_bgr, cv::Scalar& far_bgr) {
    if (acc < -0.5) {
        // Braking → greenish near, red far
        near_bgr = cv::Scalar(50,  200, 60);
        far_bgr  = cv::Scalar(0,   30,  220);
    } else if (acc > 0.5) {
        // Accelerating → blue gradient
        near_bgr = cv::Scalar(220, 120, 0);
        far_bgr  = cv::Scalar(255, 200, 0);
    } else {
        // Cruise → green near, blue far
        near_bgr = cv::Scalar(50,  200, 60);
        far_bgr  = cv::Scalar(210, 60,  0);
    }
}


// ─── Public: init_production_assets ──────────────────────────────────────────
void init_production_assets(const std::string& icons_dir) {
    if (!g_h_ready) {
        cv::Mat H64 = make_H_warp_to_world();
        H64.convertTo(g_H_warp2world, CV_32F);
        g_H_world2px = g_H_warp2world.inv();
        g_h_ready = true;
    }

    if (g_ico_ready) return;
    g_ico_ready = true;

    const std::string dir = resolve_icons_dir(icons_dir);
    if (dir.empty()) return;

    g_icon_brake     = load_icon(fs::path(dir) / "brake.png",               kIconPx);
    g_icon_collision = load_icon(fs::path(dir) / "collision.png",            kIconPx);
    g_icon_rld       = load_icon(fs::path(dir) / "right_lane_departure.png", kIconPx);

    if (!g_icon_rld.empty())
        cv::flip(g_icon_rld, g_icon_lld, 1);   // horizontal flip → left departure

}

// ─── Draw: fused-path corridor in world space (±1 m, gradient fill) ──────────
//
// Algorithm:
//   1. LateralFusion RANSAC gives polynomial y = a·x² + b·x + c in world frame.
//   2. Sample at 0.5 m steps over [path_x_min_m, path_x_max_m].
//   3. Left boundary = (x, y_center + 1 m), right = (x, y_center − 1 m).
//   4. Project both boundaries through H_world2px → warped image pixels.
//   5. Fill corridor as gradient trapezoids from far (small pixel-v) → near.
//
// The polynomial is already in metric world space, so there is no pixel→world
// projection step.  The ±1 m corridor stays exactly 2 m wide regardless of road
// curvature — it correctly fans out in image-space on the inner side of bends.
static void draw_path_corridor(cv::Mat& img, const ProductionView& view) {
    if (!view.path_valid) return;
    if (view.path_x_max_m <= view.path_x_min_m + 1.f) return;

    // Use the per-view H (from H_resized) when set; fall back to global warped H.
    const cv::Mat& H_w2px = (!view.H_world2px.empty()) ? view.H_world2px
                                                        : g_H_world2px;
    if (H_w2px.empty() && !g_h_ready) return;

    auto world_to_px = [&](float xw, float yw) -> cv::Point {
        std::vector<cv::Point2f> s = {cv::Point2f(xw, yw)}, d;
        cv::perspectiveTransform(s, d, H_w2px);
        return cv::Point(static_cast<int>(std::lround(d[0].x)),
                         static_cast<int>(std::lround(d[0].y)));
    };

    // ── Sample polynomial; build left/right boundary pixel arrays ─────────────
    const float a = view.path_a, b = view.path_b, c = view.path_c;
    const float x_start = std::max(0.5f, view.path_x_min_m);
    const float x_end   = view.path_x_max_m;

    constexpr float kStep = 0.5f;
    std::vector<cv::Point> lp, rp;

    for (float x = x_start; x <= x_end + 0.01f; x += kStep) {
        const float yc = a * x * x + b * x + c;
        lp.push_back(world_to_px(x, yc + 1.f));   // +1 m left of ego path
        rp.push_back(world_to_px(x, yc - 1.f));   // −1 m right of ego path
    }

    const int n = static_cast<int>(lp.size());
    if (n < 2) return;

    // ── Gradient fill: one trapezoid per segment, far→near ────────────────────
    // i=0 is farthest (x = x_start, small pixel v = high in image)
    // i=n−1 is nearest (x = x_end,  large pixel v = low in image)
    cv::Scalar near_bgr, far_bgr;
    gradient_colors(view.acceleration, near_bgr, far_bgr);

    cv::Mat overlay = img.clone();
    const cv::Rect bounds(0, 0, img.cols, img.rows);

    for (int i = 0; i < n - 1; ++i) {
        const float t = (n > 2) ? (static_cast<float>(i) + 0.5f) / (n - 1.f) : 0.5f;

        // t=0 → i=0 → x=x_min (nearest, bottom of image) → near_bgr (green)
        // t=1 → i=n-1 → x=x_max (farthest, horizon)    → far_bgr  (red/blue)
        const cv::Scalar color(
            near_bgr[0] * (1.f - t) + far_bgr[0] * t,
            near_bgr[1] * (1.f - t) + far_bgr[1] * t,
            near_bgr[2] * (1.f - t) + far_bgr[2] * t);

        const std::vector<cv::Point> quad = {lp[i], rp[i], rp[i + 1], lp[i + 1]};

        bool any_in = false;
        for (const auto& p : quad)
            if (bounds.contains(p)) { any_in = true; break; }
        if (!any_in) continue;

        cv::fillConvexPoly(overlay, quad, color, cv::LINE_AA);
    }

    cv::addWeighted(overlay, 0.45, img, 0.55, 0.0, img);
}

// ─── Draw: AutoSpeed bounding boxes + CIPO distance label ─────────────────────
static void draw_cipo_boxes(cv::Mat& img, const ProductionView& view) {
    if (view.detections.empty()) return;

    // Semi-transparent fill pass
    cv::Mat overlay = img.clone();
    for (const auto& d : view.detections) {
        const cv::Scalar fill = (d.class_id == 1)
            ? cv::Scalar(30, 30, 200)    // red  — same-lane (Level 1)
            : cv::Scalar(200, 80,  30);  // blue — adjacent  (Level 2)
        cv::rectangle(overlay,
            cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
            cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
            fill, -1);
    }
    cv::addWeighted(overlay, 0.32, img, 0.68, 0.0, img);

    // Outline pass (hard edges on top)
    for (const auto& d : view.detections) {
        const cv::Scalar outline = (d.class_id == 1)
            ? cv::Scalar(0,   0,   240)
            : cv::Scalar(240, 110,  30);
        cv::rectangle(img,
            cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
            cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
            outline, 2, cv::LINE_AA);
    }

    // ── Distance label on the closest L1 (most centred) ──────────────────────
    if (view.cipo.valid && view.cipo.distance_m < kDMax) {
        const ProductionView::BBox* best = nullptr;
        float best_cx_err = 1e9f;
        for (const auto& d : view.detections) {
            if (d.class_id != 1) continue;
            const float cx_err = std::abs((d.x1 + d.x2) * 0.5f - kNetW * 0.5f);
            if (cx_err < best_cx_err) { best_cx_err = cx_err; best = &d; }
        }

        if (best) {
            char lbl[32];
            std::snprintf(lbl, sizeof(lbl), "%.0fm",
                static_cast<double>(view.cipo.distance_m));

            const int tx = static_cast<int>((best->x1 + best->x2) * 0.5f);
            const int ty = static_cast<int>(best->y1) - 6;
            if (ty > 12) {
                int bl = 0;
                const cv::Size ts = cv::getTextSize(
                    lbl, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &bl);
                cv::rectangle(img,
                    cv::Point(tx - ts.width / 2 - 6, ty - ts.height - 6),
                    cv::Point(tx + ts.width / 2 + 6, ty + 4),
                    cv::Scalar(20, 20, 20), -1);
                cv::putText(img, lbl,
                    cv::Point(tx - ts.width / 2, ty),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }
    }
}

// ─── Draw: ego speed readout (top-centre) ────────────────────────────────────
static void draw_speed(cv::Mat& img, double speed_ms) {
    char num_buf[32];
    std::snprintf(num_buf, sizeof(num_buf), "%.0f", speed_ms * 2.23694);
    const char* unit_buf = "mph";

    int bl = 0;
    const cv::Size num_ts  = cv::getTextSize(num_buf,  cv::FONT_HERSHEY_DUPLEX, 1.1, 2, &bl);
    const cv::Size unit_ts = cv::getTextSize(unit_buf, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, &bl);

    const int num_tx  = (img.cols - num_ts.width)  / 2;
    const int unit_tx = (img.cols - unit_ts.width) / 2;
    const int num_ty  = 44;
    const int unit_ty = num_ty + num_ts.height -4 ;

    cv::Mat overlay;
    img.copyTo(overlay);

    cv::putText(overlay, num_buf, cv::Point(num_tx + 2, num_ty + 2),
                cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(overlay, num_buf, cv::Point(num_tx, num_ty),
                cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    cv::putText(overlay, unit_buf, cv::Point(unit_tx + 2, unit_ty + 2),
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(overlay, unit_buf, cv::Point(unit_tx, unit_ty),
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::addWeighted(overlay, 1.0, img, 0.5, 0, img);
}

// ─── Draw: alert overlays + icons ────────────────────────────────────────────
static bool has_warning(const std::vector<uint8_t>& ws, uint8_t v) {
    for (auto w : ws) if (w == v) return true;
    return false;
}

static void draw_alerts(cv::Mat& img, const ProductionView& view) {
    const int W  = img.cols;
    const int H  = img.rows;
    const int sw = W * 28 / 100;  // side strip width for LDW

    // ── FCW=1, AEB=2, LLDW=3, RLDW=4 (mirror of Warning enum) ───────────────
    const bool fcw  = has_warning(view.warnings, 1);
    const bool aeb  = has_warning(view.warnings, 2);
    const bool lldw = has_warning(view.warnings, 3);
    const bool rldw = has_warning(view.warnings, 4);

    // Lane departure strips ────────────────────────────────────────────────────
    static const cv::Scalar kOrange{0, 120, 255};
    static const cv::Scalar kWhite {255, 255, 255};

    if (lldw) {
        fill_rect_alpha(img, cv::Rect(0, 0, sw, H), kOrange, 0.45);
        paste_icon(img, g_icon_lld, sw / 2, H / 2 - 20);
        cv::putText(img, "Left Lane",
                    cv::Point(10, H / 2 + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, kWhite, 2, cv::LINE_AA);
        cv::putText(img, "Departure",
                    cv::Point(10, H / 2 + 52),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, kWhite, 2, cv::LINE_AA);
    }
    if (rldw) {
        fill_rect_alpha(img, cv::Rect(W - sw, 0, sw, H), kOrange, 0.45);
        paste_icon(img, g_icon_rld, W - sw / 2, H / 2 - 20);
        cv::putText(img, "Right Lane",
                    cv::Point(W - sw + 8, H / 2 + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, kWhite, 2, cv::LINE_AA);
        cv::putText(img, "Departure",
                    cv::Point(W - sw + 8, H / 2 + 52),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, kWhite, 2, cv::LINE_AA);
    }

    // Collision / braking overlay (bottom strip) ──────────────────────────────
    if (aeb) {
        const int bh = H * 38 / 100;
        fill_rect_alpha(img, cv::Rect(0, H - bh, W, bh), cv::Scalar(0, 0, 180), 0.50);
        paste_icon(img, g_icon_brake, W / 2, H - bh / 2 - 10);
        int bl = 0;
        cv::Size ts = cv::getTextSize(
            "Emergency Braking", cv::FONT_HERSHEY_SIMPLEX, 0.80, 2, &bl);
        cv::putText(img, "Emergency Braking",
                    cv::Point((W - ts.width) / 2, H - 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.80, kWhite, 2, cv::LINE_AA);
    } else if (fcw) {
        const int bh = H * 28 / 100;
        fill_rect_alpha(img, cv::Rect(0, H - bh, W, bh), cv::Scalar(0, 130, 230), 0.45);
        paste_icon(img, g_icon_collision, W / 2, H - bh / 2 - 10);
        int bl = 0;
        cv::Size ts = cv::getTextSize(
            "Collision Alert", cv::FONT_HERSHEY_SIMPLEX, 0.75, 2, &bl);
        cv::putText(img, "Collision Alert",
                    cv::Point((W - ts.width) / 2, H - 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, kWhite, 2, cv::LINE_AA);
    }
}

// ─── Internal: draw production UI onto frame and show window ───────────────────
static cv::Mat draw_production_frame(cv::Mat& frame, const ProductionView& view) {
    if (frame.empty()) return cv::Mat();

    if (!g_h_ready || !g_ico_ready)
        init_production_assets(view.icons_dir);

    cv::Mat display_frame = frame.clone();

    // Render order: path → boxes → alerts → speed → CIPO text
    draw_path_corridor(display_frame, view);
    draw_cipo_boxes(display_frame, view);
    draw_alerts(display_frame, view);
    draw_speed(display_frame, view.ego_speed_ms);

    return display_frame;
}

// ─── ProductionView ──────────────────────────────────────────────────────────

ProductionView ProductionView::from(
    const visionpilot::models::InferenceFrameResult& r,
    const Plan& plan,
    double ego_speed_ms,
    const cv::Mat& H_resized)
{
    ProductionView pv;
    pv.ego_speed_ms = ego_speed_ms;
    pv.acceleration = plan.acceleration;

    // H_resized maps resized-px → world.  Invert to get world → display-px.
    if (!H_resized.empty()) {
        cv::Mat H64;
        H_resized.convertTo(H64, CV_64F);
        cv::Mat H64_inv = H64.inv();          // MatExpr → cv::Mat
        H64_inv.convertTo(pv.H_world2px, CV_32F);
    }

    pv.warnings.reserve(plan.warnings.size());
    for (const auto w : plan.warnings)
        pv.warnings.push_back(static_cast<uint8_t>(w));

    if (r.lateral.path_valid) {
        pv.path_a       = r.lateral.path_a;
        pv.path_b       = r.lateral.path_b;
        pv.path_c       = r.lateral.path_c;
        pv.path_x_min_m = r.lateral.path_x_min_m;
        pv.path_x_max_m = r.lateral.path_x_max_m;
        pv.path_valid   = true;
    }

    pv.detections.reserve(r.auto_speed.detections.size());
    for (const auto& d : r.auto_speed.detections) {
        pv.detections.push_back({d.x1, d.y1, d.x2, d.y2, d.score, d.class_id});
    }

    pv.cipo = {
        r.cipo.valid,
        r.cipo.distance_m,
        r.cipo.velocity_ms,
        r.cipo.cipo_raw_found,
        r.cipo.cipo_raw_dist_m,
        r.cipo.cut_in_detected,
    };

    return pv;
}

cv::Mat ProductionView::render(cv::Mat& frame) const {
    return draw_production_frame(frame, *this);
}

cv::Mat ProductionView::visualize(
    cv::Mat& frame,
    const visionpilot::models::InferenceFrameResult& result,
    const Plan& plan,
    double ego_speed_ms,
    const cv::Mat& H_resized)
{
    return from(result, plan, ego_speed_ms, H_resized).render(frame);
}

// ─── Plain frame display (warm-up / no inference yet) ────────────────────────
bool show_frame(const cv::Mat& frame, const std::string& window_name) {
    if (frame.empty()) return false;

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, frame.cols, frame.rows);
    cv::imshow(window_name, frame);
    cv::waitKey(1);
    return true;
}

void close_windows() {
    cv::destroyAllWindows();
}

Visualization::Visualization(Config cfg)
{
    if (cfg.webrtc_on)
    {
        visual_interface = std::make_unique<WebRTCStreamer>();
        static_cast<WebRTCStreamer*>(visual_interface.get())->init(cfg.webrtc_port);
    } else
    {
        visual_interface = std::make_unique<LocalDisplay>();
    }
}


cv::Mat Visualization::build_frame(cv::Mat& frame,
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms,
        const cv::Mat& H_resized)
{
    return ProductionView::from(result, plan, ego_speed_ms, H_resized).render(frame);
}

bool Visualization::render_frame(const cv::Mat& display_frame)
{
    return visual_interface -> render_frame(display_frame);
}

}  // namespace visualization
