// Rerun recorder implementation (see logging/rerun.hpp). Development tool:
// uses the Rerun SDK, dynamic memory and the standard library freely; it is
// not part of the MISRA-compliant text logger (logger.hpp / logger.cpp).
#include <logging/logger.hpp>
#include <logging/rerun.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef ENABLE_RERUN
#include <rerun.hpp>
#endif

namespace logging
{
#ifdef ENABLE_RERUN
    namespace
    {
        std::mutex g_mutex;
        std::unique_ptr<rerun::RecordingStream> g_rec;

        // AutoSteer / AutoSpeed run on 1024×512 cropped frame.
        constexpr int NET_W = 1024;
        constexpr int NET_H = 512;

        // Log an OpenCV image inplace.
        // Colour conversion or encoding is needed as long as the Mat is contiguous.
        // A non-contiguous Mat (rare here) is made contiguous into `scratch` first.
        void log_image_inplace(const char* entity, const cv::Mat& im, cv::Mat& scratch)
        {
            if (im.empty()) return;
            const cv::Mat& m = im.isContinuous() ? im : (scratch = im.clone());
            const rerun::WidthHeight res(static_cast<uint32_t>(m.cols), static_cast<uint32_t>(m.rows));
            switch (m.channels())
            {
            case 3: g_rec->log(entity, rerun::Image(m.data, res, rerun::ColorModel::BGR));
                break;
            case 4: g_rec->log(entity, rerun::Image(m.data, res, rerun::ColorModel::BGRA));
                break;
            case 1: g_rec->log(entity, rerun::Image(m.data, res, rerun::ColorModel::L));
                break;
            default: break; // unsupported channel count — skip
            }
        }

        inline void log_scalar(const char* entity, double value)
        {
            g_rec->log(entity, rerun::Scalars(value));
        }

        // Per-section loggers (composed by Rerun::log_frame)

        void log_images(const cv::Mat& frame, const cv::Mat& warped, const cv::Mat& resized)
        {
            cv::Mat scratch;
            log_image_inplace("images/raw", frame, scratch); // raw input image
            log_image_inplace("images/warped_bev", warped, scratch); // warped BEV (AutoDrive)
            log_image_inplace("images/cropped", resized, scratch); // cropped+resized (AutoSteer/AutoSpeed)
        }

        void log_inference(const InferenceResult& r)
        {
            // AutoDrive raw predictions
            log_scalar("inference/auto_drive/dist_normalized", r.auto_drive.dist_normalized);
            log_scalar("inference/auto_drive/curvature_raw", r.auto_drive.curvature_raw);
            log_scalar("inference/auto_drive/flag_prob", r.auto_drive.flag_prob);

            // Latency
            log_scalar("inference/latency/wall_ms", r.wall_ms);
            log_scalar("inference/latency/pre_ms", r.pre_ms);
            log_scalar("inference/latency/ad_ms", r.ad_ms);
            log_scalar("inference/latency/as_ms", r.as_ms);
            log_scalar("inference/latency/asp_ms", r.asp_ms);

            // AutoSteer
            {
                const auto& xp = r.auto_steer.xp;
                const auto& hv = r.auto_steer.h_vector;
                const int n = static_cast<int>(xp.size());
                std::vector<rerun::components::Position2D> pts;
                pts.reserve(n);
                for (int i = 0; i < n; ++i)
                {
                    if (i < static_cast<int>(hv.size()) && hv[i] < 0.5f) continue; // low confidence
                    const float u = xp[i] * static_cast<float>(NET_W);
                    const float v = (static_cast<float>(i) / std::max(n - 1, 1)) * (NET_H - 1);
                    pts.emplace_back(u, v);
                }
                if (!pts.empty())
                {
                    g_rec->log("images/cropped/auto_steer_waypoints",
                               rerun::Points2D(pts).with_radii(3.0f).with_colors(rerun::Color(0, 200, 255)));
                }
                log_scalar("inference/auto_steer/num_waypoints", static_cast<double>(n));
            }

            // AutoSpeed detections
            {
                const auto& dets = r.auto_speed.detections;
                std::vector<rerun::datatypes::Vec2D> mins, sizes;
                std::vector<rerun::components::Text> labels;
                mins.reserve(dets.size());
                sizes.reserve(dets.size());
                labels.reserve(dets.size());
                for (const auto& d : dets)
                {
                    mins.emplace_back(d.x1, d.y1);
                    sizes.emplace_back(d.x2 - d.x1, d.y2 - d.y1);
                    labels.emplace_back("cls" + std::to_string(d.class_id));
                }
                if (!dets.empty())
                {
                    g_rec->log("images/cropped/auto_speed_detections",
                               rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                               .with_labels(labels)
                               .with_colors(rerun::Color(255, 80, 80)));
                }
                log_scalar("inference/auto_speed/num_detections", static_cast<double>(dets.size()));
            }

            // Fusion / Safety Guardian
            log_scalar("fusion/cipo/distance_m", r.cipo.distance_m); // in-path object distance
            log_scalar("fusion/cipo/velocity_ms", r.cipo.velocity_ms); // in-path object speed
            log_scalar("fusion/lateral/cte_m", r.lateral.cte_m); // cross-track error
            log_scalar("fusion/lateral/cte_rate_mps", r.lateral.cte_rate_mps); // cte rate
            log_scalar("fusion/lateral/yaw_rad", r.lateral.yaw_rad); // yaw error
            log_scalar("fusion/lateral/yaw_rate_rps", r.lateral.yaw_rate_rps); // yaw-error rate
            log_scalar("fusion/lateral/curvature", r.lateral.curvature); // curvature

            // BEV coords of filtered path, sampled from y = ax^2 + bx + c in
            // world/BEV frame (x = forward (m), y = lateral (m, left as +)).
            {
                const float a = r.lateral.path_a;
                const float b = r.lateral.path_b;
                const float c = r.lateral.path_c;
                float x0 = r.lateral.path_x_min_m;
                float x1 = r.lateral.path_x_max_m;
                if (!(x1 > x0))
                {
                    x0 = 2.f;
                    x1 = 40.f;
                } // fallback when extent unknown
                constexpr int N = 30;
                std::vector<rerun::datatypes::Vec2D> strip;
                strip.reserve(N);
                for (int i = 0; i < N; ++i)
                {
                    const float x = x0 + (x1 - x0) * (static_cast<float>(i) / (N - 1));
                    const float y = a * x * x + b * x + c;
                    strip.emplace_back(-y, -x);
                }
                g_rec->log("bev/filtered_path",
                           rerun::LineStrips2D(rerun::components::LineStrip2D(strip))
                           .with_colors(rerun::Color(255, 220, 0))
                           .with_radii(0.15f));
                g_rec->log("bev/ego",
                           rerun::Points2D({{0.0f, 0.0f}}).with_radii(0.4f).with_colors(rerun::Color(0, 255, 0)));
            }
        }

        void log_plan(const Plan& p)
        {
            // Desired tyre angle = first element of steering sequence [rad]
            log_scalar("planner/tyre_angle_rad", p.steering.empty() ? 0.0 : p.steering[0]);
            log_scalar("planner/acceleration", p.acceleration);

            if (!p.warnings.empty())
            {
                static const char* names[] = {"None", "FCW", "AEB", "LLDW", "RLDW"};
                std::string msg;
                for (const auto w : p.warnings)
                {
                    const int id = static_cast<int>(w);
                    if (id == 0) continue;
                    if (!msg.empty()) msg += ", ";
                    msg += (id >= 0 && id <= 4) ? names[id] : std::to_string(id);
                }
                if (!msg.empty())
                {
                    g_rec->log("planner/warnings",
                               rerun::TextLog(msg).with_level(rerun::TextLogLevel::Warning));
                }
            }
        }
    } // namespace

    void Rerun::init(const std::string& rrd_path, const std::string& app_id)
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        auto rec = std::make_unique<rerun::RecordingStream>(app_id);
        if (rec->save(rrd_path).is_err())
        {
            logging::error("[Rerun] Failed to open recording '{}' — logging disabled", rrd_path);
            return;
        }
        g_rec = std::move(rec);
        logging::info("[Rerun] Streaming recording to '{}'", rrd_path);
    }

    void Rerun::log_frame(std::uint64_t frame_id,
                          const cv::Mat& frame,
                          const cv::Mat& warped,
                          const cv::Mat& resized,
                          const InferenceResult& r,
                          const Plan& plan,
                          double ego_speed_ms,
                          const cv::Mat& viz)
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (!g_rec) return;

        // One timeline point for whole frame - every entity below shares it
        g_rec->set_time_sequence("frame", static_cast<std::int64_t>(frame_id));

        log_images(frame, warped, resized);
        log_inference(r);
        log_plan(plan);
        log_scalar("vehicle/ego_speed_ms", ego_speed_ms);
        if (!viz.empty())
        {
            cv::Mat scratch;
            log_image_inplace("images/visualization", viz, scratch);
        }
    }

    void Rerun::shutdown()
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_rec.reset(); // flushes and closes .rrd sink
    }

#else // ENABLE_RERUN not defined - compile every entry point to a no-op

    void Rerun::init(const std::string&, const std::string&)
    {
    }

    void Rerun::log_frame(std::uint64_t, const cv::Mat&, const cv::Mat&, const cv::Mat&,
                          const InferenceResult&, const Plan&,
                          double, const cv::Mat&)
    {
    }

    void Rerun::shutdown()
    {
    }

#endif // ENABLE_RERUN
} // namespace logging
