#ifndef VISIONPILOT_LOGGING_RERUN_HPP
#define VISIONPILOT_LOGGING_RERUN_HPP

// =============================================================================
// rerun.hpp
//
// Rerun recorder - streams per-frame data directly to a native .rrd recording
// through the Rerun C++ SDK. Development / debugging tool; independent of the
// text logger (logging/logger.hpp).
//
// Images are logged zero-copy - the raw OpenCV BGR buffer is borrowed in place,
// no per-frame allocation, colour conversion or PNG encoding takes place.
//
// When built without the SDK (ENABLE_RERUN off), every call is a cheap no-op.
//
// Usage:
//   #include <logging/rerun.hpp>
//   logging::Rerun::init("visionpilot.rrd");
//   logging::Rerun::log_frame(frame_id, frame, warped, resized,
//                             logging::InferenceFrameResult{inference_result},
//                             logging::Plan{plan.acceleration, plan.steering, plan.warnings},
//                             ego_speed_ms, viz_image);
//   logging::Rerun::shutdown();
//
// The logging:: data structures below hold only the fields the recorder reads.
// Their template constructors copy from any source type with the same member
// names, so this module does not depend on the models, fusion or planning
// modules: the source type is known only where the constructor is called.
// =============================================================================

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <opencv2/core/mat.hpp>   // defines cv::Mat; lighter than opencv2/core.hpp

namespace logging
{
    enum class Warning : std::uint8_t
    {
        None = 0U,
        FCW = 1U, // Forward collision warning
        AEB = 2U, // Automatic Emergency Braking
        LLDW = 3U, // Left lane departure warning
        RLDW = 4U // Right lane departure warning
    };

    /// Maps a numeric warning ID to logging::Warning; unknown IDs map to None.
    inline Warning toWarning(std::uint32_t const value) noexcept
    {
        Warning result{Warning::None};
        switch (value)
        {
        case 1U:
            {
                result = Warning::FCW;
                break;
            }
        case 2U:
            {
                result = Warning::AEB;
                break;
            }
        case 3U:
            {
                result = Warning::LLDW;
                break;
            }
        case 4U:
            {
                result = Warning::RLDW;
                break;
            }
        default:
            {
                result = Warning::None;
                break;
            }
        }
        return result;
    }

    struct Plan
    {
        double acceleration{0.0};
        std::vector<double> steering;
        std::vector<Warning> warnings;

        Plan() = default;

        // Build logging::Plan from planner Plan
        template <typename Warning>
        Plan(double const accel,
             std::vector<double> const& steer,
             std::vector<Warning> const& sourceWarnings) :
            acceleration{accel}, steering{steer}, warnings{}
        {
            static_assert(std::is_enum<Warning>::value, "warnings must be an enum");
            warnings.reserve(sourceWarnings.size());
            for (Warning const w : sourceWarnings)
            {
                warnings.push_back(toWarning(static_cast<std::underlying_type_t<Warning>>(w)));
            }
        }
    };

    struct AutoDriveOutput
    {
        float dist_normalized{0.0F};
        float curvature_raw{0.0F};
        float flag_prob{0.0F};

        AutoDriveOutput() = default;

        template <typename Source>
        explicit AutoDriveOutput(Source const& s) :
            dist_normalized{s.dist_normalized}, curvature_raw{s.curvature_raw}, flag_prob{s.flag_prob}
        {
        }
    };

    struct AutoSteerOutput
    {
        static constexpr std::size_t kPoints{64U};
        std::array<float, kPoints> xp{};
        std::array<float, kPoints> h_vector{};

        AutoSteerOutput() = default;

        template <typename Source>
        explicit AutoSteerOutput(Source const& s) :
            xp{s.xp}, h_vector{s.h_vector}
        {
        }
    };

    struct Detection
    {
        float x1{0.0F};
        float y1{0.0F};
        float x2{0.0F};
        float y2{0.0F};
        std::int32_t class_id{0};
    };

    struct AutoSpeedOutput
    {
        std::vector<Detection> detections;

        AutoSpeedOutput() = default;

        template <typename Source>
        explicit AutoSpeedOutput(Source const& s) :
            detections{}
        {
            detections.reserve(s.detections.size());
            for (auto const& d : s.detections)
            {
                detections.push_back(Detection{d.x1, d.y1, d.x2, d.y2, d.class_id});
            }
        }
    };

    struct CIPOEstimate
    {
        float distance_m{0.0F};
        float velocity_ms{0.0F};

        CIPOEstimate() = default;

        template <typename Source>
        explicit CIPOEstimate(Source const& s) :
            distance_m{s.distance_m}, velocity_ms{s.velocity_ms}
        {
        }
    };

    struct LateralEstimate
    {
        float cte_m{0.0F};
        float cte_rate_mps{0.0F};
        float yaw_rad{0.0F};
        float yaw_rate_rps{0.0F};
        float curvature{0.0F};
        float path_a{0.0F};
        float path_b{0.0F};
        float path_c{0.0F};
        float path_x_min_m{0.0F};
        float path_x_max_m{0.0F};

        LateralEstimate() = default;

        template <typename Source>
        explicit LateralEstimate(Source const& s) :
            cte_m{s.cte_m}, cte_rate_mps{s.cte_rate_mps}, yaw_rad{s.yaw_rad}, yaw_rate_rps{s.yaw_rate_rps},
            curvature{s.curvature}, path_a{s.path_a}, path_b{s.path_b}, path_c{s.path_c},
            path_x_min_m{s.path_x_min_m}, path_x_max_m{s.path_x_max_m}
        {
        }
    };

    struct InferenceResult
    {
        double wall_ms{0.0};
        double pre_ms{0.0};
        double ad_ms{0.0};
        double as_ms{0.0};
        double asp_ms{0.0};
        AutoDriveOutput auto_drive{};
        AutoSteerOutput auto_steer{};
        AutoSpeedOutput auto_speed{};
        CIPOEstimate cipo{};
        LateralEstimate lateral{};

        InferenceResult() = default;

        template <typename Source>
        explicit InferenceResult(Source const& s) :
            wall_ms{s.wall_ms}, pre_ms{s.pre_ms}, ad_ms{s.ad_ms}, as_ms{s.as_ms}, asp_ms{s.asp_ms},
            auto_drive{s.auto_drive}, auto_steer{s.auto_steer}, auto_speed{s.auto_speed},
            cipo{s.cipo}, lateral{s.lateral}
        {
        }
    };

    class Rerun
    {
    public:
        // Open a Rerun recording that streams to `rrd_path` (created/truncated).
        // `app_id` names the application in the Rerun viewer.
        static void init(const std::string& rrd_path = "visionpilot.rrd",
                         const std::string& app_id = "visionpilot");

        // Log everything for a single frame in one call.
        // Timeline is set once, all entities share it.
        //   frame        : raw input image
        //   warped       : warped BEV image (AutoDrive input)
        //   resized      : cropped + resized image (AutoSteer / AutoSpeed input)
        //   r            : model + fusion outputs
        //   plan         : planner output (tyre angle, acceleration, warnings)
        //   ego_speed_ms : ego vehicle speed (m/s)
        //   viz          : output visualization / HUD image
        static void log_frame(std::uint64_t frame_id,
                              const cv::Mat& frame,
                              const cv::Mat& warped,
                              const cv::Mat& resized,
                              const InferenceResult& r,
                              const Plan& plan,
                              double ego_speed_ms,
                              const cv::Mat& viz);

        // Flush & close
        static void shutdown();
    };
} // namespace logging

#endif // VISIONPILOT_LOGGING_RERUN_HPP
