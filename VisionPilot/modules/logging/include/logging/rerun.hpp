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
//                             inference_result, plan, ego_speed_ms, viz_image);
//   logging::Rerun::shutdown();
// =============================================================================

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <opencv2/core/mat.hpp>   // defines cv::Mat; lighter than opencv2/core.hpp

// Forward declarations: the API only takes these types by reference, so the
// heavy headers (OpenCV, models) are included in rerun.cpp only.
namespace visionpilot
{
    namespace models
    {
        struct InferenceFrameResult;
    }
}

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
        std::vector<double> steering{};
        std::vector<Warning> warnings{};

        Plan() = default;

        // Builds the logging copy from the planner's Plan without logging
        // depending on the planner's types. Source warnings are matched by
        // numeric value: the source enum must use the same numbering as
        // logging::Warning.
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
                              const visionpilot::models::InferenceFrameResult& r,
                              const Plan& plan,
                              double ego_speed_ms,
                              const cv::Mat& viz);

        // Flush & close
        static void shutdown();
    };
} // namespace logging

#endif // VISIONPILOT_LOGGING_RERUN_HPP
