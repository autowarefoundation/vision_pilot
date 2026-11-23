#ifndef VISIONPILOT_LOGGING_LOGGER_HPP
#define VISIONPILOT_LOGGING_LOGGER_HPP
#include <string>
#include <cstdint>

// Console output (log_info / log_warn / log_error) lives in <logging/console.hpp>.

// Rerun logger - streams per-frame data directly to a native .rrd recording
// through Rerun C++ SDK.
// Images are logged zero-copy - raw OpenCV BGR buffer is borrowed inplace,
// no per-frame allocation, colour conversion or PNG encoding takes place.
//
// When built without the SDK (ENABLE_RERUN off), every call is a cheap no-op.
//
// Usage:
//   logging::Rerun::init("visionpilot.rrd");
//   logging::Rerun::log_frame(frame_id, frame, warped, resized,
//                             inference_result, plan, ego_speed_ms, viz_image);
//   logging::Rerun::shutdown();

namespace cv { class Mat; }
namespace visionpilot { namespace models { struct InferenceFrameResult; } }
namespace visionpilot { namespace models { struct AutoDriveOutput; } }
namespace visionpilot { namespace models { struct AutoSteerOutput; } }
namespace visionpilot { namespace models { struct AutoSpeedOutput; } }
namespace visionpilot { namespace fusion { struct CIPOFusionEstimate; } }
namespace visionpilot { namespace fusion { struct LateralFusionEstimate; } }
namespace visionpilot { namespace planning { } }

struct Plan;

namespace logging {

class Rerun {
public:
	// Open a Rerun recording that streams to `rrd_path` (created/truncated).
	// `app_id` names application in the Rerun viewer.
	static void init(const std::string& rrd_path = "visionpilot.rrd",
					 const std::string& app_id   = "visionpilot");

	// Log everything for a single frame in one call. 
	// Timeline is set once, all entities share it.
	//   frame        : raw input image
	//   warped       : warped BEV image (AutoDrive input)
	//   resized      : cropped + resized image (AutoSteer / AutoSpeed input)
	//   r            : model + fusion outputs
	//   plan         : planner output (tyre angle, acceleration, warnings)
	//   ego_speed_ms : ego vehicle speed (m/s)
	//   viz          : output visualization / HUD image
	static void log_frame(uint64_t frame_id,
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

#endif // VISIONPILOT_LOGGING_LOGGER_HPP
