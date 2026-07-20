#pragma once
#include <cstdio>
#include <string>
#include <cstdint>

#define VP_INFO(fmt, ...)  std::printf( "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define VP_WARN(fmt, ...)  std::fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define VP_ERROR(fmt, ...) std::fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

// Lightweight Rerun-style file logger (no external deps)
// - Writes structured per-frame logs into an output directory
// - Images are encoded as PNG files, metadata as simple JSON files
// Usage:
//   logging::Rerun::init("/path/to/rerun_out");
//   logging::Rerun::log_frame_images(frame_id, frame, warped, resized);
//   logging::Rerun::log_inference(frame_id, inference_result);
//   logging::Rerun::log_plan(frame_id, plan);
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
	// Init Rerun logger output dir (creates if missing)
	static void init(const std::string& out_dir = "rerun_logs");

	// Log imgs for a single frame: raw camera frame, warped BEV, resized
	static void log_frame_images(uint64_t frame_id,
								 const cv::Mat& frame,
								 const cv::Mat& warped,
								 const cv::Mat& resized);

	// Log inference outputs (models + fusion)
	static void log_inference(uint64_t frame_id,
							  const visionpilot::models::InferenceFrameResult& r);

	// Log plan / control output
	static void log_plan(uint64_t frame_id, const Plan& p);

	// Flush & close
	static void shutdown();
};

} // namespace logging

