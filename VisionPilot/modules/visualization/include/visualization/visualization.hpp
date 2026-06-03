#ifndef VISIONPILOT_VISUALIZATION_HPP
#define VISIONPILOT_VISUALIZATION_HPP

#include <opencv2/opencv.hpp>

#include <optional>

#include <string>
#include <vector>

namespace visualization {

// Dimensions of major compartments
inline constexpr int kFrameWidth = 1024;
inline constexpr int kFrameHeight = 512;
inline constexpr int kVisualizationPanelWidth = 360;

// Background alphas
inline constexpr float kDetectionOverlayAlpha = 0.30F;
inline constexpr float kRightPanelAlpha = 0.30F;
inline constexpr float kDrivablePathAlpha = 0.30F;

// Colors
inline const cv::Scalar kCipoColor(0, 0, 255);
inline const cv::Scalar kCuttingInColor(0, 255, 255);
inline const cv::Scalar kOtherCarsColor(255, 0, 0);
inline const cv::Scalar kPositiveAccelerationColor(0, 200, 0);
inline const cv::Scalar kNegativeAccelerationColor(0, 0, 255);
inline const cv::Scalar kWhiteColor(0, 0, 0);
inline const cv::Scalar kPanelTextColor(35, 35, 35);

bool render_frame(
	const cv::Mat &frame,
	const std::string &window_name = "VisionPilot",
	const std::vector<std::string> &overlay_lines = {}
);

void close_windows();

}  // namespace visualization

#endif //VISIONPILOT_VISUALIZATION_HPP
