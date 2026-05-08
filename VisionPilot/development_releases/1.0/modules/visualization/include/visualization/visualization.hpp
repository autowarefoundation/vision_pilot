//
// Created by atanasko on 27.4.26.
//

#ifndef VISIONPILOT_VISUALIZATION_HPP
#define VISIONPILOT_VISUALIZATION_HPP

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace visualization {

bool render_frame(
	const cv::Mat &frame,
	const std::string &window_name = "VisionPilot",
	const std::vector<std::string> &overlay_lines = {}
);

void close_windows();

}  // namespace visualization

#endif //VISIONPILOT_VISUALIZATION_HPP
