#ifndef VISIONPILOT_UTILS_HPP
#define VISIONPILOT_UTILS_HPP

#include <filesystem>
#include <opencv2/opencv.hpp>

std::string find_config(const std::string& filename);

cv::Mat load_matrix(const std::string& filename, const std::string& matrix);

#endif //VISIONPILOT_UTILS_HPP
