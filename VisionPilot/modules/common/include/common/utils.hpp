#ifndef VISIONPILOT_UTILS_HPP
#define VISIONPILOT_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>

std::string find_config(const std::string& filename);

cv::Mat load_matrix(const std::string& filename, const std::string& matrix);

// Drop this many rows from the top so the kept band is width × (width/2) → 2:1 aspect.
// Matches Models/inference/auto_steer_infer.py compute_top_crop_2_1().
inline int compute_top_crop_2_1(int height, int width)
{
    return std::max(0, static_cast<int>(std::lround(
        static_cast<double>(height) - static_cast<double>(width) / 2.0)));
}

#endif //VISIONPILOT_UTILS_HPP
