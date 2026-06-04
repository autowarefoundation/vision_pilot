#include <visualization/visualization.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

    // Utility func to read a sequence of scalar values from a FileNode
    template <typename T>
    std::vector<T> read_scalar_sequence(const cv::FileNode &node) {

        std::vector<T> values;
        if (node.empty() || !node.isSeq()) {
            return values;
        }

        for (auto it = node.begin(); it != node.end(); ++it) {
            T value{};
            *it >> value;
            values.push_back(value);
        }

        return values;

    };

} // namespace