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


    // Utility func to load bounding boxes from a FileNode, returning a vector of YoloBoundingBox structs
    std::vector<visualization::YoloBoundingBox> load_bounding_boxes(const cv::FileNode &node) {
        std::vector<visualization::YoloBoundingBox> bounding_boxes;
        if (node.empty() || !node.isSeq()) {
            return bounding_boxes;
        }

        for (auto it = node.begin(); it != node.end(); ++it) {
            const std::vector<float> values = read_scalar_sequence<float>(*it);
            if (values.size() < 5) {
                continue;
            }

            visualization::YoloBoundingBox box;
            box.class_id = static_cast<int>(values[0]);
            box.center_x = values[1];
            box.center_y = values[2];
            box.width = values[3];
            box.height = values[4];
            bounding_boxes.push_back(box);
        }

        return bounding_boxes;
    }

} // namespace