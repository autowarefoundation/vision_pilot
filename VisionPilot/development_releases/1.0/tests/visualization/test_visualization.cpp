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


    // Utility func to load bounding boxes from a FileNode,
    // returning a vector of YoloBoundingBox structs
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
        
    };


    // Utility func to convert YOLOX format bbox to OpenCV Rect, 
    // with clamping to ensure it fits within the frame
    visualization::LaneShapeVisualization load_lane_shape(const cv::FileNode &node) {
        
        visualization::LaneShapeVisualization lane_shape;
        if (node.empty()) {
            return lane_shape;
        }

        node["has_cipo_object"] >> lane_shape.has_cipo_object;

        double distance_to_cipo = 0.0;
        node["distance_to_cipo"] >> distance_to_cipo;
        lane_shape.distance_to_cipo = static_cast<float>(distance_to_cipo);

        double relative_velocity = 0.0;
        node["relative_cipo_velocity"] >> relative_velocity;
        lane_shape.relative_cipo_velocity = static_cast<float>(relative_velocity);

        const cv::FileNode waypoints = node["tracked_waypoints"];
        if (!waypoints.empty() && waypoints.isSeq()) {
            for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
                const std::vector<float> values = read_scalar_sequence<float>(*it);
                if (values.size() < 2) {
                    continue;
                }
                lane_shape.tracked_waypoints.emplace_back(values[0], values[1]);
            }
        }

        return lane_shape;

    }

} // namespace