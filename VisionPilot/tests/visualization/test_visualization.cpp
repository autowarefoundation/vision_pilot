#include <visualization/visualization.hpp>

#include <cassert>
#include <vector>

// Smoke-test: visualize_frame with empty inputs must return a non-empty Mat.
static void test_visualize_frame_empty_inputs() {
    cv::Mat frame(512, 1024, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<visualization::YoloBoundingBox> bboxes;
    visualization::LaneShapeVisualization lane{};
    visualization::DesiredControlVisualization ctrl{};

    cv::Mat result = visualization::visualize_frame(frame, bboxes, lane, ctrl);
    assert(!result.empty() && "visualize_frame must return a non-empty Mat");
}

// Smoke-test: visualize_frame with a single bounding box must not crash.
static void test_visualize_frame_with_bbox() {
    cv::Mat frame(512, 1024, CV_8UC3, cv::Scalar(100, 100, 100));

    visualization::YoloBoundingBox box{};
    box.class_id = 1;
    box.center_x = 512.0F;
    box.center_y = 256.0F;
    box.width    = 80.0F;
    box.height   = 60.0F;

    std::vector<visualization::YoloBoundingBox> bboxes{box};
    visualization::LaneShapeVisualization lane{};
    visualization::DesiredControlVisualization ctrl{0.5F, 60.0F, 1.2F};

    cv::Mat result = visualization::visualize_frame(frame, bboxes, lane, ctrl);
    assert(!result.empty() && "visualize_frame must return a non-empty Mat with bbox");
}

int main() {
    test_visualize_frame_empty_inputs();
    test_visualize_frame_with_bbox();
    return 0;
}
