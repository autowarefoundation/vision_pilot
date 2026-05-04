#include <camera_subscriber/ros2_to_opencv.hpp>

namespace camera_subscriber {

    ROS2ImageSubscriber::ROS2ImageSubscriber(
        const std::string &topic_name,
        size_t queue_size,
        const std::string &node_name
    ) : (
        rclcpp::Node(node_name),
        max_queue_size_(queue_size)
    ) {
        
        RCLCPP_INFO(get_logger(), "Initializing ROS2 Image Subscriber");
        RCLCPP_INFO(get_logger(), "  Topic: %s", topic_name.c_str());
        RCLCPP_INFO(get_logger(), "  Queue Size: %zu", queue_size);

    };

}