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

        stats_.node_name = node_name;

        // Create subscription to the image topic
        // Using rclcpp::QoS profile for best effort delivery (suitable for camera streams)
        // Reference:
        //      - https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        //      - https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1QoS.html
        
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(queue_size))
            .best_effort()
            .durability_volatile();

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            topic_name,
            qos_profile,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->image_callback(msg);
            }
        );

        RCLCPP_INFO(get_logger(), "ROS2 Image Subscriber initialized successfully");

    };

}