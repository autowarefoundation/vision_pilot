#include <control_cmd_publisher/cmd_to_ros2.hpp>

#include <memory>
#include <string>

namespace control_cmd_publisher {


    ControlCmdPublisher::ControlCmdPublisher(
        const std::string& topic,
        const std::string& frame_id,
        const std::string& node_name
    ) : frame_id_(frame_id) {
        // Initialize ROS2 only if no one else already did; remember so the destructor
        // shuts down exactly the context this object owns (avoids killing a co-resident
        // node's context, e.g. the camera subscriber).
        if (!rclcpp::ok()) {
            // Mutable argv storage: rclcpp::init takes char** and is permitted to rewrite
            // argv during argument parsing, so the backing memory must not be const.
            static char arg0[] = "vision_pilot_control";
            static char* argv[] = {arg0, nullptr};
            static int argc = 1;
            rclcpp::init(argc, argv);
            owns_init_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>(node_name);

        // Control commands are state, not a stream: reliable delivery, latest-only.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        pub_ = node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(topic, qos);

        RCLCPP_INFO(node_->get_logger(), "Control command publisher on topic: %s", topic.c_str());
    }


    void ControlCmdPublisher::publish(const ControlCommand& cmd) {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = frame_id_;
        msg.drive.steering_angle = static_cast<float>(cmd.steering_angle_rad);
        msg.drive.speed = static_cast<float>(cmd.speed_mps);
        msg.drive.acceleration = static_cast<float>(cmd.acceleration_mps2);
        pub_->publish(msg);
    }


    ControlCmdPublisher::~ControlCmdPublisher() {
        if (owns_init_) {
            rclcpp::shutdown();
        }
    }

}  // namespace control_cmd_publisher
