#include <vehicle_state_subscriber/ros2_to_can.hpp>

#include <chrono>
#include <cmath>

namespace vehicle_state_subscriber {


    VehicleStateSubscriber::VehicleStateSubscriber(
        const std::string& topic,
        const std::string& node_name
    ) {
        // Initialize ROS2 only if no one else already did; remember so the destructor
        // shuts down exactly the context this object owns (avoids killing a co-resident
        // node's context, e.g. the camera subscriber).
        if (!rclcpp::ok()) {
            // Mutable argv storage: rclcpp::init takes char** and is permitted to rewrite
            // argv during argument parsing, so the backing memory must not be const.
            static char arg0[] = "vision_pilot_vehicle_state";
            static char* argv[] = {arg0, nullptr};
            static int argc = 1;
            rclcpp::init(argc, argv);
            owns_init_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>(node_name);

        // Odometry is state, not a stream: reliable delivery, latest-only.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odometry_callback(msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "Vehicle state subscriber on topic: %s", topic.c_str());

        // Flag-controlled spin loop: never calls rclcpp::shutdown(), so a co-resident node
        // owns init/shutdown and this object simply stops spinning on teardown.
        running_ = true;
        spin_thread_ = std::thread([this]() {
            while (running_ && rclcpp::ok()) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }


    void VehicleStateSubscriber::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
            return;
        }
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        std::lock_guard<std::mutex> lock(mutex_);
        ego_speed_mps_ = std::hypot(vx, vy);
        has_state_ = true;
    }


    bool VehicleStateSubscriber::has_state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_state_;
    }


    double VehicleStateSubscriber::ego_speed_mps() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ego_speed_mps_;
    }


    VehicleStateSubscriber::~VehicleStateSubscriber() {
        running_ = false;
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        if (owns_init_) {
            rclcpp::shutdown();
        }
    }

}  // namespace vehicle_state_subscriber
