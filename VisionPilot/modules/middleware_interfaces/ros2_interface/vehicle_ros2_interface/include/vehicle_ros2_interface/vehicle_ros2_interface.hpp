#ifndef VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP
#define VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <vehicle_interface/vehicle_interface.hpp>

// ── Topics ────────────────────────────────────────────────────────────────────
//
//  Subscribe  /vehicle/speed           Float64   ego speed (m/s)
//
//  Publish    /vehicle/steering_cmd    Float64   tyre angle (rad)
//             /vehicle/throttle_cmd   Float64   acceleration (m/s²)
//
// ─────────────────────────────────────────────────────────────────────────────

class VehicleRos2Interface : public VehicleInterface
{
public:
    VehicleRos2Interface(std::string vehicle_speed_topic, std::string vehicle_steering_topic,
                         std::string vehicle_acceleration_topic);
    ~VehicleRos2Interface() override;

    // Returns the latest ego speed received on /vehicle/speed (m/s).
    // Thread-safe — updated by the ROS2 spin thread via atomic.
    double read() override;

    // Publish tyre angle (rad) and acceleration (m/s²) to ROS2.
    void write(double steering, double acceleration) override;

private:
    // Inner node — owns all ROS 2 concerns
    class VehicleRos2Node : public rclcpp::Node
    {
    public:
        explicit VehicleRos2Node(std::string vehicle_speed_topic,
                                 std::string vehicle_steering_topic,
                                 std::string vehicle_acceleration_topic,
                                 std::function<void(double)> on_speed);
        ~VehicleRos2Node() = default;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    };

    std::shared_ptr<VehicleRos2Node> node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;

    // Latest ego speed — written by the ROS2 callback, read by main loop.
    std::atomic<double> speed_{0.0};
};

#endif //VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP
