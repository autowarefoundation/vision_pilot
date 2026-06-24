#ifndef VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP
#define VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <vehicle_interface/vehicle_interface.hpp>

// ── Topics ────────────────────────────────────────────────────────────────────
//
//  Subscribe  /vehicle/speed           Float32   ego speed (m/s)
//
//  Publish    /vehicle/steering_cmd    Float32   tyre angle (rad)
//             /vehicle/throttle_cmd   Float32   acceleration (m/s²)
//
// ─────────────────────────────────────────────────────────────────────────────

class VehicleRos2Interface : public VehicleInterface
{
public:
    VehicleRos2Interface();
    ~VehicleRos2Interface() override;

    // Returns the latest ego speed received on /vehicle/speed (m/s).
    // Thread-safe — may be called from the main loop while the ROS2
    // spin thread updates speed_ in the background.
    double read() override;

    // Publish tyre angle (rad) and acceleration (m/s²) to ROS2.
    void write(double steering, double acceleration) override;

private:
    // Inner ROS2 node — owns all subscription / publication handles.
    // Separated from the outer class so the executor can own the node
    // without needing shared_from_this on VehicleRos2Interface itself.
    class VehicleRos2Node : public rclcpp::Node
    {
    public:
        // on_speed: called from the subscription callback with the new
        //           speed value so the outer class can update speed_.
        explicit VehicleRos2Node(std::function<void(double)> on_speed);
        ~VehicleRos2Node() = default;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    steering_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    throttle_pub_;
    };

    std::shared_ptr<VehicleRos2Node>              node_;
    rclcpp::executors::SingleThreadedExecutor     executor_;
    std::thread                                   spin_thread_;

    // Latest ego speed, written by the ROS2 callback thread,
    // read by the main loop via read().
    std::atomic<double> speed_{0.0};
};

#endif // VISIONPILOT_VEHICLE_ROS2_INTERFACE_HPP
