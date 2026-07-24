#include <string>
#include <cmath>
#include <algorithm>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>

// ── VehicleRos2Node ───────────────────────────────────────────────────────────

VehicleRos2Interface::VehicleRos2Node::VehicleRos2Node(
    std::string vehicle_speed_topic, std::string vehicle_steering_topic,
    std::string vehicle_acceleration_topic,
    std::function<void(double)> on_speed) : rclcpp::Node("VehicleRos2Node")
{
    // Best-effort depth-1 QoS — we only need the freshest value.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    sub_ = create_subscription<std_msgs::msg::Float64>(
        vehicle_speed_topic, qos,
        [on_speed](const std_msgs::msg::Float64::SharedPtr msg)
        {
            on_speed(msg->data);
        });

    // Reliable depth-1 QoS for commands — must not be silently dropped.
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    steering_pub_ = create_publisher<std_msgs::msg::Float64>(vehicle_steering_topic, cmd_qos);
    throttle_pub_ = create_publisher<std_msgs::msg::Float64>(vehicle_acceleration_topic, cmd_qos);
    lane_path_pub_ = create_publisher<nav_msgs::msg::Path>("/vehicle/lane_path", cmd_qos);

    RCLCPP_INFO(get_logger(), "VehicleRos2Interface ready");
    RCLCPP_INFO(get_logger(), "  sub  /vehicle/speed");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/steering_cmd");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/throttle_cmd");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/lane_path");
}

// ── VehicleRos2Interface ──────────────────────────────────────────────────────

VehicleRos2Interface::VehicleRos2Interface(std::string vehicle_speed_topic, std::string vehicle_steering_topic,
                                           std::string vehicle_acceleration_topic)
{
    node_ = std::make_shared < VehicleRos2Node > (vehicle_speed_topic,
        vehicle_steering_topic,
        vehicle_acceleration_topic,
        [this](double speed) { speed_.store(speed, std::memory_order_relaxed); });

    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
}

VehicleRos2Interface::~VehicleRos2Interface()
{
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
}

// ── VehicleInterface implementation ──────────────────────────────────────────

double VehicleRos2Interface::read()
{
    return speed_.load(std::memory_order_relaxed);
}

void VehicleRos2Interface::write(const double steering, const double acceleration)
{
    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = steering;
    node_->steering_pub_->publish(steer_msg);

    std_msgs::msg::Float64 throttle_msg;
    throttle_msg.data = acceleration;
    node_->throttle_pub_->publish(throttle_msg);
}

void VehicleRos2Interface::publish_lane_path(const double stamp_sec, const bool valid,
                                             const double cte_m, const double yaw_rad,
                                             const double curvature, const bool path_valid,
                                             const double path_x_max)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "base_link";
    if (stamp_sec >= 0.0)
    {
        msg.header.stamp.sec = static_cast<int32_t>(stamp_sec);
        msg.header.stamp.nanosec =
            static_cast<uint32_t>((stamp_sec - msg.header.stamp.sec) * 1e9);
    }
    if (valid)
    {
        // Lane center as the quadratic the fused scalars describe (x fwd, y +left):
        //   y(x) = (curvature/2)·x² + tan(yaw_rad)·x + cte_m
        // cte_m is +ve when the ego sits right of the path, so the path lies at +y.
        const double a = 0.5 * curvature;
        const double b = std::tan(yaw_rad);
        const double x_end = path_valid ? std::clamp(path_x_max, 10.0, 60.0) : 40.0;
        for (double x = 0.0; x <= x_end; x += 1.0)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header = msg.header;
            p.pose.position.x = x;
            p.pose.position.y = a * x * x + b * x + cte_m;
            const double yaw = std::atan(2.0 * a * x + b);
            p.pose.orientation.z = std::sin(yaw / 2.0);
            p.pose.orientation.w = std::cos(yaw / 2.0);
            msg.poses.push_back(p);
        }
    }
    node_->lane_path_pub_->publish(msg);
}
