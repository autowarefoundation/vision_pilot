#include <string>
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
    driving_command_pub_ = create_publisher<visionpilot_msgs::msg::DrivingCommand>(
        "/vehicle/driving_command", cmd_qos);
    driving_reference_pub_ = create_publisher<visionpilot_msgs::msg::DrivingReference>(
        "/vehicle/driving_reference", cmd_qos);

    RCLCPP_INFO(get_logger(), "VehicleRos2Interface ready");
    RCLCPP_INFO(get_logger(), "  sub  /vehicle/speed");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/steering_cmd");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/throttle_cmd");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/driving_command");
    RCLCPP_INFO(get_logger(), "  pub  /vehicle/driving_reference");
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

void VehicleRos2Interface::publish_driving_command(const DrivingCommandData& command)
{
    visionpilot_msgs::msg::DrivingCommand msg;
    msg.stamp = node_->now();
    if (command.has_source_stamp)
    {
        msg.source_stamp.sec = command.source_stamp_sec;
        msg.source_stamp.nanosec = command.source_stamp_nanosec;
    }
    msg.has_source_stamp = command.has_source_stamp;
    msg.session = command.session;
    msg.cycle = command.cycle;
    msg.valid = command.valid;
    msg.steering_tire_angle_rad = command.steering_tire_angle_rad;
    msg.target_speed_mps = command.target_speed_mps;
    msg.acceleration_mps2 = command.acceleration_mps2;
    node_->driving_command_pub_->publish(msg);
}

void VehicleRos2Interface::publish_driving_reference(const DrivingReferenceData& reference)
{
    visionpilot_msgs::msg::DrivingReference msg;
    msg.stamp = node_->now();
    if (reference.has_source_stamp)
    {
        msg.source_stamp.sec = reference.source_stamp_sec;
        msg.source_stamp.nanosec = reference.source_stamp_nanosec;
    }
    msg.has_source_stamp = reference.has_source_stamp;
    msg.session = reference.session;
    msg.cycle = reference.cycle;
    msg.valid = reference.valid;
    msg.path_valid = reference.path_valid;
    msg.path_a = reference.path_a;
    msg.path_b = reference.path_b;
    msg.path_c = reference.path_c;
    msg.path_x_max_m = reference.path_x_max_m;
    msg.horizon_dt_s = reference.horizon_dt_s;
    msg.speed_horizon_mps.reserve(reference.speed_horizon_mps.size());
    for (const double speed : reference.speed_horizon_mps)
    {
        msg.speed_horizon_mps.push_back(speed);
    }
    node_->driving_reference_pub_->publish(msg);
}
