#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>

VehicleRos2Interface::VehicleRos2Node::VehicleRos2Node()
    : rclcpp::Node("vehicle_ros2_interface")
{
    // pub_ = create_publisher<...>(...);
    // sub_ = create_subscription<...>(..., callback);
}

VehicleRos2Interface::VehicleRos2Interface()
{
    node_ = std::make_shared<VehicleRos2Node>(); // node exists as shared_ptr
    executor_.add_node(node_); // no shared_from_this needed
    spin_thread_ = std::thread([this]() { executor_.spin(); });
}

VehicleRos2Interface::~VehicleRos2Interface()
{
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
}

double VehicleRos2Interface::read()
{
    return 0.0;
}

void VehicleRos2Interface::write(const double steering, const double acceleration)
{
}
