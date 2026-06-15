//
// Created by atanasko on 2/11/26.
//
#ifndef VISIONPILOT_PUBLISHER_VISIONPILOT_PUBLISHER_NODE_HPP
#define VISIONPILOT_PUBLISHER_VISIONPILOT_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include "ipc_shared_state.hpp"

class VisionPilotPublisher : public rclcpp::Node
{
public:
  VisionPilotPublisher();

private:
  void timer_callback();
  IpcSharedState ipcSharedState;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;


  double TARGET_VEL;
};

#endif //VISIONPILOT_PUBLISHER_VISIONPILOT_PUBLISHER_NODE_HPP