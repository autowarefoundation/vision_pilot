// Runtime smoke for ControlCmdPublisher: publish a known ControlCommand and read it
// back through a subscriber in the same process. No model weights, no camera needed.
#include <control_cmd_publisher/cmd_to_ros2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

namespace
{

int g_failures = 0;

void check(const char * name, bool ok)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

bool approx(double a, double b, double tol)
{
  return std::fabs(a - b) <= tol;
}

}  // namespace

int main()
{
  const std::string topic = "/control/ackermann_cmd";

  // Publisher (owns rclcpp::init) under test.
  control_cmd_publisher::ControlCmdPublisher publisher(topic, "test_frame");

  // Subscriber on the same context + matching QoS (reliable, latest-only).
  auto sub_node = std::make_shared<rclcpp::Node>("test_control_cmd_subscriber");
  ackermann_msgs::msg::AckermannDriveStamped received;
  bool got_msg = false;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto sub = sub_node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    topic, qos, [&](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      received = *msg;
      got_msg = true;
    });

  const ControlCommand cmd{0.1234, 12.5, -0.8};  // steering rad, speed m/s, accel m/s^2

  // Publish + spin until received or timeout (discovery + delivery take a moment).
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!got_msg && std::chrono::steady_clock::now() < deadline) {
    publisher.publish(cmd);
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  check("message received", got_msg);
  if (got_msg) {
    check("steering_angle maps (rad)", approx(received.drive.steering_angle, 0.1234, 1e-5));
    check("speed maps (m/s)", approx(received.drive.speed, 12.5, 1e-5));
    check("acceleration maps (m/s^2)", approx(received.drive.acceleration, -0.8, 1e-5));
    check("frame_id stamped", received.header.frame_id == "test_frame");
  }

  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
