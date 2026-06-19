// Runtime smoke for ROS2ImageSubscriber: publish a synthetic bgra8 image (as CARLA's
// native --ros2 camera does) and confirm the subscriber yields a 3-channel CV_8UC3 BGR
// frame with the alpha dropped. Guards the bgra8->bgr8 conversion the preprocessor needs.
#include <rclcpp/rclcpp.hpp>
#include <camera_subscriber/ros2_to_opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

namespace
{

int g_failures = 0;

void check(const char * name, bool ok)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

}  // namespace

int main()
{
  const std::string topic = "/camera/image";

  // Subscriber under test (owns rclcpp::init, spins in the background).
  camera_interface::ROS2ImageSubscriber subscriber(topic);

  // Publisher on a matching QoS: the subscriber uses best-effort / volatile / KeepLast(1).
  auto pub_node = std::make_shared<rclcpp::Node>("test_camera_publisher");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>(topic, qos);

  // 4x2 bgra8 image, every pixel B=10 G=20 R=30 A=40 (alpha must be dropped).
  const int W = 4, H = 2;
  sensor_msgs::msg::Image img;
  img.width = W;
  img.height = H;
  img.encoding = "bgra8";
  img.is_bigendian = 0;
  img.step = W * 4;
  img.data.resize(static_cast<size_t>(img.step) * H);
  for (size_t i = 0; i < img.data.size(); i += 4) {
    img.data[i + 0] = 10;  // B
    img.data[i + 1] = 20;  // G
    img.data[i + 2] = 30;  // R
    img.data[i + 3] = 40;  // A
  }

  // Publish + spin until a frame arrives or timeout (discovery + delivery take a moment).
  bool got = false;
  cv::Mat frame;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    pub->publish(img);
    rclcpp::spin_some(pub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::tie(got, frame) = subscriber.get_latest_frame();
  }

  check("frame received", got);
  if (got) {
    check("3-channel BGR (CV_8UC3, alpha dropped)", frame.type() == CV_8UC3);
    check("dimensions preserved", frame.cols == W && frame.rows == H);
    const cv::Vec3b px = frame.at<cv::Vec3b>(0, 0);
    check("pixel BGR = (10, 20, 30)", px[0] == 10 && px[1] == 20 && px[2] == 30);
  }

  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
