// Runtime smoke for ROS2ImageSubscriber: publish synthetic frames and confirm the
// encoding normalization — a bgra8 image (as CARLA's native --ros2 camera publishes)
// must come out as a 3-channel CV_8UC3 BGR frame with the alpha dropped, while a
// mono8 image must pass through unchanged (the documented grayscale path).
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

void check(const char* name, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

// Publish `img` until the subscriber yields a frame accepted by `accept`, or the
// deadline expires (discovery + delivery take a moment). `accept` filters out
// stale frames from a previous case still in flight in the single-slot buffer.
template <typename AcceptFn>
std::tuple<bool, cv::Mat> publish_and_receive(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
    const rclcpp::Node::SharedPtr& pub_node,
    camera_interface::ROS2ImageSubscriber& subscriber,
    const sensor_msgs::msg::Image& img,
    AcceptFn&& accept)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(img);
        rclcpp::spin_some(pub_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto [got, frame] = subscriber.get_latest_frame();
        if (got && accept(frame)) return {true, frame};
    }
    return {false, cv::Mat()};
}

}  // namespace

int main()
{
    const std::string topic = "/camera/image";
    const int W = 4, H = 2;

    // Subscriber under test (owns rclcpp::init, spins in the background).
    camera_interface::ROS2ImageSubscriber subscriber(topic);

    // Publisher on a matching QoS: the subscriber uses best-effort / volatile / KeepLast.
    auto pub_node = std::make_shared<rclcpp::Node>("test_camera_publisher");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>(topic, qos);

    // Case 1: bgra8, every pixel B=10 G=20 R=30 A=40 — alpha must be dropped.
    sensor_msgs::msg::Image bgra;
    bgra.width = W;
    bgra.height = H;
    bgra.encoding = "bgra8";
    bgra.is_bigendian = 0;
    bgra.step = W * 4;
    bgra.data.resize(static_cast<size_t>(bgra.step) * H);
    for (size_t i = 0; i < bgra.data.size(); i += 4) {
        bgra.data[i + 0] = 10;  // B
        bgra.data[i + 1] = 20;  // G
        bgra.data[i + 2] = 30;  // R
        bgra.data[i + 3] = 40;  // A
    }

    auto [got_bgra, frame_bgra] = publish_and_receive(
        pub, pub_node, subscriber, bgra, [](const cv::Mat&) { return true; });
    check("bgra8 frame received", got_bgra);
    if (got_bgra) {
        check("bgra8 -> 3-channel BGR (CV_8UC3, alpha dropped)", frame_bgra.type() == CV_8UC3);
        check("bgra8 dimensions preserved", frame_bgra.cols == W && frame_bgra.rows == H);
        const cv::Vec3b px = frame_bgra.at<cv::Vec3b>(0, 0);
        check("bgra8 pixel BGR = (10, 20, 30)", px[0] == 10 && px[1] == 20 && px[2] == 30);
    }

    // Case 2: mono8, every pixel 77 — must pass through as single-channel.
    sensor_msgs::msg::Image mono;
    mono.width = W;
    mono.height = H;
    mono.encoding = "mono8";
    mono.is_bigendian = 0;
    mono.step = W;
    mono.data.assign(static_cast<size_t>(mono.step) * H, 77);

    // Skip late bgra8 leftovers (identified by the case-1 pixel pattern); a broken
    // mono passthrough (converted to 3-channel gray) is NOT skipped and fails below.
    auto [got_mono, frame_mono] = publish_and_receive(
        pub, pub_node, subscriber, mono, [](const cv::Mat& f) {
            return f.type() != CV_8UC3 || f.at<cv::Vec3b>(0, 0) != cv::Vec3b(10, 20, 30);
        });
    check("mono8 frame received", got_mono);
    if (got_mono) {
        check("mono8 passes through single-channel (CV_8UC1)", frame_mono.type() == CV_8UC1);
        check("mono8 dimensions preserved", frame_mono.cols == W && frame_mono.rows == H);
        check("mono8 pixel value preserved", frame_mono.at<uint8_t>(0, 0) == 77);
    }

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
