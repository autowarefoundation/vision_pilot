#ifndef VISIONPILOT_ROS2_TO_OPENCV_HPP
#define VISIONPILOT_ROS2_TO_OPENCV_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#if __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#elif __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#error "cv_bridge header not found"
#endif
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>
#include <camera_interface/camera_interface.hpp>


/**
* @class ROS2ImageSubscriber
* @brief ROS2 node that subscribes to `sensor_msgs/image` topics and
*        converts ROS2 image message to OpenCV image format (cv::Mat).
*
* Features:
* - Automatically initializes ROS2 (rclcpp::init)
* - Creates and manages internal ROS2 node
* - Subscribes to ROS2 image topics
* - Spins node in background thread
* - Converts ROS2 messages to OpenCV cv::Mat format
* - Thread-safe conversion and data handling
* - Supports various image encodings (RGB, BGR, grayscale, etc.)
*/
class ROS2ImageSubscriber : public CameraInterface
{
public:
    /**
    * @brief Constructor for ROS2ImageSubscriber
    *
    * @param topic_name The name of the ROS2 topic to subscribe to ("/camera/image_raw", etc.)
    * @param node_name The name of the ROS2 node (default: "ros2_image_subscriber")
    *
    * Initializes ROS2 (rclcpp::init), creates internal node, subscribes to topic, and starts background spinning.
    */
    explicit ROS2ImageSubscriber(
        const std::string& topic_name,
        const std::string& node_name = "ros2_image_subscriber"
    );


    /**
    * @brief Destructor for ROS2ImageSubscriber
    *
    * Cleans up ROS2 subscriptions, shuts down spinning thread, and shutdowns rclcpp.
    */
    ~ROS2ImageSubscriber();


    // FRAME HANDLINGS


    /**
    * @brief Get latest frame with corresponding frame metadata
    *
    * @return cv::Mat containing the latest image frame received from the ROS2 topic.
    *
    * This method is thread-safe and can be called from multiple threads without causing data corruption.
    * Returns empty cv::Mat if no frames have been received yet.
    */
    std::tuple<bool, cv::Mat> get_latest_frame();


    // /**
    // * @brief Get latest frame with frame metadata, via timestamp and frame index
    // *
    // * @param frame_index Output parameter: frame sequence number from ROS2 message
    // * @param timestamp_sec Output parameter: ROS2 timestamp in seconds
    // * @return cv::Mat The image frame, or empty if none available
    // *
    // * Provides additional timing information along with the frame for synchronization purposes.
    // */
    // cv::Mat get_latest_frame_with_timestamp(
    //     uint32_t &frame_index,
    //     double &timestamp_sec
    // );


    /**
    * @brief Check if a latest frame is currently available
    *
    * @return true if a frame is available, false otherwise
    */
    bool has_frames() const;

    bool is_device_open() const { return true; }

    /**
    * @brief Reset the frame buffer (clear the latest frame slot)
    *
    * Useful for resetting state or handling error conditions
    */
    void clear_frame_buffer();


    /**
    * @brief Check if ROS2 stream is active
    *
    * @return true if stream has started receiving frames, false otherwise
    */
    bool is_stream_active() const;

    std::vector<std::string> get_overlay() const;


    /**
    * @brief Reset statistics counters
    */
    void reset_stats();

private:
    std::string topic_name;

    /**
    * @brief Internal callback function invoked when a new ROS2 image message arrives
    *
    * Handles thread-safe conversion from sensor_msgs::msg::Image to cv::Mat
    * and updates the latest frame slot for retrieval by the application.
    */
    void image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );


    /**
    * @brief Thread-safe conversion from ROS2 Image message to cv::Mat
    *
    * @param msg The ROS2 image message
    *
    * @return cv::Mat : The converted image, or empty Mat if conversion failed
    *
    * Uses cv_bridge library to handle various ROS2 image encodings.
    */
    cv::Mat convert_ros2_image_to_opencv(
        const sensor_msgs::msg::Image::SharedPtr& msg
    );


    // ROS2 node and subscription (managed internally)
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    std::thread spin_thread;

    // Single latest-frame slot with thread safety
    mutable std::mutex frame_mutex;
    cv::Mat latest_frame;
    bool has_latest_frame = false;

    // QoS settings for subscription
    // (KeepLast with hardcoded `depth=1` for single-slot retrieval)
    uint8_t qos_history_depth = 1;


    // Flag for started stream
    bool is_stream_started = false;


    // Statistics tracking
    mutable std::mutex stats_mutex;
    CaptureStats stats;
};

#endif //VISIONPILOT_ROS2_TO_OPENCV_HPP
