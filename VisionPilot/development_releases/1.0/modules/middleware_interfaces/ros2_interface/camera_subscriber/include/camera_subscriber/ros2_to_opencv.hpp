#ifndef VISIONPILOT_ROS2_TO_OPENCV_HPP
#define VISIONPILOT_ROS2_TO_OPENCV_HPP

#include <rclcpp/rclcpp.hpp>

namespace camera_subscriber {

    /**
    * @class ROS2ImageSubscriber
    * @brief ROS2 node that subscribes to `sensor_msgs/image` topics and 
    *        converts ROS2 image message to OpenCV image format (cv::Mat).
    * 
    * Features:
    * - Subscribes to ROS2 image topics (from any source - simulator, camera, hardware, etc.)
    * - Converts ROS2 image messages to OpenCV cv::Mat format
    * - Thread-safe conversion and data handling
    * - Supports various image encodings (RGB, BGR, grayscale, etc.)
    */

    class ROS2ImageSubscriber : public rclcpp::Node {

        public:

            /**
            * @brief Constructor for ROS2ImageSubscriber
            *
            * @param topic_name The name of the ROS2 topic to subscribe to ("/camera/image_raw", etc.)
            * @param queue_size Max number of frames to buffer for incoming images (default: 10)
            * @param node_name The name of the ROS2 node (default: "ros2_image_subscriber")
            *
            * The node automatically inits the ROS2 subscription and begins listening for incoming messages.
            */
            explicit ROS2ImageSubscriber(
                const std::string& topic_name,
                size_t queue_size = 10,
                const std::string& node_name = "ros2_image_subscriber"
            );

            /**
            * @brief Destructor for ROS2ImageSubscriber
            * 
            * Cleans up ROS2 subscriptions and resources when the node is destroyed.
            */
            ~ROS2ImageSubscriber() override = default;

            /**
            * @brief Get latest frame with corresponding frame metadata
            *
            * @return cv::Mat containing the latest image frame received from the ROS2 topic.
            *
            * This method is thread-safe and can be called from multiple threads without causing data corruption.
            * Returns empty cv::Mat if no frames have been received yet.
            */
            cv::Mat get_latest_frame();

            /**
            * @brief Get latest frame with frame metadata, via timestamp and frame index
            * 
            * @param frame_index Output parameter: frame sequence number from ROS2 message
            * @param timestamp_sec Output parameter: ROS2 timestamp in seconds
            * @return cv::Mat The image frame, or empty if none available
            * 
            * Provides additional timing information along with the frame for synchronization purposes.
            */
            cv::Mat get_latest_frame_with_timestamp(
                uint32_t &frame_index,
                double &timestamp_sec
            );

        private:

            /**
            * @brief Internal callback function invoked when a new ROS2 image message arrives
            * 
            * Handles thread-safe conversion from sensor_msgs::msg::Image to cv::Mat
            * and queuing of frames for retrieval by the application.
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
                const sensor_msgs::msg::Image::SharedPtr &msg
            );

    };

};

#endif //VISIONPILOT_ROS2_TO_OPENCV_HPP