#include <camera_ros2_interface/camera_ros2_interface.hpp>

#include <algorithm>
#include <memory>
#include <thread>


ROS2ImageSubscriber::ROS2ImageSubscriber(
    const std::string& topic_name,
    const std::string& node_name
) : topic_name(topic_name)
{
    // Init ROS2 once
    if (!rclcpp::ok())
    {
        static int argc = 1;
        static const char* argv[] = {"ros2_image_subscriber", nullptr};
        rclcpp::init(argc, const_cast<char**>(argv));
    }

    // Create internal node
    node = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO(node->get_logger(), "Initializing ROS2 Image Subscriber");
    RCLCPP_INFO(node->get_logger(), "  Topic: %s", topic_name.c_str());
    RCLCPP_INFO(node->get_logger(), "  QoS History Depth: %u", qos_history_depth);

    stats.node_name = node_name;

    // Create subscription to the image topic
    // Using rclcpp::QoS profile for best effort delivery (suitable for camera streams)
    // Reference:
    //      - https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    //      - https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1QoS.html

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(qos_history_depth))
                       .best_effort()
                       .durability_volatile();

    image_subscription = node->create_subscription<sensor_msgs::msg::Image>(
        topic_name,
        qos_profile,
        [this](const sensor_msgs::msg::Image::SharedPtr msg)
        {
            this->image_callback(msg);
        }
    );

    RCLCPP_INFO(node->get_logger(), "ROS2 Image Subscriber initialized successfully");

    // Start background spin thread
    spin_thread = std::thread([this]()
    {
        rclcpp::spin(this->node);
    });
};


void ROS2ImageSubscriber::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg
)
{
    // Check incoming msg
    if (!msg)
    {
        RCLCPP_WARN(node->get_logger(), "Received null image message");
        return;
    };
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.frames_received++;
        stats.last_encoding = msg->encoding;
    };

    // Convert ROS2 msg => OpenCV img
    cv::Mat cv_image = convert_ros2_image_to_opencv(msg);

    if (cv_image.empty())
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.conversion_errors++;
        RCLCPP_WARN(
            node->get_logger(),
            "Failed to convert ROS2 image to OpenCV (encoding: %s)",
            msg->encoding.c_str()
        );
        return;
    };

    // Store frame in single latest-frame slot with thread safety
    {
        std::lock_guard<std::mutex> lock(frame_mutex);

        // Single-slot buffering: replacing an unread frame counts as dropped.
        if (has_latest_frame)
        {
            std::lock_guard<std::mutex> stats_lock(stats_mutex);
            stats.frames_dropped++;
        }

        // Declare that stream starts
        is_stream_started = true;

        // Store latest frame
        latest_frame = cv_image.clone(); // Clone to ensure independent memory
        has_latest_frame = true;
    }
};


cv::Mat ROS2ImageSubscriber::convert_ros2_image_to_opencv(
    const sensor_msgs::msg::Image::SharedPtr& msg
)
{
    try
    {
        // Here I just use cv_bridge to convert ROS2 image message to OpenCV Mat
        // Ref: https://docs.ros.org/en/diamondback/api/cv_bridge/html/c++/namespacecv__bridge.html
        // For desired output encoding:
        //      - "bgr8"  : commonly used for color images
        //      - "mono8" : for grayscale
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        return cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "cv_bridge exception: %s, encoding: %s",
            e.what(),
            msg->encoding.c_str()
        );
        return cv::Mat(); // Return empty mat on error
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Unexpected exception during image conversion: %s",
            e.what()
        );
        return cv::Mat();
    }
};


std::tuple<bool, cv::Mat> ROS2ImageSubscriber::get_latest_frame()
{
    std::lock_guard<std::mutex> lock(frame_mutex);

    // Check if no frame is currently available
    if (!has_latest_frame)
    {
        return std::make_tuple(false, cv::Mat());
    }

    // Fetch and consume latest frame
    cv::Mat frame = latest_frame.clone();
    has_latest_frame = false;
    latest_frame.release();

    // Frame is valid
    return std::make_tuple(true, frame);
};


// Frame access helpers

bool ROS2ImageSubscriber::has_frames() const
{
    std::lock_guard<std::mutex> lock(frame_mutex);
    return has_latest_frame;
};

bool ROS2ImageSubscriber::is_stream_active() const
{
    std::lock_guard<std::mutex> lock(frame_mutex);
    return is_stream_started;
};

void ROS2ImageSubscriber::clear_frame_buffer()
{
    std::lock_guard<std::mutex> lock(frame_mutex);

    latest_frame.release();
    has_latest_frame = false;

    RCLCPP_INFO(node->get_logger(), "Frame buffer cleared");
};

std::vector<std::string> ROS2ImageSubscriber::get_overlay() const
{
    std::lock_guard<std::mutex> lock(stats_mutex);

    std::vector<std::string> overlay = {
        "topic: " + this->topic_name,
        "frames received: " + std::to_string(stats.frames_received),
        "frames dropped: " + std::to_string(stats.frames_dropped),
        "conversion errors: " + std::to_string(stats.conversion_errors)
    };

    return overlay;
}

void ROS2ImageSubscriber::reset_stats()
{
    std::lock_guard<std::mutex> lock(stats_mutex);
    stats.frames_received = 0;
    stats.frames_dropped = 0;
    stats.conversion_errors = 0;
    RCLCPP_INFO(node->get_logger(), "Statistics reset");
};


ROS2ImageSubscriber::~ROS2ImageSubscriber()
{
    // Request node to shutdown
    rclcpp::shutdown();

    // Wait for spin thread to finish
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
}
