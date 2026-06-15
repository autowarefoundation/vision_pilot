#include "sodasim_gstreamer/image_to_gstreamer_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace sodasim::gstreamer
{
namespace
{

void initializeGStreamer()
{
  static std::once_flag flag;
  std::call_once(flag, []() { gst_init(nullptr, nullptr); });
}

int fpsToInt(double fps_value)
{
  if (fps_value <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::round(fps_value));
}

}  // namespace

ImageToGStreamerNode::ImageToGStreamerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_to_gstreamer", options),
  pipeline_(nullptr),
  appsrc_(nullptr),
  pipeline_ready_(false),
  caps_configured_(false)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/vehicle/camera");
  pipeline_description_ = this->declare_parameter<std::string>(
    "pipeline",
    "appsrc name=ros_appsrc is-live=true format=time do-timestamp=true ! "
    "videoconvert ! x264enc tune=zerolatency bitrate=4000 speed-preset=veryfast ! "
    "rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5600 sync=false");
  appsrc_name_ = this->declare_parameter<std::string>("appsrc_name", "ros_appsrc");
  target_fps_ = this->declare_parameter<double>("target_fps", 30.0);

  initializeGStreamer();

  if (!configurePipeline()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure GStreamer pipeline");
    return;
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImageToGStreamerNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "Rebroadcasting sensor_msgs/Image from '%s' into pipeline: %s",
    input_topic_.c_str(), pipeline_description_.c_str());
}

ImageToGStreamerNode::~ImageToGStreamerNode()
{
  std::lock_guard<std::mutex> lock(gst_mutex_);
  cleanupPipeline();
}

bool ImageToGStreamerNode::configurePipeline()
{
  std::lock_guard<std::mutex> lock(gst_mutex_);

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
  if (!pipeline_) {
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "GStreamer parse error: %s", error->message);
      g_error_free(error);
    }
    return false;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), appsrc_name_.c_str());
  if (!appsrc_) {
    RCLCPP_ERROR(
      this->get_logger(), "appsrc '%s' not found in pipeline description", appsrc_name_.c_str());
    cleanupPipeline();
    return false;
  }

  g_object_set(G_OBJECT(appsrc_), "is-live", TRUE, "format", GST_FORMAT_TIME, nullptr);

  auto state_change = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (state_change == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set GStreamer pipeline to PLAYING");
    cleanupPipeline();
    return false;
  }

  pipeline_ready_ = true;
  caps_configured_ = false;
  return true;
}

void ImageToGStreamerNode::cleanupPipeline()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }
  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  pipeline_ready_ = false;
  caps_configured_ = false;
}

void ImageToGStreamerNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge conversion failed: %s", ex.what());
    return;
  }

  const cv::Mat & source = cv_ptr->image;
  if (source.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Received empty image frame, skipping");
    return;
  }

  cv::Mat contiguous;
  const cv::Mat & frame = source.isContinuous() ? source : (contiguous = source.clone());

  std::lock_guard<std::mutex> lock(gst_mutex_);
  if (!pipeline_ready_ || !appsrc_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "GStreamer pipeline not ready");
    return;
  }

  if (!caps_configured_) {
    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, frame.cols, "height",
      G_TYPE_INT, frame.rows, nullptr);
    if (target_fps_ > 0.0) {
      const int fps_num = std::max(1, fpsToInt(target_fps_));
      gst_caps_set_simple(caps, "framerate", GST_TYPE_FRACTION, fps_num, 1, nullptr);
    }
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);
    caps_configured_ = true;
  }

  const std::size_t image_size = static_cast<std::size_t>(frame.total() * frame.elemSize());
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, image_size, nullptr);
  if (!buffer) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate GStreamer buffer");
    return;
  }

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to map GStreamer buffer");
    gst_buffer_unref(buffer);
    return;
  }

  std::memcpy(map.data, frame.data, image_size);
  gst_buffer_unmap(buffer, &map);

  const auto timestamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  GST_BUFFER_PTS(buffer) = static_cast<guint64>(timestamp);
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  if (target_fps_ > 0.0) {
    const auto duration = static_cast<guint64>(GST_SECOND / target_fps_);
    GST_BUFFER_DURATION(buffer) = duration;
  } else {
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;
  }

  const GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (ret != GST_FLOW_OK) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Failed to push frame into GStreamer pipeline (flow: %d)", ret);
  }
}

}  // namespace sodasim::gstreamer

RCLCPP_COMPONENTS_REGISTER_NODE(sodasim::gstreamer::ImageToGStreamerNode)
