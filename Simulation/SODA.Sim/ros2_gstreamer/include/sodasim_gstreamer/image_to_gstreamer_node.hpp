#ifndef SODASIM_GSTREAMER_IMAGE_TO_GSTREAMER_NODE_HPP_
#define SODASIM_GSTREAMER_IMAGE_TO_GSTREAMER_NODE_HPP_

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <mutex>
#include <string>

namespace sodasim::gstreamer
{

class ImageToGStreamerNode : public rclcpp::Node
{
public:
  explicit ImageToGStreamerNode(const rclcpp::NodeOptions & options);
  ~ImageToGStreamerNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  bool configurePipeline();
  void cleanupPipeline();

  std::string input_topic_;
  std::string pipeline_description_;
  std::string appsrc_name_;
  double target_fps_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  GstElement * pipeline_;
  GstElement * appsrc_;
  bool pipeline_ready_;
  bool caps_configured_;
  std::mutex gst_mutex_;
};

}  // namespace sodasim::gstreamer

#endif  // SODASIM_GSTREAMER_IMAGE_TO_GSTREAMER_NODE_HPP_
