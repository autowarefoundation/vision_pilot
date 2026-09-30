#ifndef VISUALIZATION__OCCUPANCY_VIEW_HPP_
#define VISUALIZATION__OCCUPANCY_VIEW_HPP_

#include <opencv2/core.hpp>

#include <vector>

namespace visualization
{
namespace occupancy
{

// Self-contained scene for the heuristic Occupancy BEV panel.
// Built by occupancy_bridge from pipeline outputs — does not depend on
// ProductionView, so upstream HUD code stays untouched.
struct Detection
{
  float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;
  float score = 0.f;
  int class_id = 0;
};

struct Scene
{
  bool path_valid = false;
  float path_a = 0.f;
  float path_b = 0.f;
  float path_c = 0.f;

  // LateralFusion: +cte = ego right of path; +yaw = path heading left.
  // Used to animate ego into an adjacent lane in the Occupancy view.
  float cte_m = 0.f;
  float yaw_rad = 0.f;

  // Resized-px → world (CV_32F). Empty ⇒ no detection projection.
  cv::Mat H_px2world;

  // AutoSteer ego-lane samples in world metres (x_fwd, y_lat).
  std::vector<cv::Point2f> lane_world;

  std::vector<Detection> detections;

  bool cipo_valid = false;
  float cipo_distance_m = 0.f;

  bool ad_cipo_only = false;
  float ad_distance_m = 0.f;

  // Radar in the same ego frame as the path (X forward, Y left).
  // Occupancy paints returns only; vehicle boxes remain camera-derived.
  bool radar_enabled = false;
  struct RadarReturn
  {
    float x = 0.f;
    float y = 0.f;
    float range_rate = 0.f;
    bool in_match = false;
    bool moving = false;
  };
  std::vector<RadarReturn> radar_points;
};

// Heuristic 3D occupancy / BEV panel (separate window).
// Not a neural occupancy network.
cv::Mat render(const Scene & scene);

// Interactive camera controls for the Occupancy window.
void on_mouse(int event, int x, int y, int flags, void * userdata);
void on_key(int key);

}  // namespace occupancy
}  // namespace visualization

#endif  // VISUALIZATION__OCCUPANCY_VIEW_HPP_
