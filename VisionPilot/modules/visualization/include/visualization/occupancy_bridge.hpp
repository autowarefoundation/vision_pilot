#ifndef VISUALIZATION__OCCUPANCY_BRIDGE_HPP_
#define VISUALIZATION__OCCUPANCY_BRIDGE_HPP_

#include <common/types.hpp>
#include <models/inference.hpp>
#include <opencv2/core.hpp>
#include <visualization/occupancy_view.hpp>

class VisualInterface;

namespace visualization
{
namespace occupancy
{

// Convert stock VisionPilot pipeline outputs → Occupancy Scene.
// This is the only place that knows about InferenceFrameResult / Plan.
Scene make_scene(
  const visionpilot::models::InferenceFrameResult & result, const Plan & plan,
  const cv::Mat & H_resized, float ego_speed_ms = 0.f);

// make_scene() + render().
cv::Mat build_frame(
  const visionpilot::models::InferenceFrameResult & result, const Plan & plan,
  const cv::Mat & H_resized, float ego_speed_ms = 0.f);

// One-line upstream hook: build panel and push to VisualInterface::set_aux_frame.
void publish(
  VisualInterface * visual_interface, const visionpilot::models::InferenceFrameResult & result,
  const Plan & plan, const cv::Mat & H_resized, float ego_speed_ms = 0.f);

}  // namespace occupancy
}  // namespace visualization

#endif  // VISUALIZATION__OCCUPANCY_BRIDGE_HPP_
