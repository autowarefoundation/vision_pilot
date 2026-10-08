#ifndef PILOT__PILOT_HPP_
#define PILOT__PILOT_HPP_

#include <common/types.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <models/inference.hpp>
#include <opencv2/core.hpp>
#include <planning/planning.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace visionpilot::pilot
{

struct Config
{
  engine::Config engine;
  models::Config inference;
  double speed_limit = 33.3;  // m/s
  double L = 2.860;           // front axle to CoG (m)
  // A tracked CIPO further than this is treated as free road by the planner.
  double cipo_max_distance_m = 150.0;
  // Lateral MPC solver budget. The default is the vehicle's real-time budget;
  // raise it in simulation for results independent of CPU speed.
  double mpc_max_cpu_time_s = LateralPlanner::kRealTimeBudgetS;
};

// The actuator command VisionPilot writes to the vehicle interface.
struct Command
{
  double steering_tyre_rad = 0.0;  // front tyre angle, +ve = left
  double acceleration_mps2 = 0.0;
};

struct StepResult
{
  // Network inputs for this frame: BEV 1024×512 and plain-resized 1024×512.
  cv::Mat warped;
  cv::Mat resized;

  // Empty on the very first frame: AutoDrive needs frames t-1 and t.
  std::optional<models::InferenceFrameResult> perception;
  std::optional<Plan> plan;
  std::optional<Command> command;
  bool has_cipo = false;  // the planner followed a lead vehicle
};

// One VisionPilot loop iteration, without any camera, vehicle or display I/O:
//
//   frame ─► ImagePreprocessor ─► InferencePipeline ─► Planner ─► Command
//
// Stateful (two-frame buffer, particle filters, curvature-rate history):
// feed frames in order, and reset() between episodes.
class Pilot
{
public:
  // H : raw camera pixel → ground (x forward, y left) [m], 3×3.
  // C : raw camera pixel → warped 1024×512 BEV; derived from H when omitted.
  Pilot(const Config & cfg, const cv::Mat & H, const cv::Mat & C = {});

  Pilot(const Pilot &) = delete;
  Pilot & operator=(const Pilot &) = delete;

  // frame_bgr     : raw camera frame, CV_8UC3 BGR
  // ego_speed_mps : current ego speed
  // dt_s          : time since the previous frame; <= 0 uses the nominal
  //                 camera period the fusion filters were tuned for
  StepResult step(const cv::Mat & frame_bgr, double ego_speed_mps, float dt_s = 0.f);

  // Radar scan consumed by the next step() (needs inference.long_fusion.radar_enabled).
  void set_radar_points(std::vector<RadarPoint> points);

  // Forget all temporal state; the next step() is a first frame again.
  void reset();

  const Config & config() const { return cfg_; }
  const models::InferencePipeline & pipeline() const { return *pipeline_; }
  const cv::Mat & H() const { return H_; }
  const cv::Mat & C() const { return preprocessor_.C_mat(); }

private:
  Config cfg_;
  cv::Mat H_;
  ImagePreprocessor preprocessor_;
  // The engine owns the ORT environment and must outlive the sessions the
  // pipeline's models hold, so it is declared (and constructed) first.
  std::unique_ptr<engine::OnnxEngine> engine_;
  std::unique_ptr<models::InferencePipeline> pipeline_;
  std::optional<Planner> planner_;
  cv::Size frame_size_;  // size H_resized was computed for
};

}  // namespace visionpilot::pilot

#endif  // PILOT__PILOT_HPP_
