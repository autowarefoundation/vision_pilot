#include <common/utils.hpp>
#include <pilot/pilot.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace visionpilot::pilot
{

namespace
{

cv::Mat checked_homography(const cv::Mat & M, const char * name)
{
  if (M.rows != 3 || M.cols != 3 || M.channels() != 1)
    throw std::invalid_argument(std::string("Pilot: ") + name + " must be a 3x3 matrix");
  cv::Mat out;
  M.convertTo(out, CV_64F);
  return out;
}

}  // namespace

Pilot::Pilot(const Config & cfg, const cv::Mat & H, const cv::Mat & C)
: cfg_(cfg),
  H_(checked_homography(H, "H")),
  preprocessor_(C.empty() ? compute_preprocess_homography(H_) : checked_homography(C, "C")),
  engine_(std::make_unique<engine::OnnxEngine>(cfg_.engine)),
  pipeline_(std::make_unique<models::InferencePipeline>(*engine_, cfg_.inference))
{
  planner_.emplace(cfg_.speed_limit, cfg_.L, cfg_.mpc_max_cpu_time_s);
}

StepResult Pilot::step(const cv::Mat & frame_bgr, double ego_speed_mps, float dt_s)
{
  if (frame_bgr.empty() || frame_bgr.type() != CV_8UC3)
    throw std::invalid_argument("Pilot::step: frame must be a non-empty CV_8UC3 BGR image");

  StepResult out;
  const cv::Size net_size(models::AutoDrive::NET_W, models::AutoDrive::NET_H);
  preprocessor_.preprocess(frame_bgr, out.warped, out.resized, net_size);

  // Tell the pipeline how to project AutoSteer/AutoSpeed outputs back to
  // world when those networks run on the plain-resized image.
  if (frame_bgr.size() != frame_size_) {
    pipeline_->set_H_resized(H_, frame_bgr.size());
    frame_size_ = frame_bgr.size();
  }

  out.perception =
    pipeline_->process(out.warped, out.resized, static_cast<float>(ego_speed_mps), true, dt_s);
  if (!out.perception) return out;
  const auto & r = *out.perception;

  // has_cipo: tracker-based — true only when filter tracks a target closer
  // than the cut-off. cipo_raw_found alone must not gate the planner.
  out.has_cipo = r.cipo.valid && r.cipo.distance_m < cfg_.cipo_max_distance_m;
  const double cipo_v = out.has_cipo ? r.cipo.velocity_ms : cfg_.speed_limit;

  out.plan = planner_->compute_plan(
    r.lateral.cte_m, r.lateral.yaw_rad, r.lateral.curvature, ego_speed_mps, out.has_cipo, cipo_v,
    r.cipo.distance_m);
  out.command = Command{
    out.plan->steering.empty() ? 0.0 : out.plan->steering[1],
    out.plan->acceleration,
  };
  return out;
}

void Pilot::set_radar_points(std::vector<RadarPoint> points)
{
  pipeline_->set_radar_points(std::move(points));
}

void Pilot::reset()
{
  pipeline_->reset();
  planner_.emplace(cfg_.speed_limit, cfg_.L, cfg_.mpc_max_cpu_time_s);
  frame_size_ = {};
}

}  // namespace visionpilot::pilot
