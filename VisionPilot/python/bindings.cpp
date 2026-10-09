// nanobind bindings for the VisionPilot loop step: `vision_pilot._core`.
//
// Images cross the boundary without copies: an input frame is wrapped as a
// cv::Mat over the NumPy buffer, and output images are NumPy views that hold a
// reference to the cv::Mat they came from. The GIL is released while a frame
// is processed, so a simulator thread is never blocked on inference.
#include <common/types.hpp>
#include <common/utils.hpp>
#include <models/inference.hpp>
#include <opencv2/imgproc.hpp>
#include <pilot/pilot.hpp>
#include <planning/lateral_planning.hpp>
#include <planning/longitudinal_planning.hpp>
#include <planning/planning.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace vp = visionpilot::pilot;
namespace vm = visionpilot::models;
namespace vf = visionpilot::fusion;
namespace ve = visionpilot::engine;

namespace
{

using Matrix3 = nb::ndarray<const double, nb::shape<3, 3>, nb::c_contig, nb::device::cpu>;
using ImageIn = nb::ndarray<const uint8_t, nb::shape<-1, -1, 3>, nb::c_contig, nb::device::cpu>;
using ImageOut = nb::ndarray<const uint8_t, nb::numpy, nb::shape<-1, -1, 3>>;
// Arrays returned from property getters carry their own owner capsule, which
// rules out the reference_internal policy def_prop_* would otherwise apply.
constexpr auto owned = nb::rv_policy::reference;

template <typename T>
using VectorOut = nb::ndarray<const T, nb::numpy, nb::shape<-1>>;

enum class ChannelOrder { RGB, BGR };

// A read-only NumPy view of `m` that keeps the pixel buffer alive through a
// reference-counted cv::Mat header owned by a capsule.
ImageOut image_view(const cv::Mat & m)
{
  if (m.empty()) return {};
  if (m.type() != CV_8UC3) throw std::logic_error("expected a CV_8UC3 image");
  auto * header = new cv::Mat(m);
  nb::capsule owner(header, [](void * p) noexcept { delete static_cast<cv::Mat *>(p); });
  const auto row_stride = static_cast<int64_t>(header->step[0]);
  return ImageOut(
    header->data, {static_cast<size_t>(header->rows), static_cast<size_t>(header->cols), 3}, owner,
    {row_stride, 3, 1});
}

template <typename T>
VectorOut<T> to_numpy(std::vector<T> v)
{
  auto * heap = new std::vector<T>(std::move(v));
  nb::capsule owner(heap, [](void * p) noexcept { delete static_cast<std::vector<T> *>(p); });
  return VectorOut<T>(heap->data(), {heap->size()}, owner);
}

template <typename T, size_t N>
VectorOut<T> to_numpy(const std::array<T, N> & a)
{
  return to_numpy(std::vector<T>(a.begin(), a.end()));
}

nb::ndarray<nb::numpy, double, nb::shape<-1, -1>> to_numpy_matrix(const cv::Mat & m)
{
  cv::Mat d;
  m.convertTo(d, CV_64F);
  auto * heap = new cv::Mat(d.isContinuous() ? d : d.clone());
  nb::capsule owner(heap, [](void * p) noexcept { delete static_cast<cv::Mat *>(p); });
  return {
    heap->ptr<double>(), {static_cast<size_t>(heap->rows), static_cast<size_t>(heap->cols)}, owner};
}

cv::Mat to_mat(const Matrix3 & a)
{
  return cv::Mat(3, 3, CV_64F, const_cast<double *>(a.data())).clone();
}

// The C++ Pilot is not reentrant; the binding releases the GIL during step(),
// so it serialises calls itself.
struct PyPilot
{
  PyPilot(const vp::Config & cfg, const cv::Mat & H, const cv::Mat & C) : pilot(cfg, H, C) {}

  vp::Pilot pilot;
  std::mutex mutex;
};

}  // namespace

NB_MODULE(_core, m)
{
  m.doc() = "Python bindings for the VisionPilot perception → planning loop.";

  m.attr("NET_WIDTH") = vm::AutoDrive::NET_W;
  m.attr("NET_HEIGHT") = vm::AutoDrive::NET_H;
  m.attr("PLANNER_HORIZON") = N;
  m.attr("PLANNER_DT_S") = dt;

  nb::enum_<ChannelOrder>(m, "ChannelOrder", "Channel order of an input image.")
    .value("RGB", ChannelOrder::RGB)
    .value("BGR", ChannelOrder::BGR);

  nb::enum_<Warning>(m, "Warning", "Driver warnings raised by the planner.")
    .value("NONE", Warning::None)
    .value("FCW", Warning::FCW, "Forward collision warning")
    .value("AEB", Warning::AEB, "Automatic emergency braking")
    .value("LLDW", Warning::LLDW, "Left lane departure warning")
    .value("RLDW", Warning::RLDW, "Right lane departure warning");

  // ── Configuration ─────────────────────────────────────────────────────────

  nb::class_<ve::Config>(m, "EngineConfig", "ONNX Runtime execution settings.")
    .def(
      "__init__",
      [](
        ve::Config * self, std::string provider, std::string precision, std::string cache_dir,
        double workspace_gb, int device_id) {
        new (self) ve::Config{
          std::move(provider), std::move(precision), std::move(cache_dir), workspace_gb, device_id};
      },
      nb::kw_only(), "provider"_a = ve::Config{}.provider, "precision"_a = ve::Config{}.precision,
      "cache_dir"_a = ve::Config{}.cache_dir, "workspace_gb"_a = ve::Config{}.workspace_gb,
      "device_id"_a = ve::Config{}.device_id)
    .def_rw("provider", &ve::Config::provider, "Execution provider: 'cpu', 'cuda' or 'tensorrt'.")
    .def_rw("precision", &ve::Config::precision, "TensorRT precision: 'fp32' or 'fp16'.")
    .def_rw("cache_dir", &ve::Config::cache_dir, "TensorRT engine cache directory.")
    .def_rw("workspace_gb", &ve::Config::workspace_gb, "TensorRT workspace size [GiB].")
    .def_rw("device_id", &ve::Config::device_id, "GPU index (cuda and tensorrt).");

  nb::class_<vm::Config>(m, "InferenceConfig", "Model and fusion settings.")
    .def(
      "__init__",
      [](
        vm::Config * self, std::string model_dir, std::string precision,
        std::optional<uint32_t> seed, float cte_bias_m, bool fusion_debug) {
        auto * c = new (self) vm::Config{};
        c->model_dir = std::move(model_dir);
        c->precision = std::move(precision);
        c->seed = seed;
        c->cte_bias_m = cte_bias_m;
        c->fusion_debug = fusion_debug;
      },
      nb::kw_only(), "model_dir"_a = vm::Config{}.model_dir, "precision"_a = vm::Config{}.precision,
      "seed"_a = nb::none(), "cte_bias_m"_a = vm::Config{}.cte_bias_m,
      "fusion_debug"_a = vm::Config{}.fusion_debug)
    .def_rw(
      "model_dir", &vm::Config::model_dir,
      "Directory with the <model>_<precision>.onnx weights. Empty: the C++ app's search path.")
    .def_rw("precision", &vm::Config::precision, "Model weights: 'fp32' or 'int8'.")
    .def_rw("seed", &vm::Config::seed, "Particle-filter RNG seed; None is nondeterministic.")
    .def_rw(
      "cte_bias_m", &vm::Config::cte_bias_m,
      "Camera lateral mounting offset subtracted from CTE [m].")
    .def_rw("fusion_debug", &vm::Config::fusion_debug);

  nb::class_<vp::Config>(m, "PilotConfig", "Everything a Pilot is built from.")
    .def(
      "__init__",
      [](
        vp::Config * self, std::optional<ve::Config> engine, std::optional<vm::Config> inference,
        double speed_limit_mps, double front_axle_to_cog_m, double cipo_max_distance_m,
        double mpc_max_cpu_time_s) {
        auto * c = new (self) vp::Config{};
        if (engine) c->engine = *engine;
        if (inference) c->inference = *inference;
        c->speed_limit = speed_limit_mps;
        c->L = front_axle_to_cog_m;
        c->cipo_max_distance_m = cipo_max_distance_m;
        c->mpc_max_cpu_time_s = mpc_max_cpu_time_s;
      },
      nb::kw_only(), "engine"_a = nb::none(), "inference"_a = nb::none(),
      "speed_limit_mps"_a = vp::Config{}.speed_limit, "front_axle_to_cog_m"_a = vp::Config{}.L,
      "cipo_max_distance_m"_a = vp::Config{}.cipo_max_distance_m,
      "mpc_max_cpu_time_s"_a = vp::Config{}.mpc_max_cpu_time_s)
    .def_rw("engine", &vp::Config::engine)
    .def_rw("inference", &vp::Config::inference)
    .def_rw("speed_limit_mps", &vp::Config::speed_limit, "Cruise speed on free road [m/s].")
    .def_rw("front_axle_to_cog_m", &vp::Config::L, "Front axle to centre of gravity [m].")
    .def_rw(
      "cipo_max_distance_m", &vp::Config::cipo_max_distance_m,
      "A tracked lead vehicle further than this is treated as free road [m].")
    .def_rw(
      "mpc_max_cpu_time_s", &vp::Config::mpc_max_cpu_time_s,
      "Lateral MPC solver budget [s]. The default is the vehicle's real-time budget, so plans\n"
      "depend on CPU speed; raise it (e.g. 1.0) for reproducible simulation.");

  // ── Perception outputs ────────────────────────────────────────────────────

  nb::class_<vm::AutoDriveOutput>(m, "AutoDriveOutput", "Raw AutoDrive outputs.")
    .def_ro("dist_normalized", &vm::AutoDriveOutput::dist_normalized)
    .def_ro("curvature_raw", &vm::AutoDriveOutput::curvature_raw)
    .def_ro("flag_prob", &vm::AutoDriveOutput::flag_prob, "CIPO probability.")
    .def_ro("valid", &vm::AutoDriveOutput::valid);

  nb::class_<vm::AutoSteerOutput>(m, "AutoSteerOutput", "Raw AutoSteer ego-path waypoints.")
    .def_prop_ro(
      "xp", [](const vm::AutoSteerOutput & o) { return to_numpy(o.xp); }, owned,
      "Normalised lateral position at each of 64 fixed image rows.")
    .def_prop_ro(
      "h_vector", [](const vm::AutoSteerOutput & o) { return to_numpy(o.h_vector); }, owned,
      "Per-waypoint confidence.")
    .def_ro("valid", &vm::AutoSteerOutput::valid);

  nb::class_<vm::Detection>(m, "Detection", "An AutoSpeed box in 1024x512 network pixels.")
    .def_ro("x1", &vm::Detection::x1)
    .def_ro("y1", &vm::Detection::y1)
    .def_ro("x2", &vm::Detection::x2)
    .def_ro("y2", &vm::Detection::y2)
    .def_ro("score", &vm::Detection::score)
    .def_ro("class_id", &vm::Detection::class_id)
    .def("__repr__", [](const vm::Detection & d) {
      return nb::str("Detection(class_id={}, score={:.3f}, box=({:.1f}, {:.1f}, {:.1f}, {:.1f}))")
        .format(d.class_id, d.score, d.x1, d.y1, d.x2, d.y2);
    });

  nb::class_<vm::AutoSpeedOutput>(m, "AutoSpeedOutput", "AutoSpeed detections after NMS.")
    .def_ro("detections", &vm::AutoSpeedOutput::detections)
    .def_ro("valid", &vm::AutoSpeedOutput::valid);

  using Lat = vf::LateralFusionEstimate;
  nb::class_<Lat>(m, "LateralEstimate", "Lateral fusion output (x forward, y left).")
    .def_ro("valid", &Lat::valid)
    .def_ro("cte_m", &Lat::cte_m, "Cross-track error [m]; positive: ego right of path.")
    .def_ro("cte_rate_mps", &Lat::cte_rate_mps)
    .def_ro("yaw_rad", &Lat::yaw_rad, "Heading error [rad]; positive: path heads left.")
    .def_ro("yaw_rate_rps", &Lat::yaw_rate_rps)
    .def_ro("cte_stddev_m", &Lat::cte_stddev_m)
    .def_ro("yaw_stddev_rad", &Lat::yaw_stddev_rad)
    .def_ro("curvature", &Lat::curvature, "Fused road curvature [1/m]; positive: left turn.")
    .def_ro("curv_stddev", &Lat::curv_stddev)
    .def_ro("path_valid", &Lat::path_valid)
    .def_ro("raw_cte_m", &Lat::raw_cte_m)
    .def_ro("raw_yaw_rad", &Lat::raw_yaw_rad)
    .def_ro("raw_path_curvature", &Lat::raw_path_curvature)
    .def_ro("raw_ad_curvature", &Lat::raw_ad_curvature)
    .def_ro("path_inliers", &Lat::path_inliers)
    .def_ro("path_points", &Lat::path_points)
    .def_ro("path_a", &Lat::path_a)
    .def_ro("path_b", &Lat::path_b)
    .def_ro("path_c", &Lat::path_c)
    .def_ro("path_x_min_m", &Lat::path_x_min_m)
    .def_ro("path_x_max_m", &Lat::path_x_max_m);

  using Cipo = vf::CIPOFusionEstimate;
  nb::class_<Cipo>(m, "CipoEstimate", "Closest in-path object (lead vehicle) estimate.")
    .def_ro("valid", &Cipo::valid)
    .def_ro("distance_m", &Cipo::distance_m)
    .def_ro("velocity_ms", &Cipo::velocity_ms, "Relative speed [m/s]; negative: approaching.")
    .def_ro("distance_stddev_m", &Cipo::distance_stddev_m)
    .def_ro("cipo_raw_found", &Cipo::cipo_raw_found)
    .def_ro("cipo_raw_dist_m", &Cipo::cipo_raw_dist_m)
    .def_ro("cut_in_detected", &Cipo::cut_in_detected);

  using Frame = vm::InferenceFrameResult;
  nb::class_<Frame>(m, "Perception", "Model outputs and fused estimates for one frame.")
    .def_ro("frame_id", &Frame::frame_id)
    .def_ro("pre_ms", &Frame::pre_ms)
    .def_ro("wall_ms", &Frame::wall_ms)
    .def_ro("auto_drive_ms", &Frame::ad_ms)
    .def_ro("auto_steer_ms", &Frame::as_ms)
    .def_ro("auto_speed_ms", &Frame::asp_ms)
    .def_ro("auto_drive", &Frame::auto_drive)
    .def_ro("auto_steer", &Frame::auto_steer)
    .def_ro("auto_speed", &Frame::auto_speed)
    .def_ro("cipo", &Frame::cipo)
    .def_ro("lateral", &Frame::lateral);

  // ── Planning ──────────────────────────────────────────────────────────────

  nb::class_<Plan>(m, "Plan", "Planner output.")
    .def_ro("acceleration", &Plan::acceleration, "Commanded acceleration [m/s^2].")
    .def_prop_ro(
      "steering", [](const Plan & p) { return to_numpy(p.steering); }, owned,
      "Front tyre angle over the MPC horizon [rad], positive left.")
    .def_ro("warnings", &Plan::warnings);

  nb::class_<vp::Command>(m, "Command", "The actuator command VisionPilot sends to the vehicle.")
    .def(nb::init<double, double>(), "steering_tyre_rad"_a, "acceleration_mps2"_a)
    .def_rw(
      "steering_tyre_rad", &vp::Command::steering_tyre_rad,
      "Front tyre angle [rad], positive left.")
    .def_rw("acceleration_mps2", &vp::Command::acceleration_mps2)
    .def("__repr__", [](const vp::Command & c) {
      return nb::str("Command(steering_tyre_rad={:.5f}, acceleration_mps2={:.4f})")
        .format(c.steering_tyre_rad, c.acceleration_mps2);
    });

  nb::class_<Planner>(m, "Planner", "Longitudinal IDM + lateral spatial MPC planner, standalone.")
    .def(
      nb::init<double, double, double>(), "speed_limit_mps"_a, "front_axle_to_cog_m"_a,
      "mpc_max_cpu_time_s"_a = LateralPlanner::kRealTimeBudgetS)
    .def(
      "compute_plan", &Planner::compute_plan, "cte"_a, "epsi"_a, "kappa"_a, "ego_v"_a, "has_cipo"_a,
      "cipo_v"_a, "cipo_distance"_a, nb::call_guard<nb::gil_scoped_release>());

  // ── The loop step ─────────────────────────────────────────────────────────

  nb::class_<vp::StepResult>(m, "StepResult", "Outputs of one Pilot.step().")
    .def_prop_ro(
      "ready", [](const vp::StepResult & r) { return r.command.has_value(); },
      "False on the first frame of an episode, which only primes the two-frame buffer.")
    .def_ro("perception", &vp::StepResult::perception)
    .def_ro("plan", &vp::StepResult::plan)
    .def_ro("command", &vp::StepResult::command)
    .def_ro("has_cipo", &vp::StepResult::has_cipo, "The planner followed a lead vehicle.")
    .def_prop_ro(
      "warped", [](const vp::StepResult & r) { return image_view(r.warped); }, owned,
      "AutoDrive input: BEV 1024x512 BGR, read-only view.")
    .def_prop_ro(
      "resized", [](const vp::StepResult & r) { return image_view(r.resized); }, owned,
      "AutoSteer/AutoSpeed input: top-cropped 1024x512 BGR, read-only view.");

  nb::class_<PyPilot>(
    m, "Pilot",
    "One VisionPilot loop iteration per call: preprocess → inference → fusion → planning.\n\n"
    "Stateful: feed frames in order and call reset() between episodes.")
    .def(
      "__init__",
      [](PyPilot * self, const vp::Config & config, const Matrix3 & H, std::optional<Matrix3> C) {
        const cv::Mat c = C ? to_mat(*C) : cv::Mat{};
        const cv::Mat h = to_mat(H);
        nb::gil_scoped_release release;
        new (self) PyPilot(config, h, c);
      },
      "config"_a, "ground_homography"_a, "preprocess_homography"_a = nb::none(),
      "ground_homography: raw image pixel -> ground (x forward, y left) [m].\n"
      "preprocess_homography: raw pixel -> 1024x512 BEV; derived from ground_homography when None.")
    .def(
      "step",
      [](
        PyPilot & self, const ImageIn & image, double ego_speed_mps, ChannelOrder channel_order,
        float dt_s) {
        const cv::Mat view(
          static_cast<int>(image.shape(0)), static_cast<int>(image.shape(1)), CV_8UC3,
          const_cast<uint8_t *>(image.data()));
        nb::gil_scoped_release release;
        // Converting RGB writes a new buffer: the caller's array is never modified.
        cv::Mat bgr;
        if (channel_order == ChannelOrder::RGB)
          cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
        else
          bgr = view;
        std::lock_guard<std::mutex> lock(self.mutex);
        return self.pilot.step(bgr, ego_speed_mps, dt_s);
      },
      // noconvert: a float image must not be silently truncated to uint8.
      "image"_a.noconvert(), "ego_speed_mps"_a, nb::kw_only(), "channel_order"_a, "dt_s"_a = 0.f,
      "Process one camera frame: a C-contiguous (H, W, 3) uint8 array.\n\n"
      "channel_order is required: CARLA and PIL images are RGB, OpenCV images BGR.\n"
      "dt_s is the time since the previous frame; <= 0 uses the filters' nominal period.")
    .def(
      "reset",
      [](PyPilot & self) {
        std::lock_guard<std::mutex> lock(self.mutex);
        self.pilot.reset();
      },
      "Forget all temporal state; the next step() starts a new episode.")
    .def_prop_ro("config", [](const PyPilot & self) { return self.pilot.config(); })
    .def_prop_ro(
      "ground_homography", [](const PyPilot & self) { return to_numpy_matrix(self.pilot.H()); },
      owned)
    .def_prop_ro(
      "preprocess_homography", [](const PyPilot & self) { return to_numpy_matrix(self.pilot.C()); },
      owned);

  m.def(
    "compute_preprocess_homography",
    [](const Matrix3 & H) { return to_numpy_matrix(compute_preprocess_homography(to_mat(H))); },
    "ground_homography"_a,
    "Raw pixel -> 1024x512 BEV homography for a camera with the given ground homography.");
}
