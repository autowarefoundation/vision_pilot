#include "models/split_backend.hpp"

#include <chrono>
#include <future>
#include <utility>

namespace visionpilot::models {

namespace {
using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;
}  // namespace

SplitBackend::SplitBackend(engine::OnnxEngine& engine,
                           const std::string& precision)
    : auto_drive_(engine, find_model("autodrive_" + precision + ".onnx"))
    , auto_steer_(engine, find_model("autosteer_" + precision + ".onnx"))
    , auto_speed_(engine, find_model("autospeed_" + precision + ".onnx"))
{
}

BackendOutputs SplitBackend::run(const float* prev_imn,
                                 const float* curr_imn,
                                 const float* curr_01)
{
    auto f_drive = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_drive_.infer(prev_imn, curr_imn);
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });
    auto f_steer = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_steer_.infer(curr_01);
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });
    auto f_speed = std::async(std::launch::async, [&] {
        auto t = Clock::now();
        auto r = auto_speed_.infer(curr_01);
        return std::make_pair(std::move(r), Ms(Clock::now() - t).count());
    });

    auto [res_drive, ms_drive] = f_drive.get();
    auto [res_steer, ms_steer] = f_steer.get();
    auto [res_speed, ms_speed] = f_speed.get();

    BackendOutputs out;
    out.drive  = res_drive;
    out.steer  = res_steer;
    out.speed  = res_speed;
    out.ad_ms  = ms_drive;
    out.as_ms  = ms_steer;
    out.asp_ms = ms_speed;
    return out;
}

}  // namespace visionpilot::models
