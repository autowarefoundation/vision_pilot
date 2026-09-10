#pragma once

#include <engine/onnx_engine.hpp>
#include <models/backend.hpp>

#include <string>

namespace visionpilot::models {

// Today's arrangement: one ONNX Runtime session per network, all three run
// concurrently. Used for every provider except renesas, which permits only one
// NPU session per process.
class SplitBackend : public ModelBackend {
public:
    // precision selects the weights variant, e.g. "fp32" or "int8".
    SplitBackend(engine::OnnxEngine& engine, const std::string& precision);

    BackendOutputs run(const float* prev_imn,
                       const float* curr_imn,
                       const float* curr_01) override;

private:
    AutoDrive auto_drive_;
    AutoSteer auto_steer_;
    AutoSpeed auto_speed_;
};

}  // namespace visionpilot::models
