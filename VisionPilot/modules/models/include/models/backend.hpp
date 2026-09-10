#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <models/auto_steer.hpp>

#include <string>

namespace visionpilot::models {

// One frame's worth of model outputs, plus how long each branch took.
// A merged backend cannot attribute one Run to three branches: it reports the
// whole Run duration in ad_ms and leaves as_ms and asp_ms at zero.
struct BackendOutputs {
    AutoDriveOutput drive;
    AutoSteerOutput steer;
    AutoSpeedOutput speed;
    double ad_ms{0}, as_ms{0}, asp_ms{0};
};

// How InferencePipeline reaches the networks. Implementations decide how many
// ONNX Runtime sessions exist; the pipeline does not care.
class ModelBackend {
public:
    virtual ~ModelBackend() = default;

    // prev_imn, curr_imn : ImageNet-normalised warped BEV, CHW, CHW_SIZE floats
    // curr_01            : /255 resized image, CHW, CHW_SIZE floats
    virtual BackendOutputs run(const float* prev_imn,
                               const float* curr_imn,
                               const float* curr_01) = 0;
};

// Resolve a weights filename against the local build tree, then the installed
// share directory. Throws std::runtime_error when neither exists.
std::string find_model(const std::string& filename);

}  // namespace visionpilot::models
