#include "models/auto_drive.hpp"

#include <onnxruntime_run_options_config_keys.h>

#include <array>
#include <cmath>
#include <cstdio>

namespace visionpilot::models {

AutoDrive::AutoDrive(engine::OnnxEngine& engine, const std::string& model_path)
    : session_(engine.create_session(model_path, "autodrive_"))
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault))
    , frame_shape_{1, 3, NET_H, NET_W}
{
    Ort::AllocatorWithDefaultOptions alloc;
    const size_t n_in  = session_->GetInputCount();
    const size_t n_out = session_->GetOutputCount();

    in_name_strs_.resize(n_in);
    in_names_.resize(n_in);
    for (size_t i = 0; i < n_in; ++i) {
        in_name_strs_[i] = session_->GetInputNameAllocated(i, alloc).get();
        in_names_[i]     = in_name_strs_[i].c_str();
        printf("[AutoDrive] input[%zu]  = %s\n", i, in_names_[i]);
    }

    out_name_strs_.resize(n_out);
    out_names_.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
        out_names_[i]     = out_name_strs_[i].c_str();
        printf("[AutoDrive] output[%zu] = %s\n", i, out_names_[i]);
    }

    printf("[AutoDrive] Ready — %zu inputs, %zu outputs | "
           "frame [1, 3, %d, %d]\n", n_in, n_out, NET_H, NET_W);
}

AutoDriveOutput AutoDrive::infer(const float* prev_chw, const float* curr_chw)
{
    AutoDriveOutput out;

    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(
            mem_info_,
            const_cast<float*>(prev_chw), CHW_SIZE,
            frame_shape_.data(), frame_shape_.size()),
        Ort::Value::CreateTensor<float>(
            mem_info_,
            const_cast<float*>(curr_chw), CHW_SIZE,
            frame_shape_.data(), frame_shape_.size()),
    };

    std::vector<Ort::Value> results;
    try {
        Ort::RunOptions run_options;
        run_options.AddConfigEntry(kOrtRunOptionsConfigEnableMemoryArenaShrinkage, "gpu:0");
        results = session_->Run(
            run_options,
            in_names_.data(),  inputs.data(),    inputs.size(),
            out_names_.data(), out_names_.size());
    } catch (const Ort::Exception& e) {
        printf("[AutoDrive] Inference error: %s\n", e.what());
        return out;
    }

    if (results.size() < 3) {
        printf("[AutoDrive] Expected 3 outputs, got %zu\n", results.size());
        return out;
    }

    out.dist_normalized = results[0].GetTensorData<float>()[0];
    out.curvature_raw   = results[1].GetTensorData<float>()[0];
    out.flag_prob = 1.f / (1.f + std::exp(-results[2].GetTensorData<float>()[0]));
    out.valid     = true;
    return out;
}

}  // namespace visionpilot::models
