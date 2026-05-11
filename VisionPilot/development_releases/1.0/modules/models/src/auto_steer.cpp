#include "models/auto_steer.hpp"

#include <cstdio>
#include <cstring>

namespace visionpilot::models {

AutoSteerOnnx::AutoSteerOnnx(const OnnxSessionConfig& cfg)
    : env_(ORT_LOGGING_LEVEL_WARNING, "AutoSteer")
    , session_(OnnxSessionFactory::create(env_, cfg))
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    , input_shape_{1, 3, NET_H, NET_W}
{
    Ort::AllocatorWithDefaultOptions alloc;

    const size_t n_in  = session_->GetInputCount();
    const size_t n_out = session_->GetOutputCount();

    in_name_strs_.resize(n_in);
    in_names_.resize(n_in);
    for (size_t i = 0; i < n_in; ++i) {
        in_name_strs_[i] = session_->GetInputNameAllocated(i, alloc).get();
        in_names_[i]     = in_name_strs_[i].c_str();
        printf("[AutoSteerOnnx] input[%zu]  = %s\n", i, in_names_[i]);
    }

    // Expect two outputs: xp [1, 2, 64] and h_vector [1, 2, 64]
    out_name_strs_.resize(n_out);
    out_names_.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
        out_names_[i]     = out_name_strs_[i].c_str();
        printf("[AutoSteerOnnx] output[%zu] = %s\n", i, out_names_[i]);
    }

    printf("[AutoSteerOnnx] Initialized — %zu inputs, %zu outputs | "
           "frame shape [1, 3, %d, %d]\n",
           n_in, n_out, NET_H, NET_W);
}

AutoSteerOutput AutoSteerOnnx::infer(const float* image_chw)
{
    AutoSteerOutput out;

    auto input_tensor = Ort::Value::CreateTensor<float>(
        mem_info_,
        const_cast<float*>(image_chw), CHW_SIZE,
        input_shape_.data(), input_shape_.size());

    std::vector<Ort::Value> results;
    try {
        results = session_->Run(
            Ort::RunOptions{nullptr},
            in_names_.data(),  &input_tensor, 1,
            out_names_.data(), out_names_.size());
    } catch (const Ort::Exception& e) {
        printf("[AutoSteerOnnx] Inference error: %s\n", e.what());
        return out;
    }

    if (results.size() < 2) {
        printf("[AutoSteerOnnx] Expected 2 outputs, got %zu\n", results.size());
        return out;
    }

    // Output 0 — xp  : [1, 2, 64] squeezed → 128 floats
    std::memcpy(out.xp.data(),
                results[0].GetTensorData<float>(),
                out.xp.size() * sizeof(float));

    // Output 1 — h_vector : [1, 2, 64] squeezed → 128 floats
    std::memcpy(out.h_vector.data(),
                results[1].GetTensorData<float>(),
                out.h_vector.size() * sizeof(float));

    out.valid = true;
    return out;
}

}  // namespace visionpilot::models
