#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>

namespace visionpilot::engine {

// Configuration that governs how the engine creates sessions.
// One EngineConfig instance is typically shared across all models in main().
struct Config {
    // Execution provider: "cpu" | "cuda" | "tensorrt"
    std::string provider     = "cpu";

    // Used only when provider == "tensorrt"
    std::string precision    = "fp32";   // "fp32" | "fp16"
    std::string cache_dir    = "/tmp/visionpilot_trt_cache";
    double      workspace_gb = 1.0;

    // GPU device index (cuda and tensorrt)
    int device_id = 0;
};

// OnnxEngine owns the ORT environment and carries execution-provider config.
// Models call create_session() once in their constructor and hold the returned
// session for the lifetime of the model object.
//
// Adding TensorRT native (non-ORT) support later means adding a TrtEngine
// class with the same create_session() signature — models do not change.
class OnnxEngine {
public:
    explicit OnnxEngine(const Config& cfg);

    // Create an ORT session for the ONNX model at model_path.
    // The cache_prefix distinguishes per-model TRT engine cache files.
    std::unique_ptr<Ort::Session> create_session(
        const std::string& model_path,
        const std::string& cache_prefix = "model_") const;

    // Read-only access to config (models may inspect provider, etc.)
    const Config& config() const { return cfg_; }

private:
    std::unique_ptr<Ort::Session> create_cpu_session(
        const std::string& model_path) const;

    std::unique_ptr<Ort::Session> create_cuda_session(
        const std::string& model_path) const;

    std::unique_ptr<Ort::Session> create_tensorrt_session(
        const std::string& model_path,
        const std::string& cache_prefix) const;

    // Env must outlive all sessions created from it.
    // mutable because ORT session creation is logically const on the engine.
    mutable Ort::Env env_;
    Config     cfg_;
};

}  // namespace visionpilot::engine
