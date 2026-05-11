#include "models/onnx_session.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace visionpilot::models {

// ─────────────────────────────────────────────────────────────────────────────
std::unique_ptr<Ort::Session> OnnxSessionFactory::create(
    Ort::Env&                env,
    const OnnxSessionConfig& cfg)
{
    if (cfg.provider == "cpu") {
        return createCpu(env, cfg.model_path);
    }
    if (cfg.provider == "cuda") {
        return createCuda(env, cfg.model_path, cfg.device_id);
    }
    if (cfg.provider == "tensorrt") {
        return createTensorRT(env, cfg);
    }
    throw std::runtime_error(
        "[OnnxSessionFactory] Unknown provider: '" + cfg.provider +
        "'. Valid options: cpu | cuda | tensorrt");
}

// ─── CPU ─────────────────────────────────────────────────────────────────────
std::unique_ptr<Ort::Session> OnnxSessionFactory::createCpu(
    Ort::Env& env, const std::string& model_path)
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    printf("[OnnxSessionFactory] CPU session → %s\n", model_path.c_str());
    return std::make_unique<Ort::Session>(env, model_path.c_str(), opts);
}

// ─── CUDA ────────────────────────────────────────────────────────────────────
std::unique_ptr<Ort::Session> OnnxSessionFactory::createCuda(
    Ort::Env& env, const std::string& model_path, int device_id)
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    printf("[OnnxSessionFactory] CUDA session (device %d) → %s\n",
           device_id, model_path.c_str());
    return std::make_unique<Ort::Session>(env, model_path.c_str(), opts);
}

// ─── TensorRT ────────────────────────────────────────────────────────────────
// Mirrors the proven pattern from the 0.9 release.
// Kept here for future use; engines call create() with provider="tensorrt".
std::unique_ptr<Ort::Session> OnnxSessionFactory::createTensorRT(
    Ort::Env& env, const OnnxSessionConfig& cfg)
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::filesystem::create_directories(cfg.cache_dir);

    const auto& api = Ort::GetApi();
    OrtTensorRTProviderOptionsV2* trt_opts = nullptr;
    Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt_opts));

    const std::string fp16_flag    = (cfg.precision == "fp16") ? "1" : "0";
    const std::string device_str   = std::to_string(cfg.device_id);
    const std::string ws_bytes_str = std::to_string(
        static_cast<size_t>(cfg.workspace_gb * 1024.0 * 1024.0 * 1024.0));
    const std::string full_prefix  = cfg.cache_prefix + cfg.precision + "_";

    const std::vector<const char*> keys = {
        "device_id",
        "trt_max_workspace_size",
        "trt_fp16_enable",
        "trt_engine_cache_enable",
        "trt_engine_cache_path",
        "trt_engine_cache_prefix",
        "trt_timing_cache_enable",
        "trt_timing_cache_path",
        "trt_builder_optimization_level",
        "trt_min_subgraph_size",
    };
    const std::vector<const char*> vals = {
        device_str.c_str(),
        ws_bytes_str.c_str(),
        fp16_flag.c_str(),
        "1",
        cfg.cache_dir.c_str(),
        full_prefix.c_str(),
        "1",
        cfg.cache_dir.c_str(),
        "5",
        "1",
    };

    Ort::ThrowOnError(api.UpdateTensorRTProviderOptions(
        trt_opts, keys.data(), vals.data(), keys.size()));

    opts.AppendExecutionProvider_TensorRT_V2(*trt_opts);

    // CUDA fallback for any subgraph TensorRT cannot handle
    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = cfg.device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    api.ReleaseTensorRTProviderOptions(trt_opts);

    printf("[OnnxSessionFactory] TensorRT session (%s, device %d) → %s\n",
           cfg.precision.c_str(), cfg.device_id, cfg.model_path.c_str());
    return std::make_unique<Ort::Session>(env, cfg.model_path.c_str(), opts);
}

}  // namespace visionpilot::models
