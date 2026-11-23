#include "engine/onnx_engine.hpp"

#include <logging/console.hpp>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace visionpilot::engine
{
    OnnxEngine::OnnxEngine(const Config& cfg)
        : env_(ORT_LOGGING_LEVEL_WARNING, "VisionPilot")
          , cfg_(cfg)
    {
        if (cfg_.provider == "cuda")
        {
            logging::log_info("[OnnxEngine] provider=" + cfg_.provider
                              + "  device=" + std::to_string(cfg_.device_id));
        }
        else if (cfg_.provider == "tensorrt")
        {
            logging::log_info("[OnnxEngine] provider=" + cfg_.provider
                              + "  device=" + std::to_string(cfg_.device_id)
                              + "  precision=" + cfg_.precision
                              + "  workspace=" + logging::fmt_fixed(cfg_.workspace_gb, 1) + "GB"
                              + "  cache=" + cfg_.cache_dir);
        }
        else
        {
            logging::log_info("[OnnxEngine] provider=" + cfg_.provider);
        }
    }

    // ─── Public entry point ───────────────────────────────────────────────────────

    std::unique_ptr<Ort::Session> OnnxEngine::create_session(
        const std::string& model_path,
        const std::string& cache_prefix) const
    {
        if (cfg_.provider == "cpu")
        {
            return create_cpu_session(model_path);
        }
        if (cfg_.provider == "cuda")
        {
            return create_cuda_session(model_path);
        }
        if (cfg_.provider == "tensorrt")
        {
            return create_tensorrt_session(model_path, cache_prefix);
        }
        throw std::runtime_error(
            "[OnnxEngine] Unknown provider '" + cfg_.provider +
            "'. Valid: cpu | cuda | tensorrt");
    }

    // ─── CPU ─────────────────────────────────────────────────────────────────────

    std::unique_ptr<Ort::Session> OnnxEngine::create_cpu_session(
        const std::string& model_path) const
    {
        Ort::SessionOptions opts;
        static_cast<void>(opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL));

        logging::log_info("[OnnxEngine] Creating CPU session → " + model_path + "\n");
        return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
    }

    // ─── CUDA ────────────────────────────────────────────────────────────────────

    std::unique_ptr<Ort::Session> OnnxEngine::create_cuda_session(
        const std::string& model_path) const
    {
        Ort::SessionOptions opts;
        static_cast<void>(opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL));

        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = cfg_.device_id;
        static_cast<void>(opts.AppendExecutionProvider_CUDA(cuda_opts));

        logging::log_info("[OnnxEngine] Creating CUDA session (device "
                          + std::to_string(cfg_.device_id) + ") → " + model_path + "\n");
        return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
    }

    // ─── TensorRT ────────────────────────────────────────────────────────────────

    std::unique_ptr<Ort::Session> OnnxEngine::create_tensorrt_session(
        const std::string& model_path,
        const std::string& cache_prefix) const
    {
        std::error_code ec;
        const bool created = std::filesystem::create_directories(cfg_.cache_dir, ec);
        if (ec)
        {
            logging::log_error("[OnnxEngine] Cannot create TRT cache directory "
                               + cfg_.cache_dir + ": " + ec.message());
            throw std::filesystem::filesystem_error(
                "create_directories", std::filesystem::path(cfg_.cache_dir), ec);
        }
        if (created)
        {
            logging::log_info("[OnnxEngine] Created TRT cache directory: " + cfg_.cache_dir);
        }

        Ort::SessionOptions opts;
        static_cast<void>(opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL));

        const auto& api = Ort::GetApi();
        OrtTensorRTProviderOptionsV2* trt_opts = nullptr;
        Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt_opts));

        const std::string fp16_flag = (cfg_.precision == "fp16") ? "1" : "0";
        const std::string device_str = std::to_string(cfg_.device_id);
        const std::string ws_str = std::to_string(
            static_cast<size_t>(cfg_.workspace_gb * 1024.0 * 1024.0 * 1024.0));
        // Per-model prefix lets each model cache its own TRT engine file
        const std::string full_prefix = cache_prefix + cfg_.precision + "_";

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
            ws_str.c_str(),
            fp16_flag.c_str(),
            "1",
            cfg_.cache_dir.c_str(),
            full_prefix.c_str(),
            "1",
            cfg_.cache_dir.c_str(),
            "5",
            "1",
        };

        Ort::ThrowOnError(api.UpdateTensorRTProviderOptions(
            trt_opts, keys.data(), vals.data(), keys.size()));

        static_cast<void>(opts.AppendExecutionProvider_TensorRT_V2(*trt_opts));

        // CUDA fallback for any subgraph TRT cannot handle
        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = cfg_.device_id;
        static_cast<void>(opts.AppendExecutionProvider_CUDA(cuda_opts));

        api.ReleaseTensorRTProviderOptions(trt_opts);

        logging::log_info("[OnnxEngine] Creating TensorRT session (" + cfg_.precision
                          + ", device " + std::to_string(cfg_.device_id)
                          + ", prefix=" + full_prefix + ") → " + model_path + "\n");

        return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
    }
} // namespace visionpilot::engine
