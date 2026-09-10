#include "engine/onnx_engine.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace visionpilot::engine {

OnnxEngine::OnnxEngine(const Config& cfg)
    : env_(ORT_LOGGING_LEVEL_WARNING, "VisionPilot")
    , cfg_(cfg)
{
    printf("[OnnxEngine] provider=%s", cfg_.provider.c_str());
    if (cfg_.provider == "tensorrt" || cfg_.provider == "cuda") {
        printf("  device=%d", cfg_.device_id);
    }
    if (cfg_.provider == "tensorrt") {
        printf("  precision=%s  workspace=%.1fGB  cache=%s",
               cfg_.precision.c_str(), cfg_.workspace_gb, cfg_.cache_dir.c_str());
    }
    if (cfg_.provider == "renesas") {
        printf("  artifacts=%s", cfg_.artifacts_dir.c_str());
    }
    printf("\n");
}

// ─── Renesas artifact resolution ─────────────────────────────────────────────

RenesasArtifacts resolve_renesas_artifacts(const std::string& artifacts_dir)
{
    namespace fs = std::filesystem;

    if (artifacts_dir.empty()) {
        throw std::runtime_error(
            "[OnnxEngine] engine.artifacts_dir is not configured");
    }

    const fs::path dir(artifacts_dir);
    if (!fs::exists(dir)) {
        throw std::runtime_error(
            "[OnnxEngine] artifacts directory does not exist: " + artifacts_dir);
    }
    if (!fs::is_directory(dir)) {
        throw std::runtime_error(
            "[OnnxEngine] artifacts directory exists but is not a directory: " +
            artifacts_dir);
    }

    std::vector<std::string> missing;
    const fs::path nnx  = dir / "nnx";
    const fs::path fused = dir / "fused_subgraphs";
    const fs::path manifest = nnx / "manifest.json";

    if (!fs::is_directory(nnx)) {
        // manifest.json necessarily cannot exist either; report only the
        // missing directory rather than two entries for one root cause.
        missing.push_back(nnx.string());
    } else if (!fs::is_regular_file(manifest)) {
        missing.push_back(manifest.string());
    }
    if (!fs::is_directory(fused)) missing.push_back(fused.string());

    // Collect every candidate so the pick is deterministic regardless of
    // directory_iterator's unspecified enumeration order. Multiple matches
    // are not an error -- the rule is "at least one legalized_*.onnx" -- so
    // we sort and take the lexicographically first rather than throwing.
    std::vector<std::string> qdq_candidates;
    std::vector<std::string> plain_candidates;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() != ".onnx") continue;
        const std::string name = e.path().filename().string();
        if (name.rfind("qdq_inserted_legalized_", 0) == 0) {
            qdq_candidates.push_back(e.path().string());
        } else if (name.rfind("legalized_", 0) == 0) {
            plain_candidates.push_back(e.path().string());
        }
    }

    std::string legalized;
    bool qdq = false;
    if (!qdq_candidates.empty()) {
        // Prefer the qdq-inserted variant, matching the vendor's artifact
        // check.
        std::sort(qdq_candidates.begin(), qdq_candidates.end());
        legalized = qdq_candidates.front();
        qdq = true;
    } else if (!plain_candidates.empty()) {
        std::sort(plain_candidates.begin(), plain_candidates.end());
        legalized = plain_candidates.front();
    } else {
        missing.push_back((dir / "legalized_*.onnx").string());
    }

    if (!missing.empty()) {
        std::string msg =
            "[OnnxEngine] incomplete artifact set in " + artifacts_dir +
            ". Missing:";
        for (const auto& m : missing) msg += "\n  " + m;
        throw std::runtime_error(msg);
    }

    // base is the artifacts directory's PARENT, derived from a normalised
    // absolute copy rather than from `dir` directly. A configured path with a
    // trailing separator (".../v7_artifacts/") has an empty filename, so
    // parent_path() would return the artifacts directory itself; a relative
    // single-component path ("v7_artifacts") would make it empty outright.
    // Either would hand the execution provider a base_path that resolves the
    // artifacts' recorded absolute paths against the wrong root -- exactly
    // the "nnx load failed:" failure this field exists to avoid.
    fs::path base_dir = fs::absolute(dir).lexically_normal();
    if (base_dir.filename().empty()) base_dir = base_dir.parent_path();

    RenesasArtifacts a;
    a.model        = legalized;
    a.manifest     = manifest.string();
    a.base         = base_dir.parent_path().string();
    a.qdq_inserted = qdq;
    return a;
}

// ─── NPU offload profile parsing ─────────────────────────────────────────────

std::map<std::string, int> parse_profile_providers(
    const std::string& profile_json_path)
{
    if (profile_json_path.empty()) {
        throw std::runtime_error(
            "[OnnxEngine] profile_json_path is empty -- profiling was "
            "never enabled for this session, or EndProfilingAllocated() "
            "was never called");
    }

    std::ifstream in(profile_json_path);
    if (!in) {
        throw std::runtime_error(
            "[OnnxEngine] cannot open profile " + profile_json_path);
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(
            "[OnnxEngine] malformed profile JSON " + profile_json_path +
            ": " + e.what());
    }

    // ORT profiling output is always a top-level array of events. Anything
    // else means this is not the file the gate thinks it is -- reporting an
    // empty histogram here would let the gate announce "the NPU did no
    // work" for what is actually a wrong-file/environment problem.
    if (!j.is_array()) {
        throw std::runtime_error(
            "[OnnxEngine] profile is not a JSON array (ORT profiling "
            "output is always a top-level array): " + profile_json_path);
    }

    std::map<std::string, int> hist;
    for (const auto& ev : j) {
        if (!ev.is_object() || !ev.contains("name") ||
            !ev["name"].is_string()) {
            continue;
        }
        const auto name = ev["name"].get<std::string>();

        static const std::string kSuffix = "_kernel_time";
        if (name.size() <= kSuffix.size() ||
            name.compare(name.size() - kSuffix.size(), kSuffix.size(),
                         kSuffix) != 0) {
            continue;
        }
        if (!ev.contains("args") || !ev["args"].is_object()) continue;
        const auto& args = ev["args"];
        if (!args.contains("provider") || !args["provider"].is_string()) {
            continue;
        }

        ++hist[args["provider"].get<std::string>()];
    }
    return hist;
}

// ─── Public entry point ───────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_session(
    const std::string& model_path,
    const std::string& cache_prefix) const
{
    if (cfg_.provider == "cpu") {
        return create_cpu_session(model_path);
    }
    if (cfg_.provider == "cuda") {
        return create_cuda_session(model_path);
    }
    if (cfg_.provider == "tensorrt") {
        return create_tensorrt_session(model_path, cache_prefix);
    }
    if (cfg_.provider == "renesas") {
        return create_renesas_session(model_path);
    }
    throw std::runtime_error(
        "[OnnxEngine] Unknown provider '" + cfg_.provider +
        "'. Valid: cpu | cuda | tensorrt | renesas");
}

// ─── CPU ─────────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_cpu_session(
    const std::string& model_path) const
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);

    printf("[OnnxEngine] Creating CPU session → %s\n", model_path.c_str());
    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

// ─── CUDA ────────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_cuda_session(
    const std::string& model_path) const
{
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = cfg_.device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    printf("[OnnxEngine] Creating CUDA session (device %d) → %s\n",
           cfg_.device_id, model_path.c_str());
    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

// ─── TensorRT ────────────────────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_tensorrt_session(
    const std::string& model_path,
    const std::string& cache_prefix) const
{
    std::filesystem::create_directories(cfg_.cache_dir);

    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    const auto& api = Ort::GetApi();
    OrtTensorRTProviderOptionsV2* trt_opts = nullptr;
    Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt_opts));

    const std::string fp16_flag   = (cfg_.precision == "fp16") ? "1" : "0";
    const std::string device_str  = std::to_string(cfg_.device_id);
    const std::string ws_str      = std::to_string(
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

    opts.AppendExecutionProvider_TensorRT_V2(*trt_opts);

    // CUDA fallback for any subgraph TRT cannot handle
    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = cfg_.device_id;
    opts.AppendExecutionProvider_CUDA(cuda_opts);

    api.ReleaseTensorRTProviderOptions(trt_opts);

    printf("[OnnxEngine] Creating TensorRT session (%s, device %d, prefix=%s) → %s\n",
           cfg_.precision.c_str(), cfg_.device_id,
           full_prefix.c_str(), model_path.c_str());

    return std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);
}

// ─── Renesas (R-Car X5H NPU) ─────────────────────────────────────────────────

std::unique_ptr<Ort::Session> OnnxEngine::create_renesas_session(
    const std::string& model_path) const
{
    if (cfg_.arc_prog_path.empty()) {
        throw std::runtime_error(
            "[OnnxEngine] engine.arc_prog_path is required for the renesas "
            "provider");
    }
    if (!std::filesystem::is_directory(cfg_.arc_prog_path)) {
        throw std::runtime_error(
            "[OnnxEngine] engine.arc_prog_path is not a directory: " +
            cfg_.arc_prog_path);
    }

    const auto art = resolve_renesas_artifacts(model_path);

    Ort::SessionOptions opts;
    // Disabled unconditionally, not only for the qdq-inserted variant: both
    // model variants are compiled ahead of time against a fixed node
    // structure, so ORT rewriting either graph risks the execution provider
    // no longer matching its compiled subgraphs and silently shedding them
    // to CPU.
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);

    // Profiling powers the startup offload gate: the emitted JSON records the
    // execution provider each node actually ran on. There is no C++ API to
    // query node placement directly.
    //
    // The prefix is deliberately anchored under the system temp directory
    // rather than left as a bare relative name: a relative prefix resolves
    // against the process's current working directory, which may be
    // read-only in a container. If the profiler cannot open its output
    // there, EndProfilingAllocated() still returns a path -- to a file that
    // was never written -- and the gate would refuse to start on a
    // perfectly healthy NPU instead of catching a genuine offload failure.
    const std::string profile_prefix =
        (std::filesystem::temp_directory_path() / "visionpilot_renesas_profile")
            .string();
    opts.EnableProfiling(profile_prefix.c_str());

    const std::unordered_map<std::string, std::string> po = {
        {"mode",          "runtime"},
        {"manifest_path", art.manifest},
        {"base_path",     art.base},
        {"arc_prog_path", cfg_.arc_prog_path},
        {"run_rtt",       "false"},
    };
    // The generic AppendExecutionProvider overload takes the option map
    // directly; no key/value array marshalling is needed. CPU needs no
    // explicit append here: it is ORT's implicit last-resort EP, so any
    // node the Renesas EP does not claim already falls back to it.
    opts.AppendExecutionProvider("RenesasExecutionProvider", po);

    printf("[OnnxEngine] Creating Renesas session → %s\n"
           "             manifest=%s\n"
           "             base=%s (parent of the artifacts dir)\n"
           "             qdq_inserted=%s\n",
           art.model.c_str(), art.manifest.c_str(), art.base.c_str(),
           art.qdq_inserted ? "yes" : "no");

    return std::make_unique<Ort::Session>(env_, art.model.c_str(), opts);
}

}  // namespace visionpilot::engine
