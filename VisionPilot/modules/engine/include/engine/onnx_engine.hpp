#pragma once

#include <onnxruntime_cxx_api.h>

#include <map>
#include <memory>
#include <string>

namespace visionpilot::engine {

// Configuration that governs how the engine creates sessions.
// One EngineConfig instance is typically shared across all models in main().
struct Config {
    // Execution provider: "cpu" | "cuda" | "tensorrt" | "renesas"
    std::string provider     = "cpu";

    // Used only when provider == "tensorrt"
    std::string precision    = "fp32";   // "fp32" | "fp16"
    std::string cache_dir    = "/tmp/visionpilot_trt_cache";
    double      workspace_gb = 1.0;

    // GPU device index (cuda and tensorrt)
    int device_id = 0;

    // ─── renesas only ────────────────────────────────────────────────────────
    // Directory holding one compiled artifact set: nnx/, fused_subgraphs/,
    // and legalized_*.onnx. Used in place of a .onnx path.
    std::string artifacts_dir;

    // Directory holding the vendor runtime's arc_prog binaries. Python
    // discovers this through the wheel's package path; a C++ process cannot,
    // so it is explicit configuration.
    std::string arc_prog_path;

    // Minimum node count that must be placed on the NPU at startup.
    // 0 means "at least one". See the offload gate in MergedBackend.
    int require_npu_nodes = 0;
};

// One compiled artifact set, resolved the same way the vendor toolchain
// validates an artifact set.
struct RenesasArtifacts {
    std::string model;         // legalized_*.onnx, qdq-inserted preferred
    std::string manifest;      // <dir>/nnx/manifest.json
    std::string base;          // PARENT of <dir> — see the comment below
    bool        qdq_inserted = false;
};

// Validate an artifacts directory and resolve its parts.
//
// base is deliberately the artifacts directory's PARENT: compiled artifacts
// record absolute compile-host paths that the backend opens literally, so the
// mount must keep the directory name. Getting this wrong surfaces as
// "nnx load failed:" naming a path that exists only on the compile host.
//
// Throws std::runtime_error naming every missing part.
RenesasArtifacts resolve_renesas_artifacts(const std::string& artifacts_dir);

// Count the nodes each execution provider actually ran, from an ONNX Runtime
// profile JSON. Only "*_kernel_time" events are counted; fence and session
// events are not node executions.
//
// This is the only API-supported way to observe node placement from C++.
// Ort::GetAvailableProviders() is global and would report Renesas as available
// even when every node ran on the CPU.
//
// Throws std::runtime_error, naming profile_json_path, when the path is
// empty, the file cannot be opened, the JSON is malformed, or the parsed
// JSON is not a top-level array (ORT profiling output always is -- treating
// that case as "zero events found" would let a wrong-file/environment
// problem masquerade as "the NPU did no work"). An event missing "name",
// "args", or "args.provider", or with a non-string value in either field,
// is skipped rather than counted; it is never treated as fatal, since a
// skip can only ever lower a provider's count, never raise it.
std::map<std::string, int> parse_profile_providers(
    const std::string& profile_json_path);

// OnnxEngine owns the ORT environment and carries execution-provider config.
// Models call create_session() once in their constructor and hold the returned
// session for the lifetime of the model object.
//
// Adding TensorRT native (non-ORT) support later means adding a TrtEngine
// class with the same create_session() signature — models do not change.
class OnnxEngine {
public:
    explicit OnnxEngine(const Config& cfg);

    // Create an ORT session for the model at model_path.
    // For every provider except renesas, model_path is a .onnx file path.
    // For provider == renesas, model_path is instead a compiled artifacts
    // directory (see resolve_renesas_artifacts).
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

    std::unique_ptr<Ort::Session> create_renesas_session(
        const std::string& model_path) const;

    // Env must outlive all sessions created from it.
    // mutable because ORT session creation is logically const on the engine.
    mutable Ort::Env env_;
    Config     cfg_;
};

}  // namespace visionpilot::engine
