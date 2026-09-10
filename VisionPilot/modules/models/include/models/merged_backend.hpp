#pragma once

#include <engine/onnx_engine.hpp>
#include <models/backend.hpp>
#include <models/merged_contract.hpp>
#include <onnxruntime_cxx_api.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace visionpilot::models {

// An output's declared shape and element type, read from the session without
// running it (Ort::Session::GetOutputTypeInfo). Lets the structural checks
// below run either against a real session (the constructor) or a hand-built
// map (tests), with no ONNX Runtime session required for the latter.
struct DeclaredOutput {
    std::vector<int64_t>      shape;
    ONNXTensorElementDataType dtype = ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED;
};

// Every output name the contract mentions exists in output_names; every
// passthrough entry is one this backend knows how to route (steer_lane_value
// or steer_height); steer_height -- the only source of
// AutoSteerOutput::h_vector -- is present; the ego path is supplied exactly
// once, either as a steer_lane_value passthrough (v6) or a steer_xp rule
// (v7); and no rewritten output is left unhandled -- a session exposing
// drive_head_raw requires a head rule, and one exposing any speed_l<N>_box
// requires a speed rule, without which AutoDrive/AutoSpeed would stay
// permanently invalid and longitudinal fusion would read that as a clear
// road. A pure function of the parsed contract and the session's output
// name list, so it needs no session and is unit-testable on its own.
// Throws std::runtime_error naming the problem: the missing/unknown/
// duplicated output(s), alongside the full session output list where that
// helps diagnose a v6/v7 mismatch.
void validate_contract_names(const MergedContract&           contract,
                             const std::vector<std::string>& output_names);

// Every contract-named output's declared shape and element type agree with
// what its consumer requires at run time: the speed levels' box/cls geometry
// and their cross-level agreement on the class count (guards
// assemble_speed()'s box_count/cls_count and class-count checks before either
// can ever trip on frame 1), the head tensor's row count (apply_head), the
// steer_xp logits' row count (apply_steer_xp), and the passthrough tensors'
// fixed 64-element size (copy_64). Every one of those consumers calls
// GetTensorData<float>(), which throws Ort::Exception -- not
// std::runtime_error -- on a non-float tensor, so element type is checked
// here too.
//
// declared must already contain every name the contract mentions; callers
// run validate_contract_names() first to guarantee that. A shape's leading
// dimension is treated as a possibly-symbolic batch axis and excluded from
// the element count; every other dimension must be a positive, statically
// known size, or the output's geometry cannot be verified here and this
// throws saying so.
//
// Throws std::runtime_error naming the output, what was found, and what was
// required.
void validate_output_shapes(
    const MergedContract&                                  contract,
    const std::unordered_map<std::string, DeclaredOutput>& declared);

// True when output_names holds every output run()'s plain-merged branch
// resolves: steer_xp and steer_h_vector, so lateral fusion has a real ego
// path and AutoSteerOutput::valid is not set from a half-filled frame; plus
// drive_distance, drive_curvature, drive_flag_logit and speed_output, whose
// absence would leave AutoDriveOutput/AutoSpeedOutput permanently invalid.
// find_output() returns nullptr for an absent name and run() then skips that
// output with no message, so every name is required here instead.
// Throws std::runtime_error naming what is missing.
void validate_plain_merged_names(const std::vector<std::string>& output_names);

// The NPU offload gate's decision, factored out of verify_offload() so it is
// testable without a Renesas execution provider: hist is a per-provider node
// histogram (from engine::parse_profile_providers), and required is the
// configured engine.require_npu_nodes value as-is, including 0.
//
// required <= 0 means "require at least one" -- 0 is
// engine::Config::require_npu_nodes's default and must not be read as
// "require none". A configured positive value is used as the floor
// directly. Returns the number of nodes actually placed on
// RenesasExecutionProvider when the gate passes.
//
// Throws std::runtime_error naming both the found and required node counts
// when hist has no RenesasExecutionProvider entry, or fewer nodes on it
// than the floor -- the CPU execution provider is a silent fallback, so
// neither case may be treated as success.
int check_offload(const std::map<std::string, int>& hist, int required);

// All three networks in one ONNX Runtime session. Required for the Renesas
// execution provider, which permits only one NPU session per process.
//
// Inputs, fixed by the merged graph:
//   input             [1,3,512,1024]  /255 resized, shared by steer and speed
//   drive_image_prev  [1,3,512,1024]  ImageNet-normalised warped BEV, t-1
//   drive_image_curr  [1,3,512,1024]  ImageNet-normalised warped BEV, t
class MergedBackend : public ModelBackend {
public:
    static constexpr int NET_H    = 512;
    static constexpr int NET_W    = 1024;
    static constexpr int CHW_SIZE = 3 * NET_H * NET_W;

    // model_path   : the .onnx to load. For the renesas provider this is the
    //                artifacts directory; OnnxEngine resolves it.
    // contract_path: "" means plain-merged mode. A rewrite signature output
    //                with no contract is a startup error.
    MergedBackend(engine::OnnxEngine& engine,
                  const std::string&  model_path,
                  const std::string&  contract_path);

    // Ends session profiling if verify_offload() never got the chance to.
    ~MergedBackend() override;

    BackendOutputs run(const float* prev_imn,
                       const float* curr_imn,
                       const float* curr_01) override;

    // Run one warm-up frame and assert the NPU actually executed nodes.
    // Only meaningful for the renesas provider; a no-op otherwise.
    // Throws std::runtime_error when the NPU did no work, or when fewer than
    // engine.require_npu_nodes nodes were placed on it.
    void verify_offload(const float* prev_imn,
                        const float* curr_imn,
                        const float* curr_01);

private:
    const Ort::Value* find_output(const std::string& name) const;
    void validate_contract() const;

    // Read the graph's own (non-rewritten) drive and speed outputs. Used by
    // plain-merged mode and by a contract that carries no head/speed rule --
    // without which those outputs would stay default-constructed on every
    // frame, which longitudinal fusion reads as a clear road.
    void read_plain_drive(BackendOutputs& out) const;
    void read_plain_speed(BackendOutputs& out) const;

    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo               mem_info_;

    std::vector<std::string> in_name_strs_;
    std::vector<const char*> in_names_;
    std::vector<std::string> out_name_strs_;
    std::vector<const char*> out_names_;
    std::unordered_map<std::string, size_t> out_index_;

    std::optional<MergedContract> contract_;
    std::vector<int64_t>          frame_shape_;
    std::string                   arena_shrink_;
    std::string                   provider_;
    int                           require_npu_nodes_ = 0;
    // True between session creation and the EndProfiling call. Only the
    // renesas provider enables profiling; see create_renesas_session().
    bool                          profiling_active_ = false;
    float                         conf_thres_ = 0.6f;
    float                         iou_thres_  = 0.45f;

    // Filled per run so find_output() can resolve by name. A pointer
    // find_output() returns is invalidated the next time run() reassigns
    // this.
    std::vector<Ort::Value> results_;
};

}  // namespace visionpilot::models
