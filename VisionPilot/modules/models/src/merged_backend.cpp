#include "models/merged_backend.hpp"

#include <onnxruntime_run_options_config_keys.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace visionpilot::models {

namespace {

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// Copy a fixed-size 64-element tensor into a fixed 64-float array. Rejects an
// undersized tensor (would read past its end) and an oversized one (would
// otherwise be silently truncated) alike.
void copy_64(const Ort::Value& v, std::array<float, 64>& dst,
             const std::string& name)
{
    const auto info = v.GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() != dst.size()) {
        throw std::runtime_error(
            "[MergedBackend] output '" + name + "' has " +
            std::to_string(info.GetElementCount()) + " elements, expected "
            "exactly " + std::to_string(dst.size()));
    }
    std::memcpy(dst.data(), v.GetTensorData<float>(),
                dst.size() * sizeof(float));
}

// Read element 0 of a tensor that must hold at least one element. The plain
// drive scalars are read this way because validate_output_shapes() -- which
// would have caught a zero-element tensor at startup -- runs only in
// contract mode; an unguarded [0] here would be an out-of-bounds read.
float read_scalar(const Ort::Value& v, const std::string& name)
{
    const auto info = v.GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() == 0) {
        throw std::runtime_error(
            "[MergedBackend] output '" + name + "' holds no elements, but a "
            "scalar is required");
    }
    return v.GetTensorData<float>()[0];
}

}  // namespace

void validate_contract_names(const MergedContract&           contract,
                             const std::vector<std::string>& output_names)
{
    // 1. Every name the contract mentions must exist in the session. This is
    //    also what catches a v6 contract paired with a v7 model (which does not
    //    output steer_lane_value) and the reverse (no steer_silu_41).
    const auto exists = [&](const std::string& name) {
        for (const auto& n : output_names) {
            if (n == name) return true;
        }
        return false;
    };

    std::vector<std::string> missing;
    const auto require = [&](const std::string& name) {
        if (!exists(name)) missing.push_back(name);
    };

    for (const auto& n : contract.passthrough) require(n);
    if (contract.head)     require(contract.head->output);
    if (contract.steer_xp) require(contract.steer_xp->logits);
    if (contract.speed) {
        for (const auto& l : contract.speed->levels) {
            require(l.box);
            require(l.cls);
        }
    }

    if (!missing.empty()) {
        std::string msg =
            "[MergedBackend] the contract names outputs this session does not "
            "expose.\n  Missing:";
        for (const auto& m : missing) msg += "\n    " + m;
        msg += "\n  Session outputs:";
        for (const auto& n : output_names) msg += "\n    " + n;
        throw std::runtime_error(msg);
    }

    // 2. Every passthrough entry must be one this backend actually knows how
    //    to route. run() dispatches by exact name; an entry that reached this
    //    far unrecognised would throw on every single frame instead of here.
    bool lane_passthrough   = false;
    bool height_passthrough = false;
    for (const auto& n : contract.passthrough) {
        if (n == "steer_lane_value") {
            lane_passthrough = true;
        } else if (n == "steer_height") {
            height_passthrough = true;
        } else {
            throw std::runtime_error(
                "[MergedBackend] the contract's passthrough list names '" + n +
                "', which this backend does not know how to route. Expected "
                "steer_lane_value or steer_height");
        }
    }

    // 3. steer_height is mandatory in both v6 and v7: it is the only source
    //    of AutoSteerOutput::h_vector. Without it, h_vector would stay
    //    all-zero while the ego path (xp) was filled correctly, and nothing
    //    downstream would notice.
    if (!height_passthrough) {
        throw std::runtime_error(
            "[MergedBackend] the contract's passthrough list is missing "
            "'steer_height'. AutoSteerOutput::h_vector would stay all-zero "
            "with no error.");
    }

    // 4. The ego path must be supplied exactly once: either as a v6
    //    steer_lane_value passthrough, or by the v7 steer_xp rule. With
    //    neither, xp would stay zero and the vehicle would steer on an empty
    //    path with no error anywhere.
    if (!lane_passthrough && !contract.steer_xp) {
        throw std::runtime_error(
            "[MergedBackend] the contract supplies no ego path: it has neither "
            "a 'steer_lane_value' passthrough (v6) nor a 'steer_xp' rule "
            "(v7). AutoSteer xp would stay zero and lateral fusion would "
            "steer on an empty path.");
    }
    if (lane_passthrough && contract.steer_xp) {
        throw std::runtime_error(
            "[MergedBackend] the contract supplies the ego path twice: both a "
            "'steer_lane_value' passthrough and a 'steer_xp' rule. Exactly "
            "one is expected.");
    }

    // 5. A session exposing the rewritten drive head can only produce
    //    AutoDrive's outputs through a head rule. Without one, run() leaves
    //    out.drive default-constructed on every frame with no message at all,
    //    and longitudinal fusion reads a permanently invalid drive output as
    //    "no CIPO confirmed" -- a clear road at D_MAX -- rather than as an
    //    error. Both v6 and v7 contracts carry a head rule, so only a
    //    truncated or mis-staged contract can reach this.
    if (!contract.head) {
        for (const auto& n : output_names) {
            if (is_drive_head_raw_output(n)) {
                throw std::runtime_error(
                    "[MergedBackend] the session exposes '" + n + "' but the "
                    "contract has no 'head' rule. AutoDriveOutput would stay "
                    "default-constructed (valid=false) on every frame with no "
                    "error, which longitudinal fusion reads as a clear road. "
                    "A rewritten (v6/v7) contract must supply 'head'.");
            }
        }
        // Not a rewritten head, so run() falls back to the plain drive
        // scalars. All three must be present: without them out.drive stays
        // default-constructed on every frame with no message at all, which
        // is the same silent "clear road" the branch above refuses.
        for (const char* n : {"drive_distance", "drive_curvature",
                              "drive_flag_logit"}) {
            if (!exists(n)) {
                throw std::runtime_error(
                    "[MergedBackend] the contract has no 'head' rule and the "
                    "session does not expose '" + std::string(n) +
                    "' either, so AutoDriveOutput would stay "
                    "default-constructed (valid=false) on every frame with "
                    "no error, which longitudinal fusion reads as a clear "
                    "road.");
            }
        }
    }

    // 6. Likewise for the per-level speed tail: without a speed rule the
    //    detections would stay empty on every frame, silently.
    if (!contract.speed) {
        for (const auto& n : output_names) {
            if (is_speed_level_box_output(n)) {
                throw std::runtime_error(
                    "[MergedBackend] the session exposes '" + n + "' but the "
                    "contract has no 'speed' rule. AutoSpeedOutput would stay "
                    "default-constructed (no detections) on every frame with "
                    "no error, which longitudinal fusion reads as a clear "
                    "road. A rewritten (v6/v7) contract must supply 'speed'.");
            }
        }
        // Likewise, run() falls back to the plain detection tensor.
        if (!exists("speed_output")) {
            throw std::runtime_error(
                "[MergedBackend] the contract has no 'speed' rule and the "
                "session does not expose 'speed_output' either, so "
                "AutoSpeedOutput would stay default-constructed (no "
                "detections) on every frame with no error, which "
                "longitudinal fusion reads as a clear road.");
        }
    }
}

void validate_output_shapes(
    const MergedContract&                                  contract,
    const std::unordered_map<std::string, DeclaredOutput>& declared)
{
    // Element count of a declared shape, treating dims[0] as a possibly-
    // symbolic batch axis (excluded) and requiring every other dimension to
    // be a positive, statically known size. A rank-0 or rank-1 shape has no
    // batch axis to exclude.
    const auto trailing_count =
        [](const std::vector<int64_t>& shape,
           const std::string& name) -> int64_t {
        const size_t skip = shape.size() >= 2 ? 1u : 0u;
        int64_t count = 1;
        for (size_t i = skip; i < shape.size(); ++i) {
            if (shape[i] <= 0) {
                throw std::runtime_error(
                    "[MergedBackend] output '" + name + "' has a dynamic or "
                    "unknown dimension in its declared shape; its geometry "
                    "cannot be verified against the contract at startup");
            }
            count *= shape[i];
        }
        return count;
    };

    const auto require_float = [&](const std::string& name) {
        // validate_contract_names() must run first to guarantee name is a
        // key here; not re-diagnosed.
        const auto& d = declared.at(name);
        if (d.dtype != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            throw std::runtime_error(
                "[MergedBackend] output '" + name + "' is not float32. Every "
                "postprocessing path in this backend reads outputs with "
                "GetTensorData<float>(), which throws on a mismatched "
                "element type.");
        }
    };

    // AutoSteerOutput::xp and h_vector are both fixed (1, 64).
    constexpr int64_t kFixedVectorSize = 64;

    for (const auto& name : contract.passthrough) {
        require_float(name);
        const int64_t count = trailing_count(declared.at(name).shape, name);
        if (count != kFixedVectorSize) {
            throw std::runtime_error(
                "[MergedBackend] passthrough output '" + name + "' has " +
                std::to_string(count) + " elements but exactly " +
                std::to_string(kFixedVectorSize) + " are required");
        }
    }

    if (contract.head) {
        const auto& name = contract.head->output;
        require_float(name);
        const int64_t count    = trailing_count(declared.at(name).shape, name);
        const int64_t expected = static_cast<int64_t>(contract.head->map.size());
        if (count != expected) {
            throw std::runtime_error(
                "[MergedBackend] head output '" + name + "' has " +
                std::to_string(count) + " elements but the contract's "
                "head.map describes " + std::to_string(expected) + " rows");
        }
    }

    if (contract.steer_xp) {
        const auto& name = contract.steer_xp->logits;
        require_float(name);
        const int64_t count = trailing_count(declared.at(name).shape, name);
        const int64_t expected =
            static_cast<int64_t>(contract.steer_xp->positions) * kFixedVectorSize;
        if (count != expected) {
            throw std::runtime_error(
                "[MergedBackend] steer_xp logits output '" + name + "' has " +
                std::to_string(count) + " elements but positions(" +
                std::to_string(contract.steer_xp->positions) + ")*rows(" +
                std::to_string(kFixedVectorSize) + ")=" +
                std::to_string(expected) + " is required");
        }
    }

    if (contract.speed) {
        // The class dimension of the first level, carried across the loop.
        // assemble_speed() also refuses levels that disagree, but only once a
        // frame has been run; caught here it is a startup refusal instead.
        int64_t     first_num_classes = 0;
        std::string first_cls_name;

        for (const auto& l : contract.speed->levels) {
            require_float(l.box);
            require_float(l.cls);

            const int64_t box_count =
                trailing_count(declared.at(l.box).shape, l.box);
            const int64_t expected_box =
                4LL * static_cast<int64_t>(l.h) * static_cast<int64_t>(l.w);
            if (box_count != expected_box) {
                throw std::runtime_error(
                    "[MergedBackend] speed box output '" + l.box + "' has " +
                    std::to_string(box_count) + " elements but the "
                    "contract's hw=[" + std::to_string(l.h) + "," +
                    std::to_string(l.w) + "] requires 4*h*w=" +
                    std::to_string(expected_box));
            }

            const auto& cls_shape = declared.at(l.cls).shape;
            if (cls_shape.size() < 2) {
                throw std::runtime_error(
                    "[MergedBackend] speed cls output '" + l.cls +
                    "' has rank " + std::to_string(cls_shape.size()) +
                    ", expected at least 2 to read a class dimension");
            }
            // trailing_count() throws first if any dimension from index 1
            // onward -- including shape[1] itself -- is not a positive,
            // statically known size, so shape[1] is safe to read afterward.
            const int64_t cls_count   = trailing_count(cls_shape, l.cls);
            const int64_t num_classes = cls_shape[1];
            const int64_t expected_cls =
                num_classes * static_cast<int64_t>(l.h) * static_cast<int64_t>(l.w);
            if (cls_count != expected_cls) {
                throw std::runtime_error(
                    "[MergedBackend] speed cls output '" + l.cls + "' has " +
                    std::to_string(cls_count) + " elements but num_classes(" +
                    std::to_string(num_classes) + ")*h*w=" +
                    std::to_string(expected_cls) + " is required by the "
                    "contract's hw=[" + std::to_string(l.h) + "," +
                    std::to_string(l.w) + "]");
            }

            if (first_cls_name.empty()) {
                first_num_classes = num_classes;
                first_cls_name    = l.cls;
            } else if (num_classes != first_num_classes) {
                throw std::runtime_error(
                    "[MergedBackend] speed cls output '" + l.cls +
                    "' declares " + std::to_string(num_classes) +
                    " classes but '" + first_cls_name + "' declares " +
                    std::to_string(first_num_classes) +
                    ". Every speed level must agree on the class count, "
                    "because they are concatenated into one [1,4+K,N] "
                    "detection buffer.");
            }
        }
    }
}

void validate_plain_merged_names(const std::vector<std::string>& output_names)
{
    const auto exists = [&](const std::string& name) {
        for (const auto& n : output_names) {
            if (n == name) return true;
        }
        return false;
    };

    // Every name run()'s plain-merged branch resolves with find_output().
    // find_output() returns nullptr for an absent name and the branch then
    // skips that output silently, so all six are required here: the two steer
    // outputs lateral fusion needs, and the four drive/speed outputs whose
    // absence would leave AutoDrive and AutoSpeed permanently invalid -- which
    // longitudinal fusion reads as a clear road, not as an error.
    //
    // The steer names are the merge tool's PREFIXED ORIGINAL ones. AutoSteer's
    // graph emits "lane_value" and "height" (the split path reads them
    // positionally into xp and h_vector, which is why their graph names never
    // mattered there), so a merged model carries "steer_lane_value" and
    // "steer_height" -- the same two names the contract branch routes through
    // its passthrough list. "steer_xp" is a contract RULE name; no model emits
    // it, so requiring it here meant plain-merged mode could never start.
    std::vector<std::string> missing;
    if (!exists("steer_lane_value")) missing.push_back("steer_lane_value");
    if (!exists("steer_height"))     missing.push_back("steer_height");
    if (!exists("drive_distance"))   missing.push_back("drive_distance");
    if (!exists("drive_curvature"))  missing.push_back("drive_curvature");
    if (!exists("drive_flag_logit")) missing.push_back("drive_flag_logit");
    if (!exists("speed_output"))     missing.push_back("speed_output");

    if (!missing.empty()) {
        std::string msg =
            "[MergedBackend] plain-merged model is missing output(s) this "
            "backend depends on.\n  Missing:";
        for (const auto& m : missing) msg += "\n    " + m;
        msg += "\n  Session outputs:";
        for (const auto& n : output_names) msg += "\n    " + n;
        throw std::runtime_error(msg);
    }
}

MergedBackend::MergedBackend(engine::OnnxEngine& engine,
                             const std::string&  model_path,
                             const std::string&  contract_path)
    : session_(engine.create_session(model_path, "merged_"))
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    , frame_shape_{1, 3, NET_H, NET_W}
    , arena_shrink_(engine.config().provider == "cpu" ||
                    engine.config().provider == "renesas"
                        ? "cpu:0" : "cpu:0;gpu:0")
    , provider_(engine.config().provider)
    , require_npu_nodes_(engine.config().require_npu_nodes)
{
    // create_renesas_session() enables profiling for the offload gate; no
    // other provider does.
    profiling_active_ = provider_ == "renesas";

    Ort::AllocatorWithDefaultOptions alloc;

    const size_t n_in = session_->GetInputCount();
    in_name_strs_.resize(n_in);
    in_names_.resize(n_in);
    for (size_t i = 0; i < n_in; ++i) {
        in_name_strs_[i] = session_->GetInputNameAllocated(i, alloc).get();
        in_names_[i]     = in_name_strs_[i].c_str();
        printf("[MergedBackend] input[%zu]  = %s\n", i, in_names_[i]);
    }

    const size_t n_out = session_->GetOutputCount();
    out_name_strs_.resize(n_out);
    out_names_.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
        out_names_[i]     = out_name_strs_[i].c_str();
        out_index_[out_name_strs_[i]] = i;
        printf("[MergedBackend] output[%zu] = %s\n", i, out_names_[i]);
    }

    if (!contract_path.empty()) {
        contract_ = MergedContract::from_file(contract_path);
        printf("[MergedBackend] contract=%s  attn_mode=%s  lane=%s\n",
               contract_path.c_str(),
               contract_->attn_mode.empty() ? "(unset)"
                                            : contract_->attn_mode.c_str(),
               contract_->steer_xp ? "host soft-argmax (v7 R8)"
                                   : "graph output (v6)");
        if (contract_->attn_mode.empty()) {
            // Unset is not the same claim as "degraded" -- it means this
            // contract never said which attention mode is live, so xp's
            // fidelity is simply unknown. Treating unset as safe-by-default
            // would defeat the point of a gate whose whole job is telling
            // the operator which mode is actually in effect.
            printf("[MergedBackend] WARNING: attn_mode is unset in the "
                   "contract. Whether the ego-path output (AutoSteer xp) is "
                   "degraded is unknown -- do not treat xp as "
                   "reference-quality until attn_mode is confirmed.\n");
        } else if (contract_->attn_mode != "keep") {
            printf("[MergedBackend] WARNING: attn_mode='%s' — the ego-path "
                   "output (AutoSteer xp) is degraded. openadkit measured lane "
                   "correlation 0.653-0.954 against the CPU reference for "
                   "'frozen'. Do not treat xp as reference-quality.\n",
                   contract_->attn_mode.c_str());
        }
    } else if (has_rewrite_signature_output(out_name_strs_)) {
        throw std::runtime_error(
            "[MergedBackend] this model exposes rewrite outputs "
            "(drive_head_raw / speed_l*_box) but no contract was found. "
            "Running it without the contract would silently produce an "
            "unscaled drive head, undecoded speed boxes, and no ego path. "
            "Place contract.json in the artifacts directory, or set "
            "model.contract.");
    } else {
        printf("[MergedBackend] no contract — plain-merged mode\n");
    }

    // Three inputs are mandatory in both modes, and no other input is
    // understood: run() dispatches on exact name and would otherwise refuse
    // every single frame instead of failing once, here.
    for (const char* required : {"input", "drive_image_prev", "drive_image_curr"}) {
        bool found = false;
        for (const auto& n : in_name_strs_) if (n == required) found = true;
        if (!found) {
            throw std::runtime_error(
                std::string("[MergedBackend] merged model is missing required "
                            "input '") + required + "'");
        }
    }
    for (const auto& n : in_name_strs_) {
        if (n != "input" && n != "drive_image_prev" && n != "drive_image_curr") {
            throw std::runtime_error(
                "[MergedBackend] merged model exposes unexpected input '" + n +
                "'. Expected exactly input, drive_image_prev, "
                "drive_image_curr");
        }
    }

    validate_contract();

    printf("[MergedBackend] Ready — %zu inputs, %zu outputs\n", n_in, n_out);
}

MergedBackend::~MergedBackend()
{
    // Backstop for the profiling create_renesas_session() turns on.
    // verify_offload() is the only place that ends it, and it runs from
    // InferencePipeline::process() -- never reached if the source dies
    // before a second frame arrives, or if this backend is driven directly.
    // Left enabled, ORT records an event per op for the rest of the
    // process's life and grows the profile file without bound.
    if (!profiling_active_) return;
    try {
        Ort::AllocatorWithDefaultOptions alloc;
        session_->EndProfilingAllocated(alloc);
    } catch (...) {
        // A destructor must not throw, and there is nothing left to report
        // the failure to.
    }
}

const Ort::Value* MergedBackend::find_output(const std::string& name) const
{
    const auto it = out_index_.find(name);
    if (it == out_index_.end()) return nullptr;
    if (it->second >= results_.size()) return nullptr;
    return &results_[it->second];
}

void MergedBackend::validate_contract() const
{
    if (!contract_) {
        validate_plain_merged_names(out_name_strs_);
        return;
    }

    validate_contract_names(*contract_, out_name_strs_);

    // Read every output's declared shape/dtype once, then hand the whole map
    // to the pure geometry check. validate_contract_names() above already
    // guaranteed every name the contract mentions is a key here.
    std::unordered_map<std::string, DeclaredOutput> declared;
    declared.reserve(out_name_strs_.size());
    for (size_t i = 0; i < out_name_strs_.size(); ++i) {
        // GetOutputTypeInfo() returns Ort::TypeInfo by value -- an owning
        // handle. GetTensorTypeAndShapeInfo() returns a non-owning view into
        // it, valid only until the TypeInfo is freed. Bind the owning handle
        // to a named local so it outlives the GetShape()/GetElementType()
        // reads below; chaining the two calls in one expression would
        // destroy the temporary at the semicolon and read freed memory.
        const Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);
        const auto           info     = type_info.GetTensorTypeAndShapeInfo();
        declared[out_name_strs_[i]] = DeclaredOutput{info.GetShape(),
                                                      info.GetElementType()};
    }
    validate_output_shapes(*contract_, declared);
}

void MergedBackend::read_plain_drive(BackendOutputs& out) const
{
    const Ort::Value* d0 = find_output("drive_distance");
    const Ort::Value* d1 = find_output("drive_curvature");
    const Ort::Value* d2 = find_output("drive_flag_logit");
    if (d0 == nullptr || d1 == nullptr || d2 == nullptr) return;

    out.drive.dist_normalized = read_scalar(*d0, "drive_distance");
    out.drive.curvature_raw   = read_scalar(*d1, "drive_curvature");
    out.drive.flag_prob =
        1.f / (1.f + std::exp(-read_scalar(*d2, "drive_flag_logit")));
    out.drive.valid = true;
}

void MergedBackend::read_plain_speed(BackendOutputs& out) const
{
    const Ort::Value* v = find_output("speed_output");
    if (v == nullptr) return;

    const auto shape = v->GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() < 3) {
        // Throwing rather than skipping: silently leaving AutoSpeedOutput
        // empty is read downstream as a clear road, which is precisely the
        // failure every other guard in this file exists to prevent.
        throw std::runtime_error(
            "[MergedBackend] output 'speed_output' has rank " +
            std::to_string(shape.size()) + ", expected at least 3 ([1,C,N])");
    }
    // The standalone graph emits class logits.
    out.speed = decode_detections(v->GetTensorData<float>(),
                                  shape[1], shape[2],
                                  /*cls_is_probability=*/false,
                                  conf_thres_, iou_thres_);
}

BackendOutputs MergedBackend::run(const float* prev_imn,
                                  const float* curr_imn,
                                  const float* curr_01)
{
    BackendOutputs out;

    // Feed in the session's declared input order.
    std::vector<Ort::Value> inputs;
    inputs.reserve(in_name_strs_.size());
    for (const auto& name : in_name_strs_) {
        const float* src = nullptr;
        if (name == "input")                 src = curr_01;
        else if (name == "drive_image_prev") src = prev_imn;
        else if (name == "drive_image_curr") src = curr_imn;
        else {
            // Unreachable: the constructor refuses any input outside this
            // set. Kept as a defensive fallback rather than an assert.
            printf("[MergedBackend] Unexpected input '%s'\n", name.c_str());
            return out;
        }
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem_info_, const_cast<float*>(src), CHW_SIZE,
            frame_shape_.data(), frame_shape_.size()));
    }

    const auto t = Clock::now();
    try {
        Ort::RunOptions run_options;
        run_options.AddConfigEntry(
            kOrtRunOptionsConfigEnableMemoryArenaShrinkage,
            arena_shrink_.c_str());
        results_ = session_->Run(run_options,
                                 in_names_.data(), inputs.data(), inputs.size(),
                                 out_names_.data(), out_names_.size());
    } catch (const Ort::Exception& e) {
        printf("[MergedBackend] Inference error: %s\n", e.what());
        return out;
    }
    out.ad_ms = Ms(Clock::now() - t).count();

    try {
        if (contract_) {
            for (const auto& name : contract_->passthrough) {
                const Ort::Value* v = find_output(name);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract passthrough names '" + name +
                        "' but the session has no such output");
                }
                if (name == "steer_lane_value")  copy_64(*v, out.steer.xp, name);
                else if (name == "steer_height") copy_64(*v, out.steer.h_vector, name);
                else {
                    // Unreachable: validate_contract_names() refuses any
                    // other passthrough entry at construction.
                    throw std::runtime_error(
                        "[MergedBackend] unknown passthrough output '" + name +
                        "'. Expected steer_lane_value or steer_height");
                }
            }

            // v7: recover the ego path from pre-softmax logits on the host.
            if (contract_->steer_xp) {
                const Ort::Value* v = find_output(contract_->steer_xp->logits);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract steer_xp names '" +
                        contract_->steer_xp->logits +
                        "' but the session has no such output");
                }
                apply_steer_xp(*contract_->steer_xp, v->GetTensorData<float>(),
                               v->GetTensorTypeAndShapeInfo().GetElementCount(),
                               out.steer);
            } else {
                // v6: xp arrived through passthrough, validated at startup.
                out.steer.valid = true;
            }

            if (contract_->head) {
                const Ort::Value* v = find_output(contract_->head->output);
                if (v == nullptr) {
                    throw std::runtime_error(
                        "[MergedBackend] contract head names '" +
                        contract_->head->output +
                        "' but the session has no such output");
                }
                apply_head(*contract_->head, v->GetTensorData<float>(),
                           v->GetTensorTypeAndShapeInfo().GetElementCount(),
                           out.drive);
            } else {
                // No head rule means this is not a rewritten head, so the
                // plain drive scalars carry AutoDrive. Reading them here is
                // what keeps a rule-less contract from leaving out.drive
                // permanently invalid; validate_contract_names() guaranteed
                // all three outputs exist.
                read_plain_drive(out);
            }

            if (contract_->speed) {
                std::vector<SpeedLevelTensors> levels;
                levels.reserve(contract_->speed->levels.size());
                for (const auto& l : contract_->speed->levels) {
                    const Ort::Value* b = find_output(l.box);
                    const Ort::Value* c = find_output(l.cls);
                    if (b == nullptr || c == nullptr) {
                        throw std::runtime_error(
                            "[MergedBackend] contract speed level names '" +
                            l.box + "' / '" + l.cls +
                            "' but the session lacks one of them");
                    }
                    const auto cls_shape =
                        c->GetTensorTypeAndShapeInfo().GetShape();
                    if (cls_shape.size() < 2) {
                        throw std::runtime_error(
                            "[MergedBackend] speed cls output '" + l.cls +
                            "' has rank < 2");
                    }
                    SpeedLevelTensors t_l;
                    t_l.box         = b->GetTensorData<float>();
                    t_l.cls         = c->GetTensorData<float>();
                    t_l.h           = l.h;
                    t_l.w           = l.w;
                    t_l.stride      = l.stride;
                    t_l.num_classes = static_cast<int>(cls_shape[1]);
                    t_l.box_count   = b->GetTensorTypeAndShapeInfo().GetElementCount();
                    t_l.cls_count   = c->GetTensorTypeAndShapeInfo().GetElementCount();
                    levels.push_back(t_l);
                }
                const auto a = assemble_speed(levels);
                // R7 emits cls already sigmoided.
                out.speed = decode_detections(a.data.data(), a.channels,
                                              a.anchors,
                                              /*cls_is_probability=*/true,
                                              conf_thres_, iou_thres_);
            } else {
                // No speed rule means the graph's own detection tail is
                // intact, so read it the plain way rather than leaving
                // AutoSpeedOutput permanently empty.
                read_plain_speed(out);
            }
        } else {
            // Plain merged: prefixed copies of the original outputs.
            // out.steer.valid only becomes true when BOTH xp and h_vector
            // were actually filled -- validate_plain_merged_names() requires
            // both outputs to exist, but that is a startup guarantee about
            // names, not proof that copy_64() will not itself throw, so
            // valid still tracks what really happened this frame.
            bool xp_ok = false;
            bool h_vector_ok = false;
            if (const Ort::Value* v = find_output("steer_lane_value")) {
                copy_64(*v, out.steer.xp, "steer_lane_value");
                xp_ok = true;
            }
            if (const Ort::Value* v = find_output("steer_height")) {
                copy_64(*v, out.steer.h_vector, "steer_height");
                h_vector_ok = true;
            }
            out.steer.valid = xp_ok && h_vector_ok;

            read_plain_speed(out);
            read_plain_drive(out);
        }
    } catch (const std::runtime_error& e) {
        // Structural contract errors are caught at startup by
        // validate_contract(); a shape surprise can still only surface on the
        // first real run. Report and mark the frame invalid rather than
        // aborting mid-drive.
        printf("[MergedBackend] Postprocessing error: %s\n", e.what());
        return BackendOutputs{};
    } catch (const Ort::Exception& e) {
        // Ort::Exception derives from std::exception, not
        // std::runtime_error (e.g. GetTensorData<float>() on a
        // mismatched-dtype tensor), so it needs its own clause to hit the
        // same per-frame degrade-not-abort behaviour as the catch above.
        printf("[MergedBackend] Postprocessing error: %s\n", e.what());
        return BackendOutputs{};
    }

    return out;
}

int check_offload(const std::map<std::string, int>& hist, int required)
{
    // A configured 0 -- engine::Config::require_npu_nodes's default -- means
    // "require at least one," never "require none." A configured negative
    // value (never asked for by the brief, but not guarded against there
    // either) is treated the same as 0 rather than as a floor of zero.
    const int floor = required > 0 ? required : 1;

    const auto it = hist.find("RenesasExecutionProvider");
    const int npu_nodes = it == hist.end() ? 0 : it->second;

    if (npu_nodes < floor) {
        throw std::runtime_error(
            "[MergedBackend] NPU offload gate FAILED: " +
            std::to_string(npu_nodes) + " nodes on RenesasExecutionProvider, "
            "required at least " + std::to_string(floor) +
            ". The CPU execution provider is a silent fallback, so a run "
            "that completes is not proof of offload. Check that the "
            "artifacts match this model and that their recorded "
            "compile-host paths are mounted.");
    }
    return npu_nodes;
}

void MergedBackend::verify_offload(const float* prev_imn,
                                   const float* curr_imn,
                                   const float* curr_01)
{
    if (provider_ != "renesas") return;

    printf("[MergedBackend] Offload gate: running one warm-up frame\n");
    const auto probe = run(prev_imn, curr_imn, curr_01);

    // End profiling right after the probe, win or lose: leaving it enabled
    // would accumulate a profiling event per op for the rest of the
    // process's life, and the returned path is needed either way to name
    // the profile in the error below.
    Ort::AllocatorWithDefaultOptions alloc;
    const auto profile = session_->EndProfilingAllocated(alloc);
    profiling_active_ = false;
    const std::string profile_path = profile.get();

    if (probe.ad_ms <= 0.0) {
        // run() catches its own errors, prints them, and returns a
        // default-constructed BackendOutputs: ad_ms is set only after
        // session_->Run() itself succeeds, and a postprocessing failure
        // discards it again by returning BackendOutputs{}. Without this
        // check, a probe that did not complete would fall straight into an
        // empty-histogram "the NPU did no work" verdict, misreporting an
        // inference or postprocessing failure as an offload failure.
        throw std::runtime_error(
            "[MergedBackend] NPU offload gate FAILED: the probe run did not "
            "complete (see the inference or postprocessing error printed "
            "above), so no profile can be trusted to prove or disprove NPU "
            "offload. "
            "Profile: " + profile_path);
    }

    const auto hist = engine::parse_profile_providers(profile_path);

    printf("[MergedBackend] Node placement:\n");
    for (const auto& [prov, n] : hist) {
        printf("[MergedBackend]   %-32s %d\n", prov.c_str(), n);
    }

    int npu_nodes = 0;
    try {
        npu_nodes = check_offload(hist, require_npu_nodes_);
    } catch (const std::runtime_error& e) {
        // check_offload() is a pure function of (hist, required) so its own
        // message cannot name the file that hist came from; re-attach it
        // here so the FAILED message still points at the profile to inspect,
        // matching the probe-failure branch above.
        throw std::runtime_error(std::string(e.what()) +
                                  " Profile: " + profile_path);
    }

    const int required = require_npu_nodes_ > 0 ? require_npu_nodes_ : 1;
    printf("[MergedBackend] Offload gate PASSED — %d nodes on the NPU "
           "(required at least %d)\n", npu_nodes, required);
}

}  // namespace visionpilot::models
