#include "models/auto_speed.hpp"

#include <onnxruntime_run_options_config_keys.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace visionpilot::models {

// ─── Constructor ─────────────────────────────────────────────────────────────

AutoSpeed::AutoSpeed(engine::OnnxEngine& engine, const std::string& model_path)
    : session_(engine.create_session(model_path, "autospeed_"))
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault))
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
        printf("[AutoSpeed] input[%zu]  = %s\n", i, in_names_[i]);
    }

    out_name_strs_.resize(n_out);
    out_names_.resize(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
        out_names_[i]     = out_name_strs_[i].c_str();
        printf("[AutoSpeed] output[%zu] = %s\n", i, out_names_[i]);
    }

    printf("[AutoSpeed] Ready — %zu inputs, %zu outputs | "
           "frame [1, 3, %d, %d]\n", n_in, n_out, NET_H, NET_W);
}

// ─── Inference ───────────────────────────────────────────────────────────────

AutoSpeedOutput AutoSpeed::infer(
    const float* image_chw, float conf_thres, float iou_thres)
{
    AutoSpeedOutput out;

    auto input_tensor = Ort::Value::CreateTensor<float>(
        mem_info_,
        const_cast<float*>(image_chw), CHW_SIZE,
        input_shape_.data(), input_shape_.size());

    std::vector<Ort::Value> results;
    try {
        Ort::RunOptions run_options;
        run_options.AddConfigEntry(kOrtRunOptionsConfigEnableMemoryArenaShrinkage, "gpu:0");
        results = session_->Run(
            run_options,
            in_names_.data(),  &input_tensor, 1,
            out_names_.data(), out_names_.size());
    } catch (const Ort::Exception& e) {
        printf("[AutoSpeed] Inference error: %s\n", e.what());
        return out;
    }

    if (results.empty()) {
        printf("[AutoSpeed] No output tensors returned\n");
        return out;
    }

    return post_process(results[0], conf_thres, iou_thres);
}

// ─── Post-processing ─────────────────────────────────────────────────────────

AutoSpeedOutput AutoSpeed::post_process(
    const Ort::Value& tensor, float conf_thres, float iou_thres) const
{
    AutoSpeedOutput out;
    const auto shape = tensor.GetTensorTypeAndShapeInfo().GetShape();

    // Expected layout: [1, C, N]  where C = 4 + num_classes
    if (shape.size() < 3) {
        printf("[AutoSpeed] Unexpected output rank: %zu\n", shape.size());
        return out;
    }

    const int64_t C           = shape[1];
    const int64_t N           = shape[2];
    const int     num_classes = static_cast<int>(C) - 4;

    if (num_classes <= 0) {
        printf("[AutoSpeed] Invalid channel count C=%lld\n",
               static_cast<long long>(C));
        return out;
    }

    // data[c * N + n] gives channel c for anchor n
    const float* data = tensor.GetTensorData<float>();

    std::vector<Detection> candidates;
    candidates.reserve(256);

    for (int64_t n = 0; n < N; ++n) {
        const float cx = data[0 * N + n];
        const float cy = data[1 * N + n];
        const float w  = data[2 * N + n];
        const float h  = data[3 * N + n];

        float best_prob = -1.f;
        int   best_cls  =  0;
        for (int c = 0; c < num_classes; ++c) {
            const float prob = 1.f / (1.f + std::exp(-data[(4 + c) * N + n]));
            if (prob > best_prob) { best_prob = prob; best_cls = c; }
        }

        if (best_prob < conf_thres) continue;

        Detection d;
        d.x1       = cx - w * 0.5f;
        d.y1       = cy - h * 0.5f;
        d.x2       = cx + w * 0.5f;
        d.y2       = cy + h * 0.5f;
        d.score    = best_prob;
        d.class_id = best_cls;
        candidates.push_back(d);
    }

    out.detections = nms(std::move(candidates), iou_thres);
    out.valid      = true;
    return out;
}

// ─── NMS helpers ─────────────────────────────────────────────────────────────

float AutoSpeed::iou(const Detection& a, const Detection& b)
{
    const float ix1   = std::max(a.x1, b.x1);
    const float iy1   = std::max(a.y1, b.y1);
    const float ix2   = std::min(a.x2, b.x2);
    const float iy2   = std::min(a.y2, b.y2);
    const float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

std::vector<Detection> AutoSpeed::nms(
    std::vector<Detection> dets, float iou_thres)
{
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) {
                  return a.score > b.score;
              });

    std::vector<bool>      suppressed(dets.size(), false);
    std::vector<Detection> keep;
    keep.reserve(dets.size());

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!suppressed[j] && iou(dets[i], dets[j]) > iou_thres)
                suppressed[j] = true;
        }
    }
    return keep;
}

}  // namespace visionpilot::models
