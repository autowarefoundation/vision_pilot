#pragma once

#include <engine/onnx_engine.hpp>
#include <onnxruntime_cxx_api.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionpilot::models {

// ─── Output ───────────────────────────────────────────────────────────────────
// Bounding boxes in model-input pixel space (1024 × 512) after NMS.
// Coordinate mapping back to original image coordinates is the caller's job
// (reverse the preprocessing: divide by the resize scale, then add back the
// top-crop offset — there is no pad to subtract, since no letterbox is used).
struct Detection {
    float x1 = 0.f, y1 = 0.f;  // top-left
    float x2 = 0.f, y2 = 0.f;  // bottom-right
    float score    = 0.f;
    int   class_id = 0;
};

struct AutoSpeedOutput {
    std::vector<Detection> detections;
    bool valid = false;
};

// Decode a [1, channels, anchors] detection tensor into boxes.
// Memory layout is data[c * anchors + n]; channels = 4 + num_classes, with
// rows 0..3 being cx, cy, w, h in model-input pixels.
//
// cls_is_probability distinguishes the two producers: the standalone AutoSpeed
// graph emits class logits (false), while the merged rewrite emits
// already-sigmoided probabilities (true). Applying sigmoid twice would
// compress every score toward 0.5.
//
// Returns an invalid output (valid == false) when channels <= 4. A
// non-positive anchors yields a valid, empty detection list — there is
// nothing to decode, and this matches the pre-extraction behaviour.
AutoSpeedOutput decode_detections(const float* data,
                                  int64_t channels,
                                  int64_t anchors,
                                  bool    cls_is_probability,
                                  float   conf_thres,
                                  float   iou_thres);

// ─── Model ────────────────────────────────────────────────────────────────────
// YOLO-style object detection model.
//
// Preprocessing contract (caller, before infer()):
//   • Top-crop to 2:1, then resize to NET_W × NET_H (1024 × 512),
//     INTER_LINEAR. No letterbox and no padding — the pipeline shares this
//     buffer with AutoSteer (see ImagePreprocessor).
//   • Convert BGR → RGB
//   • Scale to [0, 1] — NO ImageNet normalisation
//   • Layout: CHW float32, CHW_SIZE elements
//
// Raw model output shape: [1, C, N]
//   C = 4 + num_classes   (cx, cy, w, h, logit_0 … logit_{K-1})
//   Post-processing (sigmoid + threshold + NMS) is done inside infer().
class AutoSpeed {
public:
    static constexpr int NET_H    = 512;
    static constexpr int NET_W    = 1024;
    static constexpr int CHW_SIZE = 3 * NET_H * NET_W;

    AutoSpeed(engine::OnnxEngine& engine, const std::string& model_path);

    // image_chw  : float32 CHW buffer, CHW_SIZE elements, RGB [0, 1]
    // conf_thres : sigmoid class probability threshold
    // iou_thres  : IoU threshold for NMS
    AutoSpeedOutput infer(const float* image_chw,
                          float conf_thres = 0.6f,
                          float iou_thres  = 0.45f);

private:
    AutoSpeedOutput post_process(const Ort::Value& tensor,
                                 float conf_thres,
                                 float iou_thres) const;

    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo               mem_info_;

    std::vector<std::string> in_name_strs_;
    std::vector<const char*> in_names_;
    std::vector<std::string> out_name_strs_;
    std::vector<const char*> out_names_;

    std::vector<int64_t> input_shape_;  // {1, 3, NET_H, NET_W}
    std::string arena_shrink_;
};

}  // namespace visionpilot::models
