#ifndef VISIONPILOT_AUTO_DRIVE_HPP
#define VISIONPILOT_AUTO_DRIVE_HPP

#include <engine/onnx_engine.hpp>
#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace visionpilot::models
{
    // ─── Output ───────────────────────────────────────────────────────────────────
    // Raw scalars straight from the model. Domain conversion is the caller's job:
    //   distance_m    = D_MAX_M * (1.0f - dist_normalized)   // D_MAX_M = 150.0
    //   curvature_1pm = curvature_raw * CURV_SCALE
    struct AutoDriveOutput
    {
        float dist_normalized = 0.f; // normalised distance  [0, 1]
        float curvature_raw = 0.f; // raw curvature output
        float flag_prob = 0.f; // sigmoid(flag_logit), CIPO probability [0, 1]
        bool valid = false;
    };

    // ─── Model ────────────────────────────────────────────────────────────────────
    // Two-frame unified model.
    //
    // Preprocessing contract (caller, before infer()):
    //   • Resize both frames to NET_W × NET_H  (1024 × 512)
    //   • Convert BGR → RGB
    //   • Apply ImageNet normalisation:
    //       mean = [0.485, 0.456, 0.406]  std = [0.229, 0.224, 0.225]
    //   • Layout: CHW float32, CHW_SIZE elements each frame
    class AutoDrive
    {
    public:
        static constexpr int NET_H = 512;
        static constexpr int NET_W = 1024;
        static constexpr int CHW_SIZE = 3 * NET_H * NET_W;

        // engine  — shared OnnxEngine, must outlive this object
        // model_path — path to the AutoDrive .onnx file
        AutoDrive(engine::OnnxEngine& engine, const std::string& model_path);

        // prev_chw, curr_chw : float32 CHW buffers, CHW_SIZE elements each
        AutoDriveOutput infer(float* prev_chw, float* curr_chw);

    private:
        std::unique_ptr<Ort::Session> session_;
        Ort::MemoryInfo mem_info_;

        std::vector<std::string> in_name_strs_;
        std::vector<const char*> in_names_;
        std::vector<std::string> out_name_strs_;
        std::vector<const char*> out_names_;

        std::vector<int64_t> frame_shape_; // {1, 3, NET_H, NET_W}
        std::string arena_shrink_; // arena devices to shrink per run
    };
} // namespace visionpilot::models

#endif // VISIONPILOT_AUTO_DRIVE_HPP
