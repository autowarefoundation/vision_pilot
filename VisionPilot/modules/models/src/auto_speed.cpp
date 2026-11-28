#include <models/auto_speed.hpp>
#include <logging/logger.hpp>
#include <onnxruntime_run_options_config_keys.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace visionpilot::models
{
    // ─── Constructor ─────────────────────────────────────────────────────────────

    AutoSpeed::AutoSpeed(engine::OnnxEngine& engine, const std::string& model_path)
        : session_(engine.create_session(model_path, "autospeed_"))
          , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
          , input_shape_{1, 3, NET_H, NET_W}
          , arena_shrink_(engine.config().provider == "cpu" ? "cpu:0" : "cpu:0;gpu:0")
    {
        Ort::AllocatorWithDefaultOptions alloc;
        const size_t n_in = session_->GetInputCount();
        const size_t n_out = session_->GetOutputCount();

        in_name_strs_.resize(n_in);
        in_names_.resize(n_in);
        for (size_t i = 0; i < n_in; ++i)
        {
            in_name_strs_[i] = session_->GetInputNameAllocated(i, alloc).get();
            in_names_[i] = in_name_strs_[i].c_str();
            logging::info("[AutoSpeed] input[{}]  = {}", i, in_names_[i]);
        }

        out_name_strs_.resize(n_out);
        out_names_.resize(n_out);
        for (size_t i = 0; i < n_out; ++i)
        {
            out_name_strs_[i] = session_->GetOutputNameAllocated(i, alloc).get();
            out_names_[i] = out_name_strs_[i].c_str();
            logging::info("[AutoSpeed] output[{}] = {}", i, out_names_[i]);
        }

        logging::info("[AutoSpeed] Ready — {} inputs, {} outputs | "
               "frame [1, 3, {}, {}]\n", n_in, n_out, NET_H, NET_W);
    }

    // ─── Inference ───────────────────────────────────────────────────────────────

    AutoSpeedOutput AutoSpeed::infer(
        float* image_chw, float conf_thres, float iou_thres)
    {
        AutoSpeedOutput out;

        auto input_tensor = Ort::Value::CreateTensor<float>(
            mem_info_,
            image_chw, CHW_SIZE,
            input_shape_.data(), input_shape_.size());

        std::vector<Ort::Value> results;
        try
        {
            Ort::RunOptions run_options;
            static_cast<void>(run_options.AddConfigEntry(kOrtRunOptionsConfigEnableMemoryArenaShrinkage,
                                                         arena_shrink_.c_str()));
            results = session_->Run(
                run_options,
                in_names_.data(), &input_tensor, 1,
                out_names_.data(), out_names_.size());
        }
        catch (const Ort::Exception& e)
        {
            logging::info("[AutoSpeed] Inference error: {}", e.what());
            return out;
        }

        if (results.empty())
        {
            logging::info("[AutoSpeed] No output tensors returned");
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
        if (shape.size() < 3)
        {
            logging::info("[AutoSpeed] Unexpected output rank: {}", shape.size());
            return out;
        }

        const int64_t C = shape[1];
        const int64_t N = shape[2];

        if (C <= 4)
        {
            logging::info("[AutoSpeed] Invalid channel count C={}", static_cast<long long>(C));
            return out;
        }
        // MISRA Rule 4.1.3: C > 4 (guard above), so C - 4 is in [1, INT64_MAX - 4] (deviation below).
        // codeql::misra_deviation_next_line(misra-4-1-3-autospeed-class-count)
        const int num_classes = static_cast<int>(C - 4);

        // data[c * N + n] gives channel c for anchor n
        const float* data = tensor.GetTensorData<float>();

        std::vector<Detection> candidates;
        candidates.reserve(256);

        // MISRA Rule 4.1.3: tensor [1, C, N]; every index k * N + n <= C * N - 1, the tensor size (deviation below).
        for (int64_t n = 0; n < N; ++n)
        {
            // codeql::misra_deviation_begin(misra-4-1-3-autospeed-tensor-index)
            const float cx = data[0 * N + n];
            const float cy = data[1 * N + n];
            const float w = data[2 * N + n];
            const float h = data[3 * N + n];
            // codeql::misra_deviation_end(misra-4-1-3-autospeed-tensor-index)

            float best_prob = -1.f;
            int best_cls = 0;
            for (int c = 0; c < num_classes; ++c)
            {
                // MISRA Rule 4.1.3: index (4 + c) * N + n <= C * N - 1, the tensor size (deviation below).
                // MISRA Dir 0.3.1: exp() -> +inf gives 1/inf = 0, the correct sigmoid limit.
                // codeql::misra_deviation_next_line(misra-4-1-3-autospeed-tensor-index)
                const float prob = 1.f / (1.f + std::exp(-data[(4 + c) * N + n]));
                if (prob > best_prob)
                {
                    best_prob = prob;
                    best_cls = c;
                }
            }

            if (best_prob < conf_thres) continue;

            Detection d;
            d.x1 = cx - w * 0.5f;
            d.y1 = cy - h * 0.5f;
            d.x2 = cx + w * 0.5f;
            d.y2 = cy + h * 0.5f;
            d.score = best_prob;
            d.class_id = best_cls;
            candidates.push_back(d);
        }

        out.detections = nms(std::move(candidates), iou_thres);
        out.valid = true;
        return out;
    }

    // ─── NMS helpers ─────────────────────────────────────────────────────────────

    float AutoSpeed::iou(const Detection& a, const Detection& b)
    {
        const float ix1 = std::max(a.x1, b.x1);
        const float iy1 = std::max(a.y1, b.y1);
        const float ix2 = std::min(a.x2, b.x2);
        const float iy2 = std::min(a.y2, b.y2);
        const float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
        const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
        const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
        return inter / (area_a + area_b - inter + 1e-6f);
    }

    std::vector<Detection> AutoSpeed::nms(
        std::vector<Detection> dets, float iou_thres)
    {
        // MISRA 15.1.3: [&] instead of [] avoids the implicit function-pointer conversion; nothing is captured.
        std::sort(dets.begin(), dets.end(),
                  [&](const Detection& a, const Detection& b)
                  {
                      return a.score > b.score;
                  });
        //  C++ standard guarantees initialization to false
        std::vector<bool> suppressed(dets.size());
        std::vector<Detection> keep;
        keep.reserve(dets.size());

        for (size_t i = 0; i < dets.size(); ++i)
        {
            if (std::as_const(suppressed)[i]) continue;
            keep.push_back(dets[i]);
            for (size_t j = i + 1; j < dets.size(); ++j)
            {
                if (!std::as_const(suppressed)[j] && iou(dets[i], dets[j]) > iou_thres)
                    suppressed[j] = true;
            }
        }
        return keep;
    }
} // namespace visionpilot::models
