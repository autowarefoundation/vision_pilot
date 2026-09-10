#include <gtest/gtest.h>

#include <engine/onnx_engine.hpp>
#include <models/merged_backend.hpp>
#include <models/split_backend.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <vector>

namespace fs = std::filesystem;
namespace vm = visionpilot::models;
namespace ve = visionpilot::engine;

namespace {

constexpr int kChw = vm::MergedBackend::CHW_SIZE;

// Deterministic pseudo-image so both backends see identical input.
std::vector<float> fake_chw(unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> d(0.f, 1.f);
    std::vector<float> v(kChw);
    for (auto& x : v) x = d(rng);
    return v;
}

// The merged ONNX path comes from the environment so CI can place it anywhere.
std::string merged_model_path()
{
    if (const char* p = std::getenv("VP_MERGED_MODEL")) return p;
    return "modules/models/weights/vision_pilot_merged_fp32.onnx";
}

}  // namespace

TEST(MergedEquivalence, PlainMergedMatchesSplitOnCpu)
{
    const std::string merged = merged_model_path();
    if (!fs::exists(merged)) {
        GTEST_SKIP() << "merged model not found at " << merged
                     << " — set VP_MERGED_MODEL to enable this test";
    }

    ve::Config ecfg;
    ecfg.provider = "cpu";
    ve::OnnxEngine engine(ecfg);

    vm::SplitBackend  split(engine, "fp32");
    // Empty contract path: plain-merged mode.
    vm::MergedBackend merged_backend(engine, merged, "");

    const auto prev = fake_chw(1);
    const auto curr = fake_chw(2);
    const auto res  = fake_chw(3);

    const auto a = split.run(prev.data(), curr.data(), res.data());
    const auto b = merged_backend.run(prev.data(), curr.data(), res.data());

    ASSERT_TRUE(a.drive.valid);
    ASSERT_TRUE(b.drive.valid);
    EXPECT_NEAR(a.drive.dist_normalized, b.drive.dist_normalized, 1e-4);
    EXPECT_NEAR(a.drive.curvature_raw,   b.drive.curvature_raw,   1e-4);
    EXPECT_NEAR(a.drive.flag_prob,       b.drive.flag_prob,       1e-4);

    ASSERT_TRUE(a.steer.valid);
    ASSERT_TRUE(b.steer.valid);
    for (size_t i = 0; i < a.steer.xp.size(); ++i) {
        EXPECT_NEAR(a.steer.xp[i], b.steer.xp[i], 1e-4)
            << "xp mismatch at index " << i;
        EXPECT_NEAR(a.steer.h_vector[i], b.steer.h_vector[i], 1e-4)
            << "h_vector mismatch at index " << i;
    }

    ASSERT_TRUE(a.speed.valid);
    ASSERT_TRUE(b.speed.valid);
    ASSERT_EQ(a.speed.detections.size(), b.speed.detections.size());
    for (size_t i = 0; i < a.speed.detections.size(); ++i) {
        const auto& x = a.speed.detections[i];
        const auto& y = b.speed.detections[i];
        EXPECT_EQ(x.class_id, y.class_id);
        EXPECT_NEAR(x.score, y.score, 1e-4);
        EXPECT_NEAR(x.x1, y.x1, 1e-3);
        EXPECT_NEAR(x.y1, y.y1, 1e-3);
        EXPECT_NEAR(x.x2, y.x2, 1e-3);
        EXPECT_NEAR(x.y2, y.y2, 1e-3);
    }
}
