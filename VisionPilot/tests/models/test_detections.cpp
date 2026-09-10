#include <gtest/gtest.h>
#include <models/auto_speed.hpp>

#include <cmath>
#include <vector>

using visionpilot::models::decode_detections;

namespace {

// One anchor, one class. Layout is data[c * N + n] with N = 1.
std::vector<float> single_anchor(float cx, float cy, float w, float h,
                                 float cls_value)
{
    return {cx, cy, w, h, cls_value};
}

}  // namespace

TEST(DecodeDetections, AppliesSigmoidWhenClsIsLogit)
{
    // logit 2.0 -> 0.8808, above a 0.6 threshold.
    const auto d = single_anchor(100.f, 50.f, 20.f, 10.f, 2.0f);
    const auto out = decode_detections(d.data(), 5, 1,
                                       /*cls_is_probability=*/false,
                                       0.6f, 0.45f);
    ASSERT_TRUE(out.valid);
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_NEAR(out.detections[0].score,
                1.f / (1.f + std::exp(-2.0f)), 1e-6f);
    EXPECT_FLOAT_EQ(out.detections[0].x1, 100.f - 10.f);
    EXPECT_FLOAT_EQ(out.detections[0].y1, 50.f - 5.f);
    EXPECT_FLOAT_EQ(out.detections[0].x2, 100.f + 10.f);
    EXPECT_FLOAT_EQ(out.detections[0].y2, 50.f + 5.f);
    EXPECT_EQ(out.detections[0].class_id, 0);
}

TEST(DecodeDetections, UsesProbabilityDirectlyWhenAlreadySigmoided)
{
    // 0.9 is already a probability; a second sigmoid would give 0.711.
    const auto d = single_anchor(100.f, 50.f, 20.f, 10.f, 0.9f);
    const auto out = decode_detections(d.data(), 5, 1,
                                       /*cls_is_probability=*/true,
                                       0.6f, 0.45f);
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.9f);
}

TEST(DecodeDetections, DoubleSigmoidWouldFallBelowThreshold)
{
    // The regression this flag exists to prevent: 0.9 passed as a logit
    // becomes 0.711, and with a 0.8 threshold the detection disappears.
    const auto d = single_anchor(100.f, 50.f, 20.f, 10.f, 0.9f);
    EXPECT_EQ(decode_detections(d.data(), 5, 1, false, 0.8f, 0.45f)
                  .detections.size(), 0u);
    EXPECT_EQ(decode_detections(d.data(), 5, 1, true, 0.8f, 0.45f)
                  .detections.size(), 1u);
}

TEST(DecodeDetections, PicksHighestScoringClass)
{
    // N = 1, K = 3. data[c * N + n].
    const std::vector<float> d = {10.f, 10.f, 4.f, 4.f, 0.1f, 0.95f, 0.3f};
    const auto out = decode_detections(d.data(), 7, 1, true, 0.5f, 0.45f);
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_EQ(out.detections[0].class_id, 1);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.95f);
}

TEST(DecodeDetections, SuppressesOverlappingBoxes)
{
    // Two nearly identical boxes, N = 2, K = 1.
    const std::vector<float> d = {
        100.f, 101.f,   // cx
        50.f,  50.f,    // cy
        20.f,  20.f,    // w
        10.f,  10.f,    // h
        0.9f,  0.8f,    // class 0
    };
    const auto out = decode_detections(d.data(), 5, 2, true, 0.5f, 0.45f);
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.9f);
}

TEST(DecodeDetections, RejectsChannelCountWithoutClasses)
{
    const std::vector<float> d = {0, 0, 0, 0};
    const auto out = decode_detections(d.data(), 4, 1, true, 0.5f, 0.45f);
    EXPECT_FALSE(out.valid);
    EXPECT_TRUE(out.detections.empty());
}

TEST(DecodeDetections, RejectsNullBuffer)
{
    const auto out = decode_detections(nullptr, 5, 1, true, 0.5f, 0.45f);
    EXPECT_FALSE(out.valid);
    EXPECT_TRUE(out.detections.empty());
}

TEST(DecodeDetections, KeepsDetectionAtConfidenceThresholdBoundary)
{
    // The confidence gate is a strict `<`: best_prob == conf_thres must be
    // kept, not dropped. A swap to `<=` would fail this test.
    const auto d = single_anchor(100.f, 50.f, 20.f, 10.f, 0.5f);
    const auto out = decode_detections(d.data(), 5, 1,
                                       /*cls_is_probability=*/true,
                                       0.5f, 0.45f);
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.5f);
}

TEST(DecodeDetections, KeepsSecondBoxWhenIouExactlyEqualsThreshold)
{
    // Box A: cx=100, w=20, h=10 -> [90,45]-[110,55], area 200.
    // Box B: cx=110, w=20, h=10 -> [100,45]-[120,55], area 200.
    // Intersection: [100,45]-[110,55] -> 10 x 10 = 100, all integer-valued
    // and exactly representable in float, so iou_thres below is computed
    // with the identical expression production iou() uses and is therefore
    // bit-exactly equal to the runtime result, not a rounding-dependent
    // guess.
    const float inter     = 100.f;
    const float area_a    = 200.f;
    const float area_b    = 200.f;
    const float iou_thres = inter / (area_a + area_b - inter + 1e-6f);

    const std::vector<float> d = {
        100.f, 110.f,   // cx
        50.f,  50.f,    // cy
        20.f,  20.f,    // w
        10.f,  10.f,    // h
        0.9f,  0.8f,    // class 0
    };
    const auto out = decode_detections(d.data(), 5, 2, true, 0.5f, iou_thres);
    // The NMS suppression test is strict `>`: it does not suppress at exact
    // equality, so both boxes survive.
    ASSERT_EQ(out.detections.size(), 2u);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.9f);
    EXPECT_FLOAT_EQ(out.detections[1].score, 0.8f);
}

TEST(DecodeDetections, KeepsBothNonOverlappingBoxes)
{
    // Two anchors far enough apart to have zero IoU; both must survive NMS,
    // ordered by descending score.
    const std::vector<float> d = {
        100.f, 1000.f,  // cx
        50.f,  50.f,    // cy
        20.f,  20.f,    // w
        10.f,  10.f,    // h
        0.9f,  0.8f,    // class 0
    };
    const auto out = decode_detections(d.data(), 5, 2, true, 0.5f, 0.45f);
    ASSERT_EQ(out.detections.size(), 2u);
    EXPECT_FLOAT_EQ(out.detections[0].score, 0.9f);
    EXPECT_FLOAT_EQ(out.detections[1].score, 0.8f);
}
