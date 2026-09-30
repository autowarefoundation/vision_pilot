#include <gtest/gtest.h>

#include <models/tensor_prep.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace vpm = visionpilot::models;

namespace {

// The conversion the fused kernels replace, written the long way: BGR to RGB,
// scale to [0, 1], split into planes, then normalise. This is the contract
// to_chw_imagenet() and to_chw_unit() have to keep, so the test states it
// independently rather than reusing anything from the implementation.
std::vector<float> reference(const cv::Mat& bgr, bool imagenet)
{
    cv::Mat rgb, f32;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);
    std::vector<cv::Mat> ch(3);
    cv::split(f32, ch);

    const int hw = bgr.rows * bgr.cols;
    std::vector<float> out(static_cast<std::size_t>(3 * hw));
    for (int c = 0; c < 3; ++c) {
        float* dst = out.data() + c * hw;
        const float* src = reinterpret_cast<const float*>(ch[c].data);
        for (int i = 0; i < hw; ++i)
            dst[i] = imagenet
                         ? (src[i] - vpm::IMAGENET_MEAN[c]) / vpm::IMAGENET_STD[c]
                         : src[i];
    }
    return out;
}

// Deliberately not a round multiple of any vector width, so the kernels' tail
// handling is exercised.
constexpr int W = 67;
constexpr int H = 33;

cv::Mat pseudo_random_bgr(int rows, int cols)
{
    cv::Mat bgr(rows, cols, CV_8UC3);
    unsigned int s = 12345u;
    for (int i = 0; i < rows * cols * 3; ++i) {
        s = s * 1103515245u + 12345u;
        bgr.data[i] = static_cast<unsigned char>((s >> 16) & 0xff);
    }
    return bgr;
}

double max_abs_diff(const std::vector<float>& a, const std::vector<float>& b)
{
    double m = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        m = std::max(m, static_cast<double>(std::fabs(a[i] - b[i])));
    return m;
}

}  // namespace

TEST(TensorPrep, ImagenetAgreesWithTheReferencePipeline)
{
    const cv::Mat bgr = pseudo_random_bgr(H, W);
    const std::vector<float> want = reference(bgr, true);

    std::vector<float> got(want.size());
    vpm::to_chw_imagenet(bgr, got.data(), got.size());

    // Folding /255, -MEAN and /STD into one multiply-add reassociates the
    // arithmetic, so this is an agreement bound rather than equality. The
    // normalised range is about 4.8 wide and an int8 input quantises it in
    // steps of ~0.019, so 1e-6 is four orders of magnitude below anything a
    // deployed network can resolve.
    EXPECT_LT(max_abs_diff(want, got), 1e-6);
}

TEST(TensorPrep, UnitScalingIsBitIdenticalToTheReferencePipeline)
{
    const cv::Mat bgr = pseudo_random_bgr(H, W);
    const std::vector<float> want = reference(bgr, false);

    std::vector<float> got(want.size());
    vpm::to_chw_unit(bgr, got.data(), got.size());

    // Nothing is reassociated here -- both sides compute src * (1/255) -- so
    // the fused kernel has to reproduce the old output exactly.
    ASSERT_EQ(want.size(), got.size());
    for (std::size_t i = 0; i < want.size(); ++i)
        ASSERT_EQ(want[i], got[i]) << "element " << i;
}

TEST(TensorPrep, PlanesComeOutRgbFromABgrSource)
{
    // Two pixels, chosen so every channel is distinguishable.
    cv::Mat bgr(1, 2, CV_8UC3);
    bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);    // B G R
    bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(40, 50, 60);

    std::vector<float> got(3 * 2);
    vpm::to_chw_unit(bgr, got.data(), got.size());

    // Plane 0 is red, plane 1 green, plane 2 blue.
    EXPECT_FLOAT_EQ(got[0], 30.0f / 255.0f);
    EXPECT_FLOAT_EQ(got[1], 60.0f / 255.0f);
    EXPECT_FLOAT_EQ(got[2], 20.0f / 255.0f);
    EXPECT_FLOAT_EQ(got[3], 50.0f / 255.0f);
    EXPECT_FLOAT_EQ(got[4], 10.0f / 255.0f);
    EXPECT_FLOAT_EQ(got[5], 40.0f / 255.0f);
}

TEST(TensorPrep, ReadsAPaddedSourceRowByRow)
{
    // A region of interest has a row stride wider than its own width, which is
    // exactly what a single flat pointer walk over .data would get wrong.
    const cv::Mat big = pseudo_random_bgr(H + 8, W + 11);
    const cv::Mat roi = big(cv::Rect(3, 2, W, H));
    ASSERT_FALSE(roi.isContinuous());

    const std::vector<float> want = reference(roi.clone(), true);

    std::vector<float> got(want.size());
    vpm::to_chw_imagenet(roi, got.data(), got.size());

    EXPECT_LT(max_abs_diff(want, got), 1e-6);
}

TEST(TensorPrep, WritesExactlyThreePlanesAndNothingPastThem)
{
    const cv::Mat bgr = pseudo_random_bgr(H, W);
    const std::size_t n = static_cast<std::size_t>(3 * H * W);

    // One canary element either side of the region the kernel may touch.
    std::vector<float> buf(n + 2, -12345.0f);
    vpm::to_chw_imagenet(bgr, buf.data() + 1, n);

    EXPECT_FLOAT_EQ(buf.front(), -12345.0f);
    EXPECT_FLOAT_EQ(buf.back(), -12345.0f);
    for (std::size_t i = 1; i <= n; ++i)
        ASSERT_NE(buf[i], -12345.0f) << "element " << (i - 1) << " left unwritten";
}

TEST(TensorPrep, ConstantsAreTheImageNetValues)
{
    // Pinned literally: the reference in this file derives from these, so a
    // typo in them would otherwise agree with itself and pass everything else.
    EXPECT_FLOAT_EQ(vpm::IMAGENET_MEAN[0], 0.485f);
    EXPECT_FLOAT_EQ(vpm::IMAGENET_MEAN[1], 0.456f);
    EXPECT_FLOAT_EQ(vpm::IMAGENET_MEAN[2], 0.406f);
    EXPECT_FLOAT_EQ(vpm::IMAGENET_STD[0], 0.229f);
    EXPECT_FLOAT_EQ(vpm::IMAGENET_STD[1], 0.224f);
    EXPECT_FLOAT_EQ(vpm::IMAGENET_STD[2], 0.225f);
}

TEST(TensorPrep, RefusesASourceThatIsNotBgr8)
{
    std::vector<float> out(3 * H * W);

    const cv::Mat wrong_depth(H, W, CV_32FC3, cv::Scalar::all(0));
    EXPECT_THROW(vpm::to_chw_imagenet(wrong_depth, out.data(), out.size()),
                 std::invalid_argument);

    const cv::Mat wrong_channels(H, W, CV_8UC1, cv::Scalar::all(0));
    EXPECT_THROW(vpm::to_chw_unit(wrong_channels, out.data(), out.size()),
                 std::invalid_argument);

    EXPECT_THROW(vpm::to_chw_imagenet(cv::Mat(), out.data(), out.size()),
                 std::invalid_argument);
}

TEST(TensorPrep, RefusesABufferTooSmallForTheImage)
{
    const cv::Mat bgr = pseudo_random_bgr(H, W);
    std::vector<float> out(static_cast<std::size_t>(3 * H * W));

    // One element short is still refused, rather than overrunning the caller's
    // buffer by a row.
    EXPECT_THROW(vpm::to_chw_imagenet(bgr, out.data(), out.size() - 1),
                 std::invalid_argument);
    EXPECT_THROW(vpm::to_chw_unit(bgr, out.data(), out.size() - 1),
                 std::invalid_argument);
    EXPECT_THROW(vpm::to_chw_unit(bgr, nullptr, out.size()),
                 std::invalid_argument);
}
