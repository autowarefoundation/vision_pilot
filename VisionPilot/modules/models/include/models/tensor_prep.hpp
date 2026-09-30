#pragma once

#include <opencv2/core.hpp>

#include <cstddef>

namespace visionpilot::models {

// ImageNet normalisation constants, in RGB plane order.
inline constexpr float IMAGENET_MEAN[3] = {0.485f, 0.456f, 0.406f};
inline constexpr float IMAGENET_STD[3]  = {0.229f, 0.224f, 0.225f};

// Turn a BGR8 image into the planar RGB float CHW tensor the networks take.
//
// `out` receives 3 * bgr.rows * bgr.cols floats -- plane 0 red, plane 1 green,
// plane 2 blue -- and `out_size` is its capacity in floats, checked so a
// too-small buffer is refused rather than overrun. Nothing is allocated: the
// caller owns the buffer and is meant to reuse it across frames, because at
// 1024x512 each of these tensors is 6 MB and value-initialising a fresh one
// per frame costs more than the conversion itself.
//
// to_chw_imagenet applies the normalisation the AutoDrive branch expects,
// (rgb/255 - MEAN[c]) / STD[c]; to_chw_unit only scales to [0, 1]. Both fold
// colour conversion, scaling and normalisation into a single multiply-add per
// element over one read pass of the source, which is what lets the compiler
// vectorise them.
//
// Folding the ImageNet division into that multiply-add reassociates the
// arithmetic, so to_chw_imagenet is not bit-identical to dividing separately;
// it agrees to within 1e-6 absolute on a range about 4.8 wide, four orders of
// magnitude below the ~0.019 step an int8 input quantises that range in.
// to_chw_unit performs the same multiply the old path did and is exact.
//
// Both throw std::invalid_argument if `bgr` is empty or not CV_8UC3, or if
// `out` is null or `out_size` is smaller than the image needs.
void to_chw_imagenet(const cv::Mat& bgr, float* out, std::size_t out_size);
void to_chw_unit(const cv::Mat& bgr, float* out, std::size_t out_size);

}  // namespace visionpilot::models
