#include <models/tensor_prep.hpp>

#include <stdexcept>
#include <string>

namespace visionpilot::models {

namespace {

// The source is BGR and plane c of the output is RGB, so plane 0 reads the
// third byte of each pixel.
constexpr int BGR_OF_PLANE[3] = {2, 1, 0};

void check(const cv::Mat& bgr, const float* out, std::size_t out_size,
           const char* who)
{
    if (bgr.empty())
        throw std::invalid_argument(std::string("[") + who + "] source image is empty");
    if (bgr.type() != CV_8UC3)
        throw std::invalid_argument(
            std::string("[") + who + "] source image must be CV_8UC3 (8-bit BGR), got type " +
            std::to_string(bgr.type()));
    if (out == nullptr)
        throw std::invalid_argument(std::string("[") + who + "] output buffer is null");

    const std::size_t need =
        3u * static_cast<std::size_t>(bgr.rows) * static_cast<std::size_t>(bgr.cols);
    if (out_size < need)
        throw std::invalid_argument(
            std::string("[") + who + "] output buffer holds " + std::to_string(out_size) +
            " floats but a " + std::to_string(bgr.cols) + "x" + std::to_string(bgr.rows) +
            " image needs " + std::to_string(need));
}

// One multiply-add per element, reading each source row through ptr() so a
// padded cv::Mat (a region of interest, say) is handled correctly, and writing
// straight into the three output planes. Deliberately plain: at -O3 the
// compiler turns this into an ld3 de-interleaving loop that matches
// hand-written NEON, and it stays portable.
void convert(const cv::Mat& bgr, const float scale[3], const float bias[3], float* out)
{
    const int rows = bgr.rows;
    const int cols = bgr.cols;
    const std::size_t hw = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);

    for (int c = 0; c < 3; ++c) {
        const int   off = BGR_OF_PLANE[c];
        const float s   = scale[c];
        const float b   = bias[c];
        float* plane = out + c * hw;

        for (int y = 0; y < rows; ++y) {
            const unsigned char* src = bgr.ptr<unsigned char>(y);
            float* dst = plane + static_cast<std::size_t>(y) * cols;
            for (int x = 0; x < cols; ++x)
                dst[x] = static_cast<float>(src[3 * x + off]) * s + b;
        }
    }
}

}  // namespace

void to_chw_imagenet(const cv::Mat& bgr, float* out, std::size_t out_size)
{
    check(bgr, out, out_size, "to_chw_imagenet");

    // (v/255 - MEAN) / STD  ==  v * (1 / (255 * STD)) + (-MEAN / STD)
    float scale[3], bias[3];
    for (int c = 0; c < 3; ++c) {
        scale[c] = 1.0f / (255.0f * IMAGENET_STD[c]);
        bias[c]  = -IMAGENET_MEAN[c] / IMAGENET_STD[c];
    }
    convert(bgr, scale, bias, out);
}

void to_chw_unit(const cv::Mat& bgr, float* out, std::size_t out_size)
{
    check(bgr, out, out_size, "to_chw_unit");

    // The same multiply the old cvtColor + convertTo path performed, so this
    // one is exact rather than merely close.
    const float scale[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    const float bias[3]  = {0.0f, 0.0f, 0.0f};
    convert(bgr, scale, bias, out);
}

}  // namespace visionpilot::models
