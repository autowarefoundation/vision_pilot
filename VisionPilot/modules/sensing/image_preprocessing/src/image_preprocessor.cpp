#include <filesystem>
#include <common/utils.hpp>
#include <image_preprocessing/image_preprocessor.hpp>

ImagePreprocessor::ImagePreprocessor()
{
    C_ = load_matrix("homography_C_matrix.yaml", "C");
}

void ImagePreprocessor::preprocess(const cv::Mat& image, cv::Mat& warped_image, cv::Mat& resized_image,
                                   const cv::Size& size) const
{
    cv::warpPerspective(image, warped_image, C_, cv::Size(1024, 512), cv::INTER_LINEAR,
                        cv::BORDER_REFLECT_101);

    // AutoSteer / AutoSpeed: top-crop to 2:1 → resize 1024×512
    // (matches Python --preprocess top-crop-2-1 / auto_steer_infer.py).
    const int crop_top = compute_top_crop_2_1(image.rows, image.cols);
    const cv::Rect roi(0, crop_top, image.cols, image.rows - crop_top);
    cv::Mat cropped = image(roi);
    cv::resize(cropped, resized_image, size, 0, 0, cv::INTER_LINEAR);
}
