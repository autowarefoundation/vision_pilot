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

    int crop_h = (image.rows - image.cols / 2) / 2;
    cv::Rect roi(0, crop_h, image.cols, image.rows - 2 * crop_h);
    cv::Mat cropped_image = image(roi);
    cv::resize(cropped_image, resized_image, size);
}
