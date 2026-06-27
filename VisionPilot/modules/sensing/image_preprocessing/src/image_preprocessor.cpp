#include <image_preprocessing/image_preprocessor.hpp>

ImagePreprocessor::ImagePreprocessor() : homography_C_matrix_path("config/homography_C_matrix.yaml") {
    const cv::FileStorage fs(homography_C_matrix_path, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open calibration file: " + homography_C_matrix_path);
    }
    fs["C"] >> C;
}

void ImagePreprocessor::preprocess(const cv::Mat &image, cv::Mat &warped_image, cv::Mat &resized_image,
                                   const cv::Size &size) const {
    cv::warpPerspective(image, warped_image, C, cv::Size(1024, 512), cv::INTER_LINEAR,
                        cv::BORDER_REFLECT_101);
    cv::resize(image, resized_image, size);
}
