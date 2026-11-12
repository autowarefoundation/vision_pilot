#ifndef VISIONPILOT_IMAGE_PREPROCESSOR_HPP
#define VISIONPILOT_IMAGE_PREPROCESSOR_HPP
#include <string>
#include <opencv2/opencv.hpp>

class ImagePreprocessor {
public:
    // Loads C from the build-time generated homography_C_matrix.yaml (derived from config/H.yaml).
    ImagePreprocessor();

    // Derives C at runtime from a ground homography H (raw camera pixel → world),
    // so the BEV warp always matches the H used by the fusion modules.
    explicit ImagePreprocessor(const cv::Mat& H);

    // Same computation as scripts/find_homography_C_matrix.py.
    static cv::Mat compute_C(const cv::Mat& H);

    ~ImagePreprocessor() = default;

    void preprocess(const cv::Mat &image, cv::Mat &warped_image, cv::Mat &resized_image, const cv::Size &size) const;

    // C maps raw camera pixel → warped 1024×512 BEV (perspective transform).
    const cv::Mat& C_mat()   const { return C_; }

private:
    // std::string homography_C_matrix_path;
    cv::Mat C_;
};

#endif //VISIONPILOT_IMAGE_PREPROCESSOR_HPP
