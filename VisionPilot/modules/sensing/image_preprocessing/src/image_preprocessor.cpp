#include <filesystem>
#include <stdexcept>
#include <vector>
#include <common/utils.hpp>
#include <image_preprocessing/image_preprocessor.hpp>

ImagePreprocessor::ImagePreprocessor()
{
    C_ = load_matrix("homography_C_matrix.yaml", "C");
}

ImagePreprocessor::ImagePreprocessor(const cv::Mat& H)
    : C_(compute_C(H))
{
}

cv::Mat ImagePreprocessor::compute_C(const cv::Mat& H)
{
    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world).
    // Must stay identical to V in scripts/find_homography_C_matrix.py.
    const cv::Matx33d V(0.00209514907, -0.000941721466, -9.24906396,
                        0.00662758637, -0.000352940531, -3.33396502,
                        0.000120077371, -0.00411343505, 1.0);
    const cv::Vec3d world_pts[4] = {{15, 5, 1}, {150, 5, 1}, {15, -5, 1}, {150, -5, 1}};

    cv::Mat H64;
    H.convertTo(H64, CV_64F);
    const cv::Matx33d H_inv = cv::Matx33d(H64).inv();
    const cv::Matx33d V_inv = V.inv();

    std::vector<cv::Point2d> raw_px, bev_px;
    for (const auto& w : world_pts)
    {
        const cv::Vec3d uv = H_inv * w;
        const cv::Vec3d pq = V_inv * w;
        raw_px.emplace_back(uv[0] / uv[2], uv[1] / uv[2]);
        bev_px.emplace_back(pq[0] / pq[2], pq[1] / pq[2]);
    }

    cv::Mat C = cv::findHomography(raw_px, bev_px, 0);
    if (C.empty())
    {
        throw std::runtime_error("[ImagePreprocessor] Failed to derive C from H");
    }
    C.convertTo(C, CV_32F);
    return C;
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
