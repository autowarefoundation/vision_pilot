#include <common/utils.hpp>

#include <stdexcept>
#include <string>
#include <vector>

std::string find_config(const std::string& filename)
{
    const std::string local = "config/" + filename;
    const std::string system = "/usr/share/visionpilot/config/" + filename;

    if (std::filesystem::exists(local)) return local;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("Config file not found: " + filename);
}

cv::Mat load_matrix(const std::string& filename, const std::string& matrix)
{
    const std::string path = find_config(filename);
    const cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open calibration file: ");
    }
    cv::Mat M;
    fs[matrix] >> M;

    return M;
}

cv::Mat compute_preprocess_homography(const cv::Mat& H)
{
    if (H.rows != 3 || H.cols != 3)
        throw std::invalid_argument("compute_preprocess_homography: H must be 3x3");

    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world).
    // Must match V in scripts/find_homography_C_matrix.py.
    const cv::Matx33d V(
        0.00209514907, -0.000941721466, -9.24906396,
        0.00662758637, -0.000352940531, -3.33396502,
        0.000120077371, -0.00411343505, 1.0);
    // Canonical ground points (x forward, y left) [m], as in the script.
    const std::vector<cv::Point2d> world = {{15, 5}, {150, 5}, {15, -5}, {150, -5}};

    cv::Mat H64;
    H.convertTo(H64, CV_64F);
    std::vector<cv::Point2d> raw_px, bev_px;
    cv::perspectiveTransform(world, raw_px, cv::Mat(H64.inv()));
    cv::perspectiveTransform(world, bev_px, cv::Mat(V.inv()));

    cv::Mat C = cv::findHomography(raw_px, bev_px, 0);
    if (C.empty())
        throw std::runtime_error("compute_preprocess_homography: degenerate H");
    C.convertTo(C, CV_32F);  // the script stores C as float32
    return C;
}
