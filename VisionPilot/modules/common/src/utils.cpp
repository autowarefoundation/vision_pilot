#include <common/utils.hpp>

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
