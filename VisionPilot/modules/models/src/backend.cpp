#include "models/backend.hpp"

#include <filesystem>
#include <stdexcept>

namespace visionpilot::models {

std::string find_model(const std::string& filename)
{
    if (filename.empty())
        throw std::runtime_error("[models] Model filename must not be empty");

    const std::string local  = "modules/models/weights/" + filename;
    const std::string system =
        "/usr/share/visionpilot/modules/models/weights/" + filename;

    if (std::filesystem::exists(local))  return local;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("[models] Model file not found: " + filename);
}

}  // namespace visionpilot::models
