#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>

#include <radar_interface/file_interface.hpp>

// RadarPoint is 3 contiguous floats with no padding, so a sweep can be read
// straight into the vector's backing storage.
static_assert(sizeof(RadarPoint) == 3 * sizeof(float),
              "RadarPoint must be tightly packed (3 x float32)");

namespace radar_interface {

FileInterface::FileInterface(const std::string& filename, bool loop) : loop_(loop)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        throw std::runtime_error("FileInterface: cannot open " + filename);
    }

    char magic[4] = {0};
    file.read(magic, 4);
    if (std::string(magic, 4) != "RDR1")
    {
        throw std::runtime_error("FileInterface: not an RDR1 radar file: " + filename);
    }

    std::uint32_t num_sweeps = 0;
    file.read(reinterpret_cast<char*>(&num_sweeps), sizeof(num_sweeps));
    points_.resize(num_sweeps);

    for (std::uint32_t s = 0; s < num_sweeps; ++s)
    {
        std::uint32_t n = 0;
        file.read(reinterpret_cast<char*>(&n), sizeof(n));

        points_[s].resize(n);
        if (n > 0)
        {
            file.read(reinterpret_cast<char*>(points_[s].data()),
                      static_cast<std::streamsize>(n) * sizeof(RadarPoint));
        }
        if (!file)
        {
            throw std::runtime_error("FileInterface: truncated radar file: " + filename);
        }
    }

    file.close();
}

std::vector<RadarPoint> FileInterface::read_points()
{
    if (points_.empty())
    {
        throw std::runtime_error("FileInterface: no radar sweeps loaded");
    }

    if (frame_cnt_ >= points_.size())
    {
        if (loop_)
        {
            frame_cnt_ = 0;
        }
        else
        {
            throw std::runtime_error("FileInterface: read_points() called past end of data");
        }
    }

    return points_[frame_cnt_++];
}
} // namespace radar_interface