#ifndef VISIONPILOT_radar_FILE_INTERFACE_HPP
#define VISIONPILOT_radar_FILE_INTERFACE_HPP

#include <string>
#include <radar_interface/radar_interface.hpp>

namespace radar_interface
{
    class FileInterface : public RadarInterface
    {
    public:
        FileInterface(const std::string& filename, bool loop);
        virtual ~FileInterface() = default;

        std::vector<RadarPoint> read_points();

    private:
        bool loop_;
        int frame_cnt_ = 0;
        std::vector<std::vector<RadarPoint>> points_;
    };
} // namespace radar_interface

#endif //VISIONPILOT_radar_FILE_INTERFACE_HPP
