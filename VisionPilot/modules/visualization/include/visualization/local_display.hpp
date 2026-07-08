#ifndef VISIONPILOT_DISPLAY_HPP
#define VISIONPILOT_DISPLAY_HPP
#include <opencv2/core/mat.hpp>
#include <visualization/visual_interface.hpp>

namespace visualization
{
    class LocalDisplay : public VisualInterface
    {
    public:
        LocalDisplay();
        ~LocalDisplay();

        bool render_frame(const cv::Mat& display_frame) override;
    };
}

#endif //VISIONPILOT_DISPLAY_HPP
