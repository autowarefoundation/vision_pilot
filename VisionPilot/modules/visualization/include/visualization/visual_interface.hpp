#ifndef VISIONPILOT_VISUAL_INTERFACE_HPP
#define VISIONPILOT_VISUAL_INTERFACE_HPP
#include <opencv2/highgui.hpp>

class VisualInterface
{
public:
    VisualInterface();
    virtual ~VisualInterface() = default;

    virtual bool render_frame(const cv::Mat& display_frame) = 0;
};

#endif //VISIONPILOT_VISUAL_INTERFACE_HPP
