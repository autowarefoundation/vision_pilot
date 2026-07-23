#ifndef VISIONPILOT_DISPLAY_HPP
#define VISIONPILOT_DISPLAY_HPP
#include <opencv2/core/mat.hpp>
#include <visualization/visual_interface.hpp>

namespace visualization
{
    class LocalDisplay : public VisualInterface
    {
    public:
        // show_window == false runs headless: no OpenCV window is created and
        // render_frame() becomes a no-op. This keeps VisionPilot usable in
        // environments without a display (e.g. containers, CI).
        explicit LocalDisplay(bool show_window = true);
        ~LocalDisplay();

        bool render_frame(const cv::Mat& display_frame) override;
        bool stop() override;

    private:
        bool show_window_;
    };
}

#endif //VISIONPILOT_DISPLAY_HPP
