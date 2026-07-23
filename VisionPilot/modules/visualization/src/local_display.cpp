#include <opencv2/highgui.hpp>
#include <visualization/local_display.hpp>
#include <visualization/visualization.hpp>

namespace visualization
{
    LocalDisplay::LocalDisplay(bool show_window)
        : show_window_(show_window)
    {
        if (show_window_)
            cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
    }

    LocalDisplay::~LocalDisplay()
    {
        cv::destroyAllWindows();
    }

    bool LocalDisplay::render_frame(const cv::Mat& display_frame)
    {
        if (!show_window_)
            return true;
        cv::resizeWindow("VisionPilot", display_frame.cols, display_frame.rows);
        cv::imshow("VisionPilot", display_frame);
        cv::waitKey(1);
        return true;
    }

    bool LocalDisplay::stop()
    {
        cv::destroyAllWindows();
        return true;
    }
}
