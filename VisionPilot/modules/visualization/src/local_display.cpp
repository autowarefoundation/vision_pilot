#include <opencv2/highgui.hpp>
#include <visualization/local_display.hpp>
#include <visualization/visualization.hpp>

namespace visualization
{
    LocalDisplay::LocalDisplay()
    {
        cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
    }

    LocalDisplay::~LocalDisplay()
    {
        cv::destroyAllWindows();
    }

    bool LocalDisplay::render_frame(const cv::Mat& display_frame)
    {
        cv::resizeWindow("VisionPilot", display_frame.cols, display_frame.rows);
        cv::imshow("VisionPilot", display_frame);
        cv::waitKey(1);
        return true;
    }
}
