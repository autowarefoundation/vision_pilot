#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <visualization/local_display.hpp>

#if defined(ENABLE_OCCUPANCY)
#include <visualization/occupancy_view.hpp>
#endif

namespace visualization
{
    LocalDisplay::LocalDisplay(bool show_window) : show_window_(show_window)
    {
        if (show_window_) cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
    }

    LocalDisplay::~LocalDisplay()
    {
        if (show_window_)
        {
            cv::destroyAllWindows();
        }
    }

    void LocalDisplay::set_aux_frame(const cv::Mat& aux)
    {
        if (aux.empty())
        {
            aux_frame_.release();
            return;
        }
        aux.copyTo(aux_frame_);
    }

    bool LocalDisplay::visualize(cv::Mat& frame,
                                 const visionpilot::models::InferenceFrameResult& result,
                                 const Plan& plan,
                                 double ego_speed_ms,
                                 const cv::Mat& H_resized,
                                 double speed_limit_ms)
    {
        if (!show_window_) return true;

        constexpr int kOriginX = 40;
        constexpr int kOriginY = 40;
        constexpr int kGap = 8;

        build_frame(frame, result, plan, ego_speed_ms, H_resized, speed_limit_ms);

        cv::Mat occ_show;
        if (!aux_frame_.empty())
        {
            if (aux_frame_.rows != frame.rows && aux_frame_.rows > 0)
            {
                const double s =
                    static_cast<double>(frame.rows) / static_cast<double>(aux_frame_.rows);
                cv::resize(aux_frame_, occ_show, cv::Size(), s, s, cv::INTER_AREA);
            }
            else
            {
                occ_show = aux_frame_;
            }
        }

        cv::resizeWindow("VisionPilot", frame.cols, frame.rows);
        cv::imshow("VisionPilot", frame);

        if (!occ_show.empty())
        {
            if (!aux_window_ready_)
            {
                cv::namedWindow("Occupancy", cv::WINDOW_NORMAL);
#if defined(ENABLE_OCCUPANCY)
                cv::setMouseCallback("Occupancy", occupancy::on_mouse, nullptr);
#endif
                aux_window_ready_ = true;
                layout_done_ = false;
            }

            cv::resizeWindow("Occupancy", occ_show.cols, occ_show.rows);
            cv::imshow("Occupancy", occ_show);

            const bool size_changed = frame.cols != last_main_w_ ||
                frame.rows != last_main_h_ || occ_show.cols != last_aux_w_ ||
                occ_show.rows != last_aux_h_;

            if (!layout_done_ || size_changed)
            {
                cv::moveWindow("VisionPilot", kOriginX, kOriginY);
                cv::moveWindow("Occupancy", kOriginX + frame.cols + kGap, kOriginY);
                layout_done_ = true;
                last_main_w_ = frame.cols;
                last_main_h_ = frame.rows;
                last_aux_w_ = occ_show.cols;
                last_aux_h_ = occ_show.rows;
            }
        }

        const int key = cv::waitKey(1);
#if defined(ENABLE_OCCUPANCY)
        if (key >= 0) occupancy::on_key(key);
#else
        (void)key;
#endif
        return true;
    }

    bool LocalDisplay::stop()
    {
        if (show_window_)
        {
            cv::destroyAllWindows();
        }
        aux_window_ready_ = false;
        layout_done_ = false;
        return true;
    }
} // namespace visualization
