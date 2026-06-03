//
// Created by atanasko on 27.4.26.
// Developed by TranHuuNhatHuy on 3.6.26.
//

#include <visualization/visualization.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace visualization {


	namespace {


		/**
		* @brief Utility func to get class color based on class ID
		*
		* @param class_id class ID of the detected object, as per AutoSpeed:
		*	- `1` : CIPO
		*	- `2` : cutting-in vehicles
		*	- `3` : other vehicles
		*
		* @return cv::Scalar color corresponding to the class ID :
		*	- CIPO : red (0, 0, 255)
		*	- Cutting-in vehicles : yellow (0, 255, 255)
		*	- Other vehicles : blue (255, 0, 0)
		*	- Default/Unknown : gray (180, 180, 180)
		*/
		cv::Scalar class_color(int class_id) {

			switch (class_id) {
				case 1:
					return kCipoColor;
				case 2:
					return kCuttingInColor;
				case 3:
					return kOtherCarsColor;
				default:
					return cv::Scalar(180, 180, 180);
			}

		};


		/**
		* @brief Utility func to load wheel icon with fallback paths
		*
		* @return cv::Mat containing the loaded wheel icon,
		* 		  or an empty Mat if loading somehow fails
		*/
		cv::Mat load_wheel_icon() {

			const std::vector<std::filesystem::path> candidates = {
				std::filesystem::path(__FILE__).parent_path() / "assets" / "wheel.png",
				std::filesystem::path(__FILE__).parent_path() / ".." / ".." / ".." / ".." / ".." / ".." / "Media" / "wheel.png"
			};

			for (const auto &candidate : candidates) {
				cv::Mat icon = cv::imread(candidate.string(), cv::IMREAD_UNCHANGED);
				if (!icon.empty()) {
					return icon;
				}
			}

			return cv::Mat();

		};

	}  // namespace


	// Legacy render_frame implementation
	
	bool render_frame(
		const cv::Mat &frame,
		const std::string &window_name,
		const std::vector<std::string> &overlay_lines
	) {
		if (frame.empty()) {
			return false;
		}

		cv::namedWindow(window_name, cv::WINDOW_NORMAL);
		cv::resizeWindow(window_name, frame.cols, frame.rows);

		cv::Mat display = frame.clone();

		if (!overlay_lines.empty()) {
			const int font_face = cv::FONT_HERSHEY_SIMPLEX;
			const double font_scale = 0.55;
			const int thickness = 1;
			const int line_gap = 8;
			const int left_padding = 12;
			const int top_padding = 24;

			int box_width = 0;
			int box_height = 0;
			for (const auto &line : overlay_lines) {
				int baseline = 0;
				cv::Size text_size = cv::getTextSize(line, font_face, font_scale, thickness, &baseline);
				box_width = std::max(box_width, text_size.width);
				box_height += text_size.height + line_gap;
			}

			cv::rectangle(
				display,
				cv::Rect(6, 6, box_width + left_padding * 2, box_height + top_padding),
				cv::Scalar(0, 0, 0),
				cv::FILLED
			);

			int y = 6 + top_padding - 6;
			for (const auto &line : overlay_lines) {
				int baseline = 0;
				cv::Size text_size = cv::getTextSize(line, font_face, font_scale, thickness, &baseline);
				y += text_size.height;
				cv::putText(
					display,
					line,
					cv::Point(12, y),
					font_face,
					font_scale,
					cv::Scalar(0, 255, 255),
					thickness,
					cv::LINE_AA
				);
				y += line_gap;
			}
		}

		cv::imshow(window_name, display);
		cv::waitKey(1);
		return true;
	}

	void close_windows() {
		cv::destroyAllWindows();
	}

}  // namespace visualization
