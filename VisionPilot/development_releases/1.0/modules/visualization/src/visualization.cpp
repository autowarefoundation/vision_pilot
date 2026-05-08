//
// Created by atanasko on 27.4.26.
//

#include <visualization/visualization.hpp>

#include <algorithm>

namespace visualization {

bool render_frame(
	const cv::Mat &frame,
	const std::string &window_name,
	const std::vector<std::string> &overlay_lines
) {
	if (frame.empty()) {
		return false;
	}

	cv::namedWindow(window_name, cv::WINDOW_NORMAL);

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
