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
					return kGrayColor;
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
				// The one I added in src/assets
				std::filesystem::path(__FILE__).parent_path() / "assets" / "wheel.png",
				// Fallback to the one in Media (that Atanasko or Pranav added wayyyy back in Dec 2025)
				std::filesystem::path(__FILE__).parent_path() / ".." / ".." / ".." / ".." / ".." / ".." / "Media" / "wheel.png"
			};

			for (const auto &candidate : candidates) {
				cv::Mat icon = cv::imread(
					candidate.string(), 
					cv::IMREAD_UNCHANGED
				);
				if (!icon.empty()) {
					return icon;
				}
			}

			return cv::Mat();

		};


		/**
		* @brief Utility func to rotate the steering wheel assset by a specified angle.
		*
		* @param icon cv::Mat containing the steering wheel icon (with alpha channel)
		* @param angle_degrees float angle in degrees to rotate the icon (positive = clockwise)
		*
		* @return cv::Mat containing the rotated icon, or an empty Mat if input icon is empty
		*/
		cv::Mat rotate_icon(
			const cv::Mat &icon, 
			float angle_degrees
		) {

			if (icon.empty()) {
				return cv::Mat();
			}

			// Define frame of rotation
			const cv::Point2f center(icon.cols * 0.5F, icon.rows * 0.5F);
			const cv::Mat rotation = cv::getRotationMatrix2D(center, angle_degrees, 1.0);

			const cv::Rect2f bounds = cv::RotatedRect(
				cv::Point2f(), 
				icon.size(), 
				angle_degrees
			).boundingRect2f();
			cv::Mat adjusted_rotation = rotation.clone();
			adjusted_rotation.at<double>(0, 2) += bounds.width * 0.5 - center.x;
			adjusted_rotation.at<double>(1, 2) += bounds.height * 0.5 - center.y;

			// Perform rotation
			cv::Mat rotated;
			const cv::Scalar border_color = icon.channels() == 4 ? cv::Scalar(0, 0, 0, 0) : cv::Scalar(255, 255, 255);
			cv::warpAffine(
				icon,
				rotated,
				adjusted_rotation,
				bounds.size(),
				cv::INTER_LINEAR,
				cv::BORDER_CONSTANT,
				border_color
			);

			return rotated;

		};


		/**
		* @brief Utility func to overlay an icon (with optional alpha channel) onto a canvas at a specified position
		*
		* @param canvas cv::Mat representing the background image onto which the icon will be overlaid (modified in-place)
		* @param icon cv::Mat containing the icon to overlay (can have alpha channel for transparency)
		* @param top_left cv::Point specifying the top-left corner where the icon should be placed on the canvas
		*
		* This func should handle boundary conditions to ensure icon is fully visible within the canvas.
		*/
		void overlay_icon(
			cv::Mat &canvas, 
			const cv::Mat &icon, 
			const cv::Point &top_left
		) {

			if (icon.empty()) {
				return;
			}

			// Calculate ROI on canva and icon, ensuring we aint go out of bounds
			const int x = std::clamp(
				top_left.x, 
				0, 
				std::max(0, canvas.cols - 1)
			);
			const int y = std::clamp(
				top_left.y, 
				0, 
				std::max(0, canvas.rows - 1)
			);
			const int width = std::min(
				icon.cols, 
				canvas.cols - x
			);
			const int height = std::min(
				icon.rows, 
				canvas.rows - y
			);
			if (width <= 0 || height <= 0) {
				return;
			}

			const cv::Rect dst_rect(x, y, width, height);
			const cv::Rect src_rect(0, 0, width, height);
			cv::Mat dst_roi = canvas(dst_rect);
			const cv::Mat src_roi = icon(src_rect);

			// If icon has alpha channel, do alpha blending. 
			// Otherwise, just copy itself onto canvas.
			if (src_roi.channels() == 4) {

				std::vector<cv::Mat> channels;
				cv::split(src_roi, channels);
				const cv::Mat alpha = channels[3];
				cv::Mat src_bgr;
				cv::merge(
					std::vector<cv::Mat>{
						channels[0], 
						channels[1], 
						channels[2]
					}, 
					src_bgr
				);

				cv::Mat src_float;
				cv::Mat dst_float;
				src_bgr.convertTo(src_float, CV_32FC3);
				dst_roi.convertTo(dst_float, CV_32FC3);

				cv::Mat alpha_float;
				alpha.convertTo(
					alpha_float, 
					CV_32FC1, 
					1.0 / 255.0
				);
				std::vector<cv::Mat> alpha_channels{
					alpha_float, 
					alpha_float, 
					alpha_float
				};
				cv::Mat alpha_3;
				cv::merge(alpha_channels, alpha_3);

				const cv::Mat ones(alpha_3.size(), alpha_3.type(), cv::Scalar::all(1.0));
				const cv::Mat blended = src_float.mul(alpha_3) + dst_float.mul(ones - alpha_3);
				blended.convertTo(dst_roi, dst_roi.type());
			
			} else {
				src_roi.copyTo(dst_roi);
			}

		};


		/**
		* @brief Utility wrapper func to draw centered text within a specified rectangle on the canvas
		*
		* @param canvas cv::Mat representing the image on which to draw (modified in-place)
		* @param text std::string containing the text to draw
		* @param rect cv::Rect specifying the bounding box within which the text should be centered
		* @param scale double specifying the font scale for the text
		* @param color cv::Scalar specifying the color of the text
		* @param thickness int specifying the thickness of the text (default = 1)
		*
		*/
		void draw_text_centered(
			cv::Mat &canvas, 
			const std::string &text, 
			const cv::Rect &rect, 
			double scale, 
			const cv::Scalar &color, 
			int thickness = 1
		) {
			
			int baseline = 0;
			const cv::Size text_size = cv::getTextSize(
				text, 
				cv::FONT_HERSHEY_SIMPLEX, 
				scale, 
				thickness, 
				&baseline
			);

			// div-center text within rect
			const int x = rect.x + std::max(0, (rect.width - text_size.width) / 2);
			const int y = rect.y + std::max(text_size.height, (rect.height + text_size.height) / 2);
			cv::putText(
				canvas, 
				text, 
				cv::Point(x, y), 
				cv::FONT_HERSHEY_SIMPLEX, 
				scale, 
				color, 
				thickness, 
				cv::LINE_AA
			);

		};

		void draw_inline_value(
			cv::Mat &canvas, 
			const cv::Point &origin, 
			const std::string &title, 
			const std::string &value, 
			const cv::Scalar &value_color
		) {

			// Draw title (bold thickness 2)
			cv::putText(
				canvas, 
				title + ": ", 
				origin, 
				cv::FONT_HERSHEY_SIMPLEX,
				kFontSize, 
				kYellowColor, 
				kThickBold, 
				cv::LINE_AA
			);

			// Calculate title width to place value exactly inline
			int baseline = 0;
			const cv::Size title_size = cv::getTextSize(
				title + ": ", 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSize, 
				kThickBold, 
				&baseline
			);

			// Draw value
			cv::putText(
				canvas, 
				value, 
				cv::Point(
					origin.x + title_size.width, 
					origin.y
				), 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSize, 
				value_color, 
				kThickNormal, 
				cv::LINE_AA
			);

		};


		/**
		* @brief Utility func to blend two images with specified alpha for overlay (used for drawing translucent panels and overlays)
		*
		* @param base cv::Mat representing the background image (unmodified)
		* @param overlay cv::Mat representing the foreground image to blend on top of the base
		* @param alpha float in [0, 1] representing the opacity of the overlay (0 = fully transparent, 1 = fully opaque)
		*
		* @return cv::Mat containing the blended result
		*/
		cv::Mat blend_overlay(
			const cv::Mat &base, 
			const cv::Mat &overlay, 
			float alpha
		) {
			
			cv::Mat result;
			cv::addWeighted(
				overlay, 
				alpha, 
				base, 
				1.0F - alpha, 
				0.0, 
				result
			);
			return result;

		}
		
		
		/**
		* @brief Utility func to create a translucent panel of specified size, used for right-side info panel
		*
		* @param width int width of panel
		* @param height int height of panel
		*
		* @return cv::Mat containing translucent panel
		*/
		cv::Mat make_translucent_panel(
			int width, 
			int height
		) {

			cv::Mat base(
				height, 
				width, 
				CV_8UC3, 
				kPanelBackgroundColor
			);
			cv::Mat overlay(
				height, 
				width, 
				CV_8UC3, 
				kWhiteColor
			);

			return blend_overlay(
				base, 
				overlay, 
				kRightPanelAlpha
			);

		};

		
		/**
		* @brief Utility func to clamp a rectangle within the bounds of a given size (used for bounding box drawing)
		*
		* @param rect cv::Rect representing rectangle to clamp
		* @param size cv::Size representing bounds within which to clamp rectangle
		*
		* @return cv::Rect representing clamped rectangle
		*/
		cv::Rect make_clamped_rect(
			const cv::Rect &rect, 
			const cv::Size &size
		) {

			const int x = std::clamp(
				rect.x, 
				0, 
				std::max(0, size.width - 1)
			);
			const int y = std::clamp(
				rect.y, 
				0, 
				std::max(0, size.height - 1)
			);
			const int right = std::clamp(
				rect.x + rect.width, 
				0, 
				size.width
			);
			const int bottom = std::clamp(
				rect.y + rect.height, 
				0, 
				size.height
			);

			return cv::Rect(
				x, 
				y, 
				std::max(0, right - x), 
				std::max(0, bottom - y)
			);
			
		};


		/**
		* @brief Utility func to convert YOLOX formatted bbox to OpenCV Rect, 
		* 		 with clamping to ensure it fits within image bounds.
		*
		* @param box YoloBoundingBox representing detected object in YOLOX format
		* @param size cv::Size representing dimensions of image (used for scaling and clamping)
		* 
		* @return cv::Rect representing bbox in OpenCV format, clamped to image bounds
		*/
		cv::Rect yolo_to_rect(
			const YoloBoundingBox &box, 
			const cv::Size &size
		) {

			const float cx = box.center_x * static_cast<float>(size.width);
			const float cy = box.center_y * static_cast<float>(size.height);
			const float width = box.width * static_cast<float>(size.width);
			const float height = box.height * static_cast<float>(size.height);
			
			const cv::Rect rect(
				static_cast<int>(std::lround(cx - width * 0.5F)),
				static_cast<int>(std::lround(cy - height * 0.5F)),
				std::max(1, static_cast<int>(std::lround(width))),
				std::max(1, static_cast<int>(std::lround(height)))
			);

			return make_clamped_rect(rect, size);

		};
		
		
		/**
		* @brief Utility func to draw vehicle bboxes
		*
		* @param frame cv::Mat representing image on which to draw (modified in-place)
		* @param bounding_boxes vector of YoloBoundingBox representing detected objects to draw
		*/
		void draw_detection_boxes(
			cv::Mat &frame, 
			const std::vector<YoloBoundingBox> &bounding_boxes
		) {
			
			if (bounding_boxes.empty()) return;

			cv::Mat overlay = frame.clone();
			for (const auto &box : bounding_boxes) {
				const cv::Rect rect = yolo_to_rect(box, frame.size());
				if (rect.width <= 0 || rect.height <= 0) continue;
				cv::rectangle(
					overlay, 
					rect, 
					class_color(box.class_id), 
					cv::FILLED
				);
			}
			
			cv::addWeighted(
				overlay, 
				kDetectionOverlayAlpha, 
				frame, 
				1.0F - kDetectionOverlayAlpha, 
				0.0, 
				frame
			);

		};


		/**
		* @brief Utility func to format float value to string (used for right panel info display)
		*
		* @param value float value to format
		* @param precision int number of decimal places to include
		*
		* @return std::string representing formatted float value
		*/
		std::string format_float(
			float value, 
			int precision
		) {

			std::ostringstream stream;
			stream << std::fixed << std::setprecision(precision) << value;
			
			return stream.str();

		};


		/**
		* @brief Utility func to build a polygon representing drivable path body
		*		 based on the centerline waypoints.
		*
		* @param centerline vector of cv::Point2f representing centerline waypoints of the path
		* @param size cv::Size representing dimensions of the frame (used to determine path width based on distance from bottom)
		*
		* @return vector of cv::Point representing vertices of the path polygon to be drawn
		*/
		std::vector<cv::Point> build_path_polygon(
			const std::vector<cv::Point2f> &centerline, 
			const cv::Size &size
		) {

			std::vector<cv::Point> left_side;
			std::vector<cv::Point> right_side;
			left_side.reserve(centerline.size());
			right_side.reserve(centerline.size());

			for (const auto &current : centerline) {
				const float y_ratio = std::clamp(
					current.y / std::max(1.0F, static_cast<float>(size.height - 1)), 
					0.0F, 
					1.0F
				);
				
				// Total width is 1/4 at y=1.0 and 1/16 at y=0.5
				// std::max ensures path converges to a point at horizon without inverting
				const float half_width_ratio = std::max(0.0F, 0.1875F * y_ratio - 0.0625F);
				const float half_width = size.width * half_width_ratio;

				// Expand purely horizontally (parallel to the bottom edge)
				left_side.emplace_back(cv::Point(
					static_cast<int>(std::lround(current.x - half_width)),
					static_cast<int>(std::lround(current.y))
				));
				right_side.emplace_back(cv::Point(
					static_cast<int>(std::lround(current.x + half_width)),
					static_cast<int>(std::lround(current.y))
				));
			}

			std::vector<cv::Point> polygon;
			polygon.reserve(left_side.size() + right_side.size());
			polygon.insert(polygon.end(), left_side.begin(), left_side.end());
			for (auto it = right_side.rbegin(); it != right_side.rend(); ++it) {
				polygon.push_back(*it);
			}

			return polygon;

		};


		/**
		* @brief Utility func to draw drivable path polygon and centerline
				 based on tracked waypoints and acceleration.
		*
		* @param frame cv::Mat representing image on which to draw (modified in-place)
		* @param tracked_waypoints vector of cv::Point2f representing tracked waypoints of path in image coordinates
		* @param acceleration float representing desired acceleration, used to determine path color: 
		* 					  - green for positive/zero acceleration
		*					  - red for negative acceleration
		*/
		void draw_main_drivable_path(
			cv::Mat &frame, 
			const std::vector<cv::Point2f> &tracked_waypoints, 
			float acceleration
		) {
			
			if (tracked_waypoints.size() < 2) {
				return;
			}

			std::vector<cv::Point2f> projected_points;
			projected_points.reserve(tracked_waypoints.size());
			for (const auto &waypoint : tracked_waypoints) {
				projected_points.emplace_back(
					std::clamp(
						waypoint.x, 
						0.0F, 
						static_cast<float>(frame.cols - 1)
					),
					std::clamp(
						waypoint.y, 
						0.0F, 
						static_cast<float>(frame.rows - 1)
					)
				);
			}

			if (projected_points.size() < 2) {
				return;
			}

			const cv::Scalar path_color = acceleration >= 0.0F ? kPositiveAccelerationColor : kNegativeAccelerationColor;
			const std::vector<cv::Point> polygon = build_path_polygon(
				projected_points, 
				frame.size()
			);
			if (polygon.size() < 3) {
				return;
			}

			cv::Mat overlay = frame.clone();
			std::vector<std::vector<cv::Point>> polygons{polygon};
			cv::fillPoly(
				overlay, 
				polygons, 
				path_color
			);
			cv::addWeighted(
				overlay, 
				kDrivablePathAlpha, 
				frame, 
				1.0F - kDrivablePathAlpha, 
				0.0, 
				frame
			);
			
			std::vector<cv::Point> centerline_points;
			centerline_points.reserve(projected_points.size());
			for (const auto &point : projected_points) {
				centerline_points.emplace_back(cv::Point(
					static_cast<int>(std::lround(point.x)),
					static_cast<int>(std::lround(point.y))
				));
			}
			cv::polylines(
				frame, 
				std::vector<std::vector<cv::Point>>{centerline_points}, 
				false, 
				kWhiteColor, 
				kThickPolyline,
				cv::LINE_AA
			);

		};


		/**
		* @brief Utility func to calculate total length of a polyline defined by a sequence of points
		*
		* @param points vector of cv::Point2f representing vertices of polyline
		*
		* @return float representing total length of polyline
		*/
		float polyline_length(
			const std::vector<cv::Point2f> &points
		) {

			float total = 0.0F;

			for (std::size_t index = 1; index < points.size(); ++index) {
				const cv::Point2f delta = points[index] - points[index - 1];
				total += std::sqrt(delta.x * delta.x + delta.y * delta.y);
			}

			return total;

		};

		/**
		* @brief Utility func to find a point along a polyline at a specified ratio of total length
		*
		* @param points vector of cv::Point2f representing vertices of polyline
		* @param target_ratio float in [0, 1] representing desired position along polyline (0 = start, 1 = end)
		*
		* @return cv::Point2f representing coordinates of point along polyline corresponding to target_ratio
		*/
		cv::Point2f point_along_polyline(
			const std::vector<cv::Point2f> &points, 
			float target_ratio
		) {

			if (points.empty()) {
				return cv::Point2f();
			}

			if (points.size() == 1) {
				return points.front();
			}

			const float clamped_ratio = std::clamp(
				target_ratio, 
				0.0F, 
				1.0F
			);
			const float total_length = polyline_length(points);
			if (total_length <= 1e-4F) {
				return points.front();
			}

			const float target_length = total_length * clamped_ratio;
			float accumulated = 0.0F;
			
			for (std::size_t index = 1; index < points.size(); ++index) {
				const cv::Point2f delta = points[index] - points[index - 1];
				const float segment_length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
				if (accumulated + segment_length >= target_length) {
					const float segment_ratio = (target_length - accumulated) / std::max(segment_length, 1e-4F);
					
					return points[index - 1] + delta * segment_ratio;
				}
				accumulated += segment_length;
			}

			return points.back();

		};


		/**
		* @brief Utility func to draw path preview ruler on right side of frame,
				 indicating distance scale for drivable path
		*
		* @param canvas cv::Mat representing image on which to draw (modified in-place)
		* @param area cv::Rect representing bounding box within which to draw ruler (typically rightmost portion of frame)
		* @param max_distance_m float representing maximum distance in meters to be indicated by ruler (used for labeling tick marks)
		*/
		void draw_path_preview_ruler(
			cv::Mat &canvas, 
			const cv::Rect &area, 
			float max_distance_m
		) {

			const int kRulerWidth = 62;
			const cv::Rect ruler_rect(
				area.x + area.width - kRulerWidth, 
				area.y, 
				kRulerWidth, 
				area.height
			);
			cv::rectangle(
				canvas, 
				ruler_rect,
				kPanelBackgroundColor, 
				cv::FILLED
			);
			cv::line(
				canvas, 
				cv::Point(
					ruler_rect.x, 
					ruler_rect.y
				), 
				cv::Point(
					ruler_rect.x, 
					ruler_rect.y + ruler_rect.height
				), 
				kRulerLineColor,
				kThickRulerLine
			);

			for (int tick = 0; tick <= 10; ++tick) {

				// Calculate tick position
				const float ratio = static_cast<float>(tick) / 10.0F;
				const int y = ruler_rect.y + ruler_rect.height - static_cast<int>(std::lround(ratio * static_cast<float>(ruler_rect.height)));
				const int tick_length = (tick % 5 == 0) ? 16 : 9;

				// Draw tick mark
				cv::line(
					canvas, 
					cv::Point(ruler_rect.x, y), 
					cv::Point(ruler_rect.x + tick_length, y), 
					kRulerTickColor, 
					kThickNormal
				);

				if (tick % 2 == 0) {
					const int distance = static_cast<int>(std::lround(ratio * max_distance_m));
					cv::putText(
						canvas, 
						std::to_string(distance) + "m", 
						cv::Point(
							ruler_rect.x + tick_length + 4, 
							y + 5
						), 
						cv::FONT_HERSHEY_SIMPLEX, 
						kFontSizeRuler, 
						kPanelTextColor, 
						kThickNormal, 
						cv::LINE_AA
					);
				}
			}

		};


		/**
		* @brief Utility func to draw path preview on right side of frame,
		*		 showing a top-down BEV representation of the drivable path
		*		 based on tracked waypoints.
		*
		* @param canvas cv::Mat representing image on which to draw (modified in-place)
		* @param lane_shape LaneShapeVisualization containing fused polynomial coefficients and CIPO info
		* @param area cv::Rect representing bounding box within which to draw path preview (typically rightmost portion of frame, excluding ruler area)
		* @param max_distance_m float representing maximum distance in meters for CIPO marker scaling
		*/
		void draw_path_preview(
			cv::Mat &canvas, 
			const LaneShapeVisualization &lane_shape, 
			const cv::Rect &area
		) {

			// 1. Determine exactly where path should end (CIPO distance or max distance)
            float clip_distance_m = kPathPreviewMaxDistanceMeters;
            if (lane_shape.has_cipo_object && lane_shape.distance_to_cipo.has_value()) {
                clip_distance_m = std::min(
					clip_distance_m, 
					*lane_shape.distance_to_cipo
				);
            }

            const cv::Rect path_rect(
                area.x + 8, 
                area.y + 8, 
                std::max(1, area.width - kRulerWidth - 16), 
                std::max(1, area.height - 16)
            );

			std::vector<cv::Point> polyline_points;

            // 2. Generate BEV points directly from lane shape polynomial coefficients
            // Step forward by 1.0 meter intervals from 0m to clip distance
            for (float x = 0.0F; x <= clip_distance_m; x += 1.0F) {
                
				// y = ax^2 + bx + c
                float y = lane_shape.path_a * x * x + lane_shape.path_b * x + lane_shape.path_c;

                const float x_ratio = std::clamp(
					x / kPathPreviewMaxDistanceMeters, 
					0.0F, 
					1.0F
				);
                // +Y is Left from fusion module. Map +Y toward 0.0 (left edge of the panel area)
                const float y_ratio = std::clamp(
					(kBEVMaxLateralMeters - y) / (2.0F * kBEVMaxLateralMeters), 
					0.0F, 
					1.0F
				);

                const int px = path_rect.x + static_cast<int>(std::lround(y_ratio * static_cast<float>(path_rect.width)));
                const int py = path_rect.y + static_cast<int>(std::lround((1.0F - x_ratio) * static_cast<float>(path_rect.height)));
                
				polyline_points.emplace_back(px, py);

            }

            if (polyline_points.size() < 2) return;

			// 3. Draw path preview polygon with translucent overlay
			
			cv::Mat overlay = canvas.clone();

            cv::polylines(
                overlay, 
                std::vector<std::vector<cv::Point>>{polyline_points}, 
                false, 
                kPositiveAccelerationColor, 
                kThickUltra, 
                cv::LINE_AA
            );
            cv::addWeighted(
                overlay, 
                1.0F - kRightPanelAlpha, 
                canvas, 
                kRightPanelAlpha, 
                0.0, 
                canvas
            );

		};

		
		/**
		* @brief Utility func to draw CIPO marker on path preview, indicating CIPO's relative position and distance
		*
		* @param canvas cv::Mat representing image on which to draw (modified in-place)
		* @param lane_shape LaneShapeVisualization containing CIPO distance, velocity, and polynomial info
		* @param area cv::Rect representing bounding box within which to draw CIPO marker (typically rightmost portion of frame, excluding ruler area)
		* @param max_distance_m float representing maximum distance in meters for CIPO marker scaling (should match max distance used for path preview)
		*/
		void draw_cipo_marker(
			cv::Mat &canvas, 
			const LaneShapeVisualization &lane_shape, 
			const cv::Rect &area, 
			float max_distance_m
		) {
			
			if (!lane_shape.has_cipo_object || !lane_shape.distance_to_cipo.has_value()) return;

			const float distance_m = std::clamp(
				*lane_shape.distance_to_cipo, 
				0.0F, 
				max_distance_m
			);

			const float lateral_y = lane_shape.path_a * distance_m * distance_m + 
									lane_shape.path_b * distance_m + 
									lane_shape.path_c;

			const cv::Rect path_rect(
				area.x + 8, 
				area.y + 8, 
				std::max(1, area.width - kRulerWidth - 16), 
				std::max(1, area.height - 16)
			);

			const float x_ratio = std::clamp(
				distance_m / max_distance_m, 
				0.0F, 
				1.0F
			);
			const float y_ratio = std::clamp(
				(lateral_y + kBEVMaxLateralMeters) / (2.0F * kBEVMaxLateralMeters), 
				0.0F, 
				1.0F
			);

			const int px = path_rect.x + static_cast<int>(std::lround(y_ratio * static_cast<float>(path_rect.width)));
			const int py = path_rect.y + static_cast<int>(std::lround((1.0F - x_ratio) * static_cast<float>(path_rect.height)));

			const cv::Rect marker_rect(
				px - 12, 
				py - 16, 
				24, 
				20
			);
			cv::rectangle(
				canvas, 
				marker_rect, 
				kCipoColor, 
				cv::FILLED
			);
			cv::rectangle(
				canvas, 
				marker_rect, 
				kWhiteColor, 
				kThickPolyline
			);

			const std::string distance_text = format_float(*lane_shape.distance_to_cipo, 1) + " m";
			const std::string velocity_text = lane_shape.relative_cipo_velocity.has_value() ? format_float(*lane_shape.relative_cipo_velocity, 1) + " km/h" : "-- km/h";

			// Calculate text bounds to left-align correctly
			int baseline = 0;
			const cv::Size dist_size = cv::getTextSize(
				distance_text, 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSizeRuler, 
				kThickNormal, 
				&baseline
			);
			const cv::Size vel_size = cv::getTextSize(
				velocity_text, 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSizeRuler, 
				kThickNormal, 
				&baseline
			);

			// 8 pixels padding from the left edge of the box
			const int text_x_dist = marker_rect.x - 8 - dist_size.width;
			const int text_x_vel = marker_rect.x - 8 - vel_size.width;

			cv::putText(
				canvas, 
				distance_text, 
				cv::Point(
					text_x_dist, 
					marker_rect.y + 6
				), 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSizeRuler, 
				kPanelTextColor, 
				kThickNormal, 
				cv::LINE_AA
			);
			cv::putText(
				canvas, 
				velocity_text, 
				cv::Point(
					text_x_vel, 
					marker_rect.y + 20
				), 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSizeRuler, 
				kPanelTextColor, 
				kThickNormal, 
				cv::LINE_AA
			);
		};


		/**
		* @brief Main func to draw right-side info panel, including desired control values and path preview
		*
		* @param canvas cv::Mat representing image on which to draw (modified in-place)
		* @param tracked_waypoints vector of cv::Point2f representing tracked waypoints of path in image coordinates
		* @param lane_shape LaneShapeVisualization containing CIPO distance and relative velocity info (used for CIPO marker in path preview)
		* @param desired_control DesiredControlVisualization containing desired velocity, steering angle, and acceleration (used for text and steering wheel icon in panel)
		*/
		void draw_right_panel(
			cv::Mat &canvas, 
			const LaneShapeVisualization &lane_shape, 
			const DesiredControlVisualization &desired_control
		) {
			
			const int panel_width = kVisualizationPanelWidth;
			const cv::Rect panel_rect(
				canvas.cols - panel_width, 
				0, 
				panel_width, 
				canvas.rows
			);
			if (panel_rect.x < 0) return;

			cv::Mat panel = canvas(panel_rect);
			cv::Mat white_bg(
				panel.size(), 
				panel.type(), 
				kWhiteColor
			);
			cv::addWeighted(
				white_bg, 
				kRightPanelAlpha, 
				panel, 
				1.0F - kRightPanelAlpha, 
				0.0, 
				panel
			);

			// Desired planning values title
			draw_text_centered(
				panel, 
				kTitleDesiredPlanningValue, 
				cv::Rect(
					12, 
					20, 
					panel_rect.width - 24, 
					30
				), 
				kFontSizeRightPanelDesired, 
				kYellowColor, 
				kThickBold
			);

			// Draw steering wheel
			const cv::Rect wheel_area(24, 60, 96, 96);
			const cv::Mat wheel_icon = load_wheel_icon();

			if (!wheel_icon.empty()) {

				cv::Mat rotated = rotate_icon(
					wheel_icon, 
					desired_control.steering_angle
				);
				const float max_width = static_cast<float>(wheel_area.width);
				const float max_height = static_cast<float>(wheel_area.height);
				const float scale = std::min(max_width / std::max(1, rotated.cols), max_height / std::max(1, rotated.rows));
				if (scale < 1.0F) {
					cv::resize(
						rotated, 
						rotated, 
						cv::Size(), 
						scale, 
						scale, 
						cv::INTER_AREA
					);
				}
				overlay_icon(
					panel, 
					rotated, 
					cv::Point(
						wheel_area.x + (wheel_area.width - rotated.cols) / 2, 
						wheel_area.y + (wheel_area.height - rotated.rows) / 2
					)
				);

			} else {
				cv::circle(
					panel, 
					cv::Point(
						wheel_area.x + wheel_area.width / 2, 
						wheel_area.y + wheel_area.height / 2
					), 
					kSizeWheelNonAva, 
					kBlackColor, 
					kThickSuper
				);
			}

			// Inline telemetry texts

			int text_x = 130;
			draw_inline_value(
				panel, 
				cv::Point(text_x, 86), 
				kTelemetryLabelVelocity, 
				format_float(desired_control.velocity, 1) + " " + kTelemetryUnitVelocity, 
				kYellowColor
			);

			draw_inline_value(
				panel, 
				cv::Point(text_x, 116), 
				kTelemetryLabelSteering, 
				format_float(desired_control.steering_angle, 1) + " " + kTelemetryUnitSteering, 
				kYellowColor
			); 

			const cv::Scalar accel_color = desired_control.acceleration >= 0.0F ? kAcceleratingColor : kDeceleratingColor;
			draw_inline_value(
				panel, 
				cv::Point(text_x, 146), 
				kTelemetryLabelAcceleration, 
				format_float(desired_control.acceleration, 1) + " " + kTelemetryUnitAcceleration, 
				accel_color
			);

			// Path preview title
			const cv::Rect bev_rect(
				12, 
				214, 
				panel_rect.width - 24, 
				panel_rect.height - 226
			);
			cv::putText(
				panel, 
				kTitlePathPreview, 
				cv::Point(
					bev_rect.x + 10, 
					bev_rect.y + 22
				), 
				cv::FONT_HERSHEY_SIMPLEX, 
				kFontSize, 
				kYellowColor, 
				kThickBold, 
				cv::LINE_AA
			);

			const cv::Rect path_area(
				bev_rect.x + 10, 
				bev_rect.y + 32, 
				bev_rect.width - 20, 
				bev_rect.height - 42
			);
			draw_path_preview_ruler(
				panel, 
				path_area, 
				kPathPreviewMaxDistanceMeters
			);
			draw_path_preview(
                panel, 
                lane_shape, 
                path_area
            );
            draw_cipo_marker(
                panel, 
				lane_shape, 
                path_area, 
                kPathPreviewMaxDistanceMeters
            );

		};


		/**
		* @brief Main func to draw all overlay elements on the main frame, including detection boxes, drivable path
		*
		* @param frame cv::Mat representing image on which to draw (modified in-place)
		* @param bounding_boxes vector of YoloBoundingBox representing detected objects to draw as bounding boxes
		* @param lane_shape LaneShapeVisualization containing tracked waypoints and CIPO info for drivable path visualization
		* @param desired_control DesiredControlVisualization containing desired velocity, steering angle, and acceleration for drivable path color
		*/
		void draw_main_overlay(
			cv::Mat &frame, 
			const std::vector<YoloBoundingBox> &bounding_boxes, 
			const LaneShapeVisualization &lane_shape, 
			const DesiredControlVisualization &desired_control
		) {

			draw_detection_boxes(
				frame, 
				bounding_boxes
			);
			draw_main_drivable_path(
				frame, 
				lane_shape.tracked_waypoints, 
				desired_control.acceleration
			);

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
	};


	/**
	* @brief Master func to draw the whole visualization thing
	*
	* @param frame cv::Mat representing input image frame from camera
	* @param bounding_boxes vector of YoloBoundingBox representing detected objects to draw as bounding boxes
	* @param lane_shape LaneShapeVisualization containing tracked waypoints and CIPO info for drivable path visualization
	* @param desired_control DesiredControlVisualization containing desired velocity, steering angle, and acceleration
	*
	* @return cv::Mat containing the final visualization with all overlays drawn, ready for display
	*/
	cv::Mat visualize_frame(
		const cv::Mat &frame,
		const std::vector<YoloBoundingBox> &bounding_boxes,
		const LaneShapeVisualization &lane_shape,
		const DesiredControlVisualization &desired_control
	) {
		
		if (frame.empty()) return cv::Mat();

		// Don't expand the frame, overlay directly on top
		cv::Mat output = frame.clone();

		draw_main_overlay(
			output, 
			bounding_boxes, 
			lane_shape, 
			desired_control
		);

		draw_right_panel(
			output, 
			lane_shape, 
			desired_control
		);

		return output;

	};


	void close_windows() {
		cv::destroyAllWindows();
	}

}  // namespace visualization
