//
// Created by atanasko on 27.4.26.
// Developed by TranHuuNhatHuy on 10.6.26.
//

#ifndef VISIONPILOT_VISUALIZATION_HPP
#define VISIONPILOT_VISUALIZATION_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace visualization {

// ================== HARD-CODED VISUALIZATION PARAMS ==================

// Dimensions of major compartments (all in pixels)
inline constexpr int kFrameWidth = 1024;
inline constexpr int kFrameHeight = 512;
inline constexpr int kVisualizationPanelWidth = 256;
inline constexpr int kRulerWidth = 62;

// Sizes
inline const int kSizeWheelNonAva = 34;

// Texts
inline const std::string kTitleDesiredPlanningValue = "Planning values";
inline const std::string kTitlePathPreview = "Path preview";
inline const std::string kTelemetryLabelVelocity = "Velocity";
inline const std::string kTelemetryLabelSteering = "Steering";
inline const std::string kTelemetryLabelAcceleration = "Acceleration";
inline const std::string kTelemetryUnitDistance = "m";
inline const std::string kTelemetryUnitVelocity = "km/h";
inline const std::string kTelemetryUnitSteering = "deg";
inline const std::string kTelemetryUnitAcceleration = "m/s2";

// Background alphas
inline constexpr float kDetectionOverlayAlpha = 0.30F;
inline constexpr float kRightPanelAlpha = 0.30F;
inline constexpr float kDrivablePathAlpha = 0.30F;

// Colors
// 1. Bounding boxes
inline const cv::Scalar kCipoColor(0, 0, 255);
inline const cv::Scalar kCuttingInColor(0, 255, 255);
inline const cv::Scalar kOtherCarsColor(255, 255, 0);
// 2. Drivable path body (accel & decel)
inline const cv::Scalar kPositiveAccelerationColor(0, 200, 0);
inline const cv::Scalar kNegativeAccelerationColor(0, 0, 255);
inline const cv::Scalar kNeutralAccelerationColor(255, 0, 0);
// 3. Others
inline const cv::Scalar kWhiteColor(255, 255, 255);
inline const cv::Scalar kBlackColor(0, 0, 0);
inline const cv::Scalar kYellowColor(0, 255, 255);
inline const cv::Scalar kGrayColor(180, 180, 180);
inline const cv::Scalar kPanelTextColor(255, 255, 255);
inline const cv::Scalar kPanelBackgroundColor(0, 0, 0);
inline const cv::Scalar kBEVDrivablePathColor(40, 180, 90);
inline const cv::Scalar kRulerLineColor(170, 170, 170);
inline const cv::Scalar kRulerTickColor(120, 120, 120);
inline const cv::Scalar kAcceleratingColor(0, 252, 124);
inline const cv::Scalar kDeceleratingColor(114, 128, 250);

// Font sizes and thickness
inline const int kThickBold = 2;
inline const int kThickNormal = 1;
inline const double kFontSize = 0.58;
inline const double kFontSizeRightPanelDesired = 0.58;
inline const double kFontSizeRuler = 0.42;
inline const double kFontSizeTelemetry = 0.48;

// Drawing params
inline const int kThickUltra = 4;
inline const int kThickSuper = 3;
inline const int kThickPolyline = 2;
inline const int kThickRulerLine = 1;

// BEV max render distance (m)
inline constexpr float kPathPreviewMaxDistanceMeters = 50.0F;
inline constexpr float kBEVMaxLateralMeters = 15.0F;


// ======================= VISUALIZATION UTILS =======================

// YOLOX formatted bounding box struct
struct YoloBoundingBox {
    int class_id = 0;
    float center_x = 0.0F;
    float center_y = 0.0F;
    float width = 0.0F;
    float height = 0.0F;
};

// Lane shape upstream data struct for vis
struct LaneShapeVisualization {
    bool has_cipo_object = false;
    std::optional<float> distance_to_cipo;
    std::optional<float> relative_cipo_velocity;
    
    // Normal camera image coordinates used strictly for the front projection overlay
    std::vector<cv::Point2f> tracked_waypoints;

    // Fused polynomial coefficients (y = ax^2 + bx + c) provided directly by LateralFusion
    float path_a = 0.0F;
    float path_b = 0.0F;
    float path_c = 0.0F;
};

// Desired planning control upstream data struct for vis
struct DesiredControlVisualization {
    float steering_angle = 0.0F;
    float velocity = 0.0F;
    float acceleration = 0.0F;
};

// Main vis drawing function
cv::Mat visualize_frame(
    const cv::Mat &frame,
    const std::vector<YoloBoundingBox> &bounding_boxes,
    const LaneShapeVisualization &lane_shape,
    const DesiredControlVisualization &desired_control
);


// ====================================================================


bool render_frame(
    const cv::Mat &frame,
    const std::string &window_name = "VisionPilot",
    const std::vector<std::string> &overlay_lines = {}
);

void close_windows();

}  // namespace visualization

#endif //VISIONPILOT_VISUALIZATION_HPP