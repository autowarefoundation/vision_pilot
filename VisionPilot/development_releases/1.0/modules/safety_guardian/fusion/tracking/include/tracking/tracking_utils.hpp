#pragma once

#include <opencv2/opencv.hpp>

namespace visionpilot::tracking {

// Static utility functions for bbox data association.
struct TrackingUtils {
    static float       calculateIoU(const cv::Rect& a, const cv::Rect& b);
    static float       calculateCentroidDistance(const cv::Rect& a, const cv::Rect& b);
    static float       calculateSizeSimilarity(const cv::Rect& a, const cv::Rect& b);
    static cv::Point2f getCentroid(const cv::Rect& bbox);
    static cv::Point2f getBottomCenter(const cv::Rect& bbox);

    // Weighted composite score: 0.5*IoU + 0.3*(1-norm_centroid_dist) + 0.2*size_sim
    static float calculateMatchingScore(const cv::Rect& det,
                                        const cv::Rect& track,
                                        int image_width,
                                        int image_height);
};

}  // namespace visionpilot::tracking
