#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace visionpilot::tracking {

// ORB-based feature extraction and matching for cut-in vehicle identity verification.
struct FeatureMatchingUtils {
    static bool extractFeatures(const cv::Mat& frame,
                                const cv::Rect& bbox,
                                std::vector<cv::KeyPoint>& kps,
                                cv::Mat& desc);

    static int  matchFeatures(const cv::Mat& desc1,
                               const cv::Mat& desc2,
                               std::vector<cv::DMatch>& good_matches);

    static float calculateMatchConfidence(const cv::Mat& desc1,
                                          const cv::Mat& desc2,
                                          int n_kp1, int n_kp2);

    static cv::Mat extractSafeCrop(const cv::Mat& frame, const cv::Rect& bbox);

    // High-level: returns true if both crops are likely the same vehicle.
    static bool areSameObject(const cv::Mat& crop1,
                               const cv::Rect& roi1,
                               const cv::Mat& crop2,
                               const cv::Rect& roi2,
                               float threshold = 0.3f);

private:
    static cv::Ptr<cv::ORB>       getORB();
    static cv::Ptr<cv::BFMatcher> getBFMatcher();
    static constexpr float LOWE_RATIO = 0.75f;
};

}  // namespace visionpilot::tracking
