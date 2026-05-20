#include <tracking/feature_matching_utils.hpp>
#include <algorithm>

namespace visionpilot::tracking {

cv::Ptr<cv::ORB> FeatureMatchingUtils::getORB()
{
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(
        500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    return orb;
}

cv::Ptr<cv::BFMatcher> FeatureMatchingUtils::getBFMatcher()
{
    static cv::Ptr<cv::BFMatcher> m = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    return m;
}

cv::Mat FeatureMatchingUtils::extractSafeCrop(const cv::Mat& frame, const cv::Rect& bbox)
{
    cv::Rect safe = bbox & cv::Rect(0, 0, frame.cols, frame.rows);
    return (safe.area() == 0) ? cv::Mat{} : frame(safe).clone();
}

bool FeatureMatchingUtils::extractFeatures(const cv::Mat& frame,
                                            const cv::Rect& bbox,
                                            std::vector<cv::KeyPoint>& kps,
                                            cv::Mat& desc)
{
    cv::Mat crop = extractSafeCrop(frame, bbox);
    if (crop.empty() || crop.cols < 20 || crop.rows < 20) return false;

    cv::Mat gray;
    if (crop.channels() == 3) cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
    else                       gray = crop;

    kps.clear(); desc = cv::Mat{};
    getORB()->detectAndCompute(gray, cv::noArray(), kps, desc);
    return static_cast<int>(kps.size()) >= 10;
}

int FeatureMatchingUtils::matchFeatures(const cv::Mat& d1,
                                         const cv::Mat& d2,
                                         std::vector<cv::DMatch>& good)
{
    good.clear();
    if (d1.empty() || d2.empty()) return 0;

    std::vector<std::vector<cv::DMatch>> knn;
    try { getBFMatcher()->knnMatch(d1, d2, knn, 2); }
    catch (...) { return 0; }

    for (const auto& m : knn) {
        if (m.size() >= 2 && m[0].distance < LOWE_RATIO * m[1].distance)
            good.push_back(m[0]);
    }
    return static_cast<int>(good.size());
}

float FeatureMatchingUtils::calculateMatchConfidence(
    const cv::Mat& d1, const cv::Mat& d2, int n1, int n2)
{
    std::vector<cv::DMatch> good;
    int nm = matchFeatures(d1, d2, good);
    if (nm == 0 || n1 == 0 || n2 == 0) return 0.f;
    float avg = (n1 + n2) * 0.5f;
    return std::min(static_cast<float>(nm) / avg, 1.f);
}

bool FeatureMatchingUtils::areSameObject(const cv::Mat& c1, const cv::Rect& r1,
                                          const cv::Mat& c2, const cv::Rect& r2,
                                          float threshold)
{
    std::vector<cv::KeyPoint> k1, k2;
    cv::Mat d1, d2;
    if (!extractFeatures(c1, r1, k1, d1) || !extractFeatures(c2, r2, k2, d2)) return false;
    float conf = calculateMatchConfidence(d1, d2,
                     static_cast<int>(k1.size()), static_cast<int>(k2.size()));
    return conf >= threshold;
}

}  // namespace visionpilot::tracking
