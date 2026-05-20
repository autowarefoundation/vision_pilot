#include <tracking/tracking_utils.hpp>
#include <algorithm>
#include <cmath>

namespace visionpilot::tracking {

float TrackingUtils::calculateIoU(const cv::Rect& a, const cv::Rect& b)
{
    int x1 = std::max(a.x, b.x), y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width,  b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int uni   = a.area() + b.area() - inter;
    return uni > 0 ? static_cast<float>(inter) / static_cast<float>(uni) : 0.f;
}

float TrackingUtils::calculateCentroidDistance(const cv::Rect& a, const cv::Rect& b)
{
    cv::Point2f ca = getCentroid(a), cb = getCentroid(b);
    float dx = ca.x - cb.x, dy = ca.y - cb.y;
    return std::sqrt(dx * dx + dy * dy);
}

float TrackingUtils::calculateSizeSimilarity(const cv::Rect& a, const cv::Rect& b)
{
    float aa = static_cast<float>(a.area());
    float ab = static_cast<float>(b.area());
    if (aa == 0.f || ab == 0.f) return 0.f;
    return std::min(aa, ab) / std::max(aa, ab);
}

cv::Point2f TrackingUtils::getCentroid(const cv::Rect& bbox)
{
    return {bbox.x + bbox.width * 0.5f, bbox.y + bbox.height * 0.5f};
}

cv::Point2f TrackingUtils::getBottomCenter(const cv::Rect& bbox)
{
    return {bbox.x + bbox.width * 0.5f, static_cast<float>(bbox.y + bbox.height)};
}

float TrackingUtils::calculateMatchingScore(
    const cv::Rect& det, const cv::Rect& track, int iw, int ih)
{
    float iou  = calculateIoU(det, track);
    float dist = calculateCentroidDistance(det, track);
    float size = calculateSizeSimilarity(det, track);
    float diag = std::sqrt(static_cast<float>(iw * iw + ih * ih));
    float norm_dist = 1.f - std::min(dist / diag, 1.f);
    return 0.5f * iou + 0.3f * norm_dist + 0.2f * size;
}

}  // namespace visionpilot::tracking
