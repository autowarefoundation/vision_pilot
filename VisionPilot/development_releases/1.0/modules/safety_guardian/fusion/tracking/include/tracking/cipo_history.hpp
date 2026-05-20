#pragma once

#include <chrono>
#include <deque>
#include <opencv2/opencv.hpp>

namespace visionpilot::tracking {

struct CIPOSnapshot {
    int   track_id;
    int   class_id;
    cv::Rect bbox;
    float distance_m;
    float velocity_ms;
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat frame_crop;  // for ORB identity verification
};

// Fixed-size sliding window of CIPO observations (FIFO when full).
class CIPOHistory {
public:
    explicit CIPOHistory(std::size_t max_size = 30);

    void push(const CIPOSnapshot& snap);

    const CIPOSnapshot* latest()   const;  // most recent, or nullptr
    const CIPOSnapshot* previous() const;  // one frame before latest, or nullptr
    bool  did_change() const;              // true if track_id changed between last two
    void  clear();
    bool  empty() const { return history_.empty(); }
    std::size_t size() const { return history_.size(); }

private:
    std::deque<CIPOSnapshot> history_;
    std::size_t max_size_;
};

}  // namespace visionpilot::tracking
