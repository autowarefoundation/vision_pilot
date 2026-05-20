#include <tracking/cipo_history.hpp>

namespace visionpilot::tracking {

CIPOHistory::CIPOHistory(std::size_t max_size) : max_size_(max_size) {}

void CIPOHistory::push(const CIPOSnapshot& snap)
{
    history_.push_back(snap);
    if (history_.size() > max_size_) history_.pop_front();
}

const CIPOSnapshot* CIPOHistory::latest() const
{
    return history_.empty() ? nullptr : &history_.back();
}

const CIPOSnapshot* CIPOHistory::previous() const
{
    return history_.size() < 2 ? nullptr : &history_[history_.size() - 2];
}

bool CIPOHistory::did_change() const
{
    const auto* l = latest();
    const auto* p = previous();
    if (!l || !p) return false;
    return l->track_id != p->track_id;
}

void CIPOHistory::clear() { history_.clear(); }

}  // namespace visionpilot::tracking
