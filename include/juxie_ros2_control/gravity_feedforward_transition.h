#pragma once

#include <algorithm>

namespace juxie_ros2_control {
// Blend only on topic acquisition. A timeout immediately restores the table.
class GravityFeedforwardTransition {
public:
    void reset() { topic_active_ = false; }

    double topicWeight(bool fresh, double now_s, double duration_s) {
        if (!fresh) {
            reset();
            return 0.0;
        }
        if (!topic_active_) {
            topic_active_ = true;
            started_s_ = now_s;
        }
        if (duration_s <= 0.0) return 1.0;
        const double t = std::clamp((now_s - started_s_) / duration_s, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

private:
    bool topic_active_{false};
    double started_s_{0.0};
};
}  // namespace juxie_ros2_control
