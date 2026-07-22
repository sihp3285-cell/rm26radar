#pragma once

#include "position_prior/prior_types.hpp"

#include <cstddef>
#include <cstdint>

namespace position_prior {

struct ObservationConfirmationConfig {
    std::size_t required_count = 3;
    double cluster_radius_m = 0.8;
    double maximum_gap_s = 0.35;
};

// 将连续、空间一致的真实观测确认成可靠锚点。单帧跳变只会开启新的
// 待确认聚类，不会立即污染 position prior 的最后可靠位置。
class ObservationConfirmation {
public:
    explicit ObservationConfirmation(ObservationConfirmationConfig config = {});

    bool observe(const Point2d& world_position, std::int64_t timestamp_ns);
    void reset();

    std::size_t pending_count() const { return pending_count_; }
    bool stream_confirmed() const { return stream_confirmed_; }

private:
    ObservationConfirmationConfig config_;
    bool stream_confirmed_ = false;
    std::int64_t last_sample_ns_ = 0;
    Point2d last_accepted_;
    Point2d pending_center_;
    std::size_t pending_count_ = 0;
};

}  // namespace position_prior
