/**
 * @file observation_confirmation.hpp
 * @brief 用连续、空间一致的真实检测保护“最后可靠观测”缓存。
 *
 * PositionPriorNode 为每个 slot 保存一个实例。重新出现后的单帧 observed 只建立待
 * 确认簇，不会马上清除旧猜点；连续次数、半径和最大间隔全部通过后才更新锚点。
 */
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
    /** 保存连续确认参数并创建未确认、无历史样本的状态机。 */
    explicit ObservationConfirmation(ObservationConfirmationConfig config = {});

    /** 接收一个真实 world 观测；只有连续且空间一致达到次数要求时返回 true。 */
    bool observe(const Point2d& world_position, std::int64_t timestamp_ns);
    /** 清空已确认标志、时间和待确认聚类；不改变配置。 */
    void reset();

    /** 返回当前待确认空间簇累计的样本数。 */
    std::size_t pending_count() const { return pending_count_; }
    /** 返回当前连续观测流是否已经通过可靠性确认。 */
    bool stream_confirmed() const { return stream_confirmed_; }

private:
    ObservationConfirmationConfig config_;
    bool stream_confirmed_ = false;
    std::int64_t last_sample_ns_ = 0;
    Point2d last_accepted_;
    Point2d pending_center_; // 当前待确认簇的在线均值，坐标为 world(x,z)。
    std::size_t pending_count_ = 0;
};

}  // namespace position_prior
