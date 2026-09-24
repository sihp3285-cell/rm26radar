#pragma once

#include <chrono>
#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace radar27_detection::input
{

struct Frame
{
    // Pixels stay valid while consumers hold this Mat; all consumers read only.
    cv::Mat image;
    std::uint64_t sequence = 0;
    // Source timeline, not a ROS wall-clock timestamp. Video starts at zero.
    std::int64_t source_time_ns = 0;
    std::chrono::steady_clock::time_point received_at{};
};

}  // namespace radar27_detection::input
