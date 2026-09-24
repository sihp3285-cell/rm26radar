#pragma once

#include <cstdint>
#include <string>

#include <opencv2/videoio.hpp>
#include "radar27_detection/input/frame_source.hpp"

namespace radar27_detection::input
{

struct VideoSourceConfig
{
    std::string path;
    // Zero: file FPS, falling back to 30. Positive: override playback timeline.
    // The caller controls pacing; read() never sleeps to enforce this FPS.
    double fps = 0.0;
};

class VideoSource final : public FrameSource
{
public:
    explicit VideoSource(VideoSourceConfig config);
    ~VideoSource() override;

    VideoSource(const VideoSource&) = delete;
    VideoSource& operator=(const VideoSource&) = delete;

    void open() override;
    ReadResult read() override;
    void close() noexcept override;

    double fps() const noexcept
    {
        return fps_;
    }

private:
    VideoSourceConfig config_;
    cv::VideoCapture capture_;

    double fps_ = 0.0;
    std::uint64_t next_sequence_ = 0;
    bool ended_ = false;
    std::string read_error_;
};

}  // namespace radar27_detection::input
