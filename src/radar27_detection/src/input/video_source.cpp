#include "radar27_detection/input/video_source.hpp"

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace radar27_detection::input
{

VideoSource::VideoSource(VideoSourceConfig config)
    : config_(std::move(config))
{
}

VideoSource::~VideoSource()
{
    close();
}

void VideoSource::open()
{
    if (capture_.isOpened()) {
        throw std::logic_error("VideoSource is already open");
    }
    if (config_.path.empty() || !std::filesystem::is_regular_file(config_.path)) {
        throw std::invalid_argument("Video path must name a regular file: " + config_.path);
    }
    if (!std::isfinite(config_.fps) || config_.fps < 0.0 || config_.fps > 1e9) {
        throw std::invalid_argument("Video FPS must be finite and in [0, 1e9]");
    }

    try {
        if (!capture_.open(config_.path)) {
            throw std::runtime_error("Failed to open video: " + config_.path);
        }
        const double native_fps = capture_.get(cv::CAP_PROP_FPS);
        fps_ = config_.fps > 0.0 ? config_.fps
            : (std::isfinite(native_fps) && native_fps > 0.0 && native_fps <= 1e9
                ? native_fps : 30.0);
        next_sequence_ = 0;
        ended_ = false;
        read_error_.clear();
    } catch (...) {
        close();
        throw;
    }
}

ReadResult VideoSource::read()
{
    if (!capture_.isOpened()) {
        return {ReadStatus::Error, {}, "VideoSource is not open"};
    }
    if (!read_error_.empty()) {
        return {ReadStatus::Error, {}, read_error_};
    }
    if (ended_) {
        return {ReadStatus::End, {}, {}};
    }

    try {
        // Absolute frame number avoids accumulating rounded-period error at
        // fractional frame rates. This is a constant-rate playback timeline.
        const long double time_ns = std::round(
            static_cast<long double>(next_sequence_) * 1000000000.0L / fps_);
        if (!std::isfinite(time_ns) ||
            time_ns >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
            read_error_ = "Video timeline exceeds int64 nanoseconds";
            return {ReadStatus::Error, {}, read_error_};
        }

        // A fresh Mat prevents later reads from reusing pixels held by consumers.
        cv::Mat image;
        if (!capture_.read(image) || image.empty()) {
            // OpenCV has no universal EOF/error distinction. Frame count is a
            // best-effort early-stop check; with unknown length false means End.
            const double frame_count = capture_.get(cv::CAP_PROP_FRAME_COUNT);
            if (std::isfinite(frame_count) && frame_count > 0.0 &&
                static_cast<long double>(next_sequence_) + 0.5L < frame_count) {
                read_error_ = "Video decoding stopped before the reported frame count";
                return {ReadStatus::Error, {}, read_error_};
            }
            ended_ = true;
            return {ReadStatus::End, {}, {}};
        }

        Frame frame;
        frame.image = std::move(image);
        frame.sequence = next_sequence_++;
        frame.source_time_ns = static_cast<std::int64_t>(time_ns);
        frame.received_at = std::chrono::steady_clock::now();
        return {ReadStatus::Ok, std::move(frame), {}};
    } catch (const cv::Exception& error) {
        read_error_ = error.what();
        return {ReadStatus::Error, {}, read_error_};
    }
}

void VideoSource::close() noexcept
{
    try {
        capture_.release();
    } catch (...) {
        // Destruction must not propagate backend cleanup exceptions.
    }
    fps_ = 0.0;
    next_sequence_ = 0;
    ended_ = false;
    read_error_.clear();
}

}  // namespace radar27_detection::input
