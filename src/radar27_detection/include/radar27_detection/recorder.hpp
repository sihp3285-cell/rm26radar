#pragma once

#include <opencv2/core/mat.hpp>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace radar27_detection
{

struct RecorderConfig
{
    std::string path;
    double fps = 20.0;
    std::size_t queue_size = 8;
    std::string codec = "mp4v";
};

// Shares immutable owned images, writes only on its worker. Full queues discard
// the oldest pending frame. finish() is called only after producers have stopped.
class Recorder
{
public:
    explicit Recorder(RecorderConfig config);
    ~Recorder();
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;
    bool submit(const cv::Mat& frame);
    void finish();
    std::string error() const;
    std::uint64_t dropped() const { return dropped_.load(); }
    std::uint64_t written() const { return written_.load(); }

private:
    void run();
    RecorderConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<cv::Mat> queue_;
    bool stopping_ = false;
    std::string error_;
    std::atomic<std::uint64_t> dropped_{0}, written_{0};
    std::thread worker_;
};

}  // namespace radar27_detection
