#include "radar27_detection/recorder.hpp"
#include <opencv2/videoio.hpp>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace radar27_detection
{

Recorder::Recorder(RecorderConfig config) : config_(std::move(config))
{
    if (config_.path.empty() || !std::isfinite(config_.fps) || config_.fps <= 0.0 ||
        config_.queue_size == 0 || config_.codec.size() != 4) {
        throw std::invalid_argument("Invalid recording path/FPS/queue size/codec");
    }
    const auto parent = std::filesystem::path(config_.path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    worker_ = std::thread(&Recorder::run, this);
}

Recorder::~Recorder() { finish(); }

bool Recorder::submit(const cv::Mat& frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || !error_.empty() || frame.empty()) return false;
    if (queue_.size() == config_.queue_size) {
        queue_.pop_front();
        ++dropped_;
    }
    queue_.push_back(frame);
    cv_.notify_one();
    return true;
}

void Recorder::finish()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

std::string Recorder::error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

void Recorder::run()
{
    try {
        cv::VideoWriter writer;
        cv::Size size;
        for (;;) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (queue_.empty()) break;
                frame = std::move(queue_.front());
                queue_.pop_front();
            }
            if (frame.type() != CV_8UC3) throw std::runtime_error("Recorder requires BGR8 frames");
            if (!writer.isOpened()) {
                size = frame.size();
                const auto& codec = config_.codec;
                if (!writer.open(config_.path, cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]),
                                 config_.fps, size)) {
                    throw std::runtime_error("Cannot open recording: " + config_.path);
                }
            }
            if (frame.size() != size) throw std::runtime_error("Recording frame size changed");
            writer.write(frame);
            ++written_;
        }
        writer.release();
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = error.what();
        dropped_ += queue_.size();
        queue_.clear();
        stopping_ = true;
    }
}

}  // namespace radar27_detection
