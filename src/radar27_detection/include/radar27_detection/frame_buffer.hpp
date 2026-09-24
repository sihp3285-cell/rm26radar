#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace radar27_detection
{

// One pending frame, separate from the frame currently being processed.
// Realtime replaces pending data; sequential waits for a free slot.
template<class T>
class FrameBuffer
{
public:
    bool push(T frame, bool sequential)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (sequential) cv_.wait(lock, [this] { return closed_ || !pending_; });
        if (closed_) return false;
        if (pending_) ++overwritten_;
        pending_ = std::move(frame);
        cv_.notify_all();
        return true;
    }

    std::optional<T> pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return closed_ || pending_.has_value(); });
        if (!pending_) return std::nullopt;
        auto frame = std::move(pending_);
        pending_.reset();
        cv_.notify_all();
        return frame;
    }

    // EOF drains the pending frame; cancellation discards it. Both wake all waits.
    void close(bool discard = false)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        if (discard) pending_.reset();
        cv_.notify_all();
    }

    std::uint64_t overwritten() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return overwritten_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<T> pending_;
    bool closed_ = false;
    std::uint64_t overwritten_ = 0;
};

}  // namespace radar27_detection
