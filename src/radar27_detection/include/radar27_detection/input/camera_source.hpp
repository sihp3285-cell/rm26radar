#pragma once

#include <memory>
#include <string>
#include "radar27_detection/input/frame_source.hpp"

namespace radar27_detection::input
{

struct CameraSourceConfig
{
    std::string brand = "hik";
    std::string serial_number;
    bool auto_white_balance = true;
    int exposure_time_us = 6000;
    double gain = 0.7;  // Fraction of maximum gain, [0, 1], clamped to device minimum.
    double gamma = 0.3;
    unsigned int timeout_ms = 1000;
    bool flip = false;
    bool mirror = false;
};

// Single-threaded source; only one CameraSource may be open per process.
// Stop/join the reader before closing this object.
// Frames are owned BGR8 images. Timestamps are host monotonic acquisition
// times relative to open(), not hardware exposure timestamps or ROS time.
class CameraSource final : public FrameSource
{
public:
    explicit CameraSource(CameraSourceConfig config);
    ~CameraSource() override;
    CameraSource(const CameraSource&) = delete;
    CameraSource& operator=(const CameraSource&) = delete;

    void open() override;
    ReadResult read() override;
    void close() noexcept override;

private:
    struct Impl;
    CameraSourceConfig config_;
    std::unique_ptr<Impl> impl_;
    std::uint64_t next_sequence_ = 0;
    std::chrono::steady_clock::time_point started_at_{};
};

}  // namespace radar27_detection::input
