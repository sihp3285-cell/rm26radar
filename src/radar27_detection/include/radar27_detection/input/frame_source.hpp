#pragma once
#include "radar27_detection/input/frame.hpp"
#include <string>

namespace radar27_detection::input
{

enum class ReadStatus
{
    Ok,
    Timeout,
    End,
    Error
};

struct ReadResult
{
    ReadStatus status = ReadStatus::Error;
    Frame frame;
    std::string message;
};

// Call open/read/close sequentially; concurrent calls are not supported.
class FrameSource
{
public:
    virtual ~FrameSource() = default;
    virtual void open() = 0;
    virtual ReadResult read() = 0;
    virtual void close() noexcept = 0;
};

}  // namespace radar27_detection::input
