#include "radar27_detection/frame_buffer.hpp"
#include "radar27_detection/recorder.hpp"
#include <opencv2/opencv.hpp>
#include <future>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

using namespace std::chrono_literals;
static void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

int main()
{
    char pattern[] = "/tmp/radar27-buffer-recorder-XXXXXX";
    const auto* directory = mkdtemp(pattern);
    if (!directory) return 1;
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
    } cleanup{directory};
    try {
        radar27_detection::FrameBuffer<int> realtime;
        realtime.push(1, false); realtime.push(2, false); realtime.push(3, false);
        require(realtime.overwritten() == 2, "realtime overwrite count");
        realtime.close();
        require(realtime.pop() == 3 && !realtime.pop(), "EOF must drain latest frame");
        require(!realtime.push(4, false), "push after EOF");

        radar27_detection::FrameBuffer<int> sequential;
        auto producer = std::async(std::launch::async, [&] {
            for (int i = 0; i < 100; ++i) if (!sequential.push(i, true)) return false;
            sequential.close(); return true;
        });
        for (int i = 0; i < 100; ++i) require(sequential.pop() == i, "sequential lost/reordered frame");
        require(!sequential.pop() && producer.get() && sequential.overwritten() == 0, "sequential EOF");

        radar27_detection::FrameBuffer<int> cancellation;
        cancellation.push(1, true);
        auto blocked = std::async(std::launch::async, [&] { return cancellation.push(2, true); });
        const bool initially_blocked = blocked.wait_for(20ms) == std::future_status::timeout;
        cancellation.close(true);
        require(initially_blocked && blocked.wait_for(1s) == std::future_status::ready && !blocked.get(), "cancel producer");
        require(!cancellation.pop(), "cancel must discard pending frame");

        radar27_detection::FrameBuffer<int> empty;
        auto waiting = std::async(std::launch::async, [&] { return empty.pop(); });
        empty.close();
        require(waiting.wait_for(1s) == std::future_status::ready && !waiting.get(), "cancel empty consumer");

        const auto path = (cleanup.path / "recording.avi").string();
        radar27_detection::Recorder recorder({path, 20.0, 2, "MJPG"});
        for (int i = 0; i < 200; ++i) {
            cv::Mat frame(48, 64, CV_8UC3, cv::Scalar(i % 200, i % 200, i % 200));
            require(recorder.submit(frame), "recorder rejected valid frame");
        }
        recorder.finish(); recorder.finish();
        require(recorder.error().empty(), "recording worker error");
        require(recorder.written() + recorder.dropped() == 200, "recording accounting");
        cv::VideoCapture recording(path);
        cv::Mat frame;
        std::uint64_t decoded = 0;
        while (recording.read(frame)) ++decoded;
        require(decoded == recorder.written() && decoded > 0, "recording not flushed or corrupt");
        radar27_detection::Recorder invalid({(cleanup.path / "invalid.avi").string(), 20.0, 2, "MJPG"});
        invalid.submit(cv::Mat(48, 64, CV_8UC1, cv::Scalar(0)));
        invalid.finish();
        require(!invalid.error().empty(), "recording error not surfaced");
        std::cout << "PASS: latest/sequential buffers, drain/cancel, recording flush/accounting/errors\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
