#include "radar27_detection/input/video_source.hpp"

#include <opencv2/core.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cstdlib>

using namespace radar27_detection::input;

static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    char pattern[] = "/tmp/radar27-video-test-XXXXXX";
    const char* directory = mkdtemp(pattern);
    if (!directory) return 1;
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } cleanup{directory};

    try {
        const auto path = (cleanup.path / "frames.avi").string();
        cv::VideoWriter writer(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               25.0, cv::Size(64, 48));
        require(writer.isOpened(), "fixture writer failed");
        for (int i = 0; i < 5; ++i) {
            writer.write(cv::Mat(48, 64, CV_8UC3, cv::Scalar(i * 40, i * 40, i * 40)));
        }
        writer.release();

        VideoSource source({path, 0.0});
        require(source.read().status == ReadStatus::Error, "read before open");
        source.open();
        require(std::abs(source.fps() - 25.0) < 1e-6, "native FPS");
        bool duplicate_open_rejected = false;
        try { source.open(); } catch (const std::logic_error&) {
            duplicate_open_rejected = true;
        }
        require(duplicate_open_rejected, "duplicate open");

        std::vector<Frame> retained;
        for (std::uint64_t i = 0; i < 5; ++i) {
            const auto before = std::chrono::steady_clock::now();
            auto result = source.read();
            require(result.status == ReadStatus::Ok, "frame read");
            require(result.frame.sequence == i, "sequence");
            require(result.frame.source_time_ns == static_cast<std::int64_t>(i * 40000000),
                    "native timeline");
            require(result.frame.received_at >= before &&
                    result.frame.received_at <= std::chrono::steady_clock::now(), "arrival time");
            retained.push_back(std::move(result.frame));
        }
        require(source.read().status == ReadStatus::End, "EOF");
        require(source.read().status == ReadStatus::End, "stable EOF");
        source.close();
        source.close();
        require(source.fps() == 0.0, "closed FPS");
        require(source.read().status == ReadStatus::Error, "read after close");
        for (std::size_t i = 0; i < retained.size(); ++i) {
            require(std::abs(cv::mean(retained[i].image)[0] - i * 40.0) < 5.0,
                    "retained pixels changed after later reads or close");
        }
        source.open();
        auto first = source.read();
        require(first.status == ReadStatus::Ok && first.frame.sequence == 0 &&
                first.frame.source_time_ns == 0, "reopen resets timeline");
        source.close();

        VideoSource fractional({path, 29.97});
        fractional.open();
        for (int i = 0; i < 5; ++i) {
            auto result = fractional.read();
            require(result.status == ReadStatus::Ok, "fractional frame read");
            require(result.frame.source_time_ns == std::llround(i * 1e9L / 29.97),
                    "fractional timeline");
        }

        for (double fps : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
            VideoSource invalid({path, fps});
            bool rejected = false;
            try { invalid.open(); } catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "invalid FPS accepted");
        }
        VideoSource missing({(cleanup.path / "missing.avi").string(), 0.0});
        bool rejected = false;
        try { missing.open(); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "missing file accepted");
        std::cout << "PASS: video frames, ownership, timestamps, EOF, reopen, invalid inputs\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
