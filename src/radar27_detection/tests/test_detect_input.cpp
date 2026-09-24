#define RADAR27_DETECT_NO_MAIN
#include "../src/detect_node.cpp"
#include <fstream>
#include <future>
#include <cstdlib>
#include <unistd.h>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
static void require(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }

static void until(rclcpp::Executor& executor, const std::function<bool()>& predicate, int timeout_ms = 5000)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!predicate()) {
        require(std::chrono::steady_clock::now() < deadline, "integration wait timed out");
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
    }
}
static void pump(rclcpp::Executor& executor, int ms)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    until(executor, [&] { return std::chrono::steady_clock::now() >= deadline; }, ms + 2000);
}
template<class Service>
static auto request(rclcpp::Executor& executor, typename rclcpp::Client<Service>::SharedPtr client,
                    typename Service::Request::SharedPtr message)
{
    until(executor, [&] { return client->service_is_ready(); });
    auto future = client->async_send_request(message);
    until(executor, [&] { return future.wait_for(0s) == std::future_status::ready; });
    return future.get();
}
static long value(const std::string& status, const std::string& key)
{
    const auto pos = status.find(key + "=");
    require(pos != std::string::npos, "missing status: " + key);
    return std::stol(status.substr(pos + key.size() + 1));
}
static void makeVideo(const fs::path& path, int frames)
{
    cv::VideoWriter writer(path.string(), cv::VideoWriter::fourcc('M','J','P','G'), 20.0, cv::Size(240,100));
    require(writer.isOpened(), "fixture writer");
    for (int i = 0; i < frames; ++i)
        writer.write(cv::Mat(100, 240, CV_8UC3, cv::Scalar((i % 10) * 20, (i % 10) * 20, (i % 10) * 20)));
}

int main(int argc, char** argv)
{
    char pattern[] = "/tmp/radar27-detect-input-XXXXXX";
    const auto* directory = mkdtemp(pattern);
    if (!directory) return 1;
    struct Cleanup {
        fs::path path;
        ~Cleanup() { std::error_code error; fs::remove_all(path, error); }
    } cleanup{directory};
    setenv("ROS_LOG_DIR", directory, 1);
    const auto domain = std::to_string(100 + getpid() % 80);
    setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    try {
        fs::copy_file(fs::path(RADAR27_TEST_CONFIG_DIR) / "model.yaml", cleanup.path / "model.yaml");
        const auto roi = cleanup.path / "outpost_roi.yaml";
        { std::ofstream out(roi); out << "outpost_enabled: false\noutpost_score_threshold: 0.1\n"; }
        const auto video = cleanup.path / "short.avi";
        makeVideo(video, 10);
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        auto observer = std::make_shared<rclcpp::Node>("input_observer", rclcpp::NodeOptions().use_intra_process_comms(true));
        executor.add_node(observer);
        std::vector<radar27_interfaces::msg::DetectionArray> detections;
        std::vector<radar27_interfaces::msg::PipelineTiming> timings;
        std::vector<sensor_msgs::msg::Image> raw_images;
        int debug_images = 0;
        auto detection_sub = observer->create_subscription<radar27_interfaces::msg::DetectionArray>(
            "/armor_detections", rclcpp::QoS(100).best_effort(),
            [&](const radar27_interfaces::msg::DetectionArray& message) { detections.push_back(message); });
        auto timing_sub = observer->create_subscription<radar27_interfaces::msg::PipelineTiming>(
            "/pipeline_timing", rclcpp::QoS(100),
            [&](const radar27_interfaces::msg::PipelineTiming& message) { timings.push_back(message); });
        auto debug_sub = observer->create_subscription<sensor_msgs::msg::Image>(
            "/detected_image", rclcpp::QoS(1), [&](const sensor_msgs::msg::Image&) { ++debug_images; });
        auto status_client = observer->create_client<std_srvs::srv::Trigger>("/detect_node/input_status");
        auto pause_client = observer->create_client<std_srvs::srv::SetBool>("/video_node/set_pause");
        auto new_pause_client = observer->create_client<std_srvs::srv::SetBool>("/detect_node/set_pause");
        auto roi_client = observer->create_client<std_srvs::srv::Trigger>("/detect_node/reload_roi");
        auto status = [&] {
            auto response = request<std_srvs::srv::Trigger>(executor, status_client, std::make_shared<std_srvs::srv::Trigger::Request>());
            require(response->success, response->message); return response->message;
        };
        auto options = [&](const std::string& mode, double fps, int step, bool shutdown = false) {
            rclcpp::NodeOptions result;
            result.use_intra_process_comms(true).parameter_overrides({
                rclcpp::Parameter("config_dir", directory), rclcpp::Parameter("roi_path", roi.string()),
                rclcpp::Parameter("video.path", video.string()), rclcpp::Parameter("video.fps", fps),
                rclcpp::Parameter("video.playback_mode", mode), rclcpp::Parameter("video.synthetic_stamp", true),
                rclcpp::Parameter("video.shutdown_on_eof", shutdown), rclcpp::Parameter("input.sampling_step", step),
                rclcpp::Parameter("raw_output.max_fps", 30.0), rclcpp::Parameter("debug_output_max_width", 120)});
            return result;
        };

        auto node = std::make_shared<DetectNode>(options("sequential", 20.0, 3));
        executor.add_node(node);
        until(executor, [&] { return detections.size() == 4 && timings.size() == 4; });
        until(executor, [&] { return value(status(), "done") == 1; });
        auto state = status();
        require(value(state, "acquired") == 10 && value(state, "processed") == 4 &&
                value(state, "sampled_out") == 6 && value(state, "overwritten") == 0, "sequential counts");
        require(node->count_subscribers("/image_raw") == 0, "detector still subscribes to raw image");
        for (int i = 0; i < 4; ++i) {
            const auto& detection = detections[i];
            require(rclcpp::Time(detection.header.stamp).nanoseconds() == 50000000LL + i * 150000000LL, "content timestamp");
            require(detection.header.frame_id == "camera_frame" && !detection.detections.empty(), "detection contract");
            require(std::abs(detection.detections[0].x - i * 60) < 3, "sampled wrong input frame");
            if (i) require(std::abs(detection.detections[0].class_conf - 0.15f) < 1e-5f, "algorithm dt");
            else require(detection.detections[0].class_conf == 0.0f, "first dt must not include startup wait");
            require(timings[i].end_to_end_ms >= 15 && timings[i].end_to_end_ms < 2000, "latency used synthetic ROS time");
        }
        require(debug_images > 0, "debug branch did not publish");
        auto raw_sub = observer->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(1), [&](const sensor_msgs::msg::Image& image) { raw_images.push_back(image); });
        until(executor, [&] { return !raw_images.empty(); });
        require(raw_images.back().width == 240 && raw_images.back().height == 100, "raw resolution changed");
        require(rclcpp::Time(raw_images.back().header.stamp).nanoseconds() == 500000000, "EOF raw cache lost final frame");
        executor.remove_node(node); node.reset();
        pump(executor, 30);
        raw_sub.reset(); raw_images.clear(); detections.clear(); timings.clear();

        makeVideo(video, 60);
        node = std::make_shared<DetectNode>(options("realtime", 100.0, 1));
        executor.add_node(node);
        until(executor, [&] { return detections.size() >= 3; });
        auto pause = std::make_shared<std_srvs::srv::SetBool::Request>(); pause->data = true;
        require(request<std_srvs::srv::SetBool>(executor, pause_client, pause)->success, "legacy pause failed");
        const auto acquired = value(status(), "acquired");
        pump(executor, 100);
        require(value(status(), "acquired") == acquired, "input advanced after pause acknowledgement");
        raw_sub = observer->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(1), [&](const sensor_msgs::msg::Image& image) { raw_images.push_back(image); });
        until(executor, [&] { return raw_images.size() >= 2; });
        require(raw_images.front().header.stamp == raw_images.back().header.stamp, "paused snapshot changed");
        { std::ofstream out(roi); out << "outpost_enabled: false\noutpost_score_threshold: 0.7\n"; }
        require(request<std_srvs::srv::Trigger>(executor, roi_client, std::make_shared<std_srvs::srv::Trigger::Request>())->success,
                "ROI reload while paused");
        { std::ofstream out(roi); out << "outpost_enabled: true\noutpost_roi: [0, 0, -1, 10]\n"; }
        require(!request<std_srvs::srv::Trigger>(executor, roi_client, std::make_shared<std_srvs::srv::Trigger::Request>())->success,
                "invalid ROI accepted");
        pause->data = false;
        require(request<std_srvs::srv::SetBool>(executor, new_pause_client, pause)->success, "new resume failed");
        until(executor, [&] { return value(status(), "done") == 1; });
        pump(executor, 30);
        state = status();
        require(value(state, "acquired") == 60 && value(state, "overwritten") > 0, "realtime did not overwrite stale frames");
        require(value(state, "processed") + value(state, "overwritten") == 60, "realtime accounting");
        require(std::abs(detections.back().detections[0].class_margin - 0.7f) < 1e-6f, "invalid reload mutated valid config");
        require(std::abs(detections.back().detections[0].x - 180) < 3, "final realtime frame not drained");
        executor.remove_node(node); node.reset(); raw_sub.reset();
        pump(executor, 30); detections.clear(); timings.clear();

        { std::ofstream out(roi); out << "outpost_enabled: false\n"; }
        makeVideo(video, 200);
        node = std::make_shared<DetectNode>(options("sequential", 20.0, 1));
        executor.add_node(node);
        until(executor, [&] { return detections.size() >= 2; });
        const auto before = std::chrono::steady_clock::now();
        executor.remove_node(node); node.reset();
        require(std::chrono::steady_clock::now() - before < 1s, "shutdown stuck behind full sequential queue");
        pump(executor, 30); detections.clear(); timings.clear();

        makeVideo(video, 10);
        node = std::make_shared<DetectNode>(options("sequential", 20.0, 3, true));
        executor.add_node(node);
        until(executor, [&] { return !rclcpp::ok(); });
        require(detections.size() == 4, "EOF shutdown preceded final result");
        executor.remove_node(node); node.reset();
        std::cout << "PASS: real node ingress, sampling/dt, EOF drain, raw cache, pause/resume, ROI, shutdown\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
}
