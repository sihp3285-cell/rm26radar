// Test-only source/host. Loads the UNMODIFIED production component libraries.
// CSV preserves exact frame provenance; no images are recorded into the bag.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <class_loader/class_loader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <thread>
#include <cstring>
#include <atomic>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto cfg = std::make_shared<rclcpp::Node>("replay_fixture");
  auto root = cfg->declare_parameter<std::string>("root", "/home/delphine/rm/tensorrt10_detect");
  auto video = cfg->declare_parameter<std::string>("video", "");
  auto output = cfg->declare_parameter<std::string>("output", "");
  double slope = cfg->declare_parameter<double>("slope", 1.0);
  double offset = cfg->declare_parameter<double>("offset", 0.0);
  double speed = cfg->declare_parameter<double>("speed", 1.0);
  int stride = cfg->declare_parameter<int>("stride", 1);
  double max_content = cfg->declare_parameter<double>("max_content", 1e9);
  auto params = cfg->declare_parameter<std::string>("params_file", root + "/src/tensorrt_detect/config/ros2_params.yaml");
  auto pose_library = cfg->declare_parameter<std::string>("pose_library", root + "/install/tensorrt_detect/lib/libpose_node_component.so");
  std::vector<std::shared_ptr<class_loader::ClassLoader>> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> instances;
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto pair : {std::make_pair("detect", "DetectNode"), std::make_pair("pose", "PoseNode"), std::make_pair("map", "MapNode")}) {
    auto library = std::string(pair.first) == "pose" ? pose_library : root + "/install/tensorrt_detect/lib/lib" + pair.first + "_node_component.so";
    std::cout << "FIXTURE_COMPONENT " << pair.first << " " << library << std::endl;
    auto loader = std::make_shared<class_loader::ClassLoader>(library);
    auto factory = loader->createInstance<rclcpp_components::NodeFactory>(std::string("rclcpp_components::NodeFactoryTemplate<") + pair.second + ">");
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).use_intra_process_comms(true);
    options.arguments({"--ros-args", "--params-file", params});
    if (std::string(pair.first) == "detect") options.append_parameter_override("publish_debug_image", false);
    auto instance = factory->create_node_instance(options);
    executor.add_node(instance.get_node_base_interface());
    instances.push_back(instance);
    loaders.push_back(loader);
  }
  rclcpp::NodeOptions src_opts;
  src_opts.use_intra_process_comms(true).use_global_arguments(false);
  auto src = std::make_shared<rclcpp::Node>("replay_source", src_opts);
  auto pub = src->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::QoS(1));
  executor.add_node(src);
  std::thread spin([&] { executor.spin(); });
  cv::VideoCapture cap(video);
  if (!cap.isOpened()) { executor.cancel(); spin.join(); return 2; }
  const double fps = cap.get(cv::CAP_PROP_FPS);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // All components are initialized before the origin; positive offset is allowed.
  const auto epoch = src->now().nanoseconds();
  const auto steady = std::chrono::steady_clock::now();
  std::ofstream meta(output + "/source.json");
  meta << std::setprecision(17) << "{\"epoch_ns\":" << epoch << ",\"video\":\"" << video
       << "\",\"fps\":" << fps << ",\"slope\":" << slope << ",\"offset\":" << offset
       << ",\"speed\":" << speed << ",\"stride\":" << stride << "}\n";
  meta.close();
  std::ofstream csv(output + "/source_frames.csv");
  csv << "frame,content_s,match_s,stamp_ns,publish_ns,lateness_s,published\n" << std::setprecision(17);
  std::int64_t index = -1;
  int dropped = 0, sent = 0;
  cv::Mat frame;
  while (rclcpp::ok() && cap.grab()) {
    ++index;
    double content = index / fps;
    if (content > max_content) break;
    if (index % stride != 0) continue;
    if (!cap.retrieve(frame)) break;
    double elapsed = slope * content;
    auto deadline = steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(elapsed / speed));
    std::this_thread::sleep_until(deadline);
    double late = std::chrono::duration<double>(std::chrono::steady_clock::now() - deadline).count();
    auto stamp = epoch + static_cast<std::int64_t>(elapsed * 1e9);
    // Real-time pass drops late source frames rather than slowing physical time.
    if (speed == 1.0 && late > slope * stride / fps) {
      csv << index << ',' << content << ',' << offset + elapsed << ',' << stamp << ',' << src->now().nanoseconds() << ',' << late << ",0\n";
      ++dropped;
      continue;
    }
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = rclcpp::Time(stamp);
    msg->header.frame_id = "video_frame";
    msg->height = frame.rows; msg->width = frame.cols;
    msg->encoding = "bgr8"; msg->is_bigendian = false; msg->step = frame.cols * 3;
    msg->data.resize(msg->height * msg->step);
    std::memcpy(msg->data.data(), frame.data, msg->data.size());
    csv << index << ',' << content << ',' << offset + elapsed << ',' << stamp << ',' << src->now().nanoseconds() << ',' << late << ",1\n";
    pub->publish(std::move(msg));
    ++sent;
    if (sent % 200 == 0) { csv.flush(); std::cout << "FIXTURE frame=" << index << " content=" << content << " late=" << late << " dropped=" << dropped << std::endl; }
  }
  csv.close();
  std::cout << "FIXTURE_DONE published=" << sent << " dropped=" << dropped << " frames=" << index + 1 << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  executor.cancel(); spin.join();
  rclcpp::shutdown();
  return 0;
}
