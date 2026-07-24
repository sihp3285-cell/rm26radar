#include "posesolver.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <vector>

namespace {

cv::Mat matrix_from_yaml(
    const YAML::Node& node, int rows, int columns) {
    const auto values = node.as<std::vector<double>>();
    if (static_cast<int>(values.size()) != rows * columns) {
        return {};
    }
    cv::Mat result(rows, columns, CV_64F);
    for (int index = 0; index < rows * columns; ++index) {
        result.at<double>(index / columns, index % columns) = values[index];
    }
    return result;
}

}  // namespace

TEST(ProjectionUncertainty, FarDepthDirectionReceivesLargerCovariance) {
    const std::filesystem::path config_dir(TENSORRT_DETECT_CONFIG_DIR);
    const YAML::Node camera =
        YAML::LoadFile((config_dir / "camera.yaml").string());
    const YAML::Node calibration =
        YAML::LoadFile((config_dir / "calib_result.yaml").string());

    const cv::Mat K = matrix_from_yaml(camera["cameraMatrix"], 3, 3);
    const auto distortion_values =
        camera["distCoeffs"].as<std::vector<double>>();
    cv::Mat D(1, static_cast<int>(distortion_values.size()), CV_64F);
    for (std::size_t index = 0; index < distortion_values.size(); ++index) {
        D.at<double>(0, static_cast<int>(index)) = distortion_values[index];
    }
    const cv::Mat camera_to_world = matrix_from_yaml(calibration["r"], 3, 3);
    const cv::Mat camera_world = matrix_from_yaml(calibration["t"], 3, 1);
    ASSERT_FALSE(K.empty());
    ASSERT_FALSE(camera_to_world.empty());
    ASSERT_FALSE(camera_world.empty());

    PoseSolver solver(K, D);
    solver.setExtrinsic(camera_to_world, camera_world);

    // 把远场地面点投回像素，只用于构造以该像素为底边中心的检测框。
    const cv::Mat world_to_camera = camera_to_world.t();
    const cv::Mat translation = -world_to_camera * camera_world;
    cv::Mat rotation_vector;
    cv::Rodrigues(world_to_camera, rotation_vector);
    std::vector<cv::Point3f> world_points{{0.0f, 0.0f, -10.0f}};
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
        world_points, rotation_vector, translation, K, D, image_points);
    ASSERT_EQ(image_points.size(), 1u);

    const int bottom_u = static_cast<int>(std::lround(image_points[0].x));
    const int bottom_v = static_cast<int>(std::lround(image_points[0].y));
    const cv::Rect box(bottom_u - 20, bottom_v - 30, 40, 30);
    ProjectionUncertaintyConfig config;
    config.pixel_sigma_px = 4.0f;
    config.finite_difference_px = 2.0f;
    const auto projections =
        solver.middletoworldBatchWithUncertainty({box}, config);
    ASSERT_EQ(projections.size(), 1u);
    const auto& projection = projections.front();
    ASSERT_TRUE(projection.covariance_valid);
    EXPECT_NEAR(projection.world.x, 0.0f, 0.05f);
    EXPECT_NEAR(projection.world.y, -10.0f, 0.05f);
    EXPECT_GT(projection.jacobian_condition_number, 5.0f);
    EXPECT_GT(
        projection.covariance[3],
        10.0f * projection.covariance[0]);
}
