#include "radarmap.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>

namespace {

std::string map_path() {
    return (std::filesystem::path(TENSORRT_DETECT_CONFIG_DIR) / "map.png").string();
}

TEST(RadarMapDisplayTest, UnflippedDisplayUsesRawWorldProjection) {
    RadarMap map(map_path(), true);
    map.calibrate2(28.0f, 15.0f, 722, 388);
    map.setFlipTeam(false);

    const cv::Point2f raw = map.worldtomap({2.0f, 4.0f});
    const cv::Point2f display = map.worldtomapDisplay({2.0f, 4.0f});
    EXPECT_FLOAT_EQ(display.x, raw.x);
    EXPECT_FLOAT_EQ(display.y, raw.y);
}

TEST(RadarMapDisplayTest, FlippedDisplayMatchesDrawMapRotation) {
    RadarMap map(map_path(), true);
    map.calibrate2(28.0f, 15.0f, 722, 388);
    map.setFlipTeam(true);

    // map.png 经构造函数旋转后为 388 x 722。OpenCV ROTATE_180 的像素映射
    // 是 (x, y) -> (width-1-x, height-1-y)。
    const cv::Point2f raw = map.worldtomap({2.0f, 4.0f});
    const cv::Point2f display = map.worldtomapDisplay({2.0f, 4.0f});
    EXPECT_NEAR(display.x, 387.0f - raw.x, 1e-5f);
    EXPECT_NEAR(display.y, 721.0f - raw.y, 1e-5f);
}

TEST(RadarMapDisplayTest, OppositeWorldPointsCoincideAcrossTeamViews) {
    RadarMap map(map_path(), true);
    map.calibrate2(28.0f, 15.0f, 722, 388);

    map.setFlipTeam(false);
    const cv::Point2f blue_view = map.worldtomapDisplay({2.0f, 4.0f});

    map.setFlipTeam(true);
    const cv::Point2f red_view = map.worldtomapDisplay({-2.0f, -4.0f});

    // 世界坐标与阵营视角同时中心对称后，应落在相同的屏幕位置。
    EXPECT_NEAR(red_view.x, blue_view.x - 1.0f, 1e-5f);
    EXPECT_NEAR(red_view.y, blue_view.y - 1.0f, 1e-5f);
}

}  // namespace
