/**
 * @file ui.cpp
 * @brief OpenCV 调试图窗口实现：主图 + 雷达图水平拼接显示。
 */
#include "ui.hpp"

UI::UI(const std::string& windowName)
    : windowName(windowName)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1920, 720);   // 大窗口便于肉眼检查检测效果
}

int UI::update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused)
{
    // 雷达图先按主图高度等比缩放，保证两图拼接后高度一致
    int targetHeight = frame.rows;
    int targetWidth = static_cast<int>(radarImg.cols * ((float)targetHeight / radarImg.rows));
    cv::Mat finalRadar;
    cv::resize(radarImg, finalRadar, cv::Size(targetWidth, targetHeight));

    // 水平拼接 [主图 | 雷达图]，一次 imshow 呈现
    cv::Mat combinedImg;
    cv::hconcat(std::vector<cv::Mat>{frame, finalRadar}, combinedImg);

    cv::imshow(windowName, combinedImg);
    // 暂停时 waitKey(0) 阻塞直到按键，否则 1ms 非阻塞刷新保证显示流畅
    return cv::waitKey(isPaused ? 0 : 1);
}
