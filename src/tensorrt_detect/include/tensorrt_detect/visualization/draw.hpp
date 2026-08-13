/**
 * @file draw.hpp
 * @brief DetectNode 调试图绘制接口；只修改传入 cv::Mat，不改变 Result。
 */
#ifndef __DRAW_HPP__
#define __DRAW_HPP__

#include <opencv2/opencv.hpp>

#include "model.hpp"

// 调试图两色方案：下标 0 白色给车辆框，下标 1 绿色给其余目标框
const std::vector<cv::Scalar> COLORS = {
    cv::Scalar(255,255,255),
    cv::Scalar(0, 255, 0)
};

/** 按原始图像尺寸绘制每个 Result 的矩形、类别和置信度。 */
void drawDetect(cv::Mat &frame, const std::vector<Result>& results, const std::vector<std::string> &classNames);
/** 在已缩放调试图上按 scale_x/scale_y 映射坐标后绘制同样的检测信息。 */
void drawDetect(cv::Mat &frame, const std::vector<Result>& results, const std::vector<std::string> &classNames,
                double scale_x, double scale_y);

#endif
