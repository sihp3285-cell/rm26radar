#ifndef __DRAW_HPP__
#define __DRAW_HPP__

#include <opencv2/opencv.hpp>

#include "model.hpp"

const std::vector<cv::Scalar> COLORS = {
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(255, 255, 0),   // 黄色
    cv::Scalar(0, 255, 255),   // 青色
    cv::Scalar(255, 0, 255),   // 洋红色
    cv::Scalar(255, 165, 0),   // 橙色
    cv::Scalar(128, 0, 128),   // 紫色
    cv::Scalar(0, 128, 128),   // 蓝绿色

};

void drawDetect(cv::Mat &frame, const std::vector<Result>& results, const std::vector<std::string> &classNames);

#endif
