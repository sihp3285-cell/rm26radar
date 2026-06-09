#ifndef __DRAW_HPP__
#define __DRAW_HPP__

#include <opencv2/opencv.hpp>

#include "model.hpp"

const std::vector<cv::Scalar> COLORS = {
    cv::Scalar(255,255,255),
    cv::Scalar(0, 255, 0) 
};

void drawDetect(cv::Mat &frame, const std::vector<Result>& results, const std::vector<std::string> &classNames);
void drawDetect(cv::Mat &frame, const std::vector<Result>& results, const std::vector<std::string> &classNames,
                double scale_x, double scale_y);

#endif
