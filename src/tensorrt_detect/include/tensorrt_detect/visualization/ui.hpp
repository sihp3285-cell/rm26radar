#ifndef __UI_HPP__
#define __UI_HPP__

#include <opencv2/opencv.hpp>
#include <string>
#include "radarmap.hpp"

class UI 
{
    private:
    std::string windowName;
    cv::Rect btnrect;
    public:
    UI(const std::string& windowName);
    int update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused);
};



#endif
