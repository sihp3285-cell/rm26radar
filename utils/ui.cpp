#include "include/ui.hpp"

UI::UI(const std::string& windowName) 
    : windowName(windowName)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1920, 720); 
}

int UI::update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused)
{
    int targetHeight = frame.rows;
    int targetWidth = static_cast<int>(radarImg.cols * ((float)targetHeight / radarImg.rows));
    cv::Mat finalRadar;
    cv::resize(radarImg, finalRadar, cv::Size(targetWidth, targetHeight));

    cv::Mat combinedImg;
    cv::hconcat(std::vector<cv::Mat>{frame, finalRadar}, combinedImg);

    cv::imshow(windowName, combinedImg);
    return cv::waitKey(isPaused ? 0 : 1);
}