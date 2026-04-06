#ifndef MOUSEBACK_HPP
#define MOUSEBACK_HPP
#include <opencv2/opencv.hpp>
#include <vector>
class MouseBack
{
    private:
    std::vector<cv::Point2f> points;
    std::string windowName;
    int maxpoints;
    static void onMouse(int event, int x, int y, int flags, void* userdata);
    public:
        MouseBack(const std::string& windowName,int requirePoints);
        std::vector<cv::Point2f> getPoints(const cv::Mat& frame);
};
#endif