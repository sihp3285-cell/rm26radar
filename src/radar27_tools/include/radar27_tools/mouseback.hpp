/**
 * @file mouseback.hpp
 * @brief CalibrateNode 与 ROISetNode 共用的 OpenCV 鼠标像素点采集器。
 * static callback 通过 userdata 找回对象；返回值尚未进入 world/map 坐标。
 */
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
    /** OpenCV C 风格回调入口：把左键像素坐标追加到 userdata 指向的实例。 */
    static void onMouse(int event, int x, int y, int flags, void* userdata);
    public:
        /** 绑定窗口名并设置本轮必须采集的像素点数量。 */
        MouseBack(const std::string& windowName,int requirePoints);
        /** 阻塞显示 frame 并收集点击，达到数量或用户取消后返回像素点副本。 */
        std::vector<cv::Point2f> getPoints(const cv::Mat& frame);
};
#endif
