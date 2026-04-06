#include "mouseback.hpp"
#include <iostream>

MouseBack::MouseBack(const std::string& windowName, int requirePoints) : windowName(windowName), maxpoints(requirePoints) {}
void MouseBack::onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        MouseBack* self = static_cast<MouseBack*>(userdata);
        if (self->points.size() < self->maxpoints)
        {
            self->points.emplace_back(cv::Point2f(x, y));
            std::cout << "已记录点：" << self->points.size() << " (" << x << ", " << y << ")" << std::endl;
        }
    }
}
std::vector<cv::Point2f> MouseBack::getPoints(const cv::Mat& frame)
{
    points.clear();
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::setMouseCallback(windowName, onMouse, this);
    while (true)
    {
        cv::Mat displayFrame = frame.clone();
        for (size_t i = 0; i < points.size(); ++i) {
        cv::circle(displayFrame, points[i], 6, cv::Scalar(0, 0, 255), -1);
        cv::putText(displayFrame, std::to_string(i+1), points[i] + cv::Point2f(10, 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow(windowName, displayFrame);
        int key = cv::waitKey(10);
        if (points.size() >= maxpoints) {
            std::cout << "标定完成！" << std::endl;
            cv::waitKey(1000);
            break;            
        }
        if (key == 'q') {
            std::cout << "标定取消！" << std::endl;
            break;
        }
    }
    cv::destroyWindow(windowName);
    return points;
}
