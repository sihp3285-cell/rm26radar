#ifndef __POSESOLVER_HPP__
#define __POSESOLVER_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

class PoseSolver
{
    private:
    cv::Mat K,D;//相机内参矩阵和畸变系数
    cv::Mat R,T;//相机旋转矩阵和平移向量
    bool isPoseEstimated = false;
    public:
    PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat);
    void calibrate(const std::vector<cv::Point3f>& objectPoints, 
                   const std::vector<cv::Point2f>& imagePoints);
    cv::Point2f middletoworld(const cv::Rect& box);
    float getFps();
};
#endif

