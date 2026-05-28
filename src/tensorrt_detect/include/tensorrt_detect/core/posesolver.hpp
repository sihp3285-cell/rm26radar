#ifndef __POSESOLVER_HPP__
#define __POSESOLVER_HPP__

#include <opencv2/opencv.hpp>
#include <vector>
#include "raycaster.hpp"

class PoseSolver
{
    private:
    cv::Mat K,D;//相机内参矩阵和畸变系数
    cv::Mat R,T;//相机旋转矩阵和平移向量
    bool isPoseEstimated = false;
    Raycaster raycaster_;
    public:
    PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat);
    void calibrate(const std::vector<cv::Point3f>& objectPoints, 
                   const std::vector<cv::Point2f>& imagePoints);
    void setExtrinsic(const cv::Mat& R_in, const cv::Mat& T_in);
    void getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const;
    cv::Point2f middletoworld(const cv::Rect& box);
    std::vector<cv::Point2f> middletoworldBatch(const std::vector<cv::Rect>& boxes);
    Raycaster& getRaycaster() { return raycaster_; }
    const Raycaster& getRaycaster() const { return raycaster_; }
};
#endif