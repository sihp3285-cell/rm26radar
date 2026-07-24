#ifndef __POSESOLVER_HPP__
#define __POSESOLVER_HPP__

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include "raycaster.hpp"

struct ProjectionUncertaintyConfig {
    // 检测框底边像素定位的标准差。
    float pixel_sigma_px = 4.0f;
    // 用中心差分估计 pixel -> world(x,z) 局部雅可比的像素步长。
    float finite_difference_px = 2.0f;
    // 防止近场观测被赋予不现实的零噪声。
    float minimum_world_std_m = 0.03f;
    // PLY 边界或近地平线处限制协方差上界，保证数值稳定。
    float maximum_world_std_m = 1.50f;
    // 相邻射线命中高度差超过该值时，标记为跨地形表面。
    float surface_discontinuity_m = 0.12f;
};

struct WorldProjection {
    cv::Point2f world;
    // world(x,z) 测量协方差，行主序 [xx, xz, zx, zz]。
    std::array<float, 4> covariance{{1.0f, 0.0f, 0.0f, 1.0f}};
    bool covariance_valid = false;
    bool surface_discontinuity = false;
    float jacobian_condition_number = 1.0f;
};

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
    std::vector<WorldProjection> middletoworldBatchWithUncertainty(
        const std::vector<cv::Rect>& boxes,
        const ProjectionUncertaintyConfig& config) const;
    Raycaster& getRaycaster() { return raycaster_; }
    const Raycaster& getRaycaster() const { return raycaster_; }
};
#endif
