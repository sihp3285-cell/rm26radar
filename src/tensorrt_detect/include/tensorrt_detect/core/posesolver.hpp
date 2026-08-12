/**
 * @file posesolver.hpp
 * @brief 连接相机标定与 PLY 射线求交的 pixel-to-world 门面。
 *
 * PoseNode 持有本类：外参 R/T 描述相机与世界的关系，Raycaster 负责与场地 mesh
 * 求首个交点。本类还用有限差分估计像素误差向 world(x,z) 的方向相关协方差。
 */
#ifndef __POSESOLVER_HPP__
#define __POSESOLVER_HPP__

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include "raycaster.hpp"

/** 像素定位误差传播的数值与安全边界配置。 */
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

/** 一个落地点及其测量质量；world 只承载世界平面 (x,z)。 */
struct WorldProjection {
    cv::Point2f world;
    // world(x,z) 测量协方差，行主序 [xx, xz, zx, zz]。
    std::array<float, 4> covariance{{1.0f, 0.0f, 0.0f, 1.0f}};
    bool covariance_valid = false;
    bool surface_discontinuity = false;
    float jacobian_condition_number = 1.0f;
};

/**
 * 相机投影解算器。由 PoseNode 创建、标定 service 更新外参、每帧批量调用。
 * calibrate() 求解并保存外参；middletoworld*() 只做几何查询，不负责跨帧跟踪。
 */
class PoseSolver
{
    private:
    cv::Mat K,D;//内参矩阵和畸变系数；把像素还原成相机射线。
    cv::Mat R,T;//相机旋转/平移外参，按 Raycaster 接口约定传入。
    bool isPoseEstimated = false;
    Raycaster raycaster_;
    public:
    /** 深拷贝相机内参和畸变系数；此时尚未具备有效外参。 */
    PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat);
    /** 用 3D-2D 对应点执行 solvePnP，更新并持有 R/T 外参。 */
    void calibrate(const std::vector<cv::Point3f>& objectPoints, 
                   const std::vector<cv::Point2f>& imagePoints);
    /** 深拷贝磁盘加载的外参并将投影器标记为可用。 */
    void setExtrinsic(const cv::Mat& R_in, const cv::Mat& T_in);
    /** 把当前外参深拷贝到输出参数，调用方可安全独立修改。 */
    void getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const;
    /** 取单个矩形底边中点做射线求交，返回 world(x,z)；主要用于低频兼容调用。 */
    cv::Point2f middletoworld(const cv::Rect& box);
    /** 批量投影矩形底边中点，复用一次 Open3D batch 查询。 */
    std::vector<cv::Point2f> middletoworldBatch(const std::vector<cv::Rect>& boxes);
    /** 批量投影并以有限差分传播像素误差，返回 world(x,z) 及 2x2 测量协方差。 */
    std::vector<WorldProjection> middletoworldBatchWithUncertainty(
        const std::vector<cv::Rect>& boxes,
        const ProjectionUncertaintyConfig& config) const;
    /** 返回可变 Raycaster 引用，供节点启动时加载一次 PLY mesh。 */
    Raycaster& getRaycaster() { return raycaster_; }
    /** 返回只读 Raycaster 引用，供只读状态/查询调用。 */
    const Raycaster& getRaycaster() const { return raycaster_; }
};
#endif
