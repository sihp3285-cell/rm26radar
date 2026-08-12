/**
 * @file kalman.hpp
 * @brief Tracker 使用的像素框与世界平面常速度 Kalman Filter。
 *
 * 两个滤波器都维护 state x、协方差 P、状态转移 F、观测矩阵 H、测量噪声 R
 * 与过程噪声 Q。predict() 只依据运动模型推进；update() 用真实 observation 的
 * innovation（新息）修正状态。NIS 用于关联前硬门控，P 还随 WorldTarget 下发给
 * Position Prior 衡量速度/位置估计可靠度。
 */
#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>

// ==========================================
// KalmanFilterBox - 8维状态：像素框 [cx, cy, w, h, vx, vy, vw, vh]
// 使用固定大小矩阵，栈分配，无动态内存分配
// ==========================================
class KalmanFilterBox {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @param dt 默认时间步长
     * @param q_std 过程噪声标准差 (默认2.0)
     * @param r_std 测量噪声标准差 (默认1.0)
     */
    KalmanFilterBox(float dt = 1.0f, float q_std = 2.0f, float r_std = 1.0f);
    
    /**
     * 预测下一状态
     * @param dt 时间步长，<0时使用默认值；0 表示不推进物理时间
     * @return 预测的位置 [cx, cy, w, h]
     */
    std::vector<float> predict(float dt = -1.0f);
    
    /**
     * 用测量值更新滤波器
     * @param bbox 测量的边界框 [cx, cy, w, h]
     * @return 更新后的位置 [cx, cy, w, h]
     */
    std::vector<float> update(
        const std::vector<float>& bbox,
        float gate_threshold = -1.0f,
        bool* accepted = nullptr);

    /**
     * 计算测量创新的 NIS（平方马氏距离）。
     * 可直接用于卡方门控；无效测量返回 infinity。
     */
    float innovationSquared(const std::vector<float>& bbox) const;
    
    /**
     * 重置滤波器
     * @param initial_bbox 初始边界框 [cx, cy, w, h]，空则使用默认值
     */
    void reset(const std::vector<float>& initial_bbox = {});
    
    /**
     * 获取完整状态向量
     * @return [cx, cy, w, h, vx, vy, vw, vh]
     */
    std::vector<float> get_state() const;

    // 状态向量: [cx, cy, w, h, vx, vy, vw, vh]
    Eigen::Matrix<float, 8, 1> x;  // 当前后验状态 [cx,cy,w,h,vx,vy,vw,vh]。
    Eigen::Matrix<float, 8, 8> P;  // 状态估计误差协方差；predict 后通常增长。
    Eigen::Matrix<float, 8, 8> F;  // 常速度状态转移，位置-速度项由实际 dt 更新。
    Eigen::Matrix<float, 4, 8> H;  // 从状态选择可观测的 [cx,cy,w,h]。
    Eigen::Matrix<float, 4, 4> R;  // 像素测量噪声协方差。
    Eigen::Matrix<float, 8, 8> Q;  // 未建模加速度等过程噪声，随 dt 更新。

private:
    float dt_, q_std_, r_std_;
    /** 按实际 dt 重建常速度模型的过程噪声 Q；只改变滤波器内部矩阵。 */
    void updateQ(float dt);
};

// ==========================================
// KalmanFilter2d - 4维状态：物理坐标 [x, y, vx, vy]
// 使用固定大小矩阵，栈分配，无动态内存分配
// ==========================================
class KalmanFilter2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @param q_std 过程噪声标准差 (默认2.0)
     * @param r_std 测量噪声标准差 (默认1.0)
     * @param dt 默认时间步长 (默认0.1)
     */
    KalmanFilter2d(float q_std = 2.0f, float r_std = 1.0f, float dt = 0.1f);
    
    /**
     * 预测下一状态
     * @param dt 时间步长，<0时使用默认值；0 表示不推进物理时间
     * @return 预测的位置 [x, y]
     */
    std::vector<float> predict(float dt = -1.0f);
    
    /**
     * 用测量值更新滤波器
     * @param pos 测量的位置 [x, y]
     * @return 更新后的位置 [x, y]
     */
    std::vector<float> update(
        const std::vector<float>& pos,
        float gate_threshold = -1.0f,
        bool* accepted = nullptr,
        const std::array<float, 4>* measurement_covariance = nullptr);

    /**
     * 计算测量创新的 NIS（平方马氏距离）。
     * 可直接用于卡方门控；无效测量返回 infinity。
     */
    float innovationSquared(
        const std::vector<float>& pos,
        const std::array<float, 4>* measurement_covariance = nullptr) const;
    
    /**
     * 重置滤波器
     * @param initial_pos 初始位置 [x, y]，空则使用默认值
     */
    void reset(
        const std::vector<float>& initial_pos = {},
        const std::array<float, 4>* measurement_covariance = nullptr);
    
    /**
     * 获取当前位置估计
     * @return [x, y]
     */
    std::vector<float> get_position() const;
    
    /**
     * 获取当前速度估计
     * @return [vx, vy]
     */
    std::vector<float> get_velocity() const;

    // 状态向量: [x, y, vx, vy]
    Eigen::Matrix<float, 4, 1> x;  // 世界地面状态 [x,z,vx,vz]。
    Eigen::Matrix<float, 4, 4> P;  // 随 WorldTarget 以 4x4 行主序向下游发布。
    Eigen::Matrix<float, 4, 4> F;
    Eigen::Matrix<float, 2, 4> H;  // 观测矩阵 (2维观测)
    Eigen::Matrix<float, 2, 2> R;
    Eigen::Matrix<float, 4, 4> Q;

private:
    float dt_, q_std_, r_std_;
};
