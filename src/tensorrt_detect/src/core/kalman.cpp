#include "kalman.hpp"
#include <cmath>
#include <limits>

// ==========================================
// KalmanFilterBox (对应像素框) - 8维固定大小矩阵
// ==========================================
KalmanFilterBox::KalmanFilterBox(float dt, float q_std, float r_std) 
    : dt_(dt), q_std_(q_std), r_std_(r_std)
{
    x.setZero();
    // 初始协方差较大，表示初始状态不确定，需要通过观测来更新
    P = Eigen::Matrix<float, 8, 8>::Identity() * 100.0f;
    // 状态转移矩阵：只考虑位置和速度
    F = Eigen::Matrix<float, 8, 8>::Identity();
    // 位置 += 速度 * dt
    for(int i=0; i<4; i++) F(i, i+4) = dt;

    // 观测矩阵：只观测位置 [cx, cy, w, h]
    H.setZero();
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; H(3, 3) = 1;

    // 测量噪声协方差
    R = Eigen::Matrix<float, 4, 4>::Identity() * (r_std * r_std);
    
    // 过程噪声协方差（初始值，会在predict中根据dt重新计算）
    Q.setZero();
    
    reset();
}

void KalmanFilterBox::updateQ(float dt) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt / 2.0f;
    float dt4 = dt2 * dt2 / 4.0f;
    
    // 连续时间白噪声模型：Q = q_std^2 * [dt^4/4, dt^3/2; dt^3/2, dt^2]
    // 对每个维度（x, y, w, h）独立计算
    Q.setZero();
    for (int i = 0; i < 4; i++) {
        Q(i, i) = dt4;           // 位置-位置
        Q(i, i+4) = dt3;         // 位置-速度
        Q(i+4, i) = dt3;         // 速度-位置
        Q(i+4, i+4) = dt2;       // 速度-速度
    }
    Q *= (q_std_ * q_std_);
}

std::vector<float> KalmanFilterBox::predict(float dt) {
    float use_dt = (dt > 0) ? dt : dt_;
    
    // 更新状态转移矩阵中的dt
    for(int i=0; i<4; i++) F(i, i+4) = use_dt;
    
    // 根据当前dt更新Q矩阵
    updateQ(use_dt);
    
    x = F * x;
    P = F * P * F.transpose() + Q;
    return {x(0), x(1), x(2), x(3)};
}

float KalmanFilterBox::innovationSquared(const std::vector<float>& bbox) const {
    if (bbox.size() != 4) {
        return std::numeric_limits<float>::infinity();
    }

    Eigen::Matrix<float, 4, 1> z;
    z << bbox[0], bbox[1], bbox[2], bbox[3];
    if (!z.allFinite()) {
        return std::numeric_limits<float>::infinity();
    }

    const Eigen::Matrix<float, 4, 1> innovation = z - H * x;
    const Eigen::Matrix<float, 4, 4> S = H * P * H.transpose() + R;
    const Eigen::LDLT<Eigen::Matrix<float, 4, 4>> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        return std::numeric_limits<float>::infinity();
    }

    const Eigen::Matrix<float, 4, 1> normalized = ldlt.solve(innovation);
    if (ldlt.info() != Eigen::Success || !normalized.allFinite()) {
        return std::numeric_limits<float>::infinity();
    }
    return std::max(0.0f, innovation.dot(normalized));
}

std::vector<float> KalmanFilterBox::update(
    const std::vector<float>& bbox, float gate_threshold, bool* accepted) {
    const float nis = innovationSquared(bbox);
    const bool measurement_accepted = std::isfinite(nis) &&
        (gate_threshold <= 0.0f || nis <= gate_threshold);
    if (accepted != nullptr) {
        *accepted = measurement_accepted;
    }
    if (!measurement_accepted) {
        return {x(0), x(1), x(2), x(3)};
    }

    Eigen::Matrix<float, 4, 1> z;
    z << bbox[0], bbox[1], bbox[2], bbox[3];

    Eigen::Matrix<float, 4, 4> S = H * P * H.transpose() + R;
    Eigen::LDLT<Eigen::Matrix<float, 4, 4>> ldlt(S);
    const Eigen::Matrix<float, 8, 4> K =
        ldlt.solve((H * P).eval()).transpose();

    x += K * (z - H * x);
    // Joseph 形式比 (I-KH)P 更能保持协方差的对称性和半正定性。
    const Eigen::Matrix<float, 8, 8> I_KH =
        Eigen::Matrix<float, 8, 8>::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    P = 0.5f * (P + P.transpose()).eval();
    
    return {x(0), x(1), x(2), x(3)};
}

void KalmanFilterBox::reset(const std::vector<float>& initial_bbox) {
    x.setZero();
    if (!initial_bbox.empty() && initial_bbox.size() == 4) {
        x(0) = initial_bbox[0];  // cx
        x(1) = initial_bbox[1];  // cy
        x(2) = initial_bbox[2];  // w
        x(3) = initial_bbox[3];  // h
    } else {
        x(2) = 1.0f;  // default w
        x(3) = 1.0f;  // default h
    }

    P = Eigen::Matrix<float, 8, 8>::Identity() * 100.0f;
}

std::vector<float> KalmanFilterBox::get_state() const {
    return {x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7)};
}

// ==========================================
// KalmanFilter2d (对应物理坐标) - 4维固定大小矩阵
// ==========================================
KalmanFilter2d::KalmanFilter2d(float q_std, float r_std, float dt) 
    : dt_(dt), q_std_(q_std), r_std_(r_std) 
{
    x.setZero();
    P = Eigen::Matrix<float, 4, 4>::Identity() * 100.0f;
    F = Eigen::Matrix<float, 4, 4>::Identity();
    F(0, 2) = dt; F(1, 3) = dt;

    H.setZero();
    H(0, 0) = 1; H(1, 1) = 1;

    R = Eigen::Matrix<float, 2, 2>::Identity() * (r_std * r_std);
    Q.setZero();
      
    reset();
}

std::vector<float> KalmanFilter2d::predict(float dt) {
    float use_dt = (dt > 0) ? dt : dt_;
    float dt2 = use_dt * use_dt;
    float dt3 = dt2 * use_dt / 2.0f;
    float dt4 = dt2 * dt2 / 4.0f;

    F(0, 2) = use_dt; F(1, 3) = use_dt;
    
    // 港科大 Q 矩阵数学推导 (连续时间白噪声模型)
    Q << dt4, 0, dt3, 0,
         0, dt4, 0, dt3,
         dt3, 0, dt2, 0,
         0, dt3, 0, dt2;
    Q *= (q_std_ * q_std_);

    x = F * x;
    P = F * P * F.transpose() + Q;
    return {x(0), x(1)};
}

float KalmanFilter2d::innovationSquared(const std::vector<float>& pos) const {
    if (pos.size() < 2) {
        return std::numeric_limits<float>::infinity();
    }

    Eigen::Matrix<float, 2, 1> z;
    z << pos[0], pos[1];
    if (!z.allFinite()) {
        return std::numeric_limits<float>::infinity();
    }

    const Eigen::Matrix<float, 2, 1> innovation = z - H * x;
    const Eigen::Matrix<float, 2, 2> S = H * P * H.transpose() + R;
    const Eigen::LDLT<Eigen::Matrix<float, 2, 2>> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        return std::numeric_limits<float>::infinity();
    }

    const Eigen::Matrix<float, 2, 1> normalized = ldlt.solve(innovation);
    if (ldlt.info() != Eigen::Success || !normalized.allFinite()) {
        return std::numeric_limits<float>::infinity();
    }
    return std::max(0.0f, innovation.dot(normalized));
}

std::vector<float> KalmanFilter2d::update(
    const std::vector<float>& pos, float gate_threshold, bool* accepted) {
    const float nis = innovationSquared(pos);
    const bool measurement_accepted = std::isfinite(nis) &&
        (gate_threshold <= 0.0f || nis <= gate_threshold);
    if (accepted != nullptr) {
        *accepted = measurement_accepted;
    }
    if (!measurement_accepted) {
        return {x(0), x(1)};
    }

    Eigen::Matrix<float, 2, 1> z;
    z << pos[0], pos[1];

    Eigen::Matrix<float, 2, 2> S = H * P * H.transpose() + R;
    Eigen::LDLT<Eigen::Matrix<float, 2, 2>> ldlt(S);
    const Eigen::Matrix<float, 4, 2> K =
        ldlt.solve((H * P).eval()).transpose();

    x += K * (z - H * x);
    const Eigen::Matrix<float, 4, 4> I_KH =
        Eigen::Matrix<float, 4, 4>::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    P = 0.5f * (P + P.transpose()).eval();
    return {x(0), x(1)};
}

void KalmanFilter2d::reset(const std::vector<float>& initial_pos) {
    x.setZero();
    if (!initial_pos.empty() && initial_pos.size() >= 2) {
        x(0) = initial_pos[0];
        x(1) = initial_pos[1];
    }

    P = Eigen::Matrix<float, 4, 4>::Identity() * 100.0f;
}

std::vector<float> KalmanFilter2d::get_position() const {
    return {x(0), x(1)};
}

std::vector<float> KalmanFilter2d::get_velocity() const {
    return {x(2), x(3)};
}

