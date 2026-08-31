/**
 * @file posesolver.cpp
 * @brief 标定外参管理、批量射线求交及 pixel->world 协方差传播。
 * 中心像素给出落地点；上下左右有限差分命中形成局部 Jacobian，把像素方差传播到
 * world(x,z)。命中高度突变会标记 surface_discontinuity，供 PoseNode 放大不确定性。
 */
#include "posesolver.hpp"
#include "raycaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
//初始化
PoseSolver::PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat)
{
    this->K = camMat.clone();
    this->D = disMat.clone();
}
void PoseSolver::calibrate(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints)
{
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);
    this->R = R_mat.t();
    this->T = -this->R * tvec;
    this->isPoseEstimated = true;
}

void PoseSolver::setExtrinsic(const cv::Mat& R_in, const cv::Mat& T_in)
{
    this->R = R_in.clone();
    this->T = T_in.clone();
    this->isPoseEstimated = true;
}

void PoseSolver::getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const
{
    R_out = this->R.clone();
    T_out = this->T.clone();
}

cv::Point2f PoseSolver::middletoworld(const cv::Rect& box)
{
    if (!isPoseEstimated) return cv::Point2f(0, 0);
    std::vector<cv::Point2f> middle = {cv::Point2f(box.x + box.width / 2.0f, box.y + box.height)};
    cv::Point3f worldPoint = raycaster_.pixelToWorld(middle[0], K, D, R, T);
    return {worldPoint.x, worldPoint.z};
}

std::vector<cv::Point2f> PoseSolver::middletoworldBatch(const std::vector<cv::Rect>& boxes)
{
    std::vector<cv::Point2f> results;
    if (!isPoseEstimated) {
        results.resize(boxes.size(), cv::Point2f(0, 0));
        return results;
    }

    std::vector<cv::Point2f> pixels;
    pixels.reserve(boxes.size());
    for (const auto& box : boxes) {
        pixels.emplace_back(box.x + box.width / 2.0f, box.y + box.height);
    }

    auto worldPoints = raycaster_.pixelToWorldBatch(pixels, K, D, R, T);
    results.reserve(worldPoints.size());
    for (const auto& wp : worldPoints) {
        results.emplace_back(wp.x, wp.z);
    }
    return results;
}

std::vector<WorldProjection> PoseSolver::middletoworldBatchWithUncertainty(
    const std::vector<cv::Rect>& boxes,
    const ProjectionUncertaintyConfig& input_config) const
{
    std::vector<WorldProjection> results;
    results.reserve(boxes.size());
    if (boxes.empty()) {
        return results;
    }
    if (!isPoseEstimated) {
        results.resize(boxes.size());
        return results;
    }

    ProjectionUncertaintyConfig config = input_config;
    config.pixel_sigma_px = std::max(0.1f, config.pixel_sigma_px);
    config.finite_difference_px =
        std::max(0.5f, config.finite_difference_px);
    config.minimum_world_std_m =
        std::max(1e-3f, config.minimum_world_std_m);
    config.maximum_world_std_m = std::max(
        config.minimum_world_std_m, config.maximum_world_std_m);
    config.surface_discontinuity_m =
        std::max(0.0f, config.surface_discontinuity_m);

    // 每个框使用中心射线和 u/v 两侧的中心差分射线。除了估计反投影
    // 雅可比，这些射线还能暴露沟底/坡面/高台边缘的首交点切换。
    std::vector<cv::Point2f> pixels;
    pixels.reserve(boxes.size() * 5);
    for (const auto& box : boxes) {
        const float u = box.x + box.width * 0.5f;
        const float v = box.y + box.height;
        const float delta = config.finite_difference_px;
        pixels.emplace_back(u, v);
        pixels.emplace_back(u - delta, v);
        pixels.emplace_back(u + delta, v);
        pixels.emplace_back(u, v - delta);
        pixels.emplace_back(u, v + delta);
    }
    const auto points = raycaster_.pixelToWorldBatch(pixels, K, D, R, T);
    if (points.size() != pixels.size()) {
        results.resize(boxes.size());
        return results;
    }

    const float minimum_variance =
        config.minimum_world_std_m * config.minimum_world_std_m;
    const float maximum_variance =
        config.maximum_world_std_m * config.maximum_world_std_m;
    const float sigma_squared =
        config.pixel_sigma_px * config.pixel_sigma_px;

    // 有限值检查供中心/差分射线共用，避免 NaN 进入雅可比和协方差。
    const auto finite_point = [](const cv::Point3f& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z);
    };

    for (std::size_t box_index = 0; box_index < boxes.size(); ++box_index) {
        const std::size_t offset = box_index * 5;
        const cv::Point3f& center = points[offset];
        const cv::Point3f& u_minus = points[offset + 1];
        const cv::Point3f& u_plus = points[offset + 2];
        const cv::Point3f& v_minus = points[offset + 3];
        const cv::Point3f& v_plus = points[offset + 4];

        WorldProjection result;
        if (!finite_point(center)) {
            result.covariance = {
                maximum_variance, 0.0f, 0.0f, maximum_variance};
            results.push_back(result);
            continue;
        }
        if (config.capture_debug_ray_endpoint) {
            result.ray_endpoint = center;
            result.ray_endpoint_valid = true;
        }
        result.world = cv::Point2f(center.x, center.z);

        if (!finite_point(u_minus) || !finite_point(u_plus) ||
            !finite_point(v_minus) || !finite_point(v_plus)) {
            result.covariance = {
                maximum_variance, 0.0f, 0.0f, maximum_variance};
            result.covariance_valid = true;
            result.jacobian_condition_number =
                std::numeric_limits<float>::infinity();
            results.push_back(result);
            continue;
        }

        const float inverse_span =
            1.0f / (2.0f * config.finite_difference_px);
        const float dx_du = (u_plus.x - u_minus.x) * inverse_span;
        const float dz_du = (u_plus.z - u_minus.z) * inverse_span;
        const float dx_dv = (v_plus.x - v_minus.x) * inverse_span;
        const float dz_dv = (v_plus.z - v_minus.z) * inverse_span;

        // G = J * J^T。其特征值平方根是 pixel->world 雅可比的奇异值。
        const float g_xx = dx_du * dx_du + dx_dv * dx_dv;
        const float g_xz = dx_du * dz_du + dx_dv * dz_dv;
        const float g_zz = dz_du * dz_du + dz_dv * dz_dv;
        const float trace = g_xx + g_zz;
        const float discriminant = std::sqrt(std::max(
            0.0f, (g_xx - g_zz) * (g_xx - g_zz) +
                4.0f * g_xz * g_xz));
        const float jacobian_max_eigen =
            std::max(0.0f, 0.5f * (trace + discriminant));
        const float jacobian_min_eigen =
            std::max(0.0f, 0.5f * (trace - discriminant));
        result.jacobian_condition_number =
            jacobian_min_eigen > 1e-12f
                ? std::sqrt(jacobian_max_eigen / jacobian_min_eigen)
                : std::numeric_limits<float>::infinity();

        float covariance_xx;
        float covariance_xz;
        float covariance_zz;
        if (config.covariance_mode == "fixed") {
            // 测试用固定协方差：各向同性，不随射线方向/地形变化。
            const float fixed_variance =
                config.fixed_world_std_m * config.fixed_world_std_m;
            covariance_xx = fixed_variance;
            covariance_xz = 0.0f;
            covariance_zz = fixed_variance;
        } else {
            covariance_xx = minimum_variance + sigma_squared * g_xx;
            covariance_xz = sigma_squared * g_xz;
            covariance_zz = minimum_variance + sigma_squared * g_zz;
        }

        const float minimum_height = std::min(
            {center.y, u_minus.y, u_plus.y, v_minus.y, v_plus.y});
        const float maximum_height = std::max(
            {center.y, u_minus.y, u_plus.y, v_minus.y, v_plus.y});
        result.surface_discontinuity =
            maximum_height - minimum_height >
                config.surface_discontinuity_m;
        if (result.surface_discontinuity &&
            config.covariance_mode != "fixed") {
            // 比较差分射线与中心命中的 x-z 平面距离，用于识别地形表面跳变。
            const auto planar_distance = [&center](const cv::Point3f& point) {
                return std::hypot(point.x - center.x, point.z - center.z);
            };
            const float planar_spread = std::max(
                {planar_distance(u_minus), planar_distance(u_plus),
                 planar_distance(v_minus), planar_distance(v_plus)});
            // 首交点跨层时，差分不再是光滑导数。把跨层造成的平面位置分歧
            // 作为额外各向同性方差，避免 Kalman 把它解释成真实高速运动。
            const float terrain_std = std::clamp(
                planar_spread,
                config.minimum_world_std_m,
                config.maximum_world_std_m);
            covariance_xx += terrain_std * terrain_std;
            covariance_zz += terrain_std * terrain_std;
        }

        // 保持相关项，同时把最大特征值限制到配置上界。
        const float covariance_trace = covariance_xx + covariance_zz;
        const float covariance_discriminant = std::sqrt(std::max(
            0.0f,
            (covariance_xx - covariance_zz) *
                (covariance_xx - covariance_zz) +
            4.0f * covariance_xz * covariance_xz));
        const float covariance_max_eigen =
            0.5f * (covariance_trace + covariance_discriminant);
        if (covariance_max_eigen > maximum_variance) {
            const float extra_xx =
                std::max(0.0f, covariance_xx - minimum_variance);
            const float extra_zz =
                std::max(0.0f, covariance_zz - minimum_variance);
            const float scale = std::clamp(
                (maximum_variance - minimum_variance) /
                    std::max(1e-12f,
                        covariance_max_eigen - minimum_variance),
                0.0f, 1.0f);
            covariance_xx = minimum_variance + scale * extra_xx;
            covariance_xz *= scale;
            covariance_zz = minimum_variance + scale * extra_zz;
        }
        result.covariance = {
            covariance_xx, covariance_xz,
            covariance_xz, covariance_zz};
        result.covariance_valid =
            std::isfinite(covariance_xx) &&
            std::isfinite(covariance_xz) &&
            std::isfinite(covariance_zz) &&
            covariance_xx > 0.0f && covariance_zz > 0.0f &&
            covariance_xx * covariance_zz >
                covariance_xz * covariance_xz;
        if (!result.covariance_valid) {
            result.covariance = {
                maximum_variance, 0.0f, 0.0f, maximum_variance};
            result.covariance_valid = true;
        }
        results.push_back(result);
    }
    return results;
}
