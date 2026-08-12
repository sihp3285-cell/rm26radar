/**
 * @file raycaster.hpp
 * @brief 使用 Open3D RaycastingScene 把相机像素射线与场地 PLY 求交。
 *
 * mesh 在 PoseNode 初始化时一次加载到 scene；运行期 pixelToWorld/Batch 只查询首个
 * 命中。本类不负责内外参求解，也不处理 Tracker/canonical/map 坐标。
 */
#ifndef RCASTER_HPP
#define RCASTER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory> 


namespace open3d {
    namespace t{
        namespace geometry{
            class RaycastingScene;
        }
    }
}

/**
 * RaycastingScene 的唯一所有者。unique_ptr 隐藏 Open3D 类型并保证 scene 在本类
 * 析构时释放；复制被 unique_ptr 隐式禁止，PoseSolver 只通过引用暴露它。
 */
class Raycaster {
            public:
                
                /** 构造尚未加载 mesh 的空查询器。 */
                Raycaster();
                /** 释放独占的 Open3D RaycastingScene；不执行额外 GPU 同步业务。 */
                ~Raycaster();

                /** 加载 PLY 并构建查询场景；成功后可重复执行只读射线查询。 */
                bool loadingMesh(const std::string& mesh_path);

                /** 单点 pixel->world 调试接口；高频路径优先使用 batch 版本。 */
                cv::Point3f pixelToWorld(const cv::Point2f& pixel, 
                             const cv::Mat& K, const cv::Mat& D, 
                             const cv::Mat& R_inv, const cv::Mat& T) const;

                /**
                 * 批量去畸变并求首交点。K/D 是内参，R_inv/T 按 PoseSolver 当前外参
                 * 约定构造世界射线；输出 cv::Point3f 为完整 (world_x,y,z)。
                 */
                std::vector<cv::Point3f> pixelToWorldBatch(
                             const std::vector<cv::Point2f>& pixels,
                             const cv::Mat& K, const cv::Mat& D,
                             const cv::Mat& R_inv, const cv::Mat& T) const;
            private:
                std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_; // 已加载场景。

};









#endif
