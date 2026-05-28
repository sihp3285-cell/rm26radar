#include "raycaster.hpp"
#include "model.hpp"

#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/core/Tensor.h>


Raycaster::~Raycaster() = default;
Raycaster::Raycaster() = default;

bool Raycaster::loadingMesh(const std::string& mesh_path){
    open3d::geometry::TriangleMesh legacy_mesh;
    bool success = open3d::io::ReadTriangleMesh(mesh_path, legacy_mesh);
    
    if (!success) {
        std::cerr << "错误：Open3D 无法读取网格文件: " << mesh_path << std::endl;
        scene_.reset(); 
        return false;
    }
    
    if (legacy_mesh.vertices_.empty()) {
        std::cerr << "错误：网格文件为空或格式不正确" << std::endl;
        scene_.reset();
        return false;
    }

    open3d::t::geometry::TriangleMesh tensor_mesh = open3d::t::geometry::TriangleMesh::FromLegacy(legacy_mesh);

    scene_ = std::make_unique<open3d::t::geometry::RaycastingScene>(); 
    scene_->AddTriangles(tensor_mesh);
    return true;
        }

cv::Point3f Raycaster::pixelToWorld(const cv::Point2f& pixel, 
                             const cv::Mat& K, const cv::Mat& D, 
                             const cv::Mat& R_inv, const cv::Mat& T) const
        {

            std::vector<cv::Point2f> src_pts = {pixel},dst_pts;
            cv::undistortPoints(src_pts,dst_pts,K,D);

            cv::Mat P_c = (cv::Mat_<double>(3, 1) << dst_pts[0].x, dst_pts[0].y, 1.0);

            cv::Mat Ray_world = R_inv * P_c;
            cv::Mat Cam_world = T;

            float ox = Cam_world.at<double>(0), oy = Cam_world.at<double>(1), oz = Cam_world.at<double>(2);
            float dx = Ray_world.at<double>(0), dy = Ray_world.at<double>(1), dz = Ray_world.at<double>(2);

    
            auto fallback_to_flat_ground = [&]() -> cv::Point3f {
            if (std::abs(dy) < 1e-6) return cv::Point3f(0, 0, 0); 
            double t_fb = -oy / dy; 
            return cv::Point3f(ox + t_fb * dx, 0.0f, oz + t_fb * dz);
            };

            if (!scene_) return fallback_to_flat_ground();

            std::vector<float> ray_data = {ox, oy, oz, dx, dy, dz};
            open3d::core::Tensor ray(ray_data,{1,6},open3d::core::Float32);

            float t_hit;
            {
                std::lock_guard<std::mutex> cudaLock(cuda_guard::getCudaMutex());
                auto result = scene_->CastRays(ray);
                t_hit = result["t_hit"].Item<float>();
            }

            if (std::isinf(t_hit) || std::isnan(t_hit)) return fallback_to_flat_ground();
            

            return cv::Point3f(ox + t_hit * dx, oy + t_hit * dy, oz + t_hit * dz);
        }

std::vector<cv::Point3f> Raycaster::pixelToWorldBatch(
    const std::vector<cv::Point2f>& pixels,
    const cv::Mat& K, const cv::Mat& D,
    const cv::Mat& R_inv, const cv::Mat& T) const
{
    std::vector<cv::Point3f> results;
    size_t N = pixels.size();
    if (N == 0) return results;
    results.reserve(N);

    // 1. Batch undistort
    std::vector<cv::Point2f> dst_pts;
    cv::undistortPoints(pixels, dst_pts, K, D);

    // 2. Build rays
    std::vector<float> ray_data;
    ray_data.reserve(N * 6);

    float ox = static_cast<float>(T.at<double>(0));
    float oy = static_cast<float>(T.at<double>(1));
    float oz = static_cast<float>(T.at<double>(2));

    for (size_t i = 0; i < N; ++i) {
        cv::Mat P_c = (cv::Mat_<double>(3, 1) << dst_pts[i].x, dst_pts[i].y, 1.0);
        cv::Mat Ray_world = R_inv * P_c;
        float dx = static_cast<float>(Ray_world.at<double>(0));
        float dy = static_cast<float>(Ray_world.at<double>(1));
        float dz = static_cast<float>(Ray_world.at<double>(2));
        ray_data.push_back(ox);
        ray_data.push_back(oy);
        ray_data.push_back(oz);
        ray_data.push_back(dx);
        ray_data.push_back(dy);
        ray_data.push_back(dz);
    }

    // 3. Batch cast (single CUDA lock)
    std::vector<float> t_hits(N, 0.0f);
    bool has_mesh = (scene_ != nullptr);

    if (has_mesh) {
        open3d::core::Tensor ray(ray_data, {static_cast<int64_t>(N), 6}, open3d::core::Float32);
        std::lock_guard<std::mutex> cudaLock(cuda_guard::getCudaMutex());
        auto result = scene_->CastRays(ray);
        std::vector<float> t_hit_vec = result["t_hit"].ToFlatVector<float>();
        if (t_hit_vec.size() == N) {
            t_hits = std::move(t_hit_vec);
        }
    }

    // 4. Process results
    for (size_t i = 0; i < N; ++i) {
        float dx = ray_data[i * 6 + 3];
        float dy = ray_data[i * 6 + 4];
        float dz = ray_data[i * 6 + 5];

        auto fallback_to_flat_ground = [&]() -> cv::Point3f {
            if (std::abs(dy) < 1e-6f) return cv::Point3f(0, 0, 0);
            double t_fb = -oy / dy;
            return cv::Point3f(
                static_cast<float>(ox + t_fb * dx),
                0.0f,
                static_cast<float>(oz + t_fb * dz));
        };

        if (!has_mesh) {
            results.push_back(fallback_to_flat_ground());
            continue;
        }

        float t_hit = t_hits[i];
        if (std::isinf(t_hit) || std::isnan(t_hit)) {
            results.push_back(fallback_to_flat_ground());
        } else {
            results.push_back(cv::Point3f(
                ox + t_hit * dx,
                oy + t_hit * dy,
                oz + t_hit * dz));
        }
    }

    return results;
}
