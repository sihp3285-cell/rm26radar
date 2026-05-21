#include "raycaster.hpp"

#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/core/Tensor.h>


Raycaster::~Raycaster() = default;
Raycaster::Raycaster() = default;

bool Raycaster::loadingMesh(const std::string& mesh_path){
    std::cout << "尝试加载 3D 网格文件: " << mesh_path << std::endl;
    
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
    
    std::cout << "成功加载网格，顶点数: " << legacy_mesh.vertices_.size() 
              << ", 三角面数: " << legacy_mesh.triangles_.size() << std::endl;

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

            auto result = scene_->CastRays(ray);
            float t_hit = result["t_hit"].Item<float>(); 

            if (std::isinf(t_hit) || std::isnan(t_hit)) return fallback_to_flat_ground();
            

            return cv::Point3f(ox + t_hit * dx, oy + t_hit * dy, oz + t_hit * dz);
        }