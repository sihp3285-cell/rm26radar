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

class Raycaster {
            public:
                
                Raycaster();
                ~Raycaster();

                bool loadingMesh(const std::string& mesh_path);

                cv::Point3f pixelToWorld(const cv::Point2f& pixel, 
                             const cv::Mat& K, const cv::Mat& D, 
                             const cv::Mat& R_inv, const cv::Mat& T) const;
            private:
                std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_;

};









#endif