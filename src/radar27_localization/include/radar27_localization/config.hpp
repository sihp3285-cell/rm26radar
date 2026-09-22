#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/core.hpp>

struct CameraConfig {
    cv::Mat cameraMatrix;                  // 3x3, CV_64F
    cv::Mat distCoeffs;                    // 1xN, CV_64F
    int requirePointsNum = 0;
    std::vector<cv::Point3f> worldPoints;  // PnP 3D 点
    std::string meshPath;
};
struct CalibConfig {
    std::vector<cv::Point2f> imagePoints;
    cv::Mat R;       // 3x3 CV_64F
    cv::Mat T;       // 3x1 CV_64F
    bool valid = false;
};
class LocalizationConfig {
public:
 explicit LocalizationConfig(const std::string& configDir);
CameraConfig camera; CalibConfig calib;
private:
void loadCameraConfig(const std::string& path);
void loadCalibConfig(const std::string& path);
static cv::Mat parseMat3x3(const std::vector<double>& data);
static cv::Mat parseRowMat(const std::vector<double>& data);
static std::vector<cv::Point3f> parsePoint3fList(const std::vector<std::vector<float>>& data);
static std::vector<cv::Point2f> parsePoint2fList(const std::vector<std::vector<float>>& data);
static void validateCameraConfig(const CameraConfig& cfg);
};
