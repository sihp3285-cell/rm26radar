#ifndef RADARMAP_HPP
#define RADARMAP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
struct Mappoint
{
    cv::Point2f map_point;
    std::string label;
    int classIdx;
    int armorColor;
};
class RadarMap
{
    private:
    cv::Mat map;
    cv::Mat H;
    bool isCalibrated() const { return m_isCalibrated; }    

    public:
    bool m_isCalibrated = false;
    RadarMap(const std::string& mapPath,const bool isflip);
    void calibrate2(const std::vector<cv::Point2f>& mappoints,
                    const std::vector<cv::Point2f>& worldPoints);
    cv::Point2f worldtomap(const cv::Point2f& worldPoint)const;
    cv::Mat drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames)const;

};
#endif