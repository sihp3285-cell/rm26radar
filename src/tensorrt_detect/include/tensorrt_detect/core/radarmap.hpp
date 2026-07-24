#ifndef RADARMAP_HPP
#define RADARMAP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "tracker_types.hpp"

struct Mappoint
{
    cv::Point2f map_point;
    std::string label;
    int classIdx;
    int armorColor;
    bool isDead = false;
    TrackState track_state = TrackState::ACTIVE;  // 用于区分平滑/预测绘制样式
};
class RadarMap
{
    private:
    cv::Mat map;
    float scale_x;
    float scale_y;
    float offset_x;
    float offset_y;
    bool flip_team_ = false;
    bool isCalibrated() const { return m_isCalibrated; }    

    public:
    bool m_isCalibrated = false;
    RadarMap(const std::string& mapPath,const bool isflip);
    void calibrate2(float race_length, float race_width, int map_width, int map_height);
    cv::Point2f worldtomap(const cv::Point2f& worldPoint)const;
    // 用于在 drawMap() 返回的最终视角图像上继续叠加元素。flip_team_ 为真时，
    // drawMap() 已将底图旋转 180°，后绘制元素必须同步做中心对称。
    cv::Point2f worldtomapDisplay(const cv::Point2f& worldPoint)const;
    cv::Mat drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames)const;
    void setFlipTeam(bool flip) { flip_team_ = flip; }
    bool getFlipTeam() const { return flip_team_; }

};
#endif
