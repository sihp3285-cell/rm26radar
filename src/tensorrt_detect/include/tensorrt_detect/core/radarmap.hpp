/**
 * @file radarmap.hpp
 * @brief 米制 world(x,z) 到地图像素及最终 OpenCV 底图绘制的封装。
 * MapNode 创建并标定本类；Position Prior overlay 不进入这里的结构化地图结果。
 */
#ifndef RADARMAP_HPP
#define RADARMAP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "tracker_types.hpp"

/** 已完成 world->map 转换、等待渲染的一个目标；map_point 单位为图像像素。 */
struct Mappoint
{
    cv::Point2f map_point;
    std::string label;
    int classIdx;
    int armorColor;
    bool isDead = false;
    TrackState track_state = TrackState::ACTIVE;  // 用于区分平滑/预测绘制样式
};
/**
 * 地图投影与渲染器。calibrate2() 建立米到像素比例后才可使用；flip_team_ 只影响
 * 最终显示视角，不改变上游 world 坐标或结构化目标身份。
 */
class RadarMap
{
    private:
    cv::Mat map;       // 只读底图模板；每帧 drawMap 返回其副本。
    float scale_x;     // 场地米到像素的横向比例。
    float scale_y;     // 场地米到像素的纵向比例。
    float offset_x;    // 配置保留的像素原点偏移。
    float offset_y;
    bool flip_team_ = false;
    /** 返回米制到像素比例是否已经由 calibrate2() 建立。 */
    bool isCalibrated() const { return m_isCalibrated; }    

    public:
    bool m_isCalibrated = false;
    /** 读取底图并记录初始显示方向；图像加载失败时构造失败。 */
    RadarMap(const std::string& mapPath,const bool isflip);
    /** 根据场地米制尺寸与底图像素尺寸建立 x/y 比例及投影有效标志。 */
    void calibrate2(float race_length, float race_width, int map_width, int map_height);
    /** 输入 worldPoint=(world_x,world_z)，输出未做当前 UI 旋转的地图像素。 */
    cv::Point2f worldtomap(const cv::Point2f& worldPoint)const;
    // 用于在 drawMap() 返回的最终视角图像上继续叠加元素。flip_team_ 为真时，
    // drawMap() 已将底图旋转 180°，后绘制元素必须同步做中心对称。
    /** 将 world 点投到 drawMap 最终显示视角；翻转时额外做图像中心对称。 */
    cv::Point2f worldtomapDisplay(const cv::Point2f& worldPoint)const;
    /** 复制底图、绘制所有目标标签/状态并按 flip_team_ 生成最终帧。 */
    cv::Mat drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames)const;
    /** 切换后续 drawMap/worldtomapDisplay 的显示方向，不改变 world 数据。 */
    void setFlipTeam(bool flip) { flip_team_ = flip; }
    /** 返回当前地图显示方向标志。 */
    bool getFlipTeam() const { return flip_team_; }

};
#endif
