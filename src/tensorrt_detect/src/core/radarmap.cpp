#include "radarmap.hpp"
#include "robot_id.hpp"

RadarMap::RadarMap(const std::string& mapPath,const bool isflip)
{
    map = cv::imread(mapPath);
    if(isflip)
    {
        cv::rotate(map, map, cv::ROTATE_90_CLOCKWISE);
    }
    else
    {
        cv::rotate(map, map, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
}

void RadarMap::calibrate2(float race_length, float race_width, int map_width, int map_height)
{
    if (race_length <= 0 || race_width <= 0 || map_width <= 0 || map_height <= 0) {
        std::cerr << "错误：场地尺寸和地图尺寸必须大于 0" << std::endl;
        return;
    }

    int actual_map_width = map.cols;
    int actual_map_height = map.rows;

    scale_x = static_cast<float>(actual_map_width) / race_width;
    scale_y = static_cast<float>(actual_map_height) / race_length;
    
    offset_x = actual_map_width / 2.0f;
    offset_y = actual_map_height / 2.0f;

    m_isCalibrated = true;
    

}

cv::Point2f RadarMap::worldtomap(const cv::Point2f& worldPoint)const
{
    cv::Point2f mapPoint;
    mapPoint.x = worldPoint.x * scale_x + offset_x;
    mapPoint.y = worldPoint.y * scale_y + offset_y;
    return mapPoint;
}
cv::Mat RadarMap::drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames)const
{
    cv::Mat frame = map.clone();
    for (const auto& mappoint : mappoints)
    {
        cv::Point pt(static_cast<int>(mappoint.map_point.x), 
                     static_cast<int>(mappoint.map_point.y));

        cv::Scalar drawColor = robot_id::getTeamColor(mappoint.teamId);
        int baseRadius = 4;
        int strokeSize = 1;
        cv::circle(frame, pt, baseRadius + strokeSize, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(frame, pt, baseRadius, drawColor, -1, cv::LINE_AA);
        
        std::string label = robot_id::getRobotLabel(mappoint.teamId, mappoint.classIdx);
        if (!label.empty())
        {
            cv::Point textPt(pt.x + 10, pt.y - 10);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2, cv::LINE_AA);
        }
    }
    return frame;
}
