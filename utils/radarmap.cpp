#include "radarmap.hpp"

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
    
    std::cout << "线性变换标定完成:" << std::endl;
    std::cout << "场地物理尺寸: 长(Z)=" << race_length << "米, 宽(X)=" << race_width << "米" << std::endl;
    std::cout << "地图像素尺寸: 宽=" << actual_map_width << ", 高=" << actual_map_height << std::endl;
    std::cout << "缩放因子: scale_x=" << scale_x << " (像素/米) [对应X轴], scale_y=" << scale_y << " (像素/米) [对应Z轴]" << std::endl;
    std::cout << "中心偏移: offset_x=" << offset_x << ", offset_y=" << offset_y << std::endl;
}

cv::Point2f RadarMap::worldtomap(const cv::Point2f& worldPoint)const
{
    cv::Point2f mapPoint;
    
    std::cout << "世界坐标: (" << worldPoint.x << ", " << worldPoint.y << ")" << std::endl;
    std::cout << "缩放因子: scale_x=" << scale_x << ", scale_y=" << scale_y << std::endl;
    std::cout << "中心偏移: offset_x=" << offset_x << ", offset_y=" << offset_y << std::endl;
    
    mapPoint.x = worldPoint.x * scale_x + offset_x;
    mapPoint.y = worldPoint.y * scale_y + offset_y;
    
    std::cout << "地图坐标: (" << mapPoint.x << ", " << mapPoint.y << ")" << std::endl;
    std::cout << "地图尺寸: 宽=" << map.cols << ", 高=" << map.rows << std::endl;
    
    return mapPoint;
}
cv::Mat RadarMap::drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames)const
{
    cv::Mat frame = map.clone();
    for (const auto& mappoint : mappoints)
    {
        cv::Point pt(static_cast<int>(mappoint.map_point.x), 
                     static_cast<int>(mappoint.map_point.y));
                     cv::Scalar drawColor;
        std::string teamPrefix = "";
        if (mappoint.armorColor == 1) {  
            drawColor = cv::Scalar(0, 0, 255); 
        } else if (mappoint.armorColor == 2) { 
            drawColor = cv::Scalar(255, 0, 0);
        } else if (mappoint.armorColor == 0) { 
            drawColor = cv::Scalar(0, 0, 0);
        } else {
            drawColor = cv::Scalar(0, 255, 255);   
        }
        int baseRadius = 4;
        int strokeSize = 1;
        cv::circle(frame, pt, baseRadius + strokeSize, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(frame, pt, baseRadius, drawColor, -1, cv::LINE_AA);
        
        if (mappoint.classIdx >= 0 && mappoint.classIdx < classNames.size())
        {
            std::string Name = classNames[mappoint.classIdx];
            cv::Point textPt(pt.x + 10, pt.y - 10);
            cv::putText(frame, Name, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
            cv::putText(frame, Name, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2, cv::LINE_AA);
        }
    }
    return frame;
}