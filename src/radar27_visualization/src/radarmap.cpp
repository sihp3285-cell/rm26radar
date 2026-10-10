/**
 * @file radarmap.cpp
 * @brief 场地世界平面到地图像素的标定、视角变换和目标图层绘制。
 */
#include <radar27_visualization/radarmap.hpp>
#include <rm_field/robot_id.hpp>

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
    float wx = worldPoint.x;
    float wy = worldPoint.y;
    mapPoint.x = wx * scale_x + offset_x;
    mapPoint.y = wy * scale_y + offset_y;
    return mapPoint;
}

cv::Point2f RadarMap::worldtomapDisplay(const cv::Point2f& worldPoint)const
{
    cv::Point2f mapPoint = worldtomap(worldPoint);
    if (flip_team_) {
        mapPoint.x = static_cast<float>(map.cols - 1) - mapPoint.x;
        mapPoint.y = static_cast<float>(map.rows - 1) - mapPoint.y;
    }
    return mapPoint;
}

cv::Mat RadarMap::drawMap(const std::vector<Mappoint>& mappoints,const std::vector<std::string>& classNames,
                          const radar27_interfaces::msg::MapTactics* tactics)const
{
    cv::Mat frame = map.clone();
    if (flip_team_) {
        cv::rotate(frame, frame, cv::ROTATE_180);
    }
    if (tactics) {
        const cv::Scalar palette[] = {{220,160,50},{80,180,220},{180,90,200},{70,190,110}};
        size_t index = 0;
        for (const auto& r : tactics->regions) {
            if (r.world_x.size() < 3 || r.world_x.size() != r.world_z.size()) continue;
            std::vector<cv::Point> points;
            for (size_t i=0; i<r.world_x.size(); ++i)
                points.push_back(worldtomapDisplay({static_cast<float>(r.world_x[i]), static_cast<float>(r.world_z[i])}));
            const std::vector<std::vector<cv::Point>> polygons{points};
            const auto color = palette[index++ % 4];
            cv::Mat shade = frame.clone();
            cv::fillPoly(shade, polygons, color);
            // A faint boundary band makes the generous membership rule visible.
            const int thickness = std::max(1, static_cast<int>(std::round(
                2*tactics->boundary_margin_m*(scale_x+scale_y)/2)));
            cv::polylines(shade, polygons, true, color, thickness, cv::LINE_AA);
            cv::addWeighted(shade, 0.14, frame, 0.86, 0, frame);
            cv::polylines(frame, polygons, true, color, 1, cv::LINE_AA);
            cv::Point center;
            for (const auto& point : points) center += point;
            center.x /= points.size(); center.y /= points.size();
            const std::string label = r.id + ":" + (tactics->input_fresh ? std::to_string(r.enemy_count) : "?");
            center.x = std::clamp(center.x, 2, std::max(2,frame.cols-static_cast<int>(label.size()*5)));
            center.y = std::clamp(center.y, 12, frame.rows-3);
            cv::putText(frame,label,center,cv::FONT_HERSHEY_SIMPLEX,0.28,cv::Scalar(255,255,255),3,cv::LINE_AA);
            cv::putText(frame,label,center,cv::FONT_HERSHEY_SIMPLEX,0.28,color,1,cv::LINE_AA);
        }
    }
    for (const auto& mappoint : mappoints)
    {
        if (std::isnan(mappoint.map_point.x) || std::isnan(mappoint.map_point.y)) continue;
        cv::Point pt(static_cast<int>(mappoint.map_point.x),
                     static_cast<int>(mappoint.map_point.y));
        cv::Scalar drawColor;
        if (mappoint.isDead) {
            drawColor = cv::Scalar(0, 0, 0);
        } else if (mappoint.armorColor == robot_id::RED) {
            drawColor = cv::Scalar(0, 0, 255);
        } else if (mappoint.armorColor == robot_id::BLUE) {
            drawColor = cv::Scalar(255, 0, 0);
        } else {
            drawColor = cv::Scalar(0, 255, 255);
        }

        // 统一绘制样式（白边 + 纯色圆点）
        int baseRadius = 6;
        int strokeSize = 2;
        cv::circle(frame, pt, baseRadius + strokeSize, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(frame, pt, baseRadius, drawColor, -1, cv::LINE_AA);
        
        std::string label;
        if (mappoint.isDead) {
            label = "dead";
        } else if (mappoint.classIdx >= 0 && mappoint.classIdx < classNames.size()) {
            label = classNames[mappoint.classIdx];
        }

        if (!label.empty())
        {
            cv::Point textPt(pt.x + 10, pt.y - 10);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8, drawColor, 2, cv::LINE_AA);
        }
    }
    return frame;
}
