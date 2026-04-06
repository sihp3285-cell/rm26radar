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

void RadarMap::calibrate2(const std::vector<cv::Point2f>& mappoints,
                            const std::vector<cv::Point2f>& worldPoints)
{

    H = cv::findHomography(worldPoints, mappoints, cv::RANSAC); 
    m_isCalibrated = true;
} 

cv::Point2f RadarMap::worldtomap(const cv::Point2f& worldPoint)const
{
    std::vector<cv::Point2f> worldPoints = {worldPoint};
    std::vector<cv::Point2f> mapPoints;
    cv::perspectiveTransform(worldPoints, mapPoints, H);
    return mapPoints[0];
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
        int baseRadius = 8;
        int strokeSize = 2;
        cv::circle(frame, pt, baseRadius + strokeSize, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(frame, pt, baseRadius, drawColor, -1, cv::LINE_AA);
        
        if (mappoint.classIdx >= 0 && mappoint.classIdx < classNames.size())
        {
            std::string Name = classNames[mappoint.classIdx];
            cv::Point textPt(pt.x + 10, pt.y - 10);
            cv::putText(frame, Name, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
            cv::putText(frame, Name, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8, drawColor, 2, cv::LINE_AA);
        }
    }
    return frame;
}

 