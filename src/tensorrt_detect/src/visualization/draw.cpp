/**
 * @file draw.cpp
 * @brief DetectNode 调试图的目标框、类别和置信度叠加；不影响检测结果消息。
 */
#include "draw.hpp"

void drawDetect(cv::Mat &frame, const std::vector<Result> &results, const std::vector<std::string> &classNames)
{
    // 未缩放场景等价于 scale 1:1，统一走下面的重载
    drawDetect(frame, results, classNames, 1.0, 1.0);
}

void drawDetect(cv::Mat &frame, const std::vector<Result> &results, const std::vector<std::string> &classNames,
                double scale_x, double scale_y)
{
    // 固定线宽与字号：调试图缩小后仍保持清晰
    int thickness = 1;
    double font_scale = 0.3;

    for (const auto &result : results)
    {
        // 车辆(0)白色框，其余目标统一绿色框，两类一眼可区分
        cv::Scalar carcolor = COLORS[0];
        cv::Scalar othercolor = COLORS[1];
        // 取模防止未知 idx 越界（正常不会发生，纯防御）
        cv::String label = classNames.at(result.idx % classNames.size());
        cv::Scalar color = (result.idx == 0) ? carcolor : othercolor;

        // 检测框是原图坐标，缩略调试图上按比例映射后再画
        cv::Rect scaled_box(
            static_cast<int>(result.box.x * scale_x),
            static_cast<int>(result.box.y * scale_y),
            static_cast<int>(result.box.width * scale_x),
            static_cast<int>(result.box.height * scale_y)
        );

        cv::rectangle(frame, scaled_box, color, thickness);
        cv::putText(frame, label, cv::Point(scaled_box.x, scaled_box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << result.confidence;
        // 死亡装甲板(1)与前哨站(7)是状态直通项，confidence 无意义，不画数字
        if(result.idx == 1 || result.idx == 7){
            continue;
        }
        cv::putText(frame, ss.str(), cv::Point(scaled_box.x + scaled_box.width / 2 + 4, scaled_box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }
}
