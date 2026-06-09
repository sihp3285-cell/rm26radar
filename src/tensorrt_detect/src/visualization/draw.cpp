#include "draw.hpp"

void drawDetect(cv::Mat &frame, const std::vector<Result> &results, const std::vector<std::string> &classNames)
{
    drawDetect(frame, results, classNames, 1.0, 1.0);
}

void drawDetect(cv::Mat &frame, const std::vector<Result> &results, const std::vector<std::string> &classNames,
                double scale_x, double scale_y)
{
    int thickness = 1;
    double font_scale = 0.3;

    for (const auto &result : results)
    {
        cv::Scalar carcolor = COLORS[0];
        cv::Scalar othercolor = COLORS[1];
        cv::String label = classNames.at(result.idx % classNames.size());
        cv::Scalar color = (result.idx == 0) ? carcolor : othercolor;

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
        if(result.idx == 1 || result.idx == 7){
            break;
        }
        cv::putText(frame, ss.str(), cv::Point(scaled_box.x + scaled_box.width / 2 + 4, scaled_box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }
}