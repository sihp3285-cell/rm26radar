/**
 * @file ui.hpp
 * @brief 独立调试图窗口：主图 + 雷达图并排显示的最小 OpenCV UI。
 *
 * 与 Qt 显示节点不同，该窗口只服务于 standalone 调试场景，不依赖 ROS：
 * update() 每次把雷达图按主图高度等比缩放后与主图水平拼接显示。
 */
#ifndef __UI_HPP__
#define __UI_HPP__

#include <opencv2/opencv.hpp>
#include <string>
#include "radarmap.hpp"

/** OpenCV namedWindow 封装：主图与雷达图水平拼接后同窗显示。 */
class UI
{
    private:
    std::string windowName;   // imshow 窗口名，构造后不再变化
    cv::Rect btnrect;         // 预留的按钮区域（当前未使用）
    public:
    /** 创建并命名窗口，初始尺寸 1920x720。 */
    UI(const std::string& windowName);
    /** 雷达图按主图高度等比缩放后与主图水平拼接显示；暂停时阻塞等待按键。 */
    int update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused);
};



#endif
