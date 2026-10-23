/**
 * @file mouseback.hpp
 * @brief CalibrateNode 与 ROISetNode 共用的 OpenCV 鼠标像素点采集器。
 * 采点沿用参考仓库的两段式点击：第一次左键把该处放大，第二次左键在放大图上落点，
 * 右键取消放大或撤销上一点。返回值仍是原始图像像素坐标，放大视图可带入亚像素。
 */
#ifndef MOUSEBACK_HPP
#define MOUSEBACK_HPP
#include <opencv2/opencv.hpp>
#include <vector>
class MouseBack
{
    private:
    std::vector<cv::Point2f> points;
    std::string windowName;
    int maxpoints;
    /** 正在放大的图像点，即第一次左键的位置。 */
    cv::Point2f zoomCenter;
    /** 放大倍数：裁剪窗口为画布的 1/zoomFactor，再最近邻铺满画布。 */
    int zoomFactor = 4;
    /** 是否处于放大视图；为 true 时下一次左键用于落点。 */
    bool zoomed = false;
    /** 画布尺寸，等于序列帧尺寸，用于放大窗口与坐标换算。 */
    cv::Size imageSize;
    /** OpenCV C 风格回调入口：左键放大/落点，右键取消放大或撤销上一点。 */
    static void onMouse(int event, int x, int y, int flags, void* userdata);
    /** 按 zoomFactor 与 zoomCenter 求裁剪窗口，夹在图像边界内。 */
    cv::Rect zoomRoi() const;
    /** 画布坐标换算回原始图像坐标；放大视图据此得到亚像素落点。 */
    cv::Point2f toImage(int x, int y) const;
    /** 全景视图：已采点、两点矩形预览与操作提示。 */
    void renderOverview(cv::Mat& canvas) const;
    /** 放大视图：裁剪窗口铺满画布。 */
    void renderZoom(cv::Mat& canvas, const cv::Mat& source) const;
    public:
        /** 绑定窗口名并设置本轮必须采集的像素点数量。 */
        MouseBack(const std::string& windowName,int requirePoints);
        /** 阻塞显示 frame 并收集点击，达到数量或用户取消后返回像素点副本。 */
        std::vector<cv::Point2f> getPoints(const cv::Mat& frame);
};
#endif
