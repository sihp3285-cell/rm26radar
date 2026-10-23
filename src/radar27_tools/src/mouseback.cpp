/**
 * @file mouseback.cpp
 * @brief OpenCV 窗口的同步鼠标点采集实现，仅供标定/ROI 旁路工具使用。
 *
 * 参考实现的放大依赖预览降采样；本工具的窗口按原始分辨率显示，因此把裁剪窗口
 * 上采样铺满画布来获得真实放大倍数，落点再按同一比例换算回图像坐标。
 */
#include <radar27_tools/mouseback.hpp>
#include <algorithm>
#include <string>

namespace {
/** 放大倍数范围与步进。 */
constexpr int kMinZoom = 2;
constexpr int kMaxZoom = 10;
constexpr int kZoomStep = 2;
/** 文字先描黑再描亮色，任何底色上都读得清。 */
void putLabel(cv::Mat& canvas, const std::string& text, const cv::Point& org,
              double fontScale, const cv::Scalar& color)
{
    cv::putText(canvas, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), 3);
    cv::putText(canvas, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 1);
}
}

MouseBack::MouseBack(const std::string& windowName, int requirePoints) : windowName(windowName), maxpoints(requirePoints) {}

cv::Rect MouseBack::zoomRoi() const
{
    const int w = std::max(1, imageSize.width / zoomFactor);
    const int h = std::max(1, imageSize.height / zoomFactor);
    const int x = std::clamp(cvRound(zoomCenter.x) - w / 2, 0, std::max(0, imageSize.width - w));
    const int y = std::clamp(cvRound(zoomCenter.y) - h / 2, 0, std::max(0, imageSize.height - h));
    return cv::Rect(x, y, w, h);
}

cv::Point2f MouseBack::toImage(int x, int y) const
{
    if (imageSize.width <= 0 || imageSize.height <= 0) {
        return cv::Point2f(static_cast<float>(x), static_cast<float>(y));
    }
    const cv::Rect roi = zoomRoi();
    // 用实际裁剪尺寸与画布尺寸的比例换算，与 renderZoom 的上采样严格互逆。
    const double sx = static_cast<double>(roi.width) / imageSize.width;
    const double sy = static_cast<double>(roi.height) / imageSize.height;
    return cv::Point2f(static_cast<float>(roi.x + x * sx), static_cast<float>(roi.y + y * sy));
}

void MouseBack::onMouse(int event, int x, int y, int flags, void* userdata)
{
    MouseBack* self = static_cast<MouseBack*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (!self->zoomed)
        {
            // 第一次点击只决定放大位置；点已采满时不再进入放大视图。
            if (self->points.size() < static_cast<size_t>(self->maxpoints))
            {
                self->zoomCenter = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
                self->zoomed = true;
            }
            return;
        }
        if (self->points.size() >= static_cast<size_t>(self->maxpoints))
        {
            return;
        }
        const cv::Point2f pt = self->toImage(x, y);
        self->points.emplace_back(
            std::clamp(pt.x, 0.0f, std::max(0.0f, self->imageSize.width - 1.0f)),
            std::clamp(pt.y, 0.0f, std::max(0.0f, self->imageSize.height - 1.0f)));
        self->zoomed = false;
        return;
    }
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        // 放大视图中右键取消放大，全景视图中右键撤销上一点。
        if (self->zoomed)
        {
            self->zoomed = false;
        }
        else if (!self->points.empty())
        {
            self->points.pop_back();
        }
    }
}

void MouseBack::renderOverview(cv::Mat& canvas) const
{
    for (size_t i = 0; i < points.size(); ++i) {
    cv::circle(canvas, points[i], 10, cv::Scalar(0, 0, 255), -1);
    cv::putText(canvas, std::to_string(i+1), points[i] + cv::Point2f(10, 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }
    // 两点模式下，实时绘制矩形预览
    if (maxpoints == 2 && points.size() == 2) {
        int x1 = static_cast<int>(points[0].x);
        int y1 = static_cast<int>(points[0].y);
        int x2 = static_cast<int>(points[1].x);
        int y2 = static_cast<int>(points[1].y);
        int x = std::min(x1, x2);
        int y = std::min(y1, y2);
        int w = std::abs(x1 - x2);
        int h = std::abs(y1 - y2);
        cv::rectangle(canvas, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
    }
    putLabel(canvas, cv::format("click=zoom %dx | +/- zoom | space/RMB undo | Q cancel", zoomFactor),
             cv::Point(10, 24), 0.5, cv::Scalar(255, 255, 255));
}

void MouseBack::renderZoom(cv::Mat& canvas, const cv::Mat& source) const
{
    const cv::Rect roi = zoomRoi();
    if (roi.width <= 0 || roi.height <= 0 || source.empty()) {
        return;
    }
    // 最近邻放大保留像素块边界，便于把点落在准确像素上。
    cv::Mat view;
    cv::resize(source(roi), view, canvas.size(), 0, 0, cv::INTER_NEAREST);
    view.copyTo(canvas);
    cv::rectangle(canvas, cv::Rect(0, 0, canvas.cols - 1, canvas.rows - 1),
                  cv::Scalar(0, 200, 255), 2);
    putLabel(canvas, cv::format("[ZOOM %dx] click to place | RMB cancel", zoomFactor),
             cv::Point(10, 24), 0.6, cv::Scalar(0, 255, 255));
}

std::vector<cv::Point2f> MouseBack::getPoints(const cv::Mat& frame)
{
    points.clear();
    zoomed = false;
    imageSize = frame.size();
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    struct WindowGuard {
        const std::string& name;
        ~WindowGuard() {
            try { cv::destroyWindow(name); } catch (const cv::Exception&) {}
        }
    } window_guard{windowName};
    cv::setMouseCallback(windowName, onMouse, this);
    while (true)
    {
        cv::Mat canvas = frame.clone();
        if (zoomed) {
            renderZoom(canvas, frame);
        } else {
            renderOverview(canvas);
        }
        cv::imshow(windowName, canvas);
        int key = cv::waitKey(10);
        if (cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) < 1 ||
            key == 'q' || key == 'Q' || key == 27) {
            points.clear();
            break;
        }
        if (points.size() >= static_cast<size_t>(maxpoints)) {
            cv::waitKey(1000);
            break;            
        }
        if (key == '+' || key == '=') {
            zoomFactor = std::min(kMaxZoom, zoomFactor + kZoomStep);
        } else if (key == '-' || key == '_') {
            zoomFactor = std::max(kMinZoom, zoomFactor - kZoomStep);
        }
        if (key == ' ') {
            // 空格与右键同义：放大中先退出放大，否则撤销上一点。
            if (zoomed) {
                zoomed = false;
            } else if (!points.empty()) {
                points.pop_back();

            }
            continue;
        }
    }
    return points;
}
