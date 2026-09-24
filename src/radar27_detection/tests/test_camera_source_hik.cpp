// Link-time SDK simulation: real MVS declarations, fake MV_CC_* definitions.
// 被测代码 = production 的 camera_source.cpp + rb26SDK 的 hik.cpp（原样），
// 不链接真库、不访问硬件。
#include "radar27_detection/input/camera_source.hpp"
#include <hik/MvCameraControl.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace radar27_detection::input;
namespace {
std::vector<std::string> events;
bool no_device = false, fail_open = false, fail_start = false, no_frame = false;
// 海康彩色相机默认输出 Bayer：hik.cpp 的 Bayer 分支先做转换（得到自有内存）再释放
// 缓存，本身是安全的。BGR8_Packed 分支相反（先 FreeImageBuffer 再返回缓存视图），
// 用 bayer_format=false 复现。
bool bayer_format = true;
unsigned char pixels[4 * 4 * 3]{};
int value = 10;
void require(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
int count(const char* event) { return std::count(events.begin(), events.end(), event); }
}

extern "C" {
int MV_CC_Initialize() { events.push_back("init"); return 0; }
int MV_CC_Finalize() { events.push_back("finalize"); return 0; }
int MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST* list) {
    static MV_CC_DEVICE_INFO device{};
    std::strcpy(reinterpret_cast<char*>(device.SpecialInfo.stUsb3VInfo.chSerialNumber), "test-camera");
    list->nDeviceNum = no_device ? 0 : 1;
    list->pDeviceInfo[0] = &device;
    return 0;
}
int MV_CC_CreateHandle(void** handle, const MV_CC_DEVICE_INFO*) {
    events.push_back("create"); *handle = pixels; return 0;
}
int MV_CC_OpenDevice(void*, unsigned int, unsigned short) {
    events.push_back("open"); return fail_open ? -1 : 0;
}
int MV_CC_CloseDevice(void*) { events.push_back("close"); return 0; }
int MV_CC_DestroyHandle(void*) { events.push_back("destroy"); return 0; }
int MV_CC_StartGrabbing(void*) { events.push_back("start"); return fail_start ? -1 : 0; }
int MV_CC_StopGrabbing(void*) { events.push_back("stop"); return 0; }
int MV_CC_SetEnumValue(void*, const char*, unsigned int) { return 0; }
int MV_CC_SetFloatValue(void*, const char*, float) { return 0; }
int MV_CC_SetBoolValue(void*, const char*, bool) { return 0; }
int MV_CC_GetFloatValue(void*, const char*, MVCC_FLOATVALUE* range) {
    range->fMin = 0; range->fMax = 10; return 0;
}
int MV_CC_GetIntValueEx(void*, const char*, MVCC_INTVALUE_EX* info) {
    info->nCurValue = 2; return 0;
}
int MV_CC_GetImageBuffer(void*, MV_FRAME_OUT* frame, unsigned int timeout_ms) {
    // rb26SDK 内部把超时写死为 100 ms，适配层无法覆盖：这里固定住这个事实。
    require(timeout_ms == 100, "rb26SDK keeps its hard-coded 100 ms timeout");
    if (no_frame) return MV_E_NODATA;
    events.push_back("acquire");
    std::fill(std::begin(pixels), std::end(pixels), value++);
    frame->pBufAddr = pixels;
    frame->stFrameInfo.nWidth = frame->stFrameInfo.nHeight = 4;
    frame->stFrameInfo.nFrameLen = bayer_format ? 16 : sizeof(pixels);
    frame->stFrameInfo.enPixelType = bayer_format ? PixelType_Gvsp_BayerRG8
                                                  : PixelType_Gvsp_BGR8_Packed;
    return 0;
}
int MV_CC_FreeImageBuffer(void*, MV_FRAME_OUT*) {
    events.push_back("release");
    // 模拟 SDK 把缓冲区回收给下一帧使用。
    std::fill(std::begin(pixels), std::end(pixels), 255);
    return 0;
}
}

int main()
{
    try {
        CameraSourceConfig config;
        config.serial_number = "test-camera";
        CameraSource source(config);
        source.open();
        require(count("start") == 1, "stream started once");

        // 同一进程只允许一个会话（vendor SDK 是进程级全局状态）。
        bool rejected = false;
        try { CameraSource second(config); second.open(); }
        catch (const std::runtime_error&) { rejected = true; }
        require(rejected && count("init") == 1, "concurrent source guard");

        // 暂时取不到图：可重试，且不占用帧号。
        no_frame = true;
        require(source.read().status == ReadStatus::Timeout, "no-data must be retryable");
        no_frame = false;
        auto first = source.read();
        auto next = source.read();
        require(first.status == ReadStatus::Ok && first.frame.sequence == 0, "timeout consumed frame number");
        require(next.status == ReadStatus::Ok && next.frame.sequence == 1, "next frame");
        require(next.frame.source_time_ns >= first.frame.source_time_ns, "monotonic time");

        // 默认（Bayer）路径：hik.cpp 先转换再释放缓存，图像内容必须是采集值。
        require(!first.frame.image.empty() && first.frame.image.u != nullptr,
                "frame must own its pixels");
        require(cv::mean(first.frame.image)[0] == 10, "bayer frame content");

        // 非 Bayer 路径：hik.cpp 先 FreeImageBuffer 再返回缓存视图，适配层能保证的
        // 只是“交出去的像素归自己所有”（不再指向会被下一帧复用的 SDK 内存）；
        // 内容是否已被回收由 vendor 实现决定，这里如实断言这一限制。
        bayer_format = false;
        auto recycled = source.read();
        require(recycled.status == ReadStatus::Ok && recycled.frame.image.u != nullptr,
                "sdk buffer view must be copied into owned memory");
        std::printf("note: BGR8_Packed path content after vendor FreeImageBuffer = %.0f\n",
                    cv::mean(recycled.frame.image)[0]);
        bayer_format = true;

        // 持续取不到图：HikCamera 不暴露错误码，适配层按“连续失败超过宽限期”升级成
        // 粘性错误，避免相机真的掉线时无限重试。
        no_frame = true;
        require(source.read().status == ReadStatus::Timeout, "first failure is still retried");
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        require(source.read().status == ReadStatus::Error, "persistent failure escalates");
        const int acquired_before = count("acquire");
        require(source.read().status == ReadStatus::Error && count("acquire") == acquired_before,
                "sticky error stops acquiring");
        no_frame = false;

        source.close();
        source.close();
        // 清理不变量：先停流、再关设备、再销毁句柄，最后 Finalize，且 Finalize 只一次。
        // （vendor 析构在 capture_stop() 之后还会再 CloseDevice 一次，这是 SDK 原有行为。）
        const auto stop = std::find(events.begin(), events.end(), "stop");
        const auto close = std::find(events.begin(), events.end(), "close");
        const auto destroy = std::find(events.begin(), events.end(), "destroy");
        const auto finalize = std::find(events.begin(), events.end(), "finalize");
        require(stop < close && close < destroy && destroy < finalize, "cleanup order");
        require(count("stop") == 1 && count("destroy") == 1 && count("finalize") == 1,
                "duplicate cleanup");
        require(count("close") >= 1, "device closed");
        require(cv::mean(first.frame.image)[0] == 10, "close invalidated retained frame");

        source.open();
        require(source.read().frame.sequence == 0, "reopen resets sequence");
        source.close();

        // 注入初始化失败：不能吞掉，也不能泄漏会话锁（之后必须还能再开）。
        for (int stage = 0; stage < 3; ++stage) {
            events.clear();
            no_device = stage == 0; fail_open = stage == 1; fail_start = stage == 2;
            bool failed = false;
            try { source.open(); } catch (const std::runtime_error&) { failed = true; }
            require(failed, "injected initialization failure ignored");
            // 初始化失败的 source 不能再被当作可用输入。
            require(source.read().status == ReadStatus::Error, "failed open still readable");
            source.close();
            no_device = fail_open = fail_start = false;
            bool reopened = true;
            try { source.open(); } catch (const std::exception&) { reopened = false; }
            require(reopened, "session lock leaked after failure");
            source.close();
        }
        no_device = fail_open = fail_start = false;

        source.open();
        require(source.read().status == ReadStatus::Ok, "read after recovered open");
        source.close();
        std::cout << "PASS: hik.cpp reuse, ownership, retry policy, cleanup, partial init, reopen\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
