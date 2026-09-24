#include "radar27_detection/input/camera_source.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#ifdef RADAR27_HAS_HIK
// Hik 复用 rb26SDK 的 sdk::HikCamera（rb26SDK/src/hik/hik.cpp），本文件不再直接
// 调用 MV_CC_*；SDK 头文件与实现都由 rb26SDK 目标提供，那边保持原样不动。
#include <hik/hik.hpp>
#endif
#ifdef RADAR27_HAS_DAHENG
#include <GxIAPI.h>
#endif

namespace radar27_detection::input
{
namespace
{
std::mutex camera_session_mutex;
[[maybe_unused]] void check(int code, const char* operation)
{
    if (code != 0) {
        std::ostringstream message;
        message << operation << " failed: 0x" << std::hex << static_cast<unsigned int>(code);
        throw std::runtime_error(message.str());
    }
}

#ifdef RADAR27_HAS_HIK
// HikCamera::getFrame() 只返回 cv::Mat：失败一律是空 Mat，拿不到 SDK 错误码，
// 所以“暂时没数据”和“相机真出问题”只能由本层按连续失败时长区分。突发丢帧不该
// 让整条流水线退出（detect_node 把 Error 当致命），持续失败则必须上报。
constexpr auto kHikFailureGrace = std::chrono::seconds(1);
#endif
}

struct CameraSource::Impl
{
    // This pipeline supports one source at a time. Prevent one instance from
    // finalizing a process-global vendor SDK while another is using it.
    std::unique_lock<std::mutex> session{camera_session_mutex, std::try_to_lock};
    bool hik = false;
    bool initialized = false;
    bool opened = false;
    bool grabbing = false;
    std::string error;
#ifdef RADAR27_HAS_HIK
    // HikCamera 自己负责 SDK 生命周期（析构里 StopGrabbing/CloseDevice/
    // DestroyHandle/Finalize），这里只持有一个 HikCamera 实例。
    std::unique_ptr<sdk::HikCamera> hik_camera;
    std::chrono::steady_clock::time_point hik_failure_since{};
#endif
#ifdef RADAR27_HAS_DAHENG
    GX_DEV_HANDLE gx_handle = nullptr;
    std::vector<unsigned char> gx_buffer;
#endif

    ~Impl()
    {
#ifdef RADAR27_HAS_DAHENG
        if (!hik) {
            if (grabbing) GXSendCommand(gx_handle, GX_COMMAND_ACQUISITION_STOP);
            if (gx_handle) GXCloseDevice(gx_handle);
            if (initialized) GXCloseLib();
        }
#endif
    }
};

CameraSource::CameraSource(CameraSourceConfig config) : config_(std::move(config)) {}
CameraSource::~CameraSource() = default;

void CameraSource::open()
{
    if (impl_) throw std::logic_error("CameraSource is already open");
    std::string brand = config_.brand;
    std::transform(brand.begin(), brand.end(), brand.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (brand != "hik" && brand != "daheng") {
        throw std::invalid_argument("Unknown camera brand: " + config_.brand);
    }
    if (config_.serial_number.empty() || config_.exposure_time_us <= 0 ||
        !std::isfinite(config_.gain) || config_.gain < 0.0 || config_.gain > 1.0 ||
        !std::isfinite(config_.gamma) || config_.gamma <= 0.0 ||
        config_.timeout_ms == 0 || config_.timeout_ms > 60000) {
        throw std::invalid_argument("Invalid camera serial/exposure/gain/gamma/timeout");
    }

    // Local RAII state cleans up even if initialization fails halfway through.
    auto state = std::make_unique<Impl>();
    if (!state->session.owns_lock()) {
        throw std::runtime_error("Another CameraSource is already open in this process");
    }
    state->hik = brand == "hik";
    if (state->hik) {
#ifdef RADAR27_HAS_HIK
        // 相机 SDK 的初始化、选机、曝光/增益/Gamma/白平衡、取流都交给 rb26SDK 的
        // HikCamera，本层不做任何 MV_CC_* 调用。
        // CameraSDKInit() 的返回值不可靠（SDK 内部把 MV_CC_Initialize 的返回值丢了），
        // 因此这里不据此判失败，真正的失败由 CameraInit() 报告。
        sdk::HikCamera::CameraSDKInit();
        state->hik_camera = std::make_unique<sdk::HikCamera>();
        if (!state->hik_camera->CameraInit(config_.serial_number.data(),
                config_.auto_white_balance, config_.exposure_time_us,
                config_.gain, config_.gamma)) {
            state->hik_camera.reset();
            throw std::runtime_error("Hik CameraInit failed: sn=" + config_.serial_number);
        }
#else
        throw std::runtime_error("Hik support not compiled; enable BUILD_CAMERA and install MVS SDK");
#endif
    } else {
#ifdef RADAR27_HAS_DAHENG
        check(GXInitLib(), "GXInitLib");
        state->initialized = true;
        GX_OPEN_PARAM parameters{};
        parameters.openMode = GX_OPEN_SN;
        parameters.accessMode = GX_ACCESS_EXCLUSIVE;
        parameters.pszContent = config_.serial_number.data();
        check(GXOpenDevice(&parameters, &state->gx_handle), "GXOpenDevice");
        auto handle = state->gx_handle;
        state->opened = true;
        check(GXSetEnum(handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS), "AcquisitionMode");
        check(GXSetEnum(handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF), "TriggerMode");
        check(GXSetEnum(handle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF), "ExposureAuto");
        check(GXSetEnum(handle, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF), "GainAuto");
        check(GXSetEnum(handle, GX_ENUM_BALANCE_WHITE_AUTO, config_.auto_white_balance
              ? GX_BALANCE_WHITE_AUTO_CONTINUOUS : GX_BALANCE_WHITE_AUTO_OFF), "BalanceWhiteAuto");
        check(GXSetFloat(handle, GX_FLOAT_EXPOSURE_TIME, config_.exposure_time_us), "ExposureTime");
        check(GXSetEnum(handle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL), "GainSelector");
        GX_FLOAT_RANGE gain_range{};
        check(GXGetFloatRange(handle, GX_FLOAT_GAIN, &gain_range), "Gain range");
        check(GXSetFloat(handle, GX_FLOAT_GAIN, std::clamp(gain_range.dMax * config_.gain,
              gain_range.dMin, gain_range.dMax)), "Gain");
        check(GXSetBool(handle, GX_BOOL_GAMMA_ENABLE, true), "GammaEnable");
        check(GXSetEnum(handle, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_USER), "Gamma mode");
        check(GXSetFloat(handle, GX_FLOAT_GAMMA, config_.gamma), "Gamma");
        std::int64_t payload = 0;
        check(GXGetInt(handle, GX_INT_PAYLOAD_SIZE, &payload), "PayloadSize");
        if (payload <= 0 || payload > std::numeric_limits<int>::max()) {
            throw std::runtime_error("Invalid Daheng payload size");
        }
        state->gx_buffer.resize(static_cast<std::size_t>(payload));
        check(GXSendCommand(handle, GX_COMMAND_ACQUISITION_START), "AcquisitionStart");
        state->grabbing = true;
#else
        throw std::runtime_error("Daheng support not compiled; enable BUILD_CAMERA and install Galaxy SDK");
#endif
    }
    started_at_ = std::chrono::steady_clock::now();
    next_sequence_ = 0;
    impl_ = std::move(state);
}

ReadResult CameraSource::read()
{
    if (!impl_) return {ReadStatus::Error, {}, "CameraSource is not open"};
    if (!impl_->error.empty()) return {ReadStatus::Error, {}, impl_->error};
    try {
        cv::Mat image;
        std::chrono::steady_clock::time_point received_at;
        if (impl_->hik) {
#ifdef RADAR27_HAS_HIK
            // 翻转/镜像由本层在“确认拥有像素”之后做：HikCamera 内部对非 Bayer
            // 格式可能返回指向 SDK 缓存的视图，原地翻转会写进 SDK 的内存。
            image = impl_->hik_camera->getFrame(false, false);
            if (image.empty()) {
                // HikCamera 用空 Mat 表示一切失败（拿不到 SDK 错误码）：
                // 短时失败按可重试处理，持续失败才升级为粘性错误。
                const auto now = std::chrono::steady_clock::now();
                if (impl_->hik_failure_since == std::chrono::steady_clock::time_point{}) {
                    impl_->hik_failure_since = now;
                }
                if (now - impl_->hik_failure_since < kHikFailureGrace) {
                    return {ReadStatus::Timeout, {}, {}};
                }
                impl_->error = "HikCamera::getFrame returned no image for "
                    + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - impl_->hik_failure_since).count()) + " ms";
                return {ReadStatus::Error, {}, impl_->error};
            }
            impl_->hik_failure_since = {};
            // 取帧时刻只能在 getFrame 返回后采样（HikCamera 不暴露取到 SDK 图像的
            // 时刻），因此这里比旧实现多了颜色转换的时间。
            received_at = std::chrono::steady_clock::now();
            // 非 Bayer 分支返回的是 SDK 缓存上的视图（u == nullptr，不拥有像素），
            // FreeImageBuffer 之后随时会被下一帧覆盖；这里复制成自有内存，
            // 与 Frame“消费者持有期间像素有效”的约定保持一致。
            if (image.u == nullptr) image = image.clone();
            if (config_.flip || config_.mirror) {
                cv::flip(image, image, config_.flip && config_.mirror ? -1 : (config_.flip ? 0 : 1));
            }
#endif
        } else {
#ifdef RADAR27_HAS_DAHENG
            GX_FRAME_DATA raw{};
            raw.pImgBuf = impl_->gx_buffer.data();
            const int code = GXGetImage(impl_->gx_handle, &raw, config_.timeout_ms);
            if (code == GX_STATUS_TIMEOUT) return {ReadStatus::Timeout, {}, {}};
            check(code, "GXGetImage");
            received_at = std::chrono::steady_clock::now();
            if (raw.nStatus != 0 || raw.nWidth <= 0 || raw.nHeight <= 0) {
                throw std::runtime_error("Invalid Daheng frame");
            }
            int conversion = -1;
            int shift = 0;
            switch (raw.nPixelFormat) {
                case GX_PIXEL_FORMAT_MONO8: conversion = cv::COLOR_GRAY2BGR; break;
                case GX_PIXEL_FORMAT_MONO10: conversion = cv::COLOR_GRAY2BGR; shift = 2; break;
                case GX_PIXEL_FORMAT_MONO12: conversion = cv::COLOR_GRAY2BGR; shift = 4; break;
                case GX_PIXEL_FORMAT_BAYER_RG8: conversion = cv::COLOR_BayerRGGB2BGR; break;
                case GX_PIXEL_FORMAT_BAYER_RG10: conversion = cv::COLOR_BayerRGGB2BGR; shift = 2; break;
                case GX_PIXEL_FORMAT_BAYER_RG12: conversion = cv::COLOR_BayerRGGB2BGR; shift = 4; break;
                case GX_PIXEL_FORMAT_BAYER_GR8: conversion = cv::COLOR_BayerGRBG2BGR; break;
                case GX_PIXEL_FORMAT_BAYER_GR10: conversion = cv::COLOR_BayerGRBG2BGR; shift = 2; break;
                case GX_PIXEL_FORMAT_BAYER_GR12: conversion = cv::COLOR_BayerGRBG2BGR; shift = 4; break;
                case GX_PIXEL_FORMAT_BAYER_GB8: conversion = cv::COLOR_BayerGBRG2BGR; break;
                case GX_PIXEL_FORMAT_BAYER_GB10: conversion = cv::COLOR_BayerGBRG2BGR; shift = 2; break;
                case GX_PIXEL_FORMAT_BAYER_GB12: conversion = cv::COLOR_BayerGBRG2BGR; shift = 4; break;
                case GX_PIXEL_FORMAT_BAYER_BG8: conversion = cv::COLOR_BayerBGGR2BGR; break;
                case GX_PIXEL_FORMAT_BAYER_BG10: conversion = cv::COLOR_BayerBGGR2BGR; shift = 2; break;
                case GX_PIXEL_FORMAT_BAYER_BG12: conversion = cv::COLOR_BayerBGGR2BGR; shift = 4; break;
                case GX_PIXEL_FORMAT_RGB8: conversion = cv::COLOR_RGB2BGR; break;
                case GX_PIXEL_FORMAT_BGR8: break;
                default: throw std::runtime_error("Unsupported Daheng pixel format (packed 10/12-bit not supported)");
            }
            const bool color = raw.nPixelFormat == GX_PIXEL_FORMAT_RGB8 || raw.nPixelFormat == GX_PIXEL_FORMAT_BGR8;
            const std::uint64_t bytes = static_cast<std::uint64_t>(raw.nWidth) * raw.nHeight *
                (color ? 3 : (shift ? 2 : 1));
            if (raw.nImgSize < 0 || bytes > static_cast<std::uint64_t>(raw.nImgSize) ||
                bytes > impl_->gx_buffer.size()) throw std::runtime_error("Invalid Daheng frame size");
            cv::Mat view(raw.nHeight, raw.nWidth, color ? CV_8UC3 : (shift ? CV_16UC1 : CV_8UC1), raw.pImgBuf);
            cv::Mat eight_bit;
            if (shift) {
                eight_bit.create(view.size(), CV_8UC1);
                for (int y = 0; y < view.rows; ++y) {
                    const auto* source = view.ptr<std::uint16_t>(y);
                    auto* target = eight_bit.ptr<unsigned char>(y);
                    for (int x = 0; x < view.cols; ++x) target[x] = source[x] >> shift;
                }
                view = eight_bit;
            }
            if (conversion < 0) image = view.clone();
            else cv::cvtColor(view, image, conversion);
#endif
        }
        if (image.empty()) throw std::runtime_error("Camera returned an empty frame");
        // Hik 分支在上面已经翻转过了（必须等确认拥有像素之后再翻）。
        if (!impl_->hik && (config_.flip || config_.mirror)) {
            cv::flip(image, image, config_.flip && config_.mirror ? -1 : (config_.flip ? 0 : 1));
        }
        Frame frame;
        frame.image = std::move(image);
        frame.sequence = next_sequence_++;
        frame.received_at = received_at;
        frame.source_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            received_at - started_at_).count();
        return {ReadStatus::Ok, std::move(frame), {}};
    } catch (const std::exception& error) {
        impl_->error = error.what();
        return {ReadStatus::Error, {}, impl_->error};
    }
}

void CameraSource::close() noexcept
{
    impl_.reset();
    next_sequence_ = 0;
    started_at_ = {};
}

}  // namespace radar27_detection::input
