/**
 * @file pipeline.hpp
 * @brief DetectNode 内部的多模型业务编排层。
 *
 * 它把车辆检测、车内装甲板检测、兵种分类、固定前哨站 ROI 和可选无人机检测
 * 组织成一帧 Result 列表；TensorRT buffer 细节由 Model 管理，ROS 消息由 Node 转换。
 */
#pragma once
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include "ConfigManager.hpp"

/** 最近统计窗口的阶段耗时快照，供 /pipeline_timing 发布；单位均为毫秒。 */
struct PipelineTiming {
    double car_ms = 0.0;
    double armor_ms = 0.0;
    double cls_ms = 0.0;
    double outpost_ms = 0.0;
    double airplane_ms = 0.0;
    double total_ms = 0.0;
    double end_to_end_ms = 0.0;
    double fps = 0.0;
};

/**
 * 一条 DetectNode 独占的有状态检测管线。
 * Config 由外部持有且必须覆盖本对象生命周期。前哨站 miss duration 和异步无人机
 * 缓存跨帧保存；析构会通知并 join 无人机线程。
 */
class DetectPipeline {
public:

    /** 按 Config 构造各 TensorRT Model，并在启用无人机时启动后台线程。 */
    DetectPipeline(Config& cfg);
    /** 通知无人机线程退出并 join；各 Model 随后自动释放 GPU 资源。 */
    ~DetectPipeline();
    /** 将 YAML 字符串映射为 ModelType；未知值记录错误并返回 UNKNOWN。 */
    Model::ModelType modelType(const std::string& modelType)
    {
        if(modelType == "DETECT")
        {
            return Model::ModelType::DETECT;
        }
        else if(modelType == "CLASSIFY")
        {
            return Model::ModelType::CLASSIFY;
        }
        else
        {
            std::cerr << "错误：未知的类型" << std::endl;
            return Model::ModelType::UNKNOWN;
        }

    }


    /** 执行一帧完整业务流水线并返回最终目标；elapsed_s 驱动前哨站超时状态。 */
    std::vector<Result> process(const cv::Mat& frame, float elapsed_s = -1.0f);
    /** 清零依赖帧间时间的前哨站状态和统计时钟，常用于输入时间回退后的恢复。 */
    void resetTimeState();
    /** 返回固定 ROI 前哨站是否尚未达到连续漏检死亡阈值。 */
    bool isOutpostAlive() const { return !outpostIsDead_; }
    /** 在线程锁保护下返回最近一个统计窗口的阶段耗时副本。 */
    PipelineTiming getLatestTiming() const;

private:
    Model  detectModel_;          // 全图车辆检测。
    Model  armorDetector_;        // 每个车辆 ROI 内的装甲板检测。
    Model  classifyModel_;        // 装甲板/车辆 ROI 的兵种分类。
    std::unique_ptr<Model> airplaneModel_;
    Config& cfg_;

    float outpostMissDurationS_ = 0.0f; // 按消息时间累计，超时才改变存活状态。
    bool outpostIsDead_ = false;
    cv::Rect outpostLastBox_;

    /** 在全帧运行车辆检测模型，返回车辆级 Result。 */
    std::vector<Result>   runDetect(const cv::Mat& frame);
    /** 对车辆 ROI 组装画布并运行装甲板检测，再映射回原图坐标。 */
    std::vector<Result>   runArmorDetect(const cv::Mat& frame,
                                         const std::vector<Result>& detections);
    /** 在固定前哨站 ROI 检测并累计真实 elapsed_s，生成存活/死亡 Result。 */
    std::vector<Result>   detectOutpost(const cv::Mat& frame, float elapsed_s);
    /** 对候选 ROI 运行兵种分类，原地补齐 idx、class_conf 和 class_margin。 */
    void runClassify(const cv::Mat& frame, std::vector<Result>& detections);
    /** 在无人机半幅 ROI 执行检测并把框平移回整帧坐标。 */
    std::vector<Result>   runAirplaneDetect(const cv::Mat& frame);

    /** 等待最新帧快照，按最小间隔运行无人机检测并发布到线程安全缓存。 */
    void airplaneThreadLoop();
    std::thread airplaneThread_;
    std::mutex frameMutex_;
    std::condition_variable airplaneCv_;
    cv::Mat latestFrame_;         // 必须 clone；后台线程不能借用调用者的帧 buffer。
    int airplaneRoiX_ = 0;
    bool newFrameAvailable_ = false;
    std::atomic<bool> stopThread_{false};

    std::mutex resultsMutex_;
    std::vector<Result> cachedAirplaneResults_;
    int airplaneIntervalMs_ = 33;

    // 耗时统计
    std::atomic<double> lastAirplaneMs_{0.0};
    std::atomic<double> lastArmorDetectMs_{0.0};
    std::atomic<double> lastOutpostDetectMs_{0.0};
    double accCarMs_ = 0.0;
    double accArmorMs_ = 0.0;
    double accClsMs_ = 0.0;
    double accOutpostMs_ = 0.0;
    double accTotalMs_ = 0.0;
    int accCount_ = 0;
    std::chrono::steady_clock::time_point lastStatsTime_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastProcessTime_ = std::chrono::steady_clock::now();

    mutable std::mutex timingMutex_;
    PipelineTiming latestTiming_;

    /** 累积各阶段耗时，并周期性计算均值、FPS 后原子式更新 timing 快照。 */
    void updateStats(double carMs, double armorMs, double clsMs, double outpostMs, double totalMs);

};
