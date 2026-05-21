#pragma once
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "ConfigManager.hpp"

struct PipelineTiming {
    double car_ms = 0.0;
    double armor_ms = 0.0;
    double cls_ms = 0.0;
    double outpost_ms = 0.0;
    double airplane_ms = 0.0;
    double total_ms = 0.0;
    double fps = 0.0;
};

class DetectPipeline {
public:

    DetectPipeline(Config& cfg);
    ~DetectPipeline();
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


    std::vector<Result> process(const cv::Mat& frame);
    bool isOutpostAlive() const { return !outpostIsDead_; }
    PipelineTiming getLatestTiming() const;

private:
    Model  detectModel_;
    Model  armorDetector_;
    Model  classifyModel_;
    std::unique_ptr<Model> airplaneModel_;
    Config& cfg_;

    int outpostMissCount_ = 0;
    bool outpostIsDead_ = false;
    cv::Rect outpostLastBox_;

    std::vector<Result>   runDetect(const cv::Mat& frame);
    std::vector<Result>   runArmorDetect(const cv::Mat& frame,
                                         const std::vector<Result>& detections);
    std::vector<Result>   runArmorDetectBatch(const cv::Mat& frame,
                                               const std::vector<Result>& detections,
                                               std::vector<Result>* outposts = nullptr);
    void runClassify(const cv::Mat& frame, std::vector<Result>& detections);
    void runClassifyBatch(const cv::Mat& frame, std::vector<Result>& detections);
    std::vector<Result>   runOutpostDetect(const cv::Mat& frame);
    std::vector<Result>   runAirplaneDetect(const cv::Mat& frame);

    // 异步无人机检测线程
    void airplaneThreadLoop();
    std::thread airplaneThread_;
    std::mutex frameMutex_;
    std::condition_variable airplaneCv_;
    cv::Mat latestFrame_;
    int airplaneRoiX_ = 0;
    bool newFrameAvailable_ = false;
    std::atomic<bool> stopThread_{false};

    std::mutex resultsMutex_;
    std::vector<Result> cachedAirplaneResults_;
    int airplaneIntervalMs_ = 33;

    // 耗时统计
    std::atomic<double> lastAirplaneMs_{0.0};
    double accCarMs_ = 0.0;
    double accArmorMs_ = 0.0;
    double accClsMs_ = 0.0;
    double accOutpostMs_ = 0.0;
    double accTotalMs_ = 0.0;
    int accCount_ = 0;
    std::chrono::steady_clock::time_point lastStatsTime_ = std::chrono::steady_clock::now();

    mutable std::mutex timingMutex_;
    PipelineTiming latestTiming_;

    void updateStats(double carMs, double armorMs, double clsMs, double outpostMs, double totalMs);

};
