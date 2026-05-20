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
    void runClassify(const cv::Mat& frame,std::vector<Result>& detections);
    std::vector<Result>   runOutpostDetect(const cv::Mat& frame);
    std::vector<Result>   runAirplaneDetect(const cv::Mat& frame);

    // 异步无人机检测线程
    void airplaneThreadLoop();
    std::thread airplaneThread_;
    std::mutex frameMutex_;
    std::condition_variable airplaneCv_;
    cv::Mat latestFrame_;
    bool newFrameAvailable_ = false;
    std::atomic<bool> stopThread_{false};

    std::mutex resultsMutex_;
    std::vector<Result> cachedAirplaneResults_;
    int airplaneIntervalMs_ = 33;

};