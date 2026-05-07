#pragma once
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "ConfigManager.hpp"
class DetectPipeline {
public:

    DetectPipeline(Config& cfg);
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

private:
    Model  detectModel_;
    Model  armorDetector_;
    Model  classifyModel_;
    Config& cfg_;

    int outpostMissCount_ = 0;
    bool outpostIsDead_ = false;
    cv::Rect outpostLastBox_;

    std::vector<Result>   runDetect(const cv::Mat& frame);
    std::vector<Result>   runArmorDetect(const cv::Mat& frame,
                                         const std::vector<Result>& detections);
    void runClassify(const cv::Mat& frame,std::vector<Result>& detections);
    std::vector<Result>   runOutpostDetect(const cv::Mat& frame);

};