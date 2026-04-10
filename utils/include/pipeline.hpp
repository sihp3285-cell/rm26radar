#pragma once
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "../../ConfigManager.hpp"
class DetectPipeline {
public:

    DetectPipeline(Config& cfg);

    std::vector<Result> process(const cv::Mat& frame);

private:
    Model  detectModel_;
    Model  armorDetector_;
    Model  classifyModel_;
    Config cfg_;

    std::vector<Result>   runDetect(const cv::Mat& frame);
    std::vector<Result>   runArmorDetect(const cv::Mat& frame,
                                         const std::vector<Result>& detections);
    void runClassify(const cv::Mat& frame,std::vector<Result>& detections);
    
};