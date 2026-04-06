#pragma once
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


struct PipelineConfig {
    int   minRoiSize   = 30;
    float padRatio     = 0.25f;
    int   classIdxBase = 2;

    PipelineConfig() = default;
};

class DetectPipeline {
public:

    DetectPipeline(const std::string& detectPath,
                   const std::string& armorPath,
                   const std::string& classifyPath,
                   const PipelineConfig& cfg = PipelineConfig());

    std::vector<Result> process(const cv::Mat& frame);

private:
    Model  detectModel_;
    Model  armorDetector_;
    Model  classifyModel_;
    PipelineConfig cfg_;

    std::vector<Result>   runDetect(const cv::Mat& frame);
    std::vector<Result>   runArmorDetect(const cv::Mat& frame,
                                         const std::vector<Result>& detections);
    void runClassify(const cv::Mat& frame,std::vector<Result>& detections);
    
};