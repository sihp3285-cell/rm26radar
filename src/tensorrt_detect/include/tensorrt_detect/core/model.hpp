#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

struct Result
{
    int idx = 0;  
    float confidence = 0.0f; 
    cv::Rect box; 
    int armorColor = 0;
    cv::Rect car_box{};
    cv::Point2f worldPoint{};
    float fps = 0.0f;
    bool isDead = false;
};




class Model
{
private:
    int inputSize;        
    float scoreThreshold; 
    float nmsThreshold;   
    bool isNMS;


    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
    void *buffers[2] = {nullptr, nullptr};
    std::vector<float> prob;

    int input_h, input_w;
    int output_h, output_w;
    float rx, ry;

    cv::Mat resizeFrame;
    void preprocessing(const cv::Mat &frame);
    void postprocessing();


public:
    enum class ModelType
    {
    DETECT,
    CLASSIFY,
    UNKNOWN
    };
    std::vector<Result> detectResults;
    ModelType modelType;
    Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold, const bool isNMS = true,const ModelType modelType = ModelType::DETECT);
    ~Model();
    int predictClass(const cv::Mat &roi);
    cv::Rect roi;

    bool Detect(const cv::Mat &frame);
   };

#endif