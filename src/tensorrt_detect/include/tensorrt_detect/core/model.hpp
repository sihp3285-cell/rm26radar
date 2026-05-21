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
    // Pinned host buffers for true async memcpy
    float* prob_ = nullptr;
    size_t probSize_ = 0;
    float* hOutput_ = nullptr;
    size_t hOutputSize_ = 0;

    int input_h, input_w;
    int output_h, output_w;
    float rx, ry;

    cv::Mat resizeFrame;

    // Tensor names stored for batch operations
    std::string inputName_;
    std::string outputName_;

    // Pre-allocated batch buffers (grow-on-demand)
    void* batchInputBuffer_ = nullptr;
    void* batchOutputBuffer_ = nullptr;
    size_t batchInputCapacity_ = 0;
    size_t batchOutputCapacity_ = 0;

    // GPU preprocessing buffers (grow-on-demand)
    void* gpuInputBuffer8U_ = nullptr;
    size_t gpuInputCapacity_ = 0;

    // Batch GPU preprocessing staging buffer
    void* gpuBatchInput8U_ = nullptr;
    size_t gpuBatchInputCapacity_ = 0;

    // Normalization params (ImageNet for classify, identity for detect)
    float mean_[3] = {0.0f, 0.0f, 0.0f};
    float std_[3]  = {1.0f, 1.0f, 1.0f};

    void preprocessing(const cv::Mat &frame);
    void postprocessing();

    cv::Mat preprocessSingle(const cv::Mat& frame, float& rx, float& ry);
    std::vector<Result> postprocessSingle(const cv::Mat& det_output, float rx, float ry);
    std::vector<std::vector<Result>> postprocessBatch(const float* outputData, int batchSize, const std::vector<float>& rxs, const std::vector<float>& rys);
    void ensureBatchBuffers(size_t inputBytes, size_t outputBytes);


public:
    enum class ModelType
    {
    DETECT,
    CLASSIFY,
    UNKNOWN
    };
    std::vector<Result> detectResults;
    ModelType modelType;
    Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold, const bool isNMS = true, const ModelType modelType = ModelType::DETECT);
    ~Model();
    int predictClass(const cv::Mat &roi);
    cv::Rect roi;

    bool Detect(const cv::Mat &frame);

    // Batch methods
    std::vector<int> predictClassBatchSlow(const std::vector<cv::Mat>& rois);
    std::vector<int> predictClassBatch(const std::vector<cv::Mat>& rois);
    std::vector<std::vector<Result>> DetectBatchSlow(const std::vector<cv::Mat>& rois);
    std::vector<std::vector<Result>> DetectBatch(const std::vector<cv::Mat>& rois);
   };

#endif
