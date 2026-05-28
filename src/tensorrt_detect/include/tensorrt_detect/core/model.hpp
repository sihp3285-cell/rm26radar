#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <iostream>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

// 全局 CUDA 互斥锁：序列化进程内所有 CUDA 操作（TensorRT 推理 + Open3D Raycasting）
// 解决 component_container_mt 多线程并发 CUDA 操作导致的 SIGSEGV
namespace cuda_guard {
    inline std::mutex& getCudaMutex() {
        static std::mutex mtx;
        return mtx;
    }
}

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
    cudaEvent_t readyEvent_ = nullptr;
    void *buffers[2] = {nullptr, nullptr};
    // Pinned host buffers for true async memcpy
    float* prob_ = nullptr;
    size_t probSize_ = 0;

    int input_h = 0, input_w = 0;
    int output_h = 0, output_w = 0;
    float rx = 0.0f, ry = 0.0f;

    cv::Mat resizeFrame;

    // Tensor names stored for operations
    std::string inputName_;
    std::string outputName_;

    // GPU preprocessing buffers (grow-on-demand)
    void* gpuInputBuffer8U_ = nullptr;
    size_t gpuInputCapacity_ = 0;

    // Pinned CPU staging buffer for async H2D
    uint8_t* hInputBuffer8U_ = nullptr;
    size_t hInputCapacity_ = 0;

    // CUDA texture object for hardware-accelerated bilinear interpolation
    cudaTextureObject_t inputTex_ = 0;
    int texSrcW_ = 0;
    int texSrcH_ = 0;
    int texSrcStep_ = 0;

    // Normalization params (ImageNet for classify, identity for detect)
    float mean_[3] = {0.0f, 0.0f, 0.0f};
    float std_[3]  = {1.0f, 1.0f, 1.0f};

    void preprocessing(const cv::Mat &frame);
    void postprocessing();

    cv::Mat preprocessSingle(const cv::Mat& frame, float& rx, float& ry);
    std::vector<Result> postprocessSingle(const cv::Mat& det_output, float rx, float ry);


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
   };

#endif
