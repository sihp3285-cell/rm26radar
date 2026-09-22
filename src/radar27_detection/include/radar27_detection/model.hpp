/**
 * @file model.hpp
 * @brief 单个 TensorRT engine 的资源所有者与推理接口。
 *
 * Model 由 DetectPipeline 创建并独占 runtime/engine/context、CUDA Stream、device
 * bindings 和 pinned host buffer。Detect() 用于检测 engine，predictClass() 用于
 * 分类 engine；它只解释张量，不负责车辆/装甲板之间的业务级组合。
 */
#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <iostream>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <radar27_detection/detection_types.hpp>
#include <cuda_runtime_api.h>

/** Pipeline 各推理阶段共用的目标载体；字段会随阶段逐步补齐。 */




/**
 * 一个已反序列化 TensorRT engine 的完整生命周期。
 *
 * 构造时读取 engine 并分配固定 batch=1 所需的 host/device 缓冲；调用方必须保证
 * 同一实例不被并发调用。析构释放 texture、staging、bindings、event/stream 和
 * TensorRT 对象。跨库 GPU 调度由部署方式决定；此类不依赖定位模块。
 */
class Model
{
private:
    int inputSize;        
    float scoreThreshold; 
    float nmsThreshold;   
    bool isNMS;


    nvinfer1::IRuntime *runtime = nullptr;       // 反序列化 engine 的 runtime。
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr; // 保存张量地址与执行状态。
    cudaStream_t stream = nullptr;               // H2D、kernel、enqueue、D2H 顺序队列。
    cudaEvent_t readyEvent_ = nullptr;
    void *buffers[2] = {nullptr, nullptr};       // GPU input/output tensor bindings。
    // page-locked host output，使异步 D2H 不因可分页内存而退化。
    float* prob_ = nullptr;
    size_t probSize_ = 0;

    int input_h = 0, input_w = 0;
    int output_h = 0, output_w = 0;
    float rx = 0.0f, ry = 0.0f;

    cv::Mat resizeFrame;

    // Tensor names stored for operations
    std::string inputName_;
    std::string outputName_;

    // 原始 8-bit 图像的 GPU staging，按见过的最大帧 grow-on-demand 后复用。
    void* gpuInputBuffer8U_ = nullptr;
    size_t gpuInputCapacity_ = 0;

    // CPU pinned staging 独立于 cv::Mat/ROS 消息生命周期，供异步 H2D 安全读取。
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

    /** 将一帧 BGR 图像异步送入 GPU 并生成 engine 输入张量；工作排入 stream。 */
    void preprocessing(const cv::Mat &frame);
    /** 等待输出 ready event 后，把 host 输出解释为 detectResults。 */
    void postprocessing();

    /** 旧 CPU/OpenCV 单图预处理辅助接口，返回 letterbox 后的 blob 并回填缩放比。 */
    cv::Mat preprocessSingle(const cv::Mat& frame, float& rx, float& ry);
    /** 把单张检测输出矩阵解码、阈值过滤并按配置执行 NMS。 */
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
    /** 反序列化 engine，解析张量维度并一次性分配 CUDA/host 资源；失败抛异常。 */
    Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold, const bool isNMS = true, const ModelType modelType = ModelType::DETECT);
    /** 等待未完成 CUDA 工作并按构造逆序释放 texture、buffer、stream 与 TRT 对象。 */
    ~Model();
    /** 对单个 ROI 执行分类，返回 top1 类别并可选回填 top1 置信度和 top1-top2 margin。 */
    int predictClass(
        const cv::Mat& roi,
        float* top1_conf = nullptr,
        float* class_margin = nullptr
    );
    cv::Rect roi;

    /** 对整帧/ROI 执行检测推理并刷新 detectResults；成功完成推理返回 true。 */
    bool Detect(const cv::Mat &frame);
   };

#endif
