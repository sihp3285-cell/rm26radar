// Test-only replacements. The real node, input, configuration, messages and
// thread/service code run unchanged, but no GPU/engine is needed.
#include <radar27_detection/pipeline.hpp>
#include <chrono>
#include <thread>

extern "C" {
cudaError_t CUDARTAPI cudaFree(void*) { return cudaSuccess; }
cudaError_t CUDARTAPI cudaGetDevice(int* device) { *device = 0; return cudaSuccess; }
cudaError_t CUDARTAPI cudaSetDevice(int) { return cudaSuccess; }
const char* CUDARTAPI cudaGetErrorString(cudaError_t) { return "test CUDA stub"; }
}

Model::Model(const std::string, const int&, const float&, const float&, const bool, const ModelType) {}
Model::~Model() = default;

DetectPipeline::DetectPipeline(DetectionConfig& config)
    : detectModel_("", 1, 0.0f, 0.0f), armorDetector_("", 1, 0.0f, 0.0f),
      classifyModel_("", 1, 0.0f, 0.0f), cfg_(config) {}
DetectPipeline::~DetectPipeline() = default;
std::vector<Result> DetectPipeline::process(const cv::Mat& image, float dt)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    Result result;
    result.idx = 2;
    result.box = cv::Rect(static_cast<int>(std::lround(cv::mean(image)[0])), 0, 10, 10);
    result.confidence = 1.0;
    result.class_conf = dt;  // Surface the node's time delta for assertions.
    result.class_margin = cfg_.model.outpostScoreThreshold;
    return {result};
}
void DetectPipeline::resetTimeState() {}
PipelineTiming DetectPipeline::getLatestTiming() const
{
    PipelineTiming timing;
    timing.total_ms = 20.0;
    return timing;
}
