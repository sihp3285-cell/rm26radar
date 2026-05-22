
#include <fstream>
#include <algorithm>
#include <numeric>

#include "model.hpp"
#include "preprocess.hpp"

class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kINFO)
            std::cout << msg << std::endl;
    }
} gLogger;

Model::Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold, const bool isNMS, const ModelType modelType)
{
    this->inputSize = inputSize;
    this->scoreThreshold = scoreThreshold;
    this->nmsThreshold = nmsThreshold;
    this->isNMS = isNMS;
    this->modelType = modelType;

    // 确保 CUDA primary context 提前初始化，避免多线程并发初始化导致 SIGSEGV
    cudaError_t initErr = cudaFree(0);
    if (initErr != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA 初始化失败: ") + cudaGetErrorString(initErr));
    }

    std::ifstream engineFile(modelPath, std::ios::binary);
    std::vector<char> engineData;
    int fsize = 0;

    if (engineFile.good())
    {
        engineFile.seekg(0, engineFile.end);
        fsize = engineFile.tellg();
        engineFile.seekg(0, engineFile.beg);
        engineData.resize(fsize);
        engineFile.read(engineData.data(), fsize);
        engineFile.close();
    }

    int deviceCount = 0;
    cudaError_t cudaErr = cudaGetDeviceCount(&deviceCount);
    if (cudaErr != cudaSuccess || deviceCount == 0) {
        throw std::runtime_error("没有可用的 CUDA 设备，无法加载 TensorRT 模型");
    }

    this->runtime = nvinfer1::createInferRuntime(gLogger);
    if (!this->runtime) {
        throw std::runtime_error("TensorRT runtime 创建失败，请检查 CUDA 驱动和 GPU 可用性");
    }

    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    if (!this->engine) {
        throw std::runtime_error("TensorRT engine 反序列化失败，请检查模型文件是否有效");
    }

    this->context = this->engine->createExecutionContext();
    if (!this->context) {
        throw std::runtime_error("TensorRT execution context 创建失败");
    }

    int nbTensors = this->engine->getNbIOTensors();
    for (int i = 0; i < nbTensors; ++i) {
        const char* name = this->engine->getIOTensorName(i);
        if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            inputName_ = name;
        } else if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
            outputName_ = name;
        }
    }

    nvinfer1::Dims inputDims = this->engine->getTensorShape(inputName_.c_str());
    nvinfer1::Dims outputDims = this->engine->getTensorShape(outputName_.c_str());

    this->input_h = inputDims.d[2];
    this->input_w = inputDims.d[3];

    if (outputDims.nbDims == 2) {
        this->output_h = outputDims.d[0]; 
        this->output_w = outputDims.d[1]; 
    } else {
        this->output_h = outputDims.d[1];
        this->output_w = outputDims.d[2];
    }


    cudaMalloc(&(this->buffers[0]), this->input_h * this->input_w * 3 * sizeof(float));
    cudaMalloc(&(this->buffers[1]), this->output_h * this->output_w * sizeof(float));
    this->probSize_ = this->output_h * this->output_w;
    cudaMallocHost(&(this->prob_), this->probSize_ * sizeof(float));
    cudaStreamCreate(&(this->stream));

    // Initialize normalization params
    if (this->output_h == 1 || this->output_w == 1) {
        mean_[0] = 0.485f; mean_[1] = 0.456f; mean_[2] = 0.406f;
        std_[0]  = 0.229f; std_[1]  = 0.224f; std_[2]  = 0.225f;
    } else {
        mean_[0] = 0.0f; mean_[1] = 0.0f; mean_[2] = 0.0f;
        std_[0]  = 1.0f; std_[1]  = 1.0f; std_[2]  = 1.0f;
    }
}

Model::~Model()
{
    for (auto &buffer : this->buffers)
    {
        if (buffer)
            cudaFree(buffer);
    }
    if (batchInputBuffer_)
        cudaFree(batchInputBuffer_);
    if (batchOutputBuffer_)
        cudaFree(batchOutputBuffer_);
    if (gpuInputBuffer8U_)
        cudaFree(gpuInputBuffer8U_);
    if (gpuBatchInput8U_)
        cudaFree(gpuBatchInput8U_);
    if (prob_)
        cudaFreeHost(prob_);
    if (hOutput_)
        cudaFreeHost(hOutput_);
    if (this->context)
        delete this->context;
    if (this->engine)
        delete this->engine;
    if (this->runtime)
        delete this->runtime;
    if (this->stream)
        cudaStreamDestroy(this->stream);
}

cv::Mat Model::preprocessSingle(const cv::Mat& frame, float& rx, float& ry)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);

    cv::Mat resized_img;
    cv::resize(frame, resized_img, cv::Size(new_w, new_h));

    cv::Mat canvas = cv::Mat::zeros(this->input_h, this->input_w, CV_8UC3);
    canvas.setTo(cv::Scalar(114, 114, 114));
    
    resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));

    rx = (float)img_w / new_w;
    ry = (float)img_h / new_h;

    cv::Mat blob = cv::dnn::blobFromImage(canvas, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
    if (this->output_h == 1 || this->output_w == 1) { 
        float mean[] = {0.485, 0.456, 0.406};
        float std[] = {0.229, 0.224, 0.225};
        float* data = (float*)blob.data;
        int pixels_per_channel = this->input_w * this->input_h;
        for (int c = 0; c < 3; ++c) {
            for (int i = 0; i < pixels_per_channel; ++i) {
                data[c * pixels_per_channel + i] = (data[c * pixels_per_channel + i] - mean[c]) / std[c];
            }
        }
    }
    return blob;
}

void Model::preprocessing(const cv::Mat &frame)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);

    this->rx = (float)img_w / new_w;
    this->ry = (float)img_h / new_h;

    // Ensure device-side raw image buffer
    size_t imgBytes = frame.step[0] * frame.rows;
    if (gpuInputCapacity_ < imgBytes) {
        if (gpuInputBuffer8U_) cudaFree(gpuInputBuffer8U_);
        gpuInputBuffer8U_ = nullptr;
        cudaError_t err = cudaMalloc(&gpuInputBuffer8U_, imgBytes);
        if (err != cudaSuccess) {
            gpuInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc gpuInputBuffer8U_ failed: ") + cudaGetErrorString(err));
        }
        gpuInputCapacity_ = imgBytes;
    }

    // H2D copy (asynchronous)
    cudaMemcpyAsync(gpuInputBuffer8U_, frame.data, imgBytes, cudaMemcpyHostToDevice, this->stream);

    // Launch GPU preprocess kernel directly into model input buffer
    launch_preprocess(
        static_cast<const uint8_t*>(gpuInputBuffer8U_),
        img_w, img_h, static_cast<int>(frame.step[0]),
        static_cast<float*>(buffers[0]),
        input_w, input_h,
        static_cast<float>(img_w) / new_w,
        static_cast<float>(img_h) / new_h,
        new_w, new_h,
        mean_, std_,
        true,   // swapRB (same as cv::dnn::blobFromImage behavior)
        this->stream
    );
}

std::vector<Result> Model::postprocessSingle(const cv::Mat& det_output, float rx, float ry)
{
    std::vector<Result> results;
    
    if (this->modelType == ModelType::CLASSIFY) {
        return results;
    }

    if (this->isNMS) 
    {
        bool is_transposed = (det_output.rows < det_output.cols); 
        int num_boxes = is_transposed ? det_output.cols : det_output.rows;

        for (int i = 0; i < num_boxes; ++i)
        {
            float x1, y1, x2, y2, score;
            int class_id;

            if (!is_transposed) {
                score = det_output.at<float>(i, 4);
                if (score <= this->scoreThreshold) continue;
                x1 = det_output.at<float>(i, 0);
                y1 = det_output.at<float>(i, 1);
                x2 = det_output.at<float>(i, 2);
                y2 = det_output.at<float>(i, 3);
                class_id = static_cast<int>(det_output.at<float>(i, 5));
            } else {
                score = det_output.at<float>(4, i);
                if (score <= this->scoreThreshold) continue;
                x1 = det_output.at<float>(0, i);
                y1 = det_output.at<float>(1, i);
                x2 = det_output.at<float>(2, i);
                y2 = det_output.at<float>(3, i);
                class_id = static_cast<int>(det_output.at<float>(5, i));
            }

            cv::Rect box;
            box.x = static_cast<int>(x1 * rx);
            box.y = static_cast<int>(y1 * ry);
            box.width = static_cast<int>((x2 - x1) * rx);
            box.height = static_cast<int>((y2 - y1) * ry);

            results.emplace_back(Result{class_id, score, box});
        }
    } 
    else 
    {
        std::vector<cv::Rect> boxes;
        std::vector<int> classIds;
        std::vector<float> confidences;

        bool is_transposed = (det_output.rows > det_output.cols); 
        int num_anchors = is_transposed ? det_output.rows : det_output.cols;
        int num_properties = is_transposed ? det_output.cols : det_output.rows;

        for (int idx = 0; idx < num_anchors; ++idx)
        {
            float cx, cy, ow, oh, max_score = 0.0f;
            int class_id = 0;

            if (!is_transposed) {
                cx = det_output.at<float>(0, idx);
                cy = det_output.at<float>(1, idx);
                ow = det_output.at<float>(2, idx);
                oh = det_output.at<float>(3, idx);

                if (num_properties == 5) { 
                    max_score = det_output.at<float>(4, idx);
                    class_id = 0;
                } else { 
                    cv::Mat scores = det_output.col(idx).rowRange(4, num_properties);
                    cv::Point class_id_point;
                    double score_double;
                    cv::minMaxLoc(scores, nullptr, &score_double, nullptr, &class_id_point);
                    max_score = (float)score_double;
                    class_id = class_id_point.y;
                }
            } else {
                cx = det_output.at<float>(idx, 0);
                cy = det_output.at<float>(idx, 1);
                ow = det_output.at<float>(idx, 2);
                oh = det_output.at<float>(idx, 3);

                if (num_properties == 5) {
                    max_score = det_output.at<float>(idx, 4);
                    class_id = 0;
                } else { 
                    cv::Mat scores = det_output.row(idx).colRange(4, num_properties);
                    cv::Point class_id_point;
                    double score_double;
                    cv::minMaxLoc(scores, nullptr, &score_double, nullptr, &class_id_point);
                    max_score = (float)score_double;
                    class_id = class_id_point.x; 
                }
            }

            if (max_score > this->scoreThreshold) {
                cv::Rect box;
                box.x = static_cast<int>((cx - 0.5 * ow) * rx);
                box.y = static_cast<int>((cy - 0.5 * oh) * ry);
                box.width = static_cast<int>(ow * rx);
                box.height = static_cast<int>(oh * ry);

                boxes.push_back(box);
                classIds.push_back(class_id);
                confidences.push_back(max_score);
            }
        }

        std::vector<int> indexes;
        cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, indexes);
        for (int idx : indexes) {
            results.emplace_back(Result{classIds[idx], confidences[idx], boxes[idx]});
        }
    }
    
    return results;
}

void Model::postprocessing()
{
    if (this->modelType == ModelType::CLASSIFY) {
        return;
    }

    this->detectResults.clear();
    cv::Mat det_output(this->output_h, this->output_w, CV_32F, prob_);
    this->detectResults = postprocessSingle(det_output, this->rx, this->ry);
}

std::vector<std::vector<Result>> Model::postprocessBatch(const float* outputData, int batchSize, const std::vector<float>& rxs, const std::vector<float>& rys)
{
    std::vector<std::vector<Result>> batchResults(batchSize);
    if (this->modelType == ModelType::CLASSIFY) {
        return batchResults;
    }
    for (int b = 0; b < batchSize; ++b) {
        const float* sampleData = outputData + b * output_h * output_w;
        cv::Mat det_output(this->output_h, this->output_w, CV_32F, const_cast<float*>(sampleData));
        batchResults[b] = postprocessSingle(det_output, rxs[b], rys[b]);
    }
    return batchResults;
}

bool Model::Detect(const cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        context->setTensorAddress(inputName_.c_str(), buffers[0]);
        context->setTensorAddress(outputName_.c_str(), buffers[1]);
        nvinfer1::Dims4 inputDims(1, 3, input_h, input_w);
        context->setInputShape(inputName_.c_str(), inputDims);

        if (!context->enqueueV3(this->stream)) {
            throw std::runtime_error("enqueueV3 failed in Detect");
        }

        cudaMemcpyAsync(this->prob_, this->buffers[1], this->probSize_ * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
        cudaStreamSynchronize(this->stream);

        postprocessing();

        return !detectResults.empty();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}
int Model::predictClass(const cv::Mat &roi) {
    preprocessing(roi);
    context->setTensorAddress(inputName_.c_str(), buffers[0]);
    context->setTensorAddress(outputName_.c_str(), buffers[1]);
    nvinfer1::Dims4 inputDims(1, 3, input_h, input_w);
    context->setInputShape(inputName_.c_str(), inputDims);

    if (!context->enqueueV3(this->stream)) {
        throw std::runtime_error("enqueueV3 failed in predictClass");
    }

    cudaMemcpyAsync(this->prob_, this->buffers[1], this->probSize_ * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
    cudaStreamSynchronize(this->stream);
    float* data = this->prob_;
    return std::max_element(data, data + probSize_) - data;
}

std::vector<int> Model::predictClassBatchSlow(const std::vector<cv::Mat>& rois) {
    std::vector<int> results;
    results.reserve(rois.size());
    for (const auto& roi : rois) {
        results.push_back(predictClass(roi));
    }
    return results;
}

std::vector<std::vector<Result>> Model::DetectBatchSlow(const std::vector<cv::Mat>& rois) {
    std::vector<std::vector<Result>> results;
    results.reserve(rois.size());
    for (const auto& roi : rois) {
        Detect(roi);
        results.push_back(detectResults);
    }
    return results;
}

void Model::ensureBatchBuffers(size_t inputBytes, size_t outputBytes) {
    if (batchInputCapacity_ < inputBytes) {
        if (batchInputBuffer_) cudaFree(batchInputBuffer_);
        batchInputBuffer_ = nullptr;
        cudaError_t err = cudaMalloc(&batchInputBuffer_, inputBytes);
        if (err != cudaSuccess) {
            batchInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc batchInputBuffer_ failed: ") + cudaGetErrorString(err));
        }
        batchInputCapacity_ = inputBytes;
    }
    if (batchOutputCapacity_ < outputBytes) {
        if (batchOutputBuffer_) cudaFree(batchOutputBuffer_);
        batchOutputBuffer_ = nullptr;
        cudaError_t err = cudaMalloc(&batchOutputBuffer_, outputBytes);
        if (err != cudaSuccess) {
            batchOutputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc batchOutputBuffer_ failed: ") + cudaGetErrorString(err));
        }
        batchOutputCapacity_ = outputBytes;
    }
}

std::vector<int> Model::predictClassBatch(const std::vector<cv::Mat>& rois) {
    if (rois.empty()) return {};
    
    int N = static_cast<int>(rois.size());
    
    nvinfer1::Dims inputDims = engine->getTensorShape(inputName_.c_str());
    bool hasDynamicBatch = (inputDims.nbDims > 0 && inputDims.d[0] == -1);
    
    if (!hasDynamicBatch) {
        return predictClassBatchSlow(rois);
    }
    
    size_t inputBytes = N * 3 * input_h * input_w * sizeof(float);
    size_t outputBytes = N * output_h * output_w * sizeof(float);
    ensureBatchBuffers(inputBytes, outputBytes);
    
    // Ensure device-side raw image staging buffer for all ROIs
    size_t totalImgBytes = 0;
    for (const auto& roi : rois) {
        totalImgBytes += roi.step[0] * roi.rows;
    }
    if (gpuBatchInputCapacity_ < totalImgBytes) {
        if (gpuBatchInput8U_) cudaFree(gpuBatchInput8U_);
        gpuBatchInput8U_ = nullptr;
        cudaError_t err = cudaMalloc(&gpuBatchInput8U_, totalImgBytes);
        if (err != cudaSuccess) {
            gpuBatchInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc gpuBatchInput8U_ failed: ") + cudaGetErrorString(err));
        }
        gpuBatchInputCapacity_ = totalImgBytes;
    }
    
    uint8_t* gpuImgPtr = static_cast<uint8_t*>(gpuBatchInput8U_);
    for (int i = 0; i < N; ++i) {
        const cv::Mat& roi = rois[i];
        size_t roiBytes = roi.step[0] * roi.rows;
        cudaMemcpyAsync(gpuImgPtr, roi.data, roiBytes, cudaMemcpyHostToDevice, stream);
        
        float scale = std::min((float)input_w / roi.cols, (float)input_h / roi.rows);
        int new_w = int(roi.cols * scale);
        int new_h = int(roi.rows * scale);
        
        float* dstPtr = static_cast<float*>(batchInputBuffer_) + i * 3 * input_h * input_w;
        launch_preprocess(
            gpuImgPtr, roi.cols, roi.rows, static_cast<int>(roi.step[0]),
            dstPtr, input_w, input_h,
            static_cast<float>(roi.cols) / new_w,
            static_cast<float>(roi.rows) / new_h,
            new_w, new_h,
            mean_, std_, true, stream
        );
        
        gpuImgPtr += roiBytes;
    }
    
    bool ok = true;
    ok = ok && context->setTensorAddress(inputName_.c_str(), batchInputBuffer_);
    ok = ok && context->setTensorAddress(outputName_.c_str(), batchOutputBuffer_);
    
    nvinfer1::Dims4 actualInputDims(N, 3, input_h, input_w);
    ok = ok && context->setInputShape(inputName_.c_str(), actualInputDims);
    
    if (!ok || !context->enqueueV3(stream)) {
        std::cerr << "Batch classification inference failed, falling back to slow path" << std::endl;
        nvinfer1::Dims4 resetInputDims(1, 3, input_h, input_w);
        context->setInputShape(inputName_.c_str(), resetInputDims);
        return predictClassBatchSlow(rois);
    }
    
    size_t requiredOutput = N * output_h * output_w;
    if (hOutputSize_ < requiredOutput) {
        if (hOutput_) cudaFreeHost(hOutput_);
        cudaMallocHost(&hOutput_, requiredOutput * sizeof(float));
        hOutputSize_ = requiredOutput;
    }
    cudaMemcpyAsync(hOutput_, batchOutputBuffer_, outputBytes, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    
    nvinfer1::Dims4 resetInputDims(1, 3, input_h, input_w);
    context->setInputShape(inputName_.c_str(), resetInputDims);
    
    std::vector<int> results(N);
    int classesPerSample = output_h * output_w;
    for (int i = 0; i < N; ++i) {
        const float* data = hOutput_ + i * classesPerSample;
        results[i] = std::max_element(data, data + classesPerSample) - data;
    }
    return results;
}

std::vector<std::vector<Result>> Model::DetectBatch(const std::vector<cv::Mat>& rois) {
    if (rois.empty()) return {};
    
    int N = static_cast<int>(rois.size());
    
    nvinfer1::Dims inputDims = engine->getTensorShape(inputName_.c_str());
    bool hasDynamicBatch = (inputDims.nbDims > 0 && inputDims.d[0] == -1);
    
    if (!hasDynamicBatch) {
        return DetectBatchSlow(rois);
    }
    
    size_t inputBytes = N * 3 * input_h * input_w * sizeof(float);
    size_t outputBytes = N * output_h * output_w * sizeof(float);
    ensureBatchBuffers(inputBytes, outputBytes);
    
    // Ensure device-side raw image staging buffer for all ROIs
    size_t totalImgBytes = 0;
    for (const auto& roi : rois) {
        totalImgBytes += roi.step[0] * roi.rows;
    }
    if (gpuBatchInputCapacity_ < totalImgBytes) {
        if (gpuBatchInput8U_) cudaFree(gpuBatchInput8U_);
        gpuBatchInput8U_ = nullptr;
        cudaError_t err = cudaMalloc(&gpuBatchInput8U_, totalImgBytes);
        if (err != cudaSuccess) {
            gpuBatchInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc gpuBatchInput8U_ failed: ") + cudaGetErrorString(err));
        }
        gpuBatchInputCapacity_ = totalImgBytes;
    }
    
    std::vector<float> rxs(N), rys(N);
    uint8_t* gpuImgPtr = static_cast<uint8_t*>(gpuBatchInput8U_);
    for (int i = 0; i < N; ++i) {
        const cv::Mat& roi = rois[i];
        size_t roiBytes = roi.step[0] * roi.rows;
        cudaMemcpyAsync(gpuImgPtr, roi.data, roiBytes, cudaMemcpyHostToDevice, stream);
        
        float scale = std::min((float)input_w / roi.cols, (float)input_h / roi.rows);
        int new_w = int(roi.cols * scale);
        int new_h = int(roi.rows * scale);
        rxs[i] = (float)roi.cols / new_w;
        rys[i] = (float)roi.rows / new_h;
        
        float* dstPtr = static_cast<float*>(batchInputBuffer_) + i * 3 * input_h * input_w;
        launch_preprocess(
            gpuImgPtr, roi.cols, roi.rows, static_cast<int>(roi.step[0]),
            dstPtr, input_w, input_h,
            static_cast<float>(roi.cols) / new_w,
            static_cast<float>(roi.rows) / new_h,
            new_w, new_h,
            mean_, std_, true, stream
        );
        
        gpuImgPtr += roiBytes;
    }
    
    bool ok = true;
    ok = ok && context->setTensorAddress(inputName_.c_str(), batchInputBuffer_);
    ok = ok && context->setTensorAddress(outputName_.c_str(), batchOutputBuffer_);
    
    nvinfer1::Dims4 actualInputDims(N, 3, input_h, input_w);
    ok = ok && context->setInputShape(inputName_.c_str(), actualInputDims);
    
    if (!ok || !context->enqueueV3(stream)) {
        std::cerr << "Batch detection inference failed, falling back to slow path" << std::endl;
        nvinfer1::Dims4 resetInputDims(1, 3, input_h, input_w);
        context->setInputShape(inputName_.c_str(), resetInputDims);
        return DetectBatchSlow(rois);
    }
    
    size_t requiredOutput = N * output_h * output_w;
    if (hOutputSize_ < requiredOutput) {
        if (hOutput_) cudaFreeHost(hOutput_);
        cudaMallocHost(&hOutput_, requiredOutput * sizeof(float));
        hOutputSize_ = requiredOutput;
    }
    cudaMemcpyAsync(hOutput_, batchOutputBuffer_, outputBytes, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    
    nvinfer1::Dims4 resetInputDims(1, 3, input_h, input_w);
    context->setInputShape(inputName_.c_str(), resetInputDims);
    
    return postprocessBatch(hOutput_, N, rxs, rys);
}
