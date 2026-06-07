
#include <fstream>
#include <algorithm>
#include <iostream>

#include "model.hpp"
#include "preprocess.hpp"



class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        (void)severity;
        (void)msg;
        // 静默 TensorRT 日志，避免终端被 INFO 级输出刷屏
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
    cudaFree(0);

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
    cudaEventCreate(&(this->readyEvent_));

    // Initialize normalization params
    if (this->output_h == 1 || this->output_w == 1) {
        mean_[0] = 0.485f; mean_[1] = 0.456f; mean_[2] = 0.406f;
        std_[0]  = 0.229f; std_[1]  = 0.224f; std_[2]  = 0.225f;
    } else {
        mean_[0] = 0.0f; mean_[1] = 0.0f; mean_[2] = 0.0f;
        std_[0]  = 1.0f; std_[1]  = 1.0f; std_[2]  = 1.0f;
    }

    // 单帧路径：tensor 地址和输入 shape 固定，只需设置一次
    // （批量路径会自行覆盖这些设置，不影响）
    context->setTensorAddress(inputName_.c_str(), buffers[0]);
    context->setTensorAddress(outputName_.c_str(), buffers[1]);
    nvinfer1::Dims4 fixedInputDims(1, 3, input_h, input_w);
    context->setInputShape(inputName_.c_str(), fixedInputDims);
}

Model::~Model()
{
    for (auto &buffer : this->buffers)
        cudaFree(buffer);
    cudaFree(gpuInputBuffer8U_);
    cudaFreeHost(hInputBuffer8U_);
    cudaFreeHost(prob_);
    if (inputTex_)
        cudaDestroyTextureObject(inputTex_);
    if (readyEvent_)
        cudaEventDestroy(readyEvent_);
    delete this->context;
    delete this->engine;
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

    size_t imgBytes = frame.step[0] * frame.rows;

    // Ensure pinned CPU staging buffer for async DMA
    if (hInputCapacity_ < imgBytes) {
        if (hInputBuffer8U_) cudaFreeHost(hInputBuffer8U_);
        hInputBuffer8U_ = nullptr;
        cudaError_t err = cudaMallocHost(&hInputBuffer8U_, imgBytes);
        if (err != cudaSuccess) {
            hInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMallocHost hInputBuffer8U_ failed: ") + cudaGetErrorString(err));
        }
        hInputCapacity_ = imgBytes;
    }

    memcpy(hInputBuffer8U_, frame.data, imgBytes);

    // Ensure device-side raw image buffer
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

    // Async H2D from pinned memory
    cudaMemcpyAsync(gpuInputBuffer8U_, hInputBuffer8U_, imgBytes, cudaMemcpyHostToDevice, this->stream);

    // Try texture-backed preprocessing; fall back to raw pointer kernel on failure
    bool useTex = false;
    if (inputTex_ == 0 || texSrcW_ != img_w || texSrcH_ != img_h || texSrcStep_ != static_cast<int>(frame.step[0])) {
        if (inputTex_ != 0) cudaDestroyTextureObject(inputTex_);
        inputTex_ = 0;

        cudaResourceDesc resDesc = {};
        resDesc.resType = cudaResourceTypePitch2D;
        resDesc.res.pitch2D.devPtr = gpuInputBuffer8U_;
        resDesc.res.pitch2D.desc = cudaCreateChannelDesc(8, 8, 8, 0, cudaChannelFormatKindUnsigned);
        resDesc.res.pitch2D.width = img_w;
        resDesc.res.pitch2D.height = img_h;
        resDesc.res.pitch2D.pitchInBytes = frame.step[0];

        cudaTextureDesc texDesc = {};
        texDesc.addressMode[0] = cudaAddressModeClamp;
        texDesc.addressMode[1] = cudaAddressModeClamp;
        texDesc.filterMode = cudaFilterModeLinear;
        texDesc.readMode = cudaReadModeNormalizedFloat;
        texDesc.normalizedCoords = 0;

        cudaError_t texErr = cudaCreateTextureObject(&inputTex_, &resDesc, &texDesc, nullptr);
        if (texErr == cudaSuccess) {
            texSrcW_ = img_w;
            texSrcH_ = img_h;
            texSrcStep_ = static_cast<int>(frame.step[0]);
            useTex = true;
        }
    } else {
        useTex = true;
    }

    if (useTex) {
        launch_preprocess_tex(
            inputTex_,
            img_w, img_h,
            static_cast<float*>(buffers[0]),
            input_w, input_h,
            static_cast<float>(img_w) / new_w,
            static_cast<float>(img_h) / new_h,
            new_w, new_h,
            mean_, std_,
            true,
            this->stream
        );
    } else {
        launch_preprocess(
            static_cast<const uint8_t*>(gpuInputBuffer8U_),
            img_w, img_h, static_cast<int>(frame.step[0]),
            static_cast<float*>(buffers[0]),
            input_w, input_h,
            static_cast<float>(img_w) / new_w,
            static_cast<float>(img_h) / new_h,
            new_w, new_h,
            mean_, std_,
            true,
            this->stream
        );
    }
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
                    float* prob_data = const_cast<float*>(det_output.ptr<float>(0));
                    for (int k = 4; k < num_properties; ++k) {
                        float s = prob_data[k * output_w + idx];
                        if (s > max_score) {
                            max_score = s;
                            class_id = k - 4;
                        }
                    }
                }
            } else {
                float* row_data = const_cast<float*>(det_output.ptr<float>(idx));
                cx = row_data[0];
                cy = row_data[1];
                ow = row_data[2];
                oh = row_data[3];

                if (num_properties == 5) {
                    max_score = row_data[4];
                    class_id = 0;
                } else {
                    for (int k = 4; k < num_properties; ++k) {
                        float s = row_data[k];
                        if (s > max_score) {
                            max_score = s;
                            class_id = k - 4;
                        }
                    }
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

    cv::Mat det_output(this->output_h, this->output_w, CV_32F, prob_);
    this->detectResults = postprocessSingle(det_output, this->rx, this->ry);
}


bool Model::Detect(const cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        if (!context->enqueueV3(this->stream)) {
            throw std::runtime_error("enqueueV3 failed in Detect");
        }

        cudaMemcpyAsync(this->prob_, this->buffers[1], this->probSize_ * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
        cudaEventRecord(readyEvent_, this->stream);
        cudaEventSynchronize(readyEvent_);

        postprocessing();

        return !detectResults.empty();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Model::Detect] exception: " << e.what() << std::endl;
        return false;
    }
}
int Model::predictClass(const cv::Mat &roi) {
    preprocessing(roi);

    if (!context->enqueueV3(this->stream)) {
        throw std::runtime_error("enqueueV3 failed in predictClass");
    }

    cudaMemcpyAsync(this->prob_, this->buffers[1], this->probSize_ * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
    cudaEventRecord(readyEvent_, this->stream);
    cudaEventSynchronize(readyEvent_);

    float* data = this->prob_;
    return std::max_element(data, data + probSize_) - data;
}






