
#include <fstream>
#include <algorithm>
#include <numeric>

#include "model.hpp"

class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kINFO)
            std::cout << msg << std::endl;
    }
} gLogger;

Model::Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold, const bool isNMS,const ModelType modelType)
{
    this->inputSize = inputSize;
    this->scoreThreshold = scoreThreshold;
    this->nmsThreshold = nmsThreshold;
    this->isNMS = isNMS;
    this->modelType = modelType;


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

    this->runtime = nvinfer1::createInferRuntime(gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    assert(this->engine != nullptr);

    this->context = this->engine->createExecutionContext();
    assert(this->context != nullptr);

    std::string inputName, outputName;
    int nbTensors = this->engine->getNbIOTensors();
    for (int i = 0; i < nbTensors; ++i) {
        const char* name = this->engine->getIOTensorName(i);
        if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            inputName = name;
        } else if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
            outputName = name;
        }
    }

    nvinfer1::Dims inputDims = this->engine->getTensorShape(inputName.c_str());
    nvinfer1::Dims outputDims = this->engine->getTensorShape(outputName.c_str());

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
    this->prob.resize(this->output_h * this->output_w);
    cudaStreamCreate(&(this->stream));
}

Model::~Model()
{
    for (auto &buffer : this->buffers)
    {
        if (buffer)
            cudaFree(buffer);
    }
    if (this->context)
        delete this->context;
    if (this->engine)
        delete this->engine;
    if (this->runtime)
        delete this->runtime;
    if (this->stream)
        cudaStreamDestroy(this->stream);
}
void Model::preprocessing(const cv::Mat &frame)
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

    this->rx = (float)img_w / new_w;
    this->ry = (float)img_h / new_h;

    cv::Mat canvas_non_const = canvas.clone();
    cv::Mat blob = cv::dnn::blobFromImage(canvas_non_const, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
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
    cudaMemcpyAsync(buffers[0], blob.ptr<float>(), 3 * this->input_h * this->input_w * sizeof(float), cudaMemcpyHostToDevice, this->stream);
}

void Model::postprocessing()
{
    if (this->modelType == ModelType::CLASSIFY) {
        return;
    }

    this->detectResults.clear();
    cv::Mat det_output(this->output_h, this->output_w, CV_32F, (float *)prob.data());

    if (this->isNMS) 
    {
        bool is_transposed = (this->output_h < this->output_w); 
        int num_boxes = is_transposed ? this->output_w : this->output_h;

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
            box.x = static_cast<int>(x1 * this->rx);
            box.y = static_cast<int>(y1 * this->ry);
            box.width = static_cast<int>((x2 - x1) * this->rx);
            box.height = static_cast<int>((y2 - y1) * this->ry);

            this->detectResults.emplace_back(Result{class_id, score, box});
        }
    } 
    else 
    {
        std::vector<cv::Rect> boxes;
        std::vector<int> classIds;
        std::vector<float> confidences;

        bool is_transposed = (this->output_h > this->output_w); 
        int num_anchors = is_transposed ? this->output_h : this->output_w;
        int num_properties = is_transposed ? this->output_w : this->output_h;

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
                box.x = static_cast<int>((cx - 0.5 * ow) * this->rx);
                box.y = static_cast<int>((cy - 0.5 * oh) * this->ry);
                box.width = static_cast<int>(ow * this->rx);
                box.height = static_cast<int>(oh * this->ry);

                boxes.push_back(box);
                classIds.push_back(class_id);
                confidences.push_back(max_score);
            }
        }

        std::vector<int> indexes;
        cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, indexes);
        for (int idx : indexes) {
            this->detectResults.emplace_back(Result{classIds[idx], confidences[idx], boxes[idx]});
        }
    }
}

bool Model::Detect(const cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        context->executeV2(buffers);
        cudaMemcpyAsync(this->prob.data(), this->buffers[1], this->output_h * this->output_w * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
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
context->executeV2(buffers);
cudaMemcpyAsync(this->prob.data(), this->buffers[1], this->output_h * this->output_w * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
cudaStreamSynchronize(this->stream);
float* data = (float*)prob.data();
return std::max_element(data, data + (output_h * output_w)) - data;

}

    