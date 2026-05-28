#include "pipeline.hpp"
#include "ConfigManager.hpp"
#include "model.hpp"
#include "robot_id.hpp"
#include <iostream>
#include <cuda_runtime_api.h>


DetectPipeline::DetectPipeline(Config& cfg)
    : detectModel_(cfg.model.modelPath, cfg.model.imgSize1, cfg.model.scoreThreshold1, cfg.model.iouThreshold1, cfg.model.isNMS1, modelType(cfg.model.modelType1)),
      armorDetector_(cfg.model.armorModelPath, cfg.model.imgSize2, cfg.model.scoreThreshold2, cfg.model.iouThreshold2, cfg.model.isNMS2, modelType(cfg.model.modelType2)),
      classifyModel_(cfg.model.classifyModelPath, cfg.model.imgSize3, cfg.model.scoreThreshold3, cfg.model.iouThreshold3, cfg.model.isNMS3, modelType(cfg.model.modelType3)),
      cfg_(cfg)
{
    // 确保 CUDA primary context 已初始化（Model 构造函数内部也会 cudaFree(0)，此处做双重保障）
    cudaFree(0);

    if (!cfg.model.airplaneModelPath.empty()) {
        airplaneModel_ = std::make_unique<Model>(
            cfg.model.airplaneModelPath,
            cfg.model.imgSize4,
            cfg.model.scoreThreshold4,
            cfg.model.iouThreshold4,
            cfg.model.isNMS4,
            modelType(cfg.model.modelType4)
        );
        airplaneIntervalMs_ = std::max(1, cfg.model.airplaneIntervalMs);
        airplaneThread_ = std::thread(&DetectPipeline::airplaneThreadLoop, this);
    }
}

DetectPipeline::~DetectPipeline()
{
    stopThread_ = true;
    airplaneCv_.notify_all();
    if (airplaneThread_.joinable()) {
        airplaneThread_.join();
    }
}



std::vector<Result> DetectPipeline::runDetect(const cv::Mat& frame) {
    detectModel_.Detect(frame);
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);
    std::vector<Result> filtered;
    for (const auto& det : detectModel_.detectResults) {
        cv::Rect safeBox = det.box & imgBound;
        if (safeBox.width >= cfg_.model.minRoiSize && safeBox.height >= cfg_.model.minRoiSize) {
            filtered.push_back(det);
        }
    }
    return filtered;
}

std::vector<Result> DetectPipeline::runArmorDetect(const cv::Mat& frame,
                                                    const std::vector<Result>& detections,
                                                    std::vector<Result>* outposts) {
    std::vector<Result> armorResults;
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);

    for (const auto& det : detections) {
        cv::Rect roi = det.box & imgBound;
        if (roi.width < cfg_.model.minRoiSize || roi.height < cfg_.model.minRoiSize)
            continue;

        if (!armorDetector_.Detect(frame(roi)))
            continue;

        if (!armorDetector_.detectResults.empty()) {
            auto maxArmor = std::max_element(armorDetector_.detectResults.begin(),
                                            armorDetector_.detectResults.end(),
                [](const Result& a, const Result& b) {
                    return a.confidence < b.confidence;
                });

            Result armor = *maxArmor;
            int raw_id = armor.idx;

            armor.box.x += roi.x;
            armor.box.y += roi.y;
            armor.car_box = det.box;

            constexpr int DEAD_ARMOR_ID = 0;
            armor.idx = robot_id::ARMOR;
            armor.isDead = (raw_id == DEAD_ARMOR_ID);
            if (armor.isDead) {
                armor.armorColor = robot_id::UNKNOWN;
            } else {
                armor.armorColor = raw_id;
            }

            armor.worldPoint = cv::Point2f(det.box.x + det.box.width  / 2.0f,
                                           det.box.y + det.box.height / 2.0f);
            armorResults.push_back(armor);
        }
    }

    if (outposts != nullptr && cfg_.model.outpostEnabled && cfg_.model.outpostRoi.size() == 4) {
        cv::Rect outpostRoi(
            cfg_.model.outpostRoi[0],
            cfg_.model.outpostRoi[1],
            cfg_.model.outpostRoi[2],
            cfg_.model.outpostRoi[3]
        );
        cv::Rect safeOutpostRoi = outpostRoi & imgBound;
        if (safeOutpostRoi.width > 0 && safeOutpostRoi.height > 0) {
            if (armorDetector_.Detect(frame(safeOutpostRoi))) {
                bool hasValidDetection = false;
                Result bestResult;
                float bestConf = -1.0f;

                for (const auto& res : armorDetector_.detectResults) {
                    if (res.confidence < cfg_.model.outpostScoreThreshold) continue;
                    if (res.confidence > bestConf) {
                        bestConf = res.confidence;
                        bestResult = res;
                        hasValidDetection = true;
                    }
                }

                if (hasValidDetection) {
                    outpostMissCount_ = 0;
                    outpostIsDead_ = false;

                    bestResult.box.x += safeOutpostRoi.x;
                    bestResult.box.y += safeOutpostRoi.y;
                    bestResult.idx = robot_id::OUTPOST;
                    bestResult.car_box = safeOutpostRoi;
                    bestResult.isDead = false;
                    outpostLastBox_ = bestResult.box;
                    outposts->push_back(bestResult);
                } else {
                    outpostMissCount_++;
                    if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
                        outpostMissCount_ = cfg_.model.outpostMissThreshold;
                        outpostIsDead_ = true;
                    }
                }
            } else {
                outpostMissCount_++;
                if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
                    outpostMissCount_ = cfg_.model.outpostMissThreshold;
                    outpostIsDead_ = true;
                }
            }
        }
    }

    return armorResults;
}


void DetectPipeline::runClassify(const cv::Mat& frame, std::vector<Result>& detections) {
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows); // 定义边界

    for (auto& armor : detections) {
        if (armor.isDead) {
            armor.idx = robot_id::ARMOR;
            continue;
        }

        cv::Rect safeBox = armor.box & imgBound; 

        if (safeBox.width <= 0 || safeBox.height <= 0) {
            continue;
        }

        cv::Mat armorROI = frame(safeBox); // 现在这里绝对安全了
        
        int raw_id = classifyModel_.predictClass(armorROI); 
        if (raw_id == 4) {
            armor.idx = 6; 
        } 
        else if (raw_id >= 0 && raw_id <= 3) {
            armor.idx = raw_id + 2; 
        }
    }
}


std::vector<Result> DetectPipeline::runOutpostDetect(const cv::Mat& frame) {
    std::vector<Result> results;
    if (!cfg_.model.outpostEnabled || cfg_.model.outpostRoi.size() != 4) {
        return results;
    }

    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);
    cv::Rect outpostRoi(
        cfg_.model.outpostRoi[0],
        cfg_.model.outpostRoi[1],
        cfg_.model.outpostRoi[2],
        cfg_.model.outpostRoi[3]
    );
    cv::Rect safeRoi = outpostRoi & imgBound;
    if (safeRoi.width <= 0 || safeRoi.height <= 0) {
        return results;
    }

    bool hasValidDetection = false;
    Result bestResult;
    float bestConf = -1.0f;

    if (armorDetector_.Detect(frame(safeRoi))) {
        for (auto& res : armorDetector_.detectResults) {
            if (res.confidence < cfg_.model.outpostScoreThreshold) {
                continue;
            }
            if (res.confidence > bestConf) {
                bestConf = res.confidence;
                bestResult = res;
                hasValidDetection = true;
            }
        }
    }

    if (hasValidDetection) {
        outpostMissCount_ = 0;
        outpostIsDead_ = false;

        bestResult.box.x += safeRoi.x;
        bestResult.box.y += safeRoi.y;
        bestResult.idx = robot_id::OUTPOST;
        bestResult.car_box = safeRoi;
        bestResult.isDead = false;
        outpostLastBox_ = bestResult.box;
        results.push_back(bestResult);
    } else {
        outpostMissCount_++;
        if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
            outpostMissCount_ = cfg_.model.outpostMissThreshold;
            outpostIsDead_ = true;
        }
    }
    return results;
}

std::vector<Result> DetectPipeline::runAirplaneDetect(const cv::Mat& frame) {
    if (!airplaneModel_) {
        return std::vector<Result>();
    }
    int xStart = frame.cols / 2;
    int width = frame.cols - xStart;
    cv::Rect rightRoi(xStart, 0, width, frame.rows);
    airplaneModel_->Detect(frame(rightRoi));
    std::vector<Result> results = airplaneModel_->detectResults;
    for (auto& res : results) {
        res.idx = robot_id::AIRPLANE;
        res.box.x += xStart;
    }
    return results;
}

static inline double elapsedMs(const std::chrono::steady_clock::time_point& t0,
                                 const std::chrono::steady_clock::time_point& t1)
{
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

std::vector<Result> DetectPipeline::process(const cv::Mat& frame) {
    auto t0 = std::chrono::steady_clock::now();

    // 更新共享图像缓存并通知后台线程（只存右半，带宽减半）
    if (airplaneModel_) {
        std::lock_guard<std::mutex> lock(frameMutex_);
        airplaneRoiX_ = frame.cols / 2;
        int width = frame.cols - airplaneRoiX_;
        latestFrame_ = frame(cv::Rect(airplaneRoiX_, 0, width, frame.rows)).clone();
        newFrameAvailable_ = true;
    }
    airplaneCv_.notify_one();

    auto t1 = std::chrono::steady_clock::now();
    auto cars = runDetect(frame);
    auto t2 = std::chrono::steady_clock::now();

    std::vector<Result> outposts;
    auto armors = runArmorDetect(frame, cars, &outposts);
    auto t3 = std::chrono::steady_clock::now();
    runClassify(frame, armors);
    auto t4 = std::chrono::steady_clock::now();

    std::vector<Result> all;
    all.insert(all.end(), cars.begin(), cars.end());
    all.insert(all.end(), armors.begin(), armors.end());
    all.insert(all.end(), outposts.begin(), outposts.end());

    // 获取最新缓存的无人机结果（非阻塞）
    {
        std::lock_guard<std::mutex> lock(resultsMutex_);
        all.insert(all.end(), cachedAirplaneResults_.begin(), cachedAirplaneResults_.end());
    }

    auto t6 = std::chrono::steady_clock::now();
    double car_ms   = elapsedMs(t1, t2);
    double armor_ms = elapsedMs(t2, t3);
    double cls_ms   = elapsedMs(t3, t4);
    double total_ms = elapsedMs(t0, t6);

    {
        std::lock_guard<std::mutex> lock(timingMutex_);
        latestTiming_.car_ms   = car_ms;
        latestTiming_.armor_ms = armor_ms;
        latestTiming_.cls_ms   = cls_ms;
        latestTiming_.outpost_ms = 0.0;
        latestTiming_.airplane_ms = lastAirplaneMs_.load();
        latestTiming_.total_ms = total_ms;
        latestTiming_.end_to_end_ms = 0.0;
    }

    updateStats(car_ms, armor_ms, cls_ms, 0.0, total_ms);

    return all;
}

void DetectPipeline::updateStats(double carMs, double armorMs, double clsMs,
                                 double outpostMs, double totalMs)
{
    accCarMs_ += carMs;
    accArmorMs_ += armorMs;
    accClsMs_ += clsMs;
    accOutpostMs_ += outpostMs;
    accTotalMs_ += totalMs;
    ++accCount_;

    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - lastStatsTime_).count();
    if (elapsedSec >= 5.0 && accCount_ > 0) {
        accCarMs_ = accArmorMs_ = accClsMs_ = accOutpostMs_ = accTotalMs_ = 0.0;
        accCount_ = 0;
        lastStatsTime_ = now;
    }
}

PipelineTiming DetectPipeline::getLatestTiming() const
{
    std::lock_guard<std::mutex> lock(timingMutex_);
    return latestTiming_;
}

void DetectPipeline::airplaneThreadLoop()
{
    while (!stopThread_) {
        cv::Mat frame;
        bool hasNewFrame = false;
        int xOffset = 0;
        {
            std::unique_lock<std::mutex> lock(frameMutex_);
            airplaneCv_.wait_for(lock, std::chrono::milliseconds(100),
                [this] { return newFrameAvailable_ || stopThread_.load(); });
            if (stopThread_) break;
            if (newFrameAvailable_) {
                frame = latestFrame_;          // 浅拷贝 O(1)，数据已在 process() 里 clone 过
                xOffset = airplaneRoiX_;       // 同步读取原图偏移
                newFrameAvailable_ = false;
                hasNewFrame = true;
            }
        }

        if (hasNewFrame && airplaneModel_) {
            auto ta0 = std::chrono::steady_clock::now();
            airplaneModel_->Detect(frame);
            auto ta1 = std::chrono::steady_clock::now();
            lastAirplaneMs_ = elapsedMs(ta0, ta1);

            std::vector<Result> results = airplaneModel_->detectResults;
            for (auto& res : results) {
                res.idx = robot_id::AIRPLANE;
                res.box.x += xOffset;
            }
            std::lock_guard<std::mutex> lock(resultsMutex_);
            cachedAirplaneResults_ = std::move(results);
        }

        // 低频控制，避免占用过多 GPU/CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(airplaneIntervalMs_));
    }
}
