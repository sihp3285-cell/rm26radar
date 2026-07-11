#include "pipeline.hpp"
#include "ConfigManager.hpp"
#include "model.hpp"
#include "robot_id.hpp"
#include <iostream>
#include <cuda_runtime_api.h>
#include <cmath>


DetectPipeline::DetectPipeline(Config& cfg)
    : detectModel_(cfg.model.modelPath, cfg.model.imgSize1, cfg.model.scoreThreshold1, cfg.model.iouThreshold1, cfg.model.isNMS1, modelType(cfg.model.modelType1)),
      armorDetector_(cfg.model.armorModelPath, cfg.model.imgSize2, cfg.model.scoreThreshold2, cfg.model.iouThreshold2, cfg.model.isNMS2, modelType(cfg.model.modelType2)),
      classifyModel_(cfg.model.classifyModelPath, cfg.model.imgSize3, cfg.model.scoreThreshold3, cfg.model.iouThreshold3, cfg.model.isNMS3, modelType(cfg.model.modelType3)),
      cfg_(cfg)
{
    // Model 构造函数内部已通过 cudaFree(0) 初始化 CUDA primary context
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



static inline double elapsedMs(const std::chrono::steady_clock::time_point& t0,
                                 const std::chrono::steady_clock::time_point& t1)
{
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
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
                                                    const std::vector<Result>& detections) {
    auto t0 = std::chrono::steady_clock::now();
    std::vector<Result> armorResults;
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);

    struct PackedRoi {
        const Result* car;
        cv::Rect source;
        cv::Rect canvas;
        float scale;
    };

    std::vector<PackedRoi> rois;
    rois.reserve(detections.size());
    for (const auto& det : detections) {
        cv::Rect roi = det.box & imgBound;
        if (roi.width >= cfg_.model.minRoiSize && roi.height >= cfg_.model.minRoiSize)
            rois.push_back(PackedRoi{&det, roi, {}, 1.0f});
    }

    const int batchSize = cfg_.model.multiCarRecognition
                              ? std::max(1, cfg_.model.maxArmorRois) : 1;
    for (size_t begin = 0; begin < rois.size(); begin += batchSize) {
        const int count = std::min<int>(batchSize, rois.size() - begin);
        const int cols = static_cast<int>(std::ceil(std::sqrt(count)));
        const int rows = (count + cols - 1) / cols;
        const int canvasSize = cfg_.model.imgSize2;
        const int cellW = canvasSize / cols;
        const int cellH = canvasSize / rows;
        const int padding = cfg_.model.armorCanvasPadding;
        cv::Mat canvas(canvasSize, canvasSize, CV_8UC3, cv::Scalar(114, 114, 114));

        for (int i = 0; i < count; ++i) {
            auto& packed = rois[begin + i];
            // 单 ROI 保持原实现的左上角 letterbox 布局，关闭拼接时行为不变。
            const int tilePadding = count == 1 ? 0 : padding;
            const int availableW = std::max(1, cellW - 2 * tilePadding);
            const int availableH = std::max(1, cellH - 2 * tilePadding);
            packed.scale = std::min(static_cast<float>(availableW) / packed.source.width,
                                    static_cast<float>(availableH) / packed.source.height);
            const int resizedW = std::max(1, static_cast<int>(packed.source.width * packed.scale));
            const int resizedH = std::max(1, static_cast<int>(packed.source.height * packed.scale));
            const int cellX = (i % cols) * cellW;
            const int cellY = (i / cols) * cellH;
            packed.canvas = cv::Rect(count == 1 ? 0 : cellX + (cellW - resizedW) / 2,
                                     count == 1 ? 0 : cellY + (cellH - resizedH) / 2,
                                     resizedW, resizedH);
            cv::resize(frame(packed.source), canvas(packed.canvas), packed.canvas.size());
        }

        if (!armorDetector_.Detect(canvas))
            continue;

        std::vector<const Result*> best(count, nullptr);
        for (const auto& candidate : armorDetector_.detectResults) {
            const cv::Point center(candidate.box.x + candidate.box.width / 2,
                                   candidate.box.y + candidate.box.height / 2);
            for (int i = 0; i < count; ++i) {
                const auto& packed = rois[begin + i];
                if (packed.canvas.contains(center) &&
                    (!best[i] || candidate.confidence > best[i]->confidence)) {
                    best[i] = &candidate;
                    break;
                }
            }
        }

        for (int i = 0; i < count; ++i) {
            if (!best[i]) continue;
            const auto& packed = rois[begin + i];
            Result armor = *best[i];
            const int raw_id = armor.idx;
            armor.box.x = packed.source.x +
                          static_cast<int>((armor.box.x - packed.canvas.x) / packed.scale);
            armor.box.y = packed.source.y +
                          static_cast<int>((armor.box.y - packed.canvas.y) / packed.scale);
            armor.box.width = static_cast<int>(armor.box.width / packed.scale);
            armor.box.height = static_cast<int>(armor.box.height / packed.scale);
            armor.box &= imgBound;
            armor.car_box = packed.car->box;
            armor.idx = robot_id::ARMOR;
            armor.isDead = (raw_id == 0);
            armor.armorColor = armor.isDead ? robot_id::UNKNOWN : raw_id;
            armor.worldPoint = cv::Point2f(packed.car->box.x + packed.car->box.width / 2.0f,
                                           packed.car->box.y + packed.car->box.height / 2.0f);
            armorResults.push_back(armor);
        }
    }

    lastArmorDetectMs_ = elapsedMs(t0, std::chrono::steady_clock::now());
    return armorResults;
}

std::vector<Result> DetectPipeline::detectOutpost(const cv::Mat& frame) {
    auto t0 = std::chrono::steady_clock::now();
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
    cv::Rect safeOutpostRoi = outpostRoi & imgBound;
    if (safeOutpostRoi.width <= 0 || safeOutpostRoi.height <= 0) {
        return results;
    }

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
            results.push_back(bestResult);
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
    lastOutpostDetectMs_ = elapsedMs(t0, std::chrono::steady_clock::now());
    return results;
}


void DetectPipeline::runClassify(const cv::Mat& frame, std::vector<Result>& detections) {
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);

    for (auto& armor : detections) {
        if (armor.isDead) {
            armor.idx = robot_id::ARMOR;
            continue;
        }

        cv::Rect safeBox = armor.box & imgBound;

        if (safeBox.width <= 0 || safeBox.height <= 0) {
            continue;
        }

        cv::Mat armorROI = frame(safeBox).clone();

        if (armorROI.empty()) {
            continue;
        }

        int raw_id = classifyModel_.predictClass(armorROI);

        if (raw_id == 4) {
            armor.idx = 6;
        } else if (raw_id >= 0 && raw_id <= 3) {
            armor.idx = raw_id + 2;
        }
    }
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

    // Stage 2: armor + outpost 共用 armorDetector_
    auto armors = runArmorDetect(frame, cars);
    auto outposts = detectOutpost(frame);
    auto t3 = std::chrono::steady_clock::now();

    // Stage 3: classify
    runClassify(frame, armors);
    auto t4 = std::chrono::steady_clock::now();

    std::vector<Result> all;
    all.reserve(cars.size() + armors.size() + outposts.size());
    all.insert(all.end(), std::make_move_iterator(cars.begin()), std::make_move_iterator(cars.end()));
    all.insert(all.end(), std::make_move_iterator(armors.begin()), std::make_move_iterator(armors.end()));
    all.insert(all.end(), std::make_move_iterator(outposts.begin()), std::make_move_iterator(outposts.end()));

    // 获取最新缓存的无人机结果（非阻塞）
    {
        std::lock_guard<std::mutex> lock(resultsMutex_);
        all.insert(all.end(),
                   std::make_move_iterator(cachedAirplaneResults_.begin()),
                   std::make_move_iterator(cachedAirplaneResults_.end()));
    }

    auto t6 = std::chrono::steady_clock::now();
    double car_ms   = elapsedMs(t1, t2);
    double armor_ms = lastArmorDetectMs_.load();
    double outpost_ms = lastOutpostDetectMs_.load();
    double cls_ms   = elapsedMs(t3, t4);
    double total_ms = elapsedMs(t0, t6);

    {
        std::lock_guard<std::mutex> lock(timingMutex_);
        latestTiming_.car_ms   = car_ms;
        latestTiming_.armor_ms = armor_ms;
        latestTiming_.cls_ms   = cls_ms;
        latestTiming_.outpost_ms = outpost_ms;
        latestTiming_.airplane_ms = lastAirplaneMs_.load();
        latestTiming_.total_ms = total_ms;
        latestTiming_.end_to_end_ms = 0.0;
    }

    updateStats(car_ms, armor_ms, cls_ms, outpost_ms, total_ms);

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
