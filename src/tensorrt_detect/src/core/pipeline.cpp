#include "pipeline.hpp"
#include "ConfigManager.hpp"
#include "robot_id.hpp"


DetectPipeline::DetectPipeline(Config& cfg)
    : detectModel_(cfg.model.modelPath, cfg.model.imgSize1, cfg.model.scoreThreshold1, cfg.model.iouThreshold1, cfg.model.isNMS1, modelType(cfg.model.modelType1)),
      armorDetector_(cfg.model.armorModelPath, cfg.model.imgSize2, cfg.model.scoreThreshold2, cfg.model.iouThreshold2, cfg.model.isNMS2, modelType(cfg.model.modelType2)),
      classifyModel_(cfg.model.classifyModelPath, cfg.model.imgSize3, cfg.model.scoreThreshold3, cfg.model.iouThreshold3, cfg.model.isNMS3, modelType(cfg.model.modelType3)),
      cfg_(cfg)
{}



std::vector<Result> DetectPipeline::runDetect(const cv::Mat& frame) {
    detectModel_.Detect(frame);
    cv::Rect roi = detectModel_.roi;
    if(roi.width >= 150 || roi.height >= 150) {
        return std::vector<Result>();
    }
    return detectModel_.detectResults;   
}

std::vector<Result> DetectPipeline::runArmorDetect(const cv::Mat& frame,
                                                    const std::vector<Result>& detections) {
    std::vector<Result> armorResults;
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);

    for (const auto& det : detections) {
        cv::Rect roi = det.box & imgBound;
        if (roi.width < cfg_.model.minRoiSize || roi.height < cfg_.model.minRoiSize)
            continue;

        if (!armorDetector_.Detect(frame(roi)))
            continue;

        // 只保留装甲板检测结果中置信度最高的一个
        if (!armorDetector_.detectResults.empty()) {
            // 找出置信度最高的装甲板
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

std::vector<Result> DetectPipeline::process(const cv::Mat& frame) {
    std::vector<Result> all = runDetect(frame);  

    auto detections = runArmorDetect(frame, all);
    runClassify(frame, detections);
    all.insert(all.end(), detections.begin(), detections.end());
    return all;
}