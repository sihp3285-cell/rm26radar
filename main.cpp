#include <yaml-cpp/yaml.h>
#include "utils/include/draw.hpp"
#include "ConfigManager.hpp"
#include "utils/include/posesolver.hpp"
#include "utils/include/mouseback.hpp"
#include "utils/include/radarmap.hpp"
#include "utils/include/pipeline.hpp"
#include "utils/include/ui.hpp"
#include "utils/include/tracker.hpp"


int main(int argc, char const *argv[])
{
    Config cfg("/home/delphine/rm/tensorrt10_detect/configs/detectConfig.yaml");
    cv::VideoCapture cap("/home/delphine/rm/car_project/test/005.mp4");
    
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开视频文件！" << std::endl;
        return -1;
    }

    DetectPipeline pipeline(cfg);
    PoseSolver poseSolver(cfg.cameraMatrix, cfg.distCoeffs);

    cv::Mat calibrateFrame; 

    {
        int num = cfg.worldPoints.size();
        cv::namedWindow("Video Preview", cv::WINDOW_NORMAL);
        cv::resizeWindow("Video Preview", 1280, 720);

        std::cout << "视频播放中... 请在看到合适画面时按 'S' 键截取并开始标定。" << std::endl;

        while (true) {
            cv::Mat tempFrame;
            if (!cap.read(tempFrame)) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }

            cv::imshow("Video Preview", tempFrame);
            
            int key = cv::waitKey(30); 
            if (key == 's' || key == 'S') {
                calibrateFrame = tempFrame.clone();
                std::cout << "已截取当前帧！请在弹出窗口中依次点击 " << num << " 个标定点。" << std::endl;
                cv::destroyWindow("Video Preview"); 
                break; 
            }
            if (key == 'q' || key == 27) {
                return -1;
            }
        }

        MouseBack mouseBack("Calibrate 1", num);
        std::vector<cv::Point2f> imagePoints = mouseBack.getPoints(calibrateFrame);
        
        if(imagePoints.size() == num) {
            poseSolver.calibrate(cfg.worldPoints, imagePoints);
            std::cout << "相机标定（PnP）成功！" << std::endl;
        } else {
            std::cout << "标定点数不足，程序退出。" << std::endl;
            return -1;
        }

        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    }

    RadarMap radarMap(cfg.mapPath, cfg.isflip);
    {
        cv::Mat rawMap = cv::imread(cfg.mapPath);
        int rawW = rawMap.cols; 
        int rawH = rawMap.rows; 

        std::vector<cv::Point2f> rotatedPoints;
        for (auto& pt : cfg.MapPoints) {
            cv::Point2f rPt;
            if(cfg.isflip)
            {
                rPt.x = (float)rawH - pt.y; 
                rPt.y = pt.x;
            }
            else
            {
                rPt.x = pt.y; 
                rPt.y = (float)rawW - pt.x;
            }
            rotatedPoints.push_back(rPt);
        }

        radarMap.calibrate2(rotatedPoints, cfg.worldPoints2D);
    };


    UI ui("Video & Radar");
    bool isPaused = false;
    Tracker tracker(cfg.maxMissCount, cfg.maxhistory, cfg.distheshold);
    while (true) 
    {
        cv::Mat frame;
        if(!cap.read(frame)) break;

        std::vector<Result> allresults = pipeline.process(frame);
        std::vector<Mappoint> mappoints;
        for(const auto& result : allresults)
        {
            if (result.idx == 0 ) continue;   
            cv::Point2f wp  = poseSolver.middletoworld(result.car_box);
            cv::Point2f mp  = radarMap.worldtomap(wp);
            mappoints.push_back({mp, "", result.idx, result.armorColor});
        }
  
        tracker.update(mappoints);
        std::vector<Mappoint> smoothedPoints = tracker.getSmoothedPoints();


        
        drawDetect(frame, allresults, cfg.classNames);
        cv::Mat radarImg = radarMap.drawMap(smoothedPoints, cfg.classNames);

        int key = ui.update(frame, radarImg, isPaused);

        if (key == 'q' || key == 27)  break;
        if (key == ' ')  isPaused = !isPaused;
    }    
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}