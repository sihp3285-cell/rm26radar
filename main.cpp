#include <yaml-cpp/yaml.h>
#include "utils/include/draw.hpp"
#include "ConfigManager.hpp"
#include "utils/include/posesolver.hpp"
#include "utils/include/mouseback.hpp"
#include "utils/include/radarmap.hpp"
#include "utils/include/pipeline.hpp"
#include "utils/include/ui.hpp"
#include <chrono>


int main(int argc, char const *argv[])
{
    Config cfg("/home/delphine/rm/tensorrt10_detect/configs/detectConfig.yaml");
    cv::VideoCapture cap("/home/delphine/rm/car_project/test/005.mp4");
    
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开视频文件！" << std::endl;
        return -1;
    }

    DetectPipeline pipeline(cfg);
    PoseSolver poseSolver(cfg.camera.cameraMatrix, cfg.camera.distCoeffs);

    std::cout << "配置的 mesh 路径: " << cfg.camera.meshPath << std::endl;
    if (!poseSolver.getRaycaster().loadingMesh(cfg.camera.meshPath)) {  
        std::cerr << "警告：无法加载 3D 网格文件，将使用平面地面回退方案" << std::endl;
    }



    cv::Mat calibrateFrame; 

    {
        int num = cfg.camera.requirePointsNum;
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
            poseSolver.calibrate(cfg.camera.worldPoints, imagePoints);
            std::cout << "相机标定（PnP）成功！" << std::endl;
        } else {
            std::cout << "标定点数不足，程序退出。" << std::endl;
            return -1;
        }

        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    }

    RadarMap radarMap(cfg.map.mapPath, cfg.map.isFlip);
    
    radarMap.calibrate2(cfg.map.race_size[0], cfg.map.race_size[1], 
                       cfg.map.map_size[0], cfg.map.map_size[1]);


    UI ui("Video & Radar");
    bool isPaused = false;
    using Clock = std::chrono::steady_clock;

    auto last_time = Clock::now();
    double fps = 0.0;
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
    

        
        drawDetect(frame, allresults, cfg.model.classNames);
        cv::Mat radarImg = radarMap.drawMap(mappoints, cfg.model.classNames);
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        double instant_fps = 1.0 / std::max(dt, 1e-6);
        fps = 0.9 * fps + 0.1 * instant_fps;   
        cv::putText(frame,
                cv::format("FPS: %.1f", fps),
                cv::Point(20,300),
                cv::FONT_HERSHEY_SIMPLEX,
                5.0,
                cv::Scalar(0, 255, 0),
                2);
        int key = ui.update(frame, radarImg, isPaused);



        if (key == 'q' || key == 27)  break;
        if (key == ' ')  isPaused = !isPaused;
    }    
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}