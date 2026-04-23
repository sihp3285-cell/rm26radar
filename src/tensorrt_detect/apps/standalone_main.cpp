#include <yaml-cpp/yaml.h>
#include "draw.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
#include "mouseback.hpp"
#include "radarmap.hpp"
#include "pipeline.hpp"
#include "ui.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>

namespace YAML {
    template<>
    struct convert<cv::Point2f> {
        static Node encode(const cv::Point2f& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            return node;
        }
        
        static bool decode(const Node& node, cv::Point2f& rhs) {
            if (!node.IsSequence() || node.size() != 2) {
                return false;
            }
            rhs.x = node[0].as<float>();
            rhs.y = node[1].as<float>();
            return true;
        }
    };
}


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

        std::cout << "视频播放中... 请在看到合适画面时按 'S' 键截取并开始标定，按空格键使用已保存的标定点。" << std::endl;

        bool calibrationDone = false;
        
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
            if (key == ' ' && !calibrationDone) {
                std::cout << "正在读取calib_result.yaml文件..." << std::endl;
                try{
                    std::filesystem::path configDir = std::filesystem::path("/home/delphine/rm/tensorrt10_detect/configs");
                    std::string calibPath = (configDir / "calib_result.yaml").string();
                    YAML::Node node = YAML::LoadFile(calibPath);
                    
                    if (node["r"].IsSequence() && node["t"].IsSequence()) {
                        std::vector<double> r_data = node["r"].as<std::vector<double>>();
                        std::vector<double> t_data = node["t"].as<std::vector<double>>();
                        
                        if (r_data.size() == 9 && t_data.size() == 3) {
                            cv::Mat R(3, 3, CV_64F);
                            cv::Mat T(3, 1, CV_64F);
                            
                            for (int i = 0; i < 9; ++i) {
                                R.at<double>(i / 3, i % 3) = r_data[i];
                            }
                            for (int i = 0; i < 3; ++i) {
                                T.at<double>(i, 0) = t_data[i];
                            }
                            
                            poseSolver.setExtrinsic(R, T);
                            std::cout << "成功从calib_result.yaml加载R和T矩阵！" << std::endl;
                            calibrationDone = true;
                            cv::destroyWindow("Video Preview");
                            break;
                        } else {
                            std::cerr << "错误：R矩阵需要9个元素，T向量需要3个元素！" << std::endl;
                            std::cout << "请按 'S' 键重新标定。" << std::endl;
                        }
                    } else {
                        std::cerr << "错误：calib_result.yaml 文件中缺少 r 或 t 键！" << std::endl;
                        std::cout << "请按 'S' 键重新标定。" << std::endl;
                    }
                } catch (const YAML::Exception& e) {
                    std::cerr << "错误：读取 calib_result.yaml 文件时出错: " << e.what() << std::endl;
                    std::cout << "请按 'S' 键重新标定。" << std::endl;
                }
            }
        }

        if (!calibrationDone) {
            MouseBack mouseBack("Calibrate 1", num);
            std::vector<cv::Point2f> imagePoints = mouseBack.getPoints(calibrateFrame);
            
            if(imagePoints.size() == num) {
                poseSolver.calibrate(cfg.camera.worldPoints, imagePoints);
                std::cout << "相机标定（PnP）成功！" << std::endl;
                            // 保存标定结果到 calib_result.yaml
                cv::Mat R, T;
                poseSolver.getExtrinsic(R, T);
                std::filesystem::path configDir = std::filesystem::path("/home/delphine/rm/tensorrt10_detect/configs");
                std::string calibPath = (configDir / "calib_result.yaml").string();
                YAML::Emitter out;
                out << YAML::BeginMap;
                out << YAML::Key << "image_points" << YAML::Value << YAML::BeginSeq;
                for (const auto& pt : imagePoints) {
                    out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
                }
                out << YAML::EndSeq;
                out << YAML::Key << "r" << YAML::Value << YAML::Flow << YAML::BeginSeq;
                for (int i = 0; i < 9; ++i) {
                    out << R.at<double>(i);
                }
                out << YAML::EndSeq;
                out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq;
                for (int i = 0; i < 3; ++i) {
                    out << T.at<double>(i);
                }
                out << YAML::EndSeq;
                out << YAML::EndMap;
                std::ofstream fout(calibPath);
                if (fout.is_open()) {
                    fout << out.c_str();
                    fout.close();
                    std::cout << "标定结果已保存到: " << calibPath << std::endl;
                } else {
                    std::cerr << "警告：无法写入标定结果文件 " << calibPath << std::endl;
                }
            } else {
                std::cout << "标定点数不足，程序退出。" << std::endl;
                return -1;
            }
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