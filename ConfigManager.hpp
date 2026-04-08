#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

struct Config
{

    std::string modelPath;              
    std::string armorModelPath;            
    std::string classifyModelPath;         
    std::vector<std::string> classNames;   
    bool showFlag;                        
    bool isflip;                          

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Point3f> worldPoints; 
    std::vector<cv::Point2f> worldPoints2D; 
    std::string mapPath;
    std::vector<cv::Point2f> MapPoints;
    int requirePointsNum;
    int maxMissCount;
    int maxhistory;
    float distheshold;

    Config(const std::string &configPath)
    {
        try {
            YAML::Node config = YAML::LoadFile(configPath);

            modelPath = config["modelPath"].as<std::string>();
            armorModelPath = config["armorModelPath"].as<std::string>();
            classifyModelPath = config["classifyModelPath"].as<std::string>();
            showFlag = config["showFlag"].as<bool>();
            mapPath = config["mapPath"].as<std::string>("/home/delphine/rm/map.png"); 
            requirePointsNum = config["requirePointsNum"].as<int>();
            maxMissCount = config["maxMissCount"].as<int>();
            maxhistory = config["maxhistory"].as<int>();
            distheshold = config["distheshold"].as<float>();


            classNames.clear();
            if (config["classNames"].IsSequence()) {
                for (const auto &item : config["classNames"])
                    classNames.emplace_back(item.as<std::string>());
            } else {
                for (const auto &item : config["classNames"])
                    classNames.emplace_back(item.second.as<std::string>());
            }

            YAML::Node camNode = config["cameraMatrix"];
            if (camNode && camNode.IsSequence() && camNode.size() == 9) {
                cameraMatrix = cv::Mat::zeros(3, 3, CV_64F); 
                for (int i = 0; i < 9; ++i) {
                    cameraMatrix.at<double>(i / 3, i % 3) = camNode[i].as<double>(); 
                }
            } else {
                throw std::runtime_error("YAML 中 cameraMatrix 格式错误或缺失！必须是 9 个数字。");
            }

            YAML::Node distNode = config["distCoeffs"];
            if (distNode && distNode.IsSequence()) {
                int distSize = distNode.size();
                distCoeffs = cv::Mat::zeros(1, distSize, CV_64F);
                for (int i = 0; i < distSize; ++i) {
                    distCoeffs.at<double>(0, i) = distNode[i].as<double>();
                }
            } else {
                throw std::runtime_error("YAML 中 distCoeffs 格式错误或缺失！");
            }

            worldPoints.clear();
            worldPoints2D.clear();
            YAML::Node wpNode = config["worldPoints"];
            if (wpNode && wpNode.IsSequence()) {
                for (const auto &item : wpNode) {
                    float x = item[0].as<float>();
                    float y = item[1].as<float>();
                    float z = item[2].as<float>();
                    worldPoints.emplace_back(cv::Point3f(x, y, z));
                    worldPoints2D.emplace_back(cv::Point2f(x, y));
                }
            }
            YAML::Node mpNode = config["MapPoints"];
            if (mpNode && mpNode.IsSequence()) {
                for (const auto &item : mpNode) {
                    float x = item[0].as<float>();
                    float y = item[1].as<float>();
                    MapPoints.emplace_back(cv::Point2f(x, y));
                }
            }
            showFlag = config["showFlag"].as<bool>();
            if (config["isflip"]) {
                isflip = config["isflip"].as<bool>();
            } else {
                isflip = false;
            }
            std::cout << "配置成功" << std::endl;

        } catch (const std::exception &e) {
            std::cerr << "配置失败" << e.what() << std::endl;
        }
    }
};