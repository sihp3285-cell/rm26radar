/**
 * @file hungarian.hpp
 * @brief 名称兼容 Hungarian、实际采用贪心全局最小边的轨迹—检测分配器。
 *
 * cost matrix 的行是现有 PhysicalTrack，列是本帧 WorldMeasurement；硬 gate 失败项
 * 由调用方写成极大代价，有限项由像素距离、世界距离和身份软惩罚组成。当前实现
 * 保证一对一但不保证严格最小总成本，阅读 Tracker 行为时应以此为准。
 */
#pragma once

#include <vector>
#include <iostream>
#include <limits>

namespace radar_core {
namespace tracker {

class HungarianAlgorithm {
public:
    /** 构造无状态的一对一贪心分配器，不预分配矩阵缓存。 */
    HungarianAlgorithm() {}
    /** 无动态资源，仅保留显式析构以兼容原接口。 */
    ~HungarianAlgorithm() {}

    // 返回最小总代价；Assignment 按“行/轨迹”索引，值是选中的“列/检测”索引，
    // 无匹配为 -1。调用方仍需检查极大代价，算法本身不知道业务 gate 含义。
    float Solve(std::vector<std::vector<float>>& DistMatrix, std::vector<int>& Assignment);
};

} // namespace tracker
} // namespace radar_core
