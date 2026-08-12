/**
 * @file hungarian.cpp
 * @brief 当前 Tracker 使用的一对一最小代价分配实现。
 * 输入已经包含业务硬门与软代价；本文件只求分配，不知道队伍、坐标或类别语义。
 */
#include "hungarian.hpp"
#include <cmath>

namespace radar_core {
namespace tracker {

float HungarianAlgorithm::Solve(std::vector<std::vector<float>>& DistMatrix, std::vector<int>& Assignment) {
    int nRows = DistMatrix.size();
    if (nRows == 0) return 0.0;
    int nCols = DistMatrix[0].size();

    Assignment.assign(nRows, -1);
    float cost = 0;

    // 注意：类名沿用 HungarianAlgorithm，但当前真实实现是“反复选取全局最小未用
    // 元素”的贪心一对一分配，不是严格 Kuhn-Munkres/Hungarian。它保证行列不重复，
    // 却不保证总成本全局最优；Tracker 的硬 gate 与小规模目标使其成为当前实际路径。
    std::vector<bool> col_used(nCols, false);
    std::vector<bool> row_used(nRows, false);
    
    while(true) {
        float min_cost = std::numeric_limits<float>::max();
        int best_r = -1, best_c = -1;
        
        for (int r = 0; r < nRows; ++r) {
            if (row_used[r]) continue;
            for (int c = 0; c < nCols; ++c) {
                if (col_used[c]) continue;
                if (DistMatrix[r][c] < min_cost) {
                    min_cost = DistMatrix[r][c];
                    best_r = r;
                    best_c = c;
                }
            }
        }
        
        // Tracker 用 1e6 表示硬 gate；这里以 1e4 截断，不把禁配项写入 assignment。
        if (best_r == -1 || min_cost > 1e4) break; // 找不到更多有效匹配
        
        Assignment[best_r] = best_c;
        cost += min_cost;
        row_used[best_r] = true;
        col_used[best_c] = true;
    }
    return cost;
}

} // namespace tracker
} // namespace radar_core
