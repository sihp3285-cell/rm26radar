#include "map_analyzer.hpp"

MapAnalyzer::MapAnalyzer(int our_team_id)
    : our_team_id_(our_team_id),
      my_team_(our_team_id),
      opponent_team_((our_team_id == robot_id::RED) ? robot_id::BLUE : robot_id::RED)
{}

std::pair<float, float> MapAnalyzer::toFieldCoord(float world_x, float world_z) const
{
    // field_x_flip_ 用于适配不同标定/素材中 world_z 正方向不一致的问题。
    // false: world_z 正方向指向蓝方（蓝方基地 world_z ≈ +14）
    // true : world_z 正方向指向红方（红方基地 world_z ≈ +14）
    float field_x = field_x_flip_ ? (world_z + 14.0f) : (14.0f - world_z);
    float field_y = world_x + 7.5f;
    return {field_x, field_y};
}

void MapAnalyzer::evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets)
{
    engineer_on_island_ = 0;
    opponent_attack_ = 0;
    our_attack_ = 0;
    opponent_near_fortress_ = 0;
    int opponent_attack_count = 0;
    int our_attack_count = 0;
    bool our_engineer_on_island = false;
    bool engineer_on_opponent_island = false;
    bool opponent_near_fortress = false;
    

    for (const auto& target : targets) {
        if (!target.valid) continue;
        if (target.team_id == robot_id::UNKNOWN) continue;
        auto [fx, fy] = toFieldCoord(target.world_x, target.world_z);

        float dist_red = fx;
        float dist_blue = 28.0f - fx;
        float dist_from_opponent = (opponent_team_ == robot_id::RED) ? dist_red : dist_blue;
        float dist_from_our = (my_team_ == robot_id::RED) ? dist_red : dist_blue;

        if (target.class_id == robot_id::R2) {
            if (target.team_id == my_team_ && dist_from_our >= 10.0f && dist_from_our <= 16.0f &&
                fy >= 6.0f && fy <= 9.0f) {
                our_engineer_on_island = true;
            } else if (target.team_id == opponent_team_ && dist_from_opponent >= 10.0f && dist_from_opponent <= 16.0f &&
                       fy >= 6.0f && fy <= 9.0f) {
                engineer_on_opponent_island = true;
            }
        }

        if (target.team_id == opponent_team_ && 
            (target.class_id == robot_id::R1 || target.class_id == robot_id::R3 || target.class_id == robot_id::R4 || target.class_id == robot_id::S)) {
            if (dist_from_opponent > 18.0f) {
                opponent_attack_count++;
            }
            float our_fortress_x = (my_team_ == robot_id::RED) ? 6.0f : 22.0f;
            float our_fortress_y = 7.5f;

            float dx = fx - our_fortress_x;
            float dy = fy - our_fortress_y;
            float dist2 = dx * dx + dy * dy;

            float radius = 1.8f;

            if (dist2 <= radius * radius) {
                opponent_near_fortress = true;
            }
        }

        if (target.team_id == my_team_ && 
            (target.class_id == robot_id::R1 || target.class_id == robot_id::R3 || target.class_id == robot_id::R4 || target.class_id == robot_id::S)) {
            if (dist_from_our > 14.0f) {
                our_attack_count++;
            }
        }
    }

    if (our_engineer_on_island) {
        engineer_on_island_ = 1;
    } else if (engineer_on_opponent_island) {
        engineer_on_island_ = 2;
    }

    if (opponent_attack_count >= 2) {
        opponent_attack_ = 1;
    }
    if (our_attack_count >= 2) {
        our_attack_ = 1;
    }
    if (opponent_near_fortress) {
        opponent_near_fortress_ = 1;
    }
}
