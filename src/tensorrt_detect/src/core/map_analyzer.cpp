#include "map_analyzer.hpp"

MapAnalyzer::MapAnalyzer(int our_team_id)
    : our_team_id_(our_team_id),
      my_team_(our_team_id),
      opponent_team_((our_team_id == robot_id::RED) ? robot_id::BLUE : robot_id::RED)
{}


std::pair<float, float> MapAnalyzer::toFieldCoord(float world_x, float world_z)
{
    float field_x = world_z + 14.0f;
    float field_y = world_x + 7.5f;
    return {field_x, field_y};

}

void MapAnalyzer::evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets)
{
        engineer_on_island_ = 0;
        opponent_attack_ = 0;
        our_attack_ = 0;
        int opponent_attack_count = 0;
        int our_attack_count = 0;



        for (const auto& target : targets){
            if (!target.valid) continue;
            if (target.team_id == robot_id::UNKNOWN) continue;
            auto [fx,fy] = toFieldCoord(target.world_x, target.world_z);

            float dist_red = fx;
            float dist_blue = 28.0f - fx;
            float dist_from_opponent = (opponent_team_ == robot_id::RED) ? dist_red : dist_blue;
            float dist_from_our = (my_team_ == robot_id::RED) ? dist_red : dist_blue;

            if(target.team_id == opponent_team_ && target.class_id == robot_id::R2){
            if (dist_from_opponent >= 10.0f && dist_from_opponent <= 14.0f &&
                fy >= 6.0f && fy <= 9.0f) {
                engineer_on_island_ = 1;
            }
           }

            if (target.team_id == opponent_team_ && 
           (target.class_id == robot_id::R2 || target.class_id == robot_id::R3 || target.class_id == robot_id::R4)){
            if (dist_from_opponent > 10.0f) {
                opponent_attack_count++;
            }
        }
        
        if (target.team_id == my_team_ && 
           (target.class_id == robot_id::R2 || target.class_id == robot_id::R3 || target.class_id == robot_id::R4)) {
            if (dist_from_our > 14.0f) {
                our_attack_count++;
            }
        }   
    }
    if (opponent_attack_count >= 2) {
        opponent_attack_ = 1;
    }
    if (our_attack_count >= 2) {
        our_attack_ = 1;
    }
}
