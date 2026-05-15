#ifndef MAP_ANALYZER_HPP
#define MAP_ANALYZER_HPP

#include <vector>
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "robot_id.hpp"

class MapAnalyzer{
    public:
        explicit MapAnalyzer(int our_team_id = robot_id::BLUE);
        void evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets);
        void setFieldXFlip(bool flip) { field_x_flip_ = flip; }
        void setTeamByFlip(bool flip_team) {
            // flip_team=false: 蓝方视角 → 我方=蓝方
            // flip_team=true : 红方视角 → 我方=红方
            if (flip_team) {
                my_team_ = robot_id::RED;
                opponent_team_ = robot_id::BLUE;
            } else {
                my_team_ = robot_id::BLUE;
                opponent_team_ = robot_id::RED;
            }
        }
        int engineer_on_island() const {return engineer_on_island_;}
        int opponent_attack() const {return opponent_attack_;}
        int our_attack() const {return our_attack_;}
        int opponent_near_fortress() const {return opponent_near_fortress_;}
    private:
        int our_team_id_ = robot_id::BLUE;
        int my_team_ = robot_id::BLUE;
        int opponent_team_ = robot_id::RED;
        bool field_x_flip_ = false;

        int engineer_on_island_ = 0;
        int opponent_attack_ = 0;
        int our_attack_ = 0;
        int opponent_near_fortress_ = 0;

        std::pair<float, float> toFieldCoord(float world_x, float world_z) const;

};


#endif


