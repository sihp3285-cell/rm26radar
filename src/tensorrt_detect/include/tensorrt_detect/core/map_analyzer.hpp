#ifndef MAP_ANALYZER_HPP
#define MAP_ANALYZER_HPP

#include <vector>
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "robot_id.hpp"

class MapAnalyzer{
    public:
        explicit MapAnalyzer(int our_team_id = robot_id::RED);
        void evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets);
        int engineer_on_island() const {return engineer_on_island_;}
        int opponent_attack() const {return opponent_attack_;}
        int our_attack() const {return our_attack_;}
    private:
        int our_team_id_ = robot_id::RED;
        int my_team_ = robot_id::RED;
        int opponent_team_ = robot_id::BLUE;

        int engineer_on_island_ = 0;
        int opponent_attack_ = 0;
        int our_attack_ = 0;

        static std::pair<float, float> toFieldCoord(float world_x, float world_z);

};


#endif


