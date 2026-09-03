/**
 * @file map_analyzer.hpp
 * @brief 从稳定 WorldTarget 快照提取少量场地区域战术状态。
 * MapNode 每帧调用 evaluate()；本类不绘图，也不改变 Tracker/Position Prior。
 */
#ifndef MAP_ANALYZER_HPP
#define MAP_ANALYZER_HPP

#include <rm_field/robot_class.hpp>

#include <vector>
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "robot_id.hpp"

class MapAnalyzer{
    public:
        /** 初始化默认敌我队伍；后续通常由 setTeamByFlip() 按 UI 视角覆盖。 */
        explicit MapAnalyzer(int our_team_id = robot_id::BLUE);
        /** 清零并重算当前 WorldTarget 快照的四项战术标志。 */
        void evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets);
        /** 指定 world_x 到 field_x 是否需要镜像，影响区域多边形判定。 */
        void setFieldXFlip(bool flip) { field_x_flip_ = flip; }
        /** 按显示视角同步 my_team/opponent_team；不修改目标消息本身。 */
        void setTeamByFlip(bool flip_team) {
            // 公式统一来自 rm_field/robot_class.hpp：false=我方蓝，true=我方红。
            my_team_ = rm_field::own_team_for_view(flip_team);
            opponent_team_ = rm_field::opponent_team_for_view(flip_team);
        }
        /** 返回工程位于资源岛的阵营编码：0 无、1 我方、2 敌方。 */
        int engineer_on_island() const {return engineer_on_island_;}
        /** 返回敌方是否进入代码定义的进攻区域。 */
        int opponent_attack() const {return opponent_attack_;}
        /** 返回我方是否进入代码定义的进攻区域。 */
        int our_attack() const {return our_attack_;}
        /** 返回敌方是否接近我方堡垒区域。 */
        int opponent_near_fortress() const {return opponent_near_fortress_;}
    private:
        int our_team_id_ = robot_id::BLUE;
        int my_team_ = robot_id::BLUE;
        int opponent_team_ = robot_id::RED;
        bool field_x_flip_ = false;

        int engineer_on_island_ = 0; // 0 无、1 我方、2 敌方工程位于代码定义区域。
        int opponent_attack_ = 0;
        int our_attack_ = 0;
        int opponent_near_fortress_ = 0;

        /** 把 world(x,z) 转成区域判定使用的 field(x,y)，必要时镜像 x。 */
        std::pair<float, float> toFieldCoord(float world_x, float world_z) const;

};


#endif
