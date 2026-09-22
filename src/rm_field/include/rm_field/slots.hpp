#pragma once
#include <rm_field/robot_class.hpp>
namespace rm_field {
inline constexpr int kRobotSlotCount=10;
inline constexpr int kOutpostIndex=kRobotSlotCount;
inline constexpr int slot_for(int team, int role) {
 const int offset = team==kTeamRed ? 0 : team==kTeamBlue ? 5 : -1;
 const int i=role>=kClassHero && role<=kClassSentry ? role-kClassHero : -1;
 return offset<0 || i<0 ? -1 : offset+i;
}
}
