//
// Created by guru on 12/29/21.
//

#include "robot_control/trajectory/action.h"

#define to_hex(b) (((b) < 10) ? ('0'+(b)) : ('a'+((b)-10)))

std::ostream & operator << (std::ostream &out, const rclcpp_action::GoalUUID& uuid)
{
    char s[33];
    char* p = s;
    for(int i=0; i < 16; i++) {
        auto b = uuid[i];
        *p++ = to_hex(b & 0x0f);
        *p++ = to_hex((b & 0xf0) >> 4);
    }
    *p = 0;
    out << s;
    return out;
}

namespace robotik::trajectory {

void TrajectoryActions::append(TrajectoryActionInterface::SharedPtr action)
{
    auto itr = insert(end(), action);
    if(itr == end())
        throw robotik::Exception(RE_FAILED, "cannot add action");
}

bool TrajectoryActions::complete(const rclcpp_action::GoalUUID uuid,
              Limbs& limbs,
              const Model& model,
              const rclcpp::Time& now,
              int code)
{
    for(auto a=begin(); a != end(); a++) {
        if((*a)->uuid == uuid) {
            // found the action, complete it
            (*a)->complete(limbs, model, now, code);
            // remove action
            erase(a);
            return true;
        }
    }
    return false;
}

bool TrajectoryActions::complete(std::string member_name,
                                 Limbs& limbs,
                                 const Model& model,
                                 const rclcpp::Time& now,
                                 int code)
{
    int removals = 0;
    for(auto a=begin(); a != end(); ) {
        if((*a)->complete(member_name, limbs, model, now, code)) {
            // remove action
            erase(a);
            removals++;
        } else
            a++;
    }
    return removals > 0;
}

} // ns:robotik
