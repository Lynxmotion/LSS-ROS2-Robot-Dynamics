//
// Created by guru on 12/29/21.
//

#include "robot_control/trajectory/action.h"


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
