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

void TrajectoryActions::complete(std::string member_name,
                                 Limbs& limbs,
                                 const Model& model,
                                 const rclcpp::Time& now,
                                 int code)
{
    for(auto a=begin(); a != end(); ) {
        if((*a)->complete(member_name, limbs, model, now, code)) {
            // remove action
            erase(a);
        } else
            a++;
    }
}

} // ns:robotik
