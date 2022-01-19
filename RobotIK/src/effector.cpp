//
// Created by guru on 1/7/22.
//

#include <effector.h>

#include <robot_model_msgs/msg/segment_trajectory.hpp>
#include <robot_model_msgs/msg/limb.hpp>

namespace robotik {

BaseEffector::BaseEffector(
        const std::shared_ptr<KDL::Tree>& _tree,
        std::string _base_link)
    : tree(_tree), link(_base_link)
{
}

Effector::Mode effector_mode_from_msg(int8_t m)
{
    switch(m) {
        case robot_model_msgs::msg::SegmentTrajectory::LIMP: return Effector::Mode::Limp;
        case robot_model_msgs::msg::SegmentTrajectory::HOLD: return Effector::Mode::Hold;
        case robot_model_msgs::msg::SegmentTrajectory::SUPPORT: return Effector::Mode::Support;
        case robot_model_msgs::msg::SegmentTrajectory::AUTO_SUPPORT: return Effector::Mode::AutoSupport;
        default: return Effector::Mode::Unassigned;
    }
}

int8_t effector_mode_to_msg(Effector::Mode m)
{
    switch(m) {
        case Effector::Mode::Limp: return robot_model_msgs::msg::SegmentTrajectory::LIMP;
        case Effector::Mode::Hold: return robot_model_msgs::msg::SegmentTrajectory::HOLD;
        case Effector::Mode::Support: return robot_model_msgs::msg::SegmentTrajectory::SUPPORT;
        case Effector::Mode::AutoSupport: return robot_model_msgs::msg::SegmentTrajectory::AUTO_SUPPORT;
        default: return Effector::Mode::Unassigned;
    }
}

int8_t effector_status_to_msg(Effector::Status s)
{
    switch(s) {
        case Effector::Status::Limp: return robot_model_msgs::msg::Limb::LIMP;
        case Effector::Status::Holding: return robot_model_msgs::msg::Limb::HOLDING;
        case Effector::Status::Supporting: return robot_model_msgs::msg::Limb::SUPPORTING;
        case Effector::Status::Seeking: return robot_model_msgs::msg::Limb::SEEKING;
        default: return robot_model_msgs::msg::Limb::LIMP;
    }
}

Effector::Status effector_status_from_msg(int8_t s)
{
    switch(s) {
        case robot_model_msgs::msg::Limb::LIMP: return Effector::Status::Limp;
        case robot_model_msgs::msg::Limb::HOLDING: return Effector::Status::Holding;
        case robot_model_msgs::msg::Limb::SUPPORTING: return Effector::Status::Supporting;
        case robot_model_msgs::msg::Limb::SEEKING: return Effector::Status::Seeking;
        default: return Effector::Status::Limp;
    }
}


void Effector::State::apply(const KDL::Frame& pose, CoordinateMask mask)
{
    if(mask == CoordinateMask::All) {
        target = pose;
    } else if(mask != CoordinateMask::None) {
        if((mask & CoordinateMask::X) != CoordinateMask::None)
            target.p.x(pose.p.x());
        if((mask & CoordinateMask::Y) != CoordinateMask::None)
            target.p.y(pose.p.y());
        if((mask & CoordinateMask::Z) != CoordinateMask::None)
            target.p.z(pose.p.z());

        if((mask & CoordinateMask::RPY) == CoordinateMask::RPY) {
            // apply full rotation
            target.M = pose.M;
        } else if((mask & CoordinateMask::RPY) != CoordinateMask::None) {
            // get the pose XYZ
            double pose_roll, pose_pitch, pose_yaw;
            pose.M.GetRPY(pose_roll, pose_pitch, pose_yaw);

            double target_roll, target_pitch, target_yaw;
            target.M.GetRPY(target_roll, target_pitch, target_yaw);

            if((mask & CoordinateMask::Roll) != CoordinateMask::None)
                target_roll = pose_roll;
            if((mask & CoordinateMask::Pitch) != CoordinateMask::None)
                target_pitch = pose_pitch;
            if((mask & CoordinateMask::Yaw) != CoordinateMask::None)
                target_yaw = pose_yaw;

            target.M = KDL::Rotation::RPY(target_roll, target_pitch, target_yaw);
        }
    }
}


template<>
BaseStates::iterator BaseStates::find(const std::string& s) {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}
template<>
BaseStates::const_iterator BaseStates::find(const std::string& s) const {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}

} //ns:robotik