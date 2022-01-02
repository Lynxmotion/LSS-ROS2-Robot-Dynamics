//
// Created by guru on 12/15/21.
//

#include "listeners/JointStateListener.h"

#include <utility>
#include "types.h"

namespace robotik {


JointStateListener::JointStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string topic_name, std::string frame_id)
: frame_id_(std::move(frame_id))
{
    // subscribe to joint state messages
    joint_state_subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
            topic_name, 10,
            std::bind(&JointStateListener::joint_states_callback, this, std::placeholders::_1));
}

JointStateListener::JointStateListener(rclcpp::Node& node, std::string topic_name, std::string frame_id)
: frame_id_(std::move(frame_id))
{
    // subscribe to joint state messages
    joint_state_subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
            topic_name, 10,
            std::bind(&JointStateListener::joint_states_callback, this, std::placeholders::_1));
}

void JointStateListener::state(const std::shared_ptr<JointState>& state)
{
    if(state)
        state_ = state;
    else
        state_.reset();
}

void JointStateListener::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(msg->header.frame_id != frame_id_)
        return;

    if(!state_)
        state_ = std::make_shared<JointState>();

    // keep track of all joint positions
    auto positions = msg->position.size();
    auto velocities = msg->velocity.size();
    auto efforts = msg->effort.size();

    // ensure our index is at least as big as the message
    if(msg->name.size() > joint_ordinal_map.size())
        joint_ordinal_map.resize(msg->name.size());

    unsigned int mask =
            (positions ? robotik::POSITION : 0) |
            (velocities ? robotik::VELOCITY : 0) |
            (efforts ? robotik::EFFORT : 0);

    // ensure we have the necessary state PVE arrays allocated
    state_->alloc(mask);

    for(size_t i=0, _i=msg->name.size(); i<_i; i++) {
        auto joint_name = msg->name[i];
        auto jp = joint_ordinal_map.resolve_or_insert(*state_, i, joint_name, mask);
        if(jp >= 0) {
            state_->joints_updated[jp] = true;
            state_->position(jp) = (i < positions) ? msg->position[i] : 0;
            state_->velocity(jp) = (i < velocities) ? msg->velocity[i] : 0;
            state_->effort(jp) = (i < efforts) ? msg->effort[i] : 0;
        }
    }
    state_->lastJointStateUpdate = rclcpp::Time(msg->header.stamp);
}


}
