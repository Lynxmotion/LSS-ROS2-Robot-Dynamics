//
// Created by guru on 12/15/21.
//

#ifndef ROBOT_DYNAMICS_JOINTSTATELISTENER_H
#define ROBOT_DYNAMICS_JOINTSTATELISTENER_H

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <state.h>
#include <indexes.h>

namespace robotik {

class JointStateListener {
public:
    using SharedPtr = std::shared_ptr<JointStateListener>;

    JointStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string topic_name = "joint_states", std::string frame_id = "");
    JointStateListener(rclcpp::Node& node, std::string topic_name = "joint_states", std::string frame_id = "");

    // todo: we probably have to add activate/deactivate to listener classes so they stop processing
    //       messages when main node is not active

    void state(const std::shared_ptr<JointState>& state);

    [[nodiscard]] inline const std::shared_ptr<JointState>& state() const { return state_; }
    [[nodiscard]] inline std::shared_ptr<JointState>& state() { return state_; }

protected:
    std::string frame_id_;
    std::shared_ptr<JointState> state_;

    // todo: support dynamic_joint_state control_msgs?
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // lookup the joint index in state using the index
    JointStateOrdinalMap joint_ordinal_map;

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // ns:robotik

#endif //ROBOT_DYNAMICS_JOINTSTATELISTENER_H
