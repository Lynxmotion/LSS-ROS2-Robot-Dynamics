//
// Created by guru on 1/16/22.
//

#ifndef ROBOT_DYNAMICS_LIFECYCLEMANAGER_H
#define ROBOT_DYNAMICS_LIFECYCLEMANAGER_H

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <state.h>
#include <indexes.h>

namespace robotik {

class LifecycleManager
{
public:
    using SharedPtr = std::shared_ptr<LifecycleManager>;
    using SharedFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;

    ///@brief Self-manage a node
    inline LifecycleManager() = default;

    ///@brief Self manage a node
    explicit LifecycleManager(rclcpp_lifecycle::LifecycleNode& self_managed_node);

    ///@brief Manage another node state from the given host_node
    LifecycleManager(rclcpp_lifecycle::LifecycleNode& host_node, std::string target_node_name);

    ///@brief Manages a node if it contains the typical self_manage parameter set to true
    /// This static function checks if the given node has a self_manage parameter set and if so
    /// returns a LifecycleManager for self-managing it's own state.
    /// If configure_now is set we also send a transition to configure state immediately
    static LifecycleManager::SharedPtr if_self_manage(rclcpp_lifecycle::LifecycleNode& node, bool configure_now = true);

    SharedFuture configure(std::string label = "");
    SharedFuture activate(std::string label = "");
    SharedFuture deactivate(std::string label = "");

    ///@brief initiate a shutdown on the node
    ///@arg shutdown_type - one of lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, TRANSITION_INACTIVE_SHUTDOWN or TRANSITION_ACTIVE_SHUTDOWN
    SharedFuture shutdown(uint8_t shutdown_type, std::string label = "");

protected:
    lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;

    rclcpp::Logger logger;
};

} //ns:robotik
#endif //ROBOT_DYNAMICS_LIFECYCLEMANAGER_H
