//
// Created by guru on 1/2/22.
//

#ifndef ROBOT_DYNAMICS_JOINTCONTROLPUBLISHER_H
#define ROBOT_DYNAMICS_JOINTCONTROLPUBLISHER_H

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

    class JointControlPublisher {
    public:
        using SharedPtr = std::shared_ptr<JointControlPublisher>;
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        JointControlPublisher(
                rclcpp_lifecycle::LifecycleNode& node,
                std::string position_controller,
                std::string effort_controller,
                std::string frame_id = "");
        //JointControlPublisher(rclcpp::Node& node, std::string topic_name = "joint_states", std::string frame_id = "");

        CallbackReturn on_activate();

        CallbackReturn on_deactivate();

        void state(const std::shared_ptr<JointState>& state);

        ///@brief Set the joints to be published to the joint controllers (i.e. ros2_controls)
        /// This will select the subset of joints from the JointState to publish. We don't typically send fixed or
        /// non-transmission joints to the joint controllers.
        void set_joints(const std::vector<std::string>& joint_names);

        // todo: joint_effort is being updated every loop for every joint, this could probably be done better
        void set_joint_effort(std::vector<std::string> joints, double effort, double epsilon = 0.0001);

        [[nodiscard]] inline const std::shared_ptr<JointState>& state() const { return state_; }
        [[nodiscard]] inline std::shared_ptr<JointState>& state() { return state_; }

        ///@brief Publish all joint control messages
        /// Joint state/angles are read from the target state and sent to the joint position/velocity controllers.
        void publish(const rclcpp::Time& now);

    protected:
        std::string frame_id_;
        std::string position_controller_name, effort_controller_name;
        std::shared_ptr<JointState> state_;

        bool efforts_updated;
        rclcpp::Time last_efforts_update;

        // lookup the joint index in state using the index
        JointStateOrdinalMap joint_ordinal_map;

        rclcpp::Logger logger;

        // trajectory publisher
        trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg_;
        rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

        // effort publisher
        std_msgs::msg::Float64MultiArray::SharedPtr joint_efforts_msg_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_efforts_pub_;

        // ability to change the lifecycle of the Ros2 controllers
        lifecycle_msgs::srv::ChangeState::Request::SharedPtr jointctrl_change_state_request_;
        lifecycle_msgs::srv::GetState::Request::SharedPtr jointctrl_get_state_request_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr jointctrl_get_state_client_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr jointctrl_change_state_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture jointctrl_get_state_future_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture jointctrl_change_state_future_;

        void publish_joint_trajectory(const rclcpp::Time& now);
        void publish_efforts(const rclcpp::Time& now);

        // todo: refactor this code into a LifecycleNodeService class for getting and setting service state
        //     also add a ControllerManager one
        void activate_position_controller(bool active);
    };

} // ns:robotik
#endif //ROBOT_DYNAMICS_JOINTCONTROLPUBLISHER_H
