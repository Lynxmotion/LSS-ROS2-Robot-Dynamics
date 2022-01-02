//
// Created by guru on 3/30/20.
//

#ifndef LSS_HUMANOID_CONTROL_H
#define LSS_HUMANOID_CONTROL_H

#include "types.h"
#include "state.h"

#include <robot_control/trajectory.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


namespace robotik {

#define PUBLISH_STATIC_TF_EVERY   5.0    // seconds

class Control {
public:
    using SharedPtr = std::shared_ptr<Control>;

    bool active;

    Model::SharedPtr model_;
    State::SharedPtr target;

    Control();

    void activate(Model::SharedPtr model, rclcpp_lifecycle::LifecycleNode& node);
    void deactivate();

    ///@brief Update any required trajectories based on current state
    /// Apply trajectories to state
    ///@returns the expected state at this moment in time.
    bool update(const State& current, rclcpp::Time _now);

    bool publish_control();

    void publish_efforts();

    void publish_target_preview(const rclcpp::Time& now, std::string prefix = "target");

    void resetTarget(const State& current);
    void resetTrajectory();

    ///@brief return the desired state according to any active trajectories
    inline State::SharedPtr getTargetState() const { return target; }

    inline const ModelInterface& getModelInterface() const { return *model_; }
    inline ModelInterface& getModelInterface() { return *model_; }

    void set_joint_controller_active(bool active = true);

    //@brief Render a trajectory request into segment path splines
    bool renderTrajectory(const State& initial_state, rclcpp::Time _now);

    void set_joints(const std::vector<std::string>& joint_names);

    // todo: mapping of standingHeight to feet positions should be moved into here, then control
    //  (1) updates feet TF or a Trajectory updates Control feet TF  (control could have multiple Trajectories running as well)
    //        Control is basically the user-program, it updates feet/arm TF, but typically by instantiating Trajectories
    //  (2) model uses Control info to determine how to compute IK (to fit inner joints based on end-effector chains)
    //  (3) model updates TF, CoM, CoP and dynamics

    bool executeTrajectory;

protected:
    rclcpp::Time
        lastUpdate,
        last_static_publish_target;

    Kinematics kinematics;

    trajectory::Expression expression_from_msg(
            robot_model_msgs::msg::SegmentTrajectory msg,
            std::string default_reference_frame,
            const rclcpp::Time& now);

    // subscribe to trajectory messages
    rclcpp::Subscription<robot_model_msgs::msg::MultiSegmentTrajectory>::SharedPtr trajectory_sub_;
    void trajectory_callback(robot_model_msgs::msg::MultiSegmentTrajectory::SharedPtr msg);

    // trajectory publisher
    trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg_;
    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

    // effort publisher
    std_msgs::msg::Float64MultiArray::SharedPtr joint_efforts_msg_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_efforts_pub_;
    bool efforts_updated;

    // trajectory progress publisher
    rclcpp_action::Server<trajectory::EffectorTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp_action::GoalResponse handle_trajectory_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const trajectory::EffectorTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_trajectory_cancel(
            const std::shared_ptr<trajectory::GoalHandle> goal_handle);
    void handle_trajectory_accepted(const std::shared_ptr<trajectory::GoalHandle> goal_handle);

    // stores offset into state joint position to joint names in joint_trajectory_msg_ collection.
    // index is still verified each use in case joint index position has moved
    //std::vector<std::string> joint_trajectory_names;
    std::vector<size_t> joint_trajectory_index;

    // ability to change the lifecycle of the Ros2 controllers
    lifecycle_msgs::srv::ChangeState::Request::SharedPtr jointctrl_change_state_request_;
    lifecycle_msgs::srv::GetState::Request::SharedPtr jointctrl_get_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr jointctrl_get_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr jointctrl_change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture jointctrl_get_state_future_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture jointctrl_change_state_future_;

    ///@brief Publish a state object's TF and ModelState for previewing
    /// Used to publish either target state or trajectory preview state.
    void publish_preview_state(
            const State& state,
            const std::string& prefix,
            const rclcpp::Time& now,
            rclcpp::Time& last_static_publish);

    inline rclcpp::Logger get_logger() { return rclcpp::get_logger("robot_dynamics:control"); }
};

} // ns::robot

#endif //LSS_HUMANOID_CONTROL_H
