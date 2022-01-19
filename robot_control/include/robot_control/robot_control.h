//
// Created by guru on 12/30/19.
//

#pragma once

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_srvs/srv/empty.hpp>

#include "robot_model_msgs/msg/control_state.hpp"
#include "robot_model_msgs/srv/reset.hpp"
#include "robot_model_msgs/srv/configure_limb.hpp"
#include "robot_model_msgs/srv/set_limb.hpp"

#include <robotik.h>
#include <state.h>
#include <limb.h>
#include <raf.h>
#include <kinematics.h>
#include <listeners/JointStateListener.h>
#include <listeners/ModelStateListener.h>
#include <publishers/JointControlPublisher.h>
#include <robot_control/trajectory/single-effector.h>
#include <robot_control/trajectory/coordinated-effectors.h>
#include <robot_control/trajectory/linear-trajectory.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <string>

// todo: remove this using
using namespace robotik;

namespace  robot_dynamics {

#define PUBLISH_STATIC_TF_EVERY   5.0    // seconds

class Control : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    Control();

    explicit Control(const rclcpp::NodeOptions & options);

    explicit Control(
            const std::string & node_name,
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State &) override;


    ///@brief Apply any active trajectories to limb target state
    /// Active trajectories come from Ros2 actions and apply linear and/or angular transformations to limb
    /// or base frame targets.
    void apply_actions(const State& current, rclcpp::Time _now);

    ///@brief Update any required trajectories based on current state
    /// Apply trajectories to state
    ///@returns the expected state at this moment in time.
    bool update_target(const State& current, rclcpp::Time _now);

    /// @brief Sends target model state as TF data with a "target" prefix
    /// You can add a second RobotModel node to RViz2 and set the prefix to "target" and see a rendered model
    /// representing the target state.
    /// This is optional and is only sent if the `preview_frequency` parameter is set.
    void publish_preview();

    /// @brief Publish progress on any active trajectory actions
    /// This is optional and is only sent if the `progress_frequency` parameter is set.
    void publish_progress();

    /// @brief Publish ControlState messages to the control_state topic
    /// This is optional but highly recommended for your robot control algorithm. In most casees your control
    /// program will only need the information in (RobotDyanmics) ModelState and ControlState.
    /// Published when the `publish_state_frequency` parameter is set.
    void publish_control_state();

    /// @brief Publish diagnostics on how this node is performing
    void publish_diagnostics();

    /// @brief Reset target state from current state
    void resetTarget(const State& current);

    /// @brief Cancel any active trajectory actions
    void resetTrajectory();

    ///@brief return the desired state according to any active trajectories
    inline State::SharedPtr getTargetState() const { return target; }

    inline const ModelInterface& getModelInterface() const { return *model_; }
    inline ModelInterface& getModelInterface() { return *model_; }

protected:
    rclcpp::Time
        lastUpdate,
        last_static_publish_target;

    Kinematics kinematics;

    // list of actions being performed on the robot limbs/effectors
    trajectory::TrajectoryActions actions;

    robotik::Model::SharedPtr model_;

    robotik::State::SharedPtr current;
    State::SharedPtr target;

    // List of base within the robot
    // The root robot base must be 0th element in bases array, other bases may be for example a
    // torso joint.
    BaseStates bases_;

    // arms and/or leg effectors on the robot
    Limbs limbs_;

    // stores offset into state joint position to joint names in joint_trajectory_msg_ collection.
    // index is still verified each use in case joint index position has moved
    //std::vector<std::string> joint_trajectory_names;
    std::vector<size_t> joint_trajectory_index;


    void control_update();

    ///@brief Convert an incoming trajectory message to a Trajectory Expression object
    trajectory::Expression expression_from_msg(
            robot_model_msgs::msg::SegmentTrajectory msg,
            std::string default_reference_frame,
            const rclcpp::Time& now);

    ///@brief Publish a state object's TF and ModelState for previewing
    /// Used to publish either target state or trajectory preview state.
    void publish_preview_state(
            const State& state,
            const std::string& prefix,
            const rclcpp::Time& now,
            rclcpp::Time& last_static_publish);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);

    robotik::JointStateListener::SharedPtr joint_state_listener;
    robotik::ModelStateListener::SharedPtr model_state_listener;
    //robotik::SegmentStateListener::SharedPtr segment_state_listener;  <-- incomplete implementation
    robotik::JointControlPublisher::SharedPtr joint_control_publisher;

    rclcpp::TimerBase::SharedPtr update_timer_, progress_timer_, preview_timer_, publish_state_timer_, diag_timer_;

    lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;

    // detect parameters modification during runtime
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr old_parameter_set_callback;
    rcl_interfaces::msg::SetParametersResult parameter_set_callback(const std::vector<rclcpp::Parameter> & param);

    // extended joint controller publishers
    robot_model_msgs::msg::ControlState::SharedPtr control_state_msg_;
    rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::ControlState>::SharedPtr control_state_pub_;

    // Reset service
    rclcpp::Service<robot_model_msgs::srv::Reset>::SharedPtr reset_service;
    void reset_callback(const std::shared_ptr<robot_model_msgs::srv::Reset::Request> request,
                                 std::shared_ptr<robot_model_msgs::srv::Reset::Response> response);

    // Limb Services
    rclcpp::Service<robot_model_msgs::srv::ConfigureLimb>::SharedPtr configure_limb_service;
    rclcpp::Service<robot_model_msgs::srv::SetLimb>::SharedPtr set_limb_service;
    void configure_limb_callback(const std::shared_ptr<robot_model_msgs::srv::ConfigureLimb::Request> request,
                        std::shared_ptr<robot_model_msgs::srv::ConfigureLimb::Response> response);
    void set_limb_callback(const std::shared_ptr<robot_model_msgs::srv::SetLimb::Request> request,
                        std::shared_ptr<robot_model_msgs::srv::SetLimb::Response> response);

    // Single Trajectory action
    rclcpp_action::Server<robotik::trajectory::TrajectoryAction::EffectorTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp_action::GoalResponse handle_trajectory_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const robotik::trajectory::TrajectoryAction::EffectorTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_trajectory_cancel(
            const std::shared_ptr<robotik::trajectory::TrajectoryAction::GoalHandle> goal_handle);
    void handle_trajectory_accepted(
            const std::shared_ptr<robotik::trajectory::TrajectoryAction::GoalHandle> goal_handle);

    // Coordinated Trajectory action
    rclcpp_action::Server<robotik::trajectory::CoordinatedTrajectoryAction::EffectorTrajectory>::SharedPtr coordinated_trajectory_action_server_;
    rclcpp_action::GoalResponse handle_coordinated_trajectory_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const robotik::trajectory::CoordinatedTrajectoryAction::EffectorTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_coordinated_trajectory_cancel(
            const std::shared_ptr<robotik::trajectory::CoordinatedTrajectoryAction::GoalHandle> goal_handle);
    void handle_coordinated_trajectory_accepted(
            const std::shared_ptr<robotik::trajectory::CoordinatedTrajectoryAction::GoalHandle> goal_handle);

    // Linear Trajectory action
    rclcpp_action::Server<robotik::trajectory::LinearTrajectoryAction::EffectorTrajectory>::SharedPtr linear_trajectory_action_server_;
    rclcpp_action::GoalResponse handle_linear_trajectory_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const robotik::trajectory::LinearTrajectoryAction::EffectorTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_linear_trajectory_cancel(
            const std::shared_ptr<robotik::trajectory::LinearTrajectoryAction::GoalHandle> goal_handle);
    void handle_linear_trajectory_accepted(
            const std::shared_ptr<robotik::trajectory::LinearTrajectoryAction::GoalHandle> goal_handle);
};

} // ns:robot_dynamics
