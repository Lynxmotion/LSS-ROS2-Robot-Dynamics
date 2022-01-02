//
// Created by guru on 12/30/19.
//

#pragma once

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"
#include <tf2_ros/buffer.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "robot_model_msgs/msg/control_state.hpp"

#include <robotik.h>
//#include "types.h"
#include <state.h>
#include <limb.h>
#include <raf.h>
#include <kinematics.h>
#include <robot_control/trajectory/action.h>
#include <listeners/JointStateListener.h>
#include <listeners/ModelStateListener.h>

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

    void updateRobotState();

    /*
     * Integration of public interface to old Control class here
     */

    ///@brief Update any required trajectories based on current state
    /// Apply trajectories to state
    ///@returns the expected state at this moment in time.
    bool update_target(const State& current, rclcpp::Time _now);

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

    void set_joints(const std::vector<std::string>& joint_names);

    // todo: mapping of standingHeight to feet positions should be moved into here, then control
    //  (1) updates feet TF or a Trajectory updates Control feet TF  (control could have multiple Trajectories running as well)
    //        Control is basically the user-program, it updates feet/arm TF, but typically by instantiating Trajectories
    //  (2) model uses Control info to determine how to compute IK (to fit inner joints based on end-effector chains)
    //  (3) model updates TF, CoM, CoP and dynamics
    Limbs limbs_;




protected:
    void publish();
    void publish_control_state(const robotik::State& current, const robotik::State& target, rclcpp::Time now, std::string prefix="");
    void publish_diagnostics();

    robotik::Model::SharedPtr model_;

    State::SharedPtr target;

    struct {
        robotik::State::SharedPtr state;
    } current;

    rclcpp::Time lastControlUpdate;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);

    robotik::JointStateListener::SharedPtr joint_state_listener;
    robotik::ModelStateListener::SharedPtr model_state_listener;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;

    // detect parameters modification during runtime
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr old_parameter_set_callback;
    rcl_interfaces::msg::SetParametersResult parameter_set_callback(const std::vector<rclcpp::Parameter> & param);

    tf2_ros::Buffer tfBuffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_static_;
    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);        // callback

    // extended joint controller publishers
    robot_model_msgs::msg::ControlState::SharedPtr control_state_msg_;
    rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::ControlState>::SharedPtr control_state_pub_;



    /*
     * Private integration of old Control class here
     */

    rclcpp::Time
    lastUpdate,
    last_static_publish_target;

    Kinematics kinematics;

    // list of actions being performed on the robot limbs/effectors
    trajectory::Actions actions;

    trajectory::Expression expression_from_msg(
            robot_model_msgs::msg::SegmentTrajectory msg,
            std::string default_reference_frame,
            const rclcpp::Time& now);

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
};

} // ns:robot_dynamics
