//
// Created by guru on 3/30/20.
//

#ifndef LSS_HUMANOID_CONTROL_H
#define LSS_HUMANOID_CONTROL_H

#include "types.h"
#include "state.h"
#include "trajectory.h"
#include "interaction.h"
#include "visual.h"

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


#define TRAJECTORY_ACTION_SERVER

namespace robotik {

class Control : public LimbInteractionController {
public:
    using SharedPtr = std::shared_ptr<Control>;

    bool active;
    bool override;
    bool balance;

    // if 0 or greater, then restart preview time given seconds after end of trajectory,
    // if negative then don't loop, the preview will stay active at the last position until cancelled
    double loopPreviewDelay;
    bool loopPreview;

    State::SharedPtr target;
    std::string interaction_namespace_;

    ///@brief the composite trajectory of active end-effectors
    Trajectory::SharedPtr trajectory;

    Control(std::string interaction_namespace = "target");

    void activate(Model::SharedPtr model, rclcpp_lifecycle::LifecycleNode& node);
    void deactivate();


#if 0
    ///@brief Computes a new trajectory to move the robot to the standing height
    void setBaseHeight(double pos);
    void setBaseHeight(double pos, const State& current, rclcpp::Time _stamp);
    inline double getBaseHeight() const { return model_->baseHeight; }
#endif

    std::string interaction_namespace() override;

    ///@brief show the target pose and enable manipulators
    void enableManipulators(const State& current, rclcpp::Time _stamp);

    ///@brief Handle interaction events and modify the target position and trajectory
    bool interact(InteractionEvent& ev) override;


    void enable_all_joints(double e);
    void enable_arms(double e);
    void enable_legs(double e);
    void enable_limb(const char* limb, double e);
    void enable_limb(const Limb& limb, double e);
    void enable_limbs(double e, std::function<bool(const Limb&)> filter);

    // clear any trajectory or control state
    void clear_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub);
    void clear_target_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub);
    void clear_trajectory_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub);

    ///@brief Update any required trajectories based on current state
    /// Apply trajectories to state
    ///@returns the expected state at this moment in time.
    bool update(const State& current, rclcpp::Time _now);

    bool publish();
    void publish_progress();
    void publish_visuals(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub);

    void resetTarget(const State& current);
    void resetTrajectory();

    ///@brief return the desired state according to any active trajectories
    inline State::SharedPtr getTargetState() const { return target; }

    inline const ModelInterface& getModelInterface() const { return *model_; }
    inline ModelInterface& getModelInterface() { return *model_; }

    void set_joint_controller_active(bool active = true);

    ///@brief Add a trajectory
    void addTrajectory(Trajectory& traj) {
        // todo: control should compose trajectories not replace them
        trajectory = std::make_shared<Trajectory>(traj);
    }

    //@brief Render a trajectory request into segment path splines
    bool renderTrajectory(const State& initial_state, rclcpp::Time _now);

    void set_joints(const std::vector<std::string>& joint_names);

    // todo: mapping of standingHeight to feet positions should be moved into here, then control
    //  (1) updates feet TF or a Trajectory updates Control feet TF  (control could have multiple Trajectories running as well)
    //        Control is basically the user-program, it updates feet/arm TF, but typically by instantiating Trajectories
    //  (2) model uses Control info to determine how to compute IK (to fit inner joints based on end-effector chains)
    //  (3) model updates TF, CoM, CoP and dynamics

    StateVisual::SharedPtr visual;
    StateVisual::SharedPtr trajectoryVisual;

    bool executeTrajectory;

protected:
    rclcpp::Time lastUpdate;
    State::SharedPtr trajectoryState;

    trajectory::Expression expression_from_msg(
            humanoid_model_msgs::msg::SegmentTrajectory msg,
            std::string default_reference_frame,
            const rclcpp::Time& now);

    // subscribe to trajectory messages
    rclcpp::Subscription<humanoid_model_msgs::msg::MultiSegmentTrajectory>::SharedPtr trajectory_sub_;
    void trajectory_callback(humanoid_model_msgs::msg::MultiSegmentTrajectory::SharedPtr msg);

    // trajectory publisher
    trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg_;
    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

    // effort publisher
    std_msgs::msg::Float64MultiArray::SharedPtr joint_efforts_msg_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_efforts_pub_;
    bool efforts_updated;
    void publish_efforts();

#ifndef TRAJECTORY_ACTION_SERVER
    humanoid_model_msgs::msg::MultiTrajectoryProgress::SharedPtr progress_msg_;
    rclcpp_lifecycle::LifecyclePublisher<humanoid_model_msgs::msg::MultiTrajectoryProgress>::SharedPtr progress_pub_;
#else
    // trajectory progress publisher
    rclcpp_action::Server<trajectory::EffectorTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp_action::GoalResponse handle_trajectory_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const trajectory::EffectorTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_trajectory_cancel(
            const std::shared_ptr<trajectory::GoalHandle> goal_handle);
    void handle_trajectory_accepted(const std::shared_ptr<trajectory::GoalHandle> goal_handle);
#endif

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

    void print_debug(const char* label, const State& state);

    inline rclcpp::Logger get_logger() { return rclcpp::get_logger("robot_dynamics:control"); }
};

} // ns::robot

#endif //LSS_HUMANOID_CONTROL_H
