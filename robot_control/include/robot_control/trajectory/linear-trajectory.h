//
// Created by guru on 1/3/22.
//

#ifndef ROBOT_DYNAMICS_LINEAR_TRAJECTORY_H
#define ROBOT_DYNAMICS_LINEAR_TRAJECTORY_H

#include <robot_control/trajectory/action.h>
#include <tween.h>

#include <robot_model_msgs/action/linear_effector_trajectory.hpp>

namespace robotik::trajectory {

class LinearTrajectoryAction : public TrajectoryActionInterface
{
public:
    using EffectorTrajectory = robot_model_msgs::action::LinearEffectorTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<EffectorTrajectory>;

    LinearTrajectoryAction(
            BaseStates& bases,
            Limbs& limbs,
            const rclcpp::Time& now,
            std::shared_ptr<GoalHandle> goal_handle);

    [[nodiscard]] std::string type() const override;

    /// Return the start and end time of this action
    [[nodiscard]] TimeRange time_range() const override;

    [[nodiscard]] trajectory::RenderState render_state() const override;

    [[nodiscard]] bool expired(const rclcpp::Time& now) const override;

    bool render(RenderingInterface& env) override;

    /// Apply the action request to the given state
    void apply(const rclcpp::Time& now) override;

    /// tell the client this action has completed
    void complete(
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS) override;

    /// tell the client a member of this action is being cancelled
    /// Other members are still being controlled and progress on those members will continue. If there are no other
    /// members remaining then the entire action will be cancelled using the `cancel(state, now)` function.
    /// It is ok, to call this function with any member_name and those not in this action group will be ignored with
    /// nothing sent to the client.
    /// returns true if all members have expired and this action has thus been entirely cancelled
    bool complete(
            std::string member_name,
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS) override;

    ///@brief Called to indicate the user requested this action be cancelled
    CancelResponse cancel(
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS) override;

    /// update the client on the progress of the action
    void send_feedback(const rclcpp::Time& now) override;

protected:
    class Member {
    public:
        bool is_limb;
        Effector::State* effector;
        //Limbs::iterator limb;       // will be an end() iterator if we are interacting with a base or segment
        //BaseStates::iterator base;
        KDL::Twist velocity;
        KDL::Twist acceleration;
        Tween<KDL::Vector> linear_velocity_tweener;
        Tween<KDL::Vector> angular_velocity_tweener;
    };
    BaseStates& bases_;
    Limbs& limbs_;
    double ts_;
    double last_apply_;
    // todo: add sync_duration to linear_traj - we calculate time to reach target velocity, then tween velocity based on time, so just take the max duration then instead of per element
    double linear_acceleration_;
    double angular_acceleration_;
    std::map<std::string, Member> members;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_ptr<EffectorTrajectory::Feedback> feedback_;

    inline KDL::Frame get_target_transform(const std::pair<std::string, Member>& m) const {
        // todo: can we get rid of this static cast?
        return m.second.is_limb
            ? static_cast<Limb::State*>(m.second.effector)->model->origin.Inverse() * m.second.effector->target
            : static_cast<BaseEffector::State*>(m.second.effector)->model->origin.Inverse() * m.second.effector->target;
    }
    std::shared_ptr<LinearTrajectoryAction::EffectorTrajectory::Result> get_result(const rclcpp::Time&, ResultCode code);
};

} //ns:robotik::trajectory

#endif //ROBOT_DYNAMICS_LINEAR_TRAJECTORY_H
