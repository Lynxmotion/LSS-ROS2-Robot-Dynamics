//
// Created by guru on 1/3/22.
//

#ifndef ROBOT_DYNAMICS_COORDINATED_EFFECTORS_H
#define ROBOT_DYNAMICS_COORDINATED_EFFECTORS_H

#include <robot_control/trajectory/action.h>

#include <robot_model_msgs/action/coordinated_effector_trajectory.hpp>

namespace robotik::trajectory {

class CoordinatedTrajectoryAction : public TrajectoryActionInterface
{
public:
    using EffectorTrajectory = robot_model_msgs::action::CoordinatedEffectorTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<EffectorTrajectory>;

    CoordinatedTrajectoryAction(
            Limbs& limbs, Model::SharedPtr model,
            const std::vector<trajectory::Expression>& expressions,
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
    Model::SharedPtr model_;
    Limbs& limbs_;
    trajectory::RenderState state;
    bool sync_duration_;
    std::list<TrajectoryActionMember> members;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_ptr<EffectorTrajectory::Feedback> feedback_;

    // compute the final result before completing or cancelling the action
    std::shared_ptr<CoordinatedTrajectoryAction::EffectorTrajectory::Result> get_result(const rclcpp::Time&, ResultCode code);
};

} //ns:robotik::trajectory

#endif //ROBOT_DYNAMICS_COORDINATED_EFFECTORS_H
