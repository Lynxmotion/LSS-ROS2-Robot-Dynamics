//
// Created by guru on 12/29/21.
//

#ifndef ROBOT_DYNAMICS_ACTION_H
#define ROBOT_DYNAMICS_ACTION_H

#include "rendering.h"
#include "range.h"
#include "model.h"

#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_model_msgs/msg/multi_trajectory_progress.hpp>
#include <robot_model_msgs/action/effector_trajectory.hpp>

#include <map>

namespace robotik::trajectory {

using EffectorTrajectory = robot_model_msgs::action::EffectorTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<EffectorTrajectory>;
using TimeRange = range_t<double>;

class TrajectoryActionInterface
{
public:
    using SharedPtr = std::shared_ptr<TrajectoryActionInterface>;

    /// Return the start and end time of this action
    [[nodiscard]] virtual TimeRange time_range() const=0;

    [[nodiscard]] virtual trajectory::RenderState render_state() const=0;

    virtual bool render(const Limbs& limbs, const Model& model, RenderingInterface& env)=0;

    /// Apply the action request to the given state
    virtual void apply(Limbs& limbs, const Model& model, double ts)=0;

    /// tell the client this action has completed
    virtual void complete(
            Limbs& limbs,
            const Model& model,
            const rclcpp::Time& now,
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS)=0;

    /// tell the client a member of this action is being cancelled
    /// Other members are still being controlled and progress on those members will continue. If there are no other
    /// members remaining then the entire action will be cancelled using the `cancel(state, now)` function.
    /// It is ok, to call this function with any member_name and those not in this action group will be ignored with
    /// nothing sent to the client.
    /// returns true if all members have expired and this action has thus been entirely cancelled
    virtual bool complete(
            std::string member_name,
            Limbs& limbs,
            const Model& model,
            const rclcpp::Time& now,
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS)=0;

    /// update the client on the progress of the action
    virtual void send_feedback(const Limbs& limbs, const Model& model, const rclcpp::Time& now)=0;

};

class TrajectoryActionMember
{
public:
    double ts;

    trajectory::Expression expression;
    trajectory::RenderedSegment segment;

    inline explicit TrajectoryActionMember(double ts = 0.0)
    : ts(ts) {}

    [[nodiscard]] inline double duration() const {
        return segment.Duration();
    }

    bool get_state(double t, KDL::Frame& f_out) const {
        if(t < ts)
            return false;
        t -= ts;  // get the relative time
        f_out = segment.Pos(t);
        return true;
    }

};

class TrajectoryAction : public TrajectoryActionInterface
{
public:
    inline TrajectoryAction(): state(Pending) {}

    TrajectoryAction(const trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle);

    /// Return the start and end time of this action
    [[nodiscard]] TimeRange time_range() const override;

    [[nodiscard]] trajectory::RenderState render_state() const override;

    bool render(const Limbs& limbs, const Model& model, RenderingInterface& env) override;

    /// Apply the action request to the given state
    void apply(Limbs& limbs, const Model& model, double ts) override;

    /// tell the client this action has completed
    void complete(
            Limbs& limbs,
            const Model& model,
            const rclcpp::Time& now,
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS) override;

    /// tell the client a member of this action is being cancelled
    /// Other members are still being controlled and progress on those members will continue. If there are no other
    /// members remaining then the entire action will be cancelled using the `cancel(state, now)` function.
    /// It is ok, to call this function with any member_name and those not in this action group will be ignored with
    /// nothing sent to the client.
    /// returns true if all members have expired and this action has thus been entirely cancelled
    bool complete(
            std::string member_name,
            Limbs& limbs,
            const Model& model,
            const rclcpp::Time& now,
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS) override;

    /// update the client on the progress of the action
    void send_feedback(const Limbs& limbs, const Model& model, const rclcpp::Time& now) override;

protected:
    TrajectoryActionMember member;
    trajectory::RenderState state;
    std::shared_ptr<trajectory::GoalHandle> goal_handle_;
    std::shared_ptr<trajectory::EffectorTrajectory::Feedback> feedback_;
};

class TrajectoryActions : public std::list<TrajectoryActionInterface::SharedPtr>
{
public:
    TrajectoryAction::SharedPtr append(TrajectoryActionInterface::SharedPtr action);

    void complete(std::string member_name,
                  Limbs& limbs,
                  const Model& model,
                  const rclcpp::Time& now,
                  int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);
};

}

#endif //ROBOT_DYNAMICS_ACTION_H
