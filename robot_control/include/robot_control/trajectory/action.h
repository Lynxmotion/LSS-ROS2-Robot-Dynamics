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

class TrajectoryAction
{
public:
    using SharedPtr = std::shared_ptr<TrajectoryAction>;

    inline TrajectoryAction(): state(Pending) {}

    TrajectoryAction(const trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle);

    /// Return the start and end time of this action
    [[nodiscard]] TimeRange time_range() const;

    [[nodiscard]] inline trajectory::RenderState render_state() const { return state; }

    bool render(const Limbs& limbs, const Model& model, RenderingInterface& env);

    /// Apply the action request to the given state
    void apply(Limbs& limbs, const Model& model, double ts);

    /// tell the client this action has completed
    void complete(
            Limbs& limbs,
            const Model& model,
            const rclcpp::Time& now,
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);

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
            int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);

    /// update the client on the progress of the action
    void send_feedback(const Limbs& limbs, const Model& model, const rclcpp::Time& now);

protected:
    TrajectoryActionMember member;
    trajectory::RenderState state;
    std::shared_ptr<trajectory::GoalHandle> goal_handle_;
    std::shared_ptr<trajectory::EffectorTrajectory::Feedback> feedback_;
};

class TrajectoryActions : public std::list<TrajectoryAction::SharedPtr>
{
public:
    TrajectoryAction::SharedPtr append(TrajectoryAction::SharedPtr action);

    void complete(std::string member_name,
                  Limbs& limbs,
                  const Model& model,
                  const rclcpp::Time& now,
                  int code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);
};

}

#endif //ROBOT_DYNAMICS_ACTION_H
