//
// Created by guru on 12/29/21.
//

#ifndef ROBOT_DYNAMICS_ACTION_H
#define ROBOT_DYNAMICS_ACTION_H

#include "rendering.h"
#include "range.h"
#include "model.h"

#include <rclcpp_action/rclcpp_action.hpp>
#include <map>

#include <robot_model_msgs/msg/trajectory_complete.hpp>
#include <utility>

std::ostream & operator << (std::ostream &out, const rclcpp_action::GoalUUID& uuid);

namespace robotik::trajectory {

using TimeRange = range_t<double>;

class TrajectoryActionInterface
{
public:
    using SharedPtr = std::shared_ptr<TrajectoryActionInterface>;
    using CancelResponse = rclcpp_action::CancelResponse;
    using ResultCode = robot_model_msgs::msg::TrajectoryComplete::_code_type;

    rclcpp_action::GoalUUID uuid;

    inline void id(std::string _id_) { id_ = std::move(_id_); }
    [[nodiscard]] inline std::string id() const { return id_; }

    [[nodiscard]] virtual std::string type() const=0;

    /// Return the start and end time of this action
    [[nodiscard]] virtual TimeRange time_range() const=0;

    [[nodiscard]] virtual trajectory::RenderState render_state() const=0;

    [[nodiscard]] virtual bool expired(const rclcpp::Time& now) const;

    virtual bool render(RenderingInterface& env)=0;

    /// Apply the action request to the given state
    virtual void apply(const rclcpp::Time& now)=0;

    /// tell the client this action has completed
    virtual void complete(
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS)=0;

    /// tell the client a member of this action is being cancelled
    /// Other members are still being controlled and progress on those members will continue. If there are no other
    /// members remaining then the entire action will be cancelled using the `cancel(state, now)` function.
    /// It is ok, to call this function with any member_name and those not in this action group will be ignored with
    /// nothing sent to the client.
    /// returns true if all members have expired and this action has thus been entirely cancelled
    virtual bool complete(
            std::string member_name,
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS)=0;

    ///@brief Called to indicate the user requested this action be cancelled
    virtual CancelResponse cancel(
            const rclcpp::Time& now,
            ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS)=0;

    /// update the client on the progress of the action
    virtual void send_feedback(const rclcpp::Time& now)=0;

protected:
    std::string id_;
};

class TrajectoryActionMember
{
public:
    double ts;

    trajectory::Expression expression;
    trajectory::RenderedSegment segment;

    inline explicit TrajectoryActionMember(double ts = 0.0)
    : ts(ts) {}

    inline explicit TrajectoryActionMember(trajectory::Expression expr, double ts = 0.0)
    : ts(ts), expression(std::move(expr)) {}

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

class TrajectoryActions : public std::list<TrajectoryActionInterface::SharedPtr>
{
public:
    void append(TrajectoryActionInterface::SharedPtr action);

    bool complete(const rclcpp_action::GoalUUID uuid,
                  const rclcpp::Time& now,
                  TrajectoryActionInterface::ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);

    bool complete(std::string member_name,
                  const rclcpp::Time& now,
                  TrajectoryActionInterface::ResultCode code = robot_model_msgs::msg::TrajectoryComplete::SUCCESS);

    TrajectoryActionInterface::CancelResponse cancel(const rclcpp_action::GoalUUID uuid,
                  const rclcpp::Time& now,
                  TrajectoryActionInterface::ResultCode code = robot_model_msgs::msg::TrajectoryComplete::CANCELLED);

};

} //ns:robotik::trajectory

#endif //ROBOT_DYNAMICS_ACTION_H
