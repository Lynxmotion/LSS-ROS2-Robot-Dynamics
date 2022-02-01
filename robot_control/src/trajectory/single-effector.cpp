//
// Created by guru on 1/3/22.
//

#include <robot_control/trajectory/single-effector.h>

#include <tf2_kdl/tf2_kdl.h>

namespace robotik::trajectory {

TrajectoryAction::TrajectoryAction(
        Limbs& limbs,
        const trajectory::Expression& expr,
        std::shared_ptr<GoalHandle> goal_handle)
        : limb_(limbs[expr.segment]), state(Pending)
{
    member.ts = expr.start;
    member.expression = expr;
    if(goal_handle) {
        goal_handle_ = std::move(goal_handle);
        feedback_ = std::make_shared<EffectorTrajectory::Feedback>();
    }
}

std::string TrajectoryAction::type() const {
    return "single";
}

TimeRange TrajectoryAction::time_range() const
{
    double duration = (state == Rendered)
            ? member.segment.Duration()
            : 0.0;
    return { member.ts, member.ts + duration };
}

trajectory::RenderState TrajectoryAction::render_state() const
{
    return state;
}

bool TrajectoryAction::expired(const rclcpp::Time& now) const
{
    return member.ts == 0 || TrajectoryActionInterface::expired(now);
}

bool TrajectoryAction::render(RenderingInterface& env)
{
    auto duration = member.segment.render(member.expression, limb_.position, env);
    std::cout << "rendered " << limb_.model->link << "    duration: " << duration << std::endl;
    state = Rendered;

    return duration > 0.0;
}

void TrajectoryAction::apply(const rclcpp::Time& now)
{
    auto ts = now.seconds();
    if(state != Rendered || ts < member.ts)
        return;

    ts -= member.ts;        // convert time to beginning of rendered segment trajectory

    limb_.apply(
            member.segment.Pos(ts),
            member.expression.coordinate_mask);

    if(member.expression.mode_in != Limb::Unassigned)
        limb_.mode = member.expression.mode_in;
    limb_.status = Limb::Status::Seeking;
    //std::cout << "    applied " << limb.model->to_link << "    pos: " << ts << std::endl;
}

void TrajectoryAction::complete(
        const rclcpp::Time&,
        ResultCode code)
{
    if(!goal_handle_)
        return;

#if 0
    KDL::Frame segment_state;
    if(state.findTF(member.expression.segment, segment_state)) {
        // transform to body coordinates
        KDL::Frame body_tf;
        if(state.findTF(model.base_link, body_tf)) {
            segment_state = body_tf.Inverse() * segment_state;
        } else
            throw robotik::Exception::SegmentNotFound(model.base_link);
    }
#endif

    if(member.expression.mode_out != Limb::Unassigned)
        limb_.mode = member.expression.mode_out;
    limb_.status = Limb::Status::Holding;

    // signal complete
    auto result = std::make_shared<EffectorTrajectory::Result>();
    result->result.transforms.emplace_back(
            tf2::kdlToTransform(limb_.model->origin.Inverse() * limb_.target).transform);
    // todo: add current position, velocity or error?
    //result->result.position = tf2::kdlToTransform(limb.target).transform;
    //result->result.velocity = tf2::kdlToTransform(limb.target).transform;
    result->result.effectors.emplace_back(member.expression.segment);
    result->result.duration = (float)member.segment.Duration();
    result->result.code = code;
    result->result.value = 0.0;
    if(goal_handle_->is_executing()) {
        if(code >=0)
            goal_handle_->succeed(result);
        else
            goal_handle_->abort(result);
    }
    // stop reporting via action
    goal_handle_.reset();
    feedback_.reset();

    std::cout << "    completed " << id() << " on " << limb_.model->link << "    " << std::endl;
}

bool TrajectoryAction::complete(
        std::string member_name,
        const CoordinateMask& mask,
        const rclcpp::Time& now, ResultCode code)
{
    // ignore if it isnt the member we are controlling
    if(member_name != member.expression.segment || (mask & member.expression.coordinate_mask) == CoordinateMask::None)
        return false;

    // we only have one member, so cancel the entire action
    complete(now, code);
    return true;
}

TrajectoryActionInterface::CancelResponse TrajectoryAction::cancel(
        const rclcpp::Time&,
        ResultCode)
{
    member.ts = 0;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryAction::send_feedback(const rclcpp::Time& now)
{
    if(!goal_handle_)
        return;

    // todo: ensure action is rendered?
    double t = now.seconds();
    double duration = member.segment.Duration();
    if(t < member.ts || t > member.ts + duration)
        return; // outside of range

    // report progress
    auto& fb = feedback_->progress;
    fb.duration = (float)duration;
    fb.effectors.emplace_back(member.expression.segment);
    fb.transforms.emplace_back(tf2::kdlToTransform(limb_.model->origin.Inverse() * limb_.target).transform);
    fb.progress = (float)(t - member.ts);
    fb.id = id_;
    goal_handle_->publish_feedback(feedback_);
}

} //ns:robotik::trajectory

