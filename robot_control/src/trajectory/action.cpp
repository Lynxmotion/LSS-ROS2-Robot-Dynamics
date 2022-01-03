//
// Created by guru on 12/29/21.
//

#include "robot_control/trajectory/action.h"

#include <tf2_kdl/tf2_kdl.h>

namespace robotik::trajectory {

TrajectoryAction::TrajectoryAction(const trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle)
    : state(Pending)
{
    member.ts = expr.start;
    member.expression = expr;
    if(goal_handle) {
        goal_handle_ = std::move(goal_handle);
        feedback_ = std::make_shared<trajectory::EffectorTrajectory::Feedback>();
    }
}

TimeRange TrajectoryAction::time_range() const
{
    double duration = (state == Rendered)
            ? member.segment.Duration()
            : 0.0;
    return { member.ts, member.ts + duration };
}

bool TrajectoryAction::render(const Limbs& limbs, const Model&, RenderingInterface& env)
{
    auto& limb = limbs[member.expression.segment];
    auto duration = member.segment.render(member.expression, limb.position, env);
    std::cout << "rendered " << limb.model->options_.to_link << "    duration: " << duration << std::endl;
    state = Rendered;
    return duration > 0.0;
}

void TrajectoryAction::apply(Limbs& limbs, const Model&, double ts)
{
    if(state != Rendered)
        return;

    auto& limb = limbs[member.expression.segment];
    ts -= member.ts;        // convert time to beginning of rendered segment trajectory
    limb.target = member.segment.Pos(ts);
    limb.mode = Limb::Seeking;
    std::cout << "    applied " << limb.model->options_.to_link << "    pos: " << ts << std::endl;
}

void TrajectoryAction::complete(Limbs& limbs, const Model&, const rclcpp::Time&, int code)
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

    auto& limb = limbs[member.expression.segment];

    limb.mode = Limb::Holding;

    // signal complete
    auto result = std::make_shared<trajectory::EffectorTrajectory::Result>();
    result->result.transform = tf2::kdlToTransform(limb.target).transform;
    // todo: add current position, velocity or error?
    //result->result.position = tf2::kdlToTransform(limb.target).transform;
    //result->result.velocity = tf2::kdlToTransform(limb.target).transform;
    result->result.segment = member.expression.segment;
    result->result.duration = (float)member.segment.Duration();;
    result->result.code = code;
    result->result.value = 0.0;
    if(code >=0)
        goal_handle_->succeed(result);
    else
        goal_handle_->canceled(result);

    // stop reporting via action
    goal_handle_.reset();
    feedback_.reset();

    std::cout << "    completed " << limb.model->options_.to_link << "    code: " << code << std::endl;
}

bool TrajectoryAction::complete(std::string member_name, Limbs& limbs, const Model& model, const rclcpp::Time& now, int code)
{
    // ignore if it isnt the member we are controlling
    if(member_name != member.expression.segment)
        return false;

    // we only have one member, so cancel the entire action
    complete(limbs, model, now, code);
    return true;
}

void TrajectoryAction::send_feedback(const Limbs& limbs, const Model&, const rclcpp::Time& now)
{
    if(!goal_handle_)
        return;

    // todo: ensure action is rendered?
    double t = now.seconds();
    double duration = member.segment.Duration();
    if(t < member.ts || t > member.ts + duration)
        return; // outside of range

    auto& limb = limbs[member.expression.segment];

    // report progress
    auto& fb = feedback_->progress;
    fb.duration = (float)duration;
    fb.segment = member.expression.segment;
    fb.transform = tf2::kdlToTransform(limb.model->origin.Inverse() * limb.target).transform;
    fb.progress = constrain(0.0f, (float)(t - member.ts) / fb.duration, 1.0f);
    fb.id = member.expression.id;
    fb.acceleration = 0.0;
    fb.velocity = 0.0;
    goal_handle_->publish_feedback(feedback_);
}



TrajectoryAction::SharedPtr TrajectoryActions::append(TrajectoryAction::SharedPtr action)
{
    auto itr = insert(end(), action);
    if(itr == end())
        throw robotik::Exception(RE_FAILED, "cannot add action");
    return *itr;
}

void TrajectoryActions::complete(std::string member_name,
                                 Limbs& limbs,
                                 const Model& model,
                                 const rclcpp::Time& now,
                                 int code)
{
    for(auto a=begin(); a != end(); ) {
        if((*a)->complete(member_name, limbs, model, now, code)) {
            // remove action
            erase(a);
        } else
            a++;
    }
}

} // ns:robotik
