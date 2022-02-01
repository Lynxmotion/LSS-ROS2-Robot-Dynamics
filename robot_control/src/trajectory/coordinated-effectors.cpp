//
// Created by guru on 1/3/22.
//

#include <robot_control/trajectory/coordinated-effectors.h>

#include <tf2_kdl/tf2_kdl.h>

namespace robotik::trajectory {

CoordinatedTrajectoryAction::CoordinatedTrajectoryAction(
        Limbs& limbs,
        const std::vector<trajectory::Expression>& expressions,
        std::shared_ptr<CoordinatedTrajectoryAction::GoalHandle> goal_handle)
    : limbs_(limbs), state(Pending)
{
    auto goal = goal_handle->get_goal();
    sync_duration_ = goal->goal.sync_duration;
    for(const auto& expr: expressions) {
        members.emplace_back(expr, expr.start);
    }
    if(goal_handle) {
        goal_handle_ = std::move(goal_handle);
        feedback_ = std::make_shared<EffectorTrajectory::Feedback>();
    }
}

std::string CoordinatedTrajectoryAction::type() const {
    return "coordinated";
}

TimeRange CoordinatedTrajectoryAction::time_range() const
{
    double start_ts = -1.0;
    double end_ts = 0.0;
    for(const auto& m: members) {
        // start time
        if(start_ts < 0.0 || m.ts < start_ts)
            start_ts = m.ts;

        // end time
        auto e = (state == Rendered)
            ? m.ts + m.duration()
            : m.ts;
        if(e > end_ts)
            end_ts = e;
    }
    return { start_ts, end_ts };
}

trajectory::RenderState CoordinatedTrajectoryAction::render_state() const
{
    return state;
}

bool CoordinatedTrajectoryAction::render(RenderingInterface& env)
{
    for(auto& m: members) {
        auto& limb = limbs_[m.expression.segment];
        m.segment.render(m.expression, limb.position, env);
    }
    state = Rendered;
    auto tr = time_range();
    //std::cout << "rendered coordinated " << id() << "    duration: " << tr.span() << std::endl;
    return tr.span();
}

bool CoordinatedTrajectoryAction::expired(const rclcpp::Time& now) const
{
    return members.empty() || TrajectoryActionInterface::expired(now);
}

void CoordinatedTrajectoryAction::apply(const rclcpp::Time& now)
{
    if(state != Rendered)
        return;
    double ts = now.seconds();

    for(auto& m: members) {
        auto& limb = limbs_[m.expression.segment];
        double l_ts = ts - m.ts;        // convert time to beginning of rendered segment trajectory
        if(l_ts > 0) {
            limb.apply(m.segment.Pos(l_ts), m.expression.coordinate_mask);
            if(m.expression.mode_in != Limb::Unassigned)
                limb.mode = m.expression.mode_in;
            limb.status = Limb::Status::Seeking;
            //std::cout << "    applied coordinated " << id() << ":" << limb.model->to_link + "   pos: " << l_ts << std::endl;
        }
    }
}

std::shared_ptr<CoordinatedTrajectoryAction::EffectorTrajectory::Result>
CoordinatedTrajectoryAction::get_result(const rclcpp::Time&, ResultCode code)
{
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
    auto tr = time_range();

    // signal complete
    auto result = std::make_shared<EffectorTrajectory::Result>();
    for(auto& m: members) {
        auto& limb = limbs_[m.expression.segment];
        if(m.expression.mode_out != Limb::Unassigned)
            limb.mode = m.expression.mode_out;
        limb.status = Limb::Status::Holding;
        // todo: add current position, velocity or error?
        //result->result.position = tf2::kdlToTransform(limb.target).transform;
        //result->result.velocity = tf2::kdlToTransform(limb.target).transform;
        result->result.effectors.emplace_back(m.expression.segment);
        result->result.transforms.emplace_back(
                tf2::kdlToTransform(limb.model->origin.Inverse() * limb.target).transform);
    }

    result->result.duration = (float)tr.span();
    result->result.code = code;
    result->result.value = 0.0;
    return result;
}

void CoordinatedTrajectoryAction::complete(const rclcpp::Time& time, ResultCode code)
{
    if(!goal_handle_)
        return;

    // preempted would mean this goal is already cancelled
    auto result = get_result(time, code);
    if(goal_handle_->is_executing()) {
        if(code >=0)
            goal_handle_->succeed(result);
        else
            goal_handle_->abort(result);
    }

    // stop reporting via action
    goal_handle_.reset();
    feedback_.reset();

    //std::cout << "    completed coordinated " << id() << "    code: " << code << std::endl;
}

bool CoordinatedTrajectoryAction::complete(
        std::string member_name,
        const CoordinateMask& mask,
        const rclcpp::Time& now,
        ResultCode code)
{
    if(members.size() == 1) {
        const auto& expr = members.front().expression;
        if(expr.segment == member_name && (mask & expr.coordinate_mask) != CoordinateMask::None) {
            // only 1 member and we are completing it
            complete(now, code);
            return true;
        }
    }

    for(auto m = members.begin(), _m = members.end(); m != _m; m++) {
        const auto& expr = m->expression;
        if(member_name == expr.segment && (mask & expr.coordinate_mask) != CoordinateMask::None) {
            // cancel only this member
            members.erase(m);
            return true;
        }
    }

    // member not found in this action
    return false;
}

TrajectoryActionInterface::CancelResponse CoordinatedTrajectoryAction::cancel(
        const rclcpp::Time& time,
        ResultCode code)
{
    assert(goal_handle_);
    auto result = get_result(time, code);
    goal_handle_->canceled(result);

    goal_handle_.reset();
    feedback_.reset();
    members.clear();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CoordinatedTrajectoryAction::send_feedback(const rclcpp::Time& now)
{
    if(!goal_handle_)
        return;

    auto tr = time_range();

    // todo: ensure action is rendered?
    double t = now.seconds();
    if(t < tr.begin || t >= tr.end)
        return; // outside of range

    // report progress
    auto& fb = feedback_->progress;
    for(auto& m: members) {
        auto& limb = limbs_[m.expression.segment];
        fb.effectors.emplace_back(m.expression.segment);
        fb.transforms.emplace_back(
                tf2::kdlToTransform(limb.model->origin.Inverse() * limb.target).transform);
    }
    fb.duration = (float)tr.span();
    fb.progress = (float)(t - tr.begin);
    fb.id = id();
    goal_handle_->publish_feedback(feedback_);
}

} //ns:robotik::trajectory

