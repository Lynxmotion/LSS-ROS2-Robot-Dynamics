//
// Created by guru on 1/3/22.
//

#include <robot_control/trajectory/coordinated-effectors.h>

#include <tf2_kdl/tf2_kdl.h>

namespace robotik::trajectory {

CoordinatedTrajectoryAction::CoordinatedTrajectoryAction(
        const std::vector<trajectory::Expression>& expressions,
        bool sync_duration,
        std::shared_ptr<CoordinatedTrajectoryAction::GoalHandle> goal_handle)
    : state(Pending), sync_duration_(sync_duration)
{
    for(const auto& expr: expressions) {
        members.emplace_back(expr, expr.start);
    }
    if(goal_handle) {
        goal_handle_ = std::move(goal_handle);
        feedback_ = std::make_shared<EffectorTrajectory::Feedback>();
    }
}

std::string CoordinatedTrajectoryAction::type() const {
    return "Coordinated";
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

bool CoordinatedTrajectoryAction::render(const Limbs& limbs, const Model&, RenderingInterface& env)
{
    for(auto& m: members) {
        auto& limb = limbs[m.expression.segment];
        m.segment.render(m.expression, limb.position, env);
    }
    auto tr = time_range();
    std::cout << "rendered coordinated " << id() << "    duration: " << tr.span() << std::endl;
    state = Rendered;
    return tr.span();
}

void CoordinatedTrajectoryAction::apply(Limbs& limbs, const Model&, double ts)
{
    if(state != Rendered)
        return;

    for(auto& m: members) {
        auto& limb = limbs[m.expression.segment];
        double l_ts = ts - m.ts;        // convert time to beginning of rendered segment trajectory
        if(l_ts > 0) {
            limb.target = m.segment.Pos(l_ts);
            limb.mode = Limb::Seeking;
            std::cout << "    applied coordinated " << id() << ":" << limb.model->options_.to_link + "   pos: " << ts << std::endl;
        }
    }
}

void CoordinatedTrajectoryAction::complete(Limbs& limbs, const Model&, const rclcpp::Time&, int code)
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
    auto tr = time_range();

    // signal complete
    auto result = std::make_shared<EffectorTrajectory::Result>();
    for(auto& m: members) {
        auto& limb = limbs[m.expression.segment];
        limb.mode = Limb::Holding;
        // todo: add current position, velocity or error?
        //result->result.position = tf2::kdlToTransform(limb.target).transform;
        //result->result.velocity = tf2::kdlToTransform(limb.target).transform;
        result->result.effectors.emplace_back(m.expression.segment);
        result->result.transforms.emplace_back(tf2::kdlToTransform(limb.target).transform);
    }

    result->result.duration = (float)tr.span();
    result->result.code = code;
    result->result.value = 0.0;
    // preempted would mean this goal is already cancelled
    if(goal_handle_->is_executing()) {
        if(code >=0)
            goal_handle_->succeed(result);
        else
            goal_handle_->abort(result);
    }

    // stop reporting via action
    goal_handle_.reset();
    feedback_.reset();

    std::cout << "    completed coordinated " << id() << "    code: " << code << std::endl;
}

bool CoordinatedTrajectoryAction::complete(std::string member_name, Limbs& limbs, const Model& model, const rclcpp::Time& now, int code)
{
    if(members.size() == 1 && members.front().expression.segment == member_name) {
        // only 1 member and we are completing it
        complete(limbs, model, now, code);
        return true;
    }

    for(auto m = members.begin(), _m = members.end(); m != _m; m++) {
        if(member_name == m->expression.segment) {
            // cancel only this member
            members.erase(m);
            return true;
        }
    }

    // member not found in this action
    return false;
}

void CoordinatedTrajectoryAction::send_feedback(const Limbs& limbs, const Model&, const rclcpp::Time& now)
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
        auto& limb = limbs[m.expression.segment];
        fb.effectors.emplace_back(m.expression.segment);
        fb.transforms.emplace_back(tf2::kdlToTransform(limb.model->origin.Inverse() * limb.target).transform);
    }
    fb.duration = (float)tr.span();
    fb.progress = (float)(t - tr.begin);
    fb.id = id();
    goal_handle_->publish_feedback(feedback_);
}

} //ns:robotik::trajectory

