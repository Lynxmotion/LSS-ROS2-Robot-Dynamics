//
// Created by guru on 1/3/22.
//

#include <robot_control/trajectory/linear-trajectory.h>
#include <conversions.h>

#include <tf2_kdl/tf2_kdl.h>

namespace robotik::trajectory {

LinearTrajectoryAction::LinearTrajectoryAction(
        BaseStates& bases, Limbs& limbs, const rclcpp::Time& now,
        std::shared_ptr<GoalHandle> goal_handle)
    : bases_(bases), limbs_(limbs),
      last_apply_(0.0),
      goal_handle_(std::move(goal_handle)),
      feedback_(std::make_shared<EffectorTrajectory::Feedback>())
{
    auto goal = goal_handle_->get_goal();
    id_ = goal->id;
    linear_acceleration_ = goal->linear_acceleration;
    angular_acceleration_ = goal->angular_acceleration;
    mode_in = effector_mode_from_msg(goal->mode_in);
    mode_out = effector_mode_from_msg(goal->mode_out);
    last_apply_ = ts_ = rclcpp::Time(goal->header.stamp).seconds();

    if(ts_ == 0.0)
        last_apply_ = ts_ = now.seconds();

    if(goal->effectors.size() == goal->velocity.size()) {
        int i = 0;
        for(auto e = goal->effectors.begin(), _e = goal->effectors.end(); e != _e; e++, i++) {
            Member m;
            KDL::Twist current_vel;
            geometry_msgs::msg::Twist msg_velocity = goal->velocity[i];

            // parse velocity command
            vector_to_kdl_vector(msg_velocity.linear, m.velocity.vel);
            vector_to_kdl_vector(msg_velocity.angular, m.velocity.rot);

            Limbs::iterator limb_itr;
            BaseStates::iterator base_itr;
            if(limbs_.end() != (limb_itr = limbs_.find(*e))) {
                m.is_limb = true;
                m.effector = &*limb_itr;
                current_vel = limb_itr->velocity;
            } else if(bases_.end() != (base_itr = bases_.find(*e))) {
                m.is_limb = false;
                m.effector = &*base_itr;
                //current_vel = base_itr->velocity;
            }

            // compute out the acceleration vector
            auto vel_vector = m.velocity.vel;
            auto rot_vector = m.velocity.rot;
            vel_vector.Normalize();
            rot_vector.Normalize();
            m.acceleration.vel = vel_vector * linear_acceleration_;
            m.acceleration.rot = rot_vector * angular_acceleration_;

            m.linear_velocity_tweener = Tween<KDL::Vector>::Velocity(
                    current_vel.vel,
                    m.velocity.vel,
                    m.acceleration.vel);
            m.angular_velocity_tweener = Tween<KDL::Vector>::Velocity(
                    current_vel.rot,
                    m.velocity.rot,
                    m.acceleration.rot);
            std::cout << "linear " << *e << "  duration:" << m.angular_velocity_tweener.duration() << "    delta:" << m.angular_velocity_tweener.delta() << std::endl;
            members[*e] = m;
        }
    }
}

std::string LinearTrajectoryAction::type() const {
    return "linear";
}

TimeRange LinearTrajectoryAction::time_range() const
{
    return {ts_, std::numeric_limits<double>::infinity() };
}

trajectory::RenderState LinearTrajectoryAction::render_state() const
{
    return Rendered;
}

bool LinearTrajectoryAction::render(RenderingInterface&)
{
   return true;
}

bool LinearTrajectoryAction::expired(const rclcpp::Time& now) const
{
    // todo: test that the segment has zero velocity
    return members.empty() || TrajectoryActionInterface::expired(now);
}

void LinearTrajectoryAction::apply(const rclcpp::Time& now)
{
    if(expired(now))
        return;

    double now_ts = now.seconds();
    double l_ts = now_ts - ts_;
    double dt = (now_ts - last_apply_);

    if(goal_handle_->is_canceling()) {
        // todo: we are in a cancelled state, decelerate

        // todo: if velocity is zero, then expire and indicate we cancelled
        members.clear();
        ts_ = 0;
    } else {
        for(auto& m: members) {
            // determine what our velocity should be now (as we ramp up to target velocity)
            auto cur_target_vel = m.second.linear_velocity_tweener[l_ts];
            auto cur_target_rot = m.second.angular_velocity_tweener[l_ts];
            if(std::isnan(cur_target_vel.x()) || std::isnan(cur_target_vel.y()) || std::isnan(cur_target_vel.z())) {
                cur_target_vel = KDL::Vector();
            }
            if(std::isnan(cur_target_rot.x()) || std::isnan(cur_target_rot.y()) || std::isnan(cur_target_rot.z())) {
                cur_target_rot = KDL::Vector();
            }

            auto cur_target_twist = KDL::Twist(cur_target_vel, cur_target_rot);

            // integrate velocity into current position and orientation
            if(mode_in != Effector::Mode::Unassigned)
                m.second.effector->mode = mode_in;
            m.second.effector->status = Limb::Status::Seeking;
            m.second.effector->target = addDelta(m.second.effector->target, cur_target_twist, dt);
            //m.second.effector->apply(
            //        addDelta(m.second.effector->target, cur_target_twist, dt),
            //        m.expression.coordinate_mask);
            // if we are manipulating a base, then apply inverse twist to supporting limbs
            if(!m.second.is_limb) {
                // apply inverse twist to limbs
                for(auto& l : limbs_) {
                    if(l.model->base->link == m.first && l.is_supporting()) {
                        l.apply_base_twist(cur_target_twist, dt);
                        if(l.status == Effector::Status::Holding)
                            l.status = Effector::Status::Supporting;
                    }
                }
            }
        }
    }
    last_apply_ = now_ts;
}

std::shared_ptr<LinearTrajectoryAction::EffectorTrajectory::Result>
LinearTrajectoryAction::get_result(const rclcpp::Time&, ResultCode code)
{
    auto tr = time_range();

    // signal complete
    auto result = std::make_shared<EffectorTrajectory::Result>();
    for(auto& m: members) {
        if(mode_out != Effector::Mode::Unassigned)
            m.second.effector->mode = mode_out;
        m.second.effector->status = Limb::Status::Holding;
        result->result.effectors.emplace_back(m.first);
        result->result.transforms.emplace_back(
                tf2::kdlToTransform(get_target_transform(m)).transform);

        // revert any supporting limbs back to holding
        if(!m.second.is_limb) {
            // apply inverse twist to limbs
            for(auto& l : limbs_) {
                if(l.model->base->link == m.first && l.status == Effector::Status::Supporting) {
                    l.status = Effector::Status::Holding;
                }
            }
        }
    }

    result->result.duration = (float)tr.span();
    result->result.code = code;
    result->result.value = 0.0;
    return result;
}

void LinearTrajectoryAction::complete(const rclcpp::Time& now, ResultCode code)
{
    if(!goal_handle_)
        return;

    // preempted would mean this goal is already cancelled
    auto result = get_result(now, code);
    if(goal_handle_->is_executing()) {
        if(code >=0)
            goal_handle_->succeed(result);
        else
            goal_handle_->abort(result);
    }

    // stop reporting via action
    goal_handle_.reset();
    feedback_.reset();

    ts_ = 0;
    members.clear();
    //std::cout << "    completed coordinated " << id() << "    code: " << code << std::endl;
}

bool LinearTrajectoryAction::complete(
        std::string member_name,
        const CoordinateMask& mask,
        const rclcpp::Time& now,
        ResultCode code)
{
    // remove specific effectors
    auto itr = members.find(member_name);
    if(itr != members.end() && mask != CoordinateMask::None) {
        members.erase(itr);
        if(members.empty())
            complete(now, code);
        return true;
    }

    // member not found in this action
    return false;
}

TrajectoryActionInterface::CancelResponse LinearTrajectoryAction::cancel(
        const rclcpp::Time& now,
        ResultCode code)
{
    // todo: we indicate we are cancelling but we can't complete yet until we deaccelerate
    // for now, lets just cancel outright
    auto result = get_result(now, code);
    members.clear();
    goal_handle_.reset();
    feedback_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LinearTrajectoryAction::send_feedback(const rclcpp::Time& now)
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
        fb.effectors.emplace_back(m.first);
        fb.transforms.emplace_back(tf2::kdlToTransform(get_target_transform(m)).transform);
    }
    fb.duration = (float)tr.span();
    fb.progress = (float)(t - tr.begin);
    fb.id = id();
    goal_handle_->publish_feedback(feedback_);
}

} //ns:robotik::trajectory

