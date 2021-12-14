//
// Created by guru on 3/30/20.
//

#include <robot_control/control.h>
#include "conversions.h"

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/velocityprofile_dirac.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/trajectory_segment.hpp>


namespace robotik {

Control::Control(std::string interaction_namespace)
: active(false), override(false), balance(false), loopPreviewDelay(4.0), loopPreview(true),
    interaction_namespace_(std::move(interaction_namespace)),
    efforts_updated(false)
{}

std::string Control::interaction_namespace() {
    return interaction_namespace_;
}

void Control::activate(Model::SharedPtr model, rclcpp_lifecycle::LifecycleNode& node) {
    model_ = model;

    trajectory_sub_ = node.create_subscription<humanoid_model_msgs::msg::MultiSegmentTrajectory>(
            "~/trajectory",
            10,
            std::bind(&Control::trajectory_callback, this, std::placeholders::_1)
    );

#ifndef TRAJECTORY_ACTION_SERVER
    // publisher for trajectory progress
    progress_pub_ = node.create_publisher<humanoid_model_msgs::msg::MultiTrajectoryProgress>(
            "~/trajectory/progress",
            10);

    if(!progress_msg_)
        progress_msg_ = std::make_shared<humanoid_model_msgs::msg::MultiTrajectoryProgress>();
    progress_pub_->on_activate();
#else
    // Trajectory Action Server
    this->trajectory_action_server_ = rclcpp_action::create_server<trajectory::EffectorTrajectory>(
            &node,
            "~/trajectory",
            std::bind(&Control::handle_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Control::handle_trajectory_cancel, this, std::placeholders::_1),
            std::bind(&Control::handle_trajectory_accepted, this, std::placeholders::_1));
    //if(trajectory_action_server_)
    //    trajectory_action_server_->on_activate();
#endif

    // publisher for joint trajectory
    auto joint_controller_name = node.get_parameter("joint_controller").get_value<std::string>();
    joint_trajectory_pub_ = node.create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/"+joint_controller_name+"/joint_trajectory",
            10);

    // create a joint publisher, we'll control the servo output
    if(!joint_trajectory_msg_)
        joint_trajectory_msg_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

    // effort control
    if(node.has_parameter("effort_controller")) {
        auto effort_controller_name = node.get_parameter("effort_controller").get_value<std::string>();
        joint_efforts_pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
                effort_controller_name,
                10);
        if(!joint_efforts_msg_) {
            joint_efforts_msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
            if(!joint_trajectory_msg_->joint_names.empty())
                joint_efforts_msg_->data.resize(joint_trajectory_msg_->joint_names.size());
        }
    } else {
        RCLCPP_WARN(get_logger(), "No effort_controller parameter specified so effort control will be disabled");
    }

    // ability to control state of the joint controller
    // tutorial: https://github.com/ros2/demos/blob/master/lifecycle/src/lifecycle_service_client.cpp
    jointctrl_change_state_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    jointctrl_get_state_client_ = node.create_client<lifecycle_msgs::srv::GetState>(
            joint_controller_name + "/get_state",
            rmw_qos_profile_services_default,
            nullptr);
    jointctrl_change_state_client_ = node.create_client<lifecycle_msgs::srv::ChangeState>(
            joint_controller_name + "/change_state",
            rmw_qos_profile_services_default,
            nullptr);

    visual = std::make_shared<robotik::StateVisual>("target_pose");
    //visual->show_only_updated = true;
    visual->configure(model_);

    trajectoryVisual = std::make_shared<robotik::StateVisual>("trajectory");
    trajectoryVisual->ghostColor = robotik::Color::White.withAlpha(0.3);
    //trajectoryVisual->show_only_updated = true;
    trajectoryVisual->configure(model_);
    //target.trajectoryVisual->visibility(robotik::TF, 0xffffffff);     // show only segments, no dynamics data

    if(joint_trajectory_pub_)
        joint_trajectory_pub_->on_activate();
    if(joint_efforts_pub_)
        joint_efforts_pub_->on_activate();
    if(visual)
        visual->activate();
    if(trajectoryVisual)
        trajectoryVisual->activate();

    active = true;
}

void Control::deactivate() {
    model_.reset();
    trajectory_sub_.reset();

    joint_trajectory_pub_->on_deactivate();
    joint_trajectory_msg_.reset();
    joint_trajectory_pub_.reset();

    visual->cleanup();
    trajectoryVisual->cleanup();

    jointctrl_get_state_client_.reset();
    jointctrl_change_state_client_.reset();
    jointctrl_get_state_request_.reset();
    jointctrl_change_state_request_.reset();
}

#if 0
void Control::setBaseHeight(double standingHeight) {
    model_->setBaseHeight(standingHeight);
}

void Control::setBaseHeight(double standingHeight, const State& current, rclcpp::Time _stamp) {
    // todo: set standing height should do more of the calcs to save effort
    lastControlUpdate = _stamp;
    active = true;
    override = false;

    trajectory.reset();
    target.reset();

    if(!target) {
        target = std::make_shared<State>(current);
    }

    // offset target state from robot
    KDL::Frame base;
    if(!target->findTF(model_->base_link, base))
        return;

    // get base in Euler Roll, Pitch, Yaw
    double r, p, y;
    base.M.GetRPY(r, p, y);

    // get the yaw rotation of the bot for rotating leg offsets to match heading
    KDL::Rotation heading = KDL::Rotation::RPY(0, 0, y);
    KDL::Vector posOffset(0.2, 0, 0);

    // for now, offset target in space
    target->transform = current.transform;
    target->transform.p += heading * posOffset;

/*        // copy joint states from current to target
    // todo: we need to be smarter about setting body orientation for target, most of this copy is not used once all limbs are configured
    if(target->joints.size() != current.joints.size()) {
        target->joints = current.joints;
        target->alloc(POSITION);
    }
    target->position = current.position;*/

    base.p.z(standingHeight);
    target->tf[model_->base_link] = base;

    // todo: we have to do this since main loop constantly calls control::setaBaseHeught()...we should do better here though
    model_->setBaseHeight(standingHeight);

    model_->updateNailedContacts(*target);
    model_->compute_TF_CoM(*target);
    model_->updateContacts(*target);
    model_->updateDynamics(*target);

    if(balance)
       model_->balance(*target);

    std::cout << "base to " << base.p.z() << std::endl;

    return;

    auto traj = std::make_shared<humanoid_model_msgs::msg::MultiSegmentTrajectory>();
    //traj.header.stamp = now();
    traj->header.frame_id = model_->odom_link;
    //traj->segments.
    humanoid_model_msgs::msg::SegmentTrajectory s1;
    s1.start.sec = 0;
    s1.start.nanosec = 0;
    s1.id = "stand";
    s1.segment = model_->base_link;
    s1.profile = trajectory::VelocityTrapezoidalProfile;
    s1.velocity = 0.06;
    s1.acceleration = 0.1;
    s1.path = "rounded";
    s1.reference_frame = traj->header.frame_id;
    s1.coordinate_mode = humanoid_model_msgs::msg::SegmentTrajectory::ABSOLUTE;

    s1.points.emplace_back( to_vector(base.p) );
    s1.rotations.emplace_back( to_quat(base.M) );
    traj->segments.push_back(s1);

    trajectory_callback(traj);
}
#endif

void Control::set_joints(const std::vector<std::string> &joint_names)
{
    // new joint names for publishing
    size_t N = joint_names.size();
    if(!joint_trajectory_msg_)
        joint_trajectory_msg_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    joint_trajectory_msg_->joint_names = joint_names;

    if(!joint_efforts_msg_)
        joint_efforts_msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
    joint_efforts_msg_->data.resize(N);

    joint_trajectory_index.resize(N);
}

void Control::set_joint_controller_active(bool active)
{
    if(active)  {
        jointctrl_change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        jointctrl_change_state_future_ = jointctrl_change_state_client_->async_send_request(jointctrl_change_state_request_);
    } else {
        jointctrl_change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        jointctrl_change_state_future_ = jointctrl_change_state_client_->async_send_request(jointctrl_change_state_request_);
    }
}

void Control::clear_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    // remove all the markers for the target visualization
    clear_target_markers(marker_pub);
    clear_trajectory_markers(marker_pub);
}

void Control::clear_target_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    marker_pub->publish(*visual->remove());
}

void Control::clear_trajectory_markers(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    marker_pub->publish(*trajectoryVisual->remove());
}

void Control::publish_visuals(rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub) {
    if(trajectoryVisual) {
        if(trajectory)
            marker_pub->publish(*trajectoryVisual->getMarkers());
        else
            clear_trajectory_markers(marker_pub);
    }
    if(visual) {
        if(target)
            marker_pub->publish(*visual->getMarkers());
        else
            clear_target_markers(marker_pub);
    }
}

void Control::enableManipulators(const State& current, rclcpp::Time _stamp)
{
    // todo: set standing height should do more of the calcs to save effort
    lastUpdate = _stamp;
    active = true;
    override = false;

    if(!target) {
        target = std::make_shared<State>(current);
    }
}

bool Control::interact(InteractionEvent& ev) {
    int updates = 0;
    rclcpp::Time stamp(ev.header.stamp.sec, ev.header.stamp.nanosec, RCL_ROS_TIME);

    // event is in odom frame, translate back to state transform local coords
    KDL::Frame evTF = target->transform.Inverse() * ev.tf;

    // Determine what limb this interaction is controlling
    // For end-effector events we generally match a single limb, however, when moving the base_link frame we generally
    // end up moving all limbs...but the base end of the limbs and the end-effector stays in place.
    size_t ln = 0;
    for(auto& limb: model_->limbs) {
        auto mode = model_->getLimbMode(ln, *target);

        // recompute IK if we match either the base or end-effector of the limb,
        // unless the limb is in Limp mode and we are manipulating the base then the limb should float freely
        if((limb->options_.to_link == ev.segment) || ((limb->options_.from_link == ev.segment) && (mode != Limb::Mode::Limp))) {
            KDL::Frame baseTF, limbTF;
            // retrieve the TF for the base and end-effector
            if(target->findTF(limb->options_.from_link, baseTF) && target->findTF(limb->options_.to_link, limbTF)) {
                // write the value of the InteractionEvent to the TF depending on if we are updating base or end-effector
                if(limb->options_.to_link == ev.segment)
                    limbTF = evTF;     // updating limb part
                else
                    baseTF = evTF;     // updating base part

                // limbTF needs to be relative to baseTF (it's currently relative to odom)
                KDL::Frame relLimbTF = baseTF.Inverse() * limbTF;

                // compute IK for this limb
                auto limb_target = limb->computePose(*target, relLimbTF);
                if(!limb_target.empty()) {
                    // success
                    target->tf[ev.segment] = evTF;  // store original event TF to target segment

                    // remove this limb as a contact point
                    model_->interactingSegment = ev.dragging ? ev.segment : "";

                    // updates joints in state
                    for(auto& joint: limb_target) {
                        auto jordinal = target->findJoint(joint.name);
                        if(jordinal >= 0) {
                            target->joints_updated[jordinal] = true;
                            target->position(jordinal) = joint.position;
                        }
                    }

                    updates++;
                } else {
                    // todo: update failed, return the *something
                }
            }
        }
        ln++;
    }

    if(updates > 0) {
        override = true;
        trajectory.reset();

        if(balance)
            model_->balance(*target);

        // todo: update these state updates after State dirty detection is implemented
        model_->compute_TF_CoM(*target);
        model_->updateContacts(*target);
        model_->updateDynamics(*target);
    }

    return updates > 0;
}

bool Control::renderTrajectory(const State&, rclcpp::Time) {
    trajectory->render();
    if(trajectory->state != trajectory::Rendered) {
        RCLCPP_ERROR(get_logger(), "trajectory could not be rendered");
        return false;
    } else {
        printf("new trajectory rendered\n");
        return true;
    }
}

void Control::print_debug(const char* label, const State& state) {
    size_t ln = 0;
    for(auto& limb: model_->limbs) {
        // get a limb request if it exists
        Limb::Request req;
        if(ln < state.limbs.size())
            req = state.limbs[ln];
        else {
            req.limbType = limb->options_.model;
            req.mode = (req.limbType == Limb::Leg) ? Limb::Holding : Limb::Limp;
        }

        auto mode = model_->getLimbMode(ln, state);
        if(mode != Limb::Limp) {
            KDL::Frame baseTF, limbTF;
            if (/*req.mode != Limb::Limp &&*/ state.findTF(limb->options_.from_link, baseTF) &&
                                              state.findTF(limb->options_.to_link, limbTF)) {
                if (mode == Limb::Seeking) {
                    limbTF = req.targetTF;
                    std::cout << label << ": " << limb->options_.to_link << " = " << limbTF << std::endl;
                }
            }
        }
        ln++;
    }
}

void Control::resetTarget(const State& current) {
    target = std::make_shared<robotik::State>(current);
}

void Control::resetTrajectory() {
    trajectory.reset();
    trajectoryState.reset();
}

///@brief Update any required trajectories based on current state
/// Apply trajectories to state
///@returns the expected state at this moment in time.
bool Control::update(const State& current, rclcpp::Time _now) {
    if(active) {
        bool preview_publish_static_tf = false;

        if(!target) {
            resetTarget(current);
            if(!target)
                // failed to establish target state from current
                return false;
        }

        // always move the base of the target state to match the current base state
        KDL::Frame odom_tf;
        if(current.findTF(model_->odom_link, odom_tf))
            target->tf[model_->odom_link] = odom_tf;


#if 0
        // todo: track changes in state's odom => base_link frame, and apply those changes to
        //       our target state as an offset. We can't just copy it like odom because we also
        //       want to manipulate the base frame.
        if(model_->interactingSegment != model_->base_link) {
            // as long as the user is not interacting with the base,
            // copy the base from sensed to target
            KDL::Frame base_tf;
            if(current.findTF(model_->base_link, base_tf))
                target->tf[model_->base_link] = base_tf;
        }
#endif
        bool trajectoryPreview = !executeTrajectory;

        // animate the current trajectory
        if(trajectory) {

            // determine what state we'll mix our trajectory into
            auto& source_state = target
                    ? *target
                    : current;

            // ensure trajectory is rendered
            // todo: when trajectory planning is working, improve detection of rendered state
            if (trajectory->state != robotik::trajectory::Rendered) {
                if(!renderTrajectory(source_state, _now)) {
                    goto abort_trajectory;
                }
            }

            // remove any keyframes that were cancelled by the client
            trajectory->checkForCancellation();

            // compute at what dt in the trajectory we are currently at
            //double _traj_duration = trajectory->Duration();
            double _traj_now = _now.seconds();
            //double dt = _traj_now - trajectoryAnimationStart;
            auto traj_range = trajectory->time_range();
            bool has_ended = _traj_now > traj_range.end + loopPreviewDelay;

            if(!executeTrajectory && has_ended && loopPreview) {
                // if we are not executing the trajectory then continually animate it
                // FYI if we are executing the trajectory then we just pause indefinitely at the end of the trajectory
                printf("trajectory ended, rendering with new state\n");
                trajectory->rebase_timeline(_traj_now + 0.3);
                trajectory->invalidate();
                trajectoryState.reset();

                // recompute the trajectory using the current robot state
                if(!renderTrajectory(source_state, _now)) {
                    goto abort_trajectory;
                }
            } else if(has_ended) {
                // trajectory ended, remove visuals
                resetTrajectory();
                goto abort_trajectory;
            }

            robotik::State::SharedPtr trajstate;
            if(trajectoryPreview) {
                // construct a new state based on the source state
                //trajstate = std::make_shared<robotik::State>(source_state);
                if(!trajectoryState) {
                    // create new trajectory preview
                    trajectoryState = std::make_shared<robotik::State>(source_state);
                    preview_publish_static_tf = true;
                } else {
                    // set existing trajectory preview state
                    *trajectoryState = source_state;
                }

                trajstate = trajectoryState;

                // we sometimes offset the target state so it is not exactly superimposed
                // over the robot, in this case we should use the same transform for the
                // trajectory state visuals.
                if(target)
                    trajstate->transform = target->transform;
            } else {
                // write our trajectory to target state
                trajstate = target;
                trajectoryState.reset();
            }

            // compute the robot state from the trajectory plan
            if (trajectory->updateState(*trajstate, _traj_now) >=0) {
                // run the trajectory through the balance algorithm
                // note: balance will call update on ground-forces and dynamics again
                lastUpdate = _now;

                // todo: this only needs to be done if visualizing traj because Control does it below
                //if(balance)
                //    model_->balance(*trajstate);
                model_->updateContacts(*trajstate);

#if 0
                // display what is the top 4 servos under torque
                if(trajstate->internalForces.rows()) {
                    std::vector<std::pair<std::string, double>> efforts;
                    for (int i = 0, _i = trajstate->internalForces.rows(); i < _i; i++) {
                        efforts.emplace_back(trajstate->joints[i], trajstate->internalForces(i));

                        // also record max
                        auto& jmax = top_efforts[trajstate->joints[i]];
                        jmax = std::max(jmax, std::abs(trajstate->internalForces(i)));
                    }

#if 0
                    // sort the joint efforts
                    std::sort(efforts.begin(), efforts.end(),
                              [](const std::pair<std::string, double>& left, const std::pair<std::string, double>& right) -> bool {
                        return std::abs(left.second) > std::abs(right.second);
                    });

                    // print the top N
                    for(int i = 0, _i = std::min(4, (int)efforts.size()); i < _i; i++)
                        std::cout << "   " << efforts[i].first << "=" << efforts[i].second;
                    std::cout << std::endl;
#endif
                }
#endif
                // if we are executing the trajectory, then the target state will be visualized later
                if(trajectoryPreview) {
                    // update the trajectory visual
                    trajectoryVisual->update(*trajstate);

                    // publish preview state to TF
                    model_->publishTransforms(*trajstate, _now, "preview");
                    if(preview_publish_static_tf)
                        model_->publishFixedTransforms(_now, "preview");

                    // publish model state for trajectory preview
                    model_->publishModelState(*trajstate, _now, "preview");
                }
            }
        }
abort_trajectory:

        // send visual state to RViz
        if (target) {
            // visualize target
            visual->update(*target);
        }

        // balance the robot
        if(balance)
            model_->balance(*target);

        // update state of target
        // todo: this should be done after target is updated
        model_->updateIK(*target);
        model_->compute_TF_CoM(*target);

        model_->updateContacts(*target);
        model_->updateDynamics(*target);

#if 0
        if(lastUpdate.seconds() < 0.05) {
            //std::cout << "target:   B.fp:" << fp_base_target.z() << "  S.fp:" << fp_sole_target.z() << "  B:" << base_target.z() << "  S:" << sole_target.z() << std::endl;
            std::cout << " n:" << current.tf.size() << "   tgt-base:" << target_base_location << " state-xform: " << current.transform.p << std::endl;
        }
#endif
        return true;
    }
    return false;
}

bool Control::publish() {
    // execute the trajectory
    if(executeTrajectory && target) {
        auto& trajstate = *target;
        if (trajstate.joints.size() != (size_t) trajstate.position.rows()) {
            RCLCPP_INFO(get_logger(),
                        "refuse to publish when joint names and positions arrays are not equal length");
            return false;
        }

        // todo: publishing joints order should match Q_NR joint order
        if (joint_trajectory_msg_->joint_names.empty()) {
            set_joints(trajstate.joints);
        } else {
            // there should be no case where these arent equal
            assert(joint_trajectory_index.size() == joint_trajectory_msg_->joint_names.size());
        }

        bool allow_publish = true;
        auto nj = joint_trajectory_msg_->joint_names.size();

        // allocate a single trajectory keyframe
        if (joint_trajectory_msg_->points.size() != 1) {
            joint_trajectory_msg_->points.resize(1);
        }

        // set the trajectory
        auto &points = joint_trajectory_msg_->points[0];
        if (points.positions.size() != nj)
            points.positions.resize(nj);

        for (size_t j = 0, _j = nj; j != _j; j++) {
            // lookup joint position using index
            auto j_idx = joint_trajectory_index[j];
            auto j_name = joint_trajectory_msg_->joint_names[j];

            // verify index is correct, if not make a correction
            if(trajstate.joints[j_idx] != j_name) {
                // find it the long way
                auto j_itr = std::find(trajstate.joints.begin(), trajstate.joints.end(), j_name);
                if (j_itr != trajstate.joints.end()) {
                    j_idx = joint_trajectory_index[j] = (j_itr - trajstate.joints.begin());
                } else {
                    // joint name was not found, this is a trajic error
                    RCLCPP_ERROR_STREAM_ONCE(get_logger(), "cannot publish joint "
                    << j_name << " because it is not found in state");
                    allow_publish = false;
                }
            }

            // add the joint position
            points.positions[j] = trajstate.position(j_idx);
            // todo: publish velocity and acceleration for trajectories
        }

        // publish new control values
        if(allow_publish)
            joint_trajectory_pub_->publish(*joint_trajectory_msg_);

        if(efforts_updated)
            publish_efforts();

        return true;
    }
    return false;
}

void Control::publish_efforts() {
    if(joint_efforts_pub_) {
        joint_efforts_pub_->publish(*joint_efforts_msg_);
        efforts_updated = false;
    }
}

void Control::publish_progress() {
#if 0
    if(progress_pub_ && trajectory) {
        std::vector<std::string> expired;
        for(auto t=trajectory->progress_reports.begin(), _t=trajectory->progress_reports.end();
                t != _t; t++) {
            progress_msg_->segments.insert(progress_msg_->segments.end(), t->second);
            if(t->second.progress >= 1.0)
                expired.insert(expired.end(), t->first);
        }

        // delete any segments that have expired/completed
        for(const auto& d: expired)
            trajectory->progress_reports.erase(d);

        // publish the progress
        progress_pub_->publish(*progress_msg_);

    }
#endif
}

void Control::enable_all_joints(double e) {
    size_t N = joint_trajectory_msg_->joint_names.size();
    if(joint_efforts_msg_->data.size() != N)
        joint_efforts_msg_->data.resize(N);
    for(size_t j = 0; j < N; j++) {
        joint_efforts_msg_->data[j] = e;
    }
    efforts_updated = true;
}

void Control::enable_arms(double e) {
    enable_limbs(e, [](const Limb& l) { return l.options_.model == Limb::Arm; });
}

void Control::enable_legs(double e) {
    enable_limbs(e, [](const Limb& l) { return l.options_.model == Limb::Leg; });
}

void Control::enable_limb(const char* limb, double e) {
    enable_limbs(e, [limb](const Limb& l) { return l.options_.to_link == limb; });
}

void Control::enable_limb(const Limb& limb, double e) {
    enable_limbs(e, [&limb](const Limb& l) { return &l == &limb; });
}

void Control::enable_limbs(double e, std::function<bool(const Limb&)> filter) {
    if(!target)
        return;
    size_t ln = 0;
    auto& joint_names = joint_trajectory_msg_->joint_names;
    for(auto& limb: model_->limbs) {
        // test if this limb should be updated
        if(!filter(*limb))
            continue;

        // get a limb request if it exists
        if(ln < target->limbs.size()) {
            auto& req = target->limbs[ln];
            if(e > 0) {
                // enable limb
                if(req.mode == Limb::Limp)
                    req.mode = Limb::Holding;
            } else {
                // disable limb (Limp)
                req.mode = Limb::Limp;
            }

            // set the effort on each joint in the limb
            for(auto ln: limb->joint_names) {
                // todo: use an index here
                auto idx = std::find(joint_names.begin(), joint_names.end(), ln);
                if(idx != joint_names.end()) {
                    auto n = idx - joint_names.begin();
                    joint_efforts_msg_->data[ n ] = e;
                    std::cout << "enabling joint " << ln << " [" << n << "]" << std::endl;
                }
            }
        }
    }
    efforts_updated = true;
}

trajectory::Expression Control::expression_from_msg(
        humanoid_model_msgs::msg::SegmentTrajectory seg,
        std::string default_reference_frame,
        const rclcpp::Time& now)
{
    trajectory::Expression tf;
    auto ts = now.seconds() + (double)seg.start.sec + (double)seg.start.nanosec / 1e9;
    tf.start = ts;
    tf.duration = seg.duration;
    tf.id = seg.id;
    tf.segment = seg.segment;
    tf.mix_mode = (trajectory::MixMode)seg.mix_mode;
    tf.path_expression = seg.path;
    vector_to_kdl_vector(seg.points, tf.points);
    quat_to_kdl_rotation(seg.rotations, tf.rotations);

    // calculate velocity/acceleration
    // todo: figure out what the max joint vel/acc is
    tf.velocity = std::min(seg.velocity, 1.0);
    tf.acceleration = std::min(seg.acceleration, 4.0);

    // create motion profile
    // todo: if we use the same arg parser as path then vel/acc could be encoded here
    tf.velocity_profile = trajectory::velocityProfileStringToEnum(seg.profile);
    tf.coordinate_mode = (trajectory::CoordinateMode)seg.coordinate_mode;

    if(seg.reference_frame.empty())
        tf.reference_frame = FrameRef(FrameRef::Segment, std::move(default_reference_frame));
    else
        tf.reference_frame = FrameRef::parse(seg.reference_frame);

    if(tf.reference_frame.name == model_->odom_link)
        tf.reference_frame = FrameRef(FrameRef::Odometry, model_->odom_link);
    else if(tf.reference_frame.name == model_->base_link)
        tf.reference_frame = FrameRef(FrameRef::Robot, model_->base_link);
    else if(tf.reference_frame.name == "world")
        tf.reference_frame = FrameRef(FrameRef::World);

    // if the name of our reference frame is blank, fill it with the default
    if (tf.reference_frame.type == FrameRef::Segment && tf.reference_frame.name.empty())
        tf.reference_frame.name = seg.segment;
    //else if (tf.reference_frame.type == FrameRef::Joint) {
    //    model_.
    //}
    return tf;
}

void Control::trajectory_callback(humanoid_model_msgs::msg::MultiSegmentTrajectory::SharedPtr msg)
{
    // set the absolute time
    rclcpp::Time now;
    if(msg->header.stamp.sec) {
        now = rclcpp::Time(msg->header.stamp);
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "trajectory publisher is not setting header timestamp");
        now = lastUpdate;
    }

    if(msg->mode == humanoid_model_msgs::msg::MultiSegmentTrajectory::REPLACE_ALL) {
        resetTrajectory();
    }

    // nothing to do if no segments are given
    if(msg->segments.empty())
        return;

    if(!trajectory) {
        trajectory = std::make_shared<robotik::Trajectory>(model_);
        trajectory->setInitialState(*target);
    }

    for(auto& seg: msg->segments) {
        RCLCPP_INFO(get_logger(), "Trajectory id:%s  segment:%s  profile:%s  path:%s", seg.id.c_str(), seg.segment.c_str(), seg.profile.c_str(), seg.path.c_str());

        auto expr = expression_from_msg(seg, msg->header.frame_id, now);

        if(msg->mode == humanoid_model_msgs::msg::MultiSegmentTrajectory::REPLACE_SEGMENTS) {
            trajectory->clearSegment(expr.segment);
        }

        trajectory->add_expression(expr);
    }

    //renderTrajectory(*trajectoryState, now);
}

#ifdef TRAJECTORY_ACTION_SERVER
rclcpp_action::GoalResponse Control::handle_trajectory_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const trajectory::EffectorTrajectory::Goal> goal)
{
    //RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    auto& request = goal->goal;

    if(!target) {
        RCLCPP_INFO(this->get_logger(), "Cannot accept trajectory goals without an existing state");
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    KDL::Frame f;
    if(!target->findTF(request.segment.segment, f)) {
        RCLCPP_INFO(this->get_logger(), "Segment %s in goal request doesn't exist in state", request.segment.segment.c_str());
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Control::handle_trajectory_cancel(
        const std::shared_ptr<trajectory::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for segment %s", request.segment.segment.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Control::handle_trajectory_accepted(const std::shared_ptr<trajectory::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;

    // set the absolute time
    rclcpp::Time now;
    if(request.header.stamp.sec) {
        now = rclcpp::Time(request.header.stamp);
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "trajectory publisher is not setting header timestamp");
        now = lastUpdate;
    }

    if(!trajectory) {
        trajectory = std::make_shared<robotik::Trajectory>(model_);
        trajectory->setInitialState(*target);
    }

    auto expr = expression_from_msg(request.segment, model_->odom_link, now);
    trajectory->add_expression(expr, goal_handle);
    //renderTrajectory(*trajectoryState, now);
}
#endif

} // ns::robot
