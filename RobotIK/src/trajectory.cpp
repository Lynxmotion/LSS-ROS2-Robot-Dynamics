//
// Created by guru on 4/21/20.
//

#include <trajectory.h>
#include <exception.h>


//#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

#include <utility>


namespace robotik {

using namespace trajectory;


void Keyframe::invalidate() {
    // clear any rendered state
    state = trajectory::Pending;
    segment.Destroy();
}

void SegmentTimeline::invalidate(Timerange range) {
    Keyframe* f = keyframes_head;
    while(f) {
        if(range.contains(f->ts))
            f->invalidate();
        f = f->next;
    }
}

Timerange SegmentTimeline::time_range() const {
    if(keyframes_head->state == trajectory::Rendered) {
        // get the last rendered keyframe
        // this may actually be the head position, and that is ok
        Keyframe* f = keyframes_tail->previous_rendered_frame();
        if(f)
            return { keyframes_head->ts, f->ts + f->segment.Duration()};
    }
    return { 0, 0 };
}

void SegmentTimeline::checkForCancellation()
{
    Keyframe* kf = head();
    while(kf) {
        if(kf->goal_handle && kf->goal_handle->is_canceling()) {
            // signal complete and remove this keyframe
            double duration = kf->segment.Duration();
            auto result = std::make_shared<trajectory::EffectorTrajectory::Result>();
            result->result.transform = kf->feedback->progress.transform;
            result->result.segment = kf->expression.segment;
            result->result.duration = (float)duration;
            result->result.code = humanoid_model_msgs::msg::TrajectoryComplete::SUCCESS;
            result->result.value = 0.0;
            kf->goal_handle->canceled(result);

            // stop reporting via action
            kf->goal_handle.reset();
            kf->feedback.reset();

            kf = erase(kf);
            continue;
        }
        kf = kf->next;
    }
}

void Trajectory::setInitialState(const State& _initial_state) {
    if(initial_state)
        *initial_state = _initial_state;
    else
        initial_state = std::make_shared<State>(_initial_state);

    invalidate();
}

Timerange Trajectory::time_range() const {
    // get the range of all combined segment timelines
    range_t<double> r(0.0, 0.0);
    range_t<double> none;
    bool first = true;
    for(const auto& f: timeline) {
        auto sr = f.second.time_range();
        if(sr != none) {
            if(first) {
                r = sr;
                first = false;
            } else
                r = r.merge(sr);
        }
    }
    return r;
}

void Trajectory::invalidate(Timerange range) {
    // todo: figure out what to invalidate instead of wiping all renderings
    for(auto& f: timeline)
        f.second.invalidate(range);
}

class TrajectoryRenderEnv : public RenderingInterface
{
public:
    inline TrajectoryRenderEnv()
    : traj(nullptr), model(nullptr), frame(nullptr), state(nullptr)
    {}

    Trajectory* traj;
    Model* model;
    Keyframe* frame;
    State* state;

    KDL::Frame get_relative_frame(const FrameRef& ref, double at_timestamp) override
    {
        assert(model != nullptr);
        assert(traj != nullptr);
        std::cout << "Request for relative frame " << ref.typeName() << ":" << ref.name << std::endl;
#if 0       // todo: revive the get_relative_frame check for segment ref from trajectory
        Limb* limb = nullptr;
        if(ref.type == FrameRef::Segment) {
            auto timeline = traj->timeline.find(ref.name);
            if(timeline == traj->timeline.end()) {
                // check to see if the segment is part of a limb
                for(auto& l: model->limbs) {
                    if (std::find(l->segment_names.begin(), l->segment_names.end(), ref.name) != l->segment_names.end()) {
                        // found in this limb, check to see if this limb is controlled in this trajectory
                        timeline = traj->timeline.find(l->options_.to_link);
                        if(timeline == traj->timeline.end())
                            timeline = traj->timeline.find(l->options_.from_link);
                        // for any inner/dependent segment we must compute IK
                        if(timeline != traj->timeline.end()) {
                            limb = l.get();
                        }
                    }
                }
            }

            if(timeline != traj->timeline.end()) {
                // pull the state of the segment from the timeline
                KDL::Frame f;
                // todo: WE HAVE TO GET STATE FOR FROM AND TO LINK!!
                if(timeline->second.get_state(at_timestamp, f)) {
                    std::cout << "  fetched segment state for " << ref.name << " from timline" << std::endl;
                    State st(*state);
                    if(limb && limb->updateIK(st, f)>0) {
                        //model->compute_TF_CoM(st, *limb);
                        model->compute_TF_CoM(st);
                    }
                    return model->getRelativeFrame(ref, st);
                }
            }
        }
#endif
        // reference is not dependant on trajectory/timeline state
        return model->getRelativeFrame(ref, *state);
    }

    KDL::Frame convert_frame(FrameRef from_ref, KDL::Frame frame_in, FrameRef to_ref) override
    {
        if(from_ref.type != FrameRef::Odometry) {
            auto odom_from = model->getRelativeFrame(from_ref, *state);
            frame_in = odom_from * frame_in;
        }
        if(to_ref.type != FrameRef::Odometry) {
            auto odom_to = model->getRelativeFrame(to_ref, *state);
            frame_in = odom_to.Inverse() * frame_in;
        }
        return frame_in;
    }
};

Keyframe* SegmentTimeline::insert(double ts) {
    auto* kf = new Keyframe(ts);
    if(keyframes_head) {
        // find insertion point
        Keyframe* insert_after = keyframes_tail;
        while(insert_after && insert_after->ts > ts)
            insert_after = insert_after->prev;

        if(insert_after) {
            // insertion within the list
            if(insert_after->next)
                insert_after->next->prev = kf;
            kf->next = insert_after->next;
            insert_after->next = kf;
            kf->prev = insert_after;
            if(insert_after == keyframes_tail)
                keyframes_tail = kf;        // kf becomes the new tail
            assert(keyframes_tail->next == nullptr);
        } else {
            // becomes the first item
            kf->next = keyframes_head;
            keyframes_head->prev = kf;
            keyframes_head = kf;
        }
    } else {
        // first item
        keyframes_head = keyframes_tail = kf;
    }
    return kf;
}

Keyframe* SegmentTimeline::erase(Keyframe* kf)
{
    if(kf == keyframes_head) {
        // erasing the head
        keyframes_head = kf->next;
        if(keyframes_head) {
            keyframes_head->prev = nullptr;
        }
        return keyframes_head;  // return new head
    } else {
        // removing inner keyframe
        assert(kf->prev);   // must be since we check for head above
        Keyframe* next_kf = kf->next;
        kf->prev->next = kf->next;
        if(kf->next)
            kf->next->prev = kf->prev;
        delete kf;
        return next_kf;
    }
}

void SegmentTimeline::set_initial_state(KDL::Frame initial, bool invalidate_all) {
    initial_state_ = initial;
    if(invalidate_all)
        invalidate();
}

bool SegmentTimeline::add_expression(
        trajectory::Expression& expr,
        std::shared_ptr<trajectory::GoalHandle> goal_handle)
{
    // add the expression details
    Keyframe* kf = insert(expr.start);
    if (kf) {
        kf->ts = expr.start;
        kf->expression = expr;
        if(goal_handle) {
            kf->goal_handle = std::move(goal_handle);
            kf->feedback = std::make_shared<trajectory::EffectorTrajectory::Feedback>();
        }
        if(expr.mix_mode == trajectory::Replace) {
            // delete all remaining key frames
            Keyframe *k = kf->next, *next = nullptr;
            while(k) {
                next = k->next;
                delete k;
                k = next;
            }

            // this frame is the new tail
            keyframes_tail = kf;
            kf->next = nullptr;
        }
        else if(expr.mix_mode == trajectory::Merge)
        {
            // invalidate all remaining key frames
            Keyframe* k = kf->next;
            while(k) {
                k->invalidate();
                k = k->next;
            }
        }
        else assert(false);   // mix-mode not implemented
        return true;
    }
    return false;
}

void Trajectory::clearSegment(std::string name)
{
    timeline.erase(name);
}

bool Trajectory::add_expression(trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle)
{
    auto timeline_itr = timeline.find(expr.segment);
    if(timeline_itr == timeline.end()) {
        // new segment timeline
        auto new_tl = timeline.emplace(std::pair<std::string, SegmentTimeline>(expr.segment, expr.segment));
        if(new_tl.second) {
            timeline_itr = new_tl.first;

            // get the initial state
            KDL::Frame initial_tf;
            if(initial_state->findTF(timeline_itr->first, initial_tf)) {
                timeline_itr->second.set_initial_state(initial_tf, false);
            }
        } else
            return false;
    }

    return timeline_itr->second.add_expression(expr, std::move(goal_handle));
}

double SegmentTimeline::render(Keyframe* frame, trajectory::RenderingInterface& env) {
    // get the initial state for this trajectory
    KDL::Frame kf_initial_pose;
    FrameRef kf_initial_relative_to(FrameRef::Odometry);
    std::string prev_frame_id("^");
    if(frame->prev) {
        // we pull the state from the last keyframe
        // if the timestamp is beyond the range of the last keyframe it will return the final state anyway,
        // if the timestamp is not at the end of the last keyframe then we'll still get the intermediate state
        // that is the appropriate starting point for this keyframe's state (basically, it truncates the last one
        frame->prev->get_state(frame->ts, kf_initial_pose);
        prev_frame_id = frame->prev->expression.id;
        kf_initial_relative_to = frame->prev->expression.reference_frame;
    } else {
        kf_initial_pose = initial_state_; // directly from the initial state of time zero
    }

    // convert the initial pose into the requested relative frame
    if(frame->expression.reference_frame != kf_initial_relative_to) {
#if 0
        std::cout << segment_name_ <<  " timeline: "
            << prev_frame_id << " => " << frame->expression.id
            << " converting reference frame from "
            << kf_initial_relative_to << " to "
            << frame->expression.reference_frame
            << " : " << kf_initial_pose.p;
#endif
        kf_initial_pose = env.convert_frame(
                kf_initial_relative_to,
                kf_initial_pose,
                frame->expression.reference_frame);
        //std::cout << " => " << kf_initial_pose.p << std::endl;
    }

    // render the frame
    auto duration = frame->segment.render(frame->expression, kf_initial_pose, env);
#if 0
    std::cout << "rendering frame " << segment_name_
            << " @" << ((long)frame->ts % 86400)
            << " : " << duration << "s"
            << std::endl;
#endif
    if(duration > 0.0) {
        frame->state = trajectory::Rendered;
    } else {
        // if rendering fails, we should probably remove or disable this frame
        frame->state = trajectory::InvalidTrajectory;   // disabling for now
    }
    return duration;
}

void SegmentTimeline::offset(double ofs) {
    Keyframe* kf = keyframes_head;
    while(kf) {
        kf->ts += ofs;
        kf->expression.start += ofs;
        //if(kf->state == trajectory::Rendered) {
        //    kf->segment.base_time += ofs;
        //}
        kf = kf->next;
    }
}

int Trajectory::updateState(State& state, double t) {
    int state_updates = 0;
    std::map<std::string, KDL::Frame> segment_updates;
    for(auto& tl_segment: timeline) {
        Keyframe* ref_frame = tl_segment.second.find_keyframe(t);
        if(ref_frame) {
            KDL::Frame segment_state;

            // todo: we should order segments by frame like we do in render
            if(ref_frame->state != trajectory::Rendered) {
                // render the frame
                TrajectoryRenderEnv env;
                env.traj = this;
                env.model = model.get();
                env.state = &state;

                tl_segment.second.render(ref_frame, env);
            }

            if(ref_frame->get_state(t, segment_state)) {

                //if(tl_segment.second.get_state(t, segment_state, ref_frame)) {
                // resolve the reference frame
                KDL::Frame ref = model->getRelativeFrame(ref_frame->expression.reference_frame, state);

                // apply the reference frame to the trajectory
                segment_state = ref * segment_state;

                state.tf[tl_segment.first] = segment_state;
                segment_updates[tl_segment.first] = segment_state;

                if(ref_frame->goal_handle) {
                    double duration = ref_frame->segment.Duration();
                    if(t > ref_frame->ts + duration) {
                        // signal complete
                        auto result = std::make_shared<trajectory::EffectorTrajectory::Result>();
                        result->result.transform = tf2::kdlToTransform(segment_state).transform;
                        result->result.segment = ref_frame->expression.segment;
                        result->result.duration = (float)duration;
                        result->result.code = humanoid_model_msgs::msg::TrajectoryComplete::SUCCESS;
                        result->result.value = 0.0;
                        ref_frame->goal_handle->succeed(result);

                        // stop reporting via action
                        ref_frame->goal_handle.reset();
                        ref_frame->feedback.reset();
                    } else {
                        // report progress
                        auto& fb = ref_frame->feedback->progress;
                        fb.duration = (float)duration;
                        fb.segment = ref_frame->expression.segment;
                        fb.transform = tf2::kdlToTransform(segment_state).transform;
                        fb.progress = constrain(0.0f, (float)(t - ref_frame->ts) / fb.duration, 1.0f);
                        fb.id = ref_frame->expression.id;
                        fb.acceleration = 0.0;
                        fb.velocity = 0.0;
                        ref_frame->goal_handle->publish_feedback(ref_frame->feedback);
                    }
                }

                if(tl_segment.first == model->base_link) {
                    kinematics.moveBase(state, segment_state);
                }
            }
        }
    }

    if(state.type == MeasuredState)
        state.type = TrajectoryState;

    // mark limbs that are being updated
    // todo: improve performance of segment/effector classification, it should be indexed
    for(size_t l = 0, _l = model->limbs.size(); l < _l; l++) {
        auto limb = model->limbs[l];
        if((l >= state.limbs.size()))
            continue;
        auto& limb_request = state.limbs[l];

        // check if this limb was in our trajectory
        auto to_seg = segment_updates.find(limb->options_.to_link);
        auto from_seg = segment_updates.find(limb->options_.from_link);
        if(to_seg != segment_updates.end()) {
            // set limb to seeking and update target
            if(limb_request.mode <= Limb::Seeking) {
                limb_request.mode = Limb::Seeking;
                limb_request.targetTF = to_seg->second;
                //std::cout << "seeking " << limb->options_.to_link << ": " << limb_request.targetTF << std::endl;

                // update kinematics
                kinematics.moveEffector(state, limb->options_.to_link, limb_request.targetTF);
            }
        }
        else if(from_seg != segment_updates.end()) {
            // only activate the limb, but it will track it's currently set target
            if(limb_request.mode <= Limb::Seeking) {
                limb_request.mode = Limb::Seeking;
                limb_request.targetTF = state.tf[limb->options_.to_link];
            }

            // update kinematics
            kinematics.moveEffector(state, limb->options_.to_link, limb_request.targetTF);

        } else {
            if(limb_request.mode == Limb::Seeking)
                limb_request.mode = Limb::Holding;
        }
    }

    kinematics.updateState(state);

    // updates state IK, dynamics, etc
    //model->updateIK(state);
    //model->compute_TF_CoM(state);

    // todo: should we update dynamics on trajectories as well? we are no longer updating in robot_dynamics.cpp (maybe make it a trajectory feature)
    //model->updateDynamics(state);
    //model->updateContacts(state);

    return state_updates;
}

double Trajectory::render(double up_to_time)
{
    double max_time = 0;

    TrajectoryRenderEnv env;
    env.traj = this;
    env.model = model.get();
    env.state = initial_state.get();

    // get a list of all controlled segments
    std::vector<std::string> controlled_segments;
    for(auto const& tl: timeline)
        controlled_segments.push_back(tl.first);

    // render timelines
    while (true) {
        // get the next unrendered keyframe from each segment timeline
        std::vector< std::pair<SegmentTimeline*, Keyframe*> > frames;
        for(auto& tl_segment: timeline) {
            Keyframe* kf = tl_segment.second.first_pending_frame();
            if(kf && kf->ts < up_to_time)
                frames.emplace_back(&tl_segment.second, kf);
        }

        // exit if no frames to render
        if(frames.empty())
            break;

        // sort the list of unrendered keyframes by time
        std::sort(frames.begin(), frames.end(),
                  [](const std::pair<SegmentTimeline*, Keyframe*>& a, const std::pair<SegmentTimeline*, Keyframe*>& b) {
            return a.second->ts < b.second->ts;
        });

        // render each frame in order
        for(auto& kf: frames) {
            // update the state in the environment before each next keyframe renders
            env.frame = kf.second;
            /* double end_ts = */ kf.first->render(kf.second, env);

#if 0
            // todo: OPTIMIZE instead of calling updateIK on each keyframe, defer it, when a request for a
            //  frame state comes into RenderingEnv, then we get the state in the request. Will require
            //  knowing what controlling segment affects what other segments.
            //     * RenderingEnv must have an internal pointer to Trajectory so it can do the state query
            //     * SegmentTimeline must track what segments are affected by its movement (using KDL chain)

            // updates state IK, dynamics, etc for the next rendering loop
            model->updateIK(*newstate);
            model->compute_TF_CoM(*newstate);
            // todo: should we update dynamics on trajectories as well?

            env.state = newstate;
#endif
        }
    }

    this->state = Rendered;
    return max_time;
}

void Trajectory::checkForCancellation()
{
    for(auto& tl : timeline) {
        tl.second.checkForCancellation();
    }
}

void Trajectory::offset_timeline(double offset)
{
    for(auto& tl: timeline) {
        tl.second.offset(offset);
    }
}

} // ns::robot
