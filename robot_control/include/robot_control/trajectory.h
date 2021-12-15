//
// Created by guru on 4/21/20.
//

#ifndef LSS_HUMANOID_TRAJECTORY_H
#define LSS_HUMANOID_TRAJECTORY_H


#include "types.h"
#include "state.h"
#include "model.h"
#include "range.h"
#include "trajectory/rendering.h"
#include "kinematics.h"

#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_model_msgs/msg/multi_trajectory_progress.hpp>
#include <robot_model_msgs/action/effector_trajectory.hpp>

#include <limits>
#include <map>
#include <utility>

namespace robotik {

using timestep_type = unsigned long;
using Timerange = range_t<double>;


namespace trajectory {
    using EffectorTrajectory = robot_model_msgs::action::EffectorTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<EffectorTrajectory>;
}

namespace linked_lists {
    template<class T>
    T* next_until(T* start, std::function<bool(const T*)> predicate) {
        return (start == nullptr)
            ? nullptr
            : predicate(start)
                ? start
                : next_until(start->next, predicate);
    }

    template<class T>
    T* previous_until(T* start, std::function<bool(const T*)> predicate) {
        return (start == nullptr)
            ? nullptr
            : predicate(start)
                ? start
                : previous_until(start->prev, predicate);
    }

    template<class T>
    T* next_while(T* start, std::function<bool(const T*)> predicate) {
        return (start == nullptr)
            ? nullptr
            : predicate(start)
                ? next_while(start->next, predicate)
                : start;
    }

    template<class T>
    T* previous_while(T* start, std::function<bool(const T*)> predicate) {
        return (start == nullptr)
            ? nullptr
            : predicate(start)
                ? previous_while(start->prev, predicate)
                : start;
    }
}

///@brief Holds trajectories, states and rendered assets associated with a particular time
class Keyframe
{
public:
    using SharedPtr = std::shared_ptr<Keyframe>;

    double ts;
    //double duration;      // now just determined by segment->Duration()

    //State::SharedPtr state; // todo: we should probably just store TF and not all state, and only relevant traj TF seg states
    trajectory::Expression expression;

    //KDL::Frame initial_state;
    trajectory::RenderState state;
    trajectory::RenderedSegment segment;    // todo: rename this to trajectory

    std::shared_ptr<trajectory::GoalHandle> goal_handle;
    std::shared_ptr<trajectory::EffectorTrajectory::Feedback> feedback;

    // keyframes form a linked list ordered by time
    Keyframe* next;
    Keyframe* prev;

    inline explicit Keyframe(double ts = 0.0)
    : ts(ts), state(trajectory::Pending), next(nullptr), prev(nullptr) {}

    bool get_state(double t, KDL::Frame& f_out) const {
        if(t < ts || state != trajectory::Rendered)
            return false;
        t -= ts;  // get the relative time
        f_out = segment.Pos(t);
        return true;
    }

    void invalidate();

    // get the first previously rendered frame from this point
    const Keyframe* previous_rendered_frame() const
    { return linked_lists::previous_until<const Keyframe>(this, [](const Keyframe* f) { return f->state == trajectory::Rendered; }); }
    Keyframe* previous_rendered_frame()
    { return linked_lists::previous_until<Keyframe>(this, [](const Keyframe* f) { return f->state == trajectory::Rendered; }); }
};

class SegmentTimeline
{
public:
    explicit SegmentTimeline(std::string segment_name)
    : segment_name_(std::move(segment_name)), keyframes_head(nullptr), keyframes_tail(nullptr)
    {}

    void invalidate(Timerange range = Timerange());

    Timerange time_range() const;

    void set_initial_state(KDL::Frame initial, bool invalidate_all = true);

    double render(Keyframe* frame, trajectory::RenderingInterface& env);

    bool add_expression(trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle = nullptr);

    // check all action key frames for cancellation signals
    // and if found, remove the keyframes
    void checkForCancellation();

    void offset(double ofs);

    inline Keyframe* head() const { return keyframes_head; }
    inline Keyframe* tail() const { return keyframes_tail; }

    inline Keyframe* find_keyframe(double timestamp) {
        // find the keyframe that contains this timestamp
        return linked_lists::previous_while<Keyframe>(keyframes_tail, [timestamp](const Keyframe* f) {
            return f->ts > timestamp;
        });
    }
    inline const Keyframe* find_keyframe(double timestamp) const {
        // find the keyframe that contains this timestamp
        return linked_lists::previous_while<Keyframe>(keyframes_tail, [timestamp](const Keyframe* f) {
            return f->ts > timestamp;
        });
    }

    inline Keyframe* first_pending_frame()
    { return linked_lists::next_until<Keyframe>(keyframes_head, [](const Keyframe* f) { return f->state == trajectory::Pending; }); }
    inline const Keyframe* first_pending_frame() const
    { return linked_lists::next_until<Keyframe>(keyframes_head, [](const Keyframe* f) { return f->state == trajectory::Pending; }); }

protected:
    std::string segment_name_;
    KDL::Frame initial_state_;
    Keyframe *keyframes_head, *keyframes_tail;

    // list of segments that would be affected by motion on this timeline
    // generally, this is the list of other segments in the limb's IK chain
    //std::vector<std::string> ik_dependent_segments;

    Keyframe* insert(double ts);

    // remove the given keyframe and return the one following it
    Keyframe* erase(Keyframe* kf);
};


class Trajectory {
public:
    using SharedPtr = std::shared_ptr<Trajectory>;

    std::shared_ptr<Model> model;
    Kinematics kinematics;

    // todo: keep track of segments updated in a traj down the timeline, and we can then limit
    //       state TF updates to only overwrite those segments
    State::SharedPtr initial_state;

    std::map<std::string, SegmentTimeline> timeline;    // todo: rename to timelines

    // todo: get rid of this rendered state flag
    trajectory::RenderState state;

    std::map<std::string, robot_model_msgs::msg::TrajectoryProgress> progress_reports;

    explicit Trajectory(Model::SharedPtr _model)
    : model(_model), kinematics(_model), state(trajectory::Pending) {
    }

    void setInitialState(const State& _initial_state);

    void invalidate(Timerange range = Timerange());

    // clear any timeline and all keyframes for the given segment name
    void clearSegment(std::string name);

    bool add_expression(trajectory::Expression& expr, std::shared_ptr<trajectory::GoalHandle> goal_handle = nullptr);

    double render(double up_to_time = std::numeric_limits<double>::infinity());

    range_t<double> time_range() const;

    int updateState(State& state, double dt);

    // check all action key frames for cancellation signals
    // and if found, remove the keyframes
    void checkForCancellation();

    // add this time offset to every keyframe
    // thus moving the entire timeline forward or back
    void offset_timeline(double offset);

    // move the timeline so that the trajectory starts at 'to_ts'
    inline void rebase_timeline(double to_ts) {
        auto tr = time_range();
        if(tr.begin >= 0)
            offset_timeline(to_ts - tr.begin);
    }
};


} // ns::robot
#endif //LSS_HUMANOID_TRAJECTORY_H
