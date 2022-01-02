//
// Created by guru on 1/5/21.
//

#include "robot_control/trajectory/rendering.h"

#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>

#include <kdl/path_point.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_cyclic_closed.hpp>
#include <kdl/path_roundedcomposite.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_dirac.hpp>

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <state.h>

using namespace std;

namespace robotik::trajectory {

RenderedSegment::RenderedSegment(): duration(0.0)
{
}

RenderedSegment::~RenderedSegment() {
    Destroy();
}

double RenderedSegment::Duration() const{
    return duration;
}

KDL::Frame RenderedSegment::Pos(double time) const {
    // not optimal, could be done in log(#elem)
    // or one could buffer the last segment and start looking from there.
    unsigned int i;
    double previoustime;
    assert(vt.size() > 0);  // must have at least one trajectory
    Trajectory* traj;
    if (time < 0) {
        return vt[0]->Pos(0);
    }
    previoustime = 0;
    for (i=0;i<vt.size();i++) {
        if (time < vd[i]) {
            return vt[i]->Pos(time-previoustime);
        }
        previoustime = vd[i];
    }
    traj = vt[vt.size()-1];
    return traj->Pos(traj->Duration());
}


KDL::Twist RenderedSegment::Vel(double time) const {
    // not optimal, could be done in log(#elem)
    unsigned int i;
    Trajectory* traj;
    double previoustime;
    if (time < 0) {
        return vt[0]->Vel(0);
    }
    previoustime = 0;
    for (i=0;i<vt.size();i++) {
        if (time < vd[i]) {
            return vt[i]->Vel(time-previoustime);
        }
        previoustime = vd[i];
    }
    traj = vt[vt.size()-1];
    return traj->Vel(traj->Duration());
}

KDL::Twist RenderedSegment::Acc(double time) const {
    // not optimal, could be done in log(#elem)
    unsigned int i;
    Trajectory* traj;
    double previoustime;
    if (time < 0) {
        return vt[0]->Acc(0);
    }
    previoustime = 0;
    for (i=0;i<vt.size();i++) {
        if (time < vd[i]) {
            return vt[i]->Acc(time-previoustime);
        }
        previoustime = vd[i];
    }
    traj = vt[vt.size()-1];
    return traj->Acc(traj->Duration());
}

void RenderedSegment::Add(Trajectory* elem) {
    vt.insert(vt.end(),elem);
    duration += elem->Duration();
    vd.insert(vd.end(),duration);
}

void RenderedSegment::Destroy() {
    VectorTraj::iterator it;
    for (it=vt.begin();it!=vt.end();it++) {
        delete *it;
    }
    vt.clear();
    vd.clear();
    duration = 0.0;
}

void RenderedSegment::truncate(double time)
{
    assert(vt.size() == vd.size());
    auto it_t = vt.begin();
    auto it_d = vd.begin();
    while (it_d != vd.end() && *it_d < time) {
        it_t++;
        it_d++;
    }
    // todo: partial truncate of trajectory, possibly we need to store a reference to the expression to get a partial path
    vt.erase(it_t,vt.end());
    vd.erase(it_d, vd.end());
}

template<class RItr>
KDL::Path* toRoundedPath(RItr begin, RItr end, double rradius) {
    auto path = new KDL::Path_RoundedComposite(rradius, 0.00001, new KDL::RotationalInterpolation_SingleAxis());
    while(begin != end) {
        try {
            path->Add(*begin);
        } catch(const KDL::Error_MotionPlanning_Not_Feasible& ex) {
            std::cout << "KDL motion not feasible: reason " << ex.GetType() << ": " << ex.Description() << std::endl;
        }
        begin++;
    }
    path->Finish();
    return path;
}

double RenderedSegment::render(Expression& expr, KDL::Frame initial_p, RenderingInterface&) {
    double added_duration = 0;

    Destroy();

    // parse path expression
    // this expression determines how to interpret the points array
    size_t l;
    std::string verb = expr.path_expression;
    std::vector<double> args;
    if((l = verb.find_first_of('(')) != std::string::npos) {
        if(verb[verb.length()-1] == ')') {
            std::string params = verb.substr(l+1);
            verb = verb.substr(0, l);

            std::string::size_type bop = 0, eop;
            while((eop = params.find_first_of(",)", bop)) != std::string::npos) {
                auto p = params.substr(bop,eop - bop);
                args.emplace_back(strtod(p.c_str(), nullptr));
                bop = eop + 1;
            }
        }
    }

    // create motion profile
    // todo: if we use the same arg parser as path then vel/acc could be encoded here
    KDL::VelocityProfile *velprof = nullptr;
    switch(expr.velocity_profile) {
        case trajectory::VelocityProfile::Trapezoid: velprof = new KDL::VelocityProfile_Trap(expr.velocity, expr.acceleration); break;
        case trajectory::VelocityProfile::HalfTrapezoid: velprof = new KDL::VelocityProfile_TrapHalf(expr.velocity, expr.acceleration); break;
        case trajectory::VelocityProfile::Rect: velprof = new KDL::VelocityProfile_Rectangular(expr.velocity); break;
        case trajectory::VelocityProfile::Spline: velprof = new KDL::VelocityProfile_Spline(); break;
        case trajectory::VelocityProfile::Dirac: velprof = new KDL::VelocityProfile_Dirac(); break;
    }

    // get the previous segment state
    // todo: see L185 about loading segments from previous traj
    if(duration > 0) {
        // continue segment where we left off in the last trajectory
        initial_p = Pos(duration);
    }

    // starting point will be initial_p in the relative frame
    KDL::Frame f_previous = initial_p;

    // create a lambda to process incoming points/rotations
    auto absolute_processor = [](KDL::Vector p, KDL::Rotation m) {
        // point is relative to reference frame, convert to odom
        KDL::Frame f(m, p);
        return f;
    };

    // todo: create a lambda to process relative mode points
    assert(expr.coordinate_mode != Relative);

    std::function<KDL::Frame(KDL::Vector p, KDL::Rotation m)> processor = absolute_processor;

    // create path
    // todo: use a path expression parser here
    KDL::Trajectory* traj = nullptr;
    KDL::Path* path = nullptr;
    double distance = 0;
    if(verb == "rounded") {
        double radius = args.size() ? args[0] : 0.003;
        std::vector<KDL::Frame> frames;
        frames.insert(frames.end(), initial_p);
        //std::cout << "initial: " << initial_p << std::endl;

        auto a = expr.points.begin(), _a = expr.points.end();
        auto b = expr.rotations.begin(), _b = expr.rotations.end();
        while(a != _a) {
            KDL::Vector v(*a++);

            KDL::Rotation m;
            if (b < _b)
                m = *b++;

            // the processor interprets relative or absolute
            KDL::Frame f_relative_current = processor(v, m);

            // compute delta position
            KDL::Vector dPv = f_relative_current.p - f_previous.p;
            // todo: I bet if we just add the absolute value of vector components we'd get the
            //        same answer with only one sqrt()
            double dP = std::sqrt( dPv.x()*dPv.x() + dPv.y()*dPv.y() + dPv.z()*dPv.z());
            distance += dP;

            // convert point from expression's relative frame into the state's common frame
            // todo: dont convert into odom frame yet, we keep rendered trajectory relative to desired frame so
            //       it will move with the relative joint through time
            // old_code => KDL::Frame f_odom_current = f_odom_reference * f_relative_current;
            KDL::Frame f_odom_current = f_relative_current;

            frames.insert(frames.end(), f_odom_current);
            //std::cout << "   + " << f_odom_current << std::endl;

            // prepare for next iteration
            f_previous = f_relative_current;
            if (expr.coordinate_mode == Relative && b == _b) {
                // wipe out rotation, so it doesnt keep rotation if no more rotations given
                m = KDL::Rotation::Identity();
            }
        }
#if 0
        printf("\nodom-ref %4.3f,%4.3f,%4.3f\n", f_odom_reference.p.x(), f_odom_reference.p.y(), f_odom_reference.p.z());
        for(size_t _i=frames.size(), i=0; i < _i; i++) {
            printf("%ld: %4.3f,%4.3f,%4.3f\n",
                   i,
                   frames[i].p.x(), frames[i].p.y(), frames[i].p.z());
        }
#endif

        path = toRoundedPath(frames.begin(), frames.end(), radius);
    } else if(verb == "stationary") {
        // duration required
        traj = new KDL::Trajectory_Stationary(expr.duration, initial_p);
    }

    // set profile duration
    if(path) {
        if (expr.duration > 0)
            velprof->SetProfileDuration(0, distance, expr.duration);
        else
            velprof->SetProfile(0, distance);
    }

    if(!traj && path && velprof) {
        traj = new KDL::Trajectory_Segment(path, velprof);
    }

    if(std::numeric_limits<double>::infinity() == traj->Duration()) {
        // todo: handle trajectories that have infinite duration
        printf("warning: trajectory %s for segment %s has infinite duration\n", expr.id.c_str(), expr.segment.c_str());
        expr.state = Infinite;
    } else {
        expr.state = Rendered;

        // success, so add transition and segment trajectory to composite trajectory
        Add(traj);
        added_duration += traj->Duration();
    }

    //printf("trajectory %s: %4.3f\n", expr.segment.c_str(), added_duration);

#if 0  // DEBUG_TRAJECTORY
    auto dur = Duration();
    auto fps = 10;
    printf("Replay   D%4.3f\n", dur);
    for(double t=0; t < dur; t += 1.0 / fps) {
        double r, p, y;
        auto tf = Pos(t);
        //auto tf = f_odom_reference.Inverse() * odom_tf;
        tf.M.GetRPY(r, p, y);
        printf("%5.3fT  %5.3fX %5.3fY %5.3fZ   %4.3fR %4.3fP %4.3fY\n",
               t,
               tf.p.x(), tf.p.y(), tf.p.z(),
               r, p, y);
    }
#endif
    return added_duration;
}


void RenderedSegment::Write(ostream& os) const {
    os << "COMPOSITE[ " << vt.size() << endl;
    unsigned int i;
    for (i=0;i<vt.size();i++) {
        vt[i]->Write(os);
    }
    os << "]" << endl;
}

KDL::Trajectory* RenderedSegment::Clone() const{
    RenderedSegment* comp = new RenderedSegment();
    for (unsigned int i = 0; i < vt.size(); ++i) {
        comp->Add(vt[i]->Clone());
    }
    return comp;
}

} // ns:robotik
