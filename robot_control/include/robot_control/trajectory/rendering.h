//
// Created by guru on 1/5/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_RENDERING_H
#define HUMANOID_DYNAMIC_MODEL_RENDERING_H

#include "expression.h"   // trajectory functions
#include "environment.h"  // trajectory rendering environment for accessing trajectory and projected robot state

#include <string>
#include <vector>

namespace robotik {
namespace trajectory {

/** @brief Renders trajectory expressions into KDL trajectories.
 * This class acts like a composite KDL Trajectory and performs rendering of our Trajectory Expressions into
 * actual segment trajectories.
 *
 * Notice: Most of the code in this class is borrowed from the KDL Trajectory_Composite class. Since that class
 * didnt provide an interface for truncating or modifying the existing composite path it required copying.
 * It could not be sub-classes since the required collections were private.
 * See: https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/trajectory_composite.hpp
 */
 // todo: or SegmentTrajectory - and add the Expression to it...but it doesnt render until requested
class RenderedSegment : public KDL::Trajectory {

    typedef std::vector<KDL::Trajectory*> VectorTraj;
    typedef std::vector<double>         VectorDouble;
    VectorTraj vt;      // contains the element Trajectories
    VectorDouble  vd;      // contains end time for each Trajectory
    double duration;    // total duration of the composed
    // Trajectory

    public:
    // Constructs an empty composite
    RenderedSegment();

    // no copy constructors
    RenderedSegment(const RenderedSegment& copy) = delete;
    RenderedSegment& operator=(const RenderedSegment& copy) = delete;

    ~RenderedSegment() override;

    double Duration() const override;
    KDL::Frame Pos(double time) const override;
    KDL::Twist Vel(double time) const override;
    KDL::Twist Acc(double time) const override;

    // Adds trajectory <elem> to the end of the sequence.
    virtual void Add(Trajectory* elem);

    // removes any trajectories that start after given time
    void truncate(double time);

    virtual void Destroy();     // todo: should this be clear?
    void Write(std::ostream& os) const override;
    KDL::Trajectory* Clone() const override;

    double render(Expression& expr, KDL::Frame initial_p, RenderingInterface& env);

    /*template<class TItr>
    double render(TItr begin, TItr end, RenderingInterface& env) {
        double total_time = 0;
        while(begin < end)
            total_time += render(*begin++, env);
        return total_time;
    }*/
};

} // ns:trajectory
} // ns:robotik
#endif //HUMANOID_DYNAMIC_MODEL_RENDERING_H
