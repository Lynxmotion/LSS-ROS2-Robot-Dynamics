//
// Created by guru on 1/5/21.
//

#include "trajectory/expression.h"

namespace robotik {

const char* FrameRef::typeName(FrameRef::FrameType ft) {
    switch(ft) {
        case FrameRef::Segment: return "segment";
        case FrameRef::Joint: return "joint";
        case FrameRef::World: return "world";
        case FrameRef::Odometry: return "odom";
        case FrameRef::Robot: return "robot";
        case FrameRef::CoM: return "CoM";
        case FrameRef::gCoM: return "gCoM";
        case FrameRef::CoP: return "CoP";
        case FrameRef::Footprint: return "footprint";
        default:
            throw std::runtime_error("unknown FrameRef type");
    }
}

namespace trajectory {
    VelocityProfile velocityProfileStringToEnum(std::string velocity_profile) {
        if (velocity_profile == trajectory::VelocityProfileDefault) {
            return trajectory::VelocityProfile::Default;
        } else if(velocity_profile == trajectory::VelocityTrapezoidalProfile) {
            return trajectory::VelocityProfile::Trapezoid;
        } else if(velocity_profile == trajectory::VelocityHalfTrapezoidalProfile) {
            return trajectory::VelocityProfile::HalfTrapezoid;
        } else if(velocity_profile == trajectory::VelocityRectProfile) {
            return trajectory::VelocityProfile::Rect;
        } else if(velocity_profile == trajectory::VelocitySplineProfile) {
            return trajectory::VelocityProfile::Spline;
        } else if(velocity_profile == trajectory::VelocityDiracProfile) {
            return trajectory::VelocityProfile::Dirac;
        } else {
            return trajectory::VelocityProfile::Default;
        }
    }
}

} // ns:robotik
