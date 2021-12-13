//
// Created by guru on 1/9/21.
//

#include "frame-ref.h"

#include <ostream>

namespace robotik {

FrameRef FrameRef::parse(std::string value)
{
    // todo: could make reference_frame parsing faster by using switch on the first character to limit equals() calls
    if(value == "segment") {
        return FrameRef(FrameRef::Segment);
    } else if(value == "joint") {
        return FrameRef(FrameRef::Joint);
    } else if(value.compare(0, 8, "segment:") == 0) {
        return FrameRef(FrameRef::Segment, value.substr(8));
    } else if(value.compare(0, 6, "joint:") == 0) {
        return FrameRef(FrameRef::Joint, value.substr(6));
    } else if(value == "world") {
        return FrameRef(FrameRef::World);
    } else if(value == "odometry" || value == "odom") {
        return FrameRef(FrameRef::Odometry);
    } else if(value == "robot" || value == "base") {
        return FrameRef(FrameRef::Robot);
    } else if(value == "CoM") {
        return FrameRef(FrameRef::CoM);
    } else if(value == "gCoM") {
        return FrameRef(FrameRef::gCoM);
    } else if(value == "CoP") {
        return FrameRef(FrameRef::CoP);
    } else if(value == "footprint") {
        return FrameRef(FrameRef::Footprint);
    } else {
        return FrameRef(FrameRef::Segment, value);
    }
}

std::ostream& operator<<(std::ostream& sout, const FrameRef& ref) {
    sout << ref.typeName() << ':' << ref.name;
    return sout;
}

} // ns:robotik
