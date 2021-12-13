//
// Created by guru on 1/9/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_FRAME_REF_H
#define HUMANOID_DYNAMIC_MODEL_FRAME_REF_H

#include <string>

namespace robotik {

class FrameRef
{
public:
    typedef enum {
        Segment,
        Joint,
        World,
        Odometry,
        Robot,
        CoM,
        gCoM,       // CoM as projected onto the ground
        CoP,
        Footprint
    } FrameType;

    FrameType type;
    std::string name;

    FrameRef() = default;
    FrameRef(const FrameRef&) = default;
    FrameRef(FrameRef&&) = default;

    FrameRef& operator=(const FrameRef&) = default;

    inline FrameRef(FrameType _type) : type(_type) {}
    inline FrameRef(FrameType _type, std::string _name) : type(_type), name(_name) {}

    static const char* typeName(FrameType ft);
    inline const char* typeName() const { return typeName(type); }

    static FrameRef parse(std::string value);

    inline bool operator==(const FrameRef& rhs) const { return type==rhs.type && name==rhs.name; }
    inline bool operator!=(const FrameRef& rhs) const { return type!=rhs.type || name!=rhs.name; }
};

std::ostream& operator<<(std::ostream& sout, const FrameRef& ref);

} // ns:robotik

#endif //HUMANOID_DYNAMIC_MODEL_FRAME_REF_H
