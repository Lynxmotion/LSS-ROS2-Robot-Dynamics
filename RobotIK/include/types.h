//
// Created by guru on 3/27/20.
//

#ifndef LSS_HUMANOID_TYPES_H
#define LSS_HUMANOID_TYPES_H

#include <string>
#include <vector>
#include <map>

#include <kdl/frames.hpp>


namespace robotik {

extern const std::string DEFAULT_COMPONENT_TOPIC_PREFIX;

using TwoCC = uint16_t;
using FourCC = uint32_t;

constexpr TwoCC operator"" _2cc(const char id[2], std::size_t  len) {
    return (len==1)
           ? id[0]
           : (id[1] << 8) + id[0];
}

const TwoCC AnyCC = 0;

constexpr unsigned long bit(unsigned short n) { return (((unsigned long)1)<<n); }

constexpr unsigned long operator "" _bit(unsigned long long n) {
    //static_assert(n < 64, "bit overflow, currently a limit of 64 bits");
    return (((unsigned long)1) << (n & 0xff));
}

using ParameterMask = unsigned long;
const ParameterMask POSITION                = 0_bit;        // unused, used to be limb position
const ParameterMask VELOCITY                = 1_bit;
const ParameterMask ACCELERATION            = 2_bit;
const ParameterMask EFFORT                  = 3_bit;
const ParameterMask INTERNAL_FORCE          = 4_bit;
const ParameterMask EXTERNAL_FORCE          = 5_bit;
const ParameterMask TF                      = 6_bit;
const ParameterMask CENTER_OF_MASS          = 7_bit;
const ParameterMask CENTER_OF_PRESSURE      = 8_bit;
const ParameterMask CONTACTS                = 9_bit;
const ParameterMask SUPPORT_POLYGON         = 10_bit;
const ParameterMask IMU                     = 11_bit;
const ParameterMask DEFAULT_PARAMETERS = POSITION | VELOCITY;
const ParameterMask DEFAULT_VISUALS = TF | POSITION | CENTER_OF_MASS | CENTER_OF_PRESSURE /*| INTERNAL_FORCE | EXTERNAL_FORCE*/ | CONTACTS | SUPPORT_POLYGON;


using Names = std::vector<std::string>;
using Ordinals = std::vector<size_t>;
using Frames = std::vector<KDL::Frame>;
using SegmentTransforms = std::map<std::string, KDL::Frame>;


class JointData {
public:
    double position;
    double velocity;

    inline JointData() : position(0), velocity(0) {}

    inline void set(double _position, double _velocity) {
        position = _position;
        velocity = _velocity;
    }
};

class NamedJointData : public JointData {
public:
    std::string name;
    KDL::Frame tf;
};

using NamedJointArray = std::vector<NamedJointData>;
using JointMap = std::map<std::string, JointData>;



class Origin {
public:
    using Type = enum {
        User,
        Interpolator,
        Control,
        Behavior
    };

    TwoCC id;
    Type type;

    Origin(Type _type, TwoCC _id = AnyCC) : id(_id), type(_type) {}

    static const Origin None;
};

class ElementKey {
public:
    Origin origin;               // is this worth it?
    TwoCC symbol;                // symbolic type ID
    std::string name;            // arbitrary name or segment name

    ElementKey()
            : origin(Origin::None), symbol(0)
    {}
    ElementKey(TwoCC _symbol)
            : origin(Origin::None), symbol(_symbol)
    {}
    ElementKey(TwoCC _symbol, std::string _name)
            : origin(Origin::None), symbol(_symbol), name(_name)
    {}
    ElementKey(TwoCC _symbol, const char* _name)
            : origin(Origin::None), symbol(_symbol), name(_name)
    {}

    std::size_t hash() const;

    std::string toString() const;

    inline bool operator==(const ElementKey& rhs) const { return symbol==rhs.symbol && name==rhs.name; }
    inline bool operator!=(const ElementKey& rhs) const { return symbol!=rhs.symbol || name!=rhs.name; }

    inline bool operator<(const ElementKey& rhs) const { return symbol<rhs.symbol || (symbol == rhs.symbol && name<rhs.name); }
};

class EulerRotation {
public:
    double roll, pitch, yaw;

    inline EulerRotation(double _r=0, double _p=0, double _y=0) : roll(_r), pitch(_p), yaw(_y) {}
};

} // ns::robotik


// custom specialization of std::hash for elements
// injected into std name space
namespace std {
    template<> struct hash<robotik::ElementKey> {
        inline std::size_t operator()(robotik::ElementKey const& s) const noexcept {
            return s.hash();
        }
    };
}

#endif //LSS_HUMANOID_TYPES_H
