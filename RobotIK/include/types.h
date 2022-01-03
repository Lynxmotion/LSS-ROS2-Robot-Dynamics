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

std::string generate_id(int length);

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

class EulerRotation {
public:
    double roll, pitch, yaw;

    inline EulerRotation(double _r=0, double _p=0, double _y=0) : roll(_r), pitch(_p), yaw(_y) {}
};

} // ns::robotik

#endif //LSS_HUMANOID_TYPES_H
