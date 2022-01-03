//
// Created by guru on 3/28/20.
//

#ifndef LSS_HUMANOID_EXCEPTION_H
#define LSS_HUMANOID_EXCEPTION_H

#include <exception>
#include <cstdarg>
#include <iomanip>

namespace robotik {

typedef enum {
    RE_FAILED = 1,
    RE_FAILED_COM = 2,
    RE_ZERO_MASS = 3,
    RE_ELEMENT_NOT_FOUND = 4,
    RE_SEGMENT_NOT_FOUND = 5,
    RE_JOINT_NOT_FOUND = 6,
    RE_LIMB_NOT_FOUND = 7,
    RE_EFFECTOR_NOT_FOUND = 8,
    RE_SIZE_MISMATCH = 9,
    RE_URDF_GEOMETRY_ERROR = 10,
    RE_INVALID_CHAIN = 11,
    RE_EMPTY_STATES = 12,
    RE_OUT_OF_BOUNDS = 13,
    RE_BAD_OPERATOR_STATE = 14,
    RE_BAD_TIMING = 15,
    RE_OUT_OF_ORDER = 16
} robotik_exception_code;


class Exception : public std::runtime_error
{
public:
    robotik_exception_code code;

    Exception(robotik_exception_code _code = RE_FAILED)
        : std::runtime_error(CodeToString(_code)), code(_code)
    {
    }

    Exception(robotik_exception_code _code, std::string _detail) : std::runtime_error(_detail), code(_code) {
    }

    Exception(robotik_exception_code _code, const char* fmt, ...) : Exception(_code) {
        code = _code;
        if(fmt) {
            va_list args1;
            va_start(args1, fmt);
            va_list args2;
            va_copy(args2, args1);
            std::vector<char> buf(1 + std::vsnprintf(nullptr, 0, fmt, args1));
            va_end(args1);
            std::vsnprintf(buf.data(), buf.size(), fmt, args2);
            va_end(args2);
            std::runtime_error::operator=(std::runtime_error(buf.data()));
        } else {
            std::runtime_error::operator=(std::runtime_error(CodeToString(_code)));
        }
    }

    static const char* CodeToString(robotik_exception_code _code) {
        switch(_code) {
            case RE_FAILED: return "robotik error";
            case RE_FAILED_COM: return "could not compute robot center of mass";
            case RE_ZERO_MASS: return "model had zero mass";
            case RE_SEGMENT_NOT_FOUND: return "segment not found";
            case RE_JOINT_NOT_FOUND: return "joint not found";
            case RE_LIMB_NOT_FOUND: return "limb not found";
            case RE_EFFECTOR_NOT_FOUND: return "effector not found";
            default:
                return "robotic unknown error";
        }
    }

    static Exception ElementNotFound(std::string segname) {
        return Exception(RE_ELEMENT_NOT_FOUND, segname + " element not found");
    }

    static Exception SegmentNotFound(std::string segname) {
        return Exception(RE_SEGMENT_NOT_FOUND, "segment '" + segname + "' not found");
    }

    static Exception JointNotFound(std::string segname) {
        return Exception(RE_JOINT_NOT_FOUND, "joint '" + segname + "' not found");
    }

    static Exception LimbNotFound(std::string name) {
        return Exception(RE_LIMB_NOT_FOUND, "limb '" + name + "' not found");
    }

    static Exception EffectorNotFound(std::string name) {
        return Exception(RE_EFFECTOR_NOT_FOUND, "effector '" + name + "' not found");
    }

    static Exception OutOfOrder(std::string reason) {
        return Exception(RE_OUT_OF_ORDER, reason);
    }

    static Exception OutOfBounds() {
        return Exception(RE_OUT_OF_BOUNDS, "out of bounds");
    }

    static Exception BadTiming(double time1=0, double time2=0) {
        std::stringstream msg;
        msg << "bad timing, as we all know time cannot flow backwards";
        if(time1 >0)
            msg << " @" << std::setprecision(4) << time1;
        if(time2 >0)
            msg << " < " << std::setprecision(4) << time2;
        return Exception(RE_OUT_OF_BOUNDS, msg.str());
    }
};

} // ns::robot

#endif //LSS_HUMANOID_EXCEPTION_H
