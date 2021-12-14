//
// Created by guru on 1/5/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_FUNCTION_H
#define HUMANOID_DYNAMIC_MODEL_FUNCTION_H

#include <kdl/trajectory.hpp>

#include <string>
#include <vector>
#include <memory>

#include "frame-ref.h"

namespace robotik {
namespace trajectory {

constexpr const auto VelocityProfileDefault = "velocity";
constexpr const auto VelocityTrapezoidalProfile = "velocity/trap";
constexpr const auto VelocityHalfTrapezoidalProfile = "velocity/traphalf";
constexpr const auto VelocityRectProfile = "velocity/rect";
constexpr const auto VelocitySplineProfile = "velocity/spline";
constexpr const auto VelocityDiracProfile = "velocity/dirac";

typedef enum {
    Trapezoid,
    HalfTrapezoid,
    Spline,
    Rect,
    Dirac,
    Default = Trapezoid
} VelocityProfile;

VelocityProfile velocityProfileStringToEnum(std::string velocity_profile);

typedef enum {
    Absolute = 0,
    Relative = 1
} CoordinateMode;

typedef enum {
    Pending,
    Rendered,

    // anything greater than Invalid is considered an invalid state
    InvalidTrajectory,
    Infinite
} RenderState;

typedef enum {
    Replace = 0,
    SmoothTransition = 1,
    Merge = 2
} MixMode;

class Expression
{
public:
    typedef std::shared_ptr<Expression> SharedPtr;

    /// @brief Render state of this expression.
    /// If a trajectory's initial state is updated then rendered segments may be
    /// invalidated which might clear this state.
    RenderState state;

    // when the trajectory should begin execution
    // anything less than now begins immediately
    double start;

    // the desired duration of the trajectory in seconds or 0 for best speed possible
    // could attach a unit to this and then the unit can be a pacing symbol
    double duration;

    // (optional) an id to associate with the trajectory
    std::string id;

    // the segment name to move
    std::string segment;    // todo: maybe segment name not needed in Expression object

    // todo: possibly we could indicate how we want to integrate with existing trajectory
    MixMode mix_mode;

    // target velocity and acceleration
    // values exceeding limits of segment or joint will be clamped.
    double velocity;
    double acceleration;

    // the desired motion profile
    VelocityProfile velocity_profile;

    /// @brief the path generator function
    /// these are like macros that can take an input lesser set of points and create
    /// a more detailed path. Some path functions take arguments which are to be
    /// encoded into the path string. Arguments can be numbers, strings or state
    /// variables. Suggested path functions are:
    ///   arc       - move in an arc through the given (2) points
    ///   spline    - move through a spline that touches through each point
    ///   circle    - move through a circle starting at current position to given point and back to current
    ///   line      - move in straight lines through each point ending at the last
    ///   rounded(radius)
    ///             - move through each point in a direct manner like "line" but
    ///               rounding corners with the given max radius
    /// Future ones may be:
    ///   step(height, length)
    ///   drag(length)
    std::string path_expression;
    // TSomething path; // parsed path expression

    /// @brief relative frame for coordinates
    /// Specifies how the points should be interpreted within the reference frame
    /// values:
    ///    (empty)        - relative to frame specified in msg's frame_id
    ///    world or odom  - relative to world or odom frame (as specified in the model configuration)
    ///    segment        - relative to this segment's position/frame
    ///    segment:<name> - relative to given segment's frame
    ///    joint          - relative to this segment's parent joint (ignores fixed joints)
    ///    joint:<name>   - relative to given joint's frame

    FrameRef reference_frame;

    /// @brief vector encoding mode
    /// subsequent points can be encoded as relative to the previous or absolute
    /// from within the relative_frame
    CoordinateMode coordinate_mode;

    /// @brief trajectory points
    /// can be direct trajectory transforms but also used as input to the path
    /// function so may be transformed before execution
    std::vector<KDL::Vector> points;
    std::vector<KDL::Rotation> rotations;

    inline Expression() : state(Pending) {}
    Expression(const Expression& copy) = default;
    Expression(Expression&& move) = default;
    Expression& operator=(const Expression&) = default;
};

using Expressions = std::vector<Expression>;
using SharedExpressions = std::vector<Expression::SharedPtr>;


} // ns:trajectory
} // ns:robotik
#endif //HUMANOID_DYNAMIC_MODEL_FUNCTION_H
