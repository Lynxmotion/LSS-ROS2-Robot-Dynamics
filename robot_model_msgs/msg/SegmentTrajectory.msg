# trajectory timeline merge modes
int8 REPLACE = 0
int8 SMOOTH_TRANSITION = 1
int8 MERGE = 2

# point-to-point coordinate positioning modes
int8 ABSOLUTE = 0
int8 RELATIVE = 1

# when the trajectory should begin execution
# anything less than now begins immediately
builtin_interfaces/Time start

# the desired duration of the trajectory in seconds or 0 for best speed possible
# could attach a unit to this and then the unit can be a pacing symbol
float64 duration

# (optional) an id to associate with the trajectory
string id

# (optional) if specified, this trajectory will immediately follow the specified
# trajectory id. If a start time is also given, then the start time is relative
# to the end of the specified trajectory.
string following

# if we added a track name and confidence we could support multi-trajectories
# and the dynamics module would choose appropriate trajectory based on sensor
# feedback. Like one track assumes where the ground is, others assume a raised
# ground or pot-hole and the trajectory executor could decide how to render
# the multiple trajectory expressions as input comes in. It improves response
# time so the remote planner can react. We should also express how sensor
# input relates to a track as well...for example, "early-collision" is tied to
# a ground reaction force detection in a foot earlier than expected.
# (optional) trajectory track name
# string track
# (optional) confidence that this track is the likely or expected path to occur
# possibly the system can just learn what states should select what track. The
# dynamics system runs the state and track names through a RL node. Probably best
# to make it an external node.
# float64 confidence

# the segment name to move
string segment

# todo: possibly we could indicate how we want to integrate with existing trajectory
int8 mix_mode

# target velocity and acceleration
# values exceeding limits of segment or joint will be clamped.
float64 velocity
float64 acceleration

# the desired motion profile
# currently supported are:
#   velocity/trap
#   velocity/traphalf
#   velocity/spline
#   velocity/rect
#   velocity/dirac
string profile

# the path generator function
# these are like macros that can take an input lesser set of points and create
# a more detailed path. Some path functions take arguments which are to be
# encoded into the path string. Arguments can be numbers, strings or state
# variables. Suggested path functions are:
#   arc       - move in an arc through the given (2) points
#   spline    - move through a spline that touches through each point
#   circle    - move through a circle starting at current position to given point and back to current
#   line      - move in straight lines through each point ending at the last
#   rounded(radius)
#             - move through each point in a direct manner like "line" but
#               rounding corners with the given max radius
# Future ones may be:
#   step(height, length)
#   drag(length)
string path

# relative frame for coordinates
# Specifies how the points should be interpreted within the reference frame.
# The state is evaluated from any current trajectory as evaluated from the
# given start time above.
# values:
#    (empty)        - relative to frame specified in msg's frame_id
#    world or odom  - relative to world or odom frame (as specified in the model configuration)
#    robot          - relative to the robot base frame
#    CoM            - relative to the robot Center-of-Mass
#    gCoM           - relative to the robot Center-of-Mass as projected on the ground
#    CoP            - relative to the robot Center-of-pressure (always at ground level)
#    footprint      - relative to the robot footprint frame (always at ground level)
#    segment        - relative to this segment's position/frame
#    segment:<name> - relative to given segment's frame
#    joint          - relative to this segment's parent joint (ignores fixed joints)
#    joint:<name>   - relative to given joint's frame
string reference_frame

# vector encoding mode
# subsequent points can be encoded as relative to the previous or absolute
# from within the relative_frame
int8 coordinate_mode

# trajectory points
# can be direct trajectory transforms but also used as input to the path
# function so may be transformed before execution
# todo: perhaps we should have a tolerance or hi/lo on each point. Ex. point at expected ground but allow earlier collision.
geometry_msgs/Vector3[] points
geometry_msgs/Quaternion[] rotations
