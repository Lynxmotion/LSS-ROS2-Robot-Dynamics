# successful stop codes
int8 SUCCESS = 0
int8 PREEMPTED = 1
int8 CONTACT = 2
int8 TORQUE_LIMIT = 3

# error result codes
int8 FAILED = -1
int8 DISABLED = -2
int8 INVALID_ARG = -3
int8 FOLLOWING_ERROR = -4
int8 IK_FAILURE = -5

# result code
int8 code

# value associated with result code
float64 value

# The segment name being controlled
string segment

# final duration of the trajectory in seconds
float32 duration

# the current position/rotation state of the segment relative to
# the header.child_frame_id (usually odom frame)
geometry_msgs/Transform transform

