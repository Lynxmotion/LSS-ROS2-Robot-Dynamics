# current limb position and velocity
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# target limb position
# this value is what is what is being used for limb IK computation
# and then joint data is being sent to the servos
geometry_msgs/Pose target

# distance between current and desired target position
float32 error
