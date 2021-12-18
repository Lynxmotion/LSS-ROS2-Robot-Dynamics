# center of mass projected onto the floor indicating how the robot
# is supported over the legs in contact
geometry_msgs/Vector3 center_of_pressure

# the convex hull (polygon) enclosing support points.
# this will preclude any redundant or inside points of contacts that do
# not add to the stability in robots with more than 3 legs in cotact.
geometry_msgs/Vector3[] support_polygon

# support margin
float32 margin

# estimated number of seconds until the robot
# will enter instability region
float32 seconds_of_stability

# list of contact points
Contact[] contacts
