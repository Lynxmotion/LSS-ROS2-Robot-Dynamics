### Modes ##
# Insert into existing trajectory.
int8 INSERT = 0

# remove any existing trajectory, replace with this one.
int8 REPLACE_ALL = 1

# clear timelines of segments contained in this msg and replace with this one.
# any existing timelines of other segments not indicated in this msg will remain.
int8 REPLACE_SEGMENTS = 2

std_msgs/Header header

# trajectory timeline merge modes
int8 mode

# segment trajectories
SegmentTrajectory[] segments
