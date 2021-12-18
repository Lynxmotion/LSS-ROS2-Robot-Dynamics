# event types
int8 TORQUE_SOFT_LIMIT = 1
int8 TORQUE_HARD_LIMIT = 2
int8 CONTACT = 3
int8 FLOATING = 4

# the segment associated with the event
string segment

# one of the event types listed above
int8 event

# value that triggered the event
float64 data

