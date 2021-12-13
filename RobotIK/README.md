#RobotIK for ROS2
###### Inverse Kinematics and Dynamics library for legged robots based on Orocos KDL library

#### RobotIK::State
This class has no modelling or intelligence and only holds state variables of the robot. 
  - can be a historical state or future (target) state
  - stores joint state (position, velocity, effort, ....)
  - stores IK resolved state of robot (TF)
    - Reference frame is usually _odom_ frame
    - TF is already transformed using IMU and other odometry
  - Dynamics including external forces and gravity compensation
  - also optionally stores parameters such as CoM, CoP, IMU, mass props
  - eventually this class will be dehydrated to the Kinesthetic memory.
  - states can be generated as tweens between two other states (via Tweener) for trajectory planning

#### RobotIK::Model
Models robot state, integrates control inputs and generates command output.
  - Given sensor input updates current state object
  - Given control input and current state computes a target state 

#### RobotIK::Visual
Provides a way to render robot state variables to RViz2.
  - Given a RobotState, creates RViz2 markers for visualization.
  - Can continually pass in RobotState objects and update the markers, so not tied to a specific state instance.
  - takes a frame_id and view transform when it updates.
  - multiple Visual instances can publish to the same topic (use different namespaces)

#### RobotIK::Control
  - is the user program to control robot behaviour
  - typically controls end-effectors
  - typically control will update end-effectors based on current/target state and
    create Trajectories.
  - model will fill in the IK, etc, after Control input is executed  

#### RobotIK::Trajectory
  - stores one or more segment trajectories
  - Given two or more states, generates mid-states by tweening between the key states
    for any delta time 'dt'.
  - uses Orocos trajectory classes for tweening segment frames.
  - Both position and velocity profiling are used in the tweening.


### Kinesthetic Memory  (RobotIK::KiMem)
Given a list of state variables to match, returns historical state objects that most 
closely match the variables in rank order. These results can be used to seed an Optimal
Control algorithm (such as Croccodyl) or predict what is about to happen. 

##### Inverted Index
Can probably use something like a text search's Reverse Word Index. Each criteria would 
have a seperate index (or maybe compound indexes can be too). The index is sparse, so the
value domain of the criteria is managed, maybe something like a btree or octomap kind of
index. Eventually the leaf nodes point to matching state record references. Each match
criteria is searched over their index and the results are combined and ranked. Ranking
combines a given cost multiplier for each variable and the difference between the query
value and the matched value (probably RMS value).

##### Confidence-based State Prediction/Estimation

Certain states are hard to determine, such as what feet are in contact with the ground.
We may be able to predict foot contact if we detect impulse changes when the foot hits
a hard surface (sudden deccelleration), or when the ankle joint angle is affected by the
ground (with the joint in limp state), or by seeing the IMU show or not show twist or
accelleration (falling). This is state estimation by inference and is not always reliable.
So how do we track these unreliable states without causing our state to diverge?

Since we have all our robot state contained in a single state object, we could store
mutliple copies of the current state with each assigned a confidence factor. For example,
we sense an impulse in the foot early so we copy state and set the foot to grounded.
Another function evaluates state objects and assigns a confidence value for how well
variables in that state align with other variables based on the model. For example, if
center of mass is within the support polygon, then IMU should not angular velocity. Each
current state instance will continue to be updated and evaluated until a threshold
determines it is not viable and it is deleted. Remaining states are then in competition
with decisions being made using the state of highest confidence. We can also guess if the
floor is flat, and if so, expect our foot to be grounded at local Z=0, and at this point
split the state. If our assumption is correct then the IMU and joint torques will match
with confidence and that state will remain, otherwise if the foot finds a pothole the 
state will die and the non-contact state will continue but the model will now work to
correct balance and maintain CoG over the limited support polygon.       

##### TRAC-IK

Orocos KDL apparently has some issues with humanoid IK/ID solving and sometimes false
negatives. TRAC-IK is a drop in replacement that apparently is faster and doesnt entirely
fail if a solution cant be found.

##### Minimizing Torques or Current usage

Since we now estimate joint torques using Inverse Dynamics and torque is directly related
to motor current usage we can attempt to minimize torque usage during movements. Possibly 
using this feedback to choose target poses/sequences that have shown to minimize torque.
We dont want to minimize each action though, so we should be filtering the torque using
a moving average over time and using this as a cost/reward parameter.
 