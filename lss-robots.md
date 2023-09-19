docker run --rm -it --runtime=nvidia --name=LssHumanoid -e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix/:/tmp/.X11-unix -v $(pwd)/source:/opt/ros2_ws/src --device=/dev/ttyUSB0 lss-ros-desktop-gazebo

alias ccbuild='colcon build --symlink-install'

colcon build --symlink-install && ros2 launch lss_humanoid display.launch.py
ros2 run rviz2 rviz2 -d src/lss_humanoid/config/lss_humanoid.rviz

# launch just the hardware on the RPi
ros2 launch lss_humanoid hardware.launch.py


colcon build --symlink-install && ros2 launch lss_humanoid display.launch.py
/home/guru/lss-humanoid/ros2/humanoid/install/lss_joint_publisher/lib/lss_joint_publisher/lss_joint_states --ros-args -r __node:=lss_joint_publisher --params-file /home/guru/lss-humanoid/ros2/humanoid/install/lss_humanoid/share/lss_humanoid/config/joints.yaml


# CLion Toolchain setup
  - Bender
  - Credentials: ssh://guru@bender:22
  - Remote Host GDB


# ROS Clion Plugins:
* Hatchery
* ROS Support


ttyAMA1 => TXD2
ttyAMA2 => TXD3
ttyAMA3 => TXD4
ttyAMA4 => TXD5


GPIO19 => Blue
GPIO20 => Mode/Reset
GPIO21 => Red
GPIO16 => Green


# Microcontroller boards:
RPi - obviously
Google Coral Dev Board - QuadCore A53 plus 4TOPS Google TPU supporting Tensorflow Lite!
   https://thepihut.com/products/google-coral-dev-board-4gb?ref=isp_rel_prd&isp_ref_pos=2


# Croccodyl example changes:
* createWalkingProblem() extracts foot translation but ignores rotation assuming the rotation is the SE3 eye identity. In my case I had to carry rotation through.
* I tried extracting out just the legs, like the TalosLegs example loader. It extracts using frameId<14. My URDF is different so I had to extract by name and reparent the geometry


### Balance Control

# PID loop required in Joint Controller
  * should be able to dampen or multiply control input using a manipulator
  * state now has pos, vel, acc and effort parameters

# Trajectory Control 
  * Need to store segment states as keyframes
  * Also need to store the velocity parameters in-between keyframes (perhaps as sliding control on the path
      * Possibly use a stack here, where stack elements can be a key-frame or parameter setting.
      * Behaviors can just modify the stack.
      * Execution can rebuild/adjust trajectories based on sensor input, only needs to construct next X parts of trajectory stack,
        so constant recomputation of trajectory can stay light. Even behaviors can be executed on only a small part of the stack
        at a time.
  * We can show path using line-list marker
  * The Control classes need to be able to add "soft" key-frames in-between the user ones so that balance is maintained (for example)
  * Control is basically the user-program, it updates feet/arm TF, but typically by instantiating Trajectories. We should eventually
    seperate Control into seperate ones like Balance, Walking, etc.

# KEY FRAMES and BEHAVIOR Controllers
Remember what final-cut pro said, "Behaviors were added because modifying key frames to model physics produced unrealistic
results and was too complicated." So we should follow this advice and have Behavior Controllers and Keyframing mix.
  * A walking controller needs to determine using Ground Reaction Force, leg contact friction and trajectory requests that a walking
    step is required and insert the required soft key-frames.
      * this same data is then also fed to the odometry filter. By the walking behavior? Or by humanoid dynamics? If humanoid dynamics
        then this is a post-calculation while trajectory is being executed. (probaby better in dynamics to keep behavior simple and 
        odemetry is updated no matter the behavior that causes it.)
  * Behaviors like Balance may instead opt to create soft keyframes for other idle joints instead...such as moving an arm to
    counter-balance. So Behaviors must be able to insert.
  * In the future Croccodyl could provide the soft keyframes
  * So behavior controllers are basically soft keyframe generators. Soft keyframes will always be from behavior controllers.
  * We can add symbolic elements, such as footprints.
      * User drags a manipulator that adds an odometry destination symbolic frame (frame with symbolic element)
      * So the path planner behavour adds mid-odometry locations to avoid objects
      * The stepping behavior adds footprints to the odometry path
      * The walking behavior adds arcing leg lifts in between steps
      * The balance behavior adds torso lateral shifts in the Saggital plane to maintain balance during step lifts
     (Not all these behaviors have to complete along a Traj request, they can be executed as required during walking)
  * I think we have to be able to select() a number of elements by type,
      * if we then select symbolic elements and add elements, shouldnt they be child of the parents (and invalidate on change)


# STATES
  * remember to add confidence factor to State so that we can have multiple current "possible" states that run 
    in parallel until confidence drops and that state is deleted. (Good for detecting foor contact)
       * Bifurcating trajectories?????
       * So add confidence to Trajectory itself!
       * The confidence should be a function on the state object, it checks current state against
         predicted state and returns a confidence. If confidence reversion occurs the other traj
         is taken and the bad one is pruned.
  * Add method to compare a state to another with tolerance. Compare joint position/vel/acc, TF, balance-factor. Use this to only
    draw segments that are different than current state. (get rid of flicker)
  * Could we allocate a bunch of states in an array, then use these in the interpolator? Would be more memory efficient.
  * I think we need to demarcate which segments are under control inside state
  * Only the finite N segments really matter to Trajectory in the end...so all elements must eventually map to segment control operations. How will we handle conflicts.

# MATRIX
I think we have a matrix here. We have symbolic complexity from top to bottom. We have time from left to right in discrete frames.
Each frame is a collection of elements. The same symbolic elements exist through time on the timeline, ahd therefor these element
states should be linked. So from timeline left to right the collection of frame elements are linked. We can the traverse a single
segment element in itself, both estimating it's time and building Orocos trajectories from it.
  * segments states from different timeline layers would not interact until the bottom. Duplicate symbol states are resolved using
    the confidence function.

# MEMORY
  * I think we invented music because it is a fundamental part of our motion dynamics. Even animals have been seen to enjoy
    music. In evolution, it would be a competitive advantage to be able to synchronize motion to a signal, weather that be 
    external or internal. 
    * Like musical tempo our learned muscle memory can be sped up or slowed down. So practicing Tai Chi
        or other slow motion activity really does improve reflex speed during high speed motions.
    * Variations of muscle motion can be combined, adjusted, turned up/down with different tonality and remixed.
    * Each limb can beat different like different musical instruments in a chorus.
    * Can we represent symbols on our timeline like notes on sheet music? - it seems similar.
    * Could musical compression algorithms help in storing robot muscle memory?
    * Could joint/segment motion be better encoded with sine waves? Could FFT improve muscle memory, motion generalization or remix?
    * We should be able to train a NN to take-over a timeline. Possibly some timelines like footsteps could be a NN+health function.
      Balance could also be a NN plus health function based on Balance+Speed+PowerEfficiency metric.

### AI Methods
   * Remember when I used ML to take simple geometry and create an IK solver? I reversed the input/output of the generation
     to train the NN in reverse. Could this be the secret to child learning? Our joints go all haywire but 
     motor-command => proprioception is being reversed to train the NN. Then our desired position, velocity, acc, force can
     be the input and motor-command is the output. This is establishing memory of what I attained apriori to re-acquire the
     same result later.

# Knowledge Representation
Excellent seminar on this by Michael Beetz, University of Bremen
http://www.open-ease.org/


# Operators
SymbolicOperators - when a given symbol is encountered (ex. odometry goal, footstep, grasp object) this operator generates more
                    frames or elements. Such as turning a odometry goal into a walking path, or turning a walking path into footsteps.
InterpolationOperators or StateOperators - 
OptimizationOperator - Evaluates inter-frame state for feasability or optimization and generates new frame and joint/segment
                    positions to optimize movement or trajectory.   

One type takes a left/right frame and inserts soft keyframes. Timing is possibly important here.
   Symbolic Operators are a form of KeyframeOperators
Another type is executed at various delta times to determine if a soft keyframe is required.
   Interpolation and Optimization operators are the same.
   Interpolation purely acts on state.


Operator prototypes?
Operators must operate on a state and probably have access to previous and next frame? Are operators in frame different then interpolation operators? - not really right...if I am maintaining balance (creating new frames) or adding footsteps (creating new
frames) I am doing the same. On these new frames, what operators get added to frame? all the previous ones?

Symbolic operators should be very fast to add/modify, but state/model ones are usually not. So we should be able to lay down 
symbols in place of state interp to save rerendering state a lot. Even moreso, can we lay a type of frame that only renders
when necessary?


   TODO::: Thinking now that Element should just be simple, no deriving it. It can have a boost::variant<> with some common
           data structures...then a boost::any in there too for user stuff (maybe possibly simpler if we add frame, segment, etc
	   kind of stuff in there anyway wheather the symbol needs it or not.



Frame elements is actually a stack, so if any element is changed it invalidates following elements. However, frame should be able to mark themselves as static where detection is turned off or specify their own tolerance. Deviation of position for some symbols like footprints or odometry paths may not really be that important.

    ///@brief The list of pending operators to apply to this keyframe.
    /// Operators are applied in sequence. As an operator is applied it is removed from the stack
    /// and rendered on the timeline.
    /// Some operators simulate external effects such as Coriolis, Centrifugal or Gravity have on
    /// the robot. Some operators act as control inputs to the robot joints for such things as
    /// balance or motion goal.
    InterpolationOperators Keyframe::operators;   // todo: split into two operator sets?





# TODO:
* Create a Keyframe::select() method for use on deep selecting frames/elements
* Collecting Segments from the tree is an operator function
* Rendering state is an Operator function
* Current rendering function needs to estimate distances/times....thats all
    * render recurses up the timeline tree, then down. Calculation done before and after (probably)
    * but distance of segments requires knowing the previous segment position
    * possibly rendering end condition could be an operator (assigned per Timeline or uses default)


# MUSIC

https://www.frontiersin.org/articles/10.3389/fnins.2014.00292/full

“birds and whales produce sounds, though not always melodic to our ears, but still rich in semantically communicative functions.”

Even plants, shells and other natural phenom can show a numeric basis and similarity to music. 

“Music is indeed generative, structurally recursive, and knotted to grouping.”

How music helps me concentrate. I mental ability was dragging like a blue ass fly. By putting on Rage Against the Machine and turning up the volume I was able to kick my brain into high gear and get work done. My brain synchronized to the energy and tempo of the music increasing my cognitive ability in a way the coffee couldn’t.

Was music a way to synchronize mother/father and child? Like we sing together (and synchronize our tone and vocal muscles), our muscle movements and mirror neurons are like us “singing together” except we are moving together in synchro.


Mistakes in AI modelling
https://www.youtube.com/watch?v=PYylPRX6z4Q&pbjreload=10


Users adds operators on TwoCC codes (they can add multiple ops to the same TwoCC)
Render on receiving new elements from operators checks for existing OperatorState collection, if not then it has to be created
    * OperatorState exists for every TwoCC:name pair
    * if User adds an operator, or removes one, existing states must all be updated to add or remove the operator from their list
    


TODO: Each element that is output from an operator must be grouped by ElementKey, and assembled and integrated appropriately...
    this could determine siblingship. hmm. At the least, the OperatorState must be loaded/created for given Key. Possibly then
    OperatorState as a whole should be sent to Operator (and contain left, right, prev...but then it isnt the same state obj)
     * error: newels are integrated (linked together) but then rendered. When rendered they are create seperate op-state but
       they are not linked in graph accordingly. They need to be grouped, then linked, then each group rendered.
    

ELEMENTS to OROKOS
Step 1: Render SegmentFrameElementType to Paths (Path_RoundedComposite)
	* require a path for each segment name
	* path/child relation must be kept
	* paths are computed, then time is fed back up the rendering path...so orocos trajectory planning must be done in the render loop.
Step 2: Calculate velocity profile and feedback into rendering
Step 3: Adjust velocity profiles of segments to sync them
Step 4: traj output for IK requires segments to be relative to something like "robot base". If paths are synced then this isnt
        hard since we get KDL::Frames for each segment at time 't'.

# TODO!!!!
1. DONE  Each element must have a time (maybe measure). Interpolators will interpolate the time too (1/N), but operators can adjust this
timing....so for feet, it will need to adjust and make the leg left 1/2 time, and 1/2 is holding (no movement)
2. For state, each element can have a ref ptr to state element.
3. State element holds Robot State obj in data field, each change in state causes a soft key. Posssibly we can have hard keys too.
4. All the SegmentFrame objects will update the State obj, a new key will have to be made if segment sets state for a time that 
   currently doesnt exist (can discretize this based on servo update loop Hz). I like that state is another Lane on the timeline.
   Dynamics update loop now only needs to refer to the state lane. Also, external objects can have a different state/model they
   link too via the state ref field.
5. Extra field in op state tracks when state is rendered (different than element rendering, happens later). When state renders
   each segment element sends its update to state. After all operators update state (that want to), then the state lane
   is rendered. So each State object is computed, IK, etc.
       * when does SegmentFrame link to state lane? On creation, or state render? This determines when State objects are created
       on the lane too...so think about it.
6. Main visual system or servos can create Orokos paths/trajectories with this states...done like a dinner!



* Collection of segment names need to be synchronized over time based on the parent/child node order. Probably if there is no parent/child relation present then these movements are not related and dont require velocity sync. The ones that do require sync
must arrive at target positions at the same time. That means we have to render paths using max vel, then figure out which is slower and then go back and recompute the others using time duration instead of just SetProfile(max..vel).
* How to easily keep segment relations? Can the element tree do it for us easily?
* If segments set time duration on parents, then multiple segments may try to set parent. The longer distance one should
  prevail. But that means parents now update children in other branches again, causing them to recompute. How to make
  this faster? Possibly track what track is most often dominant and do that one first.


Walk and Balance Todo:
[1] Possibly look at torques on the leg servos to determine if torso is lopsided (like it often is). 
      * Add a torso rotation until the hip? servos read 0
[2] DONE Better plan the feet in balance()
      * look at limbs in contact, if 1 its simple, we need to be over that leg. If 2 then we need to be over the midpoint, if >2 then form a polygon and keep CoM over it.
[3] DONE CoP in current state was dependant on the drift. It's calculation is also only using contact origins, not the polygon. fix. Probably the same in trajectory.
[4] DONE now instead of moving legs in contact, move the base to get CoM over CoP...also move dangling legs by this amount. what above arms?
      * what do we do with dangling legs?
[5] It might work better if ContactState was actually EndEffectorState and the state could refect arms and legs and if they are
    in contact or a point location is requested (such as grab, or put a foot 'there')



Xacro notes:
    <xacro:if value="${'$(arg target)' == 'gazebo'}">     to compare an input arg
    <xacro:if value="$(arg target)">   -- if target is a boolean input

or store in property first:
    <xacro:property name="target" value="$(arg target)" />   -- move input argument into property
    <xacro:if value="${target == 'gazebo'}">                 -- compare from property



# DEBUG NOTES
* Settings | Build, Execution... | Debuffer | Data Views | C/C++ | Enable GNU C++ library renderers often causes Ros2 to crash when inspecting std stuff
Crash is in eprosima::fastrtps::StatefulReader::processDataMsg() ... reser_Cache...int_malloc...malloc_consolidate.
Thread 12 was in gazebo_ros::GazeboRosFactoryPrivate::DeleteEntity(), was the delete causing the plugin to unload and therefor leaving resources freed after the delete occurs?
	* this thread is waiting for world to show model doesnt exist
Problem was plugin must disconnect from Gazebo before releasing the Executer.
* gazebo_plugin create ros_gazebo::Node
* that Node also has an executer, could we use it to run our controllers?



Possibly similar bug in all the gazebo_plugins:
futex_wait_cancelable 0x00007f493481f376
__pthread_cond_wait_common 0x00007f493481f376
__pthread_cond_wait 0x00007f493481f376
<unknown> 0x00007f4934b95133
gazebo::sensors::SensorManager::SensorContainer::RunLoop() 0x00007f4934b9019b
<unknown> 0x00007f49335c043b
start_thread 0x00007f4934818609
clone 0x00007f4934d04293


CLASS LOADER ERROR
Now getting a class loader error, could we be getting closer? This showed up when I explicitly reset the model_nh and controller_manager inside GazeboRosControlPlugin:
gzserver-1] Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.

CONTROLLER MANAGER
I tried to do a stop_controllers via a switch_controllers() call. Put all the controllers in the stop_controllers argument. This just causes the whole thing to lock up. I dont know why but even calling the service via ros2 also locks up the system.


rclcpp PR1469
Thinking in pseudo-code here, but what if no sharedptrs to sub-interfaces were given out, but instead, all get_X_interface() methods returned a tuple containing the sharedptr to base_interface and what sub-interface is resolved. Then all X_interface references actually hold the base_interface alive. This tuple is really a template class that contains a shared_ptr, the interface ID of some-sort, and probably a private cached raw ptr to the interface. It would have the same public interfaces as a sharedptr so no user API changes beyond a recompile.
Caveat: but what does holding the base iface give us? it doesnt keep the sub interfaces alive...so we just open that up. Even holding a sharedptr to base and sub iface does what?



I was getting log errors "Error in destruction of rcl service handle imu_sensor/get_parameters: the Node Handle was destructed too early. You will leak memory". 


# ensure plugin is being loaded in dev mode (in ~/src/ros/install/lib)
rm libgazebo_ros2_control.so && ln -s ../../gazebo_ros2_control/gazebo_ros2_control/cmake-build-debug/libgazebo_ros2_control.so libgazebo_ros2_control.so
rm libdefault_robot_hw_sim.so && ln -s ../../gazebo_ros2_control/gazebo_ros2_control/cmake-build-debug/libdefault_robot_hw_sim.so libdefault_robot_hw_sim.so





8.5Kg.cm required torque at the knee joint when standing from a sitting position. LSS HT1 servos are 29 kg.cm static and 5.8 kg.cm dynamic.


# Time Optimal Nonlinear MPC - Read this!!
https://eldorado.tu-dortmund.de/bitstream/2003/38313/1/Roesmann_Dissertation.pdf
http://www.reflexxes.ws/applications/robotics.html - Reflexxes

# Gazebo
* Nice page on controlling Gazebo with the ROS2 plugin: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete
* http://gazebosim.org/tutorials/?tut=ros_urdf#Joints in Gezebo for Joints, provideFeedback can publish joint torques back to ROS, and implicitSpringDamper should get the compliance we want
* export GAZEBO_MODEL_PATH or GAZEBO_PLUGIN_PATH, or use gazebo_ros_paths_plugin plugin to load these env variables from package
	* gazebo_ros_paths_plugin is no longer a plugin, see https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths
	* could use GazeboRosPaths as done here: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/dashing/gazebo_ros/launch/gzserver.launch.py#L29-L40


xacro lss_humanoid.urdf.xacro target:=gazebo > lss_humanoid.gazebo.urdf
gz sdf -p lss_humanoid.gazebo.urdf

# run gazebo
GAZEBO_MODEL_PATH=~/src/lss-humanoid/ros2/humanoid/src gzclient
GAZEBO_MODEL_PATH=~/src/lss-humanoid/ros2/humanoid/src gazebo --verbose --pause -s libgazebo_ros_factory.so -s libgazebo_ros_state.so -s libgazebo_ros_init.so

# spawn robot in Gazebo (see https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
ros2 run gazebo_ros spawn_entity.py -entity humanoid -x 0 -y 0 -z 0.3 -file /home/guru/src/lss-humanoid/ros2/humanoid/src/lss_humanoid/urdf/lss_humanoid.gazebo.urdf

#spawm robot with some rclpy code:
https://answers.ros.org/question/314607/spawn-model-with-ros2-gazebo/

gazebo_ros_controls code updates:
* todos in default_robot_hw_sim.cpp for deciding what command interface this actuator is

ros2 launch lss_humanoid simulation.launch.py


ros2 launch gazebo_ros2_control_demos cart_example_position_pid.launch.py
ros2 run gazebo_ros2_control_demos example_position

# launch simulation
ros2 launch lss_humanoid simulation.launch.py display:=rviz


Issue seems to be in creation of ParameterService on the Node for ControllerManager. Execution flow is:
ControllerManager::ControllerManager => Node::Node => ParameterService::ParameterService => Service::Service => 
	rcl_service_init => rmw_create_service => eprosima::fastrtps::Domain::createPubliser => PublisherImpl -> PublisherHistory => History::History => CacheChangePool & allocateGroup


# Tutorials:
* ROS Control - http://gazebosim.org/tutorials?tut=ros_control
* Using URDF in Gazebo (gazebo tags, etc) - http://gazebosim.org/tutorials/?tut=ros_urdf#Joints
* Using Gazebo Plugins - http://gazebosim.org/tutorials?tut=ros_gzplugins#Tutorial:UsingGazebopluginswithROS
	* also mentions Force Feedback Ground Truth
* Using xacro - http://wiki.ros.org/xacro
* ROS2 Migration: Gazebo ROS paths (how we tell Gazebo about Model/Plugin paths) - https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths


# Wiki:
* ROS2 Integration Overview - http://gazebosim.org/tutorials?tut=ros2_overview&cat=connect_ros
* URDF Joint description - http://wiki.ros.org/urdf/XML/joint
* Joint Safety limits - http://wiki.ros.org/pr2_controller_manager/safety_limits
* SDF format - http://sdformat.org/spec?ver=1.7&elem=joint
* ROS2 CONTROLS ROADMAP https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md#1-industrial-robots-with-only-one-interface




AppleInsider has a $42 bundle of software that is of interest. It has:
* Parallels
* Luminar image editor
* PDFpenPro for editing PDFs (I edit PDFs a lot and use Adobe Acrobat a lot but I *ahem* shouldnt)
* Screen recorder
* Art Text


# Strategy:
OVERALL: We need to transition to lifecycle:
  * DONE nodes run constantly and dont have to be restarted to test a new scenario
  * all parameters are dynamically configurable (required for training)
  * DONE a node can be built/tested/debugged without restarting whole system
TODO:
[1] Robot Dynamics Updates:
    [a] DONE Need to affix the feet, should this be done in robot dynamics or isolated?
	* should be robot-dynamics I think (at least for now)
	* each iteration, take contacts list, move robot so all contacts stay constant...use friction if contact placement isnt perfect...output a base-tf and odometry differential
    [b] DONE Stand manipulator jitters, needs a movavg to only slowly update
    [c] DONE should reset sim model when it activates (if detecting sim environment)
    [d] DONE Publish model state to topic
    [e] DONE Subscribe to trajectory point messages and add to trajectory planner
    [f] Publish model/limb static config to transient_local topic
[2] DONE Implement reset and debug any nodes that abort/crash
[3] DONE Implement joint transforms in robot_dynamics so we can nuke robot_state_publisher
[4] DONE Load URDF from topic
[5] DONE Update to ros2-controls RM/CM, implement the servo controller as SystemInterface
[6] Implement a balance metric (which we can then train from)
[7] Implement walking algorithm in python node
	[A] Turn all parameters, including the walking ones into dynamic parameters
	[B] Port c++ walking algo to python
[8] Apply Hyperopt to control dynamic parameters based on balance metric

# Trajectory topic:
A trajectory profile or traj_segment which is a list of points (vector - optionally with rotation?)
but we want to synchronize some profiles, so for example a leg lift syncs with other legs support.
Synced profiles should have the same length, if one cant complete in time then it should slow the 
other one down.

# Trajectory Refactor TODO
1. updateState() is now for sending current state into traj and re-rendering, usages need to be changed over to getState() or some mix of both?
	Issues:
		* INCREMENT_JOINTS_TOO_SMALL, joint position increments too small
		* references to segments modified in traj are getting state frame instead L185
		* add fill parameter? It decides how to bridge holes between old and new traj
			- I am defaulting to fill stationary
			- could add extend (keep V), blend (catch-up), fast-move
		* when updateState() doesnt update any states the viz just gets jumpy. This is
		  because when updateState() does do a state update it is from the remembered
		  trajectory state, not the current state which is what robot-dynamics passes
		  into updateState()
		* do the relative position on Rotation as well (forgot, its global now)
		* set default vel and acc in traj and/or model
2. Simulation issues
		* I am feeling like the simulation issues are a symptom of the URDF parameters
		  which also affect Orocos KDL dynamics stuff....we may need to track down what
		  is going on...at least, be 100% sure its not a URDF issue.
		* PidROS is showing a wierd super high i_error value, even when I have gain of 0.
		* Pid error (input) term is pretty stable at ~0.05, output is whack.
				- The ~0.05 error was causing wind-up. High I-error is still modulated
				by clamp and min/max but we dont see it in the msg.
		* Pid.I=0 helped a bit, Turning on implicitSpringDamper made a big difference.
		  SpringStiffness=10, SpringReference=0.01
		* Possibly the robot_dynamics sending height-from-floor updates is confusing gazebo?
2. Fixup Control::update(state, time) - this method adjusts the trajectory each time
   the current robot state changes.
3. When new state is set, detect changes, and implement invalidation of rendered segments that have changed (L133)
4. Implement trajectory mixing when trajectories overlap (L144, L147)
5. Should we update dynamics on trajectories? (L221)
6. Old trajectory code to boneyard!


# WalkingAlgo
  * outputs trajectory expressions
   - Does it need to get the durations back?
   - How does know when to start the next movement? unless it has to wait for the state to
     update, and possibly seeing trajectory fill amount in the Limb state.
   - switch the Trajectory expression to a service? (or both!)
  * RobotDynamics receives trajectory expressions and renders to KDL trajectories as needed
  * trajectory state needs to be added to ModelState IMO
  * walkingalgo could always add a "stand normally" or "sit" trajectory but continously 
    overwrite it, but it would run if the walking algo crashed.


# Robot Dynamics refactor:
Split robot-dynamics into seperate modules and specifically remove the walk code into 
its own node, maybe python. These are the current functions of the robot-dynamics node:
  * DOME Publish state for joints (PVAE, internal/external forces), segments, imu, CoM/CoP, limbs, 
    contacts, support and support polygons.
  * DONE publish tf_static for robot base to floor and footprint frame. Now publishes all TF.
  * Isolate manipulators and control
  * [bonepile4now] timeline trajectory doesnt work right, keep simple trajectory.
  * keep balance in robot-dynamics node (I think), which means some kind of "acceptable imbalance parameter"
    should be given for trajectories.


# Physical Nodes:
Gazebo: client and server
display:
   robot_state_publisher
   localization
   rviz2
   hardware: 
     lss_joint_publisher
     imu/bno055_driver
urdf_publisher
robot_dynamics


# Simulation Nodes:
Gazebo: client and server
ros2_control gazebo plugin (joint_controller)
robot_state_publisher
localization
robot_dynamics
rviz2
urdf_publisher
(spawn_entity)

# Reset sensative nodes:
Gazebo
  - no issue, gazebo actually becomes the time master and so the time gets reset, publishes to /clock
ros2_controls_plugin
  - limits interface was relying on period being positive, now it detects negative delta and resets limit handle
  - min/max asserts when timestamp resets, this code doesnt trip now that DT is never negative
  - fixed plugin Update() handler to reset last_update_time variable when time jumps
robot_localization
  - rviz would show robot not updating until the previous reset-time was passed again
  - ros_filter.cpp detects time jump but just logs diagnostics and ignores messages, now it calls reset if 
    a reset_on_time_jump parameter is set in yaml file.
robot_dynamics
  - no issues

Reset flow:
* launch file detects if a node goes down:
	* if possible (robot_dynamics available), run a "sit" or shutdown sequence
	* deactivate other nodes
* launch file detects if a node comes up?
* are all nodes available? if so, start activation of nodes


Nodes:
  * node imu_sensor
  * node odom_controller
  	- imu and odom are created by gazebo and are from gazebo_plugins (in gazebo_ros_pkgs)
  * node gazebo_ros2_control
	- this one is sticking around
	- created in gazebo_ros node.cpp:107 in Get(sdf). It's not static.
  * node gazebo_controller_manager
  	- created in gazebo_ros2_control_plugin.cpp:296
  	- controller_manager class from ros2_controls
  	- creates humanoid and joint_state controllers from yaml
  * lifecycle-node humanoid_controller
  * lifecycle-node joint_state_controller


Launch Events: (this was from a design doc, may not be correct)
launch_description = LaunchDescription(...)
# ...
def my_process_exit_logger_callback(event: ProcessExitedEvent) -> LaunchDescription:
    print(f"process with pid '{event.pid}' exited with return code '{event.return_code}'")
launch_description.register_event_handler(
    ProcessExitedEvent, my_process_exit_logger_callback, name='my_process_exit_logger')


Note:
The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.



##  Humanoid Demo Issues:

# State Relative Frames
* Really take a hard look at TF in state because it's not clear if those transforms are relative to algo or local-segment
      * humanoid_dynamics is publishing TF but those TF frames are local to the parent joint not a common base or odom frame
      * we need to be able to get segments in odom frame sometimes, but other times we want it in robot base frame
      * we can write methods to transform entire state to alternative frame
      * seems like we should always work in local robot base frame, but just use odom for visuals and manipulators


# Fixing IMU, Footprint and imu-base transforms
* Need a model method to publish state transforms for base, footprint based on IMU and such data
    - it should run after contacts state has been updated
    - need IMU class to read /imu/data topic and populate state

# IMU update
1. Create chain from IMU to footprint
2. Compute FK on chain, using existing joint pose/etc, to determine how base_link and footprint transforms
3. now IMU must rotate around contact-points, so I think we actually need to FK to limbs as well
4. Fit the IMU transform to the contact points
- IMU state is there, but not being updated (warning: compute_TF_CoM starts with imu transform!!)
- You might be able to further enhance pose computation by integrating accelleration, into V, then into P to compute
an expected position delta of IMU in space. Obviously EKF here.


# Robot Localization & IMU (From Links)
* Not publishing odom transform - https://answers.ros.org/question/226085/robot_localization-not-publishing-transform-to-odom/
* Gazebo p3d localization plugin - https://answers.ros.org/question/222033/how-do-i-publish-gazebo-position-of-a-robot-model-on-odometry-topic/
* Gazebo Ros2 IMU migration - https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-IMU-Sensors
* Noise parameters for IMU - https://github.com/ros-simulation/gazebo_ros_pkgs/pull/801/commits/7c60a31ec2455cd08a5266ae8f2bd22b7f96f0ab
* RL using only IMU drifts, checkout his footprint frame location! - https://answers.ros.org/question/301144/robot_localization-estimate-using-only-imu-drifts-for-a-stationary-robot-gazebo-model/

# Nailing the Foot to the Floor
     - items related to EKF prediction marked
* [EKF] keep feet position nailed down if static_friction is > threshold
* compute new FK vector to feet using new base orientation (is it delta or not?)
* [EKF] compute vectors from support feet to nailed position
* offset all segments by the same vector including the base
* [EKF] see if feet, CoM and base (IMU reported) jive...if not suggest new orientation as a movement towards our estimate
* report base position/odometry to RL
* [RL] keep covariance of IMU orientation vector and RPY accellerations in favor of IMU, but keep base position and velocity favoring robot_dynamics



APRIL 25th TODO:
* change manipulator state to be part of State object, not Model. But functions to toggle manipulators are in InteractionController.
   * InteractionController control what limbs/elements are active
   * model::updateIK calls getLimbMode and queries limb as well
   * Trajectory::updateState sets state type to Trajectory and calls model::updateIK
* Control::interact does its own IK and should be moved into Limb model
* Add weakptr to Model in State

	
BALANCE:
* balance also computes IK, can we do all the limb endpoint manipulation first, then do the IK?

Introspecting ROS message in Python: ros-foxy-rqt-msg and ros-foxy-rqt-srv


# Refactor into seperate modules
- ros2_robot_dynamics
- ros2_lss_hardware
- ros2_lss_hexapod
- ros2_lss_humanoid



# Robot Dynamics
* slow servo updates because robot_dynamics is only updating at 10Hz
   - increasing update rate causes visuals to update too fast, must seperate visuals into (a) update when changed, and (b) every second or less
* Trajectory now updates limb-request, which then UpdateIK uses as the limb to-src.
   - Could rename updateState to updateEffectors.
   - model::updateIK reads limb->request.mode and handles Seeking, but doesnt handle manipulation...if it did, this would mean
      manipulation doesnt have to do IK anymore.
   - Create a service alternative to creating/adding a trajectory
       - python has async_call() for ros2 services, returns a promise, our Task can then use this know when trajectory was completed
       - c++ server-side has an async option where you can call service->send_response(header, response) or see bottom for final solution
       - Links:
           rolling docs:  http://docs.ros.org.ros.informatik.uni-freiburg.de/en/rolling/How-To-Guides/Sync-Vs-Async.html
           foxy/writing services: https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html
           issue/request: https://github.com/ros2/rclcpp/issues/491
       
       - how would cancels be handled?
            - this is where just monitoring position for reaching target may be more appropriate,
              and if we name our tasks for what they are intended for then we just keep updating that
              task as needed and have it republish...kind of like that task is handling leg independantly,
              or maybe like sub-sumption arch?
* Limbs could be renamed Effectors, and the base is an effector
* Define limbs/effectors in SRDF
* Migrate State class to a Ros message
	- move visualization code out into a seperate node


# Trajectory Refactor
- RenderInterface must be able to store and track complete robot State during rendering in order
  for the getRelativeFrame()/etc to work. That means it must internally track all the timelines
  so that it can determine if it can just fetch from initial_state (not a manipulating segment) or
  it must go and fetch state of another timeline before computing getRelativeFrame()...it must
  even determine if that timeline has been rendered enough to even do the state fetch.
      * fix TrajectoryRenderEnv getRelativeFrame() function
* add "features" to MultiSegmentTrajectory message, it can turn on balance, dynamics and contacts.

## Issues
   - render() will generate static trajectories for holes, but we already do this in get_state()
   	- I removed the hole logic, but I probably need to add it back in but only between multi-trajectories (which are not used yet)
   - getRelativeFrame() is not computing state yet
        - need to adjust st(state) for both from_link and to_link for dependent segments

# Inverse Kinematics
* Stop creating KDL ChainSolvers per-loop, make global
   - DONE have LimbKinematics and ModelKinematics (has X limb kinematics)
   - DONE compute mass and CoM in Limb Kinematics and model kinematics has less to calculate
   - DONE moveEffector will update all kinematics and invalidate dynamics, etc
   - joint state updates can also feed into Kinematics and it will compute FK and invalidate dynamics, etc
   - port dynamics, etc over to Kinematics
   - port publishTF/staticTF to Kinematics?
   - DONE implement moveBase(KDL::Frame)? also moveEffector can recognize base-link movements
   - get rid of State::Limb stuff, or port to Kinematics?  (Trajectory::updateState is a mess now)

# Robot Dynamics UI
* Manipulating a joint will stop the trajectory
* Manipulation should be just another Ros2 node that sends trajectory updates to robot_dynamics

# Ros2 Controls Hardware
* implement joint parameters (set in urdf <joint>)
  - examples: invert, lss-type, reset-on-configure
* implement a way to reset servos (if they enter failsafe mode)

# HEXAPOD ALGORITHM
- legs holding on the ground should have an upward force
- if you lift 3 legs then the legs in contact will tell you the ground plane
- try to keep legs in contact maintain equal power

- move base forward direction at constant velocity
    - while input, set target a certain distance from the robot, set velocity on traj msg
    - on input break, set target just slightly ahead (will slow to a stop)
    - moving base is not part of gait, just the footprint targeting is...we can change
      the gait by changing the gait function in python. The gait runs on a timer
- when legs reach a specific radius, send a leg lift and move trajectory
    - we need to maintain a pose delta between our 2 sets of legs so that we dont get a situation
      where all legs are in left and move territory. 
    - compute a desired delta between the two walking leg groups, and set a rule to change
      footprint targets to establish the delta over time



# Refactor Todo
Getting robot_dynamics down to the basics so it can be distilled onto the RPi. Removing the visual components
to become standalone nodes which can run on a desktop alongside RViz.
- State needs to be closer to /tf and /dynamic_joint_states (or just /joint_states)
- Model needs to be a helper class that loads/manages URDF and SRDF and limbs
              Robot Dynamics
- will compute and publish dynamics, tf, ik, contacts (I think), so Model class doesnt need to have
  this stuff, it just has model utility functions...would it help process subscribed tf, dynamic data, etc?
- publishes odometry, actually detects when the robot has moved in odometry via the limbs in contact
- possibly it should still handle compliance?
- probably should add gravity vector to ModelState::forces::external
              Control
- Control and Trajectory could be refactored out too. I think Control+Trajectory should be together
- Control publishes LimbState not Dynamics
- Control can then just merge into it's main component
              Contact State
- We can move the limb/effector/segment contact sense into it's own component
- It receives dynamics state from robot_dynamics, determines if contacts occur
- publishes ContactState
- robot_dynamics can subscribe to ContactState and update the robot body (also fuse with IMU)
- outsourcing contact detection means user can replace or enhance contact detection themselves
              Collision Detection
- Uses the URDF collision models to detect when robot will self-collide
- possibly a part of the Contact State component - maybe
              Trajectory
"The existing trajectory module will either be Awesome! or a complicated mess"
    ## It was a mess, I got rid of it in favor of simple actions ##
- Possibly the trajectory could just be a simple singleton spline
   * Why predict trajectory path 2 - 5 seconds in the future?
   * The humanoid/hexapod node can constantly send updates so it doesnt need to track planned vs state divergence
   * since we will send updates constantly, no need for point arrays and complex stuff, just simpler spline line with speed/accell
- Refactor trajectory out of robot_dynamics (stand-alone) so user can use different planners (Awesome! or simpler streaming one)
- Trajectory action should allow multiple segments, but if so, they are still tracked as a single progress/complete event.
   - so we can lift legs as a group and only need a single complete event to indicate state transition
              URDF/SRDF
- Provide Listener classes for URDF and SRDF. We have this in the main src file for RobotDynamics but should wrap it for simplicity.
- ResourcePublisher may need a fix to xacro and publish both files at once.
              Manipulation
- Remove manipulation code from robot_dynamics
- After visualization refactor is done, create a C++ manipulation node that is sending trajectory messages
- also handles RViz menu implementation
              Calibration
- Remove calibration code
- Add calibration interface to LSS ros2 control
- Add calibration features to visualization node
              Visualization
- Convert State to a Ros2 message, then derive from that msg to add 'private' members and methods
- We can have a second Robot Model in RViz2 using a /tf frame prefix and set Alpha
    - either way, we should migrate to using /tf publishing from RD instead of visuals
- Remove visual code from robot dynamics, ensure we are publishing target and trajectory state in the right places (as namespaces)
- Create new C++ project that subscribes to published states and  visualizes on RViz2

12:51:55 110W

# TRAJECTORY ACTION SERVICES
we could have a single limb action and coordinated (multiple) limb actions. We should create action service classes for each type just like we do
for listeners and broadcasters since each action service class type requires custom action messages for goal, progress and result. However they
must add into a single Action list since new actions must cancel any pre-existing limb operations.
    * Will the main Control class or Action service classes do the pre-existing search and cancel behavior? (I would rather Control class do it)
# Low Priority
- Motors seem to be set to low rigidity
- push common code to Robot.py lib
- only publish some data when publisher->get_subscription_count() is non-zero
## Trajectories
- Implement body/base effector(s)
   - complete base effector support to trajectory actions
- Add polar coordinates mode to expressions   (only does x,y so z remains in rectangular space)
- Trajectory cancelling and mixing
   - Instead of erasing old trajectory immediately, we could save it to extract the target position/velocity and mix with the new one
   - if new trajectory is in the future, merge with new one by extracting composite path and adding to new one until ts-start
   - mix splice area by parametered amount of seconds, or perhaps better yet only if user specifies a ts-start in the future, we then morph
     old trajectories path from current to the first endpoint of the new
- implement sync_duration option in Coordinated and Linear Trajectory
## Contact Localization Node
- Limb::State only needed for updateContacts() and updateNailedContacts() so seperate Contact detection to a seperate class
- In dynamics, maybe add floor/contact delta to odom instead of body, thus in control we take odom straight from current state
   - for now, binding position only on target to current
- Contact::slippage - whatever this is supposed to be..a metric og how likely a limb will slip? or a measure of how much it has slipped?
# Final TODO
- Implement kinematics using Orocos Tree solvers
- migrate the support_margin calculation to Control
- Remove Limb States from Model
   - add to Control only, most of Model only uses Limb models not state (ex. support polygon)
   - requires moving updateContacts/NailedContacts() to Localization node
- add motor reset side-channel
## Release TODO
- Implement LimbState rotational velocity
   - fix trajectories always starts at velocity 0, have it start at current velocity
   - linear velocity is done, but rotational diff(...) is returning bad results
   - need to apply to single and coordinated trajectories too


# Tree Solver
- populate endpoints_ directly from limbs; get rid of effector_updates
How it works:
The Online solver works in velocity q-dot space:
- get the current endpoint frame using FK and the input "guess" joint angles
- get the twist by taking the diff() between the above frame and the desired frame
	- this is then limited by the cartesian velocity, so vel limit must be a update-rate limit not per-sec
- save the twist of each endpoint, then pass into the IK velocity solver
	- we pass in the "input cartesian velocity" and we get back "output joint velocities", so the IK is converting between cart to joint here
- enforce the joint velocity limits
- add the joint velocity to the current joint positions (integrate)


# Base Transformations
I think we have this a bit wrong...we have no actual direct control over the base, so we absolutely cant transform the base as a segment. We
must transform it using operations on the supporting feet. (FYI If no feet, then transform cant be done.) So make base target = current, and
be done with it.
- setting base target<=current works, the robot base translation is causing legs to move and the robot to walk. Y is aligned with forward/back
- however, once legs move, they change state (mode or supporting flag?) and they no longer are affected by base move translations



# Robot Localization
Basic node is created with subscriptions to Robot Model and Control State and Odometry publisher.
- Will need to load URDF since in order to detect contact we have to have the point cloud of the effector
- misewell do contact detection using collision model then
    - possibly robot_dynamics could detect collision, but do nothing, just publish the data..but requries ground map right?
- port Model::updateContacts() to be the main update() code for the localizer
- port Model::updateNailedContacts() to take nailed feet info, route back through robot base_link, and compute base_pose change to keep
  the nailed feet in the same position
- pull in IMU data
- compute the odometry estimate
  - add a fusion model? perhaps based on R_L


# CONTROL ALGORITHM
There is confusion between target state and limb state/request - er, whatever. Also target and thus trajectory get's out of sync with sensed
state so something is missing in the target update loop. We need to simplify the control structure and develop the rules of how target state
is synced with sensed state.
## Base
Base should always updated from current? Since limbs will determine base orientation I think it makes sense that quickly sensed base
should follow any intended base target command. So if we command base pose...IK will update limb targets, but how do we then expect sensed base
to hit commanded base?
* I think base must be an Effector, like limb...so we command the base as effector, it doesnt update target though, only through IK and
  joint updates does it feed back through sensed state and then to target state.
## Limbs
* Control component should copy limbs (refs) and then (later) an action can exist to create temp limbs. Model will only ever contain the URDF
  defined limbs but the user can create and control temp/new ones.
  	- Means all model publish or Limb functions would need to be refactored out or something - maybe not a bad thing. Create broadcaster classes.
* Have a rigidity control per axis per limb. Each joint's stiffness will then be controlled based on the joint axis. Can have moving and holding stiffness.

# Control during different contact modes
- How does status work wrt support?
  - If mode=AutoSupport then status will be Holding or Supporting depending on contact-detection
  - If mode=Support and contact-detection doesnt detect it's supporting should status be Supporting or Holding?

# Hexapod Walking Algorithm (beat mix)
- leg tri-sets often move at the same time, have legs try to lift as close to a beat as possible
  - so maybe establish a beat, then just don't lift legs if they are already in the target spot...
      - except when little motion is occuring we enable leg adjustments on beat
      - so we need an activity metric, and the lower the activity metric the more sensitive the adjustment metric (more likely to adjust)
  - each beat, take the tri-set with the lowest support margin estimate
  - legs sets may still move anytime (under strange command input) but the beat should establish a regular pattern normally


# ERRORS
# !! model reference is going NULL
- possibly in TrajectoryRenderEnv?
- Model destructor is not getting called, so the class isnt being freed (so probably not a refcount issue)
- not due to lifecycle state change

# abort when debugger breaks
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  goal_handle attempted invalid transition from state EXECUTING with event CANCELED, at /tmp/binarydeb/ros-galactic-rcl-action-3.1.2/src/rcl_action/goal_handle.c:95
      ^^ might be caused by exception in publish()
# Stack Smashing
[INFO] [1641423126.568857848] [robot_control]: Segment left-front in goal request doesn't exist in state
*** stack smashing detected ***: terminated
Signal: SIGABRT (Aborted)
# Action contention (happened with leg adjustment)
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  goal_handle attempted invalid transition from state EXECUTING with event CANCELED, at /tmp/binarydeb/ros-galactic-rcl-action-3.1.2/src/rcl_action/goal_handle.c:95
Signal: SIGABRT (Aborted)


# Rotate the base body
ros2 action send_goal /robot_control/linear_trajectory robot_model_msgs/action/LinearEffectorTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
id: 'turn'
linear_acceleration: 0.0
angular_acceleration: 0.1
mode_in: 3
effectors: ['base_link']
velocity: [{linear:{x: 0.0,y: 0.0,z: 0.0},angular:{x: 0.0,y: 0.0,z: 0.1}}]
"

ros2 service call /robot_control/reset robot_model_msgs/srv/Reset "target_state: true
trajectories: false
limp: true
"

ros2 service call /robot_control/reset robot_model_msgs/srv/Reset "target_state: true
trajectories: false
limp: true
"



# Webots Simulation TODO
* Correct collision models
    - simplify models
    - correct coordinates
* check/verify inertia and mass parameters
* configure launch file and config for Webots
* implement ros2_controls controllers config_and_start in launch
* implement Sim Reset functionality

## Webots Questions
* how does the webots_ros2_driver and webots_ros2_control nodes work?




# SCHOOL DEMO
- auto start Ros2 controllers
    - see this file for one way to start ros2 controllers
    - https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/launch/servo_cpp_interface_demo.launch.py


# LSS ROS2 Repos
RobotDynamics - senses joint and IMU/etc state and calculates model state like forces, floor contact, etc
Trajectory/Control - high level control of robot limbs, outputs to joints
Manipulation, Calibration, Visualization - these 3 nodes can be used for Rviz control/configure user interface
- LSS-ROS2-Hardware
- LSS-ROS2-Robot-Dynamics
- LSS-ROS2-Hexapod
- LSS-ROS2-Humanoid


ros2 launch lss_hexapod hexapod.launch.py | grep -v "Invalid frame" | grep -v "buffer_core"

# Completed Updates
* Trajectory publishing
   - Target must not be updated from current state, so robot wont fall slowly due to gravity
      - only on motor enable should target be set to state
   - Target should be updated from limb/effector target or trajectory
      - it looks for mode==Seeking
   - Publish target always
* Trajectory Rendering
   - Trajectory timeline key is a double! should be a decretized int (based on control frequency)
* servo positions only updated after sending a trajectory with servos disabled
      - need to make it always send positions, even if its just current state
      - but it can't just use current state (in any way, even via trajectory/target) because
        in the presence of gravity it means the robot will slowly fall...kind of like compliance.
      - should we have it limit trajectory updates only to requested limbs? 
        ...but if balance is to be in play then trajectory + balance + state needs to be integrated
      - maybe make trajectory based on target, then it's basically mixed for us
* Move trajectory code from robot-dynamics to Control class
* publish preview trajectory via /tf namespace
    - refactored publishTransforms to publish to any topic
    - add a TransformBroadcaster to Control for preview
    - when preview is active publish to /robot_dynamics/traj
* Convert timeline to be per-segment, there is no need to tangle segments up together
* get rid of storing State objects, the state of a segment is just it's KDL::Frame
      -copying Trajectory objects causes crashes because VectorTraj is storing pointers
      -can't copy KDL::Trajectory objects directly, must use it's clone method
* Get rid of ENABLE_JOINT_STATE_PUBLISHER, its deprecated, pre-ros2-controls
* Add Trajectory::offset_time(ts) to move all keyframes by ts amount. Then Trajectory::rebase_time(ts)
    that uses offset_time and the min keyframe ts - basically to restart preview animation from now.
* get rid of Limb::updateIK() in favor of Kinematics class that caches IK q variables





# we use position_trajectory_controller for position control, but forward_position_controller for effort control
ros2 control load_start_controller joint_state_controller
ros2 control load_start_controller position_trajectory_controller
ros2 control load_start_controller forward_position_controller

# Set max torque
# Joint order J11,J12,J13,J14,J15,J16,J17,J18,J19,J21,J22,J23,J24,J25,J26,J27,J28,J29
              [10, 10, 10, 10, 10, 10,  1,  1,  1, 10, 10, 10, 10, 10, 10, 1, 1, 1]
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]"
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]"
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]"
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10]"
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [10, 10, 10, 10, 10, 10,  1,  1,  1, 10, 10, 10, 10, 10, 10, 1, 1, 1]"

-- enable motors on test rig
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]"
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10]"

-- set position on test rig, must send to all joints or PJC bitches
ros2 topic pub --once /position_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names: [J0, J5]
points:
  - positions: [1.0, 1.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start:
      sec: 0
      nanosec: 0"



## ROS World
- Reflexxes Type II in Velocity mode works well for smoothing streaming commands
- Andy Zelenak's Admittance control


Yacro:
<part sku="acme:abc">
   -- this pushes the given sku onto the assembly stack
   -- now these parameters can be accessed using an expression
</part>
* Need a way to set current scope to a prop tables
     - should be easy, use symbols = Table(part_table) and then eval_all() on children.
* Need indirect addressing of properties
* Can we store Node ptrs or macros in the prop table?
* macro needs to be able to reference a symbol table (part) and build from a template,
  or refer to a macro (part) in the vendor package pass it the yml table.
* <xacro:package> handler must load the yaml, load the xacro includes and add stuff
  to the root symbol table (under assemblies?).
* dependencies can be specified in the vendor package.xml, so that deps are loaded automatically
     ...such as the vendor DB package, also duplicate deps are loaded only once.
     export tag is free-form, but tagName should match the package intended to use it, so <xacro>
* Assembly skus. Virtual parts that are simply a combo of other parts
* Parts can have options (such as horns included or not)

* base erector-set core should have a macro for generating all the parts using a macro by name
     - can we give a macro by name?
     - or just have them copy/paste a template for now?
     - or create a <xacro:generate> that generates a given macro by name for each item in a dict?
     	- have it include an optional if-unless expression

* Attachment Properties
  - PAIRED. forming a part attachment requires the child and parent attachment name 
     - so how do we add parent to this scope?
     - need a slight variation of <using>, a way to bring in another point of the symbol tree into 
       a local variable, like a load.
     - CHAINING. An attachment can actually be a chain, each member of the chain is an attachment
                 reference with an in and out. So for example:
                      servo-horn => LSS-DH :: LSS-DH => ABS-09
                 The horn => LSS-DH forms a fixed unit, so does the LSS-DH => ABS-09, the joint
                 would be at the :: point.
                 Each attachment transform adds onto the previous attachment output.
     - Isn't chaining attachments just links then? So we need a light-weight link, and one that
       can be combined as a fixed joint? maybe add an attribute that says 'merge' and/or add an
       attribute to the link to classify the part as a "attachment"....maybe also just have a 
       class attrib.
  - CLASS. Classifies links into one or more groups to describe it's purpose.
  - ROTATION. A rotation or degrees parameter, but this requires alignment of screw-mounting holes...how do we?
  - TYPE. Only compatible attachments can form a pair
     - each attachment has a list of compatible types (both as a child or parent?)
     - or child attachment is a single type, parent supports one or more types...but can an 
        attachment point be used as a child or parent?


      <joint name="J29">
        <xacro:using property="attachments.${attachment}">
            <origin
              xyz="${xyz}"
              rpy="${rpy}" />
            <axis
              xyz="${axis}" />
        </xacro:using>
      </joint>


Issues with xacro code:
* Mix of Table, YamlDictWrapper and YamlListWrapper - why not just load the yaml recursively into Table's?
* YamlDictWrapper __getitem__ returns a copy, so you cant modify/add to yaml wrappers
* Symbols in general is a bit wierd, dotted access, but members are not homogeneous in their implementation
* Could use a refactor, placing stuff like checkattr() into xmlutils, moving Table into it's own file. cli, color is good.
* (maybe) line 764 in handle_macro_call() creates a new scope, then doesnt use it?
* xacro doesnt like dashes in name due to python evaluation of - (minus)


Bottom line features:
(1) Scope to a specific point in the symbols table
(2) need a <generate> function
(3) fix <property> to allow indirect loading of parent part
(4) Package loader of includes/yamls with deps
     - maybe lazy load all these includes?
(5) Maybe a <xacro:import python="re" /> could import a python package into the eval() namespace
     - easy, now add this feature to the xacro package as well, now the erector_lss will import python expression tools
(6) <xacro:call method="pythonfunc" self="" arg1="" arg2="" />

How to start the controllers after launch
   - I setup the yaml according to rrbot, but they didnt appear in "ros2 control list_controllers" after launch
   - I'm using "ros2 control load_start_controller joint_state_controller & position_trajectory_controller" to load activate them
   - Should one of our nodes call services on CM to bring-up?
How do we pull state and send commands:
   - Once I load joint_state_controller, I get /dynamic_joint_states and /joint_states for reading state
   - Once I load position_trajectory_controller, I get /position_trajectory_controller/joint_trajectory for sending trajectories   


    

ROS Answers:
* URI paths to models: https://answers.gazebosim.org//question/6568/uri-paths-to-packages-in-the-sdf-model-file/
* Pass parameters to xacro on command line - https://answers.ros.org/question/38956/pass-parameters-to-xacro-in-launch-file/
* Creating a Constant Force Spring - https://answers.gazebosim.org//question/20335/creating-a-constant-force-spring/
* ROS2+Gazebo What options do we have for joint control? - ttps://answers.ros.org/question/362409/ros2gazebo-what-options-do-we-have-for-joint-control/

Github:
* Alternative for libgazebo_ros_control.so in ROS1 - https://github.com/ros-simulation/gazebo_ros_pkgs/issues/946
* Chapulina Gazebo Plugins - https://github.com/chapulina/gazebo_plugins
* Roadmap for ros2_control
* Port gazebo_ros_pkgs to ROS2 - https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512  (my post)
* gazebo_ros_pkgs - https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_ros


Found Dynamixels ros_controls Hardware Interface
http://www.resibots.eu/dynamixel_control_hw/reference/hardware_interface.html



Relevant Issues:
* ROS1 Problems with Mode switching - https://github.com/ros-controls/ros_control/issues/193


Questions:

* Shouldnt reset() load prev value from position state handle? - I guess it does in next iteration



Pure Components Interface - https://github.com/ros-controls/ros2_control/pull/171
Add ROS2 Control Managers - https://github.com/ros-controls/ros2_control/pull/147
Add Controller Manager Services - https://github.com/ros-controls/ros2_control/pull/139
ROS2 Control Components = https://github.com/ros-controls/ros2_control/pull/140
ROS Controls Transmission Handling Kanban - https://github.com/orgs/ros-controls/projects/4
Updated with recent ros2_control changes - https://github.com/ros-simulation/gazebo_ros2_control/pull/34
Dynamixel ROS Controls Hardware interface - http://www.resibots.eu/dynamixel_control_hw/reference/hardware_interface.html

Loading a controller should only be allowed after contriller_manager has been configured
https://github.com/ros-controls/ros2_control/issues/152

ros2/urdf / Replace fprintf with ROS2 logging
https://github.com/ros2/urdf/issues/16

loco-2d / crocoddyl Frequently commited errors by new users
https://github.com/loco-3d/crocoddyl/issues/874

icub-gazebo / enabling self collision
https://github.com/robotology/icub-gazebo/issues/21

Gazebo Collide Bitmasks
http://gazebosim.org/tutorials?tut=collide_bitmask&cat=physics


ROS Pub/Sub
* Calling a service from a subscription event - https://answers.ros.org/question/302037/ros2-how-to-call-a-service-from-the-callback-function-of-a-subscriber/
* Execution and callbacks - http://docs.ros2.org/latest/api/rclpy/api/execution_and_callbacks.html

Xacro
* http://wiki.ros.org/xacro

Gazebo
* Using URDF in Gazebo - http://gazebosim.org/tutorials/?tut=ros_urdf
* SDF Spec - http://sdformat.org/spec?ver=1.7&elem=link
* Simulation of Humanoids in Gazebo (not good news) - https://answers.ros.org/question/33755/example-of-how-to-simulate-humanoid-biped-using-rosgazebo/
* Finger contact ERP, CFM, Kp and Kd - https://answers.gazebosim.org//question/9371/finger-contact-erp-cfm-kp-and-kd/
* Parameters for ODE explained - http://www.gazebosim.org/tutorials?tut=physics_params&cat=physics

ROS Control
* Safety Limits - https://wiki.ros.org/pr2_controller_manager/safety_limits

Lifecycle Launch File
* How to correctly create a Lifecycle launch file [ROS2 LAUNCH]
  https://answers.ros.org/question/304370/ros2-launch-how-to-correctly-create-a-lifecycle-launch-file/
* Make it easier to activate lifecycle nodes started through ros2 run
  https://github.com/ros-planning/navigation2/issues/1240
* How to launch pipeline of two lifecycle nodes in ROS 2 Crystal in a concise script?
  https://answers.ros.org/question/315050/how-to-launch-pipeline-of-two-lifecycle-nodes-in-ros-2-crystal-in-a-concise-script/
* ROS2 launch: required nodes (exiting when another node exits)
  https://kyrofa.com/posts/ros2-launch-required-nodes
* Architecture of Launch
  https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst
* Launch API execute_process (how to capture/pipe IO)
  https://github.com/ros2/launch/blob/foxy/launch/launch/actions/execute_process.py
* Any way to see IO sooner with OnProcessIO event in ROS2 Launch Description?
  https://answers.ros.org/question/312737/any-way-to-see-io-sooner-with-onprocessio-event-in-ros2-launch-description/


Robot Localization & IMU
* Not publishing odom transform - https://answers.ros.org/question/226085/robot_localization-not-publishing-transform-to-odom/
* Gazebo p3d localization plugin - https://answers.ros.org/question/222033/how-do-i-publish-gazebo-position-of-a-robot-model-on-odometry-topic/
* Gazebo Ros2 IMU migration - https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-IMU-Sensors
* Noise parameters for IMU - https://github.com/ros-simulation/gazebo_ros_pkgs/pull/801/commits/7c60a31ec2455cd08a5266ae8f2bd22b7f96f0ab
* RL using only IMU drifts, checkout his footprint frame location! - https://answers.ros.org/question/301144/robot_localization-estimate-using-only-imu-drifts-for-a-stationary-robot-gazebo-model/


Gazebo unload issue
* Segv on Cntrl-C - https://github.com/ros2/rclcpp/pull/505



colcon test --packages-select joint_limit_interface
colcon test --event-handler console_direct+ --packages-select rclcpp_lifecycle --merge-install
ament_uncrustify --exclude cmake-build-debug
ament_cpplint 


/home/guru/src/ros/install/lib/controller_manager/ros2_control_node --ros-args --log-level debug --params-file /tmp/launch_params_ngjlzb5x --params-file /home/guru/src/lss-humanoid/ros2/humanoid/install/lss_hardware/share/lss_hardware/config/lss_hw_example.yaml
ln -s /home/guru/src/lss-humanoid/ros2/humanoid/src/lss_hardware/cmake-build-debug/liblss_hardware.so liblss_hardware.so
** but if you edit configuration and add Install to pre-launch steps then it will install everything and debug works

ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py




tunnel and reem-c TALOS, Gorden Chen at TUM



SEGV in control_toolbox::Pid::getGains() line 224
	* seems like an illegal access to gains_buffer trying to get gains using readFromRT(). I assume a real-time double-buff
	* gains_buffer decl is realtime_tools::RealtimeBuffer<Gains>
	* https://github.com/ros-controls/control_toolbox/blob/melodic-devel/src/pid.cpp
	* why does PIDs declare_parameter in default_robot_hw_sim lead to PID getGains? Is declare_parameter executing change-parameter hooks?
	





