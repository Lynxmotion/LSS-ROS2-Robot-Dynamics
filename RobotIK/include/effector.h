//
// Created by guru on 1/7/22.
//

#ifndef ROBOT_DYNAMICS_EFFECTOR_H
#define ROBOT_DYNAMICS_EFFECTOR_H

namespace robotik {

class JointAndSegmentState;
class JointState;
class Model;

class Effector {
public:
    using SharedPtr = std::shared_ptr<Effector>;

    ///@brief Current behavior mode this limb is in
    enum Mode {
        Limp,                   /// end-effector servos should go limp
        Holding,                /// this end-effector should try to hold the given position
        BalanceSupport,         /// use this end-effector to remain stable/balanced and support the robot
        Stepping,               /// this end-effector should perform a stepping motion to improve balance stability
        Seeking,                /// this end-effector should follow the given targetTF value
        Manipulating            /// user is currently manipulating this limb and has control
    };

    ///@brief Indicates how a limb should be used
    class State {
    public:
        ///@brief What control mode this end-effector is currently in
        Mode mode;

        ///@brief Current position of the end effector
        /// This frame is relative to the limb base frame (usually the base link).
        KDL::Frame position;

        ///@brief Current velocity of this end effector if it is moving
        KDL::Twist velocity;

        ///@brief Target trajectory position of the end effector
        /// This frame is relative to the limb base frame (usually the base link). This position is updated by
        /// active trajectory actions.
        KDL::Frame target;

        inline explicit State(Mode _mode = Limp)
        : mode(_mode)
        {}
    };

};

} //ns:robotik
#endif //ROBOT_DYNAMICS_EFFECTOR_H
