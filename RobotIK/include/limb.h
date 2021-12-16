//
// Created by guru on 1/1/20.
//

#pragma once

#include "types.h"
#include "imports.h"
#include "friction.h"


namespace robotik {

class JointAndSegmentState;
class JointState;

class Limb {
public:
    using SharedPtr = std::shared_ptr<Limb>;
    using Marker = visualization_msgs::msg::Marker;

    ///@brief A flag indicating what this limb is intended for
    /// Legs will be used primary for robot support and locomotion. Arms for object manipulation or if free may also be
    /// used for dynamic CoM balance support or rigid-contact robot support.
    /// You can also change this flag at run-time to change the behaviour of a limb if your robot structure is geared
    /// for it.
    typedef enum {
        Generic,
        Leg,
        Arm
    } DynamicModelType;

    class Options {
    public:
        std::shared_ptr<KDL::Tree> tree;
        KDL::Vector gravity;
        DynamicModelType model;
        std::string from_link, to_link;
    };

    ///@brief Current behavior mode this limb is in
    enum Mode {
        Limp,                   /// end-effector servos should go limp
        Holding,                /// this end-effector should try to hold the given position
        //BalanceSupport,         /// use this end-effector to remain stable/balanced and support the robot
        //Stepping,               /// this end-effector should perform a stepping motion to improve balance stability
        Seeking,                /// this end-effector should follow the given targetTF value
        Manipulating            /// user is currently manipulating this limb and has control
    };

    ///@brief Indicates how a limb should be used
    class Request {
    public:
        ///@brief What control mode this end-effector is currently in
        Mode mode;

        ///@brief Indicates if Limb is a leg, arm or other generic limb.
        /// This is copied from the model limb details.
        DynamicModelType limbType;

        ///@brief true if this limb can be used to support the robot
        /// This flag indicates if this limb can be recruited to support the robot. If false, the modes BalanceSupport,
        /// and Stepping would be invalid.
        bool supportive;

        ///@brief Where we want the end effector to go
        /// This frame is relative to the limb base frame (usually the base link). We may not be able to take a direct
        /// flight, that is up to the trajectory planner and controller.
        // todo: Trajectory now sets this value, so should balance/etc, so we can have this hold multiple values possibly
        //       over time and have it mix down based on weights just before updateIK happens.
        KDL::Frame targetTF;

        ///@brief The name of the algorithm controlling this limb at the moment
        /// todo: the controller, if any, should be an outside ROS node that we communicate with. See Subsumption Architecture?
        //Controller controller;

        Request();
        Request(DynamicModelType _limbType, Mode _mode, bool _supportive);

        ///@brief Command the end-effector to go to the given position
        void seek(KDL::Frame target);
    };

    explicit Limb(const Options& options);

    inline unsigned int getNrOfJoints() const { return chain->getNrOfJoints(); }

    ///@brief load limb data and allocate memory for internal variables
    bool on_activate();

    ///@brief deallocate resources for the limb
    bool on_deactivate();

    int updateIK(JointAndSegmentState& state, KDL::Frame new_effector_pose);
    NamedJointArray computePose(const JointState& state, KDL::Frame base);

    inline KDL::Chain* get_chain() { return chain.get(); }

    KDL::Frame computeTFfromBase(const JointState& state);

    Options options_;

    std::vector<std::string> joint_names;
    std::vector<std::string> segment_names;

    ///@brief The friction between this limb and the ground/floor surface
    Friction friction;

    ///@brief the direction from the robot base to which this limb is attached
    /// This is a directional vector rooted at the base to the limb's base attachment point. It is used to determine
    /// normal support stance where legs are supporting equi-distant from the base.
    KDL::Vector direction;

    ///@brief the untranslated support polygon of this end-effector
    /// For legs this is  effectively the supprt rectangle (usually) of the sole of the foot. It is determined by loading
    /// the mesh vertices and performing 'convex hull' analysis on the points to find the enclosing polygon. It prefers
    /// the collision mesh but will fallback to visual mesh if required. For arms this polygon is not yet used but could
    /// also be used to determine point of contact.
    std::vector<KDL::Vector> supportPolygon;

    ///@brief Load the mesh model for the limb and compute a support polygon for contact detection
    void loadSupportPolygon(urdf::ModelInterfaceSharedPtr urdf_model_);

protected:
    ///@brief Orokos API chain for this limb
    std::unique_ptr<KDL::Chain> chain;

    /// IK variables
    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qdotdot;
};

} // ns::robotik
