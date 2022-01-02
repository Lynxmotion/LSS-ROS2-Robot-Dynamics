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
class Model;

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
        BalanceSupport,         /// use this end-effector to remain stable/balanced and support the robot
        Stepping,               /// this end-effector should perform a stepping motion to improve balance stability
        Seeking,                /// this end-effector should follow the given targetTF value
        Manipulating            /// user is currently manipulating this limb and has control
    };

    ///@brief Indicates how a limb should be used
    class State {
    public:
        using SharedPtr = std::shared_ptr<State>;

        Limb::SharedPtr model;

        ///@brief What control mode this end-effector is currently in
        Mode mode;

        ///@brief Indicates if Limb is a leg, arm or other generic limb.
        /// This is copied from the model limb details.
        DynamicModelType limbType;

        ///@brief true if this limb can be used to support the robot
        /// This flag indicates if this limb can be recruited to support the robot. If false, the modes BalanceSupport,
        /// and Stepping would be invalid.
        bool supportive;

        ///@brief true if this limb is currently being used to support the robot
        bool supporting;

        ///@brief Current position of the end effector
        /// This frame is relative to the limb base frame (usually the base link).
        KDL::Frame position;

        ///@brief Current velocity of this end effector if it is moving
        KDL::Twist velocity;

        ///@brief Target trajectory position of the end effector
        /// This frame is relative to the limb base frame (usually the base link). This position is updated by
        /// active trajectory actions.
        KDL::Frame target;

        State();
        explicit State(Limb::SharedPtr model, Mode _mode = Limp);
        explicit State(Limb::SharedPtr model, Mode _mode, bool _supportive);
    };

    explicit Limb(const Options& options);

    inline unsigned int getNrOfJoints() const { return chain->getNrOfJoints(); }

    ///@brief load limb data and allocate memory for internal variables
    bool on_activate();

    ///@brief deallocate resources for the limb
    bool on_deactivate();

    int updateIK(JointAndSegmentState& state, KDL::Frame new_effector_pose, bool relative_to_odom = false);
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

class LimbModels : public std::vector<Limb::SharedPtr>
{
public:
};

class Limbs : public std::vector<Limb::State>
{
public:
    using std::vector<Limb::State>::operator[];

    inline Limb::State& operator[](const std::string& s) {
        auto itr = std::find_if(begin(), end(), [&s](const Limb::State& l) { return l.model->options_.to_link == s; });
        if(itr == end())
            throw std::runtime_error("no limb by the name of " + s + " exists");
        return *itr;
    }

    inline const Limb::State& operator[](const std::string& s) const {
        auto itr = find_if(begin(), end(), [&s](const Limb::State& l) { return l.model->options_.to_link == s; });
        if(itr == end())
            throw std::runtime_error("no limb by the name of " + s + " exists");
        return *itr;
    }

    void zero();

    static Limbs fromModel(const Model& model);
};


} // ns::robotik
