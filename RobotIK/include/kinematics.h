//
// Created by guru on 11/14/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_KINEMATICS_H
#define HUMANOID_DYNAMIC_MODEL_KINEMATICS_H

#include "model.h"
#include "state.h"


namespace robotik {

class LimbKinematics {
public:
    // when an effector is moved it must go through many computation
    // steps to be fully valid again
    typedef enum {
        INVALID,        // limb is pending a new IK solution
        IK_SOLVED,      // IK was computed
        JOINTS,         // joints were udpated with IK results
        SEGMENTS,       // segments were updated from joint positions (forward kinematics)
        DYNAMICS,       // torque dynamics were computed/estimated
        VALID
    } SolutionState;

    explicit LimbKinematics(Limb::SharedPtr && limb);

    LimbKinematics(const LimbKinematics& copy);

    Limb::SharedPtr limb() const { return limb_; }

    /// compute IK to move the effector to the new position,
    /// updates joint angles and segment positions within the state.
    int moveEffector(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose);

    /// compute IK to move the effector to the new position and return the solved effector state.
    /// The returned effector state may be different then the goal state if the goal state cannot be reached
    /// updates joint angles and segment positions within the state.
    int moveEffector(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose, KDL::Frame& effector_out);

    /// Internal: compute the Inverse Kinematics to reach a new effector position and return
    /// without writing back to the state. This is used by moveEffector()
    int computeIK(JointAndSegmentState& state, const KDL::Frame& new_effector_pose);

    /// Internal: apple the joint angles in state to the segment transforms by
    /// recursively computing the forward kinematics.
    ///    Applies Q, Q' and Q" => Joint Angles => Segments TF
    /// This is already done inside moveEffector but if you manually move joint angles you can use this
    /// to update the segment states. This also does the work of updateJoints()
    bool updateJointsAndSegments(JointAndSegmentState& state, KDL::Frame* effector_out = nullptr);

    /// Internal: compute forward kinematics to update segment positions based on the joint angles.
    ///    Applies Q, Q' and Q" => state Position, Velocity and Acceleration for joints
    /// This is already done inside moveEffector but if you manually update the 'q' IK variables
    /// you can use this to apply the changes to state.
    void updateJoints(JointAndSegmentState& state);

    /// Update any remaining limb state
    bool updateState(State& state);

    /// Internal: configure the kinematics and allocate necessary memory using joint information
    /// from the given state. As long as joint schema or joint order doesn't change you don't need
    /// to call this again.
    bool configure(JointAndSegmentState& state);

    // access to internal joint states
    //KDL::JntArray& q()
    //KDL::JntArray& guess()

    inline void invalidate() { state_ = INVALID; }

    inline double mass() const { return mass_; }
    inline KDL::Vector CoM() const { return center_of_mass_; }

protected:
    SolutionState state_;

    Limb::SharedPtr limb_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> lma_solver_;

    // KDL inverse kinematic input variables
    KDL::JntArray q_;
    KDL::JntArray qdot_;
    KDL::JntArray qdotdot_;

    // KDL returns the solved joint kinematics into this array
    KDL::JntArray q_out_;

    // indexes our limb joint positions into the state Joint object
    std::vector<size_t> joint_position_index_;

    // only recheck joint schema and joint order when we receive a different
    // state reference
    JointAndSegmentState* configured_state;

    // computed limb properties
    double mass_;
    KDL::Vector center_of_mass_;

    inline bool requires_configure(JointAndSegmentState& state) {
        return configured_state== nullptr || configured_state != &state;
    }
};


class Kinematics
{
public:
    static const int SUCCESS = KDL::SolverI::E_NOERROR;

    /// default constructor defers activation of kinematics for later
    Kinematics();

    /// construct and activate kinematics on the given model
    explicit Kinematics(Model::SharedPtr model);

    /// activate kinematics on the given model
    void activate(Model::SharedPtr model);

    /// reset kinematics state
    void deactivate();

    /// compute IK to move the effector to the new position,
    /// updates joint angles and segment positions within the state.
    int moveEffector(JointAndSegmentState& state, std::string limb, const KDL::Frame& effector_goal_pose);

    int moveEffector(JointAndSegmentState& state, std::string limb, const KDL::Frame& effector_goal_pose, KDL::Frame& effector_out);

    int moveBase(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose);

    int moveBase(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose, KDL::Frame& base_out);

    /// After limbs have been updated, update whole robot state
    bool updateState(State& state);

private:
    Model::SharedPtr model_;
    std::map<std::string, LimbKinematics> limbs_;
    std::set<std::string> limbs_first_joint;

    // computed properties of non-limb parts like base
    double mass_;
    KDL::Vector center_of_mass_;

    ///@brief internal recursive TF, CoM and CoP computation on KDL segments.
    bool compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                        const KDL::Frame& tf, State& state);
};


} // ns:robotik

#endif //HUMANOID_DYNAMIC_MODEL_KINEMATICS_H
