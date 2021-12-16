//
// Created by guru on 3/27/20.
//

#ifndef LSS_HUMANOID_ROBOTSTATE_H
#define LSS_HUMANOID_ROBOTSTATE_H

#include "types.h"
#include "contact.h"
#include "limb.h"


namespace robotik {

class Model;

typedef enum {
    MeasuredState,
    TargetState,
    TrajectoryState
} StateType;

//todo: If we made whole state an object with each seperate state object a field member, then we could have it set or not set. Possibly a shared_ptr (rather not) or unique_ptr.

class StateStamp {
public:
    rclcpp::Time updated;
    ParameterMask states;

    StateStamp() : states(0) {}
    StateStamp(rclcpp::Time _updated, ParameterMask _states) : updated(_updated), states(_states) {}

    ///@brief Returns true if given state variables were updated since the given timestamp
    bool isFresh(rclcpp::Time since, ParameterMask match) const;

    //inline bool operator>(StateStamp& rhs) const { return isFresh(rhs.updated, rhs.states); }
};

class StateFrame {
public:
    StateStamp frameStamp;

    ///@brief The state is visualized relative to this frame, usually the odom frame.
    std::string relativeFrameName;

    ///@brief any state coordinates are local to this transform from relativeFrameName
    /// Generally, the transform should be a translation from the odom frame so as to represent a target for the robot to
    /// reach from it's current position. Furthermore, the frame's rotational component should be the identity matrix so
    /// that it coincides with the odom frame reference.
    KDL::Frame transform;
};

class JointState {
public:
    JointState() = default;
    explicit JointState(Model& model);
    JointState(const JointState& copy) = default;

    virtual ~JointState();

    JointState& operator=(const JointState& copy) = default;

    rclcpp::Time lastJointStateUpdate;

    ///@brief Joint names
    /// Be aware that for every joint array it must either have zero elements (if unused) or the same number of elements
    /// as their are joint names. Use the addJoint() method to add a joint by name while also resizing any active
    /// parameter collections.
    Names joints;

    std::vector<bool> joints_updated;

    ///@brief standard joint state
    KDL::JntArray position;
    KDL::JntArray velocity;
    KDL::JntArray acceleration;

    /// @brief amount of torque being supplied for each joint
    KDL::JntArray effort;

    ///@brief The force of gravity acting on each joint
    /// Gravity vectors are expressed as torques on each joint.
    KDL::JntArray internalForces;

    ///@brief Compensate for external forces acting on a joint.
    /// A Wrench is the 6x1 matrix that represents a force on a Frame using a 3D translational force Vector force and
    /// a 3D moment Vector torque:
    KDL::WrenchMap externalForces;

    // the cartesian coordinates of joint axis (where external forces act upon)
    // we calculate this locally in Visual where we need it (already available there), so hopefully we dont need elsewhere
    // Frames jointTF;

    virtual void zero();

    void clear_updated_joints();

    ///@brief Find the ordinal index of a joint by name
    ///@returns Joint ordinal index or -1 if not found
    ssize_t findJoint(const char* name) const;

    ///@brief Find the ordinal index of a joint by name
    ///@returns Joint ordinal index or -1 if not found
    inline ssize_t findJoint(const std::string& name) const {
        return findJoint(name.c_str());
    }

    ///@brief Find the ordinal index of a joint by name or add a new joint if it doesnt exist
    ///@returns Joint ordinal index (existing or newly added)
    ssize_t addJoint(const char* name, unsigned int allocateParameterMask = DEFAULT_PARAMETERS);

    ///@brief Find the ordinal index of a joint by name or add a new joint if it doesnt exist
    ///@returns Joint ordinal index (existing or newly added)
    inline ssize_t addJoint(const std::string& name, unsigned int allocateParameterMask = DEFAULT_PARAMETERS) {
        return addJoint(name.c_str(), allocateParameterMask);
    }

    ///@brief Allocate required memory for the given parameters
    virtual void alloc(unsigned int allocateParameterMask);

    std::string toXmlString(std::string&& stateName, std::string&& groupName, KDL::Frame root_frame) const;
};


class LimbState {
public:
    ///@brief Capture the intention of each limb
    std::vector<Limb::Request> limbs;

    LimbState();

    ///@brief Construct the list of limbs with default states using the collection of limb definitions in the given model.
    /// Non-supportive limbs such as arms are set to limp. Legs are set to be supportive but are cleared of any
    /// target TF.
    explicit LimbState(const Model& model);

    ///@brief Keeps the list of limbs but sets the intention of each limb to defaults
    /// This essentially reverts the LimbState to the model-limbs constructor state. Non-supportive limbs such as arms
    /// are set to limp. Legs are set to be supportive but are cleared of an target TF.
    virtual void zero();
};


class SegmentState {
public:
    SegmentState();

    virtual ~SegmentState();

    rclcpp::Time lastSegmentStateUpdate;

    ///@brief computed mass of the robot
    double mass;

    ///@brief Robot Center of Mass (aka Center of Gravity)
    KDL::Vector CoM;

    ///@brief The cartesian coordinate transform of each segment
    /// The position and rotation of each segment is with respect to the frame given in relativeFrameName so long as
    /// it is a child segment of the robot's base_link. Otherwise, any transforms outside the robot, such as odom or
    /// world have no guarantee or restriction on what frame they relate to.
    // todo: we should probably track 'invalidated' segments and therefor compute IK only when required on invalidated limbs/chains
    SegmentTransforms tf;

    ///@brief Set all state variables to zero or their identity value
    ///This does not clear or release any memory. Mass, center of mass and pressure is set to 0, imu and
    /// all segment transforms are set to identity value.
    virtual void zero();

    ///@brief Get a segment and it's transform by name
    ///@returns frame associated with the given link name, or throws SEGMENT_NOT_FOUND if not found
    const KDL::Frame& findTF(const std::string& _name) const;

    ///@brief Get a segment and it's transform by name
    ///This function performs the same function as findTF(name) but does not throw if the transform doesnt exist.
    ///@param frame out the existing frame transform is returned in this parameter
    ///@returns true if transform exists
    bool findTF(const std::string& _name, KDL::Frame& frame) const;

    ///@brief Get a segments position and rotation from the given reference frame
    /// can't actually do this since it requires knowledge of the model
    //bool findTF(const std::string& _name, const std::string& _referenceFrame, KDL::Frame& frame) const;

    ///@brief Transform all segments by the given transform
    /// Each segment is multiplied by the transformation vector and rotation.
    // void transform(const KDL::Frame& transformOp);
};


class SupportState {
    // todo: would be great if this was more segment-in-contact instead of limb-in-contact, robot may contact in the knee or elbow too
    //    this is different from using an elbow as an end-effector, for that the user would need to create a temp end-effector (limb) and
    //    command it. It's still ok to add references to limbs though just for convenience but algorithms should be ok without it.
protected:
    double _baseHeightFromFloor;

public:
    SupportState();

    rclcpp::Time lastSupportStateUpdate;

    ///@brief List of (ground) support contacts
    /// These are contacts between our end-effectors and the world that we've either sensed or assumed.
    std::vector<Contact> contacts;

    ///@brief Indicates which limbs are in contact with the ground
    /// This information is also contained in the contacts array but is available here quick testing of ground contact
    /// against the limbs[] array.
    //std::vector<boolean> limbsInContact;      // suggestion only, not supported yet

    ///@brief true if the robot's Center-of-Mass is within the support polygon
    bool inSupport;

    ///@brief The convex hull representing the area of stability if containing the robot's center of mass.
    std::vector<KDL::Vector> supportPolygon;

    ///@brief Estimated Center of Pressure
    /// Center of pressure is the estimated middle point between all limbs (usually feet) in contact.
    // todo: an arm effector could also be used as a balancing contact point if it is holding onto a wall, chair, floor, etc.
    KDL::Vector CoP;

    ///@brief A measure of how balanced the robot is in this moment
    /// This metric can be used to train walking, standing or other motion sequences and represents how stable the robot
    /// is.
    /// todo: Joint velocity/acc should be taken into account against gravity to determine if robot is in dynamic
    ///       balance even if CoP is not currently within the support polygon.
    double balanceHealth();

    ///@brief clear values but don't affect size of any arrays
    /// This clears parameter collections for the entire robot state. It does not affect the joint names collection.
    virtual void zero();

    ///@brief tests if given point is within the support polygon
    ///@returns true if point is above the support polygon
    bool pointInSupport(const KDL::Vector& point) const;

    // todo: make a method that returns the closest point on a line that is inside the support polygon

    ///@brief Return the position of the base link frame from the assumed floor
    /// This height is already built into the state transform value so the robot always visualizes standing on the floor.
    [[nodiscard]] inline double baseHeightFromFloor() const { return _baseHeightFromFloor; }
    inline void baseHeightFromFloor(double height_in_Z) { _baseHeightFromFloor = height_in_Z; }
};


class JointAndSegmentState : public JointState, public SegmentState
{
public:
    inline JointAndSegmentState() {}
    explicit inline JointAndSegmentState(Model& model) : JointState(model) {}
};


class State : public StateFrame, public JointAndSegmentState, public LimbState, public SupportState {
public:
    using SharedPtr = std::shared_ptr<State>;

    State();
    State(const State& copy);
    explicit State(Model& model);


    ///@brief clear values but don't affect size of any arrays
    /// This clears parameter collections for the entire robot state. It does not affect the joint names collection.
    void zero() override;

    ///@brief Allocate required memory for the given parameters
    void alloc(unsigned int allocateParameterMask) override;

    StateType type;
};

} // ns::robot

#endif //LSS_HUMANOID_ROBOTSTATE_H
