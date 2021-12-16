//
// Created by guru on 3/27/20.
//

#include <state.h>
#include <model.h>
#include <exception.h>


namespace robotik {


bool StateStamp::isFresh(rclcpp::Time, ParameterMask) const
{
    // todo: implement isFresh for testing if state data needs updating
    return true;
}

JointState::JointState(Model& model) {
    joints = model.getJoints();
}

JointState::~JointState()
{
}

State::State()
    : type(MeasuredState)
{
}

State::State(const State& copy)
    : StateFrame(copy), JointAndSegmentState(copy), LimbState(copy), SupportState(copy), type(copy.type)
{
    // clearing updated fields since we've taken a copy we expect all
    // joints and segments return to unupdated state
    clear_updated_joints();
}

State::State(Model& model)
    : JointAndSegmentState(model), LimbState(model), type(MeasuredState)
{
}


ssize_t JointState::findJoint(const char* _name) const {
    auto itr = std::find(joints.begin(), joints.end(), _name);
    return (itr != joints.end())
           ? itr - joints.begin()
           : -1;
}

ssize_t JointState::addJoint(const char* _name, unsigned int allocateParameters) {
    auto itr = std::find(joints.begin(), joints.end(), _name);
    if (itr == joints.end())
        itr = joints.insert(joints.end(), _name);
    alloc(allocateParameters);      // resize all active or requested parameters
    return itr - joints.begin();
}


void JointState::alloc(unsigned int allocateParameterMask) {
    // now ensure the requested parameter collections have space allocated for this joint
    size_t requiredSize = joints.size();
    joints_updated.resize(requiredSize, false);
    if(allocateParameterMask & POSITION && joints.size() <= requiredSize)
        position.resize(requiredSize);
    if(allocateParameterMask & VELOCITY && velocity.rows() <= requiredSize)
        velocity.resize(requiredSize);
    if(allocateParameterMask & ACCELERATION && acceleration.rows() <= requiredSize)
        acceleration.resize(requiredSize);
    if(allocateParameterMask & EFFORT && effort.rows() <= requiredSize)
        effort.resize(requiredSize);
    if(allocateParameterMask & INTERNAL_FORCE && internalForces.rows() <= requiredSize)
        internalForces.resize(requiredSize);
    // EXTERNAL_FORCE && externalForces is dynamically sized
}


std::string JointState::toXmlString(std::string&& stateName, std::string&& groupName, KDL::Frame root_frame) const {
    std::ostringstream ss;

    // xml group header
    ss << "<group_state group=\"" << groupName << "\" name=\"" << stateName << "\">" << std::endl;

    double x, y, z, w;
    root_frame.M.GetQuaternion(x, y, z, w);
    ss << "    <joint name=\"root_joint\" value=\""
       << root_frame.p.x() << " " << root_frame.p.y() << " " << root_frame.p.z() << " "
       << x << " " << y << " " << z << " " << w << "\" />"
       << std::endl;

    for(size_t j=0; j < joints.size(); j++) {
        ss << "    <joint name=\"" << joints[j] << "\" value=\"" << position(j) << "\" />" << std::endl;
    }
    ss << "</group_state>" << std::endl;
    return ss.str();
}

void State::zero() {
    JointState::zero();
    SegmentState::zero();
    LimbState::zero();
    SupportState::zero();
}

void State::alloc(unsigned int allocateParameterMask) {
    JointState::alloc(allocateParameterMask);
}

void JointState::zero() {
    KDL::SetToZero(position);
    KDL::SetToZero(velocity);
    KDL::SetToZero(acceleration);
    KDL::SetToZero(effort);
    KDL::SetToZero(internalForces);

    clear_updated_joints();

    // clear wrenches
    externalForces.clear();
}

void JointState::clear_updated_joints()
{
    for(int i=0, _i=joints_updated.size(); i < _i; i++)
        joints_updated[i] = false;
}

SegmentState::~SegmentState()
{
}


SegmentState::SegmentState()
        : mass(0)
{

}

void SegmentState::zero() {
    mass = 0;
    CoM.Zero();

    // clear transforms
    KDL::Rotation identity = KDL::Rotation::Identity();
    for(auto& w: tf) {
        w.second.p.Zero();
        w.second.M = identity;
    }
}

const KDL::Frame& SegmentState::findTF(const std::string& _name) const {
    auto segitr = tf.find(_name);
    if(segitr == tf.end())
        throw Exception::SegmentNotFound(_name);
    return segitr->second;
}

bool SegmentState::findTF(const std::string& _name, KDL::Frame& frame) const {
    auto segitr = tf.find(_name);
    if (segitr != tf.end()) {
        frame = segitr->second;
        return true;
    }
    return false;
}


LimbState::LimbState()
{
}

LimbState::LimbState(const Model& model) {
    updateFromModel(model);
}

void LimbState::updateFromModel(const Model& model)
{
    // ensure we have the same number of limbs
    limbs.reserve(model.limbs.size());
    for(auto& l: model.limbs) {
        auto isLeg = l->options_.model == robotik::Limb::Leg;
        limbs.emplace_back(
                l->options_.model,
                isLeg ? Limb::Holding : Limb::Limp,
                isLeg);
    }
}

void LimbState::zero()
{
    for(auto& limb: limbs) {
        limb.mode = (limb.limbType == Limb::Leg) ? Limb::Holding : Limb::Limp;
        limb.position = KDL::Frame();
        limb.velocity = KDL::Twist();
    }
}


SupportState::SupportState()
    : _baseHeightFromFloor(0)
{
}

void SupportState::zero() {
    _baseHeightFromFloor = 0;
    contacts.clear();
    supportPolygon.clear();
    CoP.Zero();
}

bool SupportState::pointInSupport(const KDL::Vector& point) const {
    if (supportPolygon.size() < 3)
        return false;
    int positive_direction = 0;
    for (unsigned i = 0; i < supportPolygon.size(); ++i){
        unsigned int i2 = (i+1)% (supportPolygon.size());
        double dx = supportPolygon[i2].x() - supportPolygon[i].x();
        double dy = supportPolygon[i2].y() - supportPolygon[i].y();
        if (dx == 0.0 && dy == 0.0){
            // ROS_DEBUG("Skipping polygon connection [%d-%d] (identical points)", i, i2);
            continue;
        }
        double line_test = (point.y() - supportPolygon[i].y())*dx - (point.x() - supportPolygon[i].x())*dy;
        if (i == 0)
            positive_direction = (line_test > 0.0);
        // ROS_DEBUG("Line test [%d-%d] from (%f,%f) to (%f,%f): %f", i, i2, supportPolygon[i].x(), supportPolygon[i].y(), supportPolygon[i2].x(), supportPolygon[i2].y(), line_test);
        if ((line_test > 0.0) != positive_direction)
            return false;
    }

    return true;
}

double SupportState::balanceHealth()
{
    return inSupport ? 1.0 : 0.0;
}

} // ns::robot
