//
// Created by guru on 11/14/21.
//

#include "kinematics.h"


namespace robotik {

static double eps = 1E-5;
static double eps_joints = 1E-15;

LimbKinematics::LimbKinematics(Limb::SharedPtr && limb)
: state_(INVALID), limb_(std::move(limb)), configured_state(nullptr)
{
    assert(limb_);                   // must be given a valid limb
    assert(limb_->get_chain());       // with a valid chain
}

LimbKinematics::LimbKinematics(const LimbKinematics& copy)
: state_(copy.state_), limb_(copy.limb_), q_(copy.q_), qdot_(copy.qdot_), qdotdot_(copy.qdotdot_), q_out_(copy.q_out_),
  configured_state(nullptr), mass_(copy.mass_), center_of_mass_(copy.center_of_mass_)
{
}


bool LimbKinematics::configure(JointAndSegmentState& state)
{
    auto chain = limb_->get_chain();
    assert(chain);

    auto nj = chain->getNrOfJoints();
    auto ns = chain->getNrOfSegments();

    // ensure our joint arrays are the right size
    q_.resize(nj);
    qdot_.resize(nj);
    qdotdot_.resize(nj);
    q_out_.resize(nj);
    //KDL::JntArray jnt_quess( nj );
    //NamedJointArray joints_out( nj );

    // we'll build an index between our limb's joint number
    // and the index into the state's joint positions array
    joint_position_index_.resize(nj);

    // get information about the chain
    for(unsigned int s=0, j=0; s < ns; s++) {
        auto segment = chain->getSegment(s);
        auto joint = segment.getJoint();
        auto jname = joint.getName();
        if(joint.getType() == KDL::Joint::None)
            continue;   // skip fixed axis

        auto jordinal = state.findJoint(jname);
        if (jordinal < 0) {
            // error, joint value not found
            return false;
        }

        joint_position_index_[j] = jordinal;
        j++;
    }

    Eigen::Matrix< double, 6, 1 > L;
#if 1
    L(0)=1;
    L(1)=1;
    L(2)=1;
    L(3)=0.01;
    L(4)=0.01;
    L(5)=0.01;
#else
    L(0)=0.5;
    L(1)=0.5;
    L(2)=0.5;
    L(3)=0.005;

    L(4)=0.005;
    L(5)=0.005;
#endif

    lma_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(*chain, L, eps, 500, eps_joints);

    return true;
}

int LimbKinematics::moveEffector(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose)
{
    auto r = computeIK(state, effector_goal_pose);
    if(r != KDL::ChainIkSolverPos_LMA::E_NOERROR)
        return r;
    return updateJointsAndSegments(state, nullptr)
        ? Kinematics::SUCCESS
        : -3;
}

int LimbKinematics::moveEffector(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose, KDL::Frame& effector_out)
{
    auto r = computeIK(state, effector_goal_pose);
    if(r != KDL::ChainIkSolverPos_LMA::E_NOERROR)
        return r;
    return updateJointsAndSegments(state, &effector_out)
        ? Kinematics::SUCCESS
        : -3;
}

int LimbKinematics::computeIK(JointAndSegmentState& state, const KDL::Frame& new_effector_pose)
{
    if(requires_configure(state) && !configure(state))
        return -1;      // configure failed

    // copy the joint state into our Joint arrays
    auto state_position_sz = state.position.rows();
    auto state_velocity_sz = state.velocity.rows();
    auto state_acceleration_sz = state.acceleration.rows();
    for(size_t j=0, _j=joint_position_index_.size(); j < _j; j++) {
        auto jidx = joint_position_index_[j];
        q_(j) =       (jidx < state_position_sz)     ? state.position(jidx)     : 0.0;
        qdot_(j) =    (jidx < state_velocity_sz)     ? state.velocity(jidx)     : 0.0;
        qdotdot_(j) = (jidx < state_acceleration_sz) ? state.acceleration(jidx) : 0.0;
    }

    // get the base => effector transform
    KDL::Frame base_tf;
    KDL::Frame effector_wrt_base_tf;
    if(!state.findTF(limb_->options_.from_link, base_tf)) {
        return -2;      // no base link in state
    }

    // new_effector_pose is the effector relative to odom, so we want to transform
    // it to be relative to the effector base link
    effector_wrt_base_tf = base_tf.Inverse() * new_effector_pose;

    // perform the IK
    auto err = lma_solver_->CartToJnt(q_, effector_wrt_base_tf, q_out_);
    if(err == KDL::ChainIkSolverPos_LMA::E_NOERROR
    || err == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL
    || err == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL) {
        // todo: do some limits validation here, if not within limits then recalculate
        state_ = IK_SOLVED;
        return Kinematics::SUCCESS;
    } else {
#if defined(DEBUG_LIMB)
        std::cout << "IK failed: " << options_.to_link << ": ";
        switch(err) {
            case KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL: std::cout << "GRADIENT_JOINTS_TOO_SMALL the gradient of E towards the joints is to small" << std::endl; break;
            case KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL: std::cout << "INCREMENT_JOINTS_TOO_SMALL joint position increments are to small" << std::endl; break;
            case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: std::cout << "MAX_ITERATIONS_EXCEEDED number of iterations is exceeded" << std::endl; break;
            default:
                std::cout << " error " << err << std::endl;
                break;
        }
#if DEBUG_LIMB > 1
std::cout << "    limb: " << options_.to_link << "   joints: ";
        for (int i = 0, _i = jnt_quess.rows(); i < _i; i++) {
            std::cout << "   " << joint_names[i] << "=" << jnt_quess(i);
        }
        std::cout << std::endl;
#endif
#endif
        return err;
    }

}

void LimbKinematics::updateJoints(JointAndSegmentState& state)
{
    auto state_position_sz = state.position.rows();
    for(size_t j=0, _j=joint_position_index_.size(); j < _j; j++) {
        auto jidx = joint_position_index_[j];
        state.position(jidx) = (jidx < state_position_sz) ? q_out_(j) : 0.0;
    }
    state_ = JOINTS;
}

bool LimbKinematics::updateJointsAndSegments(JointAndSegmentState& state, KDL::Frame* effector_out)
{
    auto chain = limb_->get_chain();
    auto state_position_sz = state.position.rows();

    // start off with the limb's base transform
    KDL::Frame tf;
    if(!state.findTF(limb_->options_.from_link, tf)) {
        return false;      // no base link in state
    }

    mass_ = 0;
    center_of_mass_ = KDL::Vector::Zero();

    // get information about the chain
    int j = 0;
    auto nj = chain->getNrOfSegments();
    for(unsigned int s=0; s<nj; s++) {
        const auto &segment = chain->getSegment(s);
        const auto &joint = segment.getJoint();
        double pos = 0;

        if(joint.getType() != KDL::Joint::None) {
            // lookup our joint index within state
            auto jidx = joint_position_index_[j];

            // get the joint rotational angle
            //pos = state.position(jidx);
            pos = q_out_(j);

            // update the joint position in state
            state.position(jidx) = (jidx < state_position_sz) ? pos : 0.0;

            // we only increment joint number for non-fixed joints
            j++;
        } else
            // all fixed joints keep angle of 0
            pos = 0;

        // compute the transform for this segment
        tf = tf * segment.pose(pos);

        // update the segment transform in state
        state.tf[segment.getName()] = tf;

        // update limb properties
        const auto& inertia = segment.getInertia();
        KDL::Vector current_cog = inertia.getCOG();
        double current_m = inertia.getMass();

        center_of_mass_ = center_of_mass_ + current_m * (tf * current_cog);
        mass_ += current_m;
    }

    if(effector_out)
        *effector_out = tf;

    state_ = SEGMENTS;
    return true;
}

bool LimbKinematics::updateState(State& state)
{
    if(state_ <= INVALID) {
        KDL::Frame effector_goal;
        if(state.findTF(limb_->options_.to_link, effector_goal)) {
            if(moveEffector(state, effector_goal) != Kinematics::SUCCESS)
                return false;
        }
    }

    if(state_ < SEGMENTS && !updateJointsAndSegments(state, nullptr))
        return false;

    // todo: calculate dynamics

    state_ = VALID;
    return true;
}

Kinematics::Kinematics(Model::SharedPtr model)
: model_(std::move(model)), mass_(0.0)
{
    for(auto& l: model_->limbs) {
        limbs_first_joint.insert(l->joint_names[0]);
        limbs_.emplace(std::make_pair(l->options_.to_link, l));
    }

    // todo: calculate the mass and CoM of non-limb parts
}

/// compute IK to move the effector to the new position,
/// updates joint angles and segment positions within the state.
int Kinematics::moveEffector(JointAndSegmentState& state, std::string limb, const KDL::Frame& effector_goal_pose)
{
    if(limb == model_->base_link) {
        return moveBase(state, effector_goal_pose);
    }

    auto itr = limbs_.find(limb);
    if(itr != limbs_.end()) {
        return itr->second.moveEffector(state, effector_goal_pose);
    } else
        return -1;
}

int Kinematics::moveEffector(JointAndSegmentState& state, std::string limb, const KDL::Frame& effector_goal_pose, KDL::Frame& effector_out)
{
    if(limb == model_->base_link) {
        return moveBase(state, effector_goal_pose);
    }

    auto itr = limbs_.find(limb);
    if(itr != limbs_.end()) {
        return itr->second.moveEffector(state, effector_goal_pose, effector_out);
    } else
        return -1;
}

int Kinematics::moveBase(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose)
{
    // todo: check limits of base pose before accepting it
    state.tf[model_->base_link] = effector_goal_pose;

    // invalidate all legs
    for(auto& l: limbs_) {
        l.second.invalidate();
    }

    return SUCCESS;
}

int Kinematics::moveBase(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose, KDL::Frame& effector_out)
{
    auto r = moveBase(state, effector_goal_pose);
    if(r == 0)
        effector_out = state.tf[model_->base_link];
    return r;
}

bool Kinematics::updateState(State& state)
{
    // complete update of any pending limbs
    //mass_ = 0.0;
    //center_of_mass_.Zero();
    state.mass = 0.0;
    state.CoM = KDL::Vector::Zero();
    bool success = true;

    for(auto& l: limbs_) {
        if(!l.second.updateState(state))
            success = false;
        state.mass += l.second.mass();
        state.CoM = state.CoM + l.second.CoM();
    }

    // update non-limb segments using a recursive evaluator
    KDL::Frame transform = state.tf[model_->base_link];
    //transform.M = state.imu * transform.M; //   why was I transforming about IMU?

    auto root = model_->tree_->getSegment(model_->base_link);

    if (state.mass <= 0.0) {
        state.CoM = KDL::Vector::Zero();
        throw Exception(RE_ZERO_MASS, "Total mass is 0, no CoM possible.");
    }

    if(!compute_TF_CoM(root, transform, state)) {
        throw Exception(RE_FAILED_COM);
    }

    state.CoM = 1.0/state.mass * state.CoM;

    return success;
}

bool Kinematics::compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                           const KDL::Frame& tf, robotik::State& state)
{
    // if this joint is part of a limb then we stop recursing
    // since limbs are updated with IK
    auto joint = currentSeg->second.segment.getJoint();
    if(limbs_first_joint.find(joint.getName()) != limbs_first_joint.end())
        return true;

    KDL::Frame jnt_tf, fparent, fchild;
    std::string child_name = currentSeg->first;

    if(child_name == model_->footprint_link || child_name == model_->base_link) {
        // we compute these transforms internally, so retrieve the current TF from state
        if(state.findTF(child_name, fchild) && state.findTF(currentSeg->second.parent->first, fparent)) {
            // compute joint TF relative to parent
            jnt_tf = fparent.Inverse() * fchild;
        } else {
            // cannot retrieve from state, so compute as a fixed joint and update state
            jnt_tf = state.tf[child_name] = tf * currentSeg->second.segment.pose(0);
        }
    } else {
        if (joint.getType() != KDL::Joint::None) {
            // rotational or translational joints,
            // lookup joint position in state and update TF in state
            auto jordinal = state.findJoint(joint.getName());
            if (jordinal < 0)
                return false;
            double jnt_p = state.position(jordinal);
            jnt_tf = tf * currentSeg->second.segment.pose(jnt_p);
        } else {
            // fixed segment, transform using p=0
            jnt_tf = tf * currentSeg->second.segment.pose(0);
        }

        state.tf[child_name] = jnt_tf;
    }

    // aggregate mass and calculate it's center
    KDL::Vector current_cog = currentSeg->second.segment.getInertia().getCOG();
    double current_m = currentSeg->second.segment.getInertia().getMass();

    state.CoM = state.CoM + current_m * (jnt_tf * current_cog);
    state.mass += current_m;

    // recurse into child joints
    std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
    for (child_it = currentSeg->second.children.begin(); child_it != currentSeg->second.children.end(); ++child_it){
        if(!compute_TF_CoM(*child_it, jnt_tf, state))
            return false;
    }
    return true;
                           }


}  //ns:robotik