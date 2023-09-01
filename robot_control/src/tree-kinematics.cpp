//
// Created by guru on 2/13/22.
//

#include "robot_control/tree-kinematics.h"
#include <conversions.h>

#include <utility>

namespace robotik {

TreeKinematics::TreeKinematics()
    : reset_endpoints(true), x_dot_trans_max_(1.0), x_dot_rot_max_(10.0)
{
}

bool TreeKinematics::configure()
{
    if(!model_->tree_->getSubTree(model_->base_link, base_tree)) {
        std::cout << "kinematics: failed to get sub-tree for " << model_->base_link << std::endl;
        return false;
    }
    double update_frequency = 40.0;

    auto& limbs = model_->limbs;
    double nr_of_joints = base_tree.getNrOfJoints();

    Names endpoint_names;
    endpoints_.clear();
    reset_endpoints = true;

    for(auto& limb: limbs) {
        endpoint_names.push_back(limb->link);
        endpoints_[limb->link] = KDL::Frame();
        std::cout << "endpoint: " << limb->link << std::endl;
    }

    joint_names.resize(nr_of_joints);

    q_in_.resize(nr_of_joints);
    q_out_.resize(nr_of_joints);

    q_min_.resize(nr_of_joints);
    q_max_.resize(nr_of_joints);
    q_dot_max_.resize(nr_of_joints);
    q_dot_min_.resize(nr_of_joints);

    // load the limits from the URDF
    const auto& urdf_joints = model_->urdf_model_->joints_;

    auto& segments = model_->tree_->getSegments();
    for(auto& seg: segments) {
        auto& joint = seg.second.segment.getJoint();
        if(joint.getType() != KDL::Joint::None) {
            // our index number into the IK 'q' joint array
            auto qn = seg.second.q_nr;

            const auto& joint_name = joint_names[qn] = joint.getName();

            // get our joint info from URDF
            auto urdf_joint_itr = urdf_joints.find(joint_name);
            if(urdf_joint_itr == urdf_joints.end()) {
                q_min_(qn) = -3.14;
                q_max_(qn) =  3.14;
                q_dot_max_(qn) = 1.0 / update_frequency;
                q_dot_min_(qn) = 0.0;
                std::cout << "warning: joint " << joint_name << " not described in URDF" << std::endl;
            } else {
                const auto& limits = urdf_joint_itr->second->limits;
                if(limits) {
                    q_min_(qn) = urdf_joint_itr->second->limits->lower;
                    q_max_(qn) = urdf_joint_itr->second->limits->upper;
                    q_dot_max_(qn) = urdf_joint_itr->second->limits->velocity / update_frequency;
                }
            }

            std::cout << "  joint " << joint_name
                << "  min: " << q_min_(qn)
                << "  max: " << q_max_(qn)
                << "  vel: " << q_dot_max_(qn) << std::endl;

            // validate the limit parameters
            if(q_min_(qn) == q_max_(qn))
                std::cout << "warning: joint " << joint_name << " has zero position range limit (min: "
                    << q_min_(qn) << ", max: " << q_max_(qn) << ")" << std::endl;
            if(q_dot_max_(qn) == 0.0)
                std::cout << "warning: joint " << joint_name << " has zero velocity limit" << std::endl;
        }
    }

    x_dot_trans_max_ = 0.2 / update_frequency;
    x_dot_rot_max_ = 3.14  / update_frequency;

    // create the FK solver
    fk_solver_ = std::make_shared<KDL::TreeFkSolverPos_recursive>(base_tree);

    // create the IK velocity and position solver
    ik_vel_solver_ = std::make_shared<KDL::TreeIkSolverVel_wdls>(base_tree, endpoint_names);
    ik_vel_solver_->setLambda(0.01);

#if defined(USE_OWN_IK_SOLVER)
    ik_solver_ = std::make_shared<KDL::TreeIkSolverPos_Online>(
            nr_of_joints,
            endpoint_names,
            *fk_solver_,
            *ik_vel_solver_,
            q_min_,
            q_max_,
            q_dot_min_,
            q_dot_max_,
            x_dot_trans_max_,
            x_dot_rot_max_,
            0.0,
            0.0,
            1.0,
            100,
            0.001);
#else
    ik_solver_ = std::make_shared<KDL::TreeIkSolverPos_Online>(
            nr_of_joints,
            endpoint_names,
            q_min_,
            q_max_,
            q_dot_max_,
            x_dot_trans_max_,
            x_dot_rot_max_,
            *fk_solver_,
            *ik_vel_solver_);
#endif
    return true;
}

bool TreeKinematics::activate(Model::SharedPtr model)
{
    if(model_) {
        // already activated
        return true;
    }

    model_ = std::move(model);
    return configure();
}

void TreeKinematics::deactivate()
{
    ik_solver_.reset();
    ik_vel_solver_.reset();
    fk_solver_.reset();
    endpoints_.clear();
    joint_names.clear();
    model_.reset();
    effector_updates_.clear();
    base_updates_.clear();
}

int TreeKinematics::moveEffector(JointAndSegmentState&, const std::string& limb, const KDL::Frame& effector_goal_pose)
{
    effector_updates_[limb] = effector_goal_pose;
    return SUCCESS;
}

int TreeKinematics::moveBase(JointAndSegmentState&, const KDL::Frame& effector_goal_pose)
{
    base_updates_[model_->base_link] = effector_goal_pose;
    return SUCCESS;
}

int status_print = 0;

bool TreeKinematics::updateState(State& state)
{
    if(reset_endpoints) {
        // copy all endpoints over from TF
        KDL::Frame base_tf;
        state.findTF(model_->base_link, base_tf);

        for(auto& ep: endpoints_) {
            KDL::Frame tf;
            if(!state.findTF(ep.first, tf)) {
                std::cout << "error: cant find segment " << ep.first << " in state" << std::endl;
            }
            ep.second = base_tf.Inverse() * tf;
        }
        q_in_ = state.position;
        reset_endpoints = false;
    }

    if(!effector_updates_.empty() || !base_updates_.empty()) {
        //KDL::Frame base_tf;
        //state.findTF(model_->base_link, base_tf);

        // apply base updates first
        for(auto& link: base_updates_) {
            state.tf[link.first] = link.second;
        }
        base_updates_.clear();

        // apply effector movements
        for(auto& link: effector_updates_) {
            assert(endpoints_.find(link.first) != endpoints_.end());
            endpoints_[link.first] = link.second;
            //if(link.first == "left-front-foot")
            //    std::cout << " " << link.first << " =-> " << link.second.p << std::endl;
        }
        effector_updates_.clear();
    }

#if 0
    // build a map between our joint map and states joint map
    // todo: would be nice if Tree Kinematics didnt have to use a joint:state mapping
    if(joint_state_mapping.empty()) {
        auto& segments = model_->tree_->getSegments();
        for(auto& seg: segments) {
            auto& joint = seg.second.segment.getJoint();
            if(joint.getType() != KDL::Joint::None) {
                auto state_joint = std::find(state.joints.begin(), state.joints.end(), joint.getName());
                if(state_joint == state.joints.end()) {
                    std::cout << "joint " << joint.getName() << " in model tree doesn't exist in state joint list" << std::endl;
                    assert(false);
                }

                // our index number into the IK 'q' joint array
                auto qn = seg.second.q_nr;

                // get our index number into our state joint index
                // todo: would be nice if our state array was the same as the tree/IK array
                auto jn = state_joint - state.joints.begin();
                joint_state_mapping.push_back(jn);

                q_in_(qn) = state.position(jn);
                assert(qn == jn);
                std::cout << "   Q" << qn << ": J" << jn << "  " << joint.getName() << std::endl;
            }
        }
    }
#else
    //q_in_ = state.position;
#endif

    // compute the IK
    auto remaining_weighted_distance_to_target
        = ik_solver_->CartToJnt(q_in_, endpoints_, q_out_);

    // if less than zero then something went wrong
    if(remaining_weighted_distance_to_target >= -0.0) {
#if 0
        // good translation, save the joint angles back to our state
        for(unsigned int i=0; i < q_out_.rows(); i++) {
            auto jn = joint_state_mapping[i];
            assert(i == jn);
            state.position(jn) = q_out_( i );
        }
#else
        state.position = q_out_;
#endif

        // this output becomes the guess for next IK iteration
        q_in_ = q_out_;

        status_print++;
        if(status_print > 30) {
            if(remaining_weighted_distance_to_target > 0.05)
                std::cout << "  dtt: " << remaining_weighted_distance_to_target << std::endl;
            status_print = 0;
        }

        return updateFK(state);
    } else {
        std::cout << "IK error " << (int)remaining_weighted_distance_to_target << std::endl;
        return false;
    }
}

void TreeKinematics::invalidate(std::string, LimbKinematics::SolutionState)
{

}

bool TreeKinematics::updateFK(State& state)
{
    state.mass = 0.0;
    state.CoM = KDL::Vector::Zero();

    auto root = base_tree.getRootSegment();
    KDL::Frame transform = state.tf[model_->base_link];

    if(!compute_TF_CoM(root, transform, state)) {
        throw Exception(RE_FAILED_COM);
    }

    state.CoM = 1.0/state.mass * state.CoM;

    return true;
}

bool TreeKinematics::compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                                const KDL::Frame& tf, robotik::State& state)
                                {
    // if this joint is part of a limb then we stop recursing
    // since limbs are updated with IK
    const auto& joint = currentSeg->second.segment.getJoint();

    KDL::Frame jnt_tf;
    std::string child_name = currentSeg->first;

    if (joint.getType() != KDL::Joint::None) {
        // rotational or translational joints,
        // lookup joint position in state and update TF in state
        // todo: could be improve the joint.name => q# loopup here? it's using std::find on the joint names
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


} // ns:robotik
