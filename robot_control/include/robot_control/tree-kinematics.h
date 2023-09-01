//
// Created by guru on 2/13/22.
//

#ifndef ROBOT_DYNAMICS_TREE_KINEMATICS_H
#define ROBOT_DYNAMICS_TREE_KINEMATICS_H

//#define USE_OWN_IK_SOLVER

#ifdef USE_OWN_IK_SOLVER
#include <robot_control/trajectory/treeiksolverpos_online.hpp>
#else
#include <kdl/treeiksolverpos_online.hpp>
#endif

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>

#include <kinematics.h>

namespace robotik {

class TreeKinematics
        {
        public:
            static const int SUCCESS = KDL::SolverI::E_NOERROR;

            /// default constructor defers activation of kinematics for later
            TreeKinematics();

            /// activate kinematics on the given model
            bool activate(Model::SharedPtr model);

            /// reset kinematics state
            void deactivate();

            /// compute IK to move the effector to the new position,
            /// updates joint angles and segment positions within the state.
            int moveEffector(JointAndSegmentState& state, const std::string& limb, const KDL::Frame& effector_goal_pose);

            int moveBase(JointAndSegmentState& state, const KDL::Frame& effector_goal_pose);

            /// After limbs have been updated, update whole robot state
            bool updateState(State& state);

            void invalidate(std::string limb, LimbKinematics::SolutionState what_changed = LimbKinematics::INVALID);

        private:
            Model::SharedPtr model_;
            KDL::Tree base_tree;

            Names joint_names;
            std::vector<unsigned int> joint_state_mapping;
            KDL::Frames endpoints_;
            bool reset_endpoints;

            KDL::Frames effector_updates_;
            KDL::Frames base_updates_;

            // on each compute() we take the joint angle guesses (q_in) and the endpoint_frames and compute
            // what the actual joint angles (q_out) should be to reach the desired endpoint frame.
            KDL::JntArray q_in_;
            KDL::JntArray q_out_;

            KDL::JntArray q_min_;
            KDL::JntArray q_max_;
            KDL::JntArray q_dot_min_;

            // todo: useless parameters
            KDL::JntArray q_dot_max_;

            // velocity limits of all joints
            double x_dot_trans_max_;
            double x_dot_rot_max_;

            std::shared_ptr<KDL::TreeIkSolverPos_Online> ik_solver_;
            std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;
            std::shared_ptr<KDL::TreeIkSolverVel_wdls> ik_vel_solver_;

            // computed properties of non-limb parts like base
            //double mass_;
            //KDL::Vector center_of_mass_;

            ///@brief internal recursive TF, CoM and CoP computation on KDL segments.
            bool compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                                const KDL::Frame& tf, State& state);

            bool updateFK(State& state);

            // todo: rename this to reconfigure(), make it public, and add a call in robot_control::reset_xxx()
            bool configure();
        };


} // ns:robotik

#endif //ROBOT_DYNAMICS_TREE_KINEMATICS_H
