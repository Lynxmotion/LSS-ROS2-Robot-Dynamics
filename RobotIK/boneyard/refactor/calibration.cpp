//
// Created by guru on 6/18/20.
//

#include "calibration.h"
#include "state.h"
#include "model.h"
#include "conversions.h"

namespace robotik {

std::string JointCalibration::interaction_namespace() {
    return "calibration";
}

std::set<std::string> JointCalibration::manipulators(State&) {
    std::set<std::string> links;
    for(auto& limb: model_->limbs) {
        links.insert(limb->options_.from_link);
        links.insert(limb->options_.to_link);
    }
    return links;
}

///@brief Handle interaction events and update calibration
bool JointCalibration::interact(InteractionEvent &ev) {
    // rclcpp::Time stamp(ev.header.stamp.sec, ev.header.stamp.nanosec, RCL_ROS_TIME);
    if(state)
        jointUpdates[ev.segment].calibrate(state->transform.Inverse() * ev.tf);
    return true;
}

void JointCalibration::clear() {
    allRecalibration.clear();
    jointUpdates.clear();
}

void JointCalibration::begin(Model::SharedPtr model, State& _state) {
    clear();
    model_ = model;

    // todo: state is being copied but copy constructor/assignment is not copying position/TF fields
    if(!state) {
        state = std::make_shared<State>(_state);
        state->position = _state.position;
        state->tf = _state.tf;
    } else {
        *state = _state;
        state->position = _state.position;
        state->tf = _state.tf;
    }
}

///@brief Commit the joint offsets to non-volatile config
JointCalibration::Data JointCalibration::commit() {
    auto all = allRecalibration;
    allRecalibration.clear();
    return all;
}

JointCalibration::Data JointCalibration::update(Model& _model, State& _state, StateVisual&_visual) {
    auto jd = update(_model, _state);
    if(state) {
        std::set<std::string> modified_segments;
        for(auto& u: jointUpdates) {
            modified_segments.insert(u.second.segmentsCalibrated.begin(), u.second.segmentsCalibrated.end());
        }
        _visual.segments(modified_segments);
        _visual.update(*state);
    }
    return jd;
}

JointCalibration::Data JointCalibration::update(Model& model, State& _state) {
#if 0
    if(!state) {
        state = std::make_shared<State>(src);
        //state->position = src.position;
        //state->tf = src.tf;
    } else {
        *state = src;
        //state->position = src.position;
        //state->tf = src.tf;
    }
#endif

    // get the robot base TF which is the base of the IK
    KDL::Frame tfBase;

    // must update base first if it was manipulated and
    // retrieve the calibrated base position into tfBase for limb IK
    auto base_itr = jointUpdates.find(model.base_link);
    if (base_itr != jointUpdates.end()) {
        if(base_itr->second.updated) {
            // base position was just manipulated so update calibration state
            // and use new base location from the moved endpoint as tfBase
            tfBase = state->tf[model.base_link] = base_itr->second.endpoint;
            base_itr->second.updated = false;
            base_itr->second.segmentsCalibrated.insert(base_itr->first);   // need to show base now

            // must call IK here to update all the limbs
            model.updateIK(*state);
            model.compute_TF_CoM(*state);

            _state.tf[model.base_link] = tfBase;
        } else
            // base TF has been updated in calibration but hasnt been moved since
            // use calibration base position
            tfBase = state->findTF(model.base_link);
    } else {
        // base position not manipulated during calibration, so use and
        // keep updating calibration base position/orientation with current state
        tfBase = _state.findTF(model.base_link);
        state->tf[model.base_link] = tfBase;
    }


    Data jd;
    for(auto& u: jointUpdates) {
        // no way to calibrate robot base, footprint or odom frame
        // really you shouldnt put manipulators on footprint or odom frame, but you could on base just to move the robot
        // base for suitable calibration.
        if(!u.second.updated || u.first == model.base_link || u.first == model.odom_link || u.first == model.footprint_link)
            continue;

        // create the IK chain if not already created
        // todo: we should be able to move this code to Limb::computePose() and have it be more efficient than current Limb IK
        if(!u.second.chain) {
            u.second.chain = std::make_shared<KDL::Chain>();
            if (!model.tree_->getChain(model.base_link, u.first, *u.second.chain)) {
                throw Exception(RE_INVALID_CHAIN, "cannot create chain for calculating joint calibration");
            }

            // found the base, compute the inner joint angles
            auto nj = u.second.chain->getNrOfJoints();

            // get the orignal joint angles
            u.second.jnt_pos_mapping.resize(nj);
            u.second.jnt_original = KDL::JntArray( nj );
            auto ns = u.second.chain->getNrOfSegments();
            for(unsigned int s=0, j=0; s < ns; s++) {
                // s => segment ordinal
                // j => joint ordinal (it tracks with segment ordinal except fixed joints dont count)
                auto seg = u.second.chain->getSegment(s);
                auto joint = seg.getJoint();
                if (joint.getType() != KDL::Joint::None) {
                    // add joint angle to calibration data
                    auto jordinal = state->findJoint(joint.getName());
                    u.second.jnt_pos_mapping[j] = jordinal;

                    // todo: giving hint can make for some out-of-range angles, if the hints are already out of range
                    //  ...so add some range checking before re-enabling hints
                    u.second.jnt_original(j) = state->position(jordinal);
                    j++;
                }
            }

            u.second.q_out = KDL::JntArray( nj );
            u.second.jnt_quess = u.second.jnt_original;
        }

        // transform the joint tip from algo frame to goal frame
        auto evTF = u.second.endpoint;
        KDL::Frame goal = tfBase.Inverse() * evTF;

        KDL::ChainIkSolverPos_LMA solver(*u.second.chain);
        auto err = solver.CartToJnt(u.second.jnt_quess, goal, u.second.q_out);
        if(err == KDL::ChainIkSolverPos_LMA::E_NOERROR) {
            // KDL output becomes the guess for the next iteration
            u.second.jnt_quess = u.second.q_out;

            // reset which segments we've calibrated
            u.second.segmentsCalibrated.clear();

            // we'll compute the FK for each segment as we go
            KDL::Frame tf = tfBase;

            auto ns = u.second.chain->getNrOfSegments();
            for(unsigned int s=0, j=0; s < ns; s++) {
                // s => segment ordinal
                // j => joint ordinal (it tracks with segment ordinal except fixed joints dont count)
                auto seg = u.second.chain->getSegment(s);
                auto joint = seg.getJoint();
                double pos = 0;

                if(joint.getType() != KDL::Joint::None) {
                    // add joint to calibrated segment list
                    // but ignore if change in joint position is really small
                    if(std::abs(u.second.q_out(j) - u.second.jnt_original(j)) > 0.0000001) {
                        u.second.segmentsCalibrated.insert(seg.getName());

                        // send joint calibration
                        jd[joint.getName()] = u.second.q_out(j);
                    }

                    // get joint position as computed by IK
                    pos = u.second.q_out(j);

                    // set position in state
                    auto ord = u.second.jnt_pos_mapping[j];
                    state->joints_updated[ord] = true;
                    state->position(ord) = pos;

                    j++;
                }

                // compute the new frame position for this segment
                tf = tf * seg.pose(pos);
                state->tf[seg.getName()] = tf;
            }

            u.second.updated = false;
        }
    }

    // add all joint data to final
    for(auto& d: jd)
        allRecalibration[d.first] = d.second;

    return jd;
}

} // ns: robotik
