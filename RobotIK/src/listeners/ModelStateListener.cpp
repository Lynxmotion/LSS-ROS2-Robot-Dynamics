//
// Created by guru on 12/15/21.
//

#include "listeners/ModelStateListener.h"

#include <utility>
#include "types.h"

namespace robotik {

const std::string ModelStateListener::default_topic_name = "/robot_dynamics/model_state";

ModelStateListener::ModelStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string topic_name, std::string frame_id)
: frame_id_(std::move(frame_id))
{
    // subscribe to joint state messages
    joint_state_subscription_ = node.create_subscription<ModelStateMessageType>(
            topic_name, 10,
            std::bind(&ModelStateListener::model_state_callback, this, std::placeholders::_1));
}

ModelStateListener::ModelStateListener(rclcpp::Node& node, std::string topic_name, std::string frame_id)
: frame_id_(std::move(frame_id))
{
    // subscribe to joint state messages
    joint_state_subscription_ = node.create_subscription<ModelStateMessageType>(
            topic_name, 10,
            std::bind(&ModelStateListener::model_state_callback, this, std::placeholders::_1));
}

void ModelStateListener::model(Model::SharedPtr model)
{
    model_ = std::move(model);
}

void ModelStateListener::state(const std::shared_ptr<State>& state)
{
    if(state)
        state_ = state;
    else
        state_.reset();
}

void ModelStateListener::model_state_callback(ModelStateMessageType::SharedPtr msg)
{
    if(msg->header.frame_id != frame_id_ || !model_)
        return;

    if(!state_)
        state_ = std::make_shared<State>(*model_);

    state_->mass = msg->mass;
    tf2::fromMsg(msg->center_of_mass, state_->CoM);

#if 0       // moved publishing of limb state into Control component
    // update limbs
    if(state_->limbs.size() != model_->limbs.size())
        state_->updateFromModel(*model_);
    for(size_t i = 0, _i = std::min(state_->limbs.size(), msg->limbs.size()); i < _i; i++) {
        auto& st_limb = state_->limbs[i];
        auto& msg_limb = msg->limbs[i];
        st_limb.mode = (Limb::Mode)msg_limb.mode;
        st_limb.supportive = msg_limb.supportive;
        st_limb.supporting = msg_limb.supporting;
        tf2::fromMsg(msg_limb.supporting, st_limb.position);
        tf2::fromMsg(msg_limb.supporting, st_limb.velocity);
    }
#endif

    // update support
    tf2::fromMsg(msg->support.center_of_pressure, state_->CoP);
    tf2::fromMsg(msg->support.support_polygon, state_->supportPolygon);
    state_->inSupport = state_->pointInSupport(state_->CoM);
    state_->support_margin = msg->support.margin;
    state_->seconds_of_stability = msg->support.seconds_of_stability;

    // update support contacts
    std::vector<Contact> contacts;
    for(auto& msg_limb: msg->support.contacts) {
        Contact st_c(msg_limb.limb);

        //st_c.name = model_limb->options_.to_link;
        st_c.name = msg_limb.segment;

        tf2::fromMsg(msg_limb.grf, st_c.grf);

        tf2::fromMsg(msg_limb.wrt_center_of_mass, st_c.wrt_CoM);
        tf2::fromMsg(msg_limb.wrt_center_of_mass, st_c.wrt_base);
        tf2::fromMsg(msg_limb.wrt_center_of_mass, st_c.wrt_odom);

        tf2::fromMsg(msg_limb.polygon, st_c.pointsInContact);

        st_c.staticFriction = msg_limb.static_friction;
        st_c.slippage = msg_limb.slippage;
    }

    // update internal forces
    if(msg->forces.joints.size() == state_->joints.size()) {
        state_->alloc(INTERNAL_FORCE);
        for(size_t i = 0, _i = state_->joints.size(); i < _i; i++) {
            state_->internalForces(i) = msg->forces.joints[i];
        }
    }

    // update external forces
    KDL::Wrench w;
    KDL::WrenchMap externals;
    for(auto& e: msg->forces.external) {
        tf2::fromMsg(e.wrench, w);
        externals[e.frame_id] = w;
    }
    state_->externalForces = externals;

    state_->lastSupportStateUpdate = rclcpp::Time(msg->header.stamp);
}


}
