//
// Created by guru on 12/15/21.
//

#ifndef ROBOT_DYNAMICS_MODELSTATELISTENER_H
#define ROBOT_DYNAMICS_MODELSTATELISTENER_H

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <robot_model_msgs/msg/model_state.hpp>
#include <model.h>


namespace robotik {

class ModelStateListener {
public:
    using SharedPtr = std::shared_ptr<ModelStateListener>;
    using ModelStateMessageType = robot_model_msgs::msg::ModelState;

    static const std::string default_topic_name;

    explicit ModelStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string topic_name = default_topic_name, std::string frame_id = "");
    explicit ModelStateListener(rclcpp::Node& node, std::string topic_name = default_topic_name, std::string frame_id = "");

    // todo: we probably have to add activate/deactivate to listener classes so they stop processing
    //       messages when main node is not active

    void model(Model::SharedPtr model);

    void state(const std::shared_ptr<State>& state);

    [[nodiscard]] inline const std::shared_ptr<State>& state() const { return state_; }
    [[nodiscard]] inline std::shared_ptr<State>& state() { return state_; }

protected:
    std::string frame_id_;
    Model::SharedPtr model_;
    std::shared_ptr<State> state_;

    // todo: support dynamic_joint_state control_msgs?
    rclcpp::Subscription<ModelStateMessageType>::SharedPtr joint_state_subscription_;

    void model_state_callback(ModelStateMessageType::SharedPtr msg);
};

} // ns:robotik

#endif //ROBOT_DYNAMICS_MODELSTATELISTENER_H
