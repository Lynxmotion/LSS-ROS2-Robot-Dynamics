//
// Created by guru on 1/17/22.
//

#ifndef ROBOT_DYNAMICS_ROBOTDESCRIPTIONLISTENER_H
#define ROBOT_DYNAMICS_ROBOTDESCRIPTIONLISTENER_H


#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <std_msgs/msg/string.hpp>
#include <model.h>


namespace robotik {

class RobotDescriptionListener {
public:
    using SharedPtr = std::shared_ptr<RobotDescriptionListener>;
    using CallbackType = void(Model::SharedPtr);

    static const std::string default_topic_name;

    explicit RobotDescriptionListener(
            rclcpp_lifecycle::LifecycleNode& node,
            std::function<CallbackType>&& callback,
            std::string urdf_topic_name = default_topic_name,
            std::string srdf_topic_name = "");

    explicit RobotDescriptionListener(
            rclcpp::Node& node,
            std::function<CallbackType>&& callback,
            std::string urdf_topic_name = default_topic_name,
            std::string srdf_topic_name = "");

    [[nodiscard]] inline const Model::SharedPtr& model() const { return model_; }
    [[nodiscard]] inline Model::SharedPtr& model() { return model_; }


protected:
    Model::SharedPtr model_;
    bool received_urdf, received_srdf;
    std::string srdf_pending;
    rclcpp::Logger logger_;
    std::function<CallbackType> callback_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_srdf_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);
    void srdf_description_callback(std_msgs::msg::String::SharedPtr msg);
};

} // ns:robotik

#endif //ROBOT_DYNAMICS_ROBOTDESCRIPTIONLISTENER_H
