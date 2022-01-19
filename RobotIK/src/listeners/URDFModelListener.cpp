//
// Created by guru on 1/17/22.
//

#include "listeners/URDFModelListener.h"

#include <utility>

namespace robotik {

const std::string URDFModelListener::default_topic_name = "/robot_description";

URDFModelListener::URDFModelListener(
        rclcpp_lifecycle::LifecycleNode& node,
        std::function<CallbackType>&& callback,
        std::string topic_name)
        : logger_(node.get_logger()), callback_(callback)
{
    subscription_robot_description_ = node.template create_subscription<std_msgs::msg::String>(
            topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&URDFModelListener::robot_description_callback, this, std::placeholders::_1)
            );
}

URDFModelListener::URDFModelListener(
        rclcpp::Node& node,
        std::function<CallbackType>&& callback,
        std::string topic_name)
        : logger_(node.get_logger()), callback_(callback)
{
    subscription_robot_description_ = node.template create_subscription<std_msgs::msg::String>(
            topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&URDFModelListener::robot_description_callback, this, std::placeholders::_1)
            );
}

void URDFModelListener::robot_description_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(logger_, model_ ? "received updated robot description (URDF)" : "received robot description (URDF)");
    std::string urdf_base_path = "/home/guru/src/lss-ros2/lss/lss_hexapod/urdf/";

    if(!model_)
        model_ = std::make_shared<Model>();

    model_->setupURDF(
            msg->data,
            urdf_base_path + "lss_hexapod.srdf"
            );

    if(callback_)
        callback_(model_);
}


}
