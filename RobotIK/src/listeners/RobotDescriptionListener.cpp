//
// Created by guru on 1/17/22.
//

#include "listeners/RobotDescriptionListener.h"

#include <utility>

namespace robotik {

const std::string RobotDescriptionListener::default_topic_name = "/robot_description";

RobotDescriptionListener::RobotDescriptionListener(
        rclcpp_lifecycle::LifecycleNode& node,
        std::function<CallbackType>&& callback,
        std::string urdf_topic_name,
        std::string srdf_topic_name)
        : received_urdf(false), received_srdf(false), logger_(node.get_logger()), callback_(callback)
{
    if(srdf_topic_name.empty())
        srdf_topic_name = urdf_topic_name + "/srdf";
    subscription_robot_description_ = node.template create_subscription<std_msgs::msg::String>(
            urdf_topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&RobotDescriptionListener::robot_description_callback, this, std::placeholders::_1)
            );
    subscription_srdf_description_ = node.template create_subscription<std_msgs::msg::String>(
            srdf_topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&RobotDescriptionListener::srdf_description_callback, this, std::placeholders::_1)
            );
}

RobotDescriptionListener::RobotDescriptionListener(
        rclcpp::Node& node,
        std::function<CallbackType>&& callback,
        std::string urdf_topic_name,
        std::string srdf_topic_name)
        : received_urdf(false), received_srdf(false), logger_(node.get_logger()), callback_(callback)
{
    if(srdf_topic_name.empty())
        srdf_topic_name = urdf_topic_name + "/srdf";
    subscription_robot_description_ = node.template create_subscription<std_msgs::msg::String>(
            urdf_topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&RobotDescriptionListener::robot_description_callback, this, std::placeholders::_1)
            );
    subscription_srdf_description_ = node.template create_subscription<std_msgs::msg::String>(
            srdf_topic_name,
            rclcpp::QoS(1).transient_local(),
            std::bind(&RobotDescriptionListener::srdf_description_callback, this, std::placeholders::_1)
            );
}

void RobotDescriptionListener::robot_description_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(logger_, model_ ? "received updated robot description (URDF)" : "received robot description (URDF)");
    std::string urdf_base_path = "/home/guru/src/lss-ros2/lss/lss_hexapod/urdf/";

    if(!model_)
        model_ = std::make_shared<Model>();

    model_->setupURDF(msg->data);

    // if we already received SRDF then apply it
    if(!srdf_pending.empty()) {
        model_->setupSRDF(srdf_pending);
        srdf_pending.clear();
    }

    received_urdf = true;
    if(received_srdf && received_urdf && callback_)
        // URDF and SRDF received, notify the root node
        callback_(model_);
}

void RobotDescriptionListener::srdf_description_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(logger_, model_ ? "received updated semantic robot description (SRDF)" : "received semantic robot description (SRDF)");

    if(!model_) {
        RCLCPP_INFO(logger_, "SRDF pending until URDF received");
        srdf_pending = msg->data;
    } else {
        model_->setupSRDF(msg->data);
    }

    received_srdf = true;
    if(received_srdf && received_urdf && callback_)
        // URDF and SRDF received, notify the root node
        callback_(model_);
}

}
