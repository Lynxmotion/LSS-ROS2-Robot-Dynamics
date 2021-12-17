//
// Created by guru on 12/30/19.
//

#pragma once


#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_model_msgs/msg/control_state.hpp"

#include <std_srvs/srv/empty.hpp>

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <robotik.h>
#include <robot_control/control.h>
#include <listeners/JointStateListener.h>
#include <listeners/ModelStateListener.h>

#include <tf2_ros/buffer.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <string>
#include <limb.h>
#include <raf.h>


namespace  robot_dynamics {

class Control : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    Control();

    explicit Control(const rclcpp::NodeOptions & options);

    explicit Control(
            const std::string & node_name,
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State &) override;

    void updateRobotState();

protected:
    void publish();
    void publish_control_state(const robotik::State& current, const robotik::State& target, rclcpp::Time now, std::string prefix="");
    void publish_diagnostics();

    robotik::Model::SharedPtr model_;

    struct {
        robotik::State::SharedPtr state;
        robotik::Control::SharedPtr control;
    } current;

    rclcpp::Time lastControlUpdate;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);

    robotik::JointStateListener::SharedPtr joint_state_listener;
    robotik::ModelStateListener::SharedPtr model_state_listener;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;

    // detect parameters modification during runtime
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr old_parameter_set_callback;
    rcl_interfaces::msg::SetParametersResult parameter_set_callback(const std::vector<rclcpp::Parameter> & param);

    tf2_ros::Buffer tfBuffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_static_;
    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);        // callback

    // extended joint controller publishers
    robot_model_msgs::msg::ControlState::SharedPtr control_state_msg_;
    rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::ControlState>::SharedPtr control_state_pub_;

};

} // ns:robot_dynamics
