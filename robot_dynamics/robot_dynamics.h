//
// Created by guru on 12/30/19.
//

#pragma once


#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "humanoid_model_msgs/msg/compliant_joint_params.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/rate.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "../RobotIK/include/types.h"

#include <tf2_ros/buffer.h>

#include <robotik.h>

#include <memory>
#include <string>
#include <limb.h>
#include <raf.h>

#define LEG_SUPPORT_DISTANCE_PARAM "leg_support_distance"

namespace  robot_dynamics {

class Dynamics : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    Dynamics();

    explicit Dynamics(const rclcpp::NodeOptions & options);

    explicit Dynamics(
            const std::string & node_name,
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State &);

    void updateRobotState();

protected:
    void publish();
    void publish_model_state();
    void publish_diagnostics();

    void resetSim();

    robotik::Model::SharedPtr model_;

    struct {
        robotik::State::SharedPtr state;
    } current;

    struct {
        rclcpp::Time lastJointStateUpdate;
        bool active;
        bool publishCompliance;
        bool publishOdometry;
    } control;

    // detect heading change
    double lastHeading;

    // true if the environment is part of simulation
    bool is_simulation;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    void joint_states_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr model_state_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;

    // detect parameters modification during runtime
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr old_parameter_set_callback;
    rcl_interfaces::msg::SetParametersResult parameter_set_callback(const std::vector<rclcpp::Parameter> & param);

    // extended joint controller publishers
    humanoid_model_msgs::msg::CompliantJointParams::SharedPtr compliance_params_msg_;
    rclcpp_lifecycle::LifecyclePublisher<humanoid_model_msgs::msg::CompliantJointParams>::SharedPtr compliance_params_pub_;

    tf2_ros::Buffer tfBuffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_static_;
    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);        // callback

    // IMU sense
    sensor_msgs::msg::Imu imu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu);

    // odometry publisher
    nav_msgs::msg::Odometry::SharedPtr odometry_msg_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};

} // ns:robot_dynamics
