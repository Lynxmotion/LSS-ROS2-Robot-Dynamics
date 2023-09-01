//
// Created by guru on 1/16/22.
//

#ifndef ROBOT_DYNAMICS_ROBOT_LOCALIZATION_H
#define ROBOT_DYNAMICS_ROBOT_LOCALIZATION_H

#pragma once

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include "robot_model_msgs/msg/model_state.hpp"
#include "robot_model_msgs/msg/control_state.hpp"

#include <robotik.h>
#include <raf.h>
#include <publishers/LifecycleManager.h>
#include <listeners/RobotDescriptionListener.h>
#include <listeners/ModelStateListener.h>

#include <memory>
#include <string>


namespace  robot_dynamics {

class RobotLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    RobotLocalization();

    explicit RobotLocalization(const rclcpp::NodeOptions & options);

    explicit RobotLocalization(
            const std::string & node_name,
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
                    );

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State &) override;


protected:
    robotik::LifecycleManager::SharedPtr self_managed;
    robotik::Model::SharedPtr model_;
    robotik::State::SharedPtr state;

    // process current state and produce odometry data
    void update();

    robotik::ModelStateListener::SharedPtr model_state_listener;
    void robot_model_callback(robotik::Model::SharedPtr model);

    robotik::RobotDescriptionListener::SharedPtr robot_description_listener;

    // IMU sense
    sensor_msgs::msg::Imu imu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu);

    // odometry publisher
    nav_msgs::msg::Odometry::SharedPtr odometry_msg_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};

} // ns:robot_dynamics
#endif //ROBOT_DYNAMICS_ROBOT_LOCALIZATION_H
