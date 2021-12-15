//
// Created by guru on 12/30/19.
//

#pragma once


#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <std_srvs/srv/empty.hpp>

#include "robot_model_msgs/msg/compliant_joint_params.hpp"
#include "robot_model_msgs/msg/joint_calibration.hpp"

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "../RobotIK/include/types.h"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <robotik.h>

#include <memory>
#include <string>
#include <limb.h>
#include <raf.h>

#define LEG_SUPPORT_DISTANCE_PARAM "leg_support_distance"


class RobotDynamics : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    using Marker = visualization_msgs::msg::Marker;
    using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;
    using MenuHandler = interactive_markers::MenuHandler;
    using EntryHandle = MenuHandler::EntryHandle;
    using MenuID = visualization_msgs::msg::InteractiveMarkerFeedback::_menu_entry_id_type;

    using JointData = robotik::JointData;
    using NamedJointData = robotik::NamedJointData;
    using NamedJointArray = std::vector<robotik::NamedJointData>;
    using JointMap = robotik::JointMap;

    RobotDynamics();

    explicit RobotDynamics(const rclcpp::NodeOptions & options);

    explicit RobotDynamics(
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

    //KDL::Vector computeCoP();

    void updateRobotState();

    InteractiveMarkerServer server;

protected:
    void publish();
    void publish_model_state();
    void publish_diagnostics();

    void resetSim();

    robotik::Model::SharedPtr model_;
    robotik::RollingAverage<KDL::Vector> control_position;

    struct {
        robotik::State::SharedPtr state;
        robotik::Control::SharedPtr control;
        robotik::StateVisual::SharedPtr visual;

        bool calibrateMode;               // true if we are calibrating in RViz now (enables visual manipulators)
        bool calibrateContinuousSync;     // true to continuously update hardware as manipulators are moved
        robotik::StateVisual::SharedPtr calibrationVisual;
    } current;

    struct {
        rclcpp::Time lastJointStateUpdate;
        bool active;
        bool publish;
        bool publishCompliance;
        bool publishOdometry;
    } control;

    robotik::RobotInteraction interaction;
    robotik::JointCalibration calibration;

    // detect heading change
    double lastHeading;

    // true if the environment is part of simulation
    bool is_simulation;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_robot_description_;
    void robot_description_callback(std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    void joint_states_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    // callback when the user interacts with us in RViz2
    void processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
    void menuCommand(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
    void manipulateMenu(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

    //
    // Limb menu controls
    //
    typedef struct {
        MenuHandler::EntryHandle root;
        MenuHandler::EntryHandle all;
        MenuHandler::EntryHandle legs;
        MenuHandler::EntryHandle arms;
        MenuHandler::EntryHandle first;
        MenuHandler::EntryHandle last;
    } LimbControls;

    void add_limb_controls_menu(LimbControls& controls, const char* menu_name, double effort_value);
    void enableLimbMenu(const LimbControls& controls, double e, visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr m);

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
    robot_model_msgs::msg::CompliantJointParams::SharedPtr compliance_params_msg_;
    robot_model_msgs::msg::JointCalibration::SharedPtr calibration_msg_;
    rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::CompliantJointParams>::SharedPtr compliance_params_pub_;
    rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::JointCalibration>::SharedPtr calibration_pub_;

    void publishCalibration(const robotik::JointCalibration::Data& jd, bool commit = false);
    void calibrateToSelectedPost(srdf::Model::GroupState groupState, visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

    tf2_ros::Buffer tfBuffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_static_;
    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);        // callback

    // IMU sense
    sensor_msgs::msg::Imu imu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu);

    // marker message publisher
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // simpler access to special markers
    using MarkerPtr = visualization_msgs::msg::MarkerArray::_markers_type::iterator;

    // odometry publisher
    nav_msgs::msg::Odometry::SharedPtr odometry_msg_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    struct {
        MenuHandler handler;
        MenuHandler::EntryHandle active;
        MenuHandler::EntryHandle resetSim;
        MenuHandler::EntryHandle zeroOdometry;
        MenuHandler::EntryHandle resetTarget;
        MenuHandler::EntryHandle resetTrajectory;
        MenuHandler::EntryHandle saveCurrentState;
        MenuHandler::EntryHandle saveTargetState;

        struct {
            MenuHandler::EntryHandle root;
            MenuHandler::EntryHandle balance;
            MenuHandler::EntryHandle contactLocalizer;
            MenuHandler::EntryHandle publishOdometry;
        } features;

        LimbControls enable_limbs;
        LimbControls disable_limbs;

        // todo: move this to a LimbControls struct (so some members may not be used, so way)
        struct {
            MenuHandler::EntryHandle root;
            MenuHandler::EntryHandle clear;
            MenuHandler::EntryHandle all;
            MenuHandler::EntryHandle base;
            MenuHandler::EntryHandle legs;
            MenuHandler::EntryHandle arms;
            MenuHandler::EntryHandle legsAndBase;
            MenuHandler::EntryHandle armsAndBase;
            MenuHandler::EntryHandle first;
            MenuHandler::EntryHandle last;
        } manipulate;

        // Robot Dynamics sub menu
        struct {
            MenuHandler::EntryHandle root;
            MenuHandler::EntryHandle showCurrentStateMesh;
            MenuHandler::EntryHandle executeTrajectory;
            MenuHandler::EntryHandle activate;
            MenuHandler::EntryHandle deactivate;
        } robotDynamics;

        // Joint Controller sub menu
        struct {
            MenuHandler::EntryHandle root;
            MenuHandler::EntryHandle activate;
            MenuHandler::EntryHandle deactivate;
            MenuHandler::EntryHandle publish;
            MenuHandler::EntryHandle publishCompliance;

            MenuHandler::EntryHandle calibrateRoot;
            MenuHandler::EntryHandle calibrate;
            MenuHandler::EntryHandle calibrateCommit;
            MenuHandler::EntryHandle calibrateAbort;
            MenuHandler::EntryHandle calibrateContinuousSync;
        } jointController;
    } menu;
};


