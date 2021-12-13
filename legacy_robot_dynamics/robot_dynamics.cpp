#include <cstdio>

#include "rclcpp/rclcpp.hpp"


//
// Created by guru on 12/27/19.
//

#include "robot_dynamics.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <conversions.h>

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(RobotDynamics)

using namespace std::chrono_literals;


RobotDynamics::RobotDynamics()
        : RobotDynamics("robot_dynamics") {
}

RobotDynamics::RobotDynamics(const rclcpp::NodeOptions & options)
        : RobotDynamics("robot_dynamics", options) {
}


RobotDynamics::RobotDynamics(
        const std::string & node_name,
        const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, options),
    server("~/interactive",
           get_node_base_interface(),
           get_node_clock_interface(),
           get_node_logging_interface(),
           get_node_topics_interface(),
           get_node_services_interface()
    ),
    control_position(12),
    lastHeading(0),
    is_simulation(false),
    tfBuffer(get_clock())
{
    tfBuffer.setUsingDedicatedThread(true);

    declare_parameter("frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("model_state_frequency");
    declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)5));
    declare_parameter("self_manage", rclcpp::ParameterValue(false));
    declare_parameter("joint_controller", rclcpp::ParameterValue("lss_joint_controller"));
    declare_parameter("effort_controller", rclcpp::ParameterValue("/effort_controller/commands"));
    declare_parameter("joint_names", rclcpp::ParameterValue(std::vector<std::string>()));
    declare_parameter("sim_mode", rclcpp::ParameterValue(false));
    declare_parameter(LEG_SUPPORT_DISTANCE_PARAM);

    if (get_parameter("sim_mode").get_value<bool>()) {
      RCLCPP_INFO(get_logger(), "Simulation mode active");
      is_simulation = true;
    }

    if (get_parameter("self_manage").get_value<bool>()) {
        change_state_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
                std::string(get_name()) + "/change_state",
                rmw_qos_profile_services_default,
                nullptr);

        RCLCPP_INFO(get_logger(), "Self-transitioning to CONFIGURE");
        change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        change_state_request_->transition.label = "";
        change_state_future_ = change_state_client_->async_send_request(change_state_request_);
    }

    model_ = std::make_shared<robotik::Model>();

    // intercept parameter changes
    old_parameter_set_callback = add_on_set_parameters_callback(std::bind(&RobotDynamics::parameter_set_callback, this, std::placeholders::_1));
}

void RobotDynamics::robot_description_callback(std_msgs::msg::String::SharedPtr msg)
{
#if 0
  std::string urdf_base_path = "/home/guru/src/lss-humanoid/ros2/humanoid/src/lss_humanoid/urdf/";
  model_->setupURDF(
      msg->data,
      urdf_base_path + "lss_humanoid.srdf"
  );
#else
  std::string urdf_base_path = "/home/guru/src/lss-humanoid/ros2/humanoid/src/lss_hexapod/urdf/";
  model_->setupURDF(
          msg->data,
          urdf_base_path + "lss_hexapod.srdf"
          );
#endif
  auto numberOfJoints = model_->tree_->getNrOfJoints();

  if(has_parameter(LEG_SUPPORT_DISTANCE_PARAM)) {
    auto supportDistance = get_parameter(LEG_SUPPORT_DISTANCE_PARAM).as_double();
    RCLCPP_INFO(get_logger(), LEG_SUPPORT_DISTANCE_PARAM ": overriding URDF value with %4.2f ", supportDistance);
    for (auto &limb: model_->limbs) {
      if(limb->options_.model == robotik::Limb::Leg)
        limb->supportDistance = supportDistance;
    }
  }

  // resize some messages
  compliance_params_msg_->joints = model_->getJoints();
  compliance_params_msg_->gravity.resize(numberOfJoints);

  ///
  /// configure dynamics for new model
  ///
  current.control = std::make_shared<robotik::Control>();
  current.calibrateMode = false;
  current.calibrateContinuousSync = false;
  current.control->set_joints(get_parameter("joint_names").get_value<std::vector<std::string>>());
  current.control->activate(model_, *this);
  control.active = true;
  control.publish = true;
  control.publishOdometry = false;
  control.publishCompliance = true;
  current.control->executeTrajectory = false;

  current.state = std::make_shared<robotik::State>(*model_);

  current.visual = std::make_shared<robotik::StateVisual>("current_pose");
  current.visual->ghostColor = robotik::Color::Purple.withAlpha(0.3);
  current.visual->configure(model_);
  current.visual->visibility(0, robotik::TF);     // dont show current state mesh since we already do in Robot Model visual

  current.calibrationVisual = std::make_shared<robotik::StateVisual>("calibration");
  current.calibrationVisual->ghostColor = robotik::Color::Maroon.withAlpha(0.6);
  current.calibrationVisual->configure(model_);
  current.calibrationVisual->visibility(robotik::TF, 0xffffffff);     // only show segments

  interaction.configure(*this, model_, "~/interaction");
  interaction.setController("target", *current.control);
  //interaction.setController("current", this? );
  interaction.setController("calibration", calibration);


  // we can activate now if we havent already
  if (get_parameter("self_manage").get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Self-transitioning to ACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "configuring robot dynamics");

    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, true);
    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, shared_from_this(), false, 10, 10);

#if 1
    // load URDF from parameter server
    subscription_robot_description_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description",
        rclcpp::QoS(1).transient_local(),
        std::bind(&RobotDynamics::robot_description_callback, this, std::placeholders::_1)
    );
#else
    std::string urdf_base_path = "/home/guru/src/lss-humanoid/ros2/humanoid/src/lss_humanoid/urdf/";
    model_->setupURDF(
            urdf_base_path + "lss_humanoid.urdf",
            urdf_base_path + "lss_humanoid.srdf"
        );
    auto numberOfJoints = model_->tree_->getNrOfJoints();

    if(has_parameter(LEG_SUPPORT_DISTANCE_PARAM)) {
        auto supportDistance = get_parameter(LEG_SUPPORT_DISTANCE_PARAM).as_double();
        RCLCPP_INFO(get_logger(), LEG_SUPPORT_DISTANCE_PARAM ": overriding URDF value with %4.2f ", supportDistance);
        for (auto &limb: model_->limbs) {
            if(limb->options_.model == robotik::Limb::Leg)
                limb->supportDistance = supportDistance;
        }
    }
#endif

    using callback_t = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;
    callback_t cb = std::bind(&RobotDynamics::tf_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(&RobotDynamics::tf_callback, this, std::placeholders::_1, true);
    subscription_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        10,
        std::move(cb)
    );
    subscription_tf_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static",
        10,
        std::move(static_cb)
    );

    // odometry publisher
    odometry_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_raw",
            10);

    // extended joint compliance parameters
    compliance_params_pub_ = this->create_publisher<humanoid_model_msgs::msg::CompliantJointParams>(
            "compliance/params",
            10);
    compliance_params_msg_ = std::make_shared<humanoid_model_msgs::msg::CompliantJointParams>();

    // calibration message and publisher
    calibration_msg_ = std::make_shared<humanoid_model_msgs::msg::JointCalibration>();
    calibration_pub_ = this->create_publisher<humanoid_model_msgs::msg::JointCalibration>(
            "joint_calibration",
            10);

    // marker publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/visuals",      // was ~/target/visuals
            20);

    auto frequency = get_parameter("frequency").get_value<float>();
    RCLCPP_INFO(get_logger(), "robot dynamics set to %4.2fhz", frequency);
    update_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / frequency)),
            std::bind(&RobotDynamics::publish, this));

    double model_state_frequency = frequency;
    if(has_parameter("model_state_frequency")) {
        const auto &param = get_parameter("model_state_frequency");
        if (param.get_type() == rclcpp::PARAMETER_DOUBLE)
            model_state_frequency = param.get_value<float>();
        else if (param.get_type() == rclcpp::PARAMETER_INTEGER)
            model_state_frequency = (double)param.get_value<int64_t>();
        else
            RCLCPP_ERROR(get_logger(), "parameter model_state_frequency must be a number");
    }
    RCLCPP_INFO(get_logger(), "robot dynamics model state set to %4.2fhz", model_state_frequency);
    model_state_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / model_state_frequency)),
            std::bind(&RobotDynamics::publish_model_state, this));

    rclcpp::Parameter diagnostic_period = get_parameter("diagnostic_period");
    diag_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<rcl_duration_value_t>(
                            diagnostic_period.get_value<rcl_duration_value_t>())),
            std::bind(&RobotDynamics::publish_diagnostics, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_cleanup(const rclcpp_lifecycle::State &)
{
    // free our channels
    update_timer_.reset();
    diag_timer_.reset();

    subscription_imu_.reset();

    current.control->deactivate();
    interaction.cleanup();

    model_->clear();

    current.visual->cleanup();
    current.calibrationVisual->cleanup();

    compliance_params_msg_.reset();
    compliance_params_pub_.reset();

    calibration_msg_.reset();
    calibration_pub_.reset();

    odometry_pub_.reset();
    odometry_msg_.reset();

    change_state_client_.reset();
    change_state_request_.reset();

    marker_pub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "shutting down robot dynamics");

    current.control->deactivate();

    update_timer_.reset();
    diag_timer_.reset();
    //_joint_state_pub.reset();

    subscription_imu_.reset();

    interaction.cleanup();

    compliance_params_msg_.reset();
    compliance_params_pub_.reset();

    calibration_msg_.reset();
    calibration_pub_.reset();

    odometry_pub_.reset();
    odometry_msg_.reset();

    current.visual->cleanup();
    current.calibrationVisual->cleanup();

    change_state_client_.reset();
    change_state_request_.reset();

    marker_pub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "activating robot dynamics");
    CallbackReturn rv = CallbackReturn::SUCCESS;

    // subscribe to joint state messages
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&RobotDynamics::joint_states_callback, this, std::placeholders::_1));

    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&RobotDynamics::imu_callback, this, std::placeholders::_1));


    // odometry publisher
    odometry_pub_->on_activate();

    model_->on_activate(*this);

    interaction.activate();

    // todo: size the fields we will set in the output message
    // joint_trajectory_msg_->gravity.resize(joint_trajectory_msg_->joints.size());

    compliance_params_pub_->on_activate();
    calibration_pub_->on_activate();

    // free visualization markers
    marker_pub_->on_activate();

    current.visual->activate();
    current.calibrationVisual->activate();

    // activate interactive markers
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = model_->base_link;
    //int_marker.header.stamp = now();
    int_marker.name = "stand-z";
    int_marker.description = "Control the robot standing height";
    int_marker.scale = 0.30;
    int_marker.pose.position.x = 0.05;
    //int_marker.pose.position.z = model_->baseHeight;
    int_marker.pose.position.z = 0.06;


    // create a cylinder marker
    visualization_msgs::msg::Marker box_marker;
    //box_marker.header.frame_id = base_link;
    box_marker.type = visualization_msgs::msg::Marker::SPHERE;
    box_marker.scale.x = 0.01;
    box_marker.scale.y = 0.01;
    box_marker.scale.z = 0.01;
    box_marker.color.r = 0.;
    box_marker.color.g = 0.5;
    box_marker.color.b = 1.0;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.name = "base_button";
    box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

#if 0
    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl rotate_control;
    rotate_control.name = "stand-z-rotate";
    rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    rotate_control.orientation.x = 0.;
    rotate_control.orientation.y = 0.7071068;
    rotate_control.orientation.z = 0.;
    rotate_control.orientation.w = 0.7071068;

    // add the control to the interactive marker
    int_marker.controls.push_back(rotate_control);
#endif


    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker,
        std::bind(&RobotDynamics::processFeedback, this, std::placeholders::_1)
        );

#if 1
    // Robot Dynamics menu
    menu.active = menu.handler.insert( "Active",
            std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.active, control.active ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );

    if(is_simulation) {
        menu.resetSim = menu.handler.insert("Reset Sim",
            std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1));
    } else
        menu.resetSim = 0;
    menu.zeroOdometry = menu.handler.insert( "Zero Odometry",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    menu.resetTarget = menu.handler.insert( "Reset Target",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    menu.resetTrajectory = menu.handler.insert( "Reset Trajectory",
                                            std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    menu.saveCurrentState = menu.handler.insert( "Save Current State",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.saveTargetState = menu.handler.insert( "Save Target State",
                       std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    // Features menu
    menu.features.root = menu.handler.insert( "Features" );
    menu.features.balance = menu.handler.insert(menu.features.root, "Balance",
                                        std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.features.contactLocalizer = menu.handler.insert( menu.features.root, "Contact Localizer",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.features.publishOdometry = menu.handler.insert( menu.features.root, "Publish Odometry",
              std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.features.balance,
                                current.control->balance ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
    menu.handler.setCheckState( menu.features.contactLocalizer,
                                model_->use_contact_localizer ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
    menu.handler.setCheckState( menu.features.publishOdometry,
                                control.publishOdometry ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );

    // Manipulate menu
    menu.manipulate.root = menu.handler.insert( "Manipulate" );
    menu.manipulate.clear = menu.handler.insert(menu.manipulate.root, "Clear",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.all = menu.handler.insert(menu.manipulate.root, "All limbs",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.base = menu.handler.insert(menu.manipulate.root, "Base",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.legs = menu.handler.insert(menu.manipulate.root, "Legs",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.legsAndBase = menu.handler.insert(menu.manipulate.root, "Legs + Base",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.arms = menu.handler.insert(menu.manipulate.root, "Arms",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    menu.manipulate.armsAndBase = menu.handler.insert(menu.manipulate.root, "Arms + Base",
            std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1) );
    bool first = true;
    for(auto &limb: model_->limbs) {
        menu.manipulate.last = menu.handler.insert(menu.manipulate.root, limb->options_.to_link,
              std::bind(&RobotDynamics::manipulateMenu, this, std::placeholders::_1));
        if(first) {
            first = false;
            menu.manipulate.first = menu.manipulate.last;
        }
    }

    add_limb_controls_menu(menu.enable_limbs, "Enable Limbs", 1.0);
    add_limb_controls_menu(menu.disable_limbs, "Disable Limbs", 0.0);

    menu.robotDynamics.root = menu.handler.insert( "Robot Dynamics" );
    menu.robotDynamics.activate = menu.handler.insert( menu.robotDynamics.root, "Activate",
            std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.robotDynamics.deactivate = menu.handler.insert( menu.robotDynamics.root, "Deactivate",
            std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    menu.robotDynamics.executeTrajectory = menu.handler.insert( menu.robotDynamics.root, "Execute Trajectory",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.robotDynamics.executeTrajectory,
            current.control->executeTrajectory ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );

    menu.robotDynamics.showCurrentStateMesh = menu.handler.insert( menu.robotDynamics.root, "Show Current State Mesh",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.robotDynamics.showCurrentStateMesh,
            (current.visual->visibility() & robotik::TF) ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );

    // joint controller menu
    menu.jointController.root = menu.handler.insert( "Joint Controller" );
    menu.jointController.publish = menu.handler.insert( menu.jointController.root, "Publish",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.jointController.publish,
            control.publish ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
    menu.jointController.publishCompliance = menu.handler.insert( menu.jointController.root, "Publish Compliance",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setCheckState( menu.jointController.publishCompliance,
            control.publishCompliance ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
    menu.jointController.activate = menu.handler.insert( menu.jointController.root, "Activate",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.jointController.deactivate = menu.handler.insert( menu.jointController.root, "Deactivate",
           std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );

    // Calibration menu
    menu.jointController.calibrateRoot = menu.handler.insert("Calibrate");
    menu.jointController.calibrate = menu.handler.insert( menu.jointController.calibrateRoot, "Manual",
                      std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.jointController.calibrateContinuousSync = menu.handler.insert( menu.jointController.calibrateRoot, "Synchronize",
                      std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
menu.jointController.calibrateAbort = menu.handler.insert( menu.jointController.calibrateRoot, "Abort",
                      std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.jointController.calibrateCommit = menu.handler.insert( menu.jointController.calibrateRoot, "Commit",
                      std::bind(&RobotDynamics::menuCommand, this, std::placeholders::_1) );
    menu.handler.setVisible(menu.jointController.calibrateCommit, false);
    menu.handler.setVisible(menu.jointController.calibrateAbort, false);
    menu.handler.setVisible(menu.jointController.calibrateContinuousSync, false);

    // add SRDF group_states to calibration options for simple pose selection
    for(auto& gs: model_->srdf_model_->getGroupStates()) {
        menu.handler.insert( menu.jointController.calibrateRoot, gs.name_,
              std::bind(&RobotDynamics::calibrateToSelectedPost, this, gs, std::placeholders::_1) );
    }

    menu.handler.apply( server, "stand-z");
#endif

    // 'commit' changes and send to all clients
    server.applyChanges();

    if(is_simulation)
      resetSim();
    return rv;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "deactivating robot dynamics");

    model_->on_deactivate();

    interaction.deactivate();

    // odometry publisher
    odometry_pub_->on_deactivate();

    // free joint publisher objects
    joint_state_subscription_.reset();

    subscription_imu_.reset();

    compliance_params_pub_->on_deactivate();
    calibration_pub_->on_deactivate();

    current.visual->deactivate();

    // free visualization markers
    marker_pub_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotDynamics::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "robot dynamics has entered error state");

    if (get_parameter("self_manage").get_value<bool>()) {
        RCLCPP_INFO(get_logger(), "Self-transitioning to INACTIVE");
        change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        change_state_request_->transition.label = "";
        change_state_future_ = change_state_client_->async_send_request(change_state_request_);
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RobotDynamics::resetSim()
{
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
      create_client<std_srvs::srv::Empty>("/reset_simulation");
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  while (!client->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  // call reset simulation
  auto result_future = client->async_send_request(request);

#if 0 // todo: figure out how to call services asynchronously while spin is being called in some main executor
  // Wait for the result
  if (result_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "failed to call reset-simulation service");
  } else {
    RCLCPP_INFO(get_logger(), "successfully reset simulation");
  }
#endif
}

std::map<std::string, double> top_efforts;

void RobotDynamics::publishCalibration(const robotik::JointCalibration::Data& jd, bool commit)
{
    // compute the new calibration data
    if(!jd.empty()) {
        // send calibration data
        //std::cout << "recalibrating  ";
        calibration_msg_->commit = commit;
        calibration_msg_->joints.resize(jd.size());
        calibration_msg_->origin.resize(jd.size());
        int i = 0;
        for (auto &o: jd) {
            calibration_msg_->joints[i] = o.first;
            calibration_msg_->origin[i] = o.second;
            //std::cout << ' ' << o.first << ':' << std::setprecision(4) << o.second;
            i++;
        }
        //calibration.clear();
        //std::cout << std::endl;

        calibration_pub_->publish(*calibration_msg_);
    }
}

void RobotDynamics::updateRobotState()
{
    auto _now = now();

    try {
        // override state TF if we are calibrating
        if(current.calibrateMode) {
            if (calibration.hasJointData()) {
                auto calibrationData = calibration.update(*model_, *current.state, *current.calibrationVisual);

                // send calibration markers
                marker_pub_->publish(*current.calibrationVisual->getMarkers());
                interaction.updateManipulators(*calibration.state, "calibration");

                if(current.calibrateContinuousSync)
                    publishCalibration(calibrationData);
            } else {
                interaction.updateManipulators(*current.state, "calibration");
            }
            interaction.apply();
        }

        if (model_->updateState(*current.state)) {
            KDL::Frame tf_base, tf_footprint;

            // update height from ground and robot odometry by publishing to the robot_localization node
            if(control.publishOdometry && odometry_pub_ && current.state->findTF(model_->base_link, tf_base)) {
                // todo: code only works because base is directly connected to odom frame
                odometry_msg_->header.stamp = _now;
                odometry_msg_->header.frame_id = model_->odom_link;
                odometry_msg_->child_frame_id = model_->base_link;
                kdl_frame_to_pose(tf_base, odometry_msg_->pose.pose);
                odometry_msg_->pose.covariance = {
                        0.01,   0.0,   0.0,  0.0,  0.0,  0.0,
                        0.0,  0.01,   0.0,  0.0,  0.0,  0.0,
                        0.0,   0.0,  0.01,  0.0,  0.0,  0.0,
                        0.0,   0.0,   0.0,  0.1,  0.0,  0.0,
                        0.0,   0.0,   0.0,  0.0,  0.1,  0.0,
                        0.0,   0.0,   0.0,  0.0,  0.0,  0.1 };
                odometry_pub_->publish(*odometry_msg_);
            }

#if 0  // transform now sent in Model
            // publish footprint frame to TF
            if(current.state->findTF(model_->footprint_link, tf_footprint)) {
                // convert footprint from state frame (odom local) to base relative
                tf_footprint = tf_base.Inverse() * tf_footprint;
                static_transformStamped.header.stamp = _now;
                static_transformStamped.header.frame_id = model_->base_link;
                static_transformStamped.child_frame_id = model_->footprint_link;
                kdl_frame_to_transform(tf_footprint, static_transformStamped.transform);
                static_broadcaster.sendTransform(static_transformStamped);
            }
#endif

            unsigned int numberOfJoints = model_->tree_->getNrOfJoints();

            // publish current state markers
            if(current.state) {
                current.visual->update(*current.state);
                marker_pub_->publish(*current.visual->getMarkers());
            }

            if(current.calibrateMode)
                return;

            if(current.state && current.control->update(*current.state, _now)) {
                interaction.updateManipulators(*current.control->getTargetState(), "target");

                // send target state to (typically) ros2 controls
                current.control->publish();

                // todo: don't publish this as often
                current.control->publish_progress();

                // publish state visuals
                current.control->publish_visuals(marker_pub_);
            }

            interaction.apply();

            if(control.publishCompliance) {
                // publish compliance parameters
                for (unsigned int j = 0; j < numberOfJoints; j++) {
                    compliance_params_msg_->gravity[j] = current.state->internalForces(j);
                }
                compliance_params_pub_->publish(*compliance_params_msg_);
            }
        }
    } catch(robotik::Exception& e) {
        RCLCPP_INFO(get_logger(), e.what());
    }
}

rcl_interfaces::msg::SetParametersResult RobotDynamics::parameter_set_callback(const std::vector<rclcpp::Parameter> & params) {
    for(auto& p: params) {
        auto name = p.get_name();
        bool isNumber = p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER;

        if(isNumber && name == LEG_SUPPORT_DISTANCE_PARAM) {
            auto supportDistance = p.as_double();
            RCLCPP_INFO(get_logger(), LEG_SUPPORT_DISTANCE_PARAM ": updating with %4.2f ", supportDistance);
            for (auto &limb: model_->limbs) {
                if (limb->options_.model == robotik::Limb::Leg)
                    limb->supportDistance = supportDistance;
            }
        } else if(name == "joint_names") {
            current.control->set_joints(p.get_value<std::vector<std::string>>());
        } else {
            auto res = rcl_interfaces::msg::SetParametersResult();
            res.set__successful(false);
            res.set__reason("parameter not found, or cannot be set at run-time");
        }
    }

    auto res = rcl_interfaces::msg::SetParametersResult();
    res.set__successful(true);
    return res;
}

void RobotDynamics::processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    if(feedback->marker_name.compare("stand-z") ==0) {
        auto _now = now();
        current.control->enableManipulators( *current.state, _now);
        //target.trajectoryAnimationStart = 0;
    }
}

void RobotDynamics::menuCommand(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    auto id = feedback->menu_entry_id;
    auto boolean_feature = [id, this](bool& option) {
        option = !option;
        menu.handler.setCheckState( id, option ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
        menu.handler.reApply(server);
        server.applyChanges();
    };

    if(id == menu.robotDynamics.activate) {
        RCLCPP_INFO(get_logger(), "transitioning to ACTIVE via user menu");
        change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        change_state_future_ = change_state_client_->async_send_request(change_state_request_);
    } else if(id == menu.robotDynamics.deactivate) {
        RCLCPP_INFO(get_logger(), "transitioning to INACTIVE via user menu");
        change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        change_state_future_ = change_state_client_->async_send_request(change_state_request_);
    } else if(id == menu.jointController.activate) {
        RCLCPP_INFO(get_logger(), "transitioning joint controller to ACTIVE via user menu");
        current.control->set_joint_controller_active(true);
    } else if(id == menu.jointController.deactivate) {
        RCLCPP_INFO(get_logger(), "transitioning joint controller to INACTIVE via user menu");
        current.control->set_joint_controller_active(false);
    } else if(id == menu.jointController.publish) {
        control.publish = !control.publish;
        menu.handler.setCheckState( id, control.publish ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
        if(control.publish) {
            // ensure we dont kick into the middle of a trajectory
            current.control->resetTrajectory();
        }
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.jointController.publishCompliance) {
        control.publishCompliance = !control.publishCompliance;
        menu.handler.setCheckState( id, control.publishCompliance ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.features.balance) {
        boolean_feature(current.control->balance);
    } else if(id == menu.jointController.calibrate) {
        if(!current.calibrateMode) {
            RCLCPP_INFO(get_logger(), "entering calibration mode");
            current.calibrateMode = true;
            calibration.begin(model_, *current.state);
            menu.handler.setVisible(menu.jointController.calibrate, false);
            menu.handler.setVisible(menu.jointController.calibrateContinuousSync, true);
            menu.handler.setVisible(menu.jointController.calibrateCommit, true);
            menu.handler.setVisible(menu.jointController.calibrateAbort, true);
        }
        // must clear all on server, but active manipulators will get refreshed fast
        interaction.clearManipulators();
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.jointController.calibrateContinuousSync) {
        current.calibrateContinuousSync = !current.calibrateContinuousSync;
        menu.handler.setCheckState( id, current.calibrateContinuousSync ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.jointController.calibrateCommit || id == menu.jointController.calibrateAbort) {
        // clear the calibration
        bool commit = (id == menu.jointController.calibrateCommit);
        if (commit)
            publishCalibration(calibration.commit(), true);
        menu.handler.setVisible(menu.jointController.calibrate, true);
        menu.handler.setVisible(menu.jointController.calibrateContinuousSync, false);
        menu.handler.setVisible(menu.jointController.calibrateCommit, false);
        menu.handler.setVisible(menu.jointController.calibrateAbort, false);

        // delete the segment indicators
        current.calibrationVisual->remove();
        marker_pub_->publish(*current.calibrationVisual->getMarkers());

        // clear the manipulators
        calibration.clear();
        interaction.clearManipulators();        // must clear all on server, but active manipulators will get refreshed fast
        menu.handler.reApply(server);
        server.applyChanges();

        current.calibrateMode = false;
        RCLCPP_INFO(get_logger(), commit ? "exiting calibration mode w/commit" : "aborting calibration mode");
    } else if(id == menu.features.contactLocalizer) {
        boolean_feature(model_->use_contact_localizer);
    } else if(id == menu.features.publishOdometry) {
        boolean_feature(control.publishOdometry);
    } else if(id == menu.active) {
        control.active = !control.active;
        //MenuHandler::CheckState state;
        //menu.handler.getCheckState(id, state);
        menu.handler.setCheckState( id, control.publish ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.robotDynamics.executeTrajectory) {
        MenuHandler::CheckState checked;
        menu.handler.getCheckState(id, checked);
        current.control->executeTrajectory = !(checked == MenuHandler::CHECKED); // toggle state
        menu.handler.setCheckState( id, current.control->executeTrajectory ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.robotDynamics.showCurrentStateMesh) {
        MenuHandler::CheckState checked;
        menu.handler.getCheckState(id, checked);
        if(checked == MenuHandler::CHECKED) {
            checked = MenuHandler::UNCHECKED;
            current.visual->visibility(0, robotik::TF);
        } else if(checked == MenuHandler::UNCHECKED) {
            checked = MenuHandler::CHECKED;
            current.visual->visibility(robotik::TF, 0);
        }
        menu.handler.setCheckState( id, checked);
        menu.handler.reApply(server);
        server.applyChanges();
    } else if(id == menu.resetSim) {
      resetSim();
    } else if(id == menu.zeroOdometry) {
        // bring the robot back to x,y location 0,0
        KDL::Frame f;
        if(odometry_pub_ && current.state->findTF(model_->base_link, f)) {
            KDL::Frame se_base;
            auto& base = current.state->tf[model_->base_link];
            base.p = KDL::Vector(0, 0, current.state->baseHeightFromFloor());
            if(current.state->findTF("se/"+model_->base_link, se_base)) {
                current.state->tf["se/"+model_->base_link] = base;
            }

            current.state->contacts.clear();

            odometry_msg_->header.stamp = now();
            odometry_msg_->header.frame_id = model_->odom_link;
            odometry_msg_->child_frame_id = model_->base_link;
            f.p.x(0);
            f.p.y(0);
            kdl_vector_to_point(f.p, odometry_msg_->pose.pose.position);
            odometry_msg_->pose.covariance = {
                    1e-9,   0.0,   0.0,  0.0,  0.0,  0.0,
                    0.0,  1e-9,   0.0,  0.0,  0.0,  0.0,
                    0.0,   0.0,  1e-9,  0.0,  0.0,  0.0,
                    0.0,   0.0,   0.0,  1e-9,  0.0,  0.0,
                    0.0,   0.0,   0.0,  0.0,  1e-9,  0.0,
                    0.0,   0.0,   0.0,  0.0,  0.0,  1e-9 };
            odometry_pub_->publish(*odometry_msg_);
        }
    } else if(id == menu.saveCurrentState) {
        KDL::Frame base = current.state->tf[model_->base_link];
        std::cout << current.state->toXmlString("current", "all", base);
    } else if(id == menu.saveTargetState) {
        auto target = current.control->getTargetState();
        if(target) {
            KDL::Frame base = target->tf[model_->base_link];
            std::cout << target->toXmlString("target", "all", base);
        }
    } else if(id == menu.resetTarget) {
        // reset target
        current.control->resetTarget(*current.state);
        current.control->clear_markers(marker_pub_);
#if 0
        // output max efforts
        std::cout << "max-efforts:   ";
        for(auto& ef: top_efforts)
            std::cout << "   " << ef.first << "=" << ef.second;
        std::cout << std::endl;
#endif
    } else if(id == menu.resetTrajectory) {
        // reset target
        current.control->resetTrajectory();
        current.control->clear_trajectory_markers(marker_pub_);
    }
}

void RobotDynamics::manipulateMenu(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr m)
{
    auto id = m->menu_entry_id;
    if(id == menu.manipulate.clear) {
        if(current.control->target)
            current.control->clearManipulators(*current.control->target);
        else
            current.control->clearManipulators();
        interaction.clearManipulators();
    } else if(id == menu.manipulate.all)
        current.control->manipulateAll(true);
    else if(id == menu.manipulate.base)
        current.control->manipulateBase();
    else if(id == menu.manipulate.legs)
        current.control->manipulate(robotik::Limb::DynamicModelType::Leg, false);
    else if(id == menu.manipulate.arms)
        current.control->manipulate(robotik::Limb::DynamicModelType::Arm, false);
    else if(id == menu.manipulate.legsAndBase)
        current.control->manipulate(robotik::Limb::DynamicModelType::Leg, true);
    else if(id == menu.manipulate.armsAndBase)
        current.control->manipulate(robotik::Limb::DynamicModelType::Arm, true);
    else if(id >= menu.manipulate.first && id <= menu.manipulate.first) {
        size_t limb_id = id - menu.manipulate.first;
        assert(limb_id < model_->limbs.size());
        current.control->manipulate({model_->limbs[limb_id]->options_.to_link});
    }
}

void RobotDynamics::add_limb_controls_menu(LimbControls& controls, const char* menu_name, double effort_value) {
    // Manipulate menu
    controls.root = menu.handler.insert( menu_name );
    controls.all = menu.handler.insert(controls.root, "All limbs",
                                                std::bind(&RobotDynamics::enableLimbMenu, this, std::cref(controls), effort_value, std::placeholders::_1) );
    controls.legs = menu.handler.insert(controls.root, "Legs",
                                        std::bind(&RobotDynamics::enableLimbMenu, this, std::cref(controls), effort_value, std::placeholders::_1) );
    controls.arms = menu.handler.insert(controls.root, "Arms",
                                        std::bind(&RobotDynamics::enableLimbMenu, this, std::cref(controls), effort_value, std::placeholders::_1) );
    bool first = true;
    for(auto &limb: model_->limbs) {
        controls.last = menu.handler.insert(controls.root, limb->options_.to_link,
                                            std::bind(&RobotDynamics::enableLimbMenu, this, std::cref(controls), effort_value, std::placeholders::_1));
        if(first) {
            first = false;
            controls.first = controls.last;
        }
    }
}

void RobotDynamics::enableLimbMenu(const LimbControls& controls, double e, visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr m)
{
    auto id = m->menu_entry_id;
    if(id == controls.all)
        current.control->enable_all_joints(e);
    else if(id == controls.legs)
        current.control->enable_legs(e);
    else if(id == controls.arms)
        current.control->enable_arms(e);
    else if(id >= controls.first && id <= controls.first) {
        size_t limb_id = id - controls.first;
        assert(limb_id < model_->limbs.size());
        current.control->enable_limb(*model_->limbs[limb_id], e);
    }
}


void RobotDynamics::calibrateToSelectedPost(srdf::Model::GroupState groupState,
                                            visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr)
{
    /*if(!current.calibrateMode) {
        current.calibrateMode = true;
        calibration.begin(*model_, *current.state);
    }*/
    RCLCPP_INFO_STREAM(get_logger(), "calibrating joints to " << groupState.name_ << " pose");
    robotik::JointCalibration::Data data;
    for(const auto& j: groupState.joint_values_) {
        if(j.second.size() ==1)
            data[j.first] = j.second[0];
        else {
            RCLCPP_INFO_STREAM(get_logger(), "pose calibration: ignoring joint " << j.first << " state since it contains multiple values");
        }
    }
    publishCalibration(data, true);
    calibration.clear();
    interaction.clearManipulators();        // must clear all on server, but active manipulators will get refreshed fast
    current.calibrationVisual->remove();
    marker_pub_->publish(*current.calibrationVisual->getMarkers());
}

void RobotDynamics::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // keep track of all joint positions
    auto positions = msg->position.size();
    auto velocities = msg->velocity.size();
    auto efforts = msg->effort.size();

    unsigned int mask =
            (positions ? robotik::POSITION : 0) |
            (velocities ? robotik::VELOCITY : 0) |
            (efforts ? robotik::EFFORT : 0);
    // todo: can we speed up the update of joints in the state from the joint_state messages? (using an index?)
    auto& state = *current.state;
    for(size_t i=0, _i=msg->name.size(); i<_i; i++) {
        auto n = msg->name[i];
        auto jp = state.addJoint(n, mask);
        if(jp >= 0) {
            state.joints_updated[jp] = true;
            state.position(jp) = (i < positions) ? msg->position[i] : 0;
            state.velocity(jp) = (i < velocities) ? msg->velocity[i] : 0;
            state.effort(jp) = (i < efforts) ? msg->effort[i] : 0;
        }
    }
    control.lastJointStateUpdate = now();
}

void RobotDynamics::tf_callback( tf2_msgs::msg::TFMessage::SharedPtr msg, bool /* is_static */)
{
    if(model_ && current.state) {
        auto segments = model_->tree_->getSegments();

        for (auto i = 0u; i < msg->transforms.size(); i++) {
            auto frame_id = msg->transforms[i].child_frame_id;
            if(!frame_id.empty() && frame_id[0]=='/')
                frame_id = frame_id.substr(1);

            if(frame_id.length() > 3 && frame_id.compare(0, 3, "se/")==0) {
                // todo: this should be transformed to odom-relative (but it already is by default)
                current.state->tf[frame_id] = tf2::transformToKDL(msg->transforms[i]);
            } else if(frame_id == model_->base_link && !model_->use_internal_localization) {
                // we can save these segment transforms since they are not
                // part of the robot
                current.state->tf[frame_id] = tf2::transformToKDL(msg->transforms[i]);
            } else if(frame_id == model_->odom_link) {
                // we can save these segment transforms since they are not
                // part of the robot
                current.state->tf[frame_id] = tf2::transformToKDL(msg->transforms[i]);
            }  else {
                // warning: incoming transforms are local to segment and its parent, so this code was
                // trashing the TF state which is in algo/odom frame.
                // todo: Enable reception of TF state when robot_dynamics is not the TF publisher. The
                //       TF entries are relative to the parent joint so to update the segment state we'd
                //       have to start at base_link and apply transforms to each limb.
                // current.state->tf[frame_id] = tf2::transformToKDL(msg->transforms[i]);
            }
        }
    }
}

void RobotDynamics::imu_callback(sensor_msgs::msg::Imu::SharedPtr imu)
{
    model_->senseIMU(*current.state, *imu);
}

const double standHeightMin = 0.153;
const double standHeightMax = 0.271;

void RobotDynamics::publish() try {
    //RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");
    rclcpp::Time stamp = now();

    // ensure we have state to publish
    if(!current.state)
        return;

#if 0
    // get robot heading
    try {
        auto heading = model_->getRobotRPY(*current.state, model_->odom_link);

        // check if we need to update position of standing handle
        if (current.control->getBaseHeight() < standHeightMin
            || current.control->getBaseHeight() > standHeightMax
            || (std::abs(lastHeading - heading.yaw) > std::numeric_limits<double>::epsilon())
                ) {
            current.control->setBaseHeight(
                    std::max(standHeightMin, std::min(standHeightMax, current.control->getBaseHeight())));
            geometry_msgs::msg::Pose pose;
            auto rot = KDL::Rotation::RPY(0, 0, heading.yaw);
            auto p = rot * KDL::Vector(0, 0.14, current.control->getBaseHeight());

            // add suggested position to moving average to smooth harsh changes
            control_position.add(p);

            // update control position
            auto q = control_position.average();
            pose.position.x = q.x();
            pose.position.y = q.y();
            pose.position.z = p.z();    // always fresh Z since we compute this
            pose.orientation.x = 0.;
            pose.orientation.y = 0.;
            pose.orientation.z = 0.;
            pose.orientation.w = 1.;
            server.setPose("stand-z", pose);
            server.applyChanges();
            lastHeading = heading.yaw;
            //std::cout << "stand-z:  " << control.standHeight << std::endl;
        }
    } catch(const robotik::Exception&) {
        // do nothing, just dont update
    }
#endif

    if(model_->tree_ && !model_->limbs.empty() && control.lastJointStateUpdate.get_clock_type()==RCL_ROS_TIME) {
        auto since = stamp - control.lastJointStateUpdate;
        if(since.seconds() > 1.0) {
            // todo: we are no longer getting joint updates, we shouldnt control either
            return;
        }

        updateRobotState();

        model_->publishTransforms(*current.state, now());
        model_->publishFixedTransforms(now());
    }

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}


void RobotDynamics::publish_model_state() try {
    if(current.state)
        model_->publishModelState(*current.state, now());
} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish model state: %s", e.what());
    deactivate();
}

void RobotDynamics::publish_diagnostics() try {

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}
