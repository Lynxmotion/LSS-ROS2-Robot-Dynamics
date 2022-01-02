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

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_dynamics::Dynamics)

namespace  robot_dynamics {

using namespace std::chrono_literals;


Dynamics::Dynamics()
        : Dynamics("robot_dynamics") {
}

Dynamics::Dynamics(const rclcpp::NodeOptions & options)
        : Dynamics("robot_dynamics", options) {
}


Dynamics::Dynamics(
        const std::string & node_name,
        const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, options),
    is_simulation(false),
    tfBuffer(get_clock())
{
    tfBuffer.setUsingDedicatedThread(true);

    declare_parameter("frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("model_state_frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)5));
    declare_parameter("self_manage", rclcpp::ParameterValue(false));
    declare_parameter("joint_controller", rclcpp::ParameterValue("lss_joint_controller"));
    declare_parameter("effort_controller", rclcpp::ParameterValue("/effort_controller/commands"));
    //declare_parameter("joint_names", rclcpp::ParameterValue(std::vector<std::string>()));
    declare_parameter("sim_mode", rclcpp::ParameterValue(false));

    // declare as optional
    declare_parameter(LEG_SUPPORT_DISTANCE_PARAM, rclcpp::PARAMETER_DOUBLE);

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
    old_parameter_set_callback = add_on_set_parameters_callback(std::bind(&Dynamics::parameter_set_callback, this, std::placeholders::_1));
}

void Dynamics::robot_description_callback(std_msgs::msg::String::SharedPtr msg)
{
    // todo: SRDF must be loaded via topic!!
#if 0
  std::string urdf_base_path = "/home/guru/src/lss-humanoid/ros2/humanoid/src/lss_humanoid/urdf/";
  model_->setupURDF(
      msg->data,
      urdf_base_path + "lss_humanoid.srdf"
  );
#else
  std::string urdf_base_path = "/home/guru/src/lss-ros2/lss/lss_hexapod/urdf/";
  model_->setupURDF(
          msg->data,
          urdf_base_path + "lss_hexapod.srdf"
          );
#endif
  auto numberOfJoints = model_->tree_->getNrOfJoints();

  // resize some messages
  compliance_params_msg_->joints = model_->getJoints();
  compliance_params_msg_->gravity.resize(numberOfJoints);

  ///
  /// configure dynamics for new model
  ///
  control.active = true;
  control.publishOdometry = false;
  control.publishCompliance = true;
  current.state = std::make_shared<robotik::State>(*model_);

  // we can activate now if we havent already
  if (get_parameter("self_manage").get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Self-transitioning to ACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "configuring robot dynamics");

    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, true);
    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, shared_from_this(), false, 10, 10);

#if 1
    // load URDF from parameter server
    subscription_robot_description_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description",
        rclcpp::QoS(1).transient_local(),
        std::bind(&Dynamics::robot_description_callback, this, std::placeholders::_1)
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
    callback_t cb = std::bind(&Dynamics::tf_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(&Dynamics::tf_callback, this, std::placeholders::_1, true);
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
    compliance_params_pub_ = this->create_publisher<robot_model_msgs::msg::CompliantJointParams>(
            "compliance/params",
            10);
    compliance_params_msg_ = std::make_shared<robot_model_msgs::msg::CompliantJointParams>();

    auto frequency = get_parameter("frequency").get_value<float>();
    RCLCPP_INFO(get_logger(), "robot dynamics set to %4.2fhz", frequency);
    update_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / frequency)),
            std::bind(&Dynamics::publish, this));

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
            std::bind(&Dynamics::publish_model_state, this));

    rclcpp::Parameter diagnostic_period = get_parameter("diagnostic_period");
    diag_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<rcl_duration_value_t>(
                            diagnostic_period.get_value<rcl_duration_value_t>())),
            std::bind(&Dynamics::publish_diagnostics, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_cleanup(const rclcpp_lifecycle::State &)
{
    // free our channels
    update_timer_.reset();
    diag_timer_.reset();

    subscription_imu_.reset();

    model_->clear();

    compliance_params_msg_.reset();
    compliance_params_pub_.reset();

    odometry_pub_.reset();
    odometry_msg_.reset();

    change_state_client_.reset();
    change_state_request_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "shutting down robot dynamics");

    update_timer_.reset();
    diag_timer_.reset();
    //_joint_state_pub.reset();

    subscription_imu_.reset();

    compliance_params_msg_.reset();
    compliance_params_pub_.reset();

    odometry_pub_.reset();
    odometry_msg_.reset();

    change_state_client_.reset();
    change_state_request_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "activating robot dynamics");
    CallbackReturn rv = CallbackReturn::SUCCESS;

    // subscribe to joint state messages
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&Dynamics::joint_states_callback, this, std::placeholders::_1));

    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&Dynamics::imu_callback, this, std::placeholders::_1));


    // odometry publisher
    odometry_pub_->on_activate();

    model_->on_activate(*this);

    // todo: size the fields we will set in the output message
    // joint_trajectory_msg_->gravity.resize(joint_trajectory_msg_->joints.size());

    compliance_params_pub_->on_activate();

    if(is_simulation)
      resetSim();
    return rv;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "deactivating robot dynamics");

    model_->on_deactivate();

    // odometry publisher
    odometry_pub_->on_deactivate();

    // free joint publisher objects
    joint_state_subscription_.reset();

    subscription_imu_.reset();

    compliance_params_pub_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Dynamics::on_error(const rclcpp_lifecycle::State &)
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

void Dynamics::resetSim()
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

void Dynamics::updateRobotState()
{
    auto _now = now();

    try {
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

rcl_interfaces::msg::SetParametersResult Dynamics::parameter_set_callback(const std::vector<rclcpp::Parameter> & params) {
    for(auto& p: params) {
        auto name = p.get_name();
        //bool isNumber = p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER;

        if(name == "joint_names") {
            // todo: what do we do with the joints?
            //current.control->set_joints(p.get_value<std::vector<std::string>>());
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

void Dynamics::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void Dynamics::tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool /* is_static */)
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

void Dynamics::imu_callback(sensor_msgs::msg::Imu::SharedPtr imu)
{
    model_->senseIMU(*current.state, *imu);
}

const double standHeightMin = 0.153;
const double standHeightMax = 0.271;

void Dynamics::publish() try {
    //RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");
    rclcpp::Time stamp = now();

    // ensure we have state to publish
    if(!current.state)
        return;

    if(model_->tree_ && !model_->limbs.empty() && control.lastJointStateUpdate.get_clock_type()==RCL_ROS_TIME) {
        auto since = stamp - control.lastJointStateUpdate;
        if(since.seconds() > 1.0) {
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


void Dynamics::publish_model_state() try {
    if(current.state)
        model_->publishModelState(*current.state, now());
} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish model state: %s", e.what());
    deactivate();
}

void Dynamics::publish_diagnostics() try {

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}

} //ns: robot_dynamics