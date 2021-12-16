#include <cstdio>

#include "rclcpp/rclcpp.hpp"


//
// Created by guru on 12/27/19.
//

#include "robot_control/robot_control.h"

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
RCLCPP_COMPONENTS_REGISTER_NODE(robot_dynamics::Control)

using namespace std::chrono_literals;

namespace robot_dynamics {

Control::Control()
        : Control("robot_control") {
}

Control::Control(const rclcpp::NodeOptions & options)
: Control("robot_control", options) {
}


Control::Control(
        const std::string & node_name,
        const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, options),
    tfBuffer(get_clock())
{
    tfBuffer.setUsingDedicatedThread(true);

    declare_parameter("frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)5));
    declare_parameter("self_manage", rclcpp::ParameterValue(false));

    declare_parameter("joint_controller", rclcpp::ParameterValue("lss_joint_controller"));
    declare_parameter("effort_controller", rclcpp::ParameterValue("/effort_controller/commands"));

    declare_parameter("joint_state_topic", rclcpp::ParameterValue("joint_states"));
    declare_parameter("model_state_topic", rclcpp::ParameterValue("robot_dynamics/model_state"));
    declare_parameter("tf_topic", rclcpp::ParameterValue("tf"));

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
    old_parameter_set_callback = add_on_set_parameters_callback(std::bind(&Control::parameter_set_callback, this, std::placeholders::_1));
}

void Control::robot_description_callback(std_msgs::msg::String::SharedPtr msg)
{
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

  ///
  /// configure dynamics for new model
  ///
  current.control = std::make_shared<robotik::Control>();
  current.control->activate(model_, *this);
  current.control->executeTrajectory = false;

  current.state = std::make_shared<robotik::State>(*model_);

  // we can activate now if we havent already
  if (get_parameter("self_manage").get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Self-transitioning to ACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "configuring robot dynamics");

    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, true);
    //tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, shared_from_this(), false, 10, 10);

#if 1
    // load URDF from parameter server
    subscription_robot_description_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description",
        rclcpp::QoS(1).transient_local(),
        std::bind(&Control::robot_description_callback, this, std::placeholders::_1)
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
    callback_t cb = std::bind(&Control::tf_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(&Control::tf_callback, this, std::placeholders::_1, true);
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

    auto frequency = get_parameter("frequency").get_value<float>();
    RCLCPP_INFO(get_logger(), "robot dynamics set to %4.2fhz", frequency);
    update_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / frequency)),
            std::bind(&Control::publish, this));

    rclcpp::Parameter diagnostic_period = get_parameter("diagnostic_period");
    diag_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<rcl_duration_value_t>(
                            diagnostic_period.get_value<rcl_duration_value_t>())),
            std::bind(&Control::publish_diagnostics, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_cleanup(const rclcpp_lifecycle::State &)
{
    // free our channels
    update_timer_.reset();
    diag_timer_.reset();

    current.control->deactivate();
    model_->clear();

    change_state_client_.reset();
    change_state_request_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "shutting down robot dynamics");

    update_timer_.reset();
    diag_timer_.reset();

    current.control->deactivate();
    model_->clear();

    change_state_client_.reset();
    change_state_request_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "activating robot dynamics");
    CallbackReturn rv = CallbackReturn::SUCCESS;

    model_->on_activate(*this);

    return rv;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "deactivating robot dynamics");

    model_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_error(const rclcpp_lifecycle::State &)
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

void Control::updateRobotState()
{
    auto _now = now();

    try {
        if (model_->updateState(*current.state)) {
            KDL::Frame tf_base, tf_footprint;

            if(current.state && current.control->update(*current.state, _now)) {
                // send target state to (typically) ros2 controls
                current.control->publish();
            }
        }
    } catch(robotik::Exception& e) {
        RCLCPP_INFO(get_logger(), e.what());
    }
}

rcl_interfaces::msg::SetParametersResult Control::parameter_set_callback(const std::vector<rclcpp::Parameter> & params) {
    for(auto& p: params) {
        auto name = p.get_name();
        //bool isNumber = p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER;

        if(name == "joint_names") {
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

void Control::tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool /* is_static */)
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

const double standHeightMin = 0.153;
const double standHeightMax = 0.271;

void Control::publish() try {
    //RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");
    rclcpp::Time stamp = now();

    // ensure we have state to publish
    if(!current.state)
        return;

    if(model_->tree_ && !model_->limbs.empty() && lastJointStateUpdate.get_clock_type()==RCL_ROS_TIME) {
        auto since = stamp - lastJointStateUpdate;
        if(since.seconds() > 1.0) {
            // todo: we are no longer getting joint updates, we shouldnt control either
            return;
        }

        updateRobotState();
    }

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}

void Control::publish_diagnostics() try {

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}

}  //ns: robot_dynamics

