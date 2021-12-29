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

    const char* TF_TOPIC_PARAMETER = "tf_topic";
    const char* JOINT_STATE_TOPIC_PARAMETER = "joint_state_topic";
    const char* MODEL_STATE_TOPIC_PARAMETER = "model_state_topic";


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

    declare_parameter(JOINT_STATE_TOPIC_PARAMETER, rclcpp::ParameterValue("joint_states"));
    declare_parameter(MODEL_STATE_TOPIC_PARAMETER, rclcpp::ParameterValue("robot_dynamics/model_state"));
    declare_parameter(TF_TOPIC_PARAMETER, rclcpp::ParameterValue("tf"));

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
  current.control->executeTrajectory = false;

  current.state = std::make_shared<robotik::State>(*model_);

  // start listening for joint state updates
  if(!joint_state_listener)
    joint_state_listener = std::make_shared<robotik::JointStateListener>(
            *this,
            get_parameter(JOINT_STATE_TOPIC_PARAMETER).get_value<std::string>());
  joint_state_listener->state(current.state);

  if(!model_state_listener)
      model_state_listener = std::make_shared<robotik::ModelStateListener>(
              *this,
              get_parameter(MODEL_STATE_TOPIC_PARAMETER).get_value<std::string>(),
              "odom");
  model_state_listener->model(model_);
  model_state_listener->state(current.state);


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

    // extended joint compliance parameters
    control_state_pub_ = this->create_publisher<robot_model_msgs::msg::ControlState>(
            "robot_dynamics/control_state",
            10);
    control_state_msg_ = std::make_shared<robot_model_msgs::msg::ControlState>();

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

    joint_state_listener.reset();
    model_state_listener.reset();

    control_state_pub_.reset();
    control_state_msg_.reset();

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

    joint_state_listener.reset();
    model_state_listener.reset();

    control_state_pub_.reset();
    control_state_msg_.reset();

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

    current.control->activate(model_, *this);

    control_state_pub_->on_activate();

    return rv;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "deactivating robot dynamics");

    current.control->deactivate();
    model_->on_deactivate();
    control_state_pub_->on_deactivate();

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

        // if we dont get the segment state from /tf and /tf_static we can
        // compute it using our model information
        // if (model_->compute_TF_CoM(*current.state)) {
        // }

        current.state->lastSegmentStateUpdate = _now;

        if(current.state && current.control->update(*current.state, _now)) {
            // send target state to (typically) ros2 controls
            current.control->publish_control();
        }
    } catch(robotik::Exception& e) {
        RCLCPP_INFO(get_logger(), e.what());
    }
}

void set_effector_msg(robot_model_msgs::msg::EffectorState& effector, const KDL::Frame& current_pose, const KDL::Frame& target_pose) {
    kdl_frame_to_pose(current_pose, effector.pose);
    kdl_frame_to_pose(target_pose, effector.target);
    KDL::Vector dv = target_pose.p - current_pose.p;
    effector.error = std::sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z() * dv.z());
    effector.velocity.linear = geometry_msgs::msg::Vector3();
    effector.velocity.angular = geometry_msgs::msg::Vector3();
}

// when there is no target active
void set_effector_msg(robot_model_msgs::msg::EffectorState& effector, const KDL::Frame& current) {
    kdl_frame_to_pose(current, effector.pose);
    effector.velocity.linear = geometry_msgs::msg::Vector3();
    effector.velocity.angular = geometry_msgs::msg::Vector3();
    effector.target = effector.pose;
    effector.error = 0;
}

void Control::publish_control_state(const robotik::State& current, const robotik::State& target, rclcpp::Time now, std::string prefix)
{
    bool controlling = true;

    if(!control_state_msg_ || !control_state_pub_->is_activated())
        return;

    if(!prefix.empty() && prefix.back() != '/' && !current.relativeFrameName.empty()) {
        prefix += '/';
    }

    control_state_msg_->header.frame_id = prefix + current.relativeFrameName;
    control_state_msg_->header.stamp = now;

    KDL::Frame current_base_tf;
    if(!current.findTF(model_->base_link, current_base_tf)) {
        RCLCPP_WARN_ONCE(get_logger(), "failed to publish control state because state contains no base_link");
        return;
    }

    KDL::Frame target_base_tf;
    if(target.findTF(model_->base_link, target_base_tf)) {
        controlling = false;
        set_effector_msg(control_state_msg_->base, current_base_tf, target_base_tf);
    } else
        set_effector_msg(control_state_msg_->base, current_base_tf);


    auto limb_count = (short)current.limbs.size();

    // figure out what limbs are supporting
    std::vector<bool> supporting(limb_count, false);
    for(const auto& contact : current.contacts) {
        if(contact.limb > 0 && contact.limb < limb_count)
            supporting[contact.limb] = true;
    }

    // limbs
    if(limb_count != (short)control_state_msg_->limbs.size())
        control_state_msg_->limbs.resize(limb_count);
    for(short i=0; i < limb_count; i++) {
        auto& ml = control_state_msg_->limbs[i];
        auto& sl = current.limbs[i];
        ml.name = model_->limbs[i]->options_.to_link;
        ml.mode = sl.mode;
        ml.type = sl.limbType;
        ml.supportive = sl.supportive;
        ml.supporting = supporting[i];

        kdl_frame_to_pose(sl.position, ml.effector.pose);
        // todo: add velocity to model state
        //ml.velocity = tf2::toMsg(tf2::Stamped(sl.velocity, model_state_msg_->header.stamp, model_state_msg_->header.frame_id));

        KDL::Frame current_limb_tf;
        if(current.findTF(ml.name, current_limb_tf)) {
            // make limb_tf relative to robot body
            current_limb_tf = current_base_tf.Inverse() * current_limb_tf;

            kdl_frame_to_pose(current_limb_tf, ml.effector.target);
        } else {
            ml.effector.target = ml.effector.pose;
            ml.effector.error = 0;
        }

        KDL::Frame target_limb_tf;
        if(controlling && target.findTF(ml.name, target_limb_tf)) {
            target_limb_tf = target_base_tf.Inverse() * target_limb_tf;
            set_effector_msg(ml.effector, current_limb_tf, target_limb_tf);
        } else
            set_effector_msg(ml.effector, current_limb_tf);
    }

    control_state_pub_->publish(*control_state_msg_);
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
    int updates = 0;
    builtin_interfaces::msg::Time now;
    if(model_ && current.state) {
        auto segments = model_->tree_->getSegments();

        for (auto i = 0u; i < msg->transforms.size(); i++) {
            auto& tfx = msg->transforms[i];
            auto frame_id = tfx.child_frame_id;
            now = tfx.header.stamp;
            if(!frame_id.empty() && frame_id[0]=='/')
                frame_id = frame_id.substr(1);

            // does the transform have a prefix
            //std::string prefix;
            auto slash_itr = frame_id.find('/');
            if(slash_itr != std::string::npos) {
                // for now any prefix would indicate TF other than robot state so we ignore
                continue;
#if 0
                prefix = frame_id.substr(0, slash_itr-1);
                frame_id = frame_id.substr(slash_itr);
#endif
            }

            current.state->tf[frame_id] = tf2::transformToKDL(tfx);
            updates++;
        }

        if(updates > 0)
            current.state->lastSegmentStateUpdate = rclcpp::Time(now);
    }
}

void Control::publish() try {
    //RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");
    rclcpp::Time _now = now();

    // ensure we have state to publish
    if(!current.state)
        return;

    if(model_->tree_ && !model_->limbs.empty()
                && current.state->lastJointStateUpdate.get_clock_type()==RCL_ROS_TIME
                && current.state->lastSupportStateUpdate.get_clock_type()==RCL_ROS_TIME) {
        auto sinceJoints = _now - current.state->lastJointStateUpdate;
        auto sinceModel = _now - current.state->lastSupportStateUpdate;

        if(sinceJoints.seconds() > 1.0 || sinceModel.seconds() > 1.0) {
            // todo: we are no longer getting joint updates, we shouldnt control either
            return;
        }

        updateRobotState();

        publish_control_state(*current.state, *current.control->getTargetState(), _now);
        lastControlUpdate = _now;

        // todo: setup a timer to publish preview less often as control
        current.control->publish_target_preview(_now);
        current.control->publish_trajectory_preview(_now);
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

