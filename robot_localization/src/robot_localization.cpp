//
// Created by guru on 1/16/22.
//

#include "robot_localization/robot_localization.h"


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_dynamics::RobotLocalization)

using namespace std::chrono_literals;

namespace robot_dynamics {

const char* MODEL_DESCRIPTION_TOPIC_PARAMETER = "robot_description_topic";
const char* MODEL_STATE_TOPIC_PARAMETER = "model_state_topic";
const char* CONTROL_STATE_TOPIC_PARAMETER = "control_state_topic";


RobotLocalization::RobotLocalization()
: RobotLocalization("robot_localization") {
}

RobotLocalization::RobotLocalization(const rclcpp::NodeOptions & options)
: RobotLocalization("robot_localization", options) {
}


RobotLocalization::RobotLocalization(
        const std::string & node_name,
        const rclcpp::NodeOptions & options
        ) : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    declare_parameter("frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)5));
    declare_parameter(MODEL_DESCRIPTION_TOPIC_PARAMETER, rclcpp::ParameterValue(robotik::RobotDescriptionListener::default_topic_name));
    declare_parameter(MODEL_STATE_TOPIC_PARAMETER, rclcpp::ParameterValue("/robot_dynamics/model_state"));
    declare_parameter(CONTROL_STATE_TOPIC_PARAMETER, rclcpp::ParameterValue("/robot_control/control_state"));

    self_managed = robotik::LifecycleManager::if_self_manage(*this);
}

RobotLocalization::CallbackReturn RobotLocalization::on_configure(const rclcpp_lifecycle::State &)
{
    // subscribe to model state
    auto model_state_topic = get_parameter(MODEL_STATE_TOPIC_PARAMETER).get_value<std::string>();

#if 0
    // subscribe to model state
    auto control_state_topic = get_parameter(CONTROL_STATE_TOPIC_PARAMETER).get_value<std::string>();
    control_state_subscription_ = create_subscription<robot_model_msgs::msg::ControlState>(
            control_state_topic, 10,
            std::bind(&RobotLocalization::control_state_callback, this, std::placeholders::_1));
#endif

    // odometry publisher
    odometry_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom",
            10);

    robot_description_listener = std::make_shared<robotik::RobotDescriptionListener>(
            *this,
            std::bind(&RobotLocalization::robot_model_callback,
                      this, std::placeholders::_1));

    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&RobotLocalization::imu_callback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

void RobotLocalization::robot_model_callback(robotik::Model::SharedPtr model)
{
    model_ = model;

    state = std::make_shared<robotik::State>(*model_);

    if(!model_state_listener)
        model_state_listener = std::make_shared<robotik::ModelStateListener>(
                *this,
                get_parameter(MODEL_STATE_TOPIC_PARAMETER).get_value<std::string>(),
                "odom");
    model_state_listener->model(model_);
    model_state_listener->state(state);

    if(self_managed)
        self_managed->activate();
}

RobotLocalization::CallbackReturn RobotLocalization::on_shutdown(const rclcpp_lifecycle::State &)
{
    robot_description_listener.reset();
    model_state_listener.reset();
    odometry_pub_.reset();
    odometry_msg_.reset();
    return CallbackReturn::SUCCESS;
}

RobotLocalization::CallbackReturn RobotLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
    robot_description_listener.reset();
    model_state_listener.reset();
    odometry_pub_.reset();
    odometry_msg_.reset();
    return CallbackReturn::SUCCESS;
}

RobotLocalization::CallbackReturn RobotLocalization::on_activate(const rclcpp_lifecycle::State &)
{
    odometry_pub_->on_activate();
    return CallbackReturn::SUCCESS;
}

RobotLocalization::CallbackReturn RobotLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
    odometry_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

RobotLocalization::CallbackReturn RobotLocalization::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Robot localization error");
    return CallbackReturn::SUCCESS;
}

void RobotLocalization::imu_callback(sensor_msgs::msg::Imu::SharedPtr imu)
{
    if(model_)
        model_->senseIMU(*state, *imu);
}

#if 0
void RobotLocalization::publish_odometry(BaseEffector::State& base) try {
    KDL::Vector delta_p = base.target.p - base.position.p;
    odometry_msg_->header.stamp = now();

    // pose/position update
    kdl_frame_to_pose(delta_p, odometry_msg_->pose.pose);

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish odometry: %s", e.what());
    deactivate();
}
#endif

void RobotLocalization::update() try {

}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish target TF data: %s", e.what());
    deactivate();
}

}  // ns: robot_dynamics
