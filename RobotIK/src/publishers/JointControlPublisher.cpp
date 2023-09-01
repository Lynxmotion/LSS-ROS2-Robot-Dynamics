//
// Created by guru on 1/2/22.
//


#include "publishers/JointControlPublisher.h"

#include <utility>
//#include "types.h"

namespace robotik {


JointControlPublisher::JointControlPublisher(
        rclcpp_lifecycle::LifecycleNode& node,
        std::string position_controller,
        std::string effort_controller,
        std::string frame_id)
    : frame_id_(std::move(frame_id)),
    position_controller_name(std::move(position_controller)),
    effort_controller_name(std::move(effort_controller)),
    efforts_updated(false),
    logger(node.get_logger())
{
    // publisher for joint trajectory
    joint_trajectory_pub_ = node.create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/"+position_controller_name+"/joint_trajectory",
            10);

    // create a joint publisher, we'll control the servo output
    if(!joint_trajectory_msg_)
        joint_trajectory_msg_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

    // effort control
    if(!effort_controller_name.empty()) {
        joint_efforts_pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
                effort_controller_name,
                10);
        if(!joint_efforts_msg_) {
            joint_efforts_msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
            if(!joint_trajectory_msg_->joint_names.empty())
                joint_efforts_msg_->data.resize(joint_trajectory_msg_->joint_names.size());
        }
    } else {
        RCLCPP_WARN(logger, "No effort_controller parameter specified so effort control will be disabled");
    }

    // ability to control state of the joint controller
    // tutorial: https://github.com/ros2/demos/blob/master/lifecycle/src/lifecycle_service_client.cpp
    auto qos = rclcpp::ServicesQoS();
    jointctrl_change_state_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    jointctrl_get_state_client_ = node.create_client<lifecycle_msgs::srv::GetState>(
            position_controller_name + "/get_state",
            qos,
            nullptr);
    jointctrl_change_state_client_ = node.create_client<lifecycle_msgs::srv::ChangeState>(
            position_controller_name + "/change_state",
            qos,
            nullptr);
}

void JointControlPublisher::state(const std::shared_ptr<JointState>& state)
{
    if(state)
        state_ = state;
    else
        state_.reset();
}

void JointControlPublisher::set_joints(const std::vector<std::string> &joint_names)
{
    // new joint names for publishing
    size_t N = joint_names.size();
    if(!joint_trajectory_msg_)
        joint_trajectory_msg_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    joint_trajectory_msg_->joint_names = joint_names;

    if(!joint_efforts_msg_)
        joint_efforts_msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
    joint_efforts_msg_->data.resize(N);

    joint_ordinal_map.resize(N);
}

void JointControlPublisher::set_joint_effort(std::vector<std::string> joints, double effort, double epsilon)
{
    for(int i=0, _i=joint_trajectory_msg_->joint_names.size(); i < _i; i++) {
        auto& j_name = joint_trajectory_msg_->joint_names[i];
        // we can just check epsilon first, if effort doesnt change we dont have to binary search by name
        if(fabs(joint_efforts_msg_->data[i] - effort) < epsilon)
            continue;
        auto itr = std::find(joints.begin(), joints.end(), j_name);
        if(itr != joints.end()) {
            // joint found in set, this joint should be set...
            // and we already checked for epsilon
            joint_efforts_msg_->data[i] = effort;
            efforts_updated = true;
        }
    }
}

JointControlPublisher::CallbackReturn JointControlPublisher::on_activate()
{
    if(joint_trajectory_pub_)
        joint_trajectory_pub_->on_activate();
    if(joint_efforts_pub_)
        joint_efforts_pub_->on_activate();
    return CallbackReturn::SUCCESS;
}

JointControlPublisher::CallbackReturn JointControlPublisher::on_deactivate()
{
    if(joint_trajectory_pub_)
        joint_trajectory_pub_->on_deactivate();
    if(joint_efforts_pub_)
        joint_efforts_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

void JointControlPublisher::activate_position_controller(bool active)
{
    if(active)  {
        jointctrl_change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        jointctrl_change_state_future_ = jointctrl_change_state_client_->async_send_request(jointctrl_change_state_request_).future.share();
    } else {
        jointctrl_change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        jointctrl_change_state_future_ = jointctrl_change_state_client_->async_send_request(jointctrl_change_state_request_).future.share();
    }
}

void JointControlPublisher::publish_joint_trajectory(const rclcpp::Time&)
{
    if(!state_)
        return;

    if (state_->joints.size() != (size_t) state_->position.rows()) {
        throw robotik::Exception(
                RE_SIZE_MISMATCH,
                "refuse to publish when joint names and positions arrays are not equal length");
    }

    // todo: publishing joints order should match Q_NR joint order
    if (joint_trajectory_msg_->joint_names.empty()) {
        set_joints(state_->joints);
    }

    auto nj = joint_trajectory_msg_->joint_names.size();

    // allocate a single trajectory keyframe
    if (joint_trajectory_msg_->points.size() != 1) {
        joint_trajectory_msg_->points.resize(1);
    }

    // set the trajectory
    auto &points = joint_trajectory_msg_->points[0];
    if (points.positions.size() != nj)
        points.positions.resize(nj);

    for (size_t j = 0, _j = nj; j != _j; j++) {
        // lookup joint position using index
        auto j_name = joint_trajectory_msg_->joint_names[j];
        auto j_idx = joint_ordinal_map.resolve(*state_, j, j_name);

        if(j_idx < 0) {
            // joint name was not found, this is a trajic error
            throw robotik::Exception::JointNotFound(j_name);
        }

        // add the joint position
        points.positions[j] = state_->position(j_idx);
        // todo: publish velocity and acceleration for trajectories
    }

    // publish new control values
    joint_trajectory_pub_->publish(*joint_trajectory_msg_);
}

void JointControlPublisher::publish_efforts(const rclcpp::Time& now) {
    if(joint_efforts_pub_) {
        joint_efforts_pub_->publish(*joint_efforts_msg_);
        efforts_updated = false;
        last_efforts_update = now;
    }
}

void JointControlPublisher::publish(const rclcpp::Time& now) {
    publish_joint_trajectory(now);

    if(last_efforts_update.get_clock_type() != RCL_ROS_TIME) {
        last_efforts_update = now;
    }

    double last_efforts_update_secs = (now - last_efforts_update).seconds();
    if(efforts_updated || last_efforts_update_secs < 1.0)
        publish_efforts(now);
}

}
