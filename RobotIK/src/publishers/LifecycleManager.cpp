//
// Created by guru on 1/16/22.
//

#include <publishers/LifecycleManager.h>

namespace robotik {

LifecycleManager::LifecycleManager(rclcpp_lifecycle::LifecycleNode& self_managed_node)
    : LifecycleManager(self_managed_node, self_managed_node.get_name())
{

}

LifecycleManager::LifecycleManager(rclcpp_lifecycle::LifecycleNode& host_node, std::string target_node_name)
    : logger(host_node.get_logger())
{
    change_state_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_client_ = host_node.create_client<lifecycle_msgs::srv::ChangeState>(
            target_node_name + "/change_state",
            rclcpp::ServicesQoS(),
            nullptr);
}

LifecycleManager::SharedPtr LifecycleManager::if_self_manage(rclcpp_lifecycle::LifecycleNode& node, bool configure_now)
{
    // declare there is a self_manage parameter
    node.declare_parameter("self_manage",
        rclcpp::ParameterValue(false));

    // check the parameter value
    if (node.get_parameter("self_manage").get_value<bool>()) {
        auto mgr = std::make_shared<LifecycleManager>(node);

        if(configure_now)
            mgr->configure();
        return mgr;
    }
    return nullptr;
}

LifecycleManager::SharedFuture LifecycleManager::configure(std::string label)
{
    RCLCPP_INFO(logger, "transitioning to CONFIGURE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    change_state_request_->transition.label = label;
    return change_state_client_->async_send_request(change_state_request_).future.share();
}

LifecycleManager::SharedFuture LifecycleManager::activate(std::string label)
{
    RCLCPP_INFO(logger, "transitioning to ACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_request_->transition.label = label;
    return change_state_client_->async_send_request(change_state_request_).future.share();
}

LifecycleManager::SharedFuture LifecycleManager::deactivate(std::string label)
{
    RCLCPP_INFO(logger, "transitioning to DEACTIVATED");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    change_state_request_->transition.label = label;
    return change_state_client_->async_send_request(change_state_request_).future.share();
}

LifecycleManager::SharedFuture LifecycleManager::shutdown(uint8_t shutdown_type, std::string label)
{
    RCLCPP_INFO(logger, "transitioning to SHUTDOWN");
    change_state_request_->transition.id = shutdown_type;
    change_state_request_->transition.label = label;
    return change_state_client_->async_send_request(change_state_request_).future.share();
}

} // ns::robotik