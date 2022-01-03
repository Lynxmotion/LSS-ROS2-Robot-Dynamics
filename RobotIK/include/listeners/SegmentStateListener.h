//
// Created by guru on 1/2/22.
//

#ifndef ROBOT_DYNAMICS_SEGMENTSTATELISTENER_H
#define ROBOT_DYNAMICS_SEGMENTSTATELISTENER_H

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <model.h>
#include <indexes.h>

namespace robotik {

    // todo: WARNING --- IMPLEMENTATION INCOMPLETE ---
    //    The tf_callback needs to refer to the Model KDL tree in order to properly update SegmentState.
    //    The TF messages are relative to each parent, where-as the SegmentState segments are all relative
    //    to odom frame.

class SegmentStateListener {
public:
    using SharedPtr = std::shared_ptr<SegmentStateListener>;

    SegmentStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string prefix = "");
    //SegmentStateListener(rclcpp::Node& node, std::string prefix = "");

    // todo: we probably have to add activate/deactivate to listener classes so they stop processing
    //       messages when main node is not active

    void state(const std::shared_ptr<SegmentState>& state);

    [[nodiscard]] inline const std::shared_ptr<SegmentState>& state() const { return state_; }
    [[nodiscard]] inline std::shared_ptr<SegmentState>& state() { return state_; }

protected:
    std::string prefix_;
    Model::SharedPtr model_;
    std::shared_ptr<SegmentState> state_;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf_static_;
    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);
};

} // ns:robotik

#endif //ROBOT_DYNAMICS_SEGMENTSTATELISTENER_H
