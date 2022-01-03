//
// Created by guru on 1/2/22.
//


//
// Created by guru on 12/15/21.
//

#include "listeners/SegmentStateListener.h"

#include <utility>
#include "types.h"

namespace robotik {


SegmentStateListener::SegmentStateListener(rclcpp_lifecycle::LifecycleNode& node, std::string prefix)
    : prefix_(std::move(prefix))
{
    using callback_t = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;
    callback_t cb = std::bind(&SegmentStateListener::tf_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(&SegmentStateListener::tf_callback, this, std::placeholders::_1, true);
    subscription_tf_ = node.create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            10,
            std::move(cb));
    subscription_tf_static_ = node.create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static",
            10,
            std::move(static_cb));
}

void SegmentStateListener::state(const std::shared_ptr<SegmentState>& state)
{
    if(state)
        state_ = state;
    else
        state_.reset();
}

void SegmentStateListener::tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool /* is_static */)
{
    int updates = 0;
    builtin_interfaces::msg::Time now;
    if(model_ && state_) {
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

            // todo: we are limiting to only receiving world and odom coordinates for now
            //      because the segment transforms are relative to each links parent and not
            //      odom frame like our State is...so this is pretty useless to us unless we
            //      integrate into the model tree or change the way our State works
            if(frame_id == robotik::world_link || frame_id == model_->odom_link || frame_id == model_->base_link)
                state_->tf[frame_id] = tf2::transformToKDL(tfx);
            updates++;
        }

        if(updates > 0)
            state_->lastSegmentStateUpdate = rclcpp::Time(now);
    }
}


}
