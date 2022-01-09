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
) : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    declare_parameter("frequency", rclcpp::ParameterValue(10.0f));
    declare_parameter("progress_frequency", rclcpp::ParameterValue(5.0f));
    declare_parameter("publish_state_frequency", rclcpp::ParameterValue(0.0f));
    declare_parameter("preview_frequency", rclcpp::ParameterValue(0.0f));
    declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)5));
    declare_parameter("self_manage", rclcpp::ParameterValue(false));

    declare_parameter("joint_controller", rclcpp::ParameterValue("lss_joint_controller"));
    declare_parameter("effort_controller", rclcpp::ParameterValue("/effort_controller/commands"));
    declare_parameter("joint_names", rclcpp::ParameterValue(std::vector<std::string>()));

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
  current = std::make_shared<robotik::State>(*model_);

  // start listening for joint state updates
  if(!joint_state_listener)
    joint_state_listener = std::make_shared<robotik::JointStateListener>(
            *this,
            get_parameter(JOINT_STATE_TOPIC_PARAMETER).get_value<std::string>());
  joint_state_listener->state(current);

  if(!model_state_listener)
      model_state_listener = std::make_shared<robotik::ModelStateListener>(
              *this,
              get_parameter(MODEL_STATE_TOPIC_PARAMETER).get_value<std::string>(),
              "odom");
  model_state_listener->model(model_);
  model_state_listener->state(current);

  // todo: is there a way we can exctract this from the URDF or SRDF?
  joint_control_publisher->set_joints(get_parameter("joint_names").get_value<std::vector<std::string>>());

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
    RCLCPP_INFO(get_logger(), "configuring robot control");

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
            if(limb->model == robotik::Limb::Leg)
                limb->supportDistance = supportDistance;
        }
    }
#endif

    // publisher for joint trajectory
    auto joint_controller_name = get_parameter("joint_controller").get_value<std::string>();
    auto effort_controller_name = has_parameter("effort_controller")
            ? get_parameter("effort_controller").get_value<std::string>()
            : "";

    joint_control_publisher = std::make_shared<robotik::JointControlPublisher>(
            *this,
            joint_controller_name,
            effort_controller_name);

    // extended joint compliance parameters
    control_state_pub_ = this->create_publisher<robot_model_msgs::msg::ControlState>(
            "~/control_state",
            10);
    control_state_msg_ = std::make_shared<robot_model_msgs::msg::ControlState>();

    // reset service
    reset_service = create_service<robot_model_msgs::srv::Reset>("~/reset",
            std::bind(&Control::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // limb services
    configure_limb_service = create_service<robot_model_msgs::srv::ConfigureLimb>("~/configure_limb",
            std::bind(&Control::configure_limb_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_limb_service = create_service<robot_model_msgs::srv::SetLimb>("~/set_limb",
            std::bind(&Control::set_limb_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Trajectory Action Servers
    if(!this->trajectory_action_server_) {
        this->trajectory_action_server_ = rclcpp_action::create_server<robotik::trajectory::TrajectoryAction::EffectorTrajectory>(
                this,
                "~/trajectory",
                std::bind(&Control::handle_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Control::handle_trajectory_cancel, this, std::placeholders::_1),
                std::bind(&Control::handle_trajectory_accepted, this, std::placeholders::_1));
    }

    if(!this->coordinated_trajectory_action_server_) {
        this->coordinated_trajectory_action_server_ = rclcpp_action::create_server<robotik::trajectory::CoordinatedTrajectoryAction::EffectorTrajectory>(
                this,
                "~/coordinated_trajectory",
                std::bind(&Control::handle_coordinated_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Control::handle_coordinated_trajectory_cancel, this, std::placeholders::_1),
                std::bind(&Control::handle_coordinated_trajectory_accepted, this, std::placeholders::_1));
    }

    if(!this->linear_trajectory_action_server_) {
        this->linear_trajectory_action_server_ = rclcpp_action::create_server<robotik::trajectory::LinearTrajectoryAction::EffectorTrajectory>(
                this,
                "~/linear_trajectory",
                std::bind(&Control::handle_linear_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Control::handle_linear_trajectory_cancel, this, std::placeholders::_1),
                std::bind(&Control::handle_linear_trajectory_accepted, this, std::placeholders::_1));
    }

    auto frequency = get_parameter("frequency").get_value<float>();
    RCLCPP_INFO(get_logger(), "robot control loop set to %4.2fhz", frequency);
    update_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / frequency)),
            std::bind(&Control::control_update, this));

    auto preview_frequency = get_parameter("preview_frequency").get_value<float>();
    if(preview_frequency > 0.0) {
        RCLCPP_INFO(get_logger(), "publishing target preview data at %4.2fhz", preview_frequency);
        preview_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / preview_frequency)),
                        std::bind(&Control::publish_preview, this));
    }

    auto progress_frequency = get_parameter("progress_frequency").get_value<float>();
    if(progress_frequency > 0.0) {
        RCLCPP_INFO(get_logger(), "publishing progress notifications at %4.2fhz", progress_frequency);
        progress_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / progress_frequency)),
                        std::bind(&Control::publish_progress, this));
    }

    auto publish_state_frequency = get_parameter("publish_state_frequency").get_value<float>();
    if(publish_state_frequency > 0.0) {
        RCLCPP_INFO(get_logger(), "publishing control state messages at %4.2fhz", progress_frequency);
        publish_state_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / publish_state_frequency)),
                        std::bind(&Control::publish_control_state, this));
    } else {
        RCLCPP_WARN(get_logger(),
            "control state will be published every control loop, "
            "you can change this with the 'publish_state_frequency' parameter");
    }

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

    model_->clear();

    joint_control_publisher.reset();

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
    RCLCPP_INFO(get_logger(), "shutting down robot control");

    update_timer_.reset();
    diag_timer_.reset();

    model_->clear();

    joint_control_publisher.reset();

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
    RCLCPP_INFO(get_logger(), "activating robot control");
    CallbackReturn rv = CallbackReturn::SUCCESS;

    model_->on_activate(*this);

    // begin with the limbs loaded by the model
    bases_.clear();
    limbs_.clear();
    limbs_.reserve(model_->limbs.size());
    for(auto& limb: model_->limbs) {
        if(bases_.find(limb->base->link) == bases_.end()) {
            bases_.emplace_back(BaseEffector::State(limb->base));
        }
        limbs_.emplace_back(limb);
    }

    kinematics.activate(model_);

    joint_control_publisher->on_activate();
    control_state_pub_->on_activate();

    return rv;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "deactivating robot control");

    joint_control_publisher->on_deactivate();

    model_->on_deactivate();
    control_state_pub_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Control::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "robot control has entered error state");

    if (get_parameter("self_manage").get_value<bool>()) {
        RCLCPP_INFO(get_logger(), "Self-transitioning to INACTIVE");
        change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        change_state_request_->transition.label = "";
        change_state_future_ = change_state_client_->async_send_request(change_state_request_);
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void set_effector_msg(robot_model_msgs::msg::EffectorState& effector, const KDL::Frame& origin, const KDL::Frame& current_pose, const KDL::Frame& target_pose) {
    KDL::Frame inverted_origin = origin.Inverse();
    kdl_frame_to_pose(inverted_origin * current_pose, effector.pose);
    kdl_frame_to_pose(inverted_origin * target_pose, effector.target);
    KDL::Vector dv = target_pose.p - current_pose.p;
    effector.error = std::sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z() * dv.z());
    effector.velocity.linear = geometry_msgs::msg::Vector3();
    effector.velocity.angular = geometry_msgs::msg::Vector3();
}

// when there is no target active
void set_effector_msg(robot_model_msgs::msg::EffectorState& effector, const KDL::Frame& origin, const KDL::Frame& current) {
    kdl_frame_to_pose(origin.Inverse() * current, effector.pose);
    effector.velocity.linear = geometry_msgs::msg::Vector3();
    effector.velocity.angular = geometry_msgs::msg::Vector3();
    effector.target = effector.pose;
    effector.error = 0;
}

void Control::publish_control_state() try
{
    if(!target || !control_state_msg_ || !control_state_pub_->is_activated())
        return;

    //control_state_msg_->header.frame_id = prefix + current.relativeFrameName;
    control_state_msg_->header.stamp = now();

    KDL::Frame current_base_tf;
    if(!current->findTF(model_->base_link, current_base_tf)) {
        RCLCPP_WARN_ONCE(get_logger(), "failed to publish control state because state contains no base_link");
        return;
    }

    KDL::Frame target_base_tf;
    if(target->findTF(model_->base_link, target_base_tf)) {
        set_effector_msg(control_state_msg_->base, KDL::Frame(), current_base_tf, target_base_tf);
    } else
        set_effector_msg(control_state_msg_->base, KDL::Frame(), current_base_tf);


    auto limb_count = (short)limbs_.size();

#if 0       // todo: only control program should dictate if leg is supporting or not
    // figure out what limbs are supporting
    std::vector<bool> supporting(limb_count, false);
    for(const auto& contact : current->contacts) {
        if(contact.limb > 0 && contact.limb < limb_count)
            supporting[contact.limb] = true;
    }
#endif

    // limbs
    if(limb_count != (short)control_state_msg_->limbs.size())
        control_state_msg_->limbs.resize(limb_count);
    for(short i=0; i < limb_count; i++) {
        auto& ml = control_state_msg_->limbs[i];
        auto& sl = limbs_[i];
        auto& leg_model = *sl.model;
        ml.name = leg_model.link;
        ml.mode = sl.mode;
        ml.type = leg_model.model;
        ml.supportive = sl.supportive;
        ml.supporting = sl.supporting;
        tf2::toMsg(leg_model.origin, ml.origin);

#if 1
        if(sl.mode != robotik::Limb::Limp)
            set_effector_msg(ml.effector, leg_model.origin, sl.position, sl.target);
        else
            set_effector_msg(ml.effector, leg_model.origin, sl.position);
#else
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
#endif
    }

    control_state_pub_->publish(*control_state_msg_);
} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish control state: %s", e.what());
    deactivate();
}

rcl_interfaces::msg::SetParametersResult Control::parameter_set_callback(const std::vector<rclcpp::Parameter> & params) {
    for(auto& p: params) {
        auto name = p.get_name();
        //bool isNumber = p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER;

        if(name == "joint_names") {
            joint_control_publisher->set_joints(p.get_value<std::vector<std::string>>());
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

void Control::control_update() try {
    rclcpp::Time _now = now();

    // ensure we have state to publish
    if(!current)
        return;

    if(model_->tree_ && !model_->limbs.empty()
                && current->lastJointStateUpdate.get_clock_type()==RCL_ROS_TIME
                && current->lastSupportStateUpdate.get_clock_type()==RCL_ROS_TIME) {

        // ensure we are getting valid joint data
        auto sinceJoints = _now - current->lastJointStateUpdate;
        if(sinceJoints.seconds() > 2.0) {
            // stale joint data
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "joint state data is stale, disabling robot control!");
            return;
        }

        // ensure we are getting valid robot model data
        auto sinceModel = _now - current->lastSupportStateUpdate;
        if(sinceModel.seconds() > 2.0) {
            // stale robot model data
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "robot model state is stale, disabling robot control!");
            return;
        }

        // if we dont get the segment state from /tf and /tf_static we can
        // compute it using our model information
        if (!model_->compute_TF_CoM(*current)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to compute segment TF data for current state");
        }
        current->lastSegmentStateUpdate = _now;

        // update the target state using the limb model
        if(current && update_target(*current, _now)) {
            // send target state to (typically) ros2 controls
            joint_control_publisher->publish();
        }

        // publish our control state if we don't have it setup on a dedicated timer
        if(!publish_state_timer_)
            publish_control_state();
    }

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}

void Control::publish_progress() try {
    if(!target)
        return;

    rclcpp::Time _now = now();
    for(auto& action: actions) {
        action->send_feedback(_now);
    }

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish progress: %s", e.what());
    deactivate();
}

void Control::publish_diagnostics() try {

} catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
    deactivate();
}



/*
 * Integration of old Control class here
 */


void Control::resetTarget(const State& current) {
    target = std::make_shared<robotik::State>(current);
    joint_control_publisher->state(target);
}

void Control::resetTrajectory() {
    for(auto& action: actions) {
        action->complete(lastUpdate, robot_model_msgs::msg::TrajectoryComplete::RESET );
    }
    actions.clear();
}


class TrajectoryRenderEnv : public trajectory::RenderingInterface
{
public:
    inline TrajectoryRenderEnv()
    : model(nullptr), state(nullptr)
    {}

    virtual ~TrajectoryRenderEnv() {}

    Model* model;
    const State* state;

    KDL::Frame get_relative_frame(const FrameRef& ref, double /* at_timestamp*/) override
    {
        assert(model != nullptr);
        std::cout << "Request for relative frame " << ref.typeName() << ":" << ref.name << std::endl;
#if 0       // todo: revive the get_relative_frame check for segment ref from trajectory
        Limb* limb = nullptr;
        if(ref.type == FrameRef::Segment) {
            auto timeline = traj->timeline.find(ref.name);
            if(timeline == traj->timeline.end()) {
                // check to see if the segment is part of a limb
                for(auto& l: model->limbs) {
                    if (std::find(l->segment_names.begin(), l->segment_names.end(), ref.name) != l->segment_names.end()) {
                        // found in this limb, check to see if this limb is controlled in this trajectory
                        timeline = traj->timeline.find(l->to_link);
                        if(timeline == traj->timeline.end())
                            timeline = traj->timeline.find(l->base->base_link);
                        // for any inner/dependent segment we must compute IK
                        if(timeline != traj->timeline.end()) {
                            limb = l.get();
                        }
                    }
                }
            }

            if(timeline != traj->timeline.end()) {
                // pull the state of the segment from the timeline
                KDL::Frame f;
                // todo: WE HAVE TO GET STATE FOR FROM AND TO LINK!!
                if(timeline->second.get_state(at_timestamp, f)) {
                    std::cout << "  fetched segment state for " << ref.name << " from timline" << std::endl;
                    State st(*state);
                    if(limb && limb->updateIK(st, f)>0) {
                        //model->compute_TF_CoM(st, *limb);
                        model->compute_TF_CoM(st);
                    }
                    return model->getRelativeFrame(ref, st);
                }
            }
        }
#endif
        // reference is not dependant on trajectory/timeline state
        return model->getRelativeFrame(ref, *state);
    }

    KDL::Frame convert_frame(FrameRef from_ref, KDL::Frame frame_in, FrameRef to_ref) override
    {
        if(from_ref.type != FrameRef::Odometry) {
            auto odom_from = model->getRelativeFrame(from_ref, *state);
            frame_in = odom_from * frame_in;
        }
        if(to_ref.type != FrameRef::Odometry) {
            auto odom_to = model->getRelativeFrame(to_ref, *state);
            frame_in = odom_to.Inverse() * frame_in;
        }
        return frame_in;
    }
};


///@brief Update any required trajectories based on current state
/// Apply trajectories to state
///@returns the expected state at this moment in time.
bool Control::update_target(const State& current, rclcpp::Time _now) {
    double now_ts = _now.seconds();

    if(!target) {
        resetTarget(current);
        if(!target)
            // failed to establish target state from current
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,  "failed to reset target state from current");
            return false;
    }

    // always move the base of the target state to match the current base state
    KDL::Frame odom_tf;
    if(current.findTF(model_->odom_link, odom_tf))
        target->tf[model_->odom_link] = odom_tf;

    KDL::Frame current_base_tf;
    if(!current.findTF(model_->base_link, current_base_tf)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to publish control state because state contains no base_link");
        return false;
    }

    KDL::Frame target_base_tf;
    if(!target->findTF(model_->base_link, target_base_tf)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to publish control state because target state contains no base_link");
        return false;
    }

#if 1
    // todo: update Kinematics to have changes in base instead do changes in limbs
    // for now, copy just the base position from current to target
    target_base_tf.p = current_base_tf.p;
    target->tf[model_->base_link] = target_base_tf;

#else
    // todo: track changes in state's odom => base_link frame, and apply those changes to
    //       our target state as an offset. We can't just copy it like odom because we also
    //       want to manipulate the base frame.
    if(model_->interactingSegment != model_->base_link) {
        // as long as the user is not interacting with the base,
        // copy the base from sensed to target
        KDL::Frame base_tf;
        if(current.findTF(model_->base_link, base_tf))
            target->tf[model_->base_link] = base_tf;
    }
#endif

    double seconds_since_last_update = (lastUpdate.get_clock_type() == RCL_ROS_TIME)
            ? (_now - lastUpdate).seconds()
            : INFINITY;

    TrajectoryRenderEnv* rendering_env = nullptr;

    //
    // Update limbs using any active trajectory actions
    //
    std::vector<trajectory::TrajectoryActions::const_iterator> expired;
    for(auto a=actions.begin(), _a=actions.end(); a!=_a; a++) {
        auto& action = *a;
        trajectory::TimeRange tr = action->time_range();
        if(now_ts < tr.begin)
            continue;

        if(action->render_state() != trajectory::Rendered) {
            // this action must be rendered now
            if(!rendering_env) {
                // create the rendering environment
                rendering_env = new TrajectoryRenderEnv();
                rendering_env->model = &*model_;
                rendering_env->state = &current;
            }

            action->render(*rendering_env);

            // since we rendered, get the updated time range
            tr = action->time_range();
            std::cout << "rendered " << action->type() << ":" << action->id() << "   " << std::fixed << std::setprecision(4) << "  duration: " << tr.span() << std::endl;

        }

        // apply the action to the limb state
        action->apply(_now);

        // if this action has expired, remove it
        if(action->expired(_now)) {
            action->complete(_now);
            expired.emplace_back(a);
        }
    }

    // free rendering interface if we created one
    delete rendering_env;

    //
    // update base (as an effector)
    //
    for(auto& base: bases_) {
        auto base_link =  base.model->link;

        KDL::Frame current_limb_base_tf;
        if(current.findTF(base_link, current_limb_base_tf)) {
            // make limb_tf relative to robot body
            if(base_link != model_->base_link)
                // translate limb base to be relative to robot base
                current_limb_base_tf = current_base_tf.Inverse() * current_limb_base_tf;
            if(seconds_since_last_update < 10.0) {
                KDL::Vector delta_p = diff(current_limb_base_tf.p, base.position.p, seconds_since_last_update);
                KDL::Vector delta_r = diff(current_limb_base_tf.M, base.position.M, seconds_since_last_update);
                base.velocity.vel = delta_p;
                base.velocity.rot = delta_r;
            }
            base.position = current_limb_base_tf;
        }

        if(base.mode == Effector::Limp) {
            // this limb is not under control,
            // we will not do any Inverse Kinematics and
            // simply copy state from current
            base.target = base.position;
            kinematics.invalidate(base_link, LimbKinematics::JOINTS);
        } else if(base.mode == Effector::Holding) {
            // We want to maintain position so we should not copy from current,
            // nor perform any inverse kinematics as joint info should not change
            // tldr; do nothing
        } else if(base.mode >= Effector::Seeking) {
            // perform inverse kinematics based on Limb targets
            // todo: we must update limbs to nail legs
            auto target_tf = base.target;
            if(base_link != model_->base_link)
                target_tf = target_base_tf * target_tf;
            // todo: this needs to include the base_link, probably the collection of dependent limbs as welll
            kinematics.moveBase(*target, target_tf);
        }
    }



    //
    // update target state using control state from the limbs
    //
    for(auto& limb: limbs_) {
        auto l_name =  limb.model->link;

        KDL::Frame current_limb_tf;
        if(current.findTF(l_name, current_limb_tf)) {
            // make limb_tf relative to robot body
            current_limb_tf = current_base_tf.Inverse() * current_limb_tf;
            if(seconds_since_last_update < 10.0) {
                KDL::Vector delta_p = diff(current_limb_tf.p, limb.position.p, seconds_since_last_update);
                KDL::Vector delta_r = diff(current_limb_tf.M, limb.position.M, seconds_since_last_update);
                limb.velocity.vel = delta_p;
                limb.velocity.rot = delta_r;
            }
            limb.position = current_limb_tf;
        }

        if(limb.mode == Effector::Limp) {
            // this limb is not under control,
            // we will not do any Inverse Kinematics and
            // simply copy state from current
            limb.target = limb.position;
            joint_control_publisher->set_joint_effort(limb.model->joint_names, 0.0);

            for(auto& j_name: limb.model->joint_names) {
                auto j_index = target->findJoint(j_name);
                if(j_index >= 0) {
                    target->position(j_index) = current.position(j_index);
                    target->velocity(j_index) = current.velocity(j_index);
                    target->effort(j_index) = current.effort(j_index);
                }
            }

#if 0
            KDL::Frame f;
            for(auto& s_name: model_limb->segment_names) {
                if(current.findTF(s_name, f))
                    target->tf[s_name] = f;
            }
#endif
            kinematics.invalidate(l_name, LimbKinematics::JOINTS);
        } else if(limb.mode == Effector::Holding) {
            // We want to maintain position so we should not copy from current,
            // nor perform any inverse kinematics as joint info should not change
            // tldr; do nothing
            joint_control_publisher->set_joint_effort(limb.model->joint_names, 2.0);
        } else if(limb.mode >= Effector::Seeking) {
            // perform inverse kinematics based on Limb targets
            auto target_tf = target_base_tf * limb.target;
            kinematics.moveEffector(*target, l_name, target_tf);
            joint_control_publisher->set_joint_effort(limb.model->joint_names, 2.0);
        }
    }

    // erase expired actions
    if(!expired.empty()) {
        // erase from end to beginning so our expired iterators are not invalidated as we erase
        for(auto a=expired.rbegin(), _a=expired.rend(); a!=_a; a++) {
            actions.erase(*a);
        }
    }

    lastUpdate = _now;

    // update state of target
    // todo: this should be done after target is updated
    kinematics.updateState(*target);

    return true;
}


void Control::publish_preview() try
{
    if(!target)
        return;

    rclcpp::Time _now = now();
    return publish_preview_state(
            *target,
            "target",
            _now,
            last_static_publish_target);
}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish target TF data: %s", e.what());
    deactivate();
}

void Control::publish_preview_state(const State& state, const std::string& prefix, const rclcpp::Time& now, rclcpp::Time& last_static_publish)
{
    // publish preview state to TF
    model_->publishTransforms(state, now, prefix);

    // publish static transform every few seconds
    if(last_static_publish.get_clock_type() != RCL_ROS_TIME
    || (now - last_static_publish).seconds() > PUBLISH_STATIC_TF_EVERY) {
        model_->publishFixedTransforms(now, prefix);
        last_static_publish = now;
    }

    // publish model state for trajectory preview
    model_->publishModelState(state, now, prefix);
}


trajectory::Expression Control::expression_from_msg(
        robot_model_msgs::msg::SegmentTrajectory seg,
        std::string default_reference_frame,
        const rclcpp::Time& now)
{
    trajectory::Expression tf;
    auto ts = now.seconds() + (double)seg.start.sec + (double)seg.start.nanosec / 1e9;
    tf.start = ts;
    tf.duration = seg.duration;
    tf.segment = seg.segment;
    tf.mix_mode = (trajectory::MixMode)seg.mix_mode;
    tf.path_expression = seg.path;
    vector_to_kdl_vector(seg.points, tf.points);
    quat_to_kdl_rotation(seg.rotations, tf.rotations);

    // calculate velocity/acceleration
    // todo: figure out what the max joint vel/acc is
    tf.velocity = std::min(seg.velocity, 10.0);
    tf.acceleration = std::min(seg.acceleration, 4.0);

    // true if this trajectory is expected to support the robot
    tf.supporting = seg.supporting;

    // create motion profile
    // todo: if we use the same arg parser as path then vel/acc could be encoded here
    tf.velocity_profile = trajectory::velocityProfileStringToEnum(seg.profile);
    tf.coordinate_mode = (trajectory::CoordinateMode)seg.coordinate_mode;

    if(seg.reference_frame.empty())
        tf.reference_frame = FrameRef(FrameRef::Segment, std::move(default_reference_frame));
    else
        tf.reference_frame = FrameRef::parse(seg.reference_frame);

    if(tf.reference_frame.name == model_->odom_link)
        tf.reference_frame = FrameRef(FrameRef::Odometry, model_->odom_link);
    else if(tf.reference_frame.name == model_->base_link)
        tf.reference_frame = FrameRef(FrameRef::Robot, model_->base_link);
    else if(tf.reference_frame.name == robotik::world_link)
        tf.reference_frame = FrameRef(FrameRef::World);

    // if the name of our reference frame is blank, fill it with the default
    if (tf.reference_frame.type == FrameRef::Segment && tf.reference_frame.name.empty())
        tf.reference_frame.name = seg.segment;
    //else if (tf.reference_frame.type == FrameRef::Joint) {
    //    model_.
    //}
    return tf;
}

/*
 *   Reset Service
 */
void Control::reset_callback(const std::shared_ptr<robot_model_msgs::srv::Reset::Request> request,
         std::shared_ptr<robot_model_msgs::srv::Reset::Response> response) try
{
    if(request->target_state)
        resetTarget(*current);
    if(request->trajectories)
        resetTrajectory();
    if(request->limp) {
        // set all limbs to limp
        for(auto& l: limbs_) {
            l.mode = robotik::Limb::Limp;
        }
    }
    response->success = true;
}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "reset failed: %s", e.what());
    response->success = false;
}

/*
 *   Limb Services
 */
void Control::configure_limb_callback(const std::shared_ptr<robot_model_msgs::srv::ConfigureLimb::Request> request,
                             std::shared_ptr<robot_model_msgs::srv::ConfigureLimb::Response> response) try
{
    if(request->limbs.empty()) {
        response->success = false;
        return;
    }

    bool has_type = request->limbs.size() == request->type.size();
    bool has_origin = request->limbs.size() == request->origin.size();

    for(int i=0, _i = request->limbs.size(); i < _i; i++) {
        auto& limb = limbs_[request->limbs[i]];
        if(has_type) {
            limb.model->model = (robotik::Limb::DynamicModelType)request->type[i];
        }
        if(has_origin)
            tf2::fromMsg(request->origin[i], limb.model->origin);
    }

    response->success = true;
}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure limb failed: %s", e.what());
    response->success = false;
}

void Control::set_limb_callback(const std::shared_ptr<robot_model_msgs::srv::SetLimb::Request> request,
                       std::shared_ptr<robot_model_msgs::srv::SetLimb::Response> response) try
{
    if(request->limbs.empty()) {
        response->success = false;
        return;
    }

    bool has_mode = request->limbs.size() == request->mode.size();
    bool has_supportive = request->limbs.size() == request->supportive.size();
    bool has_compliance = request->limbs.size() == request->compliance.size();

    for(int i=0, _i = request->limbs.size(); i < _i; i++) {
        auto& limb = limbs_[request->limbs[i]];
        if(has_mode)
            limb.mode = (robotik::Limb::Mode)request->mode[i];
        if(has_supportive)
            limb.supportive = request->supportive[i];
        if(has_compliance)
            limb.compliance = request->compliance[i];
    }

    response->success = true;
}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "set limb failed: %s", e.what());
    response->success = false;
}


/*
 *   Single Trajectory Action
 */
rclcpp_action::GoalResponse Control::handle_trajectory_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const robotik::trajectory::TrajectoryAction::EffectorTrajectory::Goal> goal)
{
    //RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    auto& request = goal->goal;

    if(!target) {
        RCLCPP_INFO(get_logger(), "Cannot accept trajectory goals without an existing state");
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    KDL::Frame f;
    if(!target->findTF(request.segment.segment, f)) {
        RCLCPP_INFO(get_logger(), "Segment %s in goal request doesn't exist in state", request.segment.segment.c_str());
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Control::handle_trajectory_cancel(
        const std::shared_ptr<robotik::trajectory::TrajectoryAction::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;
    RCLCPP_INFO(get_logger(), "Received request to cancel trajectory %s for segment %s",
            request.id.c_str(),
            request.segment.segment.c_str());

    return actions.cancel(goal_handle->get_goal_id(), lastUpdate);
}

void Control::handle_trajectory_accepted(
        const std::shared_ptr<robotik::trajectory::TrajectoryAction::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;
    auto& uuid = goal_handle->get_goal_id();

    // set the absolute time
    rclcpp::Time now;
    if(request.header.stamp.sec) {
        now = rclcpp::Time(request.header.stamp);
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "trajectory publisher is not setting header timestamp");
        now = lastUpdate;
    }

    auto expr = expression_from_msg(request.segment, model_->odom_link, now);

    // cancel any existing actions for this segment
    actions.complete(
            expr.segment,
            now,
            robot_model_msgs::msg::TrajectoryComplete::PREEMPTED);

    // add the new action
    auto action = std::make_shared<trajectory::TrajectoryAction>(
            limbs_, model_,
            expr, goal_handle);
    action->uuid = uuid;
    if(!request.id.empty())
        action->id(request.id);
    actions.append(action);
}


/*
 *   Coordinated Trajectory Action
 */
rclcpp_action::GoalResponse Control::handle_coordinated_trajectory_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const robotik::trajectory::CoordinatedTrajectoryAction::EffectorTrajectory::Goal> goal)
{
    //RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    auto& request = goal->goal;

    if(!target) {
        RCLCPP_INFO(get_logger(), "Cannot accept trajectory goals without an existing state");
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    KDL::Frame f;
    for(auto& m: request.segments) {
        if(!target->findTF(m.segment, f)) {
            RCLCPP_INFO(get_logger(), "Segment %s in goal request doesn't exist in state", m.segment.c_str());
            return rclcpp_action::GoalResponse::REJECT; // no segment by this name
        }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Control::handle_coordinated_trajectory_cancel(
        const std::shared_ptr<robotik::trajectory::CoordinatedTrajectoryAction::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal for coordinated trajectory %s",
            request.id.c_str());

    return actions.cancel(goal_handle->get_goal_id(), lastUpdate);
}

void Control::handle_coordinated_trajectory_accepted(
        const std::shared_ptr<robotik::trajectory::CoordinatedTrajectoryAction::GoalHandle> goal_handle)
{
    auto& request = goal_handle->get_goal()->goal;
    auto& uuid = goal_handle->get_goal_id();

    // set the absolute time
    rclcpp::Time now;
    if(request.header.stamp.sec) {
        now = rclcpp::Time(request.header.stamp);
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "trajectory publisher is not setting header timestamp");
        now = lastUpdate;
    }

    std::vector<trajectory::Expression> expressions;
    for(const auto& e: request.segments) {
        // cancel any existing actions for this segment
        actions.complete(
                e.segment,
                now,
                robot_model_msgs::msg::TrajectoryComplete::PREEMPTED);

        // parse the expression
        expressions.emplace_back(expression_from_msg(e, model_->odom_link, now));
    }

    // add the new action
    auto action = std::make_shared<trajectory::CoordinatedTrajectoryAction>(
            limbs_, model_,
            expressions,
            goal_handle);
    action->uuid = uuid;
    if(!request.id.empty())
        action->id(request.id);
    actions.append(action);
}


/*
 *   Linear Trajectory Action
 */
rclcpp_action::GoalResponse Control::handle_linear_trajectory_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const robotik::trajectory::LinearTrajectoryAction::EffectorTrajectory::Goal> goal)
{
    //RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;

    if(!target) {
        RCLCPP_INFO(get_logger(), "Cannot accept trajectory goals without an existing state");
        return rclcpp_action::GoalResponse::REJECT; // no segment by this name
    }

    KDL::Frame f;
    for(auto& m: goal->effectors) {
        if(!target->findTF(m, f)) {
            RCLCPP_INFO(get_logger(), "Segment %s in goal request doesn't exist in state", m.c_str());
            return rclcpp_action::GoalResponse::REJECT; // no segment by this name
        }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Control::handle_linear_trajectory_cancel(
        const std::shared_ptr<robotik::trajectory::LinearTrajectoryAction::GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Received request to cancel goal for linear trajectory %s",
            goal->id.c_str());

    return actions.cancel(goal_handle->get_goal_id(), lastUpdate);
}

void Control::handle_linear_trajectory_accepted(
        const std::shared_ptr<robotik::trajectory::LinearTrajectoryAction::GoalHandle> goal_handle) try
{
    auto& request = goal_handle->get_goal();
    auto& uuid = goal_handle->get_goal_id();

    // set the absolute time
    rclcpp::Time now;
    if(request->header.stamp.sec) {
        now = rclcpp::Time(request->header.stamp);
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "trajectory publisher is not setting header timestamp");
        now = lastUpdate;
    }

    for(const auto& e: request->effectors) {
        // cancel any existing actions for this segment
        actions.complete(
                e,
                now,
                robot_model_msgs::msg::TrajectoryComplete::PREEMPTED);
    }

    // add the new action
    auto action = std::make_shared<trajectory::LinearTrajectoryAction>(
            bases_, limbs_, model_, now,
            goal_handle);
    action->uuid = uuid;
    if(!request->id.empty())
        action->id(request->id);
    actions.append(action);
}
catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "trajectory failed: %s", e.what());
    auto result = std::make_shared<robotik::trajectory::LinearTrajectoryAction::EffectorTrajectory::Result>();
    result->result.code = robot_model_msgs::msg::TrajectoryComplete::FAILED;
    goal_handle->abort(result);
}

}  //ns: robot_dynamics

