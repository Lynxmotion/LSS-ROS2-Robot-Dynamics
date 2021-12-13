//
// Created by guru on 4/26/20.
//

#include <interaction.h>
#include <conversions.h>

#include <set>

namespace robotik {

void RobotInteraction::configure(rclcpp_lifecycle::LifecycleNode& ownerNode, Model::SharedPtr model, std::string topicNamespace) {
    server = std::make_shared<InteractiveMarkerServer>(topicNamespace,
           ownerNode.get_node_base_interface(),
           ownerNode.get_node_clock_interface(),
           ownerNode.get_node_logging_interface(),
           ownerNode.get_node_topics_interface(),
           ownerNode.get_node_services_interface()
    );
    model_ = model;
}

void RobotInteraction::cleanup() {
    server.reset();
    model_.reset();
    controllers_.clear();
}

void RobotInteraction::setController(std::string interaction_ns, InteractionController& control) {
    controllers_[interaction_ns] = &control;
}

void RobotInteraction::activate() {
}

void RobotInteraction::deactivate() {
}

void RobotInteraction::clear() {
    server->clear();
    server->applyChanges();
    controllers_.clear();
}

///@brief clear all interaction manipulators
/// This clears all manipulators and controllers for all namespaces.
void RobotInteraction::clearManipulators() {
    server->clear();
    server->applyChanges();
}

LimbInteractionController::LimbInteractionController()
    : add_base_manipulators(false)
{}

std::set<std::string> LimbInteractionController::manipulators(State& state) {
    // add the end effector manipulators
    size_t n = 0;
    assert(model_->limbs.size() == state.limbs.size());
    for(auto& limb: model_->limbs) {
        state.limbs[n].manipulator = std::find(enabled_segments.begin(), enabled_segments.end(), limb->options_.to_link) != enabled_segments.end();
        n++;
    }
    return enabled_segments;
}

void LimbInteractionController::manipulate(const std::set<std::string>& by_names) {
    // technically we could just set directly, but if there are links in by_names that arent
    // part of a limb then we are no longer a Limb Interaction Controller
    size_t n = 0;
    std::set<std::string> links;
    if(std::find(
            by_names.begin(),
            by_names.end(),
            model_->base_link) != by_names.end())
        links.insert(model_->base_link);
    for(auto& limb: model_->limbs) {
        // the name must exist in the list of limbs, either the "from" or "to" segment
        if(std::find(
                by_names.begin(),
                by_names.end(),
                limb->options_.to_link) != by_names.end())
            links.insert(limb->options_.to_link);
        if(add_base_manipulators && std::find(
                by_names.begin(),
                by_names.end(),
                limb->options_.from_link) != by_names.end())
            links.insert(limb->options_.from_link);
        n++;
    }
    enabled_segments = links;
}

void LimbInteractionController::manipulateAll(bool enable_base) {
    std::set<std::string> links;
    if(enable_base)
        links.insert(model_->base_link);
    for(auto& limb: model_->limbs) {
        links.insert(limb->options_.to_link);
    }
    manipulate(links);
}

void LimbInteractionController::manipulate(Limb::DynamicModelType byType, bool enable_base) {
    std::set<std::string> links;
    if(enable_base)
        links.insert(model_->base_link);
    for(auto& limb: model_->limbs) {
        if(limb->options_.model == byType)
            links.insert(limb->options_.to_link);
    }
    manipulate(links);
}

void LimbInteractionController::manipulateBase() {
    std::set<std::string> list;
    list.insert(list.end(), model_->base_link);
    manipulate(list);
}

void LimbInteractionController::clearManipulators() {
    enabled_segments.clear();
}

void LimbInteractionController::clearManipulators(State& state)
{
    clearManipulators();
    for(auto& limb: state.limbs) {
        // the name must exist in the list of limbs, either the "from" or "to" segment
        limb.manipulator = false;
        if(limb.mode == Limb::Mode::Manipulating)
            limb.mode = Limb::Mode::Holding;
    }
}

std::string RobotInteraction::getSegmentControlName(std::string interaction_ns, std::string segment) {
    return interaction_ns+'.'+segment;
}

int RobotInteraction::updateManipulators(State& state, std::string interaction_ns) {
    int count = 0;
#if 1
    InteractionController::Handle ctrlr = controllers_[interaction_ns];
    if(ctrlr) {
        auto segments = ctrlr->manipulators(state);
        for(auto& segment: segments) {
            if (!updateSegmentManipulator(state, interaction_ns, segment)) {
                if (addSegmentManipulator(state, interaction_ns, segment, SixDOF))
                    count++;  // added
            }
        }
    }
#else
    // add the end effector manipulators
    size_t n = 0;
    for(auto& limb: model_->limbs) {
        bool enabled = state.limbs[n].manipulator;

        if(enabled) {
            baseLinks.insert(limb->options_.from_link);
            if (!updateSegmentManipulator(state, interaction_ns, limb->options_.to_link)) {
                if (addSegmentManipulator(state, interaction_ns, limb->options_.to_link, SixDOF))
                    count++;  // added
            } else
                count++;  // updated
        }
        n++;
    }

    // now add the base manipulator
    for(auto& link: baseLinks) {
        if (!updateSegmentManipulator(state, interaction_ns, link)) {
            if (!addSegmentManipulator(state, interaction_ns, link, SixDOF))
                count++;  // added
        } else
            count++; // updated
    }
#endif
    return count;
}

bool RobotInteraction::updateSegmentManipulator(State& state, std::string interaction_ns, std::string segmentName) {
    KDL::Frame tf;

    // if we have an odom frame in the state, include that in transform
    KDL::Frame rootTF;
    if(state.findTF(model_->odom_link, rootTF))
        rootTF = rootTF * state.transform;
    else
        rootTF = state.transform;

    //std_msgs::msg::Header & header;
    if(state.findTF(segmentName, tf)) {
        geometry_msgs::msg::Pose pose;
        tf = rootTF * tf;
        kdl_frame_to_pose(tf, pose);
        return server->setPose(getSegmentControlName(interaction_ns, segmentName), pose);
    }
    return false;
}

bool RobotInteraction::addSegmentManipulator(State& state, std::string interaction_ns, std::string segmentName, InteractionMode interactionMode) {
    // activate interactive markers
    // create an interactive marker for our server
    InteractiveMarker int_marker;
    int_marker.header.frame_id = model_->odom_link;
    //int_marker.header.stamp = now();
    int_marker.name = getSegmentControlName(interaction_ns, segmentName);
    int_marker.description = "Control the " + segmentName + "position";
    int_marker.scale = 0.035;

    KDL::Frame tf;
    if(state.findTF(segmentName, tf)) {
        tf = state.transform * tf;
        kdl_frame_to_pose(tf, int_marker.pose);
    }

    // create a cylinder marker
    visualization_msgs::msg::Marker box_marker;
    //box_marker.header.frame_id = base_link;
    box_marker.type = visualization_msgs::msg::Marker::SPHERE;
    box_marker.scale.x = 0.005;
    box_marker.scale.y = 0.005;
    box_marker.scale.z = 0.005;
    box_marker.color.r = 1.0;
    box_marker.color.g = 0.1;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

#if 0       // need a button?
    // create a non-interactive control which contains the box
    InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.name = "base_button";
    box_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );
#endif

#if 0
    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl rotate_control;
    rotate_control.name = "stand-z-rotate";
    rotate_control.interaction_mode = interactionMode;
    rotate_control.orientation.x = 0.;
    rotate_control.orientation.y = 0.7071068;
    rotate_control.orientation.z = 0.;
    rotate_control.orientation.w = 0.7071068;

    // add the control to the interactive marker
    int_marker.controls.push_back(rotate_control);
#else
    if(interactionMode == SixDOF) {
        InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }
#endif

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server->insert(int_marker,
                  std::bind(&RobotInteraction::processFeedback, this, std::placeholders::_1)
    );

    return true;
}

void RobotInteraction::processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    /* ROS_INFO_STREAM( feedback->marker_name << " is now at "
                                           << feedback->pose.position.x << ", " << feedback->pose.position.y
                                           << ", " << feedback->pose.position.z );*/
    /*std::cout << feedback->marker_name << " is now at "
              << feedback->pose.position.x << ", " << feedback->pose.position.y
              << ", " << feedback->pose.position.z << std::endl;*/
#if 0
    if(feedback->marker_name.compare("stand-z") ==0) {
        auto _now = now();
        current.control->setStandingHeight(feedback->pose.position.z, _now);
    }
#endif
    KDL::Frame f;
    std::string marker_name = feedback->marker_name;
    pose_to_kdl_frame(feedback->pose, f);
    //std::cout << "control: " << marker_name << " => " << f << std::endl;

    auto brk = marker_name.find('.');
    if(brk != std::string::npos) {
        // got a prefix'd marker with the controller ID
        std::string ns = marker_name.substr(0, brk);
        std::string segment = marker_name.substr(brk + 1);
        const auto &controller = controllers_.find(ns);
        if(controller != controllers_.end()) {
            InteractionEvent ev;
            ev.controlNamespace = ns;
            ev.header = feedback->header;
            ev.segment = segment;
            ev.tf = f;
            ev.dragging = feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE || feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN;
            // event sends MOUSE_DOWN=4, then many POSE_UPDATE=1 then finally MOUSE_UP=5
            //std::cout << "evt: " << (int)feedback->event_type << std::endl;

            controller->second->interact(ev);
        }
    }
}

} // ns::robot
