//
// Created by guru on 4/26/20.
//

#ifndef LSS_HUMANOID_INTERACTION_H
#define LSS_HUMANOID_INTERACTION_H

#include "model.h"
#include "state.h"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>


namespace robotik {

class InteractionEvent
{
public:
    std::string controlNamespace;
    std_msgs::msg::Header header;
    std::string segment;
    bool dragging;
    KDL::Frame tf;
};

class InteractionController {
public:
    using Handle = InteractionController*;
    virtual std::string interaction_namespace() =0;
    virtual bool interact(InteractionEvent& ev) =0;
    virtual std::set<std::string> manipulators(State& state) =0;
};

class LimbInteractionController: public InteractionController {
public:
    LimbInteractionController();

    // todo: this needs state in order to move Control::interact code to here
    // bool interact(InteractionEvent& ev) override;

    std::set<std::string> manipulators(State& state) override;

    ///@brief enable manipulations of all limbs of a type
    void manipulate(Limb::DynamicModelType byType, bool enable_base);

    ///@brief enable manipulations of all limbs of a type
    void manipulate(const std::set<std::string>& by_names);

    ///@brief enable manipulations of all limbs
    void manipulateAll(bool enable_base);

    ///@brief enable manipulations of all limbs
    void manipulateBase();

    void clearManipulators();

    void clearManipulators(State& state);

        ///@brief if true, the robot base segment will also be added for manipulation
    bool add_base_manipulators;

    Model::SharedPtr model_;
    std::set<std::string> enabled_segments;
};


class RobotInteraction {
public:
    using Marker = visualization_msgs::msg::Marker;
    using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
    using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
    using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;
    using InteractiveMarkerServerSharedPtr = std::shared_ptr<interactive_markers::InteractiveMarkerServer>;
    using InteractiveMarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;

    using StateControllers = std::map<std::string, InteractionController::Handle>;

    ///@brief the various types of manipulators we support
    typedef enum {
        SixDOF
    } InteractionMode;

public:
    void configure(rclcpp_lifecycle::LifecycleNode& ownerNode, Model::SharedPtr model, std::string topicNamespace);
    void cleanup();

    void activate();
    void deactivate();

    ///@brief clear all interaction manipulators
    /// This clears all manipulators and controllers for all namespaces.
    void clearManipulators();

    ///@brief Set the controller that handles manipulators for the given interaction namespace
    void setController(std::string interaction_ns, InteractionController& control);

    ///@brief update the set of manipulators for a given interaction namespace
    /// Manipulators will be created or updated to the pose as given in the state. The Robot model is consulted to
    /// determine what manipulators should exist based on root/base segment and the end-effectors.
    int updateManipulators(State& state, std::string interaction_ns);

    ///@brief Apply updates to manipulators.
    /// Changes to manipulators are not applied until explicitly requested. The Interaction controller is based on
    /// interactive markers sand the InteractiveMarkerServer so this method maps to server->applyChanges().
    inline void apply() { server->applyChanges(); }

    ///@brief clear all interaction manipulators
    /// This clears all manipulators and controllers for all namespaces.
    void clear();

    ///@ determine the marker name for a segment manipulator within an interaction namespace.
    std::string getSegmentControlName(std::string interaction_ns, std::string segment);

protected:
public:
    InteractiveMarkerServerSharedPtr server;

    ///@brief The Robot model
    /// Required to determine the manipulators based on base or end-effectors.
    Model::SharedPtr model_;

    ///@brief Interaction events are dispatched to these controllers
    /// This collection maps a controller to an interaction namespace. Use setController(...) method to modify this
    /// mapping collection.
    StateControllers controllers_;

    ///@brief Update the pose for an existing segment manipulator control
    bool updateSegmentManipulator(State& state, std::string interaction_ns, std::string segmentName);

    ///@brief Create a segment manipulator control that doesnt currently exist
    bool addSegmentManipulator(State& state, std::string interaction_ns, std::string segment, InteractionMode interactionMode);

private:
    void processFeedback(InteractiveMarkerFeedback::ConstSharedPtr feedback);
};

} // ns::robot

#endif //LSS_HUMANOID_INTERACTION_H
