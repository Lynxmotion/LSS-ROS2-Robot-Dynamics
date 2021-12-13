//
// Created by guru on 3/27/20.
//

#ifndef LSS_HUMANOID_ROBOTSTATEVISUAL_H
#define LSS_HUMANOID_ROBOTSTATEVISUAL_H

#include <utility>

#include "types.h"
#include "state.h"
#include "model.h"
#include "color.h"

namespace robotik {

    class StateVisual {
    public:
        using SharedPtr = std::shared_ptr<StateVisual>;
        using Marker = visualization_msgs::msg::Marker;

        std::string ref_frame_id;
        std::string marker_namespace;
        bool show_only_updated;
        Color ghostColor;

        explicit StateVisual(std::string ns = "robot_pose");

        bool update(const State& state);

        void configure(Model::SharedPtr model);

        void cleanup();

        void activate();

        void deactivate();

        inline unsigned long visibility() const { return _visibles; }
        void visibility(unsigned long add, unsigned long remove);

        void segments(Names&& ids);
        void segments(std::set<std::string> ids);

        void positions(Ordinals&& ids);

        inline visualization_msgs::msg::MarkerArray::SharedPtr getMarkers() const { return marker_msg_; }

        visualization_msgs::msg::MarkerArray::SharedPtr remove();

    protected:
        unsigned long _visibles;
        Names _segments;

        visualization_msgs::msg::MarkerArray::SharedPtr marker_msg_;
        //rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    protected:
        /* KDL Tree
         */
        Model::SharedPtr _model;

        /* URDF Resources
         */
        urdf::ModelInterfaceSharedPtr _urdf;
    };

} // ns::robot

#endif //LSS_HUMANOID_ROBOTSTATEVISUAL_H
