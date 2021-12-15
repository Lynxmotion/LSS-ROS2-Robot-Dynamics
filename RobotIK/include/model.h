//
// Created by guru on 3/27/20.
//

#ifndef LSS_HUMANOID_ROBOTMODEL_H
#define LSS_HUMANOID_ROBOTMODEL_H

#include "types.h"
#include "state.h"
#include "limb.h"
#include "frame-ref.h"
#include "interfaces/model.h"

#include <srdfdom/model.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <robot_model_msgs/msg/model_state.hpp>
#include <robot_model_msgs/msg/multi_segment_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace robotik {

    class State;

    ///@brief Stores information about segments in a KDL tree
    /// borrowed from Ros RobotStatePublisher
    class SegmentPair final
    {
    public:
        explicit SegmentPair(
            const KDL::Segment & p_segment,
            const std::string & p_parent,
            const std::string & p_child)
            : segment(p_segment), parent(p_parent), child(p_child) {}

        KDL::Segment segment;
        std::string parent;
        std::string child;
    };

    class Model : public ModelInterface {
    public:
        using SharedPtr = std::shared_ptr<Model>;

        Model();

        void clear();

        // todo: rename this _configure()
        void setupURDF(std::string urdf_filename, std::string srdf_filename);

        void on_activate(rclcpp_lifecycle::LifecycleNode& node);
        void on_deactivate();

        // todo: I think all update-state and balance algos can be const, since we shouldnt be modifying model state in those function
        bool updateState(State& state);

        /// @brief Calculate the IK on the Limbs
        /// @returns the number of limbs that were updated
        int updateIK(State& state);

        /// @brief take joint positions (pose angles) and compute TF, CoM, CoP
        /// Requires state joint positions to be updated.
        bool compute_TF_CoM(State& state);

        /// @brief Computes the dynamic joint torque forces
        /// Requires that the ground forces have been computed and current. This uses Inverse Dynamics (Orokos) to
        /// calculate the internal joint forces. Gravity is taken into account as well as the computed Ground Reaction
        /// forces. The output of this can be used to compensate for gravity in compliant joints and to detect external
        /// (ex. human) interaction.
        bool updateDynamics(State& state);

        /// @brief Update support polygons, feet in contact and Ground Reaction Forces (GRF)
        /// This detects in software which feet are in contact with the ground. It uses this contact information, and
        /// the robot mass to determine the ground forces on the limbs on contact. This is required when calculating
        /// torque dynamics on the joints since gravity pulls on a dangling limb where a supporting limb squeezes the
        /// internal joint servos.
        bool updateContacts(State& state);

        /// @brief Update robot's odoometry to keep support legs stuck to the ground.
        /// Limbs in contact with the ground are analyzed and any offset produced by joint or IMU updates are
        /// mvoed to offsets of the base odometry position. Thusly these limbs in contact will be "nailed" to
        /// the floor.
        /// This function is already called by updateContacts and senseIMU methods.
        void updateNailedContacts(State& state);

        /// @brief processes an IMU message and updates the robot's pose.
        /// The robot's pose is updated by publishing the odom => base_link transform.
        void senseIMU(State& state, sensor_msgs::msg::Imu imu);

        ///@brief Transform segment state to be relative to the given frame
        //bool transform(const std::string name);

        //void setBaseHeight(double standingHeight);


        ///@brief Return the robot orientation
        /// The orientation is the parent frame of the robot's base link.
        inline EulerRotation getRobotRPY(State& state) { return getRobotRPY(state, ""); }

        ///@brief Return the robot orientation relative to the given reference frame
        EulerRotation getRobotRPY(State& state, std::string _ref_frame);

        ///@brief Compute a state with the robot in balance
        /// Given an input (current) state, use the dynamics information to determine a goal state where the robot
        /// is in support (balanced). It will use the ContactState data to determine which limbs are in contact and
        /// can be used for balance support.
        void balance(State& state);

        ///@brief Compute a state with the robot in balance
        /// Given an input (current) state, use the dynamics information to determine a goal state where the robot
        /// is in support (balanced). You must supply the limb ordinal values (as they are ordered in model::limbs)
        /// to specify which limbs are in contact.
        void balance(State& state, const SupportState& contacts);

        ///@brief Get all the names of joints in the model
        const Names& getJoints() const { return joints_; }

        // publish TF state
        void publishTransforms(
            const JointAndSegmentState& state,
            rclcpp::Time now,
            std::string prefix = "");

        void publishFixedTransforms(rclcpp::Time now, std::string prefix = "");

        void publishModelState(State& state, rclcpp::Time now, std::string prefix = std::string());

        ///@brief Set the distance from the robot base for the given limb
        /// The support distance is the ideal location of the limb (leg) end-effector to provide balance support for
        /// the robot.
        void setSupportDistance(double distance, Limb::DynamicModelType limbType = Limb::Leg);

        // todo: how can we get a relative state, when we dont have State until we call updateState()?
        //       do we defer offset of trajectory until draw time? (but then we have to track it for each
        //       Expression during updateState() and our KDL trajectories are already rendered and not linked to Exprs.
        KDL::Frame getRelativeFrame(const FrameRef& ref, const State& state) override;
        // disabled: void balance(State current, Model& model, Trajectory& newt, double timestep = 0.05);

        urdf::ModelInterfaceSharedPtr urdf_model_;
        srdf::ModelSharedPtr srdf_model_;

        std::string odom_link, footprint_link, base_link, imu_link;
        KDL::Vector gravity;

        bool use_internal_localization;
        bool use_contact_localizer;

        ///@brief the interaction element being manipulated by the user
        /// Any model control for this segment will be surpressed while it is being manipulated. For legs this will cause
        /// the robot to balance on the other leg(s). For the robot base, this will prevent the robot from moving the base
        /// over the Center-of-Mass. Therefor, the robot will be forced to move legs to compensate for balance.
        /// todo: move interaction related stuff to an interface
        std::string interactingSegment;

        Limb::Mode getLimbMode(std::string limb, const State& state) const;
        Limb::Mode getLimbMode(size_t limb, const State& state) const;

        std::shared_ptr<KDL::Tree> tree_;
        std::vector<robotik::Limb::SharedPtr> limbs;

    protected:
        bool use_tf_static_;
        KDL::Frame urdf_base_imu_tf;    // transform vector from base to IMU, loaded from URDF

        ///@brief the order of joints in the KDL tree
        Names joints_;

        ///@brief Which segments are fixed or not
        std::map<std::string, SegmentPair> segments_;
        std::map<std::string, SegmentPair> segments_fixed_;

        // tf and tf_static broadcasters
        geometry_msgs::msg::TransformStamped transformStamped;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::TransformStamped static_transformStamped;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

        // model state publisher
        robot_model_msgs::msg::ModelState::SharedPtr model_state_msg_;
        rclcpp_lifecycle::LifecyclePublisher<robot_model_msgs::msg::ModelState>::SharedPtr model_state_pub_;

        inline rclcpp::Logger get_logger() { return rclcpp::get_logger("robot_dynamics:model"); }

        ///@brief internal recursive TF, CoM and CoP computation on KDL segments.
        bool compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                            const KDL::Frame& tf, State& state);

        ///@brief compute the convex hull of a group of points
        /// This is used to compute the enclosing support polygon of the contact polygons
        std::vector<KDL::Vector> convexHull(const std::vector<KDL::Vector>& points) const;

        ///@brief Adds child segments from a parent segment to our fixed or non-fixed lists
        void addChildren(const KDL::SegmentMap::const_iterator& segment);
    };

} // ns::robot

#endif //LSS_HUMANOID_ROBOTMODEL_H
