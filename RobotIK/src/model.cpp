//
// Created by guru on 3/27/20.
//

#include <model.h>
#include <state.h>
#include <exception.h>

#include <conversions.h>

#include <cmath>

#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>

namespace robotik {


Model::Model()
    : odom_link("odom"), footprint_link("base_footprint"), base_link("base_link"), imu_link("imu_link"),
      gravity(0, 0, -9.81), use_internal_localization(true), use_contact_localizer(true), use_tf_static_(true)
{
}

void Model::clear() {
    limbs.clear();
    tree_.reset();
}

void Model::setupURDF(std::string urdf_filename, std::string srdf_filename) {
    std::string urdf_xml;

    if(urdf_filename.empty()) {
      tree_.reset();
      urdf_model_.reset();
      // todo: clear other fields?
      return;
    }

    // load the URDF and build chains
    tree_ = std::make_shared<KDL::Tree>();

    if(urdf_filename[0] == '<') {
        // expecting XML string
        kdl_parser::treeFromString(urdf_filename, *tree_);
        urdf_model_ = urdf::parseURDF(urdf_filename);
    } else {
        kdl_parser::treeFromFile(urdf_filename, *tree_);
        urdf_model_ = urdf::parseURDFFile(urdf_filename);
    }

    // split the segments into fixed (tf_static) and non-fixed (tf) segments
    segments_.clear();
    segments_fixed_.clear();
    addChildren(tree_->getRootSegment());

    //std::cout << "Loaded URDF:" << std::endl;
    //std::cout << "number of joints: " << tree_->getNrOfJoints() << std::endl;
    //std::cout << "number of segments: " << tree_->getNrOfSegments() << std::endl;

    srdf_model_ = std::make_shared<srdf::Model>();
    if(!srdf_model_->initFile(*urdf_model_, srdf_filename)) {
        throw std::runtime_error("Unable to load semantic robot description (SRDF)");
    }

    // get the list of joints (in KDL tree order)
    joints_.resize(tree_->getNrOfJoints());
    for(auto& seg: tree_->getSegments()) {
        auto q_nr = seg.second.q_nr;
        auto& segment = seg.second.segment;
        auto& joint = segment.getJoint();
        if(joint.getType() != KDL::Joint::None) {
            joints_[q_nr] = joint.getName();
        }
    }

    // todo: many Limb::Options are no longer needed: ex. tree, gravity
#if 0
    Limb::Options chain_opts;
    chain_opts.tree = tree_;
    chain_opts.gravity = gravity;

    chain_opts.model = Limb::Leg;
    chain_opts.from_link = base_link;
    chain_opts.to_link = "r_sole";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.from_link = base_link;
    chain_opts.to_link = "l_sole";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.model = Limb::Arm;
    chain_opts.from_link = base_link;
    chain_opts.to_link = "LHand";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.model = Limb::Arm;
    chain_opts.from_link = base_link;
    chain_opts.to_link = "RHand";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    // todo: determine transform from base to IMU
    urdf_base_imu_tf = KDL::Frame(
            KDL::Rotation::RPY(1.5708, 0, 1.5708),
            KDL::Vector(-0.06, 0, 0.078)
            );

#else
    // Hexapod config
    Limb::Options chain_opts;
    chain_opts.tree = tree_;
    chain_opts.gravity = gravity;
    chain_opts.model = Limb::Leg;
    chain_opts.from_link = base_link;

    chain_opts.to_link = "left-back-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.to_link = "left-front-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.to_link = "left-middle-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.to_link = "right-back-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.to_link = "right-front-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    chain_opts.to_link = "right-middle-foot";
    limbs.emplace_back(std::make_shared<Limb>(chain_opts));

    // todo: determine transform from base to IMU
    urdf_base_imu_tf = KDL::Frame(
            KDL::Rotation::RPY(0, 0, 0),
            KDL::Vector(0, 0.04, 0.005)
            );
#endif

    // load support polys for the limbs
    for(auto& limb: limbs) {
        try {
            limb->loadSupportPolygon(urdf_model_);
        } catch(const Exception& ex) {
            // allow the limb to remain but we will be missing object detection
            std::cout << ex.what() << std::endl;
        }
    }
}

// add children to correct maps
void Model::addChildren(const KDL::SegmentMap::const_iterator& segment)
{
    const std::string & root = GetTreeElementSegment(segment->second).getName();

    std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
    for (unsigned int i = 0; i < children.size(); i++) {
        const KDL::Segment & child = GetTreeElementSegment(children[i]->second);
        auto child_name = child.getName();
        SegmentPair seg_pair(GetTreeElementSegment(children[i]->second), root, child_name);

        if (child_name == base_link || child_name == footprint_link || child_name == odom_link) {
            // special links that we control internally
            if(!use_internal_localization && child_name == base_link) {
                RCLCPP_INFO(
                        get_logger(), "not publishing computed segment from %s to %s handled by external localization",
                        root.c_str(),
                        child.getName().c_str());
            } else {
                segments_.insert(make_pair(child.getJoint().getName(), seg_pair));
                RCLCPP_INFO(
                        get_logger(), "Adding computed segment from %s to %s", root.c_str(),
                        child.getName().c_str());
            }
        } else if (child.getJoint().getType() != KDL::Joint::None) {
            segments_.insert(make_pair(child.getJoint().getName(), seg_pair));
            RCLCPP_INFO(
                    get_logger(), "Adding joint segment from %s to %s", root.c_str(),
                    child.getName().c_str());
        } else {
            if (urdf_model_->getJoint(child.getJoint().getName()) &&
                urdf_model_->getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
            {
                RCLCPP_INFO(
                        get_logger(), "Floating joint, not adding segment from %s to %s.",
                        root.c_str(), child.getName().c_str());
            } else {
                segments_fixed_.insert(make_pair(child.getJoint().getName(), seg_pair));
                RCLCPP_INFO(
                        get_logger(), "Adding fixed segment from %s to %s", root.c_str(),
                        child.getName().c_str());
            }
        }
        addChildren(children[i]);
    }
}

void Model::on_activate(rclcpp_lifecycle::LifecycleNode& node) {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(&node);
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node);

    model_state_msg_ = std::make_shared<robot_model_msgs::msg::ModelState>();
    model_state_pub_ = node.create_publisher<robot_model_msgs::msg::ModelState>(
        DEFAULT_COMPONENT_TOPIC_PREFIX + "/model_state",
        10);
  
    model_state_pub_->on_activate();

    for(auto& c: limbs) {
        c->on_activate();
    }
}

void Model::on_deactivate() {
    tf_broadcaster.reset();
    static_tf_broadcaster.reset();

    model_state_msg_.reset();
    model_state_pub_->on_deactivate();
    model_state_pub_.reset();

    for(auto& c: limbs) {
        c->on_deactivate();
    }
}

void Model::publishTransforms(
        const JointAndSegmentState& state,
        rclcpp::Time now,
        std::string prefix)
{
    //RCLCPP_DEBUG(get_logger(), "Publishing transforms for moving joints");
    std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

    if(!prefix.empty() && prefix.back() != '/') {
        prefix += '/';
    }

    // loop over all joints
    for (const std::pair<const std::string, SegmentPair> & seg : segments_) {
        // lookup in state first
        ssize_t jn;
        KDL::Frame fparent, fchild, fsegment;
        if(state.findTF(seg.second.child, fchild) && state.findTF(seg.second.parent, fparent)) {
            fsegment = fparent.Inverse() * fchild;
        } else if((jn = state.findJoint(seg.first)) >=0) {
            // found the joint so apply the transform
            auto jp = state.position(jn);
            //std::map<std::string, SegmentPair>::iterator seg = segments_.find(tf.first);
            fsegment = seg.second.segment.pose(jp);
        } else if(seg.second.child == odom_link) {
            // world to odom can default to identity
        } else {
            RCLCPP_WARN_STREAM(get_logger(), "publish transforms: cannot determine frame for "
                    << seg.second.child);
            continue;
        }

        geometry_msgs::msg::TransformStamped tf_transform = tf2::kdlToTransform(fsegment);
        tf_transform.header.stamp = now;
        if(prefix.empty()) {
            tf_transform.header.frame_id = seg.second.parent;
            tf_transform.child_frame_id = seg.second.child;
        } else {
            tf_transform.header.frame_id = prefix + seg.second.parent;
            tf_transform.child_frame_id = prefix + seg.second.child;
        }
        tf_transforms.push_back(tf_transform);
    }

    tf_broadcaster->sendTransform(tf_transforms);
}

void Model::publishFixedTransforms(rclcpp::Time now, std::string prefix)
{
    //RCLCPP_DEBUG(get_logger(), "Publishing transforms for fixed joints");
    std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

    if(!prefix.empty() && prefix.back() != '/') {
        prefix += '/';
    }

    if(!prefix.empty()) {
        // send a world transform between the root tf namespace and ours
        geometry_msgs::msg::TransformStamped tf_transform;
        tf_transform.header.frame_id = "world";
        tf_transform.child_frame_id = prefix + tf_transform.header.frame_id;
        tf_transforms.push_back(tf_transform);
    }

    // loop over all fixed segments
    for (const std::pair<const std::string, SegmentPair> & seg : segments_fixed_) {
        geometry_msgs::msg::TransformStamped tf_transform = tf2::kdlToTransform(seg.second.segment.pose(0));
        if (!use_tf_static_) {
            // todo: why do we offset time here?
            now = now + rclcpp::Duration(std::chrono::milliseconds(500));
        }
        tf_transform.header.stamp = now;
        if(prefix.empty()) {
            tf_transform.header.frame_id = seg.second.parent;
            tf_transform.child_frame_id = seg.second.child;
        } else {
            tf_transform.header.frame_id = prefix + seg.second.parent;
            tf_transform.child_frame_id = prefix + seg.second.child;
        }
        tf_transforms.push_back(tf_transform);
    }
    if (use_tf_static_) {
        static_tf_broadcaster->sendTransform(tf_transforms);
    } else {
        tf_broadcaster->sendTransform(tf_transforms);
    }
}

void Model::publishModelState(const State& state, rclcpp::Time now, std::string prefix) {
    if(!model_state_msg_ || !model_state_pub_->is_activated())
        return;

    if(!prefix.empty() && prefix.back() != '/') {
        prefix += '/';
    }

    model_state_msg_->header.frame_id = prefix + state.relativeFrameName;
    model_state_msg_->header.stamp = now;
    model_state_msg_->mass = state.mass;
    kdl_vector_to_vector(state.CoM, model_state_msg_->center_of_mass);

    KDL::Frame base_f;
    if(state.findTF(base_link, base_f))
        kdl_frame_to_transform(base_f, model_state_msg_->base_pose);

    // joint forces
    auto joint_force_count = state.internalForces.rows();
    if(joint_force_count >0) {
        if(joint_force_count != model_state_msg_->forces.joints.size())
            model_state_msg_->forces.joints.resize(joint_force_count);
        for(size_t i=0; i < joint_force_count; i++)
            model_state_msg_->forces.joints[i] = state.internalForces(i);
    }

    // external forces
    auto ex_force_count = state.externalForces.size();
    if(ex_force_count >0) {
        if(ex_force_count != model_state_msg_->forces.external.size())
            model_state_msg_->forces.external.resize(ex_force_count);
        auto ef_itr = state.externalForces.begin();
        for(size_t i=0; i < ex_force_count; i++, ef_itr++) {
            assert(ef_itr != state.externalForces.end());       // sanity check
            auto& ef = model_state_msg_->forces.external[i];
            ef.frame_id = ef_itr->first;
            kdl_vector_to_vector(ef_itr->second.force, ef.wrench.force);
            kdl_vector_to_vector(ef_itr->second.torque, ef.wrench.torque);
        }
    }

    //
    // contact state
    //
    auto& contact_state = model_state_msg_->support;
    auto contact_count = state.contacts.size();

    kdl_vector_to_vector(state.CoP, contact_state.center_of_pressure);

    // todo: compute the support margin
    contact_state.margin = 0.2;
    contact_state.seconds_of_stability = 1.5;

    // support polygon points
    kdl_vector_to_vector(state.supportPolygon, contact_state.support_polygon);

    // contacts
    if(contact_count != contact_state.contacts.size())
        contact_state.contacts.resize(contact_count);
    for(size_t i=0; i < contact_count; i++) {
        auto &sc = state.contacts[i];
        auto &mc = contact_state.contacts[i];
        mc.limb = sc.limb;
        mc.segment = sc.name;
        kdl_vector_to_vector(sc.grf, mc.grf);
        kdl_vector_to_vector(sc.pointsInContact, mc.polygon);
        kdl_vector_to_vector(sc.wrt_CoM, mc.wrt_center_of_mass);
        kdl_vector_to_vector(sc.wrt_odom, mc.wrt_odom);
        kdl_vector_to_vector(sc.wrt_base, mc.wrt_base);
        mc.static_friction = sc.staticFriction;
        mc.slippage = 0;  // sc.slippage
    }

    model_state_pub_->publish(*model_state_msg_);
}

KDL::Frame Model::getRelativeFrame(const FrameRef& ref, const State& state)
{
    KDL::Frame f, g;
    switch(ref.type) {
        case FrameRef::Segment:
            if(!state.findTF(ref.name, f))
                throw std::runtime_error("frame for segment " + ref.name + " doesnt exist");
            break;
#if 1   // todo: these get_relative_frame requires access to Model  (maybe Control needs to be RenderingInterface)
        //case FrameRef::Joint:
            // todo: implement joint relative frame lookup
        //    break;
        case FrameRef::World:
            // lookup the relativeFrame and odom and invert it, so we get worlds frame with respect to algo
            if(!state.findTF(state.relativeFrameName, f))
                throw std::runtime_error("frame for segment " + ref.name + " doesnt exist");
            if(!state.findTF(odom_link, g))
                throw std::runtime_error("frame for segment " + ref.name + " doesnt exist");
            f = f.Inverse() * g.Inverse();   // todo: test this
            break;
        case FrameRef::Odometry:
            //if(!state.findTF(odom_link, f))
            //    throw std::runtime_error("frame for segment " + ref.name + " doesnt exist");
            break;
        case FrameRef::Robot:
            if(!state.findTF(base_link, f))
                throw std::runtime_error("frame for robot base doesnt exist");
            break;
        case FrameRef::CoM:
            f.p = state.CoM;
            break;
        case FrameRef::gCoM:
            f.p = state.CoM;
            f.p.z(0.0);
            break;
        case FrameRef::CoP:
            f.p = state.CoP;
            break;
        case FrameRef::Footprint:
            if(!state.findTF(footprint_link, f))
                throw std::runtime_error("frame for footprint doesnt exist");
            break;
#endif
        default:
            throw std::runtime_error("frame reference for " + std::string(ref.typeName()) + " not implemented");
    }
    return f;
}

bool Model::compute_TF_CoM(State& state)
{
    // todo: compute_TF used to be from identity, now using whatever base_link is set to
    //KDL::Frame ident = KDL::Frame::Identity();
    KDL::Frame transform = state.tf[base_link];
    //transform.M = state.imu * transform.M; //   why was I transforming about IMU?

    // initialize input arguments
    state.mass = 0.0;
    state.CoM = KDL::Vector::Zero();
    KDL::SegmentMap::const_iterator root = tree_->getSegment(base_link);

#if 0
    try {
        geometry_msgs::msg::TransformStamped transformStamped = tfBuffer.lookupTransform(root_link_name, footprint_link,
                                                    rclcpp::Time(0));
        transform.p.x(transformStamped.transform.translation.x);
        transform.p.y(transformStamped.transform.translation.y);
        transform.p.z(transformStamped.transform.translation.z);

        transform.M.Quaternion(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
        );
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "base_link update: %s",ex.what());
        return false;
    }
#endif

    if(!compute_TF_CoM(root, transform, state)) {
        throw Exception(RE_FAILED_COM);
    }

    if (state.mass <= 0.0) {
        state.CoM = KDL::Vector::Zero();
        throw Exception(RE_ZERO_MASS, "Total mass is 0, no CoM possible.");
    }

    state.CoM = 1.0/state.mass * state.CoM;

    // since the segment data is based purely off the JointState we can use that
    // timestamp to indicate when the segments were updated as well
    state.lastSegmentStateUpdate = state.lastJointStateUpdate;
    return true;
}

bool Model::compute_TF_CoM(const KDL::SegmentMap::const_iterator& currentSeg,
                           const KDL::Frame& tf, robotik::State& state)
{
    KDL::Frame jnt_tf, fparent, fchild;
    auto joint = currentSeg->second.segment.getJoint();
    std::string child_name = currentSeg->first;

    if(child_name == footprint_link || child_name == base_link) {
        // we compute these transforms internally, so retrieve the current TF from state
        if(state.findTF(child_name, fchild) && state.findTF(currentSeg->second.parent->first, fparent)) {
            // compute joint TF relative to parent
            jnt_tf = fparent.Inverse() * fchild;
        } else {
            // cannot retrieve from state, so compute as a fixed joint and update state
            jnt_tf = state.tf[child_name] = tf * currentSeg->second.segment.pose(0);
        }
    } else {
        if (joint.getType() != KDL::Joint::None) {
            // rotational or translational joints,
            // lookup joint position in state and update TF in state
            auto jordinal = state.findJoint(joint.getName());
            if (jordinal < 0)
                return false;
            double jnt_p = state.position(jordinal);
            jnt_tf = tf * currentSeg->second.segment.pose(jnt_p);
        } else {
            // fixed segment, transform using p=0
            jnt_tf = tf * currentSeg->second.segment.pose(0);
        }

        state.tf[child_name] = jnt_tf;
    }

    // aggregate mass and calculate it's center
    KDL::Vector current_cog = currentSeg->second.segment.getInertia().getCOG();
    double current_m = currentSeg->second.segment.getInertia().getMass();

    state.CoM = state.CoM + current_m * (jnt_tf * current_cog);
    state.mass += current_m;

    // recurse into child joints
    std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
    for (child_it = currentSeg->second.children.begin(); child_it != currentSeg->second.children.end(); ++child_it){
        if(!compute_TF_CoM(*child_it, jnt_tf, state))
            return false;
    }
    return true;
}

bool Model::updateState(State& state) {
    std::string root_link_name = base_link;

    // ensure some defaults
    if(state.relativeFrameName.empty())
        state.relativeFrameName = odom_link;


    if (compute_TF_CoM(state)) {
        updateContacts(state);

        updateDynamics(state);

#if 0
        try {
            KDL::Frame tx;
            //tf2_ros::TransformReadyCallback ready = [](const tf2_ros::TransformStampedFuture& x) -> void { };
            //tfBuffer.waitForTransform(footprint_link, odom_link, rclcpp::Time(0), rclcpp::Duration(0.5), ready).wait_for(std::chrono::milliseconds (1000));
            geometry_msgs::msg::TransformStamped transformStamped = tfBuffer.lookupTransform("LRadius", "LHumerus",
                                                                                             rclcpp::Time(0), rclcpp::Duration(0,0));
            std::cout << "odom: " << tx.M << std::endl;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "base_link update: %s",ex.what());
            return;
        }
#endif

#if 0
        if (++print_stats >= 150) {
            KDL::Vector g = state.CoM - state.CoP;
            double x = g.z(), y = g.y(), z = g.z();
            double i = atan2(y, x) * 180.0 / M_PI;
            double j = atan2(z, x) * 180.0 / M_PI;
            std::cout << "CoP:" << state.CoP.z() << "    CoM:" << state.CoM.z() << "    (" << (state.CoM.z() - state.CoP.z()) << ")  i:" << i
                      << "   j:" << j << std::endl;
            print_stats = 0;
        }
#endif
        
        return true;
    }
    return false;
}

bool Model::updateContacts(State& state) {
    // if we have an odom frame in the state, include that in transform
    KDL::Frame rootTF;
    if(state.findTF(odom_link, rootTF))
        rootTF = rootTF * state.transform;
    else
        rootTF = state.transform;

    // get base orientation and transform
    double base_roll, base_pitch, base_yaw;
    auto& base = state.tf[base_link];
    base.M.GetRPY(base_roll, base_pitch, base_yaw);

    // compute the leg barycenter while we analyze contacts
    size_t leg_count = 0;
    KDL::Vector legs_barycenter;

    // get location of all grounded limbs
    // compute total lengths of all limbs from CoM
    // compute stability in each axis by taking the shortest limb distance to CoM
    double totalDistances = 0;
    double minZ = NAN;
    unsigned limbid = 0;
    for(auto& limb: limbs) {
        Contact contact(limbid++);

        // todo: this should be computing pressures relative to the gravity vector but for simplicity we'll stick with just x,y
        contact.name = limb->options_.to_link;

        // find existing contact
        auto existing_contact_itr = std::find_if(state.contacts.begin(), state.contacts.end(),
                                                 [&contact](const Contact& c) { return c.limb >= contact.limb; });

        auto linkTF = state.findTF(contact.name);

        // save limb endpoint for computing footprint
        if(limb->options_.model == Limb::Leg) {
            legs_barycenter += linkTF.p;
            leg_count++;
        }

        // get the joint associated with this limb
        const auto& segment = tree_->getSegment(contact.name);     // todo: make segment lookup and joint-idx more efficient
        assert(segment != tree_->getSegments().end());
        auto joint = segment->second.segment.getJoint();

        // transform the support polygon and see if it intersects the ground
        for(auto& p : limb->supportPolygon) {
            auto tp = linkTF * p;
            auto global_tp = rootTF * tp;
            if(global_tp.z() < 0.003 && global_tp.z() > -0.005) {       // todo: support detection should be more advanced than just a simple threshold
                contact.pointsInContact.push_back(tp);
            }
        }

        // we need to determine the lowest limb
        if(std::isnan(minZ) || linkTF.p.z() < minZ)
            minZ = linkTF.p.z();

        // check if support polygon is in contact with ground
        if(contact.pointsInContact.empty()) {
            // this is not a worthy contact, remove it
            if(existing_contact_itr != state.contacts.end())
                state.contacts.erase(existing_contact_itr);
            continue;
        }

        // save contact pivot point, divide by N to get mean
        //contact_point = contact_point / contact.pointsInContact.size();
        contact.wrt_odom = linkTF.p;
        contact.wrt_base = base.Inverse() * contact.wrt_odom;
        contact.wrt_CoM = contact.wrt_odom - state.CoM;

        // compute distance from CoM
        KDL::Vector origin = contact.wrt_CoM;
        contact.distance = std::sqrt( origin.x()*origin.x() + origin.y()*origin.y() );

#if 0 // disabled since it doesnt seem to have a use right now
        // find joint origin by walking up the segment chain until we find a non-fixed joint
        // todo: make resolving joint origin in GRF more efficient (remove lookups and chain walking)
        KDL::Frame jo = linkTF;
        KDL::Joint::JointType jt = KDL::Joint::None;
        auto el = segment->second.parent;
        do {
            const auto& segment = el->second.segment;
            const auto& joint = segment.getJoint();
            jt = joint.getType();
            jo = state.tf[ el->first ];
            el = el->second.parent;
        } while (jt != KDL::Joint::None);

        // now set the joint origin in reference to the contact origin
        // todo: this joint origin might be the better place to nail our contact point
        contact.jointOrigin = contact.tf.p - jo.p;
#endif

        // update or add contact
        if(existing_contact_itr == state.contacts.end() || existing_contact_itr->limb != contact.limb ) {
            // todo: if wrt_odom is < 0, we should snap to 0...this solves the base-height thing
            // new contact, itr is insert position
            state.contacts.insert(existing_contact_itr, contact);
        } else {
            // copy variables
            // todo: this is accounting for actual movement in contact points
            double q = 0.99;
            existing_contact_itr->distance = contact.distance;

#if 0
            bool changeInContactPoints = existing_contact_itr->pointsInContact.size() != contact.pointsInContact.size();
            if(changeInContactPoints) {
                // support details changed, update some support variables
                bool dir_less = existing_contact_itr->pointsInContact.size() < contact.pointsInContact.size();
                std::cout << "ccp  " << contact.name << "  " << (dir_less ? "▼ " : "▲ ") << contact.pointsInContact.size() << std::endl;
                q = 0.9;
            }
#endif
            //if(existing_contact_itr->pointsInContact == contact.pointsInContact)
            //    q = 0.95;
            // odom point is nailed down once contact is established, so should barely move
            existing_contact_itr->wrt_odom = existing_contact_itr->wrt_odom * q + contact.wrt_odom * (1.0 - q);

            // base and CoM should always update as the robot is updated (duh)
            existing_contact_itr->wrt_base = base.Inverse() * contact.wrt_odom;
            existing_contact_itr->wrt_CoM = contact.wrt_odom - state.CoM;

            // todo: always update points in contact for now, but we could apply some filtering here to prevent skipping
            existing_contact_itr->pointsInContact = contact.pointsInContact;
        }

        totalDistances += contact.distance;

        // todo: we can also use Z of each limb in contact to estimate floor slope, and use it for base orientation, predict IMU and future foot contacts
    }

    // update the barycenter
    legs_barycenter.x( legs_barycenter.x() / leg_count);
    legs_barycenter.y( legs_barycenter.y() / leg_count);
    legs_barycenter.z( legs_barycenter.z() / leg_count);

    // ensure we have allocated the external forces vector
    state.alloc(POSITION | VELOCITY | ACCELERATION | INTERNAL_FORCE | EXTERNAL_FORCE);

    // Compute Ground Reaction Forces
    // Mass is spread over the limbs based on it's weighted distance from CoM
    // compute force of robot mass (gravity) pushing down on each limb
    // todo: if we used gravity vector and contact vector we could also count arms in contact with walls/etc
    state.externalForces.clear();       // clear old forces
    std::vector<KDL::Vector> allpoints;
    for(auto& contact: state.contacts) {
        auto limb = limbs[contact.limb];

        double p = contact.distance / totalDistances;
        double m = state.mass * p;
        contact.grf = m * -gravity;

        // calculate friction values
        contact.staticFriction = limb->friction.staticForce(contact.grf.z());

        // fudge the static friction if we dont have many points in contact
        if (contact.pointsInContact.size() <3) {
            contact.staticFriction *= contact.pointsInContact.size() / 3.0;
        }

        // update external force wrench with ground reaction force
        // it is at the point of pressure, so we need to change the ref-point to be at the joints
        KDL::Wrench grf(contact.grf, KDL::Vector::Zero());
        KDL::Wrench j_grf = grf.RefPoint(contact.wrt_odom); // or should this be contact.jointOrigin?

        state.externalForces[contact.name] = j_grf;

        // add vertices into support polygon
        allpoints.insert(allpoints.end(), contact.pointsInContact.begin(), contact.pointsInContact.end());
    }

    // compute convex hull, the minimal polygon that encompasses all contact points
    if(!allpoints.empty())
        state.supportPolygon = convexHull(allpoints);
    else
        state.supportPolygon.clear();


    double filter_CoP = 0.85;
    double pq = 0.75;

    //
    // Compute Center of Pressure for support limbs
    //
    if(!state.supportPolygon.empty()) {
        KDL::Vector CoP;
        for (auto &p: state.supportPolygon)
            CoP += p;
        CoP = CoP / state.supportPolygon.size();
        if (std::isnan(CoP.x()) || std::isnan(CoP.y()) || std::isnan(CoP.z())) {
            std::cout << "nan cop" << std::endl;
            CoP = KDL::Vector(state.CoM.x(), state.CoM.y(), 0.);
        } else {
            // CoP should be at the floor, or at least the lowest limb position
            CoP.z(minZ);

            // now update state CoP using a filter
            state.CoP = state.CoP * filter_CoP + CoP * (1 - filter_CoP);
        }

        state.inSupport = state.pointInSupport(state.CoM);
    } else {
        state.CoP = KDL::Vector(state.CoM.x(), state.CoM.y(), 0.);
        state.inSupport = false;
    }


    // calculate height between base_link and sole
    if(!std::isnan(minZ)) {
        double nbh = base.p.z() - minZ;
        double bhff = state.baseHeightFromFloor() * pq + nbh * (1 - pq);
        state.baseHeightFromFloor(bhff);
    }

    // REP-0120 base footprint
    // The base_footprint is the representation of the robot position on the floor. The floor is usually the level
    // where the supporting leg rests, i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left
    // and right sole height respecitvely. The translation component of the frame should be the barycenter of the
    // feet projections on the floor. With respect to the odom frame, the roll and pitch angles should be zero and
    // the yaw angle should correspond to the base_link yaw angle.
    //
    // Rationale: base_footprint provides a fairly stable 2D planar representation of the humanoid even while walking
    // and swaying with the base_link.
    auto footprint = KDL::Frame(
            KDL::Rotation::RPY(
                    0,
                    0,
                    base_yaw
            ),
            KDL::Vector(
                    legs_barycenter.x(),
                    legs_barycenter.y(),
                    state.CoP.z())
    );
    state.tf[footprint_link] = footprint;

    // adjust robot back to nailed points
    updateNailedContacts(state);

    return true;
}

void Model::updateNailedContacts(State& state) {
    /// Update base pose by prediction based on contacts and slip friction
    ///
    /// This analyzes the limbs in contact to predict the change in base pose and odometry position. Limbs in
    /// contact should maintain their position in the world frame with the only exception being if the contact
    /// slips due to overcoming friction. Estimating slippage is calculating by analyzing the conflicting forces
    /// and determining if the overall force passes the friction threshold?
    if (!use_contact_localizer)
        return;

    KDL::Frame base_tf;
    if(state.findTF(base_link, base_tf)) {
        KDL::Vector avg_contact_diff;

        if(state.contacts.size()) {
            //
            // Analyze any offset of the contacts from the nailed points in the odom frame
            // and adjust the base to cancel out the offset
            //
            for (auto &contact: state.contacts) {
                // todo: calculate friction/slippage:
                //   * if weight over a foot times friction is below a certain value consider the foot *not* in contact
                //   * add up all forces into a single force vector
                //   * if only one limb in contact, then obviously skip any further friction calcs since it doesnt matter
                //   * analyze how much each limb-in-contact offset disagrees with each other. Represent as a force vector
                //     and then for each contact limb check if friction is overcome. This should take into consideration
                //     the friction K of the limb, mass over leg, and...
                //   * friction threshold will changes based on the angle of contact, how much of surface is in contact,
                //     and mass over the point of contact
                //   * readjust the overall offset correction of base
                KDL::Frame odom_contact_tf = KDL::Frame(contact.wrt_odom);
                KDL::Frame base_contact_tf = KDL::Frame(contact.wrt_base);
                auto contact_diff = odom_contact_tf.Inverse() * base_tf * base_contact_tf;
                avg_contact_diff += contact_diff.p;
            }

            // average the sum of diff vectors
            avg_contact_diff = avg_contact_diff / state.contacts.size();

            // move the base into a position that cancels out the offset
            base_tf.p -= avg_contact_diff;
        } else {
            // no contacts, ensure robot is on the ground by analyzing limb Z
            //
            int n = 0;
            int lowest_l = -1;
            KDL::Frame lowest_f;
            for(auto &limb: limbs) {
                KDL::Frame limb_f;
                if(state.findTF(limb->options_.to_link, limb_f)) {
                    if(lowest_l<0 || limb_f.p.z() < lowest_f.p.z()) {
                        lowest_l = n;
                        lowest_f = limb_f;
                    }
                }
                n++;
            }

            if(n >=0) {
                // move the robot
                if(lowest_f.p.z() < 0) {
                    // leg is below the ground
                    avg_contact_diff = KDL::Vector(0, 0, lowest_f.p.z());
                } else {
                    // leg/robot is in the air
                    // todo: we should simulate gravity here, but we'd need the body velocity then
                    avg_contact_diff = KDL::Vector(0, 0, lowest_f.p.z() * 0.1);
                }
            }
        }

        // update base and all inner robot segments
        for (auto &tf: state.tf) {
            if (tf.first != odom_link && tf.first != "world") {
                // offset joints
                tf.second.p -= avg_contact_diff;
            }
        }

        // update state variables
        state.CoP -= avg_contact_diff;
        state.CoP.z( 0);
        state.CoM -= avg_contact_diff;

    }
}

void Model::senseIMU(State& state, sensor_msgs::msg::Imu imu) {
    // todo: odom this should be updated elsewhere than IMU event
    state.tf[odom_link] = KDL::Frame();

    // translate IMU coordinate frame to base frame
    if(use_internal_localization) {
        // read in sensed IMU position with respect to IMU identity frame
        auto imu_orientation = to_kdl_rotation(imu.orientation);

        // compute frame transform from imu to base
#if 0
        // old humanoid update method
        KDL::Frame tf_imu_odom_base = urdf_base_imu_tf.Inverse();
        tf_imu_odom_base.M = imu_orientation * tf_imu_odom_base.M;

        state.tf[base_link] = tf_imu_odom_base;
#else
        // new hexapod method (and I think the correct one)
        KDL::Frame tf_imu_base = urdf_base_imu_tf.Inverse();
        tf_imu_base.M = imu_orientation * tf_imu_base.M;

        state.tf[base_link].M = tf_imu_base.M;

#endif
        updateNailedContacts(state);

        // use linear/angular velocity somewhere?
        //   - we definitely should use it in the EKF to predict what is happening
        //   - possibly use gravity on CoM and contact points to determine what angular velocity should be and correlate with EKF
    }
}

bool Model::updateDynamics(State& state) {

#if 1
    KDL::TreeIdSolver_RNE solver(*tree_, gravity);

    //todo: should we check here if all inputs are matching size?
    int rv = solver.CartToJnt(state.position, state.velocity, state.acceleration, state.externalForces, state.internalForces);
    if (rv != KDL::SolverI::E_NOERROR) {
#if 0
        // this is now sent in the Robot Dynamics main file, search "publishCompliance" in updateRobotState
        auto msg_gravity_itr = msg->gravity.begin() + ofs;
        //compliance_params_msg_->joints.push_back(jointNames[j]);
        for (unsigned int t=0, _t=getNrOfJoints(); t<_t; t++) {
            *msg_gravity_itr++ = torques_.data[t];
        }
#endif
    }
#endif
    return false;
}

EulerRotation Model::getRobotRPY(State& state, std::string _ref_frame) {
    KDL::Frame tf;
    if(state.findTF(base_link, tf)) {
        if(!_ref_frame.empty()) {
            // translate to desired reference frame
            KDL::Frame relTF;
            if(state.findTF(_ref_frame, relTF)) {
                tf = relTF.Inverse() * tf;
            } else
                throw Exception::SegmentNotFound(_ref_frame);
        }

        // get base in Eular Roll, Pitch, Yaw
        double r, p, y;
        tf.M.GetRPY(r, p, y);

        return {r, p, y};
    }
    else
        throw Exception::SegmentNotFound(base_link);
}

std::vector<KDL::Vector> Model::convexHull(const std::vector<KDL::Vector>& points) const {
    // if there are 3 or less points then we logically already have the convex hull
    if (points.size() < 4){
        return points;
    }

    std::vector<KDL::Vector> hull;

    auto pcl_points( pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
    pcl::PointCloud<pcl::PointXYZ> chull_points;
    pcl::ConvexHull<pcl::PointXYZ> chull;

    for (unsigned i = 0; i < points.size(); ++i) {
        if(std::isnan(points[i].x()) || std::isnan(points[i].z())) {
            std::cout << "NaN found in convex hull points" << std::endl;
            return hull;
        }

        pcl_points->points.push_back(pcl::PointXYZ(
                // XY point
                points[i].x(), points[i].y(),
                // force Z point to zero
                0.0));
    }

    chull.setDimension(2);
    chull.setInputCloud(pcl_points);
    std::vector<pcl::Vertices> polygons;
    chull.reconstruct(chull_points, polygons);

    if (polygons.size() == 0) {
        // ROS_ERROR("Convex hull polygons are empty");
        return hull;
    } else if (polygons.size() > 1) {
        //ROS_WARN("Convex hull polygons are larger than 1");
    }

    for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
        int idx = polygons[0].vertices[i];
        KDL::Vector p(chull_points.points[idx].x,
                    chull_points.points[idx].y,
                    chull_points.points[idx].z);
        hull.push_back(p);
    }

    return hull;
}

} // ns::robotik

