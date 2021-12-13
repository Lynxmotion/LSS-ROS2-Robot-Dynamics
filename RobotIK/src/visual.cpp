//
// Created by guru on 3/27/20.
//

#include <visual.h>
#include <conversions.h>

//#define DEBUG_LIMB_CONTACTS

namespace robotik {

StateVisual::StateVisual(std::string ns)
: ref_frame_id("odom"), marker_namespace(ns), show_only_updated(false),
    ghostColor(Color::Green, 0.5), _visibles(DEFAULT_VISUALS)
{
}

void StateVisual::configure(Model::SharedPtr model)
{
    _model = model;
    _urdf = _model->urdf_model_;
    ref_frame_id = _model->odom_link;
}

void StateVisual::activate() {
    // publisher for target pose visualization markers
    //marker_pub_->on_activate();
    marker_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
}

void StateVisual::cleanup() {
    marker_msg_.reset();
}

void StateVisual::deactivate() {
    //marker_pub_->on_deactivate();
    marker_msg_.reset();
}

void StateVisual::visibility(unsigned long add, unsigned long remove) {
    _visibles &= ~remove;
    _visibles |= add;
}

void StateVisual::segments(Names&& ids) {
    _visibles |= TF;
    _segments = std::move(ids);
}

void StateVisual::segments(std::set<std::string> ids) {
    _visibles |= TF;
    _segments.clear();
    _segments.insert(_segments.end(), ids.begin(), ids.end());
}

static void setColor(std_msgs::msg::ColorRGBA& rgba, const Color& clr) {
    rgba.r = clr.r / 255.0;
    rgba.g = clr.g / 255.0;
    rgba.b = clr.b / 255.0;
    rgba.a = clr.alpha / 255.0;
}


static StateVisual::Marker& arrow(StateVisual::Marker& m,
                                  Color color = Color::Blue, double scale = 1.0) {
    m.type = StateVisual::Marker::ARROW;
    m.points.resize(2);
    setColor(m.color, color);
    m.scale.x = scale * 0.002;      // shaft diameter
    m.scale.y = scale * 0.006;      // head diameter
    m.scale.z = scale * 0.01;      // arrow length
    return m;
}

static StateVisual::Marker& arrow(StateVisual::Marker& m,
        KDL::Vector vFrom, KDL::Vector vTo,
        Color color = Color::Blue, double scale = 1.0) {
    arrow(m, color, scale);
    auto& from = m.points[0];
    auto& to = m.points[1];
    from.x = vFrom.x();
    from.y = vFrom.y();
    from.z = vFrom.z();
    to.x = vTo.x();
    to.y = vTo.y();
    to.z = vTo.z();
    return m;
}

static StateVisual::Marker& linestrip(StateVisual::Marker& m,
                                  std::vector<KDL::Vector> points,
                                  Color color = Color::White,
                                  double scale = 1.0) {
    m.type = StateVisual::Marker::LINE_STRIP;
    m.points.resize(points.size());
    for(size_t i=0, _i=points.size(); i < _i; i++) {
        auto& dest = m.points[i];
        auto& src = points[i];
        dest.x = src.x();
        dest.y = src.y();
        dest.z = src.z();
    }
    setColor(m.color, color);
    m.scale.x = scale * 0.001;      // shaft diameter
    m.scale.y = 1.0;      // head diameter
    m.scale.z = 1.0;      // arrow length
    return m;
}

static StateVisual::Marker& sphere(StateVisual::Marker& m,
                                   KDL::Vector pos,
                                    Color color = Color::White,
                                   double scale = 1.0
                                   ) {
    m.type = StateVisual::Marker::SPHERE;
    m.points.resize(2);
    setColor(m.color, color);
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale * 0.01;
    m.scale.y = scale * 0.01;
    m.scale.z = scale * 0.01;
    return m;
}

visualization_msgs::msg::MarkerArray::SharedPtr StateVisual::remove() {
    // initialize all markers
    int id = 1;
    for(auto& m: marker_msg_->markers) {
        m.ns = marker_namespace;
        m.id = id++;
        m.type = Marker::SPHERE;
        m.action = Marker::DELETE;

        m.points.clear();

        m.lifetime = rclcpp::Duration(1, 0);
    }
    return marker_msg_;
}

bool StateVisual::update(const State& state) {

    unsigned numberOfJoints = _model->tree_->getNrOfJoints();

    if(_segments.empty()) {
        // since user specified TF instead of using the specific joints() method we add all joints to request
        for(auto tf: state.tf)
            _segments.emplace_back(tf.first);
    }

    // count the required number of markers
    size_t N = 0;
    if(_visibles & TF)
        N += _segments.size();
    if(_visibles & POSITION)
        N += 2; // todo: properly size this again ..._positions.size();
    if(_visibles & CENTER_OF_MASS)
        N++;
    if(_visibles & CENTER_OF_PRESSURE)
        N++;
    if(_visibles & (CENTER_OF_MASS | CENTER_OF_PRESSURE))
        N++;
    if(_visibles & INTERNAL_FORCE)
        N += state.internalForces.rows() + 1;
    if(_visibles & EXTERNAL_FORCE && !state.externalForces.empty())
        N += state.externalForces.size();
    if(_visibles & CONTACTS && !state.contacts.empty())
        N += state.contacts.size();
    if(_visibles & SUPPORT_POLYGON && !state.supportPolygon.empty())
        N += 1;


    // make sure we have enough markers
    if(!marker_msg_)
        return false;
    if(marker_msg_->markers.size() <= N)
        marker_msg_->markers.resize(N);

    // initialize all markers
    int id = 1;
    for(auto& m: marker_msg_->markers) {
        m.header.frame_id = state.relativeFrameName;
        m.ns = marker_namespace;
        m.id = id++;
        m.type = Marker::SPHERE;
        m.action = Marker::ADD;

        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 0;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;

        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;

        m.points.clear();

        m.lifetime = rclcpp::Duration(30, 0);
    }

    // marker advance
    size_t ofs = 0;
    auto& markers = marker_msg_->markers;

    // lambda grabs the next marker slot and returns it (ensuring marker array has enough slots)
    auto nextMarker = [&markers, &ofs]() -> Marker& {
        auto n = ofs++;
        if(ofs > markers.size()) {
            std::cout << "visual marker estimation was not large enough (" << markers.size() << "<=" << ofs << ")" << std::endl;
            markers.resize(ofs);
        }
        return markers[n];
    };

    // if we have an odom frame in the state, include that in transform
    KDL::Frame rootTF;
    if(state.findTF(_model->odom_link, rootTF))
        rootTF = rootTF * state.transform;
    else
        rootTF = state.transform;

    std::set<std::string> show_links;
    if(show_only_updated) {
        for(size_t l = 0, _l = _model->limbs.size(); l < _l; l++) {
            auto limb = _model->limbs[l];
            if((l >= state.limbs.size()))
                continue;
            auto& limb_request = state.limbs[l];

            if(limb_request.mode >= Limb::Seeking) {
                show_links.insert(limb->segment_names.begin(), limb->segment_names.end());
            }
        }
    }

    std::map<std::string, KDL::Frame> linkTF;
    std::vector<KDL::Frame> jointTF(numberOfJoints);
    std::vector<KDL::Vector> jointAxis(numberOfJoints);
    auto& segments = _model->tree_->getSegments();
    for(const auto& linkname: _segments) {
        // get information about the chain
        auto segitr = segments.find(linkname);
        if(segitr == segments.end())
            continue;

        KDL::Frame tf;
        if(!state.findTF(linkname, tf))
            continue;

        // store TF location of segment for later
        linkTF[linkname] = tf;

        // transformed TF location
        KDL::Frame vf = rootTF * tf;

        const auto& segment = segitr->second.segment;
        const auto& joint = segment.getJoint();

        // store TF location of joint for later
        if(joint.getType() != KDL::Joint::None) {
            auto q_nr = segitr->second.q_nr;
            assert(q_nr < jointTF.size());
            jointTF[q_nr] = vf;
            jointAxis[q_nr] = joint.JointAxis();
        }

        // show links unless visibility to this link is explicitly turned off or optionally if segment wasnt updated
        if((_visibles & TF) && (!show_only_updated || std::find(show_links.begin(), show_links.end(), linkname) != show_links.end())) {
            urdf::LinkConstSharedPtr urdf_link = _urdf->links_[ linkname ];
            //urdf::JointConstSharedPtr urdf_joint = _urdf->joints_[jname];

            if(urdf_link->visual == nullptr)
                continue;

            // show the mesh or geometry
            //
            Marker& m = nextMarker();
            m.header.frame_id = state.relativeFrameName; //urdf_joint->parent_link_name;
            m.ns = marker_namespace;
            m.id = ofs;
            m.action = Marker::ADD;
            m.scale.x = m.scale.y = m.scale.z = 1.;
            m.lifetime = show_only_updated
                    ? rclcpp::Duration(2, 0)
                    : rclcpp::Duration(30, 0);

            // add translation and rotation of mesh visual
            auto visual_pose = urdf_link->visual->origin;
            KDL::Frame visual_transform(
                    KDL::Rotation::Quaternion(visual_pose.rotation.x, visual_pose.rotation.y, visual_pose.rotation.z, visual_pose.rotation.w),
                    KDL::Vector(visual_pose.position.x, visual_pose.position.y, visual_pose.position.z)
                    );

            // show link meshes
            switch (urdf_link->visual->geometry->type) {
                case urdf::Geometry::MESH: {
                    auto &mesh = dynamic_cast<urdf::Mesh &>(*urdf_link->visual->geometry);
                    m.type = Marker::MESH_RESOURCE;
                    m.mesh_resource = mesh.filename;
                    m.scale.x = mesh.scale.x;
                    m.scale.y = mesh.scale.y;
                    m.scale.z = mesh.scale.z;
                    break;
                }
                default:
                    ofs--;
                    //throw std::invalid_argument("non-mesh geometry not implemented in ghost vizualization yet");
                    continue;
            }

            kdl_frame_to_pose(vf * visual_transform, m.pose);
            setColor(m.color, ghostColor);
        }
    }


    /*
     * Update other state markers
     */
#ifdef DEBUG_LIMB_CONTACTS
    KDL::Frame base_tf;
    bool base_exists = state.findTF(_model->base_link, base_tf);
#endif

    //markers.CoM->header.stamp = _now;
    if(_visibles & CENTER_OF_MASS) {
        //bool inSupport = state.pointInSupport(state.CoM);
        sphere(nextMarker(),
               rootTF * state.CoM,
               state.inSupport ? Color::Green : Color::Red, 1);
    }

    // add Center of Pressure dot
    if(_visibles & CENTER_OF_PRESSURE)
        sphere(nextMarker(),
               rootTF * state.CoP,
               Color::Blue,1);

    // show COM to COP arrow
    if(_visibles & (CENTER_OF_PRESSURE | CENTER_OF_MASS))
        arrow(nextMarker(),
              rootTF * state.CoM,
              rootTF * state.CoP,
              Color::Red, 1.);

    // external forces vector
    if(_visibles & EXTERNAL_FORCE) {
        for(auto& kv : state.externalForces) {
        //for(size_t n=0; ef!=_ef; ef++,n++) {
            auto force = kv.second.force;
            if(force.x()==0 && force.y()==0 && force.z()==0)
                continue;   // skip zero forces

            auto tf = linkTF[kv.first];
            auto newtons = force / 100;

            auto tf_force = tf.p + newtons;
            arrow(nextMarker(),
                  rootTF * tf.p, rootTF * tf_force,
                  Color::Yellow,
                  1.2);
        }
    }

    // joint torque vectors
    if(_visibles & INTERNAL_FORCE) {
        // todo: get rid of gravity some time
        KDL::Vector G(state.CoM);
        G.z(G.z() - 0.05);
        arrow(nextMarker(),
              rootTF * state.CoM, rootTF * G,
              Color::Teal, 1);

        if(state.internalForces.rows()) {
            for(int i=0, _i = state.internalForces.rows(); i < _i; i++) {
                auto p1 = jointTF[i].p;
                auto p2 = p1;
#if 0
                // align the arrow according to the joint axis
                // might look better perpendicular, but then again arent all these vectors gravity-comp so Z still makes sense?
                auto ax = jointAxis[i];
                ax = KDL::Vector(
                        std::abs(ax.x()),
                        std::abs(ax.y()),
                        std::abs(ax.z())
                );
                p2 += (state.internalForces(i) / 2) * ax;
#else
                // align arrow along the Z axis
                p2.z( p2.z() + state.internalForces(i) / 2 );
#endif
                arrow(nextMarker(),
                      p1, p2,
                      Color::Blue, 1);
            }
        }
    }

    // end-effector contact polygons
    if(_visibles & CONTACTS) {
        for(auto& contact : state.contacts) {
            // transform points to local transform
            std::vector<KDL::Vector> points(contact.pointsInContact.size()+1);
            for(int i=0, _i=contact.pointsInContact.size(); i < _i; i++)
                points[i] = rootTF * contact.pointsInContact[i];

            // close the polyline
            points[points.size()-1] = points[0];

            // draw as polyline
            linestrip(nextMarker(),
                  points,
                  Color::Aqua);

#ifdef DEBUG_LIMB_CONTACTS
            // useful for debugging limbs in contact
            // by drawing lines to point of contact
            KDL::Vector wrt_odom = rootTF * contact.wrt_odom;
            KDL::Vector wrt_base = rootTF * base_tf * contact.wrt_base;
            KDL::Vector wrt_com = rootTF * (state.CoM + contact.wrt_CoM);
            if(base_exists) {
                //arrow(nextMarker(),
                //      base_tf.p, wrt_odom,
                //      Color::White, 1);
                arrow(nextMarker(),
                      rootTF.p, wrt_odom,
                      Color::White, 1);
                arrow(nextMarker(),
                      rootTF * base_tf.p, wrt_base,
                      Color::Yellow, 1);
                arrow(nextMarker(),
                      rootTF * state.CoM, wrt_com,
                      Color::Fuchsia, 1);
            }
#endif
        }
    }

    if(_visibles & SUPPORT_POLYGON) {
        // transform points to local transform
        std::vector<KDL::Vector> points(state.supportPolygon.size()+1);
        for(int i=0, _i=state.supportPolygon.size(); i < _i; i++)
            points[i] = rootTF * state.supportPolygon[i];

        // close the polyline
        points[points.size()-1] = points[0];

        // draw as polyline
        linestrip(nextMarker(),
                  points,
                  Color::Navy);
    }

    // all remaining available markers should be marked to DELETE
    while(ofs < marker_msg_->markers.size()) {
        marker_msg_->markers[ofs++].action = Marker::DELETE;
    }

    return true;
}

} // ns::robot
