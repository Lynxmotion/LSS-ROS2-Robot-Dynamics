//
// Created by guru on 1/1/20.
//

#include <limb.h>
#include <model.h>
#include <state.h>


///@brief Log information to console about limb and limb IK errors
/// If defined, will log information about each limb Orokos Chain, and will output IK errors when they occur. If defined
/// as 2 or greater will output more joint detail for errors.
//#define DEBUG_LIMB 1

namespace robotik {

Limb::Limb(const Options& options)
    : options_(options), friction(Friction::fromTable(Aluminum, Wood))
{
}


bool Limb::on_activate() {
    // pull the chain out of the URDF file
    chain = std::make_unique<KDL::Chain>();
    if(! options_.tree->getChain(options_.from_link, options_.to_link, *chain)) {
        chain.reset();
        return false;
    }

    auto nj = chain->getNrOfSegments();

#if defined(DEBUG_LIMB)
    std::cout << "Chain  " << options_.from_link << " => " << options_.to_link << std::endl;
#endif

    // get information about the chain
    KDL::Frame tf;
    for(unsigned int s=0; s<nj; s++) {
        const auto& segment = chain->getSegment(s);
        const auto& joint = segment.getJoint();

        joint_names.push_back(joint.getName());
        segment_names.push_back(segment.getName());

        tf = tf * segment.pose(0);

#if defined(DEBUG_LIMB)
        auto inertia = segment.getInertia();
        std::cout << "segment: " << segment.getName()
                << "   joint: " << segment.getJoint().getName()
                << "   m:" << inertia.getMass()
                << "   t:" << joint.getTypeName()
                << std::endl;
                /*<< "   cog:" << inertia.getCOG()
                << "   i:";
            array2D<3,3>(ss, inertia.getRotationalInertia().data )*/;
#endif
    }

    // preallocate memory for IK variables
    joint_names.resize(nj);
    q.resize(nj);
    qdot.resize(nj);
    qdotdot.resize(nj);

    return true;
}

bool Limb::on_deactivate() {
    chain.reset();
    joint_names.clear();
    q.resize(0);
    qdot.resize(0);
    qdotdot.resize(0);
    return true;
}

KDL::Frame Limb::computeTFfromBase(const JointState& state) {
#if 0
    // this calls Orokos API to get the endpoint using FK
    // to do: The input 'q' needs to be mapped from state.position, and the result has to be mapped back to state algo frame
    KDL::Frame f;
    if(chain) {
        // to do: fill 'q' with positions from state
        KDL::ChainFkSolverPos_recursive solver(*chain);
        solver.JntToCart(q, f);
        // to do: map 'f' back to state frame
    }
    return f;
#else
    // get information about the chain
    KDL::Frame tf;
    auto nj = chain->getNrOfSegments();
    for(unsigned int s=0; s<nj; s++) {
        const auto &segment = chain->getSegment(s);
        const auto &joint = segment.getJoint();
        double pos = 0;

        if(joint.getType() != KDL::Joint::None) {
            const auto jname = joint.getName();
            int jn = state.findJoint(jname);
            if (jn < 0)
                throw Exception::JointNotFound(jname);
            pos = state.position(jn);
        }

        tf = tf * segment.pose(pos);
    }
    return tf;
#endif
}

void Limb::loadSupportPolygon(urdf::ModelInterfaceSharedPtr urdf_model_) {
    urdf::LinkConstSharedPtr eff_link =  urdf_model_->getLink(options_.to_link);
    assert(eff_link);

    urdf::GeometryConstSharedPtr geom;
    urdf::Pose geom_pose;
    if (eff_link->collision && eff_link->collision->geometry){
        geom = eff_link->collision->geometry;
        geom_pose = eff_link->collision->origin;
    } else if (eff_link->visual && eff_link->visual->geometry){
        geom = eff_link->visual->geometry;
        geom_pose = eff_link->visual->origin;
    } else{
        throw robotik::Exception(RE_URDF_GEOMETRY_ERROR, "Unable to determine support polygon for %s", options_.to_link.c_str());
    }

    auto origin = KDL::Frame(
        KDL::Rotation::Quaternion(geom_pose.rotation.x, geom_pose.rotation.y, geom_pose.rotation.z, geom_pose.rotation.w),
     KDL::Vector(geom_pose.position.x, geom_pose.position.y, geom_pose.position.z)
    );

    bool transform_to_origin = true;
    std::vector<KDL::Vector> points;
    switch (geom->type) {
        case urdf::Geometry::BOX: {
            auto &box = dynamic_cast<const urdf::Box &>(*geom);
            double x = box.dim.x/2, y = box.dim.y/2;
            points.emplace_back(+x, +y, 0.);
            points.emplace_back(-x, +y, 0.);
            points.emplace_back(-x, -y, 0.);
            points.emplace_back(+x, -y, 0.);
            break;
        }
        case urdf::Geometry::MESH:
            // todo: implement mesh parsing of support polygon (when geometry_shapes package is available in ROS2 Eloquent)
#if 0  // this code depends on geometry_shapes which isnt available in Eloquent (yet?)
            transform_to_origin = false;    // we'll do our own transform step
            auto &mesh = dynamic_cast<const urdf::Mesh &>(*geom);

            const Eigen::Vector3d scale(mesh.scale.x, mesh.scale.y, mesh.scale.z);
            shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh.filename, scale);
            size_t vertex_count = shape_mesh->vertex_count;

            //Vector storing the original foot points
            std::vector<KDL::Vector> foot_SP_right;
            for (unsigned int i = 0 ; i < vertex_count ; ++i)
            {
                unsigned int i3 = i * 3;

                KDL::Vector p(shape_mesh->vertices[i3], shape_mesh->vertices[i3 + 1], shape_mesh->vertices[i3 + 2]); // proj down (z=0)
                auto projectedP = origin * p;
                projectedP.z(0.);
                // transform into local foot frame:
                //foot_support_polygon_right_.push_back(projectedP);
                foot_SP_right.push_back(projectedP);
            }

            //Compute foot center point w.r.t local frame
            float sum_x_coord = 0.0;
            float sum_y_coord = 0.0;
            KDL::Vector r_foot_center;
            for (unsigned int i = 0 ; i < foot_SP_right.size(); ++i)
            {
                sum_x_coord = sum_x_coord + foot_SP_right[i].x();
                sum_y_coord = sum_y_coord + foot_SP_right[i].y();
            }
            //X and Y of right foot center
            r_foot_center.x(sum_x_coord/foot_SP_right.size());
            r_foot_center.y(sum_y_coord/foot_SP_right.size());

            //Vector storing foot points w.r.t foot center
            std::vector<KDL::Vector> foot_SP_right_center;
            KDL::Vector foot_point;
            for (unsigned int i = 0 ; i < foot_SP_right.size(); ++i){
                //Express point w.r.t foot center and directly apply scaling
                foot_point.x( (foot_SP_right[i].x() - r_foot_center.x()) * foot_polygon_scale_ );
                foot_point.y( (foot_SP_right[i].y() - r_foot_center.y()) * foot_polygon_scale_ );
                foot_SP_right_center.push_back(foot_point);
            }

            //Express new(scaled) coordinates in local frame
            std::vector<KDL::Vector> scaled_SP_right;
            for (unsigned int i = 0 ; i < foot_SP_right_center.size() ; ++i){
                //Express point w.r.t foot center and directly apply scaling
                foot_point.x( foot_SP_right_center[i].x() + r_foot_center.x() ) ;
                foot_point.y( foot_SP_right_center[i].y() + r_foot_center.y() ) ;
                scaled_SP_right.push_back(foot_point);
            }

            //std::cout<<"Num points"<<scaled_SP_right.size()<<std::endl;

            //Without scaling
            //foot_support_polygon_right_ = convexHull(foot_support_polygon_right_);
            //foot_support_polygon_right_ = convexHull(foot_SP_right);

            //With scaling
            //foot_support_polygon_right_ = convexHull(scaled_SP_right);
            points = foot_support_polygon_right_;
#else
            throw robotik::Exception(RE_URDF_GEOMETRY_ERROR, "Currently MESH geometry is not supported for support polygons, but wouldn't be hard to do, add a github issue.");
#endif
        //case urdf::Geometry::SPHERE:
        //case urdf::Geometry::CYLINDER:
        default:
            throw robotik::Exception(RE_URDF_GEOMETRY_ERROR, "Currently only BOX geometry is supported for support polygons");
    }

    // transform points according to the geo origin
    if(transform_to_origin) {
        for(auto& p: points) {
            p = origin * p;
        }
    }

    // loaded the support polygon
    supportPolygon = points;
}

Limb::State::State()
: mode(Limp), limbType(Generic), supportive(false), supporting(false)
{
}

Limb::State::State(Limb::SharedPtr _model, Mode _mode)
: model(_model), mode(_mode), limbType(_model->options_.model), supportive(_model->options_.model == Leg), supporting(false)
{
}

Limb::State::State(Limb::SharedPtr _model, Mode _mode, bool _supportive)
: model(_model), mode(_mode), limbType(_model->options_.model), supportive(_supportive), supporting(false)
{
}


void Limbs::zero()
{
    for(auto& limb: *this) {
        limb.mode = (limb.limbType == Limb::Leg) ? Limb::Holding : Limb::Limp;
        limb.position = KDL::Frame();
        limb.velocity = KDL::Twist();
        limb.target = KDL::Frame();
    }
}

Limbs Limbs::fromModel(const Model& model)
{
    Limbs limbs;
    // ensure we have the same number of limbs
    limbs.reserve(model.limbs.size());
    for(auto& l: model.limbs) {
        auto isLeg = l->options_.model == robotik::Limb::Leg;
        limbs.emplace_back(
                l,
                Limb::Limp,
                isLeg);
    }
    return limbs;
}

} // ns::robot
