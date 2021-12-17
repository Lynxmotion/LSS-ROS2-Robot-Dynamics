//
// Created by guru on 4/1/20.
//

#include <conversions.h>
#include <iomanip>

void kdl_vector_to_point(const KDL::Vector& src, geometry_msgs::msg::Point& dest) {
    dest.x = src.x();
    dest.y = src.y();
    dest.z = src.z();
}

void kdl_vector_to_vector(const KDL::Vector& src, geometry_msgs::msg::Vector3& dest) {
    dest.x = src.x();
    dest.y = src.y();
    dest.z = src.z();
}

geometry_msgs::msg::Vector3 to_vector(const KDL::Vector& src)
{
    geometry_msgs::msg::Vector3 dest;
    dest.x = src.x();
    dest.y = src.y();
    dest.z = src.z();
    return dest;
}

void kdl_vector_to_vector(const std::vector<KDL::Vector>& src, std::vector<geometry_msgs::msg::Vector3>& dest)
{
    if(src.size() != dest.size())
        dest.resize(src.size());
    for(size_t i=0, _i=src.size(); i < _i; i++) {
        kdl_vector_to_vector(src[i], dest[i]);
    }
}

void vector_to_kdl_vector(const geometry_msgs::msg::Vector3& src, KDL::Vector& dest)
{
    dest.x(src.x);
    dest.y(src.y);
    dest.z(src.z);
}

void vector_to_kdl_vector(const std::vector<geometry_msgs::msg::Vector3>& src, std::vector<KDL::Vector>& dest)
{
    if(src.size() != dest.size())
        dest.resize(src.size());
    for(size_t i=0, _i=src.size(); i < _i; i++) {
        vector_to_kdl_vector(src[i], dest[i]);
    }
}

void quat_to_kdl_rotation(const geometry_msgs::msg::Quaternion& src, KDL::Rotation& dest)
{
    dest = KDL::Rotation::Quaternion(src.x, src.y, src.z, src.w);
}

geometry_msgs::msg::Quaternion to_quat(const KDL::Rotation& src) {
    geometry_msgs::msg::Quaternion dest;
    src.GetQuaternion(dest.x, dest.y, dest.z, dest.w);
    return dest;
}

void quat_to_kdl_rotation(const std::vector<geometry_msgs::msg::Quaternion>& src, std::vector<KDL::Rotation>& dest)
{
    if(src.size() != dest.size())
        dest.resize(src.size());
    for(size_t i=0, _i=src.size(); i < _i; i++) {
        quat_to_kdl_rotation(src[i], dest[i]);
    }
}

void kdl_rotation_to_quat(const KDL::Rotation& src, geometry_msgs::msg::Quaternion& dest) {
    src.GetQuaternion(dest.x, dest.y, dest.z, dest.w);
}

void kdl_frame_to_transform(const KDL::Frame& frame, geometry_msgs::msg::Transform& t) {
    tf2::Quaternion quat;
    kdl_vector_to_vector(frame.p, t.translation);
    kdl_rotation_to_quat(frame.M, t.rotation);
}

void kdl_frame_to_pose(const KDL::Frame& frame, geometry_msgs::msg::Pose& pose) {
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void pose_to_kdl_frame(const geometry_msgs::msg::Pose& pose, KDL::Frame& frame) {
    frame.p.x(pose.position.x);
    frame.p.y(pose.position.y);
    frame.p.z(pose.position.z);
    frame.M = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

namespace tf {
    void fromMsg(const geometry_msgs::msg::Wrench& in, KDL::Wrench& out)
    {
        vector_to_kdl_vector(in.force, out.force);
        vector_to_kdl_vector(in.torque, out.torque);
    }

    void toMsg(const KDL::Wrench& in, geometry_msgs::msg::Wrench& out)
    {
        kdl_vector_to_vector(in.force, out.force);
        kdl_vector_to_vector(in.torque, out.torque);
    }

    void fromMsg(const geometry_msgs::msg::Twist& in, KDL::Twist& out)
    {
        vector_to_kdl_vector(in.angular, out.rot);
        vector_to_kdl_vector(in.linear, out.vel);
    }

    void toMsg(const KDL::Twist& in, geometry_msgs::msg::Twist& out)
    {
        kdl_vector_to_vector(in.rot, out.angular);
        kdl_vector_to_vector(in.vel, out.linear);
    }
}

std::ostream& operator<<(std::ostream& sout, const KDL::JntArray& arr) {
    sout << "JntArray[" << arr.columns() << 'x' << arr.rows() << ']' << std::endl;
    for(int i=0, _i=arr.rows(); i<_i; i++) {
        if(i==0)
            sout << "[ ";
        else
            sout << "  ";
        for(int j=0, _j=arr.columns(); j<_j; j++) {
            auto v = arr(i, j);
            if(j>0) sout << ", ";
            sout << v;
        }
        if(i+1 == _i)
            sout << ']';
        sout << std::endl;
    }
    return sout;
}

std::ostream& operator<<(std::ostream& sout, const KDL::Vector& vec) {
    sout << std::setprecision(5)  << std::fixed << "V[" << vec.x() << ',' << vec.y() << ',' << vec.z() << ']';
    return sout;
}

#if !defined(HAVE_KDL_FRAMES_IO)
std::ostream& operator<<(std::ostream& sout, const KDL::Rotation& q) {
    double x, y, z, w;
    q.GetQuaternion(x, y, z, w);
    sout << std::setprecision(5) << std::fixed << "R[" << x << ',' << y << ',' << z << ',' << w << ']';
    return sout;
}

std::ostream& operator<<(std::ostream& sout, const KDL::Frame& f) {
    sout << "F[" << f.M << " @ " << f.p << ']';
    return sout;
}
#endif

