//
// Created by guru on 3/31/20.
//

#ifndef LSS_HUMANOID_CONVERSIONS_H
#define LSS_HUMANOID_CONVERSIONS_H

#include "imports.h"
#include <kdl/frames_io.hpp>

// todo: move to CMake include test
#define HAVE_KDL_FRAMES_IO

void kdl_vector_to_point(const KDL::Vector& src, geometry_msgs::msg::Point& dest);

void kdl_vector_to_vector(const KDL::Vector& src, geometry_msgs::msg::Vector3& dest);
void kdl_vector_to_vector(const std::vector<KDL::Vector>& src, std::vector<geometry_msgs::msg::Vector3>& dest);

geometry_msgs::msg::Vector3 to_vector(const KDL::Vector& src);

void kdl_rotation_to_quat(const KDL::Rotation& src, geometry_msgs::msg::Quaternion& dest);

geometry_msgs::msg::Quaternion to_quat(const KDL::Rotation& src);

void kdl_frame_to_transform(const KDL::Frame& frame, geometry_msgs::msg::Transform& t);

void kdl_frame_to_pose(const KDL::Frame& frame, geometry_msgs::msg::Pose& pose);

void pose_to_kdl_frame(const geometry_msgs::msg::Pose& pose, KDL::Frame& frame);

void vector_to_kdl_vector(const geometry_msgs::msg::Vector3& src, KDL::Vector& dest);
void vector_to_kdl_vector(const std::vector<geometry_msgs::msg::Vector3>& src, std::vector<KDL::Vector>& dest);

void quat_to_kdl_rotation(const geometry_msgs::msg::Quaternion& src, KDL::Rotation& dest);
void quat_to_kdl_rotation(const std::vector<geometry_msgs::msg::Quaternion>& src, std::vector<KDL::Rotation>& dest);

namespace tf2 {
    // KDL wrench
    void fromMsg(const geometry_msgs::msg::Wrench& in, KDL::Wrench& out);
    void toMsg(const KDL::Wrench& in, geometry_msgs::msg::Wrench& out);

    // KDL twist
    void fromMsg(const geometry_msgs::msg::Twist& in, KDL::Twist& out);
    void toMsg(const KDL::Twist& in, geometry_msgs::msg::Twist& out);

    // KDL Transform
    void fromMsg(const geometry_msgs::msg::Transform& in, KDL::Frame& out);
    void toMsg(const KDL::Frame& in, geometry_msgs::msg::Transform& out);
}

inline KDL::Rotation to_kdl_rotation(const geometry_msgs::msg::Quaternion& src)
{
    KDL::Rotation dest;
    dest = KDL::Rotation::Quaternion(src.x, src.y, src.z, src.w);
    return dest;
}

template<class T>
KDL::Vector kdl_vector_from_xyz(const T& src) {
    KDL::Vector dest;
    dest.x( src.x );
    dest.y( src.y );
    dest.z( src.z );
    return dest;
}


std::ostream& operator<<(std::ostream& sout, const KDL::JntArray& arr);

std::ostream& operator<<(std::ostream& sout, const KDL::Vector& vec);

#if !defined(HAVE_KDL_FRAMES_IO)
//std::ostream& operator<<(std::ostream& sout, const KDL::Rotation& q);

//std::ostream& operator<<(std::ostream& sout, const KDL::Frame& f);
#endif

template<size_t Rows, size_t Cols, typename T>
std::ostream& array2D(std::ostream& sout, const T(&arr)[Rows*Cols]) {
    sout << "[" << Cols << 'x' << Rows << ']' << std::endl;
    for(int i=0, _i=Rows; i<_i; i++) {
        if(i==0)
            sout << "[ ";
        else
            sout << "  ";
        for(int j=0, _j=Cols; j<_j; j++) {
            auto v = arr[i*Cols + j];
            if(j>0) sout << ", ";
            sout << v;
        }
        if(i+1 == _i)
            sout << ']';
        sout << std::endl;
    }
    return sout;
}

#endif //LSS_HUMANOID_CONVERSIONS_H
