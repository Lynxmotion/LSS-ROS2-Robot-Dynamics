//
// Created by guru on 1/8/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_ENVIRONMENT_H
#define HUMANOID_DYNAMIC_MODEL_ENVIRONMENT_H

#include "expression.h"
#include "state.h"

namespace robotik {
namespace trajectory {

/// @brief trajectory rendering environment for accessing trajectory and projected robot state
class RenderingInterface {
public:
    //virtual double getFrameTime() const = 0;

    virtual KDL::Frame get_relative_frame(const FrameRef& ref, double at_timestamp) = 0;

    virtual KDL::Frame convert_frame(FrameRef from_ref, KDL::Frame frame, FrameRef to_ref) = 0;
};


} // ns:trajectory
} // ns:robotik

#endif //HUMANOID_DYNAMIC_MODEL_ENVIRONMENT_H
