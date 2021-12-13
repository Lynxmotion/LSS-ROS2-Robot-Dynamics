//
// Created by guru on 1/9/21.
//

#ifndef HUMANOID_DYNAMIC_MODEL_MODEL_H
#define HUMANOID_DYNAMIC_MODEL_MODEL_H

#include <kdl/frames.hpp>

#include "state.h"
#include "frame-ref.h"

namespace robotik {

class ModelInterface {
public:
    virtual KDL::Frame getRelativeFrame(const FrameRef& ref, const State& state) = 0;
};


} // ns:robotik

#endif //HUMANOID_DYNAMIC_MODEL_MODEL_H
