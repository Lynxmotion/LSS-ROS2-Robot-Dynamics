//
// Created by guru on 1/2/22.
//

#include <indexes.h>

namespace robotik {

ssize_t JointStateOrdinalMap::resolve(const JointState& state, size_t input_ordinal, const std::string& joint_name)
{
    // lookup joint position using index
    if(input_ordinal >= joint_trajectory_index.size())
        joint_trajectory_index.resize(input_ordinal + 1);
    auto j_idx = joint_trajectory_index[input_ordinal];

    // verify index is correct, if not make a correction
    if(j_idx < 0 || j_idx >= (ssize_t)state.joints.size() || state.joints[j_idx] != joint_name) {
        // find it the long way
        auto j_itr = std::find(state.joints.begin(), state.joints.end(), joint_name);
        if (j_itr != state.joints.end()) {
            // get the element position in JointState
            ssize_t ord = j_itr - state.joints.begin();
            // ensure the index is big enough to map the values
            if(ord >= (ssize_t)joint_trajectory_index.size())
                joint_trajectory_index.resize(ord);
            // add the mapping into the index
            return joint_trajectory_index[input_ordinal] = ord;
        } else {
            //throw std::runtime_error("joint " + joint_name + " not found in state");
            return -1;
        }
    }

    return j_idx;
}

ssize_t JointStateOrdinalMap::resolve_or_insert(JointState& state, size_t input_ordinal, const std::string& joint_name, unsigned int mask)
{
    auto ord = resolve(state, input_ordinal, joint_name);
    if(ord < 0)
        ord = state.addJoint(joint_name, mask);
    return ord;
}

} //ns: robotnik
