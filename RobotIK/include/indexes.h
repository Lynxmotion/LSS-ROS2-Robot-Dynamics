//
// Created by guru on 1/2/22.
//

#ifndef ROBOT_DYNAMICS_INDEXES_H
#define ROBOT_DYNAMICS_INDEXES_H

#include "state.h"

namespace robotik {

    ///@brief Resolves the ordinal index into a JointState joint array using an ordinal from a different source array
    /// This index is primarily used to speed up joint lookups when updating from a Ros message. The joint order in
    /// Ros messages are probably not in the same order as they are in the JointState and are not even guaranteed to be
    /// consistent from one message to another although 99% of the time they are. The `input_ordinal` value is used
    /// to lookup a guess value and the joint at that position in the JointState array is checked to see if the name
    /// matches. If matched, then the value is quickly returned otherwise the index is updated to reflect the new
    /// guess value.
    class JointStateOrdinalMap {
    public:
        inline void resize(size_t new_size) {
            joint_trajectory_index.resize(new_size);
        }

        inline size_t size() const { return joint_trajectory_index.size(); }

        ///@brief Resolve the input ordinal index to the JointState joint ordinal
        /// Returns the index into the JointState joints (position, velocity, effort) arrays, or -1 if joint not found
        ssize_t resolve(
                const JointState& state,
                size_t input_ordinal,
                const std::string& joint_name);

        ///@brief Resolve the input ordinal index to the JointState joint ordinal or adds a new joint by name
        /// Returns the index into the JointState joints (position, velocity, effort) arrays,
        ///    or adds the joint to the joints arrays using the mask parameter.
        ssize_t resolve_or_insert(
                JointState& state,
                size_t input_ordinal,
                const std::string& joint_name,
                unsigned int mask = robotik::DEFAULT_PARAMETERS);

    private:
        // quickly resolves joint names to state index 99% of the time
        // fallback uses state->addJoint(name) which does a binary search and allocate if required
        std::vector<ssize_t> joint_trajectory_index;
    };

} //ns: robotnik
#endif //ROBOT_DYNAMICS_INDEXES_H
