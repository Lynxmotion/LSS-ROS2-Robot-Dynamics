//
// Created by guru on 1/7/22.
//

#ifndef ROBOT_DYNAMICS_EFFECTOR_H
#define ROBOT_DYNAMICS_EFFECTOR_H

#include <memory>
#include <vector>

#include "exception.h"

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>


namespace robotik {

class JointAndSegmentState;
class JointState;
class Model;

class Effector {
public:
    using SharedPtr = std::shared_ptr<Effector>;

    ///@brief Current behavior mode this limb is in
    enum Mode {
        Limp,                   /// end-effector servos should go limp
        Holding,                /// this end-effector should try to hold the given position
        BalanceSupport,         /// use this end-effector to remain stable/balanced and support the robot
        Stepping,               /// this end-effector should perform a stepping motion to improve balance stability
        Seeking,                /// this end-effector should follow the given targetTF value
        Manipulating            /// user is currently manipulating this limb and has control
    };

    ///@brief Indicates how a limb should be used
    class State {
    public:
        ///@brief What control mode this end-effector is currently in
        Mode mode;

        ///@brief Current position of the end effector
        /// This frame is relative to the limb base frame (usually the base link).
        KDL::Frame position;

        ///@brief Current velocity of this end effector if it is moving
        KDL::Twist velocity;

        ///@brief Target trajectory position of the end effector
        /// This frame is relative to the limb base frame (usually the base link). This position is updated by
        /// active trajectory actions.
        KDL::Frame target;

        inline explicit State(Mode _mode = Limp)
        : mode(_mode)
        {}
    };
};

template<class ET>
class Effectors : public std::vector<ET>
{
public:
    using typename std::vector<ET>::iterator;
    using typename std::vector<ET>::const_iterator;
    using std::vector<ET>::operator[];
    using std::vector<ET>::begin;
    using std::vector<ET>::end;

    inline const_iterator find(const std::string& s) const {
        return std::find_if(begin(), end(), [&s](const ET& e) { return e.model->to_link == s; });
    }

    inline iterator find(const std::string& s) {
        return std::find_if(begin(), end(), [&s](const Effectors& e) { return e.model->to_link == s; });
    }

    inline ET& operator[](const std::string& s) {
        iterator itr = find(s);
        if(itr == end())
            throw Exception::LimbNotFound(s);
        return *itr;
    }

    inline const ET& operator[](const std::string& s) const {
        const_iterator itr = find(s);
        if(itr == end())
            throw Exception::LimbNotFound(s);
        return *itr;
    }
};

class BaseEffector : public Effector {
public:
    using SharedPtr = std::shared_ptr<BaseEffector>;

    class State : public Effector::State {
    public:
        BaseEffector::SharedPtr model;
    };

    BaseEffector(
            const std::shared_ptr<KDL::Tree>& tree,
            std::string base_link);

    std::shared_ptr<KDL::Tree> tree;
    std::string link;

};

} //ns:robotik
#endif //ROBOT_DYNAMICS_EFFECTOR_H
