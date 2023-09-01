//
// Created by guru on 1/7/22.
//

#ifndef ROBOT_DYNAMICS_EFFECTOR_H
#define ROBOT_DYNAMICS_EFFECTOR_H

#include <memory>
#include <vector>

#include "exception.h"
#include "types.h"

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
        Unassigned,
        Limp,                   /// end-effector servos should go limp
        Hold,                   /// this end-effector should try to hold the given position
        Support,                /// this end-effector is supporting the robot (ex. nailed to floor)
        AutoSupport             /// the contact detection algorithm determines if this limb is supporting or not
        //Seeking,                /// this end-effector should follow the given targetTF value
        //SupportSeeking        /// this end-effector is supporting the robot while seeking a target
        //Stepping,               /// this end-effector should perform a stepping motion to improve balance stability
        //Manipulating            /// user is currently manipulating this limb and has control
    };

    enum class Status {
        Limp = Mode::Limp,
        Holding,
        Supporting,
        Seeking
    };

    ///@brief Origin from which limb coordinates will be given (relative to robot base)
    KDL::Frame origin;

    ///@brief Indicates how a limb should be used
    class State {
    public:
        ///@brief What control mode this end-effector is currently in
        Mode mode;

        Status status;

        ///@brief Current position of the end effector
        /// This frame is relative to the limb base frame (usually the base link).
        KDL::Frame position;

        ///@brief Current velocity of this end effector if it is moving
        KDL::Twist velocity;

        ///@brief Target trajectory position of the end effector
        /// This frame is relative to the limb base frame (usually the base link). This position is updated by
        /// active trajectory actions.
        KDL::Frame target;

        inline explicit State(Mode _mode = Mode::Limp)
        : mode(_mode), status(Status::Limp)
        {}

        ///@brief Returns true if the effector is supporting the robot
        [[nodiscard]] inline bool is_supporting() const {
            return mode == Support || (mode == AutoSupport && status == Status::Supporting);
        }

        void limp() {
            mode = Limp;
            status = Status::Limp;
        }

        void apply(const KDL::Frame& pose, CoordinateMask mask = CoordinateMask::All);
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
        return std::find_if(begin(), end(), [&s](const ET& e) { return e.model->link == s; });
    }

    inline iterator find(const std::string& s) {
        return std::find_if(begin(), end(), [&s](const ET& e) { return e->link == s; });
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
        inline State() = default;
        inline explicit State(BaseEffector::SharedPtr base_model)
        : model(std::move(base_model)) {}

        BaseEffector::SharedPtr model;
    };

    BaseEffector(
            const std::shared_ptr<KDL::Tree>& tree,
            std::string base_link);

    std::shared_ptr<KDL::Tree> tree;
    std::string link;

};

using BaseStates = Effectors<BaseEffector::State>;

Effector::Mode effector_mode_from_msg(int8_t m);
int8_t effector_mode_to_msg(Effector::Mode m);

int8_t effector_status_to_msg(Effector::Status s);
Effector::Status effector_status_from_msg(int8_t s);

template<> [[nodiscard]] BaseStates::iterator BaseStates::find(const std::string& s);
template<> [[nodiscard]] BaseStates::const_iterator BaseStates::find(const std::string& s) const;


} //ns:robotik
#endif //ROBOT_DYNAMICS_EFFECTOR_H
