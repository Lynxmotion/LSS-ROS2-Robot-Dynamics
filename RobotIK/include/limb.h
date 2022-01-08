//
// Created by guru on 1/1/20.
//

#pragma once

#include <memory>

#include "types.h"
#include "friction.h"
#include "effector.h"

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>


namespace robotik {

class JointAndSegmentState;
class JointState;
class Model;

class Limb : public Effector {
public:
    using SharedPtr = std::shared_ptr<Limb>;

    ///@brief A flag indicating what this limb is intended for
    /// Legs will be used primary for robot support and locomotion. Arms for object manipulation or if free may also be
    /// used for dynamic CoM balance support or rigid-contact robot support.
    /// You can also change this flag at run-time to change the behaviour of a limb if your robot structure is geared
    /// for it.
    typedef enum {
        Generic,
        Leg,
        Arm
    } DynamicModelType;

    ///@brief Indicates how a limb should be used
    class State : public Effector::State {
    public:
        using SharedPtr = std::shared_ptr<State>;

        Limb::SharedPtr model;

        ///@brief true if this limb can be used to support the robot
        /// This flag indicates if this limb can be recruited to support the robot. If false, the modes BalanceSupport,
        /// and Stepping would be invalid.
        bool supportive;

        ///@brief true if this limb is currently being used to support the robot
        bool supporting;

        ///@brief how soft this limb should react to forces against it
        double compliance;

        State();
        explicit State(Limb::SharedPtr model, Mode _mode = Limp);
        explicit State(Limb::SharedPtr model, Mode _mode, bool _supportive);
    };

    explicit Limb(
            const std::shared_ptr<KDL::Tree>& tree,
            std::string from_link,
            std::string to_link,
            DynamicModelType type=DynamicModelType::Generic);

    [[nodiscard]] inline unsigned int getNrOfJoints() const { return chain->getNrOfJoints(); }

    ///@brief load limb data and allocate memory for internal variables
    bool on_activate();

    ///@brief deallocate resources for the limb
    bool on_deactivate();

    inline KDL::Chain* get_chain() { return chain.get(); }


    std::shared_ptr<KDL::Tree> tree;
    DynamicModelType model;
    std::string from_link;
    std::string to_link;

    std::vector<std::string> joint_names;
    std::vector<std::string> segment_names;

    ///@brief Origin from which limb coordinates will be given (relative to robot base)
    KDL::Frame origin;

    ///@brief The friction between this limb and the ground/floor surface
    Friction friction;

    ///@brief the direction from the robot base to which this limb is attached
    /// This is a directional vector rooted at the base to the limb's base attachment point. It is used to determine
    /// normal support stance where legs are supporting equi-distant from the base.
    KDL::Vector direction;

    ///@brief the untranslated support polygon of this end-effector
    /// For legs this is  effectively the supprt rectangle (usually) of the sole of the foot. It is determined by loading
    /// the mesh vertices and performing 'convex hull' analysis on the points to find the enclosing polygon. It prefers
    /// the collision mesh but will fallback to visual mesh if required. For arms this polygon is not yet used but could
    /// also be used to determine point of contact.
    std::vector<KDL::Vector> supportPolygon;

    ///@brief Load the mesh model for the limb and compute a support polygon for contact detection
    void loadSupportPolygon(urdf::ModelInterfaceSharedPtr urdf_model_);

protected:
    ///@brief Orokos API chain for this limb
    std::unique_ptr<KDL::Chain> chain;
};

class LimbModels : public std::vector<Limb::SharedPtr>
{
public:
};

class Limbs : public std::vector<Limb::State>
{
public:
    using std::vector<Limb::State>::operator[];

    inline const_iterator find(const std::string& s) const {
        return std::find_if(begin(), end(), [&s](const Limb::State& l) { return l.model->to_link == s; });
    }

    inline iterator find(const std::string& s) {
        return std::find_if(begin(), end(), [&s](const Limb::State& l) { return l.model->to_link == s; });
    }

    inline Limb::State& operator[](const std::string& s) {
        iterator itr = find(s);
        if(itr == end())
            throw robotik::Exception::LimbNotFound(s);
        return *itr;
    }

    inline const Limb::State& operator[](const std::string& s) const {
        const_iterator itr = find(s);
        if(itr == end())
            throw robotik::Exception::LimbNotFound(s);
        return *itr;
    }

    void zero();

    static Limbs fromModel(const Model& model);
};


} // ns::robotik
