//
// Created by guru on 6/18/20.
//

#ifndef LSS_HUMANOID_CALIBRATION_H
#define LSS_HUMANOID_CALIBRATION_H

#include "types.h"
#include "visual.h"
#include "interaction.h"


namespace robotik {

class State;
class Model;

/* todo: Calibration should
 *      * Lock feet to the floor - moving one foot moves the other?
 *      * maybe foot lock only if a calibration pose is selected
 *      * set feet up square?
 *      * calibration detection movement, such as feel with arm?
 */


class JointCalibration : public InteractionController {
public:
    using SharedPtr = std::shared_ptr<JointCalibration>;

    using Data = std::map<std::string, double>;

    State::SharedPtr state;

    std::string interaction_namespace() override;

    std::set<std::string> manipulators(State& state) override;

    ///@brief Handle interaction events and update calibration
    bool interact(InteractionEvent& ev) override;

    ///@brief Initiate calibration mode by initializing robot calibration state
    void begin(Model::SharedPtr model, State& _state);

    ///@brief Commit the joint offsets to non-volatile config
    Data commit();

    ///@brief Clear any stored calibration updates
    void clear();

    ///@brief Returns true if there are pending calibration updates to perform
    inline bool hasJointData() const { return !jointUpdates.empty(); }

    ///@brief Updates the given state with frame transforms from the calibration state
    /// Typically this is used to update visuals and not calibrating the joints specifically. We have no awareness of
    /// the model at this point so we only update the manipulated segment and the caller must update IK at some point
    /// to update connected joints in the chain. Call this before your normal IK update in your loop and you are golden
    /// so long as manipulated segments are limited to your limb endpoints.
    Data update(Model& model, State& _state);

    ///@brief Updates the given state with frame transforms from the calibration state then updates the visual state
    /// This first calls the update(model, state) method then also updates the visual state with the new
    /// calibration state.
    Data update(Model& _model, State& _state, StateVisual&_visual);


protected:
    class CalibratedJoint {
    public:
        KDL::Frame endpoint;                    // the new position of the joint (in state frame)
        bool updated;                           // true if this joint's endpoint was moved (manipulated since calibration state update)
        bool transmitted;                       // true if this joint's recalibration was transmitted to joint controller

        // collection of segments that whose calibration angles were updated (and should be highlighted)
        std::set<std::string> segmentsCalibrated;

        // KDL IK variables
        std::shared_ptr<KDL::Chain> chain;      // KDL IK chain from this joint endpoint to robot base
        std::vector<ssize_t> jnt_pos_mapping;
        KDL::JntArray jnt_original;
        KDL::JntArray jnt_quess;
        KDL::JntArray q_out;

        CalibratedJoint() : updated(false), transmitted(false) {}

        void calibrate(const KDL::Frame& f) {
            endpoint = f;
            updated = true;
            transmitted = false;
        }
    };

    Model::SharedPtr model_;

    ///@brief stores update locations of joints from the interactive manipulators.
    /// We dont have state, model or IK data so we must store as frames then resolve the actual IK and angle data later.
    std::map<std::string, CalibratedJoint> jointUpdates;
    Data allRecalibration;
};

} // ns::robot

#endif //LSS_HUMANOID_CALIBRATION_H
