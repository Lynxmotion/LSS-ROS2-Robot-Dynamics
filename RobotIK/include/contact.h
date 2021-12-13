//
// Created by guru on 4/18/20.
//

#ifndef LSS_HUMANOID_CONTACT_H
#define LSS_HUMANOID_CONTACT_H

#include "types.h"

#include <string>


namespace robotik {

class Contact {
public:
    std::string name;
    unsigned long limb;
    double distance;             // distance to CoM
    unsigned int j_nr;
    KDL::Frame tf;               // frame of the foot joint, but already in SegmentState::tf
    std::vector<KDL::Vector> pointsInContact;
    KDL::Vector wrt_CoM;          // vector from CoM to contact point
    KDL::Vector wrt_odom;           // point of contact, relative to odom
    KDL::Vector wrt_base;           // point of contact, relative to robot base
    KDL::Vector grf;             // ground reaction force (1N or Kg * m/s^2)
    KDL::Vector jointOrigin;     // where the joint axis is relative to contact origin (is this needed?)

    ///@brief The amount of friction required before this contact would slip
    double staticFriction;       // newtons required to start motion

    ///@brief indicator of how likely this limb contact is slipping
    /// A value of 0 to 1 indicates likelihood of slipping with zero being very unlikely and 1 being at the expected
    /// point of slippage. A value greater than 1 indicates a very high possibility of slippage according to known forces.
    //double slipping;      // todo: currently 0 in ModelState msg

    inline Contact() : j_nr(0), staticFriction(0) {}
};

} // ns::robotik

#endif //LSS_HUMANOID_CONTACT_H
