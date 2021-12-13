//
// Created by guru on 6/5/20.
//

#ifndef LSS_HUMANOID_FRICTION_H
#define LSS_HUMANOID_FRICTION_H

#include "types.h"
#include "exception.h"

namespace robotik {

enum Material {
    Aluminum,
    Steel,
    Rubber,
    Wood,
    Tile,
    Concrete,
    Ice
};

typedef struct {
    Material m1;
    Material m2;
    double staticK;
    double kineticK;
} FrictionTableEntry;


class Friction {
public:
    double staticK;
    double kineticK;

    inline Friction()
        // use some typical surface values
        : staticK(0.5), kineticK(0.3)
    {}

    inline Friction(double _staticK, double _kineticK)
    // use some typical surface values
            : staticK(_staticK), kineticK(_kineticK)
    {}

    ///@brief Use the internal lookup table to return the coefficient values between two materials
    static Friction fromTable(Material m1, Material m2);

    inline double staticForce(double normalForce) const { return normalForce * staticK; }
    inline double kineticForce(double normalForce) const { return normalForce * kineticK; }

    inline double force(double normalForce, double bodyVelocity) const {
        return (bodyVelocity > 0.0001)
            ? kineticForce(normalForce)
            : staticForce(normalForce);
    }
};


} // ns:robotik

#endif //LSS_HUMANOID_FRICTION_H
