//
// Created by guru on 6/5/20.
//

#include "friction.h"

namespace robotik {

FrictionTableEntry FrictionTable[] = {
        {  Aluminum, Wood,   0.5, 0.3},
        {  Steel, Wood,   0.5, 0.3},
        {  Wood, Wood,   0.5, 0.3},
        {  Rubber, Wood,   0.9, 0.7},
        {  Rubber, Concrete,   1., 0.7},
        {  Rubber, Wood,   0.8, 0.6},
        {  Aluminum, Ice,   0.4, 0.02},
        {  Rubber, Ice,   0.1, 0.05},
};


Friction Friction::fromTable(Material m1, Material m2) {
    FrictionTableEntry *begin = FrictionTable, *end = FrictionTable + sizeof(FrictionTable) / sizeof(FrictionTable[0]);
    while (begin < end) {
        if((begin->m1 == m1 && begin->m2 == m2) || (begin->m1 == m2 && begin->m2 == m1))
            return { begin->staticK, begin->kineticK };
    }
    return {};
}


} // ns:robotik
