//
// Created by guru on 1/7/22.
//

#include <effector.h>

namespace robotik {

BaseEffector::BaseEffector(
        const std::shared_ptr<KDL::Tree>& _tree,
        std::string _base_link)
    : tree(_tree), base_link(_base_link)
{
}

} //ns:robotik