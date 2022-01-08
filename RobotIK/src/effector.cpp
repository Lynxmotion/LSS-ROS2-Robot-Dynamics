//
// Created by guru on 1/7/22.
//

#include <effector.h>

namespace robotik {

BaseEffector::BaseEffector(
        const std::shared_ptr<KDL::Tree>& _tree,
        std::string _base_link)
    : tree(_tree), link(_base_link)
{
}

template<>
Effectors<BaseEffector::State>::iterator Effectors<BaseEffector::State>::find(const std::string& s) {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}
template<>
Effectors<BaseEffector::State>::const_iterator Effectors<BaseEffector::State>::find(const std::string& s) const {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}

} //ns:robotik