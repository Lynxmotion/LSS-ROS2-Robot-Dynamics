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
BaseStates::iterator BaseStates::find(const std::string& s) {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}
template<>
BaseStates::const_iterator BaseStates::find(const std::string& s) const {
    return std::find_if(begin(), end(), [&s](const BaseEffector::State& e) { return e.model->link == s; });
}

} //ns:robotik