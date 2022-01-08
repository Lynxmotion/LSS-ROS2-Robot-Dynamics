//
// Created by guru on 1/7/22.
//

#include "tween.h"

namespace robotik {

template<>
double Tween<KDL::Vector>::vector_length(const KDL::Vector &v) const {
    return std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
}

double tween_div(double a, double b) {
    return (b > 0)
        ? a / b
        : 0.0;
}

KDL::Vector tween_div(const KDL::Vector& a, const KDL::Vector& b) {
    auto x = tween_div(a.x(), b.x());
    auto y = tween_div(a.y(), b.y());
    auto z = tween_div(a.z(), b.z());
    return { x, y, z };
}

} //ns:robotik
