//
// Created by guru on 5/22/20.
//

#include "../include/types.h"

namespace robotik {

const std::string DEFAULT_COMPONENT_TOPIC_PREFIX = "robot_dynamics";

std::size_t ElementKey::hash() const {
    std::size_t h1 = std::hash<uint16_t>{}(symbol);
    std::size_t h2 = std::hash<std::string>{}(name);
    return h1 ^ (h2 << 1U); // or use boost::hash_combine (see Discussion)
}

std::string ElementKey::toString() const {
    std::string s;
    uint16_t sym = symbol;
    char a = sym & 0x7f;
    char b = (sym >> 8U) & 0x7f;
    if(isprint(a))
        s += a;
    if(isprint(b))
        s += b;
    s += ':';
    s += name;
    return s;
}

} // ns:robotik

