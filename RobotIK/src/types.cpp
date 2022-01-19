//
// Created by guru on 5/22/20.
//

#include "../include/types.h"

namespace robotik {

const std::string DEFAULT_COMPONENT_TOPIC_PREFIX = "robot_dynamics";


std::string generate_id(int length)
{
    std::string str;
    str.reserve(length+1);

    //hexadecimal characters
    char hex_characters[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

    int i;
    for(i=0; i<length; i++) {
        str[i] = hex_characters[rand()%16];
    }
    str[length]=0;
    return str;
}

CoordinateMask coordinate_mask_from_string(const std::string& s) {
    if(s.empty())
        return CoordinateMask::All;
    CoordinateMask m;
    unsigned part = 0;
    for(auto& c : s) {
        if(part == 0) {
            // reading position
            switch(c) {
                case 'X': m |= CoordinateMask::X; break;
                case 'Y': m |= CoordinateMask::Y; break;
                case 'Z': m |= CoordinateMask::Z; break;
                case '*': m |= CoordinateMask::XYZ; break;
            }
        } else if(part == 1) {
            // reading rotation
            switch(c) {
                case 'R': m |= CoordinateMask::Roll; break;
                case 'P': m |= CoordinateMask::Pitch; break;
                case 'Y': m |= CoordinateMask::Yaw; break;
                case '*': m |= CoordinateMask::RPY; break;
            }
        }
    }
    return m;
}

} // ns:robotik

