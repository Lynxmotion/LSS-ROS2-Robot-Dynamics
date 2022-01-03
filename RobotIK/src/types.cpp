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

} // ns:robotik

