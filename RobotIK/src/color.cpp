//
// Created by guru on 4/2/20.
//

#include "color.h"
#include <map>
#include <algorithm>

namespace robotik {

Color	Color::Aqua(0xffFFFF00);
Color	Color::Black(0xff000000);
Color	Color::Blue(0xffFF0000);
Color	Color::Cream(0xffF0FBFF);
Color	Color::Grey(0xff808080);
Color	Color::Fuchsia(0xffFF00FF);
Color	Color::Green(0xff00ff00);
Color	Color::LimeGreen(0xff00FF00);
Color	Color::Yellow(0xff00ffff);
Color	Color::Maroon(0xff000080);
Color	Color::Navy(0xff800000);
Color	Color::OliveGreen(0xff008080);
Color	Color::Purple(0xffFF00FF);
Color	Color::Red(0xff0000FF);
Color	Color::Silver(0xffC0C0C0);
Color	Color::Teal(0xff808000);
Color	Color::White(0xffFFFFFF);

static std::map<std::string, Color> standard_colors = {
        { "aqua", Color::Aqua },
        { "black", Color::Black },
        { "blue", Color::Blue },
        { "cream", Color::Cream },
        { "grey", Color::Grey },
        { "fuchsia", Color::Fuchsia },
        { "green", Color::Green },
        { "limegreen", Color::LimeGreen },
        { "yellow", Color::Yellow },
        { "maroon", Color::Maroon },
        { "navy", Color::Navy },
        { "olivegreen", Color::OliveGreen },
        { "purple", Color::Purple },
        { "red", Color::Red },
        { "silver", Color::Silver },
        { "teal", Color::Teal },
        { "white", Color::White }
};


Color::Color(std::string name_or_pound_hex) : Color(0x00ffffff) {
    if(!name_or_pound_hex.empty()) {
        if(name_or_pound_hex[0]=='#') {
            size_t ep;
            auto hex = stoul(name_or_pound_hex.substr(), &ep, 16);
            if(ep == 3) {
                // 3 nibble color
                alpha = 255;
                r = (hex & 0xf00) >> 8;
                g = (hex & 0x0f0) >> 4;
                b = (hex & 0x00f);
            } else {
                alpha = (hex & 0xff000000) >> 24;
                r     = (hex &   0xff0000) >> 16;
                g     = (hex &     0xff00) >> 8;
                b     = (hex &       0xff);
            }
        } else {
            std::transform(name_or_pound_hex.begin(), name_or_pound_hex.end(), name_or_pound_hex.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            auto color_itr = standard_colors.find(name_or_pound_hex);
            if(color_itr != standard_colors.end())
                *this = color_itr->second;
        }
    }
}

Color::Color(const Color& copy, float _alpha) : Color(copy) {
    alpha = 255 * _alpha;
}

Color Color::withAlpha(float _alpha) const {
    return Color(*this, _alpha);
}


} // ns::robotik
