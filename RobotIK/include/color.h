//
// Created by guru on 4/2/20.
//

#ifndef LSS_HUMANOID_COLOR_H
#define LSS_HUMANOID_COLOR_H

#include <string>

namespace robotik {

class Color {
public:
    using byte = unsigned char;
    using hex = unsigned long;
    
    inline Color(byte red, byte green, byte blue, byte _alpha=255) 
        : b(blue), g(green), r(red), alpha(_alpha) {
    }

    inline Color(float red, float green, float blue, float _alpha=1.0)
        : b(255*blue), g(255*green), r(255*red), alpha(255*_alpha) {
    }

    inline explicit Color(hex h) 
        : b((h & 0xff0000) >> 16), g((h & 0x00ff00) >> 8), r(h & 0xff), alpha((h & 0xff000000) >> 24) {
    }

    Color(std::string name_or_pound_hex);

    Color(const Color& copy) = default;

    Color(const Color& copy, float _alpha);

    Color withAlpha(float _alpha) const;

    static Color    Aqua;
    static Color	Black;
    static Color	Blue;
    static Color	Cream;
    static Color	Grey;
    static Color	Fuchsia;
    static Color	Green;
    static Color	LimeGreen;
    static Color	Yellow;
    static Color	Maroon;
    static Color	Navy;
    static Color	OliveGreen;
    static Color	Purple;
    static Color	Red;
    static Color	Silver;
    static Color	Teal;
    static Color	White;

    byte b, g, r;
    byte alpha;
};

} // ns::robotik

#endif //LSS_HUMANOID_COLOR_H
