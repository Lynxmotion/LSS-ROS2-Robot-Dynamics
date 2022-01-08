//
// Created by guru on 1/7/22.
//

#ifndef ROBOT_DYNAMICS_TWEEN_H
#define ROBOT_DYNAMICS_TWEEN_H

#include <kdl/frames.hpp>

namespace robotik {

// tween division avoids divide-by-zero by returning 0.0 instead
double tween_div(double a, double b);
KDL::Vector tween_div(const KDL::Vector& a, const KDL::Vector& b);


template<class T>
class Tween {
public:
    inline Tween() {}

    Tween(const T& start, const T& end, double duration)
    : start_(start), end_(end), delta_(end - start), duration_(duration)
    {
    }

    [[nodiscard]] inline T delta() const { return delta_; }

    void duration(double d) { duration_ = d; }

    [[nodiscard]] inline double duration() const { return duration_; }

    ///@brief Return the tweened value at the given time
    [[nodiscard]] T operator[](double t) {
        if(t > duration_ || duration_ == 0.0)
            return end_;
        else if(t  < 0)
            return start_;
        else
            return start_ + delta_ * (t / duration_);
    }

    static Tween<T> Duration(const T& start, const T& end, double duration) {
        return Tween<T>(start, end, duration);
    }

    static Tween<T> Velocity(const T& start, const T& end, const T& velocity) {
        Tween<T> t(start, end, 0);
        auto dur_native = tween_div(t.delta(), velocity);
        auto duration = t.vector_length(dur_native);
        t.duration(duration);
        return t;
    }

protected:
    T start_, end_, delta_;
    double duration_;

    // return the length of a tween vector
    [[nodiscard]] double vector_length(const T& v) const { return v; }
};


template<>
double Tween<KDL::Vector>::vector_length(const KDL::Vector &v) const;

} //ns:robotik

#endif //ROBOT_DYNAMICS_TWEEN_H
