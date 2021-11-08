
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    float t = time, t2 = time * time, t3 = time * time * time;
    float h00 = 2 * t3 - 3 * t2 + 1;
    float h10 = t3 - 2 * t2 + t;
    float h01 = -2 * t3 + 3 * t2;
    float h11 = t3 - t2;
    T ans = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
    return ans;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...

    if(control_points.size() == 0) return T();
    if(control_points.size() == 1) return control_points.begin()->second;
    if(time < control_points.begin()->first) 
        return control_points.begin()->second;
    if(time > std::prev(control_points.end())->first) 
        return std::prev(control_points.end())->second;

    auto k2 = control_points.upper_bound(time);
    auto k1 = std::prev(k2);
    float t = (time - k1->first) / (k2->first - k1->first);
    T m1, m2;
    if(k1 == control_points.begin()) {
        auto p0 = k1->second - (k2->second - k1->second);
        float t0 = k1->first - (k2->first - k1->first);
        m1 = (k2->second - p0) / (k2->first - t0);
    } else {
        auto k0 = prev(k1);
        m1 = (k2->second - k0->second) / (k2->first - k0->first);
    }

    if(k2 == std::prev(control_points.end())) {
        auto p3 = k2->second + (k2->second - k1->second);
        auto t3 = k2->first + (k2->first - k1->first);
        m2 = (p3 - k1->second) / (t3 - k1->first);
    } else {
        auto k3 = std::next(k2);
        m2 = (k3->second - k1->second) / (k3->first - k1->first);
    }
    
    return cubic_unit_spline(t, k1->second, k2->second, m1, m2);
}
