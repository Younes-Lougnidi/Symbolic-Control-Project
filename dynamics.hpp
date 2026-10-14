// C++ port of dynamics.m
// Defines the system (unicycle), its bounds, and the abstraction grid sizes.
#pragma once
#include <array>
#include <cmath>

using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
using Mat3 = std::array<std::array<double, 3>, 3>;
using Idx3 = std::array<int, 3>;
using Idx2 = std::array<int, 2>;

constexpr double PI = 3.14159265358979323846;

namespace dyn {

// Sampling time
constexpr double T = 1.0; // this is the sampling period i.e the time between two consecutive actions, here 1s.

// System dynamics  x+ = f(x,u,w)
inline Vec3 f(const Vec3& x, const Vec2& u, const Vec3& w) {
    return {x[0] + T * u[0] * std::cos(x[2]) + T * w[0],
            x[1] + T * u[0] * std::sin(x[2]) + T * w[1],
            x[2] + T * u[1] + T * w[2]};
}

// Bounds on the Jacobian matrices (used to over-approximate reachable sets)
inline Mat3 Jf_x(const Vec2& u) {
    const double a = T * std::fabs(u[0]);
    return {{{1, 0, a},
             {0, 1, a},
             {0, 0, 1}}};
}

inline Mat3 Jf_w(const Vec2& /*u*/) { // because u willl not figure in the derived formula
    return {{{T, 0, 0},
             {0, T, 0},
             {0, 0, T}}};
}

// Constraints: first = lower bound, second = upper bound
constexpr Vec3 bound_x_lo = {0.0, 0.0, -PI};
constexpr Vec3 bound_x_hi = {10.0, 10.0, PI};
constexpr Vec2 bound_u_lo = {0.25, -1.0}; // the bounds on the actions i.e velocities (linear/rotation-wise)
constexpr Vec2 bound_u_hi = {1.0, 1.0};
constexpr Vec3 bound_w_lo = {-0.05, -0.05, -0.05};
constexpr Vec3 bound_w_hi = {0.05, 0.05, 0.05};

// Abstraction parameters: number of cells per dimension
constexpr Idx3 n_x = {100, 100, 30};
constexpr Idx2 n_u = {3, 5};

}  // namespace dyn