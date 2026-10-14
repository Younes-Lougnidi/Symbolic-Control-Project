// C++ port of symb_abs.m -- computation of a symbolic model (abstraction)
// Original author: Antoine GIRARD (Universite Paris-Saclay, CNRS, CentraleSupelec, L2S), 2024
//
// Build:  g++ -O2 -std=c++17 -o symb_abs symb_abs.cpp
// Run:    ./symb_abs        (writes symb_abs.bin)

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "dynamics.hpp"
#include "coord2state.hpp"
#include "state2coord.hpp"

static Vec3 matvec(const Mat3& M, const Vec3& v) {
    Vec3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i] += M[i][j] * v[j];
    return r;
}

int main() {
    using namespace dyn;
    auto t0 = std::chrono::steady_clock::now();

    // ---- State discretization ------------------------------------------------
    Vec3 d_x;
    for (int i = 0; i < 3; ++i) d_x[i] = (bound_x_hi[i] - bound_x_lo[i]) / n_x[i];
    const std::array<int, 4> p_x = {1, n_x[0], n_x[0] * n_x[1], n_x[0] * n_x[1] * n_x[2]};
    const int n_xi = p_x[3];

    // ---- Input discretization ------------------------------------------------
    Vec2 d_u;
    for (int i = 0; i < 2; ++i) d_u[i] = (bound_u_hi[i] - bound_u_lo[i]) / (n_u[i] - 1);
    const std::array<int, 3> p_u = {1, n_u[0], n_u[0] * n_u[1]};
    const int n_sigma = p_u[2];

    // concretization function p(sigma) -> val_u[sigma-1]
    std::vector<Vec2> val_u(n_sigma);
    for (int i = 1; i <= n_sigma; ++i) {
        Idx2 c = state2coord<2>(i, p_u);
        for (int k = 0; k < 2; ++k) val_u[i - 1][k] = bound_u_lo[k] + (c[k] - 1) * d_u[k];
    }

    // ---- Disturbance -----------------------------------------------------------
    Vec3 d_w, w_c;
    for (int i = 0; i < 3; ++i) {
        d_w[i] = bound_w_hi[i] - bound_w_lo[i];
        w_c[i] = 0.5 * (bound_w_lo[i] + bound_w_hi[i]);
    }

    std::printf("Computation of the symbolic model\n");
    std::printf("%d symbolic states, %d symbolic commands\n", n_xi, n_sigma);

    // g[(xi-1)*n_sigma + (sigma-1)] = {min_succ, max_succ}; {0,0} = leaves state constraints
    std::vector<std::array<int32_t, 2>> g((size_t)n_xi * n_sigma, {0, 0});
    Vec3 robust_margins = d_x;

    int last_pct = -1;
    for (int xi = 1; xi <= n_xi; ++xi) {
        int pct = (100 * xi) / n_xi;
        if (pct != last_pct && pct % 10 == 0) {
            std::printf("Abstraction in progress... %d%%\n", pct);
            last_pct = pct;
        }

        Idx3 c_xi = state2coord<3>(xi, p_x);
        Vec3 x_c;  // center of the cell
        for (int i = 0; i < 3; ++i) x_c[i] = bound_x_lo[i] + (c_xi[i] - 0.5) * d_x[i];

        for (int sigma = 1; sigma <= n_sigma; ++sigma) {
            const Vec2& u = val_u[sigma - 1];

            // Over-approximation of the reachable set
            Vec3 x_c_succ = f(x_c, u, w_c);
            Vec3 a = matvec(Jf_x(u), d_x);
            Vec3 b = matvec(Jf_w(u), d_w);
            Vec3 reach_lo, reach_hi;
            for (int i = 0; i < 3; ++i) {
                double dev = 0.5 * a[i] + 0.5 * b[i];
                reach_lo[i] = x_c_succ[i] - dev;
                reach_hi[i] = x_c_succ[i] + dev;
            }

            // Special treatment of the third coordinate (angle)
            if (reach_lo[2] < -PI && reach_hi[2] >= -PI) {
                reach_lo[2] += 2 * PI;
            } else if (reach_lo[2] < -PI && reach_hi[2] < -PI) {
                reach_lo[2] += 2 * PI;
                reach_hi[2] += 2 * PI;
            } else if (reach_hi[2] > PI && reach_lo[2] <= PI) {
                reach_hi[2] -= 2 * PI;
            } else if (reach_hi[2] > PI && reach_lo[2] > PI) {
                reach_hi[2] -= 2 * PI;
                reach_lo[2] -= 2 * PI;
            }

            // Store the transition relation
            bool inside = true;
            for (int i = 0; i < 3; ++i)
                if (!(reach_lo[i] >= bound_x_lo[i] && reach_hi[i] <= bound_x_hi[i])) inside = false;

            if (inside) {
                Idx3 cmin, cmax;
                for (int i = 0; i < 3; ++i) {
                    cmin[i] = (int)std::floor((reach_lo[i] - bound_x_lo[i]) / d_x[i]) + 1;
                    cmax[i] = (int)std::ceil((reach_hi[i] - bound_x_lo[i]) / d_x[i]);
                }
                g[(size_t)(xi - 1) * n_sigma + (sigma - 1)] = {coord2state<3>(cmin, p_x),
                                                               coord2state<3>(cmax, p_x)};
                // this is meant to compute the tolerance of any minor fluctuations due to exterior unprecedent not taken into account conditions. 
                // it computes the robust margins, in the worst case so the maximum possible reachable set.
                for (int i = 0; i < 3; ++i) {
                    double lo = reach_lo[i] - bound_x_lo[i];
                    double hi = bound_x_lo[i] - reach_hi[i];
                    robust_margins[i] = std::fmin(robust_margins[i], lo - d_x[i] * std::floor(lo / d_x[i]));
                    robust_margins[i] = std::fmin(robust_margins[i],
                        hi + d_x[i] * std::ceil((reach_hi[i] - bound_x_lo[i]) / d_x[i]));
                }
            }
        }
    }

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("Elapsed time is %.2f seconds.\n", elapsed);

    // Binary layout: int32 n_xi, int32 n_sigma, double[3] robust_margins,
    //                double[n_sigma][2] val_u, int32[n_xi][n_sigma][2] g
    std::ofstream out("symb_abs.bin", std::ios::binary);
    int32_t hdr[2] = {n_xi, n_sigma};
    out.write(reinterpret_cast<const char*>(hdr), sizeof(hdr));
    out.write(reinterpret_cast<const char*>(robust_margins.data()), sizeof(double) * 3);
    for (const auto& u : val_u) out.write(reinterpret_cast<const char*>(u.data()), sizeof(double) * 2);
    out.write(reinterpret_cast<const char*>(g.data()), sizeof(int32_t) * 2 * g.size());
    std::printf("Saved symb_abs.bin\n");
    return 0;
}