// Safety controller synthesis (section 2.1 of the reference article),
// built on top of the group's symbolic abstraction (robotControl/sym_abs.cpp).
//
//   1. S    : safe symbolic states (inside X, not touching R4)
//   2. Alg.1: Z_{k+1} = S ∩ Pre(Z_k)  until the fixed point Z_inf
//   3. h(xi) = { sigma : g(xi,sigma) ⊆ Z_inf }          (Theorem 2)
//   4. closed-loop simulation with the REAL dynamics     (Corollary 1)
//
// CONNECTION: build from the repository root with the existing abstraction:
// g++ -O2 -std=c++17 -Wall -Wextra -pedantic -DRUN_SAFETY robotControl/sym_abs.cpp Safety/safety_synthesis.cpp -o Safety/safety_synthesis.exe
// Run: ./Safety/safety_synthesis.exe (writes symb_abs.bin and trajectories.csv).
//
// Conventions (same as the report): state IDs and cell coordinates are 1-based,
// x1 varies fastest (p_x = {1,100,10000,300000}); command sigma = 1..15,
// u1 varies fastest (p_u = {1,3,15}). ID 0 = outside state xi_0.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <stdexcept>
#include <vector>
#include "safety_synthesis.hpp"
// CONNECTION: reuse the existing dynamics and coordinate helpers directly.
#include "../robotControl/dynamics.hpp"
#include "../robotControl/coord2state.hpp"
#include "../robotControl/state2coord.hpp"

using namespace std;
using namespace dyn; // CONNECTION: the original safety functions use these grid names.

constexpr double R4[4] = {3.0, 7.0, 3.0, 7.0};   // x1min, x1max, x2min, x2max (obstacle)

// ---------------------------------------------------------------------------
// Unpack g once: decode the two corner IDs with state2coord into index boxes
// ---------------------------------------------------------------------------
vector<Box> unpack(const SymbolicAbstraction& A) {
    vector<Box> B(A.g.size());
    for (size_t e = 0; e < A.g.size(); ++e) {
        if (A.g[e][0] == 0) { B[e].outside = true; continue; }
        B[e].outside = false;
        Idx3 cmin = state2coord<3>(A.g[e][0], A.p_x);
        Idx3 cmax = state2coord<3>(A.g[e][1], A.p_x);
        for (int d = 0; d < 3; ++d) { B[e].lo[d] = (int16_t)cmin[d]; B[e].hi[d] = (int16_t)cmax[d]; }
    }
    return B;
}

// ---------------------------------------------------------------------------
// 3D prefix sums of Z: number of cells of Z in any box in O(1)
// (same layout as the state IDs: x1 fastest)
// ---------------------------------------------------------------------------
struct Integral {
    vector<int> I;
    static size_t id(int a, int b, int c) {
        return (size_t)a + (size_t)(n_x[0] + 1) * ((size_t)b + (size_t)(n_x[1] + 1) * c);
    }
    void build(const vector<uint8_t>& Z) {                  // Z[xi-1]
        I.assign((size_t)(n_x[0] + 1) * (n_x[1] + 1) * (n_x[2] + 1), 0);
        for (int c = 1; c <= n_x[2]; ++c)
        for (int b = 1; b <= n_x[1]; ++b)
        for (int a = 1; a <= n_x[0]; ++a)
            I[id(a, b, c)] = Z[(size_t)(a - 1) + (size_t)n_x[0] * ((b - 1) + (size_t)n_x[1] * (c - 1))]
                + I[id(a - 1, b, c)] + I[id(a, b - 1, c)] + I[id(a, b, c - 1)]
                - I[id(a - 1, b - 1, c)] - I[id(a - 1, b, c - 1)] - I[id(a, b - 1, c - 1)]
                + I[id(a - 1, b - 1, c - 1)];
    }
    // number of Z cells in the 1-based inclusive box [lo,hi]
    int count(const Box& x) const {
        // FIX: a heading interval crossing +/-pi is two ordinary boxes.
        // Keep the original prefix-sum formula below for each part.
        if (x.lo[2] > x.hi[2]) {
            Box first = x, second = x;
            first.hi[2] = n_x[2];
            second.lo[2] = 1;
            return count(first) + count(second);
        }
        int a = x.lo[0] - 1, b = x.lo[1] - 1, c = x.lo[2] - 1;
        int A = x.hi[0],     B = x.hi[1],     C = x.hi[2];
        return I[id(A, B, C)] - I[id(a, B, C)] - I[id(A, b, C)] - I[id(A, B, c)]
             + I[id(a, b, C)] + I[id(a, B, c)] + I[id(A, b, c)] - I[id(a, b, c)];
    }
};

// g(xi,sigma) ⊆ Z ?   (false as soon as xi_0 is a possible successor)
inline bool all_succ_in_Z(const Box& x, const Integral& In) {
    if (x.outside) return false;
    // FIX: count cyclic heading bins so wrapped boxes cannot have negative volume.
    int angles = x.hi[2] - x.lo[2] + 1;
    if (x.lo[2] > x.hi[2]) angles += n_x[2];
    int vol = (x.hi[0] - x.lo[0] + 1) * (x.hi[1] - x.lo[1] + 1) * angles;
    return In.count(x) == vol;
}

// ---------------------------------------------------------------------------
// 1. Safe set S : cells of X that do not touch R4 (heading free)
// ---------------------------------------------------------------------------
vector<uint8_t> safe_set(const SymbolicAbstraction& A) {
    vector<uint8_t> S(A.n_xi, 1);
    const double eps = 1e-9;
    for (int xi = 1; xi <= A.n_xi; ++xi) {
        Idx3 I = state2coord<3>(xi, A.p_x);
        double x1 = bound_x_lo[0] + (I[0] - 1) * A.d_x[0];
        double x2 = bound_x_lo[1] + (I[1] - 1) * A.d_x[1];
        // FIX: in_R4 includes the boundary, so exclude cells touching it too.
        // The tolerance expands the unsafe set instead of admitting boundary cells.
        bool touch = (x1 + A.d_x[0] >= R4[0] - eps) && (x1 <= R4[1] + eps) &&
                     (x2 + A.d_x[1] >= R4[2] - eps) && (x2 <= R4[3] + eps);
        if (touch) S[xi - 1] = 0;
    }
    return S;
}

// ---------------------------------------------------------------------------
// 2. Algorithm 1 : Z0 = S ; Z_{k+1} = S ∩ Pre(Z_k) ; until the fixed point
// ---------------------------------------------------------------------------
vector<uint8_t> algorithm1(const SymbolicAbstraction& A, const vector<uint8_t>& S,
                           const vector<Box>& B, int& iters) {
    vector<uint8_t> Z = S;
    Integral In;
    iters = 0;
    while (true) {
        In.build(Z);
        vector<uint8_t> Znew(A.n_xi, 0);
        size_t cnt = 0;
        for (int xi = 0; xi < A.n_xi; ++xi) {
            if (!S[xi] || !Z[xi]) continue;
            for (int s = 0; s < A.n_sigma; ++s)                       // ∃ sigma
                if (all_succ_in_Z(B[(size_t)xi * A.n_sigma + s], In)) { Znew[xi] = 1; ++cnt; break; }
        }
        ++iters;
        printf("  iteration %3d : |Z| = %7zu\n", iters, cnt);
        if (Znew == Z) { --iters; return Z; }
        Z.swap(Znew);
    }
}

// ---------------------------------------------------------------------------
// 3. Controller h : bit (sigma-1) of h[xi-1] is set iff sigma ∈ h(xi)
// ---------------------------------------------------------------------------
vector<uint16_t> build_controller(const SymbolicAbstraction& A, const vector<uint8_t>& Zinf,
                                  const vector<Box>& B) {
    Integral In; In.build(Zinf);
    vector<uint16_t> h(A.n_xi, 0);
    for (int xi = 0; xi < A.n_xi; ++xi) {
        if (!Zinf[xi]) continue;
        for (int s = 0; s < A.n_sigma; ++s)
            if (all_succ_in_Z(B[(size_t)xi * A.n_sigma + s], In)) h[xi] |= (uint16_t)(1u << s);
    }
    return h;
}

vector<array<int32_t, 2>> filter_safe_transitions(
    const vector<array<int32_t, 2>>& g) {
    const array<int, 4> p_x = {
        1, n_x[0], n_x[0] * n_x[1], n_x[0] * n_x[1] * n_x[2]
    };
    const int n_states = p_x[3];
    const int n_commands = n_u[0] * n_u[1];
    if (g.size() != static_cast<size_t>(n_states) * n_commands)
        throw invalid_argument("g does not match dynamics.hpp");

    // Validate before unpacking, which assumes valid IDs and spatial bounds.
    for (const auto& box : g) {
        if (box[0] == 0 && box[1] == 0) continue;
        for (int corner : box)
            if (corner < 1 || corner > n_states)
                throw invalid_argument("Invalid successor ID");
        const auto lo = state2coord<3>(box[0], p_x);
        const auto hi = state2coord<3>(box[1], p_x);
        if (lo[0] > hi[0] || lo[1] > hi[1])
            throw invalid_argument("Reversed spatial bounds");
    }

    Vec3 d_x;
    for (int d = 0; d < 3; ++d)
        d_x[d] = (bound_x_hi[d] - bound_x_lo[d]) / n_x[d];
    // Safety synthesis only uses g and the grid; velocities are for simulation.
    const vector<Vec2> unused_velocities;
    const SymbolicAbstraction A{
        g, unused_velocities, d_x, p_x, n_states, n_commands
    };
    const auto boxes = unpack(A);
    const auto safe = safe_set(A);
    int iterations = 0;
    const auto winning = algorithm1(A, safe, boxes, iterations);
    const auto commands = build_controller(A, winning, boxes);

    auto g_safe = g;
    for (int cell = 1; cell <= n_states; ++cell) {
        for (int command = 1; command <= n_commands; ++command) {
            const bool allowed =
                (commands[cell - 1] & (1u << (command - 1))) != 0;
            if (!allowed)
                g_safe[static_cast<size_t>(cell - 1) * n_commands
                       + command - 1] = {0, 0};
        }
    }
    return g_safe;
}

// Independent check of Theorem 2 by brute force: enumerate EVERY successor cell
// of every allowed (xi,sigma) and verify that it lies in Z_inf (and inside X).
bool verify_controller(const SymbolicAbstraction& A, const vector<uint8_t>& Zinf,
                       const vector<uint16_t>& h, const vector<Box>& B) {
    for (int xi = 0; xi < A.n_xi; ++xi) {
        if (!Zinf[xi]) continue;
        if (h[xi] == 0) return false;                       // Z_inf ⊆ Pre(Z_inf)
        for (int s = 0; s < A.n_sigma; ++s) {
            if (!(h[xi] & (1u << s))) continue;
            const Box& b = B[(size_t)xi * A.n_sigma + s];
            if (b.outside) return false;
            // FIX: enumerate cyclic headings; the original loop skipped wrapped boxes.
            for (int k = b.lo[2]; ; k = k % n_x[2] + 1) {
                for (int j = b.lo[1]; j <= b.hi[1]; ++j)
                for (int i = b.lo[0]; i <= b.hi[0]; ++i) {
                    int id = coord2state<3>(Idx3{i, j, k}, A.p_x);
                    if (!Zinf[id - 1]) return false;
                }
                if (k == b.hi[2]) break; // FIX: stop after the last wrapped heading.
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// 4. Abstraction interface q and closed-loop simulation
// ---------------------------------------------------------------------------
int q(const SymbolicAbstraction& A, const Vec3& x) {          // 0 if x ∉ X
    Idx3 I;
    for (int d = 0; d < 3; ++d) {
        // FIX: reject non-finite inputs before converting to an integer cell index.
        if (!isfinite(x[d])) return 0;
        // FIX: equivalent headings use the same cell; spatial binning stays unchanged.
        double value = x[d];
        if (d == 2) {
            value = fmod(value, 2 * PI);
            if (value < -PI) value += 2 * PI;
            if (value >= PI) value -= 2 * PI;
        }
        I[d] = (int)floor((value - bound_x_lo[d]) / A.d_x[d]) + 1;
        if (I[d] < 1 || I[d] > n_x[d]) return 0;
    }
    return coord2state<3>(I, A.p_x);
}

bool in_R4(const Vec3& x) { return R4[0] <= x[0] && x[0] <= R4[1] && R4[2] <= x[1] && x[1] <= R4[3]; }

vector<Vec3> simulate(const SymbolicAbstraction& A, const vector<uint16_t>& h,
                      Vec3 x, int steps, mt19937& rng) {
    // FIX: check the initial state even when no movement is requested.
    int initial = q(A, x);
    if (initial == 0) throw runtime_error("left X");
    if (in_R4(x)) throw runtime_error("hit R4");
    if (!h[initial - 1]) throw runtime_error("x0 outside the safety domain");
    uniform_real_distribution<double> unif(0.0, 1.0);
    vector<Vec3> traj{x};
    for (int t = 0; t < steps; ++t) {
        int xi = q(A, x);
        if (xi == 0) throw runtime_error("left X");
        uint16_t mask = h[xi - 1];
        if (!mask) throw runtime_error("x0 outside the safety domain");
        vector<int> choices;
        for (int s = 0; s < A.n_sigma; ++s) if (mask & (1u << s)) choices.push_back(s);
        int s = choices[rng() % choices.size()];              // h is set-valued: any choice works
        Vec3 w;
        for (int i = 0; i < 3; ++i) w[i] = bound_w_lo[i] + unif(rng) * (bound_w_hi[i] - bound_w_lo[i]);
        x = f(x, A.val_u[s], w);                              // REAL dynamics
        traj.push_back(x);
        if (in_R4(x)) throw runtime_error("hit R4");
        // FIX: also check the last successor, which has no following loop iteration.
        int next = q(A, x);
        if (next == 0) throw runtime_error("left X");
        if (!h[next - 1]) throw runtime_error("state outside the safety domain");
    }
    return traj;
}

// ---------------------------------------------------------------------------
// CONNECTION: called by the existing abstraction after its computation finishes.
// The verification and simulation below remain in their original safety file.
int run_safety(const vector<array<int32_t, 2>>& g,
               const vector<Vec2>& val_u, const Vec3& d_x,
               const array<int, 4>& p_x, int n_xi, int n_sigma) {
    auto t0 = chrono::steady_clock::now();
    auto elapsed = [&] { return chrono::duration<double>(chrono::steady_clock::now() - t0).count(); };

    // CONNECTION: use the supplied data immediately instead of rebuilding/loading it.
    printf("1) Existing symbolic abstraction (T = %.2f)...\n", T);
    const SymbolicAbstraction A{g, val_u, d_x, p_x, n_xi, n_sigma};
    size_t n_out = 0;
    for (auto& e : A.g) n_out += (e[0] == 0);
    printf("   %d states, %d commands, %zu transitions (%zu may reach xi_0), %.1fs\n",
           A.n_xi, A.n_sigma, A.g.size(), n_out, elapsed());
    vector<Box> B = unpack(A);

    printf("2) Algorithm 1 (safety domain)...\n");
    vector<uint8_t> S = safe_set(A);
    size_t nS = 0; for (auto v : S) nS += v;
    printf("   |S| = %zu / %d\n", nS, A.n_xi);
    int iters;
    vector<uint8_t> Zinf = algorithm1(A, S, B, iters);
    size_t nZ = 0; for (auto v : Zinf) nZ += v;
    printf("   fixed point after %d iterations, |Z_inf| = %zu\n", iters, nZ);
    if (nZ == 0) { printf("   Z_inf is EMPTY: refine the discretization (Remark 1).\n"); return 1; }

    printf("3) Controller h(xi)...\n");
    vector<uint16_t> h = build_controller(A, Zinf, B);
    size_t tot = 0; for (auto m : h) tot += __builtin_popcount(m);
    printf("   average number of safe commands per state of Z_inf: %.2f\n", (double)tot / nZ);
    // FIX: stop on failed verification instead of continuing with an invalid policy.
    bool verified = verify_controller(A, Zinf, h, B);
    printf("   brute-force verification of Theorem 2: %s\n", verified ? "OK" : "FAILED");
    if (!verified) return 1;

    printf("4) Closed-loop simulation (real dynamics, random disturbances)...\n");
    mt19937 rng(0);
    Vec3 x0 = {0.5, 4.5, 0.0};
    int xi0 = q(A, x0);
    printf("   x0 = (0.5, 4.5, 0): q(x0) = %d, in Z_inf: %s\n", xi0, (xi0 && Zinf[xi0 - 1]) ? "yes" : "NO");
    ofstream out("trajectories.csv");
    // FIX: report an output-open failure instead of silently discarding trajectories.
    if (!out) { fprintf(stderr, "Cannot open trajectories.csv\n"); return 1; }
    out << "traj,t,x1,x2,x3\n";
    int ok = 0;
    for (int n = 0; n < 20; ++n) {
        try {
            auto tr = simulate(A, h, x0, 100, rng);
            for (size_t t = 0; t < tr.size(); ++t)
                out << n << "," << t << "," << tr[t][0] << "," << tr[t][1] << "," << tr[t][2] << "\n";
            ++ok;
        } catch (exception& e) { printf("   trajectory %d: %s\n", n, e.what()); }
    }
    printf("   %d/20 trajectories safe. Total time: %.1fs\n", ok, elapsed());
    // FIX: file-write or trajectory failures must produce a failing exit status.
    out.close();
    return ok == 20 && out ? 0 : 1;
}
