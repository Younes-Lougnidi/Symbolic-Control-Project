// Safety controller synthesis (section 2.1 of the reference article),
// built on top of the group's symbolic abstraction (symb_abs.hpp).
//
//   1. S    : safe symbolic states (inside X, not touching R4)
//   2. Alg.1: Z_{k+1} = S ∩ Pre(Z_k)  until the fixed point Z_inf
//   3. h(xi) = { sigma : g(xi,sigma) ⊆ Z_inf }          (Theorem 2)
//   4. closed-loop simulation with the REAL dynamics     (Corollary 1)
//
// Build : g++ -O2 -std=c++17 -Wall safety_synthesis.cpp -o safety_synthesis
// Run   : ./safety_synthesis        (writes trajectories.csv)
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
#include "symb_abs.hpp"

using namespace std;

constexpr double R4[4] = {3.0, 7.0, 3.0, 7.0};   // x1min, x1max, x2min, x2max (obstacle)

// ---------------------------------------------------------------------------
// Unpack g once: decode the two corner IDs with state2coord into index boxes
// ---------------------------------------------------------------------------
struct Box {
    int16_t lo[3], hi[3];     // 1-based inclusive cell coordinates of the successor box
    bool outside;             // true if xi_0 is a possible successor
};

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
        int a = x.lo[0] - 1, b = x.lo[1] - 1, c = x.lo[2] - 1;
        int A = x.hi[0],     B = x.hi[1],     C = x.hi[2];
        return I[id(A, B, C)] - I[id(a, B, C)] - I[id(A, b, C)] - I[id(A, B, c)]
             + I[id(a, b, C)] + I[id(a, B, c)] + I[id(A, b, c)] - I[id(a, b, c)];
    }
};

// g(xi,sigma) ⊆ Z ?   (false as soon as xi_0 is a possible successor)
inline bool all_succ_in_Z(const Box& x, const Integral& In) {
    if (x.outside) return false;
    int vol = (x.hi[0] - x.lo[0] + 1) * (x.hi[1] - x.lo[1] + 1) * (x.hi[2] - x.lo[2] + 1);
    return In.count(x) == vol;
}

// ---------------------------------------------------------------------------
// 1. Safe set S : cells of X that do not overlap the interior of R4 (heading free)
// ---------------------------------------------------------------------------
vector<uint8_t> safe_set(const SymbolicAbstraction& A) {
    vector<uint8_t> S(A.n_xi, 1);
    const double eps = 1e-9;
    for (int xi = 1; xi <= A.n_xi; ++xi) {
        Idx3 I = state2coord<3>(xi, A.p_x);
        double x1 = bound_x_lo[0] + (I[0] - 1) * A.d_x[0];
        double x2 = bound_x_lo[1] + (I[1] - 1) * A.d_x[1];
        bool touch = (x1 + A.d_x[0] > R4[0] + eps) && (x1 < R4[1] - eps) &&
                     (x2 + A.d_x[1] > R4[2] + eps) && (x2 < R4[3] - eps);
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
            for (int k = b.lo[2]; k <= b.hi[2]; ++k)
            for (int j = b.lo[1]; j <= b.hi[1]; ++j)
            for (int i = b.lo[0]; i <= b.hi[0]; ++i) {
                int id = coord2state<3>(Idx3{i, j, k}, A.p_x);
                if (!Zinf[id - 1]) return false;
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
        I[d] = (int)floor((x[d] - bound_x_lo[d]) / A.d_x[d]) + 1;
        if (I[d] < 1 || I[d] > n_x[d]) return 0;
    }
    return coord2state<3>(I, A.p_x);
}

bool in_R4(const Vec3& x) { return R4[0] <= x[0] && x[0] <= R4[1] && R4[2] <= x[1] && x[1] <= R4[3]; }

vector<Vec3> simulate(const SymbolicAbstraction& A, const vector<uint16_t>& h,
                      Vec3 x, int steps, mt19937& rng) {
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
    }
    return traj;
}

// ---------------------------------------------------------------------------
int main() {
    auto t0 = chrono::steady_clock::now();
    auto elapsed = [&] { return chrono::duration<double>(chrono::steady_clock::now() - t0).count(); };

    printf("1) Symbolic abstraction (T = %.2f)...\n", T);
    SymbolicAbstraction A = build_symbolic_abstraction();
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
    printf("   brute-force verification of Theorem 2: %s\n",
           verify_controller(A, Zinf, h, B) ? "OK" : "FAILED");

    printf("4) Closed-loop simulation (real dynamics, random disturbances)...\n");
    mt19937 rng(0);
    Vec3 x0 = {0.5, 4.5, 0.0};
    int xi0 = q(A, x0);
    printf("   x0 = (0.5, 4.5, 0): q(x0) = %d, in Z_inf: %s\n", xi0, (xi0 && Zinf[xi0 - 1]) ? "yes" : "NO");
    ofstream out("trajectories.csv");
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
    return 0;
}
