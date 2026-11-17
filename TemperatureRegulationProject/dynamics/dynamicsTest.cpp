// Tests for the N-room building project:
//   dynamics.hpp    -> VEC_N, Bounds, dyn::f, dyn::reachable_bounds, dyn::multiplier, constants
//   coord2state.hpp -> long long coord2state(std::vector<int> I, std::vector<long long> p)
//   state2coord.hpp -> std::vector<int> state2coord(long long id, std::vector<long long> p)
//
// Build:  g++ -std=c++17 -Wall -Wextra -o test_rooms test_rooms.cpp
// Run:    ./test_rooms          (exit code 0 = everything passed)
//
// If your header files have other names, change the three #include lines below.

// THIS WAS AI generated, I generated this with AI to test everything

#include "dynamics.hpp"
#include "coord2state.hpp"
#include "state2coord.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

static int g_checks = 0, g_failed = 0;

#define CHECK(cond, msg) do { ++g_checks; if (!(cond)) { ++g_failed; \
    std::printf("  FAIL: %s   [%s:%d]\n", msg, __FILE__, __LINE__); } } while (0)

static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
static bool nearVec(const VEC_N& a, const VEC_N& b, double tol = 1e-9) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) if (!near(a[i], b[i], tol)) return false;
    return true;
}
static void section(const char* name) { std::printf("[%s]\n", name); }

// ---------------------------------------------------------------- dynamics.hpp
static void test_constants() {
    section("constants");
    CHECK(near(dyn::T_lo, 17.0) && near(dyn::T_hi, 22.0), "state bounds are [17, 22]");
    CHECK(near(dyn::u_lo, 0.0) && near(dyn::u_hi, 0.6), "input bounds are [0, 0.6]");
    CHECK(dyn::cpr == 10, "10 cells per room");
    CHECK(dyn::nu == 3, "3 input values per room");
    CHECK(near(dyn::d_T, 0.5), "cell width is 0.5 (divide by cpr)");
    CHECK(near(dyn::d_u, 0.3), "input spacing is 0.3 (divide by nu-1)");
}

static void test_multiplier() {
    section("multiplier");
    auto p3 = dyn::multiplier(3, dyn::cpr);
    CHECK((p3 == std::vector<long long>{1, 10, 100, 1000}), "p_T for N=3 is {1,10,100,1000}");
    auto q3 = dyn::multiplier(3, dyn::nu);
    CHECK((q3 == std::vector<long long>{1, 3, 9, 27}), "p_u for N=3 is {1,3,9,27}");
    auto p4 = dyn::multiplier(4, dyn::cpr);
    CHECK(p4.size() == 5 && p4[4] == 10000, "p_T for N=4 has 5 entries, last = 10^4");
    for (int N = 3; N <= 10; ++N) {
        CHECK((int)dyn::multiplier(N, dyn::cpr).size() == N + 1, "size is N+1 (states)");
        CHECK((int)dyn::multiplier(N, dyn::nu).size() == N + 1, "size is N+1 (inputs)");
    }
    auto p12 = dyn::multiplier(12, dyn::cpr);
    CHECK(p12[12] == 1000000000000LL, "10^12 does not overflow (long long)");
    auto q20 = dyn::multiplier(20, dyn::nu);
    CHECK(q20[20] == 3486784401LL, "3^20 = 3486784401 (above 32-bit int)");
}

static void test_step() {
    section("f (one step of the dynamics)");
    // uniform temperatures: the neighbour term cancels
    CHECK(nearVec(dyn::f({20, 20, 20}, {0, 0, 0}), {19.055, 19.055, 19.055}), "T=20, u=0   -> 19.055");
    CHECK(nearVec(dyn::f({20, 20, 20}, {0.6, 0.6, 0.6}), {20.675, 20.675, 20.675}), "T=20, u=0.6 -> 20.675");
    CHECK(nearVec(dyn::f({20, 20, 20}, {0.3, 0.3, 0.3}), {19.865, 19.865, 19.865}), "T=20, u=0.3 -> 19.865");
    // each room uses ITS OWN heater value
    CHECK(nearVec(dyn::f({20, 20, 20}, {0, 0.3, 0.6}), {19.055, 19.865, 20.675}), "u is applied per room");
    // circular neighbours (hand-computed): tests the wrap-around of room 1 and room N
    CHECK(nearVec(dyn::f({17, 20, 22}, {0, 0, 0}), {19.79, 18.605, 17.815}, 1e-9), "wrap-around neighbours");
    // size is preserved for any N
    for (int N = 3; N <= 10; ++N) {
        VEC_N T(N, 19.5), u(N, 0.3);
        CHECK((int)dyn::f(T, u).size() == N, "output size equals N");
    }
    // symmetry: shifting all rooms by one position shifts the output the same way
    VEC_N T = {17.3, 18.1, 19.9, 21.4, 20.2}, u = {0.0, 0.3, 0.6, 0.3, 0.0};
    VEC_N Ts(5), us(5);
    for (int i = 0; i < 5; ++i) { Ts[(i + 1) % 5] = T[i]; us[(i + 1) % 5] = u[i]; }
    VEC_N a = dyn::f(T, u), b = dyn::f(Ts, us);
    bool sym = true;
    for (int i = 0; i < 5; ++i) if (!near(a[i], b[(i + 1) % 5])) sym = false;
    CHECK(sym, "circular symmetry (rotating the building rotates the result)");
}

static void test_monotone_and_reach() {
    section("monotonicity + reachable_bounds");
    // analytic derivatives are all >= 0 at the worst case
    double dii = 1 - 2 * dyn::alpha - dyn::beta - dyn::gamma * dyn::u_hi;
    CHECK(dii >= 0, "dTi+/dTi >= 0 at u = 0.6");
    CHECK(near(dii, 0.001, 1e-9), "dTi+/dTi at u=0.6 is 0.001 (tight)");
    CHECK(dyn::alpha > 0, "dTi+/dTneighbour > 0");
    CHECK(dyn::gamma * (dyn::Th - dyn::T_hi) > 0, "dTi+/dui > 0 at T = 22");

    // numerical monotonicity: raising any single room's temperature never lowers any output
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(17.0, 22.0);
    std::uniform_int_distribution<int> pick(0, 2);
    bool mono = true;
    for (int N = 3; N <= 6 && mono; ++N)
        for (int t = 0; t < 2000 && mono; ++t) {
            VEC_N T(N), u(N);
            for (int i = 0; i < N; ++i) { T[i] = U(rng); u[i] = 0.3 * pick(rng); }
            VEC_N base = dyn::f(T, u);
            for (int j = 0; j < N; ++j) {
                VEC_N T2 = T; T2[j] += 0.01;
                VEC_N out = dyn::f(T2, u);
                for (int i = 0; i < N; ++i) if (out[i] < base[i] - 1e-12) mono = false;
            }
            // and raising a heater never lowers a temperature
            for (int j = 0; j < N; ++j) {
                VEC_N u2 = u; u2[j] += 0.01;
                VEC_N out = dyn::f(T, u2);
                for (int i = 0; i < N; ++i) if (out[i] < base[i] - 1e-12) mono = false;
            }
        }
    CHECK(mono, "f is monotone in every T_j and every u_j (random test, N=3..6)");

    // the known reachable box
    Bounds r = dyn::reachable_bounds({17, 17, 17}, {22, 22, 22}, {0.3, 0.3, 0.3});
    CHECK(nearVec(r.first, {17.081, 17.081, 17.081}, 1e-9), "box [17,22]^3, u=0.3: lower = 17.081");
    CHECK(nearVec(r.second, {21.721, 21.721, 21.721}, 1e-9), "box [17,22]^3, u=0.3: upper = 21.721");

    // degenerate box (a single point) equals f
    VEC_N P = {18.4, 20.1, 21.7}, uP = {0.6, 0.0, 0.3};
    Bounds pt = dyn::reachable_bounds(P, P, uP);
    CHECK(nearVec(pt.first, dyn::f(P, uP)) && nearVec(pt.second, dyn::f(P, uP)), "point box -> f(point)");

    // soundness: every point of a box lands inside the reachable box
    int outside = 0;
    for (int N = 3; N <= 5; ++N)
        for (int t = 0; t < 5000; ++t) {
            VEC_N lo(N), hi(N), u(N);
            for (int i = 0; i < N; ++i) {
                double a = U(rng), b = U(rng);
                lo[i] = std::min(a, b); hi[i] = std::max(a, b); u[i] = 0.3 * pick(rng);
            }
            Bounds rb = dyn::reachable_bounds(lo, hi, u);
            VEC_N T(N);
            for (int i = 0; i < N; ++i) T[i] = lo[i] + (hi[i] - lo[i]) * (rng() % 1001) / 1000.0;
            VEC_N y = dyn::f(T, u);
            for (int i = 0; i < N; ++i) if (y[i] < rb.first[i] - 1e-12 || y[i] > rb.second[i] + 1e-12) ++outside;
        }
    CHECK(outside == 0, "reachable box contains every sampled successor (soundness)");
}

// ---------------------------------------------------- coord2state / state2coord
static void test_coord2state() {
    section("coord2state");
    auto p = dyn::multiplier(3, dyn::cpr);
    CHECK(coord2state({1, 1, 1}, p) == 1, "(1,1,1) -> 1");
    CHECK(coord2state({10, 10, 10}, p) == 1000, "(10,10,10) -> 1000 = p[N]");
    CHECK(coord2state({3, 8, 10}, p) == 973, "(3,8,10) -> 973");
    CHECK(coord2state({2, 1, 1}, p) == 2, "room 1 moves by 1");
    CHECK(coord2state({1, 2, 1}, p) == 11, "room 2 moves by 10");
    CHECK(coord2state({1, 1, 2}, p) == 101, "room 3 moves by 100");
    auto pu = dyn::multiplier(3, dyn::nu);
    CHECK(coord2state({1, 1, 1}, pu) == 1, "inputs: (1,1,1) -> 1 (all heaters off)");
    CHECK(coord2state({3, 3, 3}, pu) == 27, "inputs: (3,3,3) -> 27 (all heaters at 0.6)");
    CHECK(coord2state({2, 1, 1}, pu) == 2 && coord2state({1, 2, 1}, pu) == 4, "inputs: steps of 1 and 3");
}

static void test_state2coord() {
    section("state2coord");
    auto p = dyn::multiplier(3, dyn::cpr);
    CHECK((state2coord(973, p) == std::vector<int>{3, 8, 10}), "973 -> (3,8,10)");
    CHECK((state2coord(1, p) == std::vector<int>{1, 1, 1}), "1 -> (1,1,1)");
    CHECK((state2coord(1000, p) == std::vector<int>{10, 10, 10}), "1000 -> (10,10,10)");
    CHECK(state2coord(973, p).size() == 3, "returns N entries (not N+1)");
    auto pu = dyn::multiplier(3, dyn::nu);
    CHECK((state2coord(1, pu) == std::vector<int>{1, 1, 1}), "inputs: 1 -> (1,1,1)");
    CHECK((state2coord(27, pu) == std::vector<int>{3, 3, 3}), "inputs: 27 -> (3,3,3)");
}

static void test_roundtrip() {
    section("round trips (both directions, whole spaces)");
    for (int N = 3; N <= 6; ++N) {
        auto p = dyn::multiplier(N, dyn::cpr);
        bool ok = true, inrange = true;
        for (long long id = 1; id <= p[N]; ++id) {
            auto c = state2coord(id, p);
            if (coord2state(c, p) != id) ok = false;
            for (int v : c) if (v < 1 || v > dyn::cpr) inrange = false;
        }
        CHECK(ok, "id -> coords -> id for every state");
        CHECK(inrange, "every coordinate is in 1..10");
    }
    for (int N = 3; N <= 8; ++N) {
        auto p = dyn::multiplier(N, dyn::nu);
        bool ok = true, inrange = true;
        for (long long id = 1; id <= p[N]; ++id) {
            auto c = state2coord(id, p);
            if (coord2state(c, p) != id) ok = false;
            for (int v : c) if (v < 1 || v > dyn::nu) inrange = false;
        }
        CHECK(ok, "inputs: id -> coords -> id for every command");
        CHECK(inrange, "inputs: every coordinate is in 1..3");
    }
    // coords -> id -> coords over the whole N=3 grid (checks uniqueness of IDs too)
    auto p = dyn::multiplier(3, dyn::cpr);
    bool ok = true;
    std::vector<char> seen(1001, 0);
    for (int a = 1; a <= 10; ++a) for (int b = 1; b <= 10; ++b) for (int c = 1; c <= 10; ++c) {
        long long id = coord2state({a, b, c}, p);
        if (id < 1 || id > 1000 || seen[id]) ok = false; else seen[id] = 1;
        if (state2coord(id, p) != std::vector<int>{a, b, c}) ok = false;
    }
    CHECK(ok, "coords -> id -> coords, and all 1000 IDs are distinct");
    // a big N: IDs beyond 32 bits
    auto p12 = dyn::multiplier(12, dyn::cpr);
    std::vector<int> top(12, 10);
    CHECK(coord2state(top, p12) == 1000000000000LL, "N=12: last state ID is 10^12 (long long works)");
    CHECK(state2coord(1000000000000LL, p12) == top, "N=12: decoding 10^12 works");
}

// ---------------------------------------------------------- the pieces together
static void test_integration() {
    section("pieces together (temperature -> cell -> ID -> cell box)");
    auto p = dyn::multiplier(3, dyn::cpr);
    VEC_N T = {18.2, 20.7, 21.9};
    std::vector<int> idx(3);
    for (int i = 0; i < 3; ++i) idx[i] = (int)std::floor((T[i] - dyn::T_lo) / dyn::d_T + 1e-9) + 1;
    CHECK((idx == std::vector<int>{3, 8, 10}), "(18.2, 20.7, 21.9) is in cells (3,8,10)");
    CHECK(coord2state(idx, p) == 973, "... which is state 973");

    // for every cell and every command: the reachable box is ordered (lo <= hi)
    bool ordered = true;
    const auto pu = dyn::multiplier(3, dyn::nu);
    for (long long xi = 1; xi <= p[3]; ++xi) {
        auto c = state2coord(xi, p);
        VEC_N lo(3), hi(3);
        for (int i = 0; i < 3; ++i) { lo[i] = dyn::T_lo + (c[i] - 1) * dyn::d_T; hi[i] = lo[i] + dyn::d_T; }
        for (long long s = 1; s <= pu[3]; ++s) {
            auto cu = state2coord(s, pu);
            VEC_N u(3);
            for (int i = 0; i < 3; ++i) u[i] = dyn::u_lo + (cu[i] - 1) * dyn::d_u;
            Bounds r = dyn::reachable_bounds(lo, hi, u);
            for (int i = 0; i < 3; ++i) if (r.first[i] > r.second[i]) ordered = false;
        }
    }
    CHECK(ordered, "all 1000 cells x 27 commands give a reachable box with lo <= hi");

    // input table: command id -> heater powers
    VEC_N u27(3);
    auto cu = state2coord(27, pu);
    for (int i = 0; i < 3; ++i) u27[i] = dyn::u_lo + (cu[i] - 1) * dyn::d_u;
    CHECK(nearVec(u27, {0.6, 0.6, 0.6}), "command 27 = all heaters at 0.6");
    cu = state2coord(1, pu);
    for (int i = 0; i < 3; ++i) u27[i] = dyn::u_lo + (cu[i] - 1) * dyn::d_u;
    CHECK(nearVec(u27, {0.0, 0.0, 0.0}), "command 1 = all heaters off");
}

int main() {
    test_constants();
    test_multiplier();
    test_step();
    test_monotone_and_reach();
    test_coord2state();
    test_state2coord();
    test_roundtrip();
    test_integration();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
    std::printf(g_failed == 0 ? "ALL TESTS PASSED\n" : "SOME TESTS FAILED\n");
    return g_failed == 0 ? 0 : 1;
}