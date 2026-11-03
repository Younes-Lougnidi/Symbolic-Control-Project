#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../robotControl/dynamics.hpp"
#include "../robotControl/state2coord.hpp"
#include "../robotControl/coord2state.hpp"

namespace reach {
// Same table type, grid, and 1-based IDs as sym_abs.cpp.
using Transitions = std::vector<std::array<int32_t, 2>>;
constexpr std::array<int, 4> p_x = {
    1, dyn::n_x[0], dyn::n_x[0]*dyn::n_x[1],
    dyn::n_x[0]*dyn::n_x[1]*dyn::n_x[2]
};
constexpr int n_states = p_x[3];
constexpr int n_commands = dyn::n_u[0]*dyn::n_u[1];
using Membership = std::vector<bool>;
struct Choice { int state; int command; };


// Tests whether every successor cell in the box belongs to A.
// box stores the state IDs of the lower and upper corner cells.

inline bool successors_in(const std::array<int32_t, 2>& box, const Membership& A) {
    if (A.size() != static_cast<std::size_t>(n_states + 1))
        throw std::invalid_argument("Wrong membership table size");
    if (box[0] == 0 && box[1] == 0) return false;
    for (int s : box)
        if (s < 1 || s > n_states) throw std::invalid_argument("Invalid successor ID");
    const auto lo = state2coord<3>(box[0], p_x);
    const auto hi = state2coord<3>(box[1], p_x);
    if (lo[0] > hi[0] || lo[1] > hi[1])
        throw std::invalid_argument("Reversed spatial bounds");
    int angle = lo[2];
    for (;;) {
        for (int y = lo[1]; y <= hi[1]; ++y)
            for (int x = lo[0]; x <= hi[0]; ++x)
                if (!A[coord2state<3>({x, y, angle}, p_x)]) return false;
        if (angle == hi[2]) break;
        angle = angle % dyn::n_x[2] + 1; // Heading wraps; x and y do not.
    }
    return true;
}

// Accepts std::array<int,N> or bcontaining state IDs.
template<class StateSet>
std::vector<Choice> pred(const Transitions& g, const StateSet& A) {
    if (g.size() != static_cast<std::size_t>(n_states)*n_commands)
        throw std::invalid_argument("g does not match dynamics.hpp");
    Membership member(n_states + 1, false);
    for (int s : A) {
        if (s < 1 || s > n_states) throw std::out_of_range("Invalid target state");
        member[s] = true;
    }
    std::vector<Choice> predecessors;
    for (int s = 1; s <= n_states; ++s)
        for (int u = 1; u <= n_commands; ++u)
            if (successors_in(g[(static_cast<std::size_t>(s)-1)*n_commands+u-1], member)) {
                // Need to analyse the hash function 
                predecessors.push_back({s, u});
                break; // One witnessing command per predecessor is sufficient.
            }
    return predecessors;
}

struct Result {
    std::vector<int> winning;
    std::vector<int> rank; // -1: uncertified, 0: target, k: at most k steps
    std::vector<int> h;    // Command ID; 0 means no command (target/uncertified).
};

template<class StateSet>
Result Max_reach(const Transitions& g, const StateSet& target) {
    Result result{{}, std::vector<int>(n_states+1, -1), std::vector<int>(n_states+1, 0)};
    for (int s : target) {
        if (s < 1 || s > n_states) throw std::out_of_range("Invalid target state");
        if (result.rank[s] == -1) {
            result.rank[s] = 0;
            result.winning.push_back(s);
        }
    }
    for (int k = 1; ; ++k) {
        // Compute Pre(R_k) completely before changing R_k.
        const auto predecessors = pred(g, result.winning);
        bool changed = false;
        for (auto [s, u] : predecessors) {
            if (result.rank[s] != -1) continue;
            result.winning.push_back(s);
            result.rank[s] = k;
            result.h[s] = u;
            changed = true;
        }
        if (!changed) return result;
    }
}
} // namespace reach
