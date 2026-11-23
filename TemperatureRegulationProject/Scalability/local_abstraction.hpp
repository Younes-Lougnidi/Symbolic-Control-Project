#pragma once

#include "../abstraction/abstraction.hpp"
#include <array>

namespace local_safety {

// Inclusive successor bins for the middle room. Both are -1 if the entire
// reachable interval is outside the grid. A partial exit also sets the flag.
struct LocalSuccessors {
    int lower = -1;
    int upper = -1;
    bool leaves_safe_set = false;
};

using HeaterTransitions = std::array<LocalSuccessors, dyn::nu>;

struct LocalAbstraction {
    HeaterTransitions transitions[dyn::cpr][dyn::cpr][dyn::cpr];
};

inline HeaterTransitions local_transitions(int left, int middle, int right) {
    const auto [low, high] = abstraction::cell_bounds({left, middle, right});
    HeaterTransitions transitions{};
    for (int level = 0; level < dyn::nu; ++level) {
        // Only the middle room's heater enters its update. Monotonicity gives
        // the exact extreme temperatures at the two corners of this box.
        const VEC_N heaters{0.0, dyn::u_lo + level * dyn::d_u, 0.0};
        const auto [next_low, next_high] = dyn::reachable_bounds(low, high, heaters);
        auto& successor = transitions[level];
        successor.leaves_safe_set = next_low[1] < dyn::T_lo || next_high[1] > dyn::T_hi;
        for (int bin = 0; bin < dyn::cpr; ++bin) {
            const double lo = dyn::T_lo + bin * dyn::d_T;
            const double hi = lo + dyn::d_T;
            // The explicit abstraction uses closed bins: touching counts.
            if (next_low[1] <= hi && next_high[1] >= lo) {
                if (successor.lower == -1) successor.lower = bin;
                successor.upper = bin;
            }
        }
    }
    return transitions;
}

inline LocalAbstraction build_local_abstraction() {
    LocalAbstraction model{};
    for (int left = 0; left < dyn::cpr; ++left)
        for (int middle = 0; middle < dyn::cpr; ++middle)
            for (int right = 0; right < dyn::cpr; ++right)
                model.transitions[left][middle][right] = local_transitions(left, middle, right);
    return model;
}

} // namespace local_safety
