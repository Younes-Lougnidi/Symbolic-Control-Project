#pragma once

#include "../abstraction/abstraction.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

namespace local_safety {

// One yes/no answer for each heater level: off, medium, high.
using HeaterChoices = std::array<bool, dyn::nu>;

inline HeaterChoices safe_heaters(int left_bin, int middle_bin, int right_bin) {
    const auto [low, high] = abstraction::cell_bounds({left_bin, middle_bin, right_bin});
    HeaterChoices safe{};
    for (int level = 0; level < dyn::nu; ++level) {
        // Only the middle room's heater enters its equation.
        const VEC_N heaters{0.0, dyn::u_lo + level * dyn::d_u, 0.0};
        const auto [next_low, next_high] = dyn::reachable_bounds(low, high, heaters);
        // For a fixed heater, warming any of the three rooms can only warm the
        // middle room's next temperature. These two corners give its exact extremes.
        safe[level] = next_low[1] >= dyn::T_lo && next_high[1] <= dyn::T_hi;
    }
    return safe;
}

struct SafetyTable {
    HeaterChoices allowed[dyn::cpr][dyn::cpr][dyn::cpr];
};

inline SafetyTable build_safety_table() {
    SafetyTable table{};
    for (int left = 0; left < dyn::cpr; ++left) {
        for (int middle = 0; middle < dyn::cpr; ++middle) {
            for (int right = 0; right < dyn::cpr; ++right) {
                auto& choices = table.allowed[left][middle][right];
                choices = safe_heaters(left, middle, right);
                // If every triple has a choice, all rooms can choose simultaneously
                // and stay inside the full safe grid. The first safety pass is final.
                if (std::find(choices.begin(), choices.end(), true) == choices.end())
                    throw std::runtime_error("The full safe grid could not be certified");
            }
        }
    }
    return table;
}

// Given a building cell, return the allowed heater levels for every room.
// A building command is safe exactly when choices[room][level] is true for every room.
inline std::vector<HeaterChoices> safe_commands_for_cell(
        const SafetyTable& table, const abstraction::Cell& bins) {
    const auto rooms = bins.size();
    if (rooms < 3) throw std::invalid_argument("A ring needs at least three rooms");
    for (int bin : bins) {
        if (bin < 0 || bin >= dyn::cpr) throw std::out_of_range("Invalid temperature bin");
    }

    std::vector<HeaterChoices> choices(rooms);
    for (std::size_t room = 0; room < rooms; ++room) {
        const auto left = (room == 0) ? rooms - 1 : room - 1;
        const auto right = (room + 1) % rooms;
        choices[room] = table.allowed[bins[left]][bins[room]][bins[right]];
    }
    return choices;
}

} // namespace local_safety
