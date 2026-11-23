#pragma once

#include "local_abstraction.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

namespace local_safety {

// One yes/no answer for each heater level: off, medium, high.
using HeaterChoices = std::array<bool, dyn::nu>;

inline HeaterChoices safe_heaters(const HeaterTransitions& transitions) {
    HeaterChoices safe{};
    for (int level = 0; level < dyn::nu; ++level) {
        const auto& next = transitions[level];
        safe[level] = !next.leaves_safe_set && next.lower >= 0 &&
                      next.lower <= next.upper && next.upper < dyn::cpr;
    }
    return safe;
}

inline HeaterChoices safe_heaters(int left_bin, int middle_bin, int right_bin) {
    return safe_heaters(local_transitions(left_bin, middle_bin, right_bin));
}

struct SafetyTable {
    HeaterChoices allowed[dyn::cpr][dyn::cpr][dyn::cpr];
};

struct LocalSafetyResult {
    bool full_grid_winning;  // All 10^N cells for the specified ring, N >= 3.
    SafetyTable controller;  // All allowed heater levels for every neighbourhood.
};

inline LocalSafetyResult synthesize_local_safety(const LocalAbstraction& model) {
    SafetyTable table{};
    for (int left = 0; left < dyn::cpr; ++left) {
        for (int middle = 0; middle < dyn::cpr; ++middle) {
            for (int right = 0; right < dyn::cpr; ++right) {
                auto& choices = table.allowed[left][middle][right];
                choices = safe_heaters(model.transitions[left][middle][right]);
                // If every triple has a choice, all rooms can choose simultaneously
                // and stay inside the full safe grid. The first safety pass is final.
                if (std::find(choices.begin(), choices.end(), true) == choices.end())
                    throw std::runtime_error("The full safe grid could not be certified");
            }
        }
    }
    return {true, table};
}

inline SafetyTable build_safety_table() {
    return synthesize_local_safety(build_local_abstraction()).controller;
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
