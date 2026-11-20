#pragma once

#include "../abstraction/abstraction.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

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

// Read each current bin once in room order. Send (room index, heater level)
// to the caller. Room 0 is emitted last because its left neighbour is the last room.
template<class ReadBin, class EmitCommand>
void apply_one_step(const SafetyTable& table, std::uint64_t rooms,
                    ReadBin read_bin, EmitCommand emit_command) {
    if (rooms < 3) throw std::invalid_argument("A ring needs at least three rooms");

    auto next_bin = [&]() {
        int bin = read_bin();
        if (bin < 0 || bin >= dyn::cpr) throw std::out_of_range("Invalid temperature bin");
        return bin;
    };
    auto choose = [&](int left, int middle, int right) {
        const auto& choices = table.allowed[left][middle][right];
        return static_cast<int>(std::find(choices.begin(), choices.end(), true) - choices.begin());
    };

    const int first = next_bin();
    const int second = next_bin();
    int left = first;
    int middle = second;
    for (std::uint64_t room = 1; room < rooms - 1; ++room) {
        const int right = next_bin();
        emit_command(room, choose(left, middle, right));
        left = middle;
        middle = right;
    }
    emit_command(rooms - 1, choose(left, middle, first));
    emit_command(0, choose(middle, first, second));
}

} // namespace local_safety
