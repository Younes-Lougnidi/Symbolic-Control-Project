#pragma once

#include "local_safety.hpp"
#include <cstdint>

namespace local_safety {

inline constexpr std::uint64_t rooms_per_block = 65536;

// Each room gets one row of safe heaters. For a huge ring, we return those
// rows a batch at a time. next_bin() supplies the next room's current bin.
// use_block(first_room, choices) consumes a batch before we reuse its memory.
template<class NextBin, class UseBlock>
void safe_commands_in_blocks(const SafetyTable& table, std::uint64_t rooms,
                             NextBin&& next_bin, UseBlock&& use_block) {
    if (rooms < 3) throw std::invalid_argument("A ring needs at least three rooms");
    auto read_bin = [&]() {
        const int bin = next_bin();
        if (bin < 0 || bin >= dyn::cpr) throw std::out_of_range("Invalid temperature bin");
        return bin;
    };

    const int first = read_bin();
    const int second = read_bin();
    int left = first;
    int middle = second;
    std::vector<HeaterChoices> choices;
    choices.reserve(std::min(rooms - 1, rooms_per_block));

    for (std::uint64_t begin = 1; begin < rooms; ) {
        const auto end = begin + std::min(rooms - begin, rooms_per_block);
        choices.clear();
        for (auto room = begin; room < end; ++room) {
            const int right = (room == rooms - 1) ? first : read_bin();
            // A-B-C-D example: (A,B,C) gives B's safe heaters.
            choices.push_back(table.allowed[left][middle][right]);
            // Slide over. Once D arrives, (B,C,D) gives C's safe heaters.
            left = middle;
            middle = right;
        }
        use_block(begin, choices);
        begin = end;
    }

    // A's turn comes last: only now do we know D, its left neighbour.
    choices.assign(1, table.allowed[left][first][second]);
    use_block(0, choices);
}

} // namespace local_safety
