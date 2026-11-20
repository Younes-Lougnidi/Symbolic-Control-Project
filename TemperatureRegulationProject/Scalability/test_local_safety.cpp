#include "local_safety.hpp"
#include "../Safety_Synthesis/safety_synthesis.hpp"
#include <iostream>

int main() {
    const auto table = local_safety::build_safety_table();
    if (table.allowed[0][0][0] != local_safety::HeaterChoices{false, true, true})
        throw std::runtime_error("Unexpected choices for three cold rooms");

    for (int rooms : {3, 4}) {
        const auto model = abstraction::build_abstraction(rooms);
        const auto reference = synthesize_safety(model);
        for (abstraction::Id cell = 0; cell < model.num_states; ++cell) {
            const auto bins = abstraction::decode_cell(cell, rooms);
            int next = 0;
            std::vector<int> selected(rooms, -1);
            local_safety::apply_one_step(table, rooms, [&]() { return bins.at(next++); },
                [&](std::uint64_t room, int level) { selected.at(room) = level; });

            for (int i = 0; i < rooms; ++i) {
                const auto& choices = table.allowed[bins[(i + rooms - 1) % rooms]]
                                                   [bins[i]][bins[(i + 1) % rooms]];
                const auto first = std::find(choices.begin(), choices.end(), true);
                if (selected[i] != first - choices.begin())
                    throw std::runtime_error("Streamed lookup chose the wrong heater");
            }

            for (abstraction::Id command = 0; command < model.num_commands; ++command) {
                // Room 0 is the least significant digit of the global command.
                auto digits = command;
                bool allowed = true;
                for (int i = 0; i < rooms; ++i) {
                    int left = bins[(i + rooms - 1) % rooms];
                    int right = bins[(i + 1) % rooms];
                    allowed &= table.allowed[left][bins[i]][right][digits % dyn::nu];
                    digits /= dyn::nu;
                }
                const auto& commands = reference[cell];
                if (allowed != std::binary_search(commands.begin(), commands.end(), command))
                    throw std::runtime_error("Safe command differs from the reference solver");
            }
        }
        std::cout << rooms << " rooms: all " << model.num_states * model.num_commands
                  << " cell-command pairs agree\n";
    }
}
