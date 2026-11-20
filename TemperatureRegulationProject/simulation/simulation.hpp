#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <string>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "../dynamics/dynamics.hpp"
#include "../abstraction/abstraction.hpp"

namespace simulation {

// Matching Cell 11 of Building_Safety_Team_Guide.ipynb:
// Controller type returned by safety synthesis: a list of safe command IDs for each cell ID.
/* 
Basically Controller is a list of cells where each cell represent its safe commands 
For example Controller controller = {
    {0, 3},       // cell 0
    {1},          // cell 1
    {},           // cell 2
    {2, 4}        // cell 3
};

THen: 
Cell 0 → safe commands: 0, 3
Cell 1 → safe command: 1
Cell 2 → no safe command
Cell 3 → safe commands: 2, 4
*/
using Controller = std::vector<std::vector<abstraction::Id>>;

struct SimulationResult {
    std::vector<VEC_N> temperatures;       // Initial state, then each new state
    std::vector<abstraction::Id> commands; // One ID per completed update
    bool safe = true;                     // All recorded temperatures are in [17,22]
    bool uncertified = false;              // A visited cell had an empty command list
};

// Check whether all room temperatures are finite and belong to [17.0, 22.0]
inline bool is_safe(const VEC_N& T) {
    for (double val : T) {
        if (!std::isfinite(val) || val < dyn::T_lo || val > dyn::T_hi) {
            return false;
        }
    }
    return true;
}

// Closed-loop simulation for N rooms over a fixed number of steps
inline SimulationResult simulate(const Controller& controller,
                                 const VEC_N& initial,
                                 int steps) {
    SimulationResult result;
    result.temperatures.push_back(initial);

    // 1. Record the initial temperatures. Check that every temperature is finite
    // and in [17,22] before each lookup, including the initial state.
    if (!is_safe(initial)) {
        result.safe = false;
        return result;
    }

    if (steps <= 0) {
        return result;
    }

    const int N = static_cast<int>(initial.size());
    VEC_N current = initial;

    for (int k = 0; k < steps; ++k) {
        // Check current temperatures before lookup
        if (!is_safe(current)) {
            result.safe = false;
            break;
        }

        // 2. Find the cell with abstraction::locate_cell(T).
        abstraction::Id cell = -1;
        try {
            cell = abstraction::locate_cell(current);
        } catch (const std::exception&) {
            result.safe = false;
            break;
        }

        // Read controller[cell]. If it is empty, mark the run uncertified and stop.
        if (cell < 0 || static_cast<std::size_t>(cell) >= controller.size() ||
            controller[cell].empty()) {
            result.uncertified = true;
            break;
        }

        // Otherwise choose the first ID so anyone can repeat your run.
        abstraction::Id cmd_id = controller[cell][0];        
        result.commands.push_back(cmd_id);

        // 3. Convert the ID to powers with abstraction::decode_command(id, N),
        // then call dyn::f(T, powers).
        VEC_N u = abstraction::decode_command(cmd_id, N);
        VEC_N next_T = dyn::f(current, u);

        // Record the new temperatures and check them
        result.temperatures.push_back(next_T);

        if (!is_safe(next_T)) {
            result.safe = false;
            break;
        }

        current = next_T;
    }

    return result;
}

// Export trajectory to CSV (one row per observation, comments for metadata)
inline void export_csv(const SimulationResult& res,
                       int rooms,
                       const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }

    // Metadata header
    file << "# status: safe=" << (res.safe ? "true" : "false")
         << ", uncertified=" << (res.uncertified ? "true" : "false")
         << ", steps=" << res.commands.size() << "\n";

    // Header line
    file << "t";
    for (int i = 0; i < rooms; ++i) {
        file << ",T" << i;
    }
    file << ",cmd_id";
    for (int i = 0; i < rooms; ++i) {
        file << ",u" << i;
    }
    file << "\n";

    file << std::fixed << std::setprecision(5);

    // K+1 temperatures, K commands
    for (std::size_t k = 0; k < res.temperatures.size(); ++k) {
        file << k;
        for (int i = 0; i < rooms; ++i) {
            file << "," << res.temperatures[k][i];
        }

        if (k < res.commands.size()) {
            abstraction::Id cid = res.commands[k];
            file << "," << cid;
            VEC_N u = abstraction::decode_command(cid, rooms);
            for (int i = 0; i < rooms; ++i) {
                file << "," << u[i];
            }
        } else {
            // Last line: no command executed from terminal state
            file << ",";
            for (int i = 0; i < rooms; ++i) {
                file << ",";
            }
        }
        file << "\n";
    }
}

} // namespace simulation
