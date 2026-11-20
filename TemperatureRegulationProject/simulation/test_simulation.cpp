#include "simulation.hpp"
#include <iostream>
#include <cassert>

using namespace simulation;

// Baseline safety synthesis matching Younes's fixed-point algorithm:
// for each cell, keeps only commands whose entire successor box stays inside the safe candidate set.
Controller build_safety_controller(int N) {
    auto A = abstraction::build_abstraction(N);
    Controller ctrl(A.num_states);

    // Iterative safety fixed-point
    std::vector<bool> winning(A.num_states, true);

    auto all_succ_safe = [&](const abstraction::Successors& box, const std::vector<bool>& win) {
        if (box.leaves_safe_set) return false;
        auto bins = box.lower;
        while (true) {
            auto c = abstraction::encode_cell(bins);
            if (!win[c]) return false;
            std::size_t r = 0;
            while (r < bins.size() && bins[r] == box.upper[r]) {
                bins[r] = box.lower[r];
                ++r;
            }
            if (r == bins.size()) return true;
            ++bins[r];
        }
    };

    while (true) {
        std::vector<bool> next_win(A.num_states, false);
        for (abstraction::Id c = 0; c < A.num_states; ++c) {
            if (!winning[c]) continue;
            for (abstraction::Id u = 0; u < A.num_commands; ++u) {
                const auto& box = A.transitions[c * A.num_commands + u];
                if (all_succ_safe(box, winning)) {
                    next_win[c] = true;
                    break;
                }
            }
        }
        if (next_win == winning) break;
        winning.swap(next_win);
    }

    // Collect all certified command IDs for each winning cell
    for (abstraction::Id c = 0; c < A.num_states; ++c) {
        if (!winning[c]) continue;
        for (abstraction::Id u = 0; u < A.num_commands; ++u) {
            const auto& box = A.transitions[c * A.num_commands + u];
            if (all_succ_safe(box, winning)) {
                ctrl[c].push_back(u);
            }
        }
    }

    return ctrl;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "  Building Safety Simulation & Verification Tests (Task 4)\n";
    std::cout << "========================================================\n" << std::endl;

    const int N = 3;
    const int horizon = 50; // Steps for simulation

    std::cout << "[Setup] Computing 3-room safety controller... " << std::flush;
    Controller ctrl = build_safety_controller(N);
    int certified_count = 0;
    for (const auto& cmds : ctrl) {
        if (!cmds.empty()) certified_count++;
    }
    std::cout << "Done! (" << certified_count << "/" << ctrl.size() << " certified winning cells)\n" << std::endl;

    // -------------------------------------------------------------
    // Required Deliverables for Rayane (Cell 11, Item 4)
    // -------------------------------------------------------------

    // Trace 1: All-cold start (17, 17, 17)
    {
        std::cout << "[Deliverable 1] Simulating All-Cold trace (17, 17, 17)... " << std::flush;
        VEC_N cold_init = {17.0, 17.0, 17.0};
        auto res = simulate(ctrl, cold_init, horizon);
        assert(res.safe);
        assert(!res.uncertified);
        assert(res.temperatures.size() == static_cast<std::size_t>(horizon + 1));
        assert(res.commands.size() == static_cast<std::size_t>(horizon));
        export_csv(res, N, "sim_trace1_cold.csv");
        std::cout << "PASSED -> sim_trace1_cold.csv (" << res.commands.size() << " steps)" << std::endl;
    }

    // Trace 2: All-hot start (22, 22, 22)
    {
        std::cout << "[Deliverable 2] Simulating All-Hot trace (22, 22, 22)... " << std::flush;
        VEC_N hot_init = {22.0, 22.0, 22.0};
        auto res = simulate(ctrl, hot_init, horizon);
        assert(res.safe);
        assert(!res.uncertified);
        assert(res.temperatures.size() == static_cast<std::size_t>(horizon + 1));
        assert(res.commands.size() == static_cast<std::size_t>(horizon));
        export_csv(res, N, "sim_trace2_hot.csv");
        std::cout << "PASSED -> sim_trace2_hot.csv (" << res.commands.size() << " steps)" << std::endl;
    }

    // Trace 3: Mixed start (17, 19.5, 22) - Target for the report plot & presentation!
    {
        std::cout << "[Deliverable 3] Simulating Mixed trace (17, 19.5, 22)... " << std::flush;
        VEC_N mixed_init = {17.0, 19.5, 22.0};
        auto res = simulate(ctrl, mixed_init, horizon);
        assert(res.safe);
        assert(!res.uncertified);
        assert(res.temperatures.size() == static_cast<std::size_t>(horizon + 1));
        assert(res.commands.size() == static_cast<std::size_t>(horizon));
        export_csv(res, N, "sim_trace3_mixed.csv");
        std::cout << "PASSED -> sim_trace3_mixed.csv (" << res.commands.size() << " steps)" << std::endl;

        // Display first update step for Batch 2 presentation preparation
        std::cout << "  -> Initial state: [" << res.temperatures[0][0] << ", "
                  << res.temperatures[0][1] << ", " << res.temperatures[0][2] << "]\n";
        std::cout << "  -> Applied command ID: " << res.commands[0] << "\n";
        auto u0 = abstraction::decode_command(res.commands[0], N);
        std::cout << "  -> Heater powers u: [" << u0[0] << ", " << u0[1] << ", " << u0[2] << "]\n";
        std::cout << "  -> Next temperatures: [" << res.temperatures[1][0] << ", "
                  << res.temperatures[1][1] << ", " << res.temperatures[1][2] << "]\n";
    }

    // -------------------------------------------------------------
    // Additional Unit Tests (Checklist in Cell 11 & Cell 12)
    // -------------------------------------------------------------

    // Test 4: Invariant check (K completed updates => K+1 temperatures, K commands)
    {
        std::cout << "\n[Checklist Test 4] Invariant check on small horizon (K=5)... " << std::flush;
        VEC_N init = {18.5, 19.0, 20.0};
        auto res = simulate(ctrl, init, 5);
        assert(res.safe);
        assert(!res.uncertified);
        assert(res.temperatures.size() == 6);
        assert(res.commands.size() == 5);
        std::cout << "PASSED" << std::endl;
    }

    // Test 5: Cell boundaries (17.5, 18.0, 21.5, 22.0)
    {
        std::cout << "[Checklist Test 5] Cell boundaries check (17.5, 18.0, 21.5)... " << std::flush;
        VEC_N init = {17.5, 18.0, 21.5};
        auto res = simulate(ctrl, init, 20);
        assert(res.safe);
        assert(!res.uncertified);
        std::cout << "PASSED" << std::endl;
    }

    // Test 6: Invalid start (< 17 or > 22 => safe = false immediately)
    {
        std::cout << "[Checklist Test 6] Invalid start detection (< 17 or > 22)... " << std::flush;
        VEC_N cold = {16.9, 18.0, 20.0};
        auto res_cold = simulate(ctrl, cold, 10);
        assert(!res_cold.safe);
        assert(res_cold.temperatures.size() == 1);
        assert(res_cold.commands.empty());

        VEC_N hot = {18.0, 22.1, 20.0};
        auto res_hot = simulate(ctrl, hot, 10);
        assert(!res_hot.safe);
        assert(res_hot.temperatures.size() == 1);
        assert(res_hot.commands.empty());
        std::cout << "PASSED" << std::endl;
    }

    // Test 7: Uncertified policy handling (empty command list => uncertified = true)
    {
        std::cout << "[Checklist Test 7] Uncertified policy handling (empty command list)... " << std::flush;
        Controller empty_ctrl(1000); // 1000 uncertified cells (empty vectors)
        VEC_N init = {19.0, 19.0, 19.0};
        auto res = simulate(empty_ctrl, init, 10);
        assert(res.safe);
        assert(res.uncertified);
        assert(res.temperatures.size() == 1);
        assert(res.commands.empty());
        std::cout << "PASSED" << std::endl;
    }

    std::cout << "\n========================================================";
    std::cout << "\n>>> ALL 7 TESTS AND DELIVERABLES PASSED SUCCESSFULLY! <<<";
    std::cout << "\n========================================================\n" << std::endl;

    return 0;
}
