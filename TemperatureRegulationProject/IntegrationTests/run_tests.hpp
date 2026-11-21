#pragma once

// Integration test and benchmark runner for building temperature control
// (Rayane Khaldi).
//
// Connects all modules:
//   - dynamics/dynamics.hpp, coord2state.hpp, state2coord.hpp (Omar)
//   - abstraction/abstraction.hpp (Aymane)
//   - Safety_Synthesis/SafetyAbstraction/checked_abstraction.hpp,
//     Safety_Synthesis/safety_synthesis.hpp (Younes)
//   - simulation/simulation.hpp and simulation/sim_trace*.csv (Amine)
//
// Entry points:
//   bool verify_controller(const abstraction::Abstraction&, const simulation::Controller&)
//   void run_experiments()
//   bool run_experiments_detailed(...)

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../dynamics/dynamics.hpp"
#include "../dynamics/coord2state.hpp"
#include "../dynamics/state2coord.hpp"
#include "../abstraction/abstraction.hpp"
#include "../Safety_Synthesis/SafetyAbstraction/checked_abstraction.hpp"
#include "../Safety_Synthesis/safety_synthesis.hpp"
#include "../simulation/simulation.hpp"

#include "verification.hpp"
#include "measurements.hpp"

namespace integ {

inline std::string vec_to_string(const VEC_N& v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "(";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) oss << ", ";
        oss << v[i];
    }
    oss << ")";
    return oss.str();
}

// ---------------------------------------------------------------------------
// [Test 1] Self-tests of the independent trajectory checker
// ---------------------------------------------------------------------------
// Checks that the verifier catches out-of-range temperatures, NaN/Inf,
// dynamics mismatches, wrong vector lengths, and forged simulator safe flags.

inline bool test_trajectory_checker_self_tests(const simulation::Controller& ctrl3) {
    std::cout << "\n[Test 1] Independent trajectory checker self-tests...\n";
    bool all_ok = true;
    auto check_case = [&](const std::string& label, bool cond) {
        std::cout << "  " << (cond ? "ok   " : "FAIL ") << label << "\n";
        all_ok = all_ok && cond;
    };

    // 1a. Genuine safe 10-step run must pass.
    {
        const auto res = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 10);
        const auto aud = check_trajectory(res, ctrl3, 3, 10);
        check_case("Genuine safe trajectory accepted", aud.ok() && aud.all_temps_safe);
    }

    // 1b. Simulator flag lies (safe=true) when temperature dips below 17.0.
    {
        simulation::SimulationResult forged = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 3);
        forged.temperatures[2][0] = 16.95;  // tamper with recorded temperature
        forged.safe = true;                 // lie on the status flag
        const auto aud = check_trajectory(forged, ctrl3, 3, 3);
        check_case("Lying safe=true flag with T=16.95 rejected directly",
                   !aud.all_temps_safe && !aud.flags_consistent && !aud.ok());
    }

    // 1c. Simulator flag lies (safe=true) when temperature exceeds 22.0.
    {
        simulation::SimulationResult forged = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 3);
        forged.temperatures[3][1] = 22.05;
        forged.safe = true;
        const auto aud = check_trajectory(forged, ctrl3, 3, 3);
        check_case("Lying safe=true flag with T=22.05 rejected directly",
                   !aud.all_temps_safe && !aud.flags_consistent && !aud.ok());
    }

    // 1d. Non-finite temperature (NaN) with safe=true flag.
    {
        simulation::SimulationResult forged = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 2);
        forged.temperatures[1][2] = std::numeric_limits<double>::quiet_NaN();
        forged.safe = true;
        const auto aud = check_trajectory(forged, ctrl3, 3, 2);
        check_case("NaN temperature with safe=true rejected",
                   !aud.all_temps_safe && !aud.ok());
    }

    // 1e. Dynamics mismatch while still staying inside [17, 22].
    {
        simulation::SimulationResult forged = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 3);
        forged.temperatures[1][0] += 0.01;  // still safe, but violates f(T, u)
        const auto aud = check_trajectory(forged, ctrl3, 3, 3);
        check_case("In-bounds dynamics tampering rejected", !aud.dynamics_ok && !aud.ok());
    }

    // 1f. Vector length invariant broken (K+1 temperatures != commands + 1).
    {
        simulation::SimulationResult forged = simulation::simulate(ctrl3, {18.5, 19.5, 20.5}, 3);
        forged.temperatures.pop_back();
        const auto aud = check_trajectory(forged, ctrl3, 3, 3);
        check_case("Broken K+1 temperature length invariant rejected", !aud.shapes_ok && !aud.ok());
    }

    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// [Test 2] Cross-module interface, boundary & overflow checks
// ---------------------------------------------------------------------------

inline bool test_interfaces_and_overflow() {
    std::cout << "\n[Test 2] Cross-module interfaces, boundaries, and overflow guards...\n";
    bool all_ok = true;
    auto check_case = [&](const std::string& label, bool cond) {
        std::cout << "  " << (cond ? "ok   " : "FAIL ") << label << "\n";
        all_ok = all_ok && cond;
    };

    // 2a. Independent vs Aymane encode/decode/command/locate checks on N=3.
    bool enc_ok = true;
    for (abstraction::Id s = 0; s < 1000; ++s) {
        abstraction::Cell ref_bins;
        reference_decode_cell(s, 3, ref_bins);
        if (abstraction::decode_cell(s, 3) != ref_bins) enc_ok = false;
        if (abstraction::encode_cell(ref_bins) != s) enc_ok = false;
    }
    check_case("All 1,000 state IDs match independent base-10 encode/decode", enc_ok);

    bool cmd_ok = true;
    for (abstraction::Id u = 0; u < 27; ++u) {
        VEC_N ref_u;
        reference_decode_command(u, 3, ref_u);
        const VEC_N got_u = abstraction::decode_command(u, 3);
        if (got_u.size() != 3) cmd_ok = false;
        for (int r = 0; r < 3; ++r) {
            if (std::fabs(got_u[static_cast<std::size_t>(r)] -
                          ref_u[static_cast<std::size_t>(r)]) > 1e-12) {
                cmd_ok = false;
            }
        }
    }
    check_case("All 27 command IDs (incl. ID 0 = all-off) match independent base-3 decode", cmd_ok);

    // 2b. Checked 3-room example: T=(18, 20, 21), u=(0.3, 0, 0.6) -> (20.259, 18.605, 19.776).
    {
        const VEC_N T = {18.0, 20.0, 21.0};
        const VEC_N u = {0.3, 0.0, 0.6};
        const VEC_N got = dyn::f(T, u);
        const VEC_N ref = reference_step(T, u);
        const bool match = std::fabs(got[0] - 20.259) < 1e-9 &&
                           std::fabs(got[1] - 18.605) < 1e-9 &&
                           std::fabs(got[2] - 19.776) < 1e-9 &&
                           std::fabs(got[0] - ref[0]) < 1e-12 &&
                           std::fabs(got[1] - ref[1]) < 1e-12 &&
                           std::fabs(got[2] - ref[2]) < 1e-12;
        check_case("3-room reference step matches dyn::f & reference_step", match);
    }

    // 2c. Boundary & out-of-range temperature location checks.
    {
        bool loc_ok = true;
        const std::vector<std::pair<VEC_N, abstraction::Id>> boundary_cases = {
            {{17.0, 17.0, 17.0}, 0},
            {{17.5, 17.0, 17.0}, 1},     // internal boundary -> upper bin (bin 1)
            {{17.0, 17.5, 17.0}, 10},
            {{17.0, 17.0, 17.5}, 100},
            {{21.5, 21.5, 21.5}, 999},   // 21.5 -> bin 9
            {{22.0, 22.0, 22.0}, 999},   // 22.0 -> bin 9 (not rejected, not bin 10)
        };
        for (const auto& bc : boundary_cases) {
            const VEC_N& T = bc.first;
            const abstraction::Id expected_id = bc.second;
            abstraction::Id ref_id = -1;
            const bool ref_valid = reference_locate_cell(T, ref_id);
            const abstraction::Id got_id = abstraction::locate_cell(T);
            if (!ref_valid || ref_id != expected_id || got_id != expected_id) {
                loc_ok = false;
            }
        }
        check_case("Internal boundaries (upper bin) and 22.0 -> bin 9 verified", loc_ok);

        const std::vector<VEC_N> invalid_temps = {
            {16.9999, 19.0, 20.0},
            {19.0, 22.0001, 20.0},
            {std::numeric_limits<double>::quiet_NaN(), 19.0, 20.0},
            {std::numeric_limits<double>::infinity(), 19.0, 20.0},
            {-std::numeric_limits<double>::infinity(), 19.0, 20.0},
        };
        bool reject_ok = true;
        for (const auto& T : invalid_temps) {
            bool threw = false;
            try {
                (void)abstraction::locate_cell(T);
            } catch (const std::out_of_range&) {
                threw = true;
            }
            if (!threw) reject_ok = false;
        }
        check_case("Out-of-range and non-finite temperatures rejected (not clamped)", reject_ok);
    }

    // 2d. Overflow & invalid-room-count guards.
    {
        bool rej_small = false;
        try {
            (void)safety_abstraction::build_abstraction(2);
        } catch (const std::invalid_argument&) {
            rej_small = true;
        }
        check_case("safety_abstraction::build_abstraction(2) throws invalid_argument", rej_small);

        bool rej_overflow_prod = false;
        try {
            // 30^13 > INT64_MAX (even though 10^13 and 3^13 individually fit).
            (void)safety_abstraction::build_abstraction(13);
        } catch (const std::overflow_error&) {
            rej_overflow_prod = true;
        }
        check_case("safety_abstraction::build_abstraction(13) rejects 30^13 overflow",
                   rej_overflow_prod);

        bool rej_overflow_pow = false;
        try {
            // 10^19 > INT64_MAX.
            (void)safety_abstraction::multiplier(19, dyn::cpr);
        } catch (const std::overflow_error&) {
            rej_overflow_pow = true;
        }
        check_case("safety_abstraction::multiplier(19, 10) rejects 10^19 overflow",
                   rej_overflow_pow);
    }

    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// [Test 3] Exhaustive Abstraction Audit + Analytical Sanity Check (N = 3)
// ---------------------------------------------------------------------------

inline bool test_abstraction_audit(const abstraction::Abstraction& A3,
                                   const simulation::Controller& ctrl3) {
    std::cout << "\n[Test 3] Exhaustive independent audit of N=3 abstraction (27,000 pairs)...\n";
    const AbstractionAudit aud = audit_abstraction(A3);
    std::cout << "  " << aud.pairs_checked << " state-command transitions independently recomputed, "
              << aud.mismatches << " mismatch(es)\n";
    for (const auto& m : aud.first_mismatches) {
        std::cout << "  MISMATCH: " << m << "\n";
    }

    // Analytical sanity check: uniform u_i = 0.3 (command ID 13 = 1*1 + 1*3 + 1*9)
    // maps [17,22]^3 into [17.081, 21.721]^3, so 100% of states must be controllable.
    abstraction::Id certified = 0;
    bool all_include_uniform_03 = true;
    for (abstraction::Id s = 0; s < A3.num_states; ++s) {
        const auto& cmds = ctrl3[static_cast<std::size_t>(s)];
        if (!cmds.empty()) ++certified;
        if (std::find(cmds.begin(), cmds.end(), 13) == cmds.end()) {
            all_include_uniform_03 = false;
        }
    }
    const bool full_invariant = (certified == A3.num_states) && all_include_uniform_03;
    std::cout << "  Analytical check (u=(0.3,0.3,0.3), cmd ID 13): " << certified << "/"
              << A3.num_states << " states certified, cmd 13 present everywhere = "
              << (all_include_uniform_03 ? "yes" : "NO") << "\n";

    const bool passed = aud.ok && full_invariant;
    std::cout << (passed ? "  -> PASSED\n" : "  -> FAILED\n");
    return passed;
}

// ---------------------------------------------------------------------------
// [Test 4] Policy verification & intentional corruption tests
// ---------------------------------------------------------------------------
// Confirms that verify_controller accepts the synthesized controller and
// rejects corrupted policies.

inline bool test_policy_verification_and_corruption(const abstraction::Abstraction& A3,
                                                    const simulation::Controller& ctrl3) {
    std::cout << "\n[Test 4] Controller verification & intentional policy corruption tests...\n";
    bool all_ok = true;
    auto check_case = [&](const std::string& label, bool cond) {
        std::cout << "  " << (cond ? "ok   " : "FAIL ") << label << "\n";
        all_ok = all_ok && cond;
    };

    // Genuine controller must pass.
    const ControllerAudit clean = audit_controller(A3, ctrl3);
    check_case("Genuine synthesized N=3 controller verified (" +
               std::to_string(clean.certified_states) + "/" +
               std::to_string(clean.total_states) + " states, " +
               std::to_string(clean.certified_pairs) + " safe pairs, " +
               std::to_string(clean.successors_checked) + " successor checks)",
               clean.ok && verify_controller(A3, ctrl3));

    // Corruption 1: Inject an exiting command (cmd 0 = all heaters off at coldest cell 0).
    {
        simulation::Controller bad = ctrl3;
        bad[0] = {0};  // cell (0,0,0) with u=(0,0,0) cools below 17°C -> leaves_safe_set = true
        check_case("Corruption 1: exiting command (cell 0 -> cmd 0) rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 2: Break invariance by uncertifying a distinct successor cell reachable from a certified cell.
    {
        simulation::Controller bad = ctrl3;
        abstraction::Id src_cell = 0;
        abstraction::Id succ_id = -1;
        for (abstraction::Id s = 0; s < A3.num_states && succ_id == -1; ++s) {
            const abstraction::Id u0 = bad[static_cast<std::size_t>(s)][0];
            const auto& box = A3.transitions[static_cast<std::size_t>(s * A3.num_commands + u0)];
            abstraction::Id lo_id = -1, hi_id = -1;
            reference_encode_cell(box.lower, lo_id);
            reference_encode_cell(box.upper, hi_id);
            if (lo_id != s) { src_cell = s; succ_id = lo_id; }
            else if (hi_id != s) { src_cell = s; succ_id = hi_id; }
        }
        bad[static_cast<std::size_t>(succ_id)].clear();
        check_case("Corruption 2: distinct successor uncertified (cell " +
                   std::to_string(src_cell) + " -> cell " + std::to_string(succ_id) + ") rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 3: Out-of-range negative command ID (-1 stored inside vector).
    {
        simulation::Controller bad = ctrl3;
        bad[42].push_back(-1);
        check_case("Corruption 3: negative command ID (-1) rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 4: Out-of-range command ID (>= num_commands).
    {
        simulation::Controller bad = ctrl3;
        bad[42].push_back(A3.num_commands);
        check_case("Corruption 4: command ID == num_commands (27) rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 5: Duplicate command ID in a cell's certified command list.
    {
        simulation::Controller bad = ctrl3;
        bad[10].push_back(bad[10][0]);
        check_case("Corruption 5: duplicate command ID in cell 10 rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 6: Wrong controller vector length.
    {
        simulation::Controller bad = ctrl3;
        bad.pop_back();
        check_case("Corruption 6: truncated controller (999 != 1000 states) rejected",
                   !verify_controller(A3, bad));
    }

    // Corruption 7: Model transition corrupted to leave safe set while controller still uses it.
    {
        abstraction::Abstraction bad_model = A3;
        const abstraction::Id u = ctrl3[500][0];
        bad_model.transitions[static_cast<std::size_t>(500 * A3.num_commands + u)].leaves_safe_set = true;
        check_case("Corruption 7: corrupted abstraction exit flag detected",
                   !verify_controller(bad_model, ctrl3));
    }

    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// [Test 5] Closed-Loop Simulation Scenarios & Fixed-Seed Sweep
// ---------------------------------------------------------------------------

struct SimTestRecord {
    std::string group;       // "named_N3", "sweep_N3", "named_N4", "sweep_N4"
    int rooms = 3;
    std::string scenario;
    VEC_N initial;
    int requested_steps = 0;
    int completed_steps = 0;
    bool sim_safe = false;
    bool sim_uncertified = false;
    bool expected_safe = true;
    bool expected_uncertified = false;
    double min_temp = 0.0;
    double max_temp = 0.0;
    double max_dyn_error = 0.0;
    bool passed = false;
    std::string notes;
};

inline SimTestRecord evaluate_sim_case(const simulation::Controller& controller,
                                       const std::string& group,
                                       int rooms,
                                       const std::string& name,
                                       const VEC_N& initial,
                                       int steps,
                                       bool expected_safe,
                                       bool expected_uncertified) {
    SimTestRecord rec;
    rec.group = group;
    rec.rooms = rooms;
    rec.scenario = name;
    rec.initial = initial;
    rec.requested_steps = steps;
    rec.expected_safe = expected_safe;
    rec.expected_uncertified = expected_uncertified;

    const auto res = simulation::simulate(controller, initial, steps);
    const auto aud = check_trajectory(res, controller, rooms, steps);

    rec.completed_steps = static_cast<int>(res.commands.size());
    rec.sim_safe = res.safe;
    rec.sim_uncertified = res.uncertified;
    rec.min_temp = std::isfinite(aud.min_temp) ? aud.min_temp : 0.0;
    rec.max_temp = std::isfinite(aud.max_temp) ? aud.max_temp : 0.0;
    rec.max_dyn_error = aud.max_dyn_error;

    std::vector<std::string> issues = aud.issues;
    if (res.safe != expected_safe) {
        issues.push_back(std::string("expected safe=") + (expected_safe ? "true" : "false") +
                         ", got " + (res.safe ? "true" : "false"));
    }
    if (res.uncertified != expected_uncertified) {
        issues.push_back(std::string("expected uncertified=") +
                         (expected_uncertified ? "true" : "false") +
                         ", got " + (res.uncertified ? "true" : "false"));
    }

    rec.passed = aud.ok() && issues.empty();
    if (issues.empty()) {
        rec.notes = "direct numerical & dynamics check passed";
    } else {
        for (std::size_t i = 0; i < issues.size(); ++i) {
            if (i) rec.notes += "; ";
            rec.notes += issues[i];
        }
    }
    return rec;
}

inline bool test_simulations(const simulation::Controller& ctrl3,
                             const simulation::Controller* ctrl4,
                             std::vector<SimTestRecord>& records,
                             int& sweep3_total, int& sweep3_pass,
                             int& sweep4_total, int& sweep4_pass) {
    std::cout << "\n[Test 5] Closed-loop simulations (ordinary, boundary, invalid, uncertified, sweep)...\n";
    bool all_ok = true;

    struct NamedCase {
        std::string name;
        VEC_N init;
        int steps;
        bool exp_safe;
        bool exp_uncert;
    };

    const std::vector<NamedCase> cases3 = {
        {"All-cold boundary (17, 17, 17)",              {17.0, 17.0, 17.0},            50, true,  false},
        {"All-hot boundary (22, 22, 22)",               {22.0, 22.0, 22.0},            50, true,  false},
        {"Mixed extremes (17, 19.5, 22)",               {17.0, 19.5, 22.0},            50, true,  false},
        {"Cell boundaries (17.5, 18.0, 21.5)",          {17.5, 18.0, 21.5},            50, true,  false},
        {"Cell boundaries (18.5, 20.0, 21.0)",          {18.5, 20.0, 21.0},            50, true,  false},
        {"Ordinary interior (18.2, 20.1, 21.3)",        {18.2, 20.1, 21.3},            50, true,  false},
        {"Ordinary interior (19.3, 19.7, 20.4)",        {19.3, 19.7, 20.4},            50, true,  false},
        {"Near-boundary epsilon (17+1e-12, 22-1e-12)",  {17.0 + 1e-12, 22.0 - 1e-12, 19.5}, 50, true, false},
        {"Near internal edge (17.5-1e-12, 17.5, +1e-12)", {17.5 - 1e-12, 17.5, 17.5 + 1e-12}, 50, true, false},
        {"Long horizon (200 steps)",                    {17.1, 21.9, 18.6},           200, true,  false},
        {"Zero horizon (steps = 0)",                    {19.0, 19.5, 20.0},             0, true,  false},
        {"Invalid start: below 17 (16.99)",             {16.99, 19.0, 20.0},           20, false, false},
        {"Invalid start: above 22 (22.01)",             {19.0, 22.01, 20.0},           20, false, false},
        {"Invalid start: NaN temperature",              {std::numeric_limits<double>::quiet_NaN(), 19.0, 20.0}, 20, false, false},
        {"Invalid start: +Inf temperature",             {19.0, std::numeric_limits<double>::infinity(), 20.0},  20, false, false},
        {"Invalid start: -Inf temperature",             {19.0, 20.0, -std::numeric_limits<double>::infinity()}, 20, false, false},
    };

    int named_pass = 0;
    for (const auto& c : cases3) {
        auto rec = evaluate_sim_case(ctrl3, "named_N3", 3, c.name, c.init, c.steps,
                                     c.exp_safe, c.exp_uncert);
        if (rec.passed) ++named_pass;
        else {
            std::cout << "  FAIL: " << rec.scenario << " -> " << rec.notes << "\n";
            all_ok = false;
        }
        records.push_back(std::move(rec));
    }

    // Uncertified policy tests: (a) completely empty controller, (b) single cell emptied.
    {
        simulation::Controller empty_ctrl(1000);
        auto rec = evaluate_sim_case(empty_ctrl, "named_N3", 3,
                                     "Uncertified policy: empty controller",
                                     {19.0, 19.5, 20.0}, 20, true, true);
        if (rec.passed) ++named_pass;
        else {
            std::cout << "  FAIL: " << rec.scenario << " -> " << rec.notes << "\n";
            all_ok = false;
        }
        records.push_back(std::move(rec));
    }
    {
        // Empty only the cell visited at step 1 from (17.0, 17.0, 17.0) so it takes 1 step then stops uncertified.
        const auto probe = simulation::simulate(ctrl3, {17.0, 17.0, 17.0}, 2);
        abstraction::Id cell_step1 = -1;
        reference_locate_cell(probe.temperatures[1], cell_step1);
        simulation::Controller partial_ctrl = ctrl3;
        // Choose a start whose step-1 cell differs from step-0 cell: (17.0, 19.5, 22.0).
        const auto probe_mix = simulation::simulate(ctrl3, {17.0, 19.5, 22.0}, 2);
        abstraction::Id cell0 = -1, cell1 = -1;
        reference_locate_cell(probe_mix.temperatures[0], cell0);
        reference_locate_cell(probe_mix.temperatures[1], cell1);
        if (cell1 >= 0 && cell1 != cell0) {
            partial_ctrl[static_cast<std::size_t>(cell1)].clear();
            auto rec = evaluate_sim_case(partial_ctrl, "named_N3", 3,
                                         "Uncertified mid-run (step-1 cell emptied)",
                                         {17.0, 19.5, 22.0}, 20, true, true);
            const bool took_one = (rec.completed_steps == 1);
            if (!took_one) {
                rec.passed = false;
                rec.notes += "; expected stop after 1 step, got " +
                             std::to_string(rec.completed_steps);
            }
            if (rec.passed) ++named_pass;
            else {
                std::cout << "  FAIL: " << rec.scenario << " -> " << rec.notes << "\n";
                all_ok = false;
            }
            records.push_back(std::move(rec));
        }
    }

    // Deterministic reproducibility check.
    {
        const VEC_N init = {17.3, 19.8, 21.6};
        const auto r1 = simulation::simulate(ctrl3, init, 50);
        const auto r2 = simulation::simulate(ctrl3, init, 50);
        const bool identical = (r1.temperatures == r2.temperatures) &&
                               (r1.commands == r2.commands) &&
                               (r1.safe == r2.safe) &&
                               (r1.uncertified == r2.uncertified);
        std::cout << "  Deterministic reproducibility (N=3, 50 steps): "
                  << (identical ? "identical" : "DIFFERED") << "\n";
        all_ok = all_ok && identical;
    }

    const int named_total = static_cast<int>(records.size());
    std::cout << "  Named N=3 scenarios: " << named_pass << " / " << named_total << " passed\n";

    // Fixed-seed random sweep for N=3 (200 random starts in [17, 22]^3, horizon 50).
    sweep3_total = 200;
    sweep3_pass = 0;
    {
        std::mt19937 rng(2026);
        std::uniform_real_distribution<double> dist(REF_T_LO, REF_T_HI);
        for (int i = 0; i < sweep3_total; ++i) {
            VEC_N init = {dist(rng), dist(rng), dist(rng)};
            auto rec = evaluate_sim_case(ctrl3, "sweep_N3", 3,
                                         "random_N3 #" + std::to_string(i),
                                         init, 50, true, false);
            if (rec.passed) ++sweep3_pass;
            else {
                if (sweep3_total - sweep3_pass <= 5) {
                    std::cout << "  FAIL: " << rec.scenario << " " << vec_to_string(init)
                              << " -> " << rec.notes << "\n";
                }
                all_ok = false;
            }
            records.push_back(std::move(rec));
        }
        std::cout << "  Fixed-seed sweep (N=3, seed=2026): " << sweep3_pass << " / "
                  << sweep3_total << " passed\n";
    }

    // Optional N=4 simulation checks if N=4 controller was synthesized.
    sweep4_total = 0;
    sweep4_pass = 0;
    if (ctrl4 != nullptr && !ctrl4->empty()) {
        const std::vector<NamedCase> cases4 = {
            {"N=4 All-cold (17, 17, 17, 17)",       {17.0, 17.0, 17.0, 17.0}, 50, true, false},
            {"N=4 All-hot (22, 22, 22, 22)",        {22.0, 22.0, 22.0, 22.0}, 50, true, false},
            {"N=4 Mixed (17, 18.5, 20.5, 22)",      {17.0, 18.5, 20.5, 22.0}, 50, true, false},
        };
        int n4_named_pass = 0;
        for (const auto& c : cases4) {
            auto rec = evaluate_sim_case(*ctrl4, "named_N4", 4, c.name, c.init, c.steps,
                                         c.exp_safe, c.exp_uncert);
            if (rec.passed) ++n4_named_pass;
            else {
                std::cout << "  FAIL: " << rec.scenario << " -> " << rec.notes << "\n";
                all_ok = false;
            }
            records.push_back(std::move(rec));
        }
        sweep4_total = 50;
        std::mt19937 rng4(4026);
        std::uniform_real_distribution<double> dist(REF_T_LO, REF_T_HI);
        for (int i = 0; i < sweep4_total; ++i) {
            VEC_N init = {dist(rng4), dist(rng4), dist(rng4), dist(rng4)};
            auto rec = evaluate_sim_case(*ctrl4, "sweep_N4", 4,
                                         "random_N4 #" + std::to_string(i),
                                         init, 50, true, false);
            if (rec.passed) ++sweep4_pass;
            else all_ok = false;
            records.push_back(std::move(rec));
        }
        std::cout << "  N=4 simulations: " << n4_named_pass << "/" << cases4.size()
                  << " named, " << sweep4_pass << "/" << sweep4_total << " random passed\n";
    }

    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// [Test 6] Verification of Amine's exported CSV traces
// ---------------------------------------------------------------------------

inline bool test_amine_csv_traces(const simulation::Controller& ctrl3,
                                  const std::string& sim_dir,
                                  std::vector<CsvTraceAudit>& out_audits) {
    std::cout << "\n[Test 6] Independent verification of Amine's CSV traces (simulation/README.md Sec. 5)...\n";
    const std::vector<std::string> files = {
        "sim_trace1_cold.csv",
        "sim_trace2_hot.csv",
        "sim_trace3_mixed.csv",
    };

    bool all_ok = true;
    for (const auto& name : files) {
        const std::string full_path = sim_dir + "/" + name;
        CsvTraceAudit aud = verify_csv_trace(full_path, ctrl3, 3, 50);
        std::cout << "  " << (aud.ok() ? "ok   " : "FAIL ") << std::left << std::setw(26) << name
                  << " rows=" << aud.temp_rows << " cmds=" << aud.cmd_rows
                  << " T in [" << std::fixed << std::setprecision(3)
                  << aud.min_temp << ", " << aud.max_temp << "]"
                  << " max_step_err=" << std::scientific << std::setprecision(2)
                  << aud.max_step_error << std::fixed << "\n";
        for (const auto& iss : aud.issues) {
            std::cout << "       ISSUE: " << iss << "\n";
        }
        all_ok = all_ok && aud.ok();
        out_audits.push_back(std::move(aud));
    }

    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// Output writers (Markdown and CSV reports)
// ---------------------------------------------------------------------------

inline void write_experiment_reports(const std::string& output_dir,
                                     const std::vector<BenchmarkRow>& benchmarks,
                                     const std::vector<SimTestRecord>& sim_records,
                                     const std::vector<CsvTraceAudit>& csv_audits,
                                     int sweep3_total, int sweep3_pass,
                                     int sweep4_total, int sweep4_pass,
                                     bool all_passed) {
    const std::string md_path = output_dir + "/integration_results.md";
    const std::string sim_csv_path = output_dir + "/integration_results.csv";
    const std::string bench_csv_path = output_dir + "/baseline_measurements.csv";

    std::ofstream md(md_path);
    std::ofstream sim_csv(sim_csv_path);
    std::ofstream bench_csv(bench_csv_path);

    if (!md || !sim_csv || !bench_csv) {
        std::cerr << "Warning: could not write report files in " << output_dir << "\n";
        return;
    }

    // --- baseline_measurements.csv ---
    bench_csv << "method,N,states,commands,transitions,abstraction_s,synthesis_s,total_s,"
                 "peak_rss_bytes,estimated_data_bytes,certified_states,total_safe_pairs,"
                 "controllable_pct,verified,status\n";
    for (const auto& b : benchmarks) {
        bench_csv << b.method << "," << b.rooms << "," << b.num_states << "," << b.num_commands
                  << "," << b.num_transitions << "," << std::fixed << std::setprecision(6)
                  << b.abstraction_seconds << "," << b.synthesis_seconds << "," << b.total_seconds
                  << ","
                  << (b.peak_rss_bytes.available ? std::to_string(b.peak_rss_bytes.bytes) : "missing")
                  << "," << b.estimated_data_bytes << "," << b.certified_states << ","
                  << b.total_safe_pairs << "," << std::setprecision(2) << b.controllable_pct << ","
                  << (b.verified ? "PASS" : "FAIL") << ",\"" << b.status << "\"\n";
    }

    // --- integration_results.csv ---
    sim_csv << "id,group,rooms,scenario,initial,requested_steps,completed_steps,"
               "sim_safe,sim_uncertified,expected_safe,expected_uncertified,"
               "min_temp,max_temp,max_dyn_error,passed,notes\n";
    int id = 0;
    for (const auto& r : sim_records) {
        ++id;
        sim_csv << id << "," << r.group << "," << r.rooms << ",\"" << r.scenario << "\",\""
                << vec_to_string(r.initial) << "\"," << r.requested_steps << ","
                << r.completed_steps << "," << (r.sim_safe ? "true" : "false") << ","
                << (r.sim_uncertified ? "true" : "false") << ","
                << (r.expected_safe ? "true" : "false") << ","
                << (r.expected_uncertified ? "true" : "false") << ","
                << std::fixed << std::setprecision(6) << r.min_temp << "," << r.max_temp << ","
                << std::scientific << std::setprecision(3) << r.max_dyn_error << ","
                << (r.passed ? "PASS" : "FAIL") << ",\"" << r.notes << "\"\n";
    }

    // --- integration_results.md ---
    md << "# Building Temperature Regulation - Integration & Baseline Results\n\n"
       << "Generated by `IntegrationTests/test_integration.cpp`.\n\n"
       << "## Experimental Setup & Measurement Scope\n\n"
       << "- **Compiler:** " << compiler_info() << "\n"
       << "- **Build flags:** `" << build_flags_info() << "`\n"
       << "- **Platform:** " << platform_info() << "\n"
       << "- **Overall status:** **" << (all_passed ? "ALL CHECKS PASSED" : "FAILURES DETECTED") << "**\n"
       << "- **Timing scope:** Wall-clock `std::chrono::steady_clock`; abstraction (`safety_abstraction::build_abstraction`) and safety synthesis (`synthesize_safety`) are timed separately, excluding verification, simulation, and file I/O.\n"
       << "- **Memory scope:** OS process peak resident set size (`ru_maxrss` / `PeakWorkingSetSize`), reported as `missing` (never `0`) if unavailable. Use `./test_integration --bench N` for single-`N` isolated peak RSS.\n\n";

    md << "## Baseline Measurements\n\n"
       << "| Method | N | States | Commands | Abstraction time | Synthesis time | Total time | Peak memory | Est. model data | Controllable % | Verified |\n"
       << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
    for (const auto& b : benchmarks) {
        const double est_mib = static_cast<double>(b.estimated_data_bytes) / (1024.0 * 1024.0);
        md << "| " << b.method << " | " << b.rooms << " | " << b.num_states << " | "
           << b.num_commands << " | " << format_seconds(b.abstraction_seconds) << " | "
           << format_seconds(b.synthesis_seconds) << " | " << format_seconds(b.total_seconds)
           << " | " << format_memory_mb(b.peak_rss_bytes) << " | "
           << std::fixed << std::setprecision(2) << est_mib << " MiB | "
           << b.controllable_pct << "% (" << b.certified_states << "/" << b.num_states << ") | "
           << (b.verified ? "PASS" : "FAIL") << " |\n";
    }

    md << "\n## Independent Verification of Amine's CSV Traces (`simulation/`)\n\n"
       << "| Trace file | Temp rows | Commands | Min T (°C) | Max T (°C) | Max step error | Live replay match | Verdict |\n"
       << "|---|---:|---:|---:|---:|---:|---|---|\n";
    for (const auto& c : csv_audits) {
        const std::size_t slash = c.path.find_last_of("/\\");
        const std::string fname = (slash == std::string::npos) ? c.path : c.path.substr(slash + 1);
        md << "| `" << fname << "` | " << c.temp_rows << " | " << c.cmd_rows << " | "
           << std::fixed << std::setprecision(5) << c.min_temp << " | " << c.max_temp << " | "
           << std::scientific << std::setprecision(2) << c.max_step_error << " | "
           << (c.replay_matches_live ? "yes" : "NO") << " | "
           << (c.ok() ? "PASS" : "FAIL") << " |\n";
    }

    md << "\n## Named Closed-Loop Simulation Scenarios\n\n"
       << "| # | N | Scenario | Initial T (°C) | Steps | Safe | Uncertified | Min T | Max T | Check |\n"
       << "|---:|---:|---|---|---:|---|---|---:|---:|---|\n";
    int idx = 0;
    for (const auto& r : sim_records) {
        ++idx;
        if (r.group != "named_N3" && r.group != "named_N4") continue;
        md << "| " << idx << " | " << r.rooms << " | " << r.scenario << " | `"
           << vec_to_string(r.initial) << "` | " << r.completed_steps << "/" << r.requested_steps
           << " | " << (r.sim_safe ? "yes" : "no") << " | "
           << (r.sim_uncertified ? "yes" : "no") << " | "
           << std::fixed << std::setprecision(3) << r.min_temp << " | " << r.max_temp << " | "
           << (r.passed ? "PASS" : "FAIL") << " |\n";
    }

    md << "\n## Fixed-Seed Random Simulation Sweeps\n\n"
       << "| Rooms (N) | Seed | Horizon | Runs | Passed |\n"
       << "|---:|---:|---:|---:|---:|\n"
       << "| 3 | 2026 | 50 | " << sweep3_total << " | " << sweep3_pass << " / " << sweep3_total << " |\n";
    if (sweep4_total > 0) {
        md << "| 4 | 4026 | 50 | " << sweep4_total << " | " << sweep4_pass << " / " << sweep4_total << " |\n";
    }
    md << "\nAll individual runs are archived in `integration_results.csv`, and benchmark rows in `baseline_measurements.csv`.\n";
}

// ---------------------------------------------------------------------------
// Main experiment runner
// ---------------------------------------------------------------------------

inline bool run_experiments_detailed(const std::vector<int>& room_counts = {3, 4},
                                     const std::string& sim_dir = "../simulation",
                                     const std::string& output_dir = ".") {
    bool all_ok = true;

    std::cout << "======================================================================\n";
    std::cout << "  BUILDING TEMPERATURE REGULATION - INTEGRATION, VERIFICATION & BENCH \n";
    std::cout << "======================================================================\n";
    std::cout << "Compiler : " << compiler_info() << "\n";
    std::cout << "Flags    : " << build_flags_info() << "\n";
    std::cout << "Platform : " << platform_info() << "\n";

    // Build & time baseline models first (keep N=3 and N=4 models/controllers for tests).
    std::vector<BenchmarkRow> benchmarks;
    abstraction::Abstraction A3;
    simulation::Controller ctrl3;
    simulation::Controller ctrl4;
    bool has_n3 = false;
    bool has_n4 = false;

    std::cout << "\n[Baseline] Measuring reference abstraction & safety synthesis...\n";
    for (int N : room_counts) {
        abstraction::Abstraction A;
        simulation::Controller ctrl;
        BenchmarkRow row = benchmark_reference(N, true, &A, &ctrl);
        std::cout << "  N=" << N
                  << " | states=" << row.num_states
                  << " | cmds=" << row.num_commands
                  << " | abs=" << format_seconds(row.abstraction_seconds)
                  << " | syn=" << format_seconds(row.synthesis_seconds)
                  << " | peak_rss=" << format_memory_mb(row.peak_rss_bytes)
                  << " | ctrl=" << std::fixed << std::setprecision(2) << row.controllable_pct << "%"
                  << " | verified=" << (row.verified ? "PASS" : "FAIL") << "\n";
        all_ok = all_ok && row.verified && (row.status == "OK");

        if (N == 3) {
            A3 = std::move(A);
            ctrl3 = std::move(ctrl);
            has_n3 = true;
        } else if (N == 4) {
            ctrl4 = std::move(ctrl);
            has_n4 = true;
        }
        benchmarks.push_back(std::move(row));
    }

    if (!has_n3) {
        BenchmarkRow r3 = benchmark_reference(3, true, &A3, &ctrl3);
        all_ok = all_ok && r3.verified;
        benchmarks.insert(benchmarks.begin(), std::move(r3));
    }

    // Run all 6 verification & integration suites.
    all_ok = test_trajectory_checker_self_tests(ctrl3) && all_ok;
    all_ok = test_interfaces_and_overflow() && all_ok;
    all_ok = test_abstraction_audit(A3, ctrl3) && all_ok;
    all_ok = test_policy_verification_and_corruption(A3, ctrl3) && all_ok;

    std::vector<SimTestRecord> sim_records;
    int sweep3_total = 0, sweep3_pass = 0, sweep4_total = 0, sweep4_pass = 0;
    all_ok = test_simulations(ctrl3, has_n4 ? &ctrl4 : nullptr, sim_records,
                              sweep3_total, sweep3_pass, sweep4_total, sweep4_pass) && all_ok;

    std::vector<CsvTraceAudit> csv_audits;
    all_ok = test_amine_csv_traces(ctrl3, sim_dir, csv_audits) && all_ok;

    // Print console baseline table.
    std::cout << "\n======================================================================\n";
    std::cout << "                        BASELINE RESULTS TABLE                        \n";
    std::cout << "======================================================================\n";
    std::cout << std::left << std::setw(12) << "Method"
              << std::setw(4) << "N"
              << std::setw(16) << "Abstraction"
              << std::setw(16) << "Synthesis"
              << std::setw(15) << "Peak memory"
              << std::setw(16) << "Controllable"
              << "Verified\n";
    std::cout << std::string(87, '-') << "\n";
    for (const auto& b : benchmarks) {
        std::ostringstream pct;
        pct << std::fixed << std::setprecision(2) << b.controllable_pct << "%";
        std::cout << std::left << std::setw(12) << b.method
                  << std::setw(4) << b.rooms
                  << std::setw(16) << format_seconds(b.abstraction_seconds)
                  << std::setw(16) << format_seconds(b.synthesis_seconds)
                  << std::setw(15) << format_memory_mb(b.peak_rss_bytes)
                  << std::setw(16) << pct.str()
                  << (b.verified ? "PASS" : "FAIL") << "\n";
    }
    std::cout << std::string(87, '-') << "\n";

    write_experiment_reports(output_dir, benchmarks, sim_records, csv_audits,
                             sweep3_total, sweep3_pass, sweep4_total, sweep4_pass, all_ok);

    std::cout << "Reports written to:\n"
              << "  - " << output_dir << "/integration_results.md\n"
              << "  - " << output_dir << "/integration_results.csv\n"
              << "  - " << output_dir << "/baseline_measurements.csv\n";
    std::cout << "\n" << (all_ok ? "ALL INTEGRATION & VERIFICATION TESTS PASSED"
                                  : "SOME INTEGRATION & VERIFICATION TESTS FAILED") << "\n";
    return all_ok;
}

inline void run_experiments() {
    (void)run_experiments_detailed({3, 4}, "../simulation", ".");
}

}  // namespace integ
