#pragma once

// Integration test runner (Rayane Khaldi).
//
//   bool check_controller_table(g, controller)  -> audits the synthesized table
//   bool run_tests(controller, val_u)           -> checker self-tests, label
//                                                  cross-check, scenarios,
//                                                  random sweep, results table
//
// Both return true when everything passed.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "trajectory_check.hpp"
#include "../Simulation/simulation.hpp"

namespace integ {

// ---------------------------------------------------------------------------
// Reference labels (independent of boxToCells / label)
// ---------------------------------------------------------------------------

// Regions in tenths of a metre: integers only, no floating-point rounding.
// A cell with coordinates (i, j) covers [i-1, i] x [j-1, j] tenths. It is
// inside a region only when it is fully inside it.
inline std::vector<Region> build_reference_labels() {
    struct Tenths { int x_lo, x_hi, y_lo, y_hi; Region region; };
    const Tenths boxes[] = {
        {30, 70, 30, 70, Region::R4},
        {40, 50, 85, 95, Region::R1},
        {85, 95, 20, 30, Region::R2},
        {20, 30, 5, 15, Region::R3},
    };
    const std::array<int, 4> strides = {1, dyn::n_x[0], dyn::n_x[0] * dyn::n_x[1],
                                        dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2]};
    const int n_states = strides[3];
    std::vector<Region> labels(n_states + 1, Region::NONE);
    for (int cell = 1; cell <= n_states; ++cell) {
        const Idx3 c = state2coord<3>(cell, strides);
        for (const Tenths& b : boxes) {
            if (c[0] - 1 >= b.x_lo && c[0] <= b.x_hi && c[1] - 1 >= b.y_lo && c[1] <= b.y_hi) {
                labels[cell] = b.region;
                break;
            }
        }
    }
    return labels;
}

// Cell of a state computed with x * 10 (not a division by the cell width).
// Only used for random states, which are never exactly on a cell edge.
inline int reference_cell(const Vec3& s) {
    if (outside_workspace(s)) return 0;
    double heading = std::fmod(s[2], 2.0 * PI);
    if (heading < -PI) heading += 2.0 * PI;
    if (heading >= PI) heading -= 2.0 * PI;
    const int i = static_cast<int>(std::floor(s[0] * 10.0)) + 1;
    const int j = static_cast<int>(std::floor(s[1] * 10.0)) + 1;
    int k = static_cast<int>(std::floor((heading + PI) / (2.0 * PI / dyn::n_x[2]))) + 1;
    k = std::min(std::max(k, 1), dyn::n_x[2]);
    const std::array<int, 4> strides = {1, dyn::n_x[0], dyn::n_x[0] * dyn::n_x[1],
                                        dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2]};
    return coord2state<3>({i, j, k}, strides);
}

// ---------------------------------------------------------------------------
// Audit of the controller table
// ---------------------------------------------------------------------------

// For every certified (cell, mission) with rank > 0, every possible successor
// of the stored command must be certified with a strictly smaller rank.
// Also checks the SUCCESS / FAILURE columns and that uncertified entries carry
// no command.
inline bool check_controller_table(const Transitions& g, const Controller& controller) {
    std::cout << "\n[Table audit] Controller table vs abstraction and reference labels...\n";
    const std::vector<Region> ref = build_reference_labels();
    const std::array<int, 4> strides = {1, dyn::n_x[0], dyn::n_x[0] * dyn::n_x[1],
                                        dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2]};
    const int n_states = strides[3];
    const int n_commands = dyn::n_u[0] * dyn::n_u[1];
    const int success = static_cast<int>(Mission::SUCCESS);
    const int failure = static_cast<int>(Mission::FAILURE);

    long checked = 0;
    long certified = 0;
    long violations = 0;
    auto report = [&](const std::string& msg) {
        if (violations < 5) std::cout << "  VIOLATION: " << msg << "\n";
        ++violations;
    };

    if (controller.rank.size() != static_cast<std::size_t>(n_states + 1) ||
        controller.command.size() != static_cast<std::size_t>(n_states + 1) ||
        g.size() != static_cast<std::size_t>(n_states) * n_commands) {
        std::cout << "  FAIL: controller or abstraction has the wrong size\n";
        return false;
    }

    for (int cell = 1; cell <= n_states; ++cell) {
        if (controller.rank[cell][success] != 0) report("SUCCESS rank is not 0 at cell " + std::to_string(cell));
        if (controller.rank[cell][failure] != -1) report("FAILURE column certified at cell " + std::to_string(cell));
        if (controller.command[cell][success] != 0 || controller.command[cell][failure] != 0)
            report("SUCCESS/FAILURE column has a command at cell " + std::to_string(cell));

        for (int m = 0; m < 3; ++m) {
            const int rank = controller.rank[cell][m];
            const int command = controller.command[cell][m];
            ++checked;
            if (rank == -1) {
                if (command != 0) report("uncertified entry with command, cell " + std::to_string(cell));
                continue;
            }
            ++certified;
            if (rank <= 0) { report("non-SUCCESS entry with rank " + std::to_string(rank) + ", cell " + std::to_string(cell)); continue; }
            if (command < 1 || command > n_commands) { report("invalid command at cell " + std::to_string(cell)); continue; }

            const std::array<int32_t, 2>& box = g[static_cast<std::size_t>(cell - 1) * n_commands + (command - 1)];
            if (box[0] == 0 && box[1] == 0) { report("command leaves the workspace, cell " + std::to_string(cell)); continue; }

            const Idx3 lo = state2coord<3>(box[0], strides);
            const Idx3 hi = state2coord<3>(box[1], strides);
            int angle = lo[2];
            bool ok = true;
            for (;;) {
                for (int y = lo[1]; y <= hi[1] && ok; ++y) {
                    for (int x = lo[0]; x <= hi[0] && ok; ++x) {
                        const int next_cell = coord2state<3>({x, y, angle}, strides);
                        // Mission after observing the successor, rebuilt from flags.
                        const Region reg = ref[next_cell];
                        Mission next = static_cast<Mission>(m);
                        if (reg == Region::R4) next = Mission::FAILURE;
                        else if (m == 0 && reg == Region::R1) next = Mission::VISITED_R1;
                        else if (m == 0 && reg == Region::R2) next = Mission::VISITED_R2;
                        else if (m == 1 && reg == Region::R2) next = Mission::FAILURE;
                        else if (m == 2 && reg == Region::R1) next = Mission::FAILURE;
                        else if (m != 0 && reg == Region::R3) next = Mission::SUCCESS;
                        const int next_rank = controller.rank[next_cell][static_cast<int>(next)];
                        if (next_rank < 0 || next_rank >= rank) {
                            report("successor " + std::to_string(next_cell) + " of cell " + std::to_string(cell) +
                                   " (mission " + std::to_string(m) + ", rank " + std::to_string(rank) +
                                   ") has rank " + std::to_string(next_rank));
                            ok = false;
                        }
                    }
                }
                if (!ok || angle == hi[2]) break;
                angle = angle % dyn::n_x[2] + 1;
            }
        }
    }

    std::cout << "  " << checked << " combinations checked, " << certified << " certified, "
              << violations << " violation(s)\n";
    std::cout << (violations == 0 ? "  -> PASSED: every successor of every stored command has a strictly smaller rank.\n"
                                  : "  -> FAILED\n");
    return violations == 0;
}

// ---------------------------------------------------------------------------
// Self-tests of the independent checker
// ---------------------------------------------------------------------------

struct CheckerCase {
    std::string name;
    std::vector<Vec3> states;
    Verdict expected;
};

inline bool test_checker_itself() {
    std::cout << "\n[Test A] Independent checker on hand-made visit sequences...\n";
    const Vec3 free_pt = {1.0, 5.0, 0.0};
    const Vec3 r1 = {4.5, 9.0, 0.0};
    const Vec3 r2 = {9.0, 2.5, 0.0};
    const Vec3 r3 = {2.5, 1.0, 0.0};
    const Vec3 r4 = {5.0, 5.0, 0.0};
    const Vec3 r1_edge = {5.0, 9.0, 0.0};   // exactly on the upper x1 edge of R1

    const std::vector<CheckerCase> cases = {
        {"R1 then R3 (valid)", {free_pt, r1, free_pt, r3}, Verdict::SATISFIED},
        {"R2 then R3 (valid)", {free_pt, r2, free_pt, r3}, Verdict::SATISFIED},
        {"R3 first, then R1, then R3 (valid)", {r3, free_pt, r1, free_pt, r3}, Verdict::SATISFIED},
        {"R1, leave, stay away, R3 (memory kept, valid)", {r1, free_pt, free_pt, free_pt, r3}, Verdict::SATISFIED},
        {"R1, R3, then R2 after completion (valid)", {r1, r3, r2}, Verdict::SATISFIED},
        {"R1, R3, then R4 after completion (valid)", {r1, r3, r4}, Verdict::SATISFIED},
        {"R3 only (no target visited)", {free_pt, r3, free_pt}, Verdict::INCOMPLETE},
        {"R1 only (R3 never reached)", {free_pt, r1, free_pt}, Verdict::INCOMPLETE},
        {"R1 then R2 (both targets)", {r1, free_pt, r2, r3}, Verdict::VIOLATED},
        {"R2 then R1 (both targets)", {r2, free_pt, r1, r3}, Verdict::VIOLATED},
        {"R1 then R4", {r1, free_pt, r4, r3}, Verdict::VIOLATED},
        {"starts inside R4", {r4}, Verdict::VIOLATED},
        {"leaves the workspace", {free_pt, {10.0, 5.0, 0.0}}, Verdict::VIOLATED},
        {"negative x1", {{-0.1, 5.0, 0.0}}, Verdict::VIOLATED},
        {"not-a-number state", {{std::nan(""), 5.0, 0.0}}, Verdict::VIOLATED},
        {"state exactly on the R1 edge is not inside R1", {r1_edge, r3}, Verdict::INCOMPLETE},
        {"empty trajectory", {}, Verdict::INCOMPLETE},
    };

    bool all_ok = true;
    for (const CheckerCase& c : cases) {
        const TrajectoryReport rep = check_trajectory(c.states);
        const bool ok = rep.verdict == c.expected;
        std::cout << "  " << (ok ? "ok   " : "FAIL ") << std::left << std::setw(50) << c.name
                  << " -> " << to_string(rep.verdict) << "\n";
        all_ok = all_ok && ok;
    }
    const TrajectoryReport edge = check_trajectory({r1_edge});
    if (!edge.boundary_touch) {
        std::cout << "  FAIL edge state not flagged as boundary_touch\n";
        all_ok = false;
    }
    std::cout << (all_ok ? "  -> PASSED\n" : "  -> FAILED\n");
    return all_ok;
}

// ---------------------------------------------------------------------------
// Cross-check of label() against the reference labels
// ---------------------------------------------------------------------------

inline bool test_label_cross_check() {
    std::cout << "\n[Test B] label() against reference labels (all cells of two heading layers)...\n";
    const std::vector<Region> ref = build_reference_labels();
    const int layer_size = dyn::n_x[0] * dyn::n_x[1];
    long mismatches = 0;
    long checked = 0;
    for (int layer : {0, 17}) {
        for (int i = 1; i <= layer_size; ++i) {
            const int cell = layer * layer_size + i;
            ++checked;
            if (label(cell) != ref[cell]) {
                if (mismatches < 5) {
                    std::cout << "  MISMATCH at cell " << cell << ": label() = " << static_cast<int>(label(cell))
                              << ", reference = " << static_cast<int>(ref[cell]) << "\n";
                }
                ++mismatches;
            }
        }
    }
    // The regions must not depend on the heading.
    for (int cell = 1; cell <= layer_size; ++cell) {
        for (int layer = 1; layer < dyn::n_x[2]; ++layer) {
            if (ref[cell] != ref[cell + layer * layer_size]) ++mismatches;
        }
    }
    std::cout << "  " << checked << " cells compared, " << mismatches << " mismatch(es)\n";
    std::cout << (mismatches == 0 ? "  -> PASSED\n" : "  -> FAILED\n");
    return mismatches == 0;
}

// ---------------------------------------------------------------------------
// Runs
// ---------------------------------------------------------------------------

struct Scenario {
    std::string name;
    Vec3 initial_state;
    int max_steps;
    std::vector<unsigned> seeds;
    StopReason expected;
};

struct RunRecord {
    std::string group;
    std::string name;
    Vec3 initial_state;
    unsigned seed = 0;
    int max_steps = 0;
    StopReason reason = StopReason::STEP_LIMIT;
    StopReason expected = StopReason::STEP_LIMIT;
    int movements = 0;
    int rank_bound = -1;       // rank of the initial combination, -1 if none
    std::string visited;       // R1 / R2 / none
    bool not_both = true;
    bool reached_r3 = false;
    bool r4_avoided = true;
    bool boundary_touch = false;
    bool passed = false;
    std::string notes;
};

inline std::string state_to_string(const Vec3& s) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << s[0] << ", " << s[1] << ", " << s[2] << ")";
    return out.str();
}

// Runs the simulation and judges it with the independent checks.
inline RunRecord evaluate_run(const Controller& controller, const std::vector<Vec2>& val_u,
                              const std::string& group, const std::string& name,
                              const Vec3& initial_state, int max_steps, unsigned seed,
                              StopReason expected, int expected_bound = -1) {
    RunRecord rec;
    rec.group = group;
    rec.name = name;
    rec.initial_state = initial_state;
    rec.seed = seed;
    rec.max_steps = max_steps;
    rec.expected = expected;

    const SimulationResult res = simulate(controller, val_u, initial_state, max_steps, seed);
    const TrajectoryReport traj = check_trajectory(res.states);
    std::vector<std::string> issues = check_run_consistency(res, val_u, controller, traj);

    rec.reason = res.reason;
    rec.movements = static_cast<int>(res.velocities.size());
    rec.visited = traj.visited_r1 && traj.visited_r2 ? "R1+R2" : traj.visited_r1 ? "R1" : traj.visited_r2 ? "R2" : "none";
    rec.not_both = !traj.visited_both;
    rec.reached_r3 = traj.reached_r3;
    rec.r4_avoided = !traj.hit_r4;
    rec.boundary_touch = traj.boundary_touch;

    const int first_cell = res.states.empty() ? 0 : numerical_state_to_cell(res.states[0]);
    if (first_cell > 0 && !res.missions.empty()) {
        rec.rank_bound = controller.rank[first_cell][static_cast<int>(res.missions[0])];
    }

    // The reported reason must agree with the independent verdict.
    bool agrees = false;
    switch (res.reason) {
        case StopReason::SUCCESS:     agrees = traj.verdict == Verdict::SATISFIED; break;
        case StopReason::FAILURE:     agrees = traj.verdict == Verdict::VIOLATED; break;
        case StopReason::UNCERTIFIED: agrees = traj.verdict == Verdict::INCOMPLETE && rec.movements == 0 && res.states.size() == 1; break;
        case StopReason::STEP_LIMIT:  agrees = traj.verdict == Verdict::INCOMPLETE; break;
    }
    if (!agrees) {
        issues.push_back(std::string("reported ") + to_string(res.reason) + " but independent verdict is " +
                         to_string(traj.verdict) + " (" + traj.summary + ")");
    }
    if (res.reason != expected) {
        issues.push_back(std::string("expected ") + to_string(expected) + " but got " + to_string(res.reason));
    }
    // Guarantee: SUCCESS within rank movements.
    if (res.reason == StopReason::SUCCESS && rec.rank_bound >= 0 && rec.movements > rec.rank_bound) {
        issues.push_back("used " + std::to_string(rec.movements) + " movements, rank bound is " + std::to_string(rec.rank_bound));
    }
    if (expected_bound >= 0 && rec.rank_bound != expected_bound) {
        issues.push_back("initial rank " + std::to_string(rec.rank_bound) + " differs from the oracle's " + std::to_string(expected_bound));
    }

    rec.passed = issues.empty();
    if (!issues.empty()) {
        for (std::size_t i = 0; i < issues.size(); ++i) rec.notes += (i ? "; " : "") + issues[i];
    } else {
        rec.notes = traj.summary;
        if (traj.boundary_touch) rec.notes += " [state on a region edge]";
    }
    return rec;
}

// Expected outcome of a random initial state, derived from the table and the
// reference labels only (no controller_step, no label()).
inline StopReason oracle_expected(const Controller& controller, const std::vector<Region>& ref,
                                  const Vec3& state, int& rank_bound) {
    rank_bound = -1;
    const int cell = reference_cell(state);
    if (cell == 0) return StopReason::FAILURE;
    const Region reg = ref[cell];
    if (reg == Region::R4) return StopReason::FAILURE;
    int m = 0;
    if (reg == Region::R1) m = 1;
    if (reg == Region::R2) m = 2;
    const int rank = controller.rank[cell][m];
    if (rank < 0) return StopReason::UNCERTIFIED;
    rank_bound = rank;
    return StopReason::SUCCESS;
}

inline void write_results(const std::vector<RunRecord>& records, const std::string& base_name,
                          int sweep_total, int sweep_pass, int sweep_cert, int sweep_uncert, int sweep_fail) {
    std::ofstream md(base_name + ".md");
    std::ofstream csv(base_name + ".csv");
    if (!md || !csv) {
        std::cout << "Warning: could not write " << base_name << ".md/.csv\n";
        return;
    }
    const std::string header_cols =
        "| # | Scenario | Initial state (x1, x2, x3) | Seed | Max steps | Stop reason | Expected | Movements | Rank bound | Visited | Not both | R3 reached | R4 avoided | Independent check |\n"
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
    md << "# Integration results\n\n"
       << "Generated by `IntegrationTests/test_integration.cpp`. \"Independent check\" re-reads the recorded states with\n"
       << "`IntegrationTests/trajectory_check.hpp` (no `label()`, no `update_mission()`).\n\n"
       << "## Named scenarios\n\n" << header_cols;
    csv << "id,group,scenario,x1,x2,x3,seed,max_steps,stop_reason,expected,movements,rank_bound,visited,not_both,reached_r3,r4_avoided,boundary_touch,passed,notes\n";

    int id = 0;
    for (const RunRecord& r : records) {
        ++id;
        csv << id << "," << r.group << ",\"" << r.name << "\"," << std::fixed << std::setprecision(6)
            << r.initial_state[0] << "," << r.initial_state[1] << "," << r.initial_state[2] << ","
            << r.seed << "," << r.max_steps << "," << to_string(r.reason) << "," << to_string(r.expected) << ","
            << r.movements << "," << r.rank_bound << "," << r.visited << ","
            << (r.not_both ? "yes" : "NO") << "," << (r.reached_r3 ? "yes" : "no") << ","
            << (r.r4_avoided ? "yes" : "NO") << "," << (r.boundary_touch ? "yes" : "no") << ","
            << (r.passed ? "PASS" : "FAIL") << ",\"" << r.notes << "\"\n";
        if (r.group != "named") continue;
        md << "| " << id << " | " << r.name << " | " << state_to_string(r.initial_state) << " | " << r.seed
           << " | " << r.max_steps << " | " << to_string(r.reason) << " | " << to_string(r.expected)
           << " | " << r.movements << " | " << (r.rank_bound >= 0 ? std::to_string(r.rank_bound) : "-")
           << " | " << r.visited << " | " << (r.not_both ? "yes" : "NO") << " | " << (r.reached_r3 ? "yes" : "no")
           << " | " << (r.r4_avoided ? "yes" : "NO") << " | " << (r.passed ? "PASS" : "FAIL") << " |\n";
    }
    md << "\n## Random sweep\n\n"
       << "| Runs | Certified starts (expected SUCCESS within the rank bound) | Uncertified starts | Failing starts (R4 / outside) | Passed |\n"
       << "|---|---|---|---|---|\n"
       << "| " << sweep_total << " | " << sweep_cert << " | " << sweep_uncert << " | " << sweep_fail
       << " | " << sweep_pass << " / " << sweep_total << " |\n\n"
       << "Every run of the sweep is listed in `" << base_name << ".csv`.\n";
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

inline bool run_tests(const Controller& controller, const std::vector<Vec2>& val_u) {
    bool all_ok = true;
    all_ok = test_checker_itself() && all_ok;
    all_ok = test_label_cross_check() && all_ok;

    const std::vector<Region> ref = build_reference_labels();
    std::vector<RunRecord> records;

    // --- Named scenarios ---------------------------------------------------
    std::cout << "\n[Test C] Named scenarios (initial positions, headings, boundaries, uncertified)...\n";
    const std::vector<unsigned> seeds = {42, 101, 2024};
    const std::vector<Scenario> scenarios = {
        {"Bottom-left start, heading 0",            {1.05, 0.25, 0.0},       60, seeds, StopReason::SUCCESS},
        {"Top-left start, heading 0",               {1.55, 8.55, 0.0},       60, seeds, StopReason::SUCCESS},
        {"Bottom-right start, heading 1.57",        {8.05, 1.05, 1.57},      60, seeds, StopReason::SUCCESS},
        {"Heading = +pi (wraps to -pi)",            {1.05, 0.25, PI},        60, seeds, StopReason::SUCCESS},
        {"Heading = 3*pi (several turns)",          {1.05, 0.25, 3.0 * PI},  60, seeds, StopReason::SUCCESS},
        {"Heading = -pi (lower bound)",             {1.05, 0.25, -PI},       60, seeds, StopReason::SUCCESS},
        {"Starts inside R1",                        {4.55, 9.05, 0.0},       60, seeds, StopReason::SUCCESS},
        {"Starts inside R2",                        {9.05, 2.55, 0.0},       60, seeds, StopReason::SUCCESS},
        {"Starts inside R3 (does not complete)",    {2.55, 1.05, 0.0},       60, seeds, StopReason::SUCCESS},
        {"Starts on a cell edge (x1 = 5.0)",        {5.0, 0.5, 0.0},         60, seeds, StopReason::SUCCESS},
        {"Short horizon (8 movements)",             {1.05, 0.25, 0.0},        8, {42},  StopReason::STEP_LIMIT},
        {"max_steps = 0",                           {1.05, 0.25, 0.0},        0, {42},  StopReason::STEP_LIMIT},
        {"Uncertified: workspace corner (0, 0)",    {0.0, 0.0, 0.0},         60, {42},  StopReason::UNCERTIFIED},
        {"Uncertified: corner cell, heading -pi",   {0.05, 0.05, -PI},       60, {42},  StopReason::UNCERTIFIED},
        {"Uncertified: right wall",                 {9.5, 5.0, 0.0},         60, {42},  StopReason::UNCERTIFIED},
        {"Uncertified: bottom-right corner",        {9.95, 0.05, 0.0},       60, {42},  StopReason::UNCERTIFIED},
        {"Starts inside R4",                        {5.0, 5.0, 0.0},         60, {42},  StopReason::FAILURE},
        {"Outside: x1 = 10.0 (upper bound)",        {10.0, 5.0, 0.0},        60, {42},  StopReason::FAILURE},
        {"Outside: negative x1",                    {-0.1, 5.0, 0.0},        60, {42},  StopReason::FAILURE},
        {"Outside: x2 above workspace",             {5.0, 10.5, 0.0},        60, {42},  StopReason::FAILURE},
        {"Invalid: NaN position",                   {std::nan(""), 5.0, 0.0}, 60, {42}, StopReason::FAILURE},
        {"Invalid: infinite heading",               {5.0, 0.5, INFINITY},    60, {42},  StopReason::FAILURE},
    };
    for (const Scenario& sc : scenarios) {
        for (unsigned seed : sc.seeds) {
            records.push_back(evaluate_run(controller, val_u, "named", sc.name, sc.initial_state,
                                           sc.max_steps, seed, sc.expected));
        }
    }

    // --- Guarantee: the rank is a real bound ---------------------------------
    // From a certified start, max_steps = rank must still end in SUCCESS.
    {
        const Vec3 x0 = {1.05, 0.25, 0.0};
        const int cell = numerical_state_to_cell(x0);
        const int bound = controller.rank[cell][0];
        for (unsigned seed : seeds) {
            records.push_back(evaluate_run(controller, val_u, "named",
                                           "max_steps = initial rank (" + std::to_string(bound) + ")",
                                           x0, bound, seed, StopReason::SUCCESS));
        }
    }

    // --- Reproducibility -----------------------------------------------------
    {
        const Vec3 x0 = {1.55, 8.55, 0.0};
        const SimulationResult a = simulate(controller, val_u, x0, 60, 42);
        const SimulationResult b = simulate(controller, val_u, x0, 60, 42);
        const SimulationResult c = simulate(controller, val_u, x0, 60, 43);
        const bool same = a.states == b.states && a.velocities == b.velocities && a.disturbances == b.disturbances &&
                          a.reason == b.reason;
        const bool different = a.disturbances != c.disturbances;
        std::cout << "  seed reproducibility: same seed identical = " << (same ? "yes" : "NO")
                  << ", other seed differs = " << (different ? "yes" : "NO") << "\n";
        all_ok = all_ok && same && different;
    }

    int named_pass = 0;
    int named_total = 0;
    for (const RunRecord& r : records) {
        ++named_total;
        named_pass += r.passed ? 1 : 0;
        if (!r.passed) {
            std::cout << "  FAIL: " << r.name << " seed " << r.seed << ": " << r.notes << "\n";
        }
    }
    std::cout << "  " << named_pass << " / " << named_total << " named runs passed\n";
    all_ok = all_ok && named_pass == named_total;

    // --- Random sweep --------------------------------------------------------
    std::cout << "\n[Test D] Random sweep (random states, expected outcome derived from the table)...\n";
    const int sweep_size = 400;
    std::mt19937 rng(2026);   // fixed: the sweep is repeatable
    std::uniform_real_distribution<double> dist_x(0.0, 10.0);
    std::uniform_real_distribution<double> dist_h(-PI, PI);
    int sweep_pass = 0, sweep_cert = 0, sweep_uncert = 0, sweep_fail = 0;
    for (int i = 0; i < sweep_size; ++i) {
        const Vec3 x0 = {dist_x(rng), dist_x(rng), dist_h(rng)};
        int bound = -1;
        const StopReason expected = oracle_expected(controller, ref, x0, bound);
        const int max_steps = expected == StopReason::SUCCESS ? bound : 60;  // the rank must be enough
        RunRecord rec = evaluate_run(controller, val_u, "sweep", "random #" + std::to_string(i), x0,
                                     max_steps, 5000u + static_cast<unsigned>(i), expected, bound);
        if (expected == StopReason::SUCCESS) ++sweep_cert;
        else if (expected == StopReason::UNCERTIFIED) ++sweep_uncert;
        else ++sweep_fail;
        sweep_pass += rec.passed ? 1 : 0;
        if (!rec.passed) std::cout << "  FAIL: " << rec.name << " " << state_to_string(x0) << ": " << rec.notes << "\n";
        records.push_back(rec);
    }
    std::cout << "  " << sweep_size << " runs: " << sweep_cert << " certified, " << sweep_uncert
              << " uncertified, " << sweep_fail << " failing starts; " << sweep_pass << " passed\n";
    all_ok = all_ok && sweep_pass == sweep_size;

    // --- Console results table + files ---------------------------------------
    std::cout << "\n======================================================================\n"
              << "                       INTEGRATION RESULTS TABLE                       \n"
              << "======================================================================\n";
    std::cout << std::left << std::setw(42) << "Scenario" << std::setw(26) << "Initial state"
              << std::setw(6) << "Seed" << std::setw(13) << "Stop reason" << std::setw(6) << "Moves"
              << std::setw(7) << "Bound" << std::setw(8) << "Visited" << std::setw(6) << "Both"
              << std::setw(5) << "R3" << std::setw(5) << "R4ok" << "Check\n";
    std::cout << std::string(132, '-') << "\n";
    for (const RunRecord& r : records) {
        if (r.group != "named") continue;
        std::cout << std::left << std::setw(42) << r.name.substr(0, 41) << std::setw(26) << state_to_string(r.initial_state)
                  << std::setw(6) << r.seed << std::setw(13) << to_string(r.reason) << std::setw(6) << r.movements
                  << std::setw(7) << (r.rank_bound >= 0 ? std::to_string(r.rank_bound) : "-")
                  << std::setw(8) << r.visited << std::setw(6) << (r.not_both ? "ok" : "BOTH")
                  << std::setw(5) << (r.reached_r3 ? "yes" : "no") << std::setw(5) << (r.r4_avoided ? "yes" : "NO")
                  << (r.passed ? "PASS" : "FAIL") << "\n";
    }
    std::cout << std::string(132, '-') << "\n";

    write_results(records, "integration_results", sweep_size, sweep_pass, sweep_cert, sweep_uncert, sweep_fail);
    std::cout << "Results written to integration_results.md and integration_results.csv\n";

    std::cout << "\n" << (all_ok ? "ALL INTEGRATION TESTS PASSED" : "SOME INTEGRATION TESTS FAILED") << "\n";
    return all_ok;
}

}  // namespace integ
