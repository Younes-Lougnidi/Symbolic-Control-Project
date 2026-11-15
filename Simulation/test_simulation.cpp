#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <cmath>
#include <vector>

#include "simulation.hpp"
#include "controller_io.hpp"

// ============================================================================
// Scripted Test Stubs (strictly for unit tests, never presented as project runs)
// ============================================================================

// Stub 1: Simulates a forced failure after a specified number of movements
struct FailureStub {
    int fail_at_movement;
    int current_movements = 0;

    ControlStep operator()(const Controller&, const std::vector<Vec2>& val_u,
                           const Vec3&, Mission previous) {
        if (current_movements >= fail_at_movement) {
            return {{0.0, 0.0}, Mission::FAILURE, Status::FAILURE};
        }
        current_movements++;
        return {val_u.empty() ? Vec2{0.5, 0.0} : val_u[0], previous, Status::RUNNING};
    }
};

// Stub 2: Simulates success occurring exactly at a given movement count
struct ExactStepSuccessStub {
    int target_movements;
    int current_movements = 0;

    ControlStep operator()(const Controller&, const std::vector<Vec2>& val_u,
                           const Vec3&, Mission) {
        if (current_movements >= target_movements) {
            return {{0.0, 0.0}, Mission::SUCCESS, Status::SUCCESS};
        }
        current_movements++;
        return {val_u.empty() ? Vec2{0.5, 0.0} : val_u[0], Mission::VISITED_R1, Status::RUNNING};
    }
};


// ============================================================================
// Unit Tests
// ============================================================================

void test_seed_reproducibility(const Controller& controller, const std::vector<Vec2>& val_u, const Vec3& x0) {
    std::cout << "[Test 1] Seed Reproducibility...\n";
    unsigned seed_a = 42;
    unsigned seed_b = 999;

    SimulationResult res_a1 = simulate(controller, val_u, x0, 15, seed_a);
    SimulationResult res_a2 = simulate(controller, val_u, x0, 15, seed_a);
    SimulationResult res_b  = simulate(controller, val_u, x0, 15, seed_b);

    // Identical seeds must produce identical results
    assert(res_a1.reason == res_a2.reason);
    assert(res_a1.states.size() == res_a2.states.size());
    assert(res_a1.velocities.size() == res_a2.velocities.size());
    assert(res_a1.disturbances.size() == res_a2.disturbances.size());

    for (size_t i = 0; i < res_a1.states.size(); ++i) {
        for (int d = 0; d < 3; ++d) {
            assert(res_a1.states[i][d] == res_a2.states[i][d]);
        }
    }
    for (size_t i = 0; i < res_a1.disturbances.size(); ++i) {
        for (int d = 0; d < 3; ++d) {
            assert(res_a1.disturbances[i][d] == res_a2.disturbances[i][d]);
        }
    }

    // Different seeds must produce different disturbances
    if (!res_a1.disturbances.empty() && !res_b.disturbances.empty()) {
        bool any_different = false;
        for (int d = 0; d < 3; ++d) {
            if (res_a1.disturbances[0][d] != res_b.disturbances[0][d]) {
                any_different = true;
                break;
            }
        }
        assert(any_different);
    }
    std::cout << "  -> PASSED: Same seed produces bit-exact match; different seed diverges.\n";
}

void test_size_invariants(const Controller& controller, const std::vector<Vec2>& val_u, const Vec3& x0) {
    std::cout << "[Test 2] Size Invariants (states = missions = k+1, vels = dists = k)...\n";
    SimulationResult res = simulate(controller, val_u, x0, 25, 1234);

    size_t k = res.velocities.size();
    assert(res.states.size() == k + 1);
    assert(res.missions.size() == k + 1);
    assert(res.disturbances.size() == k);

    std::cout << "  -> PASSED: For k = " << k << " movements, exactly "
              << res.states.size() << " states/missions and " << k << " vels/dists stored.\n";
}

void test_step_limit_boundary(const Controller& controller, const std::vector<Vec2>& val_u, const Vec3& x0) {
    std::cout << "[Test 3] STEP_LIMIT Boundary Conditions...\n";

    // 1. max_steps = 0: Must record exactly 1 state, 0 movements, and return STEP_LIMIT
    SimulationResult res_zero = simulate(controller, val_u, x0, 0, 42);
    assert(res_zero.reason == StopReason::STEP_LIMIT);
    assert(res_zero.states.size() == 1);
    assert(res_zero.missions.size() == 1);
    assert(res_zero.velocities.empty());
    assert(res_zero.disturbances.empty());
    std::cout << "  -> max_steps = 0: PASSED (1 state recorded, 0 movements, reason = STEP_LIMIT).\n";

    // 2. Success on the exact last permitted movement must count as SUCCESS, not STEP_LIMIT
    int limit = 5;
    ExactStepSuccessStub stub{limit, 0};
    SimulationResult res_last = simulate_with_step(controller, val_u, x0, limit, 42,
        [&stub](const Controller& c, const std::vector<Vec2>& u, const Vec3& s, Mission m) {
            return stub(c, u, s, m);
        }
    );
    assert(res_last.reason == StopReason::SUCCESS);
    assert(res_last.velocities.size() == static_cast<size_t>(limit));
    assert(res_last.states.size() == static_cast<size_t>(limit + 1));
    std::cout << "  -> Success on final move: PASSED (evaluated as SUCCESS, not STEP_LIMIT).\n";
}

void test_uncertified_initial_state(const Controller& controller, const std::vector<Vec2>& val_u) {
    std::cout << "[Test 4] Uncertified Initial State...\n";
    // Choose an initial state in R4 (obstacle) or known uncertified
    Vec3 uncertified_x0 = {5.0, 5.0, 0.0}; // Deep inside R4 obstacle
    SimulationResult res = simulate(controller, val_u, uncertified_x0, 20, 42);

    // Entering R4 immediately triggers Mission::FAILURE and Status::FAILURE
    assert(res.states.size() == 1);
    assert(res.missions.size() == 1);
    assert(res.velocities.empty());
    assert(res.disturbances.empty());
    assert(res.reason == StopReason::FAILURE);

    // Also test an actual uncertified state (rank -1)
    Vec3 x_uncert = {0.05, 0.05, 0.0};
    ControlStep step0 = controller_step(controller, val_u, x_uncert, Mission::START);
    if (step0.status == Status::UNCERTIFIED) {
        SimulationResult res_u = simulate(controller, val_u, x_uncert, 20, 42);
        assert(res_u.reason == StopReason::UNCERTIFIED);
        assert(res_u.states.size() == 1);
        assert(res_u.velocities.empty());
    }
    std::cout << "  -> PASSED: Non-running initial state terminates immediately with 1 observation and 0 movements.\n";
}

void test_disturbance_bounds(const Controller& controller, const std::vector<Vec2>& val_u, const Vec3& x0) {
    std::cout << "[Test 5] Disturbance Bounds Verification...\n";
    SimulationResult res = simulate(controller, val_u, x0, 50, 777);

    for (size_t i = 0; i < res.disturbances.size(); ++i) {
        for (int d = 0; d < 3; ++d) {
            double w = res.disturbances[i][d];
            assert(w >= dyn::bound_w_lo[d] - 1e-12);
            assert(w <= dyn::bound_w_hi[d] + 1e-12);
        }
    }
    std::cout << "  -> PASSED: All " << res.disturbances.size() * 3
              << " disturbance coordinates strictly inside [-0.05, 0.05].\n";
}

void test_failure_stub(const Controller& controller, const std::vector<Vec2>& val_u, const Vec3& x0) {
    std::cout << "[Test 6] Scripted Failure Stub (Test-only verification)...\n";
    FailureStub stub{3, 0};
    SimulationResult res = simulate_with_step(controller, val_u, x0, 10, 42,
        [&stub](const Controller& c, const std::vector<Vec2>& u, const Vec3& s, Mission m) {
            return stub(c, u, s, m);
        }
    );
    assert(res.reason == StopReason::FAILURE);
    assert(res.velocities.size() == 3);
    assert(res.states.size() == 4);
    std::cout << "  -> PASSED: Forced failure terminated cleanly with StopReason::FAILURE.\n";
}


// ============================================================================
// Multi-scenario Real Controller Runner
// ============================================================================

void run_real_controller_scenarios(const Controller& controller, const std::vector<Vec2>& val_u) {
    std::cout << "\n======================================================================\n";
    std::cout << "           REAL CONTROLLER BENCHMARK SCENARIOS (FOR REPORT)           \n";
    std::cout << "======================================================================\n";

    struct Scenario {
        std::string name;
        Vec3 initial_state;
        int max_steps;
        std::vector<unsigned> seeds;
    };

    std::vector<Scenario> scenarios = {
        {"Scenario 1 (South-West -> R2 -> R3)", {1.05, 0.25, 0.0}, 60, {42, 101, 2024}},
        {"Scenario 2 (North-West -> R1 -> R3)", {1.55, 8.55, 0.0}, 60, {42, 101, 2024}},
        {"Scenario 3 (South-East -> R2 -> R3)", {8.05, 1.05, 1.57}, 60, {42, 101, 2024}},
        {"Scenario 4 (Short Horizon Limit)",    {1.05, 0.25, 0.0}, 8,  {42}}
    };

    std::cout << std::left
              << std::setw(38) << "Scenario / Initial State"
              << std::setw(10) << "Seed"
              << std::setw(16) << "Stop Reason"
              << std::setw(12) << "Movements"
              << std::setw(22) << "Final State (x1, x2)"
              << "\n";
    std::cout << std::string(98, '-') << "\n";

    int run_id = 0;
    for (const auto& sc : scenarios) {
        for (unsigned s : sc.seeds) {
            run_id++;
            SimulationResult res = simulate(controller, val_u, sc.initial_state, sc.max_steps, s);

            std::string csv_filename = "sim_run_" + std::to_string(run_id) + "_seed_" + std::to_string(s) + ".csv";
            export_simulation_to_csv(res, csv_filename);

            std::ostringstream final_pos;
            if (!res.states.empty()) {
                final_pos << std::fixed << std::setprecision(2)
                          << "(" << res.states.back()[0] << ", " << res.states.back()[1] << ")";
            }

            std::ostringstream init_str;
            init_str << sc.name;

            std::cout << std::left
                      << std::setw(38) << init_str.str()
                      << std::setw(10) << s
                      << std::setw(16) << to_string(res.reason)
                      << std::setw(12) << res.velocities.size()
                      << std::setw(22) << final_pos.str()
                      << "\n";
        }
    }
    std::cout << std::string(98, '-') << "\n";
    std::cout << "All scenario CSVs exported successfully for plotting.\n";
}


// ============================================================================
// Main Runner
// ============================================================================

int main() {
    std::cout << "======================================================================\n";
    std::cout << "      UNICYCLE SYMBOLIC CONTROL - SIMULATION & VERIFICATION SUITE     \n";
    std::cout << "======================================================================\n";

    // 1. Load abstraction
    Transitions g;
    std::vector<Vec2> val_u;
    std::string abs_path = "symb_abs.bin";
    if (!sim_io::load_abstraction(abs_path, g, val_u)) {
        abs_path = "../symb_abs.bin";
        if (!sim_io::load_abstraction(abs_path, g, val_u)) {
            std::cerr << "Error: symb_abs.bin not found! Please compile and run robotControl/sym_abs.cpp first.\n";
            return 1;
        }
    }
    std::cout << "Abstraction loaded: " << g.size() << " transitions, " << val_u.size() << " commands.\n";

    // 2. Synthesize or load controller ONCE
    Controller controller = sim_io::get_or_synthesize_controller(g, "controller_cache.bin");

    // 3. Certified test initial state
    Vec3 x0 = {1.05, 0.25, 0.0};

    // 4. Run all unit tests
    std::cout << "\n--- RUNNING UNIT TESTS ---\n";
    test_seed_reproducibility(controller, val_u, x0);
    test_size_invariants(controller, val_u, x0);
    test_step_limit_boundary(controller, val_u, x0);
    test_uncertified_initial_state(controller, val_u);
    test_disturbance_bounds(controller, val_u, x0);
    test_failure_stub(controller, val_u, x0);
    std::cout << "--- ALL UNIT TESTS PASSED SUCCESSFULLY! ---\n";

    // 5. Run real controller scenarios
    run_real_controller_scenarios(controller, val_u);

    return 0;
}
