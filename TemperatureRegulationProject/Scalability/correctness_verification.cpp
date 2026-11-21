#include "local_safety.hpp"
#include "../IntegrationTests/run_tests.hpp"
#include "../IntegrationTests/large_ring_replay.hpp"
#include <charconv>
#include <string_view>

using local_safety::SafetyTable;

// Rayane's verifier expects global command IDs. Expand our per-room choices
// only on the small test grids, in the same order as the reference controller.
simulation::Controller explicit_test_controller(const SafetyTable& table,
                                                const abstraction::Abstraction& model) {
    simulation::Controller result(model.num_states);
    for (abstraction::Id cell = 0; cell < model.num_states; ++cell) {
        const auto bins = abstraction::decode_cell(cell, model.N);
        const auto choices = local_safety::safe_commands_for_cell(table, bins);
        // For example ID 21 has digits (0,1,2): off in room 0, medium
        // in room 1 and high in room 2. Each room must allow its own digit.
        for (abstraction::Id command = 0; command < model.num_commands; ++command) {
            auto digits = command;
            bool allowed = true;
            for (const auto& row : choices) {
                allowed &= row[digits % dyn::nu];
                digits /= dyn::nu;
            }
            if (allowed) result[cell].push_back(command);
        }
    }
    if (result != synthesize_safety(model) || !integ::verify_controller(model, result))
        throw std::runtime_error("Optimized controller failed the reference comparison or safety audit");
    std::cout << "All " << model.num_states * model.num_commands
              << " cell-command decisions agree for N=" << model.N << '\n';
    return result;
}

// Give our controller to Rayane's existing checks, including Amine's saved traces.
bool run_small_suite(const SafetyTable& table) {
    const auto model3 = safety_abstraction::build_abstraction(3);
    const auto ctrl3 = explicit_test_controller(table, model3);
    const auto ctrl4 = explicit_test_controller(table, safety_abstraction::build_abstraction(4));
    std::vector<integ::SimTestRecord> simulations;
    std::vector<integ::CsvTraceAudit> traces;
    int total3 = 0, pass3 = 0, total4 = 0, pass4 = 0;
    bool ok = integ::test_trajectory_checker_self_tests(ctrl3);
    ok = integ::test_interfaces_and_overflow() && ok;
    ok = integ::test_abstraction_audit(model3, ctrl3) && ok;
    ok = integ::test_policy_verification_and_corruption(model3, ctrl3) && ok;
    ok = integ::test_simulations(ctrl3, &ctrl4, simulations, total3, pass3, total4, pass4) && ok;
    ok = integ::test_amine_csv_traces(ctrl3, "../simulation", traces) && ok;
    return ok;
}

// From Scalability: g++ -std=c++17 -O2 correctness_verification.cpp -o verify
// ./verify                     Exhaustive comparison and Rayane's six suites
// ./verify --replay 65538      Two physical updates across a block boundary
int main(int argc, char** argv) {
    try {
        const auto table = local_safety::build_safety_table();
        if (argc == 1) return run_small_suite(table) ? 0 : 1;
        if (argc != 3 || std::string_view(argv[1]) != "--replay")
            throw std::invalid_argument("Usage: verify [--replay ROOM_COUNT]");
        const std::string_view arg = argv[2];
        std::uint64_t rooms = 0;
        const auto end = arg.data() + arg.size();
        const auto parsed = std::from_chars(arg.data(), end, rooms);
        if (parsed.ec != std::errc{} || parsed.ptr != end || rooms < 3)
            throw std::invalid_argument("Expected a room count of at least three");
        large_ring_replay::run(table, rooms);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
