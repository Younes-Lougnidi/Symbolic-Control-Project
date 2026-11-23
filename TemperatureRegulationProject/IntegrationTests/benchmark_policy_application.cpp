#include "../Scalability/block_safety.hpp"
#include "measurements.hpp"
#include <charconv>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string_view>

// Build on Windows: g++ -std=c++17 -O2 benchmark_policy_application.cpp -lpsapi -o benchmark
// ./benchmark --build-only 1000000000000    Measure abstraction and synthesis.
// ./benchmark 1000000000000                 Also apply the controller to a cell.
int generate_bin(std::uint64_t& seed) {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<int>((seed >> 32) % dyn::cpr);
}

int main(int argc, char** argv) {
    const bool build_only = argc == 3 && std::string_view(argv[1]) == "--build-only";
    if (argc != 2 && !build_only) {
        std::cerr << "Usage: benchmark [--build-only] ROOM_COUNT\n";
        return 1;
    }
    const std::string_view argument = argv[argc - 1];
    const char* end = argument.data() + argument.size();
    std::uint64_t rooms = 0;
    const auto parsed = std::from_chars(argument.data(), end, rooms);
    if (parsed.ec != std::errc{} || parsed.ptr != end || rooms < 3) {
        std::cerr << "Expected an integer room count of at least 3\n";
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto model = local_safety::build_local_abstraction();
    const auto abstracted = Clock::now();
    const auto result = local_safety::synthesize_local_safety(model);
    const auto ready = Clock::now();
    const auto memory = integ::measure_peak_rss_bytes();
    const auto& table = result.controller;

    // Read the complete controller even in build-only mode. Counting local
    // flags is outside the timers and does not enumerate global commands.
    std::size_t allowed_levels = 0;
    for (const auto& left : table.allowed)
        for (const auto& middle : left)
            for (const auto& choices : middle)
                allowed_levels += std::count(choices.begin(), choices.end(), true);

    std::cout << "Rooms: " << rooms
              << "\nLocal abstraction T_abs (s): " << std::chrono::duration<double>(abstracted - start).count()
              << "\nWinning domain and maximal controller T_ctrl (s): " << std::chrono::duration<double>(ready - abstracted).count()
              << "\nTable construction and certification (s): " << std::chrono::duration<double>(ready - start).count()
              << "\nPeak process RSS through synthesis (bytes): " << (memory.available ? std::to_string(memory.bytes) : "missing")
              << "\nAllowed local heater choices: " << allowed_levels << '\n';
    if (result.full_grid_winning)
        std::cout << "Winning domain: all " << dyn::cpr << '^' << rooms << " cells (100%)\n";
    if (build_only) return 0;

    // Use one reproducible cell and count one allowed heater setting per room.
    std::uint64_t seed = 42;
    std::uint64_t count = 0, checksum = 0;
    auto next_bin = [&]() { return generate_bin(seed); };
    auto count_choices = [&](std::uint64_t, const std::vector<local_safety::HeaterChoices>& choices) {
        for (const auto& row : choices) {
            checksum += std::find(row.begin(), row.end(), true) - row.begin();
        }
        count += choices.size();
    };
    const auto application_start = Clock::now();
    local_safety::safe_commands_in_blocks(table, rooms, next_bin, count_choices);
    const auto done = Clock::now();
    std::cout << "One step including bin generation (s): "
              << std::chrono::duration<double>(done - application_start).count()
              << "\nRoom commands issued: " << count << "\nChecksum: " << checksum << '\n';
    return count == rooms ? 0 : 1;
}
