#include "../Scalability/block_safety.hpp"
#include <charconv>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string_view>

// Time a single optimized policy application (the trillion-room benchmark).
// Build: g++ -std=c++17 -O2 benchmark_policy_application.cpp -o benchmark
// Example: ./benchmark 1000000000000
int generate_bin(std::uint64_t& seed) {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<int>((seed >> 32) % dyn::cpr);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: benchmark ROOM_COUNT\n";
        return 1;
    }
    const std::string_view argument = argv[1];
    const char* end = argument.data() + argument.size();
    std::uint64_t rooms = 0;
    const auto parsed = std::from_chars(argument.data(), end, rooms);
    if (parsed.ec != std::errc{} || parsed.ptr != end || rooms < 3) {
        std::cerr << "Expected an integer room count of at least 3\n";
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto table = local_safety::build_safety_table();
    const auto ready = Clock::now();
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
    local_safety::safe_commands_in_blocks(table, rooms, next_bin, count_choices);
    const auto done = Clock::now();
    std::cout << "Table construction and certification (s): "
              << std::chrono::duration<double>(ready - start).count()
              << "\nOne step including bin generation (s): "
              << std::chrono::duration<double>(done - ready).count()
              << "\nRoom commands issued: " << count << "\nChecksum: " << checksum << '\n';
    return count == rooms ? 0 : 1;
}
