#pragma once

// Timing, peak-memory, and environment measurements (Rayane Khaldi).
//
// Abstraction and synthesis are timed separately with steady_clock.
// Peak memory uses the OS high-water mark RSS and is reported as "missing"
// (never 0) if the OS query fails or is unsupported.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <sys/resource.h>
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

#include "../Safety_Synthesis/SafetyAbstraction/checked_abstraction.hpp"
#include "../Safety_Synthesis/safety_synthesis.hpp"
#include "verification.hpp"

namespace integ {

using SteadyClock = std::chrono::steady_clock;

// Wrapper so unavailable memory readings stay explicitly missing instead of 0.
struct MemorySample {
    bool available = false;
    std::uint64_t bytes = 0;
};

// Returns process peak resident set size in bytes, or {false, 0} if unavailable.
inline MemorySample measure_peak_rss_bytes() {
#if defined(__APPLE__)
    struct rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) == 0 && ru.ru_maxrss > 0) {
        // On macOS, ru_maxrss is reported directly in bytes.
        return {true, static_cast<std::uint64_t>(ru.ru_maxrss)};
    }
    return {false, 0};
#elif defined(__linux__)
    struct rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) == 0 && ru.ru_maxrss > 0) {
        // On Linux, ru_maxrss is reported in kilobytes (KiB).
        return {true, static_cast<std::uint64_t>(ru.ru_maxrss) * 1024ULL};
    }
    return {false, 0};
#elif defined(_WIN32)
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)) &&
        pmc.PeakWorkingSetSize > 0) {
        return {true, static_cast<std::uint64_t>(pmc.PeakWorkingSetSize)};
    }
    return {false, 0};
#else
    return {false, 0};
#endif
}

inline std::string format_memory_mb(const MemorySample& sample) {
    if (!sample.available) return "missing";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << (static_cast<double>(sample.bytes) / (1024.0 * 1024.0)) << " MiB";
    return oss.str();
}

inline std::string format_seconds(double sec) {
    std::ostringstream oss;
    if (sec < 1.0) {
        oss << std::fixed << std::setprecision(3) << (sec * 1000.0) << " ms";
    } else {
        oss << std::fixed << std::setprecision(4) << sec << " s";
    }
    return oss.str();
}

inline std::string compiler_info() {
#if defined(__clang__)
    return std::string("Clang ") + __clang_version__;
#elif defined(__GNUC__)
    return std::string("GCC ") + __VERSION__;
#elif defined(_MSC_VER)
    return "MSVC " + std::to_string(_MSC_VER);
#else
    return "Unknown compiler";
#endif
}

inline std::string build_flags_info() {
#ifdef COMP_FLAGS
    return std::string(COMP_FLAGS);
#else
    std::string flags = "C++" + std::to_string(__cplusplus);
#ifdef __OPTIMIZE__
    flags += " (optimized)";
#else
    flags += " (unoptimized)";
#endif
    return flags;
#endif
}

inline std::string platform_info() {
#if defined(__APPLE__)
    const std::string os = "macOS";
#elif defined(__linux__)
    const std::string os = "Linux";
#elif defined(_WIN32)
    const std::string os = "Windows";
#else
    const std::string os = "Unknown OS";
#endif

#if defined(__aarch64__) || defined(_M_ARM64)
    const std::string arch = "arm64";
#elif defined(__x86_64__) || defined(_M_X64)
    const std::string arch = "x86_64";
#else
    const std::string arch = "unknown-arch";
#endif
    return os + " (" + arch + ")";
}

// Analytical heap + struct footprint estimate of the reference Abstraction + Controller
// (complementary to OS peak RSS; useful for explaining memory scaling in the report).
inline std::uint64_t estimate_reference_bytes(const abstraction::Abstraction& A,
                                              const simulation::Controller& ctrl) {
    std::uint64_t total = 0;
    total += static_cast<std::uint64_t>(A.transitions.capacity()) *
             sizeof(abstraction::Successors);
    for (const auto& box : A.transitions) {
        total += static_cast<std::uint64_t>(box.lower.capacity() + box.upper.capacity()) *
                 sizeof(int);
    }
    total += static_cast<std::uint64_t>(ctrl.capacity()) *
             sizeof(std::vector<abstraction::Id>);
    for (const auto& cmds : ctrl) {
        total += static_cast<std::uint64_t>(cmds.capacity()) * sizeof(abstraction::Id);
    }
    return total;
}

struct BenchmarkRow {
    std::string method = "Reference";
    int rooms = 0;
    abstraction::Id num_states = 0;
    abstraction::Id num_commands = 0;
    abstraction::Id num_transitions = 0;
    double abstraction_seconds = 0.0;
    double synthesis_seconds = 0.0;
    double total_seconds = 0.0;
    MemorySample peak_rss_bytes;  // available == false => missing
    std::uint64_t estimated_data_bytes = 0;
    abstraction::Id certified_states = 0;
    abstraction::Id total_safe_pairs = 0;
    double controllable_pct = 0.0;
    bool verified = false;
    std::string status = "OK";
};

// Runs the reference pipeline for N rooms and returns separate abstraction & synthesis
// timings, peak memory, certified-state percentage, and independent verification result.
inline BenchmarkRow benchmark_reference(int N,
                                        bool run_verify = true,
                                        abstraction::Abstraction* out_model = nullptr,
                                        simulation::Controller* out_controller = nullptr) {
    BenchmarkRow row;
    row.method = "Reference";
    row.rooms = N;

    try {
        const auto t0 = SteadyClock::now();
        abstraction::Abstraction A = safety_abstraction::build_abstraction(N);
        const auto t1 = SteadyClock::now();

        simulation::Controller ctrl = synthesize_safety(A);
        const auto t2 = SteadyClock::now();

        row.num_states = A.num_states;
        row.num_commands = A.num_commands;
        row.num_transitions = static_cast<abstraction::Id>(A.transitions.size());
        row.abstraction_seconds = std::chrono::duration<double>(t1 - t0).count();
        row.synthesis_seconds = std::chrono::duration<double>(t2 - t1).count();
        row.total_seconds = row.abstraction_seconds + row.synthesis_seconds;

        for (const auto& cmds : ctrl) {
            if (!cmds.empty()) {
                ++row.certified_states;
                row.total_safe_pairs += static_cast<abstraction::Id>(cmds.size());
            }
        }
        row.controllable_pct = row.num_states > 0
            ? (100.0 * static_cast<double>(row.certified_states) /
               static_cast<double>(row.num_states))
            : 0.0;

        row.estimated_data_bytes = estimate_reference_bytes(A, ctrl);
        row.peak_rss_bytes = measure_peak_rss_bytes();
        row.verified = run_verify ? verify_controller(A, ctrl) : false;

        if (out_model) *out_model = std::move(A);
        if (out_controller) *out_controller = std::move(ctrl);
    } catch (const std::exception& e) {
        row.status = std::string("FAILED: ") + e.what();
        row.peak_rss_bytes = measure_peak_rss_bytes();
        row.verified = false;
    }

    return row;
}

}  // namespace integ
