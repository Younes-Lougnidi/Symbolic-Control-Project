#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "run_tests.hpp"

// Integration and verification runner for building temperature control (Rayane Khaldi).
//
// Usage:
//   ./test_integration                            Run full suite + N=3,4 baseline
//   ./test_integration --bench N                  Run isolated single-N benchmark (exact peak RSS)
//   ./test_integration --sim-dir PATH             Path to simulation/ directory (default: ../simulation)
//   ./test_integration --out-dir PATH             Output directory for results (default: .)
//   ./test_integration --rooms 3,4                Comma-separated room counts for baseline

namespace {

std::vector<int> parse_rooms(const std::string& arg) {
    std::vector<int> out;
    std::size_t start = 0;
    while (start < arg.size()) {
        const std::size_t comma = arg.find(',', start);
        const std::string tok = (comma == std::string::npos)
            ? arg.substr(start)
            : arg.substr(start, comma - start);
        if (!tok.empty()) out.push_back(std::stoi(tok));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string sim_dir = "../simulation";
    std::string out_dir = ".";
    std::vector<int> rooms = {3, 4};
    int isolated_bench_n = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--bench" && i + 1 < argc) {
            isolated_bench_n = std::stoi(argv[++i]);
        } else if (arg == "--sim-dir" && i + 1 < argc) {
            sim_dir = argv[++i];
        } else if (arg == "--out-dir" && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (arg == "--rooms" && i + 1 < argc) {
            rooms = parse_rooms(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: test_integration [--bench N] [--rooms 3,4] "
                         "[--sim-dir PATH] [--out-dir PATH]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return 2;
        }
    }

    if (isolated_bench_n >= 0) {
        const auto row = integ::benchmark_reference(isolated_bench_n, true);
        const double est_mib = static_cast<double>(row.estimated_data_bytes) / (1024.0 * 1024.0);
        std::cout << "[Isolated Benchmark] Method=" << row.method
                  << " | N=" << row.rooms
                  << " | states=" << row.num_states
                  << " | cmds=" << row.num_commands
                  << " | abs=" << integ::format_seconds(row.abstraction_seconds)
                  << " | syn=" << integ::format_seconds(row.synthesis_seconds)
                  << " | total=" << integ::format_seconds(row.total_seconds)
                  << " | peak_rss=" << integ::format_memory_mb(row.peak_rss_bytes)
                  << " | est_data=" << std::fixed << std::setprecision(2) << est_mib << " MiB"
                  << " | controllable=" << row.controllable_pct << "% ("
                  << row.certified_states << "/" << row.num_states << ")"
                  << " | verified=" << (row.verified ? "PASS" : "FAIL")
                  << " | status=" << row.status << "\n";
        return (row.verified && row.status == "OK") ? 0 : 1;
    }

    const bool ok = integ::run_experiments_detailed(rooms, sim_dir, out_dir);
    return ok ? 0 : 1;
}
