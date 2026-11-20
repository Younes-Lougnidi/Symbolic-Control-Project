#include "SafetyAbstraction/checked_abstraction.hpp"
#include "safety_synthesis.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using abstraction::Abstraction;

// The version identifies the building model and its fixed discretization.
// Regenerate the cache after changing the model or the grid.
constexpr char format[] = "BUILDING-10x3-v1";

void save(const Abstraction& a, const fs::path& path) {
    if (fs::exists(path)) throw std::runtime_error("Cache already exists");
    fs::path partial = path;
    partial += ".partial";
    if (fs::exists(partial)) throw std::runtime_error("Partial file already exists");
    try {
        std::ofstream out;
        out.exceptions(std::ios::badbit | std::ios::failbit);
        out.open(partial, std::ios::binary);
        out.write(format, sizeof(format));
        out.put(static_cast<char>(a.N));
        std::vector<char> record(1 + a.N);
        for (const auto& box : a.transitions) {
            record[0] = box.leaves_safe_set;
            // Both bin indices (-1 through 9) fit in four bits after adding 1.
            for (int i = 0; i < a.N; ++i)
                record[i+1] = static_cast<char>(((box.lower[i]+1) << 4) | (box.upper[i]+1));
            out.write(record.data(), record.size());
        }
        out.close();
        fs::rename(partial, path);
    } catch (...) {
        std::error_code ignored;
        fs::remove(partial, ignored);
        throw;
    }
}

Abstraction load(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    in.exceptions(std::ios::badbit | std::ios::failbit);
    char header[sizeof(format)];
    in.read(header, sizeof(header));
    if (!std::equal(std::begin(header), std::end(header), format))
        throw std::runtime_error("Unsupported cache format");
    const int rooms = in.get();
    if (rooms < 3) throw std::runtime_error("Invalid room count");
    Abstraction a{};
    a.N = rooms;
    a.num_states = safety_abstraction::multiplier(rooms, dyn::cpr).back();
    a.num_commands = safety_abstraction::multiplier(rooms, dyn::nu).back();
    const auto bytes = fs::file_size(path) - sizeof(format) - 1;
    const auto width = static_cast<unsigned>(1 + rooms);
    // Divide before multiplying to avoid overflow on a damaged header.
    if (bytes % width || bytes / width % a.num_commands ||
        bytes / width / a.num_commands != static_cast<std::uintmax_t>(a.num_states))
        throw std::runtime_error("Cache length does not match dimensions");
    a.transitions.resize(bytes / width);
    std::vector<unsigned char> record(width);
    for (auto& box : a.transitions) {
        in.read(reinterpret_cast<char*>(record.data()), width);
        if (record[0] > 1) throw std::runtime_error("Invalid exit flag");
        box.leaves_safe_set = record[0];
        box.lower.resize(rooms);
        box.upper.resize(rooms);
        for (int i = 0; i < rooms; ++i) {
            const int lo = (record[i+1] >> 4) - 1;
            const int hi = (record[i+1] & 15) - 1;
            if (!(lo >= 0 && lo <= hi && hi < dyn::cpr) &&
                !(box.leaves_safe_set && lo == -1 && hi == -1))
                throw std::runtime_error("Invalid successor bounds");
            box.lower[i] = lo;
            box.upper[i] = hi;
        }
    }
    return a;
}

double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now()-start).count();
}

int main(int argc, char** argv) {
    try {
        auto start = Clock::now();
        if (argc == 4 && std::string(argv[1]) == "generate") {
            if (fs::exists(argv[3])) throw std::runtime_error("Cache already exists");
            auto a = safety_abstraction::build_abstraction(std::stoi(argv[2]));
            std::cout << "Build seconds: " << seconds(start) << std::endl;
            start = Clock::now();
            save(a, argv[3]);
            std::cout << "Save seconds: " << seconds(start) << ", bytes: " << fs::file_size(argv[3]) << '\n';
        } else if (argc == 3 && std::string(argv[1]) == "solve") {
            auto a = load(argv[2]);
            std::cout << "Load seconds: " << seconds(start) << std::endl;
            start = Clock::now();
            auto controller = synthesize_safety(a);
            const auto synthesis = seconds(start);
            std::uint64_t winners = 0, pairs = 0;
            for (const auto& commands : controller) {
                winners += !commands.empty();
                pairs += commands.size();
            }
            std::cout << "N=" << a.N << ", synthesis seconds: " << synthesis
                      << ", winning cells: " << winners << '/' << a.num_states
                      << ", safe state-command pairs: " << pairs << '\n';
        } else throw std::runtime_error("Use: building_cache generate N FILE | building_cache solve FILE");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
