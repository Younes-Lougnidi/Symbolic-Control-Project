#pragma once
// Large-ring verification used by Scalability/correctness_verification.cpp.
#include "../Scalability/block_safety.hpp"
#include "verification.hpp"
#include <chrono>
#include <iostream>
#include <sstream>

namespace large_ring_replay {
using Index = std::uint64_t;
using Choices = local_safety::HeaterChoices;
using Table = local_safety::SafetyTable;

inline Index previous(Index i, Index n) { return i == 0 ? n - 1 : i - 1; }
inline Index following(Index i, Index n) { return i == n - 1 ? 0 : i + 1; }

// A reproducible mixed bin at any room index, so both passes can revisit the
// same initial state without a file. Start each temperature at its bin centre.
inline int initial_bin(Index i) {
    Index x = i + 42 + 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return static_cast<int>((x ^ (x >> 31)) % integ::REF_CPR);
}

inline double centre(int bin) { return integ::REF_T_LO + (bin + 0.5) * integ::REF_D_T; }
inline int bin_of(double t) {
    // Called only on temperatures already checked against the safe range.
    return std::min(integ::REF_CPR - 1, static_cast<int>((t - integ::REF_T_LO) / integ::REF_D_T));
}
inline int first_allowed(const Choices& row) {
    return static_cast<int>(std::find(row.begin(), row.end(), true) - row.begin());
}

struct Reference {
    Choices allowed[10][10][10]{};
    double after_first_step[10][10][10]{};
};

inline Reference prepare_reference(const Table& policy) {
    Reference ref{};
    for (int l = 0; l < 10; ++l) for (int m = 0; m < 10; ++m) for (int r = 0; r < 10; ++r) {
        VEC_N low, high;
        for (int bin : {l, m, r}) {
            low.push_back(integ::REF_T_LO + bin * integ::REF_D_T);
            high.push_back(integ::REF_T_LO + (bin + 1) * integ::REF_D_T);
        }
        auto& allowed = ref.allowed[l][m][r];
        for (int d = 0; d < integ::REF_NU; ++d) {
            const VEC_N u{0, integ::REF_D_U * d, 0};
            allowed[d] = integ::reference_step(low, u)[1] >= integ::REF_T_LO &&
                         integ::reference_step(high, u)[1] <= integ::REF_T_HI;
        }
        if (allowed != policy.allowed[l][m][r] || first_allowed(allowed) == integ::REF_NU)
            throw std::runtime_error("Local choices differ from the reference corner checks");
        // This is the exact updated temperature for this initial neighbourhood.
        // The first full-ring pass checks it against the actually chosen command.
        const VEC_N u{0, integ::REF_D_U * first_allowed(allowed), 0};
        ref.after_first_step[l][m][r] = integ::reference_step({centre(l), centre(m), centre(r)}, u)[1];
    }
    return ref;
}

inline void check_step(int step, Index rooms, const Table& policy, const Reference& ref) {
    const auto started = std::chrono::steady_clock::now();
    auto temperature = [&](Index i) {
        if (step == 1) return centre(initial_bin(i));
        return ref.after_first_step[initial_bin(previous(i, rooms))][initial_bin(i)]
                                   [initial_bin(following(i, rooms))];
    };
    Index inputs = 0, checked = 0, changed_bins = 0, checksum = 0;
    double max_error = 0.0;
    auto next_bin = [&]() { return bin_of(temperature(inputs++)); };
    auto check_block = [&](Index first, const std::vector<Choices>& choices) {
        // The controller emits rooms 1..N-1, then room 0. Check for skips and repeats.
        const Index expected_first = checked == rooms - 1 ? 0 : checked + 1;
        if (first != expected_first || choices.empty() || choices.size() > rooms - first ||
            (first == 0 && choices.size() != 1))
            throw std::runtime_error("Incorrect block coverage");

        // Supply the real left and right neighbours. Only interior results
        // count, so the small array's own wraparound cannot affect the audit.
        VEC_N old(choices.size() + 2), powers(old.size(), 0.0);
        Index index = previous(first, rooms);
        for (double& value : old) {
            value = temperature(index);
            index = following(index, rooms);
        }
        for (std::size_t j = 0; j < choices.size(); ++j) {
            const auto& expected = ref.allowed[bin_of(old[j])][bin_of(old[j+1])][bin_of(old[j+2])];
            if (choices[j] != expected) {
                std::ostringstream message;
                message << "Step " << step << ", room " << first + j
                        << ", bins (" << bin_of(old[j]) << ',' << bin_of(old[j+1])
                        << ',' << bin_of(old[j+2]) << "). Expected off/medium/high "
                        << expected[0] << expected[1] << expected[2] << ", received "
                        << choices[j][0] << choices[j][1] << choices[j][2];
                throw std::runtime_error(message.str());
            }
            const int d = first_allowed(choices[j]);
            powers[j+1] = d * dyn::d_u;
            checksum += d;
        }
        const auto actual = dyn::f(old, powers);
        const auto reference = integ::reference_step(old, powers);
        for (std::size_t j = 0; j < choices.size(); ++j) {
            const auto k = j + 1;
            const double t = actual[k], error = std::abs(t - reference[k]);
            if (!std::isfinite(t) || t < integ::REF_T_LO || t > integ::REF_T_HI || error > 1e-12)
                throw std::runtime_error("Unsafe or incorrect update at room " + std::to_string(first+j));
            if (step == 1 && t != ref.after_first_step[bin_of(old[j])][bin_of(old[k])][bin_of(old[k+1])])
                throw std::runtime_error("Second pass would read a different temperature");
            max_error = std::max(max_error, error);
            changed_bins += bin_of(t) != bin_of(old[k]);
        }
        const Index before = checked;
        checked += choices.size();
        constexpr Index report_every = 5000000000ULL;
        if (checked / report_every > before / report_every && checked < rooms)
            std::cout << "Progress step=" << step << " rooms_checked=" << checked << std::endl;
    };
    local_safety::safe_commands_in_blocks(policy, rooms, next_bin, check_block);
    if (checked != rooms || inputs != rooms) throw std::runtime_error("Wrong room count");
    std::cout << "PASS step=" << step << " rooms=" << checked << " changed_bins=" << changed_bins
              << " checksum=" << checksum << " max_temperature_error=" << max_error
              << " audit_seconds=" << std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()
              << std::endl;
}

inline void run(const Table& policy, Index rooms) {
    const auto ref = prepare_reference(policy);
    std::cout << "PASS 3000 reference corner decisions" << std::endl;
    check_step(1, rooms, policy, ref);
    check_step(2, rooms, policy, ref);
}
} // namespace large_ring_replay
