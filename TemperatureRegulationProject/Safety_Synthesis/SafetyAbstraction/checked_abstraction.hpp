#pragma once

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "../../abstraction/abstraction.hpp"

namespace safety_abstraction {

// refined the multiplier function to include more checks and be sure that we dont overflow
inline std::vector<abstraction::Id> multiplier(int N, int base) {
    if (N < 0 || base < 1)
        throw std::invalid_argument("Expected a nonnegative room count and a positive base");

    std::vector<abstraction::Id> powers{1};
    const auto limit = std::numeric_limits<abstraction::Id>::max();
    for (int i = 0; i < N; ++i) {
        // Divide first so the check itself cannot overflow. Integer
        // multiplication keeps even large, representable counts exact.
        if (powers.back() > limit / base)
            throw std::overflow_error("Grid power exceeds the ID range");
        powers.push_back(powers.back() * base);
    }
    return powers;
}

inline abstraction::Abstraction build_abstraction(int N) {
    if (N < 3)
        throw std::invalid_argument("The circular building requires at least three rooms");

    const auto state_counts = multiplier(N, dyn::cpr);
    const auto command_counts = multiplier(N, dyn::nu);
    const auto states = state_counts.back();
    const auto commands = command_counts.back();

    // At N=13 both counts fit individually, but their product 30^13
    // exceeds a signed 64-bit ID. Check before the builder multiplies them.
    if (states > std::numeric_limits<abstraction::Id>::max() / commands)
        throw std::overflow_error("Transition count exceeds the ID range");
    const auto entries = static_cast<unsigned long long>(states * commands);
    if (entries > std::vector<abstraction::Successors>{}.max_size())
        throw std::length_error("Transition count exceeds the vector size limit");

    // The existing builder computes its own counts. Verify that they
    // agree with our exact ones before reusing its transition algorithm.
    if (dyn::multiplier(N, dyn::cpr) != state_counts ||
        dyn::multiplier(N, dyn::nu) != command_counts)
        throw std::overflow_error("Builder counts differ from exact integer counts");

    // These checks establish index and size limits. Allocation still
    // depends on available RAM for the table and its successor boxes.
    return abstraction::build_abstraction(N);
}

}
