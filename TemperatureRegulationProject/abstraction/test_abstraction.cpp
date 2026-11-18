
#include "abstraction.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <random>
#include <limits>
#include <stdexcept>

// Compare floating-point values with a small tolerance.
bool near(double a, double b) {
    return std::abs(a - b) < 1e-9;
}

int main() {

    // --------------------------------------------------
    // TEST 1: Cell encoding and decoding
    // --------------------------------------------------

    assert((abstraction::decode_cell(863, 3)
            == abstraction::Cell{3, 6, 8}));

    assert((abstraction::decode_cell(0, 3)
            == abstraction::Cell{0, 0, 0}));

    assert((abstraction::decode_cell(999, 3)
            == abstraction::Cell{9, 9, 9}));

    assert(abstraction::encode_cell({3, 6, 8}) == 863);
    assert(abstraction::encode_cell({0, 0, 0}) == 0);
    assert(abstraction::encode_cell({9, 9, 9}) == 999);

    // Verify round trips for every three-room state.
    for (long long id = 0; id < 1000; ++id) {
        auto bins = abstraction::decode_cell(id, 3);
        assert(abstraction::encode_cell(bins) == id);
    }

    std::cout << "[PASS] Encoding and decoding\n";


    // --------------------------------------------------
    // TEST 2: Heater command decoding
    // --------------------------------------------------

    auto u0 = abstraction::decode_command(0, 3);
    auto u19 = abstraction::decode_command(19, 3);
    auto u26 = abstraction::decode_command(26, 3);

    assert((u0 == VEC_N{0.0, 0.0, 0.0}));

    assert(near(u19[0], 0.3));
    assert(near(u19[1], 0.0));
    assert(near(u19[2], 0.6));

    assert(near(u26[0], 0.6));
    assert(near(u26[1], 0.6));
    assert(near(u26[2], 0.6));

    // Every decoded command must use a valid heater level.
    for (long long id = 0; id < 27; ++id) {
        auto powers = abstraction::decode_command(id, 3);

        assert(powers.size() == 3);

        for (double p : powers) {
            assert(near(p, 0.0) ||
                   near(p, 0.3) ||
                   near(p, 0.6));
        }
    }

    std::cout << "[PASS] Heater command decoding\n";


    // --------------------------------------------------
    // TEST 3: Temperature boundaries
    // --------------------------------------------------

    assert(abstraction::locate_cell(
        {17.0, 17.0, 17.0}) == 0);

    assert(abstraction::locate_cell(
        {22.0, 22.0, 22.0}) == 999);

    assert(abstraction::locate_cell(
        {18.7, 20.2, 21.4}) == 863);

    // Internal boundary: 17.5 belongs to cell 1.
    assert(abstraction::locate_cell(
        {17.5, 17.0, 17.0}) == 1);

    // Reject temperatures outside the safe region.
    bool rejected_low = false;
    bool rejected_high = false;
    bool rejected_nan = false;

    try {
        abstraction::locate_cell({16.9, 20.0, 21.0});
    } catch (const std::out_of_range&) {
        rejected_low = true;
    }

    try {
        abstraction::locate_cell({17.0, 22.1, 21.0});
    } catch (const std::out_of_range&) {
        rejected_high = true;
    }

    try {
        abstraction::locate_cell({
            std::numeric_limits<double>::quiet_NaN(),
            20.0, 21.0
        });
    } catch (const std::out_of_range&) {
        rejected_nan = true;
    }

    assert(rejected_low);
    assert(rejected_high);
    assert(rejected_nan);

    std::cout << "[PASS] Temperature boundaries\n";


    // --------------------------------------------------
    // TEST 4: Integration with temperature dynamics
    // --------------------------------------------------

    abstraction::Cell bins = {3, 6, 8};

    Bounds current = abstraction::cell_bounds(bins);

    assert((current.first == VEC_N{18.5, 20.0, 21.0}));
    assert((current.second == VEC_N{19.0, 20.5, 21.5}));

    VEC_N heaters = {0.3, 0.0, 0.6};

    Bounds next = dyn::reachable_bounds(
        current.first,
        current.second,
        heaters
    );

    assert(near(next.first[0], 20.273));
    assert(near(next.second[0], 20.737));

    std::cout << "[PASS] Dynamics integration\n";


    // --------------------------------------------------
    // TEST 5: Abstraction dimensions and transitions
    // --------------------------------------------------

    abstraction::Abstraction A =
        abstraction::build_abstraction(3);

    assert(A.N == 3);
    assert(A.num_states == 1000);
    assert(A.num_commands == 27);
    assert(A.transitions.size() == 27000);

    // Check a known state-command transition.
    const auto& s = A.transitions[862 * 27 + 19];

    assert((s.lower == abstraction::Cell{6, 3, 5}));
    assert((s.upper == abstraction::Cell{7, 4, 6}));
    assert(!s.leaves_safe_set);

    // The coldest state with all heaters off is unsafe.
    assert(A.transitions[0].leaves_safe_set);

    // Medium heating in every room preserves safety.
    for (long long state = 0; state < A.num_states; ++state) {
        const auto& medium =
            A.transitions[state * A.num_commands + 13];

        assert(!medium.leaves_safe_set);
    }

    std::cout << "[PASS] Abstraction size and transitions\n";


    // --------------------------------------------------
    // TEST 6: No missed successors from sampled points
    // --------------------------------------------------

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    long long samples_checked = 0;

    // Test every symbolic state and every heater command.
    for (long long state = 0; state < A.num_states; ++state) {

        auto cell = abstraction::decode_cell(state, 3);
        Bounds source = abstraction::cell_bounds(cell);

        for (long long command = 0;
             command < A.num_commands;
             ++command) {

            VEC_N u = abstraction::decode_command(command, 3);

            const auto& successors =
                A.transitions[state * A.num_commands + command];

            assert(successors.lower.size() == 3);
            assert(successors.upper.size() == 3);

            // Sample five temperatures inside the source box:
            // lower corner, upper corner, midpoint, and two random points.
            for (int sample = 0; sample < 5; ++sample) {

                VEC_N T(3);

                for (int i = 0; i < 3; ++i) {
                    double fraction;

                    if (sample == 0) {
                        fraction = 0.0;
                    } else if (sample == 1) {
                        fraction = 1.0;
                    } else if (sample == 2) {
                        fraction = 0.5;
                    } else {
                        fraction = dist(rng);
                    }

                    T[i] = source.first[i] +
                        fraction * (source.second[i] - source.first[i]);
                }

                // Compute the actual successor from the sampled point.
                VEC_N actual = dyn::f(T, u);

                for (int i = 0; i < 3; ++i) {

                    // Check if the exact successor leaves safety.
                    if (actual[i] < dyn::T_lo ||
                        actual[i] > dyn::T_hi) {

                        assert(successors.leaves_safe_set);
                        continue;
                    }

                    // The sampled successor must lie inside the
                    // stored successor-cell coordinate box.
                    int low_bin = successors.lower[i];
                    int high_bin = successors.upper[i];

                    assert(low_bin >= 0);
                    assert(high_bin < dyn::cpr);
                    assert(low_bin <= high_bin);

                    double covered_low =
                        dyn::T_lo + low_bin * dyn::d_T;

                    double covered_high =
                        dyn::T_lo + (high_bin + 1) * dyn::d_T;

                    assert(actual[i] >= covered_low - 1e-9);
                    assert(actual[i] <= covered_high + 1e-9);
                }

                ++samples_checked;
            }
        }
    }

    std::cout << "[PASS] Sampled successor soundness ("
              << samples_checked << " samples)\n";


    std::cout << "\nALL ABSTRACTION TESTS PASSED!\n";

    return 0;
}
