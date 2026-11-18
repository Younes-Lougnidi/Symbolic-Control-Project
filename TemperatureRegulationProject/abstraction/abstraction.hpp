#pragma once

#include <vector>
#include <stdexcept>
#include <utility>
#include <cmath>

#include "../dynamics/dynamics.hpp"
#include "../dynamics/state2coord.hpp"
#include "../dynamics/coord2state.hpp"

namespace abstraction {

using Id = long long;
using Cell = std::vector<int>;

// Stores the possible successor cells for one state-command pair.
struct Successors {
    Cell lower;
    Cell upper;
    bool leaves_safe_set;
};

// Stores the complete symbolic abstraction of the building.
struct Abstraction {
    int N;
    Id num_states;
    Id num_commands;
    std::vector<Successors> transitions;
};

// Converts a 0-based state ID into 0-based cell coordinates.
inline Cell decode_cell(Id state, int N) {
    if (N < 1 || state < 0) {
        throw std::invalid_argument("Invalid state ID or room count");
    }

    auto p_T = dyn::multiplier(N, dyn::cpr);

    if (state >= p_T[N]) {
        throw std::out_of_range("State ID exceeds the grid");
    }

    // Convert the 0-based ID to the 1-based indexing used by state2coord.
    Cell bins = state2coord(state + 1, p_T);

    // Convert the resulting cell coordinates back to 0-based indexing.
    for (int& bin : bins) {
        --bin;
    }

    return bins;
}

// Converts 0-based cell coordinates into a unique 0-based state ID.
inline Id encode_cell(const Cell& bins) {

    int N = static_cast<int>(bins.size());

    if (N < 1) {
        throw std::invalid_argument("Cell vector cannot be empty");
    }

    // Validate that every cell index belongs to the temperature grid.
    for (int bin : bins) {
        if (bin < 0 || bin >= dyn::cpr) {
            throw std::out_of_range("Invalid cell coordinate");
        }
    }

    auto p_T = dyn::multiplier(N, dyn::cpr);

    // Convert the cell coordinates to the 1-based indexing convention.
    Cell one_based_bins = bins;

    for (int& bin : one_based_bins) {
        ++bin;
    }

    // Convert to a state ID, then return it using 0-based indexing.
    return coord2state(one_based_bins, p_T) - 1;
}

// Converts a command ID into the corresponding heater power settings.
inline VEC_N decode_command(Id input, int N) {

    if (N < 1 || input < 0) {
        throw std::invalid_argument("Invalid command ID or room count");
    }

    auto p_u = dyn::multiplier(N, dyn::nu);

    if (input >= p_u[N]) {
        throw std::out_of_range("Command ID exceeds the input grid");
    }

    VEC_N powers(N);

    // Extract the base-3 digits and convert them to heater powers.
    for (int i = 0; i < N; ++i) {
        int digit = static_cast<int>(input % dyn::nu);

        powers[i] = dyn::u_lo + digit * dyn::d_u;

        input /= dyn::nu;
    }

    return powers;
}

// Finds the symbolic state ID corresponding to exact room temperatures.
inline Id locate_cell(const VEC_N& T) {

    if (T.empty()) {
        throw std::invalid_argument("Temperature vector cannot be empty");
    }

    Cell bins(T.size());

    for (size_t i = 0; i < T.size(); ++i) {

        // Reject invalid temperatures or temperatures outside the safe region.
        if (!std::isfinite(T[i]) ||
            T[i] < dyn::T_lo ||
            T[i] > dyn::T_hi) {
            throw std::out_of_range("Temperature outside safe region");
        }

        // Determine the cell containing the current temperature.
        int bin = static_cast<int>(
            std::floor((T[i] - dyn::T_lo) / dyn::d_T)
        );

        // Include the maximum temperature in the last cell.
        if (bin == dyn::cpr) {
            bin = dyn::cpr - 1;
        }

        bins[i] = bin;
    }

    return encode_cell(bins);
}

// Converts cell coordinates into their temperature intervals.
inline Bounds cell_bounds(const Cell& bins) {

    if (bins.empty()) {
        throw std::invalid_argument("Empty cell vector");
    }

    VEC_N low(bins.size());
    VEC_N high(bins.size());

    for (size_t i = 0; i < bins.size(); ++i) {

        if (bins[i] < 0 || bins[i] >= dyn::cpr) {
            throw std::out_of_range("Invalid cell coordinate");
        }

        low[i] = dyn::T_lo + bins[i] * dyn::d_T;
        high[i] = low[i] + dyn::d_T;
    }

    return {low, high};
}

// Builds the symbolic transition abstraction for a three-room building.
inline Abstraction build_abstraction(int N) {

    if (N < 3) {
        throw std::invalid_argument(
            "Reference abstraction currently supports N >= 3"
        );
    }

    Abstraction A;
    A.N = N;

    // Compute the total numbers of symbolic states and heater commands.
    auto p_T = dyn::multiplier(N, dyn::cpr);
    auto p_u = dyn::multiplier(N, dyn::nu);

    A.num_states = p_T[N];
    A.num_commands = p_u[N];

    // Allocate one transition entry for every state-command pair.
    A.transitions.resize(
        A.num_states * A.num_commands
    );

    // Examine every symbolic state.
    for (Id state = 0; state < A.num_states; ++state) {

        Cell bins = decode_cell(state, N);

        // Determine the temperature intervals represented by this state.
        Bounds current = cell_bounds(bins);

        // Evaluate every possible heater command.
        for (Id command = 0; command < A.num_commands; ++command) {

            VEC_N u = decode_command(command, N);

            // Compute the reachable temperature intervals after one step.
            Bounds next = dyn::reachable_bounds(
                current.first,
                current.second,
                u
            );

            Successors s;
            s.lower.resize(N, -1);
            s.upper.resize(N, -1);
            s.leaves_safe_set = false;

            // Find the successor cells for every room.
            for (int i = 0; i < N; ++i) {

                double next_low = next.first[i];
                double next_high = next.second[i];

                // Detect whether any reachable temperature leaves the safe region.
                if (next_low < dyn::T_lo ||
                    next_high > dyn::T_hi) {
                    s.leaves_safe_set = true;
                }

                // Check which temperature cells intersect the reachable interval.
                for (int bin = 0; bin < dyn::cpr; ++bin) {

                    double cell_low =
                        dyn::T_lo + bin * dyn::d_T;

                    double cell_high =
                        cell_low + dyn::d_T;

                    // Include all cells that overlap or touch the interval.
                    if (next_low <= cell_high &&
                        next_high >= cell_low) {

                        if (s.lower[i] == -1) {
                            s.lower[i] = bin;
                        }

                        s.upper[i] = bin;
                    }
                }
            }

            // Save the transition using the state-command index.
            A.transitions[
                state * A.num_commands + command
            ] = std::move(s);
        }
    }

    return A;
}

}
