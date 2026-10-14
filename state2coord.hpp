// C++ port of state2coord.m
// Single state number (1-based) -> grid coordinates (one index per dimension, 1-based).
// MATLAB: I(k) = floor(mod(i-1, p(k+1)) / p(k)) + 1
#pragma once
#include <array>

template <size_t N>
std::array<int, N> state2coord(int i, const std::array<int, N + 1>& p) {
    std::array<int, N> I;
    for (size_t k = 0; k < N; ++k) I[k] = ((i - 1) % p[k + 1]) / p[k] + 1;
    return I;
}

// this does the inverse it goes from a cell in the discretized grid
// into the coords of the in the grip itself and not into (x1,x2,x3) **non bijective**
// it may link us to the respective x1,x2,x3 intervals which yield the cell