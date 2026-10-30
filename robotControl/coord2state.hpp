// C++ port of coord2state.m
// Grid coordinates (one index per dimension, 1-based) -> single state number (1-based).
// MATLAB: i = (I-1)' * p_x(1:length(I)) + 1
#pragma once
#include <array>

template <size_t N>
int coord2state(const std::array<int, N>& I, const std::array<int, N + 1>& p) {
    int i = 1;
    for (size_t k = 0; k < N; ++k) i += (I[k] - 1) * p[k];
    return i;
}


// so this file is meant for the transition from coords to states. Where it gives each coord(a,b,c) a unique state number.

// the coords are those of the states.
/*
 to explain, we will first dispose of (x1,x2,x3) so we use each x_i 
 to determin in which discrete celli it would inhabit. 
 we will endup with (a,b,c) discretized coord in the cell grid which
 will then be converted by coord2state into a unique id giving us 
 the exact cell it lives in

 */ 