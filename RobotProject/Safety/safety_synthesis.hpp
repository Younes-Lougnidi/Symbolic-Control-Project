#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include "../robotControl/dynamics.hpp"

// The existing safety algorithm uses these views of the abstraction.
struct SymbolicAbstraction {
    const std::vector<std::array<int32_t, 2>>& g;
    const std::vector<Vec2>& val_u;
    const Vec3& d_x;
    const std::array<int, 4>& p_x;
    int n_xi, n_sigma;
};

struct Box {
    int16_t lo[3], hi[3]; // 1-based grid coordinates of the two corners
    bool outside;
};

std::vector<Box> unpack(const SymbolicAbstraction& A);
std::vector<uint8_t> safe_set(const SymbolicAbstraction& A);
std::vector<uint8_t> algorithm1(
    const SymbolicAbstraction& A, const std::vector<uint8_t>& S,
    const std::vector<Box>& B, int& iters);
std::vector<uint16_t> build_controller(
    const SymbolicAbstraction& A, const std::vector<uint8_t>& Zinf,
    const std::vector<Box>& B);
bool verify_controller(
    const SymbolicAbstraction& A, const std::vector<uint8_t>& Zinf,
    const std::vector<uint16_t>& h, const std::vector<Box>& B);

// Simple entry point for mission synthesis:
//   auto g_safe = filter_safe_transitions(g);
// Returns a copy of g, with forbidden commands replaced by {0, 0}.
// Uses the existing invariant safety algorithm: avoid touching R4 forever.
// This can be more restrictive than avoiding R4 only until mission completion.
// Compile and link Safety/safety_synthesis.cpp with the calling program.
std::vector<std::array<int32_t, 2>> filter_safe_transitions(
    const std::vector<std::array<int32_t, 2>>& g);
