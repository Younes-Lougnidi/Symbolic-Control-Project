#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

#include "../robotControl/dynamics.hpp"
#include "../mission-rules/labeling&MissionRules.hpp"
#include "../ControllerSynthesis/ControllerSythesis.hpp"

namespace sim_io {

// Loads the abstraction table g and control values val_u from symb_abs.bin
inline bool load_abstraction(const std::string& path,
                             Transitions& g,
                             std::vector<Vec2>& val_u) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    int32_t hdr[2];
    in.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
    Vec3 robust_margins;
    in.read(reinterpret_cast<char*>(robust_margins.data()), sizeof(double) * 3);
    val_u.resize(hdr[1]);
    for (auto& u : val_u) {
        in.read(reinterpret_cast<char*>(u.data()), sizeof(double) * 2);
    }
    g.resize(static_cast<std::size_t>(hdr[0]) * hdr[1]);
    in.read(reinterpret_cast<char*>(g.data()), sizeof(int32_t) * 2 * g.size());
    return in.good();
}

// Saves a synthesized Controller to binary format
inline bool save_controller(const Controller& ctrl, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    int32_t n_states = static_cast<int32_t>(ctrl.rank.size() - 1);
    int32_t n_missions = 5;
    out.write(reinterpret_cast<const char*>(&n_states), sizeof(n_states));
    out.write(reinterpret_cast<const char*>(&n_missions), sizeof(n_missions));
    for (int cell = 0; cell <= n_states; ++cell) {
        out.write(reinterpret_cast<const char*>(ctrl.rank[cell].data()), sizeof(int) * 5);
        out.write(reinterpret_cast<const char*>(ctrl.command[cell].data()), sizeof(int) * 5);
    }
    return out.good();
}

// Loads a synthesized Controller from binary format
inline bool load_controller(Controller& ctrl, const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    int32_t n_states = 0;
    int32_t n_missions = 0;
    in.read(reinterpret_cast<char*>(&n_states), sizeof(n_states));
    in.read(reinterpret_cast<char*>(&n_missions), sizeof(n_missions));
    if (n_states != dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2] || n_missions != 5) {
        return false;
    }
    ctrl.rank.resize(n_states + 1);
    ctrl.command.resize(n_states + 1);
    for (int cell = 0; cell <= n_states; ++cell) {
        in.read(reinterpret_cast<char*>(ctrl.rank[cell].data()), sizeof(int) * 5);
        in.read(reinterpret_cast<char*>(ctrl.command[cell].data()), sizeof(int) * 5);
    }
    return in.good();
}

// Fast synthesis equivalent: uses precomputed cell labels to avoid redundant allocations in Omar's label()
// Mathematically produces the exact same result as Younes's synthesize_controller(g)
inline Controller synthesize_controller_accelerated(const Transitions& g) {
    const int n_states = dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2];
    Controller controller;
    controller.rank.resize(n_states + 1);
    controller.command.resize(n_states + 1);
    for (int cell = 0; cell <= n_states; ++cell) {
        controller.rank[cell].fill(-1);
        controller.command[cell].fill(0);
    }

    const int success = static_cast<int>(Mission::SUCCESS);
    for (int cell = 1; cell <= n_states; ++cell) {
        controller.rank[cell][success] = 0;
    }

    const int failure = static_cast<int>(Mission::FAILURE);
    for (int cell = 1; cell <= n_states; ++cell) {
        controller.rank[cell][failure] = -1;
    }

    const Transitions g_safe = filter_safe_transitions(g);

    // Precalculate labels once using Omar's box definitions with static persistence:
    static const auto R1 = boxToCells(4, 5, 8.5, 9.5);
    static const auto R2 = boxToCells(8.5, 9.5, 2, 3);
    static const auto R3 = boxToCells(2, 3, 0.5, 1.5);
    static const auto R4 = boxToCells(3, 7, 3, 7);

    std::vector<Region> cell_label(n_states + 1, Region::NONE);
    for (int cell = 1; cell <= n_states; ++cell) {
        if (std::binary_search(R4.begin(), R4.end(), cell)) {
            cell_label[cell] = Region::R4;
        } else if (std::binary_search(R1.begin(), R1.end(), cell)) {
            cell_label[cell] = Region::R1;
        } else if (std::binary_search(R2.begin(), R2.end(), cell)) {
            cell_label[cell] = Region::R2;
        } else if (std::binary_search(R3.begin(), R3.end(), cell)) {
            cell_label[cell] = Region::R3;
        }
    }

    // Backward reachability fixed-point loop
    for (int k = 1; ; ++k) {
        const auto previous_rank = controller.rank;
        bool changed = false;

        for (int m = 0; m < 3; ++m) {
            std::vector<int> A;
            for (int next_cell = 1; next_cell <= n_states; ++next_cell) {
                Mission next_mission = update_mission(static_cast<Mission>(m), cell_label[next_cell]);
                int next_m = static_cast<int>(next_mission);
                if (previous_rank[next_cell][next_m] >= 0) {
                    A.push_back(next_cell);
                }
            }

            const auto predecessors = reach::pred(g_safe, A);
            for (const auto& choice : predecessors) {
                int cell = choice.state;
                if (previous_rank[cell][m] != -1) {
                    continue;
                }
                controller.rank[cell][m] = k;
                controller.command[cell][m] = choice.command;
                changed = true;
            }
        }

        if (!changed) {
            break;
        }
    }

    return controller;
}

// Retrieves or computes the controller (using cache file if available)
inline Controller get_or_synthesize_controller(const Transitions& g,
                                               const std::string& cache_file = "controller_cache.bin",
                                               bool force_recompute = false) {
    Controller ctrl;
    if (!force_recompute && load_controller(ctrl, cache_file)) {
        std::cout << "[Controller Cache] Loaded precomputed controller from " << cache_file << "\n";
        return ctrl;
    }

    std::cout << "[Controller Synthesis] Synthesizing controller from abstraction...\n";
    auto t0 = std::chrono::steady_clock::now();
    ctrl = synthesize_controller_accelerated(g);
    auto t1 = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "[Controller Synthesis] Completed in " << dt << " seconds.\n";

    if (save_controller(ctrl, cache_file)) {
        std::cout << "[Controller Cache] Saved controller to " << cache_file << "\n";
    }
    return ctrl;
}

} // namespace sim_io
