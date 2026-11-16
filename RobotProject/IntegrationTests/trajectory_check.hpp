#pragma once

// Independent checks on recorded simulation runs (Rayane Khaldi, integration).
//
// Nothing here calls label() or update_mission(): the regions are rebuilt from
// the article's coordinates and the mission is rebuilt from "what has been
// visited" flags. So a bug in Omar's labels or rules cannot hide itself.
//
// Convention (same as the project): grid cells are half-open [a, b), regions are
// grid-aligned, a state belongs to a region when x1 and x2 are in [lo, hi).
// A state exactly on a region edge is flagged in boundary_touch.

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "../robotControl/dynamics.hpp"
#include "../mission-rules/labeling&MissionRules.hpp"
#include "../ControllerSynthesis/ControllerSythesis.hpp"
#include "../ControllerConcretization/controller_step.hpp"
#include "../Simulation/simulation.hpp"

namespace integ {

struct Rect {
    double x_lo, x_hi, y_lo, y_hi;
};

// Regions of the article (x1 and x2 bounds; the heading is free).
constexpr Rect REGION_R1 = {4.0, 5.0, 8.5, 9.5};
constexpr Rect REGION_R2 = {8.5, 9.5, 2.0, 3.0};
constexpr Rect REGION_R3 = {2.0, 3.0, 0.5, 1.5};
constexpr Rect REGION_R4 = {3.0, 7.0, 3.0, 7.0};

inline bool in_rect(const Rect& r, const Vec3& s) {
    return s[0] >= r.x_lo && s[0] < r.x_hi && s[1] >= r.y_lo && s[1] < r.y_hi;
}

inline bool in_closed_rect(const Rect& r, const Vec3& s) {
    return s[0] >= r.x_lo && s[0] <= r.x_hi && s[1] >= r.y_lo && s[1] <= r.y_hi;
}

inline bool is_finite_state(const Vec3& s) {
    return std::isfinite(s[0]) && std::isfinite(s[1]) && std::isfinite(s[2]);
}

inline bool outside_workspace(const Vec3& s) {
    return !is_finite_state(s) ||
           s[0] < dyn::bound_x_lo[0] || s[0] >= dyn::bound_x_hi[0] ||
           s[1] < dyn::bound_x_lo[1] || s[1] >= dyn::bound_x_hi[1];
}

inline bool on_region_edge(const Vec3& s) {
    for (const Rect* r : {&REGION_R1, &REGION_R2, &REGION_R3, &REGION_R4}) {
        if (in_closed_rect(*r, s) && !in_rect(*r, s)) return true;
    }
    return false;
}

inline Region reference_region(const Vec3& s) {
    if (in_rect(REGION_R4, s)) return Region::R4;
    if (in_rect(REGION_R1, s)) return Region::R1;
    if (in_rect(REGION_R2, s)) return Region::R2;
    if (in_rect(REGION_R3, s)) return Region::R3;
    return Region::NONE;
}

enum class Verdict {
    SATISFIED,   // R1 xor R2, then R3, R4 never touched before that
    VIOLATED,    // both targets, R4, or left the workspace
    INCOMPLETE   // nothing violated yet, R3 not reached after a target
};

inline const char* to_string(Verdict v) {
    switch (v) {
        case Verdict::SATISFIED: return "SATISFIED";
        case Verdict::VIOLATED: return "VIOLATED";
        case Verdict::INCOMPLETE: return "INCOMPLETE";
        default: return "UNKNOWN";
    }
}

struct TrajectoryReport {
    Verdict verdict = Verdict::INCOMPLETE;
    std::vector<Mission> missions;  // mission rebuilt from flags, one per observation up to the stop
    bool visited_r1 = false;
    bool visited_r2 = false;
    bool visited_both = false;
    bool reached_r3 = false;        // R3 reached AFTER a target (completion)
    bool hit_r4 = false;
    bool left_workspace = false;
    bool boundary_touch = false;    // a state sat exactly on a region edge
    int stop_index = -1;            // index of the state that decided the verdict
    std::string summary;
};

// Reads the recorded observed states only. Stops at the first deciding state,
// like the simulation does.
inline TrajectoryReport check_trajectory(const std::vector<Vec3>& states) {
    TrajectoryReport r;
    for (std::size_t t = 0; t < states.size(); ++t) {
        const Vec3& s = states[t];
        r.stop_index = static_cast<int>(t);

        if (outside_workspace(s)) {
            r.left_workspace = true;
            r.verdict = Verdict::VIOLATED;
            r.missions.push_back(Mission::FAILURE);
            r.summary = "left the workspace at observation " + std::to_string(t);
            return r;
        }
        if (on_region_edge(s)) r.boundary_touch = true;

        const Region reg = reference_region(s);
        if (reg == Region::R4) {
            r.hit_r4 = true;
            r.verdict = Verdict::VIOLATED;
            r.missions.push_back(Mission::FAILURE);
            r.summary = "entered R4 at observation " + std::to_string(t);
            return r;
        }
        if (reg == Region::R1) r.visited_r1 = true;
        if (reg == Region::R2) r.visited_r2 = true;

        if (r.visited_r1 && r.visited_r2) {
            r.visited_both = true;
            r.verdict = Verdict::VIOLATED;
            r.missions.push_back(Mission::FAILURE);
            r.summary = "visited both R1 and R2 (second one at observation " + std::to_string(t) + ")";
            return r;
        }
        if (reg == Region::R3 && (r.visited_r1 || r.visited_r2)) {
            r.reached_r3 = true;
            r.verdict = Verdict::SATISFIED;
            r.missions.push_back(Mission::SUCCESS);
            r.summary = std::string("visited ") + (r.visited_r1 ? "R1" : "R2") +
                        " then reached R3 at observation " + std::to_string(t);
            return r;
        }
        if (r.visited_r1) r.missions.push_back(Mission::VISITED_R1);
        else if (r.visited_r2) r.missions.push_back(Mission::VISITED_R2);
        else r.missions.push_back(Mission::START);
    }
    r.summary = "no violation, mission not completed";
    return r;
}

// Everything that can be checked on one run besides the mission itself:
// shapes, controls, disturbances, exact dynamics replay, mission memory and
// strictly decreasing ranks. Returns one message per problem (empty = fine).
inline std::vector<std::string> check_run_consistency(const SimulationResult& res,
                                                      const std::vector<Vec2>& val_u,
                                                      const Controller& controller,
                                                      const TrajectoryReport& traj) {
    std::vector<std::string> issues;
    const std::size_t k = res.velocities.size();

    if (res.states.empty()) {
        issues.push_back("no observed state recorded");
        return issues;
    }
    if (res.states.size() != k + 1) issues.push_back("states.size() != movements + 1");
    if (res.missions.size() != k + 1) issues.push_back("missions.size() != movements + 1");
    if (res.disturbances.size() != k) issues.push_back("disturbances.size() != movements");
    if (!issues.empty()) return issues;

    for (std::size_t t = 0; t < k; ++t) {
        bool known = false;
        for (const Vec2& u : val_u) {
            if (u[0] == res.velocities[t][0] && u[1] == res.velocities[t][1]) known = true;
        }
        if (!known) issues.push_back("movement " + std::to_string(t) + ": velocities not in val_u");

        for (int d = 0; d < 3; ++d) {
            if (res.disturbances[t][d] < dyn::bound_w_lo[d] || res.disturbances[t][d] > dyn::bound_w_hi[d]) {
                issues.push_back("movement " + std::to_string(t) + ": disturbance out of bounds");
                break;
            }
        }

        const Vec3 expected = dyn::f(res.states[t], res.velocities[t], res.disturbances[t]);
        for (int d = 0; d < 3; ++d) {
            if (std::fabs(expected[d] - res.states[t + 1][d]) > 1e-12) {
                issues.push_back("movement " + std::to_string(t) + ": states[t+1] != f(states[t], u, w)");
                break;
            }
        }
    }

    // The recorded mission must match the mission rebuilt from the geometry.
    if (traj.missions.size() != res.missions.size()) {
        issues.push_back("trajectory continues after the independent check decided (or stops before it)");
    } else {
        for (std::size_t t = 0; t < res.missions.size(); ++t) {
            if (res.missions[t] != traj.missions[t]) {
                issues.push_back(std::string("observation ") + std::to_string(t) + ": recorded mission " +
                                 to_string(res.missions[t]) + " but geometry gives " +
                                 to_string(traj.missions[t]));
                break;
            }
        }
    }

    // Ranks must strictly decrease along the run and reach 0 only at SUCCESS.
    int previous_rank = -2;
    for (std::size_t t = 0; t < res.states.size(); ++t) {
        const int cell = numerical_state_to_cell(res.states[t]);
        if (cell == 0) break;  // outside workspace: reported by the mission check
        const int rank = controller.rank.at(cell).at(static_cast<int>(res.missions[t]));
        const bool running = t < k;
        if (running && rank <= 0) {
            issues.push_back("observation " + std::to_string(t) + ": a movement was applied from rank " +
                             std::to_string(rank));
        }
        if (previous_rank > 0 && !(rank >= 0 && rank < previous_rank)) {
            issues.push_back("observation " + std::to_string(t) + ": rank " + std::to_string(rank) +
                             " is not smaller than previous rank " + std::to_string(previous_rank));
        }
        previous_rank = rank;
    }
    if (res.reason == StopReason::SUCCESS && previous_rank != 0) {
        issues.push_back("SUCCESS reported but the final rank is not 0");
    }
    return issues;
}

}  // namespace integ
