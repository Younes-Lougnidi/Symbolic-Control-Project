#pragma once

// Independent verification for the building temperature regulation project (Rayane Khaldi).
//
// Core checks do not call dyn::f(), abstraction::encode_cell(),
// abstraction::decode_command(), abstraction::locate_cell(), simulation::is_safe(),
// or all_successors_in_candidate(). The dynamics equation, base-10/base-3 indexing,
// interval intersections, and [17, 22] bounds are re-implemented directly from the
// problem definition so a bug in an upstream module cannot mask itself.
//
// Conventions shared with the other modules:
//   - VEC_N = std::vector<double>, Bounds = std::pair<VEC_N, VEC_N> (dynamics/dynamics.hpp)
//   - 0-based state IDs (0 .. 10^N - 1) and 0-based command IDs (0 .. 3^N - 1),
//     room 0 changing fastest; command ID 0 is all heaters off.
//   - simulation::Controller = std::vector<std::vector<abstraction::Id>>:
//     empty vector = uncertified cell; non-empty = certified command IDs.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../dynamics/dynamics.hpp"
#include "../abstraction/abstraction.hpp"
#include "../simulation/simulation.hpp"

namespace integ {

// ---------------------------------------------------------------------------
// Reference constants from the problem statement
// ---------------------------------------------------------------------------

constexpr double REF_T_LO = 17.0;
constexpr double REF_T_HI = 22.0;
constexpr double REF_U_LO = 0.0;
constexpr double REF_U_HI = 0.6;
constexpr int REF_CPR = 10;
constexpr int REF_NU = 3;
constexpr double REF_D_T = (REF_T_HI - REF_T_LO) / REF_CPR;       // 0.5
constexpr double REF_D_U = (REF_U_HI - REF_U_LO) / (REF_NU - 1);  // 0.3

constexpr double REF_TE = -1.0;
constexpr double REF_TH = 50.0;
constexpr double REF_ALPHA = 0.45;
constexpr double REF_BETA = 0.045;
constexpr double REF_GAMMA = 0.09;

// ---------------------------------------------------------------------------
// Independent math & indexing helpers
// ---------------------------------------------------------------------------

inline bool reference_pow(int base, int exp, abstraction::Id& out) {
    if (base < 1 || exp < 0) return false;
    abstraction::Id result = 1;
    const abstraction::Id limit = std::numeric_limits<abstraction::Id>::max();
    for (int i = 0; i < exp; ++i) {
        if (result > limit / base) return false;
        result *= base;
    }
    out = result;
    return true;
}

inline bool reference_is_safe(const VEC_N& T) {
    if (T.empty()) return false;
    for (double val : T) {
        if (!std::isfinite(val) || val < REF_T_LO || val > REF_T_HI) {
            return false;
        }
    }
    return true;
}

// Re-implements T_i^+ = T_i + alpha*(T_{i-1} + T_{i+1} - 2*T_i) + beta*(Te - T_i)
//                       + gamma*(Th - T_i)*u_i  with circular indexing.
inline VEC_N reference_step(const VEC_N& T, const VEC_N& u) {
    const std::size_t N = T.size();
    VEC_N next(N, 0.0);
    for (std::size_t i = 0; i < N; ++i) {
        const std::size_t prev = (i + N - 1) % N;
        const std::size_t succ = (i + 1) % N;
        next[i] = T[i]
                + REF_ALPHA * (T[prev] + T[succ] - 2.0 * T[i])
                + REF_BETA * (REF_TE - T[i])
                + REF_GAMMA * (REF_TH - T[i]) * u[i];
    }
    return next;
}

// Independent base-10 cell encoding: sum_i b_i * 10^i (room 0 fastest).
inline bool reference_encode_cell(const abstraction::Cell& bins, abstraction::Id& out_id) {
    if (bins.empty()) return false;
    abstraction::Id id = 0;
    abstraction::Id weight = 1;
    const abstraction::Id limit = std::numeric_limits<abstraction::Id>::max();
    for (std::size_t i = 0; i < bins.size(); ++i) {
        if (bins[i] < 0 || bins[i] >= REF_CPR) return false;
        if (bins[i] > 0 && weight > (limit - id) / bins[i]) return false;
        id += static_cast<abstraction::Id>(bins[i]) * weight;
        if (i + 1 < bins.size()) {
            if (weight > limit / REF_CPR) return false;
            weight *= REF_CPR;
        }
    }
    out_id = id;
    return true;
}

// Independent base-10 cell decoding: 0-based state ID -> 0-based bins in 0..9.
inline bool reference_decode_cell(abstraction::Id state, int N, abstraction::Cell& out_bins) {
    if (N < 1 || state < 0) return false;
    abstraction::Id num_states = 0;
    if (!reference_pow(REF_CPR, N, num_states) || state >= num_states) return false;
    out_bins.assign(static_cast<std::size_t>(N), 0);
    abstraction::Id rem = state;
    for (int i = 0; i < N; ++i) {
        out_bins[static_cast<std::size_t>(i)] = static_cast<int>(rem % REF_CPR);
        rem /= REF_CPR;
    }
    return true;
}

// Independent base-3 command decoding: 0-based command ID -> {0.0, 0.3, 0.6}^N.
inline bool reference_decode_command(abstraction::Id cmd_id, int N, VEC_N& out_u) {
    if (N < 1 || cmd_id < 0) return false;
    abstraction::Id num_cmds = 0;
    if (!reference_pow(REF_NU, N, num_cmds) || cmd_id >= num_cmds) return false;
    out_u.assign(static_cast<std::size_t>(N), 0.0);
    abstraction::Id rem = cmd_id;
    for (int i = 0; i < N; ++i) {
        const int digit = static_cast<int>(rem % REF_NU);
        out_u[static_cast<std::size_t>(i)] = REF_U_LO + digit * REF_D_U;
        rem /= REF_NU;
    }
    return true;
}

// Independent temperature -> cell ID lookup.
// Internal boundaries go to the upper bin; 22.0 belongs to bin 9; outside [17,22]
// or non-finite values are rejected (never clamped).
inline bool reference_locate_cell(const VEC_N& T, abstraction::Id& out_cell) {
    if (!reference_is_safe(T)) return false;
    abstraction::Cell bins(T.size(), 0);
    for (std::size_t i = 0; i < T.size(); ++i) {
        int bin = static_cast<int>(std::floor((T[i] - REF_T_LO) / REF_D_T));
        if (bin == REF_CPR) bin = REF_CPR - 1;  // T[i] == 22.0 -> bin 9
        if (bin < 0 || bin >= REF_CPR) return false;
        bins[i] = bin;
    }
    return reference_encode_cell(bins, out_cell);
}

// ---------------------------------------------------------------------------
// 1. Independent controller verification
// ---------------------------------------------------------------------------

struct ControllerAudit {
    abstraction::Id total_states = 0;
    abstraction::Id certified_states = 0;
    abstraction::Id certified_pairs = 0;
    abstraction::Id successors_checked = 0;
    abstraction::Id violations = 0;
    std::vector<std::string> first_violations;
    bool ok = false;
};

inline ControllerAudit audit_controller(const abstraction::Abstraction& model,
                                        const simulation::Controller& controller) {
    ControllerAudit audit;
    auto record_violation = [&](const std::string& msg) {
        if (audit.first_violations.size() < 10) {
            audit.first_violations.push_back(msg);
        }
        ++audit.violations;
    };

    if (model.N < 3) {
        record_violation("model.N < 3");
        return audit;
    }

    abstraction::Id expected_states = 0;
    abstraction::Id expected_commands = 0;
    if (!reference_pow(REF_CPR, model.N, expected_states) ||
        !reference_pow(REF_NU, model.N, expected_commands)) {
        record_violation("grid dimensions overflow 64-bit ID range");
        return audit;
    }

    audit.total_states = expected_states;

    if (model.num_states != expected_states) {
        record_violation("model.num_states (" + std::to_string(model.num_states) +
                         ") != 10^N (" + std::to_string(expected_states) + ")");
        return audit;
    }
    if (model.num_commands != expected_commands) {
        record_violation("model.num_commands (" + std::to_string(model.num_commands) +
                         ") != 3^N (" + std::to_string(expected_commands) + ")");
        return audit;
    }
    if (expected_states > std::numeric_limits<abstraction::Id>::max() / expected_commands ||
        model.transitions.size() != static_cast<std::size_t>(expected_states * expected_commands)) {
        record_violation("model.transitions.size() does not equal num_states * num_commands");
        return audit;
    }
    if (controller.size() != static_cast<std::size_t>(expected_states)) {
        record_violation("controller.size() (" + std::to_string(controller.size()) +
                         ") != num_states (" + std::to_string(expected_states) + ")");
        return audit;
    }

    // Precompute base-10 weights for fast independent Cartesian-box traversal.
    std::vector<abstraction::Id> weights(static_cast<std::size_t>(model.N), 1);
    for (int i = 1; i < model.N; ++i) {
        weights[static_cast<std::size_t>(i)] = weights[static_cast<std::size_t>(i - 1)] * REF_CPR;
    }

    for (abstraction::Id s = 0; s < expected_states; ++s) {
        const auto& cmds = controller[static_cast<std::size_t>(s)];
        if (cmds.empty()) continue;

        ++audit.certified_states;

        // Check for duplicate command IDs in controller[s].
        std::vector<abstraction::Id> seen_cmds = cmds;
        std::sort(seen_cmds.begin(), seen_cmds.end());
        if (std::adjacent_find(seen_cmds.begin(), seen_cmds.end()) != seen_cmds.end()) {
            record_violation("state " + std::to_string(s) + " contains duplicate command IDs");
        }

        for (abstraction::Id u : cmds) {
            if (u < 0 || u >= expected_commands) {
                record_violation("state " + std::to_string(s) + " has out-of-range command ID " +
                                 std::to_string(u));
                continue;
            }
            ++audit.certified_pairs;

            const auto& box = model.transitions[static_cast<std::size_t>(s * expected_commands + u)];
            if (box.leaves_safe_set) {
                record_violation("state " + std::to_string(s) + ", command " + std::to_string(u) +
                                 " has leaves_safe_set == true");
                continue;
            }
            if (static_cast<int>(box.lower.size()) != model.N ||
                static_cast<int>(box.upper.size()) != model.N) {
                record_violation("state " + std::to_string(s) + ", command " + std::to_string(u) +
                                 " has malformed successor box dimensions");
                continue;
            }

            bool bounds_valid = true;
            for (int r = 0; r < model.N; ++r) {
                const int lo = box.lower[static_cast<std::size_t>(r)];
                const int hi = box.upper[static_cast<std::size_t>(r)];
                if (lo < 0 || hi >= REF_CPR || lo > hi) {
                    record_violation("state " + std::to_string(s) + ", command " + std::to_string(u) +
                                     " has invalid bin bounds [" + std::to_string(lo) + ", " +
                                     std::to_string(hi) + "] in room " + std::to_string(r));
                    bounds_valid = false;
                    break;
                }
            }
            if (!bounds_valid) continue;

            // Enumerate the full Cartesian box of successor cells independently.
            abstraction::Cell bins = box.lower;
            while (true) {
                abstraction::Id succ = 0;
                for (int r = 0; r < model.N; ++r) {
                    succ += static_cast<abstraction::Id>(bins[static_cast<std::size_t>(r)]) *
                            weights[static_cast<std::size_t>(r)];
                }
                ++audit.successors_checked;
                if (controller[static_cast<std::size_t>(succ)].empty()) {
                    record_violation("state " + std::to_string(s) + ", command " + std::to_string(u) +
                                     " reaches uncertified successor cell " + std::to_string(succ));
                    break;  // one witness per (s, u) suffices to flag violation
                }

                int room = 0;
                while (room < model.N &&
                       bins[static_cast<std::size_t>(room)] == box.upper[static_cast<std::size_t>(room)]) {
                    bins[static_cast<std::size_t>(room)] = box.lower[static_cast<std::size_t>(room)];
                    ++room;
                }
                if (room == model.N) break;
                ++bins[static_cast<std::size_t>(room)];
            }
        }
    }

    audit.ok = (audit.violations == 0);
    return audit;
}

// Returns true iff every certified command stays inside the safe set and every
// cell in its successor box is also certified.
inline bool verify_controller(const abstraction::Abstraction& model,
                              const simulation::Controller& controller) {
    return audit_controller(model, controller).ok;
}

// ---------------------------------------------------------------------------
// 2. Independent Exhaustive Abstraction Audit
// ---------------------------------------------------------------------------

struct AbstractionAudit {
    abstraction::Id pairs_checked = 0;
    abstraction::Id mismatches = 0;
    std::vector<std::string> first_mismatches;
    bool ok = false;
};

// Recomputes every (state, command) transition from scratch using the brief's
// physical equation on corner bounds and closed-interval intersection, then
// compares against the stored Abstraction table.
inline AbstractionAudit audit_abstraction(const abstraction::Abstraction& model) {
    AbstractionAudit audit;
    auto record = [&](const std::string& msg) {
        if (audit.first_mismatches.size() < 10) {
            audit.first_mismatches.push_back(msg);
        }
        ++audit.mismatches;
    };

    if (model.N < 3) {
        record("model.N < 3");
        return audit;
    }

    abstraction::Id expected_states = 0;
    abstraction::Id expected_commands = 0;
    if (!reference_pow(REF_CPR, model.N, expected_states) ||
        !reference_pow(REF_NU, model.N, expected_commands)) {
        record("dimension overflow");
        return audit;
    }
    if (model.num_states != expected_states || model.num_commands != expected_commands ||
        model.transitions.size() != static_cast<std::size_t>(expected_states * expected_commands)) {
        record("abstraction dimensions do not match 10^N x 3^N");
        return audit;
    }

    VEC_N lo(static_cast<std::size_t>(model.N));
    VEC_N hi(static_cast<std::size_t>(model.N));
    abstraction::Cell bins;
    VEC_N u;

    for (abstraction::Id s = 0; s < expected_states; ++s) {
        reference_decode_cell(s, model.N, bins);
        for (int r = 0; r < model.N; ++r) {
            lo[static_cast<std::size_t>(r)] = REF_T_LO + bins[static_cast<std::size_t>(r)] * REF_D_T;
            hi[static_cast<std::size_t>(r)] = lo[static_cast<std::size_t>(r)] + REF_D_T;
        }

        for (abstraction::Id c = 0; c < expected_commands; ++c) {
            reference_decode_command(c, model.N, u);
            const VEC_N next_lo = reference_step(lo, u);
            const VEC_N next_hi = reference_step(hi, u);

            bool expected_exit = false;
            abstraction::Cell exp_lo(static_cast<std::size_t>(model.N), -1);
            abstraction::Cell exp_hi(static_cast<std::size_t>(model.N), -1);

            for (int r = 0; r < model.N; ++r) {
                const double nl = next_lo[static_cast<std::size_t>(r)];
                const double nh = next_hi[static_cast<std::size_t>(r)];
                if (nl > nh + 1e-12) {
                    record("non-monotone corner bounds at state " + std::to_string(s) +
                           ", cmd " + std::to_string(c));
                }
                if (nl < REF_T_LO || nh > REF_T_HI) {
                    expected_exit = true;
                }
                for (int bin = 0; bin < REF_CPR; ++bin) {
                    const double cell_lo = REF_T_LO + bin * REF_D_T;
                    const double cell_hi = cell_lo + REF_D_T;
                    if (nl <= cell_hi && nh >= cell_lo) {
                        if (exp_lo[static_cast<std::size_t>(r)] == -1) {
                            exp_lo[static_cast<std::size_t>(r)] = bin;
                        }
                        exp_hi[static_cast<std::size_t>(r)] = bin;
                    }
                }
            }

            const auto& got = model.transitions[static_cast<std::size_t>(s * expected_commands + c)];
            ++audit.pairs_checked;
            if (got.leaves_safe_set != expected_exit ||
                got.lower != exp_lo ||
                got.upper != exp_hi) {
                record("transition mismatch at state " + std::to_string(s) +
                       ", cmd " + std::to_string(c));
            }
        }
    }

    audit.ok = (audit.mismatches == 0);
    return audit;
}

// ---------------------------------------------------------------------------
// 3. Independent numerical trajectory verification
// ---------------------------------------------------------------------------

struct TrajectoryAudit {
    bool all_temps_safe = true;       // independent check of every recorded T[k][i] in [17,22]
    bool shapes_ok = true;            // K+1 temperatures for K commands, all vectors of length N
    bool dynamics_ok = true;          // T[k+1] == reference_step(T[k], u[k])
    bool policy_ok = true;            // cmd[k] in controller[cell(T[k])] and deterministic choice
    bool flags_consistent = true;     // res.safe and res.uncertified match independent findings
    double min_temp = std::numeric_limits<double>::infinity();
    double max_temp = -std::numeric_limits<double>::infinity();
    double max_dyn_error = 0.0;
    std::size_t steps_checked = 0;
    std::vector<std::string> issues;

    bool ok() const {
        return shapes_ok && dynamics_ok && policy_ok && flags_consistent && issues.empty();
    }
};

// Independently inspects all recorded numerical temperatures and commands in `res`.
// Never trusts `res.safe` or `res.uncertified` without verifying them directly.
inline TrajectoryAudit check_trajectory(const simulation::SimulationResult& res,
                                        const simulation::Controller& controller,
                                        int rooms,
                                        int requested_steps) {
    TrajectoryAudit audit;
    const std::size_t K = res.commands.size();
    audit.steps_checked = K;

    if (res.temperatures.empty()) {
        audit.shapes_ok = false;
        audit.all_temps_safe = false;
        audit.issues.push_back("no temperatures recorded");
        return audit;
    }
    if (res.temperatures.size() != K + 1) {
        audit.shapes_ok = false;
        audit.issues.push_back("temperatures.size() (" + std::to_string(res.temperatures.size()) +
                               ") != commands.size() + 1 (" + std::to_string(K + 1) + ")");
    }
    for (std::size_t k = 0; k < res.temperatures.size(); ++k) {
        if (static_cast<int>(res.temperatures[k].size()) != rooms) {
            audit.shapes_ok = false;
            audit.issues.push_back("temperature vector at step " + std::to_string(k) +
                                   " has size " + std::to_string(res.temperatures[k].size()) +
                                   " != " + std::to_string(rooms));
        }
    }
    if (!audit.shapes_ok) return audit;

    // 1. Direct numerical bounds check on EVERY recorded temperature value.
    for (std::size_t k = 0; k < res.temperatures.size(); ++k) {
        for (int r = 0; r < rooms; ++r) {
            const double val = res.temperatures[k][static_cast<std::size_t>(r)];
            if (!std::isfinite(val)) {
                audit.all_temps_safe = false;
            } else {
                audit.min_temp = std::min(audit.min_temp, val);
                audit.max_temp = std::max(audit.max_temp, val);
                if (val < REF_T_LO || val > REF_T_HI) {
                    audit.all_temps_safe = false;
                }
            }
        }
    }

    // 2. Check res.safe against independent numerical check.
    if (res.safe != audit.all_temps_safe) {
        audit.flags_consistent = false;
        audit.issues.push_back(std::string("simulator safe flag is ") +
                               (res.safe ? "true" : "false") +
                               " but direct temperature check gives " +
                               (audit.all_temps_safe ? "true" : "false"));
    }

    // 3. Replay every executed command and verify physical dynamics + policy membership.
    for (std::size_t k = 0; k < K; ++k) {
        const VEC_N& cur = res.temperatures[k];
        const VEC_N& nxt = res.temperatures[k + 1];
        const abstraction::Id cmd_id = res.commands[k];

        abstraction::Id cell = -1;
        if (!reference_locate_cell(cur, cell)) {
            audit.policy_ok = false;
            audit.issues.push_back("step " + std::to_string(k) +
                                   ": command applied from unsafe/out-of-range temperature");
            continue;
        }
        if (cell < 0 || static_cast<std::size_t>(cell) >= controller.size() ||
            controller[static_cast<std::size_t>(cell)].empty()) {
            audit.policy_ok = false;
            audit.issues.push_back("step " + std::to_string(k) +
                                   ": command applied from uncertified cell " + std::to_string(cell));
        } else {
            const auto& allowed = controller[static_cast<std::size_t>(cell)];
            if (std::find(allowed.begin(), allowed.end(), cmd_id) == allowed.end()) {
                audit.policy_ok = false;
                audit.issues.push_back("step " + std::to_string(k) +
                                       ": command " + std::to_string(cmd_id) +
                                       " is not certified for cell " + std::to_string(cell));
            } else if (allowed[0] != cmd_id) {
                audit.policy_ok = false;
                audit.issues.push_back("step " + std::to_string(k) +
                                       ": command " + std::to_string(cmd_id) +
                                       " differs from deterministic first choice " +
                                       std::to_string(allowed[0]));
            }
        }

        VEC_N u;
        if (!reference_decode_command(cmd_id, rooms, u)) {
            audit.policy_ok = false;
            audit.issues.push_back("step " + std::to_string(k) +
                                   ": invalid command ID " + std::to_string(cmd_id));
            continue;
        }

        const VEC_N expected_next = reference_step(cur, u);
        for (int r = 0; r < rooms; ++r) {
            const double err = std::fabs(expected_next[static_cast<std::size_t>(r)] -
                                         nxt[static_cast<std::size_t>(r)]);
            audit.max_dyn_error = std::max(audit.max_dyn_error, err);
            if (!std::isfinite(err) || err > 1e-12) {
                audit.dynamics_ok = false;
                audit.issues.push_back("step " + std::to_string(k) + ", room " + std::to_string(r) +
                                       ": dynamics mismatch (err=" + std::to_string(err) + ")");
                break;
            }
        }
    }

    // 4. Verify stopping / uncertified consistency.
    const std::size_t expected_steps = requested_steps > 0 ? static_cast<std::size_t>(requested_steps) : 0;
    if (res.uncertified) {
        // Must have stopped because the last recorded state mapped to an uncertified cell.
        abstraction::Id last_cell = -1;
        const bool located = reference_locate_cell(res.temperatures.back(), last_cell);
        const bool is_uncert = !located ||
                               last_cell < 0 ||
                               static_cast<std::size_t>(last_cell) >= controller.size() ||
                               controller[static_cast<std::size_t>(last_cell)].empty();
        if (!is_uncert) {
            audit.flags_consistent = false;
            audit.issues.push_back("res.uncertified is true, but final cell " +
                                   std::to_string(last_cell) + " has certified commands");
        }
    } else if (audit.all_temps_safe) {
        if (K != expected_steps) {
            audit.flags_consistent = false;
            audit.issues.push_back("safe certified run completed " + std::to_string(K) +
                                   " steps instead of requested " + std::to_string(expected_steps));
        }
    }

    return audit;
}

// ---------------------------------------------------------------------------
// 4. Independent CSV trace verifier (checks Amine's exported traces)
// ---------------------------------------------------------------------------

struct CsvTraceAudit {
    std::string path;
    bool file_found = false;
    std::size_t temp_rows = 0;
    std::size_t cmd_rows = 0;
    bool all_temps_safe = true;
    bool dynamics_ok = true;
    bool commands_ok = true;
    bool replay_matches_live = true;
    double min_temp = std::numeric_limits<double>::infinity();
    double max_temp = -std::numeric_limits<double>::infinity();
    double max_step_error = 0.0;
    double max_live_diff = 0.0;
    std::vector<std::string> issues;

    bool ok() const {
        return file_found && all_temps_safe && dynamics_ok && commands_ok &&
               replay_matches_live && issues.empty();
    }
};

inline std::vector<std::string> split_csv_line(const std::string& raw) {
    std::string line = raw;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::vector<std::string> cols;
    std::size_t start = 0;
    while (true) {
        const std::size_t pos = line.find(',', start);
        if (pos == std::string::npos) {
            cols.push_back(line.substr(start));
            break;
        }
        cols.push_back(line.substr(start, pos - start));
        start = pos + 1;
    }
    return cols;
}

// Verifies one of Amine's exported CSV traces against:
//   1. K+1 temperature rows and K commands for K = expected_steps
//   2. Every temperature entry satisfies 17.0 <= T_i <= 22.0 (finite)
//   3. u_i matches base-3 decoding of cmd_id, cmd_id is certified in controller
//   4. Every step satisfies T(k+1) = f_ref(T(k), u(k)) within 5-decimal CSV rounding
//   5. Live re-simulation from T(0) agrees with every row in the CSV
inline CsvTraceAudit verify_csv_trace(const std::string& path,
                                      const simulation::Controller& controller,
                                      int rooms,
                                      int expected_steps) {
    CsvTraceAudit audit;
    audit.path = path;

    std::ifstream in(path);
    if (!in.is_open()) {
        audit.issues.push_back("could not open CSV file: " + path);
        return audit;
    }
    audit.file_found = true;

    std::string line;
    bool header_seen = false;
    std::vector<VEC_N> temps;
    std::vector<abstraction::Id> cmds;
    std::vector<VEC_N> csv_u;
    const std::size_t expected_cols = static_cast<std::size_t>(2 * rooms + 2);

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        if (!header_seen) {
            header_seen = true;
            continue;
        }
        const auto cols = split_csv_line(line);
        if (cols.size() != expected_cols) {
            audit.issues.push_back("row " + std::to_string(temps.size()) + " has " +
                                   std::to_string(cols.size()) + " columns, expected " +
                                   std::to_string(expected_cols));
            return audit;
        }
        const int t_idx = std::stoi(cols[0]);
        if (t_idx != static_cast<int>(temps.size())) {
            audit.issues.push_back("non-sequential step index " + std::to_string(t_idx));
            return audit;
        }

        VEC_N T(static_cast<std::size_t>(rooms), 0.0);
        for (int r = 0; r < rooms; ++r) {
            T[static_cast<std::size_t>(r)] = std::stod(cols[static_cast<std::size_t>(1 + r)]);
        }
        temps.push_back(T);

        const std::string& cmd_col = cols[static_cast<std::size_t>(1 + rooms)];
        if (!cmd_col.empty()) {
            const abstraction::Id cid = std::stoll(cmd_col);
            VEC_N u(static_cast<std::size_t>(rooms), 0.0);
            for (int r = 0; r < rooms; ++r) {
                u[static_cast<std::size_t>(r)] =
                    std::stod(cols[static_cast<std::size_t>(2 + rooms + r)]);
            }
            cmds.push_back(cid);
            csv_u.push_back(u);
        }
    }

    audit.temp_rows = temps.size();
    audit.cmd_rows = cmds.size();

    if (static_cast<int>(audit.cmd_rows) != expected_steps ||
        static_cast<int>(audit.temp_rows) != expected_steps + 1) {
        audit.issues.push_back("expected " + std::to_string(expected_steps + 1) +
                               " temperature rows and " + std::to_string(expected_steps) +
                               " commands, got " + std::to_string(audit.temp_rows) + " / " +
                               std::to_string(audit.cmd_rows));
        return audit;
    }

    // Check every recorded temperature directly.
    for (std::size_t k = 0; k < temps.size(); ++k) {
        for (int r = 0; r < rooms; ++r) {
            const double val = temps[k][static_cast<std::size_t>(r)];
            if (!std::isfinite(val)) {
                audit.all_temps_safe = false;
                audit.issues.push_back("non-finite temperature at row " + std::to_string(k));
            } else {
                audit.min_temp = std::min(audit.min_temp, val);
                audit.max_temp = std::max(audit.max_temp, val);
                if (val < REF_T_LO || val > REF_T_HI) {
                    audit.all_temps_safe = false;
                    audit.issues.push_back("temperature out of [17,22] at row " + std::to_string(k));
                }
            }
        }
    }

    // Check commands and one-step physical dynamics (CSV is formatted with 5 decimals).
    constexpr double CSV_TOL = 5e-5;
    for (std::size_t k = 0; k < cmds.size(); ++k) {
        VEC_N decoded_u;
        if (!reference_decode_command(cmds[k], rooms, decoded_u)) {
            audit.commands_ok = false;
            audit.issues.push_back("invalid cmd_id at step " + std::to_string(k));
            continue;
        }
        for (int r = 0; r < rooms; ++r) {
            if (std::fabs(decoded_u[static_cast<std::size_t>(r)] -
                          csv_u[k][static_cast<std::size_t>(r)]) > CSV_TOL) {
                audit.commands_ok = false;
                audit.issues.push_back("CSV u column disagrees with cmd_id at step " +
                                       std::to_string(k));
            }
        }

        abstraction::Id cell = -1;
        if (!reference_locate_cell(temps[k], cell) ||
            cell < 0 || static_cast<std::size_t>(cell) >= controller.size() ||
            std::find(controller[static_cast<std::size_t>(cell)].begin(),
                      controller[static_cast<std::size_t>(cell)].end(),
                      cmds[k]) == controller[static_cast<std::size_t>(cell)].end()) {
            audit.commands_ok = false;
            audit.issues.push_back("cmd_id " + std::to_string(cmds[k]) +
                                   " not certified for cell at step " + std::to_string(k));
        }

        const VEC_N expected_next = reference_step(temps[k], decoded_u);
        for (int r = 0; r < rooms; ++r) {
            const double err = std::fabs(expected_next[static_cast<std::size_t>(r)] -
                                         temps[k + 1][static_cast<std::size_t>(r)]);
            audit.max_step_error = std::max(audit.max_step_error, err);
            if (err > CSV_TOL) {
                audit.dynamics_ok = false;
                audit.issues.push_back("step " + std::to_string(k) +
                                       ": CSV T(k+1) disagrees with f(T(k),u(k)) by " +
                                       std::to_string(err));
            }
        }
    }

    // Live full-precision re-simulation from T(0) must match the CSV trace.
    const auto live = simulation::simulate(controller, temps[0], expected_steps);
    if (!live.safe || live.uncertified || live.commands != cmds ||
        live.temperatures.size() != temps.size()) {
        audit.replay_matches_live = false;
        audit.issues.push_back("live re-simulation from T(0) disagrees with CSV commands/length");
    } else {
        for (std::size_t k = 0; k < temps.size(); ++k) {
            for (int r = 0; r < rooms; ++r) {
                const double diff = std::fabs(live.temperatures[k][static_cast<std::size_t>(r)] -
                                              temps[k][static_cast<std::size_t>(r)]);
                audit.max_live_diff = std::max(audit.max_live_diff, diff);
                if (diff > 1e-4) {
                    audit.replay_matches_live = false;
                    audit.issues.push_back("live replay differs from CSV at step " +
                                           std::to_string(k) + " by " + std::to_string(diff));
                }
            }
        }
    }

    return audit;
}

}  // namespace integ
