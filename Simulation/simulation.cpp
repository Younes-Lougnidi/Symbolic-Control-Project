#include "simulation.hpp"

#include <fstream>
#include <iomanip>
#include <random>

SimulationResult simulate(const Controller& controller,
                          const std::vector<Vec2>& val_u,
                          const Vec3& initial_state,
                          int max_steps,
                          unsigned seed) {
    return simulate_with_step(controller, val_u, initial_state, max_steps, seed, controller_step);
}

SimulationResult simulate_with_step(const Controller& controller,
                                    const std::vector<Vec2>& val_u,
                                    const Vec3& initial_state,
                                    int max_steps,
                                    unsigned seed,
                                    const StepFunction& step_fn) {
    SimulationResult result;
    result.seed = seed;

    // Initialize random generator ONCE per simulation run
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist_w0(dyn::bound_w_lo[0], dyn::bound_w_hi[0]);
    std::uniform_real_distribution<double> dist_w1(dyn::bound_w_lo[1], dyn::bound_w_hi[1]);
    std::uniform_real_distribution<double> dist_w2(dyn::bound_w_lo[2], dyn::bound_w_hi[2]);

    Vec3 state = initial_state;
    Mission previous_mission = Mission::START;

    for (;;) {
        // 1. Observe state and evaluate policy
        ControlStep step = step_fn(controller, val_u, state, previous_mission);

        // Record observed state and resulting mission memory
        result.states.push_back(state);
        result.missions.push_back(step.mission);

        // 2. Stop immediately if the controller returned a terminal status
        if (step.status == Status::SUCCESS) {
            result.reason = StopReason::SUCCESS;
            return result;
        }
        if (step.status == Status::FAILURE) {
            result.reason = StopReason::FAILURE;
            return result;
        }
        if (step.status == Status::UNCERTIFIED) {
            result.reason = StopReason::UNCERTIFIED;
            return result;
        }

        // 3. Check step limit after evaluating current state
        // This guarantees arrival on the last permitted movement is reported as SUCCESS.
        if (static_cast<int>(result.velocities.size()) >= max_steps) {
            result.reason = StopReason::STEP_LIMIT;
            return result;
        }

        // 4. Sample bounded disturbance uniformly in [bound_w_lo, bound_w_hi]
        Vec3 disturbance = {dist_w0(rng), dist_w1(rng), dist_w2(rng)};

        // 5. Apply robot dynamics f(x, u, w)
        Vec3 next_state = dyn::f(state, step.velocities, disturbance);

        // 6. Record applied velocities and disturbance
        result.velocities.push_back(step.velocities);
        result.disturbances.push_back(disturbance);

        // Advance state and update previous mission for next iteration
        state = next_state;
        previous_mission = step.mission;
    }
}

bool export_simulation_to_csv(const SimulationResult& result,
                              const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out) {
        return false;
    }

    // Commented metadata header
    out << "# seed: " << result.seed << "\n";
    out << "# reason: " << to_string(result.reason) << "\n";
    if (!result.states.empty()) {
        out << std::fixed << std::setprecision(6);
        out << "# initial_state: " << result.states[0][0] << ", "
            << result.states[0][1] << ", " << result.states[0][2] << "\n";
    }
    out << "# movements: " << result.velocities.size() << "\n";

    // Column headers
    out << "t,x1,x2,x3,mission,u1,u2,w1,w2,w3\n";

    // Observation rows
    out << std::fixed << std::setprecision(6);
    for (size_t t = 0; t < result.states.size(); ++t) {
        out << t << ","
            << result.states[t][0] << ","
            << result.states[t][1] << ","
            << result.states[t][2] << ","
            << to_string(result.missions[t]) << ",";

        if (t < result.velocities.size()) {
            out << result.velocities[t][0] << ","
                << result.velocities[t][1] << ","
                << result.disturbances[t][0] << ","
                << result.disturbances[t][1] << ","
                << result.disturbances[t][2] << "\n";
        } else {
            // Final observation: u and w columns are empty
            out << ",,,,\n";
        }
    }

    return true;
}
