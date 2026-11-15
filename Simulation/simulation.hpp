#pragma once

#include <vector>
#include <array>
#include <string>
#include <functional>

#include "../robotControl/dynamics.hpp"
#include "../mission-rules/labeling&MissionRules.hpp"
#include "../ControllerSynthesis/ControllerSythesis.hpp"
#include "../ControllerConcretization/controller_step.hpp"

// Reasons why a simulation run terminated
enum class StopReason {
    SUCCESS,
    FAILURE,
    UNCERTIFIED,
    STEP_LIMIT
};

// String conversions for logging, CSV headers and reporting
inline const char* to_string(StopReason reason) {
    switch (reason) {
        case StopReason::SUCCESS: return "SUCCESS";
        case StopReason::FAILURE: return "FAILURE";
        case StopReason::UNCERTIFIED: return "UNCERTIFIED";
        case StopReason::STEP_LIMIT: return "STEP_LIMIT";
        default: return "UNKNOWN";
    }
}

inline const char* to_string(Mission mission) {
    switch (mission) {
        case Mission::START: return "START";
        case Mission::VISITED_R1: return "VISITED_R1";
        case Mission::VISITED_R2: return "VISITED_R2";
        case Mission::SUCCESS: return "SUCCESS";
        case Mission::FAILURE: return "FAILURE";
        default: return "UNKNOWN";
    }
}

// Complete record of a simulation execution
struct SimulationResult {
    std::vector<Vec3> states;        // Observed state at every observation (initial and final included)
    std::vector<Mission> missions;   // Mission after observing each corresponding state
    std::vector<Vec2> velocities;    // Control applied for each movement
    std::vector<Vec3> disturbances;  // Disturbance used for each movement
    StopReason reason;               // ALWAYS set before returning
    unsigned seed;                   // Random seed used for disturbances
};

// Signature for the controller step function
using StepFunction = std::function<ControlStep(
    const Controller& controller,
    const std::vector<Vec2>& val_u,
    const Vec3& state,
    Mission previous
)>;

// Standard simulation function using controller_step
SimulationResult simulate(const Controller& controller,
                          const std::vector<Vec2>& val_u,
                          const Vec3& initial_state,
                          int max_steps,
                          unsigned seed);

// Generalized simulation function accepting any step function (for unit tests / stubs)
SimulationResult simulate_with_step(const Controller& controller,
                                    const std::vector<Vec2>& val_u,
                                    const Vec3& initial_state,
                                    int max_steps,
                                    unsigned seed,
                                    const StepFunction& step_fn);

// Export a simulation run to CSV format
bool export_simulation_to_csv(const SimulationResult& result,
                              const std::string& filepath);
