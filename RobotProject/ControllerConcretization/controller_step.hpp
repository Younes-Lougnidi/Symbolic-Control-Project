#pragma once
#include <cmath>
#include <stdexcept>

#include "../ControllerSynthesis/ControllerSythesis.hpp"

enum class Status {
    RUNNING,
    SUCCESS,
    FAILURE,
    UNCERTIFIED
};

struct ControlStep {
    Vec2 velocities;
    Mission mission;
    Status status;
};

inline double normalize_heading(double heading) {
    double angle = std::fmod(heading, 2.0 * PI);

    if (angle < -PI) {
        angle += 2.0 * PI;
    } else if (angle >= PI) {
        angle -= 2.0 * PI;
    }

    return angle;
}




inline int numerical_state_to_cell(const Vec3& state) {
    Vec3 values = state;

    // Reject invalid numbers before calculating indices.
    for (double value : values) {
        if (!std::isfinite(value)) {
            return 0;
        }
    }

    values[2] = normalize_heading(values[2]);

    Idx3 indices;

    for (int d = 0; d < 3; ++d) {
        double lower = dyn::bound_x_lo[d];
        double upper = dyn::bound_x_hi[d];

        if (values[d] < lower || values[d] >= upper) {
            return 0;
        }

        double width = (upper - lower) / dyn::n_x[d];

        indices[d] =
            static_cast<int>(std::floor((values[d] - lower) / width))
            + 1;

        if (indices[d] < 1 || indices[d] > dyn::n_x[d]) {
            return 0;
        }
    }

    const std::array<int, 4> p_x = {
        1,
        dyn::n_x[0],
        dyn::n_x[0] * dyn::n_x[1],
        dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2]
    };

    return coord2state<3>(indices, p_x);
}








inline ControlStep controller_step(
    const Controller& controller,
    const std::vector<Vec2>& val_u,
    const Vec3& state,
    Mission previous
) {
    const Vec2 unused_velocities = {0.0, 0.0};

    // 1. Find the robot's cell.
    int cell = numerical_state_to_cell(state);

    if (cell == 0) {
        return {unused_velocities, Mission::FAILURE, Status::FAILURE};
    }

    // 2. Update the mission after observing the current cell.
    Mission mission = update_mission(previous, label(cell));

    // 3. Stop if the mission has ended.
    if (mission == Mission::SUCCESS) {
        return {unused_velocities, mission, Status::SUCCESS};
    }

    if (mission == Mission::FAILURE) {
        return {unused_velocities, mission, Status::FAILURE};
    }

    // 4. Check whether this combination is certified.
    int m = static_cast<int>(mission);
    int rank = controller.rank.at(cell).at(m);

    if (rank == -1) {
        return {unused_velocities, mission, Status::UNCERTIFIED};
    }

    // 5. Read the command and check that it is valid.
    int command_id = controller.command.at(cell).at(m);

    if (rank <= 0 ||
        command_id <= 0 ||
        command_id > dyn::n_u[0] * dyn::n_u[1] ||
        static_cast<std::size_t>(command_id) > val_u.size()) {
        throw std::logic_error("Invalid controller entry");
    }

    // 6. Convert the command ID into actual velocities.
    return {
        val_u[command_id - 1],
        mission,
        Status::RUNNING
    };
}