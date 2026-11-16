#include "controller_step.hpp"

#include <cassert>
#include <iostream>
#include <limits>

int main() {
    // 1. Coordinate conversion
    Vec3 state = {0.05, 0.05, -PI + 0.01};
    assert(numerical_state_to_cell(state) == 1);

    state[0] = 0.15;
    assert(numerical_state_to_cell(state) == 2);

    state[2] += 2.0 * PI;
    assert(numerical_state_to_cell(state) == 2);

    state[0] = -0.1;
    assert(numerical_state_to_cell(state) == 0);

    state[0] = 10.0;
    assert(numerical_state_to_cell(state) == 0);

    std::cout << "Coordinate conversion tests passed!\n";

    // Empty tables: terminal cases must stop before table lookup.
    Controller controller;
    std::vector<Vec2> val_u;

    // 2. Success: R1 was visited, now observe R3.
    ControlStep success = controller_step(
        controller,
        val_u,
        Vec3{2.55, 1.05, 0.0},
        Mission::VISITED_R1
    );

    assert(success.mission == Mission::SUCCESS);
    assert(success.status == Status::SUCCESS);
    assert(success.velocities[0] == 0.0);
    assert(success.velocities[1] == 0.0);

    std::cout << "Mission success test passed!\n";

    // 3. Failure: visiting R2 after R1.
    ControlStep both_targets = controller_step(
        controller,
        val_u,
        Vec3{9.05, 2.55, 0.0},
        Mission::VISITED_R1
    );

    assert(both_targets.mission == Mission::FAILURE);
    assert(both_targets.status == Status::FAILURE);

    // Failure: entering R4.
    ControlStep obstacle = controller_step(
        controller,
        val_u,
        Vec3{5.05, 5.05, 0.0},
        Mission::START
    );

    assert(obstacle.mission == Mission::FAILURE);
    assert(obstacle.status == Status::FAILURE);

    // Failure: leaving the workspace.
    ControlStep outside = controller_step(
        controller,
        val_u,
        Vec3{-0.1, 2.0, 0.0},
        Mission::START
    );

    assert(outside.mission == Mission::FAILURE);
    assert(outside.status == Status::FAILURE);

    std::cout << "Mission failure tests passed!\n";

    // 4. Prepare a controller entry solely for testing lookup.
    const int n_states =
        dyn::n_x[0] * dyn::n_x[1] * dyn::n_x[2];

    controller.rank.resize(n_states + 1);
    controller.command.resize(n_states + 1);

    for (int cell = 0; cell <= n_states; ++cell) {
        controller.rank[cell].fill(-1);
        controller.command[cell].fill(0);
    }

    val_u = {
        Vec2{0.25, -1.0},
        Vec2{0.625, -1.0}
    };

    Vec3 state_in_R1 = {4.55, 9.05, 0.0};
    int cell_R1 = numerical_state_to_cell(state_in_R1);
    int visited_R1 = static_cast<int>(Mission::VISITED_R1);

    controller.rank[cell_R1][visited_R1] = 3;
    controller.command[cell_R1][visited_R1] = 2;

    ControlStep running = controller_step(
        controller,
        val_u,
        state_in_R1,
        Mission::START
    );

    assert(running.mission == Mission::VISITED_R1);
    assert(running.status == Status::RUNNING);
    assert(running.velocities[0] == 0.625);
    assert(running.velocities[1] == -1.0);

    std::cout << "Command lookup test passed!\n";

    // 5. Uncertified: remove the certification from that entry.
    controller.rank[cell_R1][visited_R1] = -1;
    controller.command[cell_R1][visited_R1] = 0;

    ControlStep uncertified = controller_step(
        controller,
        val_u,
        state_in_R1,
        Mission::START
    );

    assert(uncertified.mission == Mission::VISITED_R1);
    assert(uncertified.status == Status::UNCERTIFIED);
    assert(uncertified.velocities[0] == 0.0);
    assert(uncertified.velocities[1] == 0.0);

    std::cout << "Uncertified state test passed!\n";

    // 6. +PI and -PI represent the same direction.
    int cell_positive_pi =
        numerical_state_to_cell(Vec3{0.05, 0.05, PI});

    int cell_negative_pi =
        numerical_state_to_cell(Vec3{0.05, 0.05, -PI});

    assert(cell_positive_pi != 0);
    assert(cell_positive_pi == cell_negative_pi);

    // 7. Invalid numerical inputs must return FAILURE.
    double nan = std::numeric_limits<double>::quiet_NaN();
    double infinity = std::numeric_limits<double>::infinity();

    ControlStep invalid_position = controller_step(
        controller, val_u,
        Vec3{nan, 1.0, 0.0},
        Mission::START
    );

    assert(invalid_position.status == Status::FAILURE);

    ControlStep invalid_heading = controller_step(
        controller, val_u,
        Vec3{1.0, 1.0, infinity},
        Mission::START
    );

    assert(invalid_heading.status == Status::FAILURE);

    // 8. Reaching R3 after R2 must also succeed.
    ControlStep success_after_R2 = controller_step(
        controller, val_u,
        Vec3{2.55, 1.05, 0.0},
        Mission::VISITED_R2
    );

    assert(success_after_R2.mission == Mission::SUCCESS);
    assert(success_after_R2.status == Status::SUCCESS);

    // 9. A certified ongoing entry must not contain command 0.
    controller.rank[cell_R1][visited_R1] = 3;
    controller.command[cell_R1][visited_R1] = 0;

    bool invalid_command_rejected = false;

    try {
        controller_step(
            controller, val_u,
            state_in_R1,
            Mission::START
        );
    } catch (const std::logic_error&) {
        invalid_command_rejected = true;
    }

    assert(invalid_command_rejected);

    std::cout << "Edge-case tests passed!\n";
        return 0;
}