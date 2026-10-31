# Mobile Robot Symbolic Control

Computation Theory project at UM6P: use a finite symbolic model to plan the motion of a unicycle robot under bounded disturbances. The overall goal is to reach targets and avoid obstacles, ultimately visiting exactly one of R1 or R2, then R3, while avoiding R4 until completion.

## Current implementation

This folder implements the symbolic abstraction stage in C++17, ported from Antoine Girard's MATLAB code. The robot's position and heading are divided into **300,000 cells** (100 x 100 x 30), with **15 commands** combining linear and angular velocities. For each cell and command, derivative bounds enclose the possible next states, accounting for uncertainty and disturbances.

- `robotControl/dynamics.hpp`: robot motion equations, bounds, sampling period, and grid settings.
- `robotControl/coord2state.hpp` / `robotControl/state2coord.hpp`: convert between grid coordinates and unique cell IDs.
- `robotControl/sym_abs.cpp`: builds the transition data and saves `symb_abs.bin`, including command values and robustness margins.

Reachability controller synthesis is implemented in `Reachablity/`. Choosing a target, connecting the controller to the robot, specification tracking, and closed-loop simulation remain to be completed.

## Build and run

With a C++17-compatible g++ compiler, run from this folder in PowerShell:

```powershell
g++ -O2 -std=c++17 robotControl/sym_abs.cpp -o sym_abs.exe
.\sym_abs.exe
```

The program writes (or replaces) `symb_abs.bin` in the current directory.

## Reachability

The implementation is in `Reachablity/reachability.hpp`. It directly uses the existing
`g` table from `robotControl/sym_abs.cpp`, `dyn::n_x`, `dyn::n_u`,
`state2coord`, and `coord2state`. There is no second abstraction, binary loader,
or command-line application.

### Use with the existing abstraction

Add this include to `robotControl/sym_abs.cpp`:

```cpp
#include "../Reachablity/reachability.hpp"
```

After the loops that construct `g`, call:

```cpp
std::array<int, 2> target{123, 124}; // Example only: use your target cell IDs.
auto controller = reach::Max_reach(g, target);
// Alternatively: std::vector<int> target for a target of variable size.
```

No call is inserted automatically because the actual target is not specified.
For a geometric target, use cells entirely contained in the target region.
Include all heading bins if arrival orientation does not matter.

Results use the same 1-based state and command IDs as the abstraction:

- `controller.winning`: states with guaranteed finite-time arrival.
- `controller.rank[s]`: -1 if uncertified, 0 at target, otherwise a step bound.
- `controller.h[s]`: saved command, or 0 for target/uncertified states.

When `controller.rank[s] > 0`, the concrete command is
`val_u[controller.h[s] - 1]`. At rank zero the objective is complete. Never
index `val_u` using command 0. Remaining in the target is a separate objective.

### Functions

`successors_in(box, membership)` decodes the stored corners with your coordinate
helpers and checks every cell of the box, including cyclic heading intervals.
It rejects `{0,0}` (an action leaving workspace constraints).

`pred(g, A)` returns one `(state, command)` witness for each predecessor. Every
possible successor of that command must belong to A. One witness is enough to
construct the controller; it does not list every permissible command.

`Max_reach(g, target)` repeatedly computes predecessors and adds new states,
saving the iteration rank and witnessing command. Each predecessor calculation
finishes before the winning set changes, so every saved command leads to
strictly smaller ranks. Both functions are templates accepting arrays or vectors.

This follows the proposed brute-force algorithm. Large grids may take time;
no extra optimization framework is introduced. Guarantees depend on the supplied
abstraction being sound. No additional obstacle constraints are added.

### Check (from SymbolicControlProject)

```powershell
g++ -std=c++17 -O2 -Wall -Wextra -pedantic Reachablity/tests.cpp -o Reachablity/tests.exe
./Reachablity/tests.exe
```

Tests use your actual grid dimensions and helpers, covering uncertain successors,
invalid controls, rank decrease, self-loops, duplicate/empty targets, invalid IDs,
and wrapped successor boxes with missing interior cells.

