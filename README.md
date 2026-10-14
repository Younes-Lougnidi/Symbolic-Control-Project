# Mobile Robot Symbolic Control

Computation Theory project at UM6P: use a finite symbolic model to plan the motion of a unicycle robot under bounded disturbances. The overall goal is to reach targets and avoid obstacles, ultimately visiting exactly one of R1 or R2, then R3, while avoiding R4 until completion.

## Current implementation

This folder implements the symbolic abstraction stage in C++17, ported from Antoine Girard's MATLAB code. The robot's position and heading are divided into **300,000 cells** (100 x 100 x 30), with **15 commands** combining linear and angular velocities. For each cell and command, derivative bounds enclose the possible next states, accounting for uncertainty and disturbances.

- `dynamics.hpp`: robot motion equations, bounds, sampling period, and grid settings.
- `coord2state.hpp` / `state2coord.hpp`: convert between grid coordinates and unique cell IDs.
- `sym_abs.cpp`: builds the transition data and saves `symb_abs.bin`, including command values and robustness margins.

Controller synthesis, specification tracking, and closed-loop simulation are the next stages; they are not yet implemented here.

## Build and run

With a C++17-compatible g++ compiler, run from this folder in PowerShell:

```powershell
g++ -O2 -std=c++17 sym_abs.cpp -o sym_abs.exe
.\sym_abs.exe
```

The program writes (or replaces) `symb_abs.bin` in the current directory.
