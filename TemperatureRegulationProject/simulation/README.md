# Building Safety Control — Simulation & Execution (Task 4)

**Contributor:** Amine Chrif  
**Module:** Closed-loop discrete-time temperature simulation & visualization  
**Project:** Symbolic Safety Control of a Circular Building  

---

## 1. Overview

This module provides the execution engine for the 3-room building safety control problem. Given a synthesized safety controller, it simulates the discrete-time closed-loop dynamics, validates safety invariants ($T_i \in [17.0, 22.0]^\circ\text{C}$), exports trajectory logs to CSV, and generates publication-quality plots.

---

## 2. Prerequisites

- **C++ Compiler:** `g++` or `clang++` supporting **C++17** (uses `-std=c++17 -O2`).
- **Python Environment:** Python 3 with `matplotlib` (standard library `csv` is used; no `pandas` required).

---

## 3. Quickstart: Compile & Run

From the `TemperatureRegulationProject/simulation/` directory:

### Step 1: Compile and Run Unit Tests & Deliverables
```bash
g++ -std=c++17 -O2 test_simulation.cpp -o test_simulation
./test_simulation
```

This verifies all 7 unit tests and automatically generates the three official simulation traces:
- `sim_trace1_cold.csv`: All-cold start $(17.0, 17.0, 17.0)^\circ\text{C}$
- `sim_trace2_hot.csv`: All-hot start $(22.0, 22.0, 22.0)^\circ\text{C}$
- `sim_trace3_mixed.csv`: Mixed start $(17.0, 19.5, 22.0)^\circ\text{C}$

### Step 2: Generate Trajectory Plots
```bash
# Generate the 3-panel comparison figure (recommended for report)
python3 plot_temperatures.py --all-scenarios

# Or generate individual plots
python3 plot_temperatures.py --csv sim_trace1_cold.csv --output-prefix plot_cold_run
python3 plot_temperatures.py --csv sim_trace2_hot.csv --output-prefix plot_hot_run
python3 plot_temperatures.py --csv sim_trace3_mixed.csv --output-prefix plot_mixed_run
```

Outputs are saved in both high-resolution **PNG** (300 DPI) and vector **PDF** formats.

---

## 4. File Manifest

| File | Description |
|---|---|
| `simulation.hpp` | Closed-loop simulation engine (`simulate`), safety checker (`is_safe`), and CSV exporter |
| `test_simulation.cpp` | Test runner executing the 7 unit tests and exporting the 3 official traces |
| `plot_temperatures.py` | Standalone Python plotting tool (individual and multi-panel comparison) |
| `sim_trace*.csv` | Standardized execution traces for validation |
| `plot_*.png` / `plot_*.pdf` | Rendered temperature trajectories showing safety bounds |

---

## 5. Verification Protocol (For Rayane - Task 5)

To independently verify Amine's traces:
1. Ensure each trace contains $K+1$ temperature rows and $K$ commands for $K=50$ updates.
2. Verify that every temperature entry satisfies $17.0 \le T_i \le 22.0$.
3. Check that for each step, $T(k+1) = \text{dyn::f}(T(k), u(k))$ matches the physical model.
