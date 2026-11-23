# Integration Tests, Verification, and Baseline Measurements

Author: Rayane Khaldi

Connects the dynamics, abstraction, safety synthesis, and simulation modules of `TemperatureRegulationProject`, verifies the synthesized controller and recorded trajectories independently, and records baseline runtime and memory measurements.

## Connected modules

- `../dynamics/dynamics.hpp`, `coord2state.hpp`, `state2coord.hpp` (Omar): `VEC_N`, `Bounds`, `dyn::f`, `dyn::reachable_bounds`, grid constants.
- `../abstraction/abstraction.hpp` (Aymane): 0-based `abstraction::Id`, `Cell`, `Successors`, `Abstraction`.
- `../Safety_Synthesis/SafetyAbstraction/checked_abstraction.hpp`, `../Safety_Synthesis/safety_synthesis.hpp` (Younes): overflow-checked `safety_abstraction::build_abstraction` and `synthesize_safety`.
- `../simulation/simulation.hpp`, `../simulation/sim_trace*.csv` (Amine): `simulation::Controller`, `SimulationResult`, `simulate`, and the three exported 50-step traces.

Both synthesis and simulation represent the controller as `std::vector<std::vector<abstraction::Id>>` (all certified 0-based command IDs per cell; an empty vector marks an uncertified cell; command ID `0` is all heaters off). `integ::verify_controller` checks every listed command for every certified cell.

## Build and run

macOS / Linux:
```bash
./TemperatureRegulationProject/IntegrationTests/run_integration.sh
```

Windows (PowerShell):
```powershell
.\TemperatureRegulationProject\IntegrationTests\run_integration.ps1
```

Manual compile and run from `TemperatureRegulationProject/IntegrationTests/`:
```bash
g++ -O2 -std=c++17 -Wall -Wextra \
    -DCOMP_FLAGS="\"-O2 -std=c++17 -Wall -Wextra\"" \
    test_integration.cpp -o test_integration

# Full verification suite and N=3,4 baseline
./test_integration --sim-dir ../simulation --out-dir .

# Single-N isolated benchmark for per-N peak RSS
./test_integration --bench 3
./test_integration --bench 4
./test_integration --bench 5
```

## Files

- `verification.hpp`: independent constants, `reference_step`, base-10/base-3 codecs, `verify_controller`, `audit_abstraction`, `check_trajectory`, and `verify_csv_trace`.
- `measurements.hpp`: separate `steady_clock` timing for abstraction and synthesis, OS peak RSS (`MemorySample{false, 0}` printed as `missing`, never `0`), and build/platform metadata.
- `run_tests.hpp`: six test suites (checker self-tests, interface/overflow checks, exhaustive abstraction audit, seven policy-corruption checks, named + random simulations, and CSV trace verification) plus `run_experiments()`.
- `test_integration.cpp`: command-line runner supporting the full suite and `--bench N`.
- `run_integration.sh` / `run_integration.ps1`: build-and-run scripts for macOS/Linux and Windows.
- `integration_results.md` / `integration_results.csv` / `baseline_measurements.csv`: generated test and benchmark outputs.

## What each check proves (and what it does not)

- `verify_controller(A, ctrl)` proves inductive invariance on the symbolic model: every certified command stays inside the safe set (`leaves_safe_set == false`) and every cell in its Cartesian successor box is also certified. By itself, it does not prove that `A` encloses the physical dynamics `f`.
- `audit_abstraction(A)` recomputes every transition box for `N=3` (27,000 pairs) directly from corner evaluations of the dynamics and closed-interval intersections. The corner-bound enclosure itself relies on the analytical monotonicity proof from the dynamics module.
- Policy corruption checks mutate a valid controller in 7 ways (exiting command, uncertified successor, negative ID, out-of-range ID, duplicate ID, truncated table, flipped exit flag) to confirm `verify_controller` actually rejects invalid policies.
- `check_trajectory` and `verify_csv_trace` check every recorded temperature directly against `[17, 22]`, replay `f(T(k), u(k))`, and check policy membership without trusting `SimulationResult::safe`. Finite-horizon simulations demonstrate safe behavior on tested starts; they do not prove infinite-time safety for all initial states.

## Building and verifying the local safety controller

From `TemperatureRegulationProject/Scalability`, build the two executables on Windows:

```powershell
g++ -O2 -std=c++17 -Wall -Wextra correctness_verification.cpp -o verify_local_synthesis.exe
g++ -O2 -std=c++17 -Wall -Wextra ../IntegrationTests/benchmark_policy_application.cpp -lpsapi -o measure_local_synthesis.exe
```

Run the correctness check and the construction timing separately:

```powershell
.\verify_local_synthesis.exe
.\measure_local_synthesis.exe --build-only 1000000000000
```

The verifier compares all three-room successor boxes and exit flags with the explicit abstraction, then checks the winning domain and the complete controller at three and four rooms. The build-only command reports `T_abs` for the 3,000 local transitions, `T_ctrl` for the domain certificate and maximal controller, their combined time, and the process peak RSS through synthesis. Every ring with at least three rooms uses the same local transitions. When every neighbourhood has an allowed level, simultaneous choices keep the entire grid invariant. Synthesis then returns the full grid as the winning domain (100%) and the table of all allowed levels. Build-only mode stops before generating room inputs.

## Baseline measurements

Machine: Apple M3 Pro, 18 GiB RAM, macOS (arm64).  
Compiler: Apple Clang 17.0.0 (`-O2 -std=c++17 -Wall -Wextra`).  
Scope: isolated single-`N` runs (`./test_integration --bench N`); wall-clock `steady_clock` times `safety_abstraction::build_abstraction(N)` and `synthesize_safety(A)` separately (excluding verification and I/O); peak memory is OS `ru_maxrss`.

| Method | N | States | Commands | Abstraction time | Synthesis time | Total time | Peak memory (RSS) | Controllable % | Verified |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Reference | 3 | 1,000 | 27 | 4.170 ms | 7.634 ms | 11.805 ms | 4.48 MiB | 100.00% | PASS |
| Reference | 4 | 10,000 | 81 | 117.416 ms | 402.962 ms | 520.378 ms | 78.70 MiB | 100.00% | PASS |
| Reference | 5 | 100,000 | 243 | 4.1736 s | 25.9577 s | 30.1314 s | 1,939.41 MiB | 100.00% | PASS |

Each additional room multiplies the transition table by 30x (`30^N`). Because each `Successors` entry stores two `std::vector<int>`, `N=5` already uses ~1.94 GiB RSS (~2.41 GiB heap), and `N=6` (729M transitions) would require ~73 GiB. Reducing or eliminating explicit per-pair transition storage is the main bottleneck to address for scaling past `N=5`.
