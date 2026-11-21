#!/usr/bin/env bash
# One documented build-and-run procedure for the entire TemperatureRegulationProject
# (macOS / Linux). Run from anywhere; outputs stay in IntegrationTests/.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
FLAGS="-O2 -std=c++17 -Wall -Wextra"

echo "======================================================================"
echo " Temperature Regulation Project - Full Build, Test & Baseline Runner"
echo "======================================================================"
echo "Compiler : $(g++ --version | head -n 1)"
echo "Flags    : ${FLAGS}"
if [[ "$(uname -s)" == "Darwin" ]]; then
    echo "Hardware : $(sysctl -n machdep.cpu.brand_string), $(( $(sysctl -n hw.memsize) / 1073741824 )) GiB RAM"
else
    echo "Hardware : $(uname -m), $(uname -s)"
fi
echo ""

echo "[1/5] Building & running dynamics unit tests (Omar)..."
g++ ${FLAGS} "${PROJECT_DIR}/dynamics/dynamicsTest.cpp" -o "${SCRIPT_DIR}/test_dynamics"
"${SCRIPT_DIR}/test_dynamics"

echo ""
echo "[2/5] Building & running abstraction unit tests (Aymane)..."
g++ ${FLAGS} "${PROJECT_DIR}/abstraction/test_abstraction.cpp" -o "${SCRIPT_DIR}/test_abstraction"
"${SCRIPT_DIR}/test_abstraction"

echo ""
echo "[3/5] Building & running simulation unit tests (Amine)..."
g++ ${FLAGS} "${PROJECT_DIR}/simulation/test_simulation.cpp" -o "${SCRIPT_DIR}/test_simulation"
(cd "${PROJECT_DIR}/simulation" && "${SCRIPT_DIR}/test_simulation")

echo ""
echo "[4/5] Building & running integration & verification suite (Rayane)..."
g++ ${FLAGS} -DCOMP_FLAGS="\"${FLAGS}\"" \
    "${SCRIPT_DIR}/test_integration.cpp" \
    -o "${SCRIPT_DIR}/test_integration"
"${SCRIPT_DIR}/test_integration" \
    --sim-dir "${PROJECT_DIR}/simulation" \
    --out-dir "${SCRIPT_DIR}"

echo ""
echo "[5/5] Running isolated per-N peak-memory benchmarks..."
"${SCRIPT_DIR}/test_integration" --bench 3
"${SCRIPT_DIR}/test_integration" --bench 4

# Clean up compiled binaries (keep reports & CSVs).
rm -f "${SCRIPT_DIR}/test_dynamics" \
      "${SCRIPT_DIR}/test_abstraction" \
      "${SCRIPT_DIR}/test_simulation" \
      "${SCRIPT_DIR}/test_integration"

echo ""
echo "Done. Reports saved in ${SCRIPT_DIR}/"
