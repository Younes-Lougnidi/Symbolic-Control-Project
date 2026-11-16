# One build-and-run procedure for the whole project (run from anywhere).
# Builds the abstraction, builds the integration runner, runs it, and keeps
# the results table in IntegrationTests/.

$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)

Write-Host "1/3 Building and running the abstraction (writes symb_abs.bin)..."
g++ -O2 -std=c++17 robotControl/sym_abs.cpp -o sym_abs.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
.\sym_abs.exe
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "2/3 Building the integration runner..."
g++ -O2 -std=c++17 -Wall -Wextra IntegrationTests/test_integration.cpp Simulation/simulation.cpp Safety/safety_synthesis.cpp -o IntegrationTests/test_integration.exe
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "3/3 Running the integration tests..."
.\IntegrationTests\test_integration.exe
$code = $LASTEXITCODE

Move-Item -Force integration_results.md IntegrationTests/integration_results.md
Move-Item -Force integration_results.csv IntegrationTests/integration_results.csv
exit $code
