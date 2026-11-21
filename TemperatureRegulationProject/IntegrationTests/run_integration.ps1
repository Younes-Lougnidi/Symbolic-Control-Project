# One documented build-and-run procedure for the entire TemperatureRegulationProject
# (Windows PowerShell). Run from anywhere; outputs stay in IntegrationTests/.

$ErrorActionPreference = "Stop"
$ScriptDir = $PSScriptRoot
$ProjectDir = Split-Path $ScriptDir -Parent
$Flags = "-O2 -std=c++17 -Wall -Wextra"

Write-Host "======================================================================"
Write-Host " Temperature Regulation Project - Full Build, Test & Baseline Runner"
Write-Host "======================================================================"
g++ --version | Select-Object -First 1
Write-Host "Flags: $Flags"
Write-Host ""

Write-Host "[1/5] Building & running dynamics unit tests (Omar)..."
g++ -O2 -std=c++17 -Wall -Wextra "$ProjectDir/dynamics/dynamicsTest.cpp" -o "$ScriptDir/test_dynamics.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }
& "$ScriptDir/test_dynamics.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "`n[2/5] Building & running abstraction unit tests (Aymane)..."
g++ -O2 -std=c++17 -Wall -Wextra "$ProjectDir/abstraction/test_abstraction.cpp" -o "$ScriptDir/test_abstraction.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }
& "$ScriptDir/test_abstraction.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "`n[3/5] Building & running simulation unit tests (Amine)..."
g++ -O2 -std=c++17 -Wall -Wextra "$ProjectDir/simulation/test_simulation.cpp" -o "$ScriptDir/test_simulation.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }
Push-Location "$ProjectDir/simulation"
& "$ScriptDir/test_simulation.exe"
$simCode = $LASTEXITCODE
Pop-Location
if ($simCode -ne 0) { exit 1 }

Write-Host "`n[4/5] Building & running integration & verification suite (Rayane)..."
g++ -O2 -std=c++17 -Wall -Wextra "-DCOMP_FLAGS=\`"$Flags\`"" `
    "$ScriptDir/test_integration.cpp" -lpsapi -o "$ScriptDir/test_integration.exe"
if ($LASTEXITCODE -ne 0) { exit 1 }
& "$ScriptDir/test_integration.exe" --sim-dir "$ProjectDir/simulation" --out-dir "$ScriptDir"
$integCode = $LASTEXITCODE

Write-Host "`n[5/5] Running isolated per-N peak-memory benchmarks..."
& "$ScriptDir/test_integration.exe" --bench 3
& "$ScriptDir/test_integration.exe" --bench 4

Remove-Item -Force -ErrorAction SilentlyContinue `
    "$ScriptDir/test_dynamics.exe", `
    "$ScriptDir/test_abstraction.exe", `
    "$ScriptDir/test_simulation.exe", `
    "$ScriptDir/test_integration.exe"

exit $integCode
