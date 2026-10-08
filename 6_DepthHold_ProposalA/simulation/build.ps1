$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compiler = (Get-Command g++.exe -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compiler) + ';' + $env:PATH
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path 'simulation/bin' | Out-Null
    $flags = @('-std=c++14', '-O2', '-Wall', '-Wextra', '-Wpedantic')
    $sharedIncludes = @('-Isimulation/stubs', '-Isimulation/common')
    & $compiler @flags @sharedIncludes '-Imain' 'main/DepthController.cpp' 'simulation/common/KalmanFilter.cpp' 'simulation/harness/sim_main.cpp' '-o' 'simulation/bin/sim_proposal_a.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Proposal A simulation compilation failed' }
    & $compiler @flags @sharedIncludes '-Isimulation/baseline' 'simulation/baseline/DepthController.cpp' 'simulation/common/KalmanFilter.cpp' 'simulation/harness/sim_main.cpp' '-o' 'simulation/bin/sim_current_pid_only.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Baseline simulation compilation failed' }
    & $compiler @flags @sharedIncludes '-Imain' 'main/DepthController.cpp' 'simulation/harness/controller_contract.cpp' '-o' 'simulation/bin/controller_contract.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Controller contract compilation failed' }
    & '.\simulation\bin\controller_contract.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Controller contract checks failed' }
} finally {
    Pop-Location
}
