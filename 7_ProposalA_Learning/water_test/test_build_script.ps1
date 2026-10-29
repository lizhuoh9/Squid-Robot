<#
.SYNOPSIS
    Runs build_test_firmware.ps1 in the cases that can be checked without ESP-IDF v5.4.4 and writes the
    results to evidence\script_checks.txt:
    1. -PrepareOnly with git's core.autocrlf set to true, then to false;
    2. the full build path, with stand-ins for idf.py and xtensa-esp32s3-elf-gcc that report v5.4.4 and
       esp-14.2.0_20260121 and return C0's image (from commit deef5845) as the build output;
    3. the same with a stand-in reporting ESP-IDF v5.3.2, and with a stand-in for another compiler
       build: both runs must stop before preparing anything.
    The stand-ins are functions of this PowerShell session; the images go to <WorkRoot>\test-images.

.NOTES
    Run it in a PowerShell where ESP-IDF is NOT active: there idf.py is an alias, and an alias would
    win over the stand-in function.
#>
[CmdletBinding()]
param(
    [string]$RepoRoot = '',  # default set below: $PSScriptRoot is empty in parameter defaults under -File
    [string]$WorkRoot = ''
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$Script = Join-Path $PSScriptRoot 'build_test_firmware.ps1'
$Commit = 'deef58455f6f395e1b4b07fdaa39611760143435'
$Image = '7_ProposalA_Learning/firmware/build-learning/squid_robot_a_recorder'
$Tar = Join-Path $env:SystemRoot 'System32\tar.exe'
$Log = New-Object System.Collections.Generic.List[string]
$Failures = 0

function Write-Log([string]$Line) { $Log.Add($Line); Write-Host $Line }

function Test-Case([string]$Name, [bool]$Passed, [string]$Detail) {
    if (-not $Passed) { $script:Failures++ }
    Write-Log ('{0}  {1}: {2}' -f $(if ($Passed) { 'PASS' } else { 'FAIL' }), $Name, $Detail)
}

# Runs the build script; returns its host output and the error message it stopped with, if any.
function Invoke-BuildScript([hashtable]$Arguments) {
    $lines = @()
    $stopped = $null
    try { $lines = @(& $Script @Arguments *>&1 | ForEach-Object { "$_" }) }
    catch { $stopped = $_.Exception.Message }
    foreach ($line in $lines) { if ($line.Trim()) { Write-Log "    $line" } }
    if ($stopped) { Write-Log "    stopped: $stopped" }
    [pscustomobject]@{ Lines = $lines; Stopped = $stopped }
}

function Get-CrCount([string]$Path) {
    @([IO.File]::ReadAllBytes($Path) | Where-Object { $_ -eq 13 }).Count
}

function Resolve-FullPath([string]$Path) {
    $full = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Path)
    if ($full.Length -gt 3) { $full = $full.TrimEnd('\') }
    $full
}

if (-not $RepoRoot) { $RepoRoot = Join-Path $PSScriptRoot '..\..' }
$RepoRoot = Resolve-FullPath $RepoRoot
$up = @(git -C $RepoRoot rev-parse --show-cdup)  # not --show-toplevel: its UTF-8 path is misread here
if ($LASTEXITCODE -ne 0) { throw "$RepoRoot is not a git checkout" }
if ($up.Count -gt 0 -and $up[0]) { $RepoRoot = Resolve-FullPath (Join-Path $RepoRoot $up[0]) }
if (-not $WorkRoot) { $WorkRoot = Join-Path ([IO.Path]::GetPathRoot($RepoRoot)) 'sqfw' }
$WorkRoot = Resolve-FullPath $WorkRoot
$images = Join-Path $WorkRoot 'test-images'
$stage = Join-Path $WorkRoot 'test-c0'
foreach ($folder in $images, $stage) {
    if (Test-Path -LiteralPath $folder) { Remove-Item -LiteralPath $folder -Recurse -Force }
    New-Item -ItemType Directory -Path $folder | Out-Null
}
$bArgs = @{ RepoRoot = $RepoRoot; WorkRoot = $WorkRoot }
$controllerB = Join-Path $WorkRoot 'b\7_ProposalA_Learning\firmware\main\DepthController.cpp'

$savedConfig = $env:GIT_CONFIG_PARAMETERS
try {
    Write-Log "build_test_firmware.ps1 checks, $(Get-Date -Format 'yyyy-MM-dd'), sources $($Commit.Substring(0, 8))"
    foreach ($setting in 'true', 'false') {
        Write-Log ''
        Write-Log "== -PrepareOnly with core.autocrlf=$setting"
        $env:GIT_CONFIG_PARAMETERS = "'core.autocrlf=$setting'"
        $run = Invoke-BuildScript ($bArgs + @{ PrepareOnly = $true })
        $cr = Get-CrCount $controllerB
        $lineEnds = $(if ($setting -eq 'true') { $cr -gt 0 } else { $cr -eq 0 })
        Test-Case "prepare, autocrlf=$setting" (-not $run.Stopped -and $lineEnds) "stopped: $([bool]$run.Stopped); CR bytes in B's DepthController.cpp: $cr"
    }
} finally {
    $env:GIT_CONFIG_PARAMETERS = $savedConfig
}

# C0's image and ELF stand in for the build output.
$archive = Join-Path $stage 'c0.tar'
git -C $RepoRoot archive --format=tar -o $archive $Commit -- "$Image.bin" "$Image.elf"
if ($LASTEXITCODE -ne 0) { throw 'git archive of the C0 image failed' }
& $Tar -x -f $archive -C $stage
if ($LASTEXITCODE -ne 0) { throw 'tar -x of the C0 image failed' }
$global:StandInBin = Join-Path $stage ($Image.Replace('/', '\') + '.bin')
$global:StandInElf = Join-Path $stage ($Image.Replace('/', '\') + '.elf')
$global:StandInIdf = 'ESP-IDF v5.4.4'
$global:StandInGcc = 'xtensa-esp-elf-gcc.exe (crosstool-NG esp-14.2.0_20260121) 14.2.0'

function global:idf.py {
    # stand-in for test_build_script.ps1
    $global:LASTEXITCODE = 0
    if ($args -contains '--version') { return $global:StandInIdf }
    $build = $args[[array]::IndexOf($args, '-B') + 1]
    if ($args -contains 'build') {
        New-Item -ItemType Directory -Force -Path $build | Out-Null
        Copy-Item -LiteralPath $global:StandInBin -Destination (Join-Path $build 'squid_robot_a_recorder.bin')
        Copy-Item -LiteralPath $global:StandInElf -Destination (Join-Path $build 'squid_robot_a_recorder.elf')
        return 'stand-in build: copied the C0 image'
    }
    if ($args -contains 'size') { return 'stand-in size report' }
}
function global:xtensa-esp32s3-elf-gcc { $global:LASTEXITCODE = 0; $global:StandInGcc }

try {
    $command = Get-Command idf.py
    if ($command.CommandType -ne 'Function' -or $command.Definition -notmatch 'stand-in for test_build_script') {
        throw 'idf.py resolves to the real ESP-IDF here; run this test in a PowerShell without ESP-IDF'
    }

    Write-Log ''
    Write-Log '== full build path with stand-ins for ESP-IDF v5.4.4 and esp-14.2.0_20260121'
    $run = Invoke-BuildScript ($bArgs + @{ OutDir = $images })
    $files = @(Get-ChildItem -LiteralPath $images | Sort-Object Name | ForEach-Object { $_.Name })
    $c0 = (Get-FileHash -LiteralPath $global:StandInBin -Algorithm SHA256).Hash.ToLowerInvariant()
    $expected = @('B_{0}.bin', 'B_{0}.build.txt', 'B_{0}.elf', 'C1_{0}.bin', 'C1_{0}.build.txt', 'C1_{0}.elf', 'SHA256SUMS.txt') |
        ForEach-Object { $_ -f $c0.Substring(0, 8) }
    Test-Case 'build path' (-not $run.Stopped -and (($files -join ' ') -eq ($expected -join ' '))) "files: $($files -join ', ')"
    $sums = Get-Content -LiteralPath (Join-Path $images 'SHA256SUMS.txt')
    Test-Case 'SHA256SUMS.txt' (@($sums | Where-Object { $_ -like "$c0  *" }).Count -eq 2) ($sums -join ' | ')
    foreach ($line in Get-Content -LiteralPath (Join-Path $images ('B_{0}.build.txt' -f $c0.Substring(0, 8))) -TotalCount 9) {
        Write-Log "    B build record: $line"
    }

    Write-Log ''
    Write-Log '== stand-in reports ESP-IDF v5.3.2'
    $global:StandInIdf = 'ESP-IDF v5.3.2'
    $run = Invoke-BuildScript ($bArgs + @{ OutDir = $images; Variant = 'C1' })
    Test-Case 'wrong ESP-IDF' ([bool]$run.Stopped -and $run.Stopped -like '*v5.3.2 is active*') 'must stop before preparing'

    Write-Log ''
    Write-Log '== stand-in reports another compiler build'
    $global:StandInIdf = 'ESP-IDF v5.4.4'
    $global:StandInGcc = 'xtensa-esp-elf-gcc.exe (crosstool-NG esp-14.2.0_20241119) 14.2.0'
    $run = Invoke-BuildScript ($bArgs + @{ OutDir = $images; Variant = 'C1' })
    Test-Case 'wrong compiler' ([bool]$run.Stopped -and $run.Stopped -like '*esp-14.2.0_20260121*') 'must stop before preparing'
} finally {
    Remove-Item -LiteralPath 'function:idf.py', 'function:xtensa-esp32s3-elf-gcc' -ErrorAction SilentlyContinue
}

Write-Log ''
Write-Log "$Failures failed"
[IO.File]::WriteAllLines((Join-Path $PSScriptRoot 'evidence\script_checks.txt'), [string[]]$Log)
exit $Failures
