<#
.SYNOPSIS
    Builds the two water-test firmware images of 7_ProposalA_Learning from a pinned commit:
    C1 = the sources of the image deployed on 2026-10-27 (C0) with the learned correction off
         (kEnabled = false), the control;
    B  = C1 + option_b.patch (retuned depth-control constants and a delay predictor).
    The working tree is not touched, and nothing is flashed.

.DESCRIPTION
    1. Exports 7_ProposalA_Learning\firmware (without its build* folders) and
       7_ProposalA_Learning\artifacts\hybrid_mpc_v1 from commit deef5845 ("Sync all project files as of
       2026-10-28"), the sources C0 was built from, to <WorkRoot>\<variant>. ESP-IDF cannot build under
       a path with spaces.
    2. Checks the exported files against pinned SHA-256 values. Text files are hashed with LF line
       ends, so git's line-end setting does not matter.
    3. Sets kEnabled = false in ProposalStudentConfig.h; for B also applies option_b.patch. Then hashes
       the whole copy again: only the intended files may have changed, and to the pinned results.
    4. Unless -PrepareOnly or -CompileCheck: checks that the ESP-IDF version and the compiler are the
       ones C0 was built with, builds with idf.py (component manager off, build folder
       build-test-<variant>), checks that the image says v5.4.4 and that sdkconfig was not rewritten,
       and copies the .bin, the .elf and a build record with the idf.py size report to -OutDir, then
       rewrites SHA256SUMS.txt there.

    Run it in a PowerShell where ESP-IDF v5.4.4 is active (export.ps1 or the EIM activation script).
    -CompileCheck also needs compile_main_sources.py next to this script.

.PARAMETER RepoRoot
    The git checkout of the Squid-Robot repository; it must contain commit deef5845 (git fetch origin).
    Default: the repository this script is in. Relative paths are taken from the current location.

.PARAMETER WorkRoot
    Folder for the build copies, plain ASCII without spaces. Default: \sqfw on the drive of RepoRoot.

.PARAMETER OutDir
    Where the images go. Default: the images folder next to this script.

.PARAMETER CompileCheck
    For a machine without v5.4.4: accept any ESP-IDF version, configure the project and compile only
    the sources of firmware\main with the project's flags. Every change is in those files or in a
    header only they include. v5.3.2 cannot build the whole project (the bundled esp-dl uses
    MALLOC_CAP_SIMD, which v5.3.2 lacks), so no image is made. The compiler output goes to
    build-test-<variant>\compile_check.txt.

.EXAMPLE
    .\build_test_firmware.ps1 -PrepareOnly
.EXAMPLE
    .\build_test_firmware.ps1
.EXAMPLE
    .\build_test_firmware.ps1 -Variant B -WorkRoot C:\sqfw
#>
[CmdletBinding()]
param(
    # The defaults depend on $PSScriptRoot, which Windows PowerShell leaves empty in parameter defaults
    # under -File, so they are set below.
    [string]$RepoRoot = '',
    [string]$WorkRoot = '',
    [ValidateSet('C1', 'B')]
    [string[]]$Variant = @('C1', 'B'),
    [string]$OutDir = '',
    [switch]$PrepareOnly,
    [switch]$CompileCheck
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$SourceCommit = 'deef58455f6f395e1b4b07fdaa39611760143435'  # "Sync all project files as of 2026-10-28"
$RequiredIdf = 'v5.4.4'  # idf_ver in the app descriptor of C0
$RequiredToolchain = 'esp-14.2.0_20260121'  # GCC 14.2.0 build named in the .comment section of C0's ELF
$Project = 'squid_robot_a_recorder'
$Patch = Join-Path $PSScriptRoot 'option_b.patch'
$PatchSha256 = 'ace196cd2a39cb710a7cb3926e30ab909975503f34d80fca77e35b268b4c4b37'
$Marker = '.sqfw-variant'
$Tar = Join-Path $env:SystemRoot 'System32\tar.exe'
$Latin1 = [Text.Encoding]::GetEncoding(28591)  # one character per byte, so text edits keep every other byte

$StudentConfig = '7_ProposalA_Learning\artifacts\hybrid_mpc_v1\ProposalStudentConfig.h'
$ControllerH = '7_ProposalA_Learning\firmware\main\DepthController.h'
$ControllerCpp = '7_ProposalA_Learning\firmware\main\DepthController.cpp'
$Sdkconfig = '7_ProposalA_Learning\firmware\sdkconfig'

# The files at $SourceCommit (see Get-PinSha256). With CRLF line ends ProposalStudentConfig.h hashes to
# config_sha256 of C0 in artifacts\hybrid_mpc_v1\deployment_report.json (43a1656e...); the .espdl is
# embedded_espdl.sha256 there. The sdkconfig holds the same 1421 options as build-learning\config of C0.
$SourcePins = [ordered]@{
    $StudentConfig = 'f21805276d325a5f016ac033f33c82b65ebade45756250b805d05b39b0ab121b'
    '7_ProposalA_Learning\artifacts\hybrid_mpc_v1\proposal_student.espdl' = '268dcd281d70ef52b3dfd122104a8dc39f451b69568ccb15f5e51113d1cdbfa6'
    $Sdkconfig = 'c7d87ca38e9601477c3305d8573517cccf22da4b8627a477de50958878c03261'
    $ControllerH = 'f6a1c44e326af62df75934fa8e8c751657a5698028e9af3af4d2542e9e986579'
    $ControllerCpp = '02b810d614a46e37c520494cd0acb25564e81bb481926f4f0b72e02be4c8a372'
}

# The files each variant changes, and their SHA-256 afterwards.
$StudentOff = '86b5df1f766bd3b9cb31af8e42256bfd42776c108d36f7a9974266cea83fcc74'  # kEnabled = false, nothing else
$VariantPins = @{
    'C1' = [ordered]@{ $StudentConfig = $StudentOff }
    'B'  = [ordered]@{
        $StudentConfig = $StudentOff
        # the sources the simulation verified
        $ControllerH   = '4ef5fe4a699991d87e3cb651663c778430788644e296bd787c1feb9f122c82d6'
        $ControllerCpp = 'c6c21dbc9134baae6b1542cf1320e13de630f807d465893220f16e05c2dffe32'
    }
}

function Get-FileSha256([string]$Path) {
    (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

# SHA-256 of a text file (no NUL byte) with CRLF read as LF, of any other file as it is.
function Get-PinSha256([string]$Path) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    if ([Array]::IndexOf($bytes, [byte]0) -lt 0) {
        $bytes = $Latin1.GetBytes($Latin1.GetString($bytes).Replace("`r`n", "`n"))
    }
    $sha = [Security.Cryptography.SHA256]::Create()
    try { -join ($sha.ComputeHash($bytes) | ForEach-Object { $_.ToString('x2') }) } finally { $sha.Dispose() }
}

function Assert-Pins([string]$Root, $Pins, [string]$What) {
    foreach ($rel in $Pins.Keys) {
        $path = Join-Path $Root $rel
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "${What}: $rel is missing" }
        $actual = Get-PinSha256 $path
        if ($actual -ne $Pins[$rel]) { throw "${What}: $rel has SHA-256 $actual, expected $($Pins[$rel])" }
    }
}

# Native tools report failure through the exit code; their stderr is progress, not an error.
function Invoke-Native([string]$What, [scriptblock]$Command) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & $Command } finally { $ErrorActionPreference = $saved }
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit code $LASTEXITCODE)" }
}

# Relative path -> SHA-256 of every file the build reads: firmware\ without its top-level build*
# folders, and the model folder.
function Get-TreeHashes([string]$Root) {
    $hashes = @{}
    $parts = @(Get-ChildItem -LiteralPath (Join-Path $Root '7_ProposalA_Learning\firmware') -Force |
        Where-Object { -not ($_.PSIsContainer -and $_.Name -like 'build*') })
    $parts += Get-Item -LiteralPath (Join-Path $Root '7_ProposalA_Learning\artifacts\hybrid_mpc_v1')
    foreach ($part in $parts) {
        $files = @($part)
        if ($part.PSIsContainer) { $files = @(Get-ChildItem -LiteralPath $part.FullName -Recurse -File -Force) }
        foreach ($file in $files) {
            $hashes[$file.FullName.Substring($Root.Length).TrimStart('\')] = Get-FileSha256 $file.FullName
        }
    }
    $hashes
}

function Get-ChangedFiles([hashtable]$Reference, [hashtable]$Copy) {
    $all = @($Reference.Keys) + @($Copy.Keys) | Sort-Object -Unique
    @($all | Where-Object {
        -not $Reference.ContainsKey($_) -or -not $Copy.ContainsKey($_) -or $Reference[$_] -ne $Copy[$_]
    })
}

function New-WorkTree([string]$Dest, [string]$Name) {
    if (Test-Path -LiteralPath $Dest) {
        if (-not (Test-Path -LiteralPath (Join-Path $Dest $Marker))) {
            throw "$Dest exists and was not made by this script; remove it or choose another -WorkRoot"
        }
        Remove-Item -LiteralPath $Dest -Recurse -Force
    }
    New-Item -ItemType Directory -Path $Dest | Out-Null
    Set-Content -LiteralPath (Join-Path $Dest $Marker) -Value $Name -Encoding ASCII
}

# git archive writes the files with this machine's line-end setting, as a checkout would.
function Export-Sources([string]$Dest) {
    $paths = @(Invoke-Native "git ls-tree $SourceCommit" {
        git -C $RepoRoot ls-tree --name-only $SourceCommit '7_ProposalA_Learning/firmware/'
    } | Where-Object { ($_ -split '/')[-1] -notlike 'build*' })
    $paths += '7_ProposalA_Learning/artifacts/hybrid_mpc_v1'
    $archive = Join-Path $Dest 'sources.tar'
    Invoke-Native 'git archive' { git -C $RepoRoot archive --format=tar -o $archive $SourceCommit -- $paths }
    Invoke-Native 'tar -x' { & $Tar -x -f $archive -C $Dest }
    Remove-Item -LiteralPath $archive
}

function Disable-Student([string]$Dest) {
    $path = Join-Path $Dest $StudentConfig
    $text = $Latin1.GetString([IO.File]::ReadAllBytes($path))
    $old = 'constexpr bool kEnabled = true;'
    $count = ([regex]::Matches($text, [regex]::Escape($old))).Count
    if ($count -ne 1) { throw "expected one '$old' in $path, found $count" }
    [IO.File]::WriteAllBytes($path, $Latin1.GetBytes($text.Replace($old, 'constexpr bool kEnabled = false;')))
}

# The patch is applied with the line ends of the file it changes, which depend on the git settings.
function Add-OptionB([string]$Dest) {
    if ((Get-PinSha256 $Patch) -ne $PatchSha256) { throw "$Patch is not the version this script was written for" }
    $text = $Latin1.GetString([IO.File]::ReadAllBytes($Patch)).Replace("`r`n", "`n")
    if ($Latin1.GetString([IO.File]::ReadAllBytes((Join-Path $Dest $ControllerH))).Contains("`r`n")) {
        $text = $text.Replace("`n", "`r`n")
    }
    $local = Join-Path $Dest 'option_b.patch'
    [IO.File]::WriteAllBytes($local, $Latin1.GetBytes($text))
    Invoke-Native 'git apply --check option_b.patch' { git -C $Dest apply --check $local }
    Invoke-Native 'git apply option_b.patch' { git -C $Dest apply $local }
    Remove-Item -LiteralPath $local
}

function Get-IdfVersion {
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
        throw 'idf.py not found: run this in a PowerShell where ESP-IDF is active (export.ps1 or the EIM activation script)'
    }
    $text = (Invoke-Native 'idf.py --version' { idf.py --version } | Out-String).Trim()
    if ($text -notmatch 'v\d+\.\d+(\.\d+)?\S*') { throw "cannot read the ESP-IDF version from '$text'" }
    $Matches[0]
}

# esp_app_desc_t at the start of the first segment: image header (24 bytes) + segment header (8).
function Get-AppDescriptor([string]$Path) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    $base = 32
    # 2882360370 = 0xABCD5432 (ESP_APP_DESC_MAGIC_WORD); PowerShell reads that hex literal as a negative Int32.
    if ($bytes[0] -ne 0xE9 -or [BitConverter]::ToUInt32($bytes, $base) -ne [uint32]2882360370) {
        throw "$Path is not an ESP-IDF application image"
    }
    $field = { param($offset, $length) [Text.Encoding]::ASCII.GetString($bytes, $base + $offset, $length).Split([char]0)[0] }
    [pscustomobject]@{
        Project    = & $field 48 32
        Version    = & $field 16 32
        Built      = (& $field 96 16) + ' ' + (& $field 80 16)
        IdfVersion = & $field 112 32
    }
}

# LF line ends, so that sha256sum -c can read it too.
function Write-Sums {
    $lines = @(Get-ChildItem -LiteralPath $OutDir -Filter '*.bin' | Sort-Object Name |
        ForEach-Object { '{0}  {1}' -f (Get-FileSha256 $_.FullName), $_.Name })
    [IO.File]::WriteAllText((Join-Path $OutDir 'SHA256SUMS.txt'), (($lines -join "`n") + "`n"))
}

function Invoke-Build([string]$Dest, [string]$Name, [string[]]$Changed) {
    $firmware = Join-Path $Dest '7_ProposalA_Learning\firmware'
    $build = Join-Path $firmware ('build-test-' + $Name.ToLowerInvariant())
    $saved = $env:IDF_COMPONENT_MANAGER
    $env:IDF_COMPONENT_MANAGER = '0'
    try {
        Invoke-Native "idf.py build ($Name)" { idf.py -C $firmware -B $build build | Out-Host }
        # A failing size report must not cost the image.
        try { $size = @(Invoke-Native "idf.py size ($Name)" { idf.py -C $firmware -B $build size }) }
        catch { $size = @("idf.py size failed: $($_.Exception.Message)") }
    } finally {
        $env:IDF_COMPONENT_MANAGER = $saved
    }

    $bin = Join-Path $build "$Project.bin"
    $elf = Join-Path $build "$Project.elf"
    $app = Get-AppDescriptor $bin
    if ($app.Project -ne $Project) { throw "$bin is project '$($app.Project)', expected '$Project'" }
    if ($app.IdfVersion -ne $RequiredIdf) { throw "$bin was built with ESP-IDF $($app.IdfVersion), C0 with $RequiredIdf" }
    if ((Get-PinSha256 (Join-Path $Dest $Sdkconfig)) -ne $SourcePins[$Sdkconfig]) {
        throw "the build rewrote sdkconfig in $firmware, so this ESP-IDF configures the project differently from C0"
    }

    $sha = Get-FileSha256 $bin
    $record = @(
        "variant          $Name"
        "image            $Project.bin, $((Get-Item -LiteralPath $bin).Length) bytes"
        "sha256           $sha"
        "esp-idf          $($app.IdfVersion), $RequiredToolchain"
        "app descriptor   project $($app.Project), version $($app.Version), built $($app.Built)"
        "sources          commit $SourceCommit"
        "changed files    $($Changed -join ', ')"
        "sdkconfig        unchanged (same as C0)"
        "built in         $Dest"
        ''
        'idf.py size'
    ) + $size

    New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
    $stem = Join-Path $OutDir ('{0}_{1}' -f $Name, $sha.Substring(0, 8))
    Copy-Item -LiteralPath $bin -Destination "$stem.bin" -Force
    Copy-Item -LiteralPath $elf -Destination "$stem.elf" -Force
    [IO.File]::WriteAllLines("$stem.build.txt", [string[]]$record)
    Write-Sums
    [pscustomobject]@{ Variant = $Name; Sha256 = $sha; Idf = $app.IdfVersion; Image = "$stem.bin" }
}

function Invoke-CompileCheck([string]$Dest, [string]$Name) {
    $firmware = Join-Path $Dest '7_ProposalA_Learning\firmware'
    $build = Join-Path $firmware ('build-test-' + $Name.ToLowerInvariant())
    $saved = $env:IDF_COMPONENT_MANAGER
    $env:IDF_COMPONENT_MANAGER = '0'
    try {
        Invoke-Native "idf.py reconfigure ($Name)" { idf.py -C $firmware -B $build reconfigure | Out-Host }
    } finally {
        $env:IDF_COMPONENT_MANAGER = $saved
    }
    $helper = Join-Path $PSScriptRoot 'compile_main_sources.py'
    Invoke-Native "compiling firmware\main ($Name)" { python $helper $build | Out-Host }
    $logPath = Join-Path $build 'compile_check.txt'
    [pscustomobject]@{ Variant = $Name; Result = (Get-Content -LiteralPath $logPath -TotalCount 1); Log = $logPath }
}

# Relative paths are taken from the current location. A trailing backslash would break the quoting of a
# native argument that has a space, so it is dropped (except after a drive letter).
function Resolve-FullPath([string]$Path) {
    $full = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Path)
    if ($full.Length -gt 3) { $full = $full.TrimEnd('\') }
    $full
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw 'git not found on PATH' }
if (-not (Test-Path -LiteralPath $Tar)) { throw "$Tar not found (Windows 10 1803 and later include it)" }
if (-not $RepoRoot) { $RepoRoot = Join-Path $PSScriptRoot '..\..' }
if (-not $OutDir) { $OutDir = Join-Path $PSScriptRoot 'images' }
$RepoRoot = Resolve-FullPath $RepoRoot
$OutDir = Resolve-FullPath $OutDir
# git prints paths in UTF-8, which Windows PowerShell misreads, so only the relative way up is read.
try {
    $up = @(Invoke-Native 'git rev-parse --show-cdup' { git -C $RepoRoot rev-parse --show-cdup })
} catch {
    throw "$RepoRoot is not a git checkout of the Squid-Robot repository ($($_.Exception.Message))"
}
if ($up.Count -gt 0 -and $up[0]) { $RepoRoot = Resolve-FullPath (Join-Path $RepoRoot $up[0]) }
try {
    Invoke-Native "git cat-file $SourceCommit" { git -C $RepoRoot cat-file -e "$SourceCommit^{commit}" }
} catch {
    throw "commit $SourceCommit is not in $RepoRoot ($($_.Exception.Message)); run 'git fetch origin' there, or 'git fetch --unshallow origin' in a shallow clone"
}
if (-not $WorkRoot) { $WorkRoot = Join-Path ([IO.Path]::GetPathRoot($RepoRoot)) 'sqfw' }
$WorkRoot = Resolve-FullPath $WorkRoot
if ($WorkRoot -match '[^\x21-\x7E]') { throw "-WorkRoot must be plain ASCII without spaces (ESP-IDF cannot build elsewhere): '$WorkRoot'" }

if (-not $PrepareOnly) {
    $idf = Get-IdfVersion
    $gcc = @(Invoke-Native 'xtensa-esp32s3-elf-gcc --version' { xtensa-esp32s3-elf-gcc --version })[0]
    if (-not $CompileCheck) {
        if ($idf -ne $RequiredIdf) {
            throw "ESP-IDF $idf is active; C0 was built with $RequiredIdf. Activate $RequiredIdf, or use -CompileCheck, which makes no image."
        }
        if ($gcc -notmatch [regex]::Escape($RequiredToolchain)) {
            throw "the compiler is '$gcc'; C0 was built with $RequiredToolchain"
        }
    }
    Write-Host "ESP-IDF $idf, $gcc"
}

$results = @()
foreach ($name in $Variant) {
    $dest = Join-Path $WorkRoot $name.ToLowerInvariant()
    New-WorkTree $dest $name
    Export-Sources $dest
    Assert-Pins $dest $SourcePins "sources at $($SourceCommit.Substring(0, 8))"
    $reference = Get-TreeHashes $dest
    Disable-Student $dest
    if ($name -eq 'B') { Add-OptionB $dest }
    $changed = Get-ChangedFiles $reference (Get-TreeHashes $dest)
    $expected = @($VariantPins[$name].Keys | Sort-Object)
    if (($changed -join '|') -ne ($expected -join '|')) {
        throw "$name copy differs from the exported sources in [$($changed -join ', ')], expected [$($expected -join ', ')]"
    }
    Assert-Pins $dest $VariantPins[$name] $name
    Write-Host "$name prepared in $dest from $($SourceCommit.Substring(0, 8)) ($($reference.Count) files; changed: $($changed -join ', '))"
    if ($PrepareOnly) { continue }
    if ($CompileCheck) { $results += Invoke-CompileCheck $dest $name }
    else { $results += Invoke-Build $dest $name $changed }
}

if ($results.Count -gt 0) {
    $results | Format-Table -AutoSize | Out-String | Write-Host
    if ($CompileCheck) { Write-Host 'Compile check: the sources of firmware\main compiled; no image was made.' }
    else { Write-Host "Images, ELF files and build records: $OutDir" }
}
