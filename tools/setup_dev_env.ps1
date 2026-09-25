#Requires -Version 5.1
<#
.SYNOPSIS
    Verifies (or installs) the geo2map_engine toolchain: VS 2022 Build Tools
    (cl + the bundled clang-cl), CMake, Ninja, git on Windows; clang, lld,
    cmake, ninja, git and addr2line in the WSL Ubuntu distro.

.DESCRIPTION
    tools\setup_dev_env.ps1 -CheckOnly   verify only: prints every tool's
                                         version, installs nothing, exits
                                         non-zero if anything is missing
    tools\setup_dev_env.ps1              install whatever is missing
                                         (winget on Windows, apt in WSL);
                                         idempotent, skips what is present

    The same toolchain as physics_sim (so a machine set up for one builds
    both); nothing geo2map-specific is installed. Third-party libraries
    (Catch2, GeographicLib for tools/refgen) come from CMake FetchContent at
    pinned tags, never from here. clang-cl is the Build Tools' own
    VC.Llvm.Clang component, never the standalone LLVM winget package.
#>
[CmdletBinding()]
param([switch]$CheckOnly)

$script:AllOk = $true

function Write-Section($title) { Write-Host ""; Write-Host "=== $title ===" -ForegroundColor Cyan }
function Write-Ok($msg) { Write-Host "  [OK]   $msg" -ForegroundColor Green }
function Write-Missing($msg) { Write-Host "  [MISS] $msg" -ForegroundColor Red; $script:AllOk = $false }

function Update-SessionPath {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Get-VsInstallPath {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $null }
    return (& $vswhere -products * -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath)
}

function Get-ClangClPath {
    $vs = Get-VsInstallPath
    if (-not $vs) { return $null }
    $p = Join-Path $vs 'VC\Tools\Llvm\x64\bin\clang-cl.exe'
    if (Test-Path $p) { return $p } else { return $null }
}

function Get-ClPath {
    $vs = Get-VsInstallPath
    if (-not $vs) { return $null }
    $cl = Get-ChildItem (Join-Path $vs 'VC\Tools\MSVC') -Directory -ErrorAction SilentlyContinue |
          Sort-Object Name -Descending |
          ForEach-Object { Join-Path $_.FullName 'bin\Hostx64\x64\cl.exe' } |
          Where-Object { Test-Path $_ } | Select-Object -First 1
    return $cl
}

function Test-Tool([string]$Name, [string]$Command, [string[]]$Arguments = @('--version')) {
    $cmd = Get-Command $Command -ErrorAction SilentlyContinue
    if (-not $cmd) { Write-Missing "$Name ('$Command' not on PATH)"; return $false }
    $out = & $Command @Arguments 2>&1 | Select-Object -First 1
    Write-Ok "$Name -> $out"
    return $true
}

$wslTools = @(
    @{ Name = 'clang';     Cmd = 'clang --version' },
    @{ Name = 'clang++';   Cmd = 'clang++ --version' },
    @{ Name = 'ld.lld';    Cmd = 'ld.lld --version' },
    @{ Name = 'cmake';     Cmd = 'cmake --version' },
    @{ Name = 'ninja';     Cmd = 'ninja --version' },
    @{ Name = 'git';       Cmd = 'git --version' },
    # linux-tsan's test preset symbolizes through /usr/bin/addr2line.
    @{ Name = 'addr2line'; Cmd = 'addr2line --version' }
)

function Test-Wsl {
    Write-Section "WSL toolchain (Ubuntu)"
    $distros = (wsl -l -q 2>$null) -replace "`0", ""
    if (-not ($distros -match 'Ubuntu')) { Write-Missing "no Ubuntu WSL distro (wsl -l -q)"; return }
    Write-Ok "WSL distro: $((@($distros) | Where-Object { $_ }) -join ', ')"
    # One trivial command per probe: multi-line strings do not survive
    # PowerShell -> wsl -> bash -lc quoting.
    foreach ($t in $wslTools) {
        $out = wsl --cd ~ -- bash -lc "$($t.Cmd) 2>&1 | head -n1"
        if ($LASTEXITCODE -eq 0 -and $out) { Write-Ok "$($t.Name) (WSL) -> $out" } else { Write-Missing "$($t.Name) (WSL)" }
    }
}

function Test-Windows {
    Write-Section "Windows toolchain"
    Test-Tool 'git' 'git' | Out-Null
    Test-Tool 'cmake' 'cmake' | Out-Null
    Test-Tool 'ninja' 'ninja' | Out-Null
    $vs = Get-VsInstallPath
    if ($vs) { Write-Ok "VS 2022 Build Tools -> $vs" } else { Write-Missing "VS 2022 Build Tools (VC.Tools.x86.x64) via vswhere" }
    $clang = Get-ClangClPath
    if ($clang) { Write-Ok "clang-cl -> $clang ($(& $clang --version | Select-Object -First 1))" }
    else { Write-Missing "clang-cl (VC.Llvm.Clang / VC.Llvm.ClangToolset components)" }
    $cl = Get-ClPath
    if ($cl) { Write-Ok "cl -> $cl" } else { Write-Missing "cl.exe (VC.Tools.x86.x64)" }
}

Update-SessionPath

if ($CheckOnly) {
    Test-Windows
    Test-Wsl
    Write-Section "Result"
    if ($script:AllOk) { Write-Host "All tools present." -ForegroundColor Green; exit 0 }
    Write-Host "One or more tools are missing. Re-run without -CheckOnly to install them." -ForegroundColor Red
    exit 1
}

# ===========================================================================
# INSTALL MODE (only what is missing)
# ===========================================================================
function Install-WingetPackage([string]$Id, [string[]]$ExtraArgs = @()) {
    Write-Host "Installing $Id via winget..."
    $a = @('install', '--id', $Id, '-e', '--accept-package-agreements', '--accept-source-agreements',
           '--disable-interactivity') + $ExtraArgs
    & winget @a
    if ($LASTEXITCODE -ne 0) { throw "winget install of $Id failed with exit code $LASTEXITCODE" }
}

Write-Section "VS 2022 Build Tools (C++ workload, Windows SDK, clang-cl)"
if ((Get-ClangClPath) -and (Get-ClPath)) {
    Write-Host "  already installed" -ForegroundColor Yellow
} else {
    Install-WingetPackage 'Microsoft.VisualStudio.2022.BuildTools' @(
        '--override', '"--wait --quiet --norestart --nocache --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --add Microsoft.VisualStudio.Component.VC.Llvm.Clang --add Microsoft.VisualStudio.Component.VC.Llvm.ClangToolset"')
}
Write-Section "CMake"
if (Get-Command cmake -ErrorAction SilentlyContinue) { Write-Host "  already installed" -ForegroundColor Yellow } else { Install-WingetPackage 'Kitware.CMake' }
Write-Section "Ninja"
if (Get-Command ninja -ErrorAction SilentlyContinue) { Write-Host "  already installed" -ForegroundColor Yellow } else { Install-WingetPackage 'Ninja-build.Ninja' }
Update-SessionPath

Write-Section "WSL Ubuntu build toolchain"
$distros = (wsl -l -q 2>$null) -replace "`0", ""
if ($distros -match 'Ubuntu') {
    wsl -u root -- apt-get update
    wsl -u root -- apt-get install -y clang lld cmake ninja-build git binutils
    if ($LASTEXITCODE -ne 0) { throw "WSL apt-get install failed with exit code $LASTEXITCODE" }
} else {
    Write-Host "  No Ubuntu WSL distro: install WSL2 + Ubuntu first (not done by this script)." -ForegroundColor Red
    $script:AllOk = $false
}

Write-Section "Done"
Write-Host "Run with -CheckOnly to verify." -ForegroundColor Cyan
if (-not $script:AllOk) { exit 1 }
