<#
.SYNOPSIS
  geo2map_engine CI: every build leg, the deterministic-math grep and the
  cross-leg golden-hash compare.

.DESCRIPTION
  tools\ci.ps1                     every leg (Windows + WSL), grep, golden compare;
                                   ctest skips the tests tagged [long]
  tools\ci.ps1 -Full               the same, every test incl. [long] (a test taking
                                   >= 5 s in release gets the tag); before a phase counts done
  tools\ci.ps1 -SkipWsl            Windows legs only
  tools\ci.ps1 -SkipWindows        WSL legs only
  tools\ci.ps1 -Only release,linux-release   named legs only (preset names)

  Windows legs (Ninja, from a VS 2022 Build Tools environment entered via
  vcvars64.bat): debug, release, asan (clang-cl), msvc-release (cl).
  WSL legs (clang + lld, build dir ~/build/<checkout dir name>/<preset>, e.g.
  ~/build/geo2map_engine/<preset> for the main checkout, never under /mnt/s):
  linux-release, linux-asan-ubsan, linux-tsan.

  Per leg: cmake configure, build, ctest (without the [long]-tagged tests; -Full runs every test), then
  g2m_golden_dump for each golden set (g0, g1). The golden compare fails
  unless every leg's dump of a set equals testdata/golden/<set>_golden.txt
  byte for byte (comments stripped). On the WSL legs listed in $FuzzLegs,
  the libFuzzer targets (tests/fuzz_formats) are built and run briefly as
  CTest tests labelled `fuzz`; a `fuzz` result line counts them.
  The grep fails if include/ or src/ call a platform-libm transcendental
  (det_math.h/.cpp and third_party/musl_math are exempt); it runs a
  self-test of its own pattern first.

  Also (Windows only, cheap): "consumer-check" configures a SEPARATE build
  dir (out/build/consumer-check) with -DG2M_BUILD_IMPORT=OFF
  -DG2M_BUILD_TESTS=OFF -DG2M_BUILD_APPS=OFF (PLAN.md G2.0 Task D - a pure
  consumer of the tile-serving stack, e.g. a future game bridge, never
  needs the decoder/importer stack or its FetchContent deps) and builds
  only g2m_layers/g2m_server/g2m_client - proving that configuration
  actually works, not just that the option exists.

  Build/test output goes to out/ci_logs/<leg>.log; the console shows the
  per-leg summary, and the log tail of anything that failed. Exits
  non-zero on any failure.
#>
[CmdletBinding()]
param(
    [switch]$SkipWsl,
    [switch]$SkipWindows,
    [switch]$Full,
    [string[]]$Only = @()
)

$repoRoot = Split-Path -Parent $PSScriptRoot
# Windows PowerShell 5.1's redirection operators write UTF-16 by default.
$PSDefaultParameterValues['Out-File:Encoding'] = 'utf8'
# This checkout's WSL path, derived from $repoRoot so every worktree runs its
# WSL legs against itself: S:\claude_code\geo2map_portal -> /mnt/s/claude_code/geo2map_portal.
# Hand conversion (the same as Invoke-Wsl's log path), no extra wsl call.
$wslRoot = '/mnt/' + $repoRoot.Substring(0, 1).ToLower() + ($repoRoot.Substring(2) -replace '\\', '/')
# Per-checkout WSL build dir, shared with no other worktree (CMakePresets.json
# binaryDir uses ${sourceDirName}): ~/build/<checkout dir name>/<preset>.
$wslBuildRoot = '$HOME/build/' + (Split-Path -Leaf $repoRoot)
# Regular runs skip the [long] tests (the Catch2 tags are CTest labels, ADD_TAGS_AS_LABELS); -Full runs everything.
$ctestFilter = if ($Full) { @() } else { @('-LE', 'long') }
$logDir = Join-Path $repoRoot 'out\ci_logs'
$goldenDir = Join-Path $repoRoot 'out\ci_golden'
New-Item -ItemType Directory -Force $logDir, $goldenDir | Out-Null

$windowsLegs = @('debug', 'release', 'asan', 'msvc-release')
$wslLegs = @('linux-release', 'linux-asan-ubsan', 'linux-tsan')
$consumerLeg = 'consumer-check'   # PLAN.md G2.0 Task D: Windows-only, configure + partial build
# PLAN.md R2.2 section 1c (bridges/physics_sim, G5/G6): deliberately NOT part
# of $windowsLegs / the default run - it embeds a physics_sim checkout via
# G2M_PS_SOURCE_DIR (CMakePresets.json's own "bridge-release" preset) and
# needs a local physics_sim clone prepared first (Invoke-PreparePsPin below),
# so every existing CI invocation (bare `tools\ci.ps1`, `-Full`, etc.) stays
# completely unaffected. Only runs via an explicit `-Only bridge-release`
# (alongside any other named leg, e.g. `-Only release,bridge-release`).
$bridgeLeg = 'bridge-release'

# -Only accepts "a,b" as one string too.
$Only = @($Only | ForEach-Object { $_ -split ',' } | Where-Object { $_ } | ForEach-Object { $_.Trim() })
foreach ($name in $Only) {
    if (($windowsLegs + $wslLegs + @($consumerLeg, $bridgeLeg)) -notcontains $name) {
        throw "-Only: unknown leg '$name' (known: $(($windowsLegs + $wslLegs + @($consumerLeg, $bridgeLeg)) -join ', '))"
    }
}

$script:Results = New-Object System.Collections.Generic.List[object]
$script:GoldenDumps = New-Object System.Collections.Generic.List[object]
$script:GoldenSets = @('g0', 'g1', 'g1c', 'g2', 'g2mesh', 'g2roads', 'g2paint', 'g3xs', 'g3j', 'g3d', 'g3c', 'g3rail')   # testdata/golden/<set>_golden.txt
$script:FuzzLegs = @('linux-release', 'linux-asan-ubsan')  # G2M_BUILD_FUZZERS=ON (CMakePresets.json)

function Add-Result {
    param([string]$Leg, [string]$Stage, [bool]$Pass, [string]$Detail = '', [double]$Seconds = 0)
    $script:Results.Add([pscustomobject]@{ Leg = $Leg; Stage = $Stage; Pass = $Pass; Detail = $Detail; Seconds = $Seconds })
    $status = if ($Pass) { 'PASS' } else { 'FAIL' }
    $color = if ($Pass) { 'Green' } else { 'Red' }
    Write-Host ("  {0,-17} {1,-10} {2}  {3,7:N1}s  {4}" -f $Leg, $Stage, $status, $Seconds, $Detail) -ForegroundColor $color
}

function Should-Run([string]$Leg) {
    return ($Only.Count -eq 0) -or ($Only -contains $Leg)
}

function Show-LogTail([string]$Path, [int]$Lines = 40) {
    if (Test-Path $Path) { Get-Content -LiteralPath $Path -Tail $Lines | ForEach-Object { Write-Host "    $_" } }
}

# vcvars64.bat + parsing `set` (Launch-VsDevShell.ps1 is unreliable
# non-interactively). Do not run this under $ErrorActionPreference='Stop':
# vcvars64.bat's own vswhere lookup writes a harmless error to stderr.
function Enter-VsDevShell {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found; run tools\setup_dev_env.ps1" }
    $vsPath = & $vswhere -products * -latest -property installationPath
    if (-not $vsPath) { throw "vswhere found no VS installation" }
    $vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found under $vsPath" }
    $envLines = cmd.exe /c "`"$vcvars`" && set" 2>$null
    foreach ($line in $envLines) {
        if ($line -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process') }
    }
    if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) { throw "vcvars64.bat ran but clang-cl is not on PATH" }
}

function Get-CtestSummary([string]$Path) {
    $line = Select-String -LiteralPath $Path -Pattern 'tests passed' -ErrorAction SilentlyContinue | Select-Object -Last 1
    if ($line) { return $line.Line.Trim() } else { return '' }
}

# ---------------------------------------------------------------------------
function Invoke-WindowsLeg([string]$Preset) {
    Write-Host ""
    Write-Host "=== Windows leg: $Preset ===" -ForegroundColor Cyan
    $log = Join-Path $logDir "$Preset.log"
    Set-Content -LiteralPath $log -Value "" -Encoding utf8
    Push-Location $repoRoot
    try {
        $t = Measure-Command { & cmake --preset $Preset *>> $log }
        Add-Result $Preset 'configure' ($LASTEXITCODE -eq 0) '' $t.TotalSeconds
        if ($LASTEXITCODE -ne 0) { Show-LogTail $log; return }

        $t = Measure-Command { & cmake --build --preset $Preset *>> $log }
        Add-Result $Preset 'build' ($LASTEXITCODE -eq 0) '' $t.TotalSeconds
        if ($LASTEXITCODE -ne 0) { Show-LogTail $log 60; return }

        $ctestLog = Join-Path $logDir "$Preset.ctest.log"
        $t = Measure-Command { & ctest --preset $Preset --output-on-failure --parallel 12 @ctestFilter *> $ctestLog }
        $ok = ($LASTEXITCODE -eq 0)
        Add-Result $Preset 'ctest' $ok (Get-CtestSummary $ctestLog) $t.TotalSeconds
        if (-not $ok) { Show-LogTail $ctestLog 60 }

        $dump = Join-Path $repoRoot "out\build\$Preset\tests\golden\g2m_golden_dump.exe"
        foreach ($set in $script:GoldenSets) {
            $out = Join-Path $goldenDir "$Preset.$set.txt"
            if (Test-Path $out) { Remove-Item -LiteralPath $out }
            & $dump $set $out
            $ok = ($LASTEXITCODE -eq 0) -and (Test-Path $out)
            Add-Result $Preset "golden-$set" $ok 'dump written'
            if ($ok) { $script:GoldenDumps.Add([pscustomobject]@{ Leg = $Preset; Set = $set; Path = $out }) }
        }
    } finally {
        Pop-Location
    }
}

# PLAN.md G2.0 Task D: prove a pure consumer configuration (no decoders/
# importer/tests/apps) actually configures and builds, in its own build dir
# so it never disturbs the real "debug"/"release"/... build dirs. Cheap:
# Catch2 and the decoder FetchContent deps are never fetched (G2M_BUILD_TESTS
# and G2M_BUILD_IMPORT are both OFF), and only the three consumer-facing
# targets are built, not the whole tree.
function Invoke-ConsumerCheck {
    Write-Host ""
    Write-Host "=== Windows leg: $consumerLeg ===" -ForegroundColor Cyan
    $log = Join-Path $logDir "$consumerLeg.log"
    Set-Content -LiteralPath $log -Value "" -Encoding utf8
    Push-Location $repoRoot
    try {
        $t = Measure-Command { & cmake --preset $consumerLeg *>> $log }
        Add-Result $consumerLeg 'configure' ($LASTEXITCODE -eq 0) '' $t.TotalSeconds
        if ($LASTEXITCODE -ne 0) { Show-LogTail $log; return }

        $t = Measure-Command {
            & cmake --build --preset $consumerLeg --target g2m_layers --target g2m_server --target g2m_client *>> $log
        }
        Add-Result $consumerLeg 'build' ($LASTEXITCODE -eq 0) 'g2m_layers, g2m_server, g2m_client' $t.TotalSeconds
        if ($LASTEXITCODE -ne 0) { Show-LogTail $log 60 }
    } finally {
        Pop-Location
    }
}

# PLAN.md R2.2 section 1c: the bridge-release preset's own FETCHCONTENT_
# SOURCE_DIR_* cache variables (CMakePresets.json), listed here too so this
# script can fail with an ACTIONABLE message before ever invoking cmake -
# FetchContent's own error for a missing FETCHCONTENT_FULLY_DISCONNECTED
# source dir is far less clear. Two are physics_sim's own (JoltPhysics,
# nlohmann_json - always fetched by ps_core regardless of PS_BUILD_TESTS);
# the rest are geo2map_engine's own FetchContent deps (Catch2 plus, since
# the preset sets G2M_BUILD_IMPORT=ON, every decoder dependency) - reused
# from this repo's own "release" preset build, never re-downloaded.
$script:BridgeOfflineDeps = @(
    'S:\claude_code\physics_sim\out\build\release\_deps\joltphysics-src',
    'S:\claude_code\physics_sim\out\build\release\_deps\nlohmann_json-src',
    (Join-Path $repoRoot 'out\build\release\_deps\catch2-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\g2m_zstd-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\g2m_sqlite-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\g2m_json-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\libdeflate-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\zlibng-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\protozero-src'),
    (Join-Path $repoRoot 'out\build\release\_deps\libosmium-src')
)

function Test-BridgeOfflineDeps {
    $missing = @($script:BridgeOfflineDeps | Where-Object { -not (Test-Path $_) })
    if ($missing.Count -eq 0) { return $true }
    Write-Host "bridge-release: missing offline FetchContent source dir(s):" -ForegroundColor Red
    $missing | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
    Write-Host "  physics_sim's two (joltphysics-src, nlohmann_json-src): build physics_sim's own 'release' preset first (cmake --preset release && cmake --build --preset release)." -ForegroundColor Red
    Write-Host "  geo2map_engine's own (everything else above): build this repo's own 'release' preset first." -ForegroundColor Red
    return $false
}

# Prepares out/ps_pin/src as a local (no-network, --local) clone of
# physics_sim checked out to bridges/physics_sim/physics_sim.pin's commit -
# reused as-is if it is already there (matching by HEAD, short-sha aware).
# physics_sim itself is never touched (a plain read-only `git clone --local`
# + `git checkout` inside the clone).
function Invoke-PreparePsPin {
    Write-Host ""
    Write-Host "=== Preparing physics_sim pin (out/ps_pin/src) ===" -ForegroundColor Cyan
    $pinFile = Join-Path $repoRoot 'bridges\physics_sim\physics_sim.pin'
    if (-not (Test-Path $pinFile)) { throw "bridge-release: pin file not found: $pinFile" }
    $pin = (Get-Content -LiteralPath $pinFile -Raw).Trim()
    if (-not $pin) { throw "bridge-release: $pinFile is empty" }

    $physSimSrc = 'S:\claude_code\physics_sim'
    if (-not (Test-Path (Join-Path $physSimSrc '.git'))) {
        throw "bridge-release: physics_sim source not found at $physSimSrc (expected a git checkout)"
    }

    $pinRoot = Join-Path $repoRoot 'out\ps_pin'
    $clone = Join-Path $pinRoot 'src'
    New-Item -ItemType Directory -Force $pinRoot | Out-Null

    $needClone = $true
    if (Test-Path (Join-Path $clone '.git')) {
        Push-Location $clone
        try {
            $head = (& git rev-parse HEAD).Trim()
            if ($LASTEXITCODE -eq 0 -and $head -like "$pin*") {
                Write-Host "  out/ps_pin/src already at pin $pin ($head) - reusing" -ForegroundColor DarkGray
                $needClone = $false
            }
        } finally { Pop-Location }
    }

    if ($needClone) {
        if (Test-Path $clone) { Remove-Item -LiteralPath $clone -Recurse -Force }
        Write-Host "  cloning $physSimSrc -> $clone (--local, no network)" -ForegroundColor DarkGray
        & git clone --local --quiet $physSimSrc $clone
        if ($LASTEXITCODE -ne 0) { throw "bridge-release: git clone --local $physSimSrc -> $clone failed (exit $LASTEXITCODE)" }
        Push-Location $clone
        try {
            & git checkout --quiet $pin
            if ($LASTEXITCODE -ne 0) { throw "bridge-release: git checkout $pin failed inside $clone (exit $LASTEXITCODE)" }
        } finally { Pop-Location }
    }
}

function Invoke-BridgeReleaseLeg {
    Write-Host ""
    Write-Host "=== Windows leg: $bridgeLeg ===" -ForegroundColor Cyan
    if (-not (Test-BridgeOfflineDeps)) {
        Add-Result $bridgeLeg 'configure' $false 'missing offline FetchContent source dir(s), see above'
        return
    }
    try {
        Invoke-PreparePsPin
    } catch {
        Add-Result $bridgeLeg 'configure' $false "physics_sim pin prep failed: $($_.Exception.Message)"
        return
    }
    Invoke-WindowsLeg $bridgeLeg
}

# ---------------------------------------------------------------------------
function Invoke-Wsl([string]$Command, [string]$LogWin) {
    # One bash -lc string; the log is written inside WSL to the Windows path.
    $p = $LogWin -replace '\\', '/'
    $logWsl = '/mnt/' + $p.Substring(0, 1).ToLower() + $p.Substring(2)
    & wsl --cd ~ -- bash -lc "cd $wslRoot && ( $Command ) >> '$logWsl' 2>&1"
    return $LASTEXITCODE
}

function Invoke-WslLeg([string]$Preset) {
    Write-Host ""
    Write-Host "=== WSL leg: $Preset ===" -ForegroundColor Cyan
    $log = Join-Path $logDir "$Preset.log"
    Set-Content -LiteralPath $log -Value "" -Encoding utf8

    $rc = 0
    $t = Measure-Command { $script:rc = Invoke-Wsl "cmake --preset $Preset" $log }
    Add-Result $Preset 'configure' ($script:rc -eq 0) '' $t.TotalSeconds
    if ($script:rc -ne 0) { Show-LogTail $log; return }

    $t = Measure-Command { $script:rc = Invoke-Wsl "cmake --build --preset $Preset" $log }
    Add-Result $Preset 'build' ($script:rc -eq 0) '' $t.TotalSeconds
    if ($script:rc -ne 0) { Show-LogTail $log 60; return }

    $ctestLog = Join-Path $logDir "$Preset.ctest.log"
    Set-Content -LiteralPath $ctestLog -Value "" -Encoding utf8
    $t = Measure-Command { $script:rc = Invoke-Wsl "ctest --preset $Preset --output-on-failure --parallel 12 $($ctestFilter -join ' ')" $ctestLog }
    Add-Result $Preset 'ctest' ($script:rc -eq 0) (Get-CtestSummary $ctestLog) $t.TotalSeconds
    if ($script:rc -ne 0) { Show-LogTail $ctestLog 60 }

    if ($script:FuzzLegs -contains $Preset) {
        # Short libFuzzer runs (PLAN.md 2.4: WSL clang legs only). They are
        # CTest tests labelled `fuzz`, so the ctest above already ran them;
        # this line makes their presence visible - zero means the fuzzers
        # were not built.
        $n = @(Select-String -LiteralPath $ctestLog -Pattern 'Test +#\d+: fuzz_\S+ \.+ +Passed' -ErrorAction SilentlyContinue).Count
        $nAll = @(Select-String -LiteralPath $ctestLog -Pattern 'Test +#\d+: fuzz_\S+ ' -ErrorAction SilentlyContinue).Count
        Add-Result $Preset 'fuzz' (($n -gt 0) -and ($n -eq $nAll)) "$n of $nAll libFuzzer runs passed"
    }

    foreach ($set in $script:GoldenSets) {
        $out = Join-Path $goldenDir "$Preset.$set.txt"
        if (Test-Path $out) { Remove-Item -LiteralPath $out }
        $outWsl = "$wslRoot/out/ci_golden/$Preset.$set.txt"
        $script:rc = Invoke-Wsl "$wslBuildRoot/$Preset/tests/golden/g2m_golden_dump $set '$outWsl'" $log
        $ok = ($script:rc -eq 0) -and (Test-Path $out)
        Add-Result $Preset "golden-$set" $ok 'dump written'
        if ($ok) { $script:GoldenDumps.Add([pscustomobject]@{ Leg = $Preset; Set = $set; Path = $out }) }
    }
}

# ---------------------------------------------------------------------------
# Deterministic-math grep (PLAN.md 2.4). A banned name is a libm
# transcendental called unqualified, std::- or ::-qualified; the lookbehind
# (no word character or ':' right before) lets det::sin(, atan2d( and
# identifiers such as my_exp( or expect( through. // comments are stripped
# first. <complex> is banned as well: std::complex's functions call libm.
# ---------------------------------------------------------------------------
$script:BannedNames = @('sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'sinh', 'cosh', 'tanh', 'asinh',
                        'acosh', 'atanh', 'exp', 'exp2', 'expm1', 'log', 'log2', 'log10', 'log1p', 'pow', 'hypot',
                        'cbrt', 'sincos', 'erf', 'erfc', 'tgamma', 'lgamma')

function Get-BannedCallPattern {
    $alt = ($script:BannedNames | ForEach-Object { [regex]::Escape($_) }) -join '|'
    return "(?<![\w:])(?:std::|::)?(?:$alt)[fl]?\s*\("
}
$script:ComplexPattern = '#\s*include\s*<complex>'

function Test-BannedLine([string]$Line) {
    $code = $Line -replace '//.*$', ''
    return ($code -match (Get-BannedCallPattern)) -or ($code -match $script:ComplexPattern)
}

function Test-GrepPattern {
    $must = @('return std::sin(x);', 'z = ::cos(x);', 'auto y = atan2(a, b);', 'h = std::hypot(x, y);',
              'q = sinhl(x);', 'r = std::atanh (x);', 'pow(x, 2.0);', '#include <complex>', 'double log(double x);')
    $mustNot = @('return det::sin(x);', 'return g2m::det::atan2(y, x);', 'double s = atan2d(y, x);',
                 'bool b = expect(x);', 'int my_exp(int);', '// uses std::sin(x) here', 'x = std::sqrt(y);',
                 'x = std::fmod(y, 360.0);', 'auto e = exp_data_table;', 'sincosd(x, s, c);', 'ang_normalize(x);')
    $fail = 0
    foreach ($s in $must) { if (-not (Test-BannedLine $s)) { Write-Host "  [self-test] expected MATCH: '$s'" -ForegroundColor Red; $fail++ } }
    foreach ($s in $mustNot) { if (Test-BannedLine $s) { Write-Host "  [self-test] expected NO match: '$s'" -ForegroundColor Red; $fail++ } }
    Add-Result 'libm-grep' 'self-test' ($fail -eq 0) "$($must.Count + $mustNot.Count) cases, $fail failure(s)"
}

function Invoke-LibmGrep {
    Write-Host ""
    Write-Host "=== Deterministic-math grep (include/, src/) ===" -ForegroundColor Cyan
    Test-GrepPattern
    $exempt = @((Join-Path $repoRoot 'include\g2m\core\det_math.h'), (Join-Path $repoRoot 'src\core\det_math.cpp'))
    $files = Get-ChildItem -Path (Join-Path $repoRoot 'include'), (Join-Path $repoRoot 'src') -Recurse -File `
                 -Include *.h, *.hpp, *.hh, *.hxx, *.inl, *.c, *.cc, *.cpp, *.cxx |
             Where-Object { $exempt -notcontains $_.FullName }
    $hits = @()
    foreach ($f in $files) {
        $lines = @(Get-Content -LiteralPath $f.FullName)
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if (Test-BannedLine $lines[$i]) { $hits += "$($f.FullName):$($i + 1): $($lines[$i].Trim())" }
        }
    }
    if ($hits.Count -gt 0) {
        Add-Result 'libm-grep' 'grep' $false "$($hits.Count) match(es) in $($files.Count) files"
        $hits | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
    } else {
        Add-Result 'libm-grep' 'grep' $true "clean ($($files.Count) files; det_math.h/.cpp exempt)"
    }
}

# ---------------------------------------------------------------------------
function Get-GoldenBody([string]$Path) {
    return ((Get-Content -LiteralPath $Path | Where-Object { $_ -and -not $_.StartsWith('#') }) -join "`n")
}

function Invoke-GoldenCompare {
    Write-Host ""
    Write-Host "=== Golden-hash compare across legs ===" -ForegroundColor Cyan
    if ($script:GoldenDumps.Count -eq 0) {
        Add-Result 'golden' 'compare' $false 'no leg produced a dump'
        return
    }
    foreach ($set in $script:GoldenSets) {
        $file = "testdata/golden/$($set)_golden.txt"
        $expected = Get-GoldenBody (Join-Path $repoRoot $file)
        $dumps = @($script:GoldenDumps | Where-Object { $_.Set -eq $set })
        if ($dumps.Count -eq 0) {
            Add-Result 'golden' "compare-$set" $false 'no leg produced a dump'
            continue
        }
        $bad = @($dumps | Where-Object { (Get-GoldenBody $_.Path) -ne $expected })
        $legs = ($dumps | ForEach-Object { $_.Leg }) -join ', '
        if ($bad.Count -eq 0) {
            $all = ($expected -split "`n" | Where-Object { $_.StartsWith('all ') })
            Add-Result 'golden' "compare-$set" $true "identical on $($dumps.Count) legs ($legs): $all"
        } else {
            Add-Result 'golden' "compare-$set" $false "differs from $($file): $(($bad | ForEach-Object { $_.Leg }) -join ', ')"
            foreach ($d in $bad) {
                Write-Host "  --- $($d.Leg) vs committed $file ---" -ForegroundColor Red
                Compare-Object ($expected -split "`n") ((Get-GoldenBody $d.Path) -split "`n") |
                    ForEach-Object { Write-Host "    $($_.SideIndicator) $($_.InputObject)" }
            }
        }
    }
}

# ---------------------------------------------------------------------------
$start = Get-Date
Invoke-LibmGrep

# bridge-release only ever runs when explicitly named via -Only (see
# $bridgeLeg's own comment above) - Should-Run alone would also select it on
# a bare, no-args run (Only.Count -eq 0 there means "run everything").
$runBridgeLeg = ($Only.Count -gt 0) -and (Should-Run $bridgeLeg)

if (-not $SkipWindows -and (($windowsLegs + @($consumerLeg) | Where-Object { Should-Run $_ }) -or $runBridgeLeg)) {
    Enter-VsDevShell
    foreach ($leg in $windowsLegs) { if (Should-Run $leg) { Invoke-WindowsLeg $leg } }
    if (Should-Run $consumerLeg) { Invoke-ConsumerCheck }
    if ($runBridgeLeg) { Invoke-BridgeReleaseLeg }
}
if (-not $SkipWsl -and ($wslLegs | Where-Object { Should-Run $_ })) {
    foreach ($leg in $wslLegs) { if (Should-Run $leg) { Invoke-WslLeg $leg } }
}
Invoke-GoldenCompare

Write-Host ""
Write-Host "=== Summary ===" -ForegroundColor Cyan
foreach ($r in $script:Results) {
    $status = if ($r.Pass) { 'PASS' } else { 'FAIL' }
    $color = if ($r.Pass) { 'Green' } else { 'Red' }
    Write-Host ("  {0,-17} {1,-10} {2}  {3,7:N1}s  {4}" -f $r.Leg, $r.Stage, $status, $r.Seconds, $r.Detail) -ForegroundColor $color
}
$failed = @($script:Results | Where-Object { -not $_.Pass })
$total = ((Get-Date) - $start).TotalSeconds
if ($failed.Count -eq 0) {
    Write-Host ("ALL PASS ({0} checks, {1:N0} s)" -f $script:Results.Count, $total) -ForegroundColor Green
    exit 0
}
Write-Host ("FAILED: {0} of {1} checks ({2:N0} s)" -f $failed.Count, $script:Results.Count, $total) -ForegroundColor Red
exit 1
