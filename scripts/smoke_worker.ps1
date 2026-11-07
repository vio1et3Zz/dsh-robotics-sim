# T02 smoke: launch the real Franka PhysX scene and run one bounded calibration.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$OutputDir,
    [string]$Python = '',
    [string]$Joint = 'panda_joint1',
    [double]$DeltaRad = 0.05,
    [double]$HoldS = 0.25,
    [string]$Device = 'cuda:0'
)

$ErrorActionPreference = 'Continue'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (Test-Path $OutputDir) {
    throw "output directory already exists: $OutputDir"
}

# Mirror the Isaac Lab launcher: keep the Isaac source on PYTHONPATH and clear
# stale Kit path variables.  Override the interpreter/source with
# DSH_ISAAC_PYTHON / DSH_ISAACLAB_SOURCE.
if (-not $Python) {
    $Python = if ($env:DSH_ISAAC_PYTHON) { $env:DSH_ISAAC_PYTHON } else { 'python' }
}
$defaultIsaacSource = Join-Path $env:USERPROFILE 'isaac\IsaacLab\source\isaaclab'
$isaacSource = if ($env:DSH_ISAACLAB_SOURCE) {
    $env:DSH_ISAACLAB_SOURCE
} elseif (Test-Path $defaultIsaacSource) {
    $defaultIsaacSource
} else { '' }
$pythonPath = "$root\python"
if ($isaacSource) { $pythonPath = "$pythonPath;$isaacSource" }
$env:PYTHONPATH = $pythonPath
$env:ISAAC_PATH = ''
$env:EXP_PATH = ''
$env:CARB_APP_PATH = ''
$env:PYTHONHOME = ''

$entry = Join-Path $root 'python\robotics_backends\isaaclab_physx\entry.py'
$stdoutLog = "$OutputDir.stdout.log"
$stderrLog = "$OutputDir.stderr.log"
& $Python $entry --headless --device $Device --output_dir $OutputDir --joint $Joint --delta_rad $DeltaRad --hold_s $HoldS 2> $stderrLog | Out-File -Encoding utf8 $stdoutLog
$exitCode = $LASTEXITCODE
if (Test-Path $OutputDir) {
    if (Test-Path $stdoutLog) { Move-Item -Force $stdoutLog (Join-Path $OutputDir 'stdout.log') }
    if (Test-Path $stderrLog) { Move-Item -Force $stderrLog (Join-Path $OutputDir 'stderr.log') }
}
exit $exitCode
