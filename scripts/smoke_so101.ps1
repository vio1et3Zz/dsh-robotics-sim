# T05 smoke: launch the real SO-ARM101 scene and run the joint/gripper calibration.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$OutputDir,
    [string]$Python = '',
    [string]$Device = 'cuda:0'
)

$ErrorActionPreference = 'Continue'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (Test-Path $OutputDir) {
    throw "output directory already exists: $OutputDir"
}

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

$entry = Join-Path $root 'python\robotics_backends\isaaclab_physx\so101_entry.py'
$stdoutLog = "$OutputDir.stdout.log"
$stderrLog = "$OutputDir.stderr.log"
& $Python $entry --headless --device $Device --output_dir $OutputDir 2> $stderrLog | Out-File -Encoding utf8 $stdoutLog
$exitCode = $LASTEXITCODE
if (Test-Path $OutputDir) {
    if (Test-Path $stdoutLog) { Move-Item -Force $stdoutLog (Join-Path $OutputDir 'stdout.log') }
    if (Test-Path $stderrLog) { Move-Item -Force $stderrLog (Join-Path $OutputDir 'stderr.log') }
}
exit $exitCode
