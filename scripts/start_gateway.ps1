# Start the Robotics Gateway (loopback worker for T03 tests).
[CmdletBinding()]
param(
    [int]$Port = 8710,
    [string]$Db = 'artifacts\gateway.db',
    [string]$Python = '',
    [string]$Worker = 'loopback'
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $Python) {
    $Python = if ($env:DSH_ISAAC_PYTHON) { $env:DSH_ISAAC_PYTHON } else { 'python' }
}
$env:PYTHONPATH = "$root\python"
$dbPath = Join-Path $root $Db

& $Python -m robotics_gateway --host 127.0.0.1 --port $Port --db $dbPath --worker $Worker
exit $LASTEXITCODE
