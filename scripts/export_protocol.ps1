# Export the Pydantic protocol to JSON Schema and TypeScript types.
[CmdletBinding()]
param(
    [string]$Python = '',
    [string]$SchemaDir,
    [string]$TsFile
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $Python) {
    $Python = if ($env:DSH_ISAAC_PYTHON) { $env:DSH_ISAAC_PYTHON } else { 'python' }
}
$arguments = @((Join-Path $root 'scripts\export_protocol.py'))
if ($SchemaDir) { $arguments += @('--schema-dir', $SchemaDir) }
if ($TsFile) { $arguments += @('--ts-file', $TsFile) }

& $Python @arguments
exit $LASTEXITCODE
