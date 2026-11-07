# DSH Robotics doctor (T00).
#
# Reports three distinct states:
#   installed  - package metadata, source checkout, paths, versions
#   services   - Gateway / Worker liveness (not implemented before T03)
#   verified   - task records that were accepted against a real backend
#
# Exit code 0 means the installed baseline matches configs/local.example.json;
# 1 means at least one baseline check failed. Service and verification state
# never changes the exit code: they are reported, not assumed.

[CmdletBinding()]
param(
    [string]$ConfigPath,
    [switch]$Json
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $ConfigPath) {
    $ConfigPath = Join-Path $root 'configs\local.json'
    if (-not (Test-Path $ConfigPath)) {
        $ConfigPath = Join-Path $root 'configs\local.example.json'
    }
}
$ConfigPath = (Resolve-Path $ConfigPath).Path
$cfg = Get-Content -Path $ConfigPath -Raw -Encoding UTF8 | ConvertFrom-Json

$checks = New-Object System.Collections.ArrayList
function Add-Check {
    param([string]$Id, [string]$Status, [string]$Detail)
    [void]$checks.Add([pscustomobject]@{ id = $Id; status = $Status; detail = $Detail })
}

$paths = [ordered]@{}
$pathPairs = @(
    @('isaac.root', $cfg.isaac.root),
    @('isaac.labSource', $cfg.isaac.labSource),
    @('isaac.python', $cfg.isaac.python),
    @('isaac.launcher', $cfg.isaac.launcher),
    @('dsh.profileDir', $cfg.dsh.profileDir)
)
foreach ($pair in $pathPairs) {
    $exists = Test-Path $pair[1]
    $paths[$pair[0]] = [ordered]@{ path = $pair[1]; exists = $exists }
    Add-Check $pair[0] $(if ($exists) { 'ok' } else { 'fail' }) $pair[1]
}

$pyMeta = $null
if (Test-Path $cfg.isaac.python) {
    $metadataScript = Join-Path $PSScriptRoot 'isaac_metadata.py'
    $raw = & $cfg.isaac.python $metadataScript 2>$null
    if ($LASTEXITCODE -eq 0 -and $raw) {
        $pyMeta = ($raw -join "`n") | ConvertFrom-Json
        Add-Check 'isaac.python.version' 'ok' $pyMeta.python
        $expected = $cfg.isaac.expectedPackages
        foreach ($name in @('isaaclab', 'isaacsim', 'isaaclab-physx', 'torch')) {
            $actual = $pyMeta.packages.$name
            $want = $expected.$name
            if ($want -and $actual -ne $want) {
                Add-Check "package.$name" 'fail' "expected $want, got $actual"
            }
            else {
                Add-Check "package.$name" 'ok' $actual
            }
        }
    }
    else {
        Add-Check 'isaac.python.metadata' 'fail' 'python did not report package metadata'
    }
}
else {
    Add-Check 'isaac.python.metadata' 'fail' 'python executable missing'
}

$gitArgs = @('-c', ('safe.directory=' + ($cfg.isaac.labSource -replace '\\', '/')), '-C', $cfg.isaac.labSource)
$describe = $null
$commit = $null
if (Get-Command git -ErrorAction SilentlyContinue) {
    $describe = (& git @gitArgs describe --tags --always --dirty 2>$null) -join ''
    $commit = (& git @gitArgs rev-parse HEAD 2>$null) -join ''
    if ($describe -like "*$($cfg.isaac.expectedTag)*") {
        Add-Check 'isaac.git.describe' 'ok' $describe
    }
    else {
        Add-Check 'isaac.git.describe' 'fail' "expected tag $($cfg.isaac.expectedTag), got $describe"
    }
    if ($commit -eq $cfg.isaac.expectedCommit) {
        Add-Check 'isaac.git.commit' 'ok' $commit
    }
    else {
        Add-Check 'isaac.git.commit' 'fail' "expected $($cfg.isaac.expectedCommit), got $commit"
    }
}
else {
    Add-Check 'isaac.git' 'fail' 'git is not available'
}

$assetRoot = $null
$kitApp = Join-Path $cfg.isaac.labSource 'apps\isaaclab.python.kit'
if (Test-Path $kitApp) {
    $kitText = Get-Content -Path $kitApp -Raw -Encoding UTF8
    if ($kitText -match 'asset_root\.default\s*=\s*"([^"]+)"') { $assetRoot = $Matches[1] }
}
if ($assetRoot -eq $cfg.isaac.expectedAssetRoot) {
    Add-Check 'isaac.assetRoot' 'ok' $assetRoot
}
else {
    Add-Check 'isaac.assetRoot' 'fail' "expected $($cfg.isaac.expectedAssetRoot), got $assetRoot"
}

$dshPkg = $null
if (Test-Path $cfg.dsh.asarPath) {
    $asarHelper = Join-Path $PSScriptRoot 'read_asar_package.py'
    $asarJson = & $cfg.isaac.python $asarHelper $cfg.dsh.asarPath 'dsh/package.json' 2>$null
    if ($LASTEXITCODE -eq 0 -and $asarJson) {
        $dshPkg = ($asarJson -join "`n") | ConvertFrom-Json
    }
}
if ($dshPkg) {
    if ($dshPkg.version -eq $cfg.dsh.runtimeVersion) {
        Add-Check 'dsh.runtime.version' 'ok' $dshPkg.version
    }
    else {
        Add-Check 'dsh.runtime.version' 'fail' "expected $($cfg.dsh.runtimeVersion), got $($dshPkg.version)"
    }
    $cordis = $dshPkg.dependencies.'@deepseek-ai/cordis'
    if ($cordis -eq $cfg.dsh.cordisVersion) {
        Add-Check 'dsh.cordis.version' 'ok' $cordis
    }
    else {
        Add-Check 'dsh.cordis.version' 'fail' "expected $($cfg.dsh.cordisVersion), got $cordis"
    }
    $tools = $dshPkg.dependencies.'@deepseek-ai/dsh-tools'
    if ($tools -eq $cfg.dsh.toolsVersion) {
        Add-Check 'dsh.tools.version' 'ok' $tools
    }
    else {
        Add-Check 'dsh.tools.version' 'fail' "expected $($cfg.dsh.toolsVersion), got $tools"
    }
}
else {
    Add-Check 'dsh.runtime' 'fail' 'cannot read dsh/package.json from app.asar'
}

$profileBundles = @()
$profilePkg = Join-Path $cfg.dsh.profileDir 'package.json'
if (Test-Path $profilePkg) {
    $profileJson = Get-Content -Path $profilePkg -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($profileJson.dsh -and $profileJson.dsh.profile) {
        $profileBundles = @($profileJson.dsh.profile.bundles)
    }
    Add-Check 'dsh.profile' 'ok' (($profileBundles -join ', '))
}
else {
    Add-Check 'dsh.profile' 'fail' "missing $profilePkg"
}

$gatewayStatus = 'not_running'
try {
    $health = Invoke-WebRequest -UseBasicParsing -Uri ($cfg.gateway.baseUrl + '/api/v1/health') -TimeoutSec 2
    $gatewayStatus = 'http_' + [int]$health.StatusCode
}
catch {
    $gatewayStatus = 'not_running'
}
Add-Check 'service.gateway' $gatewayStatus ($cfg.gateway.baseUrl + '/api/v1/health')

$passedTasks = @()
$statePath = Join-Path $root 'docs\execution\task_state.json'
if (Test-Path $statePath) {
    $state = Get-Content -Path $statePath -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($task in @($state.tasks)) {
        if ($task.status -eq 'passed') { $passedTasks += $task.id }
    }
}
Add-Check 'verified.tasks' $(if ($passedTasks.Count -gt 0) { 'ok' } else { 'none' }) (($passedTasks -join ', '))

$legacyHits = @()
$scanFiles = @()
foreach ($dir in @('configs', 'scripts', 'docs')) {
    $full = Join-Path $root $dir
    if (Test-Path $full) {
        $scanFiles += Get-ChildItem -Path $full -Recurse -File -Include *.ps1, *.json, *.md, *.py -ErrorAction SilentlyContinue
    }
}
if ($scanFiles.Count -gt 0) {
    $legacyHits = @($scanFiles | Select-String -Pattern '(?<![A-Za-z])[HJ]:\\' -ErrorAction SilentlyContinue)
}
if ($legacyHits.Count -gt 0) {
    Add-Check 'paths.legacyDrives' 'warn' ("$($legacyHits.Count) reference(s) to H:/J: drives")
}
else {
    Add-Check 'paths.legacyDrives' 'ok' 'no H:/J: references in configs, scripts or docs'
}

$failed = @($checks | Where-Object { $_.status -eq 'fail' })
$baselineOk = ($failed.Count -eq 0)

$report = [ordered]@{
    tool = 'dsh-robotics doctor'
    generatedAt = (Get-Date).ToString('o')
    config = $ConfigPath
    machine = $env:COMPUTERNAME
    user = $env:USERNAME
    baselineOk = $baselineOk
    installed = [ordered]@{
        paths = $paths
        python = $pyMeta
        git = [ordered]@{ describe = $describe; commit = $commit }
        assetRoot = $assetRoot
        dsh = [ordered]@{
            version = $(if ($dshPkg) { $dshPkg.version } else { $null })
            cordis = $(if ($dshPkg) { $dshPkg.dependencies.'@deepseek-ai/cordis' } else { $null })
            tools = $(if ($dshPkg) { $dshPkg.dependencies.'@deepseek-ai/dsh-tools' } else { $null })
            profileBundles = $profileBundles
        }
    }
    services = [ordered]@{
        gateway = $gatewayStatus
        note = 'Gateway and Worker do not exist before T03/T02; not_running is expected.'
    }
    verified = [ordered]@{
        passedTasks = $passedTasks
        note = 'Only stages accepted against a real backend belong here.'
    }
    checks = $checks
}

if ($Json) {
    $report | ConvertTo-Json -Depth 8
}
else {
    'DSH Robotics doctor'
    "config: $ConfigPath"
    "machine: $($report.machine) / $($report.user)"
    ''
    '[installed]'
    foreach ($check in $checks | Where-Object { $_.id -notlike 'service.*' -and $_.id -notlike 'verified.*' }) {
        '  {0,-9} {1,-26} {2}' -f $check.status, $check.id, $check.detail
    }
    ''
    '[services]'
    foreach ($check in $checks | Where-Object { $_.id -like 'service.*' }) {
        '  {0,-11} {1,-26} {2}' -f $check.status, $check.id, $check.detail
    }
    ''
    '[verified]'
    foreach ($check in $checks | Where-Object { $_.id -like 'verified.*' }) {
        '  {0,-9} {1,-26} {2}' -f $check.status, $check.id, $check.detail
    }
    ''
    if ($baselineOk) { 'baseline: OK' } else { "baseline: FAILED ($($failed.Count) check(s))" }
}

if (-not $baselineOk) { exit 1 }
exit 0
