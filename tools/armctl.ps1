param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$hostDir = Join-Path $repoRoot 'host'

if (-not (Test-Path $hostDir)) {
    throw "Host package not found: $hostDir"
}

$env:PYTHONPATH = $hostDir
& python -m armctl @Rest
exit $LASTEXITCODE
