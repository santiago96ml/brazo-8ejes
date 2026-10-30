[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('build', 'upload', 'clean', 'doctor', 'boards')]
    [string]$Action = 'build',

    [string]$Port = 'COM4',
    [string]$Fqbn = 'arduino:avr:mega',
    [string]$Sketch = 'firmware/mega2560_arm'
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$sketchPath = Join-Path $repoRoot $Sketch

function Get-ArduinoCli {
    $cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\arduino-cli\arduino-cli.exe'),
        (Join-Path $env:LOCALAPPDATA 'arduino-cli\arduino-cli.exe'),
        'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe',
        (Join-Path $env:PROGRAMDATA 'chocolatey\bin\arduino-cli.exe'),
        (Join-Path $env:USERPROFILE 'scoop\shims\arduino-cli.exe')
    )
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) { return $candidate }
    }
    throw 'arduino-cli not found. Install it or open the Arduino IDE once so its bundled copy exists.'
}

if (-not (Test-Path $sketchPath)) {
    throw "Sketch folder not found: $sketchPath"
}

$cli = Get-ArduinoCli
Write-Host "[arduino-cli] $cli"

switch ($Action) {
    'build' {
        & $cli compile --fqbn $Fqbn --warnings all $sketchPath
        exit $LASTEXITCODE
    }
    'upload' {
        Write-Host "[upload] port=$Port fqbn=$Fqbn"
        & $cli compile --upload --port $Port --fqbn $Fqbn --warnings all $sketchPath
        exit $LASTEXITCODE
    }
    'clean' {
        & $cli compile --clean --fqbn $Fqbn $sketchPath
        exit $LASTEXITCODE
    }
    'boards' {
        & $cli board list
        exit $LASTEXITCODE
    }
    'doctor' {
        & $cli version
        Write-Host ''
        & $cli core list
        Write-Host ''
        & $cli board list
        Write-Host ''
        $ports = [System.IO.Ports.SerialPort]::GetPortNames()
        Write-Host ("Serial ports: " + ($ports -join ', '))
        if ($ports -notcontains $Port) {
            Write-Warning "Configured port $Port is not present."
        }
        exit 0
    }
}
