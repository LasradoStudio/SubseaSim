# Shared helpers for the SubseaSim scripts. Dot-source this file.

$ErrorActionPreference = 'Stop'
$script:ProjectRoot = Split-Path -Parent $PSScriptRoot
$script:ProjectFile = Join-Path $ProjectRoot 'SubseaSim.uproject'
$script:BridgePython = Join-Path $ProjectRoot 'Bridge\.venv\Scripts\python.exe'

function Get-UnrealRoot {
    # 1. explicit override, 2. the launcher's record of installed engines, 3. the default location.
    if ($env:UE_ROOT -and (Test-Path $env:UE_ROOT)) { return $env:UE_ROOT }

    $association = (Get-Content $ProjectFile -Raw | ConvertFrom-Json).EngineAssociation
    $manifest = 'C:\ProgramData\Epic\UnrealEngineLauncher\LauncherInstalled.dat'
    if (Test-Path $manifest) {
        $entry = (Get-Content $manifest -Raw | ConvertFrom-Json).InstallationList |
            Where-Object { $_.AppName -eq "UE_$association" } | Select-Object -First 1
        if ($entry -and (Test-Path $entry.InstallLocation)) { return $entry.InstallLocation }
    }

    $fallback = "C:\Program Files\Epic Games\UE_$association"
    if (Test-Path $fallback) { return $fallback }
    throw "Unreal Engine $association not found. Set the UE_ROOT environment variable to its install folder."
}

function Assert-BridgeEnvironment {
    if (-not (Test-Path $BridgePython)) {
        Write-Host 'Creating the Python environment for the bridge...'
        python -m venv (Join-Path $ProjectRoot 'Bridge\.venv')
        & $BridgePython -m pip install --quiet -e "$(Join-Path $ProjectRoot 'Bridge')[viewer,dev]" matplotlib
    }
}
