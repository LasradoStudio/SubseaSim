# One-command demo: starts the simulation and opens the live point-cloud viewer beside it.
#   .\Scripts\run_demo.ps1                 simulation + Rerun viewer
#   .\Scripts\run_demo.ps1 -NoViewer       simulation only
#   .\Scripts\run_demo.ps1 -BindAll        also accept connections from Docker / other machines (ROS 2 bridge)
param(
    [switch]$NoViewer,
    [switch]$BindAll,
    [int]$Port = 9870,
    [int]$Width = 1600,
    [int]$Height = 900
)

. "$PSScriptRoot\Common.ps1"
$unreal = Get-UnrealRoot

$arguments = @("`"$ProjectFile`"", '-game', '-windowed', "-ResX=$Width", "-ResY=$Height", '-nosplash', "-SubseaPort=$Port")
if ($BindAll) { $arguments += '-SubseaBindAll' }
Start-Process "$unreal\Engine\Binaries\Win64\UnrealEditor.exe" -ArgumentList $arguments
Write-Host "Simulation starting; sensor stream on tcp://127.0.0.1:$Port"

if (-not $NoViewer) {
    Assert-BridgeEnvironment
    Push-Location (Join-Path $ProjectRoot 'Bridge')
    # The viewer retries until the simulation is listening, so start order does not matter.
    & $BridgePython -m subsea_bridge.viewer_rerun --port $Port
    Pop-Location
}
