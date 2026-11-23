# ROS 2 path on Windows: the simulation runs natively, the ROS 2 bridge and RViz2 run in WSL.
#   .\Scripts\run_demo.ps1 -NoViewer -BindAll    start the sim first (WSL reaches it over the WSL network)
#   .\Scripts\wsl\run_ros2.ps1                   bridge + RViz2
#   .\Scripts\wsl\run_ros2.ps1 -NoRviz           bridge only
#
# Needs: WSL 2 with Ubuntu-24.04 in the default NAT network mode, and ROS 2 Jazzy
# (Scripts\wsl\setup_ros2.sh). Check what is being published with Scripts\wsl\ros2_check.sh.
param(
    [switch]$NoRviz,
    [switch]$NoBridge,
    [int]$Port = 9870,
    [string]$Distro = 'Ubuntu-24.04'
)

$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$wslRoot = '/mnt/' + $root.Substring(0, 1).ToLower() + ($root.Substring(2) -replace '\\', '/')
$wslScripts = "$wslRoot/Scripts/wsl"
$env:WSL_UTF8 = 1
# ros2_up.sh runs from a /tmp copy, so pass the project location through to WSL.
$env:SUBSEA_ROOT = $wslRoot
$env:WSLENV = 'SUBSEA_ROOT'

function Start-InWsl([string]$Arguments) {
    # Strip Windows line endings on the fly so the scripts work however they were checked out.
    $cmd = "sed 's/\r$//' $wslScripts/ros2_up.sh > /tmp/ros2_up.sh && bash /tmp/ros2_up.sh $Arguments"
    Start-Process wsl.exe -ArgumentList @('-d', $Distro, '-u', 'root', '--', 'bash', '-c', "`"$cmd`"")
}

if (-not $NoBridge) {
    Start-InWsl "bridge $Port"
    Write-Host "ROS 2 bridge starting in $Distro (topics under /subsea)"
}
if (-not $NoRviz) {
    Start-InWsl 'rviz'
    Write-Host 'RViz2 starting (WSLg window)'
}
