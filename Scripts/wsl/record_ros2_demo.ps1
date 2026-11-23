# Record the Unreal window and the RViz2 window at the same time, for the ROS 2 demo clip.
#   .\Scripts\wsl\record_ros2_demo.ps1 -Seconds 24 -OutDir Saved\ROS2
# Needs the sim (run_demo.ps1 -NoViewer -BindAll) and run_ros2.ps1 already running.
# Unreal is captured with Desktop Duplication (ddagrab) because GDI window capture returns stale
# frames for a DirectX swap chain, so the Unreal window is held on top while recording.
param(
    [int]$Seconds = 24,
    [string]$OutDir = "$PSScriptRoot\..\..\Saved\ROS2",
    [string]$Distro = 'Ubuntu-24.04'
)
$OutDir = (New-Item -ItemType Directory -Force $OutDir).FullName
$proc = Get-Process UnrealEditor | Where-Object MainWindowTitle -like 'SubseaSim*' | Select-Object -First 1
if (-not $proc) { throw 'SubseaSim window not found' }
Add-Type @"
using System; using System.Runtime.InteropServices;
public static class SubseaWin {
    public struct POINT { public int X, Y; }
    public struct RECT { public int L, T, R, B; }
    [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr h, ref POINT p);
    [DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
}
"@
$hwnd = $proc.MainWindowHandle
$origin = New-Object SubseaWin+POINT
[SubseaWin]::ClientToScreen($hwnd, [ref]$origin) | Out-Null
$client = New-Object SubseaWin+RECT
[SubseaWin]::GetClientRect($hwnd, [ref]$client) | Out-Null
$size = "$($client.R)x$($client.B)"

$wslOut = '/mnt/' + $OutDir.Substring(0, 1).ToLower() + ($OutDir.Substring(2) -replace '\\', '/')
$wslScripts = '/mnt/' + $PSScriptRoot.Substring(0, 1).ToLower() + ($PSScriptRoot.Substring(2) -replace '\\', '/')
$env:WSL_UTF8 = 1
wsl.exe -d $Distro -u root -- bash -c "sed 's/\r$//' $wslScripts/rviz_record.sh > /tmp/rec.sh"

$SWP_NOSIZE_NOMOVE = 0x0001 -bor 0x0002
[SubseaWin]::SetWindowPos($hwnd, [IntPtr](-1), 0, 0, 0, 0, $SWP_NOSIZE_NOMOVE) | Out-Null   # topmost
try {
    Start-Sleep -Milliseconds 500
    $rviz = Start-Process wsl.exe -PassThru -WindowStyle Hidden -ArgumentList @('-d', $Distro, '-u', 'root', '--',
        'bash', '/tmp/rec.sh', "$wslOut/rviz.mp4", "$Seconds", '30')
    $grab = "ddagrab=framerate=30:offset_x=$($origin.X):offset_y=$($origin.Y):video_size=${size}:draw_mouse=0,hwdownload,format=bgra"
    $ue = Start-Process ffmpeg -PassThru -WindowStyle Hidden -ArgumentList @('-y', '-loglevel', 'error',
        '-f', 'lavfi', '-i', "`"$grab`"", '-t', "$Seconds",
        '-c:v', 'libx264', '-preset', 'slow', '-crf', '18', '-pix_fmt', 'yuv420p', "`"$OutDir\unreal.mp4`"")
    $rviz.WaitForExit(); $ue.WaitForExit()
}
finally {
    [SubseaWin]::SetWindowPos($hwnd, [IntPtr](-2), 0, 0, 0, 0, $SWP_NOSIZE_NOMOVE) | Out-Null  # not topmost
}
Write-Host "recorded $OutDir\unreal.mp4 ($size at $($origin.X),$($origin.Y)) and $OutDir\rviz.mp4"
