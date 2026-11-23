# Copies the finished case-study media into the portfolio site and the case-study folder.
#   .\Scripts\publish_case_study.ps1 -Web <portfolio root>
# -Web defaults to the SUBSEA_WEB environment variable. The root holds portfolio\ and Projects\.
param([string]$Web = $env:SUBSEA_WEB)
$ErrorActionPreference = 'Stop'
if (-not $Web) { throw 'Pass -Web <portfolio root> or set SUBSEA_WEB.' }
if (-not (Test-Path (Join-Path $Web 'portfolio'))) { throw "No portfolio folder under '$Web'." }
$proj = Split-Path -Parent $PSScriptRoot
$out = Join-Path $proj 'Saved\Film\out'
$renders = Join-Path $proj 'Saved\Film\renders'
$site = Join-Path $web 'portfolio\content\projects\08-subsea-sim'
$cs = Join-Path $web 'Projects\SubseaSim\case-study\media'
foreach ($d in @($site, "$site\gallery", "$cs\video", "$cs\posters", "$cs\stills")) { New-Item -ItemType Directory -Force $d | Out-Null }

# Portfolio site: exact names expected by site-data.js
Copy-Item "$out\cover.jpg" "$site\cover.jpg" -Force
Copy-Item "$out\loop.mp4" "$site\loop.mp4" -Force
Copy-Item "$out\final_web.mp4" "$site\final.mp4" -Force
foreach ($c in 'autopilot-hud', 'sonar-vs-camera', 'fault') { Copy-Item "$out\$c.mp4" "$site\$c.mp4" -Force; Copy-Item "$out\$c.jpg" "$site\$c.jpg" -Force }
Copy-Item "$out\map-build_web.mp4" "$site\map-build.mp4" -Force; Copy-Item "$out\map-build.jpg" "$site\map-build.jpg" -Force
Copy-Item "$out\map-orbit_web.mp4" "$site\map-orbit.mp4" -Force
Copy-Item "$proj\Docs\media\architecture.svg" "$site\architecture.svg" -Force
$gallery = @{ '01' = '04'; '02' = '02'; '03' = '03'; '04' = '05'; '05' = '06' }
foreach ($k in $gallery.Keys) { Copy-Item "$out\gallery\$($gallery[$k]).jpg" "$site\gallery\$k.jpg" -Force }

# Case-study folder
Copy-Item "$out\final_web.mp4" "$cs\video\subseasim-film.mp4" -Force
Copy-Item "$out\loop.mp4" "$cs\video\subseasim-loop.mp4" -Force
foreach ($c in 'autopilot-hud', 'onboard', 'fault', 'sonar-vs-camera', 'flyby', 'manifold') {
    Copy-Item "$out\$c.mp4" "$cs\video\subseasim-$c.mp4" -Force; Copy-Item "$out\$c.jpg" "$cs\posters\subseasim-$c.jpg" -Force }
foreach ($c in 'map-build', 'map-orbit') { Copy-Item "$out\${c}_web.mp4" "$cs\video\subseasim-$c.mp4" -Force; Copy-Item "$out\$c.jpg" "$cs\posters\subseasim-$c.jpg" -Force }
Copy-Item "$out\cover.jpg" "$cs\posters\subseasim-cover.jpg" -Force
$names = @{ '01' = '06-wide'; '02' = '02-onboard'; '03' = '03-flyby'; '04' = '01-orbit'; '05' = '04-manifold'; '06' = '05-pilot-hud'; '07' = '07-top'; '08' = '08-side' }
foreach ($k in $names.Keys) { Copy-Item "$out\gallery\$k.jpg" "$cs\stills\$($names[$k]).jpg" -Force }
Copy-Item "$proj\Docs\media\architecture.svg" "$cs\stills\architecture.svg" -Force
Copy-Item "$proj\Docs\media\architecture.png" "$cs\stills\architecture.png" -Force
Copy-Item "$renders\orbit\frame_00150.png" "$cs\stills\09-survey-map.png" -Force
Copy-Item "$renders\sonar_turbid\frame_00010.png" "$cs\stills\10-sonar.png" -Force

# ROS 2 / RViz2 (Scripts\wsl\record_ros2_demo.ps1 + Scripts\make_ros2_clip.py)
$ros = Join-Path $proj 'Saved\ROS2'
if (Test-Path "$ros\ros2-rviz-web.mp4") {
    Copy-Item "$ros\ros2-rviz-web.mp4" "$site\ros2-rviz.mp4" -Force
    Copy-Item "$ros\ros2-rviz-poster.jpg" "$site\ros2-rviz.jpg" -Force
    Copy-Item "$ros\ros2-rviz-web.mp4" "$cs\video\subseasim-ros2-rviz.mp4" -Force
    Copy-Item "$ros\ros2-rviz.mp4" "$cs\video\subseasim-ros2-rviz-1080p.mp4" -Force
    Copy-Item "$ros\ros2-rviz-poster.jpg" "$cs\posters\subseasim-ros2-rviz.jpg" -Force
    ffmpeg -y -loglevel error -i "$ros\rviz_still.png" -q:v 2 "$site\gallery\06.jpg"
    Copy-Item "$site\gallery\06.jpg" "$cs\stills\11-rviz2.jpg" -Force
}

Get-ChildItem $site -Recurse -File | Select-Object @{n='Site file';e={$_.FullName.Substring($site.Length+1)}}, @{n='MB';e={[math]::Round($_.Length/1MB,2)}} | Format-Table -AutoSize
"case-study media: " + (Get-ChildItem $cs -Recurse -File).Count + " files, " + [math]::Round(((Get-ChildItem $cs -Recurse -File | Measure-Object Length -Sum).Sum)/1MB,1) + " MB"