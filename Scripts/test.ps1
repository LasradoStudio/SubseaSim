# Runs every automated check that needs no GPU and no display:
#   1. Unreal automation tests (maths, control, physics, protocol, sensor model) plus the benchmark
#   2. Python bridge tests
# Add -Live to also launch the simulation and run the smoke and scenario tests against it.
#   .\Scripts\test.ps1 [-Live]
param([switch]$Live)

. "$PSScriptRoot\Common.ps1"
$unreal = Get-UnrealRoot
Assert-BridgeEnvironment
$failed = @()

Write-Host "`n== Unreal automation tests ==" -ForegroundColor Cyan
$log = Join-Path $ProjectRoot 'Saved\Logs\tests.log'
& "$unreal\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" $ProjectFile `
    '-ExecCmds=Automation RunTests SubseaSim; Quit' '-TestExit=Automation Test Queue Empty' `
    -nullrhi -unattended -nopause -nosplash -nosound "-abslog=$log" | Out-Null
$unrealExit = $LASTEXITCODE
Select-String -Path $log -Pattern 'Test Completed|LogSubseaSim: Display' |
    ForEach-Object { ($_.Line -replace '^\[[^\]]+\]\[[^\]]+\]', '').Trim() }
if ($unrealExit -ne 0 -or (Select-String -Path $log -Pattern 'Result=\{(Fail|Error)' -Quiet)) { $failed += 'Unreal automation tests' }

Write-Host "`n== Python bridge tests ==" -ForegroundColor Cyan
Push-Location (Join-Path $ProjectRoot 'Bridge')
& $BridgePython -m pytest -q
if ($LASTEXITCODE -ne 0) { $failed += 'Python bridge tests' }

if ($Live) {
    Write-Host "`n== Live simulation tests ==" -ForegroundColor Cyan
    Start-Process "$unreal\Engine\Binaries\Win64\UnrealEditor.exe" -ArgumentList "`"$ProjectFile`"", '-game', '-windowed',
        '-ResX=1280', '-ResY=720', '-nosplash', '-nosound', '-unattended'
    & $BridgePython ..\Scripts\smoke_test.py --seconds 15
    if ($LASTEXITCODE -ne 0) { $failed += 'Live smoke test' }
    & $BridgePython ..\Scripts\scenario_test.py --quit
    if ($LASTEXITCODE -ne 0) { $failed += 'Live scenario test' }
}
Pop-Location

if ($failed.Count -gt 0) {
    Write-Host "`nFAILED: $($failed -join ', ')" -ForegroundColor Red
    exit 1
}
Write-Host "`nAll checks passed." -ForegroundColor Green
