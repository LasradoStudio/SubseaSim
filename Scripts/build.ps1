# Compiles the editor target (the plugin's runtime and editor modules plus the game module).
#   .\Scripts\build.ps1 [-Configuration Development|DebugGame|Shipping]
param([string]$Configuration = 'Development')

. "$PSScriptRoot\Common.ps1"
$unreal = Get-UnrealRoot

& "$unreal\Engine\Build\BatchFiles\Build.bat" SubseaSimGameEditor Win64 $Configuration "-project=$ProjectFile" -waitmutex -NoHotReload
exit $LASTEXITCODE
