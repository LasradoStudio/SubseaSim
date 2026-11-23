using UnrealBuildTool;

public class SubseaSimGameTarget : TargetRules
{
	public SubseaSimGameTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		CppStandard = CppStandardVersion.Cpp20;
		ExtraModuleNames.Add("SubseaSimGame");
	}
}
