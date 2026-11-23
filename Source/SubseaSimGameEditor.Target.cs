using UnrealBuildTool;

public class SubseaSimGameEditorTarget : TargetRules
{
	public SubseaSimGameEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		CppStandard = CppStandardVersion.Cpp20;
		ExtraModuleNames.Add("SubseaSimGame");
	}
}
