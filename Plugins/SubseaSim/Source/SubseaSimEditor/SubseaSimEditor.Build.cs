using UnrealBuildTool;

public class SubseaSimEditor : ModuleRules
{
	public SubseaSimEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"Slate",
			"SlateCore",
			"ToolMenus",
			"UnrealEd",
			"SubseaSim",
		});
	}
}
