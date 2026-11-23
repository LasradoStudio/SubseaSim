using UnrealBuildTool;

public class SubseaSimGame : ModuleRules
{
	public SubseaSimGame(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "SubseaSim" });
	}
}
