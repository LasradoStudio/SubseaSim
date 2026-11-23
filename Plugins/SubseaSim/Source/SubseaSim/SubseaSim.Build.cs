using UnrealBuildTool;

public class SubseaSim : ModuleRules
{
	public SubseaSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"ProceduralMeshComponent",
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"InputCore",
			"RenderCore",
			"RHI",
			"Sockets",
			"Networking",
			"Json",
		});
	}
}
