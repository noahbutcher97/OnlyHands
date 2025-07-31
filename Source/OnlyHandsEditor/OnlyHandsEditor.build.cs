using UnrealBuildTool;
 
public class OnlyHandsEditor : ModuleRules
{
	public OnlyHandsEditor(ReadOnlyTargetRules Target) : base(Target)
	{
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "UnrealEd", "OnlyHands", "AnimGraph", "BlueprintGraph", "AIModule","EditorScriptingUtilities"});


		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Slate", "SlateCore", "AnimGraphRuntime", "PropertyEditor", "EditorScriptingUtilities", "AssetTools", "AssetRegistry"
		});

        PrivateDependencyModuleNames.AddRange(new string[] {"EditorScriptingUtilities", "Blutility" });

		PublicIncludePaths.AddRange(new string[] {"OnlyHandsEditor/Public","OnlyHandsEditor/Public/PropertyCustomization", "OnlyHandsEditor/Public/Animation"});
		PrivateIncludePaths.AddRange(new string[] {"OnlyHandsEditor/Private", "OnlyHandsEditor/Private/PropertyCustomization", "OnlyHandsEditor/Private/Animation"});
	}
}