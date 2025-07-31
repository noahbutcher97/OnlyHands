// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.IO;

public class OnlyHands : ModuleRules
{
    public OnlyHands(ReadOnlyTargetRules Target) : base(Target)

        {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(
            new string[] {
                    Path.Combine(ModuleDirectory, "Public"),
                    Path.Combine(ModuleDirectory, "Public", "Data", "Struct"),
                    Path.Combine(ModuleDirectory, "Public", "Data", "Maps"),

                    

            }
        );

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "Niagara","AnimationCore", "PhysicsCore", "LevelSequence", "UMG", "Slate", "SlateCore", "ApplicationCore", "AnimGraphRuntime","MotionWarping", "Chaos" });
            
            
        PrivateDependencyModuleNames.AddRange(new string[] { "GameplayAbilities", "GameplayTags", "GameplayTasks", "NavigationSystem", "AnimGraphRuntime" });

        if (Target.Platform == UnrealTargetPlatform.IOS)
        {
            PublicFrameworks.AddRange(new string[] { "UIKit", "CoreHaptics", "AudioToolbox" });
        }
        // Uncomment if you are using Slate UI
        // PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");

        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true

    }
}