// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.Collections.Generic;

public class OnlyHandsEditorTarget : TargetRules
{
	public OnlyHandsEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		ExtraModuleNames.AddRange( new string[] { "OnlyHandsEditor" } );
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new string[] { "OnlyHands" } );
	}
}
  