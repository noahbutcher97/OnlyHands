#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "CharacterDataTableRow.generated.h"

USTRUCT(BlueprintType)
struct ONLYHANDS_API FCharacterDataTableRow : public FTableRowBase
{
    GENERATED_BODY()

    // Character ID or internal name
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FName CharacterID;

    // Look name (e.g. "Default", "Classic", "Alt01")
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FName LookID;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FText DisplayNameText;

    // Mesh Data
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    USkeletalMesh* BaseMesh;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TArray<USkeletalMesh*> AttachedMeshes;

    // UI Data
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UTexture2D* Portrait;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UTexture2D* Thumbnail;

    // Gameplay Data
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FName Archetype; // Can be replaced with a specific Archetype class

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UAnimMontage* Taunts;

    // Cinematic Data
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UAnimMontage* Intro;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UAnimMontage* WinPose;
};
