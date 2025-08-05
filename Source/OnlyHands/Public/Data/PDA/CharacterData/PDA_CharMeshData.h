#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharMeshData.generated.h"

USTRUCT(BlueprintType)
struct F_SelectScreen : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 SelectIndex;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 SkinIndex;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName CharacterName;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Describy;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> SkinImage;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<USkeletalMesh> SelectScreen;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UAnimSequenceBase> Anm_Idle;
};

UCLASS(BlueprintType)
class ONLYHANDS_API UPDA_CharMeshData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMesh* BaseMesh;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_SelectScreen SelectScreen;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<USkeletalMesh*> AttachedMeshes;
};
