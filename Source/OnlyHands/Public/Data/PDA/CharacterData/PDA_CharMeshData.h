#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharMeshData.generated.h"

UCLASS(BlueprintType)
class ONLYHANDS_API UPDA_CharMeshData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMesh* BaseMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<USkeletalMesh*> AttachedMeshes;
};
