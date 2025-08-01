#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharMeshData.generated.h"

UCLASS(BlueprintType)
<<<<<<< HEAD
class ONLYHANDS_API UPDA_CharMeshData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDS_API UPDA_CharMeshData : public UPrimaryDataAsset {
        GENERATED_BODY()

      public:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        USkeletalMesh* BaseMesh;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TArray<USkeletalMesh*> AttachedMeshes;
    };
