#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharUIData.generated.h"

UCLASS(BlueprintType)
<<<<<<< HEAD
class ONLYHANDS_API UPDA_CharUIData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDS_API UPDA_CharUIData : public UPrimaryDataAsset {
        GENERATED_BODY()

      public:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        UTexture2D* Portrait;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        UTexture2D* Thumbnail;
    };
