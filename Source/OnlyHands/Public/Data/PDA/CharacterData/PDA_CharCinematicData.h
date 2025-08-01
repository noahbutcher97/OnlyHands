#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharCinematicData.generated.h"

class UAnimationAsset;

UCLASS(BlueprintType)
<<<<<<< HEAD
class ONLYHANDS_API UPDA_CharCinematicData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDS_API UPDA_CharCinematicData : public UPrimaryDataAsset {
        GENERATED_BODY()

      public:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        UAnimationAsset* Intro;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        UAnimationAsset* WinPose;
    };
