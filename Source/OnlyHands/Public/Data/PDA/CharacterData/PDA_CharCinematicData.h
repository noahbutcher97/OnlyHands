#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharCinematicData.generated.h"

class UAnimationAsset;

UCLASS(BlueprintType)
class ONLYHANDS_API UPDA_CharCinematicData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UAnimationAsset* Intro;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UAnimationAsset* WinPose;
};
