#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharGameplayData.generated.h"

class UDataAsset; // Archetype
class UAnimationAsset;

UCLASS(BlueprintType)
<<<<<<< HEAD
class ONLYHANDS_API UPDA_CharGameplayData : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDS_API UPDA_CharGameplayData : public UPrimaryDataAsset {
        GENERATED_BODY()

      public:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName Archetype;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        UAnimMontage* Taunts;
    };
