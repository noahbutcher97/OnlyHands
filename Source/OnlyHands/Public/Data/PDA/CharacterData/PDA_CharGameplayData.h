#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharGameplayData.generated.h"

class UDataAsset; // Archetype
class UAnimationAsset;

UCLASS(BlueprintType)
class ONLYHANDS_API UPDA_CharGameplayData : public UPrimaryDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Archetype;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UAnimMontage* Taunts;
};
