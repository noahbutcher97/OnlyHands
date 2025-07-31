#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharacterBase.generated.h"

class UPDA_CharMeshData;
class UPDA_CharUIData;
class UPDA_CharGameplayData;
class UPDA_CharCinematicData;

UCLASS(BlueprintType)
class ONLYHANDS_API UPDA_CharacterBase : public UPrimaryDataAsset
{
    GENERATED_BODY()

public:

    virtual FPrimaryAssetId GetPrimaryAssetId() const override
    {
        return FPrimaryAssetId("Character", GetFName());
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UPDA_CharMeshData> SKMeshData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UPDA_CharUIData> UIData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UPDA_CharGameplayData> GameplayData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UPDA_CharCinematicData> CinematicData;
};
