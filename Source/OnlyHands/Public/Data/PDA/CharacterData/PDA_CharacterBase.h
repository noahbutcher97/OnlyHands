#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PDA_CharacterBase.generated.h"

class UPDA_CharMeshData;
class UPDA_CharUIData;
class UPDA_CharGameplayData;
class UPDA_CharCinematicData;

UCLASS(BlueprintType)
<<<<<<< HEAD
class ONLYHANDS_API UPDA_CharacterBase : public UPrimaryDataAsset {
    GENERATED_BODY()

  public:
    virtual FPrimaryAssetId GetPrimaryAssetId() const override {
        == == == = class ONLYHANDS_API UPDA_CharacterBase : public UPrimaryDataAsset {
            GENERATED_BODY()

          public:
            virtual FPrimaryAssetId GetPrimaryAssetId() const override {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
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
