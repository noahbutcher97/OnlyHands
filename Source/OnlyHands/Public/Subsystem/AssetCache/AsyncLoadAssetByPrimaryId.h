
#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "UObject/PrimaryAssetId.h"
#include "AsyncLoadAssetByPrimaryId.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPrimaryDataAssetLoaded, UObject*, LoadedAsset);

UCLASS()
<<<<<<< HEAD
class ONLYHANDS_API UAsyncLoadAssetByPrimaryId : public UBlueprintAsyncActionBase {
    GENERATED_BODY()

  public:
    UPROPERTY(BlueprintAssignable)
    FOnPrimaryDataAssetLoaded OnLoaded;

    UFUNCTION(BlueprintCallable, Category = "Asset Loading",
              meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
    == == == = class ONLYHANDS_API UAsyncLoadAssetByPrimaryId : public UBlueprintAsyncActionBase {
        GENERATED_BODY()

      public:
        UPROPERTY(BlueprintAssignable)
        FOnPrimaryDataAssetLoaded OnLoaded;

        UFUNCTION(BlueprintCallable, Category = "Asset Loading",
                  meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        static UAsyncLoadAssetByPrimaryId* LoadPrimaryAssetAsync(UObject* WorldContextObject, FPrimaryAssetId AssetId);

        virtual void Activate() override;

<<<<<<< HEAD

      private:
      == == == = private :
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
          UPROPERTY() UObject *
          WorldContextObject;

        FPrimaryAssetId RequestedAssetId;

        void OnAssetLoadedInternal(UObject* Loaded);
<<<<<<< HEAD
    };
    == == == =
}; 
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
