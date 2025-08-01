#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "AsyncLoadAssetBySoftPtr.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnSoftAssetLoaded, UObject*, LoadedAsset);

UCLASS()
<<<<<<< HEAD
class ONLYHANDS_API UAsyncLoadAssetBySoftPtr : public UBlueprintAsyncActionBase {
    GENERATED_BODY()

  public:
    UPROPERTY(BlueprintAssignable)
    FOnSoftAssetLoaded OnLoaded;

    UFUNCTION(BlueprintCallable, Category = "Asset Loading",
              meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
    == == == = class ONLYHANDS_API UAsyncLoadAssetBySoftPtr : public UBlueprintAsyncActionBase {
        GENERATED_BODY()

      public:
        UPROPERTY(BlueprintAssignable)
        FOnSoftAssetLoaded OnLoaded;

        UFUNCTION(BlueprintCallable, Category = "Asset Loading",
                  meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        static UAsyncLoadAssetBySoftPtr* LoadSoftAssetAsync(UObject* WorldContextObject,
                                                            TSoftObjectPtr<UObject> AssetRef);

        virtual void Activate() override;

<<<<<<< HEAD

      private:
      == == == = private :
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
          UPROPERTY() UObject *
          WorldContextObject;

        TSoftObjectPtr<UObject> RequestedAsset;

        void OnAssetLoadedInternal(UObject* Loaded);
    };