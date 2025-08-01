#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "AsyncLoadAssetBySoftPtr.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnSoftAssetLoaded, UObject*, LoadedAsset);

UCLASS()
class ONLYHANDS_API UAsyncLoadAssetBySoftPtr : public UBlueprintAsyncActionBase {
    GENERATED_BODY()

  public:
    UPROPERTY(BlueprintAssignable)
    FOnSoftAssetLoaded OnLoaded;

    UFUNCTION(BlueprintCallable, Category = "Asset Loading",
              meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
    static UAsyncLoadAssetBySoftPtr* LoadSoftAssetAsync(UObject* WorldContextObject, TSoftObjectPtr<UObject> AssetRef);

    virtual void Activate() override;

  private:
    UPROPERTY()
    UObject* WorldContextObject;

    TSoftObjectPtr<UObject> RequestedAsset;

    void OnAssetLoadedInternal(UObject* Loaded);
};