
#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "UObject/PrimaryAssetId.h"
#include "AsyncLoadAssetByPrimaryId.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPrimaryDataAssetLoaded, UObject*, LoadedAsset);

UCLASS()
class ONLYHANDS_API UAsyncLoadAssetByPrimaryId : public UBlueprintAsyncActionBase {
    GENERATED_BODY()

  public:
    UPROPERTY(BlueprintAssignable)
    FOnPrimaryDataAssetLoaded OnLoaded;

    UFUNCTION(BlueprintCallable, Category = "Asset Loading",
              meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject"))
    static UAsyncLoadAssetByPrimaryId* LoadPrimaryAssetAsync(UObject* WorldContextObject, FPrimaryAssetId AssetId);

    virtual void Activate() override;

  private:
    UPROPERTY()
    UObject* WorldContextObject;

    FPrimaryAssetId RequestedAssetId;

    void OnAssetLoadedInternal(UObject* Loaded);
};