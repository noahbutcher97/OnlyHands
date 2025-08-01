// AssetCacheSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "UObject/PrimaryAssetId.h"
#include "Engine/StreamableManager.h"
#include "AssetCacheSubsystem.generated.h"

DECLARE_DELEGATE_OneParam(FOnAssetLoadedNative, UObject*);

UCLASS()
class ONLYHANDS_API UAssetCacheSubsystem : public UGameInstanceSubsystem {
    GENERATED_BODY()

  public:
    // Request by PrimaryAssetId
    void RequestAsset(FPrimaryAssetId AssetId, FOnAssetLoadedNative OnLoaded);

    // Request by SoftObjectPtr
    void RequestAsset(TSoftObjectPtr<UObject> AssetRef, FOnAssetLoadedNative OnLoaded);

    // Manual unload
    void UnloadAssetByKey(const FString& Key);

    UFUNCTION(BlueprintCallable, Category = "Asset Cache")
    void UnloadAssetByPrimaryId(FPrimaryAssetId AssetId);

    UFUNCTION(BlueprintCallable, Category = "Asset Cache")
    void UnloadAssetBySoftPtr(TSoftObjectPtr<UObject> AssetRef);

    UFUNCTION(BlueprintCallable, Category = "Asset Cache")
    void UnloadAllAssets();

    // Used internally by async nodes, not exposed to BP
    bool IsAssetCachedByKey(const FString& Key) const;

    //    UObject* GetCachedAsset(const FString& Key) const;

  protected:
    TMap<FString, UObject*> CachedAssets;
    FStreamableManager StreamableManager;

    // Active loads in progress
    TMap<FString, TArray<FOnAssetLoadedNative>> PendingLoads;

    // Optional: Reference-counted cache
    TMap<FString, int32> AssetRefCounts;

    void LoadAssetInternal(const FString& Key, const FSoftObjectPath& AssetPath, const FOnAssetLoadedNative& OnLoaded);
};
