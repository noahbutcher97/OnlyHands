// AssetCacheSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "UObject/PrimaryAssetId.h"
#include "Engine/StreamableManager.h"
#include "AssetCacheSubsystem.generated.h"

DECLARE_DELEGATE_OneParam(FOnAssetLoadedNative, UObject*);

UCLASS()
<<<<<<< HEAD
class ONLYHANDS_API UAssetCacheSubsystem : public UGameInstanceSubsystem {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDS_API UAssetCacheSubsystem : public UGameInstanceSubsystem {
        GENERATED_BODY()

      public:

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
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

<<<<<<< HEAD
        //    UObject* GetCachedAsset(const FString& Key) const;

      protected:
      == == == =
                   //    UObject* GetCachedAsset(const FString& Key) const;

          protected :
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
          TMap<FString, UObject*>
              CachedAssets;
        FStreamableManager StreamableManager;

        // Active loads in progress
        TMap<FString, TArray<FOnAssetLoadedNative>> PendingLoads;

        // Optional: Reference-counted cache
        TMap<FString, int32> AssetRefCounts;

        void LoadAssetInternal(const FString& Key, const FSoftObjectPath& AssetPath,
                               const FOnAssetLoadedNative& OnLoaded);
<<<<<<< HEAD
        == == == =

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    };
