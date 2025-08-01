// AssetCacheSubsystem.cpp
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/AssetManager.h"

<<<<<<< HEAD void UAssetCacheSubsystem::RequestAsset(FPrimaryAssetId AssetId, FOnAssetLoadedNative OnLoaded) { UAssetManager& Manager = UAssetManager::Get(); FSoftObjectPath Path = Manager.GetPrimaryAssetPath(AssetId); if (!Path.IsValid()) { == == == = void UAssetCacheSubsystem::RequestAsset(FPrimaryAssetId AssetId, FOnAssetLoadedNative OnLoaded) { UAssetManager& Manager = UAssetManager::Get(); FSoftObjectPath Path = Manager.GetPrimaryAssetPath(AssetId); if (!Path.IsValid()) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    OnLoaded.ExecuteIfBound(nullptr);
return;
}

FString Key = AssetId.ToString();
LoadAssetInternal(Key, Path, OnLoaded);
}

<<<<<<< HEAD
void UAssetCacheSubsystem::RequestAsset(TSoftObjectPtr<UObject> AssetRef, FOnAssetLoadedNative OnLoaded) {
    if (AssetRef.ToSoftObjectPath().IsNull()) {
        == == ==
            = void UAssetCacheSubsystem::RequestAsset(TSoftObjectPtr<UObject> AssetRef, FOnAssetLoadedNative OnLoaded) {
            if (AssetRef.ToSoftObjectPath().IsNull()) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                OnLoaded.ExecuteIfBound(nullptr);
                return;
            }

            FString Key = AssetRef.ToSoftObjectPath().ToString();
            LoadAssetInternal(Key, AssetRef.ToSoftObjectPath(), OnLoaded);
        }

<<<<<<< HEAD
        void UAssetCacheSubsystem::LoadAssetInternal(const FString& Key, const FSoftObjectPath& AssetPath,
                                                     const FOnAssetLoadedNative& OnLoaded) {
            // If already cached, return immediately
            if (UObject* const* Found = CachedAssets.Find(Key)) {
                == == == = void UAssetCacheSubsystem::LoadAssetInternal(const FString& Key,
                                                                        const FSoftObjectPath& AssetPath,
                                                                        const FOnAssetLoadedNative& OnLoaded) {
                    // If already cached, return immediately
                    if (UObject* const* Found = CachedAssets.Find(Key)) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                        OnLoaded.ExecuteIfBound(*Found);
                        return;
                    }

                    // If currently pending, just queue the callback
<<<<<<< HEAD
                    if (PendingLoads.Contains(Key)) {
                        == == == = if (PendingLoads.Contains(Key)) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                            PendingLoads[Key].Add(OnLoaded);
                            return;
                        }

                        // First request for this asset — add to pending list
<<<<<<< HEAD
                        PendingLoads.Add(Key, {OnLoaded});

                        StreamableManager.RequestAsyncLoad(
                            AssetPath, FStreamableDelegate::CreateWeakLambda(this, [this, AssetPath, Key]() {
                                UObject* Loaded = AssetPath.ResolveObject();
                                if (!Loaded)
                                    Loaded = AssetPath.TryLoad();

                                if (Loaded) {
                                    CachedAssets.Add(Key, Loaded);
                                }

                                // Execute all queued delegates
                                if (TArray<FOnAssetLoadedNative>* Callbacks = PendingLoads.Find(Key)) {
                                    for (FOnAssetLoadedNative& Callback : *Callbacks) {
                                        Callback.ExecuteIfBound(Loaded);
                                    }
                                    PendingLoads.Remove(Key);
                                }
                            }));
                    }

                    bool UAssetCacheSubsystem::IsAssetCachedByKey(const FString& Key) const {
                        return CachedAssets.Contains(Key);
                    }

                    // UObject* UAssetCacheSubsystem::GetCachedAsset(const FString& Key) const
                    //{
                    //     if (const UObject* const* Found = CachedAssets.Find(Key))
                    //         return *Found;
                    //     return nullptr;
                    // }

                    void UAssetCacheSubsystem::UnloadAssetByKey(const FString& Key) {
                        if (CachedAssets.Contains(Key)) {
                            == == == = PendingLoads.Add(Key, {OnLoaded});

                            StreamableManager.RequestAsyncLoad(
                                AssetPath, FStreamableDelegate::CreateWeakLambda(this, [this, AssetPath, Key]() {
                                    UObject* Loaded = AssetPath.ResolveObject();
                                    if (!Loaded)
                                        Loaded = AssetPath.TryLoad();

                                    if (Loaded) {
                                        CachedAssets.Add(Key, Loaded);
                                    }

                                    // Execute all queued delegates
                                    if (TArray<FOnAssetLoadedNative>* Callbacks = PendingLoads.Find(Key)) {
                                        for (FOnAssetLoadedNative& Callback : *Callbacks) {
                                            Callback.ExecuteIfBound(Loaded);
                                        }
                                        PendingLoads.Remove(Key);
                                    }
                                }));
                        }

                        bool UAssetCacheSubsystem::IsAssetCachedByKey(const FString& Key) const {
                            return CachedAssets.Contains(Key);
                        }

                        // UObject* UAssetCacheSubsystem::GetCachedAsset(const FString& Key) const
                        //{
                        //     if (const UObject* const* Found = CachedAssets.Find(Key))
                        //         return *Found;
                        //     return nullptr;
                        // }

                        void UAssetCacheSubsystem::UnloadAssetByKey(const FString& Key) {
                            if (CachedAssets.Contains(Key)) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                FSoftObjectPath Path(Key);
                                StreamableManager.Unload(Path);
                                CachedAssets.Remove(Key);
                            }
                        }

<<<<<<< HEAD
                        void UAssetCacheSubsystem::UnloadAllAssets() {
                            for (const auto& Pair : CachedAssets) {
                                == == == = void UAssetCacheSubsystem::UnloadAllAssets() {
                                    for (const auto& Pair : CachedAssets) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                        FSoftObjectPath AssetPath(Pair.Key);
                                        StreamableManager.Unload(AssetPath);
                                    }

                                    CachedAssets.Empty();

                                    UE_LOG(LogTemp, Log, TEXT("Unloaded all cached assets"));
                                }

<<<<<<< HEAD
                                void UAssetCacheSubsystem::UnloadAssetByPrimaryId(FPrimaryAssetId AssetId) {
                                    == == ==
                                        = void UAssetCacheSubsystem::UnloadAssetByPrimaryId(FPrimaryAssetId AssetId) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                        FString Key = AssetId.ToString();
                                        UnloadAssetByKey(Key);
                                    }

<<<<<<< HEAD
                                    void UAssetCacheSubsystem::UnloadAssetBySoftPtr(TSoftObjectPtr<UObject> AssetRef) {
                                        FString Key = AssetRef.ToSoftObjectPath().ToString();
                                        UnloadAssetByKey(Key);
                                    }
                                    == == == = void UAssetCacheSubsystem::UnloadAssetBySoftPtr(
                                                 TSoftObjectPtr<UObject> AssetRef) {
                                        FString Key = AssetRef.ToSoftObjectPath().ToString();
                                        UnloadAssetByKey(Key);
                                    }

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
