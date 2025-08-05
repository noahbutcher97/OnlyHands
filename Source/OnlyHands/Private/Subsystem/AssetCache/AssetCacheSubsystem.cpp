// AssetCacheSubsystem.cpp
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/AssetManager.h"

void UAssetCacheSubsystem::RequestAsset(FPrimaryAssetId AssetId, FOnAssetLoadedNative OnLoaded) {
    UAssetManager& Manager = UAssetManager::Get();
    FSoftObjectPath Path = Manager.GetPrimaryAssetPath(AssetId);
    if (!Path.IsValid()) {
        OnLoaded.ExecuteIfBound(nullptr);
        return;
    }

    FString Key = AssetId.ToString();
    LoadAssetInternal(Key, Path, OnLoaded);
}

void UAssetCacheSubsystem::RequestAsset(TSoftObjectPtr<UObject> AssetRef, FOnAssetLoadedNative OnLoaded) {
    if (AssetRef.ToSoftObjectPath().IsNull()) {
        OnLoaded.ExecuteIfBound(nullptr);
        return;
    }

    FString Key = AssetRef.ToSoftObjectPath().ToString();
    LoadAssetInternal(Key, AssetRef.ToSoftObjectPath(), OnLoaded);
}

void UAssetCacheSubsystem::LoadAssetInternal(const FString& Key, const FSoftObjectPath& AssetPath,
                                             const FOnAssetLoadedNative& OnLoaded) {
    // If already cached, return immediately
    if (UObject* const* Found = CachedAssets.Find(Key)) {
        OnLoaded.ExecuteIfBound(*Found);
        return;
    }

    // If currently pending, just queue the callback
    if (PendingLoads.Contains(Key)) {
        PendingLoads[Key].Add(OnLoaded);
        return;
    }

    // First request for this asset — add to pending list
    PendingLoads.Add(Key, {OnLoaded});

    StreamableManager.RequestAsyncLoad(AssetPath, FStreamableDelegate::CreateWeakLambda(this, [this, AssetPath, Key]() {
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
        FSoftObjectPath Path(Key);
        StreamableManager.Unload(Path);
        CachedAssets.Remove(Key);
    }
}

void UAssetCacheSubsystem::UnloadAllAssets() {
    for (const auto& Pair : CachedAssets) {
        FSoftObjectPath AssetPath(Pair.Key);
        StreamableManager.Unload(AssetPath);
    }

    CachedAssets.Empty();

    UE_LOG(LogTemp, Log, TEXT("Unloaded all cached assets"));
}

void UAssetCacheSubsystem::UnloadAssetByPrimaryId(FPrimaryAssetId AssetId) {
    FString Key = AssetId.ToString();
    UnloadAssetByKey(Key);
}

void UAssetCacheSubsystem::UnloadAssetBySoftPtr(TSoftObjectPtr<UObject> AssetRef) {
    FString Key = AssetRef.ToSoftObjectPath().ToString();
    UnloadAssetByKey(Key);
}
