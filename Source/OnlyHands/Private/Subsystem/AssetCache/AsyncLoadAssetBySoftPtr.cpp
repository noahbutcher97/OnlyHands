
#include "Subsystem/AssetCache/AsyncLoadAssetBySoftPtr.h"
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"

<<<<<<< HEAD UAsyncLoadAssetBySoftPtr* UAsyncLoadAssetBySoftPtr::LoadSoftAssetAsync(UObject* WorldContextObject,
                                                                         TSoftObjectPtr<UObject> AssetRef) {
    == == == =

                 UAsyncLoadAssetBySoftPtr * UAsyncLoadAssetBySoftPtr::LoadSoftAssetAsync(
                                                UObject * WorldContextObject, TSoftObjectPtr<UObject> AssetRef) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UAsyncLoadAssetBySoftPtr* Node = NewObject<UAsyncLoadAssetBySoftPtr>();
        Node->WorldContextObject = WorldContextObject;
        Node->RequestedAsset = AssetRef;
        return Node;
    }

<<<<<<< HEAD
    void UAsyncLoadAssetBySoftPtr::Activate() {
        if (!WorldContextObject || RequestedAsset.IsNull()) {
            == == == = void UAsyncLoadAssetBySoftPtr::Activate() {
                if (!WorldContextObject || RequestedAsset.IsNull()) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    OnLoaded.Broadcast(nullptr);
                    return;
                }

<<<<<<< HEAD
                UAssetCacheSubsystem* Cache =
                    UGameplayStatics::GetGameInstance(WorldContextObject)->GetSubsystem<UAssetCacheSubsystem>();
                if (!Cache) {
                    == == ==
                        = UAssetCacheSubsystem* Cache =
                            UGameplayStatics::GetGameInstance(WorldContextObject)->GetSubsystem<UAssetCacheSubsystem>();
                    if (!Cache) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                        OnLoaded.Broadcast(nullptr);
                        return;
                    }

<<<<<<< HEAD
                    Cache->RequestAsset(RequestedAsset, FOnAssetLoadedNative::CreateUObject(
                                                            this, &UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal));
                }

                void UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal(UObject * Loaded) {
                    == == == = Cache->RequestAsset(RequestedAsset,
                                                   FOnAssetLoadedNative::CreateUObject(
                                                       this, &UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal));
                }

                void UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal(UObject * Loaded) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    OnLoaded.Broadcast(Loaded);
                }
