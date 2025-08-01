#include "Subsystem/AssetCache/AsyncLoadAssetByPrimaryId.h"
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"

<<<<<<< HEAD UAsyncLoadAssetByPrimaryId* UAsyncLoadAssetByPrimaryId::LoadPrimaryAssetAsync(UObject* WorldContextObject,
                                                                                FPrimaryAssetId AssetId) {
    == == == =

                 UAsyncLoadAssetByPrimaryId * UAsyncLoadAssetByPrimaryId::LoadPrimaryAssetAsync(
                                                  UObject * WorldContextObject, FPrimaryAssetId AssetId) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UAsyncLoadAssetByPrimaryId* Node = NewObject<UAsyncLoadAssetByPrimaryId>();
        Node->WorldContextObject = WorldContextObject;
        Node->RequestedAssetId = AssetId;
        return Node;
    }

<<<<<<< HEAD
    void UAsyncLoadAssetByPrimaryId::Activate() {
        if (!WorldContextObject) {
            == == == = void UAsyncLoadAssetByPrimaryId::Activate() {
                if (!WorldContextObject) {
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
                    Cache->RequestAsset(
                        RequestedAssetId,
                        FOnAssetLoadedNative::CreateUObject(this, &UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal));
                }

                void UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal(UObject * Loaded) {
                    == == == = Cache->RequestAsset(RequestedAssetId,
                                                   FOnAssetLoadedNative::CreateUObject(
                                                       this, &UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal));
                }

                void UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal(UObject * Loaded) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    OnLoaded.Broadcast(Loaded);
                }