#include "Subsystem/AssetCache/AsyncLoadAssetByPrimaryId.h"
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"

UAsyncLoadAssetByPrimaryId* UAsyncLoadAssetByPrimaryId::LoadPrimaryAssetAsync(UObject* WorldContextObject,
                                                                              FPrimaryAssetId AssetId) {
    UAsyncLoadAssetByPrimaryId* Node = NewObject<UAsyncLoadAssetByPrimaryId>();
    Node->WorldContextObject = WorldContextObject;
    Node->RequestedAssetId = AssetId;
    return Node;
}

void UAsyncLoadAssetByPrimaryId::Activate() {
    if (!WorldContextObject) {
        OnLoaded.Broadcast(nullptr);
        return;
    }

    UAssetCacheSubsystem* Cache =
        UGameplayStatics::GetGameInstance(WorldContextObject)->GetSubsystem<UAssetCacheSubsystem>();
    if (!Cache) {
        OnLoaded.Broadcast(nullptr);
        return;
    }

    Cache->RequestAsset(RequestedAssetId,
                        FOnAssetLoadedNative::CreateUObject(this, &UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal));
}

void UAsyncLoadAssetByPrimaryId::OnAssetLoadedInternal(UObject* Loaded) {
    OnLoaded.Broadcast(Loaded);
}