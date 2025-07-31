
#include "Subsystem/AssetCache/AsyncLoadAssetBySoftPtr.h"
#include "Subsystem/AssetCache/AssetCacheSubsystem.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"


UAsyncLoadAssetBySoftPtr* UAsyncLoadAssetBySoftPtr::LoadSoftAssetAsync(UObject* WorldContextObject, TSoftObjectPtr<UObject> AssetRef)
{
    UAsyncLoadAssetBySoftPtr* Node = NewObject<UAsyncLoadAssetBySoftPtr>();
    Node->WorldContextObject = WorldContextObject;
    Node->RequestedAsset = AssetRef;
    return Node;
}

void UAsyncLoadAssetBySoftPtr::Activate()
{
    if (!WorldContextObject || RequestedAsset.IsNull())
    {
        OnLoaded.Broadcast(nullptr);
        return;
    }

    UAssetCacheSubsystem* Cache = UGameplayStatics::GetGameInstance(WorldContextObject)->GetSubsystem<UAssetCacheSubsystem>();
    if (!Cache)
    {
        OnLoaded.Broadcast(nullptr);
        return;
    }

    Cache->RequestAsset(RequestedAsset, FOnAssetLoadedNative::CreateUObject(this, &UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal));
}

void UAsyncLoadAssetBySoftPtr::OnAssetLoadedInternal(UObject* Loaded)
{
    OnLoaded.Broadcast(Loaded);
}
