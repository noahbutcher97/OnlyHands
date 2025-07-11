
#pragma once

#include "CoreMinimal.h"
#include "Component/OHPhysicsComponent.h" // Or your custom settings struct
#include "Engine/DataAsset.h"
#include "OHPhysicalAnimationProfileDataAsset.generated.h"

UCLASS(BlueprintType)
class ONLYHANDS_API UOHPhysicalAnimationProfileDataAsset : public UDataAsset {
    GENERATED_BODY()
  public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile")
    FName ProfileName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile")
    FPhysicalAnimationDriveSettings DriveSettings;
    // ^ Replace with your own struct if desired!
};