#pragma once
#include "CoreMinimal.h"
#include "MotionWarpingComponent.h"
#include "OHPhysicsStructs.h"
#include "Animation/AnimInstanceProxy.h"
#include "Animation/AnimInstance.h"
#include "AnimationRuntime.h"
#include "Animation/AnimTypes.h"
#include "Animation/AnimTrace.h"
#include "Animation/AnimStats.h"
#include "Animation/AnimClassInterface.h"
#include "Component/OHPhysicsManager.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "OHAnimationUtils.generated.h"

UCLASS()
class ONLYHANDS_API UOHAnimationUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

  public:
    /** Sample current animation transform for a given bone (component space) */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation")
    static FTransform GetCurrentAnimPoseTransform(USkeletalMeshComponent* Mesh, FName BoneName);

    /** Compare anim transform to current FOHBoneData transform */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation")
    static float GetTransformError(UOHPhysicsManager* Manager, FName BoneName);

    /** Compare angular error between anim and tracked bone rotation */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation")
    static float GetRotationDriftError(UOHPhysicsManager* Manager, FName BoneName);

    /** Draw debug line from anim pose to tracked pose */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Animation|Debug")
    static void DrawPoseDeviation(UOHPhysicsManager* Manager, FName BoneName, FLinearColor Color = FLinearColor::Red,
                                  float Thickness = 1.5f);

    /** Return all tracked bones with transform error above threshold */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation|Debug")
    static TArray<FName> GetBonesWithHighDeviation(UOHPhysicsManager* Manager, float PositionThreshold = 5.f,
                                                   float RotationThreshold = 10.f);
};