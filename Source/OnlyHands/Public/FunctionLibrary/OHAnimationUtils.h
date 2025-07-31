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
class ONLYHANDS_API UOHAnimationUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	/**
 * Finds the contiguous bone chain below a root, stopping at any branch or the tip.
 * @param SkeletalMeshComp - Skeletal mesh component to query.
 * @param RootBone         - Root bone name (e.g., "clavicle_l" or "thigh_r").
 * @param OutChain         - Receives chain from root (inclusive) to tip.
 * @return true if a valid chain was found (length >= 2), false otherwise.
 */
	static bool GetContiguousBoneChain(const USkeletalMeshComponent* SkeletalMeshComp, FName RootBone, TArray<FName>& OutChain);


	/**
 * Given a tip bone, finds the contiguous bone chain toward the root,
 * stopping at the first branch or root. The chain is ordered root-to-tip.
 * @param SkeletalMeshComp - The skeletal mesh component.
 * @param TipBone          - The bone at the tip of the chain (e.g., "hand_l").
 * @param OutChain         - Output array of bone names, root-to-tip order.
 * @return true if a valid chain (length >= 2) was found, false otherwise.
 */
	static bool GetContiguousBoneChainToRoot(const USkeletalMeshComponent* SkeletalMeshComp, FName TipBone, TArray<FName>& OutChain);

	

	/**
 * Measures the total length of a bone chain in the reference pose.
 * @param SkeletalMeshComp   - The mesh component whose reference pose to use
 * @param BoneChain          - Ordered array of bone names (from chain root to tip)
 * @return Total length (cm) along the bone chain in reference pose, or 0 on error
 */

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation")
	static float CalculateChainLengthInReferencePose(
		const USkeletalMeshComponent* SkeletalMeshComp,
		const TArray<FName>& BoneChain);


	/**
 * Measures the maximum possible length of a bone chain by summing rest-pose local translations.
 * (Does not depend on actual arm/leg orientation in the ref pose—gives "fully straightened" length.)
 */
	static float CalculateChainRestLength(const USkeletalMeshComponent* SkeletalMeshComp, const TArray<FName>& BoneChain);

	
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
	static void DrawPoseDeviation(UOHPhysicsManager* Manager, FName BoneName, FLinearColor Color = FLinearColor::Red, float Thickness = 1.5f);

	/** Return all tracked bones with transform error above threshold */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Animation|Debug")
	static TArray<FName> GetBonesWithHighDeviation(UOHPhysicsManager* Manager, float PositionThreshold = 5.f, float RotationThreshold = 10.f);
};