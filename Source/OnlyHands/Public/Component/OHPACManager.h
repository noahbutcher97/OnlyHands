#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "OHPACManager.generated.h"

UENUM(BlueprintType)
enum class EPhysicalAnimationProfile : uint8
{
	Default		UMETA(DisplayName = "Default"),
	Ragdoll		UMETA(DisplayName = "Ragdoll"),
	Stiff		UMETA(DisplayName = "Stiff"),
	Custom1		UMETA(DisplayName = "Custom1")
};

UCLASS(Blueprintable, BlueprintType, ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ONLYHANDS_API UOHPACManager : public UActorComponent
{
	GENERATED_BODY()

public:
	UOHPACManager();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
	TMap<EPhysicalAnimationProfile, FPhysicalAnimationData> PhysicalAnimationProfiles;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physical Animation")
	UPhysicalAnimationComponent* PhysicalAnimationComponent;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physical Animation")
	USkeletalMeshComponent* SkeletalMeshComponent;

	// All bone names in the skeleton
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physical Animation")
	TArray<FName> BonesInSkeleton;

	// All body names in the PhysicsAsset
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TArray<FName> AllPhysicsBodies;

	// All constraint names in the PhysicsAsset
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TArray<FName> AllConstraintNames;

	// -------------------
	// Editor-time (PhysicsAsset) mappings
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FName, USkeletalBodySetup*> BodyNameToBodySetup;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FName, UPhysicsConstraintTemplate*> ConstraintNameToConstraintSetup;

	// -------------------
	// Runtime mappings
	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FName, FBodyInstance*> BoneNameToBodyInstance;

	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FBodyInstance*, FName> BodyInstanceToBoneName;

	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FName, FConstraintInstance*> ConstraintNameToConstraintInstance;

	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FConstraintInstance*, FName> ConstraintInstanceToConstraintName;

	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FName, TArray<FConstraintInstance*>> BoneNameToConstraintInstances;

	//UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics Snapshot")
	TMap<FBodyInstance*, TArray<FConstraintInstance*>> BodyInstanceToConstraintInstances;

	// -------------------
	// Profile state tracking
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physical Animation")
	TMap<FName, EPhysicalAnimationProfile> BoneCurrentProfileMap;

	// Initialization/caching and update
	UFUNCTION(BlueprintCallable, Category = "Physical Animation")
	void Initialize();

	// Rebuild snapshot (e.g., after mesh or asset change)
	UFUNCTION(BlueprintCallable, Category = "Physics Snapshot")
	void CachePhysicsAssetData();

	// Update/correct caches if live state changes (smart update)
	UFUNCTION(BlueprintCallable, Category = "Physics Snapshot")
	void SmartUpdatePhysicsSnapshot();

	// Visualization
	UFUNCTION(BlueprintCallable, Category = "Physics Snapshot|Debug")
	void VisualizePhysicsSnapshot(bool bDrawBodies = true, bool bDrawConstraints = true, bool bDrawBoneNames = true, float Duration = 5.0f) const;

protected:
	virtual void BeginPlay() override;
};