// ============================================================================
// OHPACManager.cpp
// PAC (Physical Animation Component) Manager for OnlyHands Project
// Streamlined version - Unified functions and removed duplicates
// ============================================================================

#include "Component/OHPACManager.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Animation/AnimInstance.h"
#include "TimerManager.h"
#include "Component/OHMovementComponent.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "FunctionLibrary/OHCollisionUtils.h"
#include "FunctionLibrary/OHCombatUtils.h"
#include "FunctionLibrary/OHLocomotionUtils.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"

DEFINE_LOG_CATEGORY(LogPACManager);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
#pragma region HELPER FUNCTIONS

void UOHPACManager::SafeLog(const FString& Message, bool bWarning) const
{
	if (bVerboseLogging)
	{
		if (bWarning)
		{
			UE_LOG(LogPACManager, Warning, TEXT("[PACManager] %s"), *Message);
		}
		else
		{
			UE_LOG(LogPACManager, Log, TEXT("[PACManager] %s"), *Message);
		}
	}
}

#pragma endregion

// ============================================================================
// MOTION DATA IMPLEMENTATION
// ============================================================================
#pragma region MOTION DATA


#pragma endregion

// ============================================================================
// CONSTRAINT DATA IMPLEMENTATION
// ============================================================================
#pragma region CONSTRAINT DATA

void FOHConstraintData::UpdateStrain(USkeletalMeshComponent* SkelMesh, float DeltaTime)
{
	if (!ConstraintInstance)
	{
		return;
	}

	FVector SwingStrain = GetSwingStrain(SkelMesh);
	float TwistStrain = GetTwistStrain(SkelMesh);

	float PreviousStrain = CurrentStrain;
	float x = SwingStrain.X;
	float y = SwingStrain.Y;
	float z = TwistStrain;
	CurrentStrain = FMath::Max3(FMath::Abs(x), FMath::Abs(y), z);
	MaxRecordedStrain = FMath::Max(MaxRecordedStrain, CurrentStrain);

	// Calculate jitter metric based on strain change
	float StrainDelta = FMath::Abs(CurrentStrain - PreviousStrain);
	JitterMetric = FMath::Lerp(JitterMetric, StrainDelta, 0.1f);
}

FVector FOHConstraintData::GetSwingStrain(USkeletalMeshComponent* SkelMesh) const
{
	if (!ConstraintInstance)
	{
		return FVector::ZeroVector;
	}

	const float Swing1 = FMath::Abs(GetConstraintInstance(SkelMesh)->GetCurrentSwing1());
	const float Swing2 = FMath::Abs(GetConstraintInstance(SkelMesh)->GetCurrentSwing2());
	const float Swing1Limit = GetConstraintInstance(SkelMesh)->GetAngularSwing1Limit();
	//->ProfileInstance.ConeLimit.Swing1LimitDegrees;
	const float Swing2Limit = GetConstraintInstance(SkelMesh)->GetAngularSwing2Limit();
	//->ProfileInstance.ConeLimit.Swing2LimitDegrees;

	return FVector(
		Swing1Limit > 0.f ? Swing1 / Swing1Limit : 0.f,
		Swing2Limit > 0.f ? Swing2 / Swing2Limit : 0.f,
		0.f
	);
}

float FOHConstraintData::GetTwistStrain(USkeletalMeshComponent* SkelMesh) const
{
	if (!GetConstraintInstance(SkelMesh))
	{
		return 0.f;
	}

	const float Twist = FMath::Abs(GetConstraintInstance(SkelMesh)->GetCurrentTwist());
	const float TwistLimit = GetConstraintInstance(SkelMesh)->GetAngularTwistLimit();
	//->ProfileInstance.TwistLimit.TwistLimitDegrees;

	return TwistLimit > 0.f ? Twist / TwistLimit : 0.f;
}

bool FOHConstraintData::IsOverstressed(float Threshold) const
{
	return CurrentStrain > Threshold;
}

#pragma endregion

// ============================================================================
// MAIN COMPONENT IMPLEMENTATION
// ============================================================================
#pragma region MAIN COMPONENT IMPLEMENTATION

UOHPACManager::UOHPACManager()
{
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	PrimaryComponentTick.bCanEverTick = true;


	// Initialize zero profile for clearing PAC
	ZeroProfile.OrientationStrength = 0.f;
	ZeroProfile.PositionStrength = 0.f;
	ZeroProfile.VelocityStrength = 0.f;
	ZeroProfile.AngularVelocityStrength = 0.f;
	ZeroProfile.bIsLocalSimulation = true;
}

// ==== ENHANCED BeginPlay ====

void UOHPACManager::BeginPlay()
{
	Super::BeginPlay();

	if (!bEnablePACManager)
	{
		return;
	}

	if (bIsInitialized)
	{
		SafeLog(TEXT("Already initialized, skipping BeginPlay initialization"), true);
		return;
	}

	// Initialize bone lists first
	InitializeBoneLists();

	// Ensure physics is properly set up
	EnsurePhysicsTickEnabled();

	ValidateComponentReferences();

	if (bEnableCollisionImpactSystem)
	{

		InitializeCombatChains(); // NEW - instead of InitializeCombatLimbs
		SafeLog(TEXT("Combat tracking initialized"));
	}
	
	// Configure skeletal mesh for physics
	if (SkeletalMesh)
	{
		// Ensure animation updates even when not rendered
		SkeletalMesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;

		// Disable fixed bounds for accurate physics culling
		if (SkeletalMesh->bComponentUseFixedSkelBounds)
		{
			SafeLog(TEXT("Disabling fixed bounds for physics simulation"));
			SkeletalMesh->bComponentUseFixedSkelBounds = false;
		}

		// Ensure proper kinematic bone updates
		SkeletalMesh->KinematicBonesUpdateType = EKinematicBonesUpdateToPhysics::SkipSimulatingBones;

		// Force initial bounds update
		SkeletalMesh->UpdateBounds();
	}
	if (bAutoSetupPhysics)
	{
		PerformAutoSetup();
	}
	else
	{
		FTimerHandle InitTimer;
		GetWorld()->GetTimerManager().SetTimer(InitTimer, this,
		                                       &UOHPACManager::InitializePACManager, InitializationDelay, false);
	}

	// Start physics verification timer
	if (bVerifyPhysicsState)
	{
		GetWorld()->GetTimerManager().SetTimer(PhysicsVerificationTimer, this,
		                                       &UOHPACManager::VerifyPhysicsStateInternal,
		                                       PhysicsVerificationInterval, true);
	}

	// Start cleanup timer
	GetWorld()->GetTimerManager().SetTimer(CleanupTimer, this,
	                                       &UOHPACManager::CleanupStaleBlends, CleanupInterval, true);
	InitializePhysicsBlending();
}


void UOHPACManager::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bEnablePACManager || !SkeletalMesh || !bIsInitialized)
	{
		return;
	}
	// 2. --- Decay Movement Vector (optional, as needed) ---


	float OutValue;
	if (PACStrengthBlendState.Tick(GetWorld()->GetTimeSeconds(), OutValue))
	{
		SetPhysicalAnimationStrengthMultiplier(OutValue);
	}

	// Your existing systems
	UpdateMovementDecay(DeltaTime);
	UpdateMotionTracking(DeltaTime);

	if (bEnableCollisionImpactSystem)
	{
		UpdateCombatChainStates(DeltaTime); // NEW - instead of UpdateCombatLimbStates
		CheckChainCombatCollisions(DeltaTime); // ACTIVE DETECTION
	}

	UpdateConstraintStates(DeltaTime);
	ProcessActiveBlends(DeltaTime);


#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (bDrawDebug)
	{
		DrawDebugOverlay();
	}
#endif
}

void UOHPACManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (bAutoSetupPhysics)
	{
		RestoreOriginalCollisionSettings();
	}

	GetWorld()->GetTimerManager().ClearTimer(InitRetryHandle);
	GetWorld()->GetTimerManager().ClearTimer(AutoSetupRetryTimer);
	GetWorld()->GetTimerManager().ClearTimer(CleanupTimer);


	ResetPACManager();

	Super::EndPlay(EndPlayReason);
}

#pragma endregion

// ============================================================================
// AUTO SETUP IMPLEMENTATION
// ============================================================================
#pragma region AUTO SETUP

void UOHPACManager::SetupPhysicalAnimationComponent()
{
	PhysicalAnimationComponent = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();
	if (!PhysicalAnimationComponent)
	{
		PhysicalAnimationComponent = NewObject<UPhysicalAnimationComponent>(
			GetOwner(), UPhysicalAnimationComponent::StaticClass(), TEXT("AutoPAC"));
		PhysicalAnimationComponent->RegisterComponentWithWorld(GetWorld());
		SafeLog(TEXT("Created new PhysicalAnimationComponent"));
	}
	PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);
	SkeletalMesh->bBlendPhysics = true;
}

void UOHPACManager::PerformAutoSetup()
{
	SafeLog(TEXT("Starting automatic physics setup..."));

	SkeletalMesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
	if (!SkeletalMesh)
	{
		SafeLog(TEXT("No SkeletalMeshComponent found on owner"), true);
		return;
	}

	SetupPhysicsAsset();
	ConfigureCollisionSettings();
	SetupPhysicalAnimationComponent();

	FTimerHandle InitTimer;
	GetWorld()->GetTimerManager().SetTimer(InitTimer, [this]()
	{
		InitializePACManager();
		SafeLog(TEXT("Auto-setup initialization complete"));
	}, AutoSetupDelaySeconds, false);
}

void UOHPACManager::RetryAutoSetup()
{
	static int32 RetryCount = 0;

	if (RetryCount >= AutoSetupMaxRetries)
	{
		SafeLog(TEXT("Max auto-setup retries reached"), true);
		return;
	}

	RetryCount++;
	SafeLog(FString::Printf(TEXT("Retrying auto-setup (attempt %d/%d)"), RetryCount, AutoSetupMaxRetries));
	PerformAutoSetup();
}

void UOHPACManager::SetupPhysicsAsset()
{
	if (!SkeletalMesh)
	{
		SafeLog(TEXT("No SkeletalMesh component available for physics asset setup"), true);
		return;
	}

	UPhysicsAsset* PhysicsAsset = nullptr;

	// 1. Try skeletal mesh asset first (most common)
	if (SkeletalMesh->GetSkeletalMeshAsset())
	{
		PhysicsAsset = SkeletalMesh->GetSkeletalMeshAsset()->GetPhysicsAsset();
		if (PhysicsAsset)
		{
			SafeLog(TEXT("Using PhysicsAsset from SkeletalMesh asset"));
		}
	}

	// 2. Try component's physics asset override
	if (!PhysicsAsset)
	{
		PhysicsAsset = SkeletalMesh->GetPhysicsAsset();
		if (PhysicsAsset)
		{
			SafeLog(TEXT("Using PhysicsAsset from component override"));
		}
	}

	// 3. Fall back to default
	if (!PhysicsAsset && DefaultPhysicsAsset)
	{
		PhysicsAsset = DefaultPhysicsAsset;
		SafeLog(FString::Printf(TEXT("No PhysicsAsset found on SkeletalMesh '%s', using default asset"),
		                        SkeletalMesh->GetSkeletalMeshAsset()
			                        ? *SkeletalMesh->GetSkeletalMeshAsset()->GetName()
			                        : TEXT("None")), true);
	}

	// 4. Final validation
	if (!PhysicsAsset)
	{
		SafeLog(TEXT("ERROR: No PhysicsAsset available - physics simulation will not work!"), true);
		CachedPhysicsAsset = nullptr;
		return;
	}

	// 5. CRITICAL: Validate this is the correct asset (has arms)
	int32 ArmBoneCount = 0;

	for (const USkeletalBodySetup* BodySetup : PhysicsAsset->SkeletalBodySetups)
	{
		if (BodySetup)
		{
			FString BoneName = BodySetup->BoneName.ToString();
			if (BoneName.Contains(TEXT("upperarm")) || BoneName.Contains(TEXT("lowerarm")) ||
				BoneName.Contains(TEXT("hand")))
			{
				ArmBoneCount++;
			}
		}
	}

	// Expect at least 6 arm bones (upper, lower, hand for each side)
	bool bHasExpectedBones = ArmBoneCount >= 6;

	if (!bHasExpectedBones)
	{
		SafeLog(FString::Printf(TEXT("WARNING: Physics asset '%s' only has %d arm bones (expected at least 6)"),
		                        *PhysicsAsset->GetName(), ArmBoneCount), true);

		// Try to use default if available
		if (DefaultPhysicsAsset && DefaultPhysicsAsset != PhysicsAsset)
		{
			SafeLog(TEXT("Attempting to use default physics asset instead"));
			PhysicsAsset = DefaultPhysicsAsset;
		}
	}

	// 6. Apply and cache
	CachedPhysicsAsset = PhysicsAsset;

	// Only set on component if different from current
	if (SkeletalMesh->GetPhysicsAsset() != PhysicsAsset)
	{
		SkeletalMesh->SetPhysicsAsset(PhysicsAsset, true);
		SafeLog(TEXT("Applied PhysicsAsset to SkeletalMeshComponent"));
	}

	SafeLog(FString::Printf(TEXT("Physics asset configured successfully: %s"), *PhysicsAsset->GetName()));
}

void UOHPACManager::InitializePhysicsBlending()
{
	if (!SkeletalMesh)
	{
		SafeLog(TEXT("InitializePhysicsBlending: SkeletalMesh is null"), true);
		return;
	}

	if (!CachedPhysicsAsset)
	{
		SetupPhysicsAsset();
		if (!CachedPhysicsAsset)
		{
			SafeLog(TEXT("InitializePhysicsBlending: No physics asset available"), true);
			return;
		}
	}

	// CRITICAL: Enable blend physics for Chaos registration
	SkeletalMesh->bBlendPhysics = true;

	// Ensure we have valid bone indices
	if (SimulatableBones.Num() == 0)
	{
		SafeLog(TEXT("InitializePhysicsBlending: No simulatable bones defined"), true);
		return;
	}

	// Step 1: Use the ORIGINAL approach - SetAllBodiesBelowSimulatePhysics
	// This is what enables smooth Chaos blending in UE 5.3
	if (!RootReferenceBone.IsNone())
	{
		SkeletalMesh->SetAllBodiesBelowSimulatePhysics(RootReferenceBone, true, false);
		SafeLog(FString::Printf(TEXT("Enabled physics simulation below %s for Chaos registration"),
		                        *RootReferenceBone.ToString()));
	}
	else
	{
		SafeLog(TEXT("InitializePhysicsBlending: RootReferenceBone is None, using pelvis"), true);
		SkeletalMesh->SetAllBodiesBelowSimulatePhysics(TEXT("pelvis"), true, false);
	}

	// Step 2: IMMEDIATELY set all blend weights to 0 and sleep bodies
	// This prevents the capsule separation
	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body || !Body->IsValidBodyInstance())
		{
			continue;
		}

		// Force blend weight to 0
		Body->PhysicsBlendWeight = 0.0f;

		// Put to sleep immediately
		Body->PutInstanceToSleep();

		// Clear any velocities
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
	}

	// Step 3: Apply minimal global blend weight for Chaos
	SkeletalMesh->SetAllBodiesPhysicsBlendWeight(0.001f, false);

	// Step 4: Reset weights again to ensure no movement
	for (const FName& BoneName : SimulatableBones)
	{
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			Body->PhysicsBlendWeight = 0.0f;
			Body->PutInstanceToSleep();
		}
	}

	// Step 5: Force physics state update
	SkeletalMesh->RecreatePhysicsState();

	SafeLog(TEXT("Physics blending initialized for Chaos (smooth interpolation ready)"));
}

void UOHPACManager::ConfigureCollisionSettings()
{
	if (!SkeletalMesh)
	{
		return;
	}

	SkeletalMesh->SetCollisionProfileName("PhysicsActor");
	// Store original collision settings
	OriginalCollisionSettings.Empty();
	const TArray<FBodyInstance*>& Bodies = SkeletalMesh->Bodies;

	for (int32 i = 0; i < Bodies.Num(); i++)
	{
		if (FBodyInstance* Body = Bodies[i])
		{
			FName BoneName = SkeletalMesh->GetBoneName(i);
			OriginalCollisionSettings.Add(BoneName, Body->GetCollisionEnabled());

			// Enable query and physics for all bodies
			Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		}
	}

	SafeLog(FString::Printf(TEXT("Configured collision for %d bodies"), OriginalCollisionSettings.Num()));
}

void UOHPACManager::RestoreOriginalCollisionSettings()
{
	if (!SkeletalMesh || OriginalCollisionSettings.Num() == 0)
	{
		return;
	}

	for (const auto& Pair : OriginalCollisionSettings)
	{
		if (FBodyInstance* Body = GetBodyInstanceDirect(Pair.Key))
		{
			Body->SetCollisionEnabled(Pair.Value);
		}
	}

	OriginalCollisionSettings.Empty();
	SafeLog(TEXT("Restored original collision settings"));
}

#pragma endregion

// ============================================================================
// INITIALIZATION
// ============================================================================
#pragma region INITIALIZATION

void UOHPACManager::InitializePACManager()
{
	if (bIsInitialized)
	{
		SafeLog(TEXT("Already initialized"), true);
		return;
	}

	if (!GetOwner())
	{
		SafeLog(TEXT("No owner found"), true);
		StartInitializationRetry();
		return;
	}

	// Find components
	if (!SkeletalMesh)
	{
		SkeletalMesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
	}

	if (!PhysicalAnimationComponent)
	{
		PhysicalAnimationComponent = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();
	}

	if (!SkeletalMesh || !PhysicalAnimationComponent)
	{
		SafeLog(TEXT("Required components not found"), true);
		StartInitializationRetry();
		return;
	}

	// Bind PAC to skeletal mesh
	PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);

	if (CoreBones.Num() == 0)
	{
		CoreBones = {
			TEXT("spine_01"), TEXT("spine_02"), TEXT("spine_03"),
			TEXT("neck_01"), TEXT("head"),
			TEXT("pelvis"), TEXT("hips"),
			TEXT("chest"), TEXT("ribcage")
		};
	}
	// ADD: Initialize combat tracking
	if (bEnableCollisionImpactSystem)
	{
		InitializeCombatChains(); // NEW - instead of InitializeCombatTracking
		SafeLog(TEXT("Combat tracking system initialized"));
		// Initialize core bones set
	}
	// Initialize subsystems
	InitializeStrengthProfiles();
	InitializeBoneLists();
	BuildConstraintData();
	// CRITICAL: Validate component references early
	ValidateComponentReferences();


#if WITH_EDITOR
	// Subscribe to mesh change events
	if (SkeletalMesh->GetSkeletalMeshAsset())
	{
		SkeletalMesh->GetSkeletalMeshAsset()->GetOnMeshChanged().
		              AddUObject(this, &UOHPACManager::OnSkeletalMeshChanged);
	}
#endif


	bIsInitialized = true;
	SafeLog(TEXT("PAC Manager initialized successfully"));
	OnPACManagerInitialized.Broadcast();
}

void UOHPACManager::StartInitializationRetry()
{
	if (GetWorld() && InitRetryTotalDurationSeconds > 0.0f)
	{
		GetWorld()->GetTimerManager().SetTimer(
			InitRetryHandle,
			this,
			&UOHPACManager::RetryInitializePACManager,
			InitRetryIntervalSeconds,
			true
		);

		GetWorld()->GetTimerManager().SetTimer(
			InitRetryHandle,
			[this]()
			{
				GetWorld()->GetTimerManager().ClearTimer(InitRetryHandle);
				SafeLog(TEXT("PAC Manager initialization failed after retries"), true);
			},
			InitRetryTotalDurationSeconds,
			false
		);
	}
}

void UOHPACManager::RetryInitializePACManager()
{
	InitializePACManager();

	if (bIsInitialized)
	{
		GetWorld()->GetTimerManager().ClearTimer(InitRetryHandle);
	}
}

void UOHPACManager::ResetPACManager()
{
	StopAllBlends();
	ActiveBlends.Empty();
	DisableAllPhysicsBodies();
	BoneMotionMap.Empty();
	ConstraintDataMap.Empty();
	ConstraintLookupMap.Empty(); // ADD THIS
	ActiveBlends.Empty();
	PersistentSimulations.Empty();
	BoneBaselines.Empty();
	ClearConstraintData();
	BoneIndexCache.Empty();

	// ADD: Ensure physics component is clean
	if (PhysicalAnimationComponent && SkeletalMesh)
	{
		// Clear all PAC settings
		for (const FName& BoneName : SimulatableBones)
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);
		}
	}

	bIsInitialized = false;

	SafeLog(TEXT("PAC Manager reset"));

	InitializePACManager();
}

void UOHPACManager::InitializeBoneLists()
{
	SafeLog(TEXT("Initializing bone lists from configuration"));

	// Initialize motion tracking for tracked bones
	InitializeMotionTracking();

	// Cache simulatable bones based on tracked bones and exclusions
	CacheSimulatableBones();

	// Build category map
	BuildCategoryBonesMap();


	SafeLog(FString::Printf(TEXT("Initialized with %d tracked bones, %d simulatable bones"),
	                        TrackedBones.Num(), SimulatableBones.Num()));
}

// ==== BUILD CATEGORY BONES MAP ====
void UOHPACManager::InitializeBoneCategoryDefinitions()
{
	// Clear any existing
	BoneCategoryDefinitions.Empty();

	// Upper Body
	BoneCategoryDefinitions.Add(EOHBoneCategory::UpperBodyBones, {
		                            TEXT("spine_01"), TEXT("spine_02"), TEXT("spine_03"),
		                            TEXT("clavicle_l"), TEXT("clavicle_r"),
		                            TEXT("upperarm_l"), TEXT("upperarm_r"),
		                            TEXT("lowerarm_l"), TEXT("lowerarm_r"),
		                            TEXT("hand_l"), TEXT("hand_r")
	                            });

	// Lower Body
	BoneCategoryDefinitions.Add(EOHBoneCategory::LowerBodyBones, {
		                            TEXT("pelvis"), TEXT("spine_01"),
		                            TEXT("thigh_l"), TEXT("thigh_r"),
		                            TEXT("calf_l"), TEXT("calf_r"),
		                            TEXT("foot_l"), TEXT("foot_r")
	                            });

	// Spine
	BoneCategoryDefinitions.Add(EOHBoneCategory::SpineBones, {
		                            TEXT("pelvis"), TEXT("spine_01"), TEXT("spine_02"), TEXT("spine_03")
	                            });

	// Head
	BoneCategoryDefinitions.Add(EOHBoneCategory::HeadBones, {
		                            TEXT("neck_01"), TEXT("head")
	                            });

	// Left Arm
	BoneCategoryDefinitions.Add(EOHBoneCategory::LeftArmBones, {
		                            TEXT("clavicle_l"), TEXT("upperarm_l"), TEXT("lowerarm_l"), TEXT("hand_l")
	                            });

	// Right Arm
	BoneCategoryDefinitions.Add(EOHBoneCategory::RightArmBones, {
		                            TEXT("clavicle_r"), TEXT("upperarm_r"), TEXT("lowerarm_r"), TEXT("hand_r")
	                            });

	// All Arms
	BoneCategoryDefinitions.Add(EOHBoneCategory::ArmBones, {
		                            TEXT("clavicle_l"), TEXT("clavicle_r"),
		                            TEXT("upperarm_l"), TEXT("upperarm_r"),
		                            TEXT("lowerarm_l"), TEXT("lowerarm_r"),
		                            TEXT("hand_l"), TEXT("hand_r")
	                            });

	// Clavicles
	BoneCategoryDefinitions.Add(EOHBoneCategory::ClavicleBones, {
		                            TEXT("clavicle_l"), TEXT("clavicle_r")
	                            });

	// Upper Arms
	BoneCategoryDefinitions.Add(EOHBoneCategory::UpperArmBones, {
		                            TEXT("upperarm_l"), TEXT("upperarm_r")
	                            });

	// Lower Arms
	BoneCategoryDefinitions.Add(EOHBoneCategory::LowerArmBones, {
		                            TEXT("lowerarm_l"), TEXT("lowerarm_r")
	                            });

	// Hands
	BoneCategoryDefinitions.Add(EOHBoneCategory::HandBones, {
		                            TEXT("hand_l"), TEXT("hand_r")
	                            });

	// Left Leg
	BoneCategoryDefinitions.Add(EOHBoneCategory::LeftLegBones, {
		                            TEXT("thigh_l"), TEXT("calf_l"), TEXT("foot_l")
	                            });

	// Right Leg
	BoneCategoryDefinitions.Add(EOHBoneCategory::RightLegBones, {
		                            TEXT("thigh_r"), TEXT("calf_r"), TEXT("foot_r")
	                            });

	// All Legs
	BoneCategoryDefinitions.Add(EOHBoneCategory::LegBones, {
		                            TEXT("thigh_l"), TEXT("thigh_r"),
		                            TEXT("calf_l"), TEXT("calf_r"),
		                            TEXT("foot_l"), TEXT("foot_r")
	                            });

	// Thighs
	BoneCategoryDefinitions.Add(EOHBoneCategory::ThighBones, {
		                            TEXT("thigh_l"), TEXT("thigh_r")
	                            });

	// Calves
	BoneCategoryDefinitions.Add(EOHBoneCategory::CalfBones, {
		                            TEXT("calf_l"), TEXT("calf_r")
	                            });

	// Feet
	BoneCategoryDefinitions.Add(EOHBoneCategory::FootBones, {
		                            TEXT("foot_l"), TEXT("foot_r")
	                            });

	// Custom (empty by default)
	BoneCategoryDefinitions.Add(EOHBoneCategory::Custom, {});
	BuildCategoryBonesMap();
}


void UOHPACManager::BuildCategoryBonesMap()
{
	CategoryBonesMap.Empty();

	// Build map for efficient runtime lookup
	for (const auto& CategoryDef : BoneCategoryDefinitions)
	{
		EOHBoneCategory Category = CategoryDef.Key;
		const TArray<FName>& Bones = CategoryDef.Value.Bones;

		// Add bones to category map
		CategoryBonesMap.Add(Category, Bones);

		// Also populate the bone to category map for reverse lookup
		for (const FName& BoneName : Bones)
		{
			if (!BoneCategoryMap.Contains(BoneName))
			{
				BoneCategoryMap.Add(BoneName, Category);
			}
		}
	}

	SafeLog(FString::Printf(TEXT("Built category map with %d categories"), CategoryBonesMap.Num()));
}


void UOHPACManager::InitializeMotionTracking()
{
	BoneMotionMap.Empty();

	if (!SkeletalMesh)
	{
		return;
	}

	for (const FName& BoneName : SimulatableBones)
	{
		BoneMotionMap.Add(BoneName, FOHBoneMotionData());
	}

	SafeLog(FString::Printf(TEXT("Initialized motion tracking for %d bones"), BoneMotionMap.Num()));
}

void UOHPACManager::InitializeStrengthProfiles()
{
	StrengthProfiles.Empty();

	// Very Light
	FPhysicalAnimationData VeryLight;
	VeryLight.PositionStrength = 100.0f;
	VeryLight.VelocityStrength = 10.0f;
	VeryLight.OrientationStrength = 500.0f;
	VeryLight.AngularVelocityStrength = 50.0f;
	VeryLight.bIsLocalSimulation = true;
	StrengthProfiles.Add(EOHPhysicsStrength::VeryLight, VeryLight);

	// Light
	FPhysicalAnimationData Light;
	Light.PositionStrength = 250.0f;
	Light.VelocityStrength = 25.0f;
	Light.OrientationStrength = 1000.0f;
	Light.AngularVelocityStrength = 100.0f;
	Light.bIsLocalSimulation = true;
	StrengthProfiles.Add(EOHPhysicsStrength::Light, Light);

	// Medium
	FPhysicalAnimationData Medium;
	Medium.PositionStrength = 500.0f;
	Medium.VelocityStrength = 50.0f;
	Medium.OrientationStrength = 2000.0f;
	Medium.AngularVelocityStrength = 200.0f;
	Medium.bIsLocalSimulation = true;
	StrengthProfiles.Add(EOHPhysicsStrength::Medium, Medium);

	// Strong
	FPhysicalAnimationData Strong;
	Strong.PositionStrength = 1000.0f;
	Strong.VelocityStrength = 100.0f;
	Strong.OrientationStrength = 4000.0f;
	Strong.AngularVelocityStrength = 400.0f;
	Strong.bIsLocalSimulation = true;
	StrengthProfiles.Add(EOHPhysicsStrength::Strong, Strong);

	// Very Strong
	FPhysicalAnimationData VeryStrong;
	VeryStrong.PositionStrength = 2000.0f;
	VeryStrong.VelocityStrength = 200.0f;
	VeryStrong.OrientationStrength = 8000.0f;
	VeryStrong.AngularVelocityStrength = 800.0f;
	VeryStrong.bIsLocalSimulation = true;
	StrengthProfiles.Add(EOHPhysicsStrength::VeryStrong, VeryStrong);

	// Initialize zero profile
	ZeroProfile.OrientationStrength = 0.f;
	ZeroProfile.PositionStrength = 0.f;
	ZeroProfile.VelocityStrength = 0.f;
	ZeroProfile.AngularVelocityStrength = 0.f;
	ZeroProfile.bIsLocalSimulation = true;

	// Initialize all bone categories HERE, not in header
	InitializeBoneCategoryDefinitions();
}

#pragma endregion


// ============================================================================
// UNIFIED PHYSICAL ANIMATION CONTROL
// ============================================================================
#pragma region UNIFIED PHYSICAL ANIMATION

// ==== UPDATE SetBonePhysicalAnimation_Internal ====

bool UOHPACManager::SetBonePhysicalAnimation_Internal(
	FName BoneName,
	bool bEnable,
	const FPhysicalAnimationData& Profile,
	bool bAffectBelow,
	const TArray<FName>& FilterBones,
	bool bEnableCollision,
	bool bWakeBody,
	bool bEnableNativeForcePropagation)
{
	if (!SkeletalMesh || !PhysicalAnimationComponent)
	{
		SafeLog(FString::Printf(
			        TEXT(
				        "SetBonePhysicalAnimation_Internal: SkeletalMesh or PhysicalAnimationComponent is missing for %s"),
			        *BoneName.ToString()), true);
		return false;
	}

	FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
	if (!Body || !Body->IsValidBodyInstance())
	{
		SafeLog(FString::Printf(
			        TEXT(
				        "SetBonePhysicalAnimation_Internal: Invalid or null BodyInstance for %s, attempting refresh..."),
			        *BoneName.ToString()), true);
		RefreshBodyInstances();
		Body = GetBodyInstanceDirect(BoneName);
		if (!Body || !Body->IsValidBodyInstance())
		{
			SafeLog(FString::Printf(
				        TEXT("SetBonePhysicalAnimation_Internal: Still invalid BodyInstance after refresh for %s"),
				        *BoneName.ToString()), true);
			return false;
		}
	}

	// Optionally skip in editor (optional, remove if you want editor-side tools to work)
	if (GIsEditor && !GetWorld()->IsGameWorld())
	{
		SafeLog(TEXT("SetBonePhysicalAnimation_Internal: Called in editor, skipping."), true);
		return false;
	}

	if (bEnable)
	{
		// STEP 1: Apply PAC profile FIRST (before enabling simulation)
		if (bAffectBelow && bEnableNativeForcePropagation)
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(BoneName, Profile);
		}
		else
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);
		}

		SafeLog(FString::Printf(TEXT("Applied PAC profile to %s (Strength: %.1f)"),
		                        *BoneName.ToString(), Profile.OrientationStrength));

		// STEP 2: Configure collision
		if (bEnableCollision)
		{
			Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		}

		// STEP 3: Enable physics simulation AFTER profile is set
		if (!Body->IsInstanceSimulatingPhysics())
		{
			Body->SetInstanceSimulatePhysics(true);

			if (bWakeBody)
			{
				Body->WakeInstance();
			}

			UpdateSkeletalMeshBounds();
			OnBoneStartedSimulating.Broadcast(BoneName);
		}

		return true;
	}
	// Disable in reverse order
	// STEP 1: Disable physics
	if (Body->IsInstanceSimulatingPhysics())
	{
		Body->SetInstanceSimulatePhysics(false);
		Body->PhysicsBlendWeight = 0.0f;
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		Body->PutInstanceToSleep();
	}

	// STEP 2: Clear PAC profile
	ClearPhysicalAnimationProfile(BoneName);

	// STEP 3: Update collision
	if (!bEnableCollision)
	{
		Body->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}

	UpdateSkeletalMeshBounds();
	OnBoneStoppedSimulating.Broadcast(BoneName);

	return true;
}

bool UOHPACManager::SetBonePhysicalAnimation(
	FName BoneName,
	bool bEnable,
	const FPhysicalAnimationData& Profile,
	bool bIncludeChain,
	bool bEnableCollision,
	bool bWakeBody)
{
	return SetBonePhysicalAnimation_Internal(BoneName, bEnable, Profile,
	                                         bIncludeChain, TArray<FName>(),
	                                         bEnableCollision, bWakeBody);
}

bool UOHPACManager::SetBonePhysicalAnimationWithFilter(
	FName BoneName,
	bool bEnable,
	const FPhysicalAnimationData& Profile,
	bool bIncludeChain,
	const TArray<FName>& FilterBones,
	bool bEnableCollision,
	bool bWakeBody)
{
	return SetBonePhysicalAnimation_Internal(BoneName, bEnable, Profile,
	                                         bIncludeChain, FilterBones,
	                                         bEnableCollision, bWakeBody);
}


void UOHPACManager::ApplyPhysicalAnimationProfile(FName BoneName, const FPhysicalAnimationData& Profile,
                                                  bool bAffectChain)
{
	if (PhysicalAnimationComponent)
	{
		if (bAffectChain)
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(BoneName, Profile);
			TArray<FName> BonesToSet = GetBoneChain(BoneName, -1);
			for (const FName& Bone : BonesToSet)
			{
				if (FBodyInstance* Body = GetBodyInstanceDirect(Bone))
				{
					Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
					Body->WakeInstance();
				}
			}
		}
		else
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);
			WakePhysicsBody(BoneName);
		}
	}
}


void UOHPACManager::ClearPhysicalAnimationProfile(FName BoneName)
{
	if (PhysicalAnimationComponent)
	{
		PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);
	}
}

#pragma endregion


// ============================================================================
// SIMULATION CONTROL
// ============================================================================
#pragma region SIMULATION CONTROL

bool UOHPACManager::SetSimulation(FName BoneName, bool bEnable, bool bAllBelow)
{
	if (!PhysicalAnimationComponent || !SkeletalMesh)
	{
		return false;
	}

	if (bEnable)
	{
		if (bAllBelow)
		{
			SkeletalMesh->SetAllBodiesBelowSimulatePhysics(BoneName, true, false);
			TArray<FName> BonesToSet = GetBoneChain(BoneName, -1);

			for (const FName& Bone : BonesToSet)
			{
				if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone))
				{
					Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
					Body->WakeInstance();
				}
				OnBoneStartedSimulating.Broadcast(Bone);
			}
			return true;
		}
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			Body->SetInstanceSimulatePhysics(true);
			Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			Body->WakeInstance();
			OnBoneStartedSimulating.Broadcast(BoneName);
			return true;
		}
	}
	else
	{
		if (bAllBelow)
		{
			SkeletalMesh->SetAllBodiesBelowSimulatePhysics(BoneName, false, false);
			TArray<FName> BonesToClear = GetBoneChain(BoneName, -1);

			for (const FName& Bone : BonesToClear)
			{
				if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone))
				{
					Body->SetLinearVelocity(FVector::ZeroVector, false);
					Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
					Body->PutInstanceToSleep();
				}
				OnBoneStoppedSimulating.Broadcast(Bone);
			}
			return true;
		}
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			Body->SetInstanceSimulatePhysics(false);
			Body->SetLinearVelocity(FVector::ZeroVector, false);
			Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
			Body->PutInstanceToSleep();
			OnBoneStoppedSimulating.Broadcast(BoneName);
			return true;
		}
	}

	return false;
}

void UOHPACManager::EnsureBoneSimulatingPhysics(FName BoneName, bool bEnableChain)
{
	if (!IsBoneSimulating(BoneName))
	{
		SetSimulation(BoneName, true, bEnableChain);
	}
}

// ==== UPDATE EnablePhysicsForBone in OHPACManager.cpp ====

bool UOHPACManager::EnablePhysicsForBone(
	FName BoneName,
	const FPhysicalAnimationData& Profile,
	bool bEnableCollision,
	bool bWakeBody,
	bool bEnableDrivePropagation) // NEW PARAMETER with default = true
{
	FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
	if (!Body || !Body->IsValidBodyInstance())
	{
		SafeLog(FString::Printf(TEXT("Invalid body instance for bone %s"), *BoneName.ToString()), true);

		// Try to refresh and get again
		RefreshBodyInstances();
		Body = GetBodyInstanceDirect(BoneName);

		if (!Body || !Body->IsValidBodyInstance())
		{
			return false;
		}
	}

	// Log state change
	if (bLogStateChanges)
	{
		SafeLog(FString::Printf(TEXT("Enabling physics for bone %s (Drive Propagation: %s)"),
		                        *BoneName.ToString(),
		                        bEnableDrivePropagation ? TEXT("Yes") : TEXT("No")));
	}

	// Clear velocities FIRST
	Body->SetLinearVelocity(FVector::ZeroVector, false);
	Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);

	// Set damping to prevent spinning
	Body->LinearDamping = 0.5f;
	Body->AngularDamping = 5.0f;

	// Set collision BEFORE enabling simulation
	if (bEnableCollision)
	{
		Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	}

	// Apply physical animation profile
	if (bEnableDrivePropagation)
	{
		// USE APPLY BELOW FOR AUTOMATIC FORCE PROPAGATION!
		PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(BoneName, Profile);

		SafeLog(FString::Printf(TEXT("Applied physics settings BELOW %s with force propagation"),
		                        *BoneName.ToString()));
	}
	else
	{
		// Single bone only
		PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);

		SafeLog(FString::Printf(TEXT("Applied physics settings to %s only"),
		                        *BoneName.ToString()));
	}
	// ADD THIS: Force constraint data rebuild after applying settings
	FTimerHandle RebuildTimer;
	GetWorld()->GetTimerManager().SetTimer(RebuildTimer, [this]()
	{
		BuildConstraintData();
		SafeLog(TEXT("Rebuilt constraint data after PAC application"));
	}, 0.1f, false);
	// Enable simulation
	Body->SetInstanceSimulatePhysics(true);

	// Force update physics state
	Body->UpdateBodyScale(Body->Scale3D, true);
	Body->UpdatePhysicsFilterData();

	// Wake body
	if (bWakeBody)
	{
		Body->WakeInstance();
	}

	// Verify simulation was actually enabled
	if (!Body->IsInstanceSimulatingPhysics())
	{
		SafeLog(FString::Printf(TEXT("Failed to enable simulation for %s, retrying..."),
		                        *BoneName.ToString()), true);

		// Force recreation of physics state
		Body->TermBody();
		Body->InitBody(Body->GetBodySetup(), SkeletalMesh->GetComponentTransform(),
		               SkeletalMesh, SkeletalMesh->GetWorld()->GetPhysicsScene());

		Body->SetInstanceSimulatePhysics(true);
	}

	OnBoneStartedSimulating.Broadcast(BoneName);
	return true;
}


bool UOHPACManager::DisablePhysicsForBone(FName BoneName)
{
	ClearPhysicalAnimationProfile(BoneName);

	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		if (bLogStateChanges)
		{
			SafeLog(FString::Printf(TEXT("Disabling physics for bone %s"), *BoneName.ToString()));
		}

		Body->SetInstanceSimulatePhysics(false);
		Body->PhysicsBlendWeight = 0.0f;
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		Body->PutInstanceToSleep();
	}

	OnBoneStoppedSimulating.Broadcast(BoneName);
	return true;
}

void UOHPACManager::DisableAllPhysicsBodies()
{
	if (!SkeletalMesh)
	{
		return;
	}

	const int32 NumBodies = SkeletalMesh->Bodies.Num();

	for (int32 i = 0; i < NumBodies; ++i)
	{
		const FName BoneName = SkeletalMesh->GetBoneName(i);
		FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);

		if (Body && Body->IsValidBodyInstance())
		{
			Body->SetInstanceSimulatePhysics(false, true);
			Body->SetLinearVelocity(FVector::ZeroVector, false);
			Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		}
	}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
#if WITH_EDITOR
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Cyan, TEXT("[PAC] All physics bodies disabled."));
	}
#endif
#endif
}

void UOHPACManager::RefreshBodyInstances()
{
	if (!SkeletalMesh)
	{
		return;
	}

	// Clear body instance cache
	//BodyInstanceCache.Empty();

	// Force recreation of body instances
	SkeletalMesh->RecreatePhysicsState();

	// Update all body scales with correct parameters
	const TArray<FBodyInstance*>& Bodies = SkeletalMesh->Bodies;
	for (FBodyInstance* Body : Bodies)
	{
		if (Body && Body->IsValidBodyInstance())
		{
			// FIXED: UpdateBodyScale with correct parameters
			FVector CurrentScale = Body->Scale3D;
			Body->UpdateBodyScale(CurrentScale, true); // Force update
			Body->UpdatePhysicsFilterData();
		}
	}

	SafeLog(TEXT("Refreshed all body instances"));
}

// ==== VERIFY BONE PHYSICS SETUP ====

bool UOHPACManager::VerifyBonePhysicsSetup(FName BoneName)
{
	if (!SkeletalMesh || !PhysicalAnimationComponent)
	{
		SafeLog(FString::Printf(TEXT("VerifyBonePhysicsSetup: Missing components for bone %s"),
		                        *BoneName.ToString()), true);
		return false;
	}

	// Check if bone exists in skeleton
	int32 BoneIndex = GetBoneIndexDirect(BoneName);
	if (BoneIndex == INDEX_NONE)
	{
		SafeLog(FString::Printf(TEXT("VerifyBonePhysicsSetup: Bone %s not found in skeleton"),
		                        *BoneName.ToString()), true);
		return false;
	}

	// Check if body instance exists
	FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
	if (!Body)
	{
		SafeLog(FString::Printf(TEXT("VerifyBonePhysicsSetup: No body instance for bone %s"),
		                        *BoneName.ToString()), true);
		return false;
	}

	// Verify body instance is valid
	if (!Body->IsValidBodyInstance())
	{
		SafeLog(FString::Printf(TEXT("VerifyBonePhysicsSetup: Invalid body instance for bone %s"),
		                        *BoneName.ToString()), true);

		// Try to fix it
		FixBodyPhysicsState(BoneName, Body);

		// Check again
		return Body->IsValidBodyInstance();
	}

	// Check if physics asset has this bone
	if (CachedPhysicsAsset)
	{
		bool bFoundInAsset = false;
		for (const UBodySetup* BodySetup : CachedPhysicsAsset->SkeletalBodySetups)
		{
			if (BodySetup && BodySetup->BoneName == BoneName)
			{
				bFoundInAsset = true;
				break;
			}
		}

		if (!bFoundInAsset)
		{
			SafeLog(FString::Printf(TEXT("VerifyBonePhysicsSetup: Bone %s not in physics asset"),
			                        *BoneName.ToString()), true);
			return false;
		}
	}

	return true;
}

// ==== FIX BODY PHYSICS STATE ====

void UOHPACManager::FixBodyPhysicsState(FName BoneName, FBodyInstance* Body)
{
	if (!Body || !SkeletalMesh)
	{
		return;
	}

	SafeLog(FString::Printf(TEXT("Attempting to fix physics state for bone %s"), *BoneName.ToString()));

	// Get the body setup
	UBodySetup* BodySetup = Body->GetBodySetup();
	if (!BodySetup)
	{
		SafeLog(FString::Printf(TEXT("No body setup found for bone %s"), *BoneName.ToString()), true);
		return;
	}

	// Store current simulation state
	bool bWasSimulating = Body->IsInstanceSimulatingPhysics();
	float OldBlendWeight = Body->PhysicsBlendWeight;

	// Terminate and reinitialize the body
	Body->TermBody();

	// Wait a frame for physics to clean up
	if (GetWorld())
	{
		// Reinitialize the body
		FTransform BodyTransform = SkeletalMesh->GetBoneTransform(GetBoneIndexDirect(BoneName));
		Body->InitBody(BodySetup, BodyTransform, SkeletalMesh, SkeletalMesh->GetWorld()->GetPhysicsScene());

		// Restore simulation state
		if (bWasSimulating)
		{
			Body->SetInstanceSimulatePhysics(true);
			Body->PhysicsBlendWeight = OldBlendWeight;

			// Reapply any active PAC settings
			if (PhysicalAnimationComponent)
			{
				const TArray<FOHActiveBlend>* ActiveBlendsForBone = ActiveBlends.Find(BoneName);
				if (ActiveBlendsForBone && ActiveBlendsForBone->Num() > 0)
				{
					// Find the strongest active blend
					FPhysicalAnimationData StrongestProfile = ZeroProfile;
					for (const FOHActiveBlend& Blend : *ActiveBlendsForBone)
					{
						if (!Blend.IsComplete())
						{
							FPhysicalAnimationData BlendedProfile = UOHSkeletalPhysicsUtils::LerpProfiles(
								Blend.PhysicsProfile_Start,
								Blend.PhysicsProfile_Target,
								Blend.PhysicsBlendWeight_Current
							);
							StrongestProfile = UOHSkeletalPhysicsUtils::GetStrongerProfile(
								StrongestProfile, BlendedProfile);
						}
					}

					// Reapply the profile
					PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, StrongestProfile);
				}
			}

			// Wake the body
			Body->WakeInstance();
		}
	}

	SafeLog(FString::Printf(TEXT("Fixed physics state for bone %s"), *BoneName.ToString()));
}


// ==== VERIFY PHYSICS STATE INTERNAL ====

void UOHPACManager::VerifyPhysicsStateInternal()
{
	if (!SkeletalMesh || !PhysicalAnimationComponent || !bVerifyPhysicsState)
	{
		return;
	}

	int32 IssuesFixed = 0;
	int32 IssuesFound = 0;

	// Check all bones with active blends
	for (const auto& Pair : ActiveBlends)
	{
		FName BoneName = Pair.Key;
		const TArray<FOHActiveBlend>& Blends = Pair.Value;

		// Skip if no active blends
		if (Blends.Num() == 0)
		{
			continue;
		}

		// Calculate expected state
		float ExpectedWeight = 0.0f;
		bool bHasActiveBlend = false;

		for (const FOHActiveBlend& Blend : Blends)
		{
			if (!Blend.IsComplete())
			{
				bHasActiveBlend = true;
				ExpectedWeight = FMath::Max(ExpectedWeight, Blend.PhysicsBlendWeight_Current);
			}
		}

		// Skip if no active blends
		if (!bHasActiveBlend)
		{
			continue;
		}

		// Get actual state
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body)
		{
			SafeLog(FString::Printf(TEXT("VerifyPhysicsState: Missing body for bone %s with active blends"),
			                        *BoneName.ToString()), true);
			IssuesFound++;
			continue;
		}

		bool bShouldSimulate = ExpectedWeight > 0.01f;
		bool bIsSimulating = Body->IsInstanceSimulatingPhysics();
		float ActualWeight = Body->PhysicsBlendWeight;

		// Check for simulation state mismatch
		if (bShouldSimulate != bIsSimulating)
		{
			IssuesFound++;

			if (bLogStateChanges)
			{
				SafeLog(FString::Printf(TEXT("Physics state mismatch for %s: Should simulate=%d, Is simulating=%d"),
				                        *BoneName.ToString(), bShouldSimulate, bIsSimulating));
			}

			// Fix simulation state
			if (bShouldSimulate)
			{
				Body->SetInstanceSimulatePhysics(true);
				Body->WakeInstance();

				// Ensure collision is enabled
				if (Body->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
				{
					Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				}
			}
			else
			{
				Body->SetInstanceSimulatePhysics(false);
				Body->PhysicsBlendWeight = 0.0f;
			}

			IssuesFixed++;
		}

		// Check for blend weight mismatch
		if (FMath::Abs(ActualWeight - ExpectedWeight) > 0.01f)
		{
			IssuesFound++;

			if (bLogStateChanges)
			{
				SafeLog(FString::Printf(TEXT("Blend weight mismatch for %s: Expected=%.2f, Actual=%.2f"),
				                        *BoneName.ToString(), ExpectedWeight, ActualWeight));
			}

			// Fix blend weight
			Body->PhysicsBlendWeight = ExpectedWeight;
			IssuesFixed++;
		}

		// Verify body validity
		if (!Body->IsValidBodyInstance() && bShouldSimulate)
		{
			IssuesFound++;
			SafeLog(FString::Printf(TEXT("Invalid body instance for simulating bone %s"),
			                        *BoneName.ToString()), true);

			// Try to fix it
			FixBodyPhysicsState(BoneName, Body);

			if (Body->IsValidBodyInstance())
			{
				IssuesFixed++;
			}
		}
	}

	// Log summary if issues were found
	if (IssuesFound > 0)
	{
		SafeLog(FString::Printf(TEXT("Physics verification: Found %d issues, fixed %d"),
		                        IssuesFound, IssuesFixed));

		// Force a bone transform refresh if we fixed anything
		if (IssuesFixed > 0 && bAutoFixStaleBodies)
		{
			SkeletalMesh->RefreshBoneTransforms();
			SkeletalMesh->UpdateKinematicBonesToAnim(
				SkeletalMesh->GetEditableComponentSpaceTransforms(),
				ETeleportType::None,
				true
			);
		}
	}
}


void UOHPACManager::EnableBonePhysicsSimple(
	FName BoneName,
	EOHPhysicsStrength Strength,
	bool bIncludeChain)
{
	FPhysicalAnimationData Profile = GetProfileForStrength(Strength);
	SetBonePhysicalAnimation(BoneName, true, Profile, bIncludeChain, true, true);
}

void UOHPACManager::DisableBonePhysicsSimple(
	FName BoneName,
	bool bIncludeChain)
{
	SetBonePhysicalAnimation(BoneName, false, FPhysicalAnimationData(), bIncludeChain, true, true);
}

// 2H. Updated SetCategorySimulation function
void UOHPACManager::SetCategorySimulation(
	EOHBoneCategory Category,
	bool bEnable,
	const FPhysicalAnimationData& Profile,
	bool bEnableCollision)
{
	TArray<FName> CategoryBones = GetBonesInCategory(Category);

	SafeLog(FString::Printf(TEXT("Setting simulation for category %s (%d bones): %s"),
	                        *UEnum::GetValueAsString(Category),
	                        CategoryBones.Num(),
	                        bEnable ? TEXT("Enable") : TEXT("Disable")));

	// Use blend system for smooth transitions
	FOHPhysicsBlendParams BlendParams;
	BlendParams.Profile = Profile;
	BlendParams.TargetAlpha = bEnable ? 1.0f : 0.0f;
	BlendParams.BlendInDuration = 0.3f;
	BlendParams.HoldDuration = bEnable ? 0.25f : 0.0f;
	BlendParams.BlendOutDuration = 0.5f;
	BlendParams.bEnableCollision = bEnableCollision;
	BlendParams.bAffectChain = false;
	BlendParams.bIsPermanent = false;

	for (const FName& BoneName : CategoryBones)
	{
		if (SimulatableBones.Contains(BoneName))
		{
			SetPhysicsBlend(BoneName, bEnable, BlendParams);
		}
	}
}

// ==== DIAGNOSTIC FUNCTIONS ====

bool UOHPACManager::VerifyComponentSetup(FString& OutReport)
{
	bool bIsValid = true;
	OutReport = TEXT("PAC Manager Component Verification Report:\n");

	// Check skeletal mesh
	if (!SkeletalMesh)
	{
		OutReport += TEXT("❌ Skeletal Mesh: Missing\n");
		bIsValid = false;
	}
	else
	{
		OutReport += TEXT("✅ Skeletal Mesh: Found\n");
		OutReport += FString::Printf(TEXT("  - Tick Enabled: %s\n"),
		                             SkeletalMesh->PrimaryComponentTick.IsTickFunctionEnabled()
			                             ? TEXT("Yes")
			                             : TEXT("No"));
		OutReport += FString::Printf(TEXT("  - Blend Physics: %s\n"),
		                             SkeletalMesh->bBlendPhysics ? TEXT("Yes") : TEXT("No"));
	}

	// Check PAC
	if (!PhysicalAnimationComponent)
	{
		OutReport += TEXT("❌ Physical Animation Component: Missing\n");
		bIsValid = false;
	}
	else
	{
		OutReport += TEXT("✅ Physical Animation Component: Found\n");
	}

	// Check physics asset
	if (!CachedPhysicsAsset)
	{
		OutReport += TEXT("❌ Physics Asset: Missing\n");
		bIsValid = false;
	}
	else
	{
		OutReport += TEXT("✅ Physics Asset: Found\n");
		OutReport += FString::Printf(TEXT("  - Body Count: %d\n"),
		                             CachedPhysicsAsset->SkeletalBodySetups.Num());
	}

	// Check simulatable bones
	OutReport += FString::Printf(TEXT("\nSimulatable Bones: %d\n"), SimulatableBones.Num());
	OutReport += FString::Printf(TEXT("Active Blends: %d\n"), GetTotalActiveBlendCount());

	UE_LOG(LogPACManager, Warning, TEXT("%s"), *OutReport);
	return bIsValid;
}

void UOHPACManager::ForceRefreshPhysics()
{
	SafeLog(TEXT("Force refreshing physics state"));

	RefreshBodyInstances();
	EnsurePhysicsTickEnabled();

	// Re-apply all active blends
	for (auto& Pair : ActiveBlends)
	{
		for (FOHActiveBlend& Blend : Pair.Value)
		{
			if (!Blend.IsComplete())
			{
				UpdateBlendState(Blend, 0.0f);
			}
		}
	}
}

int32 UOHPACManager::FixStaleBodies()
{
	int32 FixedCount = 0;

	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body || !Body->IsValidBodyInstance())
		{
			SafeLog(FString::Printf(TEXT("Fixing stale body for bone %s"), *BoneName.ToString()));

			// Clear from cache
			//BodyInstanceCache.Remove(BoneName);

			// Try to get fresh instance
			Body = SkeletalMesh->GetBodyInstance(BoneName);
			if (Body)
			{
				//BodyInstanceCache.Add(BoneName, Body);
				FixedCount++;
			}
		}
	}

	if (FixedCount > 0)
	{
		RefreshBodyInstances();
	}

	return FixedCount;
}

// 2C. New ForceDisablePhysics helper function
void UOHPACManager::ForceDisablePhysics(FName BoneName)
{
	ClearPhysicalAnimationProfile(BoneName);

	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		Body->SetInstanceSimulatePhysics(false);
		Body->PhysicsBlendWeight = 0.0f;
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		Body->PutInstanceToSleep();
	}

	UpdateSkeletalMeshBounds();
	OnBoneStoppedSimulating.Broadcast(BoneName);
}

void UOHPACManager::EnsurePhysicsTickEnabled()
{
	if (!SkeletalMesh)
	{
		return;
	}

	// CRITICAL: Enable blend physics for visual blending
	SkeletalMesh->bBlendPhysics = true;

	// Ensure component is set to tick
	SkeletalMesh->PrimaryComponentTick.SetTickFunctionEnable(true);
	SkeletalMesh->bAutoActivate = true;

	// Set update mode to blend kinematic and simulated bones
	SkeletalMesh->KinematicBonesUpdateType = EKinematicBonesUpdateToPhysics::SkipSimulatingBones;

	// Ensure we're NOT simulating the entire mesh
	SkeletalMesh->SetSimulatePhysics(false);

	// Set tick groups
	SkeletalMesh->PrimaryComponentTick.TickGroup = TG_PrePhysics;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	// Force refresh
	SkeletalMesh->RefreshBoneTransforms();
	SkeletalMesh->RecreatePhysicsState();

	SafeLog(TEXT("Physics blend settings applied:"));
	SafeLog(FString::Printf(TEXT("  - Blend Physics: %s"),
	                        SkeletalMesh->bBlendPhysics ? TEXT("Yes") : TEXT("No")));
	SafeLog(FString::Printf(TEXT("  - Component Simulating: %s"),
	                        SkeletalMesh->IsSimulatingPhysics() ? TEXT("Yes") : TEXT("No")));
}

// 1. Fix for mesh culling during physics blending
void UOHPACManager::UpdateSkeletalMeshBounds()
{
	if (!SkeletalMesh)
	{
		return;
	}

	// Force bounds update
	SkeletalMesh->UpdateBounds();
	SkeletalMesh->MarkRenderTransformDirty();
	SkeletalMesh->MarkRenderDynamicDataDirty();
	SkeletalMesh->MarkRenderStateDirty();
}

void UOHPACManager::SetupPhysicsBounds()
{
	if (!SkeletalMesh)
	{
		return;
	}

	// Option 1: Disable fixed bounds entirely (most reliable but less performant)
	SkeletalMesh->bComponentUseFixedSkelBounds = false;

	// Ensure updates happen even when not visible
	SkeletalMesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;

	// Force an initial bounds update
	SkeletalMesh->UpdateBounds();
}

// Practical approach to toggle bounds handling based on physics state
void UOHPACManager::OnPhysicsStateChanged(bool bPhysicsActive)
{
	if (!SkeletalMesh)
	{
		return;
	}

	if (bPhysicsActive)
	{
		// Disable fixed bounds when physics is active
		SkeletalMesh->bComponentUseFixedSkelBounds = false;
		SafeLog(TEXT("Physics activated - disabled fixed bounds"));
	}
	else
	{
		// Re-enable fixed bounds when physics is inactive for performance
		SkeletalMesh->bComponentUseFixedSkelBounds = true;
		SkeletalMesh->UpdateBounds(); // Update bounds one last time
		SafeLog(TEXT("Physics deactivated - re-enabled fixed bounds"));
	}
}

// Add to OHPACManager.cpp
float UOHPACManager::CalculateCombinedBlendWeight(const TArray<FOHActiveBlend>& Blends,
                                                  FPhysicalAnimationData& OutCombinedProfile)
{
	if (Blends.Num() == 0)
	{
		OutCombinedProfile = ZeroProfile;
		return 0.0f;
	}

	// Single blend - no combination needed
	if (Blends.Num() == 1)
	{
		const FOHActiveBlend& Blend = Blends[0];
		OutCombinedProfile = UOHSkeletalPhysicsUtils::LerpProfiles(Blend.PhysicsProfile_Start,
		                                                           Blend.PhysicsProfile_Target,
		                                                           Blend.PhysicsBlendWeight_Current);
		return Blend.PhysicsBlendWeight_Current;
	}

	// Multiple blends - combine weights and profiles
	float TotalWeight = 0.0f;
	float WeightSum = 0.0f;

	// First pass - calculate weights and sum for normalization
	TArray<float> NormalizedWeights;
	for (const FOHActiveBlend& Blend : Blends)
	{
		if (Blend.IsComplete() || Blend.IsPaused())
		{
			NormalizedWeights.Add(0.0f);
			continue;
		}

		float Weight = Blend.PhysicsBlendWeight_Current;

		// Apply easing for smoother transitions
		if (Blend.CurrentPhase == EOHBlendPhase::BlendIn)
		{
			Weight = FMath::SmoothStep(0.0f, 1.0f, Weight);
		}
		else if (Blend.CurrentPhase == EOHBlendPhase::BlendOut)
		{
			Weight = FMath::SmoothStep(1.0f, 0.0f, 1.0f - Weight);
		}

		NormalizedWeights.Add(Weight);
		WeightSum += Weight;
	}

	// Normalize weights
	if (WeightSum > 0.0f)
	{
		for (float& Weight : NormalizedWeights)
		{
			Weight /= WeightSum;
		}
	}

	// Combine profiles with normalized weights
	OutCombinedProfile = ZeroProfile;
	for (int32 i = 0; i < Blends.Num(); i++)
	{
		if (NormalizedWeights[i] > 0.0f)
		{
			const FOHActiveBlend& Blend = Blends[i];
			FPhysicalAnimationData BlendProfile = UOHSkeletalPhysicsUtils::LerpProfiles(
				Blend.PhysicsProfile_Start,
				Blend.PhysicsProfile_Target,
				Blend.PhysicsBlendWeight_Current
			);

			// Accumulate weighted profile
			OutCombinedProfile.OrientationStrength += BlendProfile.OrientationStrength * NormalizedWeights[i];
			OutCombinedProfile.AngularVelocityStrength += BlendProfile.AngularVelocityStrength * NormalizedWeights[i];
			OutCombinedProfile.PositionStrength += BlendProfile.PositionStrength * NormalizedWeights[i];
			OutCombinedProfile.VelocityStrength += BlendProfile.VelocityStrength * NormalizedWeights[i];

			TotalWeight += Blend.PhysicsBlendWeight_Current * NormalizedWeights[i];
		}
	}

	// Use additive blending for multiple simultaneous impacts
	return FMath::Min(TotalWeight, 1.0f);
}

#pragma endregion


// ============================================================================
// UNIFIED PHYSICS BLEND CONTROL
// ============================================================================
#pragma region UNIFIED PHYSICS BLEND
void UOHPACManager::SetPhysicsBlend(FName BoneName, bool bEnabled, const FOHPhysicsBlendParams& Params)
{
	if (!ValidatePhysicsSimulation())
	{
		return;
	}

	if (!GetWorld() || !SkeletalMesh)
	{
		return;
	}

	float WorldTime = GetWorld()->GetTimeSeconds();

	if (bEnabled)
	{
		TArray<FName> ChainBones;
		if (Params.bAffectChain)
		{
			ChainBones = GetBoneChain(BoneName, -1);
			if (Params.FilterBones.Num() > 0)
			{
				ChainBones = ChainBones.FilterByPredicate([&Params](const FName& Bone)
				{
					return Params.FilterBones.Contains(Bone);
				});
			}
		}
		else
		{
			ChainBones.Add(BoneName);
		}

		// Process each bone in the chain
		for (const FName& ChainBone : ChainBones)
		{
			// Get or create baseline for this bone
			FBoneBaseline& Baseline = BoneBaselines.FindOrAdd(ChainBone);

			// Create the new blend
			FOHActiveBlend NewBlend;
			NewBlend.BoneName = ChainBone;
			NewBlend.ReactionTag = Params.ReactionTag;
			NewBlend.BlendInDuration = Params.BlendInDuration;
			NewBlend.HoldDuration = Params.HoldDuration;
			NewBlend.BlendOutDuration = Params.BlendOutDuration;
			NewBlend.CurrentPhase = EOHBlendPhase::BlendIn;

			// Set return behavior based on permanence
			NewBlend.bIsPermanent = Params.bIsPermanent;
			NewBlend.bReturnToBaseline = !Params.bIsPermanent && Baseline.bHasBaseline;

			// Set target weight
			NewBlend.PhysicsBlendWeight_Target = FMath::Max(Params.TargetAlpha, 0.001f);

			// Handle profile selection
			bool bHasProvidedProfile = (Params.Profile.OrientationStrength > 0.0f ||
				Params.Profile.PositionStrength > 0.0f);

			if (bHasProvidedProfile)
			{
				// Use provided profile
				NewBlend.PhysicsProfile_Target = Params.Profile;
			}
			else if (Baseline.bHasBaseline)
			{
				// No profile provided, use baseline
				NewBlend.PhysicsProfile_Target = Baseline.DefaultProfile;
			}
			else
			{
				// No profile provided and no baseline, get current
				NewBlend.PhysicsProfile_Target = GetCurrentPhysicalAnimationProfile(ChainBone);
			}

			// Get current state as start point
			if (FBodyInstance* Body = GetBodyInstanceDirect(ChainBone))
			{
				NewBlend.PhysicsBlendWeight_Start = Body->PhysicsBlendWeight;
				NewBlend.PhysicsBlendWeight_Current = Body->PhysicsBlendWeight;
				NewBlend.PhysicsProfile_Start = GetCurrentPhysicalAnimationProfile(ChainBone);
			}
			else
			{
				NewBlend.PhysicsBlendWeight_Start = 0.0f;
				NewBlend.PhysicsBlendWeight_Current = 0.0f;
				NewBlend.PhysicsProfile_Start = ZeroProfile;
			}

			// Update baseline if this is permanent
			if (Params.bIsPermanent)
			{
				Baseline.DefaultWeight = NewBlend.PhysicsBlendWeight_Target;
				Baseline.DefaultProfile = NewBlend.PhysicsProfile_Target;
				Baseline.bHasBaseline = true;

				SafeLog(FString::Printf(TEXT("Updated baseline for %s: weight=%.3f"),
				                        *ChainBone.ToString(), Baseline.DefaultWeight));
			}

			// Initialize timing
			NewBlend.InitializeTiming(WorldTime);

			// Propagation settings
			NewBlend.bUsedNativePropagation = Params.bEnableNativeForcePropagation && Params.bAffectChain;
			NewBlend.bIsRootOfPropagation = (ChainBone == BoneName) && NewBlend.bUsedNativePropagation;
			NewBlend.PropagationRootBone = NewBlend.bIsRootOfPropagation ? BoneName : BoneName;

			// Clear existing blends and add new one (one blend per bone policy)
			TArray<FOHActiveBlend>& Blends = ActiveBlends.FindOrAdd(ChainBone);
			Blends.Empty();
			Blends.Add(NewBlend);

			// Ensure physics is enabled if needed
			if (!IsBoneSimulating(ChainBone) && NewBlend.PhysicsBlendWeight_Target > 0.001f)
			{
				SetBonePhysicalAnimation_Internal(
					ChainBone,
					true,
					NewBlend.PhysicsProfile_Target,
					false,
					Params.FilterBones,
					Params.bEnableCollision,
					true,
					Params.bEnableNativeForcePropagation && (ChainBone == BoneName)
				);
			}
		}

		SafeLog(FString::Printf(TEXT("Set physics blend for %s: Target=%.3f, Permanent=%s, ChainCount=%d"),
		                        *BoneName.ToString(),
		                        Params.TargetAlpha,
		                        Params.bIsPermanent ? TEXT("Yes") : TEXT("No"),
		                        ChainBones.Num()));
	}
	else
	{
		// DISABLE BLEND
		TArray<FName> ChainBones = GetBoneChain(BoneName, -1);
		if (Params.FilterBones.Num() > 0)
		{
			ChainBones = ChainBones.FilterByPredicate([&Params](const FName& Bone)
			{
				return Params.FilterBones.Contains(Bone);
			});
		}

		for (const FName& ChainBone : ChainBones)
		{
			// Get current weight
			float CurrentWeight = 0.0f;
			if (FBodyInstance* Body = GetBodyInstanceDirect(ChainBone))
			{
				CurrentWeight = Body->PhysicsBlendWeight;
			}

			if (CurrentWeight > 0.005f)
			{
				// Create blend-out
				FOHActiveBlend BlendOut;
				BlendOut.BoneName = ChainBone;
				BlendOut.PhysicsBlendWeight_Start = CurrentWeight;
				BlendOut.PhysicsBlendWeight_Current = CurrentWeight;
				BlendOut.PhysicsBlendWeight_Target = 0.0f;
				BlendOut.CurrentPhase = EOHBlendPhase::BlendOut;
				BlendOut.BlendOutDuration = Params.BlendOutDuration > 0.0f ? Params.BlendOutDuration : 0.3f;
				BlendOut.bIsBlendOutOnly = true;
				BlendOut.bReturnToBaseline = false;
				BlendOut.ReactionTag = Params.ReactionTag.IsNone() ? TEXT("ManualDisable") : Params.ReactionTag;
				BlendOut.PhysicsProfile_Target = GetCurrentPhysicalAnimationProfile(ChainBone);
				BlendOut.PhysicsProfile_Start = BlendOut.PhysicsProfile_Target;

				BlendOut.InitializeTiming(WorldTime);

				// Clear baseline when fully disabling
				BoneBaselines.Remove(ChainBone);

				// Replace any existing blends
				TArray<FOHActiveBlend>& Blends = ActiveBlends.FindOrAdd(ChainBone);
				Blends.Empty();
				Blends.Add(BlendOut);

				SafeLog(FString::Printf(TEXT("Started disable blend for %s from weight %.3f"),
				                        *ChainBone.ToString(), CurrentWeight));
			}
			else
			{
				// Already near zero, just clean up
				ForceDisablePhysics(ChainBone);
				ActiveBlends.Remove(ChainBone);
				BoneBaselines.Remove(ChainBone);
			}
		}
	}
}

void UOHPACManager::ProcessActiveBlends(float DeltaTime)
{
	TArray<FName> BonesToRemove;
	TSet<FName> RootsToCleanup;

	// First pass: Update all blend states
	for (auto& Pair : ActiveBlends)
	{
		FName BoneName = Pair.Key;
		TArray<FOHActiveBlend>& Blends = Pair.Value;

		for (int32 i = Blends.Num() - 1; i >= 0; i--)
		{
			FOHActiveBlend& Blend = Blends[i];
			UpdateBlendState(Blend, DeltaTime);

			if (Blend.IsComplete() || Blend.bForceComplete)
			{
				// Check if we should return to baseline
				if (Blend.bReturnToBaseline && !Blend.bIsBlendOutOnly)
				{
					FBoneBaseline* Baseline = BoneBaselines.Find(BoneName);
					if (Baseline && Baseline->bHasBaseline && Baseline->DefaultWeight > 0.001f)
					{
						// Create return blend
						FOHActiveBlend ReturnBlend;
						ReturnBlend.BoneName = BoneName;
						ReturnBlend.PhysicsBlendWeight_Target = Baseline->DefaultWeight;
						ReturnBlend.PhysicsProfile_Target = Baseline->DefaultProfile;
						ReturnBlend.PhysicsProfile_Start = GetCurrentPhysicalAnimationProfile(BoneName);
						ReturnBlend.BlendInDuration = 0.3f; // Default return time
						ReturnBlend.CurrentPhase = EOHBlendPhase::BlendIn;
						ReturnBlend.bReturnToBaseline = false; // Don't loop!
						ReturnBlend.bIsPermanent = false;
						ReturnBlend.ReactionTag = TEXT("BaselineReturn");

						if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
						{
							ReturnBlend.PhysicsBlendWeight_Start = Body->PhysicsBlendWeight;
							ReturnBlend.PhysicsBlendWeight_Current = Body->PhysicsBlendWeight;
						}

						ReturnBlend.InitializeTiming(GetWorld()->GetTimeSeconds());

						// Copy propagation settings from original blend
						ReturnBlend.bUsedNativePropagation = Blend.bUsedNativePropagation;
						ReturnBlend.bIsRootOfPropagation = Blend.bIsRootOfPropagation;
						ReturnBlend.PropagationRootBone = Blend.PropagationRootBone;

						// Replace completed blend with return blend
						Blends[i] = ReturnBlend;

						SafeLog(FString::Printf(TEXT("Returning %s to baseline weight %.3f"),
						                        *BoneName.ToString(), Baseline->DefaultWeight));

						continue; // Don't remove this blend
					}
				}

				// Handle propagation cleanup
				if (Blend.bIsRootOfPropagation)
				{
					RootsToCleanup.Add(Blend.BoneName);
				}

				OnBlendCompleted.Broadcast(BoneName, Blend.ReactionTag);
				Blends.RemoveAt(i);
			}
		}

		if (Blends.Num() == 0)
		{
			BonesToRemove.Add(BoneName);
		}
	}

	// Second pass: Apply blends
	for (auto& Pair : ActiveBlends)
	{
		FName BoneName = Pair.Key;
		TArray<FOHActiveBlend>& Blends = Pair.Value;

		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			if (Blends.Num() > 0)
			{
				// With one-blend-per-bone policy, this is simpler
				FOHActiveBlend& Blend = Blends[0];

				// Apply the blend
				ApplyBlendToBody(BoneName, Blends, Blend.PhysicsBlendWeight_Current,
				                 UOHSkeletalPhysicsUtils::LerpProfiles(Blend.PhysicsProfile_Start,
				                                                       Blend.PhysicsProfile_Target,
				                                                       Blend.GetBlendProgress()));
			}
			else if (Body->IsInstanceSimulatingPhysics())
			{
				// No active blends but still simulating - disable
				SafeLog(FString::Printf(TEXT("Disabling physics for %s (no active blends)"),
				                        *BoneName.ToString()));

				ClearPhysicalAnimationProfile(BoneName);
				Body->SetInstanceSimulatePhysics(false);
				Body->PhysicsBlendWeight = 0.0f;
				Body->SetLinearVelocity(FVector::ZeroVector, false);
				Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
				Body->PutInstanceToSleep();

				UpdateSkeletalMeshBounds();
				OnBoneStoppedSimulating.Broadcast(BoneName);
			}
		}
	}

	// Clean up propagation chains
	for (const FName& RootBone : RootsToCleanup)
	{
		for (auto& Pair : ActiveBlends)
		{
			Pair.Value.RemoveAll([RootBone](const FOHActiveBlend& Blend)
			{
				return Blend.PropagationRootBone == RootBone && !Blend.bIsRootOfPropagation;
			});
		}
	}

	// Remove empty entries and clear baselines for disabled bones
	for (const FName& BoneName : BonesToRemove)
	{
		ActiveBlends.Remove(BoneName);

		// Only clear baseline if weight is actually 0
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			if (Body->PhysicsBlendWeight < 0.001f)
			{
				BoneBaselines.Remove(BoneName);
			}
		}
	}
}


void UOHPACManager::UpdateBlendState(FOHActiveBlend& Blend, float DeltaTime)
{
	if (Blend.IsPaused() || Blend.CurrentPhase == EOHBlendPhase::Inactive || Blend.IsComplete())
	{
		return;
	}

	float WorldTime = GetWorld()->GetTimeSeconds();

	if (Blend.StartTime <= 0.0f)
	{
		Blend.InitializeTiming(WorldTime);

		if (FBodyInstance* Body = GetBodyInstanceDirect(Blend.BoneName))
		{
			Blend.PhysicsBlendWeight_Start = Body->PhysicsBlendWeight;
			Blend.PhysicsBlendWeight_Current = Blend.PhysicsBlendWeight_Start;
		}
		SafeLog(FString::Printf(TEXT("Blend initialized for %s: StartWeight=%.3f, TargetWeight=%.3f, IsPermanent=%s"),
		                        *Blend.BoneName.ToString(),
		                        Blend.PhysicsBlendWeight_Start,
		                        Blend.PhysicsBlendWeight_Target,
		                        Blend.bIsPermanent ? TEXT("YES") : TEXT("NO")));
	}

	Blend.ElapsedTime = WorldTime - Blend.StartTime;
	float NewWeight = Blend.CalculateWeightAtTime(WorldTime);

	if (Blend.bIsBlendOutOnly)
	{
		if (Blend.ElapsedTime >= Blend.BlendOutDuration || NewWeight < Blend.CompletionThreshold)
		{
			Blend.CurrentPhase = EOHBlendPhase::Inactive;
			Blend.PhysicsBlendWeight_Current = 0.0f;
			Blend.bForceComplete = true;
		}
		else
		{
			Blend.PhysicsBlendWeight_Current = NewWeight;
		}
	}
	else if (!Blend.bIsPermanent)
	{
		float BlendInEnd = Blend.BlendInDuration;
		float HoldEnd = BlendInEnd + Blend.HoldDuration;

		if (Blend.ElapsedTime < BlendInEnd)
		{
			Blend.CurrentPhase = EOHBlendPhase::BlendIn;
		}
		else if (Blend.ElapsedTime < HoldEnd)
		{
			Blend.CurrentPhase = EOHBlendPhase::Hold;
		}
		else if (Blend.ElapsedTime < Blend.TotalDuration)
		{
			Blend.CurrentPhase = EOHBlendPhase::BlendOut;
		}
		else
		{
			Blend.CurrentPhase = EOHBlendPhase::Inactive;
			Blend.bForceComplete = true;
		}
		Blend.PhysicsBlendWeight_Current = NewWeight;
	}
	else // Permanent blend
	{
		if (Blend.ElapsedTime >= Blend.BlendInDuration)
		{
			Blend.CurrentPhase = EOHBlendPhase::Permanent;
			Blend.PhysicsBlendWeight_Current = Blend.PhysicsBlendWeight_Target;
		}
		else
		{
			Blend.CurrentPhase = EOHBlendPhase::BlendIn;
			Blend.PhysicsBlendWeight_Current = NewWeight;
		}

		// Never mark permanent blends as complete
		Blend.bForceComplete = false;
	}
}

void UOHPACManager::ApplyBlendToBody(FName BoneName, const TArray<FOHActiveBlend>& Blends, float CombinedWeight,
                                     const FPhysicalAnimationData& CombinedProfile)
{
	if (!PhysicalAnimationComponent || BoneName == NAME_None)
	{
		return;
	}

	FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
	if (!Body)
	{
		return;
	}

	bool bIsSimulating = Body->IsInstanceSimulatingPhysics();
	bool bShouldSimulate = CombinedWeight > 0.001f;

	if (!bIsSimulating && bShouldSimulate)
	{
		// Determine propagation settings from blends
		bool bIsRootOfPropagation = false;
		bool bUsedNativePropagation = false;

		for (const FOHActiveBlend& Blend : Blends)
		{
			if (Blend.bIsRootOfPropagation)
			{
				bIsRootOfPropagation = true;
				bUsedNativePropagation = Blend.bUsedNativePropagation;
				break;
			}
		}

		// Apply physics profile as appropriate
		if (bIsRootOfPropagation)
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(
				BoneName,
				CombinedProfile
			);
			SafeLog(FString::Printf(TEXT("Applied profile BELOW %s (native propagation)"),
			                        *BoneName.ToString()));
		}
		else if (!bUsedNativePropagation)
		{
			PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(
				BoneName,
				CombinedProfile
			);
			SafeLog(FString::Printf(TEXT("Applied profile to %s (direct)"),
			                        *BoneName.ToString()));
		}
		// else: child in propagation chain

		// Enable collision
		Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		// Start simulating
		Body->SetInstanceSimulatePhysics(true);
		Body->WakeInstance();

		// Ensure blend physics is enabled
		if (!SkeletalMesh->bBlendPhysics)
		{
			SkeletalMesh->bBlendPhysics = true;
			SkeletalMesh->RecreatePhysicsState();
		}

		OnBoneStartedSimulating.Broadcast(BoneName);
	}

	if (bIsSimulating)
	{
		// Set the blend weight
		Body->PhysicsBlendWeight = CombinedWeight;

		// Check if any blend needs profile updates
		bool bNeedsProfileUpdate = false;
		bool bHasNonPropagatedBlend = false;

		for (const FOHActiveBlend& Blend : Blends)
		{
			if (!Blend.bUsedNativePropagation)
			{
				bHasNonPropagatedBlend = true;
				break;
			}
		}

		if (bHasNonPropagatedBlend)
		{
			// Scale the combined profile by the current weight
			FPhysicalAnimationData ScaledProfile = CombinedProfile;
			float ProfileScale = CombinedWeight;

			ScaledProfile.OrientationStrength *= ProfileScale;
			ScaledProfile.AngularVelocityStrength *= ProfileScale;
			ScaledProfile.PositionStrength *= ProfileScale;
			ScaledProfile.VelocityStrength *= ProfileScale;

			PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ScaledProfile);
			bNeedsProfileUpdate = true;
		}
		// For propagation chains, the profile is managed by ApplyPhysicalAnimationProfileBelow

		Body->UpdatePhysicsFilterData();

		if (bVerboseLogging)
		{
			SafeLog(FString::Printf(TEXT("Weight %.3f on %s (Updated Profile: %s)"),
			                        CombinedWeight,
			                        *BoneName.ToString(),
			                        bNeedsProfileUpdate ? TEXT("YES") : TEXT("NO")));
		}
	}
	else if (bIsSimulating && !bShouldSimulate)
	{
		// Disable physics
		ClearPhysicalAnimationProfile(BoneName);
		Body->SetInstanceSimulatePhysics(false);
		Body->PhysicsBlendWeight = 0.0f;
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		Body->PutInstanceToSleep();

		OnBoneStoppedSimulating.Broadcast(BoneName);
	}
}


void UOHPACManager::CleanupStaleBlends()
{
	TArray<FName> BonesToRemove;
	constexpr float StuckBlendTimeout = 2.0f;

	for (auto& Pair : ActiveBlends)
	{
		FName BoneName = Pair.Key;
		TArray<FOHActiveBlend>& Blends = Pair.Value;

		// With one-blend-per-bone, this is simpler
		if (Blends.Num() > 0)
		{
			FOHActiveBlend& Blend = Blends[0];

			// Check if blend is stuck
			bool bShouldRemove = false;

			if (Blend.IsComplete())
			{
				bShouldRemove = true;
			}
			else if (!Blend.bIsPermanent)
			{
				float WorldTime = GetWorld()->GetTimeSeconds();

				if (Blend.IsCompleteAtTime(WorldTime))
				{
					bShouldRemove = true;
				}
				else if (Blend.CurrentPhase == EOHBlendPhase::BlendOut &&
					FMath::IsNearlyZero(Blend.PhysicsBlendWeight_Current, 0.001f))
				{
					bShouldRemove = true;
				}
				else if (Blend.ElapsedTime > FMath::Max(Blend.TotalDuration * 2.0f, StuckBlendTimeout))
				{
					SafeLog(FString::Printf(TEXT("Removing stuck blend for %s (elapsed: %.2f)"),
					                        *BoneName.ToString(), Blend.ElapsedTime));
					bShouldRemove = true;
				}
			}

			if (bShouldRemove)
			{
				Blends.Empty();
				BonesToRemove.Add(BoneName);
			}
		}
		else
		{
			BonesToRemove.Add(BoneName);
		}
	}

	// Clean up empty entries
	for (const FName& BoneName : BonesToRemove)
	{
		ActiveBlends.Remove(BoneName);

		// Ensure physics is off
		if (IsBoneSimulating(BoneName))
		{
			ForceDisablePhysics(BoneName);
		}

		// Clear baseline if weight is 0
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
		{
			if (Body->PhysicsBlendWeight < 0.001f)
			{
				BoneBaselines.Remove(BoneName);
			}
		}
	}
}


void UOHPACManager::StopAllBlends()
{
	for (auto& Pair : ActiveBlends)
	{
		for (FOHActiveBlend& Blend : Pair.Value)
		{
			if (Blend.CurrentPhase != EOHBlendPhase::BlendOut && !Blend.IsComplete())
			{
				Blend.CurrentPhase = EOHBlendPhase::BlendOut;
				Blend.ElapsedTime = 0.0f;
			}
		}
	}
}


#if 0
void UOHPACManager::StopBlend(FName BoneName, FName Tag)
{
    TArray<FOHActiveBlend>* ExistingBlends = ActiveBlends.Find(BoneName);
    if (!ExistingBlends)
    {
        return;
    }

    // Mark blends for removal
    TArray<int32> BlendsToRemove;
    
    for (int32 i = 0; i < ExistingBlends->Num(); i++)
    {
        FOHActiveBlend& Blend = (*ExistingBlends)[i];
        
        if (Tag == NAME_None || Blend.ReactionTag == Tag)
        {
            if (!Blend.IsComplete())
            {
                // For non-permanent blends, start blend out
                if (!Blend.bIsPermanent)
                {
                    Blend.CurrentPhase = EOHBlendPhase::BlendOut;
                    Blend.ElapsedTime = 0.0f;
                }
                else
                {
                    // For permanent blends, force completion
                    Blend.CurrentPhase = EOHBlendPhase::Inactive;
                    BlendsToRemove.Add(i);
                }
            }
        }
    }
    
    // Remove completed blends
    for (int32 i = BlendsToRemove.Num() - 1; i >= 0; i--)
    {
        ExistingBlends->RemoveAt(BlendsToRemove[i]);
    }
    
    // If no blends remain, ensure physics is disabled
    if (ExistingBlends->Num() == 0)
    {
        if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
        {
            if (Body->IsInstanceSimulatingPhysics())
            {
                ClearPhysicalAnimationProfile(BoneName);
                Body->SetInstanceSimulatePhysics(false);
                Body->PhysicsBlendWeight = 0.0f;
                Body->SetLinearVelocity(FVector::ZeroVector, false);
                Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
                Body->PutInstanceToSleep();
                
                OnBoneStoppedSimulating.Broadcast(BoneName);
            }
        }
        
        ActiveBlends.Remove(BoneName);
    }
}

void UOHPACManager::PauseBlend(FName BoneName, FName ReactionTag)
{
   TArray<FOHActiveBlend>* Blends = ActiveBlends.Find(BoneName);
   if (!Blends)
   {
       return;
   }

   for (FOHActiveBlend& Blend : *Blends)
   {
       if (ReactionTag == NAME_None || Blend.ReactionTag == ReactionTag)
       {
           Blend.PauseCount++;
       }
   }
}

void UOHPACManager::ResumeBlend(FName BoneName, FName ReactionTag)
{
   TArray<FOHActiveBlend>* Blends = ActiveBlends.Find(BoneName);
   if (!Blends)
   {
       return;
   }

   for (FOHActiveBlend& Blend : *Blends)
   {
       if (ReactionTag == NAME_None || Blend.ReactionTag == ReactionTag)
       {
           Blend.PauseCount = FMath::Max(0, Blend.PauseCount - 1);
       }
   }
}
#endif

#pragma endregion

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================
#pragma region UPDATE FUNCTIONS

void UOHPACManager::UpdateMotionTracking(float DeltaTime)
{
	if (!SkeletalMesh)
	{
		return;
	}

	// Get reference velocity from root bone or component
	FVector ReferenceVelocity = GetReferenceVelocity();
	FTransform ComponentTransform = SkeletalMesh->GetComponentTransform();
	float CurrentTime = GetWorld()->GetTimeSeconds();

	// Update motion data for each simulatable bone
	for (const FName& BoneName : SimulatableBones)
	{
		FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
		if (!MotionData)
		{
			// Create new motion data if it doesn't exist
			FOHBoneMotionData NewMotionData;
			NewMotionData.Initialize(BoneName);
			BoneMotionMap.Add(BoneName, NewMotionData);
			MotionData = BoneMotionMap.Find(BoneName);

			SafeLog(FString::Printf(TEXT("Created motion data for %s"), *BoneName.ToString()));
		}

		// Get current bone transform
		int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
		FTransform BoneTransform = SkeletalMesh->GetBoneTransform(BoneIndex);

		// Get component velocity (from physics if simulating, otherwise calculated)
		FVector ComponentVelocity = FVector::ZeroVector;
		if (IsBoneSimulating(BoneName))
		{
			if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
			{
				ComponentVelocity = Body->GetUnrealWorldVelocity();
			}
		}
		else
		{
			// Calculate from position delta if not simulating
			ComponentVelocity = SkeletalMesh->GetPhysicsLinearVelocity();
		}

		// Add new motion sample with all required parameters
		MotionData->AddMotionSample(
			BoneTransform.GetLocation(), // WorldPos
			BoneTransform.GetRotation(), // WorldRot
			ComponentVelocity, // ComponentVelocity
			ReferenceVelocity, // ReferenceVelocity
			ComponentTransform, // ComponentTransform
			DeltaTime, // DeltaTime - IMPORTANT!
			CurrentTime // TimeStamp
		);
		//SafeLog(FString::Printf(TEXT("Updated motion sample for %s"), *BoneName.ToString()) );
	}
}

// Update constraint states to handle multiple constraints
void UOHPACManager::UpdateConstraintStates(float DeltaTime)
{
	for (auto& Pair : ConstraintLookupMap)
	{
		FOHConstraintLookup& Lookup = Pair.Value;

		// Update all constraints where this bone is child
		for (FOHConstraintData& Data : Lookup.AsChild)
		{
			if (Data.IsValid())
			{
				Data.UpdateStrain(SkeletalMesh, DeltaTime);
			}
		}

		// Update all constraints where this bone is parent
		for (FOHConstraintData& Data : Lookup.AsParent)
		{
			if (Data.IsValid())
			{
				Data.UpdateStrain(SkeletalMesh, DeltaTime);
			}
		}
	}
}

#pragma endregion

// ============================================================================
// PHYSICS INTERACTION
// ============================================================================
#pragma region PHYSICS INTERACTION


void UOHPACManager::WakePhysicsBody(FName BoneName)
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		Body->WakeInstance();
	}
}

void UOHPACManager::PutPhysicsBodyToSleep(FName BoneName)
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		Body->PutInstanceToSleep();
	}
}

void UOHPACManager::ClearPhysicsStateForBone(FName BoneName)
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		Body->SetLinearVelocity(FVector::ZeroVector, false);
		Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		Body->ClearForces();
		Body->ClearTorques();
	}
}

#pragma endregion

// ============================================================================
// BONE STATE QUERIES
// ============================================================================
#pragma region BONE STATE QUERIES

bool UOHPACManager::IsBoneSimulating(FName BoneName) const
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		return Body->IsInstanceSimulatingPhysics();
	}
	return false;
}

bool UOHPACManager::IsBoneDrivenByPhysicalAnimation(const FName& BoneName) const
{
	if (!PhysicalAnimationComponent)
	{
		return false;
	}

	FConstraintInstance* Constraint = FindPhysicalAnimationConstraint(BoneName);
	return Constraint && HasPhysicalAnimationDrives(Constraint);
}

float UOHPACManager::GetBonePhysicsBlendWeight(FName BoneName) const
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		return Body->PhysicsBlendWeight;
	}
	return 0.0f;
}

int32 UOHPACManager::GetActiveBlendCount(FName BoneName) const
{
	const TArray<FOHActiveBlend>* Blends = ActiveBlends.Find(BoneName);
	if (!Blends)
	{
		return 0;
	}

	int32 Count = 0;
	for (const FOHActiveBlend& Blend : *Blends)
	{
		if (!Blend.IsComplete())
		{
			Count++;
		}
	}
	return Count;
}

bool UOHPACManager::GetBoneBaseline(FName BoneName, float& OutWeight, FPhysicalAnimationData& OutProfile) const
{
	const FBoneBaseline* Baseline = BoneBaselines.Find(BoneName);
	if (Baseline && Baseline->bHasBaseline)
	{
		OutWeight = Baseline->DefaultWeight;
		OutProfile = Baseline->DefaultProfile;
		return true;
	}
	return false;
}

// Manually set baseline without creating a blend
void UOHPACManager::SetBoneBaseline(FName BoneName, float Weight, const FPhysicalAnimationData& Profile)
{
	FBoneBaseline& Baseline = BoneBaselines.FindOrAdd(BoneName);
	Baseline.DefaultWeight = Weight;
	Baseline.DefaultProfile = Profile;
	Baseline.bHasBaseline = true;

	SafeLog(FString::Printf(TEXT("Manually set baseline for %s: weight=%.3f"),
	                        *BoneName.ToString(), Weight));
}

// Clear baseline for a bone
void UOHPACManager::ClearBoneBaseline(FName BoneName)
{
	BoneBaselines.Remove(BoneName);
	SafeLog(FString::Printf(TEXT("Cleared baseline for %s"), *BoneName.ToString()));
}

int32 UOHPACManager::GetTotalActiveBlendCount() const
{
	int32 Total = 0;
	for (const auto& Pair : ActiveBlends)
	{
		for (const FOHActiveBlend& Blend : Pair.Value)
		{
			if (!Blend.IsComplete())
			{
				Total++;
			}
		}
	}
	return Total;
}

#pragma endregion

// ============================================================================
// CONSTRAINT DATA ACCESS
// ============================================================================
#pragma region CONSTRAINT DATA ACCESS

float UOHPACManager::GetConstraintStrain(FName BoneName) const
{
	// Get constraint data
	if (const FOHConstraintData* Data = ConstraintDataMap.Find(BoneName))
	{
		return Data ? Data->CurrentStrain : 0.0f;
	}
	SafeLog(FString::Printf(TEXT("Constraint strain for %s: NOT FOUND"), *BoneName.ToString()));
	return 0.0f;
}

FOHConstraintDriveData UOHPACManager::GetConstraintDriveData(FName BoneName) const
{
	FConstraintInstance* Constraint = FindPhysicalAnimationConstraint(BoneName);
	return ExtractConstraintDriveData(Constraint);
}

// New APIs for multiple constraint support
TArray<const FOHConstraintData*> UOHPACManager::GetConstraintsAsChild(FName BoneName) const
{
	TArray<const FOHConstraintData*> Result;

	if (const FOHConstraintLookup* Lookup = ConstraintLookupMap.Find(BoneName))
	{
		Result.Reserve(Lookup->AsChild.Num());
		for (const FOHConstraintData& Data : Lookup->AsChild)
		{
			Result.Add(&Data);
		}
	}

	return Result;
}

TArray<const FOHConstraintData*> UOHPACManager::GetConstraintsAsParent(FName BoneName) const
{
	TArray<const FOHConstraintData*> Result;

	if (const FOHConstraintLookup* Lookup = ConstraintLookupMap.Find(BoneName))
	{
		Result.Reserve(Lookup->AsParent.Num());
		for (const FOHConstraintData& Data : Lookup->AsParent)
		{
			Result.Add(&Data);
		}
	}

	return Result;
}

//  returns primary constraint
const FOHConstraintData* UOHPACManager::GetConstraintData(FName BoneName) const
{
	if (const FOHConstraintLookup* Lookup = ConstraintLookupMap.Find(BoneName))
	{
		if (Lookup->AsChild.Num() > 0)
		{
			return &Lookup->AsChild[0];
		}
	}

	return nullptr;
}

void UOHPACManager::ClearConstraintData()
{
	ConstraintLookupMap.Empty();
	ConstraintDataVersion++;

	SafeLog(TEXT("Constraint data cleared"), false);
}


#pragma endregion

// ============================================================================
// PROFILE MANAGEMENT
// ============================================================================
#pragma region PROFILE MANAGEMENT

FPhysicalAnimationData UOHPACManager::GetProfileForStrength(EOHPhysicsStrength Strength) const
{
	const FPhysicalAnimationData* Profile = StrengthProfiles.Find(Strength);
	return Profile ? *Profile : FPhysicalAnimationData();
}

FPhysicalAnimationData UOHPACManager::GetNamedProfile(FName ProfileName) const
{
	const FPhysicalAnimationData* Profile = NamedProfiles.Find(ProfileName);
	return Profile ? *Profile : FPhysicalAnimationData();
}

FPhysicalAnimationData UOHPACManager::GetCurrentPhysicalAnimationProfile(FName BoneName) const
{
	FPhysicalAnimationData CurrentProfile;

	// Try to find the PAC constraint for this bone
	FConstraintInstance* PACConstraint = FindPhysicalAnimationConstraint(BoneName);

	if (!PACConstraint)
	{
		// Return zero profile if no PAC active
		CurrentProfile.PositionStrength = 0.0f;
		CurrentProfile.VelocityStrength = 0.0f;
		CurrentProfile.OrientationStrength = 0.0f;
		CurrentProfile.AngularVelocityStrength = 0.0f;
		CurrentProfile.bIsLocalSimulation = true;
		return CurrentProfile;
	}

	// Extract drive parameters from the constraint
	float LinearStiffness, LinearDamping, AngularStiffness, AngularDamping;
	if (!GetConstraintDriveParametersFromInstance(PACConstraint, LinearStiffness, LinearDamping, AngularStiffness,
	                                              AngularDamping))
	{
		// Failed to extract parameters, return baseline
		CurrentProfile.PositionStrength = 1000.0f;
		CurrentProfile.VelocityStrength = 100.0f;
		CurrentProfile.OrientationStrength = 2000.0f;
		CurrentProfile.AngularVelocityStrength = 200.0f;
		CurrentProfile.bIsLocalSimulation = true;
		return CurrentProfile;
	}

	// Convert drive parameters to PAC profile
	CurrentProfile.PositionStrength = LinearStiffness;
	CurrentProfile.VelocityStrength = LinearDamping;
	CurrentProfile.OrientationStrength = AngularStiffness;
	CurrentProfile.AngularVelocityStrength = AngularDamping;
	CurrentProfile.bIsLocalSimulation = true;

	return CurrentProfile;
}

float UOHPACManager::GetPhysicalAnimationStrengthMultiplier() const
{
	if (!PhysicalAnimationComponent)
	{
		return 0.0f;
	}

	return PhysicalAnimationComponent->StrengthMultiplyer;
}

void UOHPACManager::SetPhysicalAnimationStrengthMultiplier(float Multiplier)
{
	if (!PhysicalAnimationComponent)
	{
		return;
	}
	PhysicalAnimationComponent->SetStrengthMultiplyer(Multiplier);
}

void UOHPACManager::IncrementPhysicalAnimationStrengthMultiplier(float Increment)
{
	if (!PhysicalAnimationComponent)
	{
		return;
	}
	float CurrentMultiplier = GetPhysicalAnimationStrengthMultiplier();
	float NewMultiplier = FMath::Clamp(CurrentMultiplier + Increment, 0.0f, 1000.0f);
	PhysicalAnimationComponent->SetStrengthMultiplyer(NewMultiplier);
}


#pragma endregion

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
#pragma region UTILITY FUNCTIONS

TArray<FName> UOHPACManager::GetBonesInCategory(EOHBoneCategory Category) const
{
	// First try the runtime map
	if (const TArray<FName>* CategoryBones = CategoryBonesMap.Find(Category))
	{
		// Filter to only return simulatable bones
		TArray<FName> SimulatableBonesInCategory;
		for (const FName& BoneName : *CategoryBones)
		{
			if (SimulatableBones.Contains(BoneName))
			{
				SimulatableBonesInCategory.Add(BoneName);
			}
		}
		return SimulatableBonesInCategory;
	}

	// Fallback: Try the definition map directly
	if (const FOHBoneCategoryDefinition* Definition = BoneCategoryDefinitions.Find(Category))
	{
		TArray<FName> SimulatableBonesInCategory;
		for (const FName& BoneName : Definition->Bones)
		{
			if (SimulatableBones.Contains(BoneName))
			{
				SimulatableBonesInCategory.Add(BoneName);
			}
		}
		return SimulatableBonesInCategory;
	}

	return TArray<FName>();
}

TArray<FName> UOHPACManager::GetBoneChain(FName RootBone, int32 MaxDepth) const
{
	TArray<FName> Chain;

	if (!SkeletalMesh || !SkeletalMesh->GetSkeletalMeshAsset())
	{
		SafeLog(TEXT("GetBoneChain: No skeletal mesh available"), true);
		return Chain;
	}

	const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 RootIndex = RefSkeleton.FindBoneIndex(RootBone);

	if (RootIndex == INDEX_NONE)
	{
		SafeLog(FString::Printf(TEXT("GetBoneChain: Bone %s not found in skeleton"),
		                        *RootBone.ToString()), true);
		return Chain;
	}

	// Add root bone first
	Chain.Add(RootBone);

	// Use BFS (Breadth-First Search) to get all children
	// This ensures we get bones level by level, which is useful for falloff calculations
	TQueue<TPair<int32, int32>> BonesToProcess; // Pair of BoneIndex and CurrentDepth
	BonesToProcess.Enqueue(TPair<int32, int32>(RootIndex, 0));

	// Build a children lookup map for efficiency
	TMultiMap<int32, int32> ParentToChildrenMap;
	for (int32 BoneIdx = 0; BoneIdx < RefSkeleton.GetNum(); BoneIdx++)
	{
		int32 ParentIdx = RefSkeleton.GetParentIndex(BoneIdx);
		if (ParentIdx != INDEX_NONE)
		{
			ParentToChildrenMap.Add(ParentIdx, BoneIdx);
		}
	}

	// Process bones level by level
	while (!BonesToProcess.IsEmpty())
	{
		TPair<int32, int32> Current;
		BonesToProcess.Dequeue(Current);

		int32 CurrentBoneIndex = Current.Key;
		int32 CurrentDepth = Current.Value;

		// Check depth limit
		if (MaxDepth >= 0 && CurrentDepth >= MaxDepth)
		{
			continue;
		}

		// Find all children of current bone
		TArray<int32> Children;
		ParentToChildrenMap.MultiFind(CurrentBoneIndex, Children);

		for (int32 ChildIndex : Children)
		{
			FName ChildBoneName = RefSkeleton.GetBoneName(ChildIndex);

			// Only add if it's a simulatable bone (or if we're tracking all bones)
			if (SimulatableBones.Contains(ChildBoneName) || TrackedBones.Contains(ChildBoneName))
			{
				Chain.Add(ChildBoneName);
				BonesToProcess.Enqueue(TPair<int32, int32>(ChildIndex, CurrentDepth + 1));
			}
			else if (bVerboseLogging)
			{
				SafeLog(FString::Printf(TEXT("GetBoneChain: Skipping non-simulatable bone %s"),
				                        *ChildBoneName.ToString()));
			}
		}
	}

	if (bVerboseLogging)
	{
		SafeLog(FString::Printf(TEXT("GetBoneChain: Found %d bones in chain from %s (MaxDepth=%d)"),
		                        Chain.Num(), *RootBone.ToString(), MaxDepth));
	}

	return Chain;
}


bool UOHPACManager::IsBoneValidForSimulation(FName BoneName) const
{
	return SimulatableBones.Contains(BoneName) && GetBodyInstanceDirect(BoneName) != nullptr;
}

void UOHPACManager::CacheSimulatableBones()
{
	SimulatableBones.Empty();

	if (!CachedPhysicsAsset)
	{
		SafeLog(TEXT("No physics asset available for caching simulatable bones"), true);
		return;
	}

	// Build a set of all bones that exist in the physics asset
	TSet<FName> PhysicsAssetBones;
	for (const USkeletalBodySetup* BodySetup : CachedPhysicsAsset->SkeletalBodySetups)
	{
		if (BodySetup)
		{
			PhysicsAssetBones.Add(BodySetup->BoneName);
		}
	}

	// Start with tracked bones, remove exclusions, verify they exist in physics asset
	int32 ExcludedCount = 0;
	int32 NotInPhysicsCount = 0;

	for (const FName& BoneName : TrackedBones)
	{
		// Skip if in exclusion list
		if (ExcludedBones.Contains(BoneName))
		{
			ExcludedCount++;
			continue;
		}

		// Skip if not in physics asset
		if (!PhysicsAssetBones.Contains(BoneName))
		{
			NotInPhysicsCount++;
			SafeLog(FString::Printf(TEXT("Tracked bone '%s' not found in physics asset"), *BoneName.ToString()), false);
			continue;
		}

		// This bone is trackable and exists in physics
		SimulatableBones.Add(BoneName);
	}

	SafeLog(FString::Printf(TEXT("Cached %d simulatable bones (from %d tracked, %d excluded, %d not in physics)"),
	                        SimulatableBones.Num(), TrackedBones.Num(), ExcludedCount, NotInPhysicsCount));
}

void UOHPACManager::ApplyChainStabilization(const TArray<FName>& BoneChain)
{
	// Apply progressive damping down the chain
	for (int32 i = 0; i < BoneChain.Num(); i++)
	{
		if (FBodyInstance* Body = GetBodyInstanceDirect(BoneChain[i]))
		{
			float DampingMultiplier = 1.0f + (i * 0.1f);
			Body->LinearDamping = FMath::Min(Body->LinearDamping * DampingMultiplier, 10.0f);
			Body->AngularDamping = FMath::Min(Body->AngularDamping * DampingMultiplier, 10.0f);
		}
	}
}

#pragma endregion

// ============================================================================
// DIRECT ACCESS FUNCTIONS
// ============================================================================
#pragma region DIRECT ACCESS

FBodyInstance* UOHPACManager::GetBodyInstanceDirect(FName BoneName) const
{
	if (!SkeletalMesh || BoneName == NAME_None)
	{
		return nullptr;
	}

	// Use the cached physics asset that was set up in SetupPhysicsAsset()
	if (!CachedPhysicsAsset)
	{
		SafeLog(FString::Printf(TEXT("No cached PhysicsAsset available for body lookup: %s"), *BoneName.ToString()),
		        true);
		return nullptr;
	}

	// Direct lookup - already O(1)
	FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);

	// Validate
	if (!Body || !Body->IsValidBodyInstance())
	{
		SafeLog(FString::Printf(TEXT("GetBodyInstanceDirect: Bone %s not found or invalid"),
		                        *BoneName.ToString()), true);
		return nullptr;
	}

	return Body;
}


// Update the direct access functions to use new structure
FConstraintInstance* UOHPACManager::GetConstraintInstanceDirect(FName BoneName) const
{
	if (!SkeletalMesh)
	{
		return nullptr;
	}

	// Check new lookup structure first
	if (const FOHConstraintLookup* Lookup = ConstraintLookupMap.Find(BoneName))
	{
		// Return primary constraint (first as child)
		if (Lookup->AsChild.Num() > 0)
		{
			return Lookup->AsChild[0].ConstraintInstance;
		}
	}

	// Fallback to linear search if not found (shouldn't happen if BuildConstraintData was called)
	const TArray<FConstraintInstance*>& Constraints = SkeletalMesh->Constraints;
	for (FConstraintInstance* Constraint : Constraints)
	{
		if (Constraint && Constraint->GetChildBoneName() == BoneName)
		{
			return Constraint;
		}
	}

	return nullptr;
}


int32 UOHPACManager::GetBoneIndexDirect(FName BoneName) const
{
	if (!SkeletalMesh || !SkeletalMesh->GetSkeletalMeshAsset())
	{
		return INDEX_NONE;
	}

	// Check cache first
	if (int32* CachedIndex = BoneIndexCache.Find(BoneName))
	{
		return *CachedIndex;
	}

	// Get from skeleton
	int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
	if (BoneIndex != INDEX_NONE)
	{
		BoneIndexCache.Add(BoneName, BoneIndex);
	}

	return BoneIndex;
}

#pragma endregion

// ============================================================================
// CONSTRAINT FUNCTIONS
// ============================================================================
#pragma region CONSTRAINT FUNCTIONS


void UOHPACManager::BuildConstraintData()
{
	ConstraintLookupMap.Empty();
	ConstraintDataVersion++;

	if (!SkeletalMesh)
	{
		return;
	}

	const TArray<FConstraintInstance*>& Constraints = SkeletalMesh->Constraints;
	int32 TotalConstraints = 0;
	int32 PACConstraints = 0;

	// First pass: Build comprehensive constraint data
	for (FConstraintInstance* Constraint : Constraints)
	{
		if (!Constraint)
		{
			continue;
		}

		FName ChildBone = Constraint->GetChildBoneName();
		FName ParentBone = Constraint->GetParentBoneName();

		// Skip invalid constraints
		if (ChildBone == NAME_None || ParentBone == NAME_None)
		{
			SafeLog(FString::Printf(TEXT("Invalid constraint found with missing bone names")), true);
			continue;
		}

		// Create constraint data
		FOHConstraintData Data;
		Data.ConstraintBone1 = ParentBone;
		Data.ConstraintBone2 = ChildBone;
		Data.ConstraintInstance = Constraint;
		// Add to child bone's constraint list
		if (SimulatableBones.Contains(ChildBone))
		{
			FOHConstraintLookup& ChildLookup = ConstraintLookupMap.FindOrAdd(ChildBone);
			ChildLookup.AsChild.Add(Data);

			// Check if this is a PAC constraint
			if (HasPhysicalAnimationDrives(Constraint))
			{
				ChildLookup.PACConstraint = Constraint;
				PACConstraints++;
			}
		}

		// Add to parent bone's constraint list  
		if (SimulatableBones.Contains(ParentBone))
		{
			FOHConstraintLookup& ParentLookup = ConstraintLookupMap.FindOrAdd(ParentBone);
			ParentLookup.AsParent.Add(Data);
		}

		TotalConstraints++;
	}

	// Validate constraint integrity
	int32 BonesWithMultipleConstraints = 0;
	for (const auto& Pair : ConstraintLookupMap)
	{
		const FOHConstraintLookup& Lookup = Pair.Value;
		if (Lookup.AsChild.Num() > 1)
		{
			BonesWithMultipleConstraints++;
			SafeLog(FString::Printf(TEXT("Bone %s has %d parent constraints"),
			                        *Pair.Key.ToString(), Lookup.AsChild.Num()), false);
		}
	}

	SafeLog(FString::Printf(
		TEXT(
			"Built constraint data: %d total constraints, %d bones tracked, %d PAC constraints, %d bones with multiple constraints"),
		TotalConstraints, ConstraintLookupMap.Num(), PACConstraints, BonesWithMultipleConstraints
	));
}

FConstraintInstance* UOHPACManager::FindPhysicalAnimationConstraint(FName BoneName) const
{
	// O(1) lookup from pre-built data
	if (const FOHConstraintLookup* Lookup = ConstraintLookupMap.Find(BoneName))
	{
		return Lookup->PACConstraint;
	}

	return nullptr;
}


bool UOHPACManager::HasPhysicalAnimationDrives(const FConstraintInstance* Constraint)
{
	if (!Constraint)
	{
		return false;
	}

	const FConstraintProfileProperties& Profile = Constraint->ProfileInstance;

	// Check linear drives
	bool bHasLinearDrive = false;
	if (Profile.LinearDrive.XDrive.bEnablePositionDrive || Profile.LinearDrive.XDrive.bEnableVelocityDrive)
	{
		bHasLinearDrive = Profile.LinearDrive.XDrive.Stiffness > 0.0f || Profile.LinearDrive.XDrive.Damping > 0.0f;
	}

	// Check angular drives
	bool bHasAngularDrive = false;
	if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::SLERP)
	{
		if (Profile.AngularDrive.SlerpDrive.bEnablePositionDrive || Profile.AngularDrive.SlerpDrive.
		                                                                    bEnableVelocityDrive)
		{
			bHasAngularDrive = Profile.AngularDrive.SlerpDrive.Stiffness > 0.0f || Profile.AngularDrive.SlerpDrive.
				Damping > 0.0f;
		}
	}
	else if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::TwistAndSwing)
	{
		if (Profile.AngularDrive.SwingDrive.bEnablePositionDrive || Profile.AngularDrive.SwingDrive.
		                                                                    bEnableVelocityDrive)
		{
			bHasAngularDrive = Profile.AngularDrive.SwingDrive.Stiffness > 0.0f || Profile.AngularDrive.SwingDrive.
				Damping > 0.0f;
		}
	}

	return bHasLinearDrive || bHasAngularDrive;
}

bool UOHPACManager::GetConstraintDriveParameters(
	FName BoneName,
	float& OutLinearStiffness,
	float& OutLinearDamping,
	float& OutAngularStiffness,
	float& OutAngularDamping) const
{
	FConstraintInstance* Constraint = FindPhysicalAnimationConstraint(BoneName);
	if (!Constraint)
	{
		return false;
	}

	return GetConstraintDriveParametersFromInstance(Constraint, OutLinearStiffness, OutLinearDamping,
	                                                OutAngularStiffness, OutAngularDamping);
}

bool UOHPACManager::GetConstraintDriveParametersFromInstance(
	FConstraintInstance* Constraint,
	float& OutLinearStiffness,
	float& OutLinearDamping,
	float& OutAngularStiffness,
	float& OutAngularDamping)
{
	if (!Constraint)
	{
		return false;
	}

	const FConstraintProfileProperties& Profile = Constraint->ProfileInstance;

	// Linear drive parameters (using XDrive as representative)
	if (Profile.LinearDrive.XDrive.bEnablePositionDrive || Profile.LinearDrive.XDrive.bEnableVelocityDrive)
	{
		OutLinearStiffness = Profile.LinearDrive.XDrive.Stiffness;
		OutLinearDamping = Profile.LinearDrive.XDrive.Damping;
	}

	else
	{
		OutLinearStiffness = 0.0f;
		OutLinearDamping = 0.0f;
	}

	// Angular drive parameters
	if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::SLERP)
	{
		if (Profile.AngularDrive.SlerpDrive.bEnablePositionDrive || Profile.AngularDrive.SlerpDrive.
		                                                                    bEnableVelocityDrive)
		{
			OutAngularStiffness = Profile.AngularDrive.SlerpDrive.Stiffness;
			OutAngularDamping = Profile.AngularDrive.SlerpDrive.Damping;
		}
		else
		{
			OutAngularStiffness = 0.0f;
			OutAngularDamping = 0.0f;
		}
	}
	else if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::TwistAndSwing)
	{
		// Use swing drive as representative for angular
		if (Profile.AngularDrive.SwingDrive.bEnablePositionDrive || Profile.AngularDrive.SwingDrive.
		                                                                    bEnableVelocityDrive)
		{
			OutAngularStiffness = Profile.AngularDrive.SwingDrive.Stiffness;
			OutAngularDamping = Profile.AngularDrive.SwingDrive.Damping;
		}
		else
		{
			OutAngularStiffness = 0.0f;
			OutAngularDamping = 0.0f;
		}
	}
	else
	{
		OutAngularStiffness = 0.0f;
		OutAngularDamping = 0.0f;
	}

	return true;
}

FOHConstraintDriveData UOHPACManager::ExtractConstraintDriveData(const FConstraintInstance* Constraint)
{
	FOHConstraintDriveData DriveData;

	if (!Constraint)
	{
		return DriveData;
	}

	const FConstraintProfileProperties& Profile = Constraint->ProfileInstance;

	// Linear drives
	DriveData.LinearStiffnessX = Profile.LinearDrive.XDrive.Stiffness;
	DriveData.LinearStiffnessY = Profile.LinearDrive.YDrive.Stiffness;
	DriveData.LinearStiffnessZ = Profile.LinearDrive.ZDrive.Stiffness;

	DriveData.LinearDampingX = Profile.LinearDrive.XDrive.Damping;
	DriveData.LinearDampingY = Profile.LinearDrive.YDrive.Damping;
	DriveData.LinearDampingZ = Profile.LinearDrive.ZDrive.Damping;

	DriveData.bLinearXDriveEnabled = Profile.LinearDrive.XDrive.bEnablePositionDrive || Profile.LinearDrive.XDrive.
		bEnableVelocityDrive;
	DriveData.bLinearYDriveEnabled = Profile.LinearDrive.YDrive.bEnablePositionDrive || Profile.LinearDrive.YDrive.
		bEnableVelocityDrive;
	DriveData.bLinearZDriveEnabled = Profile.LinearDrive.ZDrive.bEnablePositionDrive || Profile.LinearDrive.ZDrive.
		bEnableVelocityDrive;

	DriveData.LinearForceLimit = Profile.LinearDrive.XDrive.MaxForce; // Assuming uniform limits

	// Angular drives
	if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::SLERP)
	{
		DriveData.AngularStiffnessSlerp = Profile.AngularDrive.SlerpDrive.Stiffness;
		DriveData.AngularDampingSlerp = Profile.AngularDrive.SlerpDrive.Damping;
		DriveData.bAngularSlerpDriveEnabled = Profile.AngularDrive.SlerpDrive.bEnablePositionDrive || Profile.
			AngularDrive.SlerpDrive.bEnableVelocityDrive;
		DriveData.AngularForceLimit = Profile.AngularDrive.SlerpDrive.MaxForce;
	}
	else if (Profile.AngularDrive.AngularDriveMode == EAngularDriveMode::TwistAndSwing)
	{
		DriveData.AngularStiffnessSwing = Profile.AngularDrive.SwingDrive.Stiffness;
		DriveData.AngularStiffnessTwist = Profile.AngularDrive.TwistDrive.Stiffness;

		DriveData.AngularDampingSwing = Profile.AngularDrive.SwingDrive.Damping;
		DriveData.AngularDampingTwist = Profile.AngularDrive.TwistDrive.Damping;

		DriveData.bAngularSwingDriveEnabled = Profile.AngularDrive.SwingDrive.bEnablePositionDrive || Profile.
			AngularDrive.SwingDrive.bEnableVelocityDrive;
		DriveData.bAngularTwistDriveEnabled = Profile.AngularDrive.TwistDrive.bEnablePositionDrive || Profile.
			AngularDrive.TwistDrive.bEnableVelocityDrive;

		DriveData.AngularForceLimit = Profile.AngularDrive.SwingDrive.MaxForce;
	}

	return DriveData;
}

#pragma endregion

// ============================================================================
// VALIDATION
// ============================================================================
#pragma region VALIDATION

bool UOHPACManager::ValidatePhysicsSimulation() const
{
	if (!SkeletalMesh)
	{
		SafeLog(TEXT("Validation failed: No SkeletalMesh"), true);
		return false;
	}

	if (!PhysicalAnimationComponent)
	{
		SafeLog(TEXT("Validation failed: No PhysicalAnimationComponent"), true);
		return false;
	}

	if (!SkeletalMesh->GetPhysicsAsset())
	{
		SafeLog(TEXT("Validation failed: No PhysicsAsset"), true);
		return false;
	}

	return true;
}

bool UOHPACManager::ValidateSetup(
	TArray<FName>& OutMissingBones,
	TArray<FName>& OutInstancesWithoutBodies,
	TArray<FName>& OutMissingConstraints,
	TArray<FName>& OutRuntimeConstraintsNotInAsset,
	TArray<FName>& OutMismatchedConstraints) const
{
	bool bIsValid = true;

	// Validate bodies
	if (!ValidateBodyInstances(OutMissingBones, OutInstancesWithoutBodies))
	{
		bIsValid = false;
	}

	// Validate constraints
	if (!ValidateConstraintInstances(OutMissingConstraints, OutRuntimeConstraintsNotInAsset, OutMismatchedConstraints))
	{
		bIsValid = false;
	}

	return bIsValid;
}

bool UOHPACManager::ValidateBodyInstances(TArray<FName>& OutMissingBones,
                                          TArray<FName>& OutInstancesWithoutBodies) const
{
	if (!SkeletalMesh || !CachedPhysicsAsset)
	{
		return false;
	}

	bool bIsValid = true;

	// Check each simulatable bone
	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);

		if (!Body)
		{
			OutMissingBones.Add(BoneName);
			bIsValid = false;
		}
		else if (!Body->IsValidBodyInstance())
		{
			OutInstancesWithoutBodies.Add(BoneName);
			bIsValid = false;
		}
	}

	return bIsValid;
}

bool UOHPACManager::ValidateConstraintInstances(
	TArray<FName>& OutMissingConstraints,
	TArray<FName>& OutRuntimeConstraintsNotInAsset,
	TArray<FName>& OutMismatchedConstraints) const
{
	if (!SkeletalMesh || !CachedPhysicsAsset)
	{
		return false;
	}

	bool bIsValid = true;

	// Build a map of expected constraints from physics asset
	TMap<FName, const UPhysicsConstraintTemplate*> AssetConstraints;
	for (const UPhysicsConstraintTemplate* Template : CachedPhysicsAsset->ConstraintSetup)
	{
		if (Template)
		{
			// In UE5.3, we access the constraint bone names through the template
			FName ChildBone = Template->DefaultInstance.ConstraintBone2;
			AssetConstraints.Add(ChildBone, Template);
		}
	}

	// Check runtime constraints
	const TArray<FConstraintInstance*>& RuntimeConstraints = SkeletalMesh->Constraints;

	for (FConstraintInstance* RuntimeConstraint : RuntimeConstraints)
	{
		if (!RuntimeConstraint)
		{
			continue;
		}

		FName ChildBone = RuntimeConstraint->GetChildBoneName();

		if (!AssetConstraints.Contains(ChildBone))
		{
			OutRuntimeConstraintsNotInAsset.Add(ChildBone);
			bIsValid = false;
		}
	}

	// Check for missing runtime constraints
	for (const auto& Pair : AssetConstraints)
	{
		FName BoneName = Pair.Key;
		bool bFoundInRuntime = false;

		for (FConstraintInstance* RuntimeConstraint : RuntimeConstraints)
		{
			if (RuntimeConstraint && RuntimeConstraint->GetChildBoneName() == BoneName)
			{
				bFoundInRuntime = true;
				break;
			}
		}

		if (!bFoundInRuntime)
		{
			OutMissingConstraints.Add(BoneName);
			bIsValid = false;
		}
	}

	return bIsValid;
}

bool UOHPACManager::ValidatePhysicsAsset(
	TArray<FName>& OutMissingBones,
	TArray<FName>& OutInstancesWithoutBodies,
	TArray<FName>& OutMissingConstraints,
	TArray<FName>& OutRuntimeConstraintsNotInAsset,
	TArray<FName>& OutMismatchedConstraints) const
{
	return ValidateSetup(OutMissingBones, OutInstancesWithoutBodies,
	                     OutMissingConstraints, OutRuntimeConstraintsNotInAsset,
	                     OutMismatchedConstraints);
}

// ============================================================================
// COMPONENT REFERENCE VALIDATION
// ============================================================================

void UOHPACManager::ValidateComponentReferences()
{
	if (bComponentReferencesValidated)
	{
		return;
	}
    
	// Cache movement component with validation
	if (!CachedMovementComponent.IsValid())
	{
		if (AActor* Owner = GetOwner())
		{
			if (UOHMovementComponent* MovementComp = Owner->FindComponentByClass<UOHMovementComponent>())
			{
				CachedMovementComponent = MovementComp;
				SafeLog(TEXT("Movement component reference cached successfully"));
                
				// Verify the movement component is properly initialized
				if (!MovementComp->bMovementInitialized)
				{
					SafeLog(TEXT("WARNING: Movement component found but not initialized!"), true);
				}
			}
			else
			{
				SafeLog(TEXT("ERROR: No OHMovementComponent found on owner!"), true);
			}
		}
	}
    
	bComponentReferencesValidated = true;
}

void UOHPACManager::CheckAllPhysicsBodiesValid() const
{
	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body || !Body->IsValidBodyInstance())
		{
			UE_LOG(LogPACManager, Error, TEXT("INVALID BodyInstance for %s"), *BoneName.ToString());
		}
	}
}
#pragma endregion

// ============================================================================
// EVENT HANDLERS
// ============================================================================
#pragma region EVENT HANDLERS

void UOHPACManager::OnSkeletalMeshChanged()
{
	DisableAllPhysicsBodies();
	StopAllBlends();
	if (!SkeletalMesh)
	{
		return;
	}

	USkeletalMesh* CurrentMesh = SkeletalMesh->GetSkeletalMeshAsset();
	UPhysicsAsset* CurrentPhysicsAsset = CurrentMesh ? CurrentMesh->GetPhysicsAsset() : nullptr;


	if (!CurrentPhysicsAsset)
	{
		SafeLog(TEXT("No physics asset found on SkeletalMesh"), true);
		if (UPhysicsAsset* FallBackPhysicsAsset = DefaultPhysicsAsset)
		{
			CurrentPhysicsAsset = FallBackPhysicsAsset;
			SkeletalMesh->SetPhysicsAsset(CurrentPhysicsAsset);
			SafeLog(TEXT("Falling back to default physics asset"), true);
		}
	}


	bool bMeshChanged = CurrentMesh != PreviousMeshAsset;
	bool bPhysicsAssetChanged = CurrentPhysicsAsset != PreviousPhysicsAsset;

	if (bMeshChanged || bPhysicsAssetChanged)
	{
		SafeLog(TEXT("Skeletal mesh or physics asset changed, reinitializing..."));

		// Clear caches
		//BodyInstanceCache.Empty();
		BuildConstraintData();
		BoneIndexCache.Empty();


		// Rebuild data
		CachedPhysicsAsset = CurrentPhysicsAsset;
		InitializeMotionTracking();
		CacheSimulatableBones();
		BuildConstraintData();


		// Update references
		PreviousMeshAsset = CurrentMesh;
		PreviousPhysicsAsset = CurrentPhysicsAsset;
	}
}

void UOHPACManager::OnSkeletalAssetChanged(USkeletalMesh* NewMesh, USkeletalMesh* OldMesh)
{
	OnSkeletalMeshChanged();
}


#pragma endregion

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================
#pragma region DEBUG FUNCTIONS

void UOHPACManager::DrawDebugOverlay() const
{
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (!bDrawDebug || !SkeletalMesh || !GetWorld())
	{
		return;
	}

	// Draw body states
	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body)
		{
			continue;
		}

		FTransform BoneTransform = SkeletalMesh->GetBoneTransform(GetBoneIndexDirect(BoneName));
		FVector BoneLocation = BoneTransform.GetLocation();

		// Color based on state
		FColor DebugColor = FColor::White;
		if (Body->IsInstanceSimulatingPhysics())
		{
			if (IsBoneDrivenByPhysicalAnimation(BoneName))
			{
				DebugColor = FColor::Green; // PAC active
			}
			else
			{
				DebugColor = FColor::Yellow; // Pure physics
			}
		}
		else
		{
			DebugColor = FColor::Red; // Kinematic
		}

		// Draw sphere at bone location
		DrawDebugSphere(GetWorld(), BoneLocation, 5.0f, 8, DebugColor, false, -1.0f, 0, 2.0f);

		// Draw blend weight text if enabled
		if (bShowBlendWeightText)
		{
			float BlendWeight = Body->PhysicsBlendWeight;
			if (BlendWeight > 0.01f)
			{
				FString WeightText = FString::Printf(TEXT("%.1f%%"), BlendWeight * 100.0f);
				DrawDebugString(GetWorld(), BoneLocation + FVector(0, 0, 15),
				                WeightText, nullptr, FColor::White, 0.0f, true);
			}
		}

		// Draw PAC profile values if enabled
		if (bShowPACProfileValues && IsBoneDrivenByPhysicalAnimation(BoneName))
		{
			FPhysicalAnimationData CurrentProfile = GetCurrentPhysicalAnimationProfile(BoneName);
			FString ProfileText = FString::Printf(TEXT("P:%.0f O:%.0f"),
			                                      CurrentProfile.PositionStrength, CurrentProfile.OrientationStrength);
			DrawDebugString(GetWorld(), BoneLocation + FVector(0, 0, 25),
			                ProfileText, nullptr, FColor::Cyan, 0.0f, true);
		}

		// Draw velocity vector
		if (Body->IsInstanceSimulatingPhysics())
		{
			FVector Velocity = Body->GetUnrealWorldVelocity();
			if (!Velocity.IsNearlyZero())
			{
				DrawDebugLine(GetWorld(), BoneLocation, BoneLocation + Velocity * 0.1f,
				              FColor::Cyan, false, -1.0f, 0, 2.0f);
			}
		}


		// Draw constraint info
		if (const FOHConstraintData* ConstraintData = GetConstraintData(BoneName))
		{
			if (ConstraintData->CurrentStrain > 0.5f)
			{
				FColor StrainColor = FColor::MakeRedToGreenColorFromScalar(1.0f - ConstraintData->CurrentStrain);
				DrawDebugString(GetWorld(), BoneLocation + FVector(0, 0, 10),
				                FString::Printf(TEXT("Strain: %.2f"), ConstraintData->CurrentStrain),
				                nullptr, StrainColor, 0.0f, true);
			}
		}
	}


	// Draw active blend info
	int32 YOffset = 50;
	for (const auto& Pair : ActiveBlends)
	{
		for (const FOHActiveBlend& Blend : Pair.Value)
		{
			if (!Blend.IsComplete())
			{
				FString BlendInfo = FString::Printf(TEXT("%s: %s (%.1f%%)"),
				                                    *Blend.BoneName.ToString(),
				                                    *UEnum::GetValueAsString(Blend.CurrentPhase),
				                                    Blend.PhysicsBlendWeight_Current * 100.0f);

				DrawDebugString(GetWorld(), FVector::ZeroVector, BlendInfo,
				                nullptr, FColor::White, 0.0f, true, 1.0f);
				YOffset += 20;
			}
		}
	}
#endif
}


void UOHPACManager::LogSystemState() const
{
	UE_LOG(LogPACManager, Warning, TEXT("=== PAC Manager System State ==="));
	UE_LOG(LogPACManager, Warning, TEXT("Initialized: %s"), bIsInitialized ? TEXT("Yes") : TEXT("No"));
	UE_LOG(LogPACManager, Warning, TEXT("SkeletalMesh: %s"), SkeletalMesh ? TEXT("Valid") : TEXT("Invalid"));
	UE_LOG(LogPACManager, Warning, TEXT("PAC Component: %s"),
	       PhysicalAnimationComponent ? TEXT("Valid") : TEXT("Invalid"));
	UE_LOG(LogPACManager, Warning, TEXT("Physics Asset: %s"),
	       CachedPhysicsAsset ? *CachedPhysicsAsset->GetName() : TEXT("None"));
	UE_LOG(LogPACManager, Warning, TEXT("Simulatable Bones: %d"), SimulatableBones.Num());
	UE_LOG(LogPACManager, Warning, TEXT("Active Blends: %d"), GetTotalActiveBlendCount());
	UE_LOG(LogPACManager, Warning, TEXT("Motion Tracking: %d bones"), BoneMotionMap.Num());
	UE_LOG(LogPACManager, Warning, TEXT("Constraint Data: %d constraints"), ConstraintDataMap.Num());
}

// 7. Debug function to log blend state
void UOHPACManager::LogBlendState(FName BoneName) const
{
	const TArray<FOHActiveBlend>* Blends = ActiveBlends.Find(BoneName);
	if (!Blends)
	{
		UE_LOG(LogPACManager, Warning, TEXT("No active blends for bone %s"), *BoneName.ToString());
		return;
	}

	for (int32 i = 0; i < Blends->Num(); i++)
	{
		const FOHActiveBlend& Blend = (*Blends)[i];
		UE_LOG(LogPACManager, Warning,
		       TEXT(
			       "Blend[%d] - CurrentPhase: %s, Alpha: %.4f, PhysicsBlendWeight_Start: %.4f, PhysicsBlendWeight_Target: %.4f, ElapsedTime: %.2f"
		       ),
		       i,
		       *UEnum::GetValueAsString(Blend.CurrentPhase),
		       Blend.PhysicsBlendWeight_Current,
		       Blend.PhysicsBlendWeight_Start,
		       Blend.PhysicsBlendWeight_Target,
		       Blend.ElapsedTime
		);
	}
}


void UOHPACManager::LogActiveBlends() const
{
	UE_LOG(LogPACManager, Warning, TEXT("=== Active Blends ==="));

	for (const auto& Pair : ActiveBlends)
	{
		FName BoneName = Pair.Key;
		const TArray<FOHActiveBlend>& Blends = Pair.Value;

		UE_LOG(LogPACManager, Warning, TEXT("Bone: %s (%d blends)"), *BoneName.ToString(), Blends.Num());

		for (const FOHActiveBlend& Blend : Blends)
		{
			if (!Blend.IsComplete())
			{
				UE_LOG(LogPACManager, Warning, TEXT("  - CurrentPhase: %s, Alpha: %.2f, Tag: %s"),
				       *UEnum::GetValueAsString(Blend.CurrentPhase),
				       Blend.PhysicsBlendWeight_Current,
				       *Blend.ReactionTag.ToString());
			}
		}
	}
}

void UOHPACManager::DebugBodyPhysicsStates()
{
	if (!SkeletalMesh)
	{
		return;
	}

	UE_LOG(LogPACManager, Warning, TEXT("=== Body Physics States ==="));

	for (const FName& BoneName : SimulatableBones)
	{
		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body)
		{
			UE_LOG(LogPACManager, Error, TEXT("%s: No body instance"), *BoneName.ToString());
			continue;
		}

		bool bSimulating = Body->IsInstanceSimulatingPhysics();
		bool bHasPAC = IsBoneDrivenByPhysicalAnimation(BoneName);
		float BlendWeight = Body->PhysicsBlendWeight;

		UE_LOG(LogPACManager, Warning, TEXT("%s: Sim=%s, PAC=%s, BlendWeight=%.2f"),
		       *BoneName.ToString(),
		       bSimulating ? TEXT("Yes") : TEXT("No"),
		       bHasPAC ? TEXT("Yes") : TEXT("No"),
		       BlendWeight);
	}
}

#pragma endregion

// ============================================================================
// STATIC HELPER FUNCTIONS
// ============================================================================
#pragma region STATIC HELPERS

TArray<FBodyInstance*> UOHPACManager::GetSimulatableBodies(
	USkeletalMeshComponent* SkeletalMesh,
	const TSet<FName>& SimulatableBones)
{
	TArray<FBodyInstance*> Bodies;

	if (!SkeletalMesh)
	{
		return Bodies;
	}

	for (const FName& BoneName : SimulatableBones)
	{
		if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName))
		{
			Bodies.Add(Body);
		}
	}

	return Bodies;
}

#pragma endregion


// ============================================================================
// MOTION DATA ACCESS
// ============================================================================
#pragma region MOTION DATA ACCESS

FVector UOHPACManager::GetBoneVelocity(FName BoneName, EOHReferenceSpace Space, float SmoothingAlpha) const
{
    // Defensive validation
    if (BoneName.IsNone())
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("GetBoneVelocity: Invalid bone name provided"), true);
        }
        return FVector::ZeroVector;
    }
    
    // Retrieve motion data with validation
    const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
    if (!MotionData)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneVelocity: No motion data found for bone '%s'"), 
                *BoneName.ToString()), false);
        }
        return FVector::ZeroVector;
    }
    
    // Validate motion data integrity
    if (!MotionData->GetLatestSample())
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneVelocity: No samples available for bone '%s'"), 
                *BoneName.ToString()), false);
        }
        return FVector::ZeroVector;
    }
    
    FVector ResultVelocity = FVector::ZeroVector;
    
    // Apply smoothing if requested using OHAlgoUtils
    if (SmoothingAlpha > KINDA_SMALL_NUMBER)
    {
        // Clamp smoothing alpha to valid range
        float ClampedAlpha = FMath::Clamp(SmoothingAlpha, 0.0f, 1.0f);
        
        // Use OHAlgoUtils for robust smoothing
        ResultVelocity = UOHAlgoUtils::SmoothVelocityEMA(*MotionData, ClampedAlpha);
    }
    else
    {
        // Get velocity based on reference space
        switch (Space)
        {
            case EOHReferenceSpace::WorldSpace:
                ResultVelocity = MotionData->GetVelocity(false);
                break;
                
            case EOHReferenceSpace::LocalSpace:
                ResultVelocity = MotionData->GetVelocity(true);
                break;
                
            case EOHReferenceSpace::ComponentSpace:
                if (const FOHMotionFrameSample* Sample = MotionData->GetLatestSample())
                {
                    ResultVelocity = Sample->ComponentLinearVelocity;
                }
                break;
                
            default:
                if (bVerboseLogging)
                {
                    SafeLog(FString::Printf(TEXT("GetBoneVelocity: Unknown reference space %d, defaulting to WorldSpace"), 
                        static_cast<int32>(Space)), true);
                }
                ResultVelocity = MotionData->GetVelocity(false);
                break;
        }
    }
    
    // Validate result for potential numerical issues
    if (!ResultVelocity.ContainsNaN())
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneVelocity: Non-finite velocity detected for bone '%s', returning zero"), 
                *BoneName.ToString()), true);
        }
        return FVector::ZeroVector;
    }
    
    // Apply reasonable magnitude limits to prevent extreme values
    const float MaxReasonableVelocity = 10000.0f; // 100 m/s
    if (ResultVelocity.Size() > MaxReasonableVelocity)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneVelocity: Clamping excessive velocity %.1f for bone '%s'"), 
                ResultVelocity.Size(), *BoneName.ToString()), true);
        }
        ResultVelocity = ResultVelocity.GetClampedToMaxSize(MaxReasonableVelocity);
    }
    
    return ResultVelocity;
}

FVector UOHPACManager::GetBoneAcceleration(FName BoneName, EOHReferenceSpace Space, float SmoothingAlpha) const
{
    // Defensive validation
    if (BoneName.IsNone())
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("GetBoneAcceleration: Invalid bone name provided"), true);
        }
        return FVector::ZeroVector;
    }
    
    // Retrieve motion data with validation
    const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
    if (!MotionData)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneAcceleration: No motion data found for bone '%s'"), 
                *BoneName.ToString()), false);
        }
        return FVector::ZeroVector;
    }
    
    // Validate sample availability
    if (!MotionData->GetLatestSample())
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneAcceleration: No samples available for bone '%s'"), 
                *BoneName.ToString()), false);
        }
        return FVector::ZeroVector;
    }
    
    FVector ResultAcceleration = FVector::ZeroVector;
    
    // Apply smoothing if requested
    if (SmoothingAlpha > KINDA_SMALL_NUMBER)
    {
        float ClampedAlpha = FMath::Clamp(SmoothingAlpha, 0.0f, 1.0f);
        
        // For acceleration smoothing, we need multiple samples
        if (MotionData->GetHistoryDepth() >= 2)
        {
            // Get raw acceleration first
            FVector RawAcceleration = (Space == EOHReferenceSpace::LocalSpace) ? 
                MotionData->GetAcceleration(true) : MotionData->GetAcceleration(false);
            
            // Apply exponential smoothing manually since OHAlgoUtils doesn't have acceleration smoothing
            static TMap<FName, FVector> PreviousSmoothedAcceleration;
            FVector* PrevSmoothed = PreviousSmoothedAcceleration.Find(BoneName);
            
            if (PrevSmoothed)
            {
                ResultAcceleration = FMath::Lerp(*PrevSmoothed, RawAcceleration, ClampedAlpha);
            }
            else
            {
                ResultAcceleration = RawAcceleration;
            }
            
            // Update the smoothed value for next frame
            PreviousSmoothedAcceleration.Add(BoneName, ResultAcceleration);
        }
        else
        {
            // Insufficient history for smoothing, return raw
            ResultAcceleration = (Space == EOHReferenceSpace::LocalSpace) ? 
                MotionData->GetAcceleration(true) : MotionData->GetAcceleration(false);
        }
    }
    else
    {
        // Get acceleration based on reference space
        switch (Space)
        {
            case EOHReferenceSpace::WorldSpace:
                ResultAcceleration = MotionData->GetAcceleration(false);
                break;
                
            case EOHReferenceSpace::LocalSpace:
                ResultAcceleration = MotionData->GetAcceleration(true);
                break;
                
            case EOHReferenceSpace::ComponentSpace:
                if (const FOHMotionFrameSample* Sample = MotionData->GetLatestSample())
                {
                    // Component space acceleration needs transform
                    if (SkeletalMesh)
                    {
                        FTransform ComponentTransform = SkeletalMesh->GetComponentTransform();
                        ResultAcceleration = ComponentTransform.InverseTransformVector(Sample->WorldLinearAcceleration);
                    }
                    else
                    {
                        ResultAcceleration = Sample->WorldLinearAcceleration;
                    }
                }
                break;
                
            default:
                if (bVerboseLogging)
                {
                    SafeLog(FString::Printf(TEXT("GetBoneAcceleration: Unknown reference space %d, defaulting to WorldSpace"), 
                        static_cast<int32>(Space)), true);
                }
                ResultAcceleration = MotionData->GetAcceleration(false);
                break;
        }
    }
    
    // Validate result for potential numerical issues
    if (!ResultAcceleration.ContainsNaN())
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneAcceleration: Non-finite acceleration detected for bone '%s', returning zero"), 
                *BoneName.ToString()), true);
        }
        return FVector::ZeroVector;
    }
    
    // Apply reasonable magnitude limits
    const float MaxReasonableAcceleration = 50000.0f; // 500 m/s² (roughly 50g)
    if (ResultAcceleration.Size() > MaxReasonableAcceleration)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("GetBoneAcceleration: Clamping excessive acceleration %.1f for bone '%s'"), 
                ResultAcceleration.Size(), *BoneName.ToString()), true);
        }
        ResultAcceleration = ResultAcceleration.GetClampedToMaxSize(MaxReasonableAcceleration);
    }
    
    return ResultAcceleration;
}

float UOHPACManager::GetBoneSpeed(FName BoneName) const
{
	const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
	return MotionData ? MotionData->GetSpeed(EOHReferenceSpace::WorldSpace) : 0.0f;
}

const FOHBoneMotionData* UOHPACManager::GetBoneMotionData(FName BoneName) const
{
	return BoneMotionMap.Find(BoneName);
}

float UOHPACManager::GetBoneMotionQuality(FName BoneName) const
{
	if (const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName))
	{
		return MotionData->GetMotionQuality();
	}
	return 0.0f;
}

bool UOHPACManager::IsBoneMotionPredictable(FName BoneName) const
{
	return GetBoneMotionQuality(BoneName) > PredictionQualityThreshold;
}




FVector UOHPACManager::PredictBonePosition(FName BoneName, float PredictTime) const
{
	if (const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName))
	{
		return MotionData->PredictFuturePosition(PredictTime, EOHReferenceSpace::WorldSpace, true);
	}
	return SkeletalMesh ? SkeletalMesh->GetBoneLocation(BoneName) : FVector::ZeroVector;
}

TArray<FVector> UOHPACManager::GetBonePredictionPath(FName BoneName, float PredictTime, bool bCubic) const
{
	if (const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName))
	{
		return MotionData->GetBezierPredictionPath(PredictTime, bCubic);
	}
	return TArray<FVector>();
}

FVector UOHPACManager::GetReferenceVelocity() const
{
	if (!SkeletalMesh)
	{
		return FVector::ZeroVector;
	}

	// Try to get reference bone velocity
	if (const FOHBoneMotionData* RefMotionData = BoneMotionMap.Find(RootReferenceBone))
	{
		return RefMotionData->GetVelocity(false);
	}

	// Fallback to component velocity
	return SkeletalMesh->GetComponentVelocity();
}

FVector UOHPACManager::GetBoneAngularVelocity(FName BoneName) const
{
	const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
	if (!MotionData)
	{
		return FVector::ZeroVector;
	}

	const FOHMotionFrameSample* Latest = MotionData->GetLatestSample();
	return Latest ? Latest->AngularVelocity : FVector::ZeroVector;
}

FVector UOHPACManager::GetBoneJerk(FName BoneName, bool bUseLocalSpace) const
{
	const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
	return MotionData ? MotionData->CalculateJerk(bUseLocalSpace) : FVector::ZeroVector;
}

bool UOHPACManager::IsBoneInVelocityTransition(FName BoneName) const
{
	const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
	return MotionData ? MotionData->IsInVelocityTransition() : false;
}

bool UOHPACManager::HasBoneSuddenDirectionChange(FName BoneName, float AngleThreshold) const
{
	const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName);
	return MotionData ? MotionData->DetectSuddenDirectionChange(AngleThreshold, 3) : false;
}

#pragma endregion

#pragma region Locomotion

UOHMovementComponent* UOHPACManager::GetMovementComponent() const
{
	// Check if we have a valid cached reference
	if (CachedMovementComponent.IsValid())
	{
		return CachedMovementComponent.Get();
	}
    
	// Try to find and cache the component
	if (AActor* Owner = GetOwner())
	{
		if (UOHMovementComponent* MovementComp = Owner->FindComponentByClass<UOHMovementComponent>())
		{
			CachedMovementComponent = MovementComp;
            
			if (bVerboseLogging)
			{
				SafeLog(FString::Printf(TEXT("Movement component found and cached (Initialized: %s)"),
					MovementComp->bMovementInitialized ? TEXT("Yes") : TEXT("No")));
			}
            
			return MovementComp;
		}
	}
    
	if (bVerboseLogging && bEnableMovementPushback)
	{
		SafeLog(TEXT("Movement component not found - pushback unavailable"), true);
	}
    
	return nullptr;
}

void UOHPACManager::ProcessImpactPushback(const FVector& ImpactForce, const FVector& ImpactLocation, FName ImpactedBone)
{
    // === UNIFIED ENTRY POINT FOR ALL LOCOMOTION PUSHBACK ===
    UOHMovementComponent* MovementComp = GetMovementComponent();
    if (!MovementComp)
    {
        SafeLog(TEXT("ProcessImpactPushback: No movement component available for pushback"), true);
        return;
    }

    // === FORCE THRESHOLD VALIDATION ===
    float ImpactMagnitude = ImpactForce.Size();
    if (ImpactMagnitude < MinPushbackThreshold)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ProcessImpactPushback: Impact force %.1f below threshold %.1f"),
                                    ImpactMagnitude, MinPushbackThreshold), false);
        }
        return;
    }

    AActor* Owner = GetOwner();
    if (!Owner)
    {
        SafeLog(TEXT("ProcessImpactPushback: No owner actor available"), true);
        return;
    }

    // === ENHANCED DIRECTION CALCULATION WITH VALIDATION ===
    FVector PushbackDirection = ImpactForce.GetSafeNormal();
    
    // Validate input direction finite state
    if (!PushbackDirection.ContainsNaN())
    {
        SafeLog(TEXT("ProcessImpactPushback: Non-finite impact force direction detected"), true);
        return;
    }
    
    // Apply vertical scaling for grounded character physics
    PushbackDirection.Z *= VerticalPushbackScale;
    
    // Renormalize after vertical scaling
    if (!PushbackDirection.IsNearlyZero())
    {
        PushbackDirection.Normalize();
    }
    else
    {
        // Geometric fallback using OHAlgoUtils principles
        FVector CharacterLocation = Owner->GetActorLocation();
        FVector FallbackDirection = (CharacterLocation - ImpactLocation).GetSafeNormal();
        FallbackDirection.Z = 0.0f;
        PushbackDirection = FallbackDirection.GetSafeNormal();
        
        if (PushbackDirection.IsNearlyZero())
        {
            // Ultimate fallback - use actor forward direction
            PushbackDirection = Owner->GetActorForwardVector();
            
            if (bVerboseLogging)
            {
                SafeLog(TEXT("ProcessImpactPushback: Using actor forward as fallback direction"), true);
            }
        }
    }

    // === MAGNITUDE CALCULATION WITH CURVE INTEGRATION ===
    float PushbackMagnitude = ImpactMagnitude;
    
    // Apply force curve if configured
    if (PushbackForceCurve)
    {
        float CurveValue = PushbackForceCurve->GetFloatValue(ImpactMagnitude);
        if (FMath::IsFinite(CurveValue) && CurveValue > 0.0f)
        {
            PushbackMagnitude = CurveValue;
        }
        else
        {
            if (bVerboseLogging)
            {
                SafeLog(FString::Printf(TEXT("ProcessImpactPushback: Invalid curve value %.2f, using raw magnitude"), 
                    CurveValue), true);
            }
        }
    }
    
    // Apply force multiplier with validation
    float ValidatedMultiplier = FMath::Clamp(PushbackForceMultiplier, 0.1f, 10.0f);
    PushbackMagnitude *= ValidatedMultiplier;

    // === CHARACTER-RELATIVE TRANSFORMATION ===
    FVector CharacterForward = Owner->GetActorForwardVector();
    FVector CharacterRight = Owner->GetActorRightVector();
    
    // Validate character orientation vectors
    if (CharacterForward.IsNearlyZero() || CharacterRight.IsNearlyZero())
    {
        SafeLog(TEXT("ProcessImpactPushback: Invalid character orientation vectors"), true);
        return;
    }
    
    // Project world pushback direction onto character axes
    float ForwardComponent = FVector::DotProduct(PushbackDirection, CharacterForward);
    float RightComponent = FVector::DotProduct(PushbackDirection, CharacterRight);
    
    // Create character-relative 2D pushback vector
    FVector2D CharacterRelativePushback(ForwardComponent, RightComponent);
    CharacterRelativePushback *= PushbackMagnitude;
    
    // Validate final pushback vector
    if (!CharacterRelativePushback.ContainsNaN() && CharacterRelativePushback.Size() > KINDA_SMALL_NUMBER)
    {
        // === UNIFIED EVENT BROADCASTING (CRITICAL ADDITION) ===
        OnPushbackApplied.Broadcast(ImpactedBone, CharacterRelativePushback, PushbackMagnitude);
        
        // === MOVEMENT COMPONENT INTEGRATION ===
        MovementComp->HandlePACPushback(ImpactedBone, CharacterRelativePushback, PushbackMagnitude);
        
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(
                TEXT("ProcessImpactPushback: Applied unified pushback - WorldDir=%s, CharRelative=(F:%.2f,R:%.2f), Magnitude=%.1f"),
                *PushbackDirection.ToString(), 
                CharacterRelativePushback.X, 
                CharacterRelativePushback.Y,
                PushbackMagnitude), false);
        }
    }
    else
    {
        SafeLog(TEXT("ProcessImpactPushback: Invalid final pushback vector calculated"), true);
    }
}


FVector2D UOHPACManager::CalculatePushbackVector(const FVector& ImpactForce, const FVector& ImpactNormal)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector2D::ZeroVector;
	}

	// Use impact force directly - it should already be in world space
	// pointing in the direction we want to push the character
	FVector WorldPushback = ImpactForce;

	// Ensure we have a valid direction
	if (WorldPushback.IsNearlyZero())
	{
		// Fallback to impact normal
		WorldPushback = ImpactNormal;

		if (WorldPushback.IsNearlyZero())
		{
			return FVector2D::ZeroVector;
		}
	}

	// Keep it mostly horizontal
	WorldPushback.Z = 0.0f;
	WorldPushback.Normalize();

	// Get the magnitude
	float PushbackMagnitude = ImpactForce.Size2D();

	// Convert to 2D movement vector
	// We want this in world space, not local space
	FVector2D HorizontalPushback(WorldPushback.X, WorldPushback.Y);
	HorizontalPushback *= PushbackMagnitude;

	// Normalize to reasonable input range
	if (HorizontalPushback.Size() > 1000.0f)
	{
		HorizontalPushback = HorizontalPushback.GetSafeNormal() * 1000.0f;
	}

	// Log if we're getting unexpected values
	if (bVerboseLogging)
	{
		if (FMath::Abs(HorizontalPushback.Y) > FMath::Abs(HorizontalPushback.X) * 3.0f)
		{
			SafeLog(FString::Printf(TEXT("WARNING: Unusual Y-axis dominance in pushback: X=%.2f Y=%.2f"),
			                        HorizontalPushback.X, HorizontalPushback.Y), true);
		}
	}

	return HorizontalPushback;
}

// Directional pushback biasing based on stance
FVector2D UOHPACManager::BiasedPushbackForStance(const FVector2D& RawPushback, const FOHStanceState& StanceState)
{
	FVector2D BiasedPushback = RawPushback;

	// Wide stance = more lateral stability
	if (StanceState.CurrentStance == EOHStanceType::Wide)
	{
		BiasedPushback.Y *= 0.7f; // Reduce lateral pushback
	}
	// Narrow stance = less stability
	else if (StanceState.CurrentStance == EOHStanceType::Narrow)
	{
		BiasedPushback *= 1.3f; // More pushback overall
	}

	// If off-balance, amplify pushback
	if (!StanceState.bIsStable)
	{
		BiasedPushback *= 1.5f;
	}

	return BiasedPushback;
}

// Mass-based pushback scaling
float UOHPACManager::CalculateMassRatio(AActor* Attacker, AActor* Defender)
{
	// Get character masses (you might want to make this configurable)
	float AttackerMass = 80.0f; // Default human mass
	float DefenderMass = 80.0f;

	// Could pull from character stats, physics bodies, etc.

	float MassRatio = AttackerMass / FMath::Max(DefenderMass, 1.0f);
	return FMath::Clamp(MassRatio, 0.5f, 2.0f);
}
#pragma endregion

#pragma region COLLISION_IMPLEMENTATION


// --- Impulse application ---
void UOHPACManager::ApplyImpactImpulse(FName BoneName, const FVector& Impulse, const FVector& Location)
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		if (Body->IsInstanceSimulatingPhysics())
		{
			Body->AddImpulseAtPosition(Impulse, Location);
			Body->WakeInstance();

			if (bVerboseLogging && Impulse.Size() > 100.0f)
			{
				SafeLog(FString::Printf(TEXT("Applied impulse %.1f to %s"),
				                        Impulse.Size(), *BoneName.ToString()));
			}
		}
	}
}


FVector2D UOHPACManager::CalculateMovementFromImpact(
	const FHitResult& Hit,
	const FVector& RelativeVelocity,
	const FOHBoneMotionData* MotionData,
	float DragCoeff,
	float CrossSection,
	float Mass) const
{
	if (!MotionData)
	{
		return FVector2D::ZeroVector;
	}

	// Calculate impact force
	float ImpactSpeed = RelativeVelocity.Size();
	FVector ImpactDir = RelativeVelocity.GetSafeNormal();

	// Use drag equation: F = 0.5 * ρ * v² * Cd * A
	float AirDensity = 1.225f; // kg/m³ at sea level
	float DragForce = 0.5f * AirDensity * FMath::Square(ImpactSpeed * 0.01f) * DragCoeff * CrossSection;

	// Convert to acceleration (F = ma)
	float Acceleration = DragForce / Mass;

	// Scale for game feel
	float GameScale = ImpactToMovementScale;

	// Calculate 2D movement contribution
	FVector WorldMovement = ImpactDir * Acceleration * GameScale;

	// Convert to screen space (assuming Y is right, Z is up)
	FVector2D ScreenMovement;
	ScreenMovement.X = WorldMovement.Y; // Horizontal movement
	ScreenMovement.Y = -WorldMovement.Z; // Vertical movement (inverted for screen)

	// Apply motion quality scaling
	float MotionQuality = MotionData->GetMotionQuality();
	ScreenMovement *= FMath::Lerp(0.5f, 1.0f, MotionQuality);

	return ScreenMovement;
}

// --- Optional: Decay accumulated movement over time (call from your main tick if used) ---
void UOHPACManager::UpdateMovementDecay(float DeltaTime)
{
	// Decay accumulated movement over time
	if (AccumulatedMovementVector.Size() > KINDA_SMALL_NUMBER)
	{
		float DecayAmount = MovementDecayRate * DeltaTime;
		float CurrentSize = AccumulatedMovementVector.Size();
		float NewSize = FMath::Max(0.0f, CurrentSize - DecayAmount);

		if (NewSize > KINDA_SMALL_NUMBER)
		{
			AccumulatedMovementVector = AccumulatedMovementVector.GetSafeNormal() * NewSize;
		}
		else
		{
			AccumulatedMovementVector = FVector2D::ZeroVector;
		}
	}
}


// ---- SMOOTH BLEND LOGIC ----
void UOHPACManager::BlendPhysicalAnimationStrength(float Target, float BlendDuration)
{
	PACStrengthBlendState.StartBlend(
		GetPhysicalAnimationStrengthMultiplier(),
		Target,
		BlendDuration,
		GetWorld()->GetTimeSeconds()
	);
}

void UOHPACManager::ApplyImpactFlinch(float ImpactMagnitude)
{
    // === COMPREHENSIVE VALIDATION USING UTILITY PATTERNS ===
    if (!PhysicalAnimationComponent || !SkeletalMesh)
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("ApplyImpactFlinch: Missing required components"), true);
        }
        return;
    }
    
    // Validate impact magnitude using sophisticated bounds checking
    if (!FMath::IsFinite(ImpactMagnitude) || ImpactMagnitude < MinImpactFlinchThreshold)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ApplyImpactFlinch: Impact magnitude %.1f below threshold %.1f"), 
                ImpactMagnitude, MinImpactFlinchThreshold), false);
        }
        return;
    }
    
    // === CALCULATE FLINCH INTENSITY USING ENHANCED CURVES ===
    float FlinchIntensity = FMath::GetMappedRangeValueClamped(
        FVector2D(MinImpactFlinchThreshold, MaxImpactFlinchThreshold),
        FVector2D(0.1f, 1.0f),
        ImpactMagnitude
    );
    
    // Apply curve modulation if available
    if (ImpactResponseCurve)
    {
        float CurveValue = ImpactResponseCurve->GetFloatValue(FlinchIntensity);
        if (FMath::IsFinite(CurveValue) && CurveValue > 0.0f)
        {
            FlinchIntensity = CurveValue;
        }
    }
    
    // === LEVERAGE OHCOMBATUTILS FOR SOPHISTICATED CHARACTER ANALYSIS ===
    AActor* Owner = GetOwner();
    if (!Owner) return;
    
    // Use unified combat analysis for context-aware flinch application
    FOHCombatAnalysis MyCombatState = AnalyzeCharacterCombatState(Cast<ACharacter>(Owner));
    
    // Reduce flinch if character is in active attack to maintain combo flow
    if (MyCombatState.bIsAttacking && MyCombatState.AttackConfidence > 0.6f)
    {
        float AttackReduction = FMath::Lerp(1.0f, 0.3f, MyCombatState.AttackConfidence);
        FlinchIntensity *= AttackReduction;
        
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, 
                TEXT("Flinch reduced during active attack: Confidence=%.2f, Reduction=%.2f"),
                MyCombatState.AttackConfidence, AttackReduction);
        }
    }
    
    // === CALCULATE TARGET STRENGTH REDUCTION ===
    float CurrentStrength = GetPhysicalAnimationStrengthMultiplier();
    float TargetStrength = FMath::Clamp(
        MaxPhysicalAnimationStrength * (1.0f - FlinchIntensity * FlinchStrengthMultiplier),
        MinPhysicalAnimationStrength,
        CurrentStrength
    );
    
    // === ENHANCED BLEND DURATION CALCULATION ===
    float BlendDuration = FMath::GetMappedRangeValueClamped(
        FVector2D(0.1f, 1.0f),
        FVector2D(FlinchMinDuration, FlinchMaxDuration),
        FlinchIntensity
    );
    
    // === LEVERAGE OHLOCOMOTIONUTILS FOR MOVEMENT INTEGRATION ===
    // Check if character has movement component for enhanced coordination
    if (UOHMovementComponent* MovementComp = GetMovementComponent())
    {
        // Coordinate flinch with movement system for seamless integration
        if (MovementComp->bCombatPushbackActive)
        {
            // Extend flinch duration during active pushback for realism
            BlendDuration *= 1.3f;
            
            // Slightly reduce strength target to allow physics interaction
            TargetStrength *= 0.9f;
        }
    }
    
    // === APPLY PROGRESSIVE STRENGTH BLEND USING ENHANCED STATE MACHINE ===
    BlendPhysicalAnimationStrength(TargetStrength, BlendDuration);
    
    // === BONE-SPECIFIC FLINCH APPLICATION USING UTILITY FUNCTIONS ===
    TArray<FName> FlinchBones = GetBonesInCategory(EOHBoneCategory::UpperBodyBones);
    
    // Add core bones for comprehensive flinch response
    //TArray<FName> CoreBones = GetBonesInCategory(EOHBoneCategory::SpineBones);
    for (const FName& CoreBone : CoreBones)
    {
        FlinchBones.AddUnique(CoreBone);
    }
    
    // === APPLY TARGETED IMPULSES USING OHCOLLISIONUTILS ===
    for (const FName& BoneName : FlinchBones)
    {
        if (!IsBoneSimulating(BoneName)) continue;
        
        FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) continue;
        
        // === CALCULATE BONE-SPECIFIC FLINCH IMPULSE ===
        float BoneFlinchMultiplier = 1.0f;
        
        // Apply bone-specific flinch scaling
        FString BoneString = BoneName.ToString().ToLower();
        if (BoneString.Contains(TEXT("head")) || BoneString.Contains(TEXT("neck")))
        {
            BoneFlinchMultiplier = 0.5f; // Gentle head flinch
        }
        else if (BoneString.Contains(TEXT("shoulder")) || BoneString.Contains(TEXT("clavicle")))
        {
            BoneFlinchMultiplier = 1.2f; // Enhanced shoulder reaction
        }
        else if (BoneString.Contains(TEXT("spine")))
        {
            BoneFlinchMultiplier = 0.8f; // Moderate spine response
        }
        
        // === GENERATE REALISTIC FLINCH DIRECTION ===
        FVector FlinchDirection = FVector::ZeroVector;
        
        // Use random variation for natural flinch pattern
        FVector RandomComponent = FVector(
            FMath::RandRange(-1.0f, 1.0f),
            FMath::RandRange(-1.0f, 1.0f),
            FMath::RandRange(-0.3f, 0.3f)  // Limited vertical component
        ).GetSafeNormal();
        
        // Blend with away-from-impact direction if available
        if (LastImpactDirection.Size() > KINDA_SMALL_NUMBER)
        {
            FVector AwayFromImpact = -LastImpactDirection.GetSafeNormal();
            FlinchDirection = UOHLocomotionUtils::BlendDirectionalInput(
                RandomComponent, AwayFromImpact, 0.6f
            );
        }
        else
        {
            FlinchDirection = RandomComponent;
        }
        
        // === CALCULATE AND APPLY FLINCH IMPULSE ===
        float FlinchForce = ImpactMagnitude * FlinchIntensity * BoneFlinchMultiplier * 0.3f;
        FVector FlinchImpulse = FlinchDirection * FlinchForce;
        
        // Validate and apply impulse
        if (FlinchImpulse.ContainsNaN() && FlinchImpulse.Size() > 5.0f)
        {
            UOHCollisionUtils::ApplyImpulseToBone(SkeletalMesh, BoneName, FlinchImpulse, false);
        }
    }
    
    // === TRIGGER RECOVERY TIMER FOR AUTOMATIC RESTORATION ===
    GetWorld()->GetTimerManager().ClearTimer(FlinchRecoveryTimer);
    GetWorld()->GetTimerManager().SetTimer(
        FlinchRecoveryTimer,
        this,
        &UOHPACManager::RestorePhysicalAnimationStrengthSmooth,
        BlendDuration + FlinchRecoveryDelay,
        false
    );
    
    if (bVerboseLogging)
    {
        SafeLog(FString::Printf(
            TEXT("Impact Flinch Applied: Magnitude=%.1f, Intensity=%.2f, TargetStrength=%.2f, Duration=%.2f, Bones=%d"),
            ImpactMagnitude, FlinchIntensity, TargetStrength, BlendDuration, FlinchBones.Num()), false);
    }
    
    // === STORE IMPACT DATA FOR FUTURE REFERENCE ===
    LastImpactMagnitude = ImpactMagnitude;
    LastImpactTime = GetWorld()->GetTimeSeconds();
}
void UOHPACManager::RestorePhysicalAnimationStrengthSmooth()
{
    // === ENHANCED RESTORATION WITH CONTEXT AWARENESS ===
    if (!PhysicalAnimationComponent || !SkeletalMesh)
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("RestorePhysicalAnimationStrengthSmooth: Missing required components"), true);
        }
        return;
    }
    
    // === LEVERAGE OHCOMBATUTILS FOR CONTEXT-AWARE RESTORATION ===
    AActor* Owner = GetOwner();
    if (!Owner) return;
    
    // Use unified combat analysis for intelligent restoration timing
    FOHCombatAnalysis MyCombatState = AnalyzeCharacterCombatState(Cast<ACharacter>(Owner));
    
    // Calculate appropriate target strength based on current state
    float TargetStrength = MaxPhysicalAnimationStrength;
    float BlendDuration = ImpactFlinchBlendOutTime;
    
    // === COMBAT STATE MODULATION ===
    if (MyCombatState.bIsAttacking && MyCombatState.AttackConfidence > 0.4f)
    {
        // Slower restoration during active combat to maintain responsiveness
        BlendDuration *= 1.5f;
        
        // Slightly reduced target strength during combat for better physics interaction
        TargetStrength *= 0.95f;
        
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose,
                TEXT("Combat-aware restoration: Confidence=%.2f, Extended duration=%.2f"),
                MyCombatState.AttackConfidence, BlendDuration);
        }
    }
    
    // === MOVEMENT INTEGRATION USING UTILITY FUNCTIONS ===
    if (UOHMovementComponent* MovementComp = GetMovementComponent())
    {
        // Coordinate with movement system for seamless integration
        if (MovementComp->bCombatPushbackActive)
        {
            // Delay restoration if still experiencing combat pushback
            BlendDuration *= 1.3f;
            
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, VeryVerbose,
                    TEXT("Delaying restoration due to active combat pushback"));
            }
        }
    }
    
    // === APPLY ENHANCED PROGRESSIVE RESTORATION ===
    BlendPhysicalAnimationStrength(TargetStrength, BlendDuration);
    
    // === CLEAR IMPACT TRACKING DATA ===
    LastImpactDirection = FVector::ZeroVector;
    LastImpactMagnitude = 0.0f;
    
    if (bVerboseLogging)
    {
        SafeLog(FString::Printf(
            TEXT("PAC Strength Restoration: Target=%.2f, Duration=%.2f, Combat=%s"),
            TargetStrength, BlendDuration, 
            MyCombatState.bIsAttacking ? TEXT("Active") : TEXT("Inactive")), false);
    }
}

#pragma endregion

EOHBiologicalMaterial UOHPACManager::DetermineBoneMaterial(FName BoneName)
{
	// Map bone names to material types based on anatomy
	FString BoneStr = BoneName.ToString().ToLower();

	if (BoneStr.Contains("head") || BoneStr.Contains("skull"))
	{
		return EOHBiologicalMaterial::Bone; // Higher bone percentage
	}
	if (BoneStr.Contains("spine") || BoneStr.Contains("pelvis"))
	{
		return EOHBiologicalMaterial::Muscle; // Core muscles with bone
	}
	if (BoneStr.Contains("thigh") || BoneStr.Contains("calf"))
	{
		return EOHBiologicalMaterial::Muscle; // Large muscle groups
	}
	if (BoneStr.Contains("upperarm") || BoneStr.Contains("forearm"))
	{
		return EOHBiologicalMaterial::Muscle;
	}
	if (BoneStr.Contains("hand") || BoneStr.Contains("foot"))
	{
		return EOHBiologicalMaterial::Tendon; // More tendon/ligament
	}
	if (BoneStr.Contains("neck"))
	{
		return EOHBiologicalMaterial::Muscle;
	}
	return EOHBiologicalMaterial::Skin; // Default surface material
}

// Helper to determine bone shape
EOHShapeType UOHPACManager::DetermineBoneShape(FName BoneName)
{
	FString BoneStr = BoneName.ToString().ToLower();

	if (BoneStr.Contains("head") || BoneStr.Contains("skull"))
	{
		return EOHShapeType::Human_Head;
	}
	if (BoneStr.Contains("spine") || BoneStr.Contains("chest") ||
		BoneStr.Contains("pelvis") || BoneStr.Contains("abdomen"))
	{
		return EOHShapeType::Human_Torso;
	}
	if (BoneStr.Contains("arm") || BoneStr.Contains("leg") ||
		BoneStr.Contains("thigh") || BoneStr.Contains("calf"))
	{
		return EOHShapeType::Human_Limb;
	}
	if (BoneStr.Contains("hand") || BoneStr.Contains("foot"))
	{
		return EOHShapeType::Box; // Simplified shape for extremities
	}
	return EOHShapeType::Ellipsoid; // Default
}

// Helper to get bone bounds
FVector UOHPACManager::GetBoneBounds(FName BoneName) const
{
	if (!SkeletalMesh || !CachedPhysicsAsset)
	{
		return FVector(5.0f, 5.0f, 5.0f); // Default small bounds
	}

	// Get bounds from physics asset body setup
	for (const USkeletalBodySetup* BodySetup : CachedPhysicsAsset->SkeletalBodySetups)
	{
		if (BodySetup && BodySetup->BoneName == BoneName)
		{
			// Calculate bounds from aggregate geometry
			FBoxSphereBounds Bounds = BodySetup->AggGeom.CalcAABB(FTransform::Identity);
			return Bounds.BoxExtent;
		}
	}

	// Fallback: estimate from bone hierarchy
	int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
	if (BoneIndex != INDEX_NONE)
	{
		// Get parent bone to estimate length
		int32 ParentIndex = SkeletalMesh->GetBoneIndex(
			SkeletalMesh->GetParentBone(BoneName)
		);

		if (ParentIndex != INDEX_NONE)
		{
			FVector BonePos = SkeletalMesh->GetBoneLocation(BoneName);
			FVector ParentPos = SkeletalMesh->GetBoneLocation(
				SkeletalMesh->GetParentBone(BoneName)
			);

			float Length = FVector::Dist(BonePos, ParentPos);
			float Radius = Length * 0.15f; // Approximate radius as 15% of length

			return FVector(Radius, Radius, Length * 0.5f);
		}
	}

	return FVector(5.0f, 5.0f, 5.0f); // Default
}

// ============================================================================
// COMBAT TRACKING IMPLEMENTATION
// ============================================================================


void UOHPACManager::InitializeCombatChains()
{
    CombatChains.Empty();
    
    if (!SkeletalMesh)
    {
        SafeLog(TEXT("InitializeCombatChains: No skeletal mesh found"), true);
        return;
    }
    
    // Define limb chains
    struct FChainDefinition
    {
        FName RootBone;
        TArray<FName> ChainBones;
    };
    
    TArray<FChainDefinition> ChainDefinitions;
    
    // Right arm chain
    FChainDefinition RightArm;
    RightArm.RootBone = TEXT("hand_r");
    RightArm.ChainBones = { TEXT("hand_r"), TEXT("lowerarm_r"), TEXT("upperarm_r") };
    ChainDefinitions.Add(RightArm);
    
    // Left arm chain
    FChainDefinition LeftArm;
    LeftArm.RootBone = TEXT("hand_l");
    LeftArm.ChainBones = { TEXT("hand_l"), TEXT("lowerarm_l"), TEXT("upperarm_l") };
    ChainDefinitions.Add(LeftArm);
    
    // Right leg chain
    FChainDefinition RightLeg;
    RightLeg.RootBone = TEXT("foot_r");
    RightLeg.ChainBones = { TEXT("foot_r"), TEXT("calf_r"), TEXT("thigh_r") };
    ChainDefinitions.Add(RightLeg);
    
    // Left leg chain
    FChainDefinition LeftLeg;
    LeftLeg.RootBone = TEXT("foot_l");
    LeftLeg.ChainBones = { TEXT("foot_l"), TEXT("calf_l"), TEXT("thigh_l") };
    ChainDefinitions.Add(LeftLeg);
    
    // Get current time
    float CurrentTime = GetWorld()->GetTimeSeconds();
    
    // Create chain data for each definition
    for (const FChainDefinition& ChainDef : ChainDefinitions)
    {
        // Verify all bones exist
        bool bAllBonesValid = true;
        for (const FName& BoneName : ChainDef.ChainBones)
        {
            int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
            if (BoneIndex == INDEX_NONE)
            {
                SafeLog(FString::Printf(TEXT("InitializeCombatChains: Bone %s not found"), 
                    *BoneName.ToString()), true);
                bAllBonesValid = false;
                break;
            }
        }
        
        if (!bAllBonesValid)
        {
            SafeLog(FString::Printf(TEXT("Skipping chain %s due to missing bones"), 
                *ChainDef.RootBone.ToString()), true);
            continue;
        }
        
        // Create the chain data
        FOHCombatChainData NewChain;
        NewChain.RootBone = ChainDef.RootBone;
        NewChain.ChainBones = ChainDef.ChainBones;
        
        // Initialize arrays
        int32 NumBones = NewChain.ChainBones.Num();
        NewChain.BoneMasses.SetNum(NumBones);
        NewChain.BoneRadii.SetNum(NumBones);
        NewChain.CurrentFrameSamples.SetNum(NumBones);
        
        // Set up each bone in the chain
        for (int32 i = 0; i < NumBones; i++)
        {
            const FName& BoneName = NewChain.ChainBones[i];
            
            // Estimate mass and radius
            NewChain.BoneMasses[i] = EstimateBoneMass(BoneName);
            NewChain.BoneRadii[i] = EstimateBoneRadius(BoneName);
            
            // Initialize motion data
            FOHBoneMotionData MotionData;
            MotionData.Initialize(BoneName);
            
            // Create initial sample
            FOHMotionFrameSample InitialSample;
            InitialSample.DeltaTime = 0.016f;
            InitialSample.TimeStamp = CurrentTime;
            
            // Get current transform data
            FTransform BoneWorldTransform;
            FVector ComponentVelocity = FVector::ZeroVector;
            
            if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
            {
                BoneWorldTransform = Body->GetUnrealWorldTransform();
                InitialSample.WorldPosition = BoneWorldTransform.GetLocation();
                InitialSample.WorldRotation = BoneWorldTransform.GetRotation();
                InitialSample.WorldLinearVelocity = Body->GetUnrealWorldVelocity();
                InitialSample.AngularVelocity = Body->GetUnrealWorldAngularVelocityInRadians();
            }
            else
            {
                // Use skeletal mesh transform
                int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
                BoneWorldTransform = SkeletalMesh->GetBoneTransform(BoneIndex);
                InitialSample.WorldPosition = BoneWorldTransform.GetLocation();
                InitialSample.WorldRotation = BoneWorldTransform.GetRotation();
                InitialSample.WorldLinearVelocity = FVector::ZeroVector;
                InitialSample.AngularVelocity = FVector::ZeroVector;
            }
            
            // Set component transform
            InitialSample.ComponentTransform = SkeletalMesh->GetComponentTransform();
            InitialSample.ReferenceTransform = InitialSample.ComponentTransform;
            InitialSample.ReferenceVelocity = ComponentVelocity;
            
            // Calculate local velocities
            InitialSample.LocalLinearVelocity = InitialSample.ComponentTransform.InverseTransformVector(InitialSample.WorldLinearVelocity);
            InitialSample.ComponentLinearVelocity = ComponentVelocity;
            
            // Calculate speeds
            InitialSample.WorldSpeed = InitialSample.WorldLinearVelocity.Size();
            InitialSample.LocalSpeed = InitialSample.LocalLinearVelocity.Size();
            InitialSample.AngularSpeed = InitialSample.AngularVelocity.Size();
            
            // Initialize accelerations to zero for first frame
            InitialSample.WorldLinearAcceleration = FVector::ZeroVector;
            InitialSample.LocalLinearAcceleration = FVector::ZeroVector;
            InitialSample.AngularAcceleration = FVector::ZeroVector;
            
            // Add the sample using AddMotionSample
            MotionData.AddMotionSample(
                InitialSample.WorldPosition,
                InitialSample.WorldRotation,
                ComponentVelocity,
                InitialSample.ReferenceVelocity,
                InitialSample.ComponentTransform,
                InitialSample.DeltaTime,
                InitialSample.TimeStamp
            );
            
            // Store in chain motion data
            NewChain.ChainMotionData.Add(BoneName, MotionData);
            
            // Also store in current frame samples
            NewChain.CurrentFrameSamples[i] = InitialSample;
            
            // Ensure bone is in simulatable list
            if (!SimulatableBones.Contains(BoneName))
            {
                SimulatableBones.Add(BoneName);
            }
        }
        
        // Initialize chain metrics
        NewChain.UpdateChainMetrics(CurrentTime);
        
        // Add to combat chains map
        CombatChains.Add(ChainDef.RootBone, NewChain);
        
        SafeLog(FString::Printf(TEXT("Combat chain initialized: %s with %d bones"), 
            *ChainDef.RootBone.ToString(), NumBones));
    }
    
    // Initialize combat tracking
    CombatHitTimestamps.Empty();
    
    SafeLog(FString::Printf(TEXT("InitializeCombatChains complete: %d chains initialized"), 
        CombatChains.Num()));
}

void UOHPACManager::UpdateChainMotionData(FOHCombatChainData& Chain, float DeltaTime)
{
    if (Chain.ChainBones.Num() == 0 || !SkeletalMesh || !GetWorld()) return;
    
    float CurrentTime = GetWorld()->GetTimeSeconds();
    
    // === ENHANCED: SYSTEMATIC REFERENCE FRAME CONTEXT ACQUISITION ===
    FVector ComponentVelocity = FVector::ZeroVector;
    FVector ReferenceFrameVelocity = FVector::ZeroVector;
    FTransform ComponentTransform = SkeletalMesh->GetComponentTransform();
    
    // Acquire character locomotion context for reference frame calculations
    if (ACharacter* Character = Cast<ACharacter>(GetOwner()))
    {
        if (UCharacterMovementComponent* CharMove = Character->GetCharacterMovement())
        {
            ComponentVelocity = CharMove->Velocity;
        }
    }
    
    // === CRITICAL: ACQUIRE REFERENCE FRAME VELOCITY BASED ON CHAIN CONFIGURATION ===
    if (!Chain.MotionReferenceFrameBone.IsNone())
    {
        if (const FOHBoneMotionData* RefData = BoneMotionMap.Find(Chain.MotionReferenceFrameBone))
        {
            ReferenceFrameVelocity = RefData->GetVelocity(EOHReferenceSpace::WorldSpace);
        }
        else if (SkeletalMesh->GetBoneIndex(Chain.MotionReferenceFrameBone) != INDEX_NONE)
        {
            // Fallback to skeletal mesh bone velocity calculation
            FVector RefBoneLocation = SkeletalMesh->GetBoneLocation(Chain.MotionReferenceFrameBone);
            ReferenceFrameVelocity = ComponentVelocity; // Use component velocity as approximation
        }
    }
    
    // === MODERNIZED: SYSTEMATIC MULTI-SPACE MOTION SYNCHRONIZATION ===
    Chain.CurrentFrameSamples.SetNum(Chain.ChainBones.Num());
    
    for (int32 i = 0; i < Chain.ChainBones.Num(); i++)
    {
        const FName& BoneName = Chain.ChainBones[i];
        
        // Direct reference to chain motion data (no unnecessary copying)
        FOHBoneMotionData* ChainMotionData = Chain.ChainMotionData.Find(BoneName);
        if (!ChainMotionData) continue;
        
        // === PRIORITY 1: SYNCHRONIZE FROM MAIN MOTION MAP ===
        if (const FOHBoneMotionData* MainMotionData = BoneMotionMap.Find(BoneName))
        {
            // Efficient motion data synchronization with validation
            if (MainMotionData->GetHistoryDepth() > 0 && MainMotionData->GetLatestSample())
            {
                *ChainMotionData = *MainMotionData;
                
                // === CRITICAL: RECALCULATE REFERENCE SPACES BASED ON CHAIN CONFIGURATION ===
                if (FOHMotionFrameSample* MutableSample = const_cast<FOHMotionFrameSample*>(ChainMotionData->GetLatestSample()))
                {
                    // Update reference space calculations with chain-specific context
                    MutableSample->RecalculateReferenceSpaces(ComponentTransform, ReferenceFrameVelocity);
                    Chain.CurrentFrameSamples[i] = *MutableSample;
                }
            }
        }
        // === PRIORITY 2: ENHANCED MANUAL MOTION TRACKING WITH MULTI-SPACE CALCULATION ===
        else
        {
            FVector WorldPos = FVector::ZeroVector;
            FQuat WorldRot = FQuat::Identity;
            FVector WorldVelocity = FVector::ZeroVector;
            FVector AngularVelocity = FVector::ZeroVector;
            
            // === ROBUST STATE ACQUISITION WITH PHYSICS BODY PRIORITY ===
            if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
            {
                // Physics body provides most accurate kinematic data
                FTransform WorldTransform = Body->GetUnrealWorldTransform();
                WorldPos = WorldTransform.GetLocation();
                WorldRot = WorldTransform.GetRotation();
                WorldVelocity = Body->GetUnrealWorldVelocity();
                AngularVelocity = Body->GetUnrealWorldAngularVelocityInRadians();
            }
            else
            {
                // Fallback to skeletal mesh transform with enhanced velocity calculation
                int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
                if (BoneIndex != INDEX_NONE)
                {
                    FTransform WorldTransform = SkeletalMesh->GetBoneTransform(BoneIndex);
                    WorldPos = WorldTransform.GetLocation();
                    WorldRot = WorldTransform.GetRotation();
                    
                    // === ENHANCED: VELOCITY CALCULATION WITH VALIDATION ===
                    if (ChainMotionData->GetHistoryDepth() > 0)
                    {
                        if (const FOHMotionFrameSample* PrevSample = ChainMotionData->GetLatestSample())
                        {
                            float TimeDelta = CurrentTime - PrevSample->TimeStamp;
                            
                            if (TimeDelta > KINDA_SMALL_NUMBER && !PrevSample->WorldPosition.ContainsNaN())
                            {
                                // Linear velocity calculation with NaN protection
                                FVector PosDelta = WorldPos - PrevSample->WorldPosition;
                                if (!PosDelta.ContainsNaN())
                                {
                                    WorldVelocity = PosDelta / TimeDelta;
                                }
                                
                                // Angular velocity calculation with enhanced validation
                                if (!PrevSample->WorldRotation.ContainsNaN())
                                {
                                    FQuat QuatDelta = WorldRot * PrevSample->WorldRotation.Inverse();
                                    FVector Axis;
                                    float Angle;
                                    QuatDelta.ToAxisAndAngle(Axis, Angle);
                                    
                                    if (!FMath::IsNaN(Angle) && !Axis.ContainsNaN())
                                    {
                                        // Normalize angle to [-π, π] range
                                        while (Angle > PI) Angle -= 2.0f * PI;
                                        while (Angle < -PI) Angle += 2.0f * PI;
                                        
                                        AngularVelocity = Axis * (Angle / TimeDelta);
                                    }
                                }
                            }
                            else
                            {
                                // Use previous sample velocities if time delta is invalid
                                WorldVelocity = PrevSample->WorldLinearVelocity;
                                AngularVelocity = PrevSample->AngularVelocity;
                            }
                        }
                    }
                }
            }
            
            // === ENHANCED: MULTI-SPACE MOTION SAMPLE CREATION ===
            if (!WorldPos.ContainsNaN() && !WorldRot.ContainsNaN() && 
                !WorldVelocity.ContainsNaN() && !AngularVelocity.ContainsNaN())
            {
                // === CRITICAL: COMPREHENSIVE MOTION SAMPLE ADDITION WITH CHAIN CONTEXT ===
                ChainMotionData->AddMotionSample(
                    WorldPos,
                    WorldRot,
                    ComponentVelocity,
                    ReferenceFrameVelocity, // Chain-specific reference velocity
                    ComponentTransform,
                    DeltaTime,
                    CurrentTime
                );
                
                // Populate current frame sample with multi-space data
                if (const FOHMotionFrameSample* NewSample = ChainMotionData->GetLatestSample())
                {
                    Chain.CurrentFrameSamples[i] = *NewSample;
                }
            }
        }
    }
    
    // === ENHANCED: REFERENCE SPACE-AWARE CHAIN METRICS UPDATE ===
    Chain.UpdateChainMetrics(CurrentTime);
    
    // === CRITICAL: UPDATE MOTION COHERENCE BASED ON CHAIN ANALYSIS SPACE ===
    Chain.UpdateMotionCoherence();
}


void UOHPACManager::UpdateCombatChainStates(float DeltaTime)
{
    if (!SkeletalMesh || !bEnableCollisionImpactSystem || CombatChains.Num() == 0) return;
    
    float CurrentTime = GetWorld()->GetTimeSeconds();
    
    // === MODERNIZED: SYSTEMATIC CHAIN STATE PROCESSING ===
    for (auto& ChainPair : CombatChains)
    {
        FOHCombatChainData& Chain = ChainPair.Value;
        
        // === STEP 1: ENHANCED MOTION DATA SYNCHRONIZATION ===
        UpdateChainMotionData(Chain, DeltaTime);
        
        // === STEP 2: REFERENCE SPACE-AWARE METRICS CALCULATION ===
        Chain.UpdateChainMetrics(CurrentTime);
        
        // === STEP 3: MODERNIZED ATTACK PATTERN ANALYSIS ===
        AnalyzeChainAttackPattern(Chain, DeltaTime);
        
        // === STEP 4: CONDITIONAL TRAJECTORY PREDICTION WITH CHAIN THRESHOLDS ===
        if (Chain.AttackConfidence > MinAttackConfidenceThreshold && Chain.HasSignificantMotion())
        {
            UpdateChainTrajectory(Chain, DeltaTime);
        }
        else
        {
            // Clear trajectory data for inactive chains
            Chain.PredictedTrajectory.Empty();
            Chain.BezierControlPoints.Empty();
            Chain.TrajectoryConfidence = 0.0f;
        }
        
        // === STEP 5: ENHANCED ATTACK STATE TRACKING WITH REFERENCE SPACE CONTEXT ===
        if (Chain.bIsAttacking)
        {
            Chain.TimeInAttack += DeltaTime;
            
            // Attack initiation logging with reference space metrics
            if (Chain.TimeInAttack < DeltaTime * 2.0f) // Just started
            {
                if (bVerboseLogging)
                {
                    UE_LOG(LogPACManager, Warning,
                        TEXT("Chain Attack Initiated [%s] - Space: %s, Confidence: %.2f, Speed: %.1f, Energy: %.1f, Quality: %.2f"),
                        *Chain.RootBone.ToString(),
                        *UEnum::GetValueAsString(Chain.MotionAnalysisSpace),
                        Chain.AttackConfidence,
                        Chain.GetEffectiveMotionSpeed(),
                        Chain.ChainKineticEnergy,
                        Chain.ChainMotionQuality
                    );
                }
            }
        }
        else
        {
            // Reset attack tracking data
            Chain.TimeInAttack = 0.0f;
            Chain.PredictedTrajectory.Empty();
            Chain.BezierControlPoints.Empty();
        }
    }
}


void UOHPACManager::UpdateChainTrajectory(FOHCombatChainData& Chain, float DeltaTime)
{
    // === ENHANCED: REFERENCE SPACE-AWARE INITIALIZATION ===
    Chain.PredictedTrajectory.Empty();
    Chain.BezierControlPoints.Empty();
    Chain.TrajectoryConfidence = 0.0f;
    
    FName StrikingBone = Chain.GetPrimaryStrikingBone();
    const FOHBoneMotionData* MotionData = Chain.GetBoneMotionData(StrikingBone);
    
    // === COMPREHENSIVE VALIDATION WITH REFERENCE SPACE CONTEXT ===
    if (!MotionData)
    {
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, 
                TEXT("Trajectory prediction failed for chain '%s': No motion data for striking bone '%s'"),
                *Chain.RootBone.ToString(), *StrikingBone.ToString());
        }
        return;
    }
    
    if (MotionData->GetHistoryDepth() < 3)
    {
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, 
                TEXT("Trajectory prediction skipped for chain '%s': Insufficient history depth (%d < 3)"),
                *Chain.RootBone.ToString(), MotionData->GetHistoryDepth());
        }
        return;
    }
    
    if (!Chain.HasSignificantMotion())
    {
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, 
                TEXT("Trajectory prediction skipped for chain '%s': No significant motion (Speed=%.1f < Threshold=%.1f)"),
                *Chain.RootBone.ToString(), Chain.GetEffectiveMotionSpeed(), Chain.MotionDetectionThreshold);
        }
        return;
    }
    
    float MotionQuality = MotionData->GetMotionQuality(Chain.MotionAnalysisSpace);
    if (MotionQuality < PredictionQualityThreshold)
    {
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, 
                TEXT("Trajectory prediction skipped for chain '%s': Low motion quality (%.2f < %.2f) in space %s"),
                *Chain.RootBone.ToString(), MotionQuality, PredictionQualityThreshold,
                *UEnum::GetValueAsString(Chain.MotionAnalysisSpace));
        }
        return;
    }
    
    // === MODERNIZED: PREDICTION PARAMETERS WITH REFERENCE SPACE OPTIMIZATION ===
    const float PredictionTime = FMath::GetMappedRangeValueClamped(
        FVector2D(0.3f, 1.0f),           // Confidence range
        FVector2D(0.2f, 0.4f),           // Prediction time range
        Chain.AttackConfidence
    );
    const int32 NumSamples = 20;
    
    if (bVerboseLogging)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("Starting trajectory prediction for chain '%s': PredictTime=%.2fs, Samples=%d, Quality=%.2f, Space=%s"),
            *Chain.RootBone.ToString(), PredictionTime, NumSamples, MotionQuality,
            *UEnum::GetValueAsString(Chain.MotionAnalysisSpace));
    }
    
    // === ENHANCED: MULTI-MODEL PREDICTION WITH REFERENCE SPACE INTEGRATION ===
    TArray<TArray<FVector>> ModelPredictions;
    TArray<float> ModelWeights;
    TArray<FString> ModelNames; // For debugging
    
    // === MODEL 1: ENHANCED KALMAN FILTER WITH REFERENCE SPACE VALIDATION ===
    if (MotionData->GetHistoryDepth() >= 5 && Chain.MotionCoherence > 0.3f)
    {
        TArray<FVector> KalmanPath;
        ModelNames.Add(TEXT("Kalman"));
        
        // Initialize Kalman state from motion data in configured reference space
        FOHKalmanState KalmanState;
        KalmanState.Position = MotionData->GetCurrentPosition();
        KalmanState.Velocity = MotionData->GetVelocity(Chain.MotionAnalysisSpace);
        
        // Comprehensive initial state validation
        if (!KalmanState.Position.ContainsNaN() && !KalmanState.Velocity.ContainsNaN() &&
            KalmanState.Position.Size() < 100000.0f && KalmanState.Velocity.Size() < 10000.0f)
        {
            float TimeStep = PredictionTime / NumSamples;
            bool bKalmanSuccess = true;
            
            for (int32 i = 0; i <= NumSamples; i++)
            {
                if (i > 0)
                {
                    // Reference space-aware prediction measurement
                    float PredictAheadTime = i * TimeStep;
                    FVector PredictedMeasurement = MotionData->PredictFuturePosition(
                        PredictAheadTime, 
                        Chain.MotionAnalysisSpace,
                        true
                    );
                    
                    // Validate prediction measurement
                    if (!PredictedMeasurement.ContainsNaN() && PredictedMeasurement.Size() < 100000.0f)
                    {
                        KalmanState = UOHAlgoUtils::PredictKalmanState(
                            KalmanState,
                            PredictedMeasurement,
                            TimeStep
                        );
                    }
                    else
                    {
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Kalman prediction measurement invalid at step %d: %s"), i,
                                PredictedMeasurement.ContainsNaN() ? TEXT("NaN") : TEXT("Too Large"));
                        }
                        bKalmanSuccess = false;
                        break;
                    }
                }
                
                // Validate Kalman state before adding
                if (!KalmanState.Position.ContainsNaN() && KalmanState.Position.Size() < 100000.0f)
                {
                    KalmanPath.Add(KalmanState.Position);
                }
                else
                {
                    // Fallback to previous valid position
                    if (KalmanPath.Num() > 0)
                    {
                        KalmanPath.Add(KalmanPath.Last());
                    }
                    else
                    {
                        KalmanPath.Add(MotionData->GetCurrentPosition());
                    }
                }
            }
            
            if (bKalmanSuccess && KalmanPath.Num() == NumSamples + 1)
            {
                ModelPredictions.Add(KalmanPath);
                // Weight based on motion quality and coherence
                float KalmanWeight = MotionQuality * Chain.MotionCoherence * 0.9f;
                ModelWeights.Add(KalmanWeight);
                
                if (bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Kalman model added: Weight=%.3f, Samples=%d"), KalmanWeight, KalmanPath.Num());
                }
            }
            else if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, VeryVerbose,
                    TEXT("Kalman model failed: Success=%s, PathSamples=%d"), 
                    bKalmanSuccess ? TEXT("True") : TEXT("False"), KalmanPath.Num());
            }
        }
        else if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose,
                TEXT("Kalman model skipped: Invalid initial state - Pos NaN=%s, Vel NaN=%s, PosSize=%.1f, VelSize=%.1f"),
                KalmanState.Position.ContainsNaN() ? TEXT("True") : TEXT("False"),
                KalmanState.Velocity.ContainsNaN() ? TEXT("True") : TEXT("False"),
                KalmanState.Position.Size(), KalmanState.Velocity.Size());
        }
    }
    else if (bVerboseLogging)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("Kalman model skipped: HistoryDepth=%d (need ≥5), Coherence=%.2f (need >0.3)"),
            MotionData->GetHistoryDepth(), Chain.MotionCoherence);
    }
    
    // === MODEL 2: ENHANCED RK4 PHYSICS INTEGRATION WITH REFERENCE SPACE MOMENTUM ===
    if (Chain.ChainKineticEnergy > 500.0f && Chain.ChainTotalMass > 0.0f)
    {
        TArray<FVector> RK4Path;
        ModelNames.Add(TEXT("RK4"));
        
        // Initialize RK4 state using reference space momentum
        FRK4State PhysicsState;
        PhysicsState.Position = Chain.ChainCenterOfMass;
        PhysicsState.Velocity = Chain.GetChainMomentum(Chain.MotionAnalysisSpace) / Chain.ChainTotalMass;
        PhysicsState.Rotation = FQuat::Identity;
        PhysicsState.AngularVelocity = FVector::ZeroVector;
        
        // Comprehensive physics state validation
        if (!PhysicsState.Position.ContainsNaN() && !PhysicsState.Velocity.ContainsNaN() &&
            PhysicsState.Position.Size() < 100000.0f && PhysicsState.Velocity.Size() < 10000.0f)
        {
            float TimeStep = PredictionTime / NumSamples;
            
            // Enhanced drag coefficient based on chain properties and reference space
            float BaseDragCoeff = 0.47f * 0.01f; // Sphere drag coefficient scaled
            float CurvatureModifier = 1.0f + FMath::Clamp(Chain.AttackCurvature * 0.5f, 0.0f, 0.3f);
            float ReferenceSpaceModifier = 1.0f;
            
            // Adjust drag based on reference space (local space may have different effective drag)
            switch (Chain.MotionAnalysisSpace)
            {
                case EOHReferenceSpace::LocalSpace:
                    ReferenceSpaceModifier = 0.8f; // Reduced drag for local space motion
                    break;
                case EOHReferenceSpace::ComponentSpace:
                    ReferenceSpaceModifier = 0.9f; // Slightly reduced drag
                    break;
                default:
                    break; // World space uses base modifier
            }
            
            float DragCoeff = BaseDragCoeff * CurvatureModifier * ReferenceSpaceModifier;
            bool bRK4Success = true;
            
            for (int32 i = 0; i <= NumSamples; i++)
            {
                if (!PhysicsState.Position.ContainsNaN() && PhysicsState.Position.Size() < 100000.0f)
                {
                    RK4Path.Add(PhysicsState.Position);
                }
                else
                {
                    if (bVerboseLogging)
                    {
                        UE_LOG(LogPACManager, VeryVerbose,
                            TEXT("RK4 position invalid at step %d: NaN=%s, Size=%.1f"), i,
                            PhysicsState.Position.ContainsNaN() ? TEXT("True") : TEXT("False"),
                            PhysicsState.Position.Size());
                    }
                    bRK4Success = false;
                    break;
                }
                
                if (i < NumSamples)
                {
                    // Enhanced drag calculation with mass consideration
                    FVector DragAcceleration = FVector::ZeroVector;
                    if (!PhysicsState.Velocity.IsNearlyZero(1.0f)) // Small threshold for nearly zero
                    {
                        float VelocityMagnitude = PhysicsState.Velocity.Size();
                        if (VelocityMagnitude > 0.0f && Chain.ChainTotalMass > 0.0f)
                        {
                            DragAcceleration = -PhysicsState.Velocity.GetSafeNormal() * 
                                              DragCoeff * VelocityMagnitude * VelocityMagnitude / Chain.ChainTotalMass;
                        }
                    }
                    
                    // Validate drag acceleration
                    if (!DragAcceleration.ContainsNaN() && DragAcceleration.Size() < 50000.0f)
                    {
                        PhysicsState = UOHAlgoUtils::IntegrateRK4State(
                            PhysicsState,
                            DragAcceleration,
                            TimeStep
                        );
                    }
                    else
                    {
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("RK4 drag acceleration invalid at step %d: NaN=%s, Size=%.1f"), i,
                                DragAcceleration.ContainsNaN() ? TEXT("True") : TEXT("False"),
                                DragAcceleration.Size());
                        }
                        bRK4Success = false;
                        break;
                    }
                }
            }
            
            if (bRK4Success && RK4Path.Num() == NumSamples + 1)
            {
                ModelPredictions.Add(RK4Path);
                // Weight based on kinetic energy and motion quality
                float EnergyFactor = FMath::Clamp(Chain.ChainKineticEnergy / 3000.0f, 0.2f, 0.8f);
                float CoherenceFactor = 0.5f + Chain.MotionCoherence * 0.5f;
                float RK4Weight = EnergyFactor * CoherenceFactor;
                ModelWeights.Add(RK4Weight);
                
                if (bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("RK4 model added: Weight=%.3f, DragCoeff=%.6f, Energy=%.1f"), 
                        RK4Weight, DragCoeff, Chain.ChainKineticEnergy);
                }
            }
            else if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, VeryVerbose,
                    TEXT("RK4 model failed: Success=%s, PathSamples=%d"), 
                    bRK4Success ? TEXT("True") : TEXT("False"), RK4Path.Num());
            }
        }
        else if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose,
                TEXT("RK4 model skipped: Invalid physics state - Pos NaN=%s, Vel NaN=%s, PosSize=%.1f, VelSize=%.1f"),
                PhysicsState.Position.ContainsNaN() ? TEXT("True") : TEXT("False"),
                PhysicsState.Velocity.ContainsNaN() ? TEXT("True") : TEXT("False"),
                PhysicsState.Position.Size(), PhysicsState.Velocity.Size());
        }
    }
    else if (bVerboseLogging)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("RK4 model skipped: KineticEnergy=%.1f (need >500), TotalMass=%.2f (need >0)"),
            Chain.ChainKineticEnergy, Chain.ChainTotalMass);
    }
    
    // === MODEL 3: ENHANCED BEZIER CURVE FITTING WITH REFERENCE SPACE CURVATURE ===
    float Curvature = UOHAlgoUtils::EstimateCurvatureFromBoneData(*MotionData);
    
    if (Curvature > 0.1f && !FMath::IsNaN(Curvature) && FMath::IsFinite(Curvature))
    {
        if (Curvature > 0.25f)
        {
            // Cubic Bezier for highly curved attacks
            Chain.BezierControlPoints = UOHAlgoUtils::GetCubicBezierControlPointsFromBoneData(
                *MotionData,
                PredictionTime
            );
            
            if (Chain.BezierControlPoints.Num() >= 4)
            {
                TArray<FVector> BezierPath;
                ModelNames.Add(TEXT("CubicBezier"));
                
                // Validate all control points
                bool bValidControlPoints = true;
                for (int32 PointIdx = 0; PointIdx < Chain.BezierControlPoints.Num(); PointIdx++)
                {
                    const FVector& Point = Chain.BezierControlPoints[PointIdx];
                    if (Point.ContainsNaN() || Point.Size() > 100000.0f)
                    {
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Invalid Bezier control point %d: NaN=%s, Size=%.1f"), PointIdx,
                                Point.ContainsNaN() ? TEXT("True") : TEXT("False"), Point.Size());
                        }
                        bValidControlPoints = false;
                        break;
                    }
                }
                
                if (bValidControlPoints)
                {
                    bool bBezierSuccess = true;
                    for (int32 i = 0; i <= NumSamples; i++)
                    {
                        float t = static_cast<float>(i) / static_cast<float>(NumSamples);
                        FVector Point = UOHAlgoUtils::SampleBezierCubic(
                            Chain.BezierControlPoints[0], 
                            Chain.BezierControlPoints[1],
                            Chain.BezierControlPoints[2], 
                            Chain.BezierControlPoints[3], 
                            t
                        );
                        
                        if (!Point.ContainsNaN() && Point.Size() < 100000.0f)
                        {
                            BezierPath.Add(Point);
                        }
                        else
                        {
                            if (bVerboseLogging)
                            {
                                UE_LOG(LogPACManager, VeryVerbose,
                                    TEXT("Invalid Bezier sample at t=%.2f: NaN=%s, Size=%.1f"), t,
                                    Point.ContainsNaN() ? TEXT("True") : TEXT("False"), Point.Size());
                            }
                            bBezierSuccess = false;
                            break;
                        }
                    }
                    
                    if (bBezierSuccess && BezierPath.Num() == NumSamples + 1)
                    {
                        ModelPredictions.Add(BezierPath);
                        float BezierWeight = 0.9f * Chain.MotionCoherence;
                        ModelWeights.Add(BezierWeight);
                        
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Cubic Bezier model added: Weight=%.3f, Curvature=%.3f"), 
                                BezierWeight, Curvature);
                        }
                    }
                }
            }
        }
        else
        {
            // Quadratic Bezier for moderately curved attacks
            Chain.BezierControlPoints = UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(
                *MotionData,
                PredictionTime
            );
            
            if (Chain.BezierControlPoints.Num() >= 3)
            {
                TArray<FVector> BezierPath;
                ModelNames.Add(TEXT("QuadraticBezier"));
                
                // Validate all control points
                bool bValidControlPoints = true;
                for (int32 PointIdx = 0; PointIdx < 3; PointIdx++)
                {
                    const FVector& Point = Chain.BezierControlPoints[PointIdx];
                    if (Point.ContainsNaN() || Point.Size() > 100000.0f)
                    {
                        bValidControlPoints = false;
                        break;
                    }
                }
                
                if (bValidControlPoints)
                {
                    bool bBezierSuccess = true;
                    for (int32 i = 0; i <= NumSamples; i++)
                    {
                        float t = static_cast<float>(i) / static_cast<float>(NumSamples);
                        FVector Point = UOHAlgoUtils::SampleBezierQuadratic(
                            Chain.BezierControlPoints[0], 
                            Chain.BezierControlPoints[1], 
                            Chain.BezierControlPoints[2], 
                            t
                        );
                        
                        if (!Point.ContainsNaN() && Point.Size() < 100000.0f)
                        {
                            BezierPath.Add(Point);
                        }
                        else
                        {
                            bBezierSuccess = false;
                            break;
                        }
                    }
                    
                    if (bBezierSuccess && BezierPath.Num() == NumSamples + 1)
                    {
                        ModelPredictions.Add(BezierPath);
                        float BezierWeight = 0.85f * Chain.MotionCoherence;
                        ModelWeights.Add(BezierWeight);
                        
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Quadratic Bezier model added: Weight=%.3f, Curvature=%.3f"), 
                                BezierWeight, Curvature);
                        }
                    }
                }
            }
        }
    }
    else if (bVerboseLogging && Curvature <= 0.1f)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("Bezier model skipped: Low curvature (%.3f ≤ 0.1)"), Curvature);
    }
    
    // === MODEL 4: ENHANCED SPRING MODEL FOR TENTATIVE ATTACKS ===
    if (Chain.AttackConfidence < 0.7f && Chain.AttackConfidence > 0.3f)
    {
        TArray<FVector> SpringPath;
        ModelNames.Add(TEXT("Spring"));
        
        const FOHMotionFrameSample* CurrentSamplePtr = MotionData->GetLatestSample();
        if (CurrentSamplePtr && !CurrentSamplePtr->WorldPosition.ContainsNaN())
        {
            FOHMotionFrameSample SpringSample = *CurrentSamplePtr;
            
            // Reference space-aware target position calculation
            FVector CurrentPos = MotionData->GetCurrentPosition();
            FVector AttackVelocity = MotionData->GetVelocity(Chain.MotionAnalysisSpace);
            
            if (!CurrentPos.ContainsNaN() && !AttackVelocity.ContainsNaN() && 
                CurrentPos.Size() < 100000.0f && AttackVelocity.Size() < 10000.0f)
            {
                FVector TargetPos = CurrentPos + AttackVelocity.GetSafeNormal() * 
                                   (PunchThroughDistance * Chain.AttackConfidence);
                
                if (!TargetPos.ContainsNaN() && TargetPos.Size() < 100000.0f)
                {
                    // Adaptive spring frequency based on attack confidence and reference space
                    float BaseSpringFreq = FMath::Lerp(10.0f, 20.0f, Chain.AttackConfidence);
                    
                    // Adjust spring frequency based on reference space characteristics
                    float SpaceModifier = 1.0f;
                    switch (Chain.MotionAnalysisSpace)
                    {
                        case EOHReferenceSpace::LocalSpace:
                            SpaceModifier = 1.2f; // Higher frequency for local space
                            break;
                        case EOHReferenceSpace::ComponentSpace:
                            SpaceModifier = 0.9f; // Lower frequency for component space
                            break;
                        default:
                            break;
                    }
                    
                    float SpringFreq = BaseSpringFreq * SpaceModifier;
                    float TimeStep = PredictionTime / NumSamples;
                    bool bSpringSuccess = true;
                    
                    for (int32 i = 0; i <= NumSamples; i++)
                    {
                        if (!SpringSample.WorldPosition.ContainsNaN() && 
                            SpringSample.WorldPosition.Size() < 100000.0f)
                        {
                            SpringPath.Add(SpringSample.WorldPosition);
                        }
                        else
                        {
                            if (bVerboseLogging)
                            {
                                UE_LOG(LogPACManager, VeryVerbose,
                                    TEXT("Spring sample invalid at step %d: NaN=%s, Size=%.1f"), i,
                                    SpringSample.WorldPosition.ContainsNaN() ? TEXT("True") : TEXT("False"),
                                    SpringSample.WorldPosition.Size());
                            }
                            bSpringSuccess = false;
                            break;
                        }
                        
                        if (i < NumSamples)
                        {
                            SpringSample = UOHAlgoUtils::IntegrateCriticallyDampedSpringMotionSample(
                                SpringSample,
                                TargetPos,
                                SpringFreq,
                                TimeStep
                            );
                        }
                    }
                    
                    if (bSpringSuccess && SpringPath.Num() == NumSamples + 1)
                    {
                        ModelPredictions.Add(SpringPath);
                        float SpringWeight = 0.6f * (1.0f - Chain.AttackConfidence);
                        ModelWeights.Add(SpringWeight);
                        
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Spring model added: Weight=%.3f, Frequency=%.1f, Confidence=%.2f"), 
                                SpringWeight, SpringFreq, Chain.AttackConfidence);
                        }
                    }
                }
            }
        }
    }
    else if (bVerboseLogging)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("Spring model skipped: AttackConfidence=%.2f (need 0.3-0.7)"), Chain.AttackConfidence);
    }
    
    // === ENHANCED: WEIGHTED MODEL BLENDING WITH COMPREHENSIVE VALIDATION ===
    if (ModelPredictions.Num() > 0)
    {
        Chain.PredictedTrajectory.SetNum(NumSamples + 1);
        
        // Validate and normalize weights
        float TotalWeight = 0.0f;
        for (int32 i = 0; i < ModelWeights.Num(); i++)
        {
            if (!FMath::IsNaN(ModelWeights[i]) && FMath::IsFinite(ModelWeights[i]) && ModelWeights[i] > 0.0f)
            {
                TotalWeight += ModelWeights[i];
            }
            else
            {
                ModelWeights[i] = 0.0f; // Zero out invalid weights
                if (bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Model weight %d invalidated: NaN or non-finite"), i);
                }
            }
        }
        
        if (TotalWeight > KINDA_SMALL_NUMBER)
        {
            // Weighted prediction blending with comprehensive validation
            int32 ValidBlendedPoints = 0;
            for (int32 i = 0; i <= NumSamples; i++)
            {
                FVector BlendedPoint = FVector::ZeroVector;
                float ValidWeightSum = 0.0f;
                
                for (int32 ModelIdx = 0; ModelIdx < ModelPredictions.Num(); ModelIdx++)
                {
                    if (ModelPredictions[ModelIdx].IsValidIndex(i) && ModelWeights[ModelIdx] > 0.0f)
                    {
                        FVector ModelPoint = ModelPredictions[ModelIdx][i];
                        if (!ModelPoint.ContainsNaN() && ModelPoint.Size() < 100000.0f)
                        {
                            float NormalizedWeight = ModelWeights[ModelIdx] / TotalWeight;
                            BlendedPoint += ModelPoint * NormalizedWeight;
                            ValidWeightSum += NormalizedWeight;
                        }
                    }
                }
                
                // Ensure valid blended point
                if (ValidWeightSum > KINDA_SMALL_NUMBER && !BlendedPoint.ContainsNaN() && 
                    BlendedPoint.Size() < 100000.0f)
                {
                    Chain.PredictedTrajectory[i] = BlendedPoint;
                    ValidBlendedPoints++;
                }
                else
                {
                    // Fallback to current position for invalid points
                    Chain.PredictedTrajectory[i] = MotionData->GetCurrentPosition();
                }
            }
            
            // === ENHANCED: TRAJECTORY SMOOTHING WITH VALIDATION ===
            if (Chain.PredictedTrajectory.Num() > 3 && ValidBlendedPoints > (NumSamples * 0.8f))
            {
                TArray<FVector> SmoothedTrajectory = Chain.PredictedTrajectory;
                
                for (int32 i = 1; i < Chain.PredictedTrajectory.Num() - 1; i++)
                {
                    FVector SmoothedPoint = (
                        Chain.PredictedTrajectory[i-1] * 0.25f +
                        Chain.PredictedTrajectory[i] * 0.5f +
                        Chain.PredictedTrajectory[i+1] * 0.25f
                    );
                    
                    if (!SmoothedPoint.ContainsNaN() && SmoothedPoint.Size() < 100000.0f)
                    {
                        SmoothedTrajectory[i] = SmoothedPoint;
                    }
                }
                
                Chain.PredictedTrajectory = SmoothedTrajectory;
            }
            
            // Calculate trajectory confidence based on model consensus and validation success
            float ConsensusConfidence = (TotalWeight / ModelPredictions.Num()) * Chain.MotionCoherence;
            float ValidationConfidence = static_cast<float>(ValidBlendedPoints) / (NumSamples + 1);
            Chain.TrajectoryConfidence = FMath::Clamp(
                ConsensusConfidence * ValidationConfidence, 
                0.0f, 
                1.0f
            );
        }
        else
        {
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Warning,
                    TEXT("All model weights invalid for chain '%s': TotalWeight=%.6f"), 
                    *Chain.RootBone.ToString(), TotalWeight);
            }
        }
        
        // === COMPREHENSIVE DEBUG OUTPUT ===
        if (bVerboseLogging)
        {
            FString ModelList;
            for (int32 i = 0; i < ModelNames.Num(); i++)
            {
                if (i > 0) ModelList += TEXT(", ");
                ModelList += FString::Printf(TEXT("%s(%.3f)"), *ModelNames[i], 
                    ModelWeights.IsValidIndex(i) ? ModelWeights[i] : 0.0f);
            }
            
            UE_LOG(LogPACManager, Log,
                TEXT("Trajectory Prediction Complete [%s]: Models=[%s], Confidence=%.3f, Space=%s, Samples=%d"),
                *Chain.RootBone.ToString(),
                *ModelList,
                Chain.TrajectoryConfidence,
                *UEnum::GetValueAsString(Chain.MotionAnalysisSpace),
                Chain.PredictedTrajectory.Num());
        }
    }
    else if (bVerboseLogging)
    {
        UE_LOG(LogPACManager, VeryVerbose,
            TEXT("No valid prediction models for chain '%s'"), *Chain.RootBone.ToString());
    }
    
    // === FALLBACK: SIMPLE PHYSICS PREDICTION WITH REFERENCE SPACE INTEGRATION ===
    if (Chain.PredictedTrajectory.Num() == 0)
    {
        FVector CurrentPos = MotionData->GetCurrentPosition();
        FVector Velocity = MotionData->GetVelocity(Chain.MotionAnalysisSpace);
        FVector Acceleration = MotionData->GetAcceleration(Chain.MotionAnalysisSpace);
        
        // Validate fallback data
        if (!CurrentPos.ContainsNaN() && !Velocity.ContainsNaN() && !Acceleration.ContainsNaN() &&
            CurrentPos.Size() < 100000.0f && Velocity.Size() < 10000.0f && Acceleration.Size() < 50000.0f)
        {
            Chain.PredictedTrajectory.SetNum(NumSamples + 1);
            
            for (int32 i = 0; i <= NumSamples; i++)
            {
                float Time = (float)i / (float)NumSamples * PredictionTime;
                FVector PredictedPos = CurrentPos + Velocity * Time + 0.5f * Acceleration * Time * Time;
                
                if (!PredictedPos.ContainsNaN() && PredictedPos.Size() < 100000.0f)
                {
                    Chain.PredictedTrajectory[i] = PredictedPos;
                }
                else
                {
                    Chain.PredictedTrajectory[i] = CurrentPos;
                }
            }
            
            Chain.TrajectoryConfidence = 0.4f * Chain.MotionCoherence;
            
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Log,
                    TEXT("Fallback trajectory prediction used for chain '%s': Confidence=%.3f"), 
                    *Chain.RootBone.ToString(), Chain.TrajectoryConfidence);
            }
        }
        else
        {
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Warning,
                    TEXT("Fallback trajectory prediction failed for chain '%s': Invalid kinematic data"), 
                    *Chain.RootBone.ToString());
            }
        }
    }
}

void UOHPACManager::AnalyzeChainAttackPattern(FOHCombatChainData& Chain, float DeltaTime)
{
    // === ENHANCED: REFERENCE SPACE-AWARE PATTERN ANALYSIS INITIALIZATION ===
    float TotalJerk = 0.0f;
    float TotalCurvature = 0.0f;
    float CoherenceScore = 0.0f;
    int32 ValidBoneCount = 0;
    
    // Multi-space motion tracking arrays for comprehensive analysis
    TArray<FVector> BoneDirectionsAnalysis; // In configured analysis space
    TArray<FVector> BoneDirectionsWorld;
    TArray<FVector> BoneDirectionsLocal;
    TArray<FVector> BoneDirectionsComponent;
    TArray<float> BoneSpeedsAnalysis;
    TArray<float> BoneSpeedsWorld;
    TArray<float> BoneSpeedsLocal;
    TArray<float> BoneSpeedsComponent;
    TArray<FVector> BoneJerks;
    
    // === COMPREHENSIVE BONE-LEVEL ANALYSIS WITH CHAIN-CONFIGURED SPACE ===
    for (const FName& BoneName : Chain.ChainBones)
    {
        const FOHBoneMotionData* MotionData = Chain.GetBoneMotionData(BoneName);
        if (!MotionData || MotionData->GetHistoryDepth() < 3) continue;
        
        // === ENHANCED: MULTI-SPACE VELOCITY ANALYSIS ===
        FVector VelocityWorld = MotionData->GetVelocity(EOHReferenceSpace::WorldSpace);
        FVector VelocityLocal = MotionData->GetVelocity(EOHReferenceSpace::LocalSpace);
        FVector VelocityComponent = MotionData->GetVelocity(EOHReferenceSpace::ComponentSpace);
        FVector VelocityAnalysis = MotionData->GetVelocity(Chain.MotionAnalysisSpace); // Chain-configured space
        
        // Validate velocities before processing
        if (VelocityWorld.ContainsNaN() || VelocityLocal.ContainsNaN() || 
            VelocityComponent.ContainsNaN() || VelocityAnalysis.ContainsNaN())
        {
            continue;
        }
        
        float SpeedWorld = VelocityWorld.Size();
        float SpeedLocal = VelocityLocal.Size();
        float SpeedComponent = VelocityComponent.Size();
        float SpeedAnalysis = VelocityAnalysis.Size();
        
        // === ENHANCED: JERK AND CURVATURE CALCULATION WITH CHAIN ANALYSIS SPACE ===
        FVector Jerk = MotionData->CalculateJerk(Chain.MotionAnalysisSpace == EOHReferenceSpace::LocalSpace);
        float Curvature = UOHAlgoUtils::EstimateCurvatureFromBoneData(*MotionData);
        
        if (!Jerk.ContainsNaN() && !FMath::IsNaN(Curvature))
        {
            BoneJerks.Add(Jerk);
            TotalJerk += Jerk.Size();
            TotalCurvature += Curvature;
            ValidBoneCount++;
        }
        
        // === REFERENCE SPACE-SPECIFIC MOTION CONTRIBUTION ANALYSIS ===
        float MotionThreshold = Chain.MotionDetectionThreshold * 0.5f;
        
        // Always collect analysis space data (chain-configured)
        if (SpeedAnalysis > MotionThreshold)
        {
            BoneDirectionsAnalysis.Add(VelocityAnalysis.GetSafeNormal());
            BoneSpeedsAnalysis.Add(SpeedAnalysis);
        }
        
        // Collect all reference space data for comprehensive analysis
        if (SpeedWorld > MotionThreshold)
        {
            BoneDirectionsWorld.Add(VelocityWorld.GetSafeNormal());
            BoneSpeedsWorld.Add(SpeedWorld);
        }
        
        if (SpeedLocal > MotionThreshold)
        {
            BoneDirectionsLocal.Add(VelocityLocal.GetSafeNormal());
            BoneSpeedsLocal.Add(SpeedLocal);
        }
        
        if (SpeedComponent > MotionThreshold)
        {
            BoneDirectionsComponent.Add(VelocityComponent.GetSafeNormal());
            BoneSpeedsComponent.Add(SpeedComponent);
        }
    }
    
    // === ENHANCED: COHERENCE CALCULATION LAMBDA FOR REUSABILITY ===
    auto CalculateCoherence = [](const TArray<FVector>& Directions, const TArray<float>& Speeds) -> TPair<float, FVector>
    {
        if (Directions.Num() < 2) return TPair<float, FVector>(0.0f, FVector::ZeroVector);
        
        // Calculate weighted mean direction
        FVector MeanDirection = FVector::ZeroVector;
        float TotalSpeed = 0.0f;
        
        for (int32 i = 0; i < Directions.Num(); i++)
        {
            MeanDirection += Directions[i] * Speeds[i];
            TotalSpeed += Speeds[i];
        }
        
        if (TotalSpeed <= 0.0f) return TPair<float, FVector>(0.0f, FVector::ZeroVector);
        
        MeanDirection /= TotalSpeed;
        MeanDirection.Normalize();
        
        // Calculate coherence variance
        float Variance = 0.0f;
        for (int32 i = 0; i < Directions.Num(); i++)
        {
            float Alignment = FVector::DotProduct(Directions[i], MeanDirection);
            float Deviation = 1.0f - FMath::Clamp(Alignment, 0.0f, 1.0f);
            Variance += Deviation * Deviation * (Speeds[i] / TotalSpeed);
        }
        
        float CoherenceScore = FMath::Exp(-Variance * 2.0f);
        return TPair<float, FVector>(CoherenceScore, MeanDirection);
    };
    
    // === CRITICAL: PRIMARY COHERENCE CALCULATION IN CHAIN-CONFIGURED ANALYSIS SPACE ===
    TPair<float, FVector> PrimaryCoherenceResult = CalculateCoherence(BoneDirectionsAnalysis, BoneSpeedsAnalysis);
    CoherenceScore = PrimaryCoherenceResult.Key;
    FVector PrincipalDirection = PrimaryCoherenceResult.Value;
    
    // === ENHANCED: MULTI-SPACE COHERENCE COMPARISON FOR MOTION CLASSIFICATION ===
    TPair<float, FVector> WorldCoherenceResult = CalculateCoherence(BoneDirectionsWorld, BoneSpeedsWorld);
    TPair<float, FVector> LocalCoherenceResult = CalculateCoherence(BoneDirectionsLocal, BoneSpeedsLocal);
    TPair<float, FVector> ComponentCoherenceResult = CalculateCoherence(BoneDirectionsComponent, BoneSpeedsComponent);
    
    float WorldCoherence = WorldCoherenceResult.Key;
    float LocalCoherence = LocalCoherenceResult.Key;
    float ComponentCoherence = ComponentCoherenceResult.Key;
    
    // === ENHANCED: CHAIN METRICS UPDATE WITH COMPREHENSIVE VALIDATION ===
    Chain.MotionCoherence = CoherenceScore;
    Chain.AttackJerkMagnitude = ValidBoneCount > 0 ? (TotalJerk / ValidBoneCount) : 0.0f;
    Chain.AttackCurvature = ValidBoneCount > 0 ? (TotalCurvature / ValidBoneCount) : 0.0f;
    
    if (!PrincipalDirection.ContainsNaN() && !PrincipalDirection.IsZero())
    {
        Chain.AttackPrincipalDirection = PrincipalDirection;
    }
    
    // === MODERNIZED: ENHANCED ATTACK PATTERN CLASSIFICATION ===
    enum class EAttackPattern : uint8
    {
        None,
        Straight,        // Low curvature, high coherence
        Hook,           // High curvature, medium coherence  
        Uppercut,       // High curvature, high vertical component
        Defensive,      // Low speed, high jerk (reactive)
        IsolatedLimb,   // High local motion, low world motion
        Coordinated     // Multi-space coherent motion
    };
    
    EAttackPattern DetectedPattern = EAttackPattern::None;
    float PatternConfidence = 0.0f;
    
    // === ENHANCED: ISOLATED LIMB MOTION DETECTION WITH MULTI-SPACE ANALYSIS ===
    float LocalToWorldRatio = 0.0f;
    float ComponentToWorldRatio = 0.0f;
    
    if (BoneSpeedsWorld.Num() > 0)
    {
        float AvgWorldSpeed = 0.0f;
        float AvgLocalSpeed = 0.0f;
        float AvgComponentSpeed = 0.0f;
        
        for (float Speed : BoneSpeedsWorld) AvgWorldSpeed += Speed;
        AvgWorldSpeed /= BoneSpeedsWorld.Num();
        
        if (BoneSpeedsLocal.Num() > 0)
        {
            for (float Speed : BoneSpeedsLocal) AvgLocalSpeed += Speed;
            AvgLocalSpeed /= BoneSpeedsLocal.Num();
            LocalToWorldRatio = AvgWorldSpeed > 0.0f ? (AvgLocalSpeed / AvgWorldSpeed) : 0.0f;
        }
        
        if (BoneSpeedsComponent.Num() > 0)
        {
            for (float Speed : BoneSpeedsComponent) AvgComponentSpeed += Speed;
            AvgComponentSpeed /= BoneSpeedsComponent.Num();
            ComponentToWorldRatio = AvgWorldSpeed > 0.0f ? (AvgComponentSpeed / AvgWorldSpeed) : 0.0f;
        }
    }
    
    // === ENHANCED: MULTI-FACTOR PATTERN DETECTION ===
    
    // Check for coordinated multi-space motion
    if (WorldCoherence > 0.7f && LocalCoherence > 0.7f && ComponentCoherence > 0.7f)
    {
        DetectedPattern = EAttackPattern::Coordinated;
        PatternConfidence = (WorldCoherence + LocalCoherence + ComponentCoherence) / 3.0f;
    }
    // Check for isolated limb motion (chain analysis space shows high activity vs world space)
    else if (LocalToWorldRatio > 1.8f || ComponentToWorldRatio > 1.8f)
    {
        DetectedPattern = EAttackPattern::IsolatedLimb;
        PatternConfidence = FMath::Clamp(CoherenceScore * 0.6f + 
                                        FMath::Clamp(Chain.AttackJerkMagnitude / 12000.0f, 0.0f, 1.0f) * 0.4f, 0.0f, 1.0f);
    }
    // Check for curved attacks
    else if (Chain.AttackCurvature > 0.3f)
    {
        if (Chain.AttackPrincipalDirection.Z > 0.5f)
        {
            DetectedPattern = EAttackPattern::Uppercut;
        }
        else
        {
            DetectedPattern = EAttackPattern::Hook;
        }
        
        PatternConfidence = CoherenceScore * 0.3f +
                           FMath::Clamp(Chain.AttackCurvature / 0.5f, 0.0f, 1.0f) * 0.4f +
                           FMath::Clamp(Chain.ChainKineticEnergy / 4000.0f, 0.0f, 1.0f) * 0.3f;
    }
    // Check for defensive patterns
    else if (Chain.ChainKineticEnergy < 1000.0f && Chain.AttackJerkMagnitude > 10000.0f)
    {
        DetectedPattern = EAttackPattern::Defensive;
        PatternConfidence = 0.2f;
    }
    // Check for straight attacks
    else if (Chain.AttackJerkMagnitude > 5000.0f && CoherenceScore > 0.7f)
    {
        DetectedPattern = EAttackPattern::Straight;
        PatternConfidence = CoherenceScore * 0.5f + 
                           FMath::Clamp(Chain.AttackJerkMagnitude / 15000.0f, 0.0f, 1.0f) * 0.3f +
                           FMath::Clamp(Chain.ChainKineticEnergy / 5000.0f, 0.0f, 1.0f) * 0.2f;
    }
    
    // === ENHANCED: STRIKE LIKELIHOOD VALIDATION WITH CHAIN ANALYSIS SPACE ===
    bool bLikelyStrike = false;
    for (const FName& BoneName : Chain.ChainBones)
    {
        const FOHBoneMotionData* MotionData = Chain.GetBoneMotionData(BoneName);
        if (MotionData)
        {
            // Use chain-configured analysis space for strike detection
            float BoneSpeed = MotionData->GetSpeed(Chain.MotionAnalysisSpace);
            float BoneQuality = MotionData->GetMotionQuality(Chain.MotionAnalysisSpace);
            
            if (BoneSpeed > Chain.MotionDetectionThreshold && BoneQuality > 0.6f)
            {
                bLikelyStrike = true;
                PatternConfidence = FMath::Max(PatternConfidence, 0.75f);
                break;
            }
        }
    }
    
    // === MODERNIZED: SMOOTH CONFIDENCE UPDATE WITH VALIDATION ===
    if (!FMath::IsNaN(PatternConfidence))
    {
        Chain.AttackConfidence = FMath::FInterpTo(
            Chain.AttackConfidence,
            PatternConfidence,
            DeltaTime,
            5.0f
        );
    }
    
    // === ENHANCED: ATTACK STATE DETERMINATION WITH CHAIN THRESHOLD ===
    Chain.bIsAttacking = (Chain.AttackConfidence > MinAttackConfidenceThreshold) || 
                        (bLikelyStrike && Chain.HasSignificantMotion());
    
    // Update attack direction with validation
    if (Chain.bIsAttacking && !Chain.AttackPrincipalDirection.ContainsNaN() && !Chain.AttackPrincipalDirection.IsZero())
    {
        Chain.AttackDirection = Chain.AttackPrincipalDirection;
    }
    
    // === COMPREHENSIVE DEBUG LOGGING WITH MULTI-SPACE METRICS ===
    if (bVerboseLogging && Chain.bIsAttacking)
    {
        const TCHAR* PatternNames[] = { TEXT("None"), TEXT("Straight"), TEXT("Hook"), TEXT("Uppercut"), TEXT("Defensive"), TEXT("IsolatedLimb"), TEXT("Coordinated") };
        UE_LOG(LogPACManager, Warning,
            TEXT("Attack Pattern [%s]: Type=%s, Conf=%.2f, Coh=[A:%.2f,W:%.2f,L:%.2f,C:%.2f], Jerk=%.0f, Curve=%.2f, Space=%s, Ratios=[L:%.1f,C:%.1f]"),
            *Chain.RootBone.ToString(),
            PatternNames[static_cast<uint8>(DetectedPattern)],
            Chain.AttackConfidence,
            CoherenceScore, WorldCoherence, LocalCoherence, ComponentCoherence,
            Chain.AttackJerkMagnitude,
            Chain.AttackCurvature,
            *UEnum::GetValueAsString(Chain.MotionAnalysisSpace),
            LocalToWorldRatio,
            ComponentToWorldRatio
        );
    }
}


void UOHPACManager::CheckChainVsAllBodies(
    const FOHCombatChainData& AttackingChain,
    UOHPACManager* DefenderManager)
{
    if (!DefenderManager || AttackingChain.PredictedTrajectory.Num() < 2) return;
    
    // Time parameters
    const float TotalPredictTime = 0.3f;
    const float TimeStep = TotalPredictTime / (AttackingChain.PredictedTrajectory.Num() - 1);
    
    // Gather all candidates along trajectory
    TArray<FCombatTargetCandidate> AllCandidates;
    
    for (int32 i = 0; i < AttackingChain.PredictedTrajectory.Num() - 1; i++)
    {
        FVector TrajectoryStart = AttackingChain.PredictedTrajectory[i];
        FVector TrajectoryEnd = AttackingChain.PredictedTrajectory[i + 1];
        float CurrentTime = i * TimeStep;
        
        // Check each defender bone
        for (const FName& DefenderBone : DefenderManager->GetSimulatableBones())
        {
            const FOHBoneMotionData* DefenderMotion = DefenderManager->BoneMotionMap.Find(DefenderBone);
            if (!DefenderMotion) continue;
            
            // Predict defender position
            FVector DefenderFuturePos = DefenderMotion->PredictFuturePosition(CurrentTime, EOHReferenceSpace::WorldSpace, true);
            
            // High quality motion gets bezier prediction
            if (DefenderMotion->GetMotionQuality() > 0.7f)
            {
                TArray<FVector> DefenderBezier = UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(
                    *DefenderMotion,
                    CurrentTime
                );
                
                if (DefenderBezier.Num() >= 3)
                {
                    float t = FMath::Clamp(CurrentTime / TotalPredictTime, 0.0f, 1.0f);
                    DefenderFuturePos = UOHAlgoUtils::SampleBezierQuadratic(
                        DefenderBezier[0], 
                        DefenderBezier[1], 
                        DefenderBezier[2], 
                        t
                    );
                }
            }
            
            // Check distance to trajectory segment
            FVector ClosestPoint = FMath::ClosestPointOnSegment(
                DefenderFuturePos,
                TrajectoryStart,
                TrajectoryEnd
            );
            
            float Distance = FVector::Dist(DefenderFuturePos, ClosestPoint);
            float CheckRadius = AttackingChain.EffectiveStrikeRadius + EstimateBoneRadius(DefenderBone);
            
            if (Distance <= CheckRadius)
            {
                FCombatTargetCandidate Candidate;
                Candidate.BoneName = DefenderBone;
                Candidate.Position = DefenderFuturePos;
                Candidate.TimeToImpact = CurrentTime;
                Candidate.Distance = Distance;
                Candidate.bIsCore = IsCoreBone(DefenderBone);
                Candidate.bIsExtremity = IsExtremityBone(DefenderBone);
                
                // Calculate priority with motion quality factor
                Candidate.Priority = Candidate.bIsCore ? CoreBonePriority : ExtremityPenalty;
                Candidate.Priority *= AttackingChain.ChainMotionQuality;
                
                // Check if blocking
                if (Candidate.bIsExtremity)
                {
                    Candidate.bIsBlocking = IsExtremityProtectingCore(
                        DefenderBone,
                        DefenderManager,
                        AttackingChain.AttackDirection
                    );
                }
                
                AllCandidates.Add(Candidate);
            }
        }
    }
    
    // Select best target
    if (AllCandidates.Num() > 0)
    {
        FCombatTargetCandidate BestTarget = SelectBestTargetFromChain(
            AttackingChain,
            DefenderManager,
            AllCandidates
        );
        // Process the hit
        ProcessChainHit(AttackingChain, DefenderManager, BestTarget);
    }
}

bool UOHPACManager::ProcessChainHit(const FOHCombatChainData& AttackingChain, UOHPACManager* DefenderManager, const FCombatTargetCandidate& Target)
{
    // === EXISTING VALIDATION CODE REMAINS ===
    if (!DefenderManager || DefenderManager == this)
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("ProcessChainHit: Invalid or self-targeting defender manager"), true);
        }
        return false;
    }
    
    // === COOLDOWN AND VALIDATION LOGIC REMAINS ===
    float CurrentTime = GetWorld()->GetTimeSeconds();
    uint64 HitPairID = GetCombatHitID(AttackingChain.RootBone, Target.BoneName);

    if (float* LastHitTime = CombatHitTimestamps.Find(HitPairID))
    {
        if (CurrentTime - *LastHitTime < ImpactCooldownTime)
        {
            return false;
        }
    }
    CombatHitTimestamps.Add(HitPairID, CurrentTime);

    // === ENHANCED IMPACT DIRECTION CALCULATION ===
    FVector ImpactDirection = FVector::ZeroVector;
    
    // Priority 1: Use target-provided impact direction (from EPA analysis)
    if (!Target.ImpactDirection.IsNearlyZero() && Target.ImpactDirection.IsNormalized())
    {
        ImpactDirection = Target.ImpactDirection;
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, TEXT("Using EPA-derived impact direction: %s"), *ImpactDirection.ToString());
        }
    }
    // Priority 2: Use chain attack direction (motion-derived)
    else if (!AttackingChain.AttackDirection.IsNearlyZero())
    {
        ImpactDirection = AttackingChain.AttackDirection;
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, TEXT("Using chain attack direction: %s"), *ImpactDirection.ToString());
        }
    }
    // Priority 3: Use chain momentum direction (physics-derived)
    else if (!AttackingChain.ChainMomentum.IsNearlyZero())
    {
        ImpactDirection = AttackingChain.ChainMomentum.GetSafeNormal();
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, TEXT("Using chain momentum direction: %s"), *ImpactDirection.ToString());
        }
    }
    // Priority 4: Spatial relationship fallback (geometric)
    else
    {
        FVector AttackerToTarget = (Target.Position - AttackingChain.ChainCenterOfMass);
        if (!AttackerToTarget.IsNearlyZero())
        {
            ImpactDirection = AttackerToTarget.GetSafeNormal();
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Warning, TEXT("Using geometric fallback direction: %s"), *ImpactDirection.ToString());
            }
        }
        else
        {
            // Ultimate fallback - use attacker forward direction
            if (AActor* AttackerActor = GetOwner())
            {
                ImpactDirection = AttackerActor->GetActorForwardVector();
                if (bVerboseLogging)
                {
                    UE_LOG(LogPACManager, Warning, TEXT("Using attacker forward as final fallback: %s"), *ImpactDirection.ToString());
                }
            }
            else
            {
                if (bVerboseLogging)
                {
                    SafeLog(TEXT("ProcessChainHit: Unable to determine valid impact direction"), true);
                }
                return false;
            }
        }
    }
    
    // Ensure horizontal impact for grounded character physics (reduce Z component)
    ImpactDirection.Z = FMath::Clamp(ImpactDirection.Z, -0.3f, 0.3f);
    ImpactDirection.Normalize();

    // === FORCE CALCULATION WITH ENHANCED VALIDATION ===
    FName StrikingBone = AttackingChain.GetPrimaryStrikingBone();
    float BaseForce = AttackingChain.ChainMomentum.Size();
    
    if (!FMath::IsFinite(BaseForce) || BaseForce < KINDA_SMALL_NUMBER)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ProcessChainHit: Invalid chain momentum %.2f"), BaseForce), true);
        }
        return false;
    }
    
    // Apply motion quality scaling
    float MotionQualityFactor = FMath::Clamp(AttackingChain.ChainMotionQuality, 0.1f, 1.0f);
    BaseForce *= (0.5f + MotionQualityFactor * 0.5f);

    // Apply bone-specific multipliers
    float BoneMultiplier = 1.0f;
    FString BoneString = StrikingBone.ToString().ToLower();
    if (BoneString.Contains(TEXT("foot")) || BoneString.Contains(TEXT("toe")))
    {
        BoneMultiplier = FootForceMultiplier;
    }
    else if (BoneString.Contains(TEXT("hand")) || BoneString.Contains(TEXT("fist")))
    {
        BoneMultiplier = HandForceMultiplier;
    }
    
    BaseForce *= BoneMultiplier;

    // Apply target-specific scaling
    if (Target.bIsCore)
    {
        BaseForce *= CoreHitPushbackMultiplier;
    }
    else if (Target.bIsExtremity)
    {
        BaseForce *= ExtremityHitPushbackMultiplier;
    }

    float FinalForce = FMath::Max(BaseForce, MinimumPushbackForce);

    // === CRITICAL: ASSIGN IMPACT TRACKING DATA FOR DEFENDER ===
    DefenderManager->LastImpactDirection = ImpactDirection;
    DefenderManager->LastImpactMagnitude = FinalForce;
    DefenderManager->LastImpactTime = CurrentTime;

    // === UPPER-BODY PHYSICS REACTION (DIRECT BODY MANIPULATION) ===
    FVector PhysicsImpulse = ImpactDirection * FinalForce * ImpactImpulseScale;
    
    if (FBodyInstance* DefenderBody = DefenderManager->GetBodyInstanceDirect(Target.BoneName))
    {
        if (DefenderBody->IsInstanceSimulatingPhysics())
        {
            DefenderBody->AddImpulseAtPosition(PhysicsImpulse, Target.Position);
            DefenderBody->WakeInstance();
            
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Log, TEXT("Applied direct physics impulse %.1f to bone '%s' in direction %s"), 
                    PhysicsImpulse.Size(), *Target.BoneName.ToString(), *ImpactDirection.ToString());
            }
        }
    }

    // Apply radial force for core hits
    if (Target.bIsCore)
    {
        DefenderManager->ApplyRadialImpulseToTarget(DefenderManager, Target.Position, FinalForce);
    }

    // === PAC STRENGTH FLINCH APPLICATION ===
    DefenderManager->ApplyImpactFlinch(FinalForce * FlinchScale);

    // === CRITICAL: MOVEMENT PUSHBACK WITH PROPER DIRECTION ===
    if (DefenderManager->bEnableMovementPushback)
    {
        DefenderManager->ValidateComponentReferences();
        
        if (UOHMovementComponent* DefenderMovement = DefenderManager->GetMovementComponent())
        {
            // === CRITICAL FIX: ENSURE PUSHBACK MOVES DEFENDER AWAY FROM ATTACKER ===
            FVector MovementPushDirection = ImpactDirection; // This should push defender away
            float MovementPushMagnitude = FinalForce * DefenderManager->PushbackForceMultiplier;
            
            // Validate direction is away from attacker
            if (AActor* AttackerActor = GetOwner())
            {
                if (AActor* DefenderActor = DefenderManager->GetOwner())
                {
                    FVector AttackerToDefender = (DefenderActor->GetActorLocation() - AttackerActor->GetActorLocation()).GetSafeNormal();
                    AttackerToDefender.Z = 0.0f; // Keep horizontal
                    AttackerToDefender.Normalize();
                    
                    // If our calculated direction is pointing back toward attacker, correct it
                    float DirectionAlignment = FVector::DotProduct(MovementPushDirection, AttackerToDefender);
                    if (DirectionAlignment < 0.0f)
                    {
                        MovementPushDirection = AttackerToDefender;
                        if (bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, Warning, TEXT("Corrected push direction - was pointing toward attacker"));
                        }
                    }
                }
            }
            
            FVector MovementPushImpulse = MovementPushDirection * MovementPushMagnitude;
            DefenderMovement->ApplyPhysicsImpulse(MovementPushImpulse, true);
            
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, Log, TEXT("Applied movement pushback %.1f to defender in direction %s"), 
                    MovementPushMagnitude, *MovementPushDirection.ToString());
            }
        }
    }

    // === CORRECTED ATTACKER SELF-PUSHBACK LOGIC ===
    if (bApplySelfPushback && GetMovementComponent())
    {
        // Validate we have motion data for the attacking bone
        const FOHBoneMotionData* StrikingMotionData = GetBoneMotionData(StrikingBone);
        if (StrikingMotionData)
        {
            FVector StrikingVelocity = StrikingMotionData->GetVelocity(false);
            float AttackSpeed = StrikingVelocity.Size();
            
            // Only apply self-pushback if attack meets minimum criteria
            if (AttackSpeed > MinAttackSpeed * 0.5f)
            {
                float ForwardAlignment = FVector::DotProduct(StrikingVelocity.GetSafeNormal(), ImpactDirection);
                
                // === CRITICAL FIX: REDUCE SELF-PUSHBACK FOR COMMITTED ATTACKS ===
                bool bIsCommittedAttack = (AttackSpeed > MinAttackSpeed && 
                                         ForwardAlignment > 0.7f && 
                                         AttackingChain.AttackConfidence > 0.8f);
                
                if (bIsCommittedAttack)
                {
                    // Minimal self-pushback for committed attacks
                    float ReducedSelfPushback = SelfPushbackMultiplier * 0.2f; // 80% reduction
                    FVector AttackerPushDirection = -ImpactDirection;
                    float AttackerPushMagnitude = FinalForce * ReducedSelfPushback;
                    
                    GetMovementComponent()->ApplyPhysicsImpulse(AttackerPushDirection * AttackerPushMagnitude, true);
                    
                    if (bVerboseLogging)
                    {
                        UE_LOG(LogPACManager, Log, TEXT("Applied reduced self-pushback %.1f (committed attack)"), AttackerPushMagnitude);
                    }
                }
                else
                {
                    // Normal self-pushback for glancing or weak strikes
                    FVector AttackerPushDirection = -ImpactDirection;
                    float AttackerPushMagnitude = FinalForce * SelfPushbackMultiplier * (1.0f - ForwardAlignment * 0.5f);
                    
                    GetMovementComponent()->ApplyPhysicsImpulse(AttackerPushDirection * AttackerPushMagnitude, true);
                    
                    if (bVerboseLogging)
                    {
                        UE_LOG(LogPACManager, Log, TEXT("Applied normal self-pushback %.1f"), AttackerPushMagnitude);
                    }
                }
            }
        }
    }

    // === EVENT BROADCASTING ===
    OnPhysicsBodyHitEvent.Broadcast(
        SkeletalMesh,
        StrikingBone,
        DefenderManager->SkeletalMesh,
        Target.BoneName,
        AttackingChain.ChainMomentum / FMath::Max(AttackingChain.ChainTotalMass, 1.0f),
        FHitResult()
    );

    if (bVerboseLogging)
    {
        SafeLog(FString::Printf(
            TEXT("Combat Hit Complete: %s->%s, Dir=%s, Force=%.1f, Penetration=%.2f"),
            *StrikingBone.ToString(),
            *Target.BoneName.ToString(),
            *ImpactDirection.ToString(),
            FinalForce,
            Target.PenetrationDepth
        ), false);
    }

    return true;
}


void UOHPACManager::CheckChainCombatCollisions(float DeltaTime)
{
    if (!GetWorld() || !bEnableCollisionImpactSystem || CombatChains.Num() == 0) 
    {
        return;
    }
    
    // === ENHANCED OPPONENT DETECTION FOR 360-DEGREE COVERAGE ===
    AActor* MyActor = GetOwner();
    if (!MyActor) return;
    
    // Find all potential opponents within combat range
    TArray<ACharacter*> PotentialOpponents;
    TArray<AActor*> FoundCharacters;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACharacter::StaticClass(), FoundCharacters);
    
    for (AActor* Actor : FoundCharacters)
    {
        if (Actor == MyActor || !IsValid(Actor)) continue;
        
        float Distance = FVector::Dist(MyActor->GetActorLocation(), Actor->GetActorLocation());
        if (Distance < MaxCombatRange)
        {
            if (ACharacter* Character = Cast<ACharacter>(Actor))
            {
                PotentialOpponents.Add(Character);
            }
        }
    }
    
    if (PotentialOpponents.Num() == 0) return;
    
    // === PROCESS EACH ATTACKING CHAIN AGAINST ALL OPPONENTS ===
    for (auto& ChainPair : CombatChains)
    {
        FOHCombatChainData& AttackingChain = ChainPair.Value;
        
        // Skip inactive chains
        if (!AttackingChain.bIsAttacking || 
            AttackingChain.AttackConfidence < MinAttackConfidenceThreshold ||
            AttackingChain.ChainBones.Num() == 0)
        {
            continue;
        }
        
        // === ENHANCED ATTACK HULL GENERATION FOR 360-DEGREE DETECTION ===
        TArray<FVector> AttackHullPoints;
        
        // Collect motion data from all chain bones
        for (const FName& BoneName : AttackingChain.ChainBones)
        {
            if (const FOHBoneMotionData* MotionData = GetBoneMotionData(BoneName))
            {
                if (MotionData->GetSpeed(EOHReferenceSpace::WorldSpace) > MinAttackSpeed * 0.7f) // Slightly lower threshold for detection
                {
                    // Current position
                    FVector CurrentPos = SkeletalMesh->GetBoneLocation(BoneName);
                    AttackHullPoints.Add(CurrentPos);
                    
                    // Predicted position
                    FVector PredictedPos = MotionData->PredictFuturePosition(0.1f, EOHReferenceSpace::WorldSpace, true);
                    if (!PredictedPos.IsNearlyZero())
                    {
                        AttackHullPoints.Add(PredictedPos);
                    }
                    
                    // Velocity trail for motion blur detection
                    FVector VelocityTrail = CurrentPos + MotionData->GetVelocity(false) * 0.05f;
                    AttackHullPoints.Add(VelocityTrail);
                    
                    // === ENHANCED: ADD RADIAL SAMPLING FOR BACK ATTACKS ===
                    // Add additional points around the bone for better coverage
                    float BoneRadius = EstimateBoneRadius(BoneName);
                    for (int32 i = 0; i < 4; i++)
                    {
                        float Angle = (i * 90.0f) * PI / 180.0f;
                        FVector RadialOffset = FVector(
                            FMath::Cos(Angle) * BoneRadius,
                            FMath::Sin(Angle) * BoneRadius,
                            0.0f
                        );
                        AttackHullPoints.Add(CurrentPos + RadialOffset);
                    }
                }
            }
        }
        
        if (AttackHullPoints.Num() < 3) continue;
        
        // === PROCESS EACH POTENTIAL OPPONENT ===
        for (ACharacter* OpponentCharacter : PotentialOpponents)
        {
            UOHPACManager* OpponentPACManager = OpponentCharacter->FindComponentByClass<UOHPACManager>();
            if (!OpponentPACManager || !OpponentPACManager->bEnableCollisionImpactSystem)
            {
                continue;
            }
            
            // === ENHANCED TARGET DETECTION WITH COMPREHENSIVE BONE COVERAGE ===
            TArray<FCombatTargetCandidate> AllCandidates;
            
            // Check all simulatable bones on opponent for comprehensive coverage
            TArray<FName> OpponentBones = OpponentPACManager->GetSimulatableBones();
            
            for (const FName& OpponentBone : OpponentBones)
            {
                FVector BoneLocation = OpponentCharacter->GetMesh()->GetBoneLocation(OpponentBone);
                
                // === ENHANCED COLLISION DETECTION USING EXPANDED HULL ===
                bool bWithinStrikeRange = false;
                float MinDistance = FLT_MAX;
                
                // Check distance to any point in attack hull
                for (const FVector& HullPoint : AttackHullPoints)
                {
                    float Distance = FVector::Dist(HullPoint, BoneLocation);
                    MinDistance = FMath::Min(MinDistance, Distance);
                    
                    float EffectiveStrikeRadius = AttackingChain.EffectiveStrikeRadius + EstimateBoneRadius(OpponentBone);
                    if (Distance <= EffectiveStrikeRadius)
                    {
                        bWithinStrikeRange = true;
                        break;
                    }
                }
                
                if (bWithinStrikeRange)
                {
                    // === ENHANCED TARGET CANDIDATE CREATION ===
                    FCombatTargetCandidate Candidate;
                    Candidate.BoneName = OpponentBone;
                    Candidate.Position = BoneLocation;
                    Candidate.Distance = MinDistance;
                    Candidate.TimeToImpact = 0.1f; // Immediate impact
                    Candidate.bIsCore = IsCoreBone(OpponentBone);
                    Candidate.bIsExtremity = IsExtremityBone(OpponentBone);
                    
                    // === ENHANCED DIRECTION CALCULATION FOR BACK ATTACKS ===
                    // Use the closest hull point to determine impact direction
                    FVector ClosestHullPoint = AttackHullPoints[0];
                    float ClosestDistance = FVector::Dist(ClosestHullPoint, BoneLocation);
                    
                    for (const FVector& HullPoint : AttackHullPoints)
                    {
                        float Distance = FVector::Dist(HullPoint, BoneLocation);
                        if (Distance < ClosestDistance)
                        {
                            ClosestDistance = Distance;
                            ClosestHullPoint = HullPoint;
                        }
                    }
                    
                    Candidate.ImpactDirection = (BoneLocation - ClosestHullPoint).GetSafeNormal();
                    Candidate.PenetrationDepth = FMath::Max(0.0f, 
                        (AttackingChain.EffectiveStrikeRadius + EstimateBoneRadius(OpponentBone)) - MinDistance);
                    
                    // Only process candidates with meaningful penetration
                    if (Candidate.PenetrationDepth > MinPenetrationThreshold)
                    {
                        // Calculate priority
                        Candidate.Priority = Candidate.bIsCore ? CoreBonePriority : 1.0f;
                        if (Candidate.bIsExtremity)
                        {
                            Candidate.Priority *= ExtremityPenalty;
                        }
                        Candidate.Priority *= AttackingChain.ChainMotionQuality;
                        
                        AllCandidates.Add(Candidate);
                    }
                }
            }
            
            // === PROCESS BEST TARGET FOR THIS OPPONENT ===
            if (AllCandidates.Num() > 0)
            {
                // Sort by priority and penetration depth
                AllCandidates.Sort([](const FCombatTargetCandidate& A, const FCombatTargetCandidate& B)
                {
                    if (FMath::Abs(A.Priority - B.Priority) > 0.1f)
                    {
                        return A.Priority > B.Priority;
                    }
                    return A.PenetrationDepth > B.PenetrationDepth;
                });
                
                FCombatTargetCandidate BestTarget = AllCandidates[0];
                
                // Process the hit
                bool bHitProcessed = ProcessChainHit(AttackingChain, OpponentPACManager, BestTarget);
                
                if (bHitProcessed && bVerboseLogging)
                {
                    FVector AttackerLocation = MyActor->GetActorLocation();
                    FVector DefenderLocation = OpponentCharacter->GetActorLocation();
                    FVector AttackerToDefender = (DefenderLocation - AttackerLocation).GetSafeNormal();
                    FVector AttackerForward = MyActor->GetActorForwardVector();
                    float AttackAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(AttackerForward, AttackerToDefender)));
                    
                    UE_LOG(LogPACManager, Warning, 
                        TEXT("Combat Hit Detected: Chain=%s->%s, Bone=%s, Angle=%.1f°, Penetration=%.2f, Direction=%s"),
                        *AttackingChain.RootBone.ToString(),
                        *OpponentCharacter->GetName(),
                        *BestTarget.BoneName.ToString(),
                        AttackAngle,
                        BestTarget.PenetrationDepth,
                        *BestTarget.ImpactDirection.ToString());
                }
            }
        }
    }
}


// === UTILITY HELPER FUNCTION FOR ENHANCED TARGET BONE SELECTION ===
FName UOHPACManager::DetermineBestTargetBone(
	UOHPACManager* TargetManager, 
	const FVector& PenetrationDirection, 
	FName AttackingBone) const
{
	if (!TargetManager || !TargetManager->SkeletalMesh)
	{
		return NAME_None;
	}
    
	// === LEVERAGE OHSKELETALPYSICSUTILS FOR BONE ANALYSIS ===
	TArray<FName> CandidateBones = TargetManager->GetSimulatableBones();
    
	float BestScore = -1.0f;
	FName BestBone = NAME_None;
    
	for (const FName& BoneName : CandidateBones)
	{
		FVector BoneLocation = TargetManager->SkeletalMesh->GetBoneLocation(BoneName);
		FVector AttackOrigin = SkeletalMesh->GetBoneLocation(AttackingBone);
        
		// === USE OHCOMBATUTILS FOR DIRECTIONAL ALIGNMENT ANALYSIS ===
		FVector ToBone = (BoneLocation - AttackOrigin).GetSafeNormal();
		float DirectionalAlignment = UOHCombatUtils::ComputeDirectionalAlignment2D(
			PenetrationDirection, ToBone
		);
        
		// Favor bones aligned with penetration direction
		float Score = DirectionalAlignment;
        
		// Apply bone priority modifiers
		if (IsCoreBone(BoneName))
		{
			Score += 0.3f; // Favor core hits
		}
		else if (IsExtremityBone(BoneName))
		{
			Score += 0.1f; // Slight favor for extremities
		}
        
		if (Score > BestScore)
		{
			BestScore = Score;
			BestBone = BoneName;
		}
	}
    
	return BestBone;
}

FOHCombatAnalysis UOHPACManager::AnalyzeCombatState() const
{
    FOHCombatAnalysis Analysis;
    
    // === EARLY VALIDATION WITH COMPREHENSIVE LOGGING ===
    if (CombatChains.Num() == 0)
    {
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose, TEXT("AnalyzeCombatState: No combat chains available"));
        }
        return Analysis; // Returns reset/empty analysis
    }
    
    // === ENHANCED: COMPREHENSIVE MULTI-SPACE TRACKING INITIALIZATION ===
    float HighestConfidence = 0.0f;
    float MaxSpeedWorld = 0.0f;
    float MaxSpeedLocal = 0.0f;
    float MaxSpeedComponent = 0.0f;
    float TotalKineticEnergy = 0.0f;
    float MotionQualitySum = 0.0f;
    float PeakAcceleration = 0.0f; // RESOLVES UNUSED MaxAccel WARNING
    float PeakJerkMagnitude = 0.0f;
    float TotalMass = 0.0f;
    float TotalMomentum = 0.0f;
    float TotalAngularMomentum = 0.0f;
    float ConsistencySum = 0.0f;
    float CoordinationSum = 0.0f;
    float TrajectoryConfidenceSum = 0.0f;
    float AttackCurvatureSum = 0.0f;
    float MaxTimeInAttack = 0.0f;
    int32 ActiveChainCount = 0;
    int32 TotalActiveBones = 0;
    
    // === ENHANCED: SPATIAL AND DIRECTIONAL TRACKING ===
    EOHReferenceSpace DominantMotionSpace = EOHReferenceSpace::LocalSpace;
    FVector DominantAttackDirection = FVector::ZeroVector;
    FVector DominantPrincipalDirection = FVector::ZeroVector;
    FVector AccumulatedCenterOfMass = FVector::ZeroVector;
    float MaxEffectiveRadius = 0.0f;
    float TotalChainLength = 0.0f;
    float MaxIntensity = 0.0f;
    float MaxDirectionalStability = 0.0f;
    
    // === COMPREHENSIVE MULTI-SPACE MOTION ANALYSIS ===
    for (const auto& ChainPair : CombatChains)
    {
        const FOHCombatChainData& Chain = ChainPair.Value;
        
        // === SKIP INACTIVE CHAINS WITH DETAILED LOGGING ===
        if (!Chain.bIsAttacking || Chain.AttackConfidence < 0.1f)
        {
            if (bVerboseLogging && Chain.AttackConfidence > 0.05f)
            {
                UE_LOG(LogPACManager, VeryVerbose, 
                    TEXT("Skipping inactive chain '%s': Attacking=%s, Confidence=%.3f"),
                    *Chain.RootBone.ToString(),
                    Chain.bIsAttacking ? TEXT("True") : TEXT("False"),
                    Chain.AttackConfidence);
            }
            continue;
        }
        
        // === EXTRACT AND VALIDATE MULTI-SPACE VELOCITIES ===
        float ChainSpeedWorld = Chain.GetChainVelocity(EOHReferenceSpace::WorldSpace).Size();
        float ChainSpeedLocal = Chain.GetChainVelocity(EOHReferenceSpace::LocalSpace).Size();
        float ChainSpeedComponent = Chain.GetChainVelocity(EOHReferenceSpace::ComponentSpace).Size();
        
        // Comprehensive validation and tracking with NaN protection
        if (FMath::IsFinite(ChainSpeedWorld) && ChainSpeedWorld >= 0.0f)
        {
            MaxSpeedWorld = FMath::Max(MaxSpeedWorld, ChainSpeedWorld);
        }
        if (FMath::IsFinite(ChainSpeedLocal) && ChainSpeedLocal >= 0.0f)
        {
            MaxSpeedLocal = FMath::Max(MaxSpeedLocal, ChainSpeedLocal);
        }
        if (FMath::IsFinite(ChainSpeedComponent) && ChainSpeedComponent >= 0.0f)
        {
            MaxSpeedComponent = FMath::Max(MaxSpeedComponent, ChainSpeedComponent);
        }
        
        // === IDENTIFY PRIMARY MOTION SPACE AND DOMINANT CHAIN ===
        if (Chain.AttackConfidence > HighestConfidence)
        {
            HighestConfidence = Chain.AttackConfidence;
            DominantMotionSpace = Chain.MotionAnalysisSpace;
            Analysis.PrimaryStrikingBone = Chain.GetPrimaryStrikingBone();
            
            // Validate and assign directional vectors
            if (!Chain.AttackDirection.ContainsNaN())
            {
                DominantAttackDirection = Chain.AttackDirection;
            }
            if (!Chain.AttackPrincipalDirection.ContainsNaN())
            {
                DominantPrincipalDirection = Chain.AttackPrincipalDirection;
            }
        }
        
        // === COMPREHENSIVE KINEMATIC ANALYSIS WITH BONE-LEVEL DETAIL ===
        for (const FName& BoneName : Chain.ChainBones)
        {
            if (const FOHBoneMotionData* MotionData = Chain.GetBoneMotionData(BoneName))
            {
                // Track peak acceleration across all bones - RESOLVES UNUSED MaxAccel
                FVector BoneAcceleration = MotionData->GetAcceleration(Chain.MotionAnalysisSpace);
                if (!BoneAcceleration.ContainsNaN())
                {
                    float AccelMagnitude = BoneAcceleration.Size();
                    if (FMath::IsFinite(AccelMagnitude) && AccelMagnitude >= 0.0f)
                    {
                        PeakAcceleration = FMath::Max(PeakAcceleration, AccelMagnitude);
                    }
                }
                
                // Track peak jerk magnitude with reference space awareness
                FVector BoneJerk = MotionData->CalculateJerk(Chain.MotionAnalysisSpace == EOHReferenceSpace::LocalSpace);
                if (!BoneJerk.ContainsNaN())
                {
                    float JerkMagnitude = BoneJerk.Size();
                    if (FMath::IsFinite(JerkMagnitude) && JerkMagnitude >= 0.0f)
                    {
                        PeakJerkMagnitude = FMath::Max(PeakJerkMagnitude, JerkMagnitude);
                    }
                }
                
                // Count active bones using chain-configured motion analysis space
                float BoneSpeed = MotionData->GetSpeed(Chain.MotionAnalysisSpace);
                if (FMath::IsFinite(BoneSpeed) && BoneSpeed > Chain.MotionDetectionThreshold * 0.3f)
                {
                    TotalActiveBones++;
                }
                
                // Track motion consistency in chain analysis space
                float BoneQuality = MotionData->GetMotionQuality(Chain.MotionAnalysisSpace);
                if (FMath::IsFinite(BoneQuality) && BoneQuality >= 0.0f && BoneQuality <= 1.0f)
                {
                    ConsistencySum += BoneQuality;
                }
            }
        }
        
        // === AGGREGATE CHAIN-LEVEL METRICS WITH COMPREHENSIVE VALIDATION ===
        
        // Kinetic energy accumulation
        if (FMath::IsFinite(Chain.ChainKineticEnergy) && Chain.ChainKineticEnergy > 0.0f)
        {
            TotalKineticEnergy += Chain.ChainKineticEnergy;
        }
        
        // Motion quality accumulation
        if (FMath::IsFinite(Chain.ChainMotionQuality) && Chain.ChainMotionQuality >= 0.0f && Chain.ChainMotionQuality <= 1.0f)
        {
            MotionQualitySum += Chain.ChainMotionQuality;
        }
        
        // Mass accumulation
        if (FMath::IsFinite(Chain.ChainTotalMass) && Chain.ChainTotalMass > 0.0f)
        {
            TotalMass += Chain.ChainTotalMass;
        }
        
        // Momentum tracking in chain-configured analysis space
        FVector ChainMomentum = Chain.GetChainMomentum(Chain.MotionAnalysisSpace);
        if (!ChainMomentum.ContainsNaN())
        {
            float MomentumMagnitude = ChainMomentum.Size();
            if (FMath::IsFinite(MomentumMagnitude) && MomentumMagnitude >= 0.0f)
            {
                TotalMomentum += MomentumMagnitude;
            }
        }
        
        // Angular momentum tracking
        if (!Chain.ChainAngularMomentum.ContainsNaN())
        {
            float AngularMomentumMagnitude = Chain.ChainAngularMomentum.Size();
            if (FMath::IsFinite(AngularMomentumMagnitude) && AngularMomentumMagnitude >= 0.0f)
            {
                TotalAngularMomentum += AngularMomentumMagnitude;
            }
        }
        
        // Coordination tracking (motion coherence)
        if (FMath::IsFinite(Chain.MotionCoherence) && Chain.MotionCoherence >= 0.0f && Chain.MotionCoherence <= 1.0f)
        {
            CoordinationSum += Chain.MotionCoherence;
        }
        
        // Trajectory confidence tracking
        if (FMath::IsFinite(Chain.TrajectoryConfidence) && Chain.TrajectoryConfidence >= 0.0f && Chain.TrajectoryConfidence <= 1.0f)
        {
            TrajectoryConfidenceSum += Chain.TrajectoryConfidence;
        }
        
        // Attack curvature tracking
        if (FMath::IsFinite(Chain.AttackCurvature) && Chain.AttackCurvature >= 0.0f)
        {
            AttackCurvatureSum += Chain.AttackCurvature;
        }
        
        // Time tracking
        if (FMath::IsFinite(Chain.TimeInAttack) && Chain.TimeInAttack >= 0.0f)
        {
            MaxTimeInAttack = FMath::Max(MaxTimeInAttack, Chain.TimeInAttack);
        }
        
        // === ENHANCED: SPATIAL METRICS ACCUMULATION ===
        if (!Chain.ChainCenterOfMass.ContainsNaN())
        {
            AccumulatedCenterOfMass += Chain.ChainCenterOfMass;
        }
        
        if (FMath::IsFinite(Chain.EffectiveStrikeRadius) && Chain.EffectiveStrikeRadius >= 0.0f)
        {
            MaxEffectiveRadius = FMath::Max(MaxEffectiveRadius, Chain.EffectiveStrikeRadius);
        }
        
        if (FMath::IsFinite(Chain.ChainLength) && Chain.ChainLength >= 0.0f)
        {
            TotalChainLength += Chain.ChainLength;
        }
        
        // === ENHANCED: TRACK MAXIMUM INTENSITY AND DIRECTIONAL STABILITY ===
        if (FMath::IsFinite(Chain.AttackIntensity) && Chain.AttackIntensity >= 0.0f)
        {
            MaxIntensity = FMath::Max(MaxIntensity, Chain.AttackIntensity);
        }
        
        if (FMath::IsFinite(Chain.AttackDirectionalStability) && Chain.AttackDirectionalStability >= 0.0f)
        {
            MaxDirectionalStability = FMath::Max(MaxDirectionalStability, Chain.AttackDirectionalStability);
        }
        
        // Track active chains
        Analysis.ActiveChains.Add(Chain.RootBone);
        ActiveChainCount++;
    }
    
    // === COMPREHENSIVE ANALYSIS FINALIZATION ===
    
    // === BASIC ATTACK STATE DETERMINATION ===
    float MaxOverallSpeed = FMath::Max3(MaxSpeedWorld, MaxSpeedLocal, MaxSpeedComponent);
    Analysis.bIsAttacking = HighestConfidence > 0.3f && MaxOverallSpeed > MinAttackSpeed;
    Analysis.AttackConfidence = HighestConfidence;
    Analysis.AttackDirection = DominantAttackDirection;
    Analysis.AttackPrincipalDirection = DominantPrincipalDirection;
    
    // === MULTI-SPACE SPEED POPULATION ===
    Analysis.MaxChainSpeedWorld = MaxSpeedWorld;
    Analysis.MaxChainSpeedLocal = MaxSpeedLocal;
    Analysis.MaxChainSpeedComponent = MaxSpeedComponent;
    Analysis.MaxChainSpeed = MaxOverallSpeed; // Legacy compatibility
    Analysis.PrimaryMotionSpace = DominantMotionSpace;
    
    // === EFFECTIVE MOTION SPEED IN PRIMARY SPACE ===
    switch (DominantMotionSpace)
    {
        case EOHReferenceSpace::WorldSpace: 
            Analysis.EffectiveMotionSpeed = MaxSpeedWorld; 
            break;
        case EOHReferenceSpace::LocalSpace: 
            Analysis.EffectiveMotionSpeed = MaxSpeedLocal; 
            break;
        case EOHReferenceSpace::ComponentSpace: 
            Analysis.EffectiveMotionSpeed = MaxSpeedComponent; 
            break;
        default: 
            Analysis.EffectiveMotionSpeed = MaxSpeedWorld; 
            break;
    }
    
    // === ENHANCED KINEMATIC ANALYSIS POPULATION ===
    Analysis.PeakAcceleration = PeakAcceleration; // RESOLVES UNUSED MaxAccel WARNING
    Analysis.PeakJerkMagnitude = PeakJerkMagnitude;
    Analysis.TotalKineticEnergy = TotalKineticEnergy;
    Analysis.AverageChainMass = ActiveChainCount > 0 ? (TotalMass / ActiveChainCount) : 0.0f;
    Analysis.MomentumMagnitude = TotalMomentum;
    Analysis.AngularMomentumMagnitude = TotalAngularMomentum;
    
    // === QUALITY AND CONSISTENCY METRICS ===
    Analysis.ChainMotionQuality = ActiveChainCount > 0 ? (MotionQualitySum / ActiveChainCount) : 0.0f;
    Analysis.MotionConsistencyScore = TotalActiveBones > 0 ? (ConsistencySum / TotalActiveBones) : 0.0f;
    Analysis.CoordinationFactor = ActiveChainCount > 0 ? (CoordinationSum / ActiveChainCount) : 0.0f;
    Analysis.ActiveBoneCount = TotalActiveBones;
    Analysis.TrajectoryConfidence = ActiveChainCount > 0 ? (TrajectoryConfidenceSum / ActiveChainCount) : 0.0f;
    
    // === ATTACK CLASSIFICATION METRICS ===
    Analysis.AttackCurvature = ActiveChainCount > 0 ? (AttackCurvatureSum / ActiveChainCount) : 0.0f;
    Analysis.TimeInAttack = MaxTimeInAttack;
    
    // === ENHANCED ATTACK CLASSIFICATION WITH COMPREHENSIVE CRITERIA ===
    Analysis.bIsCommittedAttack = Analysis.bIsAttacking && 
                                  Analysis.AttackConfidence > 0.7f && 
                                  Analysis.TimeInAttack > 0.2f &&
                                  Analysis.PeakJerkMagnitude > 8000.0f &&
                                  Analysis.EffectiveMotionSpeed > MinAttackSpeed * 1.5f;
                                  
    Analysis.bIsFeintOrGlancing = Analysis.bIsAttacking && 
                                  !Analysis.bIsCommittedAttack &&
                                  (Analysis.AttackCurvature > 0.3f || 
                                   Analysis.MotionConsistencyScore < 0.6f ||
                                   Analysis.TimeInAttack < 0.15f);
    
    // === SPATIAL ANALYSIS ===
    Analysis.ChainCenterOfMass = ActiveChainCount > 0 ? (AccumulatedCenterOfMass / ActiveChainCount) : FVector::ZeroVector;
    Analysis.EffectiveStrikeRadius = MaxEffectiveRadius;
    Analysis.ChainLength = TotalChainLength;
    
    // === ENHANCED ATTACK INTENSITY CALCULATION ===
    if (Analysis.bIsAttacking)
    {
        float SpeedFactor = FMath::Clamp(Analysis.EffectiveMotionSpeed / (MinAttackSpeed * 3.0f), 0.0f, 1.0f);
        float EnergyFactor = FMath::Clamp(TotalKineticEnergy / 5000.0f, 0.0f, 1.0f);
        float JerkFactor = FMath::Clamp(PeakJerkMagnitude / 12000.0f, 0.0f, 1.0f);
        float QualityFactor = FMath::Clamp(Analysis.ChainMotionQuality, 0.0f, 1.0f);
        float CoordinationFactor = FMath::Clamp(Analysis.CoordinationFactor, 0.0f, 1.0f);
        float DirectionalFactor = FMath::Clamp(MaxDirectionalStability, 0.0f, 1.0f);
        
        Analysis.AttackIntensity = (SpeedFactor * 0.25f + 
                                   HighestConfidence * 0.20f + 
                                   EnergyFactor * 0.20f + 
                                   JerkFactor * 0.15f +
                                   QualityFactor * 0.10f +
                                   CoordinationFactor * 0.05f +
                                   DirectionalFactor * 0.05f) * 100.0f;
    }
    
    // === IMPACT POTENTIAL CALCULATION ===
    Analysis.ImpactPotential = Analysis.CalculatePredictedImpactForce();
    
    // === COMPREHENSIVE FINAL VALIDATION ===
    if (Analysis.AttackDirection.ContainsNaN())
    {
        Analysis.AttackDirection = FVector::ZeroVector;
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, Warning, TEXT("AnalyzeCombatState: Invalid AttackDirection detected and corrected"));
        }
    }
    if (Analysis.AttackPrincipalDirection.ContainsNaN())
    {
        Analysis.AttackPrincipalDirection = FVector::ZeroVector;
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, Warning, TEXT("AnalyzeCombatState: Invalid AttackPrincipalDirection detected and corrected"));
        }
    }
    if (Analysis.ChainCenterOfMass.ContainsNaN())
    {
        Analysis.ChainCenterOfMass = FVector::ZeroVector;
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, Warning, TEXT("AnalyzeCombatState: Invalid ChainCenterOfMass detected and corrected"));
        }
    }
    
    // === COMPREHENSIVE DEBUG LOGGING ===
    if (bVerboseLogging && Analysis.bIsAttacking)
    {
        UE_LOG(LogPACManager, Log,
            TEXT("=== ENHANCED COMBAT ANALYSIS ==="));
        UE_LOG(LogPACManager, Log,
            TEXT("Classification: %s | Confidence: %.2f | Primary Space: %s"),
            Analysis.bIsCommittedAttack ? TEXT("COMMITTED") : 
                (Analysis.bIsFeintOrGlancing ? TEXT("FEINT") : TEXT("STANDARD")),
            Analysis.AttackConfidence,
            *UEnum::GetValueAsString(Analysis.PrimaryMotionSpace));
        UE_LOG(LogPACManager, Log,
            TEXT("Multi-Space Speeds: [World:%.1f, Local:%.1f, Component:%.1f] | Effective:%.1f"),
            Analysis.MaxChainSpeedWorld,
            Analysis.MaxChainSpeedLocal,
            Analysis.MaxChainSpeedComponent,
            Analysis.EffectiveMotionSpeed);
        UE_LOG(LogPACManager, Log,
            TEXT("Kinematics: PeakAccel=%.0f | PeakJerk=%.0f | Energy=%.1f | Momentum=%.1f"),
            Analysis.PeakAcceleration,
            Analysis.PeakJerkMagnitude,
            Analysis.TotalKineticEnergy,
            Analysis.MomentumMagnitude);
        UE_LOG(LogPACManager, Log,
            TEXT("Quality Metrics: Motion=%.2f | Consistency=%.2f | Coordination=%.2f | Trajectory=%.2f"),
            Analysis.ChainMotionQuality,
            Analysis.MotionConsistencyScore,
            Analysis.CoordinationFactor,
            Analysis.TrajectoryConfidence);
        UE_LOG(LogPACManager, Log,
            TEXT("Attack Profile: Intensity=%.1f | Curvature=%.2f | Time=%.2fs | ActiveBones=%d | ActiveChains=%d"),
            Analysis.AttackIntensity,
            Analysis.AttackCurvature,
            Analysis.TimeInAttack,
            Analysis.ActiveBoneCount,
            Analysis.ActiveChains.Num());
    }
    
    return Analysis;
}

// === HELPER FUNCTIONS FOR ENHANCED ANALYSIS ===
float UOHPACManager::CalculateAttackIntensity(const FOHCombatAnalysis& Analysis)
{
	if (!Analysis.bIsAttacking) return 0.0f;
    
	// Multi-factor intensity calculation
	float SpeedFactor = FMath::Clamp(Analysis.EffectiveMotionSpeed / 500.0f, 0.0f, 1.0f) * 30.0f;
	float EnergyFactor = FMath::Clamp(Analysis.TotalKineticEnergy / 5000.0f, 0.0f, 1.0f) * 25.0f;
	float QualityFactor = Analysis.ChainMotionQuality * 20.0f;
	float JerkFactor = FMath::Clamp(Analysis.PeakJerkMagnitude / 15000.0f, 0.0f, 1.0f) * 15.0f;
	float CoordinationFactor = Analysis.CoordinationFactor * 10.0f;
    
	return SpeedFactor + EnergyFactor + QualityFactor + JerkFactor + CoordinationFactor;
}

float UOHPACManager::CalculateCoordinationFactor(int32 ActiveBones, int32 ActiveChains)
{
	if (ActiveChains == 0) return 0.0f;
    
	float BonesPerChain = static_cast<float>(ActiveBones) / ActiveChains;
	float ChainDensity = FMath::Clamp(BonesPerChain / 4.0f, 0.0f, 1.0f); // Normalize to 4 bones per chain
    
	// Multi-chain coordination bonus
	float MultiChainBonus = (ActiveChains > 1) ? FMath::Min(ActiveChains * 0.1f, 0.3f) : 0.0f;
    
	return FMath::Clamp(ChainDensity + MultiChainBonus, 0.0f, 1.0f);
}

FOHCombatAnalysis UOHPACManager::AnalyzeCharacterCombatState(ACharacter* Character)
{
    FOHCombatAnalysis Analysis;
    
    // === ENHANCED: COMPREHENSIVE CHARACTER VALIDATION WITH LOGGING ===
    if (!IsValid(Character))
    {
        UE_LOG(LogPACManager, VeryVerbose, TEXT("AnalyzeCharacterCombatState: Invalid character provided"));
        return Analysis; 
    }
    
    // === PAC MANAGER DISCOVERY AND VALIDATION ===
    UOHPACManager* PACManager = Character->FindComponentByClass<UOHPACManager>();
    if (!PACManager)
    {
        UE_LOG(LogPACManager, VeryVerbose, TEXT("AnalyzeCharacterCombatState: No PAC Manager found on character '%s'"), *Character->GetName());
        return Analysis;
    }
    
    if (!PACManager->bEnableCollisionImpactSystem || !PACManager->bIsInitialized)
    {
        UE_LOG(LogPACManager, VeryVerbose, TEXT("AnalyzeCharacterCombatState: PAC Manager not enabled or initialized on character '%s'"), *Character->GetName());
        return Analysis;
    }
    
    // === CORE ANALYSIS DELEGATION ===
    Analysis = PACManager->AnalyzeCombatState();
    
    // === ENHANCED: CHARACTER-SPECIFIC CONTEXT WITH COMPREHENSIVE VALIDATION ===
    if (Analysis.bIsAttacking)
    {
        // === MOVEMENT SYSTEM INTEGRATION WITH ENHANCED NaN VALIDATION ===
        if (UCharacterMovementComponent* CharMove = Character->GetCharacterMovement())
        {
            FVector CharacterVelocity = CharMove->Velocity;
            FVector AttackDirection = Analysis.AttackDirection;
            
            // Comprehensive validation before vector operations
            if (!CharacterVelocity.ContainsNaN() && !AttackDirection.ContainsNaN() && 
                !CharacterVelocity.IsNearlyZero(1.0f) && !AttackDirection.IsNearlyZero(0.001f) &&
                CharacterVelocity.Size() < 10000.0f && AttackDirection.IsNormalized())
            {
                float MomentumAlignment = FVector::DotProduct(
                    CharacterVelocity.GetSafeNormal(), 
                    AttackDirection.GetSafeNormal()
                );
                
                // Validate alignment calculation result
                if (FMath::IsFinite(MomentumAlignment) && FMath::Abs(MomentumAlignment) <= 1.0f)
                {
                    // === ATTACK INTENSITY MODULATION WITH VALIDATED METRICS ===
                    if (MomentumAlignment > 0.5f) // Forward momentum
                    {
                        float MomentumBonus = MomentumAlignment * 0.2f;
                        Analysis.AttackIntensity += MomentumBonus * 100.0f;
                        Analysis.bIsCommittedAttack = Analysis.bIsCommittedAttack || (MomentumAlignment > 0.8f);
                        
                        // Enhanced impact potential with momentum contribution
                        float VelocityMagnitude = CharacterVelocity.Size();
                        if (FMath::IsFinite(VelocityMagnitude) && VelocityMagnitude > 0.0f && VelocityMagnitude < 5000.0f)
                        {
                            float MomentumContribution = VelocityMagnitude * MomentumAlignment * 0.1f;
                            Analysis.ImpactPotential += MomentumContribution;
                            
                            if (PACManager->bVerboseLogging)
                            {
                                UE_LOG(LogPACManager, VeryVerbose,
                                    TEXT("Character momentum contribution: Alignment=%.2f, Velocity=%.1f, Contribution=%.1f"),
                                    MomentumAlignment, VelocityMagnitude, MomentumContribution);
                            }
                        }
                    }
                    else if (MomentumAlignment < -0.3f) // Retreating
                    {
                        Analysis.bIsFeintOrGlancing = true;
                        Analysis.AttackIntensity *= 0.7f;
                        
                        if (PACManager->bVerboseLogging)
                        {
                            UE_LOG(LogPACManager, VeryVerbose,
                                TEXT("Retreating motion detected: Alignment=%.2f, marking as feint"), MomentumAlignment);
                        }
                    }
                }
            }
            
            // === MOVEMENT STATE ANALYSIS WITH COMPREHENSIVE VALIDATION ===
            FVector CurrentAcceleration = CharMove->GetCurrentAcceleration();
            if (!CurrentAcceleration.ContainsNaN() && CurrentAcceleration.Size() < 50000.0f)
            {
                bool bIsAccelerating = CurrentAcceleration.Size() > 100.0f;
                bool bIsDecelerating = false;
                
                if (!CharacterVelocity.ContainsNaN() && !CurrentAcceleration.IsNearlyZero(1.0f))
                {
                    float VelAccelDot = FVector::DotProduct(CharacterVelocity, CurrentAcceleration);
                    if (FMath::IsFinite(VelAccelDot))
                    {
                        bIsDecelerating = VelAccelDot < 0.0f;
                    }
                }
                
                // === ENHANCED: ATTACK CONFIDENCE MODULATION BASED ON ACCELERATION STATE ===
                if (bIsAccelerating && !CharacterVelocity.ContainsNaN())
                {
                    float MomentumAlignment = FVector::DotProduct(
                        CharacterVelocity.GetSafeNormal(), 
                        Analysis.AttackDirection.GetSafeNormal()
                    );
                    if (FMath::IsFinite(MomentumAlignment) && MomentumAlignment > 0.3f)
                    {
                        float ConfidenceBoost = 1.15f;
                        Analysis.AttackConfidence = FMath::Min(Analysis.AttackConfidence * ConfidenceBoost, 1.0f);
                        
                        // Enhanced attack commitment detection for accelerating attacks
                        if (MomentumAlignment > 0.6f && CurrentAcceleration.Size() > 300.0f)
                        {
                            Analysis.bIsCommittedAttack = true;
                        }
                    }
                }
                else if (bIsDecelerating)
                {
                    Analysis.bIsFeintOrGlancing = Analysis.bIsFeintOrGlancing || (Analysis.AttackConfidence < 0.7f);
                    
                    // Reduce intensity for decelerating attacks
                    if (Analysis.AttackConfidence < 0.6f)
                    {
                        Analysis.AttackIntensity *= 0.85f;
                    }
                }
            }
            
            // === SPEED-BASED ATTACK CLASSIFICATION WITH ENHANCED VALIDATION ===
            if (!CharacterVelocity.ContainsNaN() && CharacterVelocity.Size() < 10000.0f)
            {
                float CharacterSpeed = CharacterVelocity.Size();
                if (FMath::IsFinite(CharacterSpeed) && CharacterSpeed >= 0.0f)
                {
                    if (CharacterSpeed > CharMove->MaxWalkSpeed * 0.8f) // Sprint speed
                    {
                        Analysis.AttackIntensity *= 1.2f;
                        Analysis.ImpactPotential *= 1.3f;
                        
                        // Mark as committed attack if sprinting into attack
                        if (!Analysis.AttackDirection.IsZero())
                        {
                            float SprintAlignment = FVector::DotProduct(
                                CharacterVelocity.GetSafeNormal(), 
                                Analysis.AttackDirection.GetSafeNormal()
                            );
                            if (SprintAlignment > 0.7f)
                            {
                                Analysis.bIsCommittedAttack = true;
                            }
                        }
                    }
                    else if (CharacterSpeed < 50.0f) // Near stationary
                    {
                        // === ENHANCED: ISOLATED LIMB MOTION ANALYSIS FOR STATIONARY CHARACTERS ===
                        float LocalSpeedBonus = FMath::Max(Analysis.MaxChainSpeedLocal - Analysis.MaxChainSpeedWorld, 0.0f);
                        if (FMath::IsFinite(LocalSpeedBonus) && LocalSpeedBonus > 100.0f)
                        {
                            // Significant local motion while stationary indicates isolated limb attack
                            Analysis.AttackIntensity += LocalSpeedBonus * 0.05f;
                            Analysis.EffectiveMotionSpeed = Analysis.MaxChainSpeedLocal;
                            
                            // Override primary motion space for isolated limb detection
                            Analysis.PrimaryMotionSpace = EOHReferenceSpace::LocalSpace;
                            
                            if (PACManager->bVerboseLogging)
                            {
                                UE_LOG(LogPACManager, VeryVerbose,
                                    TEXT("Isolated limb motion detected: LocalBonus=%.1f, CharSpeed=%.1f"), 
                                    LocalSpeedBonus, CharacterSpeed);
                            }
                        }
                    }
                }
            }
        }
        
        // === OH MOVEMENT COMPONENT INTEGRATION WITH ENHANCED VALIDATION ===
        if (UOHMovementComponent* OHMovement = Character->FindComponentByClass<UOHMovementComponent>())
        {
            // === COMBAT PUSHBACK STATE INFLUENCE ===
            if (OHMovement->bCombatPushbackActive)
            {
                Analysis.AttackConfidence *= 0.9f;
                Analysis.MotionConsistencyScore *= 0.85f;
                
                // Reduce attack intensity during pushback recovery
                Analysis.AttackIntensity *= 0.95f;
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Combat pushback active - reducing attack metrics"));
                }
            }
            
            // === PHYSICS IMPULSE DATA INTEGRATION WITH ENHANCED VALIDATION ===
            if (!OHMovement->AccumulatedPhysicsImpulse.ContainsNaN() && 
                OHMovement->AccumulatedPhysicsImpulse.Size() < 100000.0f)
            {
                float ImpulseSize = OHMovement->AccumulatedPhysicsImpulse.Size();
                if (FMath::IsFinite(ImpulseSize) && ImpulseSize > 200.0f)
                {
                    float PhysicsBonus = FMath::Clamp(ImpulseSize / 1000.0f, 0.0f, 0.3f);
                    Analysis.ImpactPotential += PhysicsBonus * 50.0f;
                    
                    // High physics impulse indicates significant motion
                    if (ImpulseSize > 800.0f)
                    {
                        Analysis.AttackIntensity *= 1.1f;
                    }
                    
                    if (PACManager->bVerboseLogging)
                    {
                        UE_LOG(LogPACManager, VeryVerbose,
                            TEXT("Physics impulse contribution: Size=%.1f, Bonus=%.2f"), 
                            ImpulseSize, PhysicsBonus);
                    }
                }
            }
        }
        
        // === SKELETAL MESH COMPONENT VALIDATION WITH ENHANCED NaN PROTECTION ===
        if (USkeletalMeshComponent* CharacterMesh = Character->GetMesh())
        {
            FVector MeshVelocity = CharacterMesh->GetComponentVelocity();
            FVector MeshAngularVelocity = CharacterMesh->GetPhysicsAngularVelocityInDegrees();
            
            // === MESH VELOCITY CONTRIBUTION WITH COMPREHENSIVE VALIDATION ===
            if (!MeshVelocity.ContainsNaN() && !MeshVelocity.IsNearlyZero(1.0f) && MeshVelocity.Size() < 10000.0f)
            {
                float MeshSpeedContribution = MeshVelocity.Size();
                if (FMath::IsFinite(MeshSpeedContribution) && MeshSpeedContribution > 0.0f)
                {
                    Analysis.EffectiveMotionSpeed += MeshSpeedContribution * 0.1f;
                    
                    // Add mesh velocity to momentum calculations
                    if (MeshSpeedContribution > 100.0f)
                    {
                        Analysis.MomentumMagnitude += MeshSpeedContribution * 0.05f;
                    }
                }
            }
            
            // === ANGULAR VELOCITY CONTRIBUTION WITH ENHANCED VALIDATION ===
            if (!MeshAngularVelocity.ContainsNaN() && MeshAngularVelocity.Size() < 36000.0f) // 36000 deg/s max
            {
                float AngularSpeed = MeshAngularVelocity.Size();
                if (FMath::IsFinite(AngularSpeed) && AngularSpeed > 50.0f && AngularSpeed < 3600.0f)
                {
                    Analysis.AttackCurvature += AngularSpeed * 0.01f;
                    Analysis.CoordinationFactor = FMath::Min(Analysis.CoordinationFactor + 0.1f, 1.0f);
                    
                    // High angular velocity indicates spinning attacks
                    if (AngularSpeed > 180.0f) // 180 deg/s threshold
                    {
                        Analysis.AttackIntensity += AngularSpeed * 0.02f;
                        Analysis.bIsCommittedAttack = Analysis.bIsCommittedAttack || (AngularSpeed > 360.0f);
                    }
                    
                    if (PACManager->bVerboseLogging)
                    {
                        UE_LOG(LogPACManager, VeryVerbose,
                            TEXT("Angular velocity contribution: Speed=%.1f deg/s, Curvature=%.3f"), 
                            AngularSpeed, Analysis.AttackCurvature);
                    }
                }
            }
        }
        
        // === ENHANCED: MULTI-SPACE MOTION VALIDATION AND OPTIMIZATION ===
        
        // Comprehensive speed metrics validation before comparison
        bool bValidSpeedMetrics = FMath::IsFinite(Analysis.MaxChainSpeedLocal) && 
                                 FMath::IsFinite(Analysis.MaxChainSpeedWorld) && 
                                 FMath::IsFinite(Analysis.MaxChainSpeedComponent) &&
                                 Analysis.MaxChainSpeedLocal >= 0.0f &&
                                 Analysis.MaxChainSpeedWorld >= 0.0f &&
                                 Analysis.MaxChainSpeedComponent >= 0.0f &&
                                 Analysis.MaxChainSpeedLocal < 20000.0f &&
                                 Analysis.MaxChainSpeedWorld < 20000.0f &&
                                 Analysis.MaxChainSpeedComponent < 20000.0f;
        
        if (bValidSpeedMetrics)
        {
            // === ISOLATED LIMB MOTION DETECTION WITH ENHANCED THRESHOLDS ===
            if (Analysis.MaxChainSpeedLocal > Analysis.MaxChainSpeedWorld * 1.5f)
            {
                // Significant local motion indicates isolated limb attack
                Analysis.PrimaryMotionSpace = EOHReferenceSpace::LocalSpace;
                Analysis.EffectiveMotionSpeed = Analysis.MaxChainSpeedLocal;
                Analysis.AttackConfidence = FMath::Min(Analysis.AttackConfidence * 1.1f, 1.0f);
                
                // Enhanced classification for isolated limb motion
                if (Analysis.MaxChainSpeedLocal > Analysis.MaxChainSpeedWorld * 2.0f)
                {
                    Analysis.AttackIntensity *= 1.15f; // Bonus for pure limb motion
                }
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Isolated limb motion: Local=%.1f, World=%.1f, Ratio=%.2f"), 
                        Analysis.MaxChainSpeedLocal, Analysis.MaxChainSpeedWorld,
                        Analysis.MaxChainSpeedLocal / FMath::Max(Analysis.MaxChainSpeedWorld, 1.0f));
                }
            }
            else if (Analysis.MaxChainSpeedComponent > FMath::Max(Analysis.MaxChainSpeedWorld, Analysis.MaxChainSpeedLocal))
            {
                Analysis.PrimaryMotionSpace = EOHReferenceSpace::ComponentSpace;
                Analysis.EffectiveMotionSpeed = Analysis.MaxChainSpeedComponent;
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Component space dominant: Component=%.1f"), Analysis.MaxChainSpeedComponent);
                }
            }
        }
        else if (PACManager->bVerboseLogging)
        {
            UE_LOG(LogPACManager, Warning,
                TEXT("Invalid speed metrics detected: Local=%.1f, World=%.1f, Component=%.1f"), 
                Analysis.MaxChainSpeedLocal, Analysis.MaxChainSpeedWorld, Analysis.MaxChainSpeedComponent);
        }
        
        // === ENHANCED: ATTACK TIMING ANALYSIS WITH COMPREHENSIVE VALIDATION ===
        if (FMath::IsFinite(Analysis.TimeInAttack) && FMath::IsFinite(Analysis.PeakJerkMagnitude) &&
            Analysis.TimeInAttack >= 0.0f && Analysis.PeakJerkMagnitude >= 0.0f)
        {
            if (Analysis.TimeInAttack > 0.5f)
            {
                // Sustained attack classification
                Analysis.bIsCommittedAttack = true;
                Analysis.AttackIntensity *= 1.15f;
                
                // Long attacks may become less effective
                if (Analysis.TimeInAttack > 1.0f)
                {
                    Analysis.AttackIntensity *= 0.95f; // Slight penalty for very long attacks
                }
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Sustained attack: Duration=%.2fs"), Analysis.TimeInAttack);
                }
            }
            else if (Analysis.TimeInAttack < 0.1f && Analysis.PeakJerkMagnitude > 12000.0f)
            {
                // Quick, sharp motion - explosive attack
                Analysis.AttackIntensity *= 1.25f;
                Analysis.ImpactPotential *= 1.2f;
                
                // Very quick, high-jerk attacks are committed
                if (Analysis.PeakJerkMagnitude > 18000.0f)
                {
                    Analysis.bIsCommittedAttack = true;
                }
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Explosive attack: Duration=%.3fs, Jerk=%.0f"), 
                        Analysis.TimeInAttack, Analysis.PeakJerkMagnitude);
                }
            }
        }
        
        // === ENHANCED: CHARACTER STATE INTEGRATION WITH GAMEPLAY CONTEXT ===
        
        // Health/stamina influence (if available through interfaces)
       // if (Character->Implements<UOHCombatInterface>())
        {
            // Interface-based enhancements could be added here
            // This allows for modular extension without hard dependencies
        }
        
        // === FINAL ATTACK CLASSIFICATION REFINEMENT ===
        
        // Enhance feint detection with multi-factor analysis
        if (!Analysis.bIsCommittedAttack && !Analysis.bIsFeintOrGlancing)
        {
            bool bLowIntensity = Analysis.AttackIntensity < 30.0f;
            bool bLowConfidence = Analysis.AttackConfidence < 0.6f;
            bool bHighCurvature = Analysis.AttackCurvature > 0.4f;
            bool bInconsistentMotion = Analysis.MotionConsistencyScore < 0.5f;
            
            if ((bLowIntensity && bLowConfidence) || (bHighCurvature && bInconsistentMotion))
            {
                Analysis.bIsFeintOrGlancing = true;
                
                if (PACManager->bVerboseLogging)
                {
                    UE_LOG(LogPACManager, VeryVerbose,
                        TEXT("Feint classification: LowIntensity=%s, LowConf=%s, HighCurve=%s, Inconsistent=%s"),
                        bLowIntensity ? TEXT("Yes") : TEXT("No"),
                        bLowConfidence ? TEXT("Yes") : TEXT("No"),
                        bHighCurvature ? TEXT("Yes") : TEXT("No"),
                        bInconsistentMotion ? TEXT("Yes") : TEXT("No"));
                }
            }
        }
    }
    
    // === COMPREHENSIVE NUMERICAL VALIDATION AND CLAMPING ===
    PACManager->ValidateAndClampAnalysisMetrics(Analysis);
    
    // === FINAL LOGGING FOR CHARACTER-SPECIFIC ANALYSIS ===
    if (PACManager->bVerboseLogging && Analysis.bIsAttacking)
    {
        UE_LOG(LogPACManager, Log,
            TEXT("=== CHARACTER COMBAT ANALYSIS [%s] ==="),
            *Character->GetName());
        UE_LOG(LogPACManager, Log,
            TEXT("Classification: %s | Confidence: %.2f | Primary Space: %s"),
            Analysis.bIsCommittedAttack ? TEXT("COMMITTED") : 
                (Analysis.bIsFeintOrGlancing ? TEXT("FEINT") : TEXT("STANDARD")),
            Analysis.AttackConfidence,
            *UEnum::GetValueAsString(Analysis.PrimaryMotionSpace));
        UE_LOG(LogPACManager, Log,
            TEXT("Multi-Space Speeds: [World:%.1f, Local:%.1f, Component:%.1f] | Effective:%.1f"),
            Analysis.MaxChainSpeedWorld,
            Analysis.MaxChainSpeedLocal,
            Analysis.MaxChainSpeedComponent,
            Analysis.EffectiveMotionSpeed);
        UE_LOG(LogPACManager, Log,
            TEXT("Character Integration: Intensity=%.1f | Impact=%.1f | ActiveChains=%d"),
            Analysis.AttackIntensity,
            Analysis.ImpactPotential,
            Analysis.ActiveChains.Num());
    }
    
    return Analysis;
}

// === MODERNIZED VALIDATION HELPER FUNCTION ===
void UOHPACManager::ValidateAndClampAnalysisMetrics(FOHCombatAnalysis& Analysis)
{
    // === COMPREHENSIVE NaN VALIDATION FOR SCALAR FIELDS ===
    if (FMath::IsNaN(Analysis.AttackConfidence) || Analysis.AttackConfidence < 0.0f)
    {
        Analysis.AttackConfidence = 0.0f;
    }
    else
    {
        Analysis.AttackConfidence = FMath::Clamp(Analysis.AttackConfidence, 0.0f, 1.0f);
    }
    
    if (FMath::IsNaN(Analysis.AttackIntensity) || Analysis.AttackIntensity < 0.0f)
    {
        Analysis.AttackIntensity = 0.0f;
    }
    else
    {
        Analysis.AttackIntensity = FMath::Clamp(Analysis.AttackIntensity, 0.0f, 200.0f);
    }
    
    if (FMath::IsNaN(Analysis.ImpactPotential) || Analysis.ImpactPotential < 0.0f)
    {
        Analysis.ImpactPotential = 0.0f;
    }
    else
    {
        Analysis.ImpactPotential = FMath::Clamp(Analysis.ImpactPotential, 0.0f, 150.0f);
    }
    
    if (FMath::IsNaN(Analysis.MotionConsistencyScore) || Analysis.MotionConsistencyScore < 0.0f)
    {
        Analysis.MotionConsistencyScore = 0.0f;
    }
    else
    {
        Analysis.MotionConsistencyScore = FMath::Clamp(Analysis.MotionConsistencyScore, 0.0f, 1.0f);
    }
    
    if (FMath::IsNaN(Analysis.CoordinationFactor) || Analysis.CoordinationFactor < 0.0f)
    {
        Analysis.CoordinationFactor = 0.0f;
    }
    else
    {
        Analysis.CoordinationFactor = FMath::Clamp(Analysis.CoordinationFactor, 0.0f, 1.0f);
    }
    
    // === COMPREHENSIVE VECTOR FIELD VALIDATION ===
    if (Analysis.AttackDirection.ContainsNaN())
    {
        Analysis.AttackDirection = FVector::ZeroVector;
    }
    
    if (Analysis.AttackPrincipalDirection.ContainsNaN())
    {
        Analysis.AttackPrincipalDirection = FVector::ZeroVector;
    }
    
    if (Analysis.ChainCenterOfMass.ContainsNaN())
    {
        Analysis.ChainCenterOfMass = FVector::ZeroVector;
    }
    
    // === SPEED METRIC VALIDATION ===
    if (FMath::IsNaN(Analysis.MaxChainSpeed) || Analysis.MaxChainSpeed < 0.0f)
    {
        Analysis.MaxChainSpeed = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.MaxChainSpeedWorld) || Analysis.MaxChainSpeedWorld < 0.0f)
    {
        Analysis.MaxChainSpeedWorld = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.MaxChainSpeedLocal) || Analysis.MaxChainSpeedLocal < 0.0f)
    {
        Analysis.MaxChainSpeedLocal = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.MaxChainSpeedComponent) || Analysis.MaxChainSpeedComponent < 0.0f)
    {
        Analysis.MaxChainSpeedComponent = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.EffectiveMotionSpeed) || Analysis.EffectiveMotionSpeed < 0.0f)
    {
        Analysis.EffectiveMotionSpeed = 0.0f;
    }
    
    // === ENERGY AND KINEMATIC VALIDATION ===
    if (FMath::IsNaN(Analysis.TotalKineticEnergy) || Analysis.TotalKineticEnergy < 0.0f)
    {
        Analysis.TotalKineticEnergy = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.PeakAcceleration) || Analysis.PeakAcceleration < 0.0f)
    {
        Analysis.PeakAcceleration = 0.0f;
    }
    
    if (FMath::IsNaN(Analysis.PeakJerkMagnitude) || Analysis.PeakJerkMagnitude < 0.0f)
    {
        Analysis.PeakJerkMagnitude = 0.0f;
    }
    
    // === REFERENCE SPACE OPTIMIZATION BASED ON VALIDATED METRICS ===
    if (Analysis.MaxChainSpeedLocal > Analysis.MaxChainSpeedWorld * 2.0f)
    {
        Analysis.PrimaryMotionSpace = EOHReferenceSpace::LocalSpace;
        Analysis.EffectiveMotionSpeed = Analysis.MaxChainSpeedLocal;
    }
    else if (Analysis.MaxChainSpeedComponent > FMath::Max(Analysis.MaxChainSpeedWorld, Analysis.MaxChainSpeedLocal))
    {
        Analysis.PrimaryMotionSpace = EOHReferenceSpace::ComponentSpace;
        Analysis.EffectiveMotionSpeed = Analysis.MaxChainSpeedComponent;
    }
}

float UOHPACManager::CalculateMovementAlignmentBonus(const FVector& CharacterVelocity, const FVector& AttackDirection)
{
	if (CharacterVelocity.IsZero() || AttackDirection.IsZero()) return 0.0f;
    
	float Alignment = FVector::DotProduct(CharacterVelocity.GetSafeNormal(), AttackDirection.GetSafeNormal());
	float SpeedFactor = FMath::Clamp(CharacterVelocity.Size() / 500.0f, 0.0f, 1.0f);
    
	return FMath::Clamp(Alignment * SpeedFactor * 0.3f, -0.2f, 0.3f);
}

float UOHPACManager::CalculatePhysicsContribution(UOHMovementComponent* MovementComponent)
{
	if (!MovementComponent) return 0.0f;
    
	float PhysicsImpulse = MovementComponent->AccumulatedPhysicsImpulse.Size();
	float CombatPushbackContribution = MovementComponent->bCombatPushbackActive ? 0.1f : 0.0f;
    
	return FMath::Clamp((PhysicsImpulse / 1000.0f) + CombatPushbackContribution, 0.0f, 0.5f);
}


FString UOHPACManager::GetBoneSide(FName BoneName)
{
	FString BoneStr = BoneName.ToString();

	// Extract side identifier
	if (BoneStr.EndsWith("_l"))
	{
		return TEXT("l");
	}
	if (BoneStr.EndsWith("_r"))
	{
		return TEXT("r");
	}
	return TEXT(""); // Center bones like spine
}


float UOHPACManager::EstimateBoneRadius(FName BoneName) const
{
	// Try to get actual radius from physics body
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		// Get bounds of the physics body
		FBox Bounds = Body->GetBodyBounds();
		FVector Extent = Bounds.GetExtent();

		// Use average of smallest two dimensions as radius
		float X = Extent.X;
		float Y = Extent.Y;
		float Z = Extent.Z;

		// Sort to find smallest two
		if (X > Y)
		{
			Swap(X, Y);
		}
		if (Y > Z)
		{
			Swap(Y, Z);
		}
		if (X > Y)
		{
			Swap(X, Y);
		}

		// Average of two smallest dimensions
		return (X + Y) * 0.5f;
	}

	// Fallback to estimates based on bone name
	FString BoneStr = BoneName.ToString().ToLower();

	// Hands/Feet
	if (BoneStr.Contains("hand"))
	{
		return 8.0f;
	}
	if (BoneStr.Contains("foot"))
	{
		return 10.0f;
	}

	// Head
	if (BoneStr.Contains("head"))
	{
		return 12.0f;
	}

	// Torso
	if (BoneStr.Contains("spine") || BoneStr.Contains("chest"))
	{
		return 15.0f;
	}
	if (BoneStr.Contains("pelvis") || BoneStr.Contains("hips"))
	{
		return 20.0f;
	}

	// Arms
	if (BoneStr.Contains("upperarm") || BoneStr.Contains("shoulder"))
	{
		return 12.0f;
	}
	if (BoneStr.Contains("lowerarm") || BoneStr.Contains("forearm"))
	{
		return 10.0f;
	}

	// Legs
	if (BoneStr.Contains("thigh") || BoneStr.Contains("upperleg"))
	{
		return 15.0f;
	}
	if (BoneStr.Contains("calf") || BoneStr.Contains("shin") || BoneStr.Contains("lowerleg"))
	{
		return 12.0f;
	}

	// Default
	return 10.0f;
}

uint64 UOHPACManager::GetCombatHitID(FName AttackerBone, FName DefenderBone)
{
	// Create deterministic ID for this hit pair
	uint32 AttackerHash = GetTypeHash(AttackerBone);
	uint32 DefenderHash = GetTypeHash(DefenderBone);

	// Combine hashes into 64-bit ID
	return (static_cast<uint64>(AttackerHash) << 32) | static_cast<uint64>(DefenderHash);
}

float UOHPACManager::EstimateBoneMass(FName BoneName)
{
	// Simple mass estimates based on bone type
	// These values create good momentum scaling

	FString BoneStr = BoneName.ToString().ToLower();

	// Hands - lighter, faster
	if (BoneStr.Contains("hand"))
	{
		return 0.5f;
	}

	// Forearms
	if (BoneStr.Contains("lowerarm") || BoneStr.Contains("forearm"))
	{
		return 1.5f;
	}

	// Upper arms - heavier
	if (BoneStr.Contains("upperarm"))
	{
		return 2.5f;
	}

	// Feet - medium weight
	if (BoneStr.Contains("foot"))
	{
		return 0.8f;
	}

	// Calves
	if (BoneStr.Contains("calf"))
	{
		return 3.0f;
	}

	// Thighs - heaviest limb segments
	if (BoneStr.Contains("thigh"))
	{
		return 5.0f;
	}

	// Torso bones
	if (BoneStr.Contains("spine") || BoneStr.Contains("chest"))
	{
		return 8.0f;
	}

	// Head
	if (BoneStr.Contains("head"))
	{
		return 4.5f;
	}

	// Default
	return 2.0f;
}


bool UOHPACManager::IsCoreBone(FName BoneName) const
{
	return CoreBones.Contains(BoneName);
}

bool UOHPACManager::IsExtremityBone(FName BoneName)
{
	FString BoneStr = BoneName.ToString().ToLower();

	// Extremity bones are limbs that can block/guard
	return BoneStr.Contains("hand") ||
		BoneStr.Contains("lowerarm") ||
		BoneStr.Contains("forearm") ||
		BoneStr.Contains("upperarm") ||
		BoneStr.Contains("foot") ||
		BoneStr.Contains("calf") ||
		BoneStr.Contains("shin") ||
		BoneStr.Contains("thigh");
}

bool UOHPACManager::IsExtremityProtectingCore(
	FName ExtremityBone,
	UOHPACManager* DefenderManager,
	const FVector& AttackDirection) const
{
	if (!DefenderManager || !DefenderManager->SkeletalMesh)
	{
		return false;
	}

	// Get extremity position
	FVector ExtremityPos = DefenderManager->SkeletalMesh->GetBoneLocation(ExtremityBone);

	// Check if any core bones are behind this extremity relative to attack direction
	bool bIsProtecting = false;

	for (const FName& CoreBone : CoreBones)
	{
		// Skip if defender doesn't have this core bone simulating
		if (!DefenderManager->SimulatableBones.Contains(CoreBone))
		{
			continue;
		}

		FVector CorePos = DefenderManager->SkeletalMesh->GetBoneLocation(CoreBone);

		// Vector from extremity to core
		FVector ExtremityToCore = (CorePos - ExtremityPos).GetSafeNormal();

		// Check if core is in the direction of the attack (behind extremity)
		float AlignmentWithAttack = FVector::DotProduct(AttackDirection, ExtremityToCore);

		if (AlignmentWithAttack > 0.5f) // Core is somewhat aligned with attack direction
		{
			// Project positions onto attack direction to check ordering
			float ExtremityProjection = FVector::DotProduct(ExtremityPos, AttackDirection);
			float CoreProjection = FVector::DotProduct(CorePos, AttackDirection);

			// If extremity projection is less than core projection, 
			// extremity is between attacker and core
			if (ExtremityProjection < CoreProjection)
			{
				// Additional check: is the extremity actually close enough to block?
				FVector ClosestPointOnAttackLine = ExtremityPos +
					AttackDirection * FVector::DotProduct(CorePos - ExtremityPos, AttackDirection);

				float DistanceFromLine = FVector::Dist(CorePos, ClosestPointOnAttackLine);

				// If core is within a reasonable distance from attack line, extremity is blocking
				float BlockingRadius = 50.0f; // Adjust based on your scale
				if (DistanceFromLine < BlockingRadius)
				{
					bIsProtecting = true;

					if (bVerboseLogging)
					{
						UE_LOG(LogPACManager, Log,
						       TEXT("Extremity %s is protecting core %s (Distance: %.1f)"),
						       *ExtremityBone.ToString(),
						       *CoreBone.ToString(),
						       DistanceFromLine);
					}

					break; // Found at least one core being protected
				}
			}
		}
	}

	return bIsProtecting;
}


FCombatTargetCandidate UOHPACManager::SelectBestTargetFromChain(
	const FOHCombatChainData& AttackingChain,
	UOHPACManager* DefenderManager,
	const TArray<FCombatTargetCandidate>& Candidates)
{
	if (Candidates.Num() == 0)
	{
		return FCombatTargetCandidate();
	}

	if (Candidates.Num() == 1)
	{
		return Candidates[0];
	}

	// Enhanced selection using chain data
	constexpr float TimeWindow = 0.15f; // 150ms window for punch-through

	// Group candidates by time windows
	TArray<TArray<FCombatTargetCandidate>> TimeGroups;
	TArray<FCombatTargetCandidate> SortedCandidates = Candidates;

	// Sort by time to impact
	SortedCandidates.Sort([](const FCombatTargetCandidate& A, const FCombatTargetCandidate& B)
	{
		return A.TimeToImpact < B.TimeToImpact;
	});

	// Group candidates by time proximity
	for (const FCombatTargetCandidate& Candidate : SortedCandidates)
	{
		bool bAddedToGroup = false;

		for (TArray<FCombatTargetCandidate>& Group : TimeGroups)
		{
			if (Group.Num() > 0)
			{
				float GroupTime = Group[0].TimeToImpact;
				if (FMath::Abs(Candidate.TimeToImpact - GroupTime) < TimeWindow)
				{
					Group.Add(Candidate);
					bAddedToGroup = true;
					break;
				}
			}
		}

		if (!bAddedToGroup)
		{
			TArray<FCombatTargetCandidate> NewGroup;
			NewGroup.Add(Candidate);
			TimeGroups.Add(NewGroup);
		}
	}

	// Enhanced scoring using chain metrics
	FCombatTargetCandidate BestTarget;
	float BestScore = -1.0f;

	for (int32 GroupIdx = 0; GroupIdx < TimeGroups.Num(); GroupIdx++)
	{
		const TArray<FCombatTargetCandidate>& Group = TimeGroups[GroupIdx];

		// Find best target in this time group
		FCombatTargetCandidate GroupBest;
		float GroupBestScore = -1.0f;

		for (const FCombatTargetCandidate& Candidate : Group)
		{
			float Score = CalculateTargetScore(Candidate, AttackingChain, DefenderManager);

			// Enhanced punch-through logic using chain trajectory
			if (Candidate.bIsExtremity && bUsePunchThroughTargeting)
			{
				// Check if chain trajectory continues past this extremity
				float PenetrationScore = CalculatePenetrationScore(
					Candidate,
					AttackingChain,
					TimeGroups,
					GroupIdx
				);

				if (PenetrationScore > 0.5f)
				{
					// This extremity is likely to be punched through
					Score *= (1.0f - PenetrationScore * 0.7f); // Reduce score significantly

					if (bVerboseLogging)
					{
						UE_LOG(LogPACManager, Warning,
						       TEXT("Extremity %s has high penetration score: %.2f"),
						       *Candidate.BoneName.ToString(),
						       PenetrationScore);
					}
				}
			}

			if (Score > GroupBestScore)
			{
				GroupBestScore = Score;
				GroupBest = Candidate;
			}
		}

		// Special handling for punch-through scenarios
		if (GroupBest.bIsExtremity && bUsePunchThroughTargeting)
		{
			// Look for core bones in future time groups
			FCombatTargetCandidate BestCoreAhead;
			float BestCoreScore = 0.0f;
			bool bFoundCore = false;

			for (int32 FutureIdx = GroupIdx + 1; FutureIdx < TimeGroups.Num(); FutureIdx++)
			{
				for (const FCombatTargetCandidate& FutureCandidate : TimeGroups[FutureIdx])
				{
					if (FutureCandidate.bIsCore)
					{
						float TimeDiff = FutureCandidate.TimeToImpact - GroupBest.TimeToImpact;

						// Check if this core is in punch-through range
						if (TimeDiff < 0.2f && IsCoreInTrajectory(FutureCandidate, AttackingChain))
						{
							float CoreScore = CalculateTargetScore(
								FutureCandidate,
								AttackingChain,
								DefenderManager
							);

							// Boost score for punch-through potential
							CoreScore *= (1.5f + AttackingChain.ChainMomentum.Size() / 1000.0f);

							if (CoreScore > BestCoreScore)
							{
								BestCoreScore = CoreScore;
								BestCoreAhead = FutureCandidate;
								bFoundCore = true;
							}
						}
					}
				}
			}

			if (bFoundCore && BestCoreScore > GroupBestScore * 1.5f)
			{
				// Skip the extremity and target the core
				if (bVerboseLogging)
				{
					UE_LOG(LogPACManager, Warning,
					       TEXT("PUNCH-THROUGH: Bypassing %s (score:%.2f) to hit %s (score:%.2f)"),
					       *GroupBest.BoneName.ToString(),
					       GroupBestScore,
					       *BestCoreAhead.BoneName.ToString(),
					       BestCoreScore);
				}

				return BestCoreAhead;
			}
		}

		// Update overall best
		if (GroupBestScore > BestScore)
		{
			BestScore = GroupBestScore;
			BestTarget = GroupBest;
		}

		// If we found a high-scoring core, stop looking
		if (BestTarget.bIsCore && BestScore > CoreBonePriority * 0.8f)
		{
			break;
		}
	}

	return BestTarget;
}

// Helper function to calculate target score using chain data
float UOHPACManager::CalculateTargetScore(
	const FCombatTargetCandidate& Candidate,
	const FOHCombatChainData& AttackingChain,
	UOHPACManager* DefenderManager)
{
	float Score = Candidate.Priority;

	// Core vs extremity base scoring
	if (Candidate.bIsCore)
	{
		Score *= 3.0f; // Strong preference for core
	}
	else if (Candidate.bIsExtremity)
	{
		Score *= ExtremityPenalty;

		// Check if blocking using chain trajectory
		if (Candidate.bIsBlocking)
		{
			Score *= 0.5f; // Further reduce for blocking extremities
		}
	}

	// Factor in chain momentum (higher momentum = better penetration)
	float MomentumFactor = FMath::Clamp(
		AttackingChain.ChainMomentum.Size() / 2000.0f,
		0.5f,
		2.0f
	);
	Score *= MomentumFactor;

	// Factor in motion quality (better tracking = more accurate hit)
	Score *= (0.5f + AttackingChain.ChainMotionQuality * 0.5f);

	// Factor in attack confidence
	Score *= (0.3f + AttackingChain.AttackConfidence * 0.7f);

	// Distance penalty (prefer closer targets)
	float DistancePenalty = 1.0f - FMath::Clamp(Candidate.Distance / 200.0f, 0.0f, 0.5f);
	Score *= DistancePenalty;

	// Time penalty (prefer sooner impacts)
	float TimePenalty = 1.0f - FMath::Clamp(Candidate.TimeToImpact / 0.5f, 0.0f, 0.3f);
	Score *= TimePenalty;

	// Alignment bonus (targets in line with attack direction score higher)
	if (!AttackingChain.AttackDirection.IsZero())
	{
		FVector ToTarget = (Candidate.Position - AttackingChain.ChainCenterOfMass).GetSafeNormal();
		float Alignment = FVector::DotProduct(AttackingChain.AttackDirection, ToTarget);
		Score *= (0.7f + Alignment * 0.3f);
	}

	return Score;
}

// Helper to calculate penetration likelihood
float UOHPACManager::CalculatePenetrationScore(
	const FCombatTargetCandidate& ExtremityTarget,
	const FOHCombatChainData& AttackingChain,
	const TArray<TArray<FCombatTargetCandidate>>& TimeGroups,
	int32 CurrentGroupIdx)
{
	float PenetrationScore = 0.0f;

	// Check chain kinetic energy (more energy = more penetration)
	float EnergyScore = FMath::Clamp(AttackingChain.ChainKineticEnergy / 5000.0f, 0.0f, 1.0f);
	PenetrationScore += EnergyScore * 0.3f;

	// Check trajectory continuation past extremity
	if (AttackingChain.PredictedTrajectory.Num() > 0)
	{
		// Find trajectory point at extremity impact time
		int32 ImpactIdx = FMath::Clamp(
			static_cast<int32>(ExtremityTarget.TimeToImpact * AttackingChain.PredictedTrajectory.Num() / 0.3f),
			0,
			AttackingChain.PredictedTrajectory.Num() - 1
		);

		// Check how much trajectory continues past impact
		float RemainingTrajectory = static_cast<float>(AttackingChain.PredictedTrajectory.Num() - ImpactIdx) /
			static_cast<float>(AttackingChain.PredictedTrajectory.Num());
		PenetrationScore += RemainingTrajectory * 0.4f;
	}

	// Check for core bones behind this extremity
	bool bCoreAhead = false;
	for (int32 FutureIdx = CurrentGroupIdx + 1; FutureIdx < TimeGroups.Num(); FutureIdx++)
	{
		for (const FCombatTargetCandidate& Future : TimeGroups[FutureIdx])
		{
			if (Future.bIsCore && Future.TimeToImpact < ExtremityTarget.TimeToImpact + 0.2f)
			{
				bCoreAhead = true;
				break;
			}
		}
		if (bCoreAhead)
		{
			break;
		}
	}

	if (bCoreAhead)
	{
		PenetrationScore += 0.3f;
	}

	return FMath::Clamp(PenetrationScore, 0.0f, 1.0f);
}

// Helper to check if core is in trajectory path
bool UOHPACManager::IsCoreInTrajectory(
	const FCombatTargetCandidate& CoreTarget,
	const FOHCombatChainData& AttackingChain)
{
	if (AttackingChain.PredictedTrajectory.Num() < 2)
	{
		return false;
	}

	// Check if core position is near the predicted trajectory
	float MinDistance = FLT_MAX;

	for (int32 i = 0; i < AttackingChain.PredictedTrajectory.Num() - 1; i++)
	{
		FVector SegmentStart = AttackingChain.PredictedTrajectory[i];
		FVector SegmentEnd = AttackingChain.PredictedTrajectory[i + 1];

		FVector ClosestPoint = FMath::ClosestPointOnSegment(
			CoreTarget.Position,
			SegmentStart,
			SegmentEnd
		);

		float Distance = FVector::Dist(CoreTarget.Position, ClosestPoint);
		MinDistance = FMath::Min(MinDistance, Distance);
	}

	// Core is in trajectory if within strike radius + some margin
	float Threshold = AttackingChain.EffectiveStrikeRadius + EstimateBoneRadius(CoreTarget.BoneName) + 20.0f;
	return MinDistance < Threshold;
}

void UOHPACManager::ApplyCoreImpact(
	UOHPACManager* DefenderManager,
	FName CoreBone,
	const FVector& Direction,
	float Force)
{
	// Core hits get full force plus radial distribution
	FVector CoreImpulse = Direction * Force * ImpactImpulseScale;

	if (FBodyInstance* Body = DefenderManager->GetBodyInstanceDirect(CoreBone))
	{
		// Apply main impact
		Body->AddImpulseAtPosition(CoreImpulse, Body->GetCOMPosition());

		// Radiate to nearby bones
		FVector ImpactPos = Body->GetCOMPosition();

		for (const FName& NearbyBone : DefenderManager->GetSimulatableBones())
		{
			if (NearbyBone == CoreBone)
			{
				continue;
			}

			FVector BonePos = DefenderManager->SkeletalMesh->GetBoneLocation(NearbyBone);
			float Distance = FVector::Dist(ImpactPos, BonePos);

			if (Distance < RadialImpulseRadius)
			{
				float Falloff = 1.0f - (Distance / RadialImpulseRadius);
				FVector RadialDir = (BonePos - ImpactPos).GetSafeNormal();
				FVector RadialImpulse = RadialDir * Force * Falloff * 0.3f;

				if (FBodyInstance* NearbyBody = DefenderManager->GetBodyInstanceDirect(NearbyBone))
				{
					NearbyBody->AddImpulse(RadialImpulse, false);
				}
			}
		}
	}
}


void UOHPACManager::ApplyExtremityImpact(
	UOHPACManager* DefenderManager,
	FName ExtremityBone,
	const FVector& Direction,
	float Force)
{
	// Extremity hits push INWARD toward core
	FVector ExtremityPos = DefenderManager->SkeletalMesh->GetBoneLocation(ExtremityBone);

	// Find nearest core bone
	FName NearestCore = NAME_None;
	float MinDist = FLT_MAX;

	for (const FName& CoreBone : CoreBones)
	{
		if (!DefenderManager->IsBoneSimulating(CoreBone))
		{
			continue;
		}

		FVector CorePos = DefenderManager->SkeletalMesh->GetBoneLocation(CoreBone);
		float Dist = FVector::Dist(ExtremityPos, CorePos);

		if (Dist < MinDist)
		{
			MinDist = Dist;
			NearestCore = CoreBone;
		}
	}

	// Apply force toward core
	FVector ToCore = NearestCore != NAME_None
		                 ? (DefenderManager->SkeletalMesh->GetBoneLocation(NearestCore) - ExtremityPos).GetSafeNormal()
		                 : Direction;

	// Blend original direction with inward push
	FVector FinalDirection = (Direction * 0.6f + ToCore * 0.4f).GetSafeNormal();
	FVector FinalImpulse = FinalDirection * Force * ExtremityPenalty * ImpactImpulseScale;

	if (FBodyInstance* Body = DefenderManager->GetBodyInstanceDirect(ExtremityBone))
	{
		Body->AddImpulseAtPosition(FinalImpulse, ExtremityPos);
	}
}


#pragma region GenericImpulses



void UOHPACManager::ApplyImpulse(FName BoneName, const FVector& Impulse, bool bVelChange)
{
	if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName))
	{
		if (Body->IsInstanceSimulatingPhysics())
		{
			Body->AddImpulse(Impulse, bVelChange);
		}
	}
}

void UOHPACManager::ApplyImpulseToChain(FName RootBone, const FVector& Impulse, float Falloff, bool bVelChange)
{
    // === COMPREHENSIVE VALIDATION USING UTILITY PATTERNS ===
    if (RootBone.IsNone() || !SkeletalMesh)
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("ApplyImpulseToChain: Invalid root bone or skeletal mesh"), true);
        }
        return;
    }
    
    // Validate impulse using sophisticated bounds checking
    if (!Impulse.ContainsNaN() || Impulse.IsNearlyZero())
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ApplyImpulseToChain: Invalid impulse %s"), *Impulse.ToString()), true);
        }
        return;
    }
    
    // === LEVERAGE OHPACMANAGER UTILITY FOR BONE CHAIN GENERATION ===
    TArray<FName> ChainBones = GetBoneChain(RootBone, -1); // Get full chain
    
    if (ChainBones.Num() == 0)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ApplyImpulseToChain: No chain found for root bone '%s'"), 
                *RootBone.ToString()), true);
        }
        return;
    }
    
    // === ENHANCED FALLOFF CALCULATION USING OHCOMBATUTILS ===
    float ValidatedFalloff = FMath::Clamp(Falloff, 0.1f, 1.0f);
    
    // Calculate total chain length for normalized distribution
    float TotalChainLength = 0.0f;
    for (int32 i = 1; i < ChainBones.Num(); i++)
    {
        FVector Pos1 = SkeletalMesh->GetBoneLocation(ChainBones[i-1]);
        FVector Pos2 = SkeletalMesh->GetBoneLocation(ChainBones[i]);
        TotalChainLength += FVector::Dist(Pos1, Pos2);
    }
    
    // === SOPHISTICATED IMPULSE DISTRIBUTION USING MOTION ANALYSIS ===
    int32 BonesAffected = 0;
    float AccumulatedDistance = 0.0f;
    
    for (int32 i = 0; i < ChainBones.Num(); i++)
    {
        const FName& BoneName = ChainBones[i];
        
        // === VALIDATE BONE PHYSICS STATE ===
        if (!IsBoneSimulating(BoneName))
        {
            if (bVerboseLogging)
            {
                UE_LOG(LogPACManager, VeryVerbose, TEXT("Skipping non-simulating bone '%s'"), *BoneName.ToString());
            }
            continue;
        }
        
        FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
        if (!Body || !Body->IsValidBodyInstance())
        {
            continue;
        }
        
        // === CALCULATE DISTANCE-BASED FALLOFF WITH ENHANCED CURVES ===
        float DistanceRatio = (TotalChainLength > KINDA_SMALL_NUMBER) ? 
            (AccumulatedDistance / TotalChainLength) : (static_cast<float>(i) / ChainBones.Num());
        
        // Apply sophisticated falloff using exponential decay
        float FalloffMultiplier = FMath::Pow(ValidatedFalloff, DistanceRatio * 2.0f);
        
        // === BONE-SPECIFIC IMPULSE MODULATION ===
        float BoneImpulseMultiplier = 1.0f;
        
        // Apply bone-specific scaling using robust analysis
        FString BoneString = BoneName.ToString().ToLower();
        if (BoneString.Contains(TEXT("root")) || BoneString.Contains(TEXT("pelvis")))
        {
            BoneImpulseMultiplier = 0.3f; // Reduce root movement for stability
        }
        else if (BoneString.Contains(TEXT("spine")) || BoneString.Contains(TEXT("chest")))
        {
            BoneImpulseMultiplier = 0.7f; // Moderate core movement
        }
        else if (BoneString.Contains(TEXT("hand")) || BoneString.Contains(TEXT("foot")))
        {
            BoneImpulseMultiplier = 1.2f; // Enhanced extremity response
        }
        
        // === MOTION-AWARE IMPULSE SCALING ===
        // Use existing motion data to enhance impulse application
        if (const FOHBoneMotionData* MotionData = GetBoneMotionData(BoneName))
        {
            float CurrentSpeed = MotionData->GetSpeed(EOHReferenceSpace::WorldSpace);
            float MotionQuality = MotionData->GetMotionQuality();
            
            // Reduce impulse for bones already in rapid motion
            if (CurrentSpeed > 300.0f)
            {
                float MotionReduction = FMath::GetMappedRangeValueClamped(
                    FVector2D(300.0f, 1000.0f), 
                    FVector2D(1.0f, 0.5f), 
                    CurrentSpeed
                );
                BoneImpulseMultiplier *= MotionReduction;
            }
            
            // Enhance impulse for bones with good motion quality
            if (MotionQuality > 0.7f)
            {
                BoneImpulseMultiplier *= (1.0f + (MotionQuality - 0.7f) * 0.5f);
            }
        }
        
        // === CALCULATE FINAL IMPULSE WITH COMPREHENSIVE VALIDATION ===
        FVector ScaledImpulse = Impulse * FalloffMultiplier * BoneImpulseMultiplier;
        
        // Validate final impulse before application
        if (!ScaledImpulse.ContainsNaN())
        {
            if (bVerboseLogging)
            {
                SafeLog(FString::Printf(TEXT("Skipping non-finite impulse for bone '%s'"), *BoneName.ToString()), true);
            }
            continue;
        }
        
        // Apply minimum threshold to avoid micro-impulses
        if (ScaledImpulse.Size() < 10.0f)
        {
            continue;
        }
        
        // === LEVERAGE OHCOLLISIONUTILS FOR SAFE IMPULSE APPLICATION ===
        UOHCollisionUtils::ApplyImpulseToBone(SkeletalMesh, BoneName, ScaledImpulse, bVelChange);
        
        BonesAffected++;
        
        // Update accumulated distance for next iteration
        if (i < ChainBones.Num() - 1)
        {
            FVector CurrentPos = SkeletalMesh->GetBoneLocation(BoneName);
            FVector NextPos = SkeletalMesh->GetBoneLocation(ChainBones[i+1]);
            AccumulatedDistance += FVector::Dist(CurrentPos, NextPos);
        }
        
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose,
                TEXT("Chain Impulse Applied: Bone=%s[%d/%d], Falloff=%.2f, BoneMult=%.2f, Impulse=%s"),
                *BoneName.ToString(), i+1, ChainBones.Num(), FalloffMultiplier, BoneImpulseMultiplier, 
                *ScaledImpulse.ToString());
        }
    }
    
    if (bVerboseLogging)
    {
        SafeLog(FString::Printf(
            TEXT("Chain Impulse Complete: Root=%s, TotalBones=%d, Affected=%d, ChainLength=%.1f, Falloff=%.2f"),
            *RootBone.ToString(), ChainBones.Num(), BonesAffected, TotalChainLength, ValidatedFalloff), false);
    }
}

// --- Radial impulse application ---
void UOHPACManager::ApplyRadialImpactImpulse(const FVector& Location, float Magnitude, float Radius)
{
	if (!SkeletalMesh || Radius <= 0.0f)
	{
		return;
	}

	for (const FName& BoneName : SimulatableBones)
	{
		if (!IsBoneSimulating(BoneName))
		{
			continue;
		}

		FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
		if (!Body)
		{
			continue;
		}

		FVector BoneLocation = Body->GetUnrealWorldTransform().GetLocation();
		float Distance = FVector::Dist(Location, BoneLocation);

		if (Distance < Radius)
		{
			float Falloff = 1.0f - FMath::Pow(Distance / Radius, ImpactRadialFalloff);
			FVector Direction = (BoneLocation - Location).GetSafeNormal();
			FVector RadialImpulse = Direction * Magnitude * Falloff;
			Body->AddImpulse(RadialImpulse, false);
		}
	}
}

void UOHPACManager::ApplyRadialImpulseToTarget(
    UOHPACManager* TargetManager,
    const FVector& ImpactPoint,
    float BaseForce)
{
    // === COMPREHENSIVE VALIDATION USING UTILITY PATTERNS ===
    if (!TargetManager || !TargetManager->SkeletalMesh || !IsValid(TargetManager->GetOwner()))
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("ApplyRadialImpulseToTarget: Invalid target manager or components"), true);
        }
        return;
    }
    
    // Validate force magnitude using sophisticated bounds checking
    if (!FMath::IsFinite(BaseForce) || BaseForce < KINDA_SMALL_NUMBER)
    {
        if (bVerboseLogging)
        {
            SafeLog(FString::Printf(TEXT("ApplyRadialImpulseToTarget: Invalid base force %.2f"), BaseForce), true);
        }
        return;
    }
    
    // === LEVERAGE OHCOMBATUTILS FOR SOPHISTICATED TARGET ANALYSIS ===
    AActor* TargetActor = TargetManager->GetOwner();
    AActor* AttackerActor = GetOwner();
    
    // Use OHCombatUtils for mass ratio analysis
    float MassRatio = UOHCombatUtils::CalculateMassRatio(AttackerActor, TargetActor);
    if (!FMath::IsFinite(MassRatio))
    {
        MassRatio = 1.0f; // Defensive fallback
    }
    
    // Apply mass-based force scaling for realistic physics
    float MassAdjustedForce = BaseForce * FMath::Clamp(MassRatio, 0.3f, 3.0f);
    
    // === ENHANCED BONE FILTERING USING OHSKELETALPYSICSUTILS ===
    TArray<FName> ValidTargetBones;
    for (const FName& BoneName : TargetManager->GetSimulatableBones())
    {
        // Use utility functions for comprehensive bone validation
        if (!TargetManager->IsBoneSimulating(BoneName))
        {
            continue;
        }
        
        // Leverage OHSkeletalPhysicsUtils for bone hierarchy analysis
        FBodyInstance* Body = TargetManager->GetBodyInstanceDirect(BoneName);
        if (!Body || !Body->IsValidBodyInstance())
        {
            continue;
        }
        
        ValidTargetBones.Add(BoneName);
    }
    
    if (ValidTargetBones.Num() == 0)
    {
        if (bVerboseLogging)
        {
            SafeLog(TEXT("ApplyRadialImpulseToTarget: No valid simulatable bones found"), true);
        }
        return;
    }
    
    // === SOPHISTICATED SPATIAL ANALYSIS USING OHCOLLISIONUTILS ===
    int32 BonesAffected = 0;
    float MaxEffectiveDistance = RadialImpulseRadius * 1.2f; // Extended range for gradient calculation
    
    for (const FName& BoneName : ValidTargetBones)
    {
        FBodyInstance* Body = TargetManager->GetBodyInstanceDirect(BoneName);
        FVector BoneLocation = Body->GetUnrealWorldTransform().GetLocation();
        
        // === ENHANCED DISTANCE AND DIRECTION CALCULATION ===
        float Distance = FVector::Dist(ImpactPoint, BoneLocation);
        
        // Skip bones outside effective range
        if (Distance > MaxEffectiveDistance)
        {
            continue;
        }
        
        // === LEVERAGE OHCOLLISIONUTILS FOR SOPHISTICATED AABB ANALYSIS ===
        // Get bone bounds for more accurate collision analysis
        FVector BoneBounds = GetBoneBounds(BoneName);
        if (BoneBounds.IsNearlyZero())
        {
            BoneBounds = FVector(10.0f, 10.0f, 10.0f); // Defensive fallback
        }
        
        // Use OHCollisionUtils for closest point calculation
        FVector ClosestPoint = UOHCollisionUtils::ClosestPointOnAABB(
            ImpactPoint, 
            BoneLocation, 
            BoneBounds * 0.5f
        );
        
        float EffectiveDistance = FVector::Dist(ImpactPoint, ClosestPoint);
        
        // Skip if still outside radius after AABB calculation
        if (EffectiveDistance > RadialImpulseRadius)
        {
            continue;
        }
        
        // === ADVANCED FALLOFF CALCULATION USING OHCOMBATUTILS ===
        float DistanceRatio = EffectiveDistance / RadialImpulseRadius;
        
        // Use OHCombatUtils for sophisticated exponential falloff
        float Falloff = 1.0f - FMath::Pow(DistanceRatio, ImpactRadialFalloff);
        Falloff = FMath::Clamp(Falloff, 0.1f, 1.0f);
        
        // === DIRECTIONAL ANALYSIS WITH ENHANCED VALIDATION ===
        FVector RadialDirection = (BoneLocation - ImpactPoint);
        if (RadialDirection.IsNearlyZero())
        {
            // Use OHCombatUtils mirroring for consistent fallback direction
            FVector AttackerForward = AttackerActor ? AttackerActor->GetActorForwardVector() : FVector::ForwardVector;
            RadialDirection = UOHCombatUtils::MirrorVectorOverAxis(AttackerForward, FVector::UpVector);
        }
        else
        {
            RadialDirection.Normalize();
        }
        
        // === BONE-SPECIFIC FORCE MODULATION ===
        float BoneForceMultiplier = 1.0f;
        
        // Apply bone-specific scaling using robust string analysis
        FString BoneString = BoneName.ToString().ToLower();
        if (BoneString.Contains(TEXT("head")) || BoneString.Contains(TEXT("neck")))
        {
            BoneForceMultiplier = 0.6f; // Reduced force for sensitive areas
        }
        else if (BoneString.Contains(TEXT("spine")) || BoneString.Contains(TEXT("chest")))
        {
            BoneForceMultiplier = 1.2f; // Enhanced force for core impact
        }
        else if (BoneString.Contains(TEXT("hand")) || BoneString.Contains(TEXT("foot")))
        {
            BoneForceMultiplier = 0.8f; // Moderate force for extremities
        }
        
        // === FINAL IMPULSE CALCULATION WITH ENHANCED VALIDATION ===
        float FinalForce = MassAdjustedForce * Falloff * BoneForceMultiplier;
        FVector RadialImpulse = RadialDirection * FinalForce;
        
        // Validate impulse before application
        if (!RadialImpulse.ContainsNaN() || RadialImpulse.IsNearlyZero())
        {
            if (bVerboseLogging)
            {
                SafeLog(FString::Printf(TEXT("Skipping invalid impulse for bone '%s'"), *BoneName.ToString()), false);
            }
            continue;
        }
        
        // === LEVERAGE OHCOLLISIONUTILS FOR SAFE IMPULSE APPLICATION ===
        // Apply impulse using validated utility function
        UOHCollisionUtils::ApplyImpulseToBone(
            TargetManager->SkeletalMesh,
            BoneName,
            RadialImpulse,
            false // Use force, not velocity change
        );
        
        BonesAffected++;
        
        if (bVerboseLogging)
        {
            UE_LOG(LogPACManager, VeryVerbose,
                TEXT("Radial Impulse Applied: Bone=%s, Distance=%.1f, Falloff=%.2f, Force=%.1f, Direction=%s"),
                *BoneName.ToString(), EffectiveDistance, Falloff, FinalForce, *RadialDirection.ToString());
        }
    }
    
    if (bVerboseLogging)
    {
        SafeLog(FString::Printf(
            TEXT("Radial Impulse Complete: Point=%s, BaseForce=%.1f, MassRatio=%.2f, BonesAffected=%d/%d"),
            *ImpactPoint.ToString(), BaseForce, MassRatio, BonesAffected, ValidTargetBones.Num()), false);
    }
}
#pragma endregion



