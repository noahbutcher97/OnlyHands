#include "Component/OHPACManager.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "TimerManager.h"
DEFINE_LOG_CATEGORY_STATIC(LogOHPAC, Log, All);

// ============================================================================
// BONE MOTION DATA IMPLEMENTATION
// ============================================================================
#pragma region BONE MOTION DATA IMPLEMENTATION
void FOHBoneMotionData::UpdateKinematics(const FVector& NewPos, const FQuat& NewRot, float DeltaTime, float TimeStamp) {
    if (DeltaTime <= KINDA_SMALL_NUMBER || !NewRot.IsNormalized()) {
        return;
    }

    // Store previous state
    SetPreviousPosition(CurrentPosition);
    SetPreviousRotation(CurrentRotation);
    SetLastDeltaTime(DeltaTime);

    // Update current state
    SetPosition(NewPos);
    SetRotation(NewRot);

    // Calculate velocities
    const FVector NewLinearVel = (NewPos - PreviousPosition) / DeltaTime;
    const FQuat DeltaRot = NewRot * PreviousRotation.Inverse();

    FVector Axis;
    float Angle;
    DeltaRot.ToAxisAndAngle(Axis, Angle);
    const FVector NewAngularVel = (Axis * Angle) / DeltaTime;

    // Calculate accelerations
    const FVector NewLinearAccel = (NewLinearVel - LinearVelocity) / DeltaTime;
    const FVector NewAngularAccel = (NewAngularVel - AngularVelocity) / DeltaTime;

    // Update velocities and accelerations
    SetLinearVelocity(NewLinearVel);
    SetAngularVelocity(NewAngularVel);
    SetLinearAcceleration(NewLinearAccel);
    SetAngularAcceleration(NewAngularAccel);

    // Add motion sample
    AddMotionSample(FTransform(NewRot, NewPos), TimeStamp);
}

void FOHBoneMotionData::AddMotionSample(const FTransform& Transform, float TimeStamp) {
    FOHMotionSample Sample = FOHMotionSample::CreateFromState(Transform, LinearVelocity, AngularVelocity,
                                                              LinearAcceleration, AngularAcceleration, TimeStamp);

    if (Sample.IsValidSample()) {
        Sample.ClampValues();
        MotionHistory.Add(Sample);

        if (MotionHistory.Num() > MaxHistorySamples) {
            MotionHistory.RemoveAt(0);
        }
    }
}

FVector FOHBoneMotionData::GetAverageVelocity(int32 SampleCount) const {
    if (MotionHistory.Num() == 0) {
        return LinearVelocity;
    }

    const int32 Count = FMath::Min(SampleCount, MotionHistory.Num());
    FVector Sum = FVector::ZeroVector;

    for (int32 i = MotionHistory.Num() - Count; i < MotionHistory.Num(); ++i) {
        Sum += MotionHistory[i].GetLinearVelocity();
    }

    return Sum / Count;
}

float FOHBoneMotionData::GetInstabilityScore() const {
    if (MotionHistory.Num() < 3) {
        return 0.f;
    }

    const FVector AvgVel = GetAverageVelocity();
    const float VelocityDeviation = (LinearVelocity - AvgVel).Size();
    const float AccelMagnitude = LinearAcceleration.Size();

    return (VelocityDeviation * 0.1f) + (AccelMagnitude * 0.01f);
}

void FOHBoneMotionData::Reset() {
    CurrentPosition = FVector::ZeroVector;
    PreviousPosition = FVector::ZeroVector;
    CurrentRotation = FQuat::Identity;
    PreviousRotation = FQuat::Identity;
    LinearVelocity = FVector::ZeroVector;
    AngularVelocity = FVector::ZeroVector;
    LinearAcceleration = FVector::ZeroVector;
    AngularAcceleration = FVector::ZeroVector;
    bIsSimulating = false;
    MotionHistory.Reset();
}
#pragma endregion

// ============================================================================
// CONSTRAINT DATA IMPLEMENTATION
// ============================================================================
#pragma region CONSTRAINT DATA IMPLEMENTATION
void FOHConstraintData::UpdateStrain() {
    if (!ConstraintInstance) {
        return;
    }

    // Update strain metrics
    PreviousStrain = CurrentStrain;

    // Calculate current strain from constraint
    const float Swing1 = FMath::Abs(ConstraintInstance->GetCurrentSwing1());
    const float Swing2 = FMath::Abs(ConstraintInstance->GetCurrentSwing2());
    const float Twist = FMath::Abs(ConstraintInstance->GetCurrentTwist());

    const float Swing1Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing1LimitDegrees;
    const float Swing2Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing2LimitDegrees;
    const float TwistLimit = ConstraintInstance->ProfileInstance.TwistLimit.TwistLimitDegrees;

    const float Swing1Ratio = Swing1Limit > 0.f ? Swing1 / Swing1Limit : 0.f;
    const float Swing2Ratio = Swing2Limit > 0.f ? Swing2 / Swing2Limit : 0.f;
    const float TwistRatio = TwistLimit > 0.f ? Twist / TwistLimit : 0.f;

    CurrentStrain = FMath::Max3(Swing1Ratio, Swing2Ratio, TwistRatio);

    // Update jitter metric
    const float StrainDelta = FMath::Abs(CurrentStrain - PreviousStrain);
    JitterMetric = FMath::Lerp(JitterMetric, StrainDelta, 0.1f);
}

float FOHConstraintData::GetSwingStrain() const {
    if (!ConstraintInstance) {
        return 0.f;
    }

    const float Swing1 = FMath::Abs(ConstraintInstance->GetCurrentSwing1());
    const float Swing2 = FMath::Abs(ConstraintInstance->GetCurrentSwing2());
    const float Swing1Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing1LimitDegrees;
    const float Swing2Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing2LimitDegrees;

    return FMath::Max(Swing1Limit > 0.f ? Swing1 / Swing1Limit : 0.f, Swing2Limit > 0.f ? Swing2 / Swing2Limit : 0.f);
}

float FOHConstraintData::GetTwistStrain() const {
    if (!ConstraintInstance) {
        return 0.f;
    }

    const float Twist = FMath::Abs(ConstraintInstance->GetCurrentTwist());
    const float TwistLimit = ConstraintInstance->ProfileInstance.TwistLimit.TwistLimitDegrees;

    return TwistLimit > 0.f ? Twist / TwistLimit : 0.f;
}

bool FOHConstraintData::IsOverstressed(float Threshold) const {
    return CurrentStrain > Threshold;
}
#pragma endregion

// ============================================================================
// MAIN COMPONENT IMPLEMENTATION
// ============================================================================
#pragma region MAIN COMPONENT IMPLEMENTATION
UOHPACManager::UOHPACManager() {
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;

    // Initialize zero profile for clearing PAC
    ZeroProfile.OrientationStrength = 0.f;
    ZeroProfile.PositionStrength = 0.f;
    ZeroProfile.VelocityStrength = 0.f;
    ZeroProfile.AngularVelocityStrength = 0.f;
    ZeroProfile.bIsLocalSimulation = true;
}

void UOHPACManager::BeginPlay() {
    Super::BeginPlay();

    if (!bEnablePACManager) {
        return;
    }

    // Prevent double initialization
    if (bIsInitialized) {
        SafeLog(TEXT("Already initialized, skipping BeginPlay initialization"), true);
        return;
    }

    // Start auto-setup process if enabled
    if (bAutoSetupPhysics) {
        PerformAutoSetup();
    } else {
        // Original delayed initialization
        FTimerHandle InitTimer;
        GetWorld()->GetTimerManager().SetTimer(InitTimer, this, &UOHPACManager::InitializePACManager,
                                               InitializationDelay, false);
    }

    // Setup periodic cleanup timer
    GetWorld()->GetTimerManager().SetTimer(CleanupTimer, this, &UOHPACManager::CleanupStaleBlends, CleanupInterval,
                                           true);
}

void UOHPACManager::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bEnablePACManager || !SkeletalMesh || !bIsInitialized) {
        return;
    }

    // Update motion tracking
    UpdateMotionTracking(DeltaTime);

    // Update constraint states
    UpdateConstraintStates(DeltaTime);

    // Process active blends
    ProcessActiveBlends(DeltaTime);

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    if (bDrawDebug) {
        DrawDebugOverlay();
        // VisualizeActivePhysicalAnimationDrives();
        // CheckSimulatingBonesForPhysicalAnimationDrives();
        // DebugBodyInstanceSimulation();
        // DebugPhysicalAnimationConstraints();
        // DebugBodyPhysicsStates();
    }
#endif
}

void UOHPACManager::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    // Restore original collision settings
    if (bAutoSetupPhysics) {
        RestoreOriginalCollisionSettings();
    }

    // Clear all timers
    GetWorld()->GetTimerManager().ClearTimer(InitRetryHandle);
    GetWorld()->GetTimerManager().ClearTimer(AutoSetupRetryTimer);
    GetWorld()->GetTimerManager().ClearTimer(CleanupTimer);

    // Reset PAC Manager
    ResetPACManager();

    Super::EndPlay(EndPlayReason);
}

#pragma endregion

// ============================================================================
// AUTO SETUP IMPLEMENTATION
// ============================================================================
#pragma region AUTO SETUP
void UOHPACManager::PerformAutoSetup() {
    SafeLog(TEXT("Starting automatic physics setup..."));

    // Find skeletal mesh component
    SkeletalMesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
    if (!SkeletalMesh) {
        SafeLog(TEXT("No SkeletalMeshComponent found on owner"), true);
        return;
    }

    // Setup physics asset
    SetupPhysicsAsset();

    // Configure collision settings
    ConfigureCollisionSettings();

    // Create or find PhysicalAnimationComponent
    PhysicalAnimationComponent = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();
    if (!PhysicalAnimationComponent) {
        PhysicalAnimationComponent = NewObject<UPhysicalAnimationComponent>(
            GetOwner(), UPhysicalAnimationComponent::StaticClass(), TEXT("AutoPAC"));
        PhysicalAnimationComponent->RegisterComponentWithWorld(GetWorld());
        SafeLog(TEXT("Created new PhysicalAnimationComponent"));
    }

    // Bind PAC to skeletal mesh
    PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);

    // Force physics state recreation if requested
    if (bForceRecreateBodies) {
        SkeletalMesh->RecreatePhysicsState();
        BodyInstanceCache.Empty();
        ConstraintInstanceCache.Empty();
        BuildDirectCaches();
        BuildConstraintData();
        ValidatePhysicsSimulation();
        SafeLog(TEXT("Forced physics state recreation"));
    }

    // Start retry timer to ensure physics bodies are ready
    AutoSetupRetryCount = 0;
    GetWorld()->GetTimerManager().SetTimer(AutoSetupRetryTimer, this, &UOHPACManager::RetryAutoSetup, 0.1f, true);
}

void UOHPACManager::RetryAutoSetup() {
    AutoSetupRetryCount++;

    if (AutoSetupRetryCount > MaxAutoSetupRetries) {
        SafeLog(FString::Printf(TEXT("Auto-setup failed after %d retries"), MaxAutoSetupRetries), true);
        GetWorld()->GetTimerManager().ClearTimer(AutoSetupRetryTimer);
        return;
    }

    // Check if physics bodies are ready
    if (!ArePhysicsBodiesReady()) {
        if (bVerboseLogging) {
            SafeLog(FString::Printf(TEXT("Physics bodies not ready, retry %d/%d"), AutoSetupRetryCount,
                                    MaxAutoSetupRetries));
        }
        return;
    }

    // Physics bodies ready, finalize setup
    GetWorld()->GetTimerManager().ClearTimer(AutoSetupRetryTimer);

    // Ensure constraint data is updated
    if (SkeletalMesh->GetPhysicsAsset()) {
        SkeletalMesh->RecreatePhysicsState();
        BodyInstanceCache.Empty();
        ConstraintInstanceCache.Empty();
        BuildDirectCaches();
        BuildConstraintData();
        ValidatePhysicsSimulation();
    }

    // Update overlaps if requested
    if (bAutoUpdateOverlaps) {
        SkeletalMesh->UpdateOverlaps();
        SafeLog(TEXT("Updated skeletal mesh overlaps"));
    }

    // Validate physics state
    ValidatePhysicsSimulation();

    bAutoSetupComplete = true;
    SafeLog(FString::Printf(TEXT("Auto-setup completed successfully after %d retries"), AutoSetupRetryCount));

    // Now initialize the PAC manager
    InitializePACManager();
}

void UOHPACManager::SetupPhysicsAsset() {
    if (!SkeletalMesh) {
        return;
    }

    // Prefer override asset if set
    if (OverridePhysicsAsset && OverridePhysicsAsset->IsValidLowLevel()) {
        CachedPhysicsAsset = OverridePhysicsAsset;
        SkeletalMesh->SetPhysicsAsset(OverridePhysicsAsset, true);
        SafeLog(FString::Printf(TEXT("Applied OverridePhysicsAsset: %s"), *OverridePhysicsAsset->GetName()));
    } else {
        // Use mesh's default physics asset
        CachedPhysicsAsset = SkeletalMesh->GetPhysicsAsset();
        if (!CachedPhysicsAsset) {
            SafeLog(TEXT("No physics asset available"), true);
            return;
        }
    }

    // Force physics asset instance update
    SkeletalMesh->SetSkeletalMeshAsset(SkeletalMesh->GetSkeletalMeshAsset());
}

void UOHPACManager::ConfigureCollisionSettings() {
    if (!SkeletalMesh) {
        return;
    }

    // Store original collision profile
    if (!bHasStoredOriginalProfile) {
        OriginalCollisionProfile = SkeletalMesh->GetCollisionProfileName();
        bHasStoredOriginalProfile = true;
    }

    // Set physics collision profile
    SkeletalMesh->SetCollisionProfileName(PhysicsCollisionProfile);
    SkeletalMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    // Ensure bodies can simulate
    SkeletalMesh->SetAllBodiesSimulatePhysics(false); // Start with all kinematic

    SafeLog(FString::Printf(TEXT("Set collision profile from '%s' to '%s'"), *OriginalCollisionProfile.ToString(),
                            *PhysicsCollisionProfile.ToString()));
}

void UOHPACManager::RestoreOriginalCollisionSettings() {
    if (SkeletalMesh && bHasStoredOriginalProfile) {
        SkeletalMesh->SetCollisionProfileName(OriginalCollisionProfile);
        SafeLog(FString::Printf(TEXT("Restored collision profile to '%s'"), *OriginalCollisionProfile.ToString()));
    }
}

void UOHPACManager::ValidatePhysicsSimulation() {
    if (!SkeletalMesh) {
        return;
    }

    // Ensure we're using a physics-enabled collision profile
    FName CurrentProfile = SkeletalMesh->GetCollisionProfileName();
    if (CurrentProfile != PhysicsCollisionProfile) {
        SafeLog(FString::Printf(TEXT("Collision profile mismatch. Expected '%s', got '%s'"),
                                *PhysicsCollisionProfile.ToString(), *CurrentProfile.ToString()),
                true);

        if (bAutoSetupPhysics) {
            ConfigureCollisionSettings();
        }
    }

    // Ensure physics asset is properly set
    if (!SkeletalMesh->GetPhysicsAsset()) {
        SafeLog(TEXT("No physics asset on skeletal mesh!"), true);
        if (CachedPhysicsAsset) {
            SkeletalMesh->SetPhysicsAsset(CachedPhysicsAsset, true);
        }
    }

    // Ensure PAC binding is valid
    IsSkeletalMeshBindingValid(true, bVerboseLogging);
}

void UOHPACManager::EnsurePhysicsStateValid() {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        return;
    }

    // Ensure PAC is bound to the correct mesh
    if (PhysicalAnimationComponent->GetSkeletalMesh() != SkeletalMesh) {
        PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);
        SafeLog(TEXT("Re-bound PhysicalAnimationComponent to SkeletalMesh"));
    }

    // Validate collision profile
    if (bAutoSetupPhysics) {
        FName CurrentProfile = SkeletalMesh->GetCollisionProfileName();
        if (CurrentProfile != PhysicsCollisionProfile) {
            ConfigureCollisionSettings();
        }
    }
}

#pragma endregion

// ============================================================================
// INITIALIZATION
// ============================================================================
#pragma region INITIALIZATION

void UOHPACManager::OnSkeletalMeshChanged() {
    // Refresh SkeletalMesh pointer (for attached/detached, runtime, or hot reload scenarios)
    USkeletalMeshComponent* NewMesh = GetOwner() ? GetOwner()->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
    SkeletalMesh = NewMesh;

    if (!SkeletalMesh) {
        SafeLog(TEXT("OnSkeletalMeshChanged: SkeletalMeshComponent is null."), true);
        return;
    }

    USkeletalMesh* CurrentMeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
    UPhysicsAsset* CurrentPhysicsAsset = SkeletalMesh->GetPhysicsAsset();

    // --- Detect mesh asset swap ---
    if (PreviousMeshAsset != CurrentMeshAsset) {
        PreviousMeshAsset = CurrentMeshAsset;
        PreviousPhysicsAsset = CurrentPhysicsAsset;
        OnSkeletalAssetChanged(); // Triggers full rebuild
        return;
    }

    // --- Detect physics asset swap ---
    if (PreviousPhysicsAsset != CurrentPhysicsAsset) {
        PreviousPhysicsAsset = CurrentPhysicsAsset;
        OnSkeletalAssetChanged();
        return;
    }

    // --- Lightweight refresh for mesh property changes ---
    BuildDirectCaches();
    BoneChildrenMap.Empty();
    BuildHierarchyMaps();
    BuildConstraintData();
    InitializeMotionTracking();
    DetermineSimulatableBones();

    // --- Validate all references ---
    TArray<FName> MissingBones, InstancesWithoutBodies, MissingConstraints, RuntimeConstraintsNotInAsset,
        MismatchedConstraints;
    if (!ValidateSetup(MissingBones, InstancesWithoutBodies, MissingConstraints, RuntimeConstraintsNotInAsset,
                       MismatchedConstraints)) {
        SafeLog(TEXT("OnSkeletalMeshChanged: Validation failed after property refresh."), true);
    }

    // --- Physics state integration: Ensure all physics properties and bindings are correct ---
    ValidatePhysicsSimulation();
    EnsurePhysicsStateValid();

    SafeLog(TEXT("OnSkeletalMeshChanged: Lightweight property/physics refresh done."), false);
}

void UOHPACManager::OnSkeletalAssetChanged() {
    PreviousMeshAsset = SkeletalMesh ? SkeletalMesh->GetSkeletalMeshAsset() : nullptr;
    PreviousPhysicsAsset = SkeletalMesh ? SkeletalMesh->GetPhysicsAsset() : nullptr;

    SafeLog(TEXT("OnSkeletalAssetChanged: Performing full reinitialization."), false);

    if (SkeletalMesh) {
        SkeletalMesh->RecreatePhysicsState();
        BodyInstanceCache.Empty();
        ConstraintInstanceCache.Empty();
        BuildDirectCaches();
        BuildConstraintData();
        ValidatePhysicsSimulation();
    }

    // Full system re-init (this handles event bindings, cache rebuild, physics setup, etc.)
    InitializePACManager();
}

#if 0

void UOHPACManager::OnSkeletalMeshChanged()
{
	// Called when the SkeletalMeshComponent changes mesh asset at runtime
	OnSkeletalAssetChanged();
}




void UOHPACManager::OnSkeletalAssetChanged()
{
	SafeLog(TEXT("Skeletal asset changed, rebuilding system..."));

	// Stop all active simulations before rebuilding
	ResetPACManager();

	// Small delay before rebuilding to ensure asset is fully loaded
	FTimerHandle RebuildTimer;
	GetWorld()->GetTimerManager().SetTimer(RebuildTimer, [this]()
	{
		// Clear BoneChildrenMap before rebuilding to prevent mismatches
		BoneChildrenMap.Empty();
		BuildHierarchyMaps();

		TArray<FName> OutMissingBones;
		TArray<FName> OutInstancesWithoutBodies;
		TArray<FName> OutMissingConstraints;
		TArray<FName> OutRuntimeConstraintsNotInAsset;
		TArray<FName> OutMismatchedConstraints;

		ValidatePhysicsAsset(
			OutMissingBones,
			OutInstancesWithoutBodies,
			OutMissingConstraints,
			OutRuntimeConstraintsNotInAsset,
			OutMismatchedConstraints
		);

		BuildConstraintData();
		BuildDirectCaches();
		DetermineSimulatableBones();

		// Mark as initialized after rebuild
		bIsInitialized = true;

		SafeLog(TEXT("Asset rebuild complete"));
	}, 0.1f, false);
}
#endif

#if WITH_EDITOR
void UOHPACManager::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) {
    Super::PostEditChangeProperty(PropertyChangedEvent);

    // Defensive: The component may not be fully initialized during some editor operations
    if (!GetOwner()) {
        SafeLog(TEXT("PostEditChangeProperty: No Owner yet. Skipping."), false);
        return;
    }

    // Always update our reference to the skeletal mesh in case it's changed/attached
    USkeletalMeshComponent* NewSkeletalMesh =
        GetOwner() ? GetOwner()->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
    SkeletalMesh = NewSkeletalMesh;
    if (SkeletalMesh != NewSkeletalMesh) {
        SkeletalMesh = NewSkeletalMesh;
        SafeLog(TEXT("SkeletalMeshComponent reference updated after property change."), false);
    }

    // Get the name of the property that changed
    const FName ChangedProp = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

    if (ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, SkeletalMesh) ||
        ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, OverridePhysicsAsset)) {
        // If the mesh or override asset changed, fully reinitialize
        SafeLog(TEXT("Mesh or physics asset changed—reinitializing manager."), false);
        InitializePACManager();
    } else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, PhysicsProfiles) ||
               ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, TrackedBones) ||
               ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, SimulationExclusions)) {
        // If profiles or bone sets changed, update caches and state (but not the full system)
        SafeLog(TEXT("Physics profile, tracked bones, or exclusions changed—rebuilding caches."), false);
        BuildDirectCaches();
        BuildHierarchyMaps();
        BuildConstraintData();
        InitializeMotionTracking();
        DetermineSimulatableBones();
    } else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, PhysicsCollisionProfile) ||
               ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, bAutoSetupPhysics)) {
        // Changes to collision or auto-setup should re-run collision configuration
        SafeLog(TEXT("Physics collision profile or auto-setup toggled—reconfiguring collision."), false);
        ConfigureCollisionSettings();
        ValidatePhysicsSimulation();
    } else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UOHPACManager, bEnablePACManager)) {
        SafeLog(TEXT("PAC manager enable/disable toggled—no action taken (handled at runtime)."), false);
        // Optionally: reset or clear state if disabling
    } else if (ChangedProp != NAME_None) {
        // For any other property, log and consider partial refresh if needed
        SafeLog(FString::Printf(TEXT("Unhandled property changed: %s—no action taken."), *ChangedProp.ToString()),
                false);
    } else {
        // Hot reload, asset reload, or ambiguous property—safe default is full refresh
        SafeLog(TEXT("Unknown or hot-reload change detected—performing safe reinitialization."), false);
        InitializePACManager();
    }
    //    OnSkeletalMeshChanged();
}
#endif

void UOHPACManager::StartInitializationRetry() {
    InitRetryElapsed = 0.0f;
    RetryInitializePACManager();
}

void UOHPACManager::RetryInitializePACManager() {
    // More robust check for physics readiness
    if (!SkeletalMesh || !SkeletalMesh->GetPhysicsAsset()) {
        InitRetryElapsed += InitRetryIntervalSeconds;
        if (InitRetryElapsed < InitRetryTotalDurationSeconds) {
            SafeLog(FString::Printf(TEXT("Physics setup not ready, retrying in %.2fs (elapsed %.2fs)..."),
                                    InitRetryIntervalSeconds, InitRetryElapsed),
                    true);
            GetWorld()->GetTimerManager().SetTimer(InitRetryHandle, this, &UOHPACManager::RetryInitializePACManager,
                                                   InitRetryIntervalSeconds, false);
        } else {
            SafeLog(FString::Printf(TEXT("Initialization failed after %.2fs: Physics setup never ready."),
                                    InitRetryElapsed),
                    true);
        }
        return;
    }

    // Check if bodies are actually initialized
    if (!ArePhysicsBodiesReady()) {
        InitRetryElapsed += InitRetryIntervalSeconds;
        if (InitRetryElapsed < InitRetryTotalDurationSeconds) {
            SafeLog(FString::Printf(TEXT("Bodies not initialized, retrying in %.2fs (elapsed %.2fs)..."),
                                    InitRetryIntervalSeconds, InitRetryElapsed),
                    true);
            GetWorld()->GetTimerManager().SetTimer(InitRetryHandle, this, &UOHPACManager::RetryInitializePACManager,
                                                   InitRetryIntervalSeconds, false);
        } else {
            SafeLog(
                FString::Printf(TEXT("Initialization failed after %.2fs: Bodies never initialized."), InitRetryElapsed),
                true);
        }
        return;
    }

    // Bodies are ready, run initialization
    SafeLog(FString::Printf(TEXT("Physics ready after %.2fs, proceeding with initialization..."), InitRetryElapsed));
    InitializePACManager();
}

void UOHPACManager::InitializePACManager() {
    // Prevent double initialization
    if (bIsInitialized) {
        SafeLog(TEXT("Already initialized, skipping redundant initialization"), true);
        return;
    }

    SafeLog(TEXT("Initializing PAC Manager..."));

    // Find required components
    FindComponents();

    // Prepare validation output arrays
    TArray<FName> MissingBones;
    TArray<FName> InstancesWithoutBodies;
    TArray<FName> MissingConstraints;
    TArray<FName> RuntimeConstraintsNotInAsset;
    TArray<FName> MismatchedConstraints;

    // Run validation
    if (!ValidateSetup(MissingBones, InstancesWithoutBodies, MissingConstraints, RuntimeConstraintsNotInAsset,
                       MismatchedConstraints)) {
        SafeLog(TEXT("Validation failed—initialization aborted."), true);
        return;
    }

    // Build efficient caches
    BuildDirectCaches();

    // Clear BoneChildrenMap before building to ensure fresh data
    BoneChildrenMap.Empty();
    BuildHierarchyMaps();

    BuildConstraintData();

    // Initialize motion tracking
    InitializeMotionTracking();

    // Determine simulatable bones
    DetermineSimulatableBones();
#if WITH_EDITOR
    // Bind to mesh change events
    if (SkeletalMesh && !SkeletalMesh->OnSkeletalMeshPropertyChanged.IsBoundToObject(this)) {
        SkeletalMesh->OnSkeletalMeshPropertyChanged.AddUObject(this, &UOHPACManager::OnSkeletalMeshChanged);
        SafeLog(TEXT("Bound to OnSkeletalMeshPropertyChanged event."));
    }
#endif

    // Mark as initialized
    bIsInitialized = true;

    SafeLog(FString::Printf(TEXT("Initialization complete: %d tracked bones, %d simulatable, %d constraints"),
                            BoneMotionMap.Num(), SimulatableBones.Num(), ConstraintDataMap.Num()));
}

void UOHPACManager::FindComponents() {
    SkeletalMesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
    if (!SkeletalMesh) {
        SafeLog(TEXT("No SkeletalMeshComponent found"), true);
        return;
    }

    // Prefer override asset if valid, fallback to mesh's asset
    if (OverridePhysicsAsset && OverridePhysicsAsset->IsValidLowLevel()) {
        CachedPhysicsAsset = OverridePhysicsAsset;
        SafeLog(FString::Printf(TEXT("Using OverridePhysicsAsset: %s"), *CachedPhysicsAsset->GetName()));
    } else {
        CachedPhysicsAsset = SkeletalMesh->GetPhysicsAsset();
        SafeLog(FString::Printf(TEXT("Using SkeletalMesh's PhysicsAsset: %s"),
                                CachedPhysicsAsset ? *CachedPhysicsAsset->GetName() : TEXT("None")));
    }

    // Find or create physical animation component
    PhysicalAnimationComponent = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();
    if (!PhysicalAnimationComponent) {
        PhysicalAnimationComponent = NewObject<UPhysicalAnimationComponent>(
            GetOwner(), UPhysicalAnimationComponent::StaticClass(), TEXT("AutoPAC"));
        PhysicalAnimationComponent->RegisterComponentWithWorld(GetWorld());
    }
    PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);

    // Always ensure the PAC is bound to our SkeletalMesh
    IsSkeletalMeshBindingValid(true, bVerboseLogging);
}

void UOHPACManager::BuildDirectCaches() {
    BodyInstanceCache.Empty();
    ConstraintInstanceCache.Empty();
    BoneIndexCache.Empty();

    if (!SkeletalMesh) {
        return;
    }

    TArray<FName> AllBoneNames;
    SkeletalMesh->GetBoneNames(AllBoneNames);

    for (const FName& BoneName : AllBoneNames) {
        // Cache bone index
        const int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
        if (BoneIndex != INDEX_NONE) {
            BoneIndexCache.Add(BoneName, BoneIndex);
        }

        // Cache body instance
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
            if (Body->IsValidBodyInstance()) {
                BodyInstanceCache.Add(BoneName, Body);
            }
        }
    }

    SafeLog(FString::Printf(TEXT("Cached %d body instances"), BodyInstanceCache.Num()));
}

void UOHPACManager::BuildHierarchyMaps() {
    BoneParentMap.Empty();

    if (!SkeletalMesh) {
        SafeLog(TEXT("Cannot build hierarchy maps: SkeletalMesh is null!"), true);
        return;
    }

    const USkeletalMesh* MeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!MeshAsset) {
        SafeLog(TEXT("Cannot build hierarchy maps: SkeletalMeshAsset is null!"), true);
        return;
    }

    const FReferenceSkeleton& RefSkel = MeshAsset->GetRefSkeleton();
    const int32 NumBones = RefSkel.GetNum();

    // Build Parent Map only
    for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex) {
        const FName BoneName = RefSkel.GetBoneName(BoneIndex);
        const int32 ParentIndex = RefSkel.GetParentIndex(BoneIndex);

        if (ParentIndex != INDEX_NONE) {
            const FName ParentName = RefSkel.GetBoneName(ParentIndex);
            BoneParentMap.Add(BoneName, ParentName);
        }
    }

    // Build and validate children map
    BuildBoneChildrenMap();

    SafeLog(FString::Printf(TEXT("Built hierarchy maps: %d parent relationships, %d valid children entries"),
                            BoneParentMap.Num(), BoneChildrenMap.Num()));
}

void UOHPACManager::BuildBoneChildrenMap() {
    if (!SkeletalMesh) {
        SafeLog(TEXT("Cannot build BoneChildrenMap: SkeletalMesh is null!"), true);
        return;
    }

    const USkeletalMesh* MeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!MeshAsset) {
        SafeLog(TEXT("Cannot build BoneChildrenMap: SkeletalMeshAsset is null!"), true);
        return;
    }

    const FReferenceSkeleton& RefSkeleton = MeshAsset->GetRefSkeleton();
    const int32 NumBones = RefSkeleton.GetNum();

    // Clear existing map to prevent false mismatches
    BoneChildrenMap.Empty(NumBones);

    // Build fresh map from RefSkeleton
    for (int32 BoneIdx = 0; BoneIdx < NumBones; ++BoneIdx) {
        FName BoneName = RefSkeleton.GetBoneName(BoneIdx);
        TArray<int32> ChildIndices;
        RefSkeleton.GetDirectChildBones(BoneIdx, ChildIndices);

        TArray<FName>& ChildNames = BoneChildrenMap.FindOrAdd(BoneName);
        ChildNames.Reserve(ChildIndices.Num());

        for (int32 ChildIdx : ChildIndices) {
            ChildNames.Add(RefSkeleton.GetBoneName(ChildIdx));
        }
    }

    SafeLog(FString::Printf(TEXT("BoneChildrenMap built with %d entries."), BoneChildrenMap.Num()));
}

void UOHPACManager::BuildConstraintData() {
    ConstraintDataMap.Empty();
    ConstraintInstanceCache.Empty();

    if (!CachedPhysicsAsset) {
        return;
    }

    for (const UPhysicsConstraintTemplate* Template : CachedPhysicsAsset->ConstraintSetup) {
        if (!Template) {
            continue;
        }

        const FConstraintInstance& CI = Template->DefaultInstance;
        const FName ParentBone = CI.ConstraintBone1;
        const FName ChildBone = CI.ConstraintBone2;
        const FName ConstraintName = CI.JointName;

        // Create constraint data
        FOHConstraintData ConstraintData;
        ConstraintData.SetConstraintName(ConstraintName);
        ConstraintData.SetParentBone(ParentBone);
        ConstraintData.SetChildBone(ChildBone);

        // Find runtime constraint instance
        if (FConstraintInstance* RuntimeCI = SkeletalMesh->FindConstraintInstance(ConstraintName)) {
            ConstraintData.SetConstraintInstance(RuntimeCI);
            ConstraintInstanceCache.Add(ChildBone, RuntimeCI);
        }

        ConstraintDataMap.Add(ChildBone, ConstraintData);
    }

    SafeLog(FString::Printf(TEXT("Built constraint data: %d constraints"), ConstraintDataMap.Num()));
}

void UOHPACManager::InitializeMotionTracking() {
    BoneMotionMap.Empty();

    for (const FName& BoneName : TrackedBones) {
        if (!SkeletalMesh->DoesSocketExist(BoneName)) {
            continue;
        }

        FOHBoneMotionData MotionData;
        MotionData.SetBoneName(BoneName);

        // Initialize with current transform
        const FTransform BoneTransform = SkeletalMesh->GetSocketTransform(BoneName);
        MotionData.SetPosition(BoneTransform.GetLocation());
        MotionData.SetRotation(BoneTransform.GetRotation());
        MotionData.SetPreviousPosition(BoneTransform.GetLocation());
        MotionData.SetPreviousRotation(BoneTransform.GetRotation());

        BoneMotionMap.Add(BoneName, MotionData);
    }

    SafeLog(FString::Printf(TEXT("Initialized motion tracking for %d bones"), BoneMotionMap.Num()));
}

void UOHPACManager::DetermineSimulatableBones() {
    SimulatableBones.Empty();

    for (const FName& BoneName : TrackedBones) {
        if (SimulationExclusions.Contains(BoneName)) {
            continue;
        }

        // 1. Pattern filter: skip known problematic or helper bones
        if (!IsBoneNamePatternValid(BoneName)) {
            continue;
        }

        // 2. Valid body instance check
        if (!HasPhysicsBody(BoneName)) {
            continue;
        }

        // 3. Mass check: skip bones with near-zero mass
        if (!IsBoneMassValid(BoneName)) {
            continue;
        }

        // All checks passed, include for simulation!
        SimulatableBones.Add(BoneName);
    }

    SafeLog(FString::Printf(TEXT("Determined %d simulatable bones from %d tracked bones"), SimulatableBones.Num(),
                            TrackedBones.Num()));
}

bool UOHPACManager::ArePhysicsBodiesReady() const {
    if (!SkeletalMesh || !SkeletalMesh->GetPhysicsAsset()) {
        return false;
    }

    // Check if we have any bodies and at least one is valid
    if (SkeletalMesh->Bodies.Num() > 0) {
        for (FBodyInstance* Body : SkeletalMesh->Bodies) {
            if (Body && Body->IsValidBodyInstance() && Body->GetBodySetup()) {
                return true;
            }
        }
    }

    return false;
}
#pragma endregion

// ============================================================================
// MOTION TRACKING
// ============================================================================
#pragma region MOTION TRACKING
void UOHPACManager::UpdateMotionTracking(float DeltaTime) {
    const float TimeStamp = GetWorld()->GetTimeSeconds();

    for (auto& Pair : BoneMotionMap) {
        const FName& BoneName = Pair.Key;
        FOHBoneMotionData& MotionData = Pair.Value;

        if (!SkeletalMesh->DoesSocketExist(BoneName)) {
            continue;
        }

        const FVector Position = SkeletalMesh->GetSocketLocation(BoneName);
        const FQuat Rotation = SkeletalMesh->GetSocketQuaternion(BoneName);

        MotionData.UpdateKinematics(Position, Rotation, DeltaTime, TimeStamp);

        // Update simulation state
        MotionData.SetIsSimulating(IsBoneSimulating(BoneName));
    }
}

void UOHPACManager::UpdateConstraintStates(float DeltaTime) {
    for (auto& Pair : ConstraintDataMap) {
        FOHConstraintData& ConstraintData = Pair.Value;
        ConstraintData.UpdateStrain();
    }
}
#pragma endregion

// ============================================================================
// BLEND PROCESSING
// ============================================================================
#pragma region BLEND PROCESSING

FOHBlendState UOHPACManager::CreateSmartBlendState(FName BoneName, float BlendIn, float Hold, float BlendOut,
                                                   FName ReactionTag) {
    FOHBlendState NewBlend;
    NewBlend.BlendID = ++NextBlendID;
    NewBlend.RootBone = BoneName;
    NewBlend.BlendInDuration = BlendIn;
    NewBlend.HoldDuration = Hold;
    NewBlend.BlendOutDuration = BlendOut;
    NewBlend.ReactionTag = ReactionTag;

    // SMART TRANSITION: Check for existing blend
    const float CurrentAlpha = GetBlendAlpha(BoneName);
    FOHBlendState* ExistingBlend = GetActiveBlendForBone(BoneName);

    if (ExistingBlend != nullptr) {
        // SMOOTH TRANSITION: Start from current alpha
        NewBlend.StartAlpha = CurrentAlpha;
        NewBlend.TargetAlpha = 1.0f;
        NewBlend.bInheritedFromPrevious = true;

        // OPTIMIZATION: If already at high alpha, skip blend-in
        if (CurrentAlpha > 0.8f) {
            NewBlend.Phase = EOHBlendPhase::Hold;
            NewBlend.BlendAlpha = CurrentAlpha;
            NewBlend.ElapsedTime = 0.0f;
        } else {
            NewBlend.Phase = EOHBlendPhase::BlendIn;
            NewBlend.BlendAlpha = CurrentAlpha;
            NewBlend.ElapsedTime = 0.0f;
        }

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("🔄 Smooth transition for %s: %.2f → %.2f (BlendID %d)"),
                   *BoneName.ToString(), CurrentAlpha, NewBlend.TargetAlpha, NewBlend.BlendID);
        }
    } else {
        // NORMAL START: No existing blend, start from 0
        NewBlend.StartAlpha = 0.0f;
        NewBlend.TargetAlpha = 1.0f;
        NewBlend.bInheritedFromPrevious = false;
        NewBlend.Phase = EOHBlendPhase::BlendIn;
        NewBlend.BlendAlpha = 0.0f;
        NewBlend.ElapsedTime = 0.0f;

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("🆕 New blend for %s: 0.0 → 1.0 (BlendID %d)"), *BoneName.ToString(),
                   NewBlend.BlendID);
        }
    }

    return NewBlend;
}

void UOHPACManager::AddBlendToBone(FName BoneName, const FOHBlendState& BlendState) {
    TArray<FOHBlendState>& BlendArray = ActiveBlends.FindOrAdd(BoneName);
    BlendArray.Add(BlendState);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("📝 Added BlendID %d to %s (total blends: %d)"), BlendState.BlendID,
               *BoneName.ToString(), BlendArray.Num());
    }
}

void UOHPACManager::ProcessActiveBlends(float DeltaTime) {
    TArray<TPair<FName, int32>> CompletedBlends;

    for (auto& Pair : ActiveBlends) {
        const FName& BoneName = Pair.Key;
        TArray<FOHBlendState>& BoneBlends = Pair.Value;

        // Process each blend for this bone
        for (int32 i = BoneBlends.Num() - 1; i >= 0; --i) {
            FOHBlendState& Blend = BoneBlends[i];
            if (Blend.IsPaused()) {
                continue;
            }

            UpdateBlendState(Blend, DeltaTime);

            // Mark completed blends (but NOT permanent ones unless forced out)
            if (Blend.IsComplete()) {
                CompletedBlends.Add(TPair<FName, int32>(BoneName, Blend.BlendID));
            }
        }

        // Calculate the EFFECTIVE alpha for this bone from ALL active blends
        float EffectiveAlpha = CalculateEffectiveBlendAlpha(BoneBlends);

        // Apply the effective blend to the bone
        ApplyBlendAlpha(BoneName, EffectiveAlpha);

        if (bVerboseLogging && BoneBlends.Num() > 1) {
            UE_LOG(LogTemp, VeryVerbose, TEXT("[PAC] %s has %d concurrent blends, effective alpha = %.2f"),
                   *BoneName.ToString(), BoneBlends.Num(), EffectiveAlpha);
        }
    }

    // Finalize completed blends
    for (const auto& CompletedPair : CompletedBlends) {
        FinalizeBlendByID(CompletedPair.Key, CompletedPair.Value);
    }

    // Remove empty blend arrays
    RemoveCompletedBlends();
}

void UOHPACManager::CleanupStaleBlends() {
    TArray<FName> StaleBones;
    TArray<FName> PermanentProtectedBones;

    // Find bones with zero ref count but still have blend weight
    for (const auto& Pair : BoneSimulationRefCount) {
        FName BoneName = Pair.Key;
        int32 RefCount = Pair.Value;

        // Check if bone is protected by permanent blends
        if (HasActivePermanentBlend(BoneName)) {
            PermanentProtectedBones.Add(BoneName);
            continue; // Don't clean up permanent blends
        }

        if (RefCount <= 0) {
            if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
                if (Body->PhysicsBlendWeight > 0.01f) {
                    SafeLog(FString::Printf(TEXT("Cleaning stale blend for bone %s (weight=%.2f)"),
                                            *BoneName.ToString(), Body->PhysicsBlendWeight),
                            true);

                    Body->PhysicsBlendWeight = 0.f;
                    Body->SetInstanceSimulatePhysics(false);
                    Body->PutInstanceToSleep();
                    StaleBones.Add(BoneName);
                }
            }
        }
    }

    // Remove stale entries (but not permanent ones)
    for (const FName& BoneName : StaleBones) {
        BoneSimulationRefCount.Remove(BoneName);
        ClearPhysicalAnimationProfile(BoneName);
    }

    // Check for orphaned bodies (simulating but not in ref count AND not permanent)
    if (SkeletalMesh) {
        for (const auto& BodyPair : BodyInstanceCache) {
            FName BoneName = BodyPair.Key;
            FBodyInstance* Body = BodyPair.Value;

            if (Body && Body->IsInstanceSimulatingPhysics() && !BoneSimulationRefCount.Contains(BoneName) &&
                !ActiveBlends.Contains(BoneName)) {
                SafeLog(FString::Printf(TEXT("Found orphaned simulating body: %s"), *BoneName.ToString()), true);

                Body->SetInstanceSimulatePhysics(false);
                Body->PhysicsBlendWeight = 0.f;
                Body->PutInstanceToSleep();
                ClearPhysicalAnimationProfile(BoneName);
            }
        }
    }

    if (bVerboseLogging && (StaleBones.Num() > 0 || PermanentProtectedBones.Num() > 0)) {
        UE_LOG(LogTemp, Warning, TEXT("Cleanup: %d stale, %d permanent-protected"), StaleBones.Num(),
               PermanentProtectedBones.Num());
    }
}

// NEW: Calculate effective alpha from multiple concurrent blends
float UOHPACManager::CalculateEffectiveBlendAlpha(const TArray<FOHBlendState>& BoneBlends) {
    if (BoneBlends.Num() == 0) {
        return 0.0f;
    }

    // Strategy: Use the HIGHEST alpha from all active, non-paused blends
    // But consider different target alphas properly
    float MaxAlpha = 0.0f;
    bool bHasPermanentBlend = false;
    float HighestPermanentAlpha = 0.0f;

    for (const FOHBlendState& Blend : BoneBlends) {
        if (Blend.IsPaused()) {
            continue;
        }

        if (Blend.bIsPermanent && Blend.Phase == EOHBlendPhase::Permanent) {
            bHasPermanentBlend = true;
            HighestPermanentAlpha = FMath::Max(HighestPermanentAlpha, Blend.BlendAlpha);
        } else {
            MaxAlpha = FMath::Max(MaxAlpha, Blend.BlendAlpha);
        }
    }

    // Permanent blends take priority when they reach their target
    if (bHasPermanentBlend) {
        // Check if permanent blend has reached its target (which might not be 1.0)
        bool bPermanentAtTarget = false;
        for (const FOHBlendState& Blend : BoneBlends) {
            if (Blend.bIsPermanent && Blend.Phase == EOHBlendPhase::Permanent) {
                float TargetDifference = FMath::Abs(Blend.BlendAlpha - Blend.TargetAlpha);
                if (TargetDifference < 0.05f) // Close enough to target
                {
                    bPermanentAtTarget = true;
                    break;
                }
            }
        }

        if (bPermanentAtTarget) {
            return HighestPermanentAlpha;
        }
    }

    // Otherwise use the highest alpha from all blends
    return FMath::Max(MaxAlpha, HighestPermanentAlpha);
}

// NEW: Check if bone has permanent blends
bool UOHPACManager::HasActivePermanentBlend(FName BoneName) const {
    const TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends) {
        return false;
    }

    for (const FOHBlendState& Blend : *BoneBlends) {
        if (Blend.bIsPermanent && !Blend.IsPaused() &&
            (Blend.Phase == EOHBlendPhase::Permanent || Blend.Phase == EOHBlendPhase::BlendIn)) {
            return true;
        }
    }

    return false;
}

// Get stronger of two profiles
FPhysicalAnimationData UOHPACManager::GetStrongerProfile(const FPhysicalAnimationData& ProfileA,
                                                         const FPhysicalAnimationData& ProfileB) {
    // Use profile with higher combined strength
    float StrengthA = ProfileA.PositionStrength + ProfileA.OrientationStrength;
    float StrengthB = ProfileB.PositionStrength + ProfileB.OrientationStrength;

    return StrengthA >= StrengthB ? ProfileA : ProfileB;
}

int32 UOHPACManager::GetTotalActiveBlendCount() const {
    int32 TotalBlends = 0;
    for (const auto& Pair : ActiveBlends) {
        TotalBlends += Pair.Value.Num();
    }
    return TotalBlends;
}

void UOHPACManager::RemoveCompletedBlends() {
    TArray<FName> EmptyBlendBones;

    for (auto& Pair : ActiveBlends) {
        if (Pair.Value.Num() == 0) {
            EmptyBlendBones.Add(Pair.Key);
        }
    }

    for (const FName& BoneName : EmptyBlendBones) {
        ActiveBlends.Remove(BoneName);
    }
}

// Update the UpdateBlendState function to handle permanent phase:
void UOHPACManager::UpdateBlendState(FOHBlendState& Blend, float DeltaTime) {
    Blend.ElapsedTime += DeltaTime;

    switch (Blend.Phase) {
    case EOHBlendPhase::BlendIn: {
        if (Blend.BlendInDuration > 0.0f) {
            const float BlendInProgress = FMath::Clamp(Blend.ElapsedTime / Blend.BlendInDuration, 0.0f, 1.0f);
            Blend.BlendAlpha = FMath::Lerp(Blend.StartAlpha, Blend.TargetAlpha, BlendInProgress);

            if (BlendInProgress >= 1.0f) {
                // Transition to permanent or hold phase
                if (Blend.bIsPermanent) {
                    Blend.Phase = EOHBlendPhase::Permanent;
                    Blend.BlendAlpha = 1.0f; // Ensure full alpha
                } else {
                    Blend.Phase = EOHBlendPhase::Hold;
                    Blend.ElapsedTime = 0.0f;
                    Blend.BlendAlpha = Blend.TargetAlpha;
                }
            }
        } else {
            // Instant blend
            Blend.BlendAlpha = Blend.TargetAlpha;
            Blend.Phase = Blend.bIsPermanent ? EOHBlendPhase::Permanent : EOHBlendPhase::Hold;
            Blend.ElapsedTime = 0.0f;
        }
        break;
    }

    case EOHBlendPhase::Permanent: {
        // Stay at full alpha permanently - no automatic transitions
        Blend.BlendAlpha = 1.0f;
        // ElapsedTime continues to accumulate but doesn't trigger transitions
        break;
    }

    case EOHBlendPhase::Hold: {
        Blend.BlendAlpha = Blend.TargetAlpha;

        // Check for infinite hold or normal timed hold
        if (Blend.HoldDuration >= 0.0f && Blend.ElapsedTime >= Blend.HoldDuration) {
            Blend.Phase = EOHBlendPhase::BlendOut;
            Blend.ElapsedTime = 0.0f;
            Blend.StartAlpha = Blend.TargetAlpha;
            Blend.TargetAlpha = 0.0f;
        }
        break;
    }

    case EOHBlendPhase::BlendOut: {
        if (Blend.BlendOutDuration > 0.0f) {
            const float BlendOutProgress = FMath::Clamp(Blend.ElapsedTime / Blend.BlendOutDuration, 0.0f, 1.0f);
            Blend.BlendAlpha = FMath::Lerp(Blend.StartAlpha, 0.0f, BlendOutProgress);
        } else {
            Blend.BlendAlpha = 0.0f;
        }
        break;
    }
    }
}

void UOHPACManager::ApplyBlendAlpha(FName BoneName, float Alpha) {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        const float Clamped = FMath::Clamp(Alpha, 0.f, 1.f);
#if !UE_BUILD_SHIPPING
        if (!FMath::IsNearlyEqual(Alpha, Clamped)) {
            UE_LOG(LogOHPAC, Verbose, TEXT("[PAC] Clamped BlendAlpha for %s: %f -> %f"), *BoneName.ToString(), Alpha,
                   Clamped);
        }
#endif
        Body->PhysicsBlendWeight = Clamped;

        // Ensure the body is awakened when blend weight > 0
        if (Clamped > 0.f && Body->IsInstanceSimulatingPhysics()) {
            Body->WakeInstance();
        }
    }
}

void UOHPACManager::FinalizeBlend(FName BoneName) {
    // Find the most recent blend for this bone and finalize it
    int32 LatestBlendID = FindActiveBlendForBone(BoneName);
    if (LatestBlendID != INDEX_NONE) {
        FinalizeBlendByID(BoneName, LatestBlendID);
    } else {
        SafeLog(FString::Printf(TEXT("FinalizeBlend: No active blend found for bone %s"), *BoneName.ToString()), true);
    }
}

void UOHPACManager::FinalizeAllBlendsForBone(FName BoneName) {
    TArray<FOHBlendState>* BlendArray = ActiveBlends.Find(BoneName);
    if (!BlendArray || BlendArray->Num() == 0) {
        return;
    }

    // Get all blend IDs before removing them
    TArray<int32> BlendIDs;
    for (const FOHBlendState& Blend : *BlendArray) {
        BlendIDs.Add(Blend.BlendID);
    }

    // Remove all blends for this bone
    ActiveBlends.Remove(BoneName);

    // Deactivate physics for the bone chain
    TArray<FName> Chain = GetBoneChain(BoneName, 0);
    for (const FName& ChainBone : Chain) {
        TryDeactivateSimForBone(ChainBone);
    }

    // Broadcast completion
    OnHitReactionComplete.Broadcast(BoneName, Chain);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🧹 Finalized all %d blends for %s"), BlendIDs.Num(), *BoneName.ToString());
    }
}

void UOHPACManager::FinalizeBlendByID(FName BoneName, int32 BlendID) {
    TArray<FOHBlendState>* BlendArray = ActiveBlends.Find(BoneName);
    if (!BlendArray) {
        SafeLog(FString::Printf(TEXT("FinalizeBlendByID: No blends found for bone %s"), *BoneName.ToString()), true);
        return;
    }

    // Find and remove the specific blend
    bool bFoundBlend = false;
    bool bWasPermanent = false;
    for (int32 i = BlendArray->Num() - 1; i >= 0; i--) {
        if ((*BlendArray)[i].BlendID == BlendID) {
            bWasPermanent = (*BlendArray)[i].bIsPermanent;
            BlendArray->RemoveAt(i);
            bFoundBlend = true;
            break;
        }
    }

    if (!bFoundBlend) {
        SafeLog(FString::Printf(TEXT("FinalizeBlendByID: Blend ID %d not found for bone %s"), BlendID,
                                *BoneName.ToString()),
                true);
        return;
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🏁 Finalized blend ID %d for %s (%d blends remaining, was permanent: %s)"),
               BlendID, *BoneName.ToString(), BlendArray->Num(), bWasPermanent ? TEXT("Yes") : TEXT("No"));
    }

    // If no more blends for this bone, handle cleanup
    if (BlendArray->Num() == 0) {
        ActiveBlends.Remove(BoneName);

        // Get affected bones and deactivate using reference counting
        // But only if this wasn't a permanent blend being forcefully removed
        TArray<FName> Chain = GetBoneChain(BoneName, 0);

        for (const FName& ChainBone : Chain) {
            bool bWasLastReference = TryDeactivateSimForBone(ChainBone);

            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("  - Finalized %s (was last ref: %s)"), *ChainBone.ToString(),
                       bWasLastReference ? TEXT("YES") : TEXT("NO"));
            }
        }

        // Broadcast completion event
        OnHitReactionComplete.Broadcast(BoneName, Chain);
    } else {
        // Still have other blends - recalculate effective alpha
        float NewEffectiveAlpha = CalculateEffectiveBlendAlpha(*BlendArray);
        ApplyBlendAlpha(BoneName, NewEffectiveAlpha);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("  - Recalculated alpha for %s: %.2f"), *BoneName.ToString(),
                   NewEffectiveAlpha);
        }
    }
}

#pragma endregion

// ============================================================================
// HIT REACTIONS
// ============================================================================
#pragma region HIT REACTIONS
void UOHPACManager::PlayHitReaction(FName BoneName, EOHPhysicsProfile Profile, float BlendIn, float Hold,
                                    float BlendOut, FVector ImpulseDirection, float ImpulseStrength,
                                    FName ReactionTag) {
    const FPhysicalAnimationData* ProfileData = PhysicsProfiles.Find(Profile);
    if (!ProfileData) {
        SafeLog(FString::Printf(TEXT("Physics profile not found: %d"), static_cast<int32>(Profile)), true);
        return;
    }

    PlayCustomHitReaction(BoneName, *ProfileData, BlendIn, Hold, BlendOut, ImpulseDirection, ImpulseStrength,
                          ReactionTag);
}

void UOHPACManager::PlayCustomHitReaction(FName BoneName, const FPhysicalAnimationData& CustomProfile, float BlendIn,
                                          float Hold, float BlendOut, FVector ImpulseDirection, float ImpulseStrength,
                                          FName ReactionTag) {
    if (!IsBoneValidForSimulation(BoneName)) {
        SafeLog(FString::Printf(TEXT("Bone not valid for simulation: %s"), *BoneName.ToString()), true);
        return;
    }

    if (!IsSkeletalMeshBindingValid(true, bVerboseLogging)) {
        SafeLog(TEXT("Aborting hit reaction: SkeletalMesh binding is invalid!"), true);
        return;
    }

    //  STEP 1: Activate physics for bones first
    TArray<FName> Chain = GetBoneChain(BoneName, 0);
    TArray<FName> SuccessfulBones;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🎯 Starting hit reaction on %s affecting %d bones"), *BoneName.ToString(),
               Chain.Num());
    }

    // Activate physics using reference counting
    for (const FName& ChainBone : Chain) {
        if (TryActivateSimForBone(ChainBone, CustomProfile)) {
            SuccessfulBones.Add(ChainBone);
        } else {
            SafeLog(FString::Printf(TEXT("Failed to activate physics for bone: %s"), *ChainBone.ToString()), true);
        }
    }

    if (SuccessfulBones.Num() == 0) {
        SafeLog(FString::Printf(TEXT("Hit reaction failed - no bones activated for %s"), *BoneName.ToString()), true);
        return;
    }

    //  STEP 2: Create smart blend state AFTER successful activation
    FOHBlendState BlendState = CreateSmartBlendState(BoneName, BlendIn, Hold, BlendOut, ReactionTag);

    // Add blend state to the bone's blend array
    AddBlendToBone(BoneName, BlendState);

    //  STEP 3: Apply impulse with delay for physics setup
    if (!ImpulseDirection.IsNearlyZero() && ImpulseStrength > 0.f) {
        FTimerHandle ImpulseTimer;
        GetWorld()->GetTimerManager().SetTimer(
            ImpulseTimer,
            [this, BoneName, ImpulseDirection, ImpulseStrength]() {
                if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
                    if (Body->IsInstanceSimulatingPhysics()) {
                        const FVector SafeImpulse = ImpulseDirection.GetSafeNormal() * ImpulseStrength;
                        Body->AddImpulse(SafeImpulse, true);
                        if (bVerboseLogging) {
                            UE_LOG(LogTemp, Warning, TEXT("Applied impulse to %s: %s"), *BoneName.ToString(),
                                   *SafeImpulse.ToString());
                        }
                    }
                }
            },
            0.02f, false);
    }

    // Broadcast start event
    OnHitReactionStarted.Broadcast(BoneName, SuccessfulBones, ReactionTag);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Hit reaction started on %s: %d/%d bones activated (BlendID: %d, Smooth: %s)"),
               *BoneName.ToString(), SuccessfulBones.Num(), Chain.Num(), BlendState.BlendID,
               BlendState.bInheritedFromPrevious ? TEXT("YES") : TEXT("NO"));
    }
}
#pragma endregion

// ============================================================================
// BLEND CONTROL
// ============================================================================
#pragma region BLEND CONTROL
void UOHPACManager::PauseBlend(FName BoneName) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (BoneBlends && BoneBlends->Num() > 0) {
        // Pause the most recent blend
        (*BoneBlends)[BoneBlends->Num() - 1].PauseCount++;
    }
}

void UOHPACManager::ResumeBlend(FName BoneName) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (BoneBlends && BoneBlends->Num() > 0) {
        FOHBlendState& LastBlend = (*BoneBlends)[BoneBlends->Num() - 1];
        if (LastBlend.PauseCount > 0) {
            LastBlend.PauseCount--;
        }
    }
}

void UOHPACManager::StopBlend(FName BoneName) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (BoneBlends && BoneBlends->Num() > 0) {
        // Stop the most recent blend
        int32 LastBlendID = (*BoneBlends)[BoneBlends->Num() - 1].BlendID;
        FinalizeBlendByID(BoneName, LastBlendID);
    }
}

void UOHPACManager::PauseAllBlendsForBone(FName BoneName) {
    if (TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName)) {
        for (FOHBlendState& Blend : *BoneBlends) {
            Blend.PauseCount++;
        }
    }
}

void UOHPACManager::ResumeAllBlendsForBone(FName BoneName) {
    if (TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName)) {
        for (FOHBlendState& Blend : *BoneBlends) {
            if (Blend.PauseCount > 0) {
                Blend.PauseCount--;
            }
        }
    }
}

void UOHPACManager::StopAllBlendsForBone(FName BoneName) {
    if (TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName)) {
        TArray<int32> BlendIDs;
        for (const FOHBlendState& Blend : *BoneBlends) {
            BlendIDs.Add(Blend.BlendID);
        }

        for (int32 BlendID : BlendIDs) {
            FinalizeBlendByID(BoneName, BlendID);
        }
    }
}

void UOHPACManager::PauseAllBlends() {
    for (auto& Pair : ActiveBlends) {
        TArray<FOHBlendState>& BoneBlends = Pair.Value;
        for (FOHBlendState& Blend : BoneBlends) {
            Blend.PauseCount++;
        }
    }

    SafeLog(TEXT("Paused all active blends"));
}

void UOHPACManager::ResumeAllBlends() {
    for (auto& Pair : ActiveBlends) {
        TArray<FOHBlendState>& BoneBlends = Pair.Value;
        for (FOHBlendState& Blend : BoneBlends) {
            if (Blend.PauseCount > 0) {
                Blend.PauseCount--;
            }
        }
    }

    SafeLog(TEXT("Resumed all active blends"));
}

void UOHPACManager::StopAllBlends() {
    TArray<FName> BlendBones;
    ActiveBlends.GetKeys(BlendBones);

    for (const FName& BoneName : BlendBones) {
        StopAllBlendsForBone(BoneName);
    }
}
#pragma endregion

// ============================================================================
// IMPULSE SYSTEM
// ============================================================================
#pragma region IMPULSE SYSTEM
void UOHPACManager::ApplyImpulse(FName BoneName, const FVector& Direction, float Magnitude) {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        if (Body->IsInstanceSimulatingPhysics()) {
            const FVector Impulse = Direction.GetSafeNormal() * Magnitude;
            Body->AddImpulse(Impulse, true);

            // Ensure body is awake after impulse
            Body->WakeInstance();
        }
    }
}

void UOHPACManager::ApplyImpulseToChain(FName RootBone, const FVector& Direction, float Magnitude, int32 Depth) {
    const TArray<FName> Chain = GetBoneChain(RootBone, Depth);

    float CurrentMagnitude = Magnitude;

    for (const FName& BoneName : Chain) {
        constexpr float Attenuation = 0.7f;
        ApplyImpulse(BoneName, Direction, CurrentMagnitude);
        CurrentMagnitude *= Attenuation;
    }
}
#pragma endregion

// ============================================================================
// SIMULATION CONTROL
// ============================================================================
#pragma region SIMULATION CONTROL
bool UOHPACManager::StartSimulation(FName BoneName, const FPhysicalAnimationData& Profile, bool bAllBelow,
                                    bool bEnableCollision) {
    if (!IsBoneValidForSimulation(BoneName)) {
        return false;
    }
    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        return false;
    }

    // Ensure physics state is valid
    ValidatePhysicsSimulation();

    // Ensure the skeletal mesh has physics properly initialized
    if (!SkeletalMesh->Bodies.Num()) {
        SafeLog(TEXT("No physics bodies found on skeletal mesh"), true);
        return false;
    }

    if (bAllBelow) {
        // Native Unreal: handles all bodies/constraints below BoneName
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(BoneName, true, false);

        // Explicitly wake all bodies after enabling simulation
        TArray<FName> BonesToSet = GetBoneChain(BoneName, 0);
        for (const FName& Bone : BonesToSet) {
            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                if (bEnableCollision) {
                    Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
                }
                Body->WakeInstance();
            }
        }

        PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(BoneName, Profile, true);
        return true;
    }
    // Single bone only
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        Body->SetInstanceSimulatePhysics(true);
        if (bEnableCollision) {
            Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        }
        PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);
        Body->WakeInstance();
        return true;
    }
    return false;
}

void UOHPACManager::StopSimulation(FName BoneName, bool bAllBelow) {
    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        return;
    }

    if (bAllBelow) {
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(BoneName, false, false);

        // Clear profiles for all below
        TArray<FName> BonesToClear = GetBoneChain(BoneName, 0);
        for (const FName& Bone : BonesToClear) {
            PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, ZeroProfile);
            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                Body->SetLinearVelocity(FVector::ZeroVector, false);
                Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
                Body->PutInstanceToSleep();
            }
            OnBoneStoppedSimulating.Broadcast(Bone);
        }
    } else {
        // Single bone
        ClearPhysicalAnimationProfile(BoneName);
        if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
            Body->SetInstanceSimulatePhysics(false);
            Body->SetLinearVelocity(FVector::ZeroVector, false);
            Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
            Body->PutInstanceToSleep();
        }
        OnBoneStoppedSimulating.Broadcast(BoneName);
    }
}

void UOHPACManager::ApplyPhysicalAnimationProfile(FName BoneName, const FPhysicalAnimationData& Profile) {
    if (PhysicalAnimationComponent) {
        PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);

        // Wake the body after applying settings
        WakePhysicsBody(BoneName);
    }
}

void UOHPACManager::ClearPhysicalAnimationProfile(FName BoneName) {
    if (PhysicalAnimationComponent) {
        PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);
    }
}

bool UOHPACManager::TryActivateSimForBone(FName BoneName, const FPhysicalAnimationData& Profile) {
    if (!SkeletalMesh || !IsBoneValidForSimulation(BoneName)) {
        if (bVerboseLogging) {
            SafeLog(FString::Printf(TEXT("TryActivate failed: Bone %s not valid"), *BoneName.ToString()), true);
        }
        return false;
    }

    int32& RefCount = BoneSimulationRefCount.FindOrAdd(BoneName);
    RefCount = FMath::Clamp(RefCount + 1, 1, 999);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🔢 TryActivate %s: RefCount now %d"), *BoneName.ToString(), RefCount);
    }

    if (RefCount == 1) {
        // First activation - actually start physics with PAC
        bool bSuccess = StartBonePhysicalAnimation(BoneName, Profile, true, true, bVerboseLogging);
        if (bSuccess) {
            OnBoneStartedSimulating.Broadcast(BoneName);
            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("✅ First activation successful for %s"), *BoneName.ToString());
            }
        } else {
            // Rollback ref count on failure
            RefCount--;
            if (RefCount <= 0) {
                BoneSimulationRefCount.Remove(BoneName);
            }
            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("❌ First activation failed for %s"), *BoneName.ToString());
            }
        }
        return bSuccess;
    } else {
        // Already active - update the PAC profile and ensure it's the strongest one
        if (PhysicalAnimationComponent) {
            // Get current profile and compare strengths
            FPhysicalAnimationData CurrentProfile = GetCurrentPhysicalAnimationProfile(BoneName);
            FPhysicalAnimationData StrongerProfile = GetStrongerProfile(CurrentProfile, Profile);

            PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, StrongerProfile);

            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("🔄 Updated PAC profile for %s (RefCount: %d, Pos: %.1f→%.1f)"),
                       *BoneName.ToString(), RefCount, CurrentProfile.PositionStrength,
                       StrongerProfile.PositionStrength);
            }
            return true;
        }
        if (bVerboseLogging) {
            SafeLog(TEXT("Missing PhysicalAnimationComponent during profile update"), true);
        }
        return false;
    }
}

bool UOHPACManager::TryDeactivateSimForBone(FName BoneName) {
    int32* RefCountPtr = BoneSimulationRefCount.Find(BoneName);
    if (!RefCountPtr) {
        if (bVerboseLogging) {
            SafeLog(FString::Printf(TEXT("TryDeactivate: Bone %s not in ref count map"), *BoneName.ToString()));
        }
        return false;
    }

    // Check if bone has permanent blends - if so, don't deactivate unless explicitly forced
    if (HasActivePermanentBlend(BoneName)) {
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("🔒 %s has permanent blend, skipping deactivation"), *BoneName.ToString());
        }
        return false; // Don't deactivate, permanent blend is protecting it
    }

    (*RefCountPtr)--;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🔢 TryDeactivate %s: RefCount now %d"), *BoneName.ToString(), *RefCountPtr);
    }

    if (*RefCountPtr <= 0) {
        BoneSimulationRefCount.Remove(BoneName);
        StopBonePhysicalAnimation(BoneName, true, true, bVerboseLogging);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("✅ Final deactivation for %s"), *BoneName.ToString());
        }
        return true;
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🔄 %s still has %d references, keeping active"), *BoneName.ToString(),
               *RefCountPtr);
    }
    return false;
}

bool UOHPACManager::ActivatePhysicsStateForBone(FName BoneName, float BlendAlpha) {
    if (!SkeletalMesh) {
        return false;
    }

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        SafeLog(FString::Printf(TEXT("Bone '%s' has invalid body instance"), *BoneName.ToString()), true);
        return false;
    }

    // Clear any residual forces
    Body->SetLinearVelocity(FVector::ZeroVector, false);
    Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);

    // Enable simulation
    Body->SetInstanceSimulatePhysics(true);
    Body->PhysicsBlendWeight = BlendAlpha;
    Body->WakeInstance();

    return true;
}

void UOHPACManager::ClearPhysicsStateForBone(FName BoneName) {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        SafeLog(TEXT("ClearPhysicsStateForBone: Missing skeletal mesh or PAC"), true);
        return;
    }

    // Ensure bone exists in skeleton
    if (SkeletalMesh->GetBoneIndex(BoneName) == INDEX_NONE) {
        SafeLog(FString::Printf(TEXT("Bone '%s' not found in skeleton"), *BoneName.ToString()), true);
        return;
    }

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        SafeLog(FString::Printf(TEXT("Bone '%s' has invalid or missing body instance"), *BoneName.ToString()), true);
        return;
    }

    if (Body->GetBodyMass() <= KINDA_SMALL_NUMBER) {
        SafeLog(FString::Printf(TEXT("Bone '%s' has near-zero mass; skipping clear"), *BoneName.ToString()), true);
        return;
    }

    // Disable physics sim
    if (Body->IsInstanceSimulatingPhysics()) {
        Body->SetInstanceSimulatePhysics(false);
    }

    // Reset velocity, forces, and wake state
    Body->SetLinearVelocity(FVector::ZeroVector, false);
    Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
    Body->PhysicsBlendWeight = 0.f;
    Body->PutInstanceToSleep();

    // Clear PAC
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);

#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
    if (Body->PhysicsBlendWeight > 0.01f) {
        SafeLog(FString::Printf(TEXT("Bone '%s' still has non-zero blend weight after clear"), *BoneName.ToString()),
                true);
    }
#endif
}

void UOHPACManager::WakePhysicsBody(FName BoneName) {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        if (Body->IsInstanceSimulatingPhysics()) {
            Body->WakeInstance();
        }
    }
}

// ============================================================================
// SIMULATION CONTROL
// ============================================================================
#pragma region SIMULATION CONTROL
// --- SINGLE BONE ---

bool UOHPACManager::StartBonePhysicalAnimation(FName BoneName, const FPhysicalAnimationData& Profile,
                                               bool bEnableCollision, bool bWakeBody, bool bVerboseLog) {
    if (!IsBoneValidForSimulation(BoneName)) {
        if (bVerboseLog) {
            SafeLog(FString::Printf(TEXT("Bone %s not valid for simulation"), *BoneName.ToString()), true);
        }
        return false;
    }

    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        if (bVerboseLog) {
            SafeLog(TEXT("Missing PhysicalAnimationComponent or SkeletalMesh"), true);
        }
        return false;
    }

    FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        if (bVerboseLog) {
            SafeLog(FString::Printf(TEXT("Invalid body instance for bone %s"), *BoneName.ToString()), true);
        }
        return false;
    }

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("🎯 Starting PAC for %s: Ori=%.1f, Pos=%.1f, Vel=%.1f"), *BoneName.ToString(),
               Profile.OrientationStrength, Profile.PositionStrength, Profile.VelocityStrength);
    }

    // ✅ STEP 1: Clear any existing forces (clean slate)
    Body->SetLinearVelocity(FVector::ZeroVector, false);
    Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);

    // ✅ STEP 2: Apply PAC profile FIRST (while the body is kinematic)
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);

    // ✅ STEP 3: Configure collision BEFORE enabling simulation
    if (bEnableCollision) {
        Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    }

    // ✅ STEP 4: Enable physics simulation (body will be constrained from the start)
    Body->SetInstanceSimulatePhysics(true);
    Body->PhysicsBlendWeight = 1.0f; // Full physics simulation with PAC constraints

    // ✅ STEP 5: Wake the body LAST (start simulation with everything configured)
    if (bWakeBody) {
        Body->WakeInstance();
    }

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("✅ PAC activation complete for %s"), *BoneName.ToString());
    }

    return true;
}

void UOHPACManager::StopBonePhysicalAnimation(FName BoneName, bool bClearVelocities = true, bool bPutToSleep = true,
                                              bool bVerboseLog = false) {
    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        if (bVerboseLog) {
            SafeLog(TEXT("Missing PhysicalAnimationComponent or SkeletalMesh"), true);
        }
        return;
    }

    FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        if (bVerboseLog) {
            SafeLog(FString::Printf(TEXT("Invalid body instance for bone %s"), *BoneName.ToString()), true);
        }
        return;
    }

    // Only proceed if actually simulating
    if (!Body->IsInstanceSimulatingPhysics()) {
        if (bVerboseLog) {
            SafeLog(FString::Printf(TEXT("Bone %s not simulating, skipping stop"), *BoneName.ToString()));
        }
        return;
    }

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("🛑 Stopping PAC for %s"), *BoneName.ToString());
    }

    // ✅ STEP 1: Disable physics simulation first (stop movement)
    Body->SetInstanceSimulatePhysics(false);

    // ✅ STEP 2: Clear PAC profile (remove constraints)
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);

    // ✅ STEP 3: Clear residual forces/velocities
    if (bClearVelocities) {
        Body->SetLinearVelocity(FVector::ZeroVector, false);
        Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
    }

    // ✅ STEP 4: Put body to sleep
    if (bPutToSleep) {
        Body->PutInstanceToSleep();
    }

    // ✅ STEP 5: Broadcast stop event
    OnBoneStoppedSimulating.Broadcast(BoneName);

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("✅ PAC stop complete for %s"), *BoneName.ToString());
    }
}

// --- CHAIN ---

bool UOHPACManager::StartChainPhysicalAnimation(FName RootBone, const FPhysicalAnimationData& Profile,
                                                bool bUseNativePropagation, bool bEnableCollision, bool bWakeBody,
                                                bool bVerboseLog) {
    return StartChainPhysicalAnimation_Filtered(
        RootBone, Profile, bUseNativePropagation, bEnableCollision, [](FName) { return true; }, bWakeBody, bVerboseLog);
}

bool UOHPACManager::StartChainPhysicalAnimation_Filtered(
    FName RootBone, const FPhysicalAnimationData& Profile, bool bUseNativePropagation = true,
    bool bEnableCollision = true, TFunctionRef<bool(FName)> BoneFilter = [](FName) { return true; },
    bool bWakeBody = true, bool bVerboseLog = false) {
    if (!IsBoneValidForSimulation(RootBone)) {
        if (bVerboseLog) {
            SafeLog(FString::Printf(TEXT("Root bone %s not valid for simulation"), *RootBone.ToString()), true);
        }
        return false;
    }

    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        if (bVerboseLog) {
            SafeLog(TEXT("Missing PhysicalAnimationComponent or SkeletalMesh"), true);
        }
        return false;
    }

    bool bAnySuccess = false;
    TArray<FName> Bones = GetBoneChain(RootBone, 0);

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("🔗 Starting chain PAC from %s (%s propagation, %d bones)"), *RootBone.ToString(),
               bUseNativePropagation ? TEXT("NATIVE") : TEXT("CUSTOM"), Bones.Num());
    }

    if (bUseNativePropagation) {
        // ✅ FIXED ORDER: Apply PAC constraints FIRST (before any physics activation)
        PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(RootBone, Profile, true);

        if (bVerboseLog) {
            UE_LOG(LogTemp, Warning, TEXT("🎯 Applied native PAC profile: Ori=%.1f, Pos=%.1f, Vel=%.1f"),
                   Profile.OrientationStrength, Profile.PositionStrength, Profile.VelocityStrength);
        }

        // ✅ FIXED ORDER: Enable physics simulation AFTER PAC is applied
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(RootBone, true, false);

        // ✅ Configure collision and finalize setup
        int32 ProcessedCount = 0;
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone)) {
                continue;
            }

            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                if (bEnableCollision) {
                    Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
                }

                Body->PhysicsBlendWeight = 1.0f; // Full physics with PAC constraints

                if (bWakeBody) {
                    Body->WakeInstance();
                }

                ProcessedCount++;
            }
        }

        if (bVerboseLog) {
            UE_LOG(LogTemp, Warning, TEXT("✅ Native chain PAC complete: %d bones processed"), ProcessedCount);
        }

        bAnySuccess = ProcessedCount > 0;
    } else {
        // Custom path - use per-bone activation with the correct order
        if (bVerboseLog) {
            UE_LOG(LogTemp, Warning, TEXT("🔧 Using custom per-bone PAC application"));
        }

        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone)) {
                continue;
            }
            bAnySuccess |= StartBonePhysicalAnimation(Bone, Profile, bEnableCollision, bWakeBody, bVerboseLog);
        }
    }

    return bAnySuccess;
}

void UOHPACManager::StopChainPhysicalAnimation(FName RootBone, bool bUseNativePropagation) {
    StopChainPhysicalAnimation_Filtered(RootBone, bUseNativePropagation, [](FName) { return true; }, true, true, false);
}

void UOHPACManager::StopChainPhysicalAnimation_Filtered(
    FName RootBone, bool bUseNativePropagation = true,
    TFunctionRef<bool(FName)> BoneFilter = [](FName) { return true; }, bool bClearVelocities = true,
    bool bPutToSleep = true, bool bVerboseLog = false) {
    if (!PhysicalAnimationComponent || !SkeletalMesh) {
        if (bVerboseLog) {
            SafeLog(TEXT("Missing PhysicalAnimationComponent or SkeletalMesh"), true);
        }
        return;
    }

    TArray<FName> Bones = GetBoneChain(RootBone, 0);

    if (bVerboseLog) {
        UE_LOG(LogTemp, Warning, TEXT("🛑 Stopping chain PAC from %s (%s propagation, %d bones)"), *RootBone.ToString(),
               bUseNativePropagation ? TEXT("NATIVE") : TEXT("CUSTOM"), Bones.Num());
    }

    if (bUseNativePropagation) {
        // ✅ STEP 1: Bulk disable physics simulation first
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(RootBone, false, false);

        // ✅ STEP 2: Individual cleanup for each bone
        int32 StoppedCount = 0;
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone)) {
                continue;
            }

            // Clear PAC profile
            PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, ZeroProfile);

            // Clean up body state
            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                if (bClearVelocities) {
                    Body->SetLinearVelocity(FVector::ZeroVector, false);
                    Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
                }

                if (bPutToSleep) {
                    Body->PutInstanceToSleep();
                }
            }

            // Update reference count if using it
            if (int32* RefCountPtr = BoneSimulationRefCount.Find(Bone)) {
                (*RefCountPtr)--;
                if (*RefCountPtr <= 0) {
                    BoneSimulationRefCount.Remove(Bone);
                }
            }

            // Broadcast stop event
            OnBoneStoppedSimulating.Broadcast(Bone);
            StoppedCount++;
        }

        if (bVerboseLog) {
            UE_LOG(LogTemp, Warning, TEXT("✅ Native bulk stop complete: %d bones"), StoppedCount);
        }
    } else {
        // Custom path - delegate to single-bone function
        int32 StoppedCount = 0;
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone)) {
                continue;
            }

            StopBonePhysicalAnimation(Bone, bClearVelocities, bPutToSleep, bVerboseLog);

            // Handle ref counting for chain operations
            if (int32* RefCountPtr = BoneSimulationRefCount.Find(Bone)) {
                (*RefCountPtr)--;
                if (*RefCountPtr <= 0) {
                    BoneSimulationRefCount.Remove(Bone);
                }
            }

            StoppedCount++;
        }

        if (bVerboseLog) {
            UE_LOG(LogTemp, Warning, TEXT("✅ Custom per-bone stop complete: %d bones"), StoppedCount);
        }
    }
}

bool UOHPACManager::StartPhysicsBlend(FName BoneName, const FPhysicalAnimationData& Profile, float BlendInDuration,
                                      float HoldDuration, float BlendOutDuration, FName BlendTag) {
    if (!IsBoneValidForSimulation(BoneName)) {
        SafeLog(FString::Printf(TEXT("Bone not valid for blending: %s"), *BoneName.ToString()), true);
        return false;
    }

    if (!IsSkeletalMeshBindingValid(true, bVerboseLogging)) {
        SafeLog(TEXT("Aborting physics blend: SkeletalMesh binding is invalid!"), true);
        return false;
    }

    // STEP 1: Activate physics for the bone using reference counting
    if (!TryActivateSimForBone(BoneName, Profile)) {
        SafeLog(FString::Printf(TEXT("Failed to activate physics for bone: %s"), *BoneName.ToString()), true);
        return false;
    }

    // STEP 2: Create smart blend state for smooth transitions
    FOHBlendState BlendState =
        CreateSmartBlendState(BoneName, BlendInDuration, HoldDuration, BlendOutDuration, BlendTag);

    // STEP 3: Add blend state to the bone's blend array
    AddBlendToBone(BoneName, BlendState);

    // STEP 4: Ensure bone starts in correct pose (no impulse, just pose retention)
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        // Clear any existing velocities for clean start
        Body->SetLinearVelocity(FVector::ZeroVector, false);
        Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);

        // Ensure body is awake for physics simulation
        Body->WakeInstance();
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Started physics blend for %s (BlendID: %d, Tag: %s)"), *BoneName.ToString(),
               BlendState.BlendID, *BlendTag.ToString());
    }

    return true;
}

bool UOHPACManager::StartPhysicsBlendChain(FName RootBoneName, const FPhysicalAnimationData& Profile,
                                           float BlendInDuration, float HoldDuration, float BlendOutDuration,
                                           FName BlendTag) {
    TArray<FName> Chain = GetBoneChain(RootBoneName, 0);
    TArray<FName> SuccessfulBones;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🔗 Starting physics blend chain from %s affecting %d bones"),
               *RootBoneName.ToString(), Chain.Num());
    }

    // Apply blending to each bone in the chain
    for (const FName& ChainBone : Chain) {
        if (StartPhysicsBlend(ChainBone, Profile, BlendInDuration, HoldDuration, BlendOutDuration, BlendTag)) {
            SuccessfulBones.Add(ChainBone);
        } else {
            SafeLog(FString::Printf(TEXT("Failed to start blend for chain bone: %s"), *ChainBone.ToString()), true);
        }
    }

    if (SuccessfulBones.Num() == 0) {
        SafeLog(
            FString::Printf(TEXT("Physics blend chain failed - no bones activated for %s"), *RootBoneName.ToString()),
            true);
        return false;
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Physics blend chain started: %d/%d bones activated"), SuccessfulBones.Num(),
               Chain.Num());
    }

    return true;
}

bool UOHPACManager::StartPhysicsBlendWithAlpha(FName BoneName, const FPhysicalAnimationData& Profile, float TargetAlpha,
                                               float BlendInDuration, float HoldDuration, float BlendOutDuration,
                                               FName BlendTag) {
    if (!IsBoneValidForSimulation(BoneName)) {
        SafeLog(FString::Printf(TEXT("Bone not valid for alpha blending: %s"), *BoneName.ToString()), true);
        return false;
    }

    if (!IsSkeletalMeshBindingValid(true, bVerboseLogging)) {
        SafeLog(TEXT("Aborting alpha blend: SkeletalMesh binding is invalid!"), true);
        return false;
    }

    // Clamp and apply bone-specific scaling to target alpha
    float ScaledTargetAlpha = ApplyBoneAlphaScaling(BoneName, FMath::Clamp(TargetAlpha, 0.0f, 1.0f));

    // Activate physics for the bone
    if (!TryActivateSimForBone(BoneName, Profile)) {
        SafeLog(FString::Printf(TEXT("Failed to activate physics for alpha blend: %s"), *BoneName.ToString()), true);
        return false;
    }

    // Create blend state with custom target alpha
    FOHBlendState BlendState = CreateSmartBlendStateWithAlpha(BoneName, ScaledTargetAlpha, BlendInDuration,
                                                              HoldDuration, BlendOutDuration, BlendTag);

    // Add blend state
    AddBlendToBone(BoneName, BlendState);

    // Clean initial state
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        Body->SetLinearVelocity(FVector::ZeroVector, false);
        Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        Body->WakeInstance();
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Started alpha blend for %s: Target=%.2f (Scaled=%.2f), Tag=%s"),
               *BoneName.ToString(), TargetAlpha, ScaledTargetAlpha, *BlendTag.ToString());
    }

    return true;
}

bool UOHPACManager::StartPhysicsBlendWithMode(FName BoneName, const FPhysicalAnimationData& Profile,
                                              EOHTargetAlphaMode AlphaMode, float BlendInDuration, float HoldDuration,
                                              float BlendOutDuration, FName BlendTag) {
    float TargetAlpha = GetTargetAlphaForMode(AlphaMode);
    return StartPhysicsBlendWithAlpha(BoneName, Profile, TargetAlpha, BlendInDuration, HoldDuration, BlendOutDuration,
                                      BlendTag);
}

bool UOHPACManager::StartPermanentPhysicsBlendWithAlpha(FName BoneName, const FPhysicalAnimationData& Profile,
                                                        float TargetAlpha, float BlendInDuration, FName BlendTag) {
    if (!IsBoneValidForSimulation(BoneName)) {
        return false;
    }

    // Clamp and apply bone-specific scaling
    float ScaledTargetAlpha = ApplyBoneAlphaScaling(BoneName, FMath::Clamp(TargetAlpha, 0.0f, 1.0f));

    // Activate physics
    if (!TryActivateSimForBone(BoneName, Profile)) {
        return false;
    }

    // Create permanent blend with custom alpha
    FOHBlendState BlendState;
    BlendState.BlendID = ++NextBlendID;
    BlendState.RootBone = BoneName;
    BlendState.BlendInDuration = BlendInDuration;
    BlendState.HoldDuration = 0.0f;
    BlendState.BlendOutDuration = 0.0f;
    BlendState.ReactionTag = BlendTag;
    BlendState.bIsPermanent = true;
    BlendState.CustomTargetAlpha = ScaledTargetAlpha;
    BlendState.TargetAlphaMode = EOHTargetAlphaMode::Custom;

    // Handle existing blends
    const float CurrentAlpha = GetBlendAlpha(BoneName);
    FOHBlendState* ExistingBlend = GetActiveBlendForBone(BoneName);

    if (ExistingBlend != nullptr) {
        BlendState.StartAlpha = CurrentAlpha;
        BlendState.TargetAlpha = ScaledTargetAlpha;
        BlendState.bInheritedFromPrevious = true;
        BlendState.Phase =
            (CurrentAlpha >= ScaledTargetAlpha - 0.1f) ? EOHBlendPhase::Permanent : EOHBlendPhase::BlendIn;
        BlendState.BlendAlpha = CurrentAlpha;
        BlendState.ElapsedTime = 0.0f;
    } else {
        BlendState.StartAlpha = 0.0f;
        BlendState.TargetAlpha = ScaledTargetAlpha;
        BlendState.bInheritedFromPrevious = false;
        BlendState.Phase = EOHBlendPhase::BlendIn;
        BlendState.BlendAlpha = 0.0f;
        BlendState.ElapsedTime = 0.0f;
    }

    AddBlendToBone(BoneName, BlendState);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Started PERMANENT alpha blend for %s: Target=%.2f (Scaled=%.2f)"),
               *BoneName.ToString(), TargetAlpha, ScaledTargetAlpha);
    }

    return true;
}

float UOHPACManager::GetTargetAlphaForMode(EOHTargetAlphaMode AlphaMode) const {
    switch (AlphaMode) {
    case EOHTargetAlphaMode::Full:
        return MaximumPhysicsAlpha;
    case EOHTargetAlphaMode::IdlePose:
        return IdlePoseRetentionAlpha;
    case EOHTargetAlphaMode::Secondary:
        return SecondaryMotionAlpha;
    case EOHTargetAlphaMode::Subtle:
        return SubtlePhysicsAlpha;
    case EOHTargetAlphaMode::Custom:
    default:
        return 1.0f; // Custom mode should specify alpha separately
    }
}

float UOHPACManager::ApplyBoneAlphaScaling(FName BoneName, float BaseAlpha) const {
    if (BaseAlpha <= 0.0f) {
        return 0.0f;
    }

    const FString BoneStr = BoneName.ToString().ToLower();
    float Multiplier = 1.0f;

    // Apply bone-specific multipliers
    if (BoneStr.Contains(TEXT("spine")) || BoneStr.Contains(TEXT("chest")))
        Multiplier = SpineAlphaMultiplier;
    else if (BoneStr.Contains(TEXT("arm")) || BoneStr.Contains(TEXT("clavicle")))
        Multiplier = ArmAlphaMultiplier;
    else if (BoneStr.Contains(TEXT("hand")))
        Multiplier = HandAlphaMultiplier;
    else if (BoneStr.Contains(TEXT("neck")) || BoneStr.Contains(TEXT("head")))
        Multiplier = NeckAlphaMultiplier;

    return FMath::Clamp(BaseAlpha * Multiplier, 0.0f, 1.0f);
}

// Helper function to create blend state with custom alpha
FOHBlendState UOHPACManager::CreateSmartBlendStateWithAlpha(FName BoneName, float TargetAlpha, float BlendIn,
                                                            float Hold, float BlendOut, FName ReactionTag) {
    FOHBlendState NewBlend;
    NewBlend.BlendID = ++NextBlendID;
    NewBlend.RootBone = BoneName;
    NewBlend.BlendInDuration = BlendIn;
    NewBlend.HoldDuration = Hold;
    NewBlend.BlendOutDuration = BlendOut;
    NewBlend.ReactionTag = ReactionTag;
    NewBlend.CustomTargetAlpha = TargetAlpha;
    NewBlend.TargetAlphaMode = EOHTargetAlphaMode::Custom;

    // Check for existing blend
    const float CurrentAlpha = GetBlendAlpha(BoneName);
    FOHBlendState* ExistingBlend = GetActiveBlendForBone(BoneName);

    if (ExistingBlend != nullptr) {
        // Smooth transition from current alpha to target alpha
        NewBlend.StartAlpha = CurrentAlpha;
        NewBlend.TargetAlpha = TargetAlpha;
        NewBlend.bInheritedFromPrevious = true;

        // Check if we're already close to target
        if (FMath::Abs(CurrentAlpha - TargetAlpha) < 0.1f) {
            NewBlend.Phase = (Hold < 0.0f) ? EOHBlendPhase::Permanent : EOHBlendPhase::Hold;
            NewBlend.BlendAlpha = TargetAlpha;
            NewBlend.ElapsedTime = 0.0f;
        } else {
            NewBlend.Phase = EOHBlendPhase::BlendIn;
            NewBlend.BlendAlpha = CurrentAlpha;
            NewBlend.ElapsedTime = 0.0f;
        }
    } else {
        // Fresh start
        NewBlend.StartAlpha = 0.0f;
        NewBlend.TargetAlpha = TargetAlpha;
        NewBlend.bInheritedFromPrevious = false;
        NewBlend.Phase = EOHBlendPhase::BlendIn;
        NewBlend.BlendAlpha = 0.0f;
        NewBlend.ElapsedTime = 0.0f;
    }

    return NewBlend;
}

void UOHPACManager::StopPhysicsBlend(FName BoneName, float BlendOutDuration) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends || BoneBlends->Num() == 0) {
        if (bVerboseLogging) {
            SafeLog(FString::Printf(TEXT("No active blends to stop for bone: %s"), *BoneName.ToString()));
        }
        return;
    }

    // Transition the most recent blend to blend-out phase
    FOHBlendState& LastBlend = (*BoneBlends)[BoneBlends->Num() - 1];

    if (LastBlend.Phase != EOHBlendPhase::BlendOut) {
        // Force transition to blend-out phase with custom duration
        LastBlend.Phase = EOHBlendPhase::BlendOut;
        LastBlend.ElapsedTime = 0.0f;
        LastBlend.BlendOutDuration = BlendOutDuration;

        // Set up blend-out transition
        LastBlend.StartAlpha = LastBlend.BlendAlpha;
        LastBlend.TargetAlpha = 0.0f;

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("🛑 Stopping physics blend for %s over %.2fs (BlendID: %d)"),
                   *BoneName.ToString(), BlendOutDuration, LastBlend.BlendID);
        }
    }
}

void UOHPACManager::StopPhysicsBlendChain(FName RootBoneName, float BlendOutDuration) {
    TArray<FName> Chain = GetBoneChain(RootBoneName, 0);

    int32 StoppedCount = 0;
    for (const FName& ChainBone : Chain) {
        if (ActiveBlends.Contains(ChainBone)) {
            StopPhysicsBlend(ChainBone, BlendOutDuration);
            StoppedCount++;
        }
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🛑 Stopped physics blend chain: %d bones"), StoppedCount);
    }
}

bool UOHPACManager::StartPermanentPhysicsBlend(FName BoneName, const FPhysicalAnimationData& Profile,
                                               float BlendInDuration, FName BlendTag) {
    if (!IsBoneValidForSimulation(BoneName)) {
        SafeLog(FString::Printf(TEXT("Bone not valid for permanent blending: %s"), *BoneName.ToString()), true);
        return false;
    }

    if (!IsSkeletalMeshBindingValid(true, bVerboseLogging)) {
        SafeLog(TEXT("Aborting permanent blend: SkeletalMesh binding is invalid!"), true);
        return false;
    }

    // STEP 1: Activate physics for the bone
    if (!TryActivateSimForBone(BoneName, Profile)) {
        SafeLog(FString::Printf(TEXT("Failed to activate physics for permanent blend: %s"), *BoneName.ToString()),
                true);
        return false;
    }

    // STEP 2: Create permanent blend state
    FOHBlendState BlendState;
    BlendState.BlendID = ++NextBlendID;
    BlendState.RootBone = BoneName;
    BlendState.BlendInDuration = BlendInDuration;
    BlendState.HoldDuration = 0.0f;     // No hold phase needed
    BlendState.BlendOutDuration = 0.0f; // No blend-out planned
    BlendState.ReactionTag = BlendTag;
    BlendState.bIsPermanent = true; // Mark as permanent

    // Check for existing blend for smooth transition
    const float CurrentAlpha = GetBlendAlpha(BoneName);
    FOHBlendState* ExistingBlend = GetActiveBlendForBone(BoneName);

    if (ExistingBlend != nullptr) {
        // Smooth transition from current state
        BlendState.StartAlpha = CurrentAlpha;
        BlendState.TargetAlpha = 1.0f;
        BlendState.bInheritedFromPrevious = true;
        BlendState.Phase = CurrentAlpha > 0.9f ? EOHBlendPhase::Permanent : EOHBlendPhase::BlendIn;
        BlendState.BlendAlpha = CurrentAlpha;
        BlendState.ElapsedTime = 0.0f;
    } else {
        // Fresh start
        BlendState.StartAlpha = 0.0f;
        BlendState.TargetAlpha = 1.0f;
        BlendState.bInheritedFromPrevious = false;
        BlendState.Phase = EOHBlendPhase::BlendIn;
        BlendState.BlendAlpha = 0.0f;
        BlendState.ElapsedTime = 0.0f;
    }

    // STEP 3: Add blend state
    AddBlendToBone(BoneName, BlendState);

    // STEP 4: Clean initial state
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        Body->SetLinearVelocity(FVector::ZeroVector, false);
        Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        Body->WakeInstance();
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ Started PERMANENT physics blend for %s (BlendID: %d, Tag: %s)"),
               *BoneName.ToString(), BlendState.BlendID, *BlendTag.ToString());
    }

    return true;
}

bool UOHPACManager::StartPermanentPhysicsBlendChain(FName RootBoneName, const FPhysicalAnimationData& Profile,
                                                    float BlendInDuration, FName BlendTag) {
    TArray<FName> Chain = GetBoneChain(RootBoneName, 0);
    TArray<FName> SuccessfulBones;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🔗 Starting PERMANENT physics blend chain from %s affecting %d bones"),
               *RootBoneName.ToString(), Chain.Num());
    }

    for (const FName& ChainBone : Chain) {
        if (StartPermanentPhysicsBlend(ChainBone, Profile, BlendInDuration, BlendTag)) {
            SuccessfulBones.Add(ChainBone);
        }
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("✅ PERMANENT blend chain started: %d/%d bones activated"), SuccessfulBones.Num(),
               Chain.Num());
    }

    return SuccessfulBones.Num() > 0;
}

void UOHPACManager::TransitionToPermanent(FName BoneName) {
    FOHBlendState* ActiveBlend = GetActiveBlendForBone(BoneName);
    if (ActiveBlend && !ActiveBlend->bIsPermanent) {
        // Convert existing blend to permanent
        ActiveBlend->bIsPermanent = true;
        ActiveBlend->Phase = EOHBlendPhase::Permanent;
        ActiveBlend->TargetAlpha = 1.0f;
        ActiveBlend->BlendAlpha = FMath::Max(ActiveBlend->BlendAlpha, 0.9f); // Ensure high alpha

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("🔄 Converted blend to permanent for %s"), *BoneName.ToString());
        }
    }
}

void UOHPACManager::ForceBlendOutPermanent(FName BoneName, float BlendOutDuration) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends || BoneBlends->Num() == 0) {
        return;
    }

    // Force the most recent blend to blend out, even if permanent
    FOHBlendState& LastBlend = (*BoneBlends)[BoneBlends->Num() - 1];

    LastBlend.bIsPermanent = false; // Remove permanent flag
    LastBlend.Phase = EOHBlendPhase::BlendOut;
    LastBlend.ElapsedTime = 0.0f;
    LastBlend.BlendOutDuration = BlendOutDuration;
    LastBlend.StartAlpha = LastBlend.BlendAlpha;
    LastBlend.TargetAlpha = 0.0f;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("🛑 FORCING blend-out for permanent blend on %s over %.2fs"),
               *BoneName.ToString(), BlendOutDuration);
    }
}

void UOHPACManager::EnsureBoneSimulatingPhysics(FName BoneName, bool bEnableChain) {
    if (!SkeletalMesh) {
        return;
    }

    if (bEnableChain) {
        // Enable simulating physics on this bone and all children
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(BoneName, true, true);
    } else {
        if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
            Body->SetInstanceSimulatePhysics(true);
            Body->WakeInstance();
            Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        }
    }
}

#pragma endregion
// ============================================================================
// DIRECT ACCESS HELPERS
// ============================================================================
#pragma region DIRECT ACCESS HELPERS
// Add cache safety validation:
FBodyInstance* UOHPACManager::GetBodyInstanceDirect(FName BoneName) const {
    if (FBodyInstance** Found = BodyInstanceCache.Find(BoneName)) {
        FBodyInstance* Body = *Found;
        // Validate cached pointer is still valid
        if (Body && Body->IsValidBodyInstance()) {
            return Body;
        }
        // Remove invalid cached entry
        BodyInstanceCache.Remove(BoneName);
        SafeLog(FString::Printf(TEXT("Removed invalid cached body for bone: %s"), *BoneName.ToString()), true);
    }

    // Lazy cache if isn't found or was invalid
    if (SkeletalMesh) {
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
            if (Body->IsValidBodyInstance()) {
                BodyInstanceCache.Add(BoneName, Body);
                return Body;
            }
        }
    }

    return nullptr;
}

FConstraintInstance* UOHPACManager::GetConstraintInstanceDirect(FName BoneName) const {
    if (FConstraintInstance** Found = ConstraintInstanceCache.Find(BoneName)) {
        return *Found;
    }

    return nullptr;
}

int32 UOHPACManager::GetBoneIndexDirect(FName BoneName) const {
    if (const int32* Found = BoneIndexCache.Find(BoneName)) {
        return *Found;
    }

    return INDEX_NONE;
}

#pragma endregion
// ============================================================================
// ACCESSORS
// ============================================================================
#pragma region ACCESSORS
FVector UOHPACManager::GetBoneVelocity(FName BoneName) const {
    if (const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName)) {
        return MotionData->GetLinearVelocity();
    }
    return FVector::ZeroVector;
}

FVector UOHPACManager::GetBoneAcceleration(FName BoneName) const {
    if (const FOHBoneMotionData* MotionData = BoneMotionMap.Find(BoneName)) {
        return MotionData->GetLinearAcceleration();
    }
    return FVector::ZeroVector;
}

// Thread-safe bone simulation check:
bool UOHPACManager::IsBoneSimulating(FName BoneName) const {
    // Use direct access but with safety checks
    if (FBodyInstance* Body = BodyInstanceCache.FindRef(BoneName)) {
        if (Body->IsValidBodyInstance()) {
            return Body->IsInstanceSimulatingPhysics();
        }
    }

    // Fallback to skeletal mesh lookup if the caches miss
    if (SkeletalMesh) {
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
            return Body->IsValidBodyInstance() && Body->IsInstanceSimulatingPhysics();
        }
    }

    return false;
}

float UOHPACManager::GetBlendAlpha(FName BoneName) const {
    // Check if bone has any active blends
    const TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (BoneBlends && BoneBlends->Num() > 0) {
        // Use the new effective alpha calculation that handles concurrent blends
        return CalculateEffectiveBlendAlpha(*BoneBlends);
    }

    // Fallback: Check if bone is simulating via reference counting
    if (BoneSimulationRefCount.Contains(BoneName)) {
        // Has reference count but no blend - assume full alpha
        return 1.0f;
    }

    // Not simulating
    return 0.0f;
}

EOHBlendPhase UOHPACManager::GetBlendPhase(FName BoneName) const {
    const TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends || BoneBlends->Num() == 0) {
        return EOHBlendPhase::BlendOut;
    }

    // Return the phase of the most recent blend
    return (*BoneBlends)[BoneBlends->Num() - 1].Phase;
}

int32 UOHPACManager::FindActiveBlendForBone(FName BoneName) const {
    if (const TArray<FOHBlendState>* BlendArray = ActiveBlends.Find(BoneName)) {
        // Return the most recent blend ID (highest ID number)
        int32 LatestBlendID = INDEX_NONE;
        for (const FOHBlendState& Blend : *BlendArray) {
            if (LatestBlendID == INDEX_NONE || Blend.BlendID > LatestBlendID) {
                LatestBlendID = Blend.BlendID;
            }
        }
        return LatestBlendID;
    }
    return INDEX_NONE;
}

FOHBlendState* UOHPACManager::GetActiveBlendForBone(FName BoneName) {
    if (TArray<FOHBlendState>* BlendArray = ActiveBlends.Find(BoneName)) {
        // Return the most recent blend (highest BlendID)
        FOHBlendState* LatestBlend = nullptr;
        for (FOHBlendState& Blend : *BlendArray) {
            if (LatestBlend == nullptr || Blend.BlendID > LatestBlend->BlendID) {
                LatestBlend = &Blend;
            }
        }
        return LatestBlend;
    }
    return nullptr;
}

FOHBlendState* UOHPACManager::GetBlendByID(int32 BlendID) {
    for (auto& BlendPair : ActiveBlends) {
        TArray<FOHBlendState>& BlendArray = BlendPair.Value;
        for (FOHBlendState& Blend : BlendArray) {
            if (Blend.BlendID == BlendID) {
                return &Blend;
            }
        }
    }
    return nullptr;
}

float UOHPACManager::GetConstraintStrain(FName BoneName) const {
    if (const FOHConstraintData* ConstraintData = ConstraintDataMap.Find(BoneName)) {
        return ConstraintData->GetCurrentStrain();
    }
    return 0.f;
}

void UOHPACManager::FixBlendSystemDiscrepancies() {
    UE_LOG(LogTemp, Warning, TEXT("=== Fixing Blend System Discrepancies ==="));

    int32 FixedBones = 0;

    for (const auto& BlendPair : ActiveBlends) {
        const FName& BoneName = BlendPair.Key;
        const TArray<FOHBlendState>& BoneBlends = BlendPair.Value;

        FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            continue;
        }

        // Calculate what the state should be
        float ExpectedAlpha = CalculateEffectiveBlendAlpha(BoneBlends);
        bool bShouldBeSimulating = ExpectedAlpha > 0.01f;

        // Check current state
        float ActualBlendWeight = Body->PhysicsBlendWeight;
        bool bActuallySimulating = Body->IsInstanceSimulatingPhysics();

        bool bNeedsFix = false;

        // Fix blend weight
        if (FMath::Abs(ActualBlendWeight - ExpectedAlpha) > 0.1f) {
            Body->PhysicsBlendWeight = ExpectedAlpha;
            bNeedsFix = true;
        }

        // Fix simulation state
        if (bActuallySimulating != bShouldBeSimulating) {
            Body->SetInstanceSimulatePhysics(bShouldBeSimulating);
            if (bShouldBeSimulating) {
                Body->WakeInstance();
            } else {
                Body->PutInstanceToSleep();
            }
            bNeedsFix = true;
        }

        if (bNeedsFix) {
            FixedBones++;
            UE_LOG(LogTemp, Log, TEXT("🔧 Fixed %s: Alpha %.2f→%.2f, Sim %s→%s"), *BoneName.ToString(),
                   ActualBlendWeight, ExpectedAlpha, bActuallySimulating ? TEXT("On") : TEXT("Off"),
                   bShouldBeSimulating ? TEXT("On") : TEXT("Off"));
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("✅ Fixed discrepancies on %d bones"), FixedBones);
}
#pragma endregion

// ============================================================================
// VALIDATION & UTILITY
// ============================================================================
#pragma region VALIDATION & UTILITY
bool UOHPACManager::IsBoneValidForSimulation(FName BoneName) const {
    // Bone must be in tracked set and not excluded
    if (!TrackedBones.Contains(BoneName)) {
        return false;
    }
    if (SimulationExclusions.Contains(BoneName)) {
        return false;
    }

    // Use cache if it's available (fast path)
    if (BodyInstanceCache.Contains(BoneName)) {
        return true;
    }

    // If cache is not built, check the physics asset directly
    if (CachedPhysicsAsset) {
        int32 BodyIndex = CachedPhysicsAsset->FindBodyIndex(BoneName);
        return BodyIndex != INDEX_NONE;
    }
    return false;
}

bool UOHPACManager::IsBoneInChain(FName BoneName, FName RootBone) const {
    if (BoneName == RootBone) {
        return true;
    }

    FName CurrentBone = BoneName;
    while (const FName* Parent = BoneParentMap.Find(CurrentBone)) {
        if (*Parent == RootBone) {
            return true;
        }
        CurrentBone = *Parent;
    }

    return false;
}

bool UOHPACManager::IsBoneNamePatternValid(FName BoneName) {
    if (BoneName.IsNone()) {
        return false;
    }

    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Disallowed substrings and prefixes/suffixes
    static const TArray<FString> DisallowedSubstrings = {TEXT("twist"), TEXT("ik"), TEXT("attach"), TEXT("helper"),
                                                         TEXT("finger")};
    static const TArray<FString> DisallowedPrefixes = {TEXT("vb_")};
    static const TArray<FString> DisallowedSuffixes = {TEXT("_dummy")};

    // Substring filters
    for (const FString& Disallowed : DisallowedSubstrings) {
        if (BoneNameStr.Contains(Disallowed)) {
            return false;
        }
    }

    // Prefix filters
    for (const FString& Prefix : DisallowedPrefixes) {
        if (BoneNameStr.StartsWith(Prefix)) {
            return false;
        }
    }

    // Suffix filters
    for (const FString& Suffix : DisallowedSuffixes) {
        if (BoneNameStr.EndsWith(Suffix)) {
            return false;
        }
    }

    // (Optional) Regex: Only allow Unreal-ish names
    static const FRegexPattern BonePattern(TEXT("^[a-z_][a-z0-9_]*$"));
    FRegexMatcher Matcher(BonePattern, BoneNameStr);
    if (!Matcher.FindNext()) {
        return false;
    }

    return true;
}

bool UOHPACManager::IsBoneMassValid(FName BoneName) const {
    if (!SkeletalMesh) {
        SafeLog(TEXT("IsBoneMassValid: SkeletalMeshComponent is null."), true);
        return false;
    }

    const FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        SafeLog(FString::Printf(TEXT("IsBoneMassValid: No valid BodyInstance for bone '%s'."), *BoneName.ToString()),
                true);
        return false;
    }

    const float BoneMass = Body->GetBodyMass();
    if (BoneMass > KINDA_SMALL_NUMBER) {
        return true;
    }
    SafeLog(FString::Printf(TEXT("IsBoneMassValid: Bone '%s' has zero or near-zero mass (%.3f)."), *BoneName.ToString(),
                            BoneMass),
            true);
    return false;
}

bool UOHPACManager::HasPhysicsBody(const FName& BoneName) const {
    // Check cache first
    if (BodyInstanceCache.Contains(BoneName)) {
        return true;
    }

    // Check physics asset
    if (CachedPhysicsAsset) {
        return CachedPhysicsAsset->FindBodyIndex(BoneName) != INDEX_NONE;
    }

    return false;
}

TArray<FName> UOHPACManager::GetBoneChain(FName RootBone, int32 MaxDepth) const {
    TArray<FName> Chain;
    TQueue<TPair<FName, int32>> BoneQueue;
    TSet<FName> Visited;
    bool bUsingFallback = false;

    auto GetChildren = [&](const FName& Bone) -> const TArray<FName>* {
        // Try BoneChildrenMap first
        if (BoneChildrenMap.Contains(Bone)) {
            return BoneChildrenMap.Find(Bone);
        }
        bUsingFallback = true;
        // Fallback: build from RefSkeleton
        static TArray<FName> TmpChildren;
        TmpChildren.Reset();

        if (!SkeletalMesh) {
            return nullptr;
        }
        const USkeletalMesh* MeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
        if (!MeshAsset) {
            return nullptr;
        }
        const FReferenceSkeleton& RefSkeleton = MeshAsset->GetRefSkeleton();
        int32 BoneIdx = RefSkeleton.FindBoneIndex(Bone);
        if (BoneIdx == INDEX_NONE) {
            return nullptr;
        }
        TArray<int32> ChildIndices;
        RefSkeleton.GetDirectChildBones(BoneIdx, ChildIndices);
        for (int32 ChildIdx : ChildIndices) {
            TmpChildren.Add(RefSkeleton.GetBoneName(ChildIdx));
        }
        return &TmpChildren;
    };

    BoneQueue.Enqueue(TPair<FName, int32>(RootBone, 0));

    while (!BoneQueue.IsEmpty()) {
        TPair<FName, int32> Current;
        BoneQueue.Dequeue(Current);

        const FName& BoneName = Current.Key;
        const int32 Depth = Current.Value;

        if (!Visited.Contains(BoneName)) {
            Visited.Add(BoneName);
            Chain.Add(BoneName);

            if (MaxDepth <= 0 || Depth < MaxDepth) {
                if (const TArray<FName>* Children = GetChildren(BoneName)) {
                    for (const FName& Child : *Children) {
                        BoneQueue.Enqueue(TPair<FName, int32>(Child, Depth + 1));
                    }
                }
            }
        }
    }

    if (bUsingFallback && bVerboseLogging) {
        SafeLog(TEXT("BoneChildrenMap missing/corrupt, used RefSkeleton fallback in GetBoneChain!"), true);
    }

    return Chain;
}

TArray<FBodyInstance*> UOHPACManager::GetSimulatableBodies(USkeletalMeshComponent* SkeletalMesh,
                                                           const TSet<FName>& SimulatableBones) {
    TArray<FBodyInstance*> OutBodies;
    if (!SkeletalMesh) {
        return OutBodies;
    }

    for (FBodyInstance* Body : SkeletalMesh->Bodies) {
        if (Body && Body->IsValidBodyInstance() && Body->BodySetup.IsValid()) {
            FName BoneName = Body->BodySetup->BoneName;
            if (SimulatableBones.Contains(BoneName)) {
                OutBodies.Add(Body);
            }
        }
    }
    return OutBodies;
}

void UOHPACManager::GetSimulatingBodiesBySimulatable(USkeletalMeshComponent* MeshComp,
                                                     const TSet<FName>& SimulatableBones,
                                                     TArray<FBodyInstance*>& OutValidSim,
                                                     TArray<FBodyInstance*>& OutInvalidSim) {
    OutValidSim.Empty();
    OutInvalidSim.Empty();
    if (!MeshComp) {
        return;
    }

    for (FBodyInstance* Body : MeshComp->Bodies) {
        if (!Body || !Body->IsValidBodyInstance()) {
            continue;
        }

        // IsInstanceSimulatingPhysics() is the safest runtime check!
        if (Body->IsInstanceSimulatingPhysics()) {
            FName BoneName = Body->BodySetup.IsValid() ? Body->BodySetup->BoneName : NAME_None;
            if (SimulatableBones.Contains(BoneName)) {
                OutValidSim.Add(Body);
            } else {
                OutInvalidSim.Add(Body);
            }
        }
    }
}

bool UOHPACManager::ValidateBodyInstances(TArray<FName>& OutMissingBones,
                                          TArray<FName>& OutInstancesWithoutBodies) const {
    OutMissingBones.Empty();
    OutInstancesWithoutBodies.Empty();

    if (!CachedPhysicsAsset) {
        SafeLog(TEXT("No PhysicsAsset is set."), true);
        return false;
    }
    if (!SkeletalMesh) {
        SafeLog(TEXT("No SkeletalMeshComponent is set."), true);
        return false;
    }

    // Bodies defined in the PhysicsAsset (BodySetups)
    TSet<FName> AssetBodyNames;
    for (const USkeletalBodySetup* BodySetup : CachedPhysicsAsset->SkeletalBodySetups) {
        if (BodySetup && !BodySetup->BoneName.IsNone()) {
            AssetBodyNames.Add(BodySetup->BoneName);
        }
    }

    // Runtime bodies (BodyInstances)
    TSet<FName> RuntimeBodyNames;
    for (const FBodyInstance* BodyInstance : SkeletalMesh->Bodies) {
        if (BodyInstance && BodyInstance->IsValidBodyInstance()) {
            if (BodyInstance->BodySetup.IsValid() && !BodyInstance->BodySetup->BoneName.IsNone()) {
                RuntimeBodyNames.Add(BodyInstance->BodySetup->BoneName);
            }
        }
    }

    // Bones you intend to track that are not in the PhysicsAsset
    for (const FName& BoneName : TrackedBones) {
        if (!AssetBodyNames.Contains(BoneName)) {
            OutMissingBones.Add(BoneName);
            SafeLog(FString::Printf(TEXT("Tracked bone missing in PhysicsAsset: %s"), *BoneName.ToString()), true);
        }
    }

    // Bodies defined in PhysicsAsset but not present at runtime
    for (const FName& AssetName : AssetBodyNames) {
        if (!RuntimeBodyNames.Contains(AssetName)) {
            OutMissingBones.Add(AssetName);
            SafeLog(FString::Printf(TEXT("Body in PhysicsAsset but NOT present at runtime: %s"), *AssetName.ToString()),
                    true);
        }
    }

    // Bodies present at runtime but not in the PhysicsAsset (rare)
    for (const FName& InstanceName : RuntimeBodyNames) {
        if (!AssetBodyNames.Contains(InstanceName)) {
            OutInstancesWithoutBodies.Add(InstanceName);
            SafeLog(
                FString::Printf(TEXT("Body present at runtime but NOT in PhysicsAsset: %s"), *InstanceName.ToString()),
                true);
        }
    }

    return OutMissingBones.Num() == 0 && OutInstancesWithoutBodies.Num() == 0;
}

bool UOHPACManager::ValidateConstraintInstances(TArray<FName>& OutMissingConstraints,
                                                TArray<FName>& OutRuntimeConstraintsNotInAsset,
                                                TArray<FName>& OutMismatchedConstraints) const {
    OutMissingConstraints.Empty();
    OutRuntimeConstraintsNotInAsset.Empty();
    OutMismatchedConstraints.Empty();

    if (!CachedPhysicsAsset) {
        SafeLog(TEXT("No PhysicsAsset is set."), true);
        return false;
    }
    if (!SkeletalMesh) {
        SafeLog(TEXT("No SkeletalMeshComponent is set."), true);
        return false;
    }

    // Constraints defined in the PhysicsAsset
    TMap<FName, const FConstraintInstance*> AssetConstraintMap;
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : CachedPhysicsAsset->ConstraintSetup) {
        if (!ConstraintTemplate) {
            continue;
        }
        const FConstraintInstance& TemplateInstance = ConstraintTemplate->DefaultInstance;
        const FName ConstraintName = TemplateInstance.JointName;
        if (ConstraintName.IsNone()) {
            continue;
        }
        AssetConstraintMap.Add(ConstraintName, &TemplateInstance);
    }

    // Runtime constraints in the SkeletalMesh
    TMap<FName, FConstraintInstance*> RuntimeConstraintMap;
    for (FConstraintInstance* RuntimeConstraint : SkeletalMesh->Constraints) {
        if (RuntimeConstraint && !RuntimeConstraint->JointName.IsNone()) {
            RuntimeConstraintMap.Add(RuntimeConstraint->JointName, RuntimeConstraint);
        }
    }

    // Validate constraints
    for (const auto& AssetPair : AssetConstraintMap) {
        const FName& ConstraintName = AssetPair.Key;
        const FConstraintInstance* AssetInstance = AssetPair.Value;

        FConstraintInstance** RuntimeInstancePtr = RuntimeConstraintMap.Find(ConstraintName);
        if (!RuntimeInstancePtr) {
            OutMissingConstraints.Add(ConstraintName);
            SafeLog(FString::Printf(TEXT("Constraint in PhysicsAsset but NOT present at runtime: %s"),
                                    *ConstraintName.ToString()),
                    true);
            continue;
        }

        // Compare parent/child bones for mismatches
        FConstraintInstance* RuntimeInstance = *RuntimeInstancePtr;
        if (RuntimeInstance->ConstraintBone1 != AssetInstance->ConstraintBone1 ||
            RuntimeInstance->ConstraintBone2 != AssetInstance->ConstraintBone2) {
            OutMismatchedConstraints.Add(ConstraintName);
            SafeLog(FString::Printf(TEXT("Constraint binding mismatch: %s. Asset (Parent: %s, Child: %s), Runtime "
                                         "(Parent: %s, Child: %s)"),
                                    *ConstraintName.ToString(), *AssetInstance->ConstraintBone1.ToString(),
                                    *AssetInstance->ConstraintBone2.ToString(),
                                    *RuntimeInstance->ConstraintBone1.ToString(),
                                    *RuntimeInstance->ConstraintBone2.ToString()),
                    true);
        }
    }

    // Constraints present at runtime but not in asset
    for (const auto& RuntimePair : RuntimeConstraintMap) {
        const FName& ConstraintName = RuntimePair.Key;
        if (!AssetConstraintMap.Contains(ConstraintName)) {
            OutRuntimeConstraintsNotInAsset.Add(ConstraintName);
            SafeLog(FString::Printf(TEXT("Constraint present at runtime but NOT in PhysicsAsset: %s"),
                                    *ConstraintName.ToString()),
                    true);
        }
    }

    return OutMissingConstraints.Num() == 0 && OutRuntimeConstraintsNotInAsset.Num() == 0 &&
           OutMismatchedConstraints.Num() == 0;
}

bool UOHPACManager::ValidatePhysicsAsset(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                                         TArray<FName>& OutMissingConstraints,
                                         TArray<FName>& OutRuntimeConstraintsNotInAsset,
                                         TArray<FName>& OutMismatchedConstraints) const {
    OutMissingBones.Empty();
    OutInstancesWithoutBodies.Empty();
    OutMissingConstraints.Empty();
    OutRuntimeConstraintsNotInAsset.Empty();
    OutMismatchedConstraints.Empty();

    if (!CachedPhysicsAsset) {
        SafeLog(TEXT("No PhysicsAsset is set."), true);
        return false;
    }
    if (!SkeletalMesh) {
        SafeLog(TEXT("SkeletalMeshComponent missing."), true);
        return false;
    }

    bool bBodiesValid = ValidateBodyInstances(OutMissingBones, OutInstancesWithoutBodies);
    bool bConstraintsValid =
        ValidateConstraintInstances(OutMissingConstraints, OutRuntimeConstraintsNotInAsset, OutMismatchedConstraints);

    return bBodiesValid && bConstraintsValid;
}

bool UOHPACManager::ValidateSetup(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                                  TArray<FName>& OutMissingConstraints, TArray<FName>& OutRuntimeConstraintsNotInAsset,
                                  TArray<FName>& OutMismatchedConstraints) const {
    OutMissingBones.Empty();
    OutInstancesWithoutBodies.Empty();
    OutMissingConstraints.Empty();
    OutRuntimeConstraintsNotInAsset.Empty();
    OutMismatchedConstraints.Empty();

    bool bValid = true;

    // Core component validation
    if (!SkeletalMesh) {
        SafeLog(TEXT("No SkeletalMeshComponent."), true);
        bValid = false;
    }
    if (!PhysicalAnimationComponent) {
        SafeLog(TEXT("No PhysicalAnimationComponent."), true);
        bValid = false;
    }

    if (!bValid) {
        SafeLog(TEXT("Core component validation failed. Skipping PhysicsAsset validation."), true);
        return false;
    }

    // Physics asset validation (but don't fail on warnings)
    ValidatePhysicsAsset(OutMissingBones, OutInstancesWithoutBodies, OutMissingConstraints,
                         OutRuntimeConstraintsNotInAsset, OutMismatchedConstraints);

    // Log warnings but don't fail initialization for missing optional bones
    if (OutMissingBones.Num() > 0) {
        SafeLog(FString::Printf(TEXT("%d tracked bones missing bodies (will be excluded from simulation)"),
                                OutMissingBones.Num()),
                true);
        for (const FName& Bone : OutMissingBones) {
            UE_LOG(LogTemp, Verbose, TEXT("[OHPACManager] Missing body for bone: %s"), *Bone.ToString());
        }
    }

    // Only fail on critical errors (mismatched constraints are critical)
    if (OutMismatchedConstraints.Num() > 0) {
        SafeLog(TEXT("Critical validation errors found - initialization failed"), true);
        return false;
    }

    SafeLog(FString::Printf(TEXT("Validation passed with %d warnings"),
                            OutMissingBones.Num() + OutInstancesWithoutBodies.Num() + OutMissingConstraints.Num() +
                                OutRuntimeConstraintsNotInAsset.Num()));

    return true; // Pass even with non-critical warnings
}

bool UOHPACManager::IsSkeletalMeshBindingValid(bool bAutoFix, bool bLog) const {
    USkeletalMeshComponent* BoundMesh = SkeletalMesh;
    USkeletalMeshComponent* PACMesh = nullptr;
    if (PhysicalAnimationComponent) {
        PACMesh = PhysicalAnimationComponent->GetSkeletalMesh();
    }

    bool bMatch = (BoundMesh && PACMesh && BoundMesh == PACMesh);

    if (!bMatch && bAutoFix && PhysicalAnimationComponent && BoundMesh) {
        // Heal the binding automatically!
        PhysicalAnimationComponent->SetSkeletalMeshComponent(BoundMesh);
        PACMesh = PhysicalAnimationComponent->GetSkeletalMesh();
        bMatch = (BoundMesh == PACMesh);

        if (bLog) {
            SafeLog(FString::Printf(TEXT("Auto-fixed PhysicalAnimationComponent SkeletalMesh binding: BoundMesh=%s, "
                                         "PACMesh=%s, Match=%d"),
                                    BoundMesh ? *BoundMesh->GetName() : TEXT("NULL"),
                                    PACMesh ? *PACMesh->GetName() : TEXT("NULL"), bMatch ? 1 : 0),
                    true);
        }
    } else if (bLog) {
        SafeLog(FString::Printf(TEXT("SkeletalMesh binding check: BoundMesh=%s, PACMesh=%s, Match=%d"),
                                BoundMesh ? *BoundMesh->GetName() : TEXT("NULL"),
                                PACMesh ? *PACMesh->GetName() : TEXT("NULL"), bMatch ? 1 : 0),
                true);
    }

    return bMatch;
}

// Add to ResetPACManager() method - replace the existing implementation:
void UOHPACManager::ResetPACManager() {
    SafeLog(TEXT("Resetting PAC Manager..."));

    // Stop all active blends first
    StopAllBlends();

    // Clear simulation states
    for (const auto& RefPair : BoneSimulationRefCount) {
        StopChainPhysicalAnimation(RefPair.Key, true);
    }

    // Clear all data structures
    BoneMotionMap.Empty();
    ConstraintDataMap.Empty();
    ActiveBlends.Empty();
    BoneSimulationRefCount.Empty();
    SimulatableBones.Empty();

    // Invalidate caches to prevent stale pointer access
    InvalidateCaches();

    // Reset ID counter
    NextBlendID = 1;

    // Mark as hasn't been initialized
    bIsInitialized = false;

    SafeLog(TEXT("PAC Manager reset complete"));
}

void UOHPACManager::InvalidateCaches() {
    BodyInstanceCache.Empty();
    ConstraintInstanceCache.Empty();
    BoneIndexCache.Empty();
    BoneParentMap.Empty();
    BoneChildrenMap.Empty();
}

void UOHPACManager::LogSystemState() const {
    UE_LOG(LogTemp, Log, TEXT("=== OHPACManager System State ==="));
    UE_LOG(LogTemp, Log, TEXT("Initialized: %s"), bIsInitialized ? TEXT("Yes") : TEXT("No"));
    UE_LOG(LogTemp, Log, TEXT("Tracked Bones: %d"), BoneMotionMap.Num());
    UE_LOG(LogTemp, Log, TEXT("Simulatable Bones: %d"), SimulatableBones.Num());
    UE_LOG(LogTemp, Log, TEXT("Active Blends: %d"), ActiveBlends.Num());
    UE_LOG(LogTemp, Log, TEXT("Constraints: %d"), ConstraintDataMap.Num());
    UE_LOG(LogTemp, Log, TEXT("Cached Bodies: %d"), BodyInstanceCache.Num());

    // Log cache efficiency
    const int32 TotalBones = TrackedBones.Num();
    const float CacheHitRate = TotalBones > 0 ? static_cast<float>(BodyInstanceCache.Num()) / TotalBones : 0.f;
    UE_LOG(LogTemp, Log, TEXT("Body Cache Hit Rate: %.1f%%"), CacheHitRate * 100.f);
}

void UOHPACManager::LogActiveBlends() const {
    UE_LOG(LogTemp, Log, TEXT("=== Active Blends Debug ==="));
    for (const auto& Pair : ActiveBlends) {
        const FName& BoneName = Pair.Key;
        const TArray<FOHBlendState>& BoneBlends = Pair.Value;

        UE_LOG(LogTemp, Log, TEXT("Bone %s: %d active blends"), *BoneName.ToString(), BoneBlends.Num());

        for (int32 i = 0; i < BoneBlends.Num(); ++i) {
            const FOHBlendState& Blend = BoneBlends[i];
            UE_LOG(LogTemp, Log, TEXT("  [%d] ID:%d Phase:%d Alpha:%.3f Elapsed:%.3f Tag:%s"), i, Blend.BlendID,
                   (int32)Blend.Phase, Blend.BlendAlpha, Blend.ElapsedTime, *Blend.ReactionTag.ToString());
        }

        const int32* RefCount = BoneSimulationRefCount.Find(BoneName);
        UE_LOG(LogTemp, Log, TEXT("  RefCount: %d"), RefCount ? *RefCount : 0);
    }
}

void UOHPACManager::LogSimState(FName BoneName) {
    if (!SkeletalMesh) {
        return;
    }
    FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
    if (!Body) {
        SafeLog(FString::Printf(TEXT("Bone %s: No body instance!"), *BoneName.ToString()), true);
        return;
    }
    UE_LOG(LogTemp, Log, TEXT("[OHPACManager] Bone %s: Simulating=%d, BlendWeight=%.2f, Mass=%.2f, Profile=%s"),
           *BoneName.ToString(), Body->IsInstanceSimulatingPhysics() ? 1 : 0, Body->PhysicsBlendWeight,
           Body->GetBodyMass(), *Body->GetBodySetup()->BoneName.ToString());
}

void UOHPACManager::LogPerformanceStats() const {
    const int32 MemoryFootprint = BoneMotionMap.GetAllocatedSize() + ConstraintDataMap.GetAllocatedSize() +
                                  ActiveBlends.GetAllocatedSize() + BodyInstanceCache.GetAllocatedSize() +
                                  ConstraintInstanceCache.GetAllocatedSize();

    SafeLog(FString::Printf(TEXT("Memory footprint: %d KB"), MemoryFootprint / 1024));
}

void UOHPACManager::DrawDebugOverlay() const {
    if (!SkeletalMesh) {
        return;
    }

    int32 DisplayIndex = 0;
    constexpr float VerticalStep = 25.f;
    constexpr float BaseOffset = 30.f;

    // Display header
    FVector HeaderOffset = FVector(0, 0, BaseOffset + VerticalStep * DisplayIndex++);
    DrawDebugString(GetWorld(), SkeletalMesh->GetComponentLocation() + HeaderOffset, TEXT("PAC Debug Overlay"), nullptr,
                    FColor::White, 0.f, true, 1.5f);

    for (const auto& BlendPair : ActiveBlends) {
        const FName& BoneName = BlendPair.Key;
        const TArray<FOHBlendState>& BoneBlends = BlendPair.Value;

        if (BoneBlends.Num() == 0) {
            continue;
        }

        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            continue;
        }

        float ActualBlendWeight = Body->PhysicsBlendWeight;
        bool bSim = Body->IsInstanceSimulatingPhysics();

        // Calculate effective alpha from our blend system
        float EffectiveAlpha = CalculateEffectiveBlendAlpha(BoneBlends);

        // Check for discrepancies
        float AlphaDifference = FMath::Abs(ActualBlendWeight - EffectiveAlpha);
        bool bDiscrepancy = AlphaDifference > 0.05f;

        FVector BoneLocation = SkeletalMesh->GetBoneLocation(BoneName);
        FVector Offset = FVector(0, 0, BaseOffset + VerticalStep * DisplayIndex++);

        // Color coding: Green = good, Yellow = minor issue, Red = major issue
        FColor DebugColor;
        if (!bSim) {
            DebugColor = FColor::Red; // Not simulating but should be
        } else if (bDiscrepancy) {
            DebugColor = FColor::Orange; // Blend weight mismatch
        } else if (BoneBlends.Num() > 1) {
            DebugColor = FColor::Yellow; // Concurrent blends
        } else {
            DebugColor = FColor::Green; // All good
        }

        // Build debug info string
        FString DebugText = FString::Printf(TEXT("%s"), *BoneName.ToString());

        // Add blend count and types
        int32 PermanentCount = 0;
        int32 TimedCount = 0;
        for (const FOHBlendState& Blend : BoneBlends) {
            if (Blend.bIsPermanent)
                PermanentCount++;
            else
                TimedCount++;
        }

        if (PermanentCount > 0 && TimedCount > 0) {
            DebugText += FString::Printf(TEXT("\n[%dP+%dT]"), PermanentCount, TimedCount);
        } else if (PermanentCount > 0) {
            DebugText += FString::Printf(TEXT("\n[%dP]"), PermanentCount);
        } else {
            DebugText += FString::Printf(TEXT("\n[%dT]"), TimedCount);
        }

        // Add weight info
        DebugText += FString::Printf(TEXT("\nEff: %.2f Act: %.2f"), EffectiveAlpha, ActualBlendWeight);

        // Add reference count
        if (const int32* RefCount = BoneSimulationRefCount.Find(BoneName)) {
            DebugText += FString::Printf(TEXT("\nRef: %d"), *RefCount);
        }

        // Add discrepancy warning
        if (bDiscrepancy) {
            DebugText += FString::Printf(TEXT("\n⚠️ MISMATCH"));
        }

        DrawDebugString(GetWorld(), BoneLocation + Offset, DebugText, nullptr, DebugColor, 0.f, true, 1.0f);

        // Draw line to show blend weight visually
        const float LineLength = 50.0f;
        FVector LineStart = BoneLocation + FVector(0, 0, 10);
        FVector LineEnd = LineStart + FVector(LineLength * EffectiveAlpha, 0, 0);
        DrawDebugLine(GetWorld(), LineStart, LineEnd, DebugColor, false, 0.05f, 0, 3.0f);
    }

    // Summary at bottom
    FVector SummaryOffset = FVector(0, 0, BaseOffset + VerticalStep * DisplayIndex);
    FString SummaryText = FString::Printf(TEXT("Total Active: %d bones, %d total blends"), ActiveBlends.Num(),
                                          GetTotalActiveBlendCount());
    DrawDebugString(GetWorld(), SkeletalMesh->GetComponentLocation() + SummaryOffset, SummaryText, nullptr,
                    FColor::Cyan, 0.f, true, 1.2f);
}

void UOHPACManager::SafeLog(const FString& Message, bool bWarning /*= false*/, bool bOnScreen /*= false*/) {
    if (bWarning) {
        UE_LOG(LogOHPAC, Warning, TEXT("[OHPACManager] %s"), *Message);
    } else {
        UE_LOG(LogOHPAC, Log, TEXT("[OHPACManager] %s"), *Message);
    }

    if (bOnScreen && GEngine) {
        const FColor TextColor = bWarning ? FColor::Yellow : FColor::Green;
        GEngine->AddOnScreenDebugMessage(
            /* Key      */ -1,
            /* Time     */ 3.0f,
            /* Color    */ TextColor,
            /* Message  */ FString::Printf(TEXT("[OHPACManager] %s"), *Message));
    }
}

bool UOHPACManager::IsBoneDrivenByPhysicalAnimation(const FName& BoneName) const {
    if (!SkeletalMesh) {
        return false;
    }

    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(/*bIncludesTerminated=*/false, ConstraintAccessors);

    const FString PhysicalAnimPrefix(TEXT("PhysicalAnimation_"));
    const FString TargetConstraintName = PhysicalAnimPrefix + BoneName.ToString();

    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        const FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint) {
            continue;
        }

        const FName& ConstraintName = Constraint->JointName;
        if (ConstraintName.ToString().Equals(TargetConstraintName, ESearchCase::IgnoreCase)) {
            // Check drive values
            const bool bLinearDrive = Constraint->ProfileInstance.LinearDrive.XDrive.Stiffness > KINDA_SMALL_NUMBER ||
                                      Constraint->ProfileInstance.LinearDrive.YDrive.Stiffness > KINDA_SMALL_NUMBER ||
                                      Constraint->ProfileInstance.LinearDrive.ZDrive.Stiffness > KINDA_SMALL_NUMBER;
            const bool bAngularDrive =
                Constraint->ProfileInstance.AngularDrive.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER ||
                Constraint->ProfileInstance.AngularDrive.SwingDrive.Stiffness > KINDA_SMALL_NUMBER ||
                Constraint->ProfileInstance.AngularDrive.TwistDrive.Stiffness > KINDA_SMALL_NUMBER;

            if (bLinearDrive || bAngularDrive) {
                return true;
            }
        }
    }
    return false;
}

bool UOHPACManager::HasPhysicalAnimationDrives(const FConstraintInstance* Constraint) {
    if (!Constraint) {
        return false;
    }

    // Check if constraint has active drives (indicating PAC usage)
    const auto& LinearDrive = Constraint->ProfileInstance.LinearDrive;
    const auto& AngularDrive = Constraint->ProfileInstance.AngularDrive;

    bool bHasLinearDrive = LinearDrive.XDrive.Stiffness > KINDA_SMALL_NUMBER ||
                           LinearDrive.YDrive.Stiffness > KINDA_SMALL_NUMBER ||
                           LinearDrive.ZDrive.Stiffness > KINDA_SMALL_NUMBER;

    bool bHasAngularDrive = AngularDrive.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER ||
                            AngularDrive.SwingDrive.Stiffness > KINDA_SMALL_NUMBER ||
                            AngularDrive.TwistDrive.Stiffness > KINDA_SMALL_NUMBER;

    return bHasLinearDrive || bHasAngularDrive;
}

FConstraintInstance* UOHPACManager::FindPhysicalAnimationConstraint(FName BoneName) const {
    if (!SkeletalMesh) {
        return nullptr;
    }

    // Physical Animation constraints are typically named "PhysicalAnimation_BoneName"
    const FString PAConstraintName = FString::Printf(TEXT("PhysicalAnimation_%s"), *BoneName.ToString());
    const FName PAConstraintFName(*PAConstraintName);

    // Search through all constraint instances
    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(false, ConstraintAccessors);

    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        const FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint) {
            continue;
        }

        // Check if this constraint matches our bone
        if (Constraint->JointName == PAConstraintFName || Constraint->ConstraintBone2 == BoneName || // Child bone
            Constraint->JointName.ToString().Contains(BoneName.ToString())) {
            // Verify this is actually a PAC constraint by checking drive values
            if (HasPhysicalAnimationDrives(Constraint)) {
                return const_cast<FConstraintInstance*>(Constraint);
            }
        }
    }

    // Fallback: Check cached constraint data
    const FOHConstraintData* ConstraintData = ConstraintDataMap.Find(BoneName);
    if (ConstraintData && ConstraintData->GetConstraintInstance()) {
        FConstraintInstance* CI = ConstraintData->GetConstraintInstance();
        if (HasPhysicalAnimationDrives(CI)) {
            return CI;
        }
    }

    return nullptr;
}

void UOHPACManager::DebugBodyPhysicsStates() {
    if (!SkeletalMesh) {
        return;
    }

    // Assume you have SimulatableBones as a TSet<FName>
    TArray<FBodyInstance*> SimBodies = GetSimulatableBodies(SkeletalMesh, SimulatableBones);

    int32 SimCount = 0;
    for (FBodyInstance* Body : SimBodies) {
        bool bSim = Body->IsInstanceSimulatingPhysics();
        FString BoneName = Body->BodySetup.IsValid() ? Body->BodySetup->BoneName.ToString() : TEXT("None");
        FString OwnerName = SkeletalMesh->GetName();
        FString OwnerPtr = FString::Printf(TEXT("%p"), SkeletalMesh);
        GEngine->AddOnScreenDebugMessage(
            -1, 0.15f, bSim ? FColor::Green : FColor::Red,
            FString::Printf(TEXT("[%s][%s] %s Simulating: %d"), *OwnerName, *OwnerPtr, *BoneName, bSim));
        if (bSim) {
            ++SimCount;
        }
    }
    GEngine->AddOnScreenDebugMessage(-1, 0.3f, FColor::Yellow,
                                     FString::Printf(TEXT("Total Simulatable Sim: %d [Component: %s, %p]"), SimCount,
                                                     *SkeletalMesh->GetName(), SkeletalMesh));
}

#pragma endregion

#pragma endregion

#pragma region PAC PROFILE MATHEMATICS

FPhysicalAnimationData UOHPACManager::CalculateOptimalPACProfile(FName BoneName, EOHProfileIntensity Intensity,
                                                                 bool bUseCriticalDamping, float MassOverride) const {
    if (!IsBoneValidForSimulation(BoneName)) {
        SafeLog(FString::Printf(TEXT("Cannot calculate profile for invalid bone: %s"), *BoneName.ToString()), true);
        return FPhysicalAnimationData();
    }

    // Get actual bone properties from physics system
    FOHBoneProperties BoneProps = AnalyzeBoneProperties(BoneName, MassOverride);

    if (BoneProps.Mass <= KINDA_SMALL_NUMBER) {
        SafeLog(FString::Printf(TEXT("Bone %s has invalid mass: %.3f"), *BoneName.ToString(), BoneProps.Mass), true);
        return FPhysicalAnimationData();
    }

    // Base strength values from mathematical guide
    float BasePositionStrength = 1000.0f;     // N/m - guide starting point
    float BaseOrientationStrength = 15000.0f; // N·m/rad - guide starting point

    // Apply intensity scaling
    float IntensityMultiplier = GetIntensityMultiplier(Intensity);
    BasePositionStrength *= IntensityMultiplier;
    BaseOrientationStrength *= IntensityMultiplier;

    // Apply bone-specific scaling based on analysis
    float BoneScalar = CalculateBoneScalar(BoneProps);
    BasePositionStrength *= BoneScalar;
    BaseOrientationStrength *= BoneScalar;

    // Mass-dependent scaling (from guide: PositionStrength_required ∝ Mass)
    const float ReferenceMass = 3.0f; // 3kg reference from guide
    const float MassScalar = BoneProps.Mass / ReferenceMass;

    float FinalPositionStrength = BasePositionStrength * MassScalar;

    // Inertia-dependent scaling (from guide: OrientationStrength_required ∝ MomentOfInertia)
    const float ReferenceInertia = 0.027f; // Reference from guide
    const float InertiaScalar = BoneProps.MomentOfInertia / ReferenceInertia;

    float FinalOrientationStrength = BaseOrientationStrength * InertiaScalar;

    // Calculate velocity strengths using critical damping for maximum stability
    float VelocityStrength, AngularVelocityStrength;

    if (bUseCriticalDamping) {
        // Critical damping formulas from guide (ζ = 1):
        // VelocityStrength = 2 × √(PositionStrength × Mass)
        // AngularVelocityStrength = 2 × √(OrientationStrength × MomentOfInertia)
        VelocityStrength = 2.0f * FMath::Sqrt(FinalPositionStrength * BoneProps.Mass);
        AngularVelocityStrength = 2.0f * FMath::Sqrt(FinalOrientationStrength * BoneProps.MomentOfInertia);
    } else {
        // Manual ratio for custom tuning
        VelocityStrength = FinalPositionStrength * 0.1f;
        AngularVelocityStrength = FinalOrientationStrength * 0.08f;
    }

    // Apply stability boundaries from guide to prevent oscillations
    FinalPositionStrength = FMath::Clamp(FinalPositionStrength, 100.0f, 10000.0f);
    VelocityStrength = FMath::Clamp(VelocityStrength, 10.0f, 1000.0f);
    FinalOrientationStrength = FMath::Clamp(FinalOrientationStrength, 1000.0f, 100000.0f);
    AngularVelocityStrength = FMath::Clamp(AngularVelocityStrength, 100.0f, 10000.0f);

    // Create the profile
    FPhysicalAnimationData Profile;
    Profile.PositionStrength = FinalPositionStrength;
    Profile.VelocityStrength = VelocityStrength;
    Profile.OrientationStrength = FinalOrientationStrength;
    Profile.AngularVelocityStrength = AngularVelocityStrength;
    Profile.bIsLocalSimulation = true;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning,
               TEXT("Profile for %s: Pos=%.1f, Vel=%.1f, Ori=%.1f, AngVel=%.1f (Mass=%.2f, Inertia=%.4f, Category=%s)"),
               *BoneName.ToString(), Profile.PositionStrength, Profile.VelocityStrength, Profile.OrientationStrength,
               Profile.AngularVelocityStrength, BoneProps.Mass, BoneProps.MomentOfInertia,
               *BoneProps.Category.ToString());
    }

    return Profile;
}

FOHBoneProperties UOHPACManager::AnalyzeBoneProperties(FName BoneName, float MassOverride) const {
    FOHBoneProperties Props;
    Props.BoneName = BoneName;

    // Get actual physics properties
    Props.Mass = MassOverride > 0.0f ? MassOverride : CalculateActualBoneMass(BoneName);
    Props.MomentOfInertia = CalculateActualMomentOfInertia(BoneName);
    Props.Length = CalculateBoneLength(BoneName);
    Props.HierarchyLevel = CalculateHierarchyLevel(BoneName);
    Props.Category = ClassifyBoneByStructure(BoneName);
    Props.bHasValidPhysics = HasValidPhysicsBody(BoneName);

    // Calculate derived properties
    Props.EffectiveMass = CalculateEffectiveMass(Props);
    Props.StabilityFactor = CalculateStabilityFactor(Props);

    return Props;
}

float UOHPACManager::CalculateActualBoneMass(FName BoneName) const {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        if (Body->IsValidBodyInstance()) {
            const float ActualMass = Body->GetBodyMass();
            if (ActualMass > KINDA_SMALL_NUMBER) {
                return ActualMass;
            }

            // If mass is zero, try to calculate from body setup
            if (Body->GetBodySetup()) {
                const float Volume = Body->GetBodySetup()->AggGeom.GetScaledVolume(FVector::OneVector);
                if (Volume > KINDA_SMALL_NUMBER) {
                    // Assume reasonable density for human tissue (1000 kg/m³)
                    const float Density = 1000.0f;
                    const float CalculatedMass = Volume * Density * 0.000001f; // Convert to kg
                    return FMath::Max(CalculatedMass, 0.1f);                   // Minimum 100g
                }
            }
        }
    }

    // Fallback: anatomically reasonable estimates
    return EstimateBoneMassFromAnatomy(BoneName);
}

float UOHPACManager::CalculateActualMomentOfInertia(FName BoneName) const {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        if (Body->IsValidBodyInstance() && Body->GetBodySetup()) {
            const float Mass = Body->GetBodyMass();
            if (Mass > KINDA_SMALL_NUMBER) {
                // Get body dimensions for inertia calculation
                FVector BoxExtent = FVector::ZeroVector;

                // Try to get actual collision bounds
                const FKAggregateGeom& AggGeom = Body->GetBodySetup()->AggGeom;
                if (AggGeom.GetElementCount() > 0) {
                    FBox BoundingBox = AggGeom.CalcAABB(FTransform::Identity);
                    BoxExtent = BoundingBox.GetExtent();
                }

                if (!BoxExtent.IsNearlyZero()) {
                    // Calculate moment of inertia for a box: I = (1/12) * m * (h² + w²)
                    const float Ix = (1.0f / 12.0f) * Mass * (BoxExtent.Y * BoxExtent.Y + BoxExtent.Z * BoxExtent.Z);
                    const float Iy = (1.0f / 12.0f) * Mass * (BoxExtent.X * BoxExtent.X + BoxExtent.Z * BoxExtent.Z);
                    const float Iz = (1.0f / 12.0f) * Mass * (BoxExtent.X * BoxExtent.X + BoxExtent.Y * BoxExtent.Y);

                    // Return average moment of inertia
                    return (Ix + Iy + Iz) / 3.0f;
                }
            }
        }
    }

    // Fallback: estimate based on mass and bone type
    const float Mass = CalculateActualBoneMass(BoneName);
    return EstimateMomentOfInertiaFromAnatomy(BoneName, Mass);
}

float UOHPACManager::EstimateBoneMassFromAnatomy(FName BoneName) {
    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Anatomically reasonable mass estimates for adult human
    if (BoneNameStr.Contains(TEXT("pelvis")) || BoneNameStr.Contains(TEXT("hip")))
        return 8.0f; // Heavy pelvic structure
    if (BoneNameStr.Contains(TEXT("spine")) || BoneNameStr.Contains(TEXT("chest")))
        return 5.0f; // Vertebrae and rib cage
    if (BoneNameStr.Contains(TEXT("head")))
        return 5.0f; // Head weight
    if (BoneNameStr.Contains(TEXT("neck")))
        return 2.0f; // Neck vertebrae

    // Upper extremities
    if (BoneNameStr.Contains(TEXT("clavicle")))
        return 1.5f; // Collar bone
    if (BoneNameStr.Contains(TEXT("upperarm")) || BoneNameStr.Contains(TEXT("humerus")))
        return 3.0f; // Upper arm (reference from guide)
    if (BoneNameStr.Contains(TEXT("lowerarm")) || BoneNameStr.Contains(TEXT("forearm")))
        return 2.0f; // Forearm
    if (BoneNameStr.Contains(TEXT("hand")))
        return 0.8f; // Hand

    // Lower extremities
    if (BoneNameStr.Contains(TEXT("thigh")) || BoneNameStr.Contains(TEXT("femur")))
        return 6.0f; // Thigh (heaviest limb bone)
    if (BoneNameStr.Contains(TEXT("calf")) || BoneNameStr.Contains(TEXT("shin")))
        return 3.5f; // Lower leg
    if (BoneNameStr.Contains(TEXT("foot")))
        return 1.2f; // Foot

    return 1.0f; // Safe default for unknown bones
}

float UOHPACManager::EstimateMomentOfInertiaFromAnatomy(FName BoneName, float Mass) {
    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Inertia scaling factors based on bone geometry
    float GeometryFactor = 0.027f; // Reference from guide

    if (BoneNameStr.Contains(TEXT("pelvis")) || BoneNameStr.Contains(TEXT("hip")))
        GeometryFactor = 0.15f; // Large, wide structure
    else if (BoneNameStr.Contains(TEXT("spine")) || BoneNameStr.Contains(TEXT("chest")))
        GeometryFactor = 0.08f; // Medium cylindrical
    else if (BoneNameStr.Contains(TEXT("head")))
        GeometryFactor = 0.06f; // Spherical approximation
    else if (BoneNameStr.Contains(TEXT("upperarm")) || BoneNameStr.Contains(TEXT("thigh")))
        GeometryFactor = 0.027f; // Reference from guide (long bones)
    else if (BoneNameStr.Contains(TEXT("lowerarm")) || BoneNameStr.Contains(TEXT("calf")))
        GeometryFactor = 0.02f; // Slightly smaller long bones
    else if (BoneNameStr.Contains(TEXT("hand")) || BoneNameStr.Contains(TEXT("foot")))
        GeometryFactor = 0.015f; // Complex but smaller structures
    else if (BoneNameStr.Contains(TEXT("neck")))
        GeometryFactor = 0.01f; // Small cylindrical

    return Mass * GeometryFactor;
}

FName UOHPACManager::ClassifyBoneByStructure(FName BoneName) {
    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Structural classification for physics purposes
    if (BoneNameStr.Contains(TEXT("pelvis")) || BoneNameStr.Contains(TEXT("hip")))
        return FName("Root");
    if (BoneNameStr.Contains(TEXT("spine")) || BoneNameStr.Contains(TEXT("chest")) ||
        BoneNameStr.Contains(TEXT("neck")))
        return FName("Axial");
    if (BoneNameStr.Contains(TEXT("arm")) || BoneNameStr.Contains(TEXT("clavicle")))
        return FName("UpperLimb");
    if (BoneNameStr.Contains(TEXT("leg")) || BoneNameStr.Contains(TEXT("thigh")) || BoneNameStr.Contains(TEXT("calf")))
        return FName("LowerLimb");
    if (BoneNameStr.Contains(TEXT("hand")) || BoneNameStr.Contains(TEXT("foot")))
        return FName("Extremity");
    if (BoneNameStr.Contains(TEXT("head")))
        return FName("Head");

    return FName("Unknown");
}

float UOHPACManager::CalculateBoneLength(FName BoneName) const {
    if (!SkeletalMesh) {
        return EstimateBoneLengthFromAnatomy(BoneName);
    }

    // Get bone transform
    const FTransform BoneTransform = SkeletalMesh->GetSocketTransform(BoneName);

    // Find primary child bone to calculate length
    const TArray<FName>* Children = BoneChildrenMap.Find(BoneName);
    if (Children && Children->Num() > 0) {
        // Use the first child (usually the primary continuation)
        const FTransform ChildTransform = SkeletalMesh->GetSocketTransform((*Children)[0]);
        const float Distance = FVector::Dist(BoneTransform.GetLocation(), ChildTransform.GetLocation());
        return FMath::Max(Distance, 2.0f); // Minimum 2cm
    }

    return EstimateBoneLengthFromAnatomy(BoneName);
}

float UOHPACManager::EstimateBoneLengthFromAnatomy(FName BoneName) {
    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Anatomical length estimates in cm
    if (BoneNameStr.Contains(TEXT("upperarm")) || BoneNameStr.Contains(TEXT("humerus")))
        return 35.0f;
    if (BoneNameStr.Contains(TEXT("lowerarm")) || BoneNameStr.Contains(TEXT("forearm")))
        return 30.0f;
    if (BoneNameStr.Contains(TEXT("thigh")) || BoneNameStr.Contains(TEXT("femur")))
        return 45.0f;
    if (BoneNameStr.Contains(TEXT("calf")) || BoneNameStr.Contains(TEXT("shin")))
        return 35.0f;
    if (BoneNameStr.Contains(TEXT("spine")))
        return 15.0f;
    if (BoneNameStr.Contains(TEXT("hand")))
        return 18.0f;
    if (BoneNameStr.Contains(TEXT("foot")))
        return 25.0f;
    if (BoneNameStr.Contains(TEXT("neck")))
        return 12.0f;
    if (BoneNameStr.Contains(TEXT("head")))
        return 20.0f;
    if (BoneNameStr.Contains(TEXT("pelvis")))
        return 18.0f;

    return 20.0f; // Default reasonable length
}

int32 UOHPACManager::CalculateHierarchyLevel(FName BoneName) const {
    int32 Level = 0;
    FName CurrentBone = BoneName;

    // Traverse up to root
    while (const FName* Parent = BoneParentMap.Find(CurrentBone)) {
        CurrentBone = *Parent;
        Level++;

        if (Level > 20)
            break; // Safety
    }

    return Level;
}

float UOHPACManager::CalculateEffectiveMass(const FOHBoneProperties& Props) const {
    // Effective mass considers hierarchy and constraint coupling
    float EffectiveMass = Props.Mass;

    // Add partial mass contribution from child bones
    if (const TArray<FName>* Children = BoneChildrenMap.Find(Props.BoneName)) {
        for (const FName& Child : *Children) {
            const float ChildMass = CalculateActualBoneMass(Child);
            EffectiveMass += ChildMass * 0.3f; // 30% coupling factor
        }
    }

    return EffectiveMass;
}

float UOHPACManager::CalculateStabilityFactor(const FOHBoneProperties& Props) {
    // Stability factor based on bone properties
    float StabilityFactor = 1.0f;

    // Hierarchy level effect (deeper bones are less stable)
    StabilityFactor *= FMath::Pow(0.9f, Props.HierarchyLevel);

    // Length effect (longer bones are less stable)
    const float ReferenceLength = 30.0f; // cm
    if (Props.Length > ReferenceLength) {
        StabilityFactor *= ReferenceLength / Props.Length;
    }

    // Mass effect (very light bones are less stable)
    if (Props.Mass < 1.0f) {
        StabilityFactor *= Props.Mass;
    }

    return FMath::Clamp(StabilityFactor, 0.1f, 2.0f);
}

float UOHPACManager::CalculateBoneScalar(const FOHBoneProperties& Props) {
    float Scalar = 1.0f;

    // Apply category-based scaling
    if (Props.Category == FName("Root"))
        Scalar *= 2.0f; // Root needs maximum stability
    else if (Props.Category == FName("Axial"))
        Scalar *= 1.5f; // Spine needs good control
    else if (Props.Category == FName("Head"))
        Scalar *= 1.3f; // Head needs stability
    else if (Props.Category == FName("UpperLimb"))
        Scalar *= 1.0f; // Base reference
    else if (Props.Category == FName("LowerLimb"))
        Scalar *= 1.2f; // Legs need more stability
    else if (Props.Category == FName("Extremity"))
        Scalar *= 0.8f; // Hands/feet more flexible

    // Apply stability factor
    Scalar *= Props.StabilityFactor;

    return Scalar;
}

bool UOHPACManager::HasValidPhysicsBody(FName BoneName) const {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        return Body->IsValidBodyInstance() && Body->GetBodySetup() && Body->GetBodyMass() > KINDA_SMALL_NUMBER;
    }
    return false;
}

float UOHPACManager::GetIntensityMultiplier(EOHProfileIntensity Intensity) {
    switch (Intensity) {
    case EOHProfileIntensity::VeryLight:
        return 0.3f;
    case EOHProfileIntensity::Light:
        return 0.6f;
    case EOHProfileIntensity::Medium:
        return 1.0f; // Base reference
    case EOHProfileIntensity::Strong:
        return 1.5f;
    case EOHProfileIntensity::VeryStrong:
        return 2.0f;
    default:
        return 1.0f;
    }
}

#pragma endregion

#pragma region IDLE POSE RETENTION

void UOHPACManager::StartIdlePoseRetention(const TArray<FName>& TestBones, EOHProfileIntensity Intensity,
                                           bool bValidateCalculations) {
    if (!bIsInitialized) {
        SafeLog(TEXT("PAC Manager not initialized - run InitializePACManager first"), true);
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("=== Testing Idle Pose Retention ==="));

    // Determine bones to test
    TArray<FName> BonesToTest = TestBones;
    if (BonesToTest.Num() == 0) {
        // Use upper body bones for idle test
        BonesToTest = GetIdleTestBones();
    }

    UE_LOG(LogTemp, Warning, TEXT("Testing %d bones for idle pose retention"), BonesToTest.Num());

    // Validate calculations if requested
    if (bValidateCalculations) {
        ValidateBoneCalculations(BonesToTest);
    }

    // Apply pose retention profiles
    int32 SuccessfulBones = 0;
    for (const FName& BoneName : BonesToTest) {
        if (ApplyIdlePoseRetention(BoneName, Intensity)) {
            SuccessfulBones++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("✅ Idle pose retention applied to %d/%d bones"), SuccessfulBones, BonesToTest.Num());

    // Log summary
    LogPoseRetentionSummary(BonesToTest);
}

void UOHPACManager::StartIdlePoseRetentionDefault() {
    StartIdlePoseRetention(TArray<FName>(), EOHProfileIntensity::Light, true);
}

void UOHPACManager::StopIdlePoseRetention(const TArray<FName>& BonesToStop, float BlendOutDuration) {
    TArray<FName> BonesToProcess = BonesToStop;
    if (BonesToProcess.Num() == 0) {
        BonesToProcess = GetIdleTestBones();
    }

    int32 StoppedCount = 0;
    for (const FName& BoneName : BonesToProcess) {
        if (ActiveBlends.Contains(BoneName)) {
            StopPhysicsBlend(BoneName, BlendOutDuration);
            StoppedCount++;
        } else if (IsBoneSimulating(BoneName)) {
            StopBonePhysicalAnimation(BoneName, true, true, bVerboseLogging);
            StoppedCount++;
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("Smoothly stopping pose retention on %d bones over %.2fs"), StoppedCount,
           BlendOutDuration);
}

void UOHPACManager::StopIdlePoseRetentionDefault() {
    StopIdlePoseRetention(TArray<FName>(), 0.5f);
}

void UOHPACManager::QuickIdleTest() {
    // Simple one-button test for idle pose retention
    StartIdlePoseRetention(TArray<FName>(), EOHProfileIntensity::Light, true);
}

bool UOHPACManager::ApplyIdlePoseRetention(FName BoneName, EOHProfileIntensity Intensity) {
    if (!IsBoneValidForSimulation(BoneName)) {
        return false;
    }

    // Calculate optimal profile for idle pose retention
    FPhysicalAnimationData IdleProfile = CalculateOptimalPACProfile(BoneName, Intensity, true);

    if (IdleProfile.PositionStrength <= 0.0f) {
        SafeLog(FString::Printf(TEXT("Invalid profile calculated for bone: %s"), *BoneName.ToString()), true);
        return false;
    }

    // Get current bone state before applying physics
    const FTransform InitialTransform = SkeletalMesh->GetSocketTransform(BoneName);

    // Start physics simulation with pose retention
    bool bSuccess = StartBonePhysicalAnimation(BoneName, IdleProfile, true, true, bVerboseLogging);

    if (bSuccess) {
        // Ensure bone starts in correct pose
        if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
            // Set initial state to match animation
            Body->SetBodyTransform(InitialTransform, ETeleportType::ResetPhysics);
            Body->SetLinearVelocity(FVector::ZeroVector, false);
            Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);

            // Ensure physics blend weight is set for pose retention
            Body->PhysicsBlendWeight = 1.0f;

            // Wake body to start physics
            Body->WakeInstance();
        }

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Log, TEXT("Applied idle pose retention to %s: Pos=%.1f, Ori=%.1f"), *BoneName.ToString(),
                   IdleProfile.PositionStrength, IdleProfile.OrientationStrength);
        }
    } else {
        SafeLog(FString::Printf(TEXT("Failed to apply pose retention to bone: %s"), *BoneName.ToString()), true);
    }

    return bSuccess;
}

TArray<FName> UOHPACManager::GetIdleTestBones() const {
    TArray<FName> IdleBones;

    // Conservative bone set for idle testing (upper body focus)
    const TArray<FName> CandidateBones = {
        "spine_01",   "spine_02",   "spine_03",   "head",       "clavicle_l",
        "clavicle_r", "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r",
    };

    // Only include bones that are actually simulatable
    for (const FName& BoneName : CandidateBones) {
        if (SimulatableBones.Contains(BoneName) && IsBoneValidForSimulation(BoneName)) {
            IdleBones.Add(BoneName);
        }
    }

    return IdleBones;
}

void UOHPACManager::ValidateBoneCalculations(const TArray<FName>& BonesToValidate) {
    UE_LOG(LogTemp, Warning, TEXT("=== Validating Bone Calculations ==="));

    for (const FName& BoneName : BonesToValidate) {
        // Analyze bone properties
        FOHBoneProperties Props = AnalyzeBoneProperties(BoneName);

        // Calculate profile
        FPhysicalAnimationData Profile = CalculateOptimalPACProfile(BoneName, EOHProfileIntensity::Medium, true);

        // Validate critical damping ratios
        float ExpectedVelocityStrength = 2.0f * FMath::Sqrt(Profile.PositionStrength * Props.Mass);
        float ExpectedAngularVelocityStrength = 2.0f * FMath::Sqrt(Profile.OrientationStrength * Props.MomentOfInertia);

        float VelocityRatio = Profile.VelocityStrength / ExpectedVelocityStrength;
        float AngularRatio = Profile.AngularVelocityStrength / ExpectedAngularVelocityStrength;

        // Check if ratios are close to 1.0 (indicating proper critical damping)
        bool bVelocityValid = FMath::Abs(VelocityRatio - 1.0f) < 0.15f;
        bool bAngularValid = FMath::Abs(AngularRatio - 1.0f) < 0.15f;

        FString ValidationStatus = (bVelocityValid && bAngularValid) ? TEXT("✅") : TEXT("⚠️");

        UE_LOG(LogTemp, Log, TEXT("%s %s: Mass=%.2f, Inertia=%.4f, Length=%.1f, Category=%s"), *ValidationStatus,
               *BoneName.ToString(), Props.Mass, Props.MomentOfInertia, Props.Length, *Props.Category.ToString());

        UE_LOG(LogTemp, Log, TEXT("    Profile: Pos=%.1f, Vel=%.1f (ratio=%.2f), Ori=%.1f, AngVel=%.1f (ratio=%.2f)"),
               Profile.PositionStrength, Profile.VelocityStrength, VelocityRatio, Profile.OrientationStrength,
               Profile.AngularVelocityStrength, AngularRatio);

        if (!bVelocityValid || !bAngularValid) {
            UE_LOG(LogTemp, Warning, TEXT("    ⚠️ Critical damping ratios out of range - may cause instability"));
        }
    }
}

void UOHPACManager::LogPoseRetentionSummary(const TArray<FName>& TestedBones) {
    UE_LOG(LogTemp, Warning, TEXT("=== Pose Retention Summary ==="));

    int32 SimulatingCount = 0;
    int32 ValidPhysicsCount = 0;
    float TotalMass = 0.0f;

    for (const FName& BoneName : TestedBones) {
        FOHBoneProperties Props = AnalyzeBoneProperties(BoneName);

        if (Props.bHasValidPhysics) {
            ValidPhysicsCount++;
            TotalMass += Props.Mass;
        }

        if (IsBoneSimulating(BoneName)) {
            SimulatingCount++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("Tested Bones: %d"), TestedBones.Num());
    UE_LOG(LogTemp, Warning, TEXT("Valid Physics Bodies: %d"), ValidPhysicsCount);
    UE_LOG(LogTemp, Warning, TEXT("Currently Simulating: %d"), SimulatingCount);
    UE_LOG(LogTemp, Warning, TEXT("Total Mass: %.2f kg"), TotalMass);

    // Performance estimate
    float EstimatedFrameTime = SimulatingCount * 0.5f; // Rough estimate: 0.5ms per bone
    UE_LOG(LogTemp, Warning, TEXT("Estimated Performance Impact: %.1f ms/frame"), EstimatedFrameTime);

    if (SimulatingCount < TestedBones.Num()) {
        UE_LOG(LogTemp, Warning, TEXT("⚠️ Some bones failed to simulate - check bone validity and physics setup"));
    }
}
#pragma endregion

// Testing function for full chain simulation
void UOHPACManager::TestFullChainSimulation() {
    UE_LOG(LogTemp, Warning, TEXT("=== Testing Full Chain Simulation ==="));
    // Test spine + neck + head chain
    TArray<FName> SpineChain = {"pelvis", "spine_01", "spine_02", "spine_03", "neck_01", "head"};

    // Test arm chains including hands
    TArray<FName> LeftArmChain = {"clavicle_l", "upperarm_l", "lowerarm_l", "hand_l"};

    TArray<FName> RightArmChain = {"clavicle_r", "upperarm_r", "lowerarm_r", "hand_r"};

    // Apply chain-aware stabilization
    ApplyChainStabilization(SpineChain, EOHProfileIntensity::Strong);
    ApplyChainStabilization(LeftArmChain, EOHProfileIntensity::Medium);
    ApplyChainStabilization(RightArmChain, EOHProfileIntensity::Medium);

    // Analyze chains
    FOHChainAnalysis SpineAnalysis = AnalyzeChain("pelvis");
    FOHChainAnalysis ArmAnalysis = AnalyzeChain("clavicle_l");

    UE_LOG(LogTemp, Warning, TEXT("Spine Chain: Length=%d, Risk=%.2f, Anchors=%s"), SpineAnalysis.ChainLength,
           SpineAnalysis.StabilityRisk, SpineAnalysis.bHasKinematicAnchors ? TEXT("Yes") : TEXT("No"));

    UE_LOG(LogTemp, Warning, TEXT("Arm Chain: Length=%d, Risk=%.2f, Anchors=%s"), ArmAnalysis.ChainLength,
           ArmAnalysis.StabilityRisk, ArmAnalysis.bHasKinematicAnchors ? TEXT("Yes") : TEXT("No"));
}

#pragma region Chain Analysis
FPhysicalAnimationData UOHPACManager::CalculateChainAwareProfile(FName BoneName, EOHProfileIntensity BaseIntensity,
                                                                 bool bAccountForChainPosition) const {
    // Start with base optimal profile
    FPhysicalAnimationData Profile = CalculateOptimalPACProfile(BoneName, BaseIntensity, true);

    if (!bAccountForChainPosition) {
        return Profile;
    }

    // Analyze the chain this bone belongs to
    FOHChainAnalysis ChainInfo = AnalyzeChainForBone(BoneName);

    // Calculate chain-specific modifiers using configurable values
    float ChainStabilityMultiplier = CalculateChainStabilityMultiplier(BoneName, ChainInfo);
    float PositionInChainMultiplier = CalculatePositionInChainMultiplier(BoneName, ChainInfo);
    float ContinuityMultiplier = CalculateContinuityMultiplier(ChainInfo);

    // Apply chain-aware scaling
    Profile.PositionStrength *= ChainStabilityMultiplier * PositionInChainMultiplier * ContinuityMultiplier;
    Profile.OrientationStrength *= ChainStabilityMultiplier * PositionInChainMultiplier * ContinuityMultiplier;

    // Increase damping for chain stability using configurable values
    float DampingMultiplier = FMath::Lerp(1.0f, MaxDampingMultiplier, ChainInfo.StabilityRisk);
    Profile.VelocityStrength *= DampingMultiplier;
    Profile.AngularVelocityStrength *= DampingMultiplier;

    // Use configurable bounds
    Profile.PositionStrength = FMath::Clamp(Profile.PositionStrength, MinPositionStrength, MaxPositionStrength);
    Profile.VelocityStrength = FMath::Clamp(Profile.VelocityStrength, MinVelocityStrength, MaxVelocityStrength);
    Profile.OrientationStrength =
        FMath::Clamp(Profile.OrientationStrength, MinOrientationStrength, MaxOrientationStrength);
    Profile.AngularVelocityStrength =
        FMath::Clamp(Profile.AngularVelocityStrength, MinAngularVelocityStrength, MaxAngularVelocityStrength);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning,
               TEXT("Chain-aware profile for %s: Pos=%.1f, Ori=%.1f (Chain Risk=%.2f, Continuity=%.2f)"),
               *BoneName.ToString(), Profile.PositionStrength, Profile.OrientationStrength, ChainInfo.StabilityRisk,
               ContinuityMultiplier);
    }

    return Profile;
}

FOHChainAnalysis UOHPACManager::AnalyzeChain(FName RootBone) const {
    FOHChainAnalysis Analysis;

    // Get full chain from root
    Analysis.ChainBones = GetBoneChain(RootBone, 0);
    Analysis.ChainLength = Analysis.ChainBones.Num();

    // Analyze each bone in chain
    int32 ConsecutiveSimCount = 0;
    int32 MaxConsecutiveSimCount = 0;
    bool bPreviousWasSimulated = false;

    for (int32 i = 0; i < Analysis.ChainBones.Num(); ++i) {
        const FName& BoneName = Analysis.ChainBones[i];

        // Check if bone is simulatable and currently simulating
        bool bIsSimulated = SimulatableBones.Contains(BoneName) && IsBoneSimulating(BoneName);
        Analysis.SimulationStates.Add(bIsSimulated);

        if (bIsSimulated) {
            // Add mass
            Analysis.TotalChainMass += CalculateActualBoneMass(BoneName);

            // Count consecutive simulated bones
            if (bPreviousWasSimulated) {
                ConsecutiveSimCount++;
            } else {
                ConsecutiveSimCount = 1;
            }
            MaxConsecutiveSimCount = FMath::Max(MaxConsecutiveSimCount, ConsecutiveSimCount);
        } else {
            // Reset consecutive count on kinematic bone
            ConsecutiveSimCount = 0;
        }

        bPreviousWasSimulated = bIsSimulated;
    }

    Analysis.ContinuousSimCount = MaxConsecutiveSimCount;

    // Check for kinematic anchors (non-simulated endpoints)
    bool bHasRootAnchor = Analysis.SimulationStates.Num() > 0 && !Analysis.SimulationStates[0];
    bool bHasEndAnchor = Analysis.SimulationStates.Num() > 0 && !Analysis.SimulationStates.Last();
    Analysis.bHasKinematicAnchors = bHasRootAnchor || bHasEndAnchor;

    // Calculate stability risk (0 = stable, 1 = very risky)
    float LengthRisk = FMath::Clamp(Analysis.ContinuousSimCount / 6.0f, 0.0f, 1.0f); // Risk increases with chain length
    float AnchorRisk = Analysis.bHasKinematicAnchors ? 0.0f : 0.4f;                  // Higher risk without anchors
    float MassRisk = FMath::Clamp(Analysis.TotalChainMass / 20.0f, 0.0f, 0.3f);      // Risk with heavy chains

    Analysis.StabilityRisk = FMath::Clamp(LengthRisk + AnchorRisk + MassRisk, 0.0f, 1.0f);

    return Analysis;
}

FOHChainAnalysis UOHPACManager::AnalyzeChainForBone(FName BoneName) const {
    // Find the root of the chain this bone belongs to
    FName ChainRoot = FindChainRoot(BoneName);
    return AnalyzeChain(ChainRoot);
}

FName UOHPACManager::FindChainRoot(FName BoneName) const {
    // Traverse up to find a logical chain root (pelvis, clavicle, etc.)
    FName CurrentBone = BoneName;

    while (const FName* Parent = BoneParentMap.Find(CurrentBone)) {
        FName ParentBone = *Parent;
        FString ParentStr = ParentBone.ToString().ToLower();

        // Stop at logical chain roots
        if (ParentStr.Contains(TEXT("pelvis")) || ParentStr.Contains(TEXT("root")) ||
            ParentStr.Contains(TEXT("clavicle"))) {
            return ParentBone;
        }

        CurrentBone = ParentBone;
    }

    return CurrentBone; // Return topmost bone if no logical root found
}

float UOHPACManager::CalculateChainStabilityMultiplier(FName BoneName, const FOHChainAnalysis& ChainInfo) const {
    float BaseMultiplier = 1.0f;

    // Increase strength based on stability risk using configurable max
    BaseMultiplier += ChainInfo.StabilityRisk * MaxStabilityRiskMultiplier;

    // Extra strength for critical bones using configurable value
    FString BoneStr = BoneName.ToString().ToLower();
    if (BoneStr.Contains(TEXT("spine")) || BoneStr.Contains(TEXT("pelvis"))) {
        BaseMultiplier *= CoreBoneExtraMultiplier;
    }

    return BaseMultiplier;
}

float UOHPACManager::CalculatePositionInChainMultiplier(FName BoneName, const FOHChainAnalysis& ChainInfo) const {
    int32 BoneIndex = ChainInfo.ChainBones.Find(BoneName);
    if (BoneIndex == INDEX_NONE) {
        return 1.0f;
    }

    float ChainPosition = static_cast<float>(BoneIndex) / FMath::Max(ChainInfo.ChainLength - 1, 1);

    // Use configurable multipliers
    if (ChainPosition < 0.2f) {
        return ChainRootStrengthMultiplier; // Root area
    } else if (ChainPosition < 0.8f) {
        return ChainMiddleStrengthMultiplier; // Middle area
    } else {
        return ChainEndStrengthMultiplier; // End area
    }
}

float UOHPACManager::CalculateContinuityMultiplier(const FOHChainAnalysis& ChainInfo) const {
    // Use configurable continuous chain multiplier
    if (ChainInfo.ContinuousSimCount <= 2) {
        return 1.0f; // Short chains are stable
    } else if (ChainInfo.ContinuousSimCount <= 4) {
        return FMath::Lerp(1.0f, ContinuousChainMultiplier, 0.5f); // Medium chains
    } else {
        return ContinuousChainMultiplier; // Long continuous chains
    }
}

void UOHPACManager::ApplyChainStabilization(const TArray<FName>& ChainBones, EOHProfileIntensity BaseIntensity) {
    UE_LOG(LogTemp, Warning, TEXT("=== Applying Chain Stabilization ==="));

    for (const FName& BoneName : ChainBones) {
        if (!IsBoneValidForSimulation(BoneName)) {
            continue;
        }

        // Calculate chain-aware profile
        FPhysicalAnimationData StabilizedProfile = CalculateChainAwareProfile(BoneName, BaseIntensity, true);

        // Apply immediately for testing
        if (ApplyIdlePoseRetention(BoneName, BaseIntensity)) {
            // Override with stabilized profile
            ApplyPhysicalAnimationProfile(BoneName, StabilizedProfile);

            UE_LOG(LogTemp, Log, TEXT("Applied stabilized profile to %s: Pos=%.1f, Ori=%.1f"), *BoneName.ToString(),
                   StabilizedProfile.PositionStrength, StabilizedProfile.OrientationStrength);
        }
    }
}

void UOHPACManager::AnalyzeAllBones() {
    UE_LOG(LogTemp, Warning, TEXT("=== Complete Bone Analysis ==="));

    if (!bIsInitialized) {
        SafeLog(TEXT("PAC Manager not initialized"), true);
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("Analyzing %d simulatable bones"), SimulatableBones.Num());

    // Categories for analysis
    TMap<FName, TArray<FName>> BonesByCategory;

    for (const FName& BoneName : SimulatableBones) {
        FOHBoneProperties Props = AnalyzeBoneProperties(BoneName);

        // Group by category
        TArray<FName>& CategoryBones = BonesByCategory.FindOrAdd(Props.Category);
        CategoryBones.Add(BoneName);

        UE_LOG(LogTemp, Log, TEXT("%s: Mass=%.2f kg, Inertia=%.4f, Length=%.1f cm, Level=%d, Category=%s, Physics=%s"),
               *BoneName.ToString(), Props.Mass, Props.MomentOfInertia, Props.Length, Props.HierarchyLevel,
               *Props.Category.ToString(), Props.bHasValidPhysics ? TEXT("Valid") : TEXT("Invalid"));
    }

    // Summary by category
    UE_LOG(LogTemp, Warning, TEXT("=== Bone Categories ==="));
    for (const auto& CategoryPair : BonesByCategory) {
        const FName& Category = CategoryPair.Key;
        const TArray<FName>& Bones = CategoryPair.Value;

        UE_LOG(LogTemp, Warning, TEXT("%s: %d bones"), *Category.ToString(), Bones.Num());
    }
}
#pragma endregion

#pragma region Permanent Pose Retention

// ============================================================================
// TESTING FUNCTIONS FOR PERMANENT BLENDS
// ============================================================================

void UOHPACManager::TestPermanentPoseRetention() {
    UE_LOG(LogTemp, Warning, TEXT("=== Testing PERMANENT Pose Retention ==="));

    // Test with smooth blend-in but permanent activation
    TArray<FName> TestBones = GetIdleTestBones();

    int32 SuccessfulBones = 0;
    for (const FName& BoneName : TestBones) {
        FPhysicalAnimationData Profile = CalculateChainAwareProfile(BoneName, EOHProfileIntensity::Medium, true);

        if (StartPermanentPhysicsBlend(BoneName, Profile, TestBlendInDuration, FName("PermanentPoseTest"))) {
            SuccessfulBones++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("✅ Started permanent pose retention on %d/%d bones (%.2fs blend-in)"),
           SuccessfulBones, TestBones.Num(), TestBlendInDuration);
    UE_LOG(LogTemp, Warning, TEXT("   Physics will remain active until manually stopped"));
}

void UOHPACManager::StopPermanentPoseRetention(float BlendOutDuration) {
    TArray<FName> TestBones = GetIdleTestBones();

    int32 StoppedCount = 0;
    for (const FName& BoneName : TestBones) {
        if (ActiveBlends.Contains(BoneName)) {
            ForceBlendOutPermanent(BoneName, BlendOutDuration);
            StoppedCount++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("🛑 Forcing blend-out for %d permanent physics over %.2fs"), StoppedCount,
           BlendOutDuration);
}

#pragma endregion

#pragma region ReversePACCalculation
FPhysicalAnimationData UOHPACManager::GetCurrentPhysicalAnimationProfile(FName BoneName) const {
    FPhysicalAnimationData ReversedProfile;

    // Try to find the PAC constraint for this bone
    FConstraintInstance* PACConstraint = FindPACConstraintForBone(BoneName);

    if (!PACConstraint) {
        // No PAC constraint found - bone might not be using PAC or constraint not created yet
        if (bVerboseLogging) {
            SafeLog(FString::Printf(TEXT("No PAC constraint found for bone: %s"), *BoneName.ToString()));
        }

        // Return zero profile to indicate no PAC active
        ReversedProfile.PositionStrength = 0.0f;
        ReversedProfile.VelocityStrength = 0.0f;
        ReversedProfile.OrientationStrength = 0.0f;
        ReversedProfile.AngularVelocityStrength = 0.0f;
        ReversedProfile.bIsLocalSimulation = true;

        return ReversedProfile;
    }

    // Extract drive parameters from the constraint
    float LinearStiffness, LinearDamping, AngularStiffness, AngularDamping;
    if (!GetConstraintDriveParametersFromInstance(PACConstraint, LinearStiffness, LinearDamping, AngularStiffness,
                                                  AngularDamping)) {
        // Failed to extract parameters, return baseline
        ReversedProfile.PositionStrength = 1000.0f;
        ReversedProfile.VelocityStrength = 100.0f;
        ReversedProfile.OrientationStrength = 15000.0f;
        ReversedProfile.AngularVelocityStrength = 1000.0f;
        ReversedProfile.bIsLocalSimulation = true;

        return ReversedProfile;
    }

    // Convert constraint drive parameters back to PAC profile values
    ReversedProfile =
        ConvertDriveParametersToPACProfile(BoneName, LinearStiffness, LinearDamping, AngularStiffness, AngularDamping);

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Verbose, TEXT("Reversed PAC profile for %s: Pos=%.1f, Vel=%.1f, Ori=%.1f, AngVel=%.1f"),
               *BoneName.ToString(), ReversedProfile.PositionStrength, ReversedProfile.VelocityStrength,
               ReversedProfile.OrientationStrength, ReversedProfile.AngularVelocityStrength);
    }

    return ReversedProfile;
}

FConstraintInstance* UOHPACManager::FindPACConstraintForBone(FName BoneName) const {
    if (!SkeletalMesh) {
        return nullptr;
    }

    // PAC creates constraints with the naming pattern "PhysicalAnimation_BoneName"
    const FString PACConstraintName = FString::Printf(TEXT("PhysicalAnimation_%s"), *BoneName.ToString());
    const FName ConstraintFName(*PACConstraintName);

    // Search through all constraint instances
    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(false, ConstraintAccessors);

    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint) {
            continue;
        }

        // Check if this is the PAC constraint for our bone
        if (Constraint->JointName == ConstraintFName ||
            Constraint->JointName.ToString().Equals(PACConstraintName, ESearchCase::IgnoreCase)) {
            return Constraint;
        }
    }

    // Alternative search by constraint bone names (sometimes PAC uses different naming)
    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint) {
            continue;
        }

        // Check if this constraint connects to our bone
        if (Constraint->ConstraintBone1 == BoneName || Constraint->ConstraintBone2 == BoneName) {
            // Verify it has drive parameters (indicating it's likely a PAC constraint)
            bool bHasLinearDrive = Constraint->ProfileInstance.LinearDrive.XDrive.Stiffness > KINDA_SMALL_NUMBER ||
                                   Constraint->ProfileInstance.LinearDrive.YDrive.Stiffness > KINDA_SMALL_NUMBER ||
                                   Constraint->ProfileInstance.LinearDrive.ZDrive.Stiffness > KINDA_SMALL_NUMBER;

            bool bHasAngularDrive =
                Constraint->ProfileInstance.AngularDrive.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER ||
                Constraint->ProfileInstance.AngularDrive.SwingDrive.Stiffness > KINDA_SMALL_NUMBER ||
                Constraint->ProfileInstance.AngularDrive.TwistDrive.Stiffness > KINDA_SMALL_NUMBER;

            if (bHasLinearDrive || bHasAngularDrive) {
                return Constraint;
            }
        }
    }

    return nullptr;
}

bool UOHPACManager::GetConstraintDriveParameters(FName BoneName, float& OutLinearStiffness, float& OutLinearDamping,
                                                 float& OutAngularStiffness, float& OutAngularDamping) const {
    FConstraintInstance* PACConstraint = FindPACConstraintForBone(BoneName);
    if (!PACConstraint) {
        return false;
    }

    return GetConstraintDriveParametersFromInstance(PACConstraint, OutLinearStiffness, OutLinearDamping,
                                                    OutAngularStiffness, OutAngularDamping);
}

bool UOHPACManager::GetConstraintDriveParametersFromInstance(FConstraintInstance* Constraint, float& OutLinearStiffness,
                                                             float& OutLinearDamping, float& OutAngularStiffness,
                                                             float& OutAngularDamping) {
    if (!Constraint) {
        return false;
    }

    // Extract linear drive parameters
    const auto& LinearDrive = Constraint->ProfileInstance.LinearDrive;

    // PAC typically uses the same stiffness/damping for all linear axes
    OutLinearStiffness =
        FMath::Max3(LinearDrive.XDrive.Stiffness, LinearDrive.YDrive.Stiffness, LinearDrive.ZDrive.Stiffness);

    OutLinearDamping = FMath::Max3(LinearDrive.XDrive.Damping, LinearDrive.YDrive.Damping, LinearDrive.ZDrive.Damping);

    // Extract angular drive parameters
    const auto& AngularDrive = Constraint->ProfileInstance.AngularDrive;

    // PAC typically uses either Slerp drive or Swing/Twist drives
    if (AngularDrive.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER) {
        // Using Slerp drive
        OutAngularStiffness = AngularDrive.SlerpDrive.Stiffness;
        OutAngularDamping = AngularDrive.SlerpDrive.Damping;
    } else {
        // Using Swing/Twist drives
        OutAngularStiffness = FMath::Max(AngularDrive.SwingDrive.Stiffness, AngularDrive.TwistDrive.Stiffness);

        OutAngularDamping = FMath::Max(AngularDrive.SwingDrive.Damping, AngularDrive.TwistDrive.Damping);
    }

    return true;
}

FPhysicalAnimationData UOHPACManager::ConvertDriveParametersToPACProfile(FName BoneName, float LinearStiffness,
                                                                         float LinearDamping, float AngularStiffness,
                                                                         float AngularDamping) const {
    FPhysicalAnimationData Profile;

    // Get bone properties for accurate conversion
    FOHBoneProperties BoneProps = AnalyzeBoneProperties(BoneName);

    // Direct mapping from constraint drive parameters to PAC profile
    // In PAC, these values are often used directly as constraint drive parameters
    Profile.PositionStrength = LinearStiffness;
    Profile.VelocityStrength = LinearDamping;
    Profile.OrientationStrength = AngularStiffness;
    Profile.AngularVelocityStrength = AngularDamping;
    Profile.bIsLocalSimulation = true;

    // Validate against expected ranges and adjust if necessary
    Profile.PositionStrength = FMath::Clamp(Profile.PositionStrength, 0.0f, 50000.0f);
    Profile.VelocityStrength = FMath::Clamp(Profile.VelocityStrength, 0.0f, 5000.0f);
    Profile.OrientationStrength = FMath::Clamp(Profile.OrientationStrength, 0.0f, 500000.0f);
    Profile.AngularVelocityStrength = FMath::Clamp(Profile.AngularVelocityStrength, 0.0f, 50000.0f);

    // Validate critical damping ratios and log if they seem off
    if (BoneProps.Mass > KINDA_SMALL_NUMBER && BoneProps.MomentOfInertia > KINDA_SMALL_NUMBER) {
        float ExpectedVelStrength = 2.0f * FMath::Sqrt(Profile.PositionStrength * BoneProps.Mass);
        float ExpectedAngVelStrength = 2.0f * FMath::Sqrt(Profile.OrientationStrength * BoneProps.MomentOfInertia);

        float VelRatio =
            (ExpectedVelStrength > KINDA_SMALL_NUMBER) ? Profile.VelocityStrength / ExpectedVelStrength : 0.0f;
        float AngVelRatio = (ExpectedAngVelStrength > KINDA_SMALL_NUMBER)
                                ? Profile.AngularVelocityStrength / ExpectedAngVelStrength
                                : 0.0f;

        if (bVerboseLogging && (FMath::Abs(VelRatio - 1.0f) > 0.3f || FMath::Abs(AngVelRatio - 1.0f) > 0.3f)) {
            UE_LOG(LogTemp, Warning,
                   TEXT("Reversed profile for %s may not follow critical damping: VelRatio=%.2f, AngVelRatio=%.2f"),
                   *BoneName.ToString(), VelRatio, AngVelRatio);
        }
    }

    return Profile;
}

void UOHPACManager::AnalyzeCurrentPACConstraints() {
    UE_LOG(LogTemp, Warning, TEXT("=== Current PAC Constraint Analysis ==="));

    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Error, TEXT("No SkeletalMeshComponent available"));
        return;
    }

    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(false, ConstraintAccessors);

    int32 PACConstraintCount = 0;
    int32 ActiveConstraintCount = 0;

    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint) {
            continue;
        }

        FString ConstraintName = Constraint->JointName.ToString();

        // Check if this looks like a PAC constraint

        if (ConstraintName.StartsWith(TEXT("PhysicalAnimation_"))) {
            PACConstraintCount++;

            // Extract drive parameters
            float LinearStiffness, LinearDamping, AngularStiffness, AngularDamping;
            if (GetConstraintDriveParametersFromInstance(Constraint, LinearStiffness, LinearDamping, AngularStiffness,
                                                         AngularDamping)) {
                // Check if constraint is actively driving
                bool bDriveIsActive = LinearStiffness > KINDA_SMALL_NUMBER || AngularStiffness > KINDA_SMALL_NUMBER;
                if (bDriveIsActive) {
                    ActiveConstraintCount++;
                }

                // Extract bone name from constraint name
                FString BoneNameStr = ConstraintName.RightChop(18); // Remove "PhysicalAnimation_"
                FName BoneName(*BoneNameStr);

                // Get the reversed profile
                FPhysicalAnimationData ReversedProfile = ConvertDriveParametersToPACProfile(
                    BoneName, LinearStiffness, LinearDamping, AngularStiffness, AngularDamping);

                UE_LOG(LogTemp, Log, TEXT("%s%s:"), bDriveIsActive ? TEXT("✅") : TEXT("💤"), *BoneName.ToString());
                UE_LOG(LogTemp, Log, TEXT("  Constraint: Lin=%.1f/%.1f, Ang=%.1f/%.1f"), LinearStiffness, LinearDamping,
                       AngularStiffness, AngularDamping);
                UE_LOG(LogTemp, Log, TEXT("  PAC Profile: Pos=%.1f, Vel=%.1f, Ori=%.1f, AngVel=%.1f"),
                       ReversedProfile.PositionStrength, ReversedProfile.VelocityStrength,
                       ReversedProfile.OrientationStrength, ReversedProfile.AngularVelocityStrength);

                // Compare with bone simulation state
                bool bBoneSimulating = IsBoneSimulating(BoneName);
                float BlendAlpha = GetBlendAlpha(BoneName);

                if (bDriveIsActive != bBoneSimulating) {
                    UE_LOG(LogTemp, Warning, TEXT("  ⚠️ Constraint active (%s) != bone simulating (%s)"),
                           bDriveIsActive ? TEXT("Yes") : TEXT("No"), bBoneSimulating ? TEXT("Yes") : TEXT("No"));
                }

                UE_LOG(LogTemp, Log, TEXT("  Simulation: Active=%s, BlendAlpha=%.2f"),
                       bBoneSimulating ? TEXT("Yes") : TEXT("No"), BlendAlpha);
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("Summary: %d total constraints, %d PAC constraints, %d active PAC"),
           ConstraintAccessors.Num(), PACConstraintCount, ActiveConstraintCount);

    // Compare with our tracking
    int32 TrackedSimulating = 0;
    for (const auto& RefPair : BoneSimulationRefCount) {
        if (RefPair.Value > 0) {
            TrackedSimulating++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("Our tracking: %d bones with ref count > 0, %d active blends"), TrackedSimulating,
           ActiveBlends.Num());
}

FOHConstraintDriveData UOHPACManager::ExtractConstraintDriveData(const FConstraintInstance* Constraint) {
    FOHConstraintDriveData DriveData;

    if (!Constraint) {
        return DriveData;
    }

    const auto& LinearDrive = Constraint->ProfileInstance.LinearDrive;
    const auto& AngularDrive = Constraint->ProfileInstance.AngularDrive;

    // Extract linear drive values
    DriveData.LinearStiffnessX = LinearDrive.XDrive.Stiffness;
    DriveData.LinearStiffnessY = LinearDrive.YDrive.Stiffness;
    DriveData.LinearStiffnessZ = LinearDrive.ZDrive.Stiffness;

    DriveData.LinearDampingX = LinearDrive.XDrive.Damping;
    DriveData.LinearDampingY = LinearDrive.YDrive.Damping;
    DriveData.LinearDampingZ = LinearDrive.ZDrive.Damping;

    // Extract angular drive values
    DriveData.AngularStiffnessSlerp = AngularDrive.SlerpDrive.Stiffness;
    DriveData.AngularStiffnessSwing = AngularDrive.SwingDrive.Stiffness;
    DriveData.AngularStiffnessTwist = AngularDrive.TwistDrive.Stiffness;

    DriveData.AngularDampingSlerp = AngularDrive.SlerpDrive.Damping;
    DriveData.AngularDampingSwing = AngularDrive.SwingDrive.Damping;
    DriveData.AngularDampingTwist = AngularDrive.TwistDrive.Damping;

    // Extract force limits
    DriveData.LinearForceLimit = LinearDrive.XDrive.MaxForce; // Assuming same for all axes
    DriveData.AngularForceLimit = AngularDrive.SlerpDrive.MaxForce;

    // Check if drives are enabled
    DriveData.bLinearXDriveEnabled = LinearDrive.XDrive.bEnablePositionDrive || LinearDrive.XDrive.bEnableVelocityDrive;
    DriveData.bLinearYDriveEnabled = LinearDrive.YDrive.bEnablePositionDrive || LinearDrive.YDrive.bEnableVelocityDrive;
    DriveData.bLinearZDriveEnabled = LinearDrive.ZDrive.bEnablePositionDrive || LinearDrive.ZDrive.bEnableVelocityDrive;

    DriveData.bAngularSlerpDriveEnabled =
        AngularDrive.SlerpDrive.bEnablePositionDrive || AngularDrive.SlerpDrive.bEnableVelocityDrive;
    DriveData.bAngularSwingDriveEnabled =
        AngularDrive.SwingDrive.bEnablePositionDrive || AngularDrive.SwingDrive.bEnableVelocityDrive;
    DriveData.bAngularTwistDriveEnabled =
        AngularDrive.TwistDrive.bEnablePositionDrive || AngularDrive.TwistDrive.bEnableVelocityDrive;

    return DriveData;
}
// ============================================================================
// PROFILE COMPARISON AND VALIDATION
// ============================================================================

void UOHPACManager::CompareCalculatedVsActualProfiles() {
    UE_LOG(LogTemp, Warning, TEXT("=== Calculated vs Actual Profile Comparison ==="));

    for (const FName& BoneName : SimulatableBones) {
        if (!IsBoneSimulating(BoneName)) {
            continue; // Skip non-simulating bones
        }

        // Get what we think the profile should be
        FPhysicalAnimationData CalculatedProfile =
            CalculateChainAwareProfile(BoneName, EOHProfileIntensity::Medium, true);

        // Get what the actual runtime profile is
        FPhysicalAnimationData ActualProfile = GetCurrentPhysicalAnimationProfile(BoneName);

        // Compare them
        float PosDiff = FMath::Abs(CalculatedProfile.PositionStrength - ActualProfile.PositionStrength);
        float VelDiff = FMath::Abs(CalculatedProfile.VelocityStrength - ActualProfile.VelocityStrength);
        float OriDiff = FMath::Abs(CalculatedProfile.OrientationStrength - ActualProfile.OrientationStrength);
        float AngVelDiff =
            FMath::Abs(CalculatedProfile.AngularVelocityStrength - ActualProfile.AngularVelocityStrength);

        // Calculate percentage differences
        float PosPercent =
            (CalculatedProfile.PositionStrength > 0) ? (PosDiff / CalculatedProfile.PositionStrength) * 100.0f : 0.0f;
        float VelPercent =
            (CalculatedProfile.VelocityStrength > 0) ? (VelDiff / CalculatedProfile.VelocityStrength) * 100.0f : 0.0f;
        float OriPercent = (CalculatedProfile.OrientationStrength > 0)
                               ? (OriDiff / CalculatedProfile.OrientationStrength) * 100.0f
                               : 0.0f;
        float AngVelPercent = (CalculatedProfile.AngularVelocityStrength > 0)
                                  ? (AngVelDiff / CalculatedProfile.AngularVelocityStrength) * 100.0f
                                  : 0.0f;

        // Determine if there's a significant difference
        bool bSignificantDifference =
            PosPercent > 20.0f || VelPercent > 20.0f || OriPercent > 20.0f || AngVelPercent > 20.0f;

        FString StatusIcon = bSignificantDifference ? TEXT("⚠️") : TEXT("✅");

        UE_LOG(LogTemp, Log, TEXT("%s %s:"), *StatusIcon, *BoneName.ToString());
        UE_LOG(LogTemp, Log, TEXT("  Calculated: Pos=%.1f, Vel=%.1f, Ori=%.1f, AngVel=%.1f"),
               CalculatedProfile.PositionStrength, CalculatedProfile.VelocityStrength,
               CalculatedProfile.OrientationStrength, CalculatedProfile.AngularVelocityStrength);
        UE_LOG(LogTemp, Log, TEXT("  Actual:     Pos=%.1f, Vel=%.1f, Ori=%.1f, AngVel=%.1f"),
               ActualProfile.PositionStrength, ActualProfile.VelocityStrength, ActualProfile.OrientationStrength,
               ActualProfile.AngularVelocityStrength);

        if (bSignificantDifference) {
            UE_LOG(LogTemp, Warning, TEXT("  Differences: Pos=%.1f%%, Vel=%.1f%%, Ori=%.1f%%, AngVel=%.1f%%"),
                   PosPercent, VelPercent, OriPercent, AngVelPercent);
        }
    }
}

#pragma endregion

FPhysicalAnimationData UOHPACManager::GetBaselineProfile() {
    FPhysicalAnimationData BaseProfile;
    BaseProfile.PositionStrength = 1000.0f;
    BaseProfile.OrientationStrength = 15000.0f;
    BaseProfile.VelocityStrength = 100.0f;
    BaseProfile.AngularVelocityStrength = 1000.0f;
    BaseProfile.bIsLocalSimulation = true;
    return BaseProfile;
}

float UOHPACManager::CalculatePercentageDifference(float ValueA, float ValueB) {
    if (FMath::Abs(ValueB) < KINDA_SMALL_NUMBER) {
        return ValueA > KINDA_SMALL_NUMBER ? 100.0f : 0.0f;
    }

    return ((ValueA - ValueB) / ValueB) * 100.0f;
}

FPhysicalAnimationData UOHPACManager::ReversePACProfileFromDrives(const FOHConstraintDriveData& DriveData,
                                                                  FName BoneName) const {
    FPhysicalAnimationData ReversedProfile;

    // Reverse-engineer PositionStrength from linear drive stiffness
    // PAC typically applies the same stiffness to all linear axes
    float AvgLinearStiffness = 0.0f;
    int32 ActiveLinearAxes = 0;

    if (DriveData.bLinearXDriveEnabled && DriveData.LinearStiffnessX > KINDA_SMALL_NUMBER) {
        AvgLinearStiffness += DriveData.LinearStiffnessX;
        ActiveLinearAxes++;
    }
    if (DriveData.bLinearYDriveEnabled && DriveData.LinearStiffnessY > KINDA_SMALL_NUMBER) {
        AvgLinearStiffness += DriveData.LinearStiffnessY;
        ActiveLinearAxes++;
    }
    if (DriveData.bLinearZDriveEnabled && DriveData.LinearStiffnessZ > KINDA_SMALL_NUMBER) {
        AvgLinearStiffness += DriveData.LinearStiffnessZ;
        ActiveLinearAxes++;
    }

    ReversedProfile.PositionStrength = ActiveLinearAxes > 0 ? (AvgLinearStiffness / ActiveLinearAxes) : 0.0f;

    // Reverse-engineer VelocityStrength from linear drive damping
    float AvgLinearDamping = 0.0f;
    int32 ActiveLinearDampingAxes = 0;

    if (DriveData.bLinearXDriveEnabled && DriveData.LinearDampingX > KINDA_SMALL_NUMBER) {
        AvgLinearDamping += DriveData.LinearDampingX;
        ActiveLinearDampingAxes++;
    }
    if (DriveData.bLinearYDriveEnabled && DriveData.LinearDampingY > KINDA_SMALL_NUMBER) {
        AvgLinearDamping += DriveData.LinearDampingY;
        ActiveLinearDampingAxes++;
    }
    if (DriveData.bLinearZDriveEnabled && DriveData.LinearDampingZ > KINDA_SMALL_NUMBER) {
        AvgLinearDamping += DriveData.LinearDampingZ;
        ActiveLinearDampingAxes++;
    }

    ReversedProfile.VelocityStrength =
        ActiveLinearDampingAxes > 0 ? (AvgLinearDamping / ActiveLinearDampingAxes) : 0.0f;

    // Reverse-engineer OrientationStrength from angular drive stiffness
    float AvgAngularStiffness = 0.0f;
    int32 ActiveAngularAxes = 0;

    if (DriveData.bAngularSlerpDriveEnabled && DriveData.AngularStiffnessSlerp > KINDA_SMALL_NUMBER) {
        AvgAngularStiffness += DriveData.AngularStiffnessSlerp;
        ActiveAngularAxes++;
    }
    if (DriveData.bAngularSwingDriveEnabled && DriveData.AngularStiffnessSwing > KINDA_SMALL_NUMBER) {
        AvgAngularStiffness += DriveData.AngularStiffnessSwing;
        ActiveAngularAxes++;
    }
    if (DriveData.bAngularTwistDriveEnabled && DriveData.AngularStiffnessTwist > KINDA_SMALL_NUMBER) {
        AvgAngularStiffness += DriveData.AngularStiffnessTwist;
        ActiveAngularAxes++;
    }

    ReversedProfile.OrientationStrength = ActiveAngularAxes > 0 ? (AvgAngularStiffness / ActiveAngularAxes) : 0.0f;

    // Reverse-engineer AngularVelocityStrength from angular drive damping
    float AvgAngularDamping = 0.0f;
    int32 ActiveAngularDampingAxes = 0;

    if (DriveData.bAngularSlerpDriveEnabled && DriveData.AngularDampingSlerp > KINDA_SMALL_NUMBER) {
        AvgAngularDamping += DriveData.AngularDampingSlerp;
        ActiveAngularDampingAxes++;
    }
    if (DriveData.bAngularSwingDriveEnabled && DriveData.AngularDampingSwing > KINDA_SMALL_NUMBER) {
        AvgAngularDamping += DriveData.AngularDampingSwing;
        ActiveAngularDampingAxes++;
    }
    if (DriveData.bAngularTwistDriveEnabled && DriveData.AngularDampingTwist > KINDA_SMALL_NUMBER) {
        AvgAngularDamping += DriveData.AngularDampingTwist;
        ActiveAngularDampingAxes++;
    }

    ReversedProfile.AngularVelocityStrength =
        ActiveAngularDampingAxes > 0 ? (AvgAngularDamping / ActiveAngularDampingAxes) : 0.0f;

    // Set other profile properties based on constraint data
    ReversedProfile.bIsLocalSimulation = true; // PAC typically uses local simulation

    // Validate the reversed profile
    if (ReversedProfile.PositionStrength < KINDA_SMALL_NUMBER &&
        ReversedProfile.OrientationStrength < KINDA_SMALL_NUMBER) {
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("Reversed profile for %s has no significant drives, using baseline"),
                   *BoneName.ToString());
        }
        return GetBaselineProfile();
    }

    return ReversedProfile;
}
void UOHPACManager::TestCustomAlphaBlends(EOHBoneType BoneType) {
    UE_LOG(LogTemp, Warning, TEXT("=== Testing Custom Alpha Blends ==="));

    TArray<FName> TestBones = GetBonesByType(BoneType);
    if (TestBones.Num() == 0) {
        UE_LOG(LogTemp, Error, TEXT("No test bones available"));
        return;
    }

    // Test different alpha modes
    TArray<EOHTargetAlphaMode> TestModes = {EOHTargetAlphaMode::Subtle, EOHTargetAlphaMode::Secondary,
                                            EOHTargetAlphaMode::IdlePose, EOHTargetAlphaMode::Full};

    for (int32 i = 0; i < FMath::Min(TestBones.Num(), TestModes.Num()); ++i) {
        const FName& BoneName = TestBones[i];
        EOHTargetAlphaMode Mode = TestModes[i];

        FPhysicalAnimationData Profile = CalculateChainAwareProfile(BoneName, EOHProfileIntensity::Medium, true);
        float TargetAlpha = GetTargetAlphaForMode(Mode);
        float ScaledAlpha = ApplyBoneAlphaScaling(BoneName, TargetAlpha);

        bool bSuccess = StartPhysicsBlendWithMode(BoneName, Profile, Mode, 0.5f, 5.0f, 0.8f, FName("AlphaTest"));

        UE_LOG(LogTemp, Warning, TEXT("Bone %s: Mode=%s, Target=%.2f, Scaled=%.2f, Success=%s"), *BoneName.ToString(),
               *UEnum::GetValueAsString(Mode), TargetAlpha, ScaledAlpha, bSuccess ? TEXT("Yes") : TEXT("No"));
    }

    // Log the effective blend alphas
    FTimerHandle LogTimer;
    GetWorld()->GetTimerManager().SetTimer(
        LogTimer,
        [this, TestBones]() {
            UE_LOG(LogTemp, Warning, TEXT("--- Alpha Test Results ---"));
            for (const FName& BoneName : TestBones) {
                float EffectiveAlpha = GetBlendAlpha(BoneName);
                bool bSimulating = IsBoneSimulating(BoneName);

                if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
                    float ActualWeight = Body->PhysicsBlendWeight;
                    UE_LOG(LogTemp, Log, TEXT("%s: Effective=%.2f, Actual=%.2f, Sim=%s"), *BoneName.ToString(),
                           EffectiveAlpha, ActualWeight, bSimulating ? TEXT("Yes") : TEXT("No"));
                }
            }
        },
        1.0f, false);
}

void UOHPACManager::TestSubtlePhysicsMode(EOHBoneType BoneType) {
    UE_LOG(LogTemp, Warning, TEXT("=== Testing Subtle Physics Mode ==="));
    TArray<FName> BonesToUse = GetBonesByType(BoneType);

    for (const FName& BoneName : BonesToUse) {
        if (!IsBoneValidForSimulation(BoneName)) {
            continue;
        }

        FPhysicalAnimationData Profile = CalculateChainAwareProfile(BoneName, EOHProfileIntensity::Light, true);

        // Use subtle physics mode for secondary motion effects
        StartPermanentPhysicsBlendWithAlpha(BoneName, Profile, SubtlePhysicsAlpha, 1.0f, FName("SubtleTest"));
    }

    UE_LOG(LogTemp, Warning, TEXT("✅ Applied subtle physics to %d bones (Alpha=%.2f)"), UpperBodyBones.Num(),
           SubtlePhysicsAlpha);
}

TArray<FName> UOHPACManager::GetBonesByType(EOHBoneType BoneType) const {
    switch (BoneType) {
    case EOHBoneType::UpperBodyBones:
        return UpperBodyBones;
    case EOHBoneType::LowerBodyBones:
        return LowerBodyBones;
    case EOHBoneType::SpineBones:
        return SpineBones;
    case EOHBoneType::HeadBones:
        return HeadBones;
    case EOHBoneType::LeftArmBones:
        return LeftArmBones;
    case EOHBoneType::RightArmBones:
        return RightArmBones;
    case EOHBoneType::ArmBones:
        return ArmBones;
    case EOHBoneType::ClavicleBones:
        return ClavicleBones;
    case EOHBoneType::UpperArmBones:
        return UpperArmBones;
    case EOHBoneType::LowerArmBones:
        return LowerArmBones;
    case EOHBoneType::HandBones:
        return HandBones;
    case EOHBoneType::LeftLegBones:
        return LeftLegBones;
    case EOHBoneType::RightLegBones:
        return RightLegBones;
    case EOHBoneType::LegBones:
        return LegBones;
    case EOHBoneType::ThighBones:
        return ThighBones;
    case EOHBoneType::CalfBones:
        return CalfBones;
    case EOHBoneType::FootBones:
        return FootBones;
    default:
        return TArray<FName>();
    }
}
