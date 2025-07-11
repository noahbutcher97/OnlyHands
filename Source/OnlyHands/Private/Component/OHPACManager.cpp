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
    if (MotionHistory.Num() == 0)
        return LinearVelocity;

    const int32 Count = FMath::Min(SampleCount, MotionHistory.Num());
    FVector Sum = FVector::ZeroVector;

    for (int32 i = MotionHistory.Num() - Count; i < MotionHistory.Num(); ++i) {
        Sum += MotionHistory[i].GetLinearVelocity();
    }

    return Sum / Count;
}

float FOHBoneMotionData::GetInstabilityScore() const {
    if (MotionHistory.Num() < 3)
        return 0.f;

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
    if (!ConstraintInstance)
        return;

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
    if (!ConstraintInstance)
        return 0.f;

    const float Swing1 = FMath::Abs(ConstraintInstance->GetCurrentSwing1());
    const float Swing2 = FMath::Abs(ConstraintInstance->GetCurrentSwing2());
    const float Swing1Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing1LimitDegrees;
    const float Swing2Limit = ConstraintInstance->ProfileInstance.ConeLimit.Swing2LimitDegrees;

    return FMath::Max(Swing1Limit > 0.f ? Swing1 / Swing1Limit : 0.f, Swing2Limit > 0.f ? Swing2 / Swing2Limit : 0.f);
}

float FOHConstraintData::GetTwistStrain() const {
    if (!ConstraintInstance)
        return 0.f;

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

    if (!bEnablePACManager)
        return;

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

    if (!bEnablePACManager || !SkeletalMesh || !bIsInitialized)
        return;

    // Update motion tracking
    UpdateMotionTracking(DeltaTime);

    // Update constraint states
    UpdateConstraintStates(DeltaTime);

    // Process active blends
    ProcessActiveBlends(DeltaTime);

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    if (bDrawDebug) {
        // DrawDebugOverlay();
        // VisualizeActivePhysicalAnimationDrives();
        // CheckSimulatingBonesForPhysicalAnimationDrives();
        // DebugBodyInstanceSimulation();
        // DebugPhysicalAnimationConstraints();
        DebugBodyPhysicsStates();
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
    if (!SkeletalMesh)
        return;

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
    if (!SkeletalMesh)
        return;

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
    if (!SkeletalMesh)
        return;

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
    if (!SkeletalMesh || !PhysicalAnimationComponent)
        return;

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

    // Full system re-init (this handles event bindings, cache rebuild, physics setup, etc.)
    InitializePACManager();
}

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

    // Bind to mesh change events
    if (SkeletalMesh && !SkeletalMesh->OnSkeletalMeshPropertyChanged.IsBoundToObject(this)) {
        SkeletalMesh->OnSkeletalMeshPropertyChanged.AddUObject(this, &UOHPACManager::OnSkeletalMeshChanged);
        SafeLog(TEXT("Bound to OnSkeletalMeshPropertyChanged event."));
    }

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

    if (!SkeletalMesh)
        return;

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

    if (!CachedPhysicsAsset)
        return;

    for (const UPhysicsConstraintTemplate* Template : CachedPhysicsAsset->ConstraintSetup) {
        if (!Template)
            continue;

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
        if (!SkeletalMesh->DoesSocketExist(BoneName))
            continue;

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
        if (SimulationExclusions.Contains(BoneName))
            continue;

        // 1. Pattern filter: skip known problematic or helper bones
        if (!IsBoneNamePatternValid(BoneName))
            continue;

        // 2. Valid body instance check
        if (!HasPhysicsBody(BoneName))
            continue;

        // 3. Mass check: skip bones with near-zero mass
        if (!IsBoneMassValid(BoneName))
            continue;

        // All checks passed, include for simulation!
        SimulatableBones.Add(BoneName);
    }

    SafeLog(FString::Printf(TEXT("Determined %d simulatable bones from %d tracked bones"), SimulatableBones.Num(),
                            TrackedBones.Num()));
}

bool UOHPACManager::ArePhysicsBodiesReady() const {
    if (!SkeletalMesh || !SkeletalMesh->GetPhysicsAsset())
        return false;

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

        if (!SkeletalMesh->DoesSocketExist(BoneName))
            continue;

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
void UOHPACManager::ProcessActiveBlends(float DeltaTime) {
    TArray<TPair<FName, int32>> CompletedBlends;

    for (auto& Pair : ActiveBlends) {
        const FName& BoneName = Pair.Key;
        TArray<FOHBlendState>& BoneBlends = Pair.Value;

        // Process each blend for this bone
        for (int32 i = BoneBlends.Num() - 1; i >= 0; --i) {
            FOHBlendState& Blend = BoneBlends[i];
            if (Blend.IsPaused())
                continue;

            UpdateBlendState(Blend, DeltaTime);

            // Find the maximum alpha from all active blends
            float MaxAlpha = 0.f;
            for (const FOHBlendState& BlendCheck : BoneBlends) {
                if (!BlendCheck.IsPaused()) {
                    MaxAlpha = FMath::Max(MaxAlpha, BlendCheck.BlendAlpha);
                }
            }

            // Apply blend to entire chain
            TArray<FName> Chain = GetBoneChain(BoneName, 0);
            for (const FName& ChainBone : Chain) {
                ApplyBlendAlpha(ChainBone, MaxAlpha);

                if (bVerboseLogging) {
                    if (FBodyInstance* Body = GetBodyInstanceDirect(ChainBone)) {
                        UE_LOG(LogTemp, Verbose, TEXT("[PAC] %s blend weight = %.2f, sim=%d"), *ChainBone.ToString(),
                               Body->PhysicsBlendWeight, Body->IsInstanceSimulatingPhysics());
                    }
                }
            }

            // Mark completed blends
            if (Blend.IsComplete()) {
                CompletedBlends.Add(TPair<FName, int32>(BoneName, Blend.BlendID));
            }
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

    // Find bones with zero ref count but still have blend weight
    for (const auto& Pair : BoneSimulationRefCount) {
        FName BoneName = Pair.Key;
        int32 RefCount = Pair.Value;

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

    // Remove stale entries
    for (const FName& BoneName : StaleBones) {
        BoneSimulationRefCount.Remove(BoneName);
        ClearPhysicalAnimationProfile(BoneName);
    }

    // Also check for orphaned bodies (simulating but not in ref count)
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

void UOHPACManager::UpdateBlendState(FOHBlendState& Blend, float DeltaTime) {
    Blend.ElapsedTime += DeltaTime;

    switch (Blend.Phase) {
    case EOHBlendPhase::BlendIn:
        if (Blend.BlendInDuration > 0.f) {
            Blend.BlendAlpha = FMath::Clamp(Blend.ElapsedTime / Blend.BlendInDuration, 0.f, 1.f);
            if (Blend.ElapsedTime >= Blend.BlendInDuration) {
                Blend.Phase = EOHBlendPhase::Hold;
                Blend.ElapsedTime = 0.f;
            }
        } else {
            Blend.BlendAlpha = 1.f;
            Blend.Phase = EOHBlendPhase::Hold;
        }
        break;

    case EOHBlendPhase::Hold:
        Blend.BlendAlpha = 1.f;
        if (Blend.ElapsedTime >= Blend.HoldDuration) {
            Blend.Phase = EOHBlendPhase::BlendOut;
            Blend.ElapsedTime = 0.f;
        }
        break;

    case EOHBlendPhase::BlendOut:
        if (Blend.BlendOutDuration > 0.f) {
            Blend.BlendAlpha = 1.f - FMath::Clamp(Blend.ElapsedTime / Blend.BlendOutDuration, 0.f, 1.f);
        } else {
            Blend.BlendAlpha = 0.f;
        }
        break;
    }
}

void UOHPACManager::ApplyBlendAlpha(FName BoneName, float Alpha) {
    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        Body->PhysicsBlendWeight = Alpha;

        // Ensure the body is awakened when blend weight > 0
        if (Alpha > 0.f && Body->IsInstanceSimulatingPhysics()) {
            Body->WakeInstance();
        }
    }
}

void UOHPACManager::FinalizeBlend(FName BoneName) {
    // Remove blend state
    ActiveBlends.Remove(BoneName);

    // Decrement simulation reference count
    if (int32* RefCount = BoneSimulationRefCount.Find(BoneName)) {
        (*RefCount)--;
        if (*RefCount <= 0) {
            BoneSimulationRefCount.Remove(BoneName);
            StopChainPhysicalAnimation(BoneName, true);
        }
    }

    OnHitReactionComplete.Broadcast(BoneName, TArray<FName>());
}

void UOHPACManager::FinalizeBlendByID(FName BoneName, int32 BlendID) {
    TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends)
        return;

    // Remove the specific blend by ID
    for (int32 i = BoneBlends->Num() - 1; i >= 0; --i) {
        if ((*BoneBlends)[i].BlendID == BlendID) {
            BoneBlends->RemoveAt(i);
            break;
        }
    }

    // For every bone in the affected chain
    TArray<FName> Chain = GetBoneChain(BoneName, 0);

    for (const FName& ChainBone : Chain) {
        int32& RefCount = BoneSimulationRefCount.FindOrAdd(ChainBone);
        RefCount = FMath::Max(RefCount - 1, 0);

        SafeLog(FString::Printf(TEXT("Finalized blend ID %d for bone %s (remaining ref count: %d)"), BlendID,
                                *ChainBone.ToString(), RefCount));

        if (RefCount == 0) {
            ClearPhysicsStateForBone(ChainBone);
            BoneSimulationRefCount.Remove(ChainBone);
            OnBoneStoppedSimulating.Broadcast(ChainBone);
        }
    }

    OnHitReactionComplete.Broadcast(BoneName, Chain);
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

    // Ensure the PAC is always bound to the correct mesh
    if (!IsSkeletalMeshBindingValid(true, bVerboseLogging)) {
        SafeLog(TEXT("Aborting hit reaction: SkeletalMesh binding is invalid and could not be auto-fixed!"), true);
        return;
    }

    // Ensure physics state is valid
    EnsurePhysicsStateValid();

    // Enable physics simulation on bone/chain
    EnsureBoneSimulatingPhysics(BoneName, true);

    // Start simulation with profile ONLY if not already simulating
    if (!IsBoneSimulating(BoneName)) {
        if (!StartChainPhysicalAnimation_Filtered(BoneName, CustomProfile,
                                                  true, // bUseNativePropagation
                                                  true, // bEnableCollision
                                                  [this](FName Bone) {
                                                      return SimulatableBones.Contains(Bone) &&
                                                             !SimulationExclusions.Contains(Bone);
                                                  })) {
            SafeLog(FString::Printf(TEXT("Failed to start simulation for bone: %s"), *BoneName.ToString()), true);
            return;
        }
    } else {
        // If already simulating, just update the profile
        ApplyPhysicalAnimationProfile(BoneName, CustomProfile);
    }

    // Preserve velocity as an impulse for smoother transitions
    ApplyPoseInertia(BoneName);

    // Create blend state with unique ID
    FOHBlendState BlendState;
    BlendState.BlendID = NextBlendID++;
    BlendState.RootBone = BoneName;
    BlendState.BlendInDuration = BlendIn;
    BlendState.HoldDuration = Hold;
    BlendState.BlendOutDuration = BlendOut;
    BlendState.ReactionTag = ReactionTag;
    const float CurrentAlpha = FMath::Clamp(GetBlendAlpha(BoneName), 0.f, 1.f);
    BlendState.BlendAlpha = CurrentAlpha;
    BlendState.Phase = EOHBlendPhase::BlendIn;
    BlendState.ElapsedTime = CurrentAlpha * BlendIn;

    // Add to blend array for this bone
    TArray<FOHBlendState>& BoneBlends = ActiveBlends.FindOrAdd(BoneName);
    BoneBlends.Add(BlendState);

    // Increment simulation reference count for entire chain
    TArray<FName> Chain = GetBoneChain(BoneName, 0);
    for (const FName& ChainBone : Chain) {
        BoneSimulationRefCount.FindOrAdd(ChainBone)++;
    }

    // Apply impulse if specified
    if (!ImpulseDirection.IsNearlyZero() && ImpulseStrength > 0.f) {
        ApplyImpulse(BoneName, ImpulseDirection, ImpulseStrength);
    }

    OnBoneStartedSimulating.Broadcast(BoneName);

    SafeLog(FString::Printf(TEXT("Started hit reaction (ID: %d) on bone: %s (total reactions: %d)"), BlendState.BlendID,
                            *BoneName.ToString(), BoneBlends.Num()));
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
    const float Attenuation = 0.7f;

    for (const FName& BoneName : Chain) {
        ApplyImpulse(BoneName, Direction, CurrentMagnitude);
        CurrentMagnitude *= Attenuation;
    }
}

void UOHPACManager::ApplyPoseInertia(FName BoneName, float Scale) {
    if (const FOHBoneMotionData* Data = BoneMotionMap.Find(BoneName)) {
        if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
            if (Body->IsInstanceSimulatingPhysics()) {
                const FVector Impulse = Data->GetLinearVelocity() * Body->GetBodyMass() * Scale;
                Body->AddImpulse(Impulse, true);
            }
        }
    }
}

void UOHPACManager::TuneConstraintStrength(FName BoneName, float StiffnessMul, float DampingMul) {
    if (FOHConstraintData* Data = ConstraintDataMap.Find(BoneName)) {
        if (FConstraintInstance* CI = Data->GetConstraintInstance()) {
            FConstraintProfileProperties& Profile = CI->ProfileInstance;
            Profile.ConeLimit.Stiffness *= StiffnessMul;
            Profile.ConeLimit.Damping *= DampingMul;
            Profile.TwistLimit.Stiffness *= StiffnessMul;
            Profile.TwistLimit.Damping *= DampingMul;

            CI->SetAngularDriveParams(Profile.ConeLimit.Stiffness, Profile.ConeLimit.Damping, 0.f);
            CI->UpdateAngularLimit();
        }
    }
}
#pragma endregion

// ============================================================================
// SIMULATION CONTROL
// ============================================================================
#pragma region SIMULATION CONTROL
bool UOHPACManager::StartSimulation(FName BoneName, const FPhysicalAnimationData& Profile, bool bAllBelow,
                                    bool bEnableCollision) {
    if (!IsBoneValidForSimulation(BoneName))
        return false;
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return false;

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
    } else {
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
    }
    return false;
}

void UOHPACManager::StopSimulation(FName BoneName, bool bAllBelow) {
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return;

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

bool UOHPACManager::TryActivateSimForBone(FName BoneName) {
    if (!SkeletalMesh || !IsBoneValidForSimulation(BoneName))
        return false;

    int32& RefCount = BoneSimulationRefCount.FindOrAdd(BoneName);
    RefCount = FMath::Clamp(RefCount + 1, 1, 999);

    if (RefCount == 1) {
        return ActivatePhysicsStateForBone(BoneName, 1.0f);
    }

    return false;
}

bool UOHPACManager::TryDeactivateSimForBone(FName BoneName) {
    int32* RefCountPtr = BoneSimulationRefCount.Find(BoneName);
    if (!RefCountPtr)
        return false;

    (*RefCountPtr)--;

    if (*RefCountPtr <= 0) {
        BoneSimulationRefCount.Remove(BoneName);
        ClearPhysicsStateForBone(BoneName);
        return true;
    }

    return false;
}

bool UOHPACManager::ActivatePhysicsStateForBone(FName BoneName, float BlendAlpha) {
    if (!SkeletalMesh)
        return false;

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
                                               bool bEnableCollision) {
    if (!IsBoneValidForSimulation(BoneName))
        return false;
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return false;

    FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
    if (!Body)
        return false;

    Body->SetInstanceSimulatePhysics(true);
    if (bEnableCollision)
        Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);
    Body->WakeInstance();

    return true;
}

void UOHPACManager::StopBonePhysicalAnimation(FName BoneName) {
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return;

    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, ZeroProfile);

    if (FBodyInstance* Body = GetBodyInstanceDirect(BoneName)) {
        Body->SetInstanceSimulatePhysics(false);
        Body->SetLinearVelocity(FVector::ZeroVector, false);
        Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        Body->PutInstanceToSleep();
    }
    OnBoneStoppedSimulating.Broadcast(BoneName);
}

// --- CHAIN ---

bool UOHPACManager::StartChainPhysicalAnimation(FName RootBone, const FPhysicalAnimationData& Profile,
                                                bool bUseNativePropagation, bool bEnableCollision) {
    // Default: no filtering
    return StartChainPhysicalAnimation_Filtered(RootBone, Profile, bUseNativePropagation, bEnableCollision,
                                                [](FName) { return true; });
}

bool UOHPACManager::StartChainPhysicalAnimation_Filtered(FName RootBone, const FPhysicalAnimationData& Profile,
                                                         bool bUseNativePropagation, bool bEnableCollision,
                                                         TFunctionRef<bool(FName)> BoneFilter) {
    if (!IsBoneValidForSimulation(RootBone))
        return false;
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return false;

    bool bAnySuccess = false;
    TArray<FName> Bones = GetBoneChain(RootBone, 0);

    if (bUseNativePropagation) {
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(RootBone, true, false);
        if (bEnableCollision) {
            for (const FName& Bone : Bones) {
                if (!BoneFilter(Bone))
                    continue;
                if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                    Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
                    Body->WakeInstance();
                }
            }
        }
        PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(RootBone, Profile, true);
        bAnySuccess = true;
    } else {
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone))
                continue;
            bAnySuccess |= StartBonePhysicalAnimation(Bone, Profile, bEnableCollision);
        }
    }
    return bAnySuccess;
}

void UOHPACManager::StopChainPhysicalAnimation(FName RootBone, bool bUseNativePropagation) {
    StopChainPhysicalAnimation_Filtered(RootBone, bUseNativePropagation, [](FName) { return true; });
}

void UOHPACManager::StopChainPhysicalAnimation_Filtered(FName RootBone, bool bUseNativePropagation,
                                                        TFunctionRef<bool(FName)> BoneFilter) {
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return;

    TArray<FName> Bones = GetBoneChain(RootBone, 0);

    if (bUseNativePropagation) {
        SkeletalMesh->SetAllBodiesBelowSimulatePhysics(RootBone, false, false);
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone))
                continue;

            PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, ZeroProfile);
            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                Body->SetLinearVelocity(FVector::ZeroVector, false);
                Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
                Body->PutInstanceToSleep();
            }
            OnBoneStoppedSimulating.Broadcast(Bone);
        }
    } else {
        for (const FName& Bone : Bones) {
            if (!BoneFilter(Bone))
                continue;
            StopBonePhysicalAnimation(Bone);
        }
    }
}

void UOHPACManager::EnsureBoneSimulatingPhysics(FName BoneName, bool bEnableChain) {
    if (!SkeletalMesh)
        return;

    if (bEnableChain) {
        // Enable simulating physics on this bone and all children (chain)
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
FBodyInstance* UOHPACManager::GetBodyInstanceDirect(FName BoneName) {
    if (FBodyInstance** Found = BodyInstanceCache.Find(BoneName)) {
        FBodyInstance* Body = *Found;
        // Validate cached pointer is still valid
        if (Body && Body->IsValidBodyInstance()) {
            return Body;
        } else {
            // Remove invalid cached entry
            BodyInstanceCache.Remove(BoneName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Removed invalid cached body for bone: %s"),
                   *BoneName.ToString());
        }
    }

    // Lazy cache if not found or was invalid
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

FORCEINLINE FConstraintInstance* UOHPACManager::GetConstraintInstanceDirect(FName BoneName) {
    if (FConstraintInstance** Found = ConstraintInstanceCache.Find(BoneName)) {
        return *Found;
    }

    return nullptr;
}

FORCEINLINE int32 UOHPACManager::GetBoneIndexDirect(FName BoneName) {
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

    // Fallback to skeletal mesh lookup if cache miss
    if (SkeletalMesh) {
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
            return Body->IsValidBodyInstance() && Body->IsInstanceSimulatingPhysics();
        }
    }

    return false;
}

float UOHPACManager::GetBlendAlpha(FName BoneName) const {
    const TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends || BoneBlends->Num() == 0)
        return 0.f;

    // Return the highest alpha from all active blends
    float MaxAlpha = 0.f;
    for (const FOHBlendState& Blend : *BoneBlends) {
        if (!Blend.IsPaused()) {
            MaxAlpha = FMath::Max(MaxAlpha, Blend.BlendAlpha);
        }
    }
    return MaxAlpha;
}

EOHBlendPhase UOHPACManager::GetBlendPhase(FName BoneName) const {
    const TArray<FOHBlendState>* BoneBlends = ActiveBlends.Find(BoneName);
    if (!BoneBlends || BoneBlends->Num() == 0)
        return EOHBlendPhase::BlendOut;

    // Return the phase of the most recent blend
    return (*BoneBlends)[BoneBlends->Num() - 1].Phase;
}

float UOHPACManager::GetConstraintStrain(FName BoneName) const {
    if (const FOHConstraintData* ConstraintData = ConstraintDataMap.Find(BoneName)) {
        return ConstraintData->GetCurrentStrain();
    }
    return 0.f;
}
#pragma endregion

// ============================================================================
// VALIDATION & UTILITY
// ============================================================================
#pragma region VALIDATION & UTILITY
bool UOHPACManager::IsBoneValidForSimulation(FName BoneName) const {
    // Bone must be in tracked set and not excluded
    if (!TrackedBones.Contains(BoneName))
        return false;
    if (SimulationExclusions.Contains(BoneName))
        return false;

    // Use cache if it's available (fast path)
    if (BodyInstanceCache.Contains(BoneName))
        return true;

    // If cache is not built, check the physics asset directly
    if (CachedPhysicsAsset) {
        int32 BodyIndex = CachedPhysicsAsset->FindBodyIndex(BoneName);
        return BodyIndex != INDEX_NONE;
    }
    return false;
}

bool UOHPACManager::IsBoneInChain(FName BoneName, FName RootBone) const {
    if (BoneName == RootBone)
        return true;

    FName CurrentBone = BoneName;
    while (const FName* Parent = BoneParentMap.Find(CurrentBone)) {
        if (*Parent == RootBone)
            return true;
        CurrentBone = *Parent;
    }

    return false;
}

bool UOHPACManager::IsBoneNamePatternValid(FName BoneName) {
    if (BoneName.IsNone())
        return false;

    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Disallowed substrings and prefixes/suffixes
    static const TArray<FString> DisallowedSubstrings = {TEXT("twist"), TEXT("ik"), TEXT("attach"), TEXT("helper")};
    static const TArray<FString> DisallowedPrefixes = {TEXT("vb_")};
    static const TArray<FString> DisallowedSuffixes = {TEXT("_dummy")};

    // Substring filters
    for (const FString& Disallowed : DisallowedSubstrings) {
        if (BoneNameStr.Contains(Disallowed))
            return false;
    }

    // Prefix filters
    for (const FString& Prefix : DisallowedPrefixes) {
        if (BoneNameStr.StartsWith(Prefix))
            return false;
    }

    // Suffix filters
    for (const FString& Suffix : DisallowedSuffixes) {
        if (BoneNameStr.EndsWith(Suffix))
            return false;
    }

    // (Optional) Regex: Only allow Unreal-ish names
    static const FRegexPattern BonePattern(TEXT("^[a-z_][a-z0-9_]*$"));
    FRegexMatcher Matcher(BonePattern, BoneNameStr);
    if (!Matcher.FindNext())
        return false;

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
    } else {
        SafeLog(FString::Printf(TEXT("IsBoneMassValid: Bone '%s' has zero or near-zero mass (%.3f)."),
                                *BoneName.ToString(), BoneMass),
                true);
        return false;
    }
}

bool UOHPACManager::HasPhysicsBody(const FName& BoneName) const {
    if (!SkeletalMesh) {
        SafeLog(TEXT("HasPhysicsBody: SkeletalMeshComponent is null."), true);
        return false;
    }

    const FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
    return (Body && Body->IsValidBodyInstance());
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
        } else {
            bUsingFallback = true;
            // Fallback: build from RefSkeleton
            static TArray<FName> TmpChildren; // Will be overwritten
            TmpChildren.Reset();

            if (!SkeletalMesh)
                return nullptr;
            const USkeletalMesh* MeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
            if (!MeshAsset)
                return nullptr;
            const FReferenceSkeleton& RefSkeleton = MeshAsset->GetRefSkeleton();
            int32 BoneIdx = RefSkeleton.FindBoneIndex(Bone);
            if (BoneIdx == INDEX_NONE)
                return nullptr;
            TArray<int32> ChildIndices;
            RefSkeleton.GetDirectChildBones(BoneIdx, ChildIndices);
            for (int32 ChildIdx : ChildIndices) {
                TmpChildren.Add(RefSkeleton.GetBoneName(ChildIdx));
            }
            return &TmpChildren;
        }
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

            if (MaxDepth == 0 || Depth < MaxDepth) {
                if (const TArray<FName>* Children = GetChildren(BoneName)) {
                    for (const FName& Child : *Children) {
                        BoneQueue.Enqueue(TPair<FName, int32>(Child, Depth + 1));
                    }
                }
            }
        }
    }

    if (bUsingFallback) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPACManager] BoneChildrenMap missing/corrupt, used RefSkeleton fallback in GetBoneChain! "
                    "Consider rebuilding BoneChildrenMap."));
    }

    return Chain;
}

TArray<FBodyInstance*> UOHPACManager::GetSimulatableBodies(USkeletalMeshComponent* SkeletalMesh,
                                                           const TSet<FName>& SimulatableBones) {
    TArray<FBodyInstance*> OutBodies;
    if (!SkeletalMesh)
        return OutBodies;

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
    if (!MeshComp)
        return;

    for (FBodyInstance* Body : MeshComp->Bodies) {
        if (!Body || !Body->IsValidBodyInstance())
            continue;

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
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No PhysicsAsset is set."));
        return false;
    }
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No SkeletalMeshComponent is set."));
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

    // Bones you intend to track (TrackedBones) that are not in the PhysicsAsset
    for (const FName& BoneName : TrackedBones) {
        if (!AssetBodyNames.Contains(BoneName)) {
            OutMissingBones.Add(BoneName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Tracked bone missing in PhysicsAsset: %s"),
                   *BoneName.ToString());
        }
    }

    // Bodies defined in PhysicsAsset but not present at runtime
    for (const FName& AssetName : AssetBodyNames) {
        if (!RuntimeBodyNames.Contains(AssetName)) {
            OutMissingBones.Add(AssetName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Body in PhysicsAsset but NOT present at runtime: %s"),
                   *AssetName.ToString());
        }
    }

    // Bodies present at runtime but not in the PhysicsAsset (rare)
    for (const FName& InstanceName : RuntimeBodyNames) {
        if (!AssetBodyNames.Contains(InstanceName)) {
            OutInstancesWithoutBodies.Add(InstanceName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Body present at runtime but NOT in PhysicsAsset: %s"),
                   *InstanceName.ToString());
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
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No PhysicsAsset is set."));
        return false;
    }
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No SkeletalMeshComponent is set."));
        return false;
    }

    // Constraints defined in the PhysicsAsset
    TMap<FName, const FConstraintInstance*> AssetConstraintMap;
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : CachedPhysicsAsset->ConstraintSetup) {
        if (!ConstraintTemplate)
            continue;
        const FConstraintInstance& TemplateInstance = ConstraintTemplate->DefaultInstance;
        const FName ConstraintName = TemplateInstance.JointName;
        if (ConstraintName.IsNone())
            continue;
        AssetConstraintMap.Add(ConstraintName, &TemplateInstance);
    }

    // Runtime constraints in the SkeletalMesh
    TMap<FName, FConstraintInstance*> RuntimeConstraintMap;
    for (FConstraintInstance* RuntimeConstraint : SkeletalMesh->Constraints) {
        if (RuntimeConstraint && !RuntimeConstraint->JointName.IsNone()) {
            RuntimeConstraintMap.Add(RuntimeConstraint->JointName, RuntimeConstraint);
        }
    }

    // 1. Constraints in asset but missing at runtime
    for (const auto& AssetPair : AssetConstraintMap) {
        const FName& ConstraintName = AssetPair.Key;
        const FConstraintInstance* AssetInstance = AssetPair.Value;

        FConstraintInstance** RuntimeInstancePtr = RuntimeConstraintMap.Find(ConstraintName);
        if (!RuntimeInstancePtr) {
            OutMissingConstraints.Add(ConstraintName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Constraint in PhysicsAsset but NOT present at runtime: %s"),
                   *ConstraintName.ToString());
            continue;
        }

        // 2. Compare parent/child bones for mismatches
        FConstraintInstance* RuntimeInstance = *RuntimeInstancePtr;
        if (RuntimeInstance->ConstraintBone1 != AssetInstance->ConstraintBone1 ||
            RuntimeInstance->ConstraintBone2 != AssetInstance->ConstraintBone2) {
            OutMismatchedConstraints.Add(ConstraintName);
            UE_LOG(LogTemp, Error,
                   TEXT("[OHPACManager] Constraint binding mismatch: %s. Asset (Parent: %s, Child: %s), Runtime "
                        "(Parent: %s, Child: %s)"),
                   *ConstraintName.ToString(), *AssetInstance->ConstraintBone1.ToString(),
                   *AssetInstance->ConstraintBone2.ToString(), *RuntimeInstance->ConstraintBone1.ToString(),
                   *RuntimeInstance->ConstraintBone2.ToString());
        }
    }

    // 3. Constraints present at runtime but not in asset (rare, possible for dynamic/additional constraints)
    for (const auto& RuntimePair : RuntimeConstraintMap) {
        const FName& ConstraintName = RuntimePair.Key;
        if (!AssetConstraintMap.Contains(ConstraintName)) {
            OutRuntimeConstraintsNotInAsset.Add(ConstraintName);
            UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Constraint present at runtime but NOT in PhysicsAsset: %s"),
                   *ConstraintName.ToString());
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
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No PhysicsAsset is set."));
        return false;
    }
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] SkeletalMeshComponent missing."));
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
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No SkeletalMeshComponent."));
        bValid = false;
    }
    if (!PhysicalAnimationComponent) {
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] No PhysicalAnimationComponent."));
        bValid = false;
    }

    if (!bValid) {
        UE_LOG(LogTemp, Error,
               TEXT("[OHPACManager] Core component validation failed. Skipping PhysicsAsset validation."));
        return false;
    }

    // Physics asset validation (but don't fail on warnings)
    ValidatePhysicsAsset(OutMissingBones, OutInstancesWithoutBodies, OutMissingConstraints,
                         OutRuntimeConstraintsNotInAsset, OutMismatchedConstraints);

    // Log warnings but don't fail initialization for missing optional bones
    if (OutMissingBones.Num() > 0) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPACManager] %d tracked bones missing bodies (will be excluded from simulation)"),
               OutMissingBones.Num());
        for (const FName& Bone : OutMissingBones) {
            UE_LOG(LogTemp, Verbose, TEXT("[OHPACManager] Missing body for bone: %s"), *Bone.ToString());
        }
    }

    // Only fail on critical errors (mismatched constraints are critical)

    if (OutMismatchedConstraints.Num() > 0) {
        UE_LOG(LogTemp, Error, TEXT("[OHPACManager] Critical validation errors found - initialization failed"));
        return false;
    }

    UE_LOG(LogTemp, Log, TEXT("[OHPACManager] Validation passed with %d warnings"),
           OutMissingBones.Num() + OutInstancesWithoutBodies.Num() + OutMissingConstraints.Num() +
               OutRuntimeConstraintsNotInAsset.Num());

    return true; // Pass even with non-critical warnings
}

bool UOHPACManager::IsSkeletalMeshBindingValid(bool bAutoFix, bool bLog) const {
    USkeletalMeshComponent* BoundMesh = SkeletalMesh;
    USkeletalMeshComponent* PACMesh = nullptr;
    if (PhysicalAnimationComponent)
        PACMesh = PhysicalAnimationComponent->GetSkeletalMesh();

    bool bMatch = (BoundMesh && PACMesh && BoundMesh == PACMesh);

    if (!bMatch && bAutoFix && PhysicalAnimationComponent && BoundMesh) {
        // Heal the binding automatically!
        PhysicalAnimationComponent->SetSkeletalMeshComponent(BoundMesh);
        PACMesh = PhysicalAnimationComponent->GetSkeletalMesh();
        bMatch = (BoundMesh == PACMesh);

        if (bLog) {
            UE_LOG(LogTemp, Warning,
                   TEXT("[OHPACManager] Auto-fixed PhysicalAnimationComponent SkeletalMesh binding: BoundMesh=%s, "
                        "PACMesh=%s, Match=%d"),
                   BoundMesh ? *BoundMesh->GetName() : TEXT("NULL"), PACMesh ? *PACMesh->GetName() : TEXT("NULL"),
                   bMatch ? 1 : 0);
        }
    } else if (bLog) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] SkeletalMesh binding check: BoundMesh=%s, PACMesh=%s, Match=%d"),
               BoundMesh ? *BoundMesh->GetName() : TEXT("NULL"), PACMesh ? *PACMesh->GetName() : TEXT("NULL"),
               bMatch ? 1 : 0);
    }

    return bMatch;
}

// Add to ResetPACManager() method - replace existing implementation:
void UOHPACManager::ResetPACManager() {
    UE_LOG(LogTemp, Log, TEXT("[OHPACManager] Resetting PAC Manager..."));

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

    // Mark as not initialized
    bIsInitialized = false;

    UE_LOG(LogTemp, Log, TEXT("[OHPACManager] PAC Manager reset complete"));
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
    if (!SkeletalMesh)
        return;
    FBodyInstance* Body = GetBodyInstanceDirect(BoneName);
    if (!Body) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPACManager] Bone %s: No body instance!"), *BoneName.ToString());
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

    UE_LOG(LogTemp, Log, TEXT("[OHPACManager] Memory footprint: %d KB"), MemoryFootprint / 1024);
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

void UOHPACManager::DrawDebugOverlay() const {
    if (!SkeletalMesh)
        return;

    int32 BoneIdx = 0;

    for (const FName& BoneName : SimulatableBones) {
        float VerticalStep = 22.f;
        float BaseOffset = 20.f;
        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        float BlendWeight = Body ? Body->PhysicsBlendWeight : -1.f;
        bool bSim = Body ? Body->IsInstanceSimulatingPhysics() : false;

        // Only show for simulated/blending bones
        if (!(bSim || BlendWeight > 0.05f))
            continue;

        FVector BoneLocation = SkeletalMesh->GetBoneLocation(BoneName);
        FVector Offset = FVector(0, 0, BaseOffset + VerticalStep * BoneIdx++);

        FColor LabelColor = !Body                  ? FColor::White
                            : !bSim                ? FColor::Red
                            : BlendWeight >= 0.95f ? FColor::Green
                                                   : FColor::Yellow;

        FString DebugText =
            FString::Printf(TEXT("%s\nBW: %.2f Sim: %d"), *BoneName.ToString(), BlendWeight, bSim ? 1 : 0);

        DrawDebugString(GetWorld(), BoneLocation + Offset, DebugText, nullptr, LabelColor, 0.f, true, 1.2f);
    }
}

bool UOHPACManager::IsBoneDrivenByPhysicalAnimation(const FName& BoneName) const {
    if (!SkeletalMesh)
        return false;

    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(/*bIncludesTerminated=*/false, ConstraintAccessors);

    const FString PhysicalAnimPrefix(TEXT("PhysicalAnimation_"));
    const FString TargetConstraintName = PhysicalAnimPrefix + BoneName.ToString();

    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        const FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint)
            continue;

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

void UOHPACManager::LogAllPhysicallyDrivenBonesWithDriveValues() const {
    if (!SkeletalMesh) {
        UE_LOG(LogOHPAC, Warning, TEXT("[OHPACManager] SkeletalMeshComponent is null, cannot list driven bones."));
        return;
    }

    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(/*bIncludesTerminated=*/false, ConstraintAccessors);

    int32 DrivenCount = 0;

    for (const FName& BoneName : SimulatableBones) {
        if (!IsBoneDrivenByPhysicalAnimation(BoneName))
            continue;

        // Now, find and log drive values for this bone
        const FString PhysicalAnimPrefix(TEXT("PhysicalAnimation_"));
        const FString TargetConstraintName = PhysicalAnimPrefix + BoneName.ToString();

        for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
            const FConstraintInstance* Constraint = Accessor.Get();
            if (!Constraint)
                continue;

            if (Constraint->JointName.ToString().Equals(TargetConstraintName, ESearchCase::IgnoreCase)) {
                // Gather drive values
                const auto& Linear = Constraint->ProfileInstance.LinearDrive;
                const auto& Angular = Constraint->ProfileInstance.AngularDrive;
                FString DriveInfo;
                if (Linear.XDrive.Stiffness > KINDA_SMALL_NUMBER || Linear.YDrive.Stiffness > KINDA_SMALL_NUMBER ||
                    Linear.ZDrive.Stiffness > KINDA_SMALL_NUMBER) {
                    DriveInfo += FString::Printf(TEXT("Linear [X: %.1f, Y: %.1f, Z: %.1f]"), Linear.XDrive.Stiffness,
                                                 Linear.YDrive.Stiffness, Linear.ZDrive.Stiffness);
                }
                if (Angular.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER ||
                    Angular.SwingDrive.Stiffness > KINDA_SMALL_NUMBER ||
                    Angular.TwistDrive.Stiffness > KINDA_SMALL_NUMBER) {
                    if (!DriveInfo.IsEmpty())
                        DriveInfo += TEXT(" | ");
                    DriveInfo += FString::Printf(TEXT("Angular [Slerp: %.1f, Swing: %.1f, Twist: %.1f]"),
                                                 Angular.SlerpDrive.Stiffness, Angular.SwingDrive.Stiffness,
                                                 Angular.TwistDrive.Stiffness);
                }

                UE_LOG(LogOHPAC, Log, TEXT("[OHPACManager] Driven bone: %s | %s"), *BoneName.ToString(), *DriveInfo);
                DrivenCount++;
                break; // Only one constraint per driven bone
            }
        }
    }

    UE_LOG(LogOHPAC, Log, TEXT("[OHPACManager] Found %d physically animated (driven) bones at runtime."), DrivenCount);
}

void UOHPACManager::VisualizeActivePhysicalAnimationDrives() const {
    if (!SkeletalMesh)
        return;

    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(false, ConstraintAccessors);

    int32 VisualizedCount = 0;

    // Lambda to get the parent bone name (safe for nullptrs, works for all Unreal versions)
    auto GetParentBone = [this](const FName& Bone) -> FName {
        if (!SkeletalMesh || !SkeletalMesh->GetSkeletalMeshAsset())
            return NAME_None;
        int32 BoneIndex = SkeletalMesh->GetBoneIndex(Bone);
        if (BoneIndex == INDEX_NONE)
            return NAME_None;
        int32 ParentIndex = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton().GetParentIndex(BoneIndex);
        return (ParentIndex != INDEX_NONE) ? SkeletalMesh->GetBoneName(ParentIndex) : NAME_None;
    };

    for (const FName& BoneName : SimulatableBones) {
        if (!IsBoneDrivenByPhysicalAnimation(BoneName))
            continue;

        const FString PhysicalAnimPrefix(TEXT("PhysicalAnimation_"));
        const FString TargetConstraintName = PhysicalAnimPrefix + BoneName.ToString();

        for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
            const FConstraintInstance* Constraint = Accessor.Get();
            if (!Constraint)
                continue;

            if (Constraint->JointName.ToString().Equals(TargetConstraintName, ESearchCase::IgnoreCase)) {
                const auto& Linear = Constraint->ProfileInstance.LinearDrive;
                const auto& Angular = Constraint->ProfileInstance.AngularDrive;
                float LinearMag =
                    FMath::Max3(Linear.XDrive.Stiffness, Linear.YDrive.Stiffness, Linear.ZDrive.Stiffness);
                float AngularMag = FMath::Max3(Angular.SlerpDrive.Stiffness, Angular.SwingDrive.Stiffness,
                                               Angular.TwistDrive.Stiffness);
                float DriveMag = FMath::Max(LinearMag, AngularMag);

                if (DriveMag < KINDA_SMALL_NUMBER)
                    continue;

                // Color scale: 0 = blue, max = red (adjust 5000.f as needed for your project)
                FLinearColor Color = FLinearColor::LerpUsingHSV(FLinearColor::Blue, FLinearColor::Red,
                                                                FMath::Clamp(DriveMag / 5000.f, 0.f, 1.f));
                FColor FinalColor = Color.ToFColor(true);

                FString DriveInfo;
                if (LinearMag > KINDA_SMALL_NUMBER) {
                    DriveInfo += FString::Printf(TEXT("L[%.1f,%.1f,%.1f]"), Linear.XDrive.Stiffness,
                                                 Linear.YDrive.Stiffness, Linear.ZDrive.Stiffness);
                }
                if (AngularMag > KINDA_SMALL_NUMBER) {
                    if (!DriveInfo.IsEmpty())
                        DriveInfo += TEXT(" | ");
                    DriveInfo += FString::Printf(TEXT("A[%.1f,%.1f,%.1f]"), Angular.SlerpDrive.Stiffness,
                                                 Angular.SwingDrive.Stiffness, Angular.TwistDrive.Stiffness);
                }

                // Draw debug sphere at bone
                const FVector BoneLoc = SkeletalMesh->GetBoneLocation(BoneName);
                DrawDebugSphere(SkeletalMesh->GetWorld(), BoneLoc, 4.0f, 8, FinalColor, false, 0.05f);

                // Draw debug string above bone
                DrawDebugString(SkeletalMesh->GetWorld(), BoneLoc + FVector(0, 0, 8),
                                FString::Printf(TEXT("%s: %s"), *BoneName.ToString(), *DriveInfo), nullptr, FinalColor,
                                0.05f, false, 0.85f);

                // Print to on-screen debug message (unique key per bone)
                if (GEngine) {
                    int32 MsgKey = 100000 + VisualizedCount;
                    GEngine->AddOnScreenDebugMessage(MsgKey, 0.11f, FinalColor,
                                                     FString::Printf(TEXT("%s: %s"), *BoneName.ToString(), *DriveInfo));
                }

                // Draw line to parent if parent is also simulated/driven
                FName ParentBone = GetParentBone(BoneName);
                if (ParentBone != NAME_None && SimulatableBones.Contains(ParentBone)) {
                    FVector ParentLoc = SkeletalMesh->GetBoneLocation(ParentBone);
                    DrawDebugLine(SkeletalMesh->GetWorld(), BoneLoc, ParentLoc, FinalColor, false, 0.05f, 0, 1.5f);
                }

                VisualizedCount++;
                break; // Only one constraint per driven bone
            }
        }
    }

    if (GEngine && VisualizedCount > 0) {
        GEngine->AddOnScreenDebugMessage(
            22002, 0.11f, FColor::White,
            FString::Printf(TEXT("[OHPACManager] %d PhysAnim bones visualized."), VisualizedCount));
    }
}

void UOHPACManager::CheckSimulatingBonesForPhysicalAnimationDrives() const {
    constexpr int32 MaxListOnScreen = 10;
    constexpr bool bPrintConstraintNames = true;

    if (!SkeletalMesh) {
        GEngine->AddOnScreenDebugMessage(20244, 4.0f, FColor::Red, TEXT("PAC: SkeletalMesh is NULL!"));
        UE_LOG(LogOHPAC, Error, TEXT("[OHPACManager] SkeletalMesh pointer is NULL!"));
        return;
    }

    // --- Simulatable Bones ---
    TArray<FName> SimulatableArray = SimulatableBones.Array();
    FString SimulatableList;
    for (int32 i = 0; i < FMath::Min(MaxListOnScreen, SimulatableArray.Num()); ++i) {
        if (i > 0)
            SimulatableList += TEXT(", ");
        SimulatableList += SimulatableArray[i].ToString();
    }
    if (SimulatableArray.Num() > MaxListOnScreen)
        SimulatableList += TEXT(", ...");

    // --- Simulating Bones ---
    TArray<FName> SimulatingBones;
    TMap<FName, FString> BoneSimStatus;
    for (const FName& BoneName : SimulatableBones) {
        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        bool bValid = (Body && Body->IsValidBodyInstance());
        bool bSim = (bValid && Body->IsInstanceSimulatingPhysics());

        if (bSim)
            SimulatingBones.Add(BoneName);

        BoneSimStatus.Add(BoneName,
                          FString::Printf(TEXT("Valid:%d Sim:%d"), static_cast<int>(bValid), static_cast<int>(bSim)));
    }
    FString SimulatingList;
    for (int32 i = 0; i < FMath::Min(MaxListOnScreen, SimulatingBones.Num()); ++i) {
        if (i > 0)
            SimulatingList += TEXT(", ");
        SimulatingList += SimulatingBones[i].ToString();
    }
    if (SimulatingBones.Num() > MaxListOnScreen)
        SimulatingList += TEXT(", ...");

    // --- Driven Bones ---
    TArray<FConstraintInstanceAccessor> ConstraintAccessors;
    SkeletalMesh->GetConstraints(false, ConstraintAccessors);

    TSet<FName> DrivenBones;
    TArray<FString> ConstraintNames;
    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        const FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint)
            continue;
        FString ConstraintName = Constraint->JointName.ToString();
        if (bPrintConstraintNames)
            ConstraintNames.Add(ConstraintName);

        if (ConstraintName.StartsWith(TEXT("PhysicalAnimation_"))) {
            FString BoneNameStr = ConstraintName.RightChop(18);
            FName DrivenBone(*BoneNameStr);

            const auto& Linear = Constraint->ProfileInstance.LinearDrive;
            const auto& Angular = Constraint->ProfileInstance.AngularDrive;
            bool bHasLinear = Linear.XDrive.Stiffness > KINDA_SMALL_NUMBER ||
                              Linear.YDrive.Stiffness > KINDA_SMALL_NUMBER ||
                              Linear.ZDrive.Stiffness > KINDA_SMALL_NUMBER;
            bool bHasAngular = Angular.SlerpDrive.Stiffness > KINDA_SMALL_NUMBER ||
                               Angular.SwingDrive.Stiffness > KINDA_SMALL_NUMBER ||
                               Angular.TwistDrive.Stiffness > KINDA_SMALL_NUMBER;

            if (bHasLinear || bHasAngular)
                DrivenBones.Add(DrivenBone);
        }
    }
    TArray<FName> DrivenBonesArray = DrivenBones.Array();
    FString DrivenList;
    for (int32 i = 0; i < FMath::Min(MaxListOnScreen, DrivenBonesArray.Num()); ++i) {
        if (i > 0)
            DrivenList += TEXT(", ");
        DrivenList += DrivenBonesArray[i].ToString();
    }
    if (DrivenBonesArray.Num() > MaxListOnScreen)
        DrivenList += TEXT(", ...");

    // --- Missing (Simulating but not Driven) ---
    TArray<FName> MissingBones;
    TArray<FString> MissingBonesDiag;
    for (const FName& Bone : SimulatingBones) {
        if (!DrivenBones.Contains(Bone)) {
            MissingBones.Add(Bone);
            FString Diag = BoneSimStatus.Contains(Bone) ? BoneSimStatus[Bone] : TEXT("NoInfo");
            MissingBonesDiag.Add(Bone.ToString() + TEXT(" [") + Diag + TEXT("]"));
        }
    }
    FString MissingList;
    for (int32 i = 0; i < FMath::Min(MaxListOnScreen, MissingBonesDiag.Num()); ++i) {
        MissingList += MissingBonesDiag[i] + TEXT("\n");
    }
    if (MissingBonesDiag.Num() > MaxListOnScreen)
        MissingList += TEXT("...\n");

    // --- Constraint Names ---
    FString ConstraintNamesList;
    if (bPrintConstraintNames) {
        for (int32 i = 0; i < FMath::Min(MaxListOnScreen, ConstraintNames.Num()); ++i) {
            ConstraintNamesList += ConstraintNames[i] + TEXT("\n");
        }
        if (ConstraintNames.Num() > MaxListOnScreen)
            ConstraintNamesList += TEXT("...\n");
    }

    // --- On Screen Message ---
    FString OnScreenMsg;
    OnScreenMsg +=
        FString::Printf(TEXT("[PAC] Simulatable: %d  Sim: %d  Driven: %d  Missing: %d"), SimulatableArray.Num(),
                        SimulatingBones.Num(), DrivenBonesArray.Num(), MissingBones.Num());

    OnScreenMsg += TEXT("\n--- Simulatable Bones ---\n");
    OnScreenMsg += SimulatableList;

    OnScreenMsg += TEXT("\n\n--- Simulating Bones ---\n");
    OnScreenMsg += SimulatingList;

    OnScreenMsg += TEXT("\n\n--- Driven (PhysAnim) Bones ---\n");
    OnScreenMsg += DrivenList;

    if (MissingBones.Num() > 0) {
        OnScreenMsg += TEXT("\n\n--- MISSING Sim+PhysAnim ---\n");
        OnScreenMsg += MissingList;
    }

    if (bPrintConstraintNames) {
        OnScreenMsg += TEXT("\n--- Constraint Names ---\n");
        OnScreenMsg += ConstraintNamesList;
    }

    // --- Print to screen ---
    FColor MsgColor = (MissingBones.Num() == 0) ? FColor::Green : FColor::Orange;
    if (GEngine) {
        GEngine->AddOnScreenDebugMessage(20244, 2.2f, MsgColor, OnScreenMsg);
    }

    // --- Print low-level drive/constraint details to log only ---
    UE_LOG(LogOHPAC, Warning, TEXT("[OHPACManager] ----------- Per-frame PhysicalAnim Diag -----------"));
    for (const FConstraintInstanceAccessor& Accessor : ConstraintAccessors) {
        const FConstraintInstance* Constraint = Accessor.Get();
        if (!Constraint)
            continue;
        FString ConstraintName = Constraint->JointName.ToString();

        const auto& Linear = Constraint->ProfileInstance.LinearDrive;
        const auto& Angular = Constraint->ProfileInstance.AngularDrive;
        UE_LOG(LogOHPAC, Display, TEXT("Constraint: %s | LinX: %.2f Y: %.2f Z: %.2f  | AngS: %.2f SW: %.2f TW: %.2f"),
               *ConstraintName, Linear.XDrive.Stiffness, Linear.YDrive.Stiffness, Linear.ZDrive.Stiffness,
               Angular.SlerpDrive.Stiffness, Angular.SwingDrive.Stiffness, Angular.TwistDrive.Stiffness);
    }
}

void UOHPACManager::DebugBodyInstanceSimulation() const {
    if (!SkeletalMesh) {
        GEngine->AddOnScreenDebugMessage(20245, 4.0f, FColor::Red, TEXT("PAC: SkeletalMesh is NULL!"));
        return;
    }

    FString Info;
    Info +=
        FString::Printf(TEXT("Checking SkeletalMeshComponent: %s [0x%p]\n"), *SkeletalMesh->GetName(), SkeletalMesh);
    Info += FString::Printf(TEXT("IsSimulatingPhysics() [component]: %d\n"), (int)SkeletalMesh->IsSimulatingPhysics());

    int32 SimulatingCount = 0;
    int32 ValidBodyCount = 0;

    for (const FName& BoneName : SimulatableBones) {
        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        FString BoneMsg = BoneName.ToString();

        if (!Body) {
            BoneMsg += TEXT("  -- NO BODY INSTANCE");
        } else if (!Body->IsValidBodyInstance()) {
            BoneMsg += TEXT("  -- INVALID BODY INSTANCE");
            ValidBodyCount++;
        } else {
            ValidBodyCount++;
            bool bSim = Body->IsInstanceSimulatingPhysics();
            if (bSim)
                SimulatingCount++;
            BoneMsg += FString::Printf(TEXT("  -- Valid. IsSimPhysics: %d, BoneIdx: %d, Owner: %s"), (int)bSim,
                                       SkeletalMesh->GetBoneIndex(BoneName), *SkeletalMesh->GetName());
        }

        GEngine->AddOnScreenDebugMessage(30000 + ValidBodyCount, 6.f, FColor::Yellow, BoneMsg);
    }

    Info += FString::Printf(TEXT("Total Simulatable: %d. Found valid bodies: %d, Simulating: %d"),
                            SimulatableBones.Num(), ValidBodyCount, SimulatingCount);

    GEngine->AddOnScreenDebugMessage(20246, 8.0f, FColor::Green, Info);
}

void UOHPACManager::DebugPhysicalAnimationConstraints() {
    if (!SkeletalMesh)
        return;

    // Short summary for on-screen, detailed for log
    FString DriveBonesList;

    // You want to minimize screen clutter, so limit this list size as needed.
    int32 DrivesActive = 0;
    int32 SimulatedBones = 0;

    // Access runtime constraints:
    for (FConstraintInstance* CI : SkeletalMesh->Constraints) {
        if (!CI)
            continue;
        const FName& BoneName = CI->GetChildBoneName();

        // Is this bone simulating physics? (Assume you have a quick utility for this, e.g. IsBoneSimulating(BoneName))
        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance() || !Body->IsInstanceSimulatingPhysics())
            continue;
        SimulatedBones++;

        // Drive states
        bool bLinPos = CI->IsLinearPositionDriveEnabled();
        bool bLinVel = CI->IsLinearVelocityDriveEnabled();
        bool bAngOrient = CI->IsAngularOrientationDriveEnabled();
        bool bAngVel = CI->IsAngularVelocityDriveEnabled();

        if (bLinPos || bLinVel || bAngOrient || bAngVel) {
            DrivesActive++;
            // List to screen
            DriveBonesList += BoneName.ToString() + TEXT(" ");
        }

        // Get drive strengths (for log)
        float LinPosStrength, LinVelStrength, LinForceLimit;
        CI->GetLinearDriveParams(LinPosStrength, LinVelStrength, LinForceLimit);
        float AngSpring, AngDamping, AngForceLimit;
        CI->GetAngularDriveParams(AngSpring, AngDamping, AngForceLimit);

        // What axes are limited
        ELinearConstraintMotion LinX = CI->GetLinearXMotion();
        ELinearConstraintMotion LinY = CI->GetLinearYMotion();
        ELinearConstraintMotion LinZ = CI->GetLinearZMotion();

        UE_LOG(LogOHPAC, Display,
               TEXT("Constraint: %s | Sim | LinPos:%d Vel:%d | AngOrient:%d Vel:%d | Lin: %.2f/%.2f/%.2f | Ang: "
                    "%.2f/%.2f/%.2f | Axis: X=%d Y=%d Z=%d"),
               *BoneName.ToString(), (int)bLinPos, (int)bLinVel, (int)bAngOrient, (int)bAngVel, LinPosStrength,
               LinVelStrength, LinForceLimit, AngSpring, AngDamping, AngForceLimit, (int)LinX, (int)LinY, (int)LinZ);
    }

    FString ScreenSummary = FString::Printf(TEXT("PhysAnim Drv: %d | Sim: %d | ["), DrivesActive, SimulatedBones);
    ScreenSummary += DriveBonesList;
    ScreenSummary += TEXT("]");

    // Show on screen (lasts 0.3s, updates every tick)
    GEngine->AddOnScreenDebugMessage((uint64)-422, 0.3f, FColor::Yellow, ScreenSummary);
}

void UOHPACManager::DebugBodyPhysicsStates() {
    if (!SkeletalMesh)
        return;

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
        if (bSim)
            ++SimCount;
    }
    GEngine->AddOnScreenDebugMessage(-1, 0.3f, FColor::Yellow,
                                     FString::Printf(TEXT("Total Simulatable Sim: %d [Component: %s, %p]"), SimCount,
                                                     *SkeletalMesh->GetName(), SkeletalMesh));
}

#pragma endregion

#pragma endregion