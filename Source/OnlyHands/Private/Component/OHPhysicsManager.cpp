#include "Component/OHPhysicsManager.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "Data/Struct/OHPhysicsStructs.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "DrawDebugHelpers.h"
#include "TimerManager.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "FunctionLibrary/OHGraphUtils.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"

UOHPhysicsManager::UOHPhysicsManager() : PhysicsGraph() {
    PrimaryComponentTick.bCanEverTick = true;
    bShowSimBlendOverlay = false;
    bShowTrackedBoneKinematics = false;
    CachedZeroStrengthPACProfile.OrientationStrength = 0.f;
    CachedZeroStrengthPACProfile.PositionStrength = 0.f;
    CachedZeroStrengthPACProfile.VelocityStrength = 0.f;
    CachedZeroStrengthPACProfile.AngularVelocityStrength = 0.f;
    CachedZeroStrengthPACProfile.bIsLocalSimulation = true;

    TrackedBoneDefinitions = {"root",       "pelvis",     "spine_01",   "spine_02",   "spine_03",   "neck_01",
                              "head",       "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r", "lowerarm_l",
                              "lowerarm_r", "hand_l",     "hand_r",     "thigh_l",    "thigh_r",    "calf_l",
                              "calf_r",     "foot_l",     "foot_r",     "ball_l",     "ball_r"};

    SimExclusionBoneSet = {"root", "pelvis", "neck_01", "ball_l", "ball_r"};
}

#pragma region Lifecycle
void UOHPhysicsManager::BeginPlay() {
    if (!bEnablePhysicsManager)
        return;

    Super::BeginPlay();

    // Defer init to ensure components are valid
    FTimerHandle TimerHandle;
    GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &UOHPhysicsManager::InitializePhysicsManager,
                                           StartTickDelay, false);
}

void UOHPhysicsManager::TickComponent(float DeltaTime, ELevelTick TickType,
                                      FActorComponentTickFunction* ThisTickFunction) {
    if (!bEnablePhysicsManager)
        return;

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!SkeletalMesh)
        return;

    // === 1. Update kinematic tracking for all bones (foundation data) ===
    UpdateTrackedBoneKinematics(DeltaTime);

    // === 2. Process physics blend phases and apply physics changes ===
    if (ActiveBlends.Num() > 0) {
        ProcessBlendPhases(DeltaTime);
    }

    // === 3. Update constraint runtime states (strain tracking, etc.) ===
    if (PhysicalAnimationComponent && SkeletalMesh) {
        UpdateConstraintRuntimeStates(DeltaTime);
    }

    // === 4. Sync graph simulation flags with actual physics engine state ===
    UpdateBoneSimulationStates();

    // === 5. Debug visualizations (based on current state) ===
    if (bEnableDebugOverlay && GEngine && GEngine->IsEditor()) {
        TickDebugOverlay();
    }

    if (UOHGraphUtils::GetPhysicsGraphOverlayMode() != EPhysicsGraphOverlayMode::None) {
        UOHGraphUtils::DrawPhysicsGraphOverlay(PhysicsGraph, // FOHPhysicsGraphNode
                                               SkeletalMesh, // USkeletalMeshComponent*
                                               GetWorld(),
                                               0.05f, // Duration (short for per-frame overlay)
                                               UOHGraphUtils::GetPhysicsGraphOverlayMode(),
                                               true // Draw labels
        );
    }

    // === 6. Development analysis (editor-only, after all state updates) ===
#if WITH_EDITOR
    if (bEnableRuntimeTuningEvaluation) {
        EvaluateAndSuggestTuningForBones();
    }
#endif

    // === 7. Debug system analysis (captures final state after all processing) ===
    if (bEnableDebugDisplay) {
        DebugTick(DeltaTime);
    }
}

void UOHPhysicsManager::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    ResetPhysicsManager(); // must go before Super
    Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Initialization
void UOHPhysicsManager::InitializePhysicsManager() {
    CachedBoneMasses.Empty();
    CachedBoneLengths.Empty();

    // === STEP 1: Cache SkeletalMesh ===
    SkeletalMesh = GetOwner() ? GetOwner()->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
    if (!SkeletalMesh || !IsValid(SkeletalMesh) || !SkeletalMesh->IsRegistered()) {
        UE_LOG(LogTemp, Error, TEXT("[OHPhysicsManager] Initialization aborted — SkeletalMesh is missing or invalid."));
        return;
    }

    // === STEP 2: Cache the Physics Asset ===
    CachedPhysicsAsset = SkeletalMesh->GetPhysicsAsset();
    if (!CachedPhysicsAsset) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] SkeletalMesh has no PhysicsAsset — simulation features will be limited."));
    }

    // === STEP 3: Find or create PhysicalAnimationComponent ===
    InitializePhysicalAnimationSystem();

    // === STEP 4: Reset runtime state ===
    TrackedBoneDataMap.Reset();
    ActiveBlends.Reset();

    // === STEP 5: Initialize tracking and sim eligibility ===
    InitializeTrackedBoneData();
    InitializeSimulatableBones();

    if (PhysicalAnimationComponent && IsValid(PhysicalAnimationComponent)) {
        InitializeAllPhysicalAnimationProfiles(CachedZeroStrengthPACProfile);
    }

    // === STEP 6: Recreate physics state only if safe ===
    UWorld* World = GetWorld();
    if (World && World->IsGameWorld() && SkeletalMesh->IsRegistered()) {
        SkeletalMesh->RecreatePhysicsState();
    } else {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] Skipping RecreatePhysicsState — invalid or editor-only world."));
    }

    // === STEP 7: Debug logging ===
    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Init complete: %d Tracked, %d Simulatable"), TrackedBoneDataMap.Num(),
           SimulatableBones.Num());

    UOHGraphUtils::BuildPhysicsGraphFromComponent(SkeletalMesh, PhysicsGraph, "INIT", true);

    // Initialize constraint components with cached skeletal mesh
    PhysicsGraph.InitializeConstraintComponents(SkeletalMesh);

    // Validate geometry
    if (!PhysicsGraph.ValidateAllConstraintGeometry()) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] Constraint validation returned failure! Check individual logs for details."));
    }
    // Optional: Log performance metrics
    if (bEnableDebugOverlay) {
        PhysicsGraph.LogConstraintPerformanceReport();
    }
}

void UOHPhysicsManager::InitializePhysicalAnimationSystem() {
    // Try to find existing PAC on the owning actor
    PhysicalAnimationComponent = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();
    if (!PhysicalAnimationComponent) {
        // Dynamically create and register a new PAC if needed
        PhysicalAnimationComponent = NewObject<UPhysicalAnimationComponent>(
            GetOwner(), UPhysicalAnimationComponent::StaticClass(), TEXT("AutoPhysicalAnim"));

        if (PhysicalAnimationComponent) {
            PhysicalAnimationComponent->RegisterComponentWithWorld(GetWorld());
            // No need to attach; PAC is not a scene component
        }
    }

    // Bind the PAC to the skeletal mesh (required!)
    if (PhysicalAnimationComponent) {
        PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);
    }
}

void UOHPhysicsManager::InitializeTrackedBoneData() {
    TrackedBoneDataMap.Reset();

    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] InitializeTrackedBoneData aborted — SkeletalMesh is null."));
        return;
    }

    // Run validation first
    ValidateBoneMappings(true);

    constexpr float InitDeltaTime = 1 / 60.0f;
    const float InitTimeStamp = GetWorld()->GetTimeSeconds();

    // Use validated mappings
    for (const FName& TrackedBone : TrackedBoneDefinitions) {
        FName ActualBone = TrackedBone;

        // Use validated or suggested bone name
        if (ValidatedBoneMapping.Contains(TrackedBone)) {
            ActualBone = ValidatedBoneMapping[TrackedBone];
        } else if (BoneSuggestions.Contains(TrackedBone)) {
            ActualBone = BoneSuggestions[TrackedBone];
            UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Using suggested bone '%s' for '%s'"),
                   *ActualBone.ToString(), *TrackedBone.ToString());
        } else {
            continue; // Skip if no valid mapping
        }

        const int32 BoneIndex = SkeletalMesh->GetBoneIndex(ActualBone);
        if (BoneIndex == INDEX_NONE)
            continue;

        const FTransform BoneTransform = SkeletalMesh->GetBoneTransform(BoneIndex);

        FOHBoneData BoneData;
        BoneData.SetBoneName(ActualBone); // Use actual bone name
        BoneData.SetCurrentPosition(BoneTransform.GetLocation());
        BoneData.SetPreviousPosition(BoneTransform.GetLocation());
        BoneData.SetCurrentRotation(BoneTransform.GetRotation());
        BoneData.SetPreviousRotation(BoneTransform.GetRotation());

        BoneData.SetLinearVelocity(FVector::ZeroVector);
        BoneData.SetAngularVelocity(FVector::ZeroVector);
        BoneData.SetLinearAcceleration(FVector::ZeroVector);
        BoneData.SetAngularAcceleration(FVector::ZeroVector);
        BoneData.SetLastDeltaTime(InitDeltaTime);
        BoneData.SetIsSimulating(false); // may be updated later during SimulatableBone initialization

        // Optional: Prime motion history with an initial zero sample
        FOHMotionSample InitialSample =
            FOHMotionSample::CreateFromState(BoneTransform, FVector::ZeroVector, FVector::ZeroVector,
                                             FVector::ZeroVector, FVector::ZeroVector, InitTimeStamp);
        BoneData.PushMotionSample(InitialSample);

        TrackedBoneDataMap.Add(ActualBone, BoneData); // Key by original name
    }
}

void UOHPhysicsManager::InitializeSimulatableBones() {
    SimulatableBones.Reset();

    UPhysicsAsset* UsePhysicsAsset = CachedPhysicsAsset;
    if (!UsePhysicsAsset && SkeletalMesh) {
        UsePhysicsAsset = SkeletalMesh->GetPhysicsAsset();
        if (UsePhysicsAsset) {
            UE_LOG(LogTemp, Warning,
                   TEXT("[OHPhysicsManager] CachedPhysicsAsset was null — fallback to SkeletalMesh asset."));
        }
    }

    if (!UsePhysicsAsset) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Skipping simulatable bone setup — no valid PhysicsAsset."));
        return;
    }

    for (const FName& Bone : TrackedBoneDefinitions) {
        if (SimExclusionBoneSet.Contains(Bone))
            continue;

        if (!IsBoneValidForChain(Bone, "Root"))
            continue;

        if (UsePhysicsAsset->FindBodyIndex(Bone) != INDEX_NONE) {
            SimulatableBones.Add(Bone);

            // Optional: mark in tracked map
            if (FOHBoneData* BoneData = TrackedBoneDataMap.Find(Bone)) {
                BoneData->SetIsSimulating(false);
            }
        }
    }
}

void UOHPhysicsManager::InitializeAllPhysicalAnimationProfiles(const FPhysicalAnimationData& Profile) {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] InitializeAllPhysicalAnimationProfiles aborted — missing components."));
        return;
    }

    for (const FName& Bone : SimulatableBones) {
        ApplyUnifiedPhysicsProfileToChain(Bone, Profile, 5.0f, 10.0f, bEnableRuntimeTuningEvaluation);

        UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Applied PAC zero profile to Bone: %s"), *Bone.ToString());
    }

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Physical animation profiles applied to %d bones."),
           SimulatableBones.Num());
}

void UOHPhysicsManager::InitializeBoneLinksFromPhysicsAsset() {
    if (!CachedPhysicsAsset) {
        UE_LOG(LogTemp, Warning, TEXT("InitializeBoneLinksFromPhysicsAsset: No physics asset found."));
        return;
    }

    BoneLinkMap.Reset();

    for (const UPhysicsConstraintTemplate* Constraint : CachedPhysicsAsset->ConstraintSetup) {
        if (!Constraint)
            continue;

        const FName BoneA = Constraint->DefaultInstance.ConstraintBone1;
        const FName BoneB = Constraint->DefaultInstance.ConstraintBone2;

        if (BoneA.IsNone() || BoneB.IsNone())
            continue;

        // Estimate angular constraint fidelity
        const float Swing1 = Constraint->DefaultInstance.GetAngularSwing1Limit();
        const float Swing2 = Constraint->DefaultInstance.GetAngularSwing2Limit();
        const float Twist = Constraint->DefaultInstance.GetAngularTwistLimit();
        const float AngularTolerance = (Swing1 + Swing2 + Twist) / 3.f;

        const float Damping = Constraint->DefaultInstance.ProfileInstance.AngularDrive.SlerpDrive.Damping;

        // Parent → Child
        FOHPhysicsBoneLink& ParentLink = BoneLinkMap.FindOrAdd(BoneA);
        ParentLink.Children.AddUnique(BoneB);
        ParentLink.AngularConstraintTolerance = AngularTolerance;
        ParentLink.DampingMultiplier = FMath::Clamp(Damping / 50.f, 0.1f, 2.f); // Normalize to range
        ParentLink.PropagationAttenuation = 0.5f;

        // Child → Parent
        FOHPhysicsBoneLink& ChildLink = BoneLinkMap.FindOrAdd(BoneB);
        if (!ChildLink.Parent.IsNone() && ChildLink.Parent != BoneA) {
            UE_LOG(LogTemp, Warning, TEXT("Bone %s already has parent %s, now also %s — check constraint structure."),
                   *BoneB.ToString(), *ChildLink.Parent.ToString(), *BoneA.ToString());
        }
        ChildLink.Parent = BoneA;
        ChildLink.AngularConstraintTolerance = AngularTolerance;
        ChildLink.DampingMultiplier = FMath::Clamp(Damping / 50.f, 0.1f, 2.f);
        ChildLink.PropagationAttenuation = 0.5f;
    }

    UE_LOG(LogTemp, Log, TEXT("BoneLinkMap initialized with %d entries from PhysicsAsset."), BoneLinkMap.Num());
}

#pragma endregion

#pragma region BoneLinks
void UOHPhysicsManager::BuildBoneLinkGraph() {
    for (auto& Pair : BoneLinkMap) {
        for (const FName& Child : Pair.Value.Children) {
            ChildGraph.FindOrAdd(Pair.Key).Add(Child);
        }
    }
}

#pragma endregion

#pragma region RuntimeManagement

void UOHPhysicsManager::ResetPhysicsManager() {
    if (!SkeletalMesh || !IsValid(SkeletalMesh) || !SkeletalMesh->IsRegistered()) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Reset aborted: SkeletalMesh is invalid or unregistered."));
        return;
    }

    // STEP 1: Fully clear physics and transforms
    ClearPhysicsState();

    // STEP 2: Reset physics bodies
    if (SkeletalMesh->GetWorld() && SkeletalMesh->GetWorld()->IsGameWorld()) {
        SkeletalMesh->ResetAllBodiesSimulatePhysics();
        SkeletalMesh->RecreatePhysicsState();
    } else {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] Skipping RecreatePhysicsState: Not in game world or world is invalid."));
    }

    // STEP 3: Rebind PAC
    if (IsValid(PhysicalAnimationComponent)) {
        PhysicalAnimationComponent->SetSkeletalMeshComponent(nullptr);
        PhysicalAnimationComponent->SetSkeletalMeshComponent(SkeletalMesh);
    }

    // STEP 4: Clear PAC profile assignments
    if (IsValid(PhysicalAnimationComponent)) {
        TArray<FName> BoneNames;
        SkeletalMesh->GetBoneNames(BoneNames);
        for (const FName& BoneName : BoneNames) {
            PhysicalAnimationComponent->ApplyPhysicalAnimationProfileBelow(BoneName, NAME_None, false, false);
        }
    }

    // STEP 5: Reset internal state
    ActiveBlends.Reset();
    BoneSimBlendRefCount.Reset();
    ActiveBonePACData.Reset();
    TrackedBoneDataMap.Reset();
    SimulatableBones.Reset();

    // STEP 6: Re-initialize runtime tracking and sim eligibility
    InitializeTrackedBoneData();
    InitializeSimulatableBones();

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Full physics reset and reinit complete."));
}

void UOHPhysicsManager::ClearPhysicsState() {
    if (!SkeletalMesh || !IsValid(SkeletalMesh) || !SkeletalMesh->IsRegistered()) {
        UE_LOG(LogTemp, Warning,
               TEXT("[OHPhysicsManager] ClearPhysicsState aborted: SkeletalMesh is invalid or unregistered."));
        return;
    }

    // STEP 1: Stop simulation and clear velocities
    SkeletalMesh->SetSimulatePhysics(false);
    SkeletalMesh->SetAllBodiesSimulatePhysics(false);
    SkeletalMesh->SetAllPhysicsLinearVelocity(FVector::ZeroVector, false);
    SkeletalMesh->SetAllPhysicsAngularVelocityInDegrees(FVector::ZeroVector, false);
    SkeletalMesh->SetAllBodiesBelowPhysicsBlendWeight(NAME_None, 0.f, false);

    // STEP 2: Reset transforms
    SkeletalMesh->ClearMotionVector();
    SkeletalMesh->RefreshBoneTransforms();

    // Avoid unsafe ticks during teardown
    if (!GetWorld() || GetWorld()->bIsTearingDown)
        return;

    SkeletalMesh->TickComponent(0.f, ELevelTick::LEVELTICK_All, nullptr);

    if (UAnimInstance* AnimInstance = SkeletalMesh->GetAnimInstance()) {
        AnimInstance->UpdateAnimation(0.f, false);
    }

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Physics state cleared."));
}

void UOHPhysicsManager::ForceReinitializePhysics(bool bLogResults) {
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] ForceReinitializePhysics aborted — SkeletalMesh is null."));
        return;
    }

    const double StartTime = FPlatformTime::Seconds();

    ResetPhysicsManager();
    InitializePhysicsManager();

    const double ElapsedMS = (FPlatformTime::Seconds() - StartTime) * 1000.0;

    if (bLogResults) {
        UE_LOG(
            LogTemp, Log,
            TEXT(
                "[OHPhysicsManager] ForceReinitializePhysics completed in %.2f ms. Bones: %d tracked, %d simulatable."),
            ElapsedMS, TrackedBoneDataMap.Num(), SimulatableBones.Num());
    }
}

#pragma endregion

#pragma region Accessors

USkeletalMeshComponent* UOHPhysicsManager::GetSkeletalMeshComponent() const {
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] GetSkeletalMeshComponent: Missing SkeletalMesh"));
        return nullptr;
    }
    return SkeletalMesh;
}

float UOHPhysicsManager::GetCurrentMotionTime() const {
    // Use delegate if bound
    if (MotionTimeOverride.IsBound()) {
        return MotionTimeOverride.Execute();
    }

    switch (ActiveTimeDomain) {
    case EOHMotionTimeDomain::AnimationTime: {
        if (SkeletalMesh && SkeletalMesh->GetAnimInstance()) {
            return SkeletalMesh->GetAnimInstance()
                ->GetWorld()
                ->GetTimeSeconds(); // Can be replaced with actual animation time
        }
        break; // Fall through to default
    }
    case EOHMotionTimeDomain::PhysicsTime: {
        // Replace with accurate physics time if you support substepping
        return GetWorld()->GetTimeSeconds();
    }
    case EOHMotionTimeDomain::GameWorldTime:
    default:
        return GetWorld()->GetTimeSeconds();
    }

    // Fallback safety return (in case something above fails)
    return GetWorld()->GetTimeSeconds();
}

#pragma endregion

#pragma region MotionTracking

void UOHPhysicsManager::UpdateTrackedBoneKinematics(float DeltaTime) {
    if (!SkeletalMesh)
        return;

    const float TimeStamp = GetCurrentMotionTime(); // Supports anim vs phys tick domain

    for (const FName& BoneName : TrackedBoneDefinitions) {
        if (!SkeletalMesh->DoesSocketExist(BoneName))
            continue;

        // Get current transform data
        const FVector Position = SkeletalMesh->GetSocketLocation(BoneName);
        const FQuat Rotation = SkeletalMesh->GetSocketQuaternion(BoneName);

        // Access or create the bone data entry
        FOHBoneData& BoneData = TrackedBoneDataMap.FindOrAdd(BoneName);

        // Assign name on first creation
        if (!BoneData.IsValid()) {
            BoneData.SetBoneName(BoneName);
        }

        // Directly pass the timestamp to UpdateKinematics (which calls AddMotionSample internally)
        BoneData.UpdateKinematics(Position, Rotation, DeltaTime, TimeStamp);
    }
}

void UOHPhysicsManager::UpdateBoneSimulationStates() {
    const TArray<FName>& SimBones = GetSimulatedBones();

    for (auto& Pair : TrackedBoneDataMap) {
        const FName Bone = Pair.Key;
        FOHBoneData& Data = Pair.Value;
        Data.SetIsSimulating(SimBones.Contains(Bone));
    }
}

bool UOHPhysicsManager::IsBoneTrackable(FName BoneName) const {
    if (!SkeletalMesh)
        return false;

    const int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
    if (BoneIndex == INDEX_NONE)
        return false;

    if (!IsBoneNamePatternValid(BoneName))
        return false;

    return true;
}

#pragma region BoneDataAccessors
FVector UOHPhysicsManager::GetTrackedBoneVelocity(FName Bone) const {
    const FOHBoneData* Data = TrackedBoneDataMap.Find(Bone);
    return (Data && Data->IsValid()) ? Data->GetBodyLinearVelocity() : FVector::ZeroVector;
}

FVector UOHPhysicsManager::GetTrackedBoneLinearAcceleration(FName Bone) const {
    const FOHBoneData* Data = TrackedBoneDataMap.Find(Bone);
    return (Data && Data->IsValid()) ? Data->GetLinearAcceleration() : FVector::ZeroVector;
}

FVector UOHPhysicsManager::GetTrackedBoneAngularVelocity(FName Bone) const {
    const FOHBoneData* Data = TrackedBoneDataMap.Find(Bone);
    return (Data && Data->IsValid()) ? Data->GetAngularVelocity() : FVector::ZeroVector;
}

FVector UOHPhysicsManager::GetTrackedBonePosition(FName Bone) const {
    const FOHBoneData* Data = TrackedBoneDataMap.Find(Bone);
    return (Data && Data->IsValid()) ? Data->GetCurrentPosition() : FVector::ZeroVector;
}

FRotator UOHPhysicsManager::GetTrackedBoneRotation(FName Bone) const {
    const FOHBoneData* Data = TrackedBoneDataMap.Find(Bone);
    return (Data && Data->IsValid()) ? Data->GetCurrentRotation().Rotator() : FRotator::ZeroRotator;
}

TArray<FName> UOHPhysicsManager::GetTrackedBoneNames() const {
    TArray<FName> BoneNames;
    for (const TPair<FName, FOHBoneData>& Pair : TrackedBoneDataMap) {
        if (Pair.Value.IsValid()) {
            BoneNames.Add(Pair.Key);
        }
    }
    return BoneNames;
}

#pragma endregion

#pragma endregion

#pragma region Hitreaction

void UOHPhysicsManager::PlayPhysicsHitReaction(FName BoneName, EOHPhysicsProfile Profile, float BlendIn, float Hold,
                                               float BlendOut, FVector OptionalImpulseDir,
                                               float OptionalImpulseStrength, FName ReactionTag) {
    const FPhysicalAnimationData* ProfileData = PhysicsProfiles.Find(Profile);
    if (!ProfileData) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] No PAC profile found for enum %d"), static_cast<uint8>(Profile));
        return;
    }

    PlayPhysicsHitReaction_Internal(BoneName, *ProfileData, BlendIn, Hold, BlendOut, OptionalImpulseDir,
                                    OptionalImpulseStrength, ReactionTag);
}

void UOHPhysicsManager::PlayPhysicsHitReactionCustom(FName BoneName, const FPhysicalAnimationData& CustomProfile,
                                                     float BlendIn, float Hold, float BlendOut,
                                                     FVector OptionalImpulseDir, float OptionalImpulseStrength,
                                                     FName ReactionTag) {
    PlayPhysicsHitReaction_Internal(BoneName, CustomProfile, BlendIn, Hold, BlendOut, OptionalImpulseDir,
                                    OptionalImpulseStrength, ReactionTag);
}

void UOHPhysicsManager::PlayPhysicsHitReaction_Internal(FName& BoneName, const FPhysicalAnimationData& ProfileData,
                                                        float BlendIn, float Hold, float BlendOut,
                                                        FVector OptionalImpulseDir, float OptionalImpulseStrength,
                                                        FName ReactionTag,
                                                        TArray<FName>* OutAffectedBones /* optional */) {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] HitReaction failed: Missing SkeletalMesh or PAC"));
        return;
    }
    if (!SkeletalMesh->DoesSocketExist(BoneName)) {
        if (BoneSuggestions.Contains(BoneName)) {
            FName OriginalBone = BoneName;        // Preserve for logging if needed
            BoneName = BoneSuggestions[BoneName]; // ← The money shot
            UE_LOG(LogTemp, Log, TEXT("[OH] Using suggested bone '%s' for hit reaction on '%s'"), *BoneName.ToString(),
                   *OriginalBone.ToString());
        } else {
            UE_LOG(LogTemp, Warning, TEXT("[OH] HitReaction failed: Bone '%s' not found"), *BoneName.ToString());
            return;
        }
    }

    // Skip if already blending and fully held
    if (FActivePhysicsBlend* Existing = ActiveBlends.Find(BoneName)) {
        if (Existing->Phase == EOHBlendPhase::Hold && Existing->BlendAlpha >= 0.99f)
            return;
    }

    const float StartAlpha = FMath::Clamp(GetBlendAlpha(BoneName), 0.f, 1.f);

    // === Phase 1: Clean old state ===
    ClearPhysicsStateForBone(BoneName); // This resets PAC and simulation cleanly

    // === Phase 2: Apply PAC + Start Simulation ===
    TArray<FName> BoneNames;
    SkeletalMesh->GetBoneNames(BoneNames);

    TArray<FName> AffectedBones;

    for (const FName& Bone : BoneNames) {
        if (!IsBoneValidForChain(Bone, BoneName))
            continue;

        // Compare and apply PAC if different or new
        const FPhysicalAnimationData* ExistingPAC = ActiveBonePACData.Find(Bone);
        if (!ExistingPAC || !ArePACProfilesEqual(*ExistingPAC, ProfileData) || !IsBoneBlending(Bone)) {
            const FPhysicalAnimationData ScaledProfile = GetScaledPACProfile(Bone, ProfileData);
            ApplySimSettingsToBone(Bone, ScaledProfile);
            ActiveBonePACData.Add(Bone, ScaledProfile);
        }

        // Start simulation with ref count tracking
        if (TryActivateSimForBone(Bone, StartAlpha)) {
            AffectedBones.Add(Bone);
        }
    }

    // === Phase 3: Apply optional impulse ===
    if (!OptionalImpulseDir.IsNearlyZero() && OptionalImpulseStrength > 0.f) {
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
            if (Body->IsValidBodyInstance() && Body->IsInstanceSimulatingPhysics()) {
                const FVector SafeImpulse = OptionalImpulseDir.GetSafeNormal() * OptionalImpulseStrength;
                Body->AddImpulse(SafeImpulse, true);
            }
        }
    }

    // === Phase 4: Register Blend State ===
    const FActivePhysicsBlend Blend =
        CreatePhysicsBlendState(BoneName, StartAlpha, BlendIn, Hold, BlendOut, ReactionTag);

    ActiveBlends.Add(BoneName, Blend);

    UE_LOG(LogTemp, Log, TEXT("[OH] Hit reaction: %s | In=%.2f Hold=%.2f Out=%.2f | Alpha=%.2f | Bones=%d"),
           *BoneName.ToString(), BlendIn, Hold, BlendOut, StartAlpha, AffectedBones.Num());

    // === Phase 5: Output affected bones (optional)
    if (OutAffectedBones) {
        *OutAffectedBones = AffectedBones;
    }

    // === Phase 6: (Optional) Event Broadcast
    if (AffectedBones.Num() > 0) {
        OnHitReactionBonesUpdated.Broadcast(BoneName, AffectedBones);
    }
}
#pragma endregion

#pragma region Impulse

void UOHPhysicsManager::ApplyImpulseToBone(FName Bone, const FVector& Direction, float Magnitude) {
    if (!SkeletalMesh || !SkeletalMesh->IsSimulatingPhysics(Bone)) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Skipping impulse: Bone %s not simulating."),
               *Bone.ToString());
        return;
    }

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone);
    if (!Body)
        return;

    FOHBoneData* BoneData = TrackedBoneDataMap.Find(Bone);
    if (!BoneData)
        return;

    // Optional: wake up
    Body->WakeInstance();

    // Scale impulse based on mass/damping
    const float Scale = BoneData->GetImpulseScaleFactor();
    const FVector ScaledImpulse = Direction.GetSafeNormal() * Magnitude * Scale;

    Body->AddImpulse(ScaledImpulse, true);

    UE_LOG(LogTemp, Verbose, TEXT("[OHPhysicsManager] Applied impulse to %s: %s (raw %.2f, scaled %.2f)"),
           *Bone.ToString(), *ScaledImpulse.ToString(), Magnitude, ScaledImpulse.Size());
}

void UOHPhysicsManager::PropagateImpulse(FName OriginBone, FVector Impulse, float Strength, EOHPhysicsProfile Profile,
                                         int32 Depth) {
    if (Depth <= 0 || Strength <= KINDA_SMALL_NUMBER)
        return;

    if (!BoneLinkMap.Contains(OriginBone))
        return;
    const FOHPhysicsBoneLink& Link = BoneLinkMap[OriginBone];

    for (const FName& Child : Link.Children) {
        float NextStrength = Strength * Link.PropagationAttenuation;
        if (NextStrength <= 1.f)
            continue;

        ApplyImpulseToBone(Child, Impulse, NextStrength);

        // Recursive propagate
        PropagateImpulse(Child, Impulse, NextStrength, Profile, Depth - 1);
    }
}
#pragma endregion

#pragma region SimulationLifecycle

/*void UOHPhysicsManager::ApplySimSettingsToBone(const FName& Bone, const FPhysicalAnimationData& BaseProfile)
{
    if (!PhysicalAnimationComponent || !SkeletalMesh) return;

    // === PAC Tweaks ===
    float PACMultiplier = 1.f, LinearDamping = 5.f, AngularDamping = 3.f;
    ComputePhysicsTweaksForBone(Bone, PACMultiplier, LinearDamping, AngularDamping);

    FPhysicalAnimationData ScaledProfile = BaseProfile;
    ScaledProfile.OrientationStrength *= PACMultiplier;
    ScaledProfile.PositionStrength *= PACMultiplier;
    ScaledProfile.VelocityStrength *= PACMultiplier;

    // Apply PAC
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, ScaledProfile);
    ActiveBonePACData.Add(Bone, ScaledProfile);

    // === Damping & Sim ===
    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone))
    {
        Body->SetInstanceSimulatePhysics(true);
        Body->LinearDamping = LinearDamping;
        Body->AngularDamping = AngularDamping;
    }

    // === Constraint Tuning ===
    if (UPhysicsAsset* PhysicsAsset = SkeletalMesh->GetPhysicsAsset())
    {
        const int32 ConstraintIndex = PhysicsAsset->FindConstraintIndex(Bone);
        if (ConstraintIndex != INDEX_NONE)
        {
            // Ensure the setup pointer is valid
            if (PhysicsAsset->ConstraintSetup.IsValidIndex(ConstraintIndex) &&
                PhysicsAsset->ConstraintSetup[ConstraintIndex] != nullptr)
            {
                FConstraintInstance& Constraint = PhysicsAsset->ConstraintSetup[ConstraintIndex]->DefaultInstance;
                FConstraintProfileProperties Profile = Constraint.ProfileInstance;

                ComputeOptimalConstraintSettings(&Constraint, Profile, Bone);
                Constraint.ProfileInstance = Profile;

                UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] [%s] Applied PAC x%.2f | Damping L %.2f A %.2f |
Constraint tuned to %s"), *GetOwner()->GetName(), PACMultiplier, LinearDamping, AngularDamping, *Bone.ToString());
                return;
            }
        }
    }

    // Fallback log if constraint not applied
    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] [%s] Applied PAC x%.2f | Damping L %.2f A %.2f to %s (No
constraint)"), *GetOwner()->GetName(), PACMultiplier, LinearDamping, AngularDamping, *Bone.ToString());
}*/

void UOHPhysicsManager::ApplySimSettingsToBone(const FName& Bone, const FPhysicalAnimationData& BaseProfile) {
    if (!PhysicalAnimationComponent || !SkeletalMesh)
        return;

    // === PAC Tweaks ===
    float PACMultiplier = 1.f, LinearDamping = 5.f, AngularDamping = 3.f;
    ComputePhysicsTweaksForBone(Bone, PACMultiplier, LinearDamping, AngularDamping);

    FPhysicalAnimationData ScaledProfile = BaseProfile;
    ScaledProfile.OrientationStrength *= PACMultiplier;
    ScaledProfile.PositionStrength *= PACMultiplier;
    ScaledProfile.VelocityStrength *= PACMultiplier;

    // Apply PAC
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, ScaledProfile);
    ActiveBonePACData.Add(Bone, ScaledProfile);

    // === Body Simulation + Damping ===
    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
        Body->SetInstanceSimulatePhysics(true); // Make sure physics is actually simulating
        Body->LinearDamping = LinearDamping;
        Body->AngularDamping = AngularDamping;
        Body->UpdateDampingProperties();
        Body->UpdateMassProperties(); // Optional: forces inertia/mass recalculation
    }

    // === Constraint Tuning (Runtime Instance) ===
    if (UPhysicsAsset* PhysicsAsset = SkeletalMesh->GetPhysicsAsset()) {
        const int32 ConstraintIndex = PhysicsAsset->FindConstraintIndex(Bone);
        if (ConstraintIndex != INDEX_NONE && PhysicsAsset->ConstraintSetup.IsValidIndex(ConstraintIndex) &&
            PhysicsAsset->ConstraintSetup[ConstraintIndex] != nullptr) {
            FConstraintInstance& TemplateConstraint = PhysicsAsset->ConstraintSetup[ConstraintIndex]->DefaultInstance;
            FConstraintProfileProperties Profile = TemplateConstraint.ProfileInstance;

            // Compute new profile
            ComputeOptimalConstraintSettings(&TemplateConstraint, Profile, Bone);

            // Apply to runtime instance
            if (FConstraintInstance* RuntimeCI = SkeletalMesh->FindConstraintInstance(TemplateConstraint.JointName)) {
                RuntimeCI->ProfileInstance = Profile;

                // Motion modes
                RuntimeCI->SetAngularSwing1Motion(Profile.ConeLimit.Swing1Motion);
                RuntimeCI->SetAngularSwing2Motion(Profile.ConeLimit.Swing2Motion);
                RuntimeCI->SetAngularTwistMotion(Profile.TwistLimit.TwistMotion);

                // Enable SLERP drive
                RuntimeCI->SetAngularDriveMode(EAngularDriveMode::SLERP);
                RuntimeCI->SetAngularDriveParams(Profile.ConeLimit.Stiffness, Profile.ConeLimit.Damping,
                                                 0.f // ForceLimit (0 = unlimited)
                );

                // Enforce limit updates
                RuntimeCI->UpdateAngularLimit();
            }

            UE_LOG(LogTemp, Log,
                   TEXT("[OHPhysicsManager] [%s] Applied PAC x%.2f | Damping L %.2f A %.2f | Constraint tuned"),
                   *Bone.ToString(), PACMultiplier, LinearDamping, AngularDamping);
            return;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] [%s] Applied PAC x%.2f | Damping L %.2f A %.2f (No constraint)"),
           *Bone.ToString(), PACMultiplier, LinearDamping, AngularDamping);
}

void UOHPhysicsManager::ActivateSimulationForChain(FName RootBone, const FPhysicalAnimationData& ProfileData,
                                                   float StartAlpha) {
    if (!SkeletalMesh || !PhysicalAnimationComponent)
        return;

    TArray<FName> BoneNames;
    SkeletalMesh->GetBoneNames(BoneNames);

    for (const FName& Bone : BoneNames) {
        if (!IsBoneValidForChain(Bone, RootBone))
            continue;

        // Compare with existing PAC profile if one is active
        const FPhysicalAnimationData* ExistingPAC = ActiveBonePACData.Find(Bone);

        if (!ExistingPAC || !ArePACProfilesEqual(*ExistingPAC, ProfileData) || !IsBoneBlending(Bone)) {
            const FPhysicalAnimationData ScaledProfile = GetScaledPACProfile(Bone, ProfileData);
            ApplySimSettingsToBone(Bone, ScaledProfile);
            ActiveBonePACData.Add(Bone, ScaledProfile);
        }

        TryActivateSimForBone(Bone, StartAlpha);
    }
}

bool UOHPhysicsManager::TryActivateSimForBone(FName BoneName, float BlendAlpha) {
    if (!SkeletalMesh || !IsBoneValidForChain(BoneName, BoneName))
        return false;

    int32& RefCount = BoneSimBlendRefCount.FindOrAdd(BoneName);
    RefCount = FMath::Clamp(RefCount + 1, 1, 999);

    if (RefCount == 1) {
        return ActivatePhysicsStateForBone(BoneName, BlendAlpha);
    }

    return false;
}

bool UOHPhysicsManager::TryDeactivateSimForBone(FName BoneName) {
    int32* RefCountPtr = BoneSimBlendRefCount.Find(BoneName);
    if (!RefCountPtr)
        return false;

    (*RefCountPtr)--;

    if (*RefCountPtr <= 0) {
        BoneSimBlendRefCount.Remove(BoneName);
        ClearPhysicsStateForBone(BoneName);
        return true;
    }

    return false;
}

bool UOHPhysicsManager::ActivatePhysicsStateForBone(FName BoneName, float BlendAlpha) {
    if (!SkeletalMesh)
        return false;

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] Bone '%s' has invalid body instance"), *BoneName.ToString());
        return false;
    }

    ResetBoneForces(BoneName);
    Body->SetInstanceSimulatePhysics(true);
    Body->PhysicsBlendWeight = BlendAlpha;
    Body->WakeInstance();

    return true;
}

void UOHPhysicsManager::ClearPhysicsStateForBone(FName Bone) {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] ClearPhysicsStateForBone: Missing skeletal mesh or PAC"));
        return;
    }

    // Ensure bone exists in skeleton
    if (SkeletalMesh->GetBoneIndex(Bone) == INDEX_NONE) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] Bone '%s' not found in skeleton"), *Bone.ToString());
        return;
    }

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone);
    if (!Body || !Body->IsValidBodyInstance()) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] Bone '%s' has invalid or missing body instance"), *Bone.ToString());
        return;
    }

    if (Body->GetBodyMass() <= KINDA_SMALL_NUMBER) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] Bone '%s' has near-zero mass; skipping clear"), *Bone.ToString());
        return;
    }

    // Disable physics sim
    if (Body->IsInstanceSimulatingPhysics()) {
        Body->SetInstanceSimulatePhysics(false);
    }

    // Reset velocity, forces, and wake state
    Body->SetLinearVelocity(FVector::ZeroVector, false);
    Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
    Body->PutInstanceToSleep();

    // Clear PAC
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(Bone, FPhysicalAnimationData());

    // Cleanup internal tracking
    ActiveBonePACData.Remove(Bone);
    BoneSimBlendRefCount.Remove(Bone); // Defensive, though handled in TryDeactivate

#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
    if (Body->PhysicsBlendWeight > 0.01f) {
        UE_LOG(LogTemp, Warning, TEXT("[OH] Bone '%s' still has non-zero blend weight after clear"), *Bone.ToString());
    }
#endif
}

void UOHPhysicsManager::ClearAllSimulatedBones() {
    if (!SkeletalMesh)
        return;

    TArray<FName> BoneNames;
    SkeletalMesh->GetBoneNames(BoneNames);

    for (const FName& Bone : BoneNames) {
        if (BoneSimBlendRefCount.Contains(Bone) || ActiveBonePACData.Contains(Bone)) {
            ClearPhysicsStateForBone(Bone);
        }
    }

    ActiveBlends.Empty();
    BoneSimBlendRefCount.Empty();
    ActiveBonePACData.Empty();
}

void UOHPhysicsManager::ResetBoneForces(FName Bone) {
    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
        if (Body->IsValidBodyInstance()) {
            Body->SetLinearVelocity(FVector::ZeroVector, false);
            Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        }
    }
}

void UOHPhysicsManager::ApplyUnifiedPhysicsProfile(FName BoneName, const FPhysicalAnimationData& Profile,
                                                   float CustomLinearDamping, float CustomAngularDamping,
                                                   bool bVerboseLog) {
    if (!SkeletalMesh || !PhysicalAnimationComponent) {
        UE_LOG(LogTemp, Warning, TEXT("[ApplyUnifiedPhysicsProfile] Missing SkeletalMesh or PAC"));
        return;
    }

    // === Apply PAC Settings ===
    PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);

    if (bVerboseLog) {
        UE_LOG(LogTemp, Log, TEXT("[ApplyUnifiedPhysicsProfile] PAC applied to bone: %s"), *BoneName.ToString());
    }

    // === Apply Body Instance Settings ===
    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
        Body->LinearDamping = CustomLinearDamping;
        Body->AngularDamping = CustomAngularDamping;

        // Apply updated damping to simulation
        Body->UpdateDampingProperties();

        // Optional: force mass property update (if mass customization added)
        Body->UpdateMassProperties();

        if (bVerboseLog) {
            UE_LOG(LogTemp, Log, TEXT("  - Damping set [Linear: %.2f | Angular: %.2f]"), CustomLinearDamping,
                   CustomAngularDamping);
        }
    }

    // === Apply Constraint Tuning ===
    if (UPhysicsAsset* PhysicsAsset = SkeletalMesh->GetPhysicsAsset()) {
        for (const UPhysicsConstraintTemplate* Template : PhysicsAsset->ConstraintSetup) {
            if (!Template)
                continue;

            const FConstraintInstance& TemplateInstance = Template->DefaultInstance;
            const FName ConstraintBone = TemplateInstance.ConstraintBone1;

            if (ConstraintBone == BoneName) {
                if (FConstraintInstance* Instance =
                        SkeletalMesh->FindConstraintInstance(Template->DefaultInstance.JointName)) {
                    const float Stiffness = Profile.OrientationStrength;
                    const float Damping = Profile.PositionStrength;

                    Instance->SetAngularDriveMode(EAngularDriveMode::SLERP);
                    Instance->SetAngularDriveParams(Stiffness, Damping, 0.f); // Unlimited force

                    Instance->ProfileInstance.ConeLimit.Swing1Motion = ACM_Limited;
                    Instance->ProfileInstance.ConeLimit.Swing2Motion = ACM_Limited;
                    Instance->ProfileInstance.TwistLimit.TwistMotion = ACM_Limited;

                    Instance->UpdateAngularLimit();

                    if (bVerboseLog) {
                        UE_LOG(LogTemp, Log, TEXT("  - Constraint '%s' updated [Stiffness: %.1f | Damping: %.1f]"),
                               *Instance->JointName.ToString(), Stiffness, Damping);
                    }
                }
            }
        }
    }
}

void UOHPhysicsManager::ApplyUnifiedPhysicsProfileToChain(FName RootBone, const FPhysicalAnimationData& Profile,
                                                          float CustomLinearDamping, float CustomAngularDamping,
                                                          bool bVerboseLog) {
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Warning, TEXT("[ApplyUnifiedPhysicsProfileToChain] SkeletalMesh is null"));
        return;
    }

    TQueue<FName> BoneQueue;
    BoneQueue.Enqueue(RootBone);

    while (!BoneQueue.IsEmpty()) {
        FName CurrentBone;
        BoneQueue.Dequeue(CurrentBone);

        // Apply profile to this bone
        ApplyUnifiedPhysicsProfile(CurrentBone, Profile, CustomLinearDamping, CustomAngularDamping, bVerboseLog);

        // Find children and enqueue them
        const int32 BoneIndex = SkeletalMesh->GetBoneIndex(CurrentBone);
        if (BoneIndex == INDEX_NONE)
            continue;

        const FReferenceSkeleton& RefSkel = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
        const int32 NumBones = RefSkel.GetNum();

        for (int32 i = 0; i < NumBones; ++i) {
            if (RefSkel.GetParentIndex(i) == BoneIndex) {
                const FName ChildBone = RefSkel.GetBoneName(i);
                BoneQueue.Enqueue(ChildBone);
            }
        }
    }
}
#pragma endregion

#pragma region BoneValidation

bool UOHPhysicsManager::IsBoneValidForChain(FName Bone, FName RootBone) const {
    if (!SkeletalMesh)
        return false;
    return IsBoneMassValid(Bone) && IsBoneNamePatternValid(Bone) && IsBoneInChain(Bone, RootBone);
}

bool UOHPhysicsManager::IsBoneMassValid(FName BoneName) const {
    if (!SkeletalMesh)
        return false;

    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName)) {
        return Body->IsValidBodyInstance() && Body->GetBodyMass() > KINDA_SMALL_NUMBER;
    }

    return false;
}

bool UOHPhysicsManager::IsBoneNamePatternValid(FName BoneName) {
    const FString BoneNameStr = BoneName.ToString().ToLower();

    // Add filters for commonly problematic bones
    if (BoneNameStr.Contains(TEXT("twist")) || BoneNameStr.Contains(TEXT("ik")) ||
        BoneNameStr.Contains(TEXT("attach")) || BoneNameStr.Contains(TEXT("helper")) ||
        BoneNameStr.StartsWith(TEXT("vb_")) || BoneNameStr.EndsWith(TEXT("_dummy"))) {
        return false; // Bone is disallowed based on pattern
    }

    return true; // Bone name passes checks
}

bool UOHPhysicsManager::IsBoneInChain(FName Bone, FName RootBone) const {
    if (!SkeletalMesh)
        return false;
    return Bone == RootBone || SkeletalMesh->BoneIsChildOf(Bone, RootBone);
}

bool UOHPhysicsManager::IsBoneSimulatable(const FName& Bone) const {
    return SimulatableBones.Contains(Bone);
}

bool UOHPhysicsManager::ShouldSkipBone(const FName& Bone) const {
    return !SimulatableBones.Contains(Bone);
}

bool UOHPhysicsManager::HasPhysicsBody(const FName& BoneName) const {
    UPhysicsAsset* Asset =
        CachedPhysicsAsset ? CachedPhysicsAsset : (SkeletalMesh ? SkeletalMesh->GetPhysicsAsset() : nullptr);
    return Asset && Asset->FindBodyIndex(BoneName) != INDEX_NONE;
}

#pragma endregion

#pragma region Debug

void UOHPhysicsManager::DrawDebugPACValues() const {
    if (!SkeletalMesh)
        return;

    const FVector Offset(0.f, 0.f, 20.f); // small vertical offset for readability

    for (const FName& Bone : SimulatableBones) {
        const int32 BoneIndex = SkeletalMesh->GetBoneIndex(Bone);
        if (BoneIndex == INDEX_NONE)
            continue;

        const FVector BoneLocation = SkeletalMesh->GetBoneLocation(Bone);

        const FPhysicalAnimationData* Profile = ActiveBonePACData.Find(Bone);
        if (!Profile)
            continue;

        const FString DebugText =
            FString::Printf(TEXT("%s\nOri: %.1f Pos: %.1f\nVel: %.1f AngVel: %.1f\nMaxF: %.0f/%.0f"), *Bone.ToString(),
                            Profile->OrientationStrength, Profile->PositionStrength, Profile->VelocityStrength,
                            Profile->AngularVelocityStrength, Profile->MaxLinearForce, Profile->MaxAngularForce);

        FColor StrengthColor = FColor::Green;

        const float TotalStrength = Profile->OrientationStrength + Profile->PositionStrength +
                                    Profile->VelocityStrength + Profile->AngularVelocityStrength;

        if (TotalStrength > 2000.f)
            StrengthColor = FColor::Red;
        else if (TotalStrength > 1000.f)
            StrengthColor = FColor::Yellow;

        DrawDebugString(GetWorld(), BoneLocation + Offset, DebugText, nullptr, StrengthColor, 0.f, false);
    }
}

void UOHPhysicsManager::LogSimulatableBones() const {
    static bool bWasEmptyLastTick = false;

    AActor* Owner = GetOwner();
    const FString OwnerName = Owner ? Owner->GetName() : TEXT("UnknownOwner");

    UE_LOG(LogTemp, VeryVerbose, TEXT("[OHPhysicsManager] [%s] LogSimulatableBones called on frame %d, list size: %d"),
           *OwnerName, GFrameNumber, SimulatableBones.Num());

    if (SimulatableBones.Num() == 0) {
        if (!bWasEmptyLastTick) {
            UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] [%s] No bones in SimulatableBones list."), *OwnerName);
            bWasEmptyLastTick = true;
        }
        return;
    }

    bWasEmptyLastTick = false;

    FString BoneList;
    for (const FName& Bone : SimulatableBones) {
        BoneList += Bone.ToString() + TEXT(", ");
    }

    if (BoneList.Len() > 2) {
        BoneList.LeftChopInline(2);
    }

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] [%s] SimulatableBones (%d): [%s]"), *OwnerName,
           SimulatableBones.Num(), *BoneList);
}

void UOHPhysicsManager::EditorValidateSkeletonCompatibility() {
#if WITH_EDITOR
    if (!SkeletalMesh) {
        UE_LOG(LogTemp, Error, TEXT("No skeletal mesh component found!"));
        return;
    }

    FString Report = TEXT("=== Skeleton Validation Report ===\n\n");

    ValidateBoneMappings(false); // Don't log, we'll make custom report

    Report += FString::Printf(TEXT("Skeleton: %s\n"), *SkeletalMesh->GetSkeletalMeshAsset()->GetName());

    Report +=
        FString::Printf(TEXT("Valid Bones: %d / %d\n\n"), ValidatedBoneMapping.Num(), TrackedBoneDefinitions.Num());

    if (MissingBones.Num() > 0) {
        Report += TEXT("❌ Missing Bones:\n");
        for (const FName& Missing : MissingBones) {
            Report += FString::Printf(TEXT("  • %s"), *Missing.ToString());
            if (BoneSuggestions.Contains(Missing)) {
                Report += FString::Printf(TEXT(" → Try '%s'"), *BoneSuggestions[Missing].ToString());
            }
            Report += TEXT("\n");
        }
        Report += TEXT("\n");
    }

    Report += FString::Printf(TEXT("✅ Simulatable Bones: %d\n"), SimulatableBones.Num());

    // Show in message dialog
    FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(Report));
#endif
}

#pragma region Debug_Special
//================================================================
// OHPhysicsManager.cpp - Debug Implementation
//================================================================

void UOHPhysicsManager::DebugTick(float DeltaTime) {
    if (!bEnableDebugDisplay || !GetWorld())
        return;

    // Reset frame data
    CurrentFrameDebugData.Reset();
    CurrentFrameDebugData.LastFrameTime = GetWorld()->GetRealTimeSeconds();

    // Gather performance data
    PerformSystemComparison();

    // Display requested debug info
    if (bShowSystemOverview)
        DisplaySystemOverview();

    if (bShowPhysicsGraphAnalysis)
        DisplayPhysicsGraphAnalysis();

    if (bShowDirectPointerComparison)
        DisplayDirectPointerComparison();

    if (bShowPerformanceMetrics)
        DisplayPerformanceMetrics();

    if (bShowBoneDetails)
        DisplayBoneDetails();

    if (bShowConstraintDetails)
        DisplayConstraintDetails();

    // Store frame data for history
    if (DebugHistory.Num() > 60) // Keep 1 second of history at 60fps
        DebugHistory.RemoveAt(0);
    DebugHistory.Add(CurrentFrameDebugData);
}

void UOHPhysicsManager::RebuildDirectPointerMapsForTesting() {
    if (!SkeletalMesh || !CachedPhysicsAsset)
        return;

    DirectBodySetupMap.Reset();
    DirectConstraintTemplateMap.Reset();
    DirectConstraintInstanceMap.Reset();
    DirectBoneIndexMap.Reset();
    DirectBodyInstanceMap.Reset();

    StartDirectOperationTiming();

    // Build direct body pointer map
    TArray<FName> AllBoneNames;
    SkeletalMesh->GetBoneNames(AllBoneNames);

    for (const FName& BoneName : AllBoneNames) {
        // Direct bone index lookup
        int32 BoneIndex = SkeletalMesh->GetBoneIndex(BoneName);
        if (BoneIndex != INDEX_NONE) {
            DirectBoneIndexMap.Add(BoneName, BoneIndex);
        }

        // Direct body instance lookup (component data, safe to store raw pointer)
        FBodyInstance* Body = SkeletalMesh->GetBodyInstance(BoneName);
        if (Body && Body->IsValidBodyInstance()) {
            DirectBodyInstanceMap.Add(BoneName, Body);
        }

        // Store body setup via weak pointer for safety
        if (Body && Body->GetBodySetup()) {
            DirectBodySetupMap.Add(BoneName, Body->GetBodySetup());
        }
    }

    // Build direct constraint template AND instance maps
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : CachedPhysicsAsset->ConstraintSetup) {
        if (ConstraintTemplate) {
            FName Bone1 = ConstraintTemplate->DefaultInstance.ConstraintBone1;
            FName Bone2 = ConstraintTemplate->DefaultInstance.ConstraintBone2;
            FName JointName = ConstraintTemplate->DefaultInstance.JointName;

            // Store constraint templates
            if (!Bone1.IsNone())
                DirectConstraintTemplateMap.Add(Bone1, const_cast<UPhysicsConstraintTemplate*>(ConstraintTemplate));
            if (!Bone2.IsNone())
                DirectConstraintTemplateMap.Add(Bone2, const_cast<UPhysicsConstraintTemplate*>(ConstraintTemplate));

            // Store runtime constraint instances (the actual important ones!)
            if (FConstraintInstance* RuntimeInstance = SkeletalMesh->FindConstraintInstance(JointName)) {
                if (!Bone1.IsNone())
                    DirectConstraintInstanceMap.Add(Bone1, RuntimeInstance);
                if (!Bone2.IsNone())
                    DirectConstraintInstanceMap.Add(Bone2, RuntimeInstance);
            }
        }
    }

    EndDirectOperationTiming();

    // Calculate memory footprint more accurately
    CurrentFrameDebugData.DirectMapMemoryFootprint =
        DirectBodySetupMap.GetAllocatedSize() + DirectConstraintTemplateMap.GetAllocatedSize() +
        DirectConstraintInstanceMap.GetAllocatedSize() + DirectBoneIndexMap.GetAllocatedSize() +
        DirectBodyInstanceMap.GetAllocatedSize();
}

void UOHPhysicsManager::PerformSystemComparison() {
    if (!SkeletalMesh)
        return;

    // Rebuild direct maps for fresh comparison
    RebuildDirectPointerMapsForTesting();

    // Test graph system performance
    StartGraphOperationTiming();
    for (const auto& Pair : TrackedBoneDataMap) {
        const FName& BoneName = Pair.Key;

        // Graph-based access patterns using actual API
        if (const FOHBoneData* GraphBoneData = PhysicsGraph.GetBoneMap().Find(BoneName)) {
            CurrentFrameDebugData.GraphTraversalCount++;

            // Use actual FOHBoneData methods and consume results
            bool bIsSimulating = GraphBoneData->GetIsSimulating();
            FBodyInstance* Body = GraphBoneData->GetBodyInstance();
            float Mass = GraphBoneData->GetCachedBodyMass();

            // Track valid bones and use variables
            if (Body && Body->IsValidBodyInstance()) {
                CurrentFrameDebugData.ValidBoneCount++;
                // Use variables to prevent optimization and warnings
                if (bIsSimulating && Mass > 0.0f) {
                    // Variables consumed
                }
            } else {
                CurrentFrameDebugData.InvalidBoneCount++;
            }
        }
    }

    // Test graph constraint access patterns
    for (const FOHConstraintInstanceData& ConstraintData : PhysicsGraph.GetConstraintLinks()) {
        CurrentFrameDebugData.GraphTraversalCount++;

        // Test graph-based constraint instance access
        if (FConstraintInstance* GraphConstraintInstance = ConstraintData.GetConstraintInstance()) {
            CurrentFrameDebugData.ActiveConstraintCount++;

            // Simulate typical graph constraint operations and consume results
            float Swing1 = GraphConstraintInstance->GetCurrentSwing1();
            float Swing2 = GraphConstraintInstance->GetCurrentSwing2();
            bool bBreakable = GraphConstraintInstance->IsAngularBreakable();

            // Meaningfully use values (prevents both optimization and unused warnings)
            CurrentFrameDebugData.GraphMemoryFootprint += static_cast<int32>(Swing1 + Swing2) + (bBreakable ? 1 : 0);
        } else {
            CurrentFrameDebugData.InactiveConstraintCount++;
        }
    }

    EndGraphOperationTiming();

    // Test direct pointer system performance
    StartDirectOperationTiming();
    for (const auto& Pair : TrackedBoneDataMap) {
        const FName& BoneName = Pair.Key;

        // Direct pointer access patterns
        FBodyInstance* const* BodyPtr = DirectBodyInstanceMap.Find(BoneName);
        if (BodyPtr && *BodyPtr) {
            CurrentFrameDebugData.DirectLookupCount++;

            // Equivalent direct operations and consume results
            bool bIsSimulating = (*BodyPtr)->IsInstanceSimulatingPhysics();
            float Mass = (*BodyPtr)->GetBodyMass();
            bool bValidBody = (*BodyPtr)->IsValidBodyInstance();

            // Meaningfully use the values
            if (bIsSimulating && Mass > 0.0f && bValidBody) {
                CurrentFrameDebugData.DirectMapMemoryFootprint += static_cast<int32>(Mass);
            }
        }
    }

    // Test direct constraint instance access
    for (const auto& Pair : DirectConstraintInstanceMap) {
        CurrentFrameDebugData.DirectLookupCount++;

        if (FConstraintInstance* DirectConstraintInstance = Pair.Value) {
            // Equivalent direct constraint operations and consume results
            float Swing1 = DirectConstraintInstance->GetCurrentSwing1();
            float Swing2 = DirectConstraintInstance->GetCurrentSwing2();
            bool bBreakable = DirectConstraintInstance->IsAngularBreakable();

            // Meaningfully use values
            CurrentFrameDebugData.DirectMapMemoryFootprint +=
                static_cast<int32>(Swing1 + Swing2) + (bBreakable ? 1 : 0);
        }
    }

    EndDirectOperationTiming();

    // Calculate memory footprints using proper methods (this will overwrite the small additions above)
    CurrentFrameDebugData.GraphMemoryFootprint = CalculateGraphMemoryFootprint();

    // Validate consistency and store result
    bool bConsistencyValid = ValidateGraphVsDirectConsistency();
    if (!bConsistencyValid) {
        CurrentFrameDebugData.InvalidBoneCount++;
    }
}

void UOHPhysicsManager::DisplaySystemOverview() const {
    if (!GEngine)
        return;

    FString OverviewText = FString::Printf(
        TEXT("=== ONLYHANDS PHYSICS MANAGER DEBUG ===\n") TEXT("Skeletal Mesh: %s\n") TEXT("Physics Asset: %s\n")
            TEXT("Tracked Bones: %d | Simulatable: %d\n") TEXT("Active Blends: %d | Sim Ref Count Bones: %d\n")
                TEXT("Physics Graph Nodes: %d | Constraint Links: %d\n")
                    TEXT("Graph Constraint Instances: %d Active, %d Inactive\n")
                        TEXT("Direct Body Map: %d | Direct Constraint Templates: %d\n")
                            TEXT("Direct Constraint Instances: %d\n"),
        SkeletalMesh ? *SkeletalMesh->GetName() : TEXT("NULL"),
        CachedPhysicsAsset ? *CachedPhysicsAsset->GetName() : TEXT("NULL"), TrackedBoneDataMap.Num(),
        SimulatableBones.Num(), ActiveBlends.Num(), BoneSimBlendRefCount.Num(), PhysicsGraph.GetBoneMap().Num(),
        PhysicsGraph.GetConstraintLinks().Num(), CurrentFrameDebugData.ActiveConstraintCount,
        CurrentFrameDebugData.InactiveConstraintCount, DirectBodyInstanceMap.Num(), DirectConstraintTemplateMap.Num(),
        DirectConstraintInstanceMap.Num());

    GEngine->AddOnScreenDebugMessage(1001, DebugDisplayDuration, FColor::White, OverviewText, true,
                                     FVector2D(DebugTextScale, DebugTextScale));
}

void UOHPhysicsManager::DisplayPhysicsGraphAnalysis() const {
    if (!GEngine)
        return;

    // Analyze graph efficiency
    int32 GraphNodes = PhysicsGraph.GetBoneMap().Num();
    int32 GraphConstraints = PhysicsGraph.GetConstraintLinks().Num();
    int32 GraphMemoryKB = CurrentFrameDebugData.GraphMemoryFootprint / 1024;

    // Check for graph-specific issues
    TArray<FString> GraphIssues = GetGraphInconsistencies();

    FString AnalysisText =
        FString::Printf(TEXT("=== PHYSICS GRAPH ANALYSIS ===\n") TEXT("Graph Nodes: %d | Memory: %d KB\n")
                            TEXT("Constraint Links: %d\n") TEXT("Graph Traversals This Frame: %d\n")
                                TEXT("Graph Access Time: %.4f ms\n") TEXT("Graph Issues Found: %d\n"),
                        GraphNodes, GraphMemoryKB, GraphConstraints, CurrentFrameDebugData.GraphTraversalCount,
                        CurrentFrameDebugData.PhysicsGraphAccessTime * 1000.0f, GraphIssues.Num());

    if (GraphIssues.Num() > 0) {
        AnalysisText += TEXT("Issues: ");
        for (const FString& Issue : GraphIssues) {
            AnalysisText += Issue + TEXT(" | ");
        }
        AnalysisText += TEXT("\n");
    }

    GEngine->AddOnScreenDebugMessage(1002, DebugDisplayDuration, GraphIssues.Num() > 0 ? FColor::Yellow : FColor::Green,
                                     AnalysisText, true, FVector2D(DebugTextScale, DebugTextScale));
}

void UOHPhysicsManager::DisplayDirectPointerComparison() const {
    if (!GEngine)
        return;

    int32 DirectMemoryKB = CurrentFrameDebugData.DirectMapMemoryFootprint / 1024;
    int32 GraphMemoryKB = CurrentFrameDebugData.GraphMemoryFootprint / 1024;

    float PerformanceRatio =
        CurrentFrameDebugData.DirectPointerAccessTime > 0.0f
            ? CurrentFrameDebugData.PhysicsGraphAccessTime / CurrentFrameDebugData.DirectPointerAccessTime
            : 0.0f;

    bool bDirectIsFaster = CurrentFrameDebugData.DirectPointerAccessTime < CurrentFrameDebugData.PhysicsGraphAccessTime;
    bool bDirectUsesLessMemory = DirectMemoryKB < GraphMemoryKB;

    TArray<FString> DirectIssues = GetDirectPointerIssues();

    // Calculate constraint instance efficiency
    int32 GraphConstraintOps =
        CurrentFrameDebugData.ActiveConstraintCount + CurrentFrameDebugData.InactiveConstraintCount;
    int32 DirectConstraintOps = DirectConstraintInstanceMap.Num();
    float ConstraintEfficiencyRatio =
        DirectConstraintOps > 0 ? static_cast<float>(GraphConstraintOps) / static_cast<float>(DirectConstraintOps)
                                : 0.0f;

    FString ComparisonText = FString::Printf(
        TEXT("=== DIRECT POINTER vs GRAPH COMPARISON ===\n") TEXT("Direct Memory: %d KB | Graph Memory: %d KB\n")
            TEXT("Direct Lookups: %d | Graph Traversals: %d\n")
                TEXT("Direct Access Time: %.4f ms | Graph Access Time: %.4f ms\n")
                    TEXT("Performance Ratio (Graph/Direct): %.2fx\n")
                        TEXT("Constraint Efficiency (Graph/Direct Ops): %.2fx\n")
                            TEXT("Runtime Constraint Instances: Graph %d | Direct %d\n")
                                TEXT("Direct is Faster: %s | Uses Less Memory: %s\n") TEXT("Direct Issues: %d\n"),
        DirectMemoryKB, GraphMemoryKB, CurrentFrameDebugData.DirectLookupCount,
        CurrentFrameDebugData.GraphTraversalCount, CurrentFrameDebugData.DirectPointerAccessTime * 1000.0f,
        CurrentFrameDebugData.PhysicsGraphAccessTime * 1000.0f, PerformanceRatio, ConstraintEfficiencyRatio,
        GraphConstraintOps, DirectConstraintOps, bDirectIsFaster ? TEXT("YES") : TEXT("NO"),
        bDirectUsesLessMemory ? TEXT("YES") : TEXT("NO"), DirectIssues.Num());

    // Enhanced recommendation based on constraint instance analysis
    if (bDirectIsFaster && bDirectUsesLessMemory && DirectIssues.Num() == 0 && ConstraintEfficiencyRatio > 1.5f) {
        ComparisonText += TEXT("RECOMMENDATION: Strong case for Direct Pointers\n");
        ComparisonText += TEXT("  - Graph has significant constraint overhead\n");
    } else if (bDirectIsFaster && bDirectUsesLessMemory && DirectIssues.Num() == 0) {
        ComparisonText += TEXT("RECOMMENDATION: Switch to Direct Pointers\n");
    } else if (GraphMemoryKB > DirectMemoryKB * 2 || PerformanceRatio > 2.0f) {
        ComparisonText += TEXT("RECOMMENDATION: Consider Direct Pointers\n");
        ComparisonText += FString::Printf(
            TEXT("  - Graph overhead is significant (%.1fx)\n"),
            FMath::Max(PerformanceRatio, static_cast<float>(GraphMemoryKB) / static_cast<float>(DirectMemoryKB)));
    } else if (ConstraintEfficiencyRatio > 2.0f) {
        ComparisonText += TEXT("RECOMMENDATION: Graph constraint system is inefficient\n");
        ComparisonText += TEXT("  - Consider direct constraint instance access\n");
    } else {
        ComparisonText += TEXT("RECOMMENDATION: Graph system is reasonable\n");
    }

    FColor DisplayColor = (bDirectIsFaster && bDirectUsesLessMemory) ? FColor::Cyan : FColor::White;

    GEngine->AddOnScreenDebugMessage(1003, DebugDisplayDuration, DisplayColor, ComparisonText, true,
                                     FVector2D(DebugTextScale, DebugTextScale));
}

void UOHPhysicsManager::DisplayPerformanceMetrics() const {
    if (!GEngine || DebugHistory.Num() < 2)
        return;

    // Calculate averages over last 30 frames
    int32 SampleCount = FMath::Min(30, DebugHistory.Num());
    float AvgGraphTime = 0.0f;
    float AvgDirectTime = 0.0f;
    int32 AvgGraphTraversals = 0;
    int32 AvgDirectLookups = 0;

    for (int32 i = DebugHistory.Num() - SampleCount; i < DebugHistory.Num(); ++i) {
        const FDebugFrameData& Frame = DebugHistory[i];
        AvgGraphTime += Frame.PhysicsGraphAccessTime;
        AvgDirectTime += Frame.DirectPointerAccessTime;
        AvgGraphTraversals += Frame.GraphTraversalCount;
        AvgDirectLookups += Frame.DirectLookupCount;
    }

    AvgGraphTime /= SampleCount;
    AvgDirectTime /= SampleCount;
    AvgGraphTraversals /= SampleCount;
    AvgDirectLookups /= SampleCount;

    FString MetricsText =
        FString::Printf(TEXT("=== PERFORMANCE METRICS (30-frame avg) ===\n") TEXT("Graph Access Time: %.4f ms (avg)\n")
                            TEXT("Direct Access Time: %.4f ms (avg)\n") TEXT("Graph Traversals: %d (avg)\n")
                                TEXT("Direct Lookups: %d (avg)\n") TEXT("Operations per ms (Graph): %.1f\n")
                                    TEXT("Operations per ms (Direct): %.1f\n"),
                        AvgGraphTime * 1000.0f, AvgDirectTime * 1000.0f, AvgGraphTraversals, AvgDirectLookups,
                        AvgGraphTime > 0.0f ? AvgGraphTraversals / (AvgGraphTime * 1000.0f) : 0.0f,
                        AvgDirectTime > 0.0f ? AvgDirectLookups / (AvgDirectTime * 1000.0f) : 0.0f);

    GEngine->AddOnScreenDebugMessage(1004, DebugDisplayDuration, FColor::Magenta, MetricsText, true,
                                     FVector2D(DebugTextScale, DebugTextScale));
}

// === HELPER IMPLEMENTATIONS ===

void UOHPhysicsManager::StartGraphOperationTiming() const {
    GraphOperationStartTime = FPlatformTime::Seconds();
}

void UOHPhysicsManager::EndGraphOperationTiming() const {
    CurrentFrameDebugData.PhysicsGraphAccessTime = FPlatformTime::Seconds() - GraphOperationStartTime;
}

void UOHPhysicsManager::StartDirectOperationTiming() const {
    DirectOperationStartTime = FPlatformTime::Seconds();
}

void UOHPhysicsManager::EndDirectOperationTiming() const {
    CurrentFrameDebugData.DirectPointerAccessTime = FPlatformTime::Seconds() - DirectOperationStartTime;
}

bool UOHPhysicsManager::ValidateGraphVsDirectConsistency() const {
    bool bConsistent = true;

    // Check if graph has bones that direct system doesn't
    for (const auto& Pair : PhysicsGraph.GetBoneMap()) {
        const FName& BoneName = Pair.Key;
        if (!DirectBodyInstanceMap.Contains(BoneName)) {
            CurrentFrameDebugData.InvalidBoneCount++;
            bConsistent = false;
        } else {
            CurrentFrameDebugData.ValidBoneCount++;
        }
    }

    return bConsistent;
}

TArray<FString> UOHPhysicsManager::GetDirectPointerIssues() const {
    TArray<FString> Issues;

    // Check for null pointers in direct maps
    int32 NullBodyPointers = 0;
    for (const auto& Pair : DirectBodyInstanceMap) {
        if (!Pair.Value || !Pair.Value->IsValidBodyInstance()) {
            NullBodyPointers++;
        }
    }

    if (NullBodyPointers > 0)
        Issues.Add(FString::Printf(TEXT("NullBodyPointers(%d)"), NullBodyPointers));

    // Check for stale weak references in constraint templates
    int32 StaleTemplateReferences = 0;
    for (const auto& Pair : DirectConstraintTemplateMap) {
        if (!Pair.Value.IsValid()) {
            StaleTemplateReferences++;
        }
    }

    if (StaleTemplateReferences > 0)
        Issues.Add(FString::Printf(TEXT("StaleTemplateRefs(%d)"), StaleTemplateReferences));

    // Check for null constraint instances (this is the critical check!)
    int32 NullConstraintInstances = 0;
    for (const auto& Pair : DirectConstraintInstanceMap) {
        if (!Pair.Value) {
            NullConstraintInstances++;
        }
    }

    if (NullConstraintInstances > 0)
        Issues.Add(FString::Printf(TEXT("NullConstraintInstances(%d)"), NullConstraintInstances));

    // Check for mismatched template vs instance counts
    int32 TemplateMismatch = FMath::Abs(DirectConstraintTemplateMap.Num() - DirectConstraintInstanceMap.Num());
    if (TemplateMismatch > 0)
        Issues.Add(FString::Printf(TEXT("TemplateInstanceMismatch(%d)"), TemplateMismatch));

    return Issues;
}

TArray<FString> UOHPhysicsManager::GetGraphInconsistencies() const {
    TArray<FString> Issues;

    // Check for orphaned nodes
    int32 OrphanedNodes = 0;
    for (const auto& Pair : PhysicsGraph.GetBoneMap()) {
        if (!SkeletalMesh->DoesSocketExist(Pair.Key)) {
            OrphanedNodes++;
        }
    }

    if (OrphanedNodes > 0)
        Issues.Add(FString::Printf(TEXT("OrphanedNodes(%d)"), OrphanedNodes));

    // Check for missing constraint links
    int32 MissingConstraints = 0;
    for (const auto& Constraint : PhysicsGraph.GetConstraintLinks()) {
        if (!PhysicsGraph.GetBoneMap().Contains(Constraint.GetParentBone()) ||
            !PhysicsGraph.GetBoneMap().Contains(Constraint.GetChildBone())) {
            MissingConstraints++;
        }
    }

    if (MissingConstraints > 0)
        Issues.Add(FString::Printf(TEXT("MissingConstraints(%d)"), MissingConstraints));

    // Check for bones without valid body instances (renamed to avoid class member collision)
    int32 InvalidBodyCount = 0;
    for (const auto& Pair : PhysicsGraph.GetBoneMap()) {
        const FOHBoneData& BoneData = Pair.Value;
        if (!BoneData.GetBodyInstance() || !BoneData.GetBodyInstance()->IsValidBodyInstance()) {
            InvalidBodyCount++;
        }
    }

    if (InvalidBodyCount > 0)
        Issues.Add(FString::Printf(TEXT("InvalidBodies(%d)"), InvalidBodyCount));

    return Issues;
}

void UOHPhysicsManager::DisplayBoneDetails() const {
    if (!GEngine || !bShowBoneDetails)
        return;

    FString BoneText = TEXT("=== BONE DETAILS ANALYSIS ===\n");

    // If specific bones are requested, analyze only those
    if (DebugSpecificBones.Num() > 0) {
        for (const FName& BoneName : DebugSpecificBones) {
            // Graph bone analysis
            if (const FOHBoneData* GraphBone = PhysicsGraph.GetBoneMap().Find(BoneName)) {
                FBodyInstance* Body = GraphBone->GetBodyInstance();
                bool bIsSimulating = GraphBone->GetIsSimulating();
                float Mass = GraphBone->GetCachedBodyMass();
                int32 MotionSamples = GraphBone->GetMotionHistory().Num();

                BoneText +=
                    FString::Printf(TEXT("%s: Mass=%.2f | Sim=%s | Samples=%d | Valid=%s\n"), *BoneName.ToString(),
                                    Mass, bIsSimulating ? TEXT("Yes") : TEXT("No"), MotionSamples,
                                    (Body && Body->IsValidBodyInstance()) ? TEXT("Yes") : TEXT("No"));
            } else {
                BoneText += FString::Printf(TEXT("%s: NOT FOUND IN GRAPH\n"), *BoneName.ToString());
            }

            // Direct bone analysis
            FBodyInstance* const* DirectBody = DirectBodyInstanceMap.Find(BoneName);
            if (DirectBody && *DirectBody) {
                float DirectMass = (*DirectBody)->GetBodyMass();
                bool bDirectSimulating = (*DirectBody)->IsInstanceSimulatingPhysics();

                BoneText += FString::Printf(TEXT("  Direct: Mass=%.2f | Sim=%s\n"), DirectMass,
                                            bDirectSimulating ? TEXT("Yes") : TEXT("No"));
            } else {
                BoneText += TEXT("  Direct: NOT FOUND\n");
            }
        }
    } else {
        // General bone analysis summary
        int32 GraphBones = PhysicsGraph.GetBoneMap().Num();
        int32 DirectBones = DirectBodyInstanceMap.Num();
        int32 SimulatingBones = 0;
        int32 BonesWithMotionHistory = 0;

        for (const auto& Pair : PhysicsGraph.GetBoneMap()) {
            if (Pair.Value.GetIsSimulating())
                SimulatingBones++;
            if (Pair.Value.HasMotionHistory())
                BonesWithMotionHistory++;
        }

        BoneText += FString::Printf(TEXT("Graph Bones: %d | Direct Bones: %d\n")
                                        TEXT("Simulating: %d | With Motion History: %d\n")
                                            TEXT("Use DebugSpecificBones array for detailed bone analysis\n"),
                                    GraphBones, DirectBones, SimulatingBones, BonesWithMotionHistory);
    }

    GEngine->AddOnScreenDebugMessage(1005, DebugDisplayDuration, FColor::Cyan, BoneText, true,
                                     FVector2D(DebugTextScale, DebugTextScale));
}

void UOHPhysicsManager::DisplayConstraintDetails() const {
    if (!GEngine || !bShowConstraintDetails)
        return;

    FString ConstraintText = TEXT("=== CONSTRAINT INSTANCE ANALYSIS ===\n");

    // Analyze graph constraint instances
    int32 GraphValidInstances = 0;
    int32 GraphNullInstances = 0;
    float GraphTotalStrain = 0.0f;

    for (const FOHConstraintInstanceData& ConstraintData : PhysicsGraph.GetConstraintLinks()) {
        if (FConstraintInstance* Instance = ConstraintData.GetConstraintInstance()) {
            GraphValidInstances++;

            // Get runtime strain data from both sources for comparison
            const FConstraintRuntimeState& RuntimeState = ConstraintData.GetRuntimeState();
            float GraphStrain = RuntimeState.GetStrain();

            // Also get direct strain from the constraint instance for validation
            FVector Force, Torque;
            Instance->GetConstraintForce(Force, Torque);
            float DirectStrain = Force.Size() + Torque.Size();

            // Use graph strain for totals, but validate against direct calculation
            GraphTotalStrain += GraphStrain;

            // Log significant discrepancies (optional validation)
            if (FMath::Abs(GraphStrain - DirectStrain) > 50.0f && bEnableDebugDisplay) {
                UE_LOG(LogTemp, Warning, TEXT("Constraint strain mismatch: Graph=%.1f Direct=%.1f"), GraphStrain,
                       DirectStrain);
            }
        } else {
            GraphNullInstances++;
        }
    }

    // Analyze direct constraint instances
    int32 DirectValidInstances = 0;
    int32 DirectNullInstances = 0;
    float DirectTotalStrain = 0.0f;

    for (const auto& Pair : DirectConstraintInstanceMap) {
        if (FConstraintInstance* Instance = Pair.Value) {
            DirectValidInstances++;

            // Calculate equivalent strain (force + torque magnitude)
            FVector Force, Torque;
            Instance->GetConstraintForce(Force, Torque);
            DirectTotalStrain += Force.Size() + Torque.Size();
        } else {
            DirectNullInstances++;
        }
    }

    float GraphAvgStrain = GraphValidInstances > 0 ? GraphTotalStrain / GraphValidInstances : 0.0f;
    float DirectAvgStrain = DirectValidInstances > 0 ? DirectTotalStrain / DirectValidInstances : 0.0f;

    ConstraintText += FString::Printf(
        TEXT("Graph Constraints: %d Valid, %d Null | Avg Strain: %.2f\n")
            TEXT("Direct Constraints: %d Valid, %d Null | Avg Strain: %.2f\n")
                TEXT("Instance Validity: Graph %.1f%% | Direct %.1f%%\n")
                    TEXT("Strain Difference: %.2f (Graph-Direct)\n"),
        GraphValidInstances, GraphNullInstances, GraphAvgStrain, DirectValidInstances, DirectNullInstances,
        DirectAvgStrain,
        GraphValidInstances > 0 ? (GraphValidInstances * 100.0f / (GraphValidInstances + GraphNullInstances)) : 0.0f,
        DirectValidInstances > 0 ? (DirectValidInstances * 100.0f / (DirectValidInstances + DirectNullInstances))
                                 : 0.0f,
        GraphAvgStrain - DirectAvgStrain);

    // Add constraint-specific recommendations
    if (GraphNullInstances > DirectNullInstances) {
        ConstraintText += TEXT("WARNING: Graph has more null constraint instances\n");
    }

    if (FMath::Abs(GraphAvgStrain - DirectAvgStrain) > 10.0f) {
        ConstraintText += TEXT("WARNING: Significant strain calculation differences\n");
    }

    FColor ConstraintColor = (GraphNullInstances > 0) ? FColor::Yellow : FColor::Green;

    GEngine->AddOnScreenDebugMessage(1006, DebugDisplayDuration, ConstraintColor, ConstraintText, true,
                                     FVector2D(DebugTextScale, DebugTextScale));
}

// Add constraint instance validation helper
bool UOHPhysicsManager::ValidateConstraintInstanceConsistency() const {
    bool bConsistent = true;

    // Check if graph constraint instances match direct ones
    for (const FOHConstraintInstanceData& ConstraintData : PhysicsGraph.GetConstraintLinks()) {
        FName ParentBone = ConstraintData.GetParentBone();
        FName ChildBone = ConstraintData.GetChildBone();

        FConstraintInstance* GraphInstance = ConstraintData.GetConstraintInstance();
        FConstraintInstance* DirectInstance1 = nullptr;
        FConstraintInstance* DirectInstance2 = nullptr;

        // Check both bones for direct instances
        if (FConstraintInstance* const* Found1 = DirectConstraintInstanceMap.Find(ParentBone))
            DirectInstance1 = *Found1;
        if (FConstraintInstance* const* Found2 = DirectConstraintInstanceMap.Find(ChildBone))
            DirectInstance2 = *Found2;

        // Validate consistency
        if (GraphInstance && (!DirectInstance1 && !DirectInstance2)) {
            bConsistent = false;
            UE_LOG(LogTemp, Warning, TEXT("Graph has constraint instance for %s-%s but direct system doesn't"),
                   *ParentBone.ToString(), *ChildBone.ToString());
        } else if (!GraphInstance && (DirectInstance1 || DirectInstance2)) {
            bConsistent = false;
            UE_LOG(LogTemp, Warning, TEXT("Direct system has constraint instance for %s-%s but graph doesn't"),
                   *ParentBone.ToString(), *ChildBone.ToString());
        }
    }

    return bConsistent;
}

// Add helper method for memory calculation
int32 UOHPhysicsManager::CalculateGraphMemoryFootprint() const {
    // Comprehensive memory usage calculation for the graph structure
    int32 TotalMemory = 0;

    // === CORE CONTAINER MEMORY ===
    // Size of bone map container itself
    TotalMemory += PhysicsGraph.GetBoneMap().GetAllocatedSize();

    // Size of constraint links container itself
    TotalMemory += PhysicsGraph.GetConstraintLinks().GetAllocatedSize();

    // === BONE DATA MEMORY (DETAILED) ===
    for (const auto& Pair : PhysicsGraph.GetBoneMap()) {
        const FOHBoneData& BoneData = Pair.Value;

        // Base bone data structure
        TotalMemory += sizeof(FOHBoneData);

        // Motion history memory (this is significant!)
        const TArray<FOHMotionSample>& MotionHistory = BoneData.GetMotionHistory();
        TotalMemory += MotionHistory.GetAllocatedSize();
        TotalMemory += MotionHistory.Num() * sizeof(FOHMotionSample);

        // Child bones array memory
        // Note: GetChildBones() returns a TArray, which has overhead
        TotalMemory += sizeof(TArray<FName>) + (sizeof(FName) * 4); // Estimate avg 4 children per bone
    }

    // === CONSTRAINT DATA MEMORY (DETAILED) ===
    int32 ConstraintCount = PhysicsGraph.GetConstraintLinks().Num();

    // Calculate constraint memory more efficiently without iterating
    TotalMemory += ConstraintCount * (sizeof(FOHConstraintInstanceData) + sizeof(FConstraintRuntimeState));

    // Add buffer for potential dynamic constraint data (strain history, etc.)
    TotalMemory += ConstraintCount * 32;

    // === GRAPH OVERHEAD ESTIMATION ===
    // Hash table overhead for bone map (approximation)
    TotalMemory += PhysicsGraph.GetBoneMap().Num() * 16; // Hash overhead per entry

    // Array overhead for constraint links
    TotalMemory += sizeof(TArray<FOHConstraintInstanceData>);

    // === CACHED DATA MEMORY ===
    // Add estimated memory for cached masses, lengths, etc.
    TotalMemory += PhysicsGraph.GetBoneMap().Num() * (sizeof(float) * 4); // Mass, length, and other cached values

    // === VALIDATION AND LOGGING ===
    if (bEnableDebugDisplay) {
        // In debug builds, add some buffer for debug data structures
        TotalMemory += ConstraintCount * 64;                 // Debug overhead per constraint
        TotalMemory += PhysicsGraph.GetBoneMap().Num() * 32; // Debug overhead per bone
    }

    return TotalMemory;
}

void UOHPhysicsManager::ToggleDebugMode(bool bNewEnabled) {
    bEnableDebugDisplay = bNewEnabled;
    if (bNewEnabled) {
        UE_LOG(LogTemp, Log, TEXT("OHPhysicsManager Debug Mode ENABLED"));
        UE_LOG(LogTemp, Log, TEXT("  - Monitoring both constraint templates AND runtime instances"));
        UE_LOG(LogTemp, Log, TEXT("  - Tracking graph vs direct pointer performance"));
        UE_LOG(LogTemp, Log, TEXT("  - Validating constraint instance consistency"));
    } else {
        UE_LOG(LogTemp, Log, TEXT("OHPhysicsManager Debug Mode DISABLED"));
    }
}

#pragma endregion
void UOHPhysicsManager::TickDebugOverlay() const {
    if (!bEnableDebugOverlay || !SkeletalMesh)
        return;

    if (bShowSimBlendOverlay) {
        DrawSimBlendDebugOverlay();
        // DrawDebugPACValues();
        // LogSimulatableBones();
    }

    if (bShowTrackedBoneKinematics) {
        DrawTrackedBoneKinematicsDebug();
    }
}

void UOHPhysicsManager::DrawSimBlendDebugOverlay() const {
    if (!SkeletalMesh || !GetWorld())
        return;

    for (const TPair<FName, FActivePhysicsBlend>& Elem : ActiveBlends) {
        const FActivePhysicsBlend& Blend = Elem.Value;
        const FName& RootBone = Blend.RootBone;

        TArray<FName> BoneNames;
        SkeletalMesh->GetBoneNames(BoneNames);

        for (const FName& Bone : BoneNames) {
            if (!IsBoneValidForChain(Bone, RootBone)) // ✅ NEW: skip non-simulatable bones
                continue;

            const FTransform BoneTransform = SkeletalMesh->GetSocketTransform(Bone, ERelativeTransformSpace::RTS_World);
            const FVector Location = BoneTransform.GetLocation();

            // Determine phase color
            FColor PhaseColor;
            switch (Blend.Phase) {
            case EOHBlendPhase::BlendIn:
                PhaseColor = FColor::Yellow;
                break;
            case EOHBlendPhase::Hold:
                PhaseColor = FColor::Green;
                break;
            case EOHBlendPhase::BlendOut:
                PhaseColor = FColor::Red;
                break;
            default:
                PhaseColor = FColor::White;
                break;
            }

            const float ScaledAlpha = FMath::Clamp(Blend.BlendAlpha, 0.0f, 1.0f);
            const FLinearColor LinearColor = FLinearColor(PhaseColor) * ScaledAlpha;
            const FColor FinalColor = LinearColor.ToFColor(true);

            DrawDebugSphere(GetWorld(), Location, 4.0f, 6, FinalColor, false, -1.f, 0, 0.75f);

            const FString Label = FString::Printf(TEXT("%s (%.2f)"), *Bone.ToString(), Blend.BlendAlpha);
            DrawDebugString(GetWorld(), Location + FVector(0, 0, 6.f), Label, nullptr, FinalColor, 0.0f, true);
        }
    }
}

void UOHPhysicsManager::DrawTrackedBoneKinematicsDebug() const {
    if (!bShowTrackedBoneKinematics || !SkeletalMesh)
        return;

    for (const auto& Pair : TrackedBoneDataMap) {
        const FName Bone = Pair.Key;
        const FOHBoneData& Data = Pair.Value;

        if (!Data.IsValid())
            continue;

        const FVector Pos = Data.GetCurrentPosition();
        const FVector Vel = Data.GetBodyLinearVelocity();
        const FVector Acc = Data.GetLinearAcceleration();
        const FVector AngVel = Data.GetAngularVelocity();

        // Label
        DrawDebugString(GetWorld(), Pos + FVector(0, 0, 15), *Bone.ToString(), nullptr, FColor::White, 0.f, true);

        // Velocity (Blue)
        DrawDebugLine(GetWorld(), Pos, Pos + Vel * DebugVelocityScale, FColor::Blue, false, 0.f, 0, 1.5f);

        // Acceleration (Green, highlight if large)
        const float AccMag = Acc.Size();
        const FColor AccColor = (AccMag > AccelerationAlertThreshold) ? FColor(150, 255, 150) : FColor::Green;

        DrawDebugLine(GetWorld(), Pos, Pos + Acc * DebugAccelerationScale, AccColor, false, 0.f, 0, 1.5f);

        // Angular Velocity (Red, highlight if large)
        const float AngMag = AngVel.Size();
        const FColor AngColor = (AngMag > AngularVelocityAlertThreshold) ? FColor(255, 100, 100) : FColor::Red;

        DrawDebugLine(GetWorld(), Pos, Pos + AngVel * DebugAngularVelocityScale, AngColor, false, 0.f, 0, 1.5f);
    }
}

void UOHPhysicsManager::PrintBoneTrackingInfo() const {
    UE_LOG(LogTemp, Log, TEXT("---- Tracked Bone Definitions (%d) ----"), TrackedBoneDefinitions.Num());
    for (const FName& Bone : TrackedBoneDefinitions) {
        UE_LOG(LogTemp, Log, TEXT("Tracked: %s"), *Bone.ToString());
    }

    UE_LOG(LogTemp, Log, TEXT("---- Sim Exclusions (%d) ----"), SimExclusionBoneSet.Num());
    for (const FName& Bone : SimExclusionBoneSet) {
        UE_LOG(LogTemp, Log, TEXT("Excluded: %s"), *Bone.ToString());
    }

    UE_LOG(LogTemp, Log, TEXT("---- Simulatable Bones (%d) ----"), SimulatableBones.Num());
    for (const FName& Bone : SimulatableBones) {
        UE_LOG(LogTemp, Log, TEXT("Simulatable: %s"), *Bone.ToString());
    }
}

const TMap<FName, FOHBoneData>& UOHPhysicsManager::GetTrackedBoneData() const {
    static const TMap<FName, FOHBoneData> EmptyMap;

    if (!this) {
        UE_LOG(LogTemp, Error, TEXT("UOHPhysicsManager::GetTrackedBoneData called on null instance."));
        return EmptyMap;
    }

    if (!GetOwner()) {
        UE_LOG(LogTemp, Warning, TEXT("UOHPhysicsManager::GetTrackedBoneData: Manager has no owning actor."));
        return EmptyMap;
    }

#if WITH_EDITOR
    if (TrackedBoneDataMap.Num() == 0) {
        UE_LOG(LogTemp, Warning,
               TEXT("UOHPhysicsManager::GetTrackedBoneData: TrackedBoneDataMap is empty. Tracking might not be "
                    "initialized."));
    }
#endif

    return TrackedBoneDataMap;
}

void UOHPhysicsManager::PrintSimulatedPhysicsBones() const {
    if (!SkeletalMesh)
        return;

    TArray<FName> BoneNames;
    SkeletalMesh->GetBoneNames(BoneNames);

    UE_LOG(LogTemp, Log, TEXT("==== Simulated Physics Bones ===="));
    for (const FName& Bone : BoneNames) {
        if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
            if (Body->IsInstanceSimulatingPhysics()) {
                const bool IsInSimList = SimulatableBones.Contains(Bone);
                UE_LOG(LogTemp, Log, TEXT("%s  [%s]"), *Bone.ToString(),
                       IsInSimList ? TEXT("Simulatable") : TEXT("UNEXPECTED"));
            }
        }
    }
}
FColor GetSuggestionColor(const FString& Suggestion) {
    if (Suggestion == "IncreaseAngularDamping")
        return FColor::Cyan;
    if (Suggestion == "IncreaseLinearDamping")
        return FColor::Green;
    if (Suggestion == "IncreasePositionStrength")
        return FColor::Yellow;
    if (Suggestion == "IncreaseOrientationStrength")
        return FColor::Orange;
    if (Suggestion == "TightenConstraintLimits")
        return FColor::Red;
    if (Suggestion == "StabilizeConstraint")
        return FColor::Purple;
    if (Suggestion == "FreezeBoneTemporarily")
        return FColor::Magenta;

    return FColor::White;
}

void UOHPhysicsManager::EvaluateAndSuggestTuningForBones() {
    if (!SkeletalMesh || !GetWorld() || !bEnableRuntimeTuningEvaluation)
        return;

    // === Throttle debug drawing to once per second ===
    static float LastEvaluationTime = -1000.f;
    const float CurrentTime = GetWorld()->TimeSeconds;
    const float EvaluationInterval = 1.0f;

    if (CurrentTime - LastEvaluationTime < EvaluationInterval)
        return;

    LastEvaluationTime = CurrentTime;

    // === Gather simulation data ===
    const TArray<FName>& SimulatedBones = GetSimulatedBones();

    bool bAnyBoneSimulating = false;
    int32 SuggestionCount = 0;

    for (auto& Pair : TrackedBoneDataMap) {
        const FName& Bone = Pair.Key;
        FOHBoneData& Data = Pair.Value;

        const bool bSimulating = SimulatedBones.Contains(Bone);
        Data.SetIsSimulating(bSimulating);

        if (!Data.IsValid() || !bSimulating)
            continue;

        bAnyBoneSimulating = true;

        const float InstabilityScore = Data.GetCompositeInstabilityScore();
        if (InstabilityScore < 0.5f)
            continue;

        TArray<FString> Suggestions;
        FString NoteSummary;

        // === Suggestion Logic ===
        if (Data.GetVelocitySpikeScore() > 1.5f) {
            Suggestions.Add(TEXT("IncreaseLinearDamping"));
            NoteSummary += TEXT("[Spike] ");
        }
        if (Data.GetAngularJitterScore() > 1.2f) {
            Suggestions.Add(TEXT("IncreaseAngularDamping"));
            NoteSummary += TEXT("[Jitter] ");
        }
        if (Data.GetPoseDriftScore() > 10.f) {
            Suggestions.Add(TEXT("IncreasePositionStrength"));
            NoteSummary += TEXT("[PoseDrift] ");
        }
        if (Data.GetRotationalDriftScore() > 20.f) {
            Suggestions.Add(TEXT("IncreaseOrientationStrength"));
            NoteSummary += TEXT("[RotationDrift] ");
        }
        if (PhysicsGraph.GetConstraintStrain(Data.GetBoneName()) > 1.0f) {
            Suggestions.Add(TEXT("TightenConstraintLimits"));
            NoteSummary += TEXT("[OverSwing] ");
        }
        if (Data.GetJerkMagnitude() > 2000.f) {
            Suggestions.Add(TEXT("StabilizeConstraint"));
            NoteSummary += TEXT("[Jerk] ");
        }
        if (Data.GetTrajectoryShiftScore() > 90.f) {
            Suggestions.Add(TEXT("FreezeBoneTemporarily"));
            NoteSummary += TEXT("[TrajectoryFlip] ");
        }

        if (Suggestions.Num() > 0) {
            SuggestionCount++;

            const FTransform BoneTransform = SkeletalMesh->GetSocketTransform(Bone, RTS_World);
            const FVector BoneLocation = BoneTransform.GetLocation();
            const FVector BoneUp = BoneTransform.GetUnitAxis(EAxis::Z);

            const float Severity = FMath::Clamp(InstabilityScore / 3.f, 0.f, 1.f);
            const float BaseRadius = 2.5f;
            float ZStep = 0.f;

            for (const FString& Suggestion : Suggestions) {
                const FColor BaseColor = GetSuggestionColor(Suggestion);
                const FLinearColor HighlightColor = FLinearColor(BaseColor) * Severity;
                const FColor FinalColor = HighlightColor.ToFColor(true);

                const float Radius = BaseRadius + Severity * 2.f;
                const FVector SpherePos = BoneLocation + BoneUp * (10.f + ZStep);

                DrawDebugSphere(GetWorld(), SpherePos, Radius, 8, FinalColor, false,
                                EvaluationInterval, // lifetime matches interval
                                0, 2.0f);

                const FVector LabelOffset = FVector(0.f, 0.f, Radius + 6.f);
                DrawDebugString(GetWorld(), SpherePos + LabelOffset, Suggestion, nullptr, FinalColor,
                                EvaluationInterval, false, 1.5f);

                ZStep += 8.f;
            }

            // Optional logging
            UE_LOG(LogTemp, Warning, TEXT("[Tuning] %s | Instability: %.2f | Suggestions: %s | Notes: %s"),
                   *Bone.ToString(), InstabilityScore, *FString::Join(Suggestions, TEXT(", ")), *NoteSummary);
        }
    }

    if (!bAnyBoneSimulating) {
        UE_LOG(LogTemp, Verbose, TEXT("[Tuning] No bones simulating; skipping suggestions."));
    } else if (SuggestionCount == 0) {
        UE_LOG(LogTemp, Verbose, TEXT("[Tuning] Bones simulating but no instability detected."));
    }
}

#pragma endregion

#pragma region Utilities

bool UOHPhysicsManager::IsBoneSimulating(FName BoneName) const {
    const FOHBoneData* BoneData = OHSafeMapUtils::GetReference(PhysicsGraph.GetBoneMap(), BoneName);
    return BoneData && BoneData->GetIsSimulating();
}

FOHBoneData* UOHPhysicsManager::GetBoneDataChecked(FName BoneName) {
    return OHSafeMapUtils::GetMutableReference(PhysicsGraph.GetBoneMap(), BoneName);
}

void UOHPhysicsManager::ComputePhysicsTweaksForBone(const FName& Bone, float& OutPACMultiplier, float& OutLinearDamping,
                                                    float& OutAngularDamping) const {
    OutPACMultiplier = 1.f;
    OutLinearDamping = 5.f;
    OutAngularDamping = 3.f;

    if (!SkeletalMesh || !SkeletalMesh->GetSkeletalMeshAsset())
        return;

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
    const int32 BoneIndex = RefSkeleton.FindBoneIndex(Bone);
    if (BoneIndex == INDEX_NONE)
        return;

    const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
    float Length = 10.f;

    if (ParentIndex != INDEX_NONE) {
        const FVector BoneLoc = SkeletalMesh->GetBoneLocation(Bone, EBoneSpaces::ComponentSpace);
        const FName ParentBoneName = RefSkeleton.GetBoneName(ParentIndex);
        const FVector ParentLoc = SkeletalMesh->GetBoneLocation(ParentBoneName, EBoneSpaces::ComponentSpace);
        Length = FVector::Dist(BoneLoc, ParentLoc);
    }

    FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone);
    if (!Body)
        return;

    const float Mass = Body->GetBodyMass();

    // Compute bone depth (distance from root)
    int32 Depth = 0;
    int32 Current = BoneIndex;
    while (RefSkeleton.GetParentIndex(Current) != INDEX_NONE) {
        Current = RefSkeleton.GetParentIndex(Current);
        ++Depth;
    }

    // Combined responsiveness: longer + heavier + deeper = stronger
    const float Responsiveness = (Length * 0.3f + Mass * 1.0f) * (1.0f + Depth * 0.05f);

    OutPACMultiplier = FMath::Clamp(Responsiveness / 10.f, 0.5f, 2.0f);

    // Base damping
    float DampingBase = Responsiveness * 0.5f;

    // Boost damping for high-jitter small bones
    const FString BoneStr = Bone.ToString().ToLower();
    if (BoneStr.Contains("hand") || BoneStr.Contains("foot") || BoneStr.Contains("head")) {
        DampingBase *= 1.5f;
    }

    // Clamp final damping values
    OutLinearDamping = FMath::Clamp(DampingBase, 2.0f, 14.0f);
    OutAngularDamping = FMath::Clamp(DampingBase * 0.8f, 1.0f, 10.0f);
}

void UOHPhysicsManager::UpdateConstraintRuntimeStates(float DeltaTime) {
    // Tweak these for responsiveness vs. smoothness
    constexpr float JitterSmoothingAlpha = 0.1f;
    constexpr float DriveSmoothingAlpha = 0.1f;

    if (!SkeletalMesh) {
        return;
    }

    // For each constraint edge in our graph...
    for (FOHConstraintInstanceData& Constraint : PhysicsGraph.GetConstraintLinks()) {
        // 1) Grab our runtime‐state wrapper
        FConstraintRuntimeState& RS = Constraint.GetRuntimeState();

        // 2) Lazy‐init the FConstraintInstance* if missing
        if (!RS.GetConstraintInstance() && Constraint.GetOwnerComponent()) {
            RS.SetConstraintInstance(SkeletalMesh->FindConstraintInstance(Constraint.GetConstraintName()));
        }
        FConstraintInstance* CI = RS.GetConstraintInstance();
        if (!CI) {
            continue;
        }

        // 3) Skip any bodies not currently simulating physics
        //    (assumes you simulate the ChildBone during blends)
        FBodyInstance* BI = SkeletalMesh->GetBodyInstance(Constraint.GetChildBone());
        if (!(BI && BI->IsInstanceSimulatingPhysics())) {
            continue;
        }

        // 4) Sample force & torque → compute total “strain”
        FVector Force, Torque;
        CI->GetConstraintForce(Force, Torque);
        float NewStrain = Force.Size() + Torque.Size();

        // 5) Compute Δstrain and update jitter metric
        float PrevStrain = RS.GetStrain();
        float DeltaStrain = FMath::Abs(NewStrain - PrevStrain);
        RS.SetPrevStrain(PrevStrain);
        RS.SetStrain(NewStrain);
        RS.SetJitterMetric(FMath::Lerp(RS.GetJitterMetric(), DeltaStrain, JitterSmoothingAlpha));

        // 6) Read back the four drive strengths from the constraint
        float LinSpring, LinDamp, LinLimit;
        CI->GetLinearDriveParams(LinSpring, LinDamp, LinLimit);

        float AngSpring, AngDamp, AngLimit;
        CI->GetAngularDriveParams(AngSpring, AngDamp, AngLimit);

        // 7) Smooth them locally via a copy of LastReadback
        FConstraintRuntimeReadback Smoothed = RS.GetLastReadback();
        Smoothed.SetPositionStrength(FMath::Lerp(Smoothed.GetPositionStrength(), LinSpring, DriveSmoothingAlpha));
        Smoothed.SetVelocityStrength(FMath::Lerp(Smoothed.GetVelocityStrength(), LinDamp, DriveSmoothingAlpha));
        Smoothed.SetOrientationStrength(FMath::Lerp(Smoothed.GetOrientationStrength(), AngSpring, DriveSmoothingAlpha));
        Smoothed.SetAngularVelocityStrength(
            FMath::Lerp(Smoothed.GetAngularVelocityStrength(), AngDamp, DriveSmoothingAlpha));
        // (If you have a readback for bLocalSimulation, set it here too)
        RS.SetLastReadback(Smoothed);

        // 8) Re‐apply the smoothed drive strengths into the constraint

        // -- Linear drives on X/Y/Z
        CI->SetLinearPositionDrive(true, true, true);
        CI->SetLinearVelocityDrive(true, true, true);
        CI->SetLinearDriveParams(Smoothed.GetPositionStrength(), Smoothed.GetVelocityStrength(), 0.f);

        // -- Angular drives in Twist & Swing mode
        CI->SetOrientationDriveTwistAndSwing(/*bEnableTwist=*/true, /*bEnableSwing=*/true);
        CI->SetAngularVelocityDriveTwistAndSwing(/*bEnableTwist=*/true, /*bEnableSwing=*/true);
        CI->SetAngularDriveParams(Smoothed.GetOrientationStrength(), Smoothed.GetAngularVelocityStrength(), 0.f);
    }
}

void UOHPhysicsManager::ComputeOptimalConstraintSettings(const FConstraintInstance* ConstraintInstance,
                                                         FConstraintProfileProperties& OutProfile,
                                                         const FName& BoneName) const {
    if (!SkeletalMesh || !ConstraintInstance)
        return;

    USkeletalMesh* MeshAsset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!MeshAsset)
        return;

    const FReferenceSkeleton& RefSkel = MeshAsset->GetRefSkeleton();
    const int32 BoneIndex = RefSkel.FindBoneIndex(BoneName);
    if (BoneIndex == INDEX_NONE)
        return;

    const int32 ParentIndex = RefSkel.GetParentIndex(BoneIndex);
    if (ParentIndex == INDEX_NONE)
        return;

    const FName ParentBone = RefSkel.GetBoneName(ParentIndex);

    // === Cache mass ===
    float Mass = CachedBoneMasses.FindOrAdd(BoneName);
    if (Mass <= 0.f)
        Mass = SkeletalMesh->GetBoneMass(BoneName);

    float ParentMass = CachedBoneMasses.FindOrAdd(ParentBone);
    if (ParentMass <= 0.f)
        ParentMass = SkeletalMesh->GetBoneMass(ParentBone);

    const float MassRatio = (ParentMass > SMALL_NUMBER) ? Mass / ParentMass : 1.f;

    // === Cache length ===
    float& Length = CachedBoneLengths.FindOrAdd(BoneName);
    if (Length <= 0.f) {
        const FVector BonePos = SkeletalMesh->GetBoneLocation(BoneName);
        const FVector ParentPos = SkeletalMesh->GetBoneLocation(ParentBone);
        Length = FVector::Dist(BonePos, ParentPos);
    }

    // === Classification based on bone name ===
    const FString BoneStr = BoneName.ToString().ToLower();
    const bool bIsDistal = BoneStr.Contains("hand") || BoneStr.Contains("foot");
    const bool bIsSpine = BoneStr.Contains("spine");
    const bool bIsClavicle = BoneStr.Contains("clavicle");

    // === Angular limits ===
    float SwingLimit = FMath::Clamp(Length * 1.2f, 20.f, bIsDistal ? 60.f : 40.f);
    float TwistLimit = FMath::Clamp(Length * 0.8f, 10.f, 35.f);
    if (bIsSpine || bIsClavicle) {
        SwingLimit *= 1.4f;
        TwistLimit *= 1.2f;
    }

    OutProfile.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Limited;
    OutProfile.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Limited;
    OutProfile.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Limited;

    OutProfile.ConeLimit.Swing1LimitDegrees = SwingLimit;
    OutProfile.ConeLimit.Swing2LimitDegrees = SwingLimit;
    OutProfile.TwistLimit.TwistLimitDegrees = TwistLimit;

    // === Angular stiffness/damping ===
    float DampingBase = bIsDistal ? 4.f : 2.f;
    float StiffnessBase = bIsDistal ? 35.f : 25.f;

    if (bIsSpine || bIsClavicle) {
        StiffnessBase *= 0.8f;
        DampingBase *= 1.5f;
    }

    OutProfile.ConeLimit.Stiffness = StiffnessBase * MassRatio;
    OutProfile.ConeLimit.Damping = DampingBase * MassRatio;
    OutProfile.TwistLimit.Stiffness = StiffnessBase * MassRatio;
    OutProfile.TwistLimit.Damping = DampingBase * MassRatio;

    // === Optional linear limits ===
    if (bIsDistal) {
        OutProfile.LinearLimit.XMotion = ELinearConstraintMotion::LCM_Limited;
        OutProfile.LinearLimit.YMotion = ELinearConstraintMotion::LCM_Limited;
        OutProfile.LinearLimit.ZMotion = ELinearConstraintMotion::LCM_Limited;

        OutProfile.LinearLimit.Limit = FMath::Clamp(Length * 0.3f, 2.0f, 6.0f);
        OutProfile.LinearLimit.Stiffness = 1000.f;
        OutProfile.LinearLimit.Damping = 50.f;
    } else {
        OutProfile.LinearLimit.XMotion = ELinearConstraintMotion::LCM_Locked;
        OutProfile.LinearLimit.YMotion = ELinearConstraintMotion::LCM_Locked;
        OutProfile.LinearLimit.ZMotion = ELinearConstraintMotion::LCM_Locked;
    }

    // === Projection is disabled by default ===
    OutProfile.bEnableProjection = false;

    // === Optional: debug log ===
    UE_LOG(LogTemp, Log,
           TEXT("[OHPhysicsManager] [%s] Constraint Profile | Swing=%.1f Twist=%.1f | MassRatio=%.2f | Length=%.1f"),
           *BoneName.ToString(), SwingLimit, TwistLimit, MassRatio, Length);
}

FPhysicalAnimationData UOHPhysicsManager::GetScaledPACProfile(const FName& Bone,
                                                              const FPhysicalAnimationData& BaseProfile) const {
    FPhysicalAnimationData ScaledProfile = BaseProfile;

    float PACMultiplier = 1.f;
    float LinearDamping = 5.f;
    float AngularDamping = 3.f;

    ComputePhysicsTweaksForBone(Bone, PACMultiplier, LinearDamping, AngularDamping);

    ScaledProfile.OrientationStrength *= PACMultiplier;
    ScaledProfile.PositionStrength *= PACMultiplier;
    ScaledProfile.VelocityStrength *= PACMultiplier;

    return ScaledProfile;
}

FPhysicalAnimationData UOHPhysicsManager::GetPhysicalAnimationData(FName BoneName, EDriveAccessMode AccessMode) const {
    FPhysicalAnimationData Result;
    Result.BodyName = BoneName;

    // 1) Find the constraint where this is the child bone
    const FOHConstraintInstanceData* Data = PhysicsGraph.FindConstraintByChildBone(BoneName);
    if (!Data) {
        return Result;
    }

    // 2) Live vs. cached drive strengths
    if (AccessMode == EDriveAccessMode::Live) {
        const FConstraintRuntimeReadback& RB = Data->GetRuntimeState().GetLastReadback();
        Result.OrientationStrength = RB.GetOrientationStrength();
        Result.AngularVelocityStrength = RB.GetAngularVelocityStrength();
        Result.PositionStrength = RB.GetPositionStrength();
        Result.VelocityStrength = RB.GetVelocityStrength();
        Result.bIsLocalSimulation = RB.GetLocalSimulation();
    } else {
        // Fall back to template defaults
        if (const UPhysicsConstraintTemplate* Tem = Data->GetConstraintTemplate()) {
            const FConstraintInstance& DI = Tem->DefaultInstance;
            Result.OrientationStrength = DI.ProfileInstance.AngularDrive.SlerpDrive.Stiffness;
            Result.AngularVelocityStrength = DI.ProfileInstance.AngularDrive.SlerpDrive.Damping;
            Result.PositionStrength = DI.ProfileInstance.LinearDrive.XDrive.Stiffness;
            Result.VelocityStrength = DI.ProfileInstance.LinearDrive.XDrive.Damping;
            Result.bIsLocalSimulation = true; // bone-space drives
        }
    }

    // 3) Always fill force limits from the template’s default instance
    if (const UPhysicsConstraintTemplate* Tem = Data->GetConstraintTemplate()) {
        const FConstraintInstance& DI = Tem->DefaultInstance;
        Result.MaxAngularForce = DI.ProfileInstance.AngularDrive.SlerpDrive.MaxForce;
        Result.MaxLinearForce = DI.ProfileInstance.LinearDrive.XDrive.MaxForce;
    }

    return Result;
}

void UOHPhysicsManager::SetPhysicalAnimationData(FName BoneName, const FPhysicalAnimationData& Data) {
    // 1) Find the constraint in our runtime graph (child‐bone lookup)
    FOHConstraintInstanceData* ConstraintData = PhysicsGraph.FindConstraintByChildBone(BoneName);
    if (!ConstraintData) {
        UE_LOG(LogTemp, Warning, TEXT("SetPhysicalAnimationData: no constraint found for bone %s"),
               *BoneName.ToString());
        return;
    }

    // 2) Grab the live FConstraintInstance pointer
    FConstraintInstance* CI = ConstraintData->GetRuntimeState().GetConstraintInstance();
    if (!CI) {
        UE_LOG(LogTemp, Warning, TEXT("SetPhysicalAnimationData: constraint instance is null for bone %s"),
               *BoneName.ToString());
        return;
    }

    // 3) Apply linear drives (X/Y/Z)
    CI->SetLinearPositionDrive(true, true, true);
    CI->SetLinearVelocityDrive(true, true, true);
    CI->SetLinearDriveParams(Data.PositionStrength, Data.VelocityStrength,
                             /*Limit=*/0.f);

    // 4) Apply angular drives (Twist & Swing)
    CI->SetOrientationDriveTwistAndSwing(/*bEnableTwist=*/true, /*bEnableSwing=*/true);
    CI->SetAngularVelocityDriveTwistAndSwing(/*bEnableTwist=*/true, /*bEnableSwing=*/true);
    CI->SetAngularDriveParams(Data.OrientationStrength, Data.AngularVelocityStrength,
                              /*Limit=*/0.f);

    // 5) (Optional) cache local‐simulation flag
    // ConstraintData->GetRuntimeState().GetLastReadback().SetLocalSimulation(Data.bIsLocalSimulation);
}

bool UOHPhysicsManager::ArePACProfilesEqual(const FPhysicalAnimationData& A, const FPhysicalAnimationData& B) {
    constexpr float Tolerance = KINDA_SMALL_NUMBER;

    return FMath::IsNearlyEqual(A.OrientationStrength, B.OrientationStrength, Tolerance) &&
           FMath::IsNearlyEqual(A.PositionStrength, B.PositionStrength, Tolerance) &&
           FMath::IsNearlyEqual(A.VelocityStrength, B.VelocityStrength, Tolerance) &&
           FMath::IsNearlyEqual(A.AngularVelocityStrength, B.AngularVelocityStrength, Tolerance) &&
           A.bIsLocalSimulation == B.bIsLocalSimulation;
}

float UOHPhysicsManager::GetPosePositionError(FName Bone) const {
    if (!SkeletalMesh)
        return 0.f;

    const FTransform AnimTransform = SkeletalMesh->GetSocketTransform(Bone, RTS_World);
    const FVector SimulatedPos = GetPosition(Bone);

    return FVector::Dist(SimulatedPos, AnimTransform.GetLocation());
}

float UOHPhysicsManager::GetPoseRotationError(FName Bone) const {
    if (!SkeletalMesh)
        return 0.f;

    const FQuat AnimQuat = SkeletalMesh->GetSocketQuaternion(Bone);
    const FQuat SimQuat = GetRotation(Bone);

    const float AngleDelta = SimQuat.AngularDistance(AnimQuat);
    return FMath::RadiansToDegrees(AngleDelta);
}

#pragma endregion

#pragma region BlendState

FActivePhysicsBlend UOHPhysicsManager::CreatePhysicsBlendState(FName RootBone, float StartAlpha, float BlendIn,
                                                               float Hold, float BlendOut, FName ReactionTag) {
    FActivePhysicsBlend Blend;
    Blend.RootBone = RootBone;
    Blend.BlendAlpha = StartAlpha;
    Blend.StartAlpha = StartAlpha; // <-- Set this
    Blend.Phase = EOHBlendPhase::BlendIn;
    Blend.Elapsed = 0.0f;
    Blend.BlendInDuration = BlendIn;
    Blend.HoldDuration = Hold;
    Blend.BlendOutDuration = BlendOut;
    Blend.TotalBlendTime = BlendIn + Hold + BlendOut;
    Blend.ReactionTag = ReactionTag;
    return Blend;
}

void UOHPhysicsManager::UpdateBlendState(FActivePhysicsBlend& Blend, float DeltaTime) {
    if (Blend.IsPaused())
        return;

    Blend.Elapsed += DeltaTime;

    auto AdvancePhase = [&Blend](EOHBlendPhase NewPhase) {
        Blend.Phase = NewPhase;
        Blend.Elapsed = 0.f;
    };

    switch (Blend.Phase) {
    case EOHBlendPhase::BlendIn:
        if (Blend.BlendInDuration <= KINDA_SMALL_NUMBER) {
            AdvancePhase(EOHBlendPhase::Hold);
            Blend.BlendAlpha = 1.f;
            break;
        }

        {
            float Progress = Blend.Elapsed / FMath::Max(Blend.BlendInDuration, KINDA_SMALL_NUMBER);
            Blend.BlendAlpha = FMath::Clamp(Progress, 0.f, 1.f);

            if (Progress >= 1.f) {
                AdvancePhase(EOHBlendPhase::Hold);
            }
            break;
        }

    case EOHBlendPhase::Hold:
        if (Blend.HoldDuration <= KINDA_SMALL_NUMBER) {
            AdvancePhase(EOHBlendPhase::BlendOut);
            break;
        }

        Blend.BlendAlpha = 1.f;

        if (Blend.Elapsed >= Blend.HoldDuration) {
            AdvancePhase(EOHBlendPhase::BlendOut);
        }
        break;

    case EOHBlendPhase::BlendOut:
        if (Blend.BlendOutDuration <= KINDA_SMALL_NUMBER) {
            Blend.BlendAlpha = 0.f;
            break;
        }

        {
            float Progress = Blend.Elapsed / FMath::Max(Blend.BlendOutDuration, KINDA_SMALL_NUMBER);
            Blend.BlendAlpha = 1.f - FMath::Clamp(Progress, 0.f, 1.f);
            break;
        }

    default:
        ensureMsgf(false, TEXT("[OH] Invalid blend phase"));
        break;
    }
}

void UOHPhysicsManager::ProcessBlendPhases(float DeltaTime) {
    TArray<FName> CompletedRootBones;

    for (auto& Elem : ActiveBlends) {
        FActivePhysicsBlend& Blend = Elem.Value;

        // Use centralized update logic
        UpdateBlendState(Blend, DeltaTime);

        // Mark for cleanup if complete
        if (IsBlendComplete(Blend)) {
            CompletedRootBones.Add(Blend.RootBone);
        }

        // Apply BlendAlpha to full bone chain
        TArray<FName> BoneNames;
        SkeletalMesh->GetBoneNames(BoneNames);

        for (const FName& Bone : BoneNames) {
            if (!IsBoneValidForChain(Bone, Blend.RootBone))
                continue;

            if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
                Body->PhysicsBlendWeight = Blend.BlendAlpha;
            }
        }
    }

    // Finalize completed blends
    for (const FName& RootBone : CompletedRootBones) {
        FinalizeBlend(RootBone);
    }
}

void UOHPhysicsManager::PauseBlend(FName Bone) {
    if (FActivePhysicsBlend* Blend = ActiveBlends.Find(Bone)) {
        Blend->PauseCount++;
        if (Blend->PauseCount == 1) {
            UE_LOG(LogTemp, Log, TEXT("[OH] Paused blend on %s"), *Bone.ToString());
        }
    }
}

void UOHPhysicsManager::ResumeBlend(FName Bone) {
    if (FActivePhysicsBlend* Blend = ActiveBlends.Find(Bone)) {
        if (Blend->PauseCount > 0) {
            Blend->PauseCount--;
            if (Blend->PauseCount == 0) {
                UE_LOG(LogTemp, Log, TEXT("[OH] Resumed blend on %s"), *Bone.ToString());
            }
        }
    }
}
void UOHPhysicsManager::PauseAllBlends() {
    for (const auto& Elem : ActiveBlends) {
        PauseBlend(Elem.Key);
    }
}

void UOHPhysicsManager::ResumeAllBlends() {
    for (const auto& Elem : ActiveBlends) {
        ResumeBlend(Elem.Key);
    }
}

void UOHPhysicsManager::ForceUnpause(FName Bone) {
    if (FActivePhysicsBlend* Blend = ActiveBlends.Find(Bone)) {
        if (Blend->PauseCount > 0) {
            Blend->PauseCount = 0;
            UE_LOG(LogTemp, Log, TEXT("[OH] Force-unpaused blend on %s"), *Bone.ToString());
        }
    }
}

bool UOHPhysicsManager::IsBoneBlending(FName BoneName) const {
    return ActiveBlends.Contains(BoneName);
}

EOHBlendPhase UOHPhysicsManager::GetCurrentBlendPhase(FName BoneName) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    return Blend ? Blend->Phase : EOHBlendPhase::BlendOut; // default = not blending / fallback state
}

float UOHPhysicsManager::GetBlendAlpha(FName BoneName) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    return Blend ? Blend->BlendAlpha : 0.f;
}

FName UOHPhysicsManager::GetActiveReactionTag(FName BoneName) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    return Blend ? Blend->ReactionTag : NAME_None;
}

TArray<FName> UOHPhysicsManager::GetSimulatedBones() const {
    TArray<FName> OutBones;
    ActiveBlends.GetKeys(OutBones);
    return OutBones;
}

float UOHPhysicsManager::GetRemainingBlendTime(FName BoneName) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    if (!Blend)
        return 0.f;

    return FMath::Max(0.f, Blend->TotalBlendTime - Blend->Elapsed);
}

float UOHPhysicsManager::GetCurrentPhaseProgress(FName BoneName) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    if (!Blend)
        return 0.f;

    const float PhaseDuration = [Blend]() -> float {
        switch (Blend->Phase) {
        case EOHBlendPhase::BlendIn:
            return FMath::Max(Blend->BlendInDuration, KINDA_SMALL_NUMBER);
        case EOHBlendPhase::Hold:
            return FMath::Max(Blend->HoldDuration, KINDA_SMALL_NUMBER);
        case EOHBlendPhase::BlendOut:
            return FMath::Max(Blend->BlendOutDuration, KINDA_SMALL_NUMBER);
        default:
            return 1.f; // fallback duration avoids divide by 0
        }
    }();

    return FMath::Clamp(Blend->Elapsed / PhaseDuration, 0.f, 1.f);
}

float UOHPhysicsManager::GetPhaseDuration(FName BoneName, EOHBlendPhase Phase) const {
    const FActivePhysicsBlend* Blend = ActiveBlends.Find(BoneName);
    if (!Blend)
        return 0.f;

    switch (Phase) {
    case EOHBlendPhase::BlendIn:
        return Blend->BlendInDuration;
    case EOHBlendPhase::Hold:
        return Blend->HoldDuration;
    case EOHBlendPhase::BlendOut:
        return Blend->BlendOutDuration;
    default:
        return 0.f;
    }
}

void UOHPhysicsManager::ApplyBlendAlphaToBone(FName Bone, float BlendAlpha) {
    if (FBodyInstance* Body = SkeletalMesh->GetBodyInstance(Bone)) {
        Body->PhysicsBlendWeight = BlendAlpha;
    }
}

bool UOHPhysicsManager::IsBlendComplete(const FActivePhysicsBlend& Blend) {
    return Blend.Phase == EOHBlendPhase::BlendOut && Blend.Elapsed >= Blend.BlendOutDuration;
}

void UOHPhysicsManager::FinalizeBlend(FName RootBone) {
    ActiveBlends.Remove(RootBone);

    // Collect bones in the chain
    TArray<FName> BoneNames;
    SkeletalMesh->GetBoneNames(BoneNames);

    for (const FName& Bone : BoneNames) {
        if (!IsBoneValidForChain(Bone, RootBone))
            continue;

        if (int32* RefCountPtr = BoneSimBlendRefCount.Find(Bone)) {
            (*RefCountPtr)--;

            if (*RefCountPtr <= 0) {
                BoneSimBlendRefCount.Remove(Bone);
                ClearPhysicsStateForBone(Bone); // Also clears PAC + tracking
            }
        } else {
            // This bone was never activated — clean PAC if it got applied
            ClearPhysicsStateForBone(Bone);
        }
    }

    OnHitReactionComplete.Broadcast(RootBone);
}

void UOHPhysicsManager::FinalizeAllBlends() {
    if (!SkeletalMesh)
        return;

    TArray<FName> Roots;
    ActiveBlends.GetKeys(Roots);

    for (const FName& RootBone : Roots) {
        FinalizeBlend(RootBone);
    }

    UE_LOG(LogTemp, Log, TEXT("[OH] Finalized %d active blends"), Roots.Num());
}

#pragma endregion

#pragma region Experimental

#pragma region BodyPartStatus

void UOHPhysicsManager::ApplyDamageToBone(FName Bone, float Damage) {
    if (FOHBodyPartStatus* Status = BoneStatusMap.Find(Bone)) {
        Status->ApplyDamage(Damage);
        if (Status->IsDestroyed()) {
            FinalizeBlend(Bone);
            UE_LOG(LogTemp, Warning, TEXT("[OH] Bone %s destroyed"), *Bone.ToString());
        }
    }
}

bool UOHPhysicsManager::IsBoneDestroyed(FName Bone) const {
    if (const FOHBodyPartStatus* Status = BoneStatusMap.Find(Bone)) {
        return Status->IsDestroyed();
    }
    return false;
}

float UOHPhysicsManager::GetBoneHealth(FName Bone) const {
    if (const FOHBodyPartStatus* Status = BoneStatusMap.Find(Bone)) {
        return Status->CurrentHealth;
    }
    return -1.f;
}

void UOHPhysicsManager::ResetBoneStatus(FName Bone) {
    if (FOHBodyPartStatus* Status = BoneStatusMap.Find(Bone)) {
        Status->Reset();
    }
}

TArray<FName> UOHPhysicsManager::GetSkeletalBonesInBodyPart(EOHBodyPart Part) const {
    return UOHSkeletalPhysicsUtils::GetPrimaryBoneNamesFromBodyPart(Part);
}

bool UOHPhysicsManager::IsBodyPartFullyDestroyed(EOHBodyPart Part) const {
    for (const auto& Elem : BoneStatusMap) {
        if (Elem.Value.BodyPart == Part && !Elem.Value.IsDestroyed()) {
            return false;
        }
    }
    return true;
}
#pragma endregion

#pragma region SkeletalAssetValidation

bool UOHPhysicsManager::ValidateBoneMappings(bool bLogResults) {
    if (!SkeletalMesh || !SkeletalMesh->GetSkeletalMeshAsset()) {
        UE_LOG(LogTemp, Error, TEXT("[OHPhysicsManager] ValidateBoneMappings: No skeletal mesh"));
        return false;
    }

    ResolvedBoneData.Empty();
    SimulatableBones.Empty();

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton();

    TArray<FName> ActualBoneNames;
    for (int32 i = 0; i < RefSkeleton.GetNum(); ++i) {
        ActualBoneNames.Add(RefSkeleton.GetBoneName(i));
    }

    int32 ValidCount = 0;

    for (const FName& ExpectedBone : TrackedBoneDefinitions) {
        FOHResolvedBoneData Data;
        Data.ResolvedBone = NAME_None;
        Data.MatchResult = UOHAlgoUtils::FindBestNameMatchAutoStrategy(ExpectedBone, ActualBoneNames);
        Data.ResolvedBone = FName(Data.MatchResult.Candidate);
        Data.bResolved = ActualBoneNames.Contains(Data.ResolvedBone);

        if (Data.bResolved) {
            ++ValidCount;
        }

        // Infer skeletal enums if possible
        Data.LogicalBone =
            UOHSkeletalPhysicsUtils::ResolveSkeletalBoneFromNameSmart(Data.ResolvedBone, SkeletalMesh, 0.75f);
        Data.BodyPart = UOHSkeletalPhysicsUtils::GetBodyPartFromBone(Data.LogicalBone);
        Data.BodyZone = UOHSkeletalPhysicsUtils::GetBodyZoneFromBone(Data.LogicalBone);
        Data.FunctionalGroup = UOHSkeletalPhysicsUtils::GetFunctionalGroupFromBone(Data.LogicalBone);

        ResolvedBoneData.Add(ExpectedBone, Data);

        if (bLogResults && !Data.bResolved && Data.MatchResult.Candidate != "") {
            UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Bone '%s' not found. Did you mean '%s'? (Score: %.2f)"),
                   *ExpectedBone.ToString(), *Data.MatchResult.Candidate, Data.MatchResult.Score);
        }
    }

    // Filter simulatable bones
    for (const TPair<FName, FOHResolvedBoneData>& Pair : ResolvedBoneData) {
        const FName& ActualBone = Pair.Value.ResolvedBone;
        if (Pair.Value.bResolved && !SimExclusionBoneSet.Contains(ActualBone) && HasPhysicsBody(ActualBone)) {
            SimulatableBones.Add(ActualBone);
        }
    }

    if (bLogResults) {
        UE_LOG(LogTemp, Log, TEXT("[OHPhysicsManager] Validation complete: %d/%d bones valid, %d simulatable"),
               ValidCount, TrackedBoneDefinitions.Num(), SimulatableBones.Num());
    }

    return ValidCount == TrackedBoneDefinitions.Num();
}

TArray<FString> UOHPhysicsManager::GetMissingBones() const {
    TArray<FString> Result;
    for (const auto& Pair : ResolvedBoneData) {
        if (!Pair.Value.bResolved) {
            Result.Add(Pair.Key.ToString());
        }
    }
    return Result;
}

TMap<FName, FName> UOHPhysicsManager::GetBoneSuggestions() const {
    TMap<FName, FName> Result;
    for (const auto& Pair : ResolvedBoneData) {
        if (!Pair.Value.bResolved && !Pair.Value.MatchResult.Candidate.IsEmpty()) {
            Result.Add(Pair.Key, FName(Pair.Value.MatchResult.Candidate));
        }
    }
    return Result;
}

#pragma endregion

#pragma region PhysicsGraph

void UOHPhysicsManager::ValidateAndRepairGraph() {
    if (!SkeletalMesh || !CachedPhysicsAsset) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] ValidateAndRepairGraph aborted — missing components."));
        return;
    }

    InvalidBodies.Reset();
    InvalidConstraints.Reset();

    PerformValidationPass();

    if (InvalidBodies.Num() > 0 || InvalidConstraints.Num() > 0) {
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsManager] Found %d invalid bodies and %d invalid constraints."),
               InvalidBodies.Num(), InvalidConstraints.Num());

        PerformRepairPass();

        if (InvalidBodies.Num() > 0 || InvalidConstraints.Num() > 0) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsManager] Repair incomplete. Consider full physics reset."));
            // Optional: RecreatePhysicsState() or trigger rebuild here
        }
    }
}

void UOHPhysicsManager::PerformValidationPass() {
    for (auto& Pair : TrackedBoneDataMap) {
        const FName& BoneName = Pair.Key;
        FOHBoneData& BoneData = Pair.Value;

        FBodyInstance* BodyInstance = BoneData.GetBodyInstance();
        if (!IsBodyInstanceValid(BodyInstance)) {
            InvalidBodies.Add(BoneName);
            continue;
        }

        FConstraintInstance* ConstraintInstance = BoneData.GetParentConstraintInstance();
        if (!IsConstraintInstanceValid(ConstraintInstance)) {
            InvalidConstraints.Add(BoneName);
        }
    }
}

void UOHPhysicsManager::PerformRepairPass() {
    for (const FName& BoneName : InvalidBodies) {
        if (TryRecreateBodyInstance(BoneName)) {
            InvalidBodies.Remove(BoneName);
        } else {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsManager] Failed to recreate body instance for bone %s"),
                   *BoneName.ToString());
        }
    }

    for (const FName& BoneName : InvalidConstraints) {
        FOHBoneData* BoneData = TrackedBoneDataMap.Find(BoneName);
        if (!BoneData)
            continue;

        FName ParentBone = BoneData->GetParentBone();
        if (TryRecreateConstraint(ParentBone, BoneName)) {
            InvalidConstraints.Remove(BoneName);
        } else {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsManager] Failed to recreate constraint for bone %s"),
                   *BoneName.ToString());
        }
    }
}

bool UOHPhysicsManager::TryRecreateBodyInstance(const FName& BoneName) {
    if (!SkeletalMesh || BoneName.IsNone())
        return false;

    FBodyInstance* BodyInstance = SkeletalMesh->GetBodyInstance(BoneName);
    if (IsBodyInstanceValid(BodyInstance)) {
        return true; // Already valid
    }

    // Recreate physics state for the skeletal mesh — this refreshes all bodies and constraints
    SkeletalMesh->RecreatePhysicsState();

    BodyInstance = SkeletalMesh->GetBodyInstance(BoneName);
    return IsBodyInstanceValid(BodyInstance);
}

bool UOHPhysicsManager::TryRecreateConstraint(const FName& ParentBone, const FName& ChildBone) {
    if (!SkeletalMesh || ParentBone.IsNone() || ChildBone.IsNone())
        return false;

    // Usually, constraints are created with physics state recreation
    SkeletalMesh->RecreatePhysicsState();

    FOHBoneData* ChildBoneData = TrackedBoneDataMap.Find(ChildBone);
    if (!ChildBoneData)
        return false;

    FConstraintInstance* ConstraintInstance = ChildBoneData->GetParentConstraintInstance();
    return IsConstraintInstanceValid(ConstraintInstance);
}

bool UOHPhysicsManager::IsBodyInstanceValid(const FBodyInstance* BodyInstance) {
    return BodyInstance && BodyInstance->IsValidBodyInstance();
}

bool UOHPhysicsManager::IsConstraintInstanceValid(const FConstraintInstance* ConstraintInstance) {
    // You may want to add more detailed validity checks here if needed
    return ConstraintInstance != nullptr;
}

/////////////////////////////////////////////////
void UOHPhysicsManager::OnSkeletalMeshChanged() {
    bPhysicsGraphDirty = true;
    ++PhysicsGraphVersion;
}

void UOHPhysicsManager::OnPhysicsAssetChanged() {
    bPhysicsGraphDirty = true;
    ++PhysicsGraphVersion;
}

// Example: if you hot-reload, or reconfigure bones:
void UOHPhysicsManager::OnGraphRelevantSettingsChanged() {
    bPhysicsGraphDirty = true;
    ++PhysicsGraphVersion;
}
//////////////////////////////////////////////////
#pragma endregion

#pragma endregion