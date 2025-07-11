#pragma once

#include "CoreMinimal.h"
#include "OHPhysicsStructs.h"
#include "Components/ActorComponent.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "OHPhysicsManager.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOHBoneSimEvent, FName, BoneName);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOHHitReactionComplete, FName, BoneName);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOHOnHitReactionBonesUpdated, FName, RootBone, const TArray<FName>&,
                                             AffectedBones);

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHPhysicsManager : public UActorComponent {
    GENERATED_BODY()

  public:
    UOHPhysicsManager();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    bool bEnablePhysicsManager = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Physics|Simulation")
    float StartTickDelay = 0.0f;

#pragma region Initialization
    void InitializePhysicsManager();

    void InitializePhysicalAnimationSystem();

    /** Rebuilds SimulatableBones by filtering TrackedBoneDefinitions against the skeletal mesh’s physics asset and
     * exclusion rules */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|MotionTracking")
    void InitializeTrackedBoneData();

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Simulation")
    void InitializeSimulatableBones();

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Simulation")
    void InitializeAllPhysicalAnimationProfiles(const FPhysicalAnimationData& Profile);

    void InitializeBoneLinksFromPhysicsAsset();
#pragma endregion
#pragma region BoneLinks

    void BuildBoneLinkGraph();

#pragma endregion
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|PhysicsManager")
    void ResetPhysicsManager();

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void ForceReinitializePhysics(bool bVerboseLog = true);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void ClearPhysicsState();

#pragma region ComponentFieldAccessors
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|ManagerAccessors")
    USkeletalMeshComponent* GetSkeletalMeshComponent() const;

#pragma endregion

#pragma region MotionTimeConfig

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    EOHMotionTimeDomain ActiveTimeDomain = EOHMotionTimeDomain::GameWorldTime;

    /** Returns the current motion time used for bone sample timestamps */
    UFUNCTION(BlueprintPure, Category = "Motion Tracking")
    float GetCurrentMotionTime() const;

    DECLARE_DELEGATE_RetVal(float, FMotionTimeOverride);
    FMotionTimeOverride MotionTimeOverride;

#pragma endregion

#pragma region Motion Tracking
    // === CONFIGURATION ===

    /** Bones to track regardless of simulation (core postural + motion data) */
    UPROPERTY(EditDefaultsOnly, Category = "OnlyHands|Physics|MotionTracking")
    TSet<FName> TrackedBoneDefinitions;

    /** Bones excluded from simulation even if they are tracked and present in the physics asset */
    UPROPERTY(EditDefaultsOnly, Category = "OnlyHands|Physics|MotionTracking")
    TSet<FName> SimExclusionBoneSet;

    // === RUNTIME STATE ===

    /** Bones allowed to be simulated, based on current physics asset and exclusions */
    UPROPERTY(VisibleAnywhere, Category = "OnlyHands|Physics|MotionTracking")
    TSet<FName> SimulatableBones;

    /** Runtime motion data for each tracked bone */
    UPROPERTY(VisibleAnywhere, Category = "OnlyHands|Physics|MotionTracking")
    TMap<FName, FOHBoneData> TrackedBoneDataMap;

    // === LOGIC ===

    /** Updates kinematic motion tracking data for all tracked bones */
    void UpdateTrackedBoneKinematics(float DeltaTime);

    void UpdateBoneSimulationStates();

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    bool IsBoneTrackable(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "Physics|Motion")
    FORCEINLINE bool IsTrackedBone(FName Bone) const {
        return TrackedBoneDefinitions.Contains(Bone);
    }

    UFUNCTION(BlueprintPure, Category = "Physics|Simulation")
    FORCEINLINE bool IsSimulatableBone(FName Bone) const {
        return SimulatableBones.Contains(Bone);
    }

    UFUNCTION(BlueprintPure, Category = "OH|Physics|Simulation")
    FORCEINLINE TArray<FName> GetSimulatableBoneNames() const {
        return SimulatableBones.Array();
    }

    /** Debug: Print bone status to log */
    UFUNCTION(CallInEditor, Category = "OnlyHands|Debug")
    void PrintBoneTrackingInfo() const;

#pragma region BoneDataAccessors

    // -- Bone Data Accessors --

    /** Returns a const reference to all tracked bone data */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    const TMap<FName, FOHBoneData>& GetTrackedBoneData() const;

    // === Blueprint Motion Accessors ===

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    FVector GetTrackedBoneVelocity(FName Bone) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    FVector GetTrackedBoneLinearAcceleration(FName Bone) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    FVector GetTrackedBoneAngularVelocity(FName Bone) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    FVector GetTrackedBonePosition(FName Bone) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Motion")
    FRotator GetTrackedBoneRotation(FName Bone) const;

    /** Returns the list of currently tracked bones */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    TArray<FName> GetTrackedBoneNames() const;

    // -- Motion Accessors --

    FORCEINLINE const FOHBoneData* GetBoneData(FName Bone) const {
        return TrackedBoneDataMap.Find(Bone);
    }

    FORCEINLINE FVector GetVelocity(FName Bone) const {
        const FOHBoneData* Data = GetBoneData(Bone);
        return Data && Data->IsValid() ? Data->GetBodyLinearVelocity() : FVector::ZeroVector;
    }

    FORCEINLINE FVector GetLinearAcceleration(FName Bone) const {
        const FOHBoneData* Data = GetBoneData(Bone);
        return Data && Data->IsValid() ? Data->GetLinearAcceleration() : FVector::ZeroVector;
    }

    FORCEINLINE FVector GetAngularVelocity(FName Bone) const {
        const FOHBoneData* Data = GetBoneData(Bone);
        return Data && Data->IsValid() ? Data->GetBodyAngularVelocity() : FVector::ZeroVector;
    }

    FORCEINLINE FVector GetPosition(FName Bone) const {
        const FOHBoneData* Data = GetBoneData(Bone);
        return Data && Data->IsValid() ? Data->GetCurrentPosition() : FVector::ZeroVector;
    }

    FORCEINLINE FQuat GetRotation(FName Bone) const {
        const FOHBoneData* Data = GetBoneData(Bone);
        return Data && Data->IsValid() ? Data->GetCurrentRotation() : FQuat::Identity;
    }
#pragma endregion
#pragma endregion

#pragma region HitReaction

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void PlayPhysicsHitReaction(FName BoneName, EOHPhysicsProfile Profile, float BlendIn = 0.15f, float Hold = 0.25f,
                                float BlendOut = 0.2f, FVector OptionalImpulseDir = FVector::ZeroVector,
                                float OptionalImpulseStrength = 0.f, FName ReactionTag = NAME_None);

    /** Plays a custom PAC-driven hit reaction using a fully defined PAC profile */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void PlayPhysicsHitReactionCustom(FName BoneName, const FPhysicalAnimationData& CustomProfile, float BlendIn,
                                      float Hold, float BlendOut, FVector OptionalImpulseDir = FVector::ZeroVector,
                                      float OptionalImpulseStrength = 0.f, FName ReactionTag = NAME_None);

    // Internal utility for unified reaction logic
    void PlayPhysicsHitReaction_Internal(FName& BoneName, const FPhysicalAnimationData& ProfileData, float BlendIn,
                                         float Hold, float BlendOut, FVector OptionalImpulseDir,
                                         float OptionalImpulseStrength, FName ReactionTag,
                                         TArray<FName>* OutAffectedBones = nullptr // Optional output
    );
    void ApplySimSettingsToBone(const FName& Bone, const FPhysicalAnimationData& BaseProfile);

#pragma endregion

#pragma region Debug

#pragma region Debug_Special
  public:
    // === DEBUG CONFIGURATION ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug")
    bool bEnableDebugDisplay = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowSystemOverview = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowPhysicsGraphAnalysis = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowDirectPointerComparison = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowPerformanceMetrics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowBoneDetails = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    bool bShowConstraintDetails = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay && bShowBoneDetails"))
    TArray<FName> DebugSpecificBones;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    float DebugDisplayDuration = 0.0f; // 0 = permanent

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    FVector2D DebugScreenPosition = FVector2D(50.0f, 100.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug",
              meta = (EditCondition = "bEnableDebugDisplay"))
    float DebugTextScale = 1.0f;

  protected:
    // === DEBUG STATE TRACKING ===
    struct FDebugFrameData {
        float PhysicsGraphAccessTime = 0.0f;
        float DirectPointerAccessTime = 0.0f;
        int32 GraphTraversalCount = 0;
        int32 DirectLookupCount = 0;

        // Performance counters
        double LastFrameTime = 0.0;
        int32 ValidBoneCount = 0;
        int32 InvalidBoneCount = 0;
        int32 ActiveConstraintCount = 0;
        int32 InactiveConstraintCount = 0;
        int32 GraphMemoryFootprint = 0;
        int32 DirectMapMemoryFootprint = 0;

        void Reset() {
            PhysicsGraphAccessTime = 0.0f;
            DirectPointerAccessTime = 0.0f;
            GraphTraversalCount = 0;
            DirectLookupCount = 0;
            GraphMemoryFootprint = 0;
            DirectMapMemoryFootprint = 0;
            ValidBoneCount = 0;
            InvalidBoneCount = 0;
            ActiveConstraintCount = 0;
            InactiveConstraintCount = 0;
        }
    };

    mutable FDebugFrameData CurrentFrameDebugData;
    TArray<FDebugFrameData> DebugHistory;

    // Direct pointer comparison maps (for testing) - using weak pointers to avoid GC issues
    mutable TMap<FName, TWeakObjectPtr<UBodySetup>> DirectBodySetupMap;
    mutable TMap<FName, TWeakObjectPtr<UPhysicsConstraintTemplate>> DirectConstraintTemplateMap;
    mutable TMap<FName, FConstraintInstance*> DirectConstraintInstanceMap; // Runtime instances
    mutable TMap<FName, int32> DirectBoneIndexMap;
    mutable TMap<FName, FBodyInstance*> DirectBodyInstanceMap; // Safe - these are components, not UObjects

    // Performance timing helpers
    mutable double GraphOperationStartTime = 0.0;
    mutable double DirectOperationStartTime = 0.0;

  public:
    // === DEBUG FUNCTIONS ===

    /** Main debug tick function - call this from your main TickComponent */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
    void DebugTick(float DeltaTime);

    /** Forces a rebuild of direct pointer maps for comparison */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
    void RebuildDirectPointerMapsForTesting();

    /** Performs comprehensive analysis of both systems */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
    void PerformSystemComparison();

    /** Console command to toggle debug modes */
    UFUNCTION(CallInEditor, Category = "OnlyHands|Debug")
    void ToggleDebugMode(bool bNewEnabled);

  protected:
    // === INTERNAL DEBUG FUNCTIONS ===

    void DisplaySystemOverview() const;
    void DisplayPhysicsGraphAnalysis() const;
    void DisplayDirectPointerComparison() const;
    void DisplayPerformanceMetrics() const;
    void DisplayBoneDetails() const;
    void DisplayConstraintDetails() const;

    // Performance measurement helpers
    void StartGraphOperationTiming() const;
    void EndGraphOperationTiming() const;
    void StartDirectOperationTiming() const;
    void EndDirectOperationTiming() const;

    // Validation helpers
    bool ValidateGraphVsDirectConsistency() const;
    bool ValidateConstraintInstanceConsistency() const;
    TArray<FString> GetGraphInconsistencies() const;
    TArray<FString> GetDirectPointerIssues() const;

    // Memory calculation helper
    int32 CalculateGraphMemoryFootprint() const;

#pragma endregion

    // Normal Debug ---------------------------------------------------------//
    /** Visual scale multiplier for velocity vectors */
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Debug|Kinematics")
    float DebugVelocityScale = 0.05f;

    /** Visual scale multiplier for linear acceleration vectors */
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Debug|Kinematics")
    float DebugAccelerationScale = 0.01f;

    /** Visual scale multiplier for angular velocity vectors */
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Debug|Kinematics")
    float DebugAngularVelocityScale = 0.01f;

    /** Threshold above which angular velocity is tinted brighter */
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Debug|Kinematics")
    float AngularVelocityAlertThreshold = 500.0f;

    /** Threshold above which linear acceleration is tinted brighter */
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Debug|Kinematics")
    float AccelerationAlertThreshold = 1000.0f;

    /** Master toggle for all debug overlay drawing */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug")
    bool bEnableDebugOverlay = false;

    /** Show debug overlays for bone simulation blending */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug")
    bool bShowSimBlendOverlay = true;

    /** Show debug overlays for tracked bone kinematics (velocity, acceleration, angular velocity) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug")
    bool bShowTrackedBoneKinematics = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OnlyHands|Debug")
    bool bEnableRuntimeTuningEvaluation = false;
    ;

    /** Central entry point for drawing all debug overlays */
    void TickDebugOverlay() const;

    /** Draws debug info for tracked bone kinematics */
    void DrawTrackedBoneKinematicsDebug() const;

    /** Draws debug info for blend state of simulated bones */
    void DrawSimBlendDebugOverlay() const;

    UFUNCTION(CallInEditor, Category = "OnlyHands|Debug")
    void PrintSimulatedPhysicsBones() const;

    void EvaluateAndSuggestTuningForBones();
    bool IsBoneSimulating(FName BoneName) const;

    void ComputeOptimalConstraintSettings(const FConstraintInstance* ConstraintInstance,
                                          FConstraintProfileProperties& OutProfile, const FName& BoneName) const;

    FOHBoneData* GetBoneDataChecked(FName BoneName);
    void ComputePhysicsTweaksForBone(const FName& Bone, float& OutPACMultiplier, float& OutLinearDamping,
                                     float& OutAngularDamping) const;
    void UpdateConstraintRuntimeStates(float DeltaTime);

    FPhysicalAnimationData GetScaledPACProfile(const FName& Bone, const FPhysicalAnimationData& BaseProfile) const;

    void DrawDebugPACValues() const;

    void LogSimulatableBones() const;

    UFUNCTION(CallInEditor, Category = "OnlyHands|Debug",
              meta = (CallInEditor = "true", DisplayName = "Validate Skeleton Compatibility"))
    void EditorValidateSkeletonCompatibility();

#pragma endregion

#pragma region SimulationLifecycle

    void ActivateSimulationForChain(FName RootBone, const FPhysicalAnimationData& ProfileData, float Alpha);

    /** Increments sim ref count and activates simulation if this is the first activation */
    bool TryActivateSimForBone(FName BoneName, float BlendAlpha);

    /** Decrements sim ref count and clears simulation if no blends remain */
    bool TryDeactivateSimForBone(FName BoneName);

    /** Fully applies simulation settings to a bone body */
    bool ActivatePhysicsStateForBone(FName BoneName, float BlendAlpha);

    /** Fully clears physics simulation and PAC from a bone */
    void ClearPhysicsStateForBone(FName BoneName);

    void ClearAllSimulatedBones();

    /** Clears residual motion forces from a physics body */
    void ResetBoneForces(FName BoneName);

    /** Applies a unified physical animation profile to a single bone (PAC + constraint + body tuning) */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Simulation")
    void ApplyUnifiedPhysicsProfile(FName BoneName, const FPhysicalAnimationData& Profile,
                                    float CustomLinearDamping = 5.0f, float CustomAngularDamping = 10.0f,
                                    bool bVerboseLog = false);

    /** Recursively applies a unified physical animation profile to a bone and all of its children */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Simulation")
    void ApplyUnifiedPhysicsProfileToChain(FName RootBone, const FPhysicalAnimationData& Profile,
                                           float CustomLinearDamping = 5.0f, float CustomAngularDamping = 10.0f,
                                           bool bVerboseLog = false);
#pragma region BoneValidation

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    TArray<FName> GetSimulatedBones() const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    bool IsBoneValidForChain(FName Bone, FName RootBone) const;

    bool IsBoneInChain(FName Bone, FName RootBone) const;

    bool IsBoneSimulatable(const FName& Bone) const;

    /** Returns true if the bone is allowed to simulate based on name patterns */
    static bool IsBoneNamePatternValid(FName BoneName);

    /** Returns true if the bone has a valid, non-zero mass */
    bool IsBoneMassValid(FName BoneName) const;

    // Utility for querying if a bone has a physics body
    bool HasPhysicsBody(const FName& BoneName) const;

    bool ShouldSkipBone(const FName& Bone) const;

#pragma endregion

#pragma endregion

#pragma region BlendState

    static FActivePhysicsBlend CreatePhysicsBlendState(FName RootBone, float StartAlpha, float BlendIn, float Hold,
                                                       float BlendOut, FName ReactionTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void PauseBlend(FName Bone);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void ResumeBlend(FName Bone);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void PauseAllBlends();

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void ResumeAllBlends();

    void ForceUnpause(FName Bone);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetRemainingBlendTime(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetCurrentPhaseProgress(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    bool IsBoneBlending(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    EOHBlendPhase GetCurrentBlendPhase(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetBlendAlpha(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetPhaseDuration(FName BoneName, EOHBlendPhase Phase) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    FName GetActiveReactionTag(FName BoneName) const;

    static void UpdateBlendState(FActivePhysicsBlend& Blend, float DeltaTime);

    void ApplyBlendAlphaToBone(FName Bone, float BlendAlpha);

    static bool IsBlendComplete(const FActivePhysicsBlend& Blend);

    void FinalizeBlend(FName Bone);

    /** Finalizes all active physics blends and clears associated simulation and PAC */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void FinalizeAllBlends();

#pragma endregion

#pragma region Impulse

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void ApplyImpulseToBone(FName Bone, const FVector& Dir, float Strength);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics")
    void PropagateImpulse(FName OriginBone, FVector Impulse, float Strength, EOHPhysicsProfile Profile, int32 Depth);

#pragma endregion

#pragma region Delegates

    UPROPERTY(BlueprintAssignable, Category = "OnlyHands|Events")
    FOHBoneSimEvent OnBoneStartedSimulating;

    UPROPERTY(BlueprintAssignable, Category = "OnlyHands|Events")
    FOHBoneSimEvent OnBoneStoppedSimulating;

    UPROPERTY(BlueprintAssignable, Category = "OnlyHands|Events")
    FOHHitReactionComplete OnHitReactionComplete;

    UPROPERTY(BlueprintAssignable, Category = "OnlyHands|Events")
    FOHOnHitReactionBonesUpdated OnHitReactionBonesUpdated;
#pragma endregion

    // UFUNCTION(BlueprintCallable, Category = "OH Physics")
    // void ResetAndReapplyAllPhysics(bool bReapplyPAC = true);

    // UFUNCTION()
    // void HandleMeshTeleported();

  protected:
    void ProcessBlendPhases(float DeltaTime);

    // virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UPROPERTY()
    UPhysicsAsset* CachedPhysicsAsset = nullptr;

    UPROPERTY()
    USkeletalMeshComponent* SkeletalMesh = nullptr;

    UPROPERTY()
    UPhysicalAnimationComponent* PhysicalAnimationComponent = nullptr;

    UPROPERTY(EditAnywhere, Category = "OnlyHands|Physics")
    TMap<EOHPhysicsProfile, FPhysicalAnimationData> PhysicsProfiles;

    UPROPERTY()
    TMap<FName, FActivePhysicsBlend> ActiveBlends;

    UPROPERTY()
    TMap<FName, int32> BoneSimBlendRefCount;

    // === Cached per-bone physics properties for performance ===
    mutable TMap<FName, float> CachedBoneMasses;
    mutable TMap<FName, float> CachedBoneLengths;

#pragma region Utilities

    UFUNCTION(BlueprintCallable)
    FPhysicalAnimationData GetPhysicalAnimationData(FName BoneName,
                                                    EDriveAccessMode AccessMode = EDriveAccessMode::Live) const;
    UFUNCTION(BlueprintCallable)
    void SetPhysicalAnimationData(const FName BoneName, const FPhysicalAnimationData& Data);

    /** Compare two FPhysicalAnimationData structs (debug only) */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Debug")
    static bool ArePACProfilesEqual(const FPhysicalAnimationData& A, const FPhysicalAnimationData& B);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetPosePositionError(FName Bone) const;

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics")
    float GetPoseRotationError(FName Bone) const;

#pragma endregion

#pragma region BodyPartStatus
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OH|BodyParts")
    TMap<FName, FOHBodyPartStatus> BoneStatusMap;

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    void ApplyDamageToBone(FName Bone, float Damage);

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    bool IsBoneDestroyed(FName Bone) const;

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    float GetBoneHealth(FName Bone) const;

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    void ResetBoneStatus(FName Bone);

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    TArray<FName> GetSkeletalBonesInBodyPart(EOHBodyPart Part) const;

    UFUNCTION(BlueprintCallable, Category = "OH|BodyParts")
    bool IsBodyPartFullyDestroyed(EOHBodyPart Part) const;

#pragma endregion
    // === Skeletal Asset Validation ===
#pragma region SkeletalAssetValidation
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Validation")
    bool ValidateBoneMappings(bool bLogResults = true);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Validation")
    TArray<FString> GetMissingBones() const;

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Validation")
    TMap<FName, FName> GetBoneSuggestions() const;

    UFUNCTION(BlueprintCallable, Category = "PhysicsGraph")
    static TArray<FName> BP_GetChildrenOfBone(const FOHPhysicsGraphNode& Graph, FName ParentBone) {
        return Graph.GetChildrenOfBone(ParentBone);
    }

    UFUNCTION(BlueprintCallable, Category = "PhysicsGraph")
    static TArray<FOHConstraintInstanceData> BP_GetConstraintsOfBone(const FOHPhysicsGraphNode& Graph, FName BoneName) {
        return Graph.GetConstraintsOfBone(BoneName);
    }
#pragma endregion

#pragma region PhysicsGraph

  public:
    /** Checks all tracked bones and constraints for validity */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Validation")
    void ValidateAndRepairGraph();

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Graph")
    const FOHPhysicsGraphNode& GetPhysicsGraph() const {
        return PhysicsGraph;
    }

  protected:
    /** Internal validation pass */
    void PerformValidationPass();

    /** Internal repair pass */
    void PerformRepairPass();

    /** Attempt to recreate missing or invalid physics body for a bone */
    bool TryRecreateBodyInstance(const FName& BoneName);

    /** Attempt to recreate missing or invalid constraint between bones */
    bool TryRecreateConstraint(const FName& ParentBone, const FName& ChildBone);

    /** Helper: Checks if a body instance pointer is valid and simulating */
    static bool IsBodyInstanceValid(const FBodyInstance* BodyInstance);

    /** Helper: Checks if a constraint instance pointer is valid */
    static bool IsConstraintInstanceValid(const FConstraintInstance* ConstraintInstance);
    void OnSkeletalMeshChanged();
    void OnPhysicsAssetChanged();
    void OnGraphRelevantSettingsChanged();

    /** Holds bones and constraints marked invalid this frame */
    TSet<FName> InvalidBodies;
    TSet<FName> InvalidConstraints;

  private:
    // Validation cache
    UPROPERTY()
    FOHPhysicsGraphNode PhysicsGraph;

    // Cached graph version and last successful build version
    // Used to detect changes and trigger a rebuild
    mutable uint64 PhysicsGraphVersion = 0;   // Monotonically increasing version counter
    mutable uint64 LastGraphBuildVersion = 0; // Version at last successful graph build
    mutable bool bPhysicsGraphDirty = true;   // Set true whenever something changes

    // Cached bone data for performance
    UPROPERTY()
    TMap<FName, FOHResolvedBoneData> ResolvedBoneData;

    UPROPERTY()
    TMap<FName, FName> ValidatedBoneMapping;

    UPROPERTY()
    TArray<FName> MissingBones;

    UPROPERTY()
    TMap<FName, FName> BoneSuggestions;
    /** A reusable zero-strength PAC profile used to clear PAC data from bones */
    FPhysicalAnimationData CachedZeroStrengthPACProfile;
    TMap<FName, FPhysicalAnimationData> ActiveBonePACData;
    UPROPERTY(EditAnywhere, Category = "OnlyHands|Physics|Hierarchy", meta = (ShowOnlyInnerProperties))
    TMap<FName, FOHPhysicsBoneLink> BoneLinkMap;
    TMap<FName, TArray<FName>> ChildGraph;
};

#pragma endregion