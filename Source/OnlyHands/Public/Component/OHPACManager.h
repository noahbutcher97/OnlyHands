#pragma once

#include "CoreMinimal.h"
#include "OHPhysicsStructs.h"
#include "Components/ActorComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "DrawDebugHelpers.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "OHPACManager.generated.h"

// Forward declarations
struct FOHMotionSample;
struct FOHConstraintRuntimeState;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOHPACBoneEvent, FName, BoneName);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOHPACHitReaction, FName, RootBone, const TArray<FName>&, AffectedBones);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnHitReactionStarted, FName, RootBone, const TArray<FName>&,
                                               AffectedBones, FName, ReactionTag);

// ============================================================================
// CORE DATA STRUCTURES
// ============================================================================
#pragma region CORE DATA STRUCTURES

#pragma region FOHBoneMotionData
USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHBoneMotionData {
    GENERATED_BODY()

  private:
    UPROPERTY()
    FName BoneName = NAME_None;

    UPROPERTY()
    FVector CurrentPosition = FVector::ZeroVector;

    UPROPERTY()
    FVector PreviousPosition = FVector::ZeroVector;

    UPROPERTY()
    FQuat CurrentRotation = FQuat::Identity;

    UPROPERTY()
    FQuat PreviousRotation = FQuat::Identity;

    UPROPERTY()
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector LinearAcceleration = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularAcceleration = FVector::ZeroVector;

    UPROPERTY()
    float LastDeltaTime = 0.016f;

    UPROPERTY()
    bool bIsSimulating = false;

    UPROPERTY()
    TArray<FOHMotionSample> MotionHistory;

    UPROPERTY()
    int32 MaxHistorySamples = 10;

  public:
    // === ACCESSORS ===
    FORCEINLINE FName GetBoneName() const {
        return BoneName;
    }
    FORCEINLINE FVector GetPosition() const {
        return CurrentPosition;
    }
    FORCEINLINE FVector GetPreviousPosition() const {
        return PreviousPosition;
    }
    FORCEINLINE FQuat GetRotation() const {
        return CurrentRotation;
    }
    FORCEINLINE FQuat GetPreviousRotation() const {
        return PreviousRotation;
    }
    FORCEINLINE FVector GetLinearVelocity() const {
        return LinearVelocity;
    }
    FORCEINLINE FVector GetAngularVelocity() const {
        return AngularVelocity;
    }
    FORCEINLINE FVector GetLinearAcceleration() const {
        return LinearAcceleration;
    }
    FORCEINLINE FVector GetAngularAcceleration() const {
        return AngularAcceleration;
    }
    FORCEINLINE bool IsSimulating() const {
        return bIsSimulating;
    }
    FORCEINLINE const TArray<FOHMotionSample>& GetHistory() const {
        return MotionHistory;
    }
    FORCEINLINE bool IsValid() const {
        return !BoneName.IsNone();
    }

    // === MUTATORS ===
    FORCEINLINE void SetBoneName(FName InName) {
        BoneName = InName;
    }
    FORCEINLINE void SetPosition(const FVector& Pos) {
        CurrentPosition = Pos;
    }
    FORCEINLINE void SetPreviousPosition(const FVector& Pos) {
        PreviousPosition = Pos;
    }
    FORCEINLINE void SetRotation(const FQuat& Rot) {
        CurrentRotation = Rot;
    }
    FORCEINLINE void SetPreviousRotation(const FQuat& Rot) {
        PreviousRotation = Rot;
    }
    FORCEINLINE void SetLinearVelocity(const FVector& Vel) {
        LinearVelocity = Vel;
    }
    FORCEINLINE void SetAngularVelocity(const FVector& AngVel) {
        AngularVelocity = AngVel;
    }
    FORCEINLINE void SetLinearAcceleration(const FVector& Accel) {
        LinearAcceleration = Accel;
    }
    FORCEINLINE void SetAngularAcceleration(const FVector& AngAccel) {
        AngularAcceleration = AngAccel;
    }
    FORCEINLINE void SetIsSimulating(bool bSim) {
        bIsSimulating = bSim;
    }
    FORCEINLINE void SetLastDeltaTime(float DT) {
        LastDeltaTime = FMath::Max(DT, 0.001f);
    }

    // === METHODS ===
    void UpdateKinematics(const FVector& NewPos, const FQuat& NewRot, float DeltaTime, float TimeStamp);
    void AddMotionSample(const FTransform& Transform, float TimeStamp);
    FVector GetAverageVelocity(int32 SampleCount = 5) const;
    float GetInstabilityScore() const;
    void Reset();
};
#pragma endregion

#pragma region FOHConstraintData
USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHConstraintData {
    GENERATED_BODY()

  private:
    UPROPERTY()
    FName ConstraintName = NAME_None;

    UPROPERTY()
    FName ParentBone = NAME_None;

    UPROPERTY()
    FName ChildBone = NAME_None;

    // Runtime constraint instance pointer (not serialized)
    FConstraintInstance* ConstraintInstance = nullptr;

    UPROPERTY()
    float CurrentStrain = 0.f;

    UPROPERTY()
    float PreviousStrain = 0.f;

    UPROPERTY()
    float JitterMetric = 0.f;

  public:
    // === ACCESSORS ===
    FORCEINLINE FName GetConstraintName() const {
        return ConstraintName;
    }
    FORCEINLINE FName GetParentBone() const {
        return ParentBone;
    }
    FORCEINLINE FName GetChildBone() const {
        return ChildBone;
    }
    FORCEINLINE FConstraintInstance* GetConstraintInstance() const {
        return ConstraintInstance;
    }
    FORCEINLINE float GetCurrentStrain() const {
        return CurrentStrain;
    }
    FORCEINLINE float GetJitterMetric() const {
        return JitterMetric;
    }
    FORCEINLINE bool IsValid() const {
        return !ConstraintName.IsNone() && ConstraintInstance != nullptr;
    }

    // === MUTATORS ===
    FORCEINLINE void SetConstraintName(FName Name) {
        ConstraintName = Name;
    }
    FORCEINLINE void SetParentBone(FName Bone) {
        ParentBone = Bone;
    }
    FORCEINLINE void SetChildBone(FName Bone) {
        ChildBone = Bone;
    }
    FORCEINLINE void SetConstraintInstance(FConstraintInstance* Instance) {
        ConstraintInstance = Instance;
    }

    // === METHODS ===
    void UpdateStrain();
    float GetSwingStrain() const;
    float GetTwistStrain() const;
    bool IsOverstressed(float Threshold = 1.5f) const;
};
#pragma endregion

#pragma region FOHBlendState
USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHBlendState {
    GENERATED_BODY()

    UPROPERTY()
    int32 BlendID = 0;

    UPROPERTY()
    FName RootBone = NAME_None;

    UPROPERTY()
    EOHBlendPhase Phase = EOHBlendPhase::BlendIn;

    UPROPERTY()
    float BlendAlpha = 0.f;

    UPROPERTY()
    float ElapsedTime = 0.f;

    UPROPERTY()
    float BlendInDuration = 0.15f;

    UPROPERTY()
    float HoldDuration = 0.25f;

    UPROPERTY()
    float BlendOutDuration = 0.2f;

    UPROPERTY()
    FName ReactionTag = NAME_None;

    UPROPERTY()
    int32 PauseCount = 0;
    // Smooth transition support
    UPROPERTY()
    float StartAlpha = 0.f; // Alpha to start blending from

    UPROPERTY()
    float TargetAlpha = 1.f; // Alpha to blend toward

    UPROPERTY()
    bool bInheritedFromPrevious = false; // Started from existing blend

    // ✅ NEW: Blend transition helpers
    FORCEINLINE bool IsInherited() const {
        return bInheritedFromPrevious;
    }
    FORCEINLINE float GetBlendRange() const {
        return TargetAlpha - StartAlpha;
    }
    FORCEINLINE bool IsPaused() const {
        return PauseCount > 0;
    }
    FORCEINLINE bool IsComplete() const {
        return Phase == EOHBlendPhase::BlendOut && ElapsedTime >= BlendOutDuration;
    }
    FORCEINLINE float GetTotalDuration() const {
        return BlendInDuration + HoldDuration + BlendOutDuration;
    }
};
#pragma endregion

#pragma endregion

// ============================================================================
// MAIN COMPONENT CLASS
// ============================================================================
#pragma region MAIN COMPONENT CLASS
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHPACManager : public UActorComponent {
    GENERATED_BODY()

  public:
    UOHPACManager();

    // === LIFECYCLE ===
#pragma region LIFECYCLE
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#pragma endregion

    // === CONFIGURATION ===
#pragma region CONFIGURATION
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager")
    bool bEnablePACManager = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager")
    float InitializationDelay = 0.1f;

    // Retry parameters
    UPROPERTY(EditDefaultsOnly, Category = "PAC Manager|Init")
    float InitRetryIntervalSeconds = 0.1f;

    UPROPERTY(EditDefaultsOnly, Category = "PAC Manager|Init")
    float InitRetryTotalDurationSeconds = 5.0f;

    // === AUTO SETUP ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Auto Setup")
    bool bAutoSetupPhysics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Auto Setup")
    FName PhysicsCollisionProfile = "PhysicsActor";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Auto Setup")
    bool bAutoUpdateOverlaps = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Auto Setup")
    bool bForceRecreateBodies = false;

    // === PHYSICS SETUP ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager")
    UPhysicsAsset* OverridePhysicsAsset = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    TSet<FName> TrackedBones = {"pelvis",     "spine_01",   "spine_02",   "spine_03",   "neck_01",
                                "head",       "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r",
                                "lowerarm_l", "lowerarm_r", "hand_l",     "hand_r",     "thigh_l",
                                "thigh_r",    "calf_l",     "calf_r",     "foot_l",     "foot_r"};

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    TSet<FName> SimulationExclusions = {"root", "pelvis", "neck_01"};

    // === PHYSICS PROFILES ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Profiles")
    TMap<EOHPhysicsProfile, FPhysicalAnimationData> PhysicsProfiles;

    // === DELEGATES ===
    UPROPERTY(BlueprintAssignable, Category = "PAC Events")
    FOHPACBoneEvent OnBoneStartedSimulating;

    UPROPERTY(BlueprintAssignable, Category = "PAC Events")
    FOHPACBoneEvent OnBoneStoppedSimulating;

    // (Add this with your other UPROPERTY delegates)

    UPROPERTY(BlueprintAssignable, Category = "PAC Events")
    FOnHitReactionStarted OnHitReactionStarted;

    UPROPERTY(BlueprintAssignable, Category = "PAC Events")
    FOHPACHitReaction OnHitReactionComplete;

    // === DEBUG ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Debug")
    bool bDrawDebug = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Debug")
    bool bVerboseLogging = false;

#pragma endregion

    // ========================================================================
    // PUBLIC API
    // ========================================================================
#pragma region PUBLIC API
    // === SYSTEM EVENTS ===
    UFUNCTION()
    void OnSkeletalMeshChanged();

    UFUNCTION(CallInEditor, Category = "PAC Manager|Debug")
    void OnSkeletalAssetChanged();

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    // === INITIALIZATION ===
    UFUNCTION()
    void StartInitializationRetry();

    UFUNCTION()
    void RetryInitializePACManager();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager")
    void InitializePACManager();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Setup")
    void BuildBoneChildrenMap();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager")
    void ResetPACManager();

    // === HIT REACTIONS ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Hit Reactions")
    void PlayHitReaction(FName BoneName, EOHPhysicsProfile Profile = EOHPhysicsProfile::Medium, float BlendIn = 0.15f,
                         float Hold = 0.25f, float BlendOut = 0.2f, FVector ImpulseDirection = FVector::ZeroVector,
                         float ImpulseStrength = 0.f, FName ReactionTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Hit Reactions")
    void PlayCustomHitReaction(FName BoneName, const FPhysicalAnimationData& CustomProfile, float BlendIn = 0.15f,
                               float Hold = 0.25f, float BlendOut = 0.2f,
                               FVector ImpulseDirection = FVector::ZeroVector, float ImpulseStrength = 0.f,
                               FName ReactionTag = NAME_None);

    // === BLEND CONTROL ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void PauseBlend(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void ResumeBlend(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void StopBlend(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void PauseAllBlendsForBone(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void ResumeAllBlendsForBone(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void StopAllBlendsForBone(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void StopAllBlends();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void PauseAllBlends();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Blending")
    void ResumeAllBlends();

    // === IMPULSE SYSTEM ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Impulse")
    void ApplyImpulse(FName BoneName, const FVector& Direction, float Magnitude);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Impulse")
    void ApplyImpulseToChain(FName RootBone, const FVector& Direction, float Magnitude, int32 Depth = 3);

    // === ACCESSORS ===
    UFUNCTION(BlueprintPure, Category = "PAC Manager|Motion")
    FVector GetBoneVelocity(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Motion")
    FVector GetBoneAcceleration(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Motion")
    bool IsBoneSimulating(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Blending")
    float GetBlendAlpha(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Blending")
    EOHBlendPhase GetBlendPhase(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Blending")
    int32 FindActiveBlendForBone(FName BoneName) const;

    FOHBlendState* GetActiveBlendForBone(FName BoneName);

    FOHBlendState* GetBlendByID(int32 BlendID);

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Constraints")
    float GetConstraintStrain(FName BoneName) const;

    // === SIMULATION CONTROL ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics")
    bool StartBonePhysicalAnimation(FName BoneName, const FPhysicalAnimationData& Profile, bool bEnableCollision = true,
                                    bool bWakeBody = true, bool bVerboseLog = false);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics")
    void StopBonePhysicalAnimation(FName BoneName, bool bClearVelocities, bool bPutToSleep, bool bVerboseLog);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics")
    bool StartChainPhysicalAnimation(FName RootBone, const FPhysicalAnimationData& Profile,
                                     bool bUseNativePropagation = true, bool bEnableCollision = true,
                                     bool bWakeBody = true, bool bVerboseLog = false);

    bool StartChainPhysicalAnimation_Filtered(FName RootBone, const FPhysicalAnimationData& Profile,
                                              bool bUseNativePropagation, bool bEnableCollision,
                                              TFunctionRef<bool(FName)> BoneFilter, bool bWakeBody, bool bVerboseLog);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics")
    void StopChainPhysicalAnimation(FName RootBone, bool bUseNativePropagation = true);

    void StopChainPhysicalAnimation_Filtered(FName RootBone, bool bUseNativePropagation,
                                             TFunctionRef<bool(FName)> BoneFilter, bool bClearVelocities,
                                             bool bPutToSleep, bool bVerboseLog);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics")
    void EnsureBoneSimulatingPhysics(FName BoneName, bool bEnableChain = true);

    // === VALIDATION ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Validation")
    bool ValidateBodyInstances(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies) const;

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Validation")
    bool ValidateConstraintInstances(TArray<FName>& OutMissingConstraints,
                                     TArray<FName>& OutRuntimeConstraintsNotInAsset,
                                     TArray<FName>& OutMismatchedConstraints) const;

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Validation")
    bool ValidatePhysicsAsset(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                              TArray<FName>& OutMissingConstraints, TArray<FName>& OutRuntimeConstraintsNotInAsset,
                              TArray<FName>& OutMismatchedConstraints) const;

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Validation")
    bool ValidateSetup(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                       TArray<FName>& OutMissingConstraints, TArray<FName>& OutRuntimeConstraintsNotInAsset,
                       TArray<FName>& OutMismatchedConstraints) const;

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Debug")
    bool IsSkeletalMeshBindingValid(bool bAutoFix = true, bool bLog = true) const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Debug")
    void LogSystemState() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Debug")
    void LogActiveBlends() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Debug")
    void LogSimState(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Debug")
    void DrawDebugOverlay() const;
    bool IsBoneDrivenByPhysicalAnimation(const FName& BoneName) const;
    void LogAllPhysicallyDrivenBonesWithDriveValues() const;
    void VisualizeActivePhysicalAnimationDrives() const;
    void CheckSimulatingBonesForPhysicalAnimationDrives() const;
    void DebugBodyInstanceSimulation() const;
    void DebugPhysicalAnimationConstraints();
    void DebugBodyPhysicsStates();

    // === BONE CHAIN ACCESS ===
    UFUNCTION(BlueprintPure, Category = "PAC Manager|Utility")
    TArray<FName> GetBoneChain(FName RootBone, int32 MaxDepth = -1) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Utility")
    TArray<FName> GetSimulatableBones() const {
        return SimulatableBones.Array();
    }

    /** Returns all FBodyInstance* for the given SkeletalMesh whose bone names are in SimulatableBones */
    static TArray<FBodyInstance*> GetSimulatableBodies(USkeletalMeshComponent* SkeletalMesh,
                                                       const TSet<FName>& SimulatableBones);

    static void GetSimulatingBodiesBySimulatable(USkeletalMeshComponent* MeshComp, const TSet<FName>& SimulatableBones,
                                                 TArray<FBodyInstance*>& OutValidSim,
                                                 TArray<FBodyInstance*>& OutInvalidSim);
#pragma endregion

  private:
    // ========================================================================
    // INTERNAL DATA
    // ========================================================================
#pragma region INTERNAL DATA
    // === COMPONENT REFERENCES ===
    UPROPERTY()
    USkeletalMeshComponent* SkeletalMesh = nullptr;

    UPROPERTY()
    UPhysicalAnimationComponent* PhysicalAnimationComponent = nullptr;

    UPROPERTY()
    UPhysicsAsset* CachedPhysicsAsset = nullptr;

    UPROPERTY(Transient)
    USkeletalMesh* PreviousMeshAsset = nullptr;

    UPROPERTY(Transient)
    UPhysicsAsset* PreviousPhysicsAsset = nullptr;

    // === MOTION TRACKING ===
    UPROPERTY()
    TMap<FName, FOHBoneMotionData> BoneMotionMap;

    // === DIRECT ACCESS CACHES ===
    TMap<FName, FBodyInstance*> BodyInstanceCache;
    TMap<FName, FConstraintInstance*> ConstraintInstanceCache;
    TMap<FName, int32> BoneIndexCache;

    // === CONSTRAINT DATA ===
    UPROPERTY()
    TMap<FName, FOHConstraintData> ConstraintDataMap;

    // === BLEND STATES ===
    TMap<FName, TArray<FOHBlendState>> ActiveBlends;

    UPROPERTY()
    int32 NextBlendID = 1;

    UPROPERTY()
    TMap<FName, int32> BoneSimulationRefCount;

    // === HIERARCHY MAPS ===
    TMap<FName, FName> BoneParentMap;
    TMap<FName, TArray<FName>> BoneChildrenMap;

    // === SIMULATABLE BONES ===
    UPROPERTY()
    TSet<FName> SimulatableBones;

    // === INITIALIZATION STATE ===
    FTimerHandle InitRetryHandle;
    float InitRetryElapsed = 0.0f;
    bool bIsInitialized = false;

    // === AUTO SETUP STATE ===
    bool bAutoSetupComplete = false;
    FTimerHandle AutoSetupRetryTimer;
    int32 AutoSetupRetryCount = 0;
    static constexpr int32 MaxAutoSetupRetries = 10;

    // === COLLISION PROFILE MANAGEMENT ===
    FName OriginalCollisionProfile;
    bool bHasStoredOriginalProfile = false;

    // === ZERO PROFILE CACHE ===
    FPhysicalAnimationData ZeroProfile;

    // === CLEANUP TIMER ===
    FTimerHandle CleanupTimer;
    float CleanupInterval = 1.0f;

#pragma endregion

    // ========================================================================
    // INTERNAL METHODS
    // ========================================================================
#pragma region INTERNAL METHODS
    // === AUTO SETUP ===
    void PerformAutoSetup();
    void RetryAutoSetup();
    void SetupPhysicsAsset();
    void ConfigureCollisionSettings();
    void RestoreOriginalCollisionSettings();
    void ValidatePhysicsSimulation();
    void EnsurePhysicsStateValid();

    // === INITIALIZATION ===
    void FindComponents();
    void BuildDirectCaches();
    void BuildHierarchyMaps();
    void BuildConstraintData();
    void InitializeMotionTracking();
    void DetermineSimulatableBones();
    bool ArePhysicsBodiesReady() const;

    // === MOTION TRACKING ===
    void UpdateMotionTracking(float DeltaTime);
    void UpdateConstraintStates(float DeltaTime);
    FOHBlendState CreateSmartBlendState(FName BoneName, float BlendIn, float Hold, float BlendOut, FName ReactionTag);
    void AddBlendToBone(FName BoneName, const FOHBlendState& BlendState);

    // === BLEND PROCESSING ===
    void ProcessActiveBlends(float DeltaTime);
    void CleanupStaleBlends();
    void RemoveCompletedBlends();
    static void UpdateBlendState(FOHBlendState& Blend, float DeltaTime);
    void ApplyBlendAlpha(FName BoneName, float Alpha);
    void FinalizeBlend(FName BoneName);
    void FinalizeAllBlendsForBone(FName BoneName);
    void FinalizeBlendByID(FName BoneName, int32 BlendID);

    // === SIMULATION CONTROL ===
    bool StartSimulation(FName BoneName, const FPhysicalAnimationData& Profile, bool bAllBelow = false,
                         bool bEnableCollision = true);
    void StopSimulation(FName BoneName, bool bAllBelow = false);
    void ApplyPhysicalAnimationProfile(FName BoneName, const FPhysicalAnimationData& Profile);
    void ClearPhysicalAnimationProfile(FName BoneName);
    bool TryActivateSimForBone(FName BoneName, const FPhysicalAnimationData& Profile);
    bool TryDeactivateSimForBone(FName BoneName);
    bool ActivatePhysicsStateForBone(FName BoneName, float BlendAlpha);
    void ClearPhysicsStateForBone(FName BoneName);
    void WakePhysicsBody(FName BoneName);

    // === DIRECT ACCESS HELPERS ===
    FBodyInstance* GetBodyInstanceDirect(FName BoneName);
    FConstraintInstance* GetConstraintInstanceDirect(FName BoneName);
    int32 GetBoneIndexDirect(FName BoneName);

    // === BONE VALIDATION ===
    bool IsBoneValidForSimulation(FName BoneName) const;
    bool IsBoneInChain(FName BoneName, FName RootBone) const;
    static bool IsBoneNamePatternValid(FName BoneName);
    bool IsBoneMassValid(FName BoneName) const;
    bool HasPhysicsBody(const FName& BoneName) const;

    // === UTILITY ===
    void InvalidateCaches();
    void LogPerformanceStats() const;
    static void SafeLog(const FString& Message, bool bWarning = false, bool bOnScreen = false);

#pragma endregion

#pragma endregion
};