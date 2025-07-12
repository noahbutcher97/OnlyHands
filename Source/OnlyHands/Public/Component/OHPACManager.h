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

UENUM(BlueprintType)
enum class EOHBoneType : uint8 {
    UpperBodyBones UMETA(DisplayName = "Upper Body"),
    LowerBodyBones UMETA(DisplayName = "Lower Body"),
    SpineBones UMETA(DisplayName = "Spine"),
    HeadBones UMETA(DisplayName = "Head"),
    LeftArmBones UMETA(DisplayName = "Left Arm"),
    RightArmBones UMETA(DisplayName = "Right Arm"),
    ArmBones UMETA(DisplayName = "All Arms"),
    ClavicleBones UMETA(DisplayName = "Clavicles"),
    UpperArmBones UMETA(DisplayName = "Upper Arms"),
    LowerArmBones UMETA(DisplayName = "Lower Arms"),
    HandBones UMETA(DisplayName = "Hands"),
    LeftLegBones UMETA(DisplayName = "Left Leg"),
    RightLegBones UMETA(DisplayName = "Right Leg"),
    LegBones UMETA(DisplayName = "All Legs"),
    ThighBones UMETA(DisplayName = "Thighs"),
    CalfBones UMETA(DisplayName = "Calves"),
    FootBones UMETA(DisplayName = "Feet"),
};

UENUM(BlueprintType)
enum class EOHProfileIntensity : uint8 {
    VeryLight UMETA(DisplayName = "Very Light"),
    Light UMETA(DisplayName = "Light"),
    Medium UMETA(DisplayName = "Medium"),
    Strong UMETA(DisplayName = "Strong"),
    VeryStrong UMETA(DisplayName = "Very Strong")
};

// Add enum for target alpha modes
UENUM(BlueprintType)
enum class EOHTargetAlphaMode : uint8 {
    Full UMETA(DisplayName = "Full Physics (1.0)"),
    IdlePose UMETA(DisplayName = "Idle Pose Retention"),
    Secondary UMETA(DisplayName = "Secondary Motion"),
    Subtle UMETA(DisplayName = "Subtle Physics"),
    Custom UMETA(DisplayName = "Custom Value")
};

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
#pragma region FOHConstraintDriveData
USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHConstraintDriveData {
    GENERATED_BODY()

    UPROPERTY()
    float LinearStiffnessX;
    UPROPERTY()
    float LinearStiffnessY;
    UPROPERTY()
    float LinearStiffnessZ;

    UPROPERTY()
    float LinearDampingX;
    UPROPERTY()
    float LinearDampingY;
    UPROPERTY()
    float LinearDampingZ;

    UPROPERTY()
    float AngularStiffnessSlerp;
    UPROPERTY()
    float AngularStiffnessSwing;
    UPROPERTY()
    float AngularStiffnessTwist;

    UPROPERTY()
    float AngularDampingSlerp;
    UPROPERTY()
    float AngularDampingSwing;
    UPROPERTY()
    float AngularDampingTwist;

    UPROPERTY()
    float LinearForceLimit;
    UPROPERTY()
    float AngularForceLimit;

    UPROPERTY()
    bool bLinearXDriveEnabled;
    UPROPERTY()
    bool bLinearYDriveEnabled;
    UPROPERTY()
    bool bLinearZDriveEnabled;

    UPROPERTY()
    bool bAngularSlerpDriveEnabled;
    UPROPERTY()
    bool bAngularSwingDriveEnabled;
    UPROPERTY()
    bool bAngularTwistDriveEnabled;

    FOHConstraintDriveData() {
        LinearStiffnessX = 0.f;
        LinearStiffnessY = 0.f;
        LinearStiffnessZ = 0.f;

        LinearDampingX = 0.f;
        LinearDampingY = 0.f;
        LinearDampingZ = 0.f;

        AngularStiffnessSlerp = 0.f;
        AngularStiffnessSwing = 0.f;
        AngularStiffnessTwist = 0.f;

        AngularDampingSlerp = 0.f;
        AngularDampingSwing = 0.f;
        AngularDampingTwist = 0.f;

        LinearForceLimit = 0.f;
        AngularForceLimit = 0.f;

        bLinearXDriveEnabled = false;
        bLinearYDriveEnabled = false;
        bLinearZDriveEnabled = false;

        bAngularSlerpDriveEnabled = false;
        bAngularSwingDriveEnabled = false;
        bAngularTwistDriveEnabled = false;
    }
};
#pragma endregion

#pragma region BoneProperties

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHBoneProperties {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FName BoneName = NAME_None;

    UPROPERTY(BlueprintReadOnly)
    float Mass = 1.0f;

    UPROPERTY(BlueprintReadOnly)
    float MomentOfInertia = 0.027f;

    UPROPERTY(BlueprintReadOnly)
    float Length = 20.0f;

    UPROPERTY(BlueprintReadOnly)
    int32 HierarchyLevel = 0;

    UPROPERTY(BlueprintReadOnly)
    FName Category = NAME_None;

    UPROPERTY(BlueprintReadOnly)
    bool bHasValidPhysics = false;

    UPROPERTY(BlueprintReadOnly)
    float EffectiveMass = 1.0f;

    UPROPERTY(BlueprintReadOnly)
    float StabilityFactor = 1.0f;

    FOHBoneProperties() {
        BoneName = NAME_None;
        Mass = 1.0f;
        MomentOfInertia = 0.027f;
        Length = 20.0f;
        HierarchyLevel = 0;
        Category = NAME_None;
        bHasValidPhysics = false;
        EffectiveMass = 1.0f;
        StabilityFactor = 1.0f;
    }
};
#pragma endregion

#pragma region FOHChainAnalysis
USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHChainAnalysis {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    TArray<FName> ChainBones;

    UPROPERTY(BlueprintReadOnly)
    int32 ChainLength = 0;

    UPROPERTY(BlueprintReadOnly)
    float TotalChainMass = 0.0f;

    UPROPERTY(BlueprintReadOnly)
    TArray<bool> SimulationStates; // Which bones are simulated

    UPROPERTY(BlueprintReadOnly)
    int32 ContinuousSimCount = 0; // Consecutive simulated bones

    UPROPERTY(BlueprintReadOnly)
    bool bHasKinematicAnchors = false; // Has kinematic endpoints

    UPROPERTY(BlueprintReadOnly)
    float StabilityRisk = 0.0f; // 0-1, higher = more unstable

    FOHChainAnalysis() {
        ChainLength = 0;
        TotalChainMass = 0.0f;
        ContinuousSimCount = 0;
        bHasKinematicAnchors = false;
        StabilityRisk = 0.0f;
    }
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
    bool bIsPermanent = false; // If true, never auto-blend-out

    UPROPERTY()
    float BlendOutDuration = 0.2f;

    UPROPERTY()
    float CustomTargetAlpha = 1.0f; // Custom target alpha (when not using 1.0)

    UPROPERTY()
    EOHTargetAlphaMode TargetAlphaMode = EOHTargetAlphaMode::Full;

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
    // Update IsComplete to handle permanent blends:
    FORCEINLINE bool IsComplete() const {
        // Permanent blends are never "complete" unless forced to blend out
        if (bIsPermanent && Phase == EOHBlendPhase::Permanent) {
            return false;
        }
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
#pragma region Static Bone Lookups

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    TSet<FName> TrackedBones = {"pelvis",     "spine_01",   "spine_02",   "spine_03",   "neck_01",
                                "head",       "clavicle_l", "clavicle_r", "upperarm_l", "upperarm_r",
                                "lowerarm_l", "lowerarm_r", "hand_l",     "hand_r",     "thigh_l",
                                "thigh_r",    "calf_l",     "calf_r",     "foot_l",     "foot_r"};
    // Bones that are excluded from simulation, but still tracked
    // This is useful for bones that are not simulated, but are still used for IK
    // For example, the hand bones are simulated, but the hand joints are not
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Tracking")
    TSet<FName> SimulationExclusions = {"root", "pelvis"};

    UPROPERTY()
    TArray<FName> UpperBodyBones = {"spine_01",   "spine_02",   "spine_03",   "clavicle_l", "clavicle_r",
                                    "upperarm_l", "upperarm_r", "lowerarm_l", "lowerarm_r", "hand_l",
                                    "hand_r",     "neck_01",    "head"};

    UPROPERTY()
    TArray<FName> LowerBodyBones = {"pelvis", "thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r"};

    UPROPERTY()
    TArray<FName> LeftLegBones = {"thigh_l", "calf_l", "foot_l"};

    UPROPERTY()
    TArray<FName> RightLegBones = {"thigh_r", "calf_r", "foot_r"};

    UPROPERTY()
    TArray<FName> LegBones = {"thigh_l", "thigh_r", "calf_l", "calf_r", "foot_l", "foot_r"};

    UPROPERTY()
    TArray<FName> ThighBones = {"thigh_l", "thigh_r"};

    UPROPERTY()
    TArray<FName> CalfBones = {"calf_l", "calf_r"};

    UPROPERTY()
    TArray<FName> FootBones = {"foot_l", "foot_r"};

    UPROPERTY()
    TArray<FName> SpineBones = {"spine_01", "spine_02", "spine_03"};

    UPROPERTY()
    TArray<FName> HeadBones = {"neck_01", "head"};

    UPROPERTY()
    TArray<FName> LeftArmBones = {"clavicle_l", "upperarm_l", "lowerarm_l", "hand_l"};

    UPROPERTY()
    TArray<FName> RightArmBones = {"clavicle_r", "upperarm_r", "lowerarm_r", "hand_r"};

    UPROPERTY()
    TArray<FName> ArmBones = {"clavicle_l", "upperarm_l", "clavicle_r", "upperarm_r",
                              "lowerarm_l", "lowerarm_r", "hand_l",     "hand_r"};

    UPROPERTY()
    TArray<FName> ClavicleBones = {"clavicle_l", "clavicle_r"};

    UPROPERTY()
    TArray<FName> UpperArmBones = {"upperarm_l", "upperarm_r"};

    UPROPERTY()
    TArray<FName> LowerArmBones = {"lowerarm_l", "lowerarm_r"};

    UPROPERTY()
    TArray<FName> HandBones = {"hand_l", "hand_r"};

    UFUNCTION(BlueprintPure, Category = "Bones")
    TArray<FName> GetBonesByType(EOHBoneType BoneType) const;

#pragma endregion
    // === PHYSICS PROFILES ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Profiles")
    TMap<EOHPhysicsProfile, FPhysicalAnimationData> PhysicsProfiles;

#pragma endregion

#pragma region TARGET ALPHA CONFIGURATION
    // === TARGET ALPHA PRESETS ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Target Alpha Presets",
              meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float IdlePoseRetentionAlpha = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Target Alpha Presets",
              meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float SecondaryMotionAlpha = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Target Alpha Presets",
              meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float HitReactionAlpha = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Target Alpha Presets",
              meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float SubtlePhysicsAlpha = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Target Alpha Presets",
              meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float MaximumPhysicsAlpha = 1.0f;

    // === BONE-SPECIFIC ALPHA SCALING ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Alpha Scaling")
    float SpineAlphaMultiplier = 0.8f; // Spine usually wants less physics

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Alpha Scaling")
    float ArmAlphaMultiplier = 1.0f; // Arms can handle full physics

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Alpha Scaling")
    float HandAlphaMultiplier = 0.9f; // Hands want mostly physics but some animation control

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Alpha Scaling")
    float NeckAlphaMultiplier = 0.6f; // Neck wants significant animation control
#pragma endregion

#pragma region CHAIN TUNING CONFIGURATION
    // === CHAIN STABILITY TUNING ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "0.5", ClampMax = "5.0", UIMin = "0.5", UIMax = "5.0"))
    float ChainRootStrengthMultiplier = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "0.5", ClampMax = "3.0", UIMin = "0.5", UIMax = "3.0"))
    float ChainMiddleStrengthMultiplier = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "0.5", ClampMax = "2.0", UIMin = "0.5", UIMax = "2.0"))
    float ChainEndStrengthMultiplier = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "1.0", ClampMax = "5.0", UIMin = "1.0", UIMax = "5.0"))
    float MaxStabilityRiskMultiplier = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "1.0", ClampMax = "4.0", UIMin = "1.0", UIMax = "4.0"))
    float MaxDampingMultiplier = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "1.0", ClampMax = "3.0", UIMin = "1.0", UIMax = "3.0"))
    float CoreBoneExtraMultiplier = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Chain Tuning",
              meta = (ClampMin = "1.0", ClampMax = "3.0", UIMin = "1.0", UIMax = "3.0"))
    float ContinuousChainMultiplier = 1.8f;

    // === PROFILE STRENGTH BOUNDS ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MinPositionStrength = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MaxPositionStrength = 25000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MinOrientationStrength = 5000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MaxOrientationStrength = 200000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MinVelocityStrength = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MaxVelocityStrength = 3000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MinAngularVelocityStrength = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Profile Bounds")
    float MaxAngularVelocityStrength = 20000.0f;

    // === TEST CONFIGURATION ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Testing")
    float DefaultTestDuration = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Testing")
    bool bUsePersistentSimulation = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Testing")
    float TestBlendInDuration = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC Manager|Testing")
    float TestBlendOutDuration = 0.5f;
#pragma endregion

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

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Validation")
    void FixBlendSystemDiscrepancies();

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

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics Blending")
    bool StartPhysicsBlend(FName BoneName, const FPhysicalAnimationData& Profile, float BlendInDuration = 0.2f,
                           float HoldDuration = -1.0f, // -1 = infinite hold
                           float BlendOutDuration = 0.3f, FName BlendTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics Blending")
    bool StartPhysicsBlendChain(FName RootBoneName, const FPhysicalAnimationData& Profile, float BlendInDuration = 0.2f,
                                float HoldDuration = -1.0f, float BlendOutDuration = 0.3f, FName BlendTag = NAME_None);
    bool StartPhysicsBlendWithAlpha(FName BoneName, const FPhysicalAnimationData& Profile, float TargetAlpha,
                                    float BlendInDuration, float HoldDuration, float BlendOutDuration, FName BlendTag);
    bool StartPhysicsBlendWithMode(FName BoneName, const FPhysicalAnimationData& Profile, EOHTargetAlphaMode AlphaMode,
                                   float BlendInDuration, float HoldDuration, float BlendOutDuration, FName BlendTag);

    bool StartPermanentPhysicsBlendWithAlpha(FName BoneName, const FPhysicalAnimationData& Profile, float TargetAlpha,
                                             float BlendInDuration, FName BlendTag);
    float GetTargetAlphaForMode(EOHTargetAlphaMode AlphaMode) const;
    float ApplyBoneAlphaScaling(FName BoneName, float BaseAlpha) const;
    FOHBlendState CreateSmartBlendStateWithAlpha(FName BoneName, float TargetAlpha, float BlendIn, float Hold,
                                                 float BlendOut, FName ReactionTag);
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics Blending")
    void StopPhysicsBlend(FName BoneName, float BlendOutDuration = 0.3f);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Physics Blending")
    void StopPhysicsBlendChain(FName RootBoneName, float BlendOutDuration = 0.3f);

    // === PERMANENT PHYSICS BLENDS ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Permanent Physics")
    bool StartPermanentPhysicsBlend(FName BoneName, const FPhysicalAnimationData& Profile, float BlendInDuration = 0.3f,
                                    FName BlendTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Permanent Physics")
    bool StartPermanentPhysicsBlendChain(FName RootBoneName, const FPhysicalAnimationData& Profile,
                                         float BlendInDuration = 0.3f, FName BlendTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Permanent Physics")
    void TransitionToPermanent(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Permanent Physics")
    void ForceBlendOutPermanent(FName BoneName, float BlendOutDuration = 0.5f);

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void TestPermanentPoseRetention();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Testing")
    void StopPermanentPoseRetention(float BlendOutDuration = 0.5f);

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
    static bool HasPhysicalAnimationDrives(const FConstraintInstance* Constraint);
    FConstraintInstance* FindPhysicalAnimationConstraint(FName BoneName) const;
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
    mutable TMap<FName, FBodyInstance*> BodyInstanceCache;
    mutable TMap<FName, FConstraintInstance*> ConstraintInstanceCache;
    mutable TMap<FName, int32> BoneIndexCache;

    // === CONSTRAINT DATA ===
    UPROPERTY()
    TMap<FName, FOHConstraintData> ConstraintDataMap;

    // === PERSISTENT SIMULATIONS ===
    UPROPERTY()
    TMap<FName, FName> PersistentSimulations; // BoneName -> SimulationTag

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

    // === ENHANCED BLEND PROCESSING ===
    static float CalculateEffectiveBlendAlpha(const TArray<FOHBlendState>& BoneBlends);
    bool HasActivePermanentBlend(FName BoneName) const;
    static FPhysicalAnimationData GetStrongerProfile(const FPhysicalAnimationData& ProfileA,
                                                     const FPhysicalAnimationData& ProfileB);
    int32 GetTotalActiveBlendCount() const;

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
    FBodyInstance* GetBodyInstanceDirect(FName BoneName) const;
    FConstraintInstance* GetConstraintInstanceDirect(FName BoneName) const;
    int32 GetBoneIndexDirect(FName BoneName) const;

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

#pragma region BoneAnalysis

  public:
    // === MATHEMATICAL PROFILE CALCULATION ===
    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Profile Calculation")
    FPhysicalAnimationData CalculateOptimalPACProfile(FName BoneName,
                                                      EOHProfileIntensity Intensity = EOHProfileIntensity::Medium,
                                                      bool bUseCriticalDamping = true,
                                                      float MassOverride = -1.0f) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Bone Analysis")
    FOHBoneProperties AnalyzeBoneProperties(FName BoneName, float MassOverride = -1.0f) const;

    // === TESTING & VALIDATION ===
    // Remove default arguments from UFUNCTION declarations
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void StartIdlePoseRetention(const TArray<FName>& TestBones, EOHProfileIntensity Intensity,
                                bool bValidateCalculations);

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void StopIdlePoseRetention(const TArray<FName>& BonesToStop, float BlendOutDuration = 0.5f);

    // Add convenience functions with no parameters for Blueprint/Editor use
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void StartIdlePoseRetentionDefault();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Testing")
    void StopIdlePoseRetentionDefault();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Analysis")
    void AnalyzeAllBones();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void QuickIdleTest();

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Testing")
    TArray<FName> GetIdleTestBones() const;

  private:
    // === BONE ANALYSIS HELPERS ===
    float CalculateActualBoneMass(FName BoneName) const;
    float CalculateActualMomentOfInertia(FName BoneName) const;
    static float EstimateBoneMassFromAnatomy(FName BoneName);
    static float EstimateMomentOfInertiaFromAnatomy(FName BoneName, float Mass);
    static FName ClassifyBoneByStructure(FName BoneName);
    float CalculateBoneLength(FName BoneName) const;
    static float EstimateBoneLengthFromAnatomy(FName BoneName);
    int32 CalculateHierarchyLevel(FName BoneName) const;
    float CalculateEffectiveMass(const FOHBoneProperties& Props) const;
    static float CalculateStabilityFactor(const FOHBoneProperties& Props);
    static float CalculateBoneScalar(const FOHBoneProperties& Props);
    bool HasValidPhysicsBody(FName BoneName) const;
    static float GetIntensityMultiplier(EOHProfileIntensity Intensity);

    // === POSE RETENTION HELPERS ===
    bool ApplyIdlePoseRetention(FName BoneName, EOHProfileIntensity Intensity);
    void ValidateBoneCalculations(const TArray<FName>& BonesToValidate);
    void LogPoseRetentionSummary(const TArray<FName>& TestedBones);

#pragma endregion

#pragma region ChainAnalysis
  public:
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void TestFullChainSimulation();

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Chain Analysis")
    FPhysicalAnimationData CalculateChainAwareProfile(FName BoneName,
                                                      EOHProfileIntensity BaseIntensity = EOHProfileIntensity::Medium,
                                                      bool bAccountForChainPosition = true) const;

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Chain Analysis")
    void ApplyChainStabilization(const TArray<FName>& ChainBones,
                                 EOHProfileIntensity BaseIntensity = EOHProfileIntensity::Medium);

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Chain Analysis")
    FOHChainAnalysis AnalyzeChain(FName RootBone) const;

  private:
    // === CHAIN ANALYSIS HELPERS ===
    FOHChainAnalysis AnalyzeChainForBone(FName BoneName) const;
    FName FindChainRoot(FName BoneName) const;
    float CalculateChainStabilityMultiplier(FName BoneName, const FOHChainAnalysis& ChainInfo) const;
    float CalculatePositionInChainMultiplier(FName BoneName, const FOHChainAnalysis& ChainInfo) const;
    float CalculateContinuityMultiplier(const FOHChainAnalysis& ChainInfo) const;

#pragma endregion

#pragma region ReversePACCalculations
    // ============================================================================
    // REVERSE PAC PROFILE CALCULATION FROM RUNTIME DATA
    // ============================================================================
  public:
    // Add to OHPACManager.h public API:
    UFUNCTION(BlueprintPure, Category = "PAC Manager|Profile Analysis")
    FPhysicalAnimationData GetCurrentPhysicalAnimationProfile(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC Manager|Profile Analysis")
    bool GetConstraintDriveParameters(FName BoneName, float& OutLinearStiffness, float& OutLinearDamping,
                                      float& OutAngularStiffness, float& OutAngularDamping) const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Analysis")
    void CompareCalculatedVsActualProfiles();
    static FPhysicalAnimationData GetBaselineProfile();
    static float CalculatePercentageDifference(float ValueA, float ValueB);
    FPhysicalAnimationData ReversePACProfileFromDrives(const FOHConstraintDriveData& DriveData, FName BoneName) const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Testing")
    void TestCustomAlphaBlends(EOHBoneType BoneType);

    UFUNCTION(BlueprintCallable, Category = "PAC Manager|Testing")
    void TestSubtlePhysicsMode(EOHBoneType BoneType);

  private:
    static bool GetConstraintDriveParametersFromInstance(FConstraintInstance* Constraint, float& OutLinearStiffness,
                                                         float& OutLinearDamping, float& OutAngularStiffness,
                                                         float& OutAngularDamping);
    FPhysicalAnimationData ConvertDriveParametersToPACProfile(FName BoneName, float LinearStiffness,
                                                              float LinearDamping, float AngularStiffness,
                                                              float AngularDamping) const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC Manager|Analysis")
    void AnalyzeCurrentPACConstraints();
    static FOHConstraintDriveData ExtractConstraintDriveData(const FConstraintInstance* Constraint);

    FConstraintInstance* FindPACConstraintForBone(FName BoneName) const;

#pragma endregion
};
