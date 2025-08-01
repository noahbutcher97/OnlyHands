// ============================================================================
// OHPACManager.h
// PAC (Physical Animation Component) Manager for OnlyHands Project
// Streamlined version - Unified functions and removed duplicates
// ============================================================================

#pragma once

#include "CoreMinimal.h"
#include "OHMovementComponent.h"
#include "Components/ActorComponent.h"
#include "Engine/EngineTypes.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Utilities/OHSafeMapUtils.h"
#include "OHPACManager.generated.h"

<<<<<<< HEAD
#define CHECK_BODY_VALID(BodyPtr, BoneName)                                                                            \
    if (!BodyPtr || !BodyPtr->IsValidBodyInstance()) {                                                                 \
        SafeLog(FString::Printf(TEXT("Invalid or stale BodyInstance for %s at %s:%d"), *BoneName.ToString(),           \
                                TEXT(__FILE__), __LINE__),                                                             \
                true);                                                                                                 \
        RefreshBodyInstances();                                                                                        \
        BodyPtr = GetBodyInstanceDirect(BoneName);                                                                     \
        if (!BodyPtr || !BodyPtr->IsValidBodyInstance()) {                                                             \
            SafeLog(FString::Printf(TEXT("BodyInstance unrecoverable for %s at %s:%d"), *BoneName.ToString(),          \
                                    TEXT(__FILE__), __LINE__),                                                         \
                    true);                                                                                             \
            return;                                                                                                    \
        }                                                                                                              \
    }
    == == ==
    = DECLARE_LOG_CATEGORY_EXTERN(LogPACManager, Log, All);
#define CHECK_BODY_VALID(BodyPtr, BoneName)                                                                            \
    if (!BodyPtr || !BodyPtr->IsValidBodyInstance()) {                                                                 \
        SafeLog(FString::Printf(TEXT("Invalid or stale BodyInstance for %s at %s:%d"), *BoneName.ToString(),           \
                                TEXT(__FILE__), __LINE__),                                                             \
                true);                                                                                                 \
        RefreshBodyInstances();                                                                                        \
        BodyPtr = GetBodyInstanceDirect(BoneName);                                                                     \
        if (!BodyPtr || !BodyPtr->IsValidBodyInstance()) {                                                             \
            SafeLog(FString::Printf(TEXT("BodyInstance unrecoverable for %s at %s:%d"), *BoneName.ToString(),          \
                                    TEXT(__FILE__), __LINE__),                                                         \
                    true);                                                                                             \
            return;                                                                                                    \
        }                                                                                                              \
    }
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

    // Forward declarations
    class UPhysicalAnimationComponent;
class USkeletalMeshComponent;
class UPhysicsAsset;
class UPhysicsConstraintTemplate;
struct FBodyInstance;
struct FConstraintInstance;

// ============================================================================
// ENUMS
// ============================================================================
#pragma region ENUMS

UENUM(BlueprintType)
<<<<<<< HEAD
enum class EOHBoneCategory : uint8 {
    Custom UMETA(DisplayName = "Custom"),
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
enum class EOHPhysicsStrength : uint8 {
    VeryLight UMETA(DisplayName = "Very Light"),
    Light UMETA(DisplayName = "Light"),
    Medium UMETA(DisplayName = "Medium"),
    Strong UMETA(DisplayName = "Strong"),
    VeryStrong UMETA(DisplayName = "Very Strong"),
    Custom UMETA(DisplayName = "Custom")
};

UENUM(BlueprintType)
enum class EOHBlendPhase : uint8 { Inactive, BlendIn, Hold, BlendOut, Permanent };
== == == = enum class EOHBoneCategory : uint8 {
    Custom UMETA(DisplayName = "Custom"),
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
enum class EOHPhysicsStrength : uint8 {
    VeryLight UMETA(DisplayName = "Very Light"),
    Light UMETA(DisplayName = "Light"),
    Medium UMETA(DisplayName = "Medium"),
    Strong UMETA(DisplayName = "Strong"),
    VeryStrong UMETA(DisplayName = "Very Strong"),
    Custom UMETA(DisplayName = "Custom")
};

UENUM(BlueprintType)
enum class EOHBlendPhase : uint8 { Inactive, BlendIn, Hold, BlendOut, Permanent };
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

#pragma endregion

// ============================================================================
// STRUCTS
// ============================================================================
#pragma region STRUCTS

<<<<<<< HEAD struct FCombatTargetCandidate { FName BoneName; FVector Position; float Distance; float Priority; // Core bones get higher priority bool bIsCore; bool bIsExtremity; float TimeToImpact; bool bIsBlocking = false; // This was missing float PenetrationDepth; FVector ImpactDirection;
}
;
/*
=======

struct FCombatTargetCandidate
{
    FName BoneName;
    FVector Position;
    float Distance;
    float Priority;  // Core bones get higher priority
    bool bIsCore;
    bool bIsExtremity;
    float TimeToImpact;
    bool bIsBlocking = false;  // This was missing
    float PenetrationDepth;
    FVector ImpactDirection;
};

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
USTRUCT(BlueprintType)
struct FOHBroadphaseZone
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName BoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Radius = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsActive = true;

    // Runtime data - changed from TArray<FOverlapResult> to TArray<FHitResult>
    TArray<FHitResult> LastHits;  // Changed from LastOverlaps
    float LastSweepTime = 0.0f;
};

<<<<<<< HEAD
*/

USTRUCT(BlueprintType)
struct FOHConstraintData {
    == == == =

                 USTRUCT(BlueprintType) struct FOHConstraintData {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        GENERATED_BODY()

        UPROPERTY(BlueprintReadOnly)
        FName ConstraintBone1 = NAME_None;

        UPROPERTY(BlueprintReadOnly)
        FName ConstraintBone2 = NAME_None;

        FConstraintInstance* ConstraintInstance = nullptr;

        UPROPERTY(BlueprintReadOnly)
        float CurrentStrain = 0.0f;

        UPROPERTY(BlueprintReadOnly)
        float MaxRecordedStrain = 0.0f;

        UPROPERTY(BlueprintReadOnly)
        float JitterMetric = 0.0f;

<<<<<<< HEAD
        FORCEINLINE bool IsValid() const {
            return ConstraintInstance != nullptr && ConstraintBone1 != NAME_None && ConstraintBone2 != NAME_None;
        }

        FORCEINLINE FConstraintInstance* GetConstraintInstance(const USkeletalMeshComponent* SkelMesh) const {
            if (!SkelMesh)
                return nullptr;
            return UOHSkeletalPhysicsUtils::GetActiveConstraintBetweenBones(SkelMesh, ConstraintBone1, ConstraintBone2);
        }
        FORCEINLINE bool IsConstraintBroken(const USkeletalMeshComponent* SkelMesh) const {
            if (!SkelMesh)
                return false;
            if (!GetConstraintInstance(SkelMesh)) {
                return false;
            }
            FConstraintInstance* ConstraintInstanceTemp = GetConstraintInstance(SkelMesh);
            return ConstraintInstanceTemp->IsBroken();
            == == == = FORCEINLINE bool IsValid() const {
                return ConstraintInstance != nullptr && ConstraintBone1 != NAME_None && ConstraintBone2 != NAME_None;
            }

            FORCEINLINE FConstraintInstance* GetConstraintInstance(const USkeletalMeshComponent* SkelMesh) const {
                if (!SkelMesh)
                    return nullptr;
                return UOHSkeletalPhysicsUtils::GetActiveConstraintBetweenBones(SkelMesh, ConstraintBone1,
                                                                                ConstraintBone2);
            }
            FORCEINLINE bool IsConstraintBroken(const USkeletalMeshComponent* SkelMesh) const {
                if (!SkelMesh)
                    return false;
                if (!GetConstraintInstance(SkelMesh)) {
                    return false;
                }
                FConstraintInstance* ConstraintInstanceTemp = GetConstraintInstance(SkelMesh);
                return ConstraintInstanceTemp->IsBroken();
        
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
            }
            void UpdateStrain(USkeletalMeshComponent * SkeletalMesh, float DeltaTime);
            FVector GetSwingStrain(USkeletalMeshComponent * SkeletalMesh) const;
            float GetTwistStrain(USkeletalMeshComponent * SkeletalMesh) const;
            bool IsOverstressed(float Threshold = 0.8f) const;
        };

        USTRUCT(BlueprintType)
<<<<<<< HEAD
        struct FOHConstraintDriveData {
            == == == = struct FOHConstraintDriveData {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                GENERATED_BODY()

                UPROPERTY()
                float LinearStiffnessX = 0.f;
<<<<<<< HEAD

                UPROPERTY()
                float LinearStiffnessY = 0.f;

                == == == =

                             UPROPERTY() float LinearStiffnessY = 0.f;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                float LinearStiffnessZ = 0.f;

                UPROPERTY()
                float LinearDampingX = 0.f;
<<<<<<< HEAD

                UPROPERTY()
                float LinearDampingY = 0.f;

                == == == =

                             UPROPERTY() float LinearDampingY = 0.f;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                float LinearDampingZ = 0.f;

                UPROPERTY()
                float AngularStiffnessSlerp = 0.f;
<<<<<<< HEAD

                UPROPERTY()
                float AngularStiffnessSwing = 0.f;

                == == == =

                             UPROPERTY() float AngularStiffnessSwing = 0.f;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                float AngularStiffnessTwist = 0.f;

                UPROPERTY()
                float AngularDampingSlerp = 0.f;
<<<<<<< HEAD

                UPROPERTY()
                float AngularDampingSwing = 0.f;

                == == == =

                             UPROPERTY() float AngularDampingSwing = 0.f;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                float AngularDampingTwist = 0.f;

                UPROPERTY()
                float LinearForceLimit = 0.f;
<<<<<<< HEAD

                == == == =
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                             UPROPERTY() float AngularForceLimit = 0.f;

                UPROPERTY()
                bool bLinearXDriveEnabled = false;
<<<<<<< HEAD

                UPROPERTY()
                bool bLinearYDriveEnabled = false;

                == == == =

                             UPROPERTY() bool bLinearYDriveEnabled = false;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                bool bLinearZDriveEnabled = false;

                UPROPERTY()
                bool bAngularSlerpDriveEnabled = false;
<<<<<<< HEAD

                UPROPERTY()
                bool bAngularSwingDriveEnabled = false;

                == == == =

                             UPROPERTY() bool bAngularSwingDriveEnabled = false;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UPROPERTY()
                bool bAngularTwistDriveEnabled = false;
            };

            USTRUCT(BlueprintType)
<<<<<<< HEAD
            struct FOHPhysicsBlendParams {
                == == == = struct FOHPhysicsBlendParams {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    GENERATED_BODY()

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    FPhysicalAnimationData Profile;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    float BlendInDuration = 0.2f;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    float HoldDuration = 0.25f;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    float BlendOutDuration = 0.2f;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    float TargetAlpha = 1.0f;
<<<<<<< HEAD

                    == == == =
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                 // REMOVED: float BlendOutAlpha = 0.0f;

                        UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bIsPermanent = false;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    FName ReactionTag = NAME_None;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    bool bAffectChain = false;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    bool bEnableCollision = true;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite)
                    TArray<FName> FilterBones;

<<<<<<< HEAD
                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Advanced",
                              meta = (DisplayName = "Use Native Force Propagation",
                                      ToolTip = "When affecting chains, use Unreal's built-in force falloff "
                                                "calculation (recommended for stability)"))
                    == == == = UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Advanced",
                                         meta = (DisplayName = "Use Native Force Propagation",
                                                 ToolTip = "When affecting chains, use Unreal's built-in force falloff "
                                                           "calculation (recommended for stability)"))
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                 bool bEnableNativeForcePropagation = true;

                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Blend")
                    int32 Priority = 0;
<<<<<<< HEAD

                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Blend")
                    float ImpactStrength = 1.0f;
                };

                == == == =

                             UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Blend") float ImpactStrength =
                                 1.0f;
            };


>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
            // ============================================================================
            // SIMPLIFIED TIMELINE-BASED PHYSICS BLENDING
            // ============================================================================

            // ============================================================================

            USTRUCT()
<<<<<<< HEAD
            struct FOHActiveBlend {
                == == == = struct FOHActiveBlend {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    GENERATED_BODY()

                    UPROPERTY()
                    FName BoneName = NAME_None;

                    UPROPERTY()
                    FPhysicalAnimationData PhysicsProfile_Current;
<<<<<<< HEAD

                    UPROPERTY()
                    FPhysicalAnimationData PhysicsProfile_Start;

                    UPROPERTY()
                    FPhysicalAnimationData PhysicsProfile_Target;

                    UPROPERTY()
                    float PhysicsBlendWeight_Current = 0.0f;

                    UPROPERTY()
                    float PhysicsBlendWeight_Start = 0.0f;

                    UPROPERTY()
                    float PhysicsBlendWeight_Target = 1.0f;

                    UPROPERTY()
                    EOHBlendPhase CurrentPhase = EOHBlendPhase::Inactive;

                    UPROPERTY()
                    float ElapsedTime = 0.0f;
                    == == == =

                                 UPROPERTY() FPhysicalAnimationData PhysicsProfile_Start;

                    UPROPERTY()
                    FPhysicalAnimationData PhysicsProfile_Target;

                    UPROPERTY()
                    float PhysicsBlendWeight_Current = 0.0f;

                    UPROPERTY()
                    float PhysicsBlendWeight_Start = 0.0f;

                    UPROPERTY()
                    float PhysicsBlendWeight_Target = 1.0f;

                    UPROPERTY()
                    EOHBlendPhase CurrentPhase = EOHBlendPhase::Inactive;

                    UPROPERTY()
                    float ElapsedTime = 0.0f; 
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

                    UPROPERTY()
                    float BlendInDuration = 0.2f;

                    UPROPERTY()
                    float HoldDuration = 0.25f;

                    UPROPERTY()
                    float BlendOutDuration = 0.2f;
<<<<<<< HEAD

                    UPROPERTY()
                    bool bIsPermanent = false;

                    UPROPERTY()
                    bool bForceComplete = false;
                    == == == =
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

                                 UPROPERTY() bool bIsPermanent = false;

                    UPROPERTY()
                    bool bForceComplete = false;

                    UPROPERTY()
                    FName ReactionTag = NAME_None;

                    UPROPERTY()
                    int32 PauseCount = 0;
<<<<<<< HEAD

                    UPROPERTY()
                    float CompletionThreshold = 0.005f;

                    UPROPERTY()
                    float StartTime = 0.0f;

                    UPROPERTY()
                    float TotalDuration = 0.0f;

                    UPROPERTY()
                    bool bIsBlendOutOnly = false;

                    UPROPERTY()
                    bool bUsedNativePropagation = false;

                    UPROPERTY()
                    bool bIsRootOfPropagation = false;

                    UPROPERTY()
                    FName PropagationRootBone = NAME_None;

                    UPROPERTY()
                    bool bReturnToBaseline = false;

                    void InitializeTiming(float WorldTime) {
                        StartTime = WorldTime;
                        ElapsedTime = 0.0f;

                        // Calculate total duration based on blend type
                        if (bIsBlendOutOnly) {
                            TotalDuration = BlendOutDuration;
                            CurrentPhase = EOHBlendPhase::BlendOut;
                        } else if (bIsPermanent) {
                            TotalDuration = BlendInDuration;
                            CurrentPhase = EOHBlendPhase::BlendIn;
                        } else {
                            == == == =

                                         UPROPERTY() float CompletionThreshold = 0.005f;

                            UPROPERTY()
                            float StartTime = 0.0f;

                            UPROPERTY()
                            float TotalDuration = 0.0f;

                            UPROPERTY()
                            bool bIsBlendOutOnly = false;

                            UPROPERTY()
                            bool bUsedNativePropagation = false;

                            UPROPERTY()
                            bool bIsRootOfPropagation = false;

                            UPROPERTY()
                            FName PropagationRootBone = NAME_None;

                            UPROPERTY()
                            bool bReturnToBaseline = false;

                            void InitializeTiming(float WorldTime) {
                                StartTime = WorldTime;
                                ElapsedTime = 0.0f;

                                // Calculate total duration based on blend type
                                if (bIsBlendOutOnly) {
                                    TotalDuration = BlendOutDuration;
                                    CurrentPhase = EOHBlendPhase::BlendOut;
                                } else if (bIsPermanent) {
                                    TotalDuration = BlendInDuration;
                                    CurrentPhase = EOHBlendPhase::BlendIn;
                                } else {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                    // Temporary blend has all three phases
                                    TotalDuration = BlendInDuration + HoldDuration + BlendOutDuration;
                                    CurrentPhase = EOHBlendPhase::BlendIn;
                                }
                            }
<<<<<<< HEAD

                            float CalculateWeightAtTime(float WorldTime) const {
                                float TimeElapsed = WorldTime - StartTime;

                                if (TimeElapsed <= 0.0f) {
                                    return PhysicsBlendWeight_Start;
                                }

                                if (bIsBlendOutOnly) {
                                    float Progress = FMath::Clamp(TimeElapsed / BlendOutDuration, 0.0f, 1.0f);
                                    float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                    return FMath::Lerp(PhysicsBlendWeight_Start, 0.0f, SmoothProgress);
                                } else if (bIsPermanent) {
                                    // For permanent blends, stay at target weight after blend in
                                    if (TimeElapsed >= BlendInDuration) {
                                        return PhysicsBlendWeight_Target; // Stay at target weight forever
                                    } else {
                                        == == == =

                                                     float CalculateWeightAtTime(float WorldTime) const {
                                            float TimeElapsed = WorldTime - StartTime;

                                            if (TimeElapsed <= 0.0f) {
                                                return PhysicsBlendWeight_Start;
                                            }

                                            if (bIsBlendOutOnly) {
                                                float Progress =
                                                    FMath::Clamp(TimeElapsed / BlendOutDuration, 0.0f, 1.0f);
                                                float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                                return FMath::Lerp(PhysicsBlendWeight_Start, 0.0f, SmoothProgress);
                                            } else if (bIsPermanent) {
                                                // For permanent blends, stay at target weight after blend in
                                                if (TimeElapsed >= BlendInDuration) {
                                                    return PhysicsBlendWeight_Target; // Stay at target weight forever
                                                } else {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    float Progress =
                                                        FMath::Clamp(TimeElapsed / BlendInDuration, 0.0f, 1.0f);
                                                    float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                                    return FMath::Lerp(PhysicsBlendWeight_Start,
                                                                       PhysicsBlendWeight_Target, SmoothProgress);
                                                }
<<<<<<< HEAD
                                            } else // Temporary blend
                                            {
                                                if (TimeElapsed < BlendInDuration) {
                                                    float Progress = TimeElapsed / BlendInDuration;
                                                    float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                                    return FMath::Lerp(PhysicsBlendWeight_Start,
                                                                       PhysicsBlendWeight_Target, SmoothProgress);
                                                } else if (TimeElapsed < BlendInDuration + HoldDuration) {
                                                    return PhysicsBlendWeight_Target;
                                                } else {
                                                    == == == =
                                                }
                                                else // Temporary blend
                                                {
                                                    if (TimeElapsed < BlendInDuration) {
                                                        float Progress = TimeElapsed / BlendInDuration;
                                                        float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                                        return FMath::Lerp(PhysicsBlendWeight_Start,
                                                                           PhysicsBlendWeight_Target, SmoothProgress);
                                                    } else if (TimeElapsed < BlendInDuration + HoldDuration) {
                                                        return PhysicsBlendWeight_Target;
                                                    } else {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                        float BlendOutElapsed =
                                                            TimeElapsed - (BlendInDuration + HoldDuration);
                                                        float Progress = FMath::Clamp(
                                                            BlendOutElapsed / BlendOutDuration, 0.0f, 1.0f);
                                                        float SmoothProgress = FMath::SmoothStep(0.0f, 1.0f, Progress);
                                                        return FMath::Lerp(PhysicsBlendWeight_Target, 0.0f,
                                                                           SmoothProgress);
                                                    }
                                                }
                                            }
<<<<<<< HEAD

                                            // Check if complete at time
                                            bool IsCompleteAtTime(float WorldTime) const {
                                                if (bForceComplete)
                                                    return true;

                                                // Permanent blends are never complete based on time
                                                if (bIsPermanent)
                                                    return false;

                                                float TimeElapsed = WorldTime - StartTime;
                                                return TimeElapsed >= TotalDuration;
                                            }

                                            // Add to FOHActiveBlend struct
                                            FORCEINLINE bool IsPaused() const {
                                                return PauseCount > 0;
                                            }

                                            // Update the IsComplete() method in FOHActiveBlend:
                                            FORCEINLINE bool IsComplete() const {
                                                // Permanent blends are never complete unless forced
                                                if (bIsPermanent && !bForceComplete) {
                                                    return false;
                                                }

                                                return bForceComplete || CurrentPhase == EOHBlendPhase::Inactive ||
                                                       (PhysicsBlendWeight_Current < CompletionThreshold &&
                                                        CurrentPhase == EOHBlendPhase::BlendOut);
                                            }

                                            FORCEINLINE bool IsBlendingIn() const {
                                                return CurrentPhase == EOHBlendPhase::BlendIn &&
                                                       PhysicsBlendWeight_Current < PhysicsBlendWeight_Target;
                                            }

                                            FORCEINLINE bool IsHolding() const {
                                                return CurrentPhase == EOHBlendPhase::Hold;
                                            }

                                            FORCEINLINE bool IsBlendingOut() const {
                                                return CurrentPhase == EOHBlendPhase::BlendOut &&
                                                       PhysicsBlendWeight_Current > 0.0f;
                                            }

                                            FORCEINLINE float GetBlendProgress() const {
                                                if (PhysicsBlendWeight_Target <= 0.0f)
                                                    return 0.0f;
                                                return PhysicsBlendWeight_Current / PhysicsBlendWeight_Target;
                                            }

                                            FORCEINLINE float GetPhaseProgress(float WorldTime) const {
                                                float TimeElapsed = WorldTime - StartTime;

                                                if (bIsBlendOutOnly) {
                                                    return FMath::Clamp(TimeElapsed / BlendOutDuration, 0.0f, 1.0f);
                                                } else if (bIsPermanent) {
                                                    return FMath::Clamp(TimeElapsed / BlendInDuration, 0.0f, 1.0f);
                                                } else // Temporary blend
                                                    == == == =

                                                                 // Check if complete at time
                                                        bool IsCompleteAtTime(float WorldTime) const {
                                                        if (bForceComplete)
                                                            return true;

                                                        // Permanent blends are never complete based on time
                                                        if (bIsPermanent)
                                                            return false;

                                                        float TimeElapsed = WorldTime - StartTime;
                                                        return TimeElapsed >= TotalDuration;
                                                    }

                                                // Add to FOHActiveBlend struct
                                                FORCEINLINE bool IsPaused() const {
                                                    return PauseCount > 0;
                                                }

                                                // Update the IsComplete() method in FOHActiveBlend:
                                                FORCEINLINE bool IsComplete() const {
                                                    // Permanent blends are never complete unless forced
                                                    if (bIsPermanent && !bForceComplete) {
                                                        return false;
                                                    }

                                                    return bForceComplete || CurrentPhase == EOHBlendPhase::Inactive ||
                                                           (PhysicsBlendWeight_Current < CompletionThreshold &&
                                                            CurrentPhase == EOHBlendPhase::BlendOut);
                                                }

                                                FORCEINLINE bool IsBlendingIn() const {
                                                    return CurrentPhase == EOHBlendPhase::BlendIn &&
                                                           PhysicsBlendWeight_Current < PhysicsBlendWeight_Target;
                                                }

                                                FORCEINLINE bool IsHolding() const {
                                                    return CurrentPhase == EOHBlendPhase::Hold;
                                                }

                                                FORCEINLINE bool IsBlendingOut() const {
                                                    return CurrentPhase == EOHBlendPhase::BlendOut &&
                                                           PhysicsBlendWeight_Current > 0.0f;
                                                }

                                                FORCEINLINE float GetBlendProgress() const {
                                                    if (PhysicsBlendWeight_Target <= 0.0f)
                                                        return 0.0f;
                                                    return PhysicsBlendWeight_Current / PhysicsBlendWeight_Target;
                                                }

                                                FORCEINLINE float GetPhaseProgress(float WorldTime) const {
                                                    float TimeElapsed = WorldTime - StartTime;

                                                    if (bIsBlendOutOnly) {
                                                        return FMath::Clamp(TimeElapsed / BlendOutDuration, 0.0f, 1.0f);
                                                    } else if (bIsPermanent) {
                                                        return FMath::Clamp(TimeElapsed / BlendInDuration, 0.0f, 1.0f);
                                                    } else // Temporary blend
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    {
                                                        float Total = BlendInDuration + HoldDuration + BlendOutDuration;
                                                        return FMath::Clamp(TimeElapsed / Total, 0.0f, 1.0f);
                                                    }
                                                }
<<<<<<< HEAD
                                            };

                                            USTRUCT(BlueprintType)
                                            struct FOHBoneCategoryDefinition {
                                                == == == =
                                            };

                                            USTRUCT(BlueprintType)
                                            struct FOHBoneCategoryDefinition {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                GENERATED_BODY()

                                                UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
                                                TArray<FName> Bones;

                                                FOHBoneCategoryDefinition() {}
<<<<<<< HEAD

                                                FOHBoneCategoryDefinition(std::initializer_list<FName> InBones){
                                                    == == ==
                                                    =

                                                        FOHBoneCategoryDefinition(std::initializer_list<FName> InBones){
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                            Bones.Append(InBones);
                                            }
                                        };
#pragma endregion

                                        < < < < < < < HEAD == == == =

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                                        // === EVENTS ===
                                            DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnPACManagerInitialized);
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnBoneStartedSimulating, FName,
                                                                                    BoneName);
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnBoneStoppedSimulating, FName,
                                                                                    BoneName);
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnBlendCompleted, FName, BoneName,
                                                                                     FName, ReactionTag);
<<<<<<< HEAD
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPhysicsImpactProcessed, FName,
                                                                                     BoneName, const FVector2D&,
                                                                                     MovementVector);
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(
                                            FOnPushbackApplied, FName, BoneName, const FVector2D&, PushbackVector,
                                            float, ImpactForce);
                                        DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnCombatImpulseGenerated,
                                                                                       FVector2D, ImpulseVector, float,
                                                                                       Magnitude, FName, SourceBone);
                                        // DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPhysicsBodyHit, FName,
                                        // BoneName, const FVector&, ImpactVelocity, const FHitResult&, HitResult);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_SixParams(FOnPhysicsBodyHit, USkeletalMeshComponent*,
                                             SelfMesh,        // The mesh that owns this PAC
                                             FName, SelfBone, // The bone on this mesh that was hit
                                             USkeletalMeshComponent*,
                                             OtherMesh,               // The other mesh involved in the collision
                                             FName, OtherBone,        // The bone on the other mesh involved
                                             FVector, ImpactVelocity, // The collision impulse or relative velocity
                                             FHitResult, HitResult    // The full hit result
=======
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPhysicsImpactProcessed, FName, BoneName, const FVector2D&, MovementVector);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPushbackApplied,FName, BoneName, const FVector2D&, PushbackVector, float, ImpactForce);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(
    FOnCombatImpulseGenerated, 
    FVector2D, ImpulseVector, 
    float, Magnitude,
    FName, SourceBone
);
//DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPhysicsBodyHit, FName, BoneName, const FVector&, ImpactVelocity, const FHitResult&, HitResult);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_SixParams(
    FOnPhysicsBodyHit,
    USkeletalMeshComponent*, SelfMesh,      // The mesh that owns this PAC
    FName, SelfBone,                        // The bone on this mesh that was hit
    USkeletalMeshComponent*, OtherMesh,     // The other mesh involved in the collision
    FName, OtherBone,                       // The bone on the other mesh involved
    FVector, ImpactVelocity,                // The collision impulse or relative velocity
    FHitResult, HitResult                   // The full hit result
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
);
// ============================================================================
// MAIN COMPONENT CLASS
// ============================================================================
#pragma region MAIN COMPONENT CLASS

<<<<<<< HEAD
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHPACManager : public UActorComponent {
                                            == == == = UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom),
                                                              meta = (BlueprintSpawnableComponent)) class ONLYHANDS_API
                                                UOHPACManager : public UActorComponent {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                GENERATED_BODY()

                                              public:
                                                UOHPACManager();

                                                // === LIFECYCLE ===
                                                virtual void BeginPlay() override;
                                                virtual void
                                                TickComponent(float DeltaTime, ELevelTick TickType,
                                                              FActorComponentTickFunction* ThisTickFunction) override;
                                                virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

                                                // === CONFIGURATION ===
                                                UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC")
                                                bool bEnablePACManager = true;

                                                UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC")
                                                float InitializationDelay = 0.1f;

<<<<<<< HEAD
                                                UPROPERTY(EditDefaultsOnly, Category = "PAC|Init")
                                                float InitRetryIntervalSeconds = 0.1f;

                                                UPROPERTY(EditDefaultsOnly, Category = "PAC|Init")
                                                == == ==
                                                    = UPROPERTY(EditDefaultsOnly,
                                                                Category = "PAC|Init") float InitRetryIntervalSeconds =
                                                        0.1f;

                                                UPROPERTY(EditDefaultsOnly, Category = "PAC|Init")
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                float InitRetryTotalDurationSeconds = 5.0f;

                                                // === AUTO SETUP ===
                                                UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup")
                                                bool bAutoSetupPhysics = false;

<<<<<<< HEAD
                                                UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup",
                                                          meta = (EditCondition = "bAutoSetupPhysics"))
                                                float AutoSetupDelaySeconds = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup",
=======
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup", 
              meta = (EditCondition = "bAutoSetupPhysics"))
    float AutoSetupDelaySeconds = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup", 
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
              meta = (EditCondition = "bAutoSetupPhysics"))
    int32 AutoSetupMaxRetries = 10;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup")
    UPhysicsAsset* DefaultPhysicsAsset = nullptr;
<<<<<<< HEAD

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup")
    TArray<FName> SimulatableBonesList;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Categories",
              meta = (DisplayName = "Bone Category Definitions"))
    TMap<EOHBoneCategory, FOHBoneCategoryDefinition> BoneCategoryDefinitions;

    // Bones to track for motion data (physics velocity, acceleration, etc.)
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Tracking",
              meta = (DisplayName = "Tracked Bones"))
    TArray<FName> TrackedBones = {
        // Spine (3 bones only)
        TEXT("pelvis"), TEXT("spine_01"), TEXT("spine_02"), TEXT("spine_03"),

        // Head
        TEXT("neck_01"), TEXT("head"),

        // Left Arm
        TEXT("clavicle_l"), TEXT("upperarm_l"), TEXT("lowerarm_l"), TEXT("hand_l"),

        // Right Arm
        TEXT("clavicle_r"), TEXT("upperarm_r"), TEXT("lowerarm_r"), TEXT("hand_r"),

        // Left Leg
        TEXT("thigh_l"), TEXT("calf_l"), TEXT("foot_l"),
        TEXT("ball_l"), // Tracked but excluded from simulation

        // Right Leg
        TEXT("thigh_r"), TEXT("calf_r"), TEXT("foot_r"),
        TEXT("ball_r") // Tracked but excluded from simulation
    };

    // Bones to exclude from simulation (even if they're in TrackedBones)
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Tracking",
              meta = (DisplayName = "Excluded Bones"))
    TArray<FName> ExcludedBones = {TEXT("root"),
                                   TEXT("ball_l"), // Added - track but don't simulate
                                   TEXT("ball_r"), // Added - track but don't simulate
                                   TEXT("ik_foot_root"), TEXT("ik_hand_root"), TEXT("ik_hand_gun"), TEXT("ik_hand_l"),
                                   TEXT("ik_hand_r"),    TEXT("ik_foot_l"),    TEXT("ik_foot_r")};

  protected:
    void InitializeBoneCategoryDefinitions();
    void BuildCategoryBonesMap();
    void ForceDisablePhysics(FName BoneName);
    void UpdateSkeletalMeshBounds();

  private:
    // Runtime maps (not exposed to Blueprint)
    TMap<EOHBoneCategory, TArray<FName>> CategoryBonesMap;
    TMap<FName, EOHBoneCategory> BoneCategoryMap;

  public:
=======
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Auto Setup")
    TArray<FName> SimulatableBonesList;
    
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Categories",
                meta = (DisplayName = "Bone Category Definitions"))
    TMap<EOHBoneCategory, FOHBoneCategoryDefinition> BoneCategoryDefinitions;

    // Bones to track for motion data (physics velocity, acceleration, etc.)
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Tracking", 
              meta = (DisplayName = "Tracked Bones"))
    TArray<FName> TrackedBones = {
        // Spine (3 bones only)
        TEXT("pelvis"),
        TEXT("spine_01"), 
        TEXT("spine_02"), 
        TEXT("spine_03"),
        
        // Head
        TEXT("neck_01"),
        TEXT("head"),
        
        // Left Arm
        TEXT("clavicle_l"),
        TEXT("upperarm_l"),
        TEXT("lowerarm_l"),
        TEXT("hand_l"),
        
        // Right Arm  
        TEXT("clavicle_r"),
        TEXT("upperarm_r"),
        TEXT("lowerarm_r"),
        TEXT("hand_r"),
        
        // Left Leg
        TEXT("thigh_l"),
        TEXT("calf_l"),
        TEXT("foot_l"),
        TEXT("ball_l"),  // Tracked but excluded from simulation
        
        // Right Leg
        TEXT("thigh_r"),
        TEXT("calf_r"),
        TEXT("foot_r"),
        TEXT("ball_r")   // Tracked but excluded from simulation
    };

    // Bones to exclude from simulation (even if they're in TrackedBones)
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PAC|Bone Tracking",
              meta = (DisplayName = "Excluded Bones"))
    TArray<FName> ExcludedBones = {
        TEXT("root"),
        TEXT("ball_l"),      // Added - track but don't simulate
        TEXT("ball_r"),      // Added - track but don't simulate
        TEXT("ik_foot_root"),
        TEXT("ik_hand_root"),
        TEXT("ik_hand_gun"),
        TEXT("ik_hand_l"),
        TEXT("ik_hand_r"),
        TEXT("ik_foot_l"),
        TEXT("ik_foot_r")
    };
    
 

protected:
    void InitializeBoneCategoryDefinitions();
    void BuildCategoryBonesMap();
    void ForceDisablePhysics(FName BoneName);
    void UpdateSkeletalMeshBounds();

private:
    // Runtime maps (not exposed to Blueprint)
    TMap<EOHBoneCategory, TArray<FName>> CategoryBonesMap;
    TMap<FName, EOHBoneCategory> BoneCategoryMap;

public:

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    // === PHYSICS PROFILES ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Profiles")
    TMap<EOHPhysicsStrength, FPhysicalAnimationData> StrengthProfiles;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Profiles")
    TMap<FName, FPhysicalAnimationData> NamedProfiles;

    // === DEBUG ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug")
    bool bDrawDebug = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug")
    bool bVerboseLogging = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Cleanup")
    float CleanupInterval = 1.0f;

    // === INITIALIZATION ===
    UFUNCTION(BlueprintCallable, Category = "PAC|Initialization")
    void InitializePACManager();

    UFUNCTION(BlueprintCallable, Category = "PAC|Initialization")
    void ResetPACManager();
    void InitializeBoneLists();

    UFUNCTION(BlueprintPure, Category = "PAC|State")
<<<<<<< HEAD
    bool IsInitialized() const {
                                                    return bIsInitialized;
    }

    // === UNIFIED PHYSICAL ANIMATION CONTROL ===

    bool SetBonePhysicalAnimation_Internal(FName BoneName, bool bEnable,
                                           const FPhysicalAnimationData& Profile = FPhysicalAnimationData(),
                                           bool bIncludeChain = false,
                                           const TArray<FName>& FilterBones = TArray<FName>(),
                                           bool bEnableCollision = true, bool bWakeBody = true,
                                           bool bUseNativePropagation = true);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation",
              meta = (DisplayName = "Set Bone Physical Animation"))
    bool SetBonePhysicalAnimation(FName BoneName, bool bEnable,
                                  const FPhysicalAnimationData& Profile = FPhysicalAnimationData(),
                                  bool bIncludeChain = false, bool bEnableCollision = true, bool bWakeBody = true);

    // Version with filter for Blueprint
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation",
              meta = (DisplayName = "Set Bone Physical Animation With Filter"))
    bool SetBonePhysicalAnimationWithFilter(FName BoneName, bool bEnable, const FPhysicalAnimationData& Profile,
                                            bool bIncludeChain, const TArray<FName>& FilterBones,
                                            bool bEnableCollision = true, bool bWakeBody = true);

=======
    bool IsInitialized() const {
                                                    return bIsInitialized; }


    
    // === UNIFIED PHYSICAL ANIMATION CONTROL ===
    
    bool SetBonePhysicalAnimation_Internal(
        FName BoneName,
        bool bEnable,
        const FPhysicalAnimationData& Profile = FPhysicalAnimationData(),
        bool bIncludeChain = false,
        const TArray<FName>& FilterBones = TArray<FName>(),
        bool bEnableCollision = true,
        bool bWakeBody = true,
        bool bUseNativePropagation = true
    );

    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation", 
            meta = (DisplayName = "Set Bone Physical Animation"))
    bool SetBonePhysicalAnimation(
        FName BoneName,
        bool bEnable,
        const FPhysicalAnimationData& Profile = FPhysicalAnimationData(),
        bool bIncludeChain = false,
        bool bEnableCollision = true,
        bool bWakeBody = true
    );

    // Version with filter for Blueprint
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation", 
              meta = (DisplayName = "Set Bone Physical Animation With Filter"))
    bool SetBonePhysicalAnimationWithFilter(
        FName BoneName,
        bool bEnable,
        const FPhysicalAnimationData& Profile,
        bool bIncludeChain,
        const TArray<FName>& FilterBones,
        bool bEnableCollision = true,
        bool bWakeBody = true
    );

    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation")
    void ApplyPhysicalAnimationProfile(FName BoneName, const FPhysicalAnimationData& Profile, bool bAffectChain);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation")
    void ClearPhysicalAnimationProfile(FName BoneName);

    // === UNIFIED PHYSICS BLEND CONTROL ===
<<<<<<< HEAD
    UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend", meta = (DisplayName = "Set Physics Blend"))
    void SetPhysicsBlend(FName BoneName, bool bEnable, const FOHPhysicsBlendParams& Params = FOHPhysicsBlendParams());

    /* UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void StopBlend(FName BoneName, FName Tag);

     UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void PauseBlend(FName BoneName, FName ReactionTag = NAME_None);

     UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void ResumeBlend(FName BoneName, FName ReactionTag = NAME_None);
 */

    UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
    void StopAllBlends();

=======
    UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend", 
              meta = (DisplayName = "Set Physics Blend"))
    void SetPhysicsBlend(
        FName BoneName,
        bool bEnable,
        const FOHPhysicsBlendParams& Params = FOHPhysicsBlendParams()
    );
    
    /* UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void StopBlend(FName BoneName, FName Tag);
     
     UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void PauseBlend(FName BoneName, FName ReactionTag = NAME_None);
 
     UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
     void ResumeBlend(FName BoneName, FName ReactionTag = NAME_None);
 */
    
    UFUNCTION(BlueprintCallable, Category = "PAC|Physics Blend")
    void StopAllBlends();


>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    // === SIMULATION CONTROL ===
    UFUNCTION(BlueprintCallable, Category = "PAC|Simulation")
    bool SetSimulation(FName BoneName, bool bEnable, bool bAllBelow = false);

    UFUNCTION(BlueprintCallable, Category = "PAC|Simulation")
    void EnsureBoneSimulatingPhysics(FName BoneName, bool bEnableChain = true);

<<<<<<< HEAD
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation",
=======
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation", 
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
              meta = (DisplayName = "Enable Bone Physics Simulation"))
    bool EnablePhysicsForBone(FName BoneName, const FPhysicalAnimationData& Profile, bool bEnableCollision,
                              bool bWakeBody, bool bEnableDrivePropagation);

<<<<<<< HEAD
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation",
              meta = (DisplayName = "Disable Bone Physics Simulation"))
=======
    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation", 
                meta = (DisplayName = "Disable Bone Physics Simulation"))
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    bool DisablePhysicsForBone(FName Name);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physical Animation")
    void DisableAllPhysicsBodies();

    // === BONE STATE QUERIES ===
    UFUNCTION(BlueprintPure, Category = "PAC|State")
    bool IsBoneSimulating(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|State")
    bool IsBoneDrivenByPhysicalAnimation(const FName& BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|State")
    float GetBonePhysicsBlendWeight(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|State")
    int32 GetActiveBlendCount(FName BoneName) const;
    bool GetBoneBaseline(FName BoneName, float& OutWeight, FPhysicalAnimationData& OutProfile) const;
    void SetBoneBaseline(FName BoneName, float Weight, const FPhysicalAnimationData& Profile);
    void ClearBoneBaseline(FName BoneName);

    UFUNCTION(BlueprintPure, Category = "PAC|State")
    int32 GetTotalActiveBlendCount() const;

    // === PHYSICS INTERACTION ===
    UFUNCTION(BlueprintCallable, Category = "PAC|Physics")
    void ApplyImpulse(FName BoneName, const FVector& Impulse, bool bVelChange = false);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physics")
    void ApplyImpulseToChain(FName RootBone, const FVector& Impulse, float Falloff = 0.5f, bool bVelChange = false);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physics")
    void WakePhysicsBody(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physics")
    void PutPhysicsBodyToSleep(FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "PAC|Physics")
    void ClearPhysicsStateForBone(FName BoneName);
<<<<<<< HEAD
=======
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

    // === CONSTRAINT DATA ===
    UFUNCTION(BlueprintPure, Category = "PAC|Constraints")
    float GetConstraintStrain(FName BoneName) const;

    const FOHConstraintData* GetConstraintData(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Constraints")
    FOHConstraintDriveData GetConstraintDriveData(FName BoneName) const;

    // === UTILITY ===
    UFUNCTION(BlueprintPure, Category = "PAC|Utility")
    TArray<FName> GetBonesInCategory(EOHBoneCategory Category) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Utility")
    TArray<FName> GetBoneChain(FName RootBone, int32 MaxDepth = -1) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Utility")
<<<<<<< HEAD
    TArray<FName> GetSimulatableBones() const {
                                                    return SimulatableBones.Array();
    }

    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
    FPhysicalAnimationData GetProfileForStrength(EOHPhysicsStrength Strength) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
    FPhysicalAnimationData GetNamedProfile(FName ProfileName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
    FPhysicalAnimationData GetCurrentPhysicalAnimationProfile(FName BoneName) const;

    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
    float GetPhysicalAnimationStrengthMultiplier() const;

    UFUNCTION(BlueprintCallable, Category = "PAC|Profiles")
    void SetPhysicalAnimationStrengthMultiplier(float Multiplier);
    UFUNCTION(BlueprintCallable, Category = "PAC|Profiles")
    void IncrementPhysicalAnimationStrengthMultiplier(float Increment = 0.1f);

    // === VALIDATION ===
    UFUNCTION(BlueprintCallable, Category = "PAC|Validation")
    bool ValidateSetup(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                       TArray<FName>& OutMissingConstraints, TArray<FName>& OutRuntimeConstraintsNotInAsset,
                       TArray<FName>& OutMismatchedConstraints) const;

    // === DEBUG ===
    mutable float LastVerboseLogTime = 0.0f;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC|Debug")
    void LogSystemState() const;

    void LogBlendState(FName BoneName) const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC|Debug")
    void LogActiveBlends() const;

    UFUNCTION(BlueprintCallable, Category = "PAC|Debug")
    void DrawDebugOverlay() const;

    UFUNCTION(BlueprintCallable, Category = "PAC|Auto Setup")
    void InitializePhysicsBlending();

  protected:
    // === ENHANCED RELIABILITY SETTINGS ===

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Reliability",
              meta = (DisplayName = "Force Update Tick"))
    bool bForceUpdateEveryTick = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Reliability",
              meta = (DisplayName = "Verify Physics State",
                      ToolTip = "Continuously verify and fix physics state discrepancies"))
    bool bVerifyPhysicsState = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Reliability",
              meta = (DisplayName = "Physics Verification Interval"))
    float PhysicsVerificationInterval = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Reliability",
              meta = (DisplayName = "Auto Fix Stale Bodies"))
    bool bAutoFixStaleBodies = true;

    // === ENHANCED DEBUG SETTINGS ===

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug", meta = (DisplayName = "Show Blend Weight Text"))
    bool bShowBlendWeightText = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug",
              meta = (DisplayName = "Show PAC Profile Values"))
    bool bShowPACProfileValues = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug", meta = (DisplayName = "Log State Changes"))
    bool bLogStateChanges = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug")
    bool bDrawCollisionData = false;

    // === BLUEPRINT-FRIENDLY FUNCTIONS ===

    // Simplified enable/disable for Blueprint use
    UFUNCTION(BlueprintCallable, Category = "PAC|Quick Actions", meta = (DisplayName = "Enable Bone Physics Simple"))
    void EnableBonePhysicsSimple(FName BoneName, EOHPhysicsStrength Strength = EOHPhysicsStrength::Medium,
                                 bool bIncludeChain = false);

    UFUNCTION(BlueprintCallable, Category = "PAC|Quick Actions", meta = (DisplayName = "Disable Bone Physics Simple"))
    void DisableBonePhysicsSimple(FName BoneName, bool bIncludeChain = false);

    UFUNCTION()
    float CalculateCombinedBlendWeight(const TArray<FOHActiveBlend>& Blends,
                                       FPhysicalAnimationData& OutCombinedProfile);

    // Set simulation for entire category
    UFUNCTION(BlueprintCallable, Category = "PAC|Simulation", meta = (DisplayName = "Set Category Simulation"))
    void SetCategorySimulation(EOHBoneCategory Category, bool bEnable,
                               const FPhysicalAnimationData& Profile = FPhysicalAnimationData(),
                               bool bEnableCollision = true);

    // Diagnostic functions
    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics", meta = (DisplayName = "Verify Component Setup"))
    bool VerifyComponentSetup(FString& OutReport);

    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics", meta = (DisplayName = "Force Refresh Physics"))
    void ForceRefreshPhysics();

    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics", meta = (DisplayName = "Fix Stale Bodies"))
    int32 FixStaleBodies();

    // === ENHANCED RELIABILITY FUNCTIONS ===
    void VerifyPhysicsStateInternal();
    void EnsurePhysicsTickEnabled();
    void SetupPhysicsBounds();
    void OnPhysicsStateChanged(bool bPhysicsActive);
    void RefreshBodyInstances();
    bool VerifyBonePhysicsSetup(FName BoneName);
    void FixBodyPhysicsState(FName BoneName, FBodyInstance* Body);

    // === AUTO SETUP ===
    void PerformAutoSetup();
    void RetryAutoSetup();
    void SetupPhysicsAsset();
    void SetupPhysicalAnimationComponent();
    void ConfigureCollisionSettings();
    void RestoreOriginalCollisionSettings();

    // === INITIALIZATION HELPERS ===
    void StartInitializationRetry();
    void RetryInitializePACManager();
    void InitializeMotionTracking();
    void BuildConstraintData();
    void CacheSimulatableBones();
    void InitializeStrengthProfiles();

    // === UPDATE FUNCTIONS ===
    void UpdateMotionTracking(float DeltaTime);
    void UpdateConstraintStates(float DeltaTime);
    void ProcessActiveBlends(float DeltaTime);
    void UpdateBlendState(FOHActiveBlend& Blend, float DeltaTime);
    void ApplyBlendToBody(FName BoneName, const TArray<FOHActiveBlend>& Blends, float CombinedWeight,
                          const FPhysicalAnimationData& CombinedProfile);
    void CleanupStaleBlends();

    // === BONE HELPERS ===
    bool IsBoneValidForSimulation(FName BoneName) const;
    // void ApplyBoneAlphaScaling(FName BoneName, float Alpha);
    void ApplyChainStabilization(const TArray<FName>& BoneChain);

    // === DIRECT ACCESS ===
    FBodyInstance* GetBodyInstanceDirect(FName BoneName) const;
    FConstraintInstance* GetConstraintInstanceDirect(FName BoneName) const;
    int32 GetBoneIndexDirect(FName BoneName) const;

    // === VALIDATION HELPERS ===
    bool ValidatePhysicsSimulation() const;
    bool ValidateBodyInstances(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies) const;
    bool ValidateConstraintInstances(TArray<FName>& OutMissingConstraints,
                                     TArray<FName>& OutRuntimeConstraintsNotInAsset,
                                     TArray<FName>& OutMismatchedConstraints) const;
    bool ValidatePhysicsAsset(TArray<FName>& OutMissingBones, TArray<FName>& OutInstancesWithoutBodies,
                              TArray<FName>& OutMissingConstraints, TArray<FName>& OutRuntimeConstraintsNotInAsset,
                              TArray<FName>& OutMismatchedConstraints) const;
    void CheckAllPhysicsBodiesValid() const;

    // === CONSTRAINT HELPERS ===
    FConstraintInstance* FindPhysicalAnimationConstraint(FName BoneName) const;
    bool GetConstraintDriveParameters(FName BoneName, float& OutLinearStiffness, float& OutLinearDamping,
                                      float& OutAngularStiffness, float& OutAngularDamping) const;

    static bool GetConstraintDriveParametersFromInstance(FConstraintInstance* Constraint, float& OutLinearStiffness,
                                                         float& OutLinearDamping, float& OutAngularStiffness,
                                                         float& OutAngularDamping);

    static FOHConstraintDriveData ExtractConstraintDriveData(const FConstraintInstance* Constraint);

    // === DEBUG HELPERS ===
    void DebugBodyPhysicsStates();
    void SafeLog(const FString& Message, bool bWarning = false) const;

    // === STATIC HELPERS ===
    static bool HasPhysicalAnimationDrives(const FConstraintInstance* Constraint);
    static TArray<FBodyInstance*> GetSimulatableBodies(USkeletalMeshComponent* SkeletalMesh,
                                                       const TSet<FName>& SimulatableBones);

  private:
    // Baseline tracking for return-to-default functionality
    struct FBoneBaseline {
                                                    float DefaultWeight = 0.0f;
                                                    FPhysicalAnimationData DefaultProfile;
                                                    bool bHasBaseline = false;
                                                    == == == = TArray<FName> GetSimulatableBones() const {
                                                        return SimulatableBones.Array();
                                                    }

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
                                                    FPhysicalAnimationData GetProfileForStrength(
                                                        EOHPhysicsStrength Strength) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
                                                    FPhysicalAnimationData GetNamedProfile(FName ProfileName) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
                                                    FPhysicalAnimationData GetCurrentPhysicalAnimationProfile(
                                                        FName BoneName) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Profiles")
                                                    float GetPhysicalAnimationStrengthMultiplier() const;

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Profiles")
                                                    void SetPhysicalAnimationStrengthMultiplier(float Multiplier);
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Profiles")
                                                    void IncrementPhysicalAnimationStrengthMultiplier(float Increment =
                                                                                                          0.1f);

                                                    // === VALIDATION ===
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Validation")
                                                    bool ValidateSetup(TArray<FName> & OutMissingBones,
                                                                       TArray<FName> & OutInstancesWithoutBodies,
                                                                       TArray<FName> & OutMissingConstraints,
                                                                       TArray<FName> & OutRuntimeConstraintsNotInAsset,
                                                                       TArray<FName> & OutMismatchedConstraints) const;

                                                    // === DEBUG ===
                                                    mutable float LastVerboseLogTime = 0.0f;

                                                    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC|Debug")
                                                    void LogSystemState() const;

                                                    void LogBlendState(FName BoneName) const;

                                                    UFUNCTION(BlueprintCallable, CallInEditor, Category = "PAC|Debug")
                                                    void LogActiveBlends() const;

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Debug")
                                                    void DrawDebugOverlay() const;

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Auto Setup")
                                                    void InitializePhysicsBlending();

                                                  protected:
                                                    // === ENHANCED RELIABILITY SETTINGS ===

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Reliability",
                                                              meta = (DisplayName = "Force Update Tick"))
                                                    bool bForceUpdateEveryTick = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Reliability",
                                                              meta = (DisplayName = "Verify Physics State",
                                                                      ToolTip = "Continuously verify and fix physics "
                                                                                "state discrepancies"))
                                                    bool bVerifyPhysicsState = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Reliability",
                                                              meta = (DisplayName = "Physics Verification Interval"))
                                                    float PhysicsVerificationInterval = 0.1f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Reliability",
                                                              meta = (DisplayName = "Auto Fix Stale Bodies"))
                                                    bool bAutoFixStaleBodies = true;

                                                    // === ENHANCED DEBUG SETTINGS ===

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug",
                                                              meta = (DisplayName = "Show Blend Weight Text"))
                                                    bool bShowBlendWeightText = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug",
                                                              meta = (DisplayName = "Show PAC Profile Values"))
                                                    bool bShowPACProfileValues = false;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug",
                                                              meta = (DisplayName = "Log State Changes"))
                                                    bool bLogStateChanges = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Debug")
                                                    bool bDrawCollisionData = false;

                                                    // === BLUEPRINT-FRIENDLY FUNCTIONS ===

                                                    // Simplified enable/disable for Blueprint use
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Quick Actions",
                                                              meta = (DisplayName = "Enable Bone Physics Simple"))
                                                    void EnableBonePhysicsSimple(FName BoneName,
                                                                                 EOHPhysicsStrength Strength =
                                                                                     EOHPhysicsStrength::Medium,
                                                                                 bool bIncludeChain = false);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Quick Actions",
                                                              meta = (DisplayName = "Disable Bone Physics Simple"))
                                                    void DisableBonePhysicsSimple(FName BoneName,
                                                                                  bool bIncludeChain = false);

                                                    UFUNCTION()
                                                    float CalculateCombinedBlendWeight(
                                                        const TArray<FOHActiveBlend>& Blends,
                                                        FPhysicalAnimationData& OutCombinedProfile);

                                                    // Set simulation for entire category
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Simulation",
                                                              meta = (DisplayName = "Set Category Simulation"))
                                                    void SetCategorySimulation(EOHBoneCategory Category, bool bEnable,
                                                                               const FPhysicalAnimationData& Profile =
                                                                                   FPhysicalAnimationData(),
                                                                               bool bEnableCollision = true);

                                                    // Diagnostic functions
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics",
                                                              meta = (DisplayName = "Verify Component Setup"))
                                                    bool VerifyComponentSetup(FString & OutReport);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics",
                                                              meta = (DisplayName = "Force Refresh Physics"))
                                                    void ForceRefreshPhysics();

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Diagnostics",
                                                              meta = (DisplayName = "Fix Stale Bodies"))
                                                    int32 FixStaleBodies();

                                                    // === ENHANCED RELIABILITY FUNCTIONS ===
                                                    void VerifyPhysicsStateInternal();
                                                    void EnsurePhysicsTickEnabled();
                                                    void SetupPhysicsBounds();
                                                    void OnPhysicsStateChanged(bool bPhysicsActive);
                                                    void RefreshBodyInstances();
                                                    bool VerifyBonePhysicsSetup(FName BoneName);
                                                    void FixBodyPhysicsState(FName BoneName, FBodyInstance * Body);

                                                    // === AUTO SETUP ===
                                                    void PerformAutoSetup();
                                                    void RetryAutoSetup();
                                                    void SetupPhysicsAsset();
                                                    void SetupPhysicalAnimationComponent();
                                                    void ConfigureCollisionSettings();
                                                    void RestoreOriginalCollisionSettings();

                                                    // === INITIALIZATION HELPERS ===
                                                    void StartInitializationRetry();
                                                    void RetryInitializePACManager();
                                                    void InitializeMotionTracking();
                                                    void BuildConstraintData();
                                                    void CacheSimulatableBones();
                                                    void InitializeStrengthProfiles();

                                                    // === UPDATE FUNCTIONS ===
                                                    void UpdateMotionTracking(float DeltaTime);
                                                    void UpdateConstraintStates(float DeltaTime);
                                                    void ProcessActiveBlends(float DeltaTime);
                                                    void UpdateBlendState(FOHActiveBlend & Blend, float DeltaTime);
                                                    void ApplyBlendToBody(
                                                        FName BoneName, const TArray<FOHActiveBlend>& Blends,
                                                        float CombinedWeight,
                                                        const FPhysicalAnimationData& CombinedProfile);
                                                    void CleanupStaleBlends();

                                                    // === BONE HELPERS ===
                                                    bool IsBoneValidForSimulation(FName BoneName) const;
                                                    // void ApplyBoneAlphaScaling(FName BoneName, float Alpha);
                                                    void ApplyChainStabilization(const TArray<FName>& BoneChain);

                                                    // === DIRECT ACCESS ===
                                                    FBodyInstance* GetBodyInstanceDirect(FName BoneName) const;
                                                    FConstraintInstance* GetConstraintInstanceDirect(FName BoneName)
                                                        const;
                                                    int32 GetBoneIndexDirect(FName BoneName) const;

                                                    // === VALIDATION HELPERS ===
                                                    bool ValidatePhysicsSimulation() const;
                                                    bool ValidateBodyInstances(TArray<FName> & OutMissingBones,
                                                                               TArray<FName> &
                                                                                   OutInstancesWithoutBodies) const;
                                                    bool ValidateConstraintInstances(
                                                        TArray<FName> & OutMissingConstraints,
                                                        TArray<FName> & OutRuntimeConstraintsNotInAsset,
                                                        TArray<FName> & OutMismatchedConstraints) const;
                                                    bool ValidatePhysicsAsset(
                                                        TArray<FName> & OutMissingBones,
                                                        TArray<FName> & OutInstancesWithoutBodies,
                                                        TArray<FName> & OutMissingConstraints,
                                                        TArray<FName> & OutRuntimeConstraintsNotInAsset,
                                                        TArray<FName> & OutMismatchedConstraints) const;
                                                    void CheckAllPhysicsBodiesValid() const;

                                                    // === CONSTRAINT HELPERS ===
                                                    FConstraintInstance* FindPhysicalAnimationConstraint(FName BoneName)
                                                        const;
                                                    bool GetConstraintDriveParameters(
                                                        FName BoneName, float& OutLinearStiffness,
                                                        float& OutLinearDamping, float& OutAngularStiffness,
                                                        float& OutAngularDamping) const;

                                                    static bool GetConstraintDriveParametersFromInstance(
                                                        FConstraintInstance * Constraint, float& OutLinearStiffness,
                                                        float& OutLinearDamping, float& OutAngularStiffness,
                                                        float& OutAngularDamping);

                                                    static FOHConstraintDriveData ExtractConstraintDriveData(
                                                        const FConstraintInstance* Constraint);

                                                    // === DEBUG HELPERS ===
                                                    void DebugBodyPhysicsStates();
                                                    void SafeLog(const FString& Message, bool bWarning = false) const;

                                                    // === STATIC HELPERS ===
                                                    static bool HasPhysicalAnimationDrives(
                                                        const FConstraintInstance* Constraint);
                                                    static TArray<FBodyInstance*> GetSimulatableBodies(
                                                        USkeletalMeshComponent * SkeletalMesh,
                                                        const TSet<FName>& SimulatableBones);

                                                  private:
                                                    // Baseline tracking for return-to-default functionality
                                                    struct FBoneBaseline {
                                                        float DefaultWeight = 0.0f;
                                                        FPhysicalAnimationData DefaultProfile;
                                                        bool bHasBaseline = false;
        
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    };
                                                    TMap<FName, FBoneBaseline> BoneBaselines;

                                                    // === COMPONENT REFERENCES ===
                                                    UPROPERTY()
                                                    USkeletalMeshComponent* SkeletalMesh = nullptr;

                                                    UPROPERTY(Transient)
                                                    USkeletalMeshComponent* CollisionShadowMesh = nullptr;
<<<<<<< HEAD

                                                    == == ==
                                                        =
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                            UPROPERTY()
                                                                UPhysicalAnimationComponent* PhysicalAnimationComponent =
                                                                    nullptr;

                                                    UPROPERTY()
                                                    UPhysicsAsset* CachedPhysicsAsset = nullptr;

                                                    UPROPERTY(Transient)
                                                    USkeletalMesh* PreviousMeshAsset = nullptr;

                                                    UPROPERTY(Transient)
                                                    UPhysicsAsset* PreviousPhysicsAsset = nullptr;

<<<<<<< HEAD
                                                    == == == =
  

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                                 // === DIRECT ACCESS CACHES ===
                                                        mutable TMap<FName, int32> BoneIndexCache;

                                                    // === CONSTRAINT DATA ===
                                                    UPROPERTY()
                                                    TMap<FName, FOHConstraintData> ConstraintDataMap;

                                                    // === PERSISTENT SIMULATIONS ===
                                                    UPROPERTY()
                                                    TMap<FName, FName> PersistentSimulations;

                                                    // === BLEND STATES ===
                                                    TMap<FName, TArray<FOHActiveBlend>> ActiveBlends;

                                                    // === BONE SETS ===
                                                    UPROPERTY()
                                                    TSet<FName> SimulatableBones;

                                                    TMap<FName, ECollisionEnabled::Type> OriginalCollisionSettings;

                                                    // === STATE FLAGS ===
                                                    UPROPERTY()
                                                    bool bIsInitialized = false;

                                                    // Impact tracking for enhanced flinch system
                                                    UPROPERTY(Transient)
                                                    FVector LastImpactDirection = FVector::ZeroVector;
<<<<<<< HEAD

                                                    UPROPERTY(Transient)
                                                    float LastImpactMagnitude = 0.0f;

                                                    UPROPERTY(Transient)
                                                    float LastImpactTime = 0.0f;

                                                    == == == =

                                                                 UPROPERTY(Transient) float LastImpactMagnitude = 0.0f;

                                                    UPROPERTY(Transient)
                                                    float LastImpactTime = 0.0f;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    // Timer for flinch recovery
                                                    FTimerHandle FlinchRecoveryTimer;
                                                    // === TIMERS ===
                                                    FTimerHandle InitRetryHandle;
                                                    FTimerHandle AutoSetupRetryTimer;
                                                    FTimerHandle CleanupTimer;
                                                    // Timer handle for physics verification
                                                    FTimerHandle PhysicsVerificationTimer;

                                                    // === INTERNAL PROFILE ===
                                                    FPhysicalAnimationData ZeroProfile;

                                                    // === EVENTS ===
                                                    void OnSkeletalMeshChanged();
                                                    void OnSkeletalAssetChanged(USkeletalMesh * NewMesh,
                                                                                USkeletalMesh * OldMesh);
<<<<<<< HEAD

#pragma region Lookups

                                                    struct FOHConstraintLookup {
                                                        TArray<FOHConstraintData>
                                                            AsChild; // Constraints where this bone is child
                                                        TArray<FOHConstraintData>
                                                            AsParent; // Constraints where this bone is parent
                                                        FConstraintInstance* PACConstraint =
                                                            nullptr; // Cached PAC constraint if any
                                                    };

                                                    TMap<FName, FOHConstraintLookup> ConstraintLookupMap;
                                                    // Version tracking for cache invalidation
                                                    uint32 ConstraintDataVersion = 0;

                                                  public:
                                                    // New APIs that support multiple constraints
                                                    TArray<const FOHConstraintData*> GetConstraintsAsChild(
                                                        FName BoneName) const;
                                                    TArray<const FOHConstraintData*> GetConstraintsAsParent(
                                                        FName BoneName) const;
                                                    void ClearConstraintData();
                                                    == == == =
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

#pragma region Lookups

                                                                 struct FOHConstraintLookup {
                                                        TArray<FOHConstraintData>
                                                            AsChild; // Constraints where this bone is child
                                                        TArray<FOHConstraintData>
                                                            AsParent; // Constraints where this bone is parent
                                                        FConstraintInstance* PACConstraint =
                                                            nullptr; // Cached PAC constraint if any
                                                    };

                                                    TMap<FName, FOHConstraintLookup> ConstraintLookupMap;
                                                    // Version tracking for cache invalidation
                                                    uint32 ConstraintDataVersion = 0;

                                                  public:
                                                    // New APIs that support multiple constraints
                                                    TArray<const FOHConstraintData*> GetConstraintsAsChild(
                                                        FName BoneName) const;
                                                    TArray<const FOHConstraintData*> GetConstraintsAsParent(
                                                        FName BoneName) const;
                                                    void ClearConstraintData();

#pragma endregion

                                                    < < < < < < < HEAD == == == =

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                                                    // === MOTION TRACKING ===
                                                        UPROPERTY() TMap<FName, FOHBoneMotionData> BoneMotionMap;

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Motion Tracking")
                                                    FName RootReferenceBone =
                                                        "pelvis"; // Reference bone for relative motion

                                                    // Motion quality thresholds
                                                    UPROPERTY(EditAnywhere, Category = "PAC|Motion Tracking")
                                                    float HighQualityMotionThreshold = 0.7f;

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Motion Tracking")
                                                    float PredictionQualityThreshold = 0.5f;
<<<<<<< HEAD

                                                  public:
                                                    // ============== Motion Analysis API ==============

                                                    // === MOTION DATA ===
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneVelocity(FName BoneName,
                                                                            EOHReferenceSpace Space =
                                                                                EOHReferenceSpace::WorldSpace,
                                                                            float SmoothingAlpha = 0.0f) const;
                                                  == == == =

                                                               public :
                                                      // ============== Motion Analysis API ==============

                                                      // === MOTION DATA ===
                                                      UFUNCTION(BlueprintPure, Category = "PAC|Motion") FVector
                                                      GetBoneVelocity(FName BoneName,
                                                                      EOHReferenceSpace Space =
                                                                          EOHReferenceSpace::WorldSpace,
                                                                      float SmoothingAlpha = 0.0f) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneAcceleration(FName BoneName,
                                                                                EOHReferenceSpace Space =
                                                                                    EOHReferenceSpace::WorldSpace,
                                                                                float SmoothingAlpha = 0.0f) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    float GetBoneSpeed(FName BoneName) const;

                                                    const FOHBoneMotionData* GetBoneMotionData(FName BoneName) const;

                                                    // Get motion quality for a specific bone
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    float GetBoneMotionQuality(FName BoneName) const;

                                                    // Check if bone motion is predictable
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    bool IsBoneMotionPredictable(FName BoneName) const;

                                                    // Predict bone position with quality-based method selection
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    FVector PredictBonePosition(FName BoneName, float PredictTime)
                                                        const;

                                                    // Get bezier prediction path for visualization
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    TArray<FVector> GetBonePredictionPath(
                                                        FName BoneName, float PredictTime, bool bCubic = true) const;

                                                    // Get angular velocity
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneAngularVelocity(FName BoneName) const;

                                                    // Get jerk (rate of change of acceleration)
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneJerk(FName BoneName, bool bUseLocalSpace = false)
                                                        const;

                                                    // Check if bone is in motion transition
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    bool IsBoneInVelocityTransition(FName BoneName) const;

                                                    // Detect sudden direction changes
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    bool HasBoneSuddenDirectionChange(
                                                        FName BoneName, float AngleThreshold = 45.0f) const;
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneAcceleration(FName BoneName,
                                                                                EOHReferenceSpace Space =
                                                                                    EOHReferenceSpace::WorldSpace,
                                                                                float SmoothingAlpha = 0.0f) const;

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    float GetBoneSpeed(FName BoneName) const;

                                                    const FOHBoneMotionData* GetBoneMotionData(FName BoneName) const;

                                                    // Get motion quality for a specific bone
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    float GetBoneMotionQuality(FName BoneName) const;

                                                    // Check if bone motion is predictable
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    bool IsBoneMotionPredictable(FName BoneName) const;

                                                    // Predict bone position with quality-based method selection
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    FVector PredictBonePosition(FName BoneName, float PredictTime)
                                                        const;

                                                    // Get bezier prediction path for visualization
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion Analysis")
                                                    TArray<FVector> GetBonePredictionPath(
                                                        FName BoneName, float PredictTime, bool bCubic = true) const;

                                                    // Get angular velocity
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneAngularVelocity(FName BoneName) const;

                                                    // Get jerk (rate of change of acceleration)
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    FVector GetBoneJerk(FName BoneName, bool bUseLocalSpace = false)
                                                        const;

                                                    // Check if bone is in motion transition
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    bool IsBoneInVelocityTransition(FName BoneName) const;

                                                    // Detect sudden direction changes
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Motion")
                                                    bool HasBoneSuddenDirectionChange(
                                                        FName BoneName, float AngleThreshold = 45.0f) const;

<<<<<<< HEAD

                                                  private:
                                                    // Helper to get reference velocity for relative motion
                                                    FVector GetReferenceVelocity() const;

                                                  == == == =

                                                               private :
                                                      // Helper to get reference velocity for relative motion
                                                      FVector
                                                      GetReferenceVelocity() const;
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    // ============================================================================
                                                    // COLLISION AND IMPACT SYSTEM
                                                    // ============================================================================
#pragma region COLLISION_SYSTEM
                                                  < < < < < < < HEAD

                                                      public :
                                                      // ---- Impact Strength Blend System ----
                                                      UFUNCTION(BlueprintCallable, Category = "PAC|Collision") void
                                                      BlendPhysicalAnimationStrength(float Target, float BlendDuration);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyImpactFlinch(float ImpactMagnitude);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void RestorePhysicalAnimationStrengthSmooth();
                                                  == == == =
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

                                                               protected :
                                                      // Flinch blending state
                                                      FOHBlendFloatState PACStrengthBlendState;

<<<<<<< HEAD

                                                  private:
                                                    // Impact tracking
                                                    TMap<FName, float> BoneImpactTimestamps;
                                                    TMap<FName, FVector> LastBoneImpactLocations;

                                                    FVector2D CalculateMovementFromImpact(
                                                        const FHitResult& Hit, const FVector& RelativeVelocity,
                                                        const FOHBoneMotionData* MotionData, float DragCoeff,
                                                        float CrossSection, float Mass) const;
                                                    void UpdateMovementDecay(float DeltaTime);

                                                  == == == = public :
                                                      // ---- Impact Strength Blend System ----
                                                      UFUNCTION(BlueprintCallable, Category = "PAC|Collision") void
                                                      BlendPhysicalAnimationStrength(float Target, float BlendDuration);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyImpactFlinch(float ImpactMagnitude);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void RestorePhysicalAnimationStrengthSmooth();

                                                  protected:
                                                    // Flinch blending state
                                                    FOHBlendFloatState PACStrengthBlendState;

                                                  private:
                                                    // Impact tracking
                                                    TMap<FName, float> BoneImpactTimestamps;
                                                    TMap<FName, FVector> LastBoneImpactLocations;

                                                    FVector2D CalculateMovementFromImpact(
                                                        const FHitResult& Hit, const FVector& RelativeVelocity,
                                                        const FOHBoneMotionData* MotionData, float DragCoeff,
                                                        float CrossSection, float Mass) const;
                                                    void UpdateMovementDecay(float DeltaTime);



>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    static EOHBiologicalMaterial DetermineBoneMaterial(FName BoneName);
                                                    static EOHShapeType DetermineBoneShape(FName BoneName);
                                                    FVector GetBoneBounds(FName BoneName) const;

<<<<<<< HEAD
#pragma region Locomotion

                                                  public:
                                                    // Pushback configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    bool bEnableMovementPushback = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    bool bApplySelfPushback = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float SelfPushbackMultiplier =
                                                        0.3f; // 30% of force applied back to attacker

                                                  == == == =

#pragma region Locomotion
                                                               public :
                                                      // Pushback configuration
                                                      UPROPERTY(
                                                          EditAnywhere, BlueprintReadWrite,
                                                          Category =
                                                              "PAC|Locomotion|Pushback") bool bEnableMovementPushback =
                                                          true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    bool bApplySelfPushback = true;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float SelfPushbackMultiplier =
                                                        0.3f; // 30% of force applied back to attacker
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    bool bUseMassBasedPushback = true;

                                                    // Pushback configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float PushbackForceMultiplier = 1.0f;
<<<<<<< HEAD

                                                    == == ==
                                                        =
    
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                            UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                                      Category =
                                                                          "PAC|Locomotion") float MovementDecayRate =
                                                                3.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion")
                                                    float ImpactToMovementScale = 0.002f;

<<<<<<< HEAD
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    UCurveFloat* PushbackForceCurve =
                                                        nullptr; // Maps impact magnitude to pushback force

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float MinPushbackThreshold =
                                                        200.0f; // Minimum impact force to trigger pushback

                                                    == == ==
                                                        =

                                                            UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                                      Category = "PAC|Locomotion|Pushback")
                                                                UCurveFloat* PushbackForceCurve =
                                                                    nullptr; // Maps impact magnitude to pushback force

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float MinPushbackThreshold =
                                                        200.0f; // Minimum impact force to trigger pushback
    
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Locomotion|Pushback")
                                                    float VerticalPushbackScale =
                                                        0.3f; // How much vertical force converts to pushback

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Locomotion")
                                                    UOHMovementComponent* GetMovementComponent() const;
<<<<<<< HEAD

                                                    void ProcessImpactPushback(const FVector& ImpactForce,
                                                                               const FVector& ImpactLocation,
                                                                               FName ImpactedBone);
                                                    FVector2D CalculatePushbackVector(const FVector& ImpactForce,
                                                                                      const FVector& ImpactNormal);

                                                  private:
                                                    == == ==
                                                        =

                                                            void ProcessImpactPushback(const FVector& ImpactForce,
                                                                                       const FVector& ImpactLocation,
                                                                                       FName ImpactedBone);
                                                    FVector2D CalculatePushbackVector(const FVector& ImpactForce,
                                                                                      const FVector& ImpactNormal);

                                                  private:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                                    // Cached reference to movement component
                                                    UPROPERTY(Transient)
                                                    mutable TWeakObjectPtr<UOHMovementComponent>
                                                        CachedMovementComponent;
                                                    // Add validation flag
                                                    bool bComponentReferencesValidated = false;
                                                    static FVector2D BiasedPushbackForStance(
                                                        const FVector2D& RawPushback,
                                                        const FOHStanceState& StanceState);
                                                    static float CalculateMassRatio(AActor * Attacker,
                                                                                    AActor * Defender);
<<<<<<< HEAD

#pragma endregion

#pragma endregion

                                                  public:
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyImpactImpulse(FName BoneName, const FVector& Impulse,
                                                                            const FVector& Location);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyRadialImpactImpulse(const FVector& Location,
                                                                                  float Magnitude, float Radius);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyRadialImpulseToTarget(UOHPACManager * TargetManager,
                                                                                    const FVector& ImpactPoint,
                                                                                    float BaseForce);

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Collision")
                                                    FVector2D GetAccumulatedMovement() const {
                                                        return AccumulatedMovementVector;
                                                    }

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ClearAccumulatedMovement() {
                                                        AccumulatedMovementVector = FVector2D::ZeroVector;
                                                    }
                                                    // Runtime State
                                                    UPROPERTY(BlueprintReadOnly, Category = "PAC|Collision")
                                                    FVector2D AccumulatedMovementVector = FVector2D::ZeroVector;

                                                    // Configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    bool bEnableCollisionImpactSystem = true;

                                                    // Combat configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactImpulseScale = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactCooldownTime = 0.1f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactRadialFalloff = 0.7f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float RadialImpulseRadius = 30.0f;

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Flinch")
                                                    float ImpactFlinchBlendOutTime = 0.18f;

                                                    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite,
                                                              Category = "PAC|Flinch")
                                                    float FlinchScale = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float MinImpactFlinchThreshold =
                                                        200.0f; // Minimum impact to trigger flinch

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float MaxImpactFlinchThreshold =
                                                        2000.0f; // Maximum impact for flinch scaling

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchStrengthMultiplier =
                                                        0.8f; // How much to reduce PAC strength during flinch

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchMinDuration = 0.1f; // Minimum flinch duration

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchMaxDuration = 0.5f; // Maximum flinch duration

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchRecoveryDelay = 0.2f; // Delay before automatic recovery

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Collision")
                                                    float MinPhysicalAnimationStrength =
                                                        0.7f; // Minimum allowed blend strength

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Collision")
                                                    float MaxPhysicalAnimationStrength = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    UCurveFloat* ImpactResponseCurve =
                                                        nullptr; // Curve for impact response scaling

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float MinAttackSpeed =
                                                        300.0f; // Minimum hand speed to register as attack

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    float MinAttackConfidenceThreshold =
                                                        0.3f; // Minimum confidence to register attack

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    float MinPenetrationThreshold =
                                                        5.0f; // Minimum penetration depth for hit registration

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float MaxCombatRange = 800.0f; // Base radius for collision checks

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float HandForceMultiplier = 1.5f; // Extra oomph for punches

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float FootForceMultiplier = 2.0f; // Kicks hit harder

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    bool bUsePredictiveHitDetection = true;

                                                    // Add these combat targeting settings
                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float PunchThroughDistance = 30.0f; // How far to look ahead

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float CoreBonePriority = 2.0f; // Multiplier for core targets

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float ExtremityPenalty = 0.5f; // Reduction for hitting guards

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    bool bUsePunchThroughTargeting = true;

                                                  protected:
                                                    /** Additional multiplier for clean core hits */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "3.0"))
                                                    float CoreHitPushbackMultiplier = 1.5f;

                                                    /** Multiplier for blocked/extremity hits */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "2.0"))
                                                    float ExtremityHitPushbackMultiplier = 0.8f;

                                                    /** Minimum pushback force to ensure visible effect */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "2000.0"))

                                                    float MinimumPushbackForce = 800.0f;

                                                  private:
                                                    // Core bone definitions
                                                    TSet<FName> CoreBones = {TEXT("spine_01"), TEXT("spine_02"),
                                                                             TEXT("spine_03"), TEXT("neck_01"),
                                                                             TEXT("head"),     TEXT("pelvis")};

                                                    FCombatTargetCandidate SelectBestTargetFromChain(
                                                        const FOHCombatChainData& AttackingChain,
                                                        UOHPACManager* DefenderManager,
                                                        const TArray<FCombatTargetCandidate>& Candidates);

                                                    bool IsCoreBone(FName BoneName) const;
                                                    static bool IsExtremityBone(FName BoneName);
                                                    bool IsExtremityProtectingCore(
                                                        FName ExtremityBone, UOHPACManager * DefenderManager,
                                                        const FVector& AttackDirection) const;
                                                    bool IsCoreInTrajectory(const FCombatTargetCandidate& CoreTarget,
                                                                            const FOHCombatChainData& AttackingChain);
                                                    float CalculateTargetScore(const FCombatTargetCandidate& Candidate,
                                                                               const FOHCombatChainData& AttackingChain,
                                                                               UOHPACManager* DefenderManager);
                                                    static float CalculatePenetrationScore(
                                                        const FCombatTargetCandidate& ExtremityTarget,
                                                        const FOHCombatChainData& AttackingChain,
                                                        const TArray<TArray<FCombatTargetCandidate>>& TimeGroups,
                                                        int32 CurrentGroupIdx);
                                                    void ApplyCoreImpact(UOHPACManager * DefenderManager,
                                                                         FName CoreBone, const FVector& Direction,
                                                                         float Force);
                                                    void ApplyExtremityImpact(UOHPACManager * DefenderManager,
                                                                              FName ExtremityBone,
                                                                              const FVector& Direction, float Force);

#pragma region Delegates

                                                  public:
                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBoneStartedSimulating OnBoneStartedSimulating;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBoneStoppedSimulating OnBoneStoppedSimulating;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBlendCompleted OnBlendCompleted;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPACManagerInitialized OnPACManagerInitialized;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPhysicsImpactProcessed OnPhysicsMovementCalculated;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPhysicsBodyHit OnPhysicsBodyHitEvent;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPushbackApplied OnPushbackApplied;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnCombatImpulseGenerated OnCombatImpulseGenerated;

#pragma endregion

                                                  private:
                                                    static float CalculatePhysicsContribution(UOHMovementComponent *
                                                                                              MovementComponent);
                                                    // Helper functions
                                                    static FString GetBoneSide(FName BoneName);
                                                    float EstimateBoneRadius(FName BoneName) const;
                                                    static uint64 GetCombatHitID(FName AttackerBone,
                                                                                 FName DefenderBone);
                                                    static float EstimateBoneMass(FName BoneName);
                                                    // Combat tracking
                                                    TMap<FName, FOHCombatChainData> CombatChains;

                                                    // Combat hit tracking
                                                    TMap<uint64, float> CombatHitTimestamps;
                                                    // Helper functions
                                                    void InitializeCombatChains();
                                                    void UpdateCombatChainStates(float DeltaTime);
                                                    void UpdateChainMotionData(FOHCombatChainData & Chain,
                                                                               float DeltaTime);
                                                    void UpdateChainTrajectory(FOHCombatChainData & Chain,
                                                                               float DeltaTime);
                                                    void AnalyzeChainAttackPattern(FOHCombatChainData & Chain,
                                                                                   float DeltaTime);
                                                    void CheckChainVsAllBodies(const FOHCombatChainData& AttackingChain,
                                                                               UOHPACManager* DefenderManager);
                                                    bool ProcessChainHit(const FOHCombatChainData& AttackingChain,
                                                                         UOHPACManager* DefenderManager,
                                                                         const FCombatTargetCandidate& Target);
                                                    void CheckChainCombatCollisions(float DeltaTime); // New name
                                                    FName DetermineBestTargetBone(UOHPACManager * TargetManager,
                                                                                  const FVector& PenetrationDirection,
                                                                                  FName AttackingBone) const;

                                                  public:
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Combat Analysis")
                                                    FOHCombatAnalysis AnalyzeCombatState() const;
                                                    static float CalculateAttackIntensity(
                                                        const FOHCombatAnalysis& Analysis);
                                                    static float CalculateCoordinationFactor(int32 ActiveBones,
                                                                                             int32 ActiveChains);

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Combat Analysis")
                                                    static FOHCombatAnalysis AnalyzeCharacterCombatState(ACharacter *
                                                                                                         Character);
                                                    static void ValidateAndClampAnalysisMetrics(FOHCombatAnalysis &
                                                                                                Analysis);
                                                    static float CalculateMovementAlignmentBonus(
                                                        const FVector& CharacterVelocity,
                                                        const FVector& AttackDirection);
                                                    const TMap<FName, FOHCombatChainData>& GetCombatChains() const {
                                                        return CombatChains;
                                                    }

                                                    // Add this to ensure movement component is properly cached
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Setup")
                                                    void ValidateComponentReferences();

#pragma endregion
                                                  == == == =

#pragma endregion

#pragma endregion
                                                               public :

                                                      UFUNCTION(BlueprintCallable, Category = "PAC|Collision") void
                                                      ApplyImpactImpulse(FName BoneName, const FVector& Impulse,
                                                                         const FVector& Location);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyRadialImpactImpulse(const FVector& Location,
                                                                                  float Magnitude, float Radius);

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ApplyRadialImpulseToTarget(UOHPACManager * TargetManager,
                                                                                    const FVector& ImpactPoint,
                                                                                    float BaseForce);

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Collision")
                                                    FVector2D GetAccumulatedMovement() const {
                                                        return AccumulatedMovementVector;
                                                    }

                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Collision")
                                                    void ClearAccumulatedMovement() {
                                                        AccumulatedMovementVector = FVector2D::ZeroVector;
                                                    }
                                                    // Runtime State
                                                    UPROPERTY(BlueprintReadOnly, Category = "PAC|Collision")
                                                    FVector2D AccumulatedMovementVector = FVector2D::ZeroVector;

                                                    // Configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    bool bEnableCollisionImpactSystem = true;

                                                    // Combat configuration
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactImpulseScale = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactCooldownTime = 0.1f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float ImpactRadialFalloff = 0.7f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Collision")
                                                    float RadialImpulseRadius = 30.0f;

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Flinch")
                                                    float ImpactFlinchBlendOutTime = 0.18f;

                                                    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite,
                                                              Category = "PAC|Flinch")
                                                    float FlinchScale = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float MinImpactFlinchThreshold =
                                                        200.0f; // Minimum impact to trigger flinch

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float MaxImpactFlinchThreshold =
                                                        2000.0f; // Maximum impact for flinch scaling

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchStrengthMultiplier =
                                                        0.8f; // How much to reduce PAC strength during flinch

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchMinDuration = 0.1f; // Minimum flinch duration

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchMaxDuration = 0.5f; // Maximum flinch duration

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Combat|Flinch")
                                                    float FlinchRecoveryDelay = 0.2f; // Delay before automatic recovery

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Collision")
                                                    float MinPhysicalAnimationStrength =
                                                        0.7f; // Minimum allowed blend strength

                                                    UPROPERTY(EditDefaultsOnly, Category = "PAC|Combat|Collision")
                                                    float MaxPhysicalAnimationStrength = 1.0f;

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    UCurveFloat* ImpactResponseCurve =
                                                        nullptr; // Curve for impact response scaling

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float MinAttackSpeed =
                                                        300.0f; // Minimum hand speed to register as attack

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    float MinAttackConfidenceThreshold =
                                                        0.3f; // Minimum confidence to register attack

                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PAC|Combat")
                                                    float MinPenetrationThreshold =
                                                        5.0f; // Minimum penetration depth for hit registration

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float MaxCombatRange = 800.0f; // Base radius for collision checks

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float HandForceMultiplier = 1.5f; // Extra oomph for punches

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    float FootForceMultiplier = 2.0f; // Kicks hit harder

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat")
                                                    bool bUsePredictiveHitDetection = true;

                                                    // Add these combat targeting settings
                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float PunchThroughDistance = 30.0f; // How far to look ahead

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float CoreBonePriority = 2.0f; // Multiplier for core targets

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    float ExtremityPenalty = 0.5f; // Reduction for hitting guards

                                                    UPROPERTY(EditAnywhere, Category = "PAC|Combat Targeting")
                                                    bool bUsePunchThroughTargeting = true;

                                                  protected:
                                                    /** Additional multiplier for clean core hits */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "3.0"))
                                                    float CoreHitPushbackMultiplier = 1.5f;

                                                    /** Multiplier for blocked/extremity hits */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "2.0"))
                                                    float ExtremityHitPushbackMultiplier = 0.8f;

                                                    /** Minimum pushback force to ensure visible effect */
                                                    UPROPERTY(EditAnywhere, BlueprintReadWrite,
                                                              Category = "PAC|Movement Pushback",
                                                              meta = (ClampMin = "0.0", ClampMax = "2000.0"))

                                                    float MinimumPushbackForce = 800.0f;

                                                  private:
                                                    // Core bone definitions
                                                    TSet<FName> CoreBones = {TEXT("spine_01"), TEXT("spine_02"),
                                                                             TEXT("spine_03"), TEXT("neck_01"),
                                                                             TEXT("head"),     TEXT("pelvis")};

                                                    FCombatTargetCandidate SelectBestTargetFromChain(
                                                        const FOHCombatChainData& AttackingChain,
                                                        UOHPACManager* DefenderManager,
                                                        const TArray<FCombatTargetCandidate>& Candidates);

                                                    bool IsCoreBone(FName BoneName) const;
                                                    static bool IsExtremityBone(FName BoneName);
                                                    bool IsExtremityProtectingCore(
                                                        FName ExtremityBone, UOHPACManager * DefenderManager,
                                                        const FVector& AttackDirection) const;
                                                    bool IsCoreInTrajectory(const FCombatTargetCandidate& CoreTarget,
                                                                            const FOHCombatChainData& AttackingChain);
                                                    float CalculateTargetScore(const FCombatTargetCandidate& Candidate,
                                                                               const FOHCombatChainData& AttackingChain,
                                                                               UOHPACManager* DefenderManager);
                                                    static float CalculatePenetrationScore(
                                                        const FCombatTargetCandidate& ExtremityTarget,
                                                        const FOHCombatChainData& AttackingChain,
                                                        const TArray<TArray<FCombatTargetCandidate>>& TimeGroups,
                                                        int32 CurrentGroupIdx);
                                                    void ApplyCoreImpact(UOHPACManager * DefenderManager,
                                                                         FName CoreBone, const FVector& Direction,
                                                                         float Force);
                                                    void ApplyExtremityImpact(UOHPACManager * DefenderManager,
                                                                              FName ExtremityBone,
                                                                              const FVector& Direction, float Force);

#pragma region Delegates

                                                  public:
                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBoneStartedSimulating OnBoneStartedSimulating;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBoneStoppedSimulating OnBoneStoppedSimulating;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnBlendCompleted OnBlendCompleted;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPACManagerInitialized OnPACManagerInitialized;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPhysicsImpactProcessed OnPhysicsMovementCalculated;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPhysicsBodyHit OnPhysicsBodyHitEvent;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnPushbackApplied OnPushbackApplied;

                                                    UPROPERTY(BlueprintAssignable, Category = "PAC|Events")
                                                    FOnCombatImpulseGenerated OnCombatImpulseGenerated;

#pragma endregion

                                                  private:
                                                    static float CalculatePhysicsContribution(UOHMovementComponent *
                                                                                              MovementComponent);
                                                    // Helper functions
                                                    static FString GetBoneSide(FName BoneName);
                                                    float EstimateBoneRadius(FName BoneName) const;
                                                    static uint64 GetCombatHitID(FName AttackerBone,
                                                                                 FName DefenderBone);
                                                    static float EstimateBoneMass(FName BoneName);
                                                    // Combat tracking
                                                    TMap<FName, FOHCombatChainData> CombatChains;

                                                    // Combat hit tracking
                                                    TMap<uint64, float> CombatHitTimestamps;
                                                    // Helper functions
                                                    void InitializeCombatChains();
                                                    void UpdateCombatChainStates(float DeltaTime);
                                                    void UpdateChainMotionData(FOHCombatChainData & Chain,
                                                                               float DeltaTime);
                                                    void UpdateChainTrajectory(FOHCombatChainData & Chain,
                                                                               float DeltaTime);
                                                    void AnalyzeChainAttackPattern(FOHCombatChainData & Chain,
                                                                                   float DeltaTime);
                                                    void CheckChainVsAllBodies(const FOHCombatChainData& AttackingChain,
                                                                               UOHPACManager* DefenderManager);
                                                    bool ProcessChainHit(const FOHCombatChainData& AttackingChain,
                                                                         UOHPACManager* DefenderManager,
                                                                         const FCombatTargetCandidate& Target);
                                                    void CheckChainCombatCollisions(float DeltaTime); // New name
                                                    FName DetermineBestTargetBone(UOHPACManager * TargetManager,
                                                                                  const FVector& PenetrationDirection,
                                                                                  FName AttackingBone) const;

                                                  public:
                                                    UFUNCTION(BlueprintPure, Category = "PAC|Combat Analysis")
                                                    FOHCombatAnalysis AnalyzeCombatState() const;
                                                    static float CalculateAttackIntensity(
                                                        const FOHCombatAnalysis& Analysis);
                                                    static float CalculateCoordinationFactor(int32 ActiveBones,
                                                                                             int32 ActiveChains);

                                                    UFUNCTION(BlueprintPure, Category = "PAC|Combat Analysis")
                                                    static FOHCombatAnalysis AnalyzeCharacterCombatState(ACharacter *
                                                                                                         Character);
                                                    static void ValidateAndClampAnalysisMetrics(FOHCombatAnalysis &
                                                                                                Analysis);
                                                    static float CalculateMovementAlignmentBonus(
                                                        const FVector& CharacterVelocity,
                                                        const FVector& AttackDirection);
                                                    const TMap<FName, FOHCombatChainData>& GetCombatChains() const {
                                                        return CombatChains;
                                                    }

                                                    // Add this to ensure movement component is properly cached
                                                    UFUNCTION(BlueprintCallable, Category = "PAC|Setup")
                                                    void ValidateComponentReferences();

#pragma endregion

                                                    >>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
};