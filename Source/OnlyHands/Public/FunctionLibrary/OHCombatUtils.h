// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "MotionWarpingComponent.h"
#include "OHPhysicsStructs.h"
#include "../Game/Structures/S_FightStructure.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Animation/Notify/InputWindowNotifyState.h"
#include "OHCombatUtils.generated.h"

/**
 *
 */

UENUM(BlueprintType)
enum class ERotationContext : uint8 {
    Off UMETA(DisplayName = "Off"),
    TurnOn UMETA(DisplayName = "Turn On"),
    TurnOff UMETA(DisplayName = "Turn Off"),
    Tracking UMETA(DisplayName = "Tracking")
};

USTRUCT(BlueprintType)
struct FStrikeAnalysisResult {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FVector ClosestPointOnTrajectory;

    UPROPERTY(BlueprintReadOnly)
    FName ClosestTargetBone;

    UPROPERTY(BlueprintReadOnly)
    float DistanceToTarget;

    UPROPERTY(BlueprintReadOnly)
    float TimeToCollision;

    UPROPERTY(BlueprintReadOnly)
    float AngularAlignmentScore;

    UPROPERTY(BlueprintReadOnly)
    ACharacter* TargetCharacter;
};

USTRUCT(BlueprintType)
struct FCombatStrikePredictionResult {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    bool bHitPredictionValid = false;

    UPROPERTY(BlueprintReadOnly)
    FVector PredictedStrikePoint = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    FName ClosestTargetBone = NAME_None;

    UPROPERTY(BlueprintReadOnly)
    FVector ClosestTargetBoneLocation = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    float ClosestDistance = 0.f;

    UPROPERTY(BlueprintReadOnly)
    float TimeToClosestPoint = 0.f;

    UPROPERTY(BlueprintReadOnly)
    ACharacter* TargetCharacter = nullptr;

    UPROPERTY(BlueprintReadOnly)
    float DirectionalityScore = 0.f;

    UPROPERTY(BlueprintReadOnly)
    float AngularDeviationDegrees = 0.f;

    UPROPERTY(BlueprintReadOnly)
    FName SkeletalRegion = NAME_None;
};

UCLASS()
class ONLYHANDS_API UOHCombatUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

  public:
    static void AddCharacterUpdateState(ACharacter* Character, UPARAM(ref) TArray<ACharacter*>& OverlappingCharacters,
                                        FVector SourceLocation, ERotationContext CurrentState,
                                        ERotationContext& NextState, ACharacter*& ClosestCharacter);

    static void RemoveCharacterUpdateState(ACharacter* Character, TArray<ACharacter*>& OverlappingCharacters,
                                           FVector SourceLocation, ERotationContext CurrentState,
                                           ERotationContext& NextState, ACharacter*& ClosestCharacter);

    /**
     * Interpolates a scene component's rotation towards the specified target depending on context.
     * @param Component         The scene component to rotate.
     * @param DeltaTime         Interp step (pass world delta time).
     * @param Context           TurnOn, TurnOff, or Tracking.
     * @param OnRotation        Target rotation for TurnOn.
     * @param OffRotation       Target rotation for TurnOff.
     * @param TrackingRotation  Target rotation for Tracking (can be updated externally each tick).
     * @param InterpSpeed       How quickly to rotate (degrees/sec).
     */
    UFUNCTION(BlueprintCallable, Category = "Rotation|Utils")
    static void InterpComponentRotation(USceneComponent* Component, float DeltaTime, ERotationContext Context,
                                        const FRotator& OnRotation, const FRotator& OffRotation,
                                        const FRotator& TrackingRotation, float InterpSpeed = 5.f);

    // Returns true if the pivot's relative rotation equals target rotation within a tolerance
    UFUNCTION(BlueprintPure, Category = "SecurityCamera|Utils")
    static bool HasReachedRotation(USceneComponent* Pivot, FRotator TargetRotation, float Tolerance = 1.0f);

    // Interpolates pivot's relative rotation towards target rotation
    UFUNCTION(BlueprintCallable, Category = "SecurityCamera|Utils")
    static void InterpPivotToTarget(USceneComponent* Pivot, FRotator TargetRotation, float DeltaTime,
                                    float InterpSpeed);

    UFUNCTION(BlueprintPure, Category = "Helper|Utils")
    static ACharacter* GetClosestCharacter(FVector SourceLocation, const TArray<ACharacter*>& Characters);

    static ACharacter* GetClosestCharacter(USceneComponent* Pivot, const TArray<ACharacter*>& Characters);

    UFUNCTION(BlueprintPure, Category = "SecurityCamera|Utils")
    static FRotator GetLookAtRotationToCharacter(USceneComponent* Pivot, ACharacter* TargetCharacter);

    UFUNCTION(BlueprintPure, Category = "SecurityCamera|Utils")
    static ERotationContext DetermineNextRotationContext(ERotationContext CurrentContext,
                                                         int32 NumOverlappingCharacters);

    /** Projects a 3D vector onto the XY (2D) plane */
    UFUNCTION(BlueprintPure, Category = "Math|Vector")
    static FVector2D ProjectVectorTo2D(const FVector& Vector);

    UFUNCTION(BlueprintPure, Category = "Math|Vector")
    static float ComputeDirectionalAlignment2D(const FVector& A, const FVector& B);

    /** Reflects a vector across a normalized axis */
    UFUNCTION(BlueprintPure, Category = "Math|Vector")
    static FVector MirrorVectorOverAxis(const FVector& Input, const FVector& Axis);

    /** Estimates travel time between From and To at a given speed */
    UFUNCTION(BlueprintPure, Category = "Math|Motion")
    static float PredictTravelTime(const FVector& From, const FVector& To, float Speed);

    /** Applies drag to a velocity vector over time */
    UFUNCTION(BlueprintPure, Category = "Math|Physics")
    static FVector ApplyLinearDrag(const FVector& Velocity, float DragCoefficient, float DeltaTime);

    /** Applies friction to a velocity vector */
    UFUNCTION(BlueprintPure, Category = "Math|Physics")
    static FVector ApplyFriction(FVector Velocity, float Friction, float DeltaTime);

    // -------- Simple Math Helpers (FORCE INLINE + BlueprintPure)

    /** Returns 0 if B is zero, otherwise A / B */
    UFUNCTION(BlueprintPure, Category = "Math|Safe")
    static FORCEINLINE float SafeDivide(float A, float B) {
        return FMath::IsNearlyZero(B) ? 0.0f : A / B;
    }

    /** Returns -1, 0, or +1 based on the sign of the float */
    UFUNCTION(BlueprintPure, Category = "Math|Utility")
    static FORCEINLINE int32 GetFloatSign(float Value) {
        return (Value > 0.f) ? 1 : (Value < 0.f) ? -1 : 0;
    }

    /** Clamps float between 0 and 1 */
    UFUNCTION(BlueprintPure, Category = "Math|Clamp")
    static FORCEINLINE float Clamp01(float Value) {
        return FMath::Clamp(Value, 0.f, 1.f);
    }

    /** Snaps a value to the nearest step increment (supports negatives) */
    UFUNCTION(BlueprintPure, Category = "Math|Grid")
    static FORCEINLINE float SnapToStep(float Value, float StepSize) {
        if (FMath::IsNearlyZero(StepSize)) {
            return Value;
        }
        return FMath::GridSnap(Value, StepSize);
    }

    /** Maps value from one range to another */
    UFUNCTION(BlueprintPure, Category = "Math|Remap")
    static FORCEINLINE float RemapRange(float Value, float InMin, float InMax, float OutMin, float OutMax) {
        return (InMax != InMin) ? OutMin + (Value - InMin) * (OutMax - OutMin) / (InMax - InMin) : OutMin;
    }

    // FOOT & LEG ANALYSIS

    /** Returns the height of a foot bone relative to the root */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static float GetRelativeFootHeight(USkeletalMeshComponent* Mesh, FName FootBone);

    /** Returns normalized foot spacing ratio (useful for wide/narrow stances) */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static float GetLegSeparationRatio(USkeletalMeshComponent* Mesh, FName LeftFootBone, FName RightFootBone);

    /** Raycasts downward from a foot bone to detect ground contact */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static bool TraceGroundBelowBone(USkeletalMeshComponent* Mesh, FName BoneName, float MaxDistance,
                                     FHitResult& OutHit);

    /** Simple proximity-only check */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static bool IsFootNearGround(USkeletalMeshComponent* Mesh, FName FootBone, float MaxDistance = 10.f);

    /** Predictive contact using vertical bone velocity */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static bool IsFootApproachingGround(USkeletalMeshComponent* Mesh, FName FootBone, FVector BoneVelocity,
                                        float MaxDistance = 10.f, float MinDescendSpeed = -5.f);

    /** Returns the vertical distance from foot to pelvis (useful for kicks) */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static float GetKneeRaiseHeight(USkeletalMeshComponent* Mesh, FName FootBone, FName PelvisBone);

    /** Returns horizontal distance between both foot bones */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static float GetStrideWidth(USkeletalMeshComponent* Mesh, FName LeftFootBone, FName RightFootBone);

    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static float GetStepSyncRatio(FVector BoneVelocity, float MaxSpeed = 100.f);

    /** Returns 0 for Left foot leading, 1 for Right foot leading */
    UFUNCTION(BlueprintPure, Category = "Movement|Foot")
    static int32 GetStepLeadFoot(USkeletalMeshComponent* Mesh, FName LeftFootBone, FName RightFootBone);

    // --- Spine & Torso Analysis ---

    /** Measures spine_03 lean forward/backward in degrees relative to vertical */
    UFUNCTION(BlueprintPure, Category = "Posture|Spine")
    static float GetTorsoLeanAngle(USkeletalMeshComponent* Mesh, FName SpineBone = "spine_03");

    /** Measures upper body twist (rotation between spine_01 and spine_03) */
    UFUNCTION(BlueprintPure, Category = "Posture|Spine")
    static float GetUpperBodyTwist(USkeletalMeshComponent* Mesh, FName LowerSpine = "spine_01",
                                   FName UpperSpine = "spine_03");

    /** Returns the compression ratio based on pelvis height and distance from the baseline */
    UFUNCTION(BlueprintPure, Category = "Posture|Core")
    static float GetPelvisCompressionRatio(USkeletalMeshComponent* Mesh, FName PelvisBone = "pelvis",
                                           float StandingHeight = 100.f);

    /** Returns the spine arch angle from spine_03 to neck_01 or head */
    UFUNCTION(BlueprintPure, Category = "Posture|Spine")
    static float GetSpineArchAngle(USkeletalMeshComponent* Mesh, FName UpperSpine = "spine_03",
                                   FName HeadBone = "neck_01");

    /** Aggregates lean, twist, and compression for posture analysis (0�1 readiness) */
    UFUNCTION(BlueprintPure, Category = "Posture|Analysis")
    static float GetCumulativeLean(USkeletalMeshComponent* Mesh);

    /** Returns true if arms are significantly outside the core defensive region */
    UFUNCTION(BlueprintPure, Category = "Guard|Detection")
    static bool IsUpperBodyGuardDropped(USkeletalMeshComponent* Mesh, FName ChestBone = "spine_03",
                                        FName LeftElbow = "lowerarm_l", FName RightElbow = "lowerarm_r");

    /** Returns true if the torso is leaning forward in alignment with the actor forward */
    UFUNCTION(BlueprintPure, Category = "Posture|Spine")
    static bool IsCharacterLeaningForward(USkeletalMeshComponent* Mesh, FName SpineBone = "spine_03");

    // --- Arm & Hand Posture ---

    /** Returns 0�1 extension ratios between clavicle and hand */
    UFUNCTION(BlueprintPure, Category = "Pose|Arms")
    static float GetArmExtensionRatio(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone);

    /** Returns true if the hand crosses the torso midline */
    UFUNCTION(BlueprintPure, Category = "Pose|Arms")
    static bool IsArmCrossedMidline(USkeletalMeshComponent* Mesh, FName HandBone, FName PelvisBone);

    /** Returns the shoulder (clavicle) compression ratio, 0 = wide, 1 = tight */
    UFUNCTION(BlueprintPure, Category = "Pose|Core")
    static float GetShoulderCompressionRatio(USkeletalMeshComponent* Mesh, FName ClavicleLeft, FName ClavicleRight);

    /** Returns distance between hand and pelvis or root */
    UFUNCTION(BlueprintPure, Category = "Pose|Arms")
    static float GetHandReachExtent(USkeletalMeshComponent* Mesh, FName HandBone, FName PelvisBone);

    /** Returns lateral cross-body reach amount from hand */
    UFUNCTION(BlueprintPure, Category = "Pose|Arms")
    static float GetCrossBodyReach(USkeletalMeshComponent* Mesh, FName HandBone, FName PelvisBone);

    /** Returns direction vector from clavicle to hand */
    UFUNCTION(BlueprintPure, Category = "Pose|Arms")
    static FVector GetArmLineDirection(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone);

    /** Returns true if hand is within guard zone relative to the chest */
    UFUNCTION(BlueprintPure, Category = "Guard|Arms")
    static bool IsHandWithinGuardZone(USkeletalMeshComponent* Mesh, FName HandBone, FName ChestBone,
                                      float MaxDistance = 35.f);

    UFUNCTION(BlueprintPure, Category = "Motion|StrikeLogic")
    static bool IsArmAlignedWithMovement(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone,
                                         FVector ActorVelocity, float ThresholdDegrees = 30.f);

    /** Returns angle in degrees between arm direction and actor forward */
    UFUNCTION(BlueprintPure, Category = "Motion|StrikeLogic")
    static float GetArmToFacingAngle(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone);

    /** Returns scalar score (0�1) of how well a strike prep aligns with lean and movement */
    UFUNCTION(BlueprintPure, Category = "Motion|StrikeLogic")
    static float GetStrikePrepAlignmentScore(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone,
                                             FVector ActorVelocity);

    /** Returns true if the current arm line is generally pointing toward the target */
    UFUNCTION(BlueprintPure, Category = "Motion|StrikeLogic")
    static bool IsStrikeArcingTowardTarget(USkeletalMeshComponent* Mesh, FName ClavicleBone, FName HandBone,
                                           AActor* Target, float ThresholdDegrees = 45.f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Camera")
    static FRotator GetActorRotationRelativeToCamera(const AActor* TargetActor);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Camera")
    static FVector GetActorDirectionFromCamera(const AActor* TargetActor);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Camera")
    static float GetActorFacingAngleRelativeToCamera(const AActor* TargetActor);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Camera")
    static float GetActorMovementDirectionRelativeToCamera(const AActor* TargetActor);

    /**
     * Returns an array containing only ACharacter actors from the input.
     * @param Actors           Array of actor pointers to filter.
     * @return                 Array of ACharacter* contained in input.
     */
    UFUNCTION(BlueprintPure, Category = "Character|Utils")
    static TArray<ACharacter*> GetCharactersFromActors(const TArray<AActor*>& Actors);

    /**
     * Returns true if any actors are characters, and outputs the closest character to SourceLocation.
     * @param SourceLocation        Point to measure from.
     * @param Actors                Array of actors to check.
     * @param ClosestCharacter      (Out) The closest character pointer (nullptr if none).
     * @return                      True if any valid character is in the array.
     */
    UFUNCTION(BlueprintPure, Category = "Character|Utils")
    static bool GetClosestCharacterToLocationFromActors(const FVector& SourceLocation, const TArray<AActor*>& Actors,
                                                        ACharacter*& ClosestCharacter);

    /** Gets the local player's camera location and rotation (client-side only).
     *  Use with caution in multiplayer. This retrieves the local viewport player's camera.
     */
    UFUNCTION(BlueprintPure, Category = "Combat|Camera", meta = (WorldContext = "ContextObject"))
    static bool GetPlayerCameraView(const UObject* ContextObject, FTransform& OutTransform);

    UFUNCTION(BlueprintPure, Category = "Combat|Camera", meta = (WorldContext = "ContextObject"))
    static bool GetActorView(const UObject* ContextObject, FTransform& OutTransform);

    static FTransform GetActorView(const UObject* ContextObject, bool& bSuccess);

    static FTransform GetActorView(const UObject* ContextObject);

    static bool GetActorView_Internal(const UObject* ContextObject, FTransform& OutTransform);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void DebugDrawActorMovementDirection(const AActor* TargetActor, float Duration = 2.0f,
                                                FColor Color = FColor::Yellow);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Detection")
    static bool IsActorWithinRange(const AActor* Source, const AActor* Target, float MaxDistance, float Radius = 0.f);

    /** Returns hit direction relative to the target's forward direction. Supports 4 or 8 directional modes. */
    UFUNCTION(BlueprintPure, Category = "OH|HitAnalysis")
    static EOHHitDirection ClassifyHitDirection(const FHitResult& Hit, const AActor* TargetActor,
                                                bool bUse8Directions = true);

    UFUNCTION(BlueprintCallable, Category = "OH|Debug")
    static void DebugDrawHitDirection(const FHitResult& Hit, const AActor* TargetActor, bool bUse8Directions = true,
                                      float Duration = 2.f, float ArrowSize = 100.f);

#pragma region Animation

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Contact")
    static FStrikeContactMetrics ComputeStrikeContactMetrics(const FVector& ContactNormal, const FVector& Velocity,
                                                             float PenetrationDepth = 1.f);

    /** Returns the world-space velocity of a bone, using physics or fallback pose delta */
    UFUNCTION(BlueprintCallable, Category = "OH|Animation|Velocity")
    static FVector GetBoneVelocitySafe(USkeletalMeshComponent* Mesh, FName BoneName, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "OH|Notify")
    static void BeginDeferredStrikeSweep(UPARAM(ref) FDeferredStrikeSweepState& State, USkeletalMeshComponent* Mesh,
                                         AActor* Owner, const TArray<FName>& BoneChain, float SampleInterval = 0.016f);

    UFUNCTION(BlueprintCallable, Category = "OH|Notify")
    static void TickDeferredStrikeSweep(UPARAM(ref) FDeferredStrikeSweepState& State, float DeltaTime);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static bool GetActiveMontageBlendWeight(USkeletalMeshComponent* Mesh, float& OutBlendWeight,
                                            FName& OutCurrentSectionName);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static FOHMontagePlaybackState GetMontagePlaybackState(USkeletalMeshComponent* Mesh);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static FOHAnimInstancePlaybackState GetAnimInstancePlaybackState(USkeletalMeshComponent* Mesh);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static FOHSlotBlendState GetSlotBlendState(USkeletalMeshComponent* Mesh, FName SlotName);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static FOHRootMotionState GetPoseRootMotionState(USkeletalMeshComponent* Mesh, float DeltaTime);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation", meta = (WorldContext = "Mesh"))
    static FOHMontageSectionWindow GetAnimSectionTransitionWindow(USkeletalMeshComponent* Mesh,
                                                                  float CancelWindowStart = 0.2f,
                                                                  float CancelWindowEnd = 0.8f);

    UFUNCTION(BlueprintCallable, Category = "Combat|Debug")
    static void DrawPredictedRootMotionArc(const UObject* WorldContextObject, const FVector& StartLocation,
                                           const FOHRootMotionState& RootMotion, FColor Color = FColor::Cyan,
                                           float Duration = 1.f, float Thickness = 2.f);

    /** Returns the time range of a section in a montage */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static bool GetMontageSectionTimeRange(UAnimMontage* Montage, FName SectionName, float& OutStartTime,
                                           float& OutEndTime);

    /** Returns the normalized time (0.0–1.0) within a montage section given a playback time */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static bool GetMontageSectionNormalizedTime(UAnimMontage* Montage, FName SectionName, float PlaybackTime,
                                                float& OutNormalizedTime);

    /** Returns true if the montage section is within the specified normalized window (e.g. cancel or chain window) */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static bool IsInMontageSectionWindow(const FOHMontagePlaybackState& PlaybackState, float WindowStart,
                                         float WindowEnd);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static TArray<FInputWindow> ExtractInputWindowsFromMontage(UAnimMontage* Montage, FName SectionName);

    /** True if NOT within section window */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static bool IsOutsideMontageSectionWindow(const FOHMontagePlaybackState& State, float WindowStart, float WindowEnd);

    /** True if within normalized segment play window */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static bool IsInMontageSegmentWindow(const FOHMontagePlaybackState& State, float SegmentStart, float SegmentEnd);

    /** Returns remaining time in the current section */
    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static float GetRemainingSectionTime(const FOHMontagePlaybackState& State);

    UFUNCTION(BlueprintCallable, Category = "Combat|Animation")
    static UAnimSequence* ExtractActiveSequenceFromMesh(USkeletalMeshComponent* Mesh);

    UFUNCTION(BlueprintPure, Category = "Combat|Animation")
    static UAnimSequence* ExtractActiveSequenceFromMeshPure(USkeletalMeshComponent* Mesh);

    UFUNCTION(BlueprintCallable, Category = "Combat|Animation")
    static bool ExtractActiveSequenceAndTimeFromMesh(USkeletalMeshComponent* Mesh, UAnimSequence*& OutSequence,
                                                     float& OutLocalTime);

    static bool ResolveAnimSequenceAtTimeRecursive(UAnimSequenceBase* AnimBase, float Time, UAnimSequence*& OutSequence,
                                                   float& OutLocalTime);

#pragma endregion

#pragma region StrikeComputation

    /** Validates contact using velocity vs. normal and estimated force */
    UFUNCTION(BlueprintPure, Category = "Combat|HitValidation")
    static bool IsStrikeContactValid(FVector Velocity, FVector ContactNormal, float EstimatedForce,
                                     float MinDotThreshold = 0.2f, float MinEstimatedForce = 20.f);

    /** Scores an impact based on velocity alignment and estimated force */
    UFUNCTION(BlueprintPure, Category = "Combat|HitScoring")
    static float ComputeContactScore(FVector Velocity, FVector ContactNormal, float EstimatedForce);

    // Calculate contact quality from bone velocities in world space (0=tangential, 1=perpendicular)
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Contact", meta = (DisplayName = "Calculate Contact Quality"))
    static float CalculateContactQuality(const FVector& StrikingBoneWorldVelocity, const FVector& ContactNormal,
                                         const FVector& TargetBoneWorldVelocity = FVector::ZeroVector) {
        FVector RelativeVelocity = StrikingBoneWorldVelocity - TargetBoneWorldVelocity;
        float Speed = RelativeVelocity.Size();

        if (Speed < KINDA_SMALL_NUMBER)
            return 0.0f;

        // Dot product with negative normal (velocity INTO surface)
        float Alignment = FVector::DotProduct(RelativeVelocity.GetSafeNormal(), -ContactNormal);

        // Remap [-1,1] to [0,1] where 1 = perpendicular impact
        return FMath::Clamp((Alignment + 1.0f) * 0.5f, 0.0f, 1.0f);
    }

    // Determine if contact is glancing based on world velocity and normal
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Contact", meta = (DisplayName = "Is Glancing Contact"))
    static bool IsGlancingContact(const FVector& BoneWorldVelocity, const FVector& ContactNormal,
                                  float GlancingThreshold = 0.3f // Below this = glancing
    ) {
        float Quality = CalculateContactQuality(BoneWorldVelocity, ContactNormal);
        return Quality < GlancingThreshold;
    }

    // Calculate deflection direction for glancing blows in world space
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Contact", meta = (DisplayName = "Calculate Deflection Direction"))
    static FVector CalculateDeflectionDirection(const FVector& IncomingWorldVelocity, const FVector& ContactNormal,
                                                float Elasticity = 0.3f // 0=pure slide, 1=perfect bounce
    ) {
        // Reflect velocity about normal
        FVector Reflected =
            IncomingWorldVelocity - 2.0f * FVector::DotProduct(IncomingWorldVelocity, ContactNormal) * ContactNormal;

        // Tangent component (sliding along surface)
        FVector Tangent =
            IncomingWorldVelocity - FVector::DotProduct(IncomingWorldVelocity, ContactNormal) * ContactNormal;

        // Blend between sliding and bouncing
        return FMath::Lerp(Tangent, Reflected, Elasticity).GetSafeNormal();
    }

    // Calculate how flush/direct a strike is (0=glancing, 1=perpendicular)
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Analysis", meta = (DisplayName = "Calculate Strike Flushness"))
    static float CalculateStrikeFlushness(const FHitResult& Hit);

    /** Sorts contacts by impact severity */
    UFUNCTION(BlueprintCallable, Category = "Combat|HitScoring")
    static void SortContactsByImpactScore_Raw(TArray<FVector>& Velocities, TArray<FVector>& Normals,
                                              TArray<float>& Forces, TArray<int32>& OutSortedIndices);

    UFUNCTION(BlueprintPure, Category = "Combat|HitScoring")
    static FVector ConvertWorldToActorLocal(const AActor* ReferenceActor, const FVector& WorldLocation);

    UFUNCTION(BlueprintPure, Category = "Combat|HitValidation")
    static bool IsVelocityAlignedWithImpactNormal(const FVector& Velocity, const FVector& Normal,
                                                  float ThresholdDegrees = 45.f);

    UFUNCTION(BlueprintCallable, Category = "Combat|HitScoring")
    static void FilterRedundantHitsByActor(const TArray<AActor*>& HitActors, TArray<int32>& OutUniqueIndices);

    /** Draws a debug path connecting world positions (strike arc, motion trail, etc.) */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void DrawDebugPath(const UObject* WorldContextObject, const TArray<FVector>& Points, FColor Color,
                              float Duration = 1.0f, float Thickness = 1.5f, bool bLoop = false,
                              bool bPersistentLines = true);

    /** Draws a debug strike cone from origin toward velocity direction */
    UFUNCTION(BlueprintCallable, Category = "Combat|Debug")
    static void DrawDebugStrikeCone(const UObject* WorldContextObject, const FVector& Origin, const FVector& Velocity,
                                    float Length = 100.f, float AngleDegrees = 30.f, FColor Color = FColor::White,
                                    float Duration = 1.f, float Thickness = 1.f);

    /** Returns the world-space linear velocity of a bone on a skeletal mesh */
    UFUNCTION(BlueprintPure, Category = "Combat|Motion")
    static FVector GetBoneLinearVelocity(const USkeletalMeshComponent* Mesh, FName BoneName,
                                         float DeltaTime = 0.016f); // Optional: override frame delta

    /** Computes velocity from manually sampled bone positions */
    UFUNCTION(BlueprintPure, Category = "Combat|Motion")
    static FVector ComputeBoneLinearVelocityFromSamples(const FVector& CurrentPos, const FVector& PreviousPos,
                                                        float DeltaTime = 0.016f);

    /** Computes a bone's velocity relative to the mesh (excluding root/component movement) */
    UFUNCTION(BlueprintPure, Category = "Combat|Motion")
    static FVector ComputeBoneVelocityMeshRelative(const USkeletalMeshComponent* Mesh, FName BoneName,
                                                   float DeltaTime = 0.016f);

    /** Predicts where a bone will be based on current animation + time offset */
    UFUNCTION(BlueprintPure, Category = "Combat|Prediction")
    static FVector PredictBonePositionFromAnimation(const USkeletalMeshComponent* Mesh, FName BoneName,
                                                    float TimeOffset = 0.1f);

    /** Samples the animation pose at a given time and returns the component-space transform of the specified bone. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static bool GetBoneTransformAtTime(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh, FName BoneName,
                                       double Time, FTransform& OutTransform);

    /** Samples the animation pose and returns the world-space transform of the specified bone at the given time. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static bool GetBoneTransformWorldAtTime(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh, FName BoneName,
                                            double Time, FTransform& OutTransform);

    /** Accepts UAnimSequenceBase and resolves to current UAnimSequence at time. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static bool GetBoneTransformFromAnimBase(UAnimSequenceBase* AnimBase, USkeletalMeshComponent* Mesh, FName BoneName,
                                             double Time, FTransform& OutTransform, bool bWorldSpace = false);

    /** Outputs rotation separately as quaternion (from resolved transform). */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static bool GetBoneRotationAtTime(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh, FName BoneName,
                                      double Time, FRotator& OutRotation, bool bWorldSpace = false);

    /** Computes world-space linear velocity of a bone over a window centered on the given time. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static bool GetBoneVelocityAtTime(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh, FName BoneName,
                                      double Time, FVector& OutVelocity,
                                      float SampleWindow = 0.01f); // 1 frame @ 100fps

    UFUNCTION(BlueprintPure, Category = "OH|Animation")
    static FTransform GetBoneComponentSpaceTransformAtTime(UAnimSequence* Sequence, const USkeletalMeshComponent* Mesh,
                                                           FName BoneName, double Time);

    static FAnimExtractContext MakeExtractContext(double Time, bool bExtractRootMotion = false, bool bLooping = false) {
        static FDeltaTimeRecord DummyRecord;
        return FAnimExtractContext(Time, bExtractRootMotion, DummyRecord, bLooping);
    }

    static FAnimExtractContext MakeExtractContext(double Time, const FDeltaTimeRecord& DeltaTime,
                                                  bool bExtractRootMotion = false, bool bLooping = false) {
        return FAnimExtractContext(Time, bExtractRootMotion, DeltaTime, bLooping);
    }

    /** Performs line traces along a strike arc to detect hits */
    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static void DetectContactAlongStrikeArc(const UObject* WorldContextObject, const TArray<FVector>& ArcPoints,
                                            TArray<FHitResult>& OutHits, ECollisionChannel TraceChannel = ECC_Pawn,
                                            float SegmentThickness = 0.f); // 0 = line trace, > 0 = sphere sweep

    /** Approximates a convex sweep volume (capsule segments) along an arc and tests for hits */
    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static void SweepCapsuleAlongStrikeArc(const UObject* WorldContextObject, const TArray<FVector>& ArcPoints,
                                           float Radius, float HalfHeight, TArray<FHitResult>& OutHits,
                                           ECollisionChannel TraceChannel = ECC_Pawn);

    /** Uses GJK to detect collision between a strike arc and a target convex shape */
    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static bool DetectGJKContactFromStrikeArc(const TArray<FVector>& ArcPoints,
                                              const TArray<FVector>& TargetConvexPoints, FVector& OutContactPoint,
                                              FVector& OutPenetrationVector);

    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static void GenerateStrikeArcFromLiveMesh(const USkeletalMeshComponent* Mesh, FName BoneName, int32 NumSamples,
                                              float SampleInterval, TArray<FVector>& OutArc);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling")
    static bool GenerateStrikeArcFromAnimBase(UAnimSequenceBase* AnimBase, USkeletalMeshComponent* Mesh, FName BoneName,
                                              float StartTime, float EndTime, TArray<FVector>& OutPoints,
                                              float SampleRate = 60.f);

    /** Runtime-safe arc sampling using time-based SampleRate (Hz). */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling")
    static bool GenerateStrikeArcFromAnimSequence(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh, FName BoneName,
                                                  float StartTime, float EndTime, TArray<FVector>& OutPoints,
                                                  float SampleRate = 60.f);

    // === INTERNAL / DEBUG ===

    /** Internal: Fixed-resolution arc sampling using a target number of samples (for preview/debug). */
    static bool GenerateStrikeArcFromAnimSequence_FixedResolution(UAnimSequence* Sequence, USkeletalMeshComponent* Mesh,
                                                                  FName BoneName, int32 NumSamples, float StartTime,
                                                                  float EndTime, TArray<FVector>& OutPoints);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling")
    static bool GenerateStrikeArcFromComposite(UAnimComposite* Composite, USkeletalMeshComponent* Mesh, FName BoneName,
                                               float StartTime, float EndTime, TArray<FVector>& OutPoints,
                                               float SampleRate = 60.f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling")
    static bool GenerateStrikeArcFromMontage(UAnimMontage* Montage, USkeletalMeshComponent* Mesh, FName BoneName,
                                             float StartTime, float EndTime, TArray<FVector>& OutPoints,
                                             float SampleRate = 60.f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static FCombatStrikePredictionResult PredictStrikeCollision(const UObject* WorldContextObject, ACharacter* Attacker,
                                                                UOHPhysicsManager* PhysicsManager, FName BoneName,
                                                                float PredictionTime = 0.25f, int Steps = 10,
                                                                bool bEnableDebug = true);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static FCombatStrikePredictionResult AnalyzeStrikeTrajectoryAndTargeting(
        const UObject* WorldContextObject, ACharacter* Attacker, UOHPhysicsManager* PhysicsManager, FName StrikeBone,
        float PredictionTime = 0.35f, int Steps = 8, bool bEnableDebug = false, FName DebugTag = NAME_None);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static FCombatStrikePredictionResult
    AnalyzeAdvancedStrikeTrajectory3D(const UObject* WorldContextObject, ACharacter* Attacker,
                                      UOHPhysicsManager* PhysicsManager, FName StrikeBone, float PredictionTime,
                                      int Steps, bool bEnableDebug, FName DebugTag, FName OptionalChainStart,
                                      int32 MaxChainDepth, bool bIsFullBodyAnimation,
                                      float HistoricalBlendAlpha // new input
    );

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static FCombatStrikePredictionResult
    AnalyzeAdvancedStrikeTrajectory2D(const UObject* WorldContextObject, ACharacter* Attacker,
                                      UOHPhysicsManager* PhysicsManager, FName StrikeBone, float PredictionTime,
                                      int Steps, bool bEnableDebug, FName DebugTag, FName OptionalChainStart,
                                      int32 MaxChainDepth, bool bIsFullBodyAnimation, float HistoricalBlendAlpha);
#pragma endregion

#pragma region Sampling

    /** Computes velocity and acceleration from a set of bone positions. */
    UFUNCTION(BlueprintPure, Category = "Combat|Prediction")
    static void ComputeMotionDerivatives(const TArray<FVector>& Points, float SampleDelta,
                                         TArray<FVector>& OutVelocities, TArray<FVector>& OutAccelerations);

    /**
     * Samples bone world positions from any UAnimSequenceBase-derived animation and outputs a motion sample.
     * Optionally draws debug lines in world space.
     */
    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static FOHBoneMotionSample SampleBoneMotionFromAnimBase(UAnimSequenceBase* AnimBase, USkeletalMeshComponent* Mesh,
                                                            FName BoneName, int32 NumSamples, float StartTime = -1.f,
                                                            float EndTime = -1.f, FName MontageSectionName = NAME_None,
                                                            bool bDrawDebug = false, FColor DebugColor = FColor::White,
                                                            float DebugDuration = 2.0f, float DebugThickness = 2.0f);

    /**
     * Pure version of SampleBoneMotionFromAnimBase: no debug draw, no side effects.
     */
    UFUNCTION(BlueprintPure, Category = "Combat|Prediction")
    static FOHBoneMotionSample SampleBoneMotionFromAnimBasePure(UAnimSequenceBase* AnimBase,
                                                                USkeletalMeshComponent* Mesh, FName BoneName,
                                                                int32 NumSamples, float StartTime = -1.f,
                                                                float EndTime = -1.f,
                                                                FName MontageSectionName = NAME_None);

    /** Extracts a usable UAnimSequence from any UAnimSequenceBase, Composite, or Montage. Returns nullptr if none
     * found. */
    UFUNCTION(BlueprintCallable, Category = "Combat|Prediction")
    static UAnimSequence* ExtractSequenceFromAnimBase(UAnimSequenceBase* AnimBase);

    /** Extracts the underlying UAnimSequence active at a specific time in a composite or montage */
    UFUNCTION(BlueprintCallable, Category = "Combat|Animation")
    static UAnimSequence* ExtractSequenceFromAnimBaseAtTime(UAnimSequenceBase* AnimBase, float Time);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling|Debug")
    static void VisualizeBoneMotionArcFromAnim(const UObject* WorldContextObject, UAnimSequenceBase* AnimBase,
                                               USkeletalMeshComponent* Mesh, FName BoneName, int32 NumSamples = 16,
                                               FName MontageSectionName = NAME_None, FColor Color = FColor::Red,
                                               float Duration = 2.f, float Thickness = 2.f, bool bLoop = false,
                                               bool bDrawVelocity = false, bool bDrawImpactNormals = false);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|ArcSampling|Debug")
    static void VisualizePredictedArc(const UObject* WorldContextObject, USkeletalMeshComponent* Mesh, FName BoneName,
                                      float PredictionWindow = 0.5f, int32 NumSamples = 12,
                                      FColor Color = FColor::Green, float Duration = 2.f, float Thickness = 2.f,
                                      bool bDrawVelocity = false, bool bDrawImpactNormals = false);

    static void StartTimedBoneArcSampling(UObject* WorldContextObject, USkeletalMeshComponent* Mesh, FName BoneName,
                                          float Duration, int32 NumSamples,
                                          TFunction<void(const TArray<FVector>&)> OnComplete);

    /** Estimates world-space linear velocity of a bone at a given time in the animation. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Motion")
    static bool GetBoneWorldSpaceVelocityAtTime(UAnimSequence* Sequence, const USkeletalMeshComponent* Mesh,
                                                FName BoneName, float Time, FVector& OutVelocity,
                                                float SampleWindow = 0.01f); // default to ~1 frame @ 60fps

    /** Computes motion derivatives (velocity, acceleration) from sampled positions. */
    static void SampleMotionDerivatives(const TArray<FVector>& Positions, float SampleInterval,
                                        TArray<FVector>& OutVelocities, TArray<FVector>& OutAccelerations);

    /** Visualizes velocity and acceleration vectors along a motion arc. */
    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void VisualizeMotionDerivatives(const UObject* WorldContextObject, const TArray<FVector>& Points,
                                           const TArray<FVector>& Velocities, const TArray<FVector>& Accelerations,
                                           float Duration = 1.5f, float Scale = 15.f, float Thickness = 1.5f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static FOHBoneMotionSample GenerateBoneMotionSample(UAnimSequenceBase* AnimBase, USkeletalMeshComponent* Mesh,
                                                        FName BoneName, int32 NumSamples = 16,
                                                        FName MontageSectionName = NAME_None, float StartTime = 0.f,
                                                        float EndTime = 0.f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Sampling")
    static void ScheduleDeferredBoneArcSample(UObject* WorldContext, UAnimSequenceBase* AnimBase,
                                              USkeletalMeshComponent* Mesh, FName BoneName, int32 NumSamples,
                                              FName MontageSectionName, FColor DebugColor = FColor::Green,
                                              float Duration = 1.0f, float Thickness = 1.5f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void TestVisualizeBoneMotionArcFromMesh(USkeletalMeshComponent* Mesh, FName BoneName,
                                                   float PredictDuration = 0.5f, int32 NumSamples = 12,
                                                   FColor DebugColor = FColor::Blue, float DebugLineDuration = 3.0f,
                                                   float LineThickness = 2.0f, bool bLoop = false);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void TestDrawArcFromMesh(USkeletalMeshComponent* Mesh, FName BoneName, float Duration, FColor Color);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void QuickDrawBoneArcFromMesh(USkeletalMeshComponent* Mesh, FName BoneName, float PredictDuration = 0.5f,
                                         FColor Color = FColor::Green, float Duration = 5.f, float Thickness = 2.f);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat|Debug")
    static void ScheduleDeferredArcDraw(USkeletalMeshComponent* Mesh, FName BoneName = FName("hand_r"),
                                        float PredictDuration = 0.5f, FColor DebugColor = FColor::Green,
                                        float Duration = 10.f, float Thickness = 3.f);

  private:
    /** Internal shared implementation for motion sampling. Do not call directly. */
    static FOHBoneMotionSample SampleBoneMotionInternal(UAnimSequenceBase* AnimBase, USkeletalMeshComponent* Mesh,
                                                        FName BoneName, int32 NumSamples, float StartTime,
                                                        float EndTime, FName MontageSectionName);

    static UObject* GetValidBoneAssetForContainer(const USkeletalMeshComponent* Mesh);

    UFUNCTION(BlueprintPure, Category = "OH|Combat")
    static void GetBoneChainAngularMotion(UOHPhysicsManager* PhysicsManager, const TArray<FName>& BoneChain,
                                          FVector& OutAngularVelocity, FVector& OutAngularAcceleration);

    UFUNCTION(BlueprintCallable, Category = "OH|Physics")
    static FVector GetBlendedChainVelocity(UOHPhysicsManager* PhysicsManager, const USkeletalMeshComponent* Mesh,
                                           FName StartBone, FName EndBone, float HistoryWeight = 0.5f,
                                           int NumSamples = 5);

    UFUNCTION(BlueprintCallable, Category = "OH|Physics")
    static FVector GetBlendedChainAcceleration(UOHPhysicsManager* PhysicsManager, const USkeletalMeshComponent* Mesh,
                                               FName StartBone, FName EndBone, float HistoryWeight = 0.5f,
                                               int NumSamples = 5);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static void GetBlendedBoneChainMotion(UOHPhysicsManager* PhysicsManager, const TArray<FName>& BoneChain,
                                          float HistoryWeight, int NumSamples, FVector& OutVelocity,
                                          FVector& OutAcceleration, FVector& OutStartLocation, FVector& OutForward);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static void GetBlendedBoneChainMotionData(UOHPhysicsManager* PhysicsManager, const TArray<FName>& BoneChain,
                                              float HistoryWeight, int NumSamples, FVector& OutVelocity,
                                              FVector& OutAcceleration, FVector& OutStartLocation, FVector& OutForward,
                                              FVector& OutAngularVelocity, FVector& OutAngularAcceleration,
                                              float& OutJitterScore);

    UFUNCTION(BlueprintCallable, Category = "OH|Combat")
    static void GetBlendedBoneChainMotionFromStrikeBone(UOHPhysicsManager* PhysicsManager,
                                                        const USkeletalMeshComponent* Mesh, FName StrikeBone,
                                                        FName ChainStart, int32 MaxDepth, bool bIsFullBodyAnimation,
                                                        float HistoryWeight, int NumSamples, FVector& OutVelocity,
                                                        FVector& OutAcceleration, FVector& OutStartLocation,
                                                        FVector& OutForward, FVector& OutAngularVelocity,
                                                        FVector& OutAngularAcceleration, float& OutJitterScore);

#pragma endregion

#pragma region MotionWarping
    UFUNCTION(BlueprintCallable, Category = "Combat|MotionWarping")
    static void AddWarpTargetToNearestEnemy(UMotionWarpingComponent* MotionWarpingComponent,
                                            float SearchRadius = 800.0f, FName WarpTargetName = "EnemyTarget");

#pragma endregion

#pragma region InlineFunctions

    static FVector GetEnhancedPredictedDirection(const FOHBoneData* BoneData) {
        if (!BoneData || !BoneData->IsValid())
            return FVector::ZeroVector;

        const FVector BlendedVel = BoneData->GetBlendedLinearVelocity(0.5f, 5);       // blend current and historical
        const FVector BlendedAccel = BoneData->GetBlendedLinearAcceleration(0.5f, 5); // more stable acceleration
        const FVector AngularInfluence = FVector::CrossProduct(BoneData->GetBodyAngularVelocity(), FVector::UpVector);

        // Weight each contributor based on empirical importance
        const FVector Composite = (BlendedVel * 0.55f) + (AngularInfluence * 0.15f) + (BlendedAccel * 0.3f);

        return Composite.GetSafeNormal();
    }

#pragma endregion

#pragma region ForceFeedback

    /** Computes a vulnerability score (0–1) based on impact angle and graph data */
    UFUNCTION(BlueprintCallable, Category = "OH|GraphUtils|Impulse")
    static float ComputeImpulseVulnerabilityScore(const FOHPhysicsGraphNode& Graph, FName BoneName,
                                                  const FVector& ImpactNormal, const FVector& RootLocation);

    static float ComputeImpulseVulnerabilityScore(const FOHBoneData& BoneData, const FVector& ImpactDirection,
                                                  const FVector& RootLocation);

    /** Computes an impulse vector for a given bone from the graph */
    static FVector
    ComputeImpulseVectorForBone(const FOHPhysicsGraphNode& Graph, FName BoneName, const FHitResult& Hit,
                                const FVector& RootLocation, float BaseStrength, EOHImpulseMode ModeOverride,
                                bool bAutoInferMode,
                                EImpulseDirectionMode DirectionMode = EImpulseDirectionMode::FromHitNormal);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static float ComputeImpulseStrengthFromBoneData(const FOHBoneData& BoneData,
                                                    const FVector& OptionalStrikeDir = FVector::ZeroVector,
                                                    float BaseMultiplier = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static float ComputeImpulseStrengthFromHit(const FOHBoneData& BoneData, const FHitResult& Hit,
                                               const USkeletalMeshComponent* SkeletalMesh, float BaseMultiplier = 1.0f);

    // Direction-only helpers
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector GetImpulseDirectionFromHitNormal(const FHitResult& Hit);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector GetImpulseDirectionFromBoneToImpactPoint(const FOHBoneData& BoneData, const FHitResult& Hit);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector GetImpulseDirectionFromHit(const FOHBoneData& BoneData, const FHitResult& Hit,
                                              EImpulseDirectionMode Mode);

    // Strength-only wrappers
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static float ComputeImpulseStrengthFromNormal(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                  float BaseMultiplier = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static float ComputeImpulseStrengthFromImpactVector(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                        float BaseMultiplier = 1.0f);

    // Final impulse vector (strength × direction)
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector ComputeImpulseVectorFromHit(const FOHBoneData& BoneData, const FHitResult& Hit,
                                               float BaseMultiplier = 1.0f,
                                               EImpulseDirectionMode Mode = EImpulseDirectionMode::FromHitNormal);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector ComputeImpulseVectorFromHitNormal(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                     float BaseMultiplier = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector ComputeImpulseVectorFromImpactVector(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                        float BaseMultiplier = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static EImpulseDirectionMode InferBestImpulseDirectionMode(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                               float AlignmentThreshold = 0.5f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector ComputeSelfReactionImpulse(const FOHBoneData& BoneData, const FHitResult& Hit,
                                              const FVector& RootLocation, float BaseStrength = 1000.f,
                                              float Mass = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse")
    static FVector ComputeImpulseVectorFromBone(const FOHBoneData& BoneData, const FHitResult& Hit,
                                                const FVector& RootLocation, float BaseStrength,
                                                EImpulseDirectionMode ModeOverride, bool bAutoInferMode);
#pragma endregion

  public:
    UFUNCTION(BlueprintPure, Category = "OH|Targeting")
    static AActor* FindMostLikelyOpponent(const AActor* SelfActor, const FString& ContextualHint = TEXT(""));

    UFUNCTION(BlueprintPure, Category = "OH|Targeting")
    static AActor* FindFuzzyTarget(const AActor* SelfActor, const FString& ContextualHint = TEXT(""));

    // Predictive, capsule-based foot ground trace with debug
    static bool PredictiveCapsuleGroundTrace(USkeletalMeshComponent* Mesh, FName BoneName, float MaxDistance,
                                             float TraceForwardBias, float CapsuleRadius, float CapsuleHalfHeight,
                                             const FVector& OwnerVelocity, FHitResult& OutHit, bool bDrawDebug = false);

    /**
     * Calculates the ratio of Attacker mass to Target mass for physics force scaling.
     * Returns a safe, clamped positive value. Falls back to 1.0 if mass is invalid or actors missing.
     *
     * @param Attacker   The actor applying force (can be nullptr).
     * @param Target     The actor receiving force (can be nullptr).
     * @param bVerbose   Enable logging for debugging.
     */
    UFUNCTION(BlueprintPure, Category = "Combat|Physics")
    static float CalculateMassRatio(AActor* Attacker, AActor* Target, bool bVerbose = false);

    /**
     * Returns the sum (or optionally average) mass of all specified bones for the given actor.
     * Skips missing or non-simulated bones. Returns 0.0 if no valid bones found.
     *
     * @param Actor           The actor whose skeletal mesh to query.
     * @param BoneNames       Array of bone names to aggregate mass from.
     * @param bAverage        If true, returns the average mass instead of the total.
     * @param bVerbose        If true, logs missing bones and other info.
     */
    UFUNCTION(BlueprintPure, Category = "Combat|Physics")
    static float CalculateChainMass(AActor* Actor, const TArray<FName>& BoneNames, bool bAverage = false,
                                    bool bVerbose = false);
};
