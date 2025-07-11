// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "OHPhysicsStructs.h"
#include "Kismet/BlueprintFunctionLibrary.h"
// #include "FunctionLibrary/OHGraphUtils.cpp"
#include "OHLocomotionUtils.generated.h"

UENUM(BlueprintType)
enum class EScreenDirection4 : uint8 {
    Forward UMETA(DisplayName = "Forward"),
    Right UMETA(DisplayName = "Right"),
    Backward UMETA(DisplayName = "Backward"),
    Left UMETA(DisplayName = "Left")
};

UENUM(BlueprintType)
enum class EScreenDirection8 : uint8 {
    Forward UMETA(DisplayName = "Forward"),
    ForwardRight UMETA(DisplayName = "ForwardRight"),
    Right UMETA(DisplayName = "Right"),
    BackwardRight UMETA(DisplayName = "BackwardRight"),
    Backward UMETA(DisplayName = "Backward"),
    BackwardLeft UMETA(DisplayName = "BackwardLeft"),
    Left UMETA(DisplayName = "Left"),
    ForwardLeft UMETA(DisplayName = "ForwardLeft")
};
UENUM(BlueprintType)
enum class EScreenDirectionQuadrant : uint8 {
    ForwardRight UMETA(DisplayName = "ForwardRight"),
    BackwardRight UMETA(DisplayName = "BackwardRight"),
    BackwardLeft UMETA(DisplayName = "BackwardLeft"),
    ForwardLeft UMETA(DisplayName = "ForwardLeft")
};

USTRUCT(BlueprintType)
struct FScreenRelativeDirectionData {
    GENERATED_BODY()

    // Angle in degrees, [-180, 180], suitable for symmetric blendspaces
    UPROPERTY(BlueprintReadOnly)
    float Angle = 0.0f;

    // Wrapped angle in degrees, [0, 360), useful for enum sector logic
    UPROPERTY(BlueprintReadOnly)
    float WrappedAngle = 0.0f;

    // Directional blend values: X = Right/Left, Y = Forward/Backward
    UPROPERTY(BlueprintReadOnly)
    FVector2D BlendFactors = FVector2D::ZeroVector;

    // Discrete 8-way directional classification
    UPROPERTY(BlueprintReadOnly)
    EScreenDirection8 Direction = EScreenDirection8::Forward;

    // Discrete 4-way classification
    UPROPERTY(BlueprintReadOnly)
    EScreenDirection4 Direction4 = EScreenDirection4::Forward;

    // Quadrant grouping (e.g. ForwardLeft)
    UPROPERTY(BlueprintReadOnly)
    EScreenDirectionQuadrant Quadrant = EScreenDirectionQuadrant::ForwardRight;

    // Clamped directional X axis from -1.0 (left) to +1.0 (right)
    UPROPERTY(BlueprintReadOnly)
    float ClampedStrafeDirection = 0.0f;

    static FORCEINLINE FScreenRelativeDirectionData Default() {
        return {0.0f,
                0.0f,
                FVector2D::ZeroVector,
                EScreenDirection8::Forward,
                EScreenDirection4::Forward,
                EScreenDirectionQuadrant::ForwardRight,
                0.0f};
    }

    FORCEINLINE static bool ShouldMirrorFromBlendFactors(const FVector2D& BlendFactors) {
        return BlendFactors.X < 0.f;
    }

    FORCEINLINE static bool ShouldMirrorFromDirection(const FScreenRelativeDirectionData& Dir) {
        return Dir.ClampedStrafeDirection < 0.f;
    }

    FORCEINLINE static bool IsStrafingLeft(const FScreenRelativeDirectionData& Dir) {
        return Dir.ClampedStrafeDirection < 0.f;
    }

    FORCEINLINE static bool IsStrafingRight(const FScreenRelativeDirectionData& Dir) {
        return Dir.ClampedStrafeDirection > 0.f;
    }

    FORCEINLINE static bool IsStrafing(const FScreenRelativeDirectionData& Dir, float Threshold = 0.1f) {
        return FMath::Abs(Dir.ClampedStrafeDirection) > Threshold;
    }
};

USTRUCT(BlueprintType)
struct FPredictedMotionDirectionData {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    float SmoothedAngle = 0.f;

    UPROPERTY(BlueprintReadOnly)
    FVector2D BlendFactors = FVector2D::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    EScreenDirection8 Direction8 = EScreenDirection8::Forward;

    UPROPERTY(BlueprintReadOnly)
    EScreenDirection4 Direction4 = EScreenDirection4::Forward;

    UPROPERTY(BlueprintReadOnly)
    EScreenDirectionQuadrant Quadrant = EScreenDirectionQuadrant::ForwardRight;

    UPROPERTY(BlueprintReadOnly)
    bool bMirror = false;
};

UCLASS()
class ONLYHANDS_API UOHLocomotionUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

#pragma region StructuredDirectionOutput

#pragma region Screen-Relative Directions
    // ----- Screen-Relative -----

    UFUNCTION(BlueprintCallable, Category = "OH|Locomotion|Movement")
    static void ApplyStableCameraRelativeMovement(ACharacter* Character, const FVector2D& Input);

    UFUNCTION(BlueprintCallable, Category = "Input")
    static void ApplyJoystickMovementRelativeToActor(ACharacter* Character, const FVector2D& Input,
                                                     const FRotator& ActorRotation, float Deadzone = 0.05f);

    UFUNCTION(BlueprintPure)
    void GetLockOnMovementBasis(ACharacter* Character, AActor* LockOnTarget, FVector& ForwardVector,
                                FVector& RightVector);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static float ComputeSmoothedQuadrantMovementAngle(const FVector& Velocity, const FRotator& FacingRotation,
                                                      float PreviousAngle, float DeltaTime,
                                                      float PredictionWeight = 0.6f, float BiasStrength = 0.7f);

    /**
     * Computes the predictive smoothed direction angle and associated animation data.
     * Internally caches the last angle per character and automatically detects physics manager or velocity fallback.
     * Requires no setup. Ideal for plug-and-play use in AnimBP.
     */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FPredictedMotionDirectionData ComputeSmartAnimationDirectionAuto(ACharacter* Character, float DeltaTime,
                                                                            float StrafeBias = 0.0f,
                                                                            float PredictionTime = 0.2f,
                                                                            float SmoothingStrength = 0.7f,
                                                                            float PelvisTranslationThreshold = 1.25f);

    UFUNCTION(BlueprintCallable, Category = "Input")
    static void ApplyJoystickMovementWithBasis(APawn* Pawn, const FVector2D& Input, const FRotator& BasisRotation,
                                               FVector UpVector = FVector::UpVector, float Deadzone = 0.05f);

    UFUNCTION(BlueprintCallable, Category = "Input")
    static void ApplyAutoCameraRelativeJoystickMovement(ACharacter* Character, const FVector2D& Input, float Deadzone);

    UFUNCTION(BlueprintCallable, Category = "Input")
    static void ApplySmartCameraRelativeJoystickMovement(ACharacter* Character, const FVector2D& Input,
                                                         bool bFaceMovementDirection = false,
                                                         bool bConstrainToXYPlane = true, float Deadzone = 0.05f);
    UFUNCTION(BlueprintCallable, Category = "Input")
    static void
    ApplyFullCameraRelativeJoystickMovement(ACharacter* Character, const FVector2D& Input,
                                            bool bFaceMovementDirection = false, bool bConstrainToXYPlane = true,
                                            float Deadzone = 0.05f, float TurnSpeed = 10.f,
                                            int32 SnapToCardinalDirections = 0 // 0 = off, 4 = 4-way, 8 = 8-way
    );

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeDirectionFromInput(const FVector2D& InputVector,
                                                                  const APlayerController* PlayerController,
                                                                  bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeDirectionFromVelocity(const APlayerController* PlayerController,
                                                                     const ACharacter* Character,
                                                                     bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeDirectionToTarget(const AActor* TargetActor,
                                                                 const APlayerController* PlayerController,
                                                                 const ACharacter* Character, bool bNormalize = true);

#pragma endregion

#pragma region Player-Relative Directions
    // ----- Player-Relative -----

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputePlayerRelativeDirectionFromInput(const FVector2D& InputVector,
                                                                                const ACharacter* Character,
                                                                                bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputePlayerRelativeDirectionFromVelocity(const ACharacter* Character,
                                                                                   bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputePlayerRelativeDirectionToTarget(const AActor* TargetActor,
                                                                               const ACharacter* Character,
                                                                               bool bNormalize = true);

#pragma endregion

    // ----- World-Relative -----
#pragma region World-Relative Directions

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeWorldRelativeDirectionFromInput(const FVector2D& InputVector,
                                                                               bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeWorldRelativeDirectionFromVelocity(const ACharacter* Character,
                                                                                  bool bNormalize = true);

    UFUNCTION(BlueprintPure)
    static FScreenRelativeDirectionData ComputeWorldRelativeDirectionToTarget(const AActor* TargetActor,
                                                                              const ACharacter* Character,
                                                                              bool bNormalize = true);
#pragma endregion

#pragma region Actor-Relative Directions
    // ===== Actor-Relative Locomotion =====

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData ComputeActorRelativeDirectionFromInput(const FVector2D& InputVector,
                                                                               const AActor* ReferenceActor,
                                                                               bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData ComputeActorRelativeDirectionFromVelocity(const ACharacter* SourceCharacter,
                                                                                  const AActor* ReferenceActor,
                                                                                  bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData ComputeActorRelativeDirectionToTarget(const AActor* TargetActor,
                                                                              const AActor* ReferenceActor,
                                                                              bool bNormalize = true);

#pragma endregion

#pragma region Component-Relative Directions
    // ===== Component-Relative Locomotion =====

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData
    ComputeComponentRelativeDirectionFromInput(const FVector2D& InputVector, const USceneComponent* ReferenceComponent,
                                               bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData
    ComputeComponentRelativeDirectionFromVelocity(const ACharacter* SourceCharacter,
                                                  const USceneComponent* ReferenceComponent, bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Direction")
    static FScreenRelativeDirectionData
    ComputeComponentRelativeDirectionToTarget(const AActor* TargetActor, const USceneComponent* ReferenceComponent,
                                              bool bNormalize = true);
#pragma endregion

#pragma endregion

#pragma region ComputeRaw

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float ComputeScreenRelativeDirection(const FVector2D& InputVector, const APlayerController* PlayerController,
                                                const ACharacter* Character, bool bClampToCardinal = false);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Helpers")
    static float ComputeScreenRelativeAngle(const FVector2D& FromVector, const FVector2D& ToVector,
                                            bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float PredictMovementAngle(ACharacter* Character, const FRotator& FacingRotation, float DeltaTime,
                                      float& InOutPrevAngle, float BasePredictionTime, float SpeedPredictionScale,
                                      float BaseSmoothingAlpha, float SpeedSmoothScale,
                                      float PelvisTranslationThreshold, float FootLockSpeedThreshold,
                                      float FootLockAccelThreshold, float PivotScale, float PivotDeadzone,
                                      float StrafeBias, float JitterThreshold, float MaxAngleJumpPerFrame,
                                      float JerkTriggerScale, float MaxTrajectoryBlend);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector2D GetScreenRelativeVector(const FVector2D& InputVector, const APlayerController* PlayerController);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static EScreenDirection4 GetScreenRelativeCardinalDirectionToTarget(const AActor* TargetActor,
                                                                        const APlayerController* PlayerController,
                                                                        const ACharacter* Character);

    /** Returns the screen-relative angle [-180, 180] from the character to the target actor */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float GetScreenRelativeAngleToTarget(const AActor* TargetActor, const APlayerController* PlayerController,
                                                const ACharacter* Character);

    /** Returns the 8-way screen-relative direction from the character to the target actor */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static EScreenDirection8 GetScreenRelative8WayDirectionToTarget(const AActor* TargetActor,
                                                                    const APlayerController* PlayerController,
                                                                    const ACharacter* Character);

    /** Returns blend factors (X: Right, Y: Forward) for screen-relative direction to the target actor */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector2D GetScreenRelativeBlendFactorsToTarget(const AActor* TargetActor,
                                                           const APlayerController* PlayerController,
                                                           const ACharacter* Character);

    /** Returns the screen-relative angle [-180, 180] based on the character's velocity */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float GetScreenRelativeAngleFromVelocity(const APlayerController* PlayerController,
                                                    const ACharacter* Character);

    /** Returns the 8-way screen-relative direction based on the character's velocity */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static EScreenDirection8 GetScreenRelative8WayDirectionFromVelocity(const APlayerController* PlayerController,
                                                                        const ACharacter* Character);

    /** Returns blend factors (X: Right, Y: Forward) for screen-relative direction based on velocity */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector2D GetScreenRelativeBlendFactorsFromVelocity(const APlayerController* PlayerController,
                                                               const ACharacter* Character);

    /** Returns the screen-relative angle [-180, 180] based on input vector */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float GetScreenRelativeAngleFromInput(const FVector2D& InputVector,
                                                 const APlayerController* PlayerController);

    /** Returns the 8-way screen-relative direction based on input vector */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static EScreenDirection8 GetScreenRelative8WayDirectionFromInput(const FVector2D& InputVector,
                                                                     const APlayerController* PlayerController);

    /** Returns blend factors (X: Right, Y: Forward) for screen-relative direction based on input vector */
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector2D GetScreenRelativeBlendFactorsFromInput(const FVector2D& InputVector,
                                                            const APlayerController* PlayerController);

    ///////////////////////////////////////////////
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Output")
    static FVector GetWorldMovementVectorFromInput(const FVector2D& InputVector,
                                                   const APlayerController* PlayerController, bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Output")
    static FRotator GetFacingRotationFromInput(const FVector2D& InputVector, const APlayerController* PlayerController,
                                               bool bNormalize = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Output")
    static FVector GetWorldMovementVectorFromBlendFactors(const FVector2D& BlendFactors,
                                                          const APlayerController* PlayerController);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Output")
    static FRotator GetFacingRotationFromDirection(const FScreenRelativeDirectionData& DirectionData,
                                                   const APlayerController* PlayerController);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Output")
    static FVector GetStrafeMovementVectorFromBlendFactors(const FVector2D& BlendFactors,
                                                           const FRotator& FacingRotation);

    ///////////////////////////////////////////////
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    FVector GetCameraRelativeMovementVectorFromInput(const FVector2D& InputVector, const FRotator& CameraRotation,
                                                     bool bNormalize);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    FRotator GetCameraRelativeFacingRotationFromInput(const FVector2D& InputVector, const FRotator& CameraRotation,
                                                      bool bNormalize);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector GetActorRelativeMovementVectorFromInput(const FVector2D& InputVector, const AActor* ReferenceActor,
                                                           bool bNormalize);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FRotator GetActorRelativeFacingRotationFromInput(const FVector2D& InputVector, const AActor* ReferenceActor,
                                                            bool bNormalize);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FVector GetComponentRelativeMovementVectorFromInput(const FVector2D& InputVector,
                                                               const USceneComponent* ReferenceComponent,
                                                               bool bNormalize);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static FRotator GetComponentRelativeFacingRotationFromInput(const FVector2D& InputVector,
                                                                const USceneComponent* ReferenceComponent,
                                                                bool bNormalize);

#pragma endregion

#pragma region StructuredDirectionInput

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Animation")
    static FScreenRelativeDirectionData BuildScreenDirectionFromInput(const FVector2D& InputVector,
                                                                      const FRotator& CameraRotation,
                                                                      const AActor* OptionalActor,
                                                                      const APlayerController* OptionalController,
                                                                      bool bAllowMirroring = true, bool bClamp = false);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Animation")
    static FVector2D GetOptimalAnimationBlendVector(const FScreenRelativeDirectionData& DirectionData,
                                                    bool bNormalize = true, bool bClamp = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Animation")
    static FVector2D GetVisualConsistentAnimationBlendVector(const FScreenRelativeDirectionData& DirectionData,
                                                             const AActor* ReferenceActor,
                                                             const APlayerController* PlayerController,
                                                             bool bNormalize = true, bool bClamp = true,
                                                             bool bApplyMirroring = true);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Animation")
    static float GetVisualConsistentAnimationDirectionAngle(const FScreenRelativeDirectionData& DirectionData,
                                                            const AActor* ReferenceActor,
                                                            const APlayerController* PlayerController,
                                                            bool bApplyMirroring = true, bool bNormalize = false,
                                                            bool bClamp = false);

#pragma endregion

#pragma region Bone-Based Locomotion
    // ===== Bone-Based Locomotion =====

    // Predictive direction angle (for blendspace float input)
    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Physics")
    static float GetPredictedDirectionAngleFromBoneMotion(const FOHBoneData& BoneData, const AActor* ReferenceActor,
                                                          const APlayerController* PlayerController,
                                                          float PredictAheadTime = 0.2f, bool bApplyMirroring = true,
                                                          bool bNormalize = false, bool bClamp = false);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Physics")
    static FVector2D GetPredictedBlendVectorFromBoneMotion(const FOHBoneData& BoneData, const AActor* ReferenceActor,
                                                           const APlayerController* PlayerController,
                                                           float PredictAheadTime = 0.2f, bool bApplyMirroring = true,
                                                           bool bNormalize = true, bool bClamp = true);

#pragma endregion

#pragma region BPHelpers

    // === Blueprint-Accessible Helper Utilities ===

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Helpers")
    static EScreenDirection8 MapAngleTo8WayDirection(float Angle);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Helpers")
    static EScreenDirection4 MapAngleTo4WayDirection(float Angle);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Helpers")
    static EScreenDirectionQuadrant MapAngleToQuadrant(float Angle);

    UFUNCTION(BlueprintPure)
    static float PredictMovementAngleMonsterExtended(
        ACharacter* Character, const FRotator& FacingRotation, float DeltaTime, float& OutBezierT, float PredictionTime,
        float SpeedPredictionScale, float AccelPredictionScale, float JerkScale, float CurvatureScale, float MaxBlend,
        float PivotDeadzone, float PivotScale, float StrafeBias, float RotationThreshold, float MoveThreshold,
        float SGCurvThreshold, float FootPlantSpeedThresh, float FootPlantAccelThresh, float FootPlantHoldTime,
        float TeleportThreshold, float VarMin, float VarMax, float ProcNoise, float MeasNoise);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Helpers")
    static FVector2D ComputeBlendFactors(const FVector2D& Direction, const FRotator& CameraRotation,
                                         bool bNormalize = true);

#pragma endregion

#pragma region InlineHelpers
    // ===== Inline Utilities =====

    FORCEINLINE static FVector2D ComputeBlendFactors_Inline(const FVector2D& Direction, const FRotator& ReferenceRot,
                                                            bool bNormalize = true) {
        if (Direction.IsNearlyZero()) {
            return FVector2D::ZeroVector;
        }

        const FVector Forward3D = ReferenceRot.Vector();
        const FVector Right3D = FRotationMatrix(ReferenceRot).GetUnitAxis(EAxis::Y);

        FVector2D Forward2D(Forward3D.X, Forward3D.Y);
        FVector2D Right2D(Right3D.X, Right3D.Y);
        FVector2D Dir2D = Direction;

        if (bNormalize) {
            Dir2D.Normalize();
            Forward2D.Normalize();
            Right2D.Normalize();
        }

        const float ForwardBlend = FVector2D::DotProduct(Dir2D, Forward2D);
        const float RightBlend = FVector2D::DotProduct(Dir2D, Right2D);

        return FVector2D(RightBlend, ForwardBlend);
    }

    FORCEINLINE float static ComputeScreenRelativeAngle_Inline(const FVector2D& Forward,
                                                               const FVector2D& TargetDirection, bool bNormalize) {
        const FVector2D NormFwd = bNormalize ? Forward.GetSafeNormal() : Forward;
        const FVector2D NormTarget = bNormalize ? TargetDirection.GetSafeNormal() : TargetDirection;
        const float Rad =
            FMath::Atan2(FVector2D::CrossProduct(NormFwd, NormTarget), FVector2D::DotProduct(NormFwd, NormTarget));
        return FMath::RadiansToDegrees(Rad);
    }

    FORCEINLINE EScreenDirection8 static MapAngleTo8WayDirection_Inline(float Angle) {
        const float Normalized = FMath::Fmod(Angle + 360.f, 360.f);
        int32 Sector = FMath::RoundToInt(Normalized / 45.f) % 8;
        return static_cast<EScreenDirection8>(Sector);
    }

    FORCEINLINE EScreenDirection4 static MapAngleTo4WayDirection_Inline(float Angle) {
        const float Normalized = FMath::Fmod(Angle + 360.f, 360.f);
        int32 Sector = FMath::RoundToInt(Normalized / 90.f) % 4;
        return static_cast<EScreenDirection4>(Sector);
    }

    FORCEINLINE EScreenDirectionQuadrant static MapAngleToQuadrant_Inline(float Angle) {
        const float Normalized = FMath::Fmod(Angle + 360.f, 360.f);
        int32 Sector = FMath::RoundToInt(Normalized / 90.f) % 4;
        return static_cast<EScreenDirectionQuadrant>(Sector);
    }

#pragma endregion

    /**
     * Predicts a movement‐direction angle (in degrees) relative to FacingRotation,
     * entirely driven by the root‐bone’s motion sample history—no previous‐angle input.
     *
     * @param Character           The pawn whose root bone history we sample.
     * @param FacingRotation      The “camera” or character facing rotation.
     * @param DeltaTime           The frame’s Δ-time.
     * @param PredictionTime      How far ahead to predict (unused for history‐based).
     * @param SpeedPredictionScale Scale factor if you ever wanted to weight speed.
     * @param PivotDeadzone       Minimum curve strength before pivoting.
     * @param PivotScale          How strongly to pivot when curve > deadzone.
     * @param StrafeBias          −1=always strafe left, +1=always strafe right, 0=no bias.
     * @return                    Angle in [0,360).
     */

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float PredictMovementAngleFromHistory(ACharacter* Character, const FRotator& FacingRotation, float DeltaTime,
                                                 float PredictionTime = 0.2f, float SpeedPredictionScale = 0.004f,
                                                 float PivotDeadzone = 0.1f, float PivotScale = 20.f,
                                                 float StrafeBias = 0.f);

    /**
     * Same as above, but also draws a quadratic Bézier (P0→P1→P2) in‐world for debugging.
     *
     * @param Character   The pawn whose root bone we sample.
     * @param FacingRotation  The “camera” or character facing rotation.
     * @param DeltaTime   The frame’s Δ-time.
     * @param World       The world to draw in.
     * @param DrawDuration How long the debug lines persist.
     * @param CurveColor  Color for the debug curve.
     * @param NumSegments Number of line segments to sample the curve.
     * @param PredictionTime      How far ahead to predict via the curve.
     * @param SpeedPredictionScale How speed scales the middle control point.
     * @param PivotDeadzone       As above.
     * @param PivotScale          As above.
     * @param JerkThreshold       If the root’s jerk > this, draw thicker.
     * @param StrafeBias          As above.
     * @return             Angle in [0,360).
     */

    UFUNCTION(BlueprintCallable, Category = "OH|Locomotion")
    static float PredictMovementAngleWithBezierDebug(ACharacter* Character, const FRotator& FacingRotation,
                                                     float DeltaTime, UWorld* World, float DrawDuration,
                                                     FColor CurveColor = FColor::Green, int32 NumSegments = 16,
                                                     float PredictionTime = 0.2f, float SpeedPredictionScale = 0.004f,
                                                     float PivotDeadzone = 0.1f, float PivotScale = 20.f,
                                                     float JerkThreshold = 0.1f, float StrafeBias = 0.f);

    /**
     * MONSTER predictor implementing:
     *   • Rotation-only detection
     *   • Multi-bone centroid trajectory fit
     *   • 2nd-order polynomial least-squares extrapolation
     *   • Jerk & curvature–driven blend between raw/poly/Bezier estimates
     *   • Quadratic Bézier trajectory + optional debug draw
     *   • Strafe bias & pivot anticipation
     *   • Dynamic horizon (speed + accel scaled)
     *   • Savitzky–Golay smoothing (5-point) on the measured angle
     *   • Kalman-style filtering (process & measurement noise)
     *
     * @param Character            Actor with OHPhysicsManager & bone history
     * @param FacingRotation       Screen-space “forward” rotation
     * @param DeltaTime            Frame Δ-seconds
     * @param World                If non-null & DrawDuration>0, used for DrawDebugLine()
     * @param DrawDuration         Lifetime of debug Bézier lines (≤0 disables draw)
     * @param PredictionTime       Base look-ahead (s)
     * @param SpeedPredictionScale Extra sec per unit speed
     * @param AccelPredictionScale Horizon scale per unit accel
     * @param JerkScale            Scale mapping jerk→poly blend
     * @param CurvatureScale       Scale mapping curvature→Bezier blend
     * @param MaxBlend             Cap for any trajectory blend [0..1]
     * @param PivotDeadzone        Min cross-strength to apply pivot (0–1)
     * @param PivotScale           Degrees of pivot per cross-strength unit
     * @param StrafeBias           Lateral bias: –1=left, +1=right, 0=none
     * @param RotationThreshold    Pure-turn yaw threshold (deg)
     * @param MoveThreshold        Max position Δ to consider turn-in-place
     * @param ProcNoise            Kalman process noise Q
     * @param MeasNoise            Kalman measurement noise R
     * @return                     Final smoothed movement angle [0..360)
     */
    UFUNCTION(BlueprintCallable, Category = "OH|Locomotion")
    float PredictMovementAngleMonster(ACharacter* Character, const FRotator& FacingRotation, float DeltaTime,
                                      UWorld* World, float DrawDuration, float PredictionTime = 0.3f,
                                      float SpeedPredictionScale = 0.004f, float AccelPredictionScale = 0.1f,
                                      float JerkScale = 0.01f, float CurvatureScale = 0.1f, float MaxBlend = 1.0f,
                                      float PivotDeadzone = 0.1f, float PivotScale = 20.f, float StrafeBias = 0.f,
                                      float RotationThreshold = 30.f, float MoveThreshold = 5.f,
                                      float ProcNoise = 1e-3f, float MeasNoise = 1e-1f);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion")
    static float
    PredictMovementAngleMonsterNoDebug(ACharacter* Character, const FRotator& FacingRotation, float DeltaTime,
                                       float PredictionTime = 0.3f, float SpeedPredictionScale = 0.004f,
                                       float AccelPredictionScale = 0.1f, float JerkScale = 0.01f,
                                       float CurvatureScale = 0.1f, float MaxBlend = 1.0f, float PivotDeadzone = 0.1f,
                                       float PivotScale = 20.f, float StrafeBias = 0.f, float RotationThreshold = 30.f,
                                       float MoveThreshold = 5.f, float ProcNoise = 1e-3f, float MeasNoise = 1e-1f);
};
