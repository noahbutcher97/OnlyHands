
#pragma once

#include "CoreMinimal.h"
#include "OHPrimaryStructs.h"
#include "Components/ActorComponent.h"
#include "Utilities/OHSafeMapUtils.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "OHMovementComponent.generated.h"

USTRUCT(BlueprintType)
struct FOHMovementConfig {
    GENERATED_BODY()

    // ==== USER-FACING PROPERTIES (UPROPERTY) ====
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    bool bUseMomentumBasedMovement = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    bool bUseInputMagnitudeForSpeed = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Debug")
    bool bEnableDebugOutput = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    UCurveVector* MovementCurve = nullptr; // X = input, Y/Z/W = walk/run/sprint

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    UCurveFloat* DirectionalSpeedCurve = nullptr; // For strafing/backpedal

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    UCurveFloat* RotationalSpeedCurve = nullptr; // For turn-in-place or pivots

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    float SpeedFactor = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float FrictionControl = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    float GaitFlexibility = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    float InputResponsiveness = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Input")
    float JoystickSensitivity = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    float OverallMaxSpeed = 600.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float MovementWeight = 0.9f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float InertiaFactor = 0.5f;

    // ---- SUGGESTED NEW USER FIELDS ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float CharacterMass = 80.f; // (kg) Used for force/accel calcs, defaults to human mass

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Curves")
    UCurveFloat* AccelerationCurve = nullptr; // (Optional: modulate accel based on input/gait)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Curves")
    UCurveFloat* BrakingCurve = nullptr; // (Optional: for designer braking profile)

    // Rotation Parameters

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    bool bUseRotationBlendedMovementInput = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float RotationBlendedMovementWeight = 0.3f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float DeadzoneAngle = 10.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float MaxOffsetAngle = 22.5f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float PullStrength = 9.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float HardSnapStrength = 20.0f;

    // ======== Accessors ========
    FORCEINLINE float GetMaxWalkSpeed() const {
        return MaxWalkSpeed;
    }
    FORCEINLINE float GetMaxRunSpeed() const {
        return MaxRunSpeed;
    }
    FORCEINLINE float GetMaxSprintSpeed() const {
        return MaxSprintSpeed;
    }
    FORCEINLINE float GetWalkSpeedThreshold() const {
        return WalkSpeedThreshold;
    }
    FORCEINLINE float GetRunSpeedThreshold() const {
        return RunSpeedThreshold;
    }
    FORCEINLINE float GetSprintSpeedThreshold() const {
        return SprintSpeedThreshold;
    }
    FORCEINLINE float GetMaxAccelerationWalking() const {
        return MaxAccelerationWalking;
    }
    FORCEINLINE float GetMaxAccelerationRunning() const {
        return MaxAccelerationRunning;
    }
    FORCEINLINE float GetMaxAccelerationSprinting() const {
        return MaxAccelerationSprinting;
    }
    FORCEINLINE float GetBrakingDecelerationWalking() const {
        return BrakingDecelerationWalking;
    }
    FORCEINLINE float GetBrakingDecelerationRunning() const {
        return BrakingDecelerationRunning;
    }
    FORCEINLINE float GetBrakingDecelerationSprinting() const {
        return BrakingDecelerationSprinting;
    }
    FORCEINLINE float GetGroundFrictionWalking() const {
        return GroundFrictionWalking;
    }
    FORCEINLINE float GetGroundFrictionRunning() const {
        return GroundFrictionRunning;
    }
    FORCEINLINE float GetGroundFrictionSprinting() const {
        return GroundFrictionSprinting;
    }
    FORCEINLINE float GetMaxFriction() const {
        return MaxFriction;
    }
    FORCEINLINE float GetMinFriction() const {
        return MinFriction;
    }
    FORCEINLINE float GetNoInputSpeedDecayRate() const {
        return NoInputSpeedDecayRate;
    }
    FORCEINLINE float GetInputSpeedBuildRate() const {
        return InputSpeedBuildRate;
    }
    FORCEINLINE float GetSpeedRetentionOnDirectionChange() const {
        return SpeedRetentionOnDirectionChange;
    }
    FORCEINLINE float GetStrafingThreshold() const {
        return StrafingThreshold;
    }
    FORCEINLINE float GetDirectionChangeSharpness() const {
        return DirectionChangeSharpness;
    }
    FORCEINLINE float GetPivotThreshold() const {
        return PivotThreshold;
    }
    FORCEINLINE float GetGaitSpeedTransitionRate() const {
        return GaitSpeedTransitionRate;
    }
    FORCEINLINE float GetGaitAccelerationMultiplier() const {
        return GaitAccelerationMultiplier;
    }
    FORCEINLINE float GetGaitDecelerationMultiplier() const {
        return GaitDecelerationMultiplier;
    }
    FORCEINLINE float GetRunHysteresis() const {
        return RunHysteresis;
    }
    FORCEINLINE float GetSprintHysteresis() const {
        return SprintHysteresis;
    }
    FORCEINLINE float GetInputBufferRiseRate() const {
        return InputBufferRiseRate;
    }
    FORCEINLINE float GetInputBufferDecay() const {
        return InputBufferDecay;
    }
    FORCEINLINE float GetHoldTimeAboveRunThreshold() const {
        return HoldTimeAboveRunThreshold;
    }
    FORCEINLINE float GetHoldTimeAboveSprintThreshold() const {
        return HoldTimeAboveSprintThreshold;
    }
    FORCEINLINE float GetGaitInterpSpeed() const {
        return GaitInterpSpeed;
    }
    FORCEINLINE float GetBackpedalSpeedMultiplier() const {
        return BackpedalSpeedMultiplier;
    }
    FORCEINLINE float GetStrafeSpeedMultiplier() const {
        return StrafeSpeedMultiplier;
    }
    FORCEINLINE float GetDiagonalSpeedMultiplier() const {
        return DiagonalSpeedMultiplier;
    }
    FORCEINLINE float GetJoystickDeadzone() const {
        return JoystickDeadzone;
    }
    FORCEINLINE float GetJoystickWalkZone() const {
        return JoystickWalkZone;
    }
    FORCEINLINE float GetJoystickRunZone() const {
        return JoystickRunZone;
    }
    FORCEINLINE float GetInputSmoothingBlendSpeed() const {
        return InputSmoothingBlendSpeed;
    }
    FORCEINLINE float GetInputIntentMagnitudeThreshold() const {
        return InputIntentMagnitudeThreshold;
    }
    FORCEINLINE float GetMomentumBasedMovement() const {
        return bUseMomentumBasedMovement;
    }
    FORCEINLINE float GetMomentumBasedMovementWeight() const {
        return MovementWeight;
    }
    // ==== PRIVATE DERIVED FIELDS ====
  private:
    float MaxWalkSpeed = 0.f;
    float MaxRunSpeed = 0.f;
    float MaxSprintSpeed = 0.f;

    float WalkSpeedThreshold = 0.f;
    float RunSpeedThreshold = 0.f;
    float SprintSpeedThreshold = 0.f;

    float MaxAccelerationWalking = 0.f;
    float MaxAccelerationRunning = 0.f;
    float MaxAccelerationSprinting = 0.f;

    float BrakingDecelerationWalking = 0.f;
    float BrakingDecelerationRunning = 0.f;
    float BrakingDecelerationSprinting = 0.f;

    float GroundFrictionWalking = 0.f;
    float GroundFrictionRunning = 0.f;
    float GroundFrictionSprinting = 0.f;

    float MaxFriction = 0.f;
    float MinFriction = 0.f;

    float NoInputSpeedDecayRate = 0.f;
    float InputSpeedBuildRate = 0.f;

    float SpeedRetentionOnDirectionChange = 0.f;
    float StrafingThreshold = 0.f;
    float DirectionChangeSharpness = 0.f;
    float PivotThreshold = 0.f;

    float GaitSpeedTransitionRate = 0.f;
    float GaitAccelerationMultiplier = 0.f;
    float GaitDecelerationMultiplier = 0.f;
    float RunHysteresis = 0.f;
    float SprintHysteresis = 0.f;

    float InputBufferRiseRate = 0.f;
    float InputBufferDecay = 0.f;
    float HoldTimeAboveRunThreshold = 0.f;
    float HoldTimeAboveSprintThreshold = 0.f;

    float GaitInterpSpeed = 0.f;
    float BackpedalSpeedMultiplier = 0.f;
    float StrafeSpeedMultiplier = 0.f;
    float DiagonalSpeedMultiplier = 0.f;

    float JoystickDeadzone = 0.f;
    float JoystickWalkZone = 0.f;
    float JoystickRunZone = 0.f;

    float InputSmoothingBlendSpeed = 0.f;
    float InputIntentMagnitudeThreshold = 0.f;

  public:
    // --------- Update function -----------
    void Update(float DeltaTime = 1.f / 60.f);
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGaitChanged, EOHGait, NewGait);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMovementInputUpdated, FVector2D, MovementInput);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPhysicsInputReceived, FVector2D, PhysicsInput, FVector2D,
                                             EffectiveMovement);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPushbackReceived, const FVector2D&, PushbackVector, float, Magnitude);

/**
 * Custom movement component for character control in a game.
 */
UCLASS(Blueprintable, BlueprintType, ClassGroup = (State), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHMovementComponent : public UActorComponent {
    GENERATED_BODY()

  public:
    UOHMovementComponent();

    /** Direct reference to the movement configuration */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Config")
    FOHMovementConfig MovementConfig;

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Config")
    bool bVerboseLogging = true;

    // References
    UPROPERTY(BlueprintReadWrite, Category = "OH|Target")
    mutable AActor* TargetActor = nullptr;

    UFUNCTION(BlueprintCallable, Category = "OH|Target")
    void FindTargetActor();
    ACharacter* FindTargetCharacter();

    UFUNCTION(BlueprintCallable, Category = "OH|Target")
    void SetTargetActor(AActor* NewTarget);

    // === Runtime State ===
    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentMomentumSpeed = 0.f; // Current momentum-based speed

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float DirectionalSpeedMultiplier = 1.0f; // Current direction-based speed modifier

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float RotationalSpeedMultiplier = 1.0f; // Current rotation-based speed modifier

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentMovementAngle = 0.f; // Angle between movement and facing direction

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    FVector LastMovementDirection = FVector::ZeroVector; // For direction change detection

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentAngularVelocity = 0.f; // Current turning speed

    UPROPERTY(BlueprintReadWrite, Category = "OH|Initialization")
    bool bMovementInitialized = false; // Track whether movement has been initialized

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentTargetSpeed = 0.f; // Target speed based on gait

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float SmoothedTargetSpeed = 0.f; // Smoothly interpolated target speed

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    EOHGait PreviousGait = EOHGait::Walking; // Track gait changes

    // --- Movement Input ---
    // Apply movement input to CharacterMovementComponent
    UFUNCTION()
    void ApplyMovementInput();

    // Default version: always uses CalculateFacingRotation
    UFUNCTION()
    void ApplyRotation(float DeltaTime, bool bBlend = true, float BlendSpeed = 10.f);

    // Overload: direct application
    void ApplyRotation(const FRotator& TargetRotation, bool bBlend, float BlendSpeed, float DeltaTime);

    // Calculate movement input vector from input magnitude and direction
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FVector CalculateMovementInputVector() const;

    // --- Movement State ---
    // Calculate control rotation based on Camera Perspective
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FRotator CalculateControlRotation() const;

    // Calculate control rotation based on Camera Perspective
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FRotator CalculateFacingRotation(float DeltaTime) const;

    UFUNCTION(BlueprintPure, Category = "Movement|Collision")
    FVector2D GetEffectiveMovementInput() const;

    UFUNCTION(BlueprintCallable, Category = "Movement|Input")
    void ApplyPhysicsImpulse(const FVector& WorldImpulse, bool bIsCombatHit);

    // --- Movement Settings ---
    // Calculate movement settings based on current gait
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void CalculateDirectionalSpeedModifiers(ACharacter* OwnerChar);

    // Update the HandlePACPushback declaration to ensure it's properly exposed
    UFUNCTION(BlueprintCallable, Category = "Movement|Combat")
    void HandlePACPushback(FName BoneName, const FVector2D& PushbackVector, float ImpactForce);
    FVector2D ConvertImpulseToInput(const FVector& WorldImpulse) const;

    // Movement input vector from external input sources (clamped 0-1)
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Input")
    FVector2D MovementInputVector;

    // Buffered smoothed input magnitude and speed for intent detection
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Buffering")
    float BufferedInputAmount;

    // Buffered smoothed speed for intent detection
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Buffering")
    float BufferedSpeed;

    // Current interpolated gait state and blend alpha for smooth transitions
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Gait")
    EOHGait CurrentGait;

    // Current interpolated gait state and blend alpha for smooth transitions
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Gait")
    float GaitBlendAlpha;

    // --- Movement State ---
    // Current movement speed (used to Calculate Gait)
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    float CurrentSpeed;

    // Movement play rate for animation
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    float MovementPlayRate;

    // Input intent magnitude from input panel (0 to 1), used in calculations
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Input")
    float InputIntentMagnitude;

    // Movement mode tracking from CharacterMovementComponent
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    TEnumAsByte<EMovementMode> CurrentMovementMode;

    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    uint8 CurrentCustomMovementMode;

    // Public setter to update input vector (to be called from player controller or input system)
    UFUNCTION(BlueprintCallable, Category = "OH|Movement|Input")
    void SetMovementInputVector(const FVector2D& NewInput);

    // Desired gait calculated from input and speed
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    EOHGait CalculateDesiredGait(float Deltatime);

    // Allowed gait from external constraints (e.g., stamina, crouching)
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    EOHGait GetAllowedGait() const;

    // Actual gait combining desired and allowed gait
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    static EOHGait GetActualGait(EOHGait DesiredGait, EOHGait AllowedGait);

    // Smooth interpolation of gait changes
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void InterpolateGait(EOHGait TargetGait, float DeltaTime);

    // Update movement state per tick
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateMovement(float DeltaTime);

    // Update input buffering values from input vector
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateInputBuffers(float DeltaTime);

    // Updates character movement component properties dynamically based on allowed gait
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateDynamicMovementSettings(EOHGait AllowedGait);

    // Returns normalized speed [0-1] for curve evaluation
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float GetMappedSpeed() const;

    // Calculates current horizontal movement speed ignoring vertical velocity
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementSpeed() const;

    // Calculates input magnitude (0-1) from input vector
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementInputAmount() const;

    // Calculates movement play rate (for animation speed scaling) based on speed and input
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementPlayRate(float Speed, float InputAmount) const;

    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    static bool ShouldRotateToTarget();

    UFUNCTION(BlueprintPure, Category = "OH|Movement|Direction")
    float GetInputWorldAngleDegrees() const;

    // Calculates a custom rotation looking from owner to locked target (implement in cpp)
    UFUNCTION(BlueprintPure, Category = "OH|Movement|Pivot")
    FRotator CalculateRotationToTarget_Internal() const;

    // Calculates the offset rotator between owner rotation and target rotation (implement in cpp)
    UFUNCTION(BlueprintPure, Category = "OH|Movement|Pivot")
    FRotator CalculateTargetOffset_Internal() const;

    // Movement mode changed handler, to be called when CharacterMovement changes movement mode
    UFUNCTION()
    void HandleMovementModeChanged(EMovementMode NewMovementMode, uint8 NewCustomMode);

    // Returns whether the character should perform a pivot based on the pivot threshold
    UFUNCTION(BlueprintCallable, Category = "OH|Movement|Pivot")
    bool ShouldPivot_Internal() const;

    UFUNCTION(BlueprintPure, Category = "OH|Movement|AI")
    bool GetIsPlayerControlled() const;
#pragma region Delegates
    // Input delegate handlers (bind these in your player controller)
    UFUNCTION()
    void OnReceiveMoveForward(float ForwardValue);

    UFUNCTION()
    void OnReceiveMoveRight(float RightValue);

    UFUNCTION()
    void OnReceiveMoveVector(float ForwardValue, float RightValue);

    // Delegate called when gait changes
    UPROPERTY(BlueprintAssignable, Category = "OH|Movement|Gait")
    FOnGaitChanged OnGaitChanged;

    // Delegate called when movement input updates
    UPROPERTY(BlueprintAssignable, Category = "OH|Movement|Input")
    FOnMovementInputUpdated OnMovementInputUpdated;

    UPROPERTY(BlueprintAssignable, Category = "Movement|Events")
    FOnPushbackReceived OnPushbackReceived;

#pragma endregion

  private:
    // Buffered timers to track intent hold durations
    float BufferedTimeAboveRunThreshold;
    float BufferedTimeAboveSprintThreshold;

    // Cached axis values
    float CurrentForwardInput;
    float CurrentRightInput;

    bool bIsPlayerControlled = false;

    // Helper to get locked target actor (implement this to suit your locking logic)
    static AActor* GetLockedTargetActor();

    // --- For Debug Input Reception ---
    FVector2D LastReceivedInputVector;
    float LastInputDebugTimestamp = 0.f;
    bool bLastInputIntentValid = false;
    float LastBufferedInputAmount = 0.f;
    float LastCalculatedSpeed = 0.f;

    // --- Stance Tracking ---
#pragma region StanceTracking
  public:
    // Configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    bool bEnableStanceTracking = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float StanceUpdateRate = 60.0f; // Updates per second

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float FootTraceDistance = 100.0f; // How far to trace for ground

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float GroundedThreshold = 10.0f; // Distance to consider foot grounded

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float PivotVelocityThreshold = 50.0f; // Below this velocity, foot is pivoting

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float PhaseTransitionSpeed = 5.0f; // How fast phases transition

    // Runtime data
    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHFootStanceData LeftFootData = FOHFootStanceData();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHFootStanceData RightFootData = FOHFootStanceData();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHStanceState StanceState = FOHStanceState();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    bool bLeftFootDominant = true;

    // Bone names
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftFootBoneName = "ball_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightFootBoneName = "ball_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName PelvisBoneName = "pelvis";

    // Additional bone names for kinematic chain
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftAnkleBoneName = "foot_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightAnkleBoneName = "foot_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftKneeBoneName = "calf_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightKneeBoneName = "calf_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftHipBoneName = "thigh_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightHipBoneName = "thigh_r";

    // Sliding detection thresholds
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Sliding")
    float FootSlideThreshold = 5.0f; // cm/s when planted

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Sliding")
    float MaxAllowedSlideDistance = 10.0f; // cm before correction

    // Clipping detection
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Clipping")
    float LegSeparationMin = 15.0f; // Minimum distance between feet

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Clipping")
    float KneeCollisionRadius = 8.0f; // Knee capsule radius

    // Environment detection
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Environment")
    float EdgeDetectionDistance = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Environment")
    float SurfaceTraceExtension = 30.0f; // Extra trace beyond foot

    // Debug
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Debug")
    bool bDebugStanceTracking = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Debug")
    float DebugDrawDuration = 0.0f; // 0 = single frame

    // Functions
    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateStanceTracking(float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateFootData(FOHFootStanceData& FootData, const FName& BoneName, bool bIsLeftFoot, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    EOHFootPhase CalculateFootPhase(const FOHFootStanceData& FootData, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateStanceState(float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void CalculateWeightDistribution();

    UFUNCTION(BlueprintPure, Category = "Stance")
    bool IsFootForward(const FOHFootStanceData& FootData) const;

    UFUNCTION(BlueprintPure, Category = "Stance")
    float GetStrideCadence() const {
        return StanceState.StrideCadence;
    }

    UFUNCTION(BlueprintPure, Category = "Stance")
    EOHStanceType GetCurrentStance() const {
        return StanceState.CurrentStance;
    }

    UFUNCTION(BlueprintPure, Category = "Stance")
    bool IsStancePivoting() const {
        return StanceState.bIsPivoting;
    }

  private:
    // Internal helpers
    FVector GetBoneWorldLocation(const FName& BoneName) const;
    FVector GetBoneVelocity(const FName& BoneName) const;
    float TraceToGround(const FVector& StartPos) const;
    float TraceToGround(const FVector& StartPos, FHitResult& OutHit) const;
    void UpdateKinematicChain(FOHKinematicChainData& ChainData, bool bIsLeftFoot, USkeletalMeshComponent* Mesh);
    void UpdateSlidingDetection(FOHFootStanceData& FootData, float DeltaTime);
    void UpdateClippingDetection(FOHFootStanceData& FootData, FOHFootStanceData& OtherFoot);
    bool DetectNearbyEdge(const FVector& FootPos, const FVector& SurfaceNormal);
    EOHFootPhase CalculatePhaseFromKinematics(const FOHFootStanceData& FootData, float DeltaTime);
    static void CalculateWeightBearingFromCompression(FOHFootStanceData& FootData, float DeltaTime);
    void DrawDebug_MovementData(float DeltaTime);
    void PrintDebug_MovementInput();
    void PrintDebug_MovementData();
    void DrawDebug_StanceData() const;
    void DrawDebug_KinematicChainData(const FOHFootStanceData& FootData, bool bIsLeft,
                                      USkeletalMeshComponent* Mesh) const;
    void DrawDebug_SlidingIndicator(const FOHFootStanceData& FootData, const FColor& FootColor) const;
    void DrawDebug_ClippingWarning() const;
    void DrawDebug_CompressionBars() const;
    void DrawDebug_BalanceAndWeightDistribution() const;
    void DrawDebug_EnvironmentInfo(const FOHFootStanceData& FootData) const;
    void DrawDebug_FootPhases() const;
    void PrintDebug_StanceData() const;
    static FColor GetPhaseColor(EOHFootPhase Phase);
    bool CanPerformKick(bool bLeftFoot) const;
    float GetKickPower(bool bLeftFoot) const;
    bool ValidateKinematicSetup() const;

    // Timing
    float StanceUpdateAccumulator = 0.0f;

    // History tracking
    float LastLeftStepTime = 0.0f;
    float LastRightStepTime = 0.0f;
    TArray<float> RecentStepIntervals;

    // Bone location cache
    mutable TMap<FName, FVector> BoneLocationCache;
    mutable TMap<FName, FVector> LastBoneLocations;

    OHSafeMapUtils::TRollingBuffer<FOHMovementHistoryFrame, 16> MovementHistory;
#pragma endregion

#pragma region COLLISION_INTEGRATION

  public:
    // Configuration

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    bool bUseImpulseBasedPhysics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    bool bForceImpulseBasedPhysics = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float PhysicsImpulseStrength = 500.0f; // World units per second

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float PhysicsImpulseDecayRate = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    UCurveFloat* PhysicsImpulseDecayCurve = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    bool bAcceptPhysicsInput = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float PhysicsInputWeight = 0.3f; // How much physics affects movement vs player input

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float PhysicsInputDecayRate = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float MaxPhysicsInputMagnitude = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    bool bEnableProximitySpacing = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    float PersonalSpaceRadius = 80.0f; // Start pushing at this distance

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    float ProximityPushStrength = 0.5f; // How strong the push is

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing",
              meta = (ClampMin = "50.0", ClampMax = "300.0"))
    float BaseSpacingDistance = 120.0f; // Optimal fighting range

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    UCurveFloat* SpacingCurve = nullptr; // Distance to push strength mapping

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    bool bDebugProximitySystem = false;

    // Runtime State
    UPROPERTY(BlueprintReadOnly, Category = "Movement|Collision")
    FVector2D PhysicsInputVector = FVector2D::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Movement|Collision")
    FVector2D LastReceivedPhysicsInput = FVector2D::ZeroVector;

    UPROPERTY()
    FVector AccumulatedPhysicsImpulse;

  private:
    /**
     * Updates the physics input based on delta time.
     *
     * This method handles the old physics input system, which is a 2D vector that blends with player input.
     * It applies decay to the physics input vector and handles friction-based decay when grounded.
     * Small values are cleared from the physics input vector. Additionally, it can convert remaining physics input to
     * impulse for unified handling if bUseImpulseBasedPhysics is enabled.
     *
     * @param DeltaTime The time elapsed since the last frame in seconds.
     */
    void UpdatePhysicsInput(float DeltaTime);

    // Spacing system internals
    UPROPERTY(Transient)
    TWeakObjectPtr<ACharacter> CurrentOpponent;

    float ProximityUpdateTimer = 0.0f;

    const float ProximityUpdateInterval = 0.1f; // Update spacing every 100ms
    // Helper functions

    void CalculateSmartSpacing(ACharacter* Opponent, float& OutPushX, float& OutPushY);

    void UpdateProximitySpacing(float DeltaTime);

    void ProcessPhysicsImpulses(float DeltaTime);

    // Visualization
    TArray<FVector> PhysicsImpulseHistory;
    const int32 MaxImpulseHistorySize = 10;

  public:
#pragma endregion

    // Combat awareness
    FVector LastCombatPushback;
    float LastCombatPushbackTime = 0.0f;
    bool bCombatPushbackActive = false;

    // Add initialization function
    UFUNCTION(BlueprintCallable, Category = "Movement|Setup")
    void EnsureInitialized();

  protected:
    // Single new variable for stance state (should already exist)
    FOHStanceState CurrentStanceState = FOHStanceState();

  private:
    bool IsCharacterInCombat(ACharacter* Character, float& OutAttackConfidence, float& OutChainSpeed);

    // Combat state tracking
    float LastCombatStateTime = 0.0f;
    bool bRecentlyHit = false;
    float HitRecoveryTime = 0.0f;
};