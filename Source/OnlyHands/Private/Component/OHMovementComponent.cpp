#include "Component/OHMovementComponent.h"

#include "Component/OHPACManager.h"
#include "Curves/CurveVector.h"
#include "FunctionLibrary/OHCombatUtils.h"
#include "FunctionLibrary/OHLocomotionUtils.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Math/Vector2D.h"
#include "GameFramework/Character.h"
#include "Kismet/KismetMathLibrary.h"

void FOHMovementConfig::Update(float DeltaTime) {
    // --- SPEED & GAIT THRESHOLDS (Curve or User-Defined) ---
    if (MovementCurve) {
        FVector CurveVals = MovementCurve->GetVectorValue(1.f); // 1.0 = max input
        MaxWalkSpeed = CurveVals.X * SpeedFactor;
        MaxRunSpeed = CurveVals.Y * SpeedFactor;
        MaxSprintSpeed = CurveVals.Z * SpeedFactor;
    } else {
        MaxWalkSpeed = 150.f * SpeedFactor;
        MaxRunSpeed = 400.f * SpeedFactor;
        MaxSprintSpeed = OverallMaxSpeed;
    }

    // Clamp to max for safety
    MaxWalkSpeed = FMath::Min(MaxWalkSpeed, OverallMaxSpeed * 0.3f);
    MaxRunSpeed = FMath::Min(MaxRunSpeed, OverallMaxSpeed * 0.7f);
    MaxSprintSpeed = FMath::Min(MaxSprintSpeed, OverallMaxSpeed);

    // FIX: Set thresholds BELOW the max speeds so they're actually achievable!
    // Thresholds should be ~70-80% of the max speed for that gait
    WalkSpeedThreshold = MaxWalkSpeed * FMath::Lerp(0.5f, 0.7f, GaitFlexibility);      // 50-70% of max walk
    RunSpeedThreshold = MaxRunSpeed * FMath::Lerp(0.6f, 0.8f, GaitFlexibility);        // 60-80% of max run
    SprintSpeedThreshold = MaxSprintSpeed * FMath::Lerp(0.7f, 0.85f, GaitFlexibility); // 70-85% of max sprint

    // --- ACCELERATION (Physics-based: a = F/m or designer/curve) ---
    float AccelBase = 2000.f * InputResponsiveness;
    if (AccelerationCurve)
        AccelBase = AccelerationCurve->GetFloatValue(1.f) * InputResponsiveness;

    MaxAccelerationWalking = AccelBase * 0.8f; // Slightly slower accel for walking
    MaxAccelerationRunning = AccelBase * 1.2f;
    MaxAccelerationSprinting = AccelBase * 1.5f; // Faster accel for sprinting

    // --- BRAKING/DECELERATION ---
    if (BrakingCurve) {
        BrakingDecelerationWalking = BrakingCurve->GetFloatValue(MaxWalkSpeed);
        BrakingDecelerationRunning = BrakingCurve->GetFloatValue(MaxRunSpeed);
        BrakingDecelerationSprinting = BrakingCurve->GetFloatValue(MaxSprintSpeed);
    } else {
        // More aggressive braking for walking, less for sprinting (momentum)
        BrakingDecelerationWalking = 2048.f * FMath::Lerp(1.2f, 1.0f, MovementWeight);
        BrakingDecelerationRunning = 2048.f * FMath::Lerp(1.0f, 0.8f, MovementWeight);
        BrakingDecelerationSprinting = 2048.f * FMath::Lerp(0.8f, 0.6f, MovementWeight);
    }

    // --- FRICTION ---
    GroundFrictionWalking = FrictionControl * 8.0f;
    GroundFrictionRunning = GroundFrictionWalking * 0.8f;   // Less friction for running
    GroundFrictionSprinting = GroundFrictionWalking * 0.6f; // Even less for sprinting

    MaxFriction = FMath::Max(GroundFrictionWalking, FMath::Max(GroundFrictionRunning, GroundFrictionSprinting));
    MinFriction = FMath::Min(GroundFrictionWalking, FMath::Min(GroundFrictionRunning, GroundFrictionSprinting));

    // --- MOMENTUM/DRAG ---
    NoInputSpeedDecayRate = 1.0f / FMath::Max(DeltaTime, 1e-3f);
    InputSpeedBuildRate = MaxAccelerationWalking * DeltaTime / FMath::Max(CharacterMass, 1.f);

    // --- DIRECTIONAL / MOMENTUM / BLENDING ---
    SpeedRetentionOnDirectionChange = FMath::Clamp(MovementWeight, 0.2f, 0.98f);
    StrafingThreshold = FMath::Clamp(0.7f * JoystickSensitivity, 0.5f, 0.99f);
    DirectionChangeSharpness = 3.0f * InputResponsiveness;
    PivotThreshold = 45.f * (2.0f - InputResponsiveness);

    // --- GAIT BLENDING / TRANSITION ---
    GaitSpeedTransitionRate = FMath::Lerp(1.0f, 3.0f, GaitFlexibility);
    GaitAccelerationMultiplier = FMath::Lerp(1.0f, 2.0f, GaitFlexibility);
    GaitDecelerationMultiplier = FMath::Lerp(0.7f, 1.3f, 1.0f - GaitFlexibility);

    // Reduce hysteresis values for more responsive transitions
    RunHysteresis = FMath::Lerp(10.f, 20.f, GaitFlexibility);                // Reduced from 20-40
    SprintHysteresis = FMath::Lerp(10.f, 20.f, GaitFlexibility);             // Reduced from 20-40
    HoldTimeAboveRunThreshold = FMath::Lerp(0.05f, 0.15f, GaitFlexibility);  // Reduced from 0.1-0.25
    HoldTimeAboveSprintThreshold = FMath::Lerp(0.1f, 0.2f, GaitFlexibility); // Reduced from 0.15-0.35

    GaitInterpSpeed = FMath::Lerp(4.f, 12.f, InputResponsiveness);

    // --- INPUT HANDLING ---
    JoystickDeadzone = FMath::Lerp(0.1f, 0.25f, 1.f - JoystickSensitivity);
    JoystickWalkZone = FMath::Lerp(0.3f, 0.5f, 1.f - JoystickSensitivity); // Reduced from 0.4-0.6
    JoystickRunZone = FMath::Lerp(0.6f, 0.8f, 1.f - JoystickSensitivity);  // Reduced from 0.8-0.95
    InputSmoothingBlendSpeed = FMath::Lerp(6.f, 12.f, InputResponsiveness);
    InputIntentMagnitudeThreshold = FMath::Lerp(0.2f, 0.4f, 1.f - InputResponsiveness);

    // --- MULTIPLIERS ---
    BackpedalSpeedMultiplier = DirectionalSpeedCurve ? DirectionalSpeedCurve->GetFloatValue(180.f) : 0.75f;
    StrafeSpeedMultiplier = DirectionalSpeedCurve ? DirectionalSpeedCurve->GetFloatValue(90.f) : 0.85f;
    DiagonalSpeedMultiplier = DirectionalSpeedCurve ? DirectionalSpeedCurve->GetFloatValue(45.f) : 0.9f;

    // --- DERIVED INPUT VALUES ---
    InputBufferRiseRate = FMath::Lerp(8.f, 15.f, InputResponsiveness);
    InputBufferDecay = FMath::Lerp(4.f, 8.f, InputResponsiveness);
}

UOHMovementComponent::UOHMovementComponent() {
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_DuringPhysics; // Movement during physics

    // Initialize buffered values and timers
    BufferedInputAmount = 0.f;
    BufferedSpeed = 0.f;
    GaitBlendAlpha = 1.f;
    BufferedTimeAboveRunThreshold = 0.f;
    BufferedTimeAboveSprintThreshold = 0.f;
    CurrentGait = EOHGait::Walking;
    MovementPlayRate = 1.f;

    CurrentMovementMode = MOVE_Walking;
    CurrentCustomMovementMode = 0;
    InputIntentMagnitude = 0.f;

    CurrentForwardInput = 0.f;
    CurrentRightInput = 0.f;
    CurrentSpeed = 0.f;
    MovementInputVector = FVector2D::ZeroVector;

    void UpdateMovementConfig();
}

void UOHMovementComponent::BeginPlay() {
    Super::BeginPlay();
    UE_LOG(LogTemp, Warning, TEXT("MC: BeginPlay called for %s"), *GetNameSafe(this));

    bIsPlayerControlled = GetIsPlayerControlled();

    // Connect to PAC Manager pushback events
    if (UOHPACManager* PACManager = GetOwner()->FindComponentByClass<UOHPACManager>()) {
        PACManager->OnPushbackApplied.AddDynamic(this, &UOHMovementComponent::HandlePACPushback);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("MovementComponent connected to PAC pushback events"));
        }
    }
}

void UOHMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    Super::EndPlay(EndPlayReason);
}
#if WITH_EDITOR
void UOHMovementComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) {
    Super::PostEditChangeProperty(PropertyChangedEvent);
    MovementConfig.Update();
}
#endif
bool UOHMovementComponent::ShouldRotateToTarget() {
    return true;
}

void UOHMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                         FActorComponentTickFunction* ThisTickFunction) {
    if (!bMovementInitialized || !GetOwner() || !GetIsPlayerControlled()) {
        return;
    }

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1. Process physics IMPULSES first (NEW SYSTEM - for immediate combat pushback)
    ProcessPhysicsImpulses(DeltaTime);

    // 2. Update physics INPUT decay (OLD SYSTEM - for sustained forces)
    UpdatePhysicsInput(DeltaTime);

    // 3. Update proximity-based spacing (feeds into physics systems)
    UpdateProximitySpacing(DeltaTime);

    // 4. Normal movement pipeline
    UpdateInputBuffers(DeltaTime);
    UpdateMovement(DeltaTime);
    UpdateStanceTracking(DeltaTime);
    ApplyMovementInput();

    // 5. Rotation handling
    if (ShouldRotateToTarget()) {
        ApplyRotation(DeltaTime, true, 3);
    }

    // 6. Debug visualization (ENHANCED VERSION)
    if (MovementConfig.bEnableDebugOutput && GEngine) {
        DrawDebug_MovementData(DeltaTime);

        // Additional debug for physics systems
        if (bAcceptPhysicsInput) {
            // Show physics input (OLD SYSTEM)
            if (!PhysicsInputVector.IsNearlyZero()) {
                FString DebugText =
                    FString::Printf(TEXT("PhysicsInput: %.2f, %.2f"), PhysicsInputVector.X, PhysicsInputVector.Y);
                GEngine->AddOnScreenDebugMessage(999, 0.0f, FColor::Red, DebugText);
            }

            // Show physics impulse (NEW SYSTEM)
            if (!AccumulatedPhysicsImpulse.IsNearlyZero()) {
                FString ImpulseText =
                    FString::Printf(TEXT("PhysicsImpulse: %s"), *AccumulatedPhysicsImpulse.ToString());
                GEngine->AddOnScreenDebugMessage(998, 0.0f, FColor::Orange, ImpulseText);
            }

            // Show effective input
            FVector2D EffectiveInput = GetEffectiveMovementInput();
            if (!EffectiveInput.IsNearlyZero()) {
                FString EffectiveText =
                    FString::Printf(TEXT("Effective: %.2f, %.2f"), EffectiveInput.X, EffectiveInput.Y);
                GEngine->AddOnScreenDebugMessage(997, 0.0f, FColor::Cyan, EffectiveText);
            }
        }
    }

    // 7. Log warnings for debugging
    if (bVerboseLogging) {
        // Check if physics input exists but no movement
        if (bAcceptPhysicsInput && !PhysicsInputVector.IsNearlyZero()) {
            UE_LOG(LogTemp, Warning, TEXT("Physics Input Active: %.3f, %.3f"), PhysicsInputVector.X,
                   PhysicsInputVector.Y);
        }

        // Check effective input
        FVector2D EffectiveInput = GetEffectiveMovementInput();
        if (!EffectiveInput.IsNearlyZero()) {
            UE_LOG(LogTemp, Warning, TEXT("Effective Input: %.3f, %.3f"), EffectiveInput.X, EffectiveInput.Y);
        }
    }
}

void UOHMovementComponent::FindTargetActor() {
    if (AActor* Opponent = UOHCombatUtils::FindMostLikelyOpponent(GetOwner(), TEXT("john") /*optional*/)) {
        SetTargetActor(Opponent);
#if WITH_EDITOR
        GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::Red, TEXT("Found target actor!"));
#endif
    }
}

ACharacter* UOHMovementComponent::FindTargetCharacter() {
    AActor* Opponent = UOHCombatUtils::FindMostLikelyOpponent(GetOwner(), TEXT("john") /*optional*/);
    if (ACharacter* Char = Cast<ACharacter>(Opponent)) {
        return Char;
    }
    return nullptr;
}

void UOHMovementComponent::SetTargetActor(AActor* NewTarget) {
    if (NewTarget) {
        TargetActor = NewTarget;
    }
}

void UOHMovementComponent::ApplyMovementInput() {
    // Get the blended input using enhanced system
    FVector2D EffectiveInput = GetEffectiveMovementInput();

    // Early termination for zero input
    if (EffectiveInput.IsNearlyZero()) {
        LastMovementDirection = FVector::ZeroVector;
        return;
    }

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character || !Character->GetCharacterMovement()) {
        return;
    }

    // === ENHANCED CONTROL ROTATION CALCULATION ===
    FRotator ControlRotation = CalculateControlRotation();

    // === LEVERAGE OHLOCOMOTIONUTILS FOR MOVEMENT BASIS CALCULATION ===
    FVector Forward, Right;

    // Check if we have a lock-on target for enhanced movement
    if (TargetActor && IsValid(TargetActor)) {
        // Use OHLocomotionUtils for sophisticated lock-on movement
        UOHLocomotionUtils::GetLockOnMovementBasis(Character, TargetActor, Forward, Right);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Log, TEXT("Using lock-on movement basis for target: %s"), *TargetActor->GetName());
        }
    } else {
        // Standard movement basis calculation
        Forward = FRotationMatrix(ControlRotation).GetUnitAxis(EAxis::X);
        Right = FRotationMatrix(ControlRotation).GetUnitAxis(EAxis::Y);
    }

    // === CORRECTED 3D MOVEMENT VECTOR CONSTRUCTION ===
    // FIXED: Swap the input mapping to match Unreal conventions
    // X input = Right/Left movement, Y input = Forward/Backward movement
    FVector MoveVector = Forward * EffectiveInput.Y + Right * -EffectiveInput.X;
    MoveVector.Z = 0.0f; // Maintain ground-based movement

    // === SOPHISTICATED MOVEMENT APPLICATION USING OHLOCOMOTIONUTILS ===
    if (!MoveVector.IsNearlyZero()) {
        MoveVector.Normalize();
        float InputMagnitude = EffectiveInput.Size();
        MoveVector *= InputMagnitude;

        // === LEVERAGE OHLOCOMOTIONUTILS FOR WEIGHTED MOVEMENT APPLICATION ===
        UOHLocomotionUtils::ApplyWeightedMovementInput(Character, MoveVector, LastMovementDirection,
                                                       MovementConfig.GetMomentumBasedMovementWeight());

        // === ENHANCED DIRECTIONAL BLENDING USING UTILITY FUNCTIONS ===
        FVector NewDirection = MoveVector.GetSafeNormal();
        if (!LastMovementDirection.IsNearlyZero()) {
            // Use OHLocomotionUtils for sophisticated directional blending
            LastMovementDirection = UOHLocomotionUtils::BlendDirectionalInput(
                LastMovementDirection, NewDirection,
                MovementConfig.GetDirectionChangeSharpness() * GetWorld()->GetDeltaSeconds());
        } else {
            LastMovementDirection = NewDirection;
        }

        if (bVerboseLogging) {
            UE_LOG(LogTemp, VeryVerbose, TEXT("Movement Applied: Input=(X:%.2f,Y:%.2f), Move=%s, Weight=%.2f"),
                   EffectiveInput.X, EffectiveInput.Y, *MoveVector.ToString(),
                   MovementConfig.GetMomentumBasedMovementWeight());
        }
    }
}

void UOHMovementComponent::PrintDebug_MovementInput() {
#if WITH_EDITOR
    if (MovementConfig.bEnableDebugOutput && GetOwner()) {
        FVector Start = GetOwner()->GetActorLocation();

        // Green = Player input
        FVector PlayerEnd = Start + FVector(MovementInputVector.X, MovementInputVector.Y, 0) * 200.f;
        DrawDebugLine(GetWorld(), Start, PlayerEnd, FColor::Green, false, -1, 0, 3.0f);

        // Yellow = Physics input
        FVector PhysicsEnd = Start + FVector(PhysicsInputVector.X, PhysicsInputVector.Y, 0) * 200.f;
        DrawDebugLine(GetWorld(), Start + FVector(0, 0, 10), PhysicsEnd + FVector(0, 0, 10), FColor::Yellow, false, -1,
                      0, 3.0f);

        // Cyan = Combined effective
        DrawDebugLine(GetWorld(), Start, Start + LastMovementDirection * 200.f, FColor::Cyan, false, -1, 0, 5.0f);
    }
#endif
}

void UOHMovementComponent::PrintDebug_MovementData() {
#if WITH_EDITOR

    if (!MovementConfig.bEnableDebugOutput || !GEngine)
        return;

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character)
        return;

    FVector CharLocation = Character->GetActorLocation();

    FVector ActualVelocity = Character->GetCharacterMovement()->Velocity;
    // 1. Show Input Channels
    int32 MessageKey = 1000;

    // Player input
    GEngine->AddOnScreenDebugMessage(
        MessageKey++, 0.f, FColor::Green,
        FString::Printf(TEXT("Player Input: (%.2f, %.2f)"), MovementInputVector.X, MovementInputVector.Y));

    // Physics input (OLD SYSTEM)
    GEngine->AddOnScreenDebugMessage(MessageKey++, 0.f, FColor::Yellow,
                                     FString::Printf(TEXT("Physics Input: (%.2f, %.2f) Weight: %.2f"),
                                                     PhysicsInputVector.X, PhysicsInputVector.Y, PhysicsInputWeight));

    // Physics impulse (NEW SYSTEM)
    GEngine->AddOnScreenDebugMessage(
        MessageKey++, 0.f, FColor::Orange,
        FString::Printf(TEXT("Physics Impulse: %s"), *AccumulatedPhysicsImpulse.ToString()));

    // Effective movement
    GEngine->AddOnScreenDebugMessage(
        MessageKey++, 0.f, FColor::Cyan,
        FString::Printf(TEXT("Actual Velocity: %s Speed: %.1f"), *ActualVelocity.ToString(), ActualVelocity.Size()));
    // Debug output for tuning
    GEngine->AddOnScreenDebugMessage(
        -1, 5.0f, FColor::Yellow,
        FString::Printf(
            TEXT("Gait Speeds - Walk: %.0f (Thresh: %.0f) | Run: %.0f (Thresh: %.0f) | Sprint: %.0f (Thresh: %.0f)"),
            MovementConfig.GetMaxWalkSpeed(), MovementConfig.GetWalkSpeedThreshold(), MovementConfig.GetMaxRunSpeed(),
            MovementConfig.GetRunSpeedThreshold(), MovementConfig.GetMaxSprintSpeed(),
            MovementConfig.GetSprintSpeedThreshold()));

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Green,
                                     FString::Printf(TEXT("Momentum: %.0f | Dir: %.2fx | Rot: %.2fx | Angle: %.0f°"),
                                                     CurrentMomentumSpeed, DirectionalSpeedMultiplier,
                                                     RotationalSpeedMultiplier, CurrentMovementAngle));
#endif
}

void UOHMovementComponent::ApplyRotation(float DeltaTime, bool bBlend, float BlendSpeed) {
    FRotator TargetRotation = CalculateFacingRotation(DeltaTime);
    ApplyRotation(TargetRotation, bBlend, BlendSpeed, DeltaTime);
}

void UOHMovementComponent::ApplyRotation(const FRotator& TargetRotation, bool bBlend, float BlendSpeed,
                                         float DeltaTime) {
    if (AActor* Owner = GetOwner()) {
        ACharacter* Char = Cast<ACharacter>(Owner);
        /*if (Char && Char->GetMesh() && Char->GetMesh()->IsPlayingRootMotion())
        {
            return;
        }
    */
        FRotator CurrentRot = Owner->GetActorRotation();
        FRotator NewRot = bBlend ? FMath::RInterpTo(CurrentRot, TargetRotation, DeltaTime, BlendSpeed) : TargetRotation;
        Owner->SetActorRotation(NewRot);

#if WITH_EDITOR
        if (MovementConfig.bEnableDebugOutput) {
            FVector Start = Owner->GetActorLocation();
            FVector FacingEnd = Start + Owner->GetActorForwardVector() * 200.f;
            DrawDebugLine(GetWorld(), Start, FacingEnd, FColor::Blue, false, -1, 0, 3.0f);
        }
#endif
    }
}

FVector UOHMovementComponent::CalculateMovementInputVector() const {
    FVector2D MoveVector = GetEffectiveMovementInput(); //  This blends both!
    FRotator ControlRotation = CalculateControlRotation();
    float FacingBlend =
        MovementConfig.bUseRotationBlendedMovementInput ? MovementConfig.RotationBlendedMovementWeight : 0.0f;

    return UOHLocomotionUtils::CalculateBlendedMovementInputVector(MoveVector, // Now using EFFECTIVE input
                                                                   ControlRotation, GetOwner()->GetActorRotation(),
                                                                   FacingBlend);
}

FRotator UOHMovementComponent::CalculateControlRotation() const {

    if (AActor* Owner = GetOwner()) {
        return UOHLocomotionUtils::CalculateControlRotation(Owner);
    }
    return FRotator::ZeroRotator;
}

FRotator UOHMovementComponent::CalculateFacingRotation(float DeltaTime) const {
    if (!TargetActor)
        return GetOwner()->GetActorRotation();
    return UOHLocomotionUtils::CalculateBlendedHybridFacing(
        GetOwner()->GetActorRotation(), GetOwner()->GetActorLocation(), TargetActor->GetActorLocation(),
        MovementConfig.DeadzoneAngle, MovementConfig.MaxOffsetAngle, MovementConfig.PullStrength,
        MovementConfig.HardSnapStrength, DeltaTime);
}

void UOHMovementComponent::UpdateMovement(float DeltaTime) {

    MovementConfig.Update();
    if (DeltaTime <= 0.f) {
        return;
    }

    if (!bIsPlayerControlled) {
        // --- AI MOVEMENT PATH ---

        // Just pick a gait or set max speed directly.
        // Option 1: Always use running
        CurrentGait = EOHGait::Running;

        // Option 2: Expose a variable (set by AI/BT), e.g. DesiredAIGait
        // CurrentGait = DesiredAIGait;

        // Set speeds and movement settings for this gait
        UpdateDynamicMovementSettings(CurrentGait);

        // Optionally: Skip input buffer, gait threshold, input intent, etc.
        return;
    }

    // 1. Calculate the desired gait based on buffered input and speed
    EOHGait DesiredGait = CalculateDesiredGait(DeltaTime);

    // 2. Get allowed gait from gameplay/state rules (stub here)
    EOHGait AllowedGait = GetAllowedGait();

    // 3. Clamp actual gait between desired and allowed
    EOHGait ActualGait = GetActualGait(DesiredGait, AllowedGait);

    // 4. Smoothly interpolate current gait towards actual gait
    InterpolateGait(ActualGait, DeltaTime);

    // 5. Calculate animation play rate based on speed and input (optional fallback)
    float InputAmount = CalculateMovementInputAmount();
    float Speed = CalculateMovementSpeed();
    MovementPlayRate = CalculateMovementPlayRate(Speed, InputAmount);

    // 6. Update dynamic movement settings (speeds, acceleration, friction, etc.)
    UpdateDynamicMovementSettings(CurrentGait);

    // Optional: Broadcast gait change event if needed
    OnGaitChanged.Broadcast(CurrentGait);
}

void UOHMovementComponent::UpdateInputBuffers(float DeltaTime) {
    // Use MovementInputVector size as raw input magnitude (0-1)
    float CurrentInputAmount = MovementInputVector.Size();

    // Store InputIntentMagnitude for external use if needed
    InputIntentMagnitude = CurrentInputAmount;

    // Get flat horizontal speed
    CurrentSpeed = CalculateMovementSpeed();

    // Smooth buffered input magnitude: faster rise, slower decay
    if (CurrentInputAmount > BufferedInputAmount) {
        BufferedInputAmount = FMath::FInterpTo(BufferedInputAmount, CurrentInputAmount, DeltaTime,
                                               MovementConfig.GetInputBufferRiseRate());
    } else {
        BufferedInputAmount =
            FMath::FInterpTo(BufferedInputAmount, CurrentInputAmount, DeltaTime, MovementConfig.GetInputBufferDecay());
    }

    // Smooth buffered speed (always rise rate for smoothing)
    BufferedSpeed = FMath::FInterpTo(BufferedSpeed, CurrentSpeed, DeltaTime, MovementConfig.GetInputBufferRiseRate());

    // Update run hold timer with hysteresis
    if (BufferedSpeed > MovementConfig.GetRunSpeedThreshold() + MovementConfig.GetRunHysteresis()) {
        BufferedTimeAboveRunThreshold += DeltaTime;
    } else if (BufferedSpeed < MovementConfig.GetRunSpeedThreshold() - MovementConfig.GetRunHysteresis()) {
        BufferedTimeAboveRunThreshold = 0.f;
    }

    // Update sprint hold timer with hysteresis
    if (BufferedSpeed > MovementConfig.GetSprintSpeedThreshold() + MovementConfig.GetSprintHysteresis()) {
        BufferedTimeAboveSprintThreshold += DeltaTime;
    } else if (BufferedSpeed < MovementConfig.GetSprintSpeedThreshold() - MovementConfig.GetSprintHysteresis()) {
        BufferedTimeAboveSprintThreshold = 0.f;
    }
}

// Override UpdateDynamicMovementSettings for momentum-based approach
void UOHMovementComponent::UpdateDynamicMovementSettings(EOHGait AllowedGait) {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return;

    UCharacterMovementComponent* MoveComp = OwnerChar->GetCharacterMovement();
    if (!MoveComp)
        return;

    // Determine base target speed for current gait
    float BaseTargetSpeed = MovementConfig.GetMaxWalkSpeed();

    switch (AllowedGait) {
    case EOHGait::Walking:
        BaseTargetSpeed = MovementConfig.GetMaxWalkSpeed();
        break;
    case EOHGait::Running:
        BaseTargetSpeed = MovementConfig.GetMaxRunSpeed();
        break;
    case EOHGait::Sprinting:
        BaseTargetSpeed = MovementConfig.GetMaxSprintSpeed();
        break;
    }

    // === MOMENTUM-BASED SPEED SYSTEM ===
    if (MovementConfig.bUseMomentumBasedMovement) {
        float DeltaTime = GetWorld()->GetDeltaSeconds();

        // 1. Check if we have input
        bool bHasInput = BufferedInputAmount > MovementConfig.GetJoystickDeadzone();

        // 2. Build or decay momentum speed based on input
        if (bHasInput) {
            // Build speed when input detected
            CurrentMomentumSpeed = FMath::FInterpTo(CurrentMomentumSpeed, BaseTargetSpeed, DeltaTime,
                                                    MovementConfig.GetInputSpeedBuildRate());
        } else {
            // Decay speed when no input
            CurrentMomentumSpeed =
                FMath::FInterpTo(CurrentMomentumSpeed, 0.f, DeltaTime, MovementConfig.GetNoInputSpeedDecayRate());
        }

        // 3. Calculate directional modifiers
        CalculateDirectionalSpeedModifiers(OwnerChar);
        // 4. Apply all modifiers to get final speed
        float FinalSpeed = CurrentMomentumSpeed;
        FinalSpeed *= DirectionalSpeedMultiplier;
        FinalSpeed *= RotationalSpeedMultiplier;

        // 5. Apply input magnitude scaling (for analog feel)
        if (bHasInput && MovementConfig.bUseInputMagnitudeForSpeed) {
            float InputScale = FMath::GetMappedRangeValueClamped(FVector2D(MovementConfig.GetJoystickDeadzone(), 1.0f),
                                                                 FVector2D(0.3f, 1.0f), BufferedInputAmount);
            FinalSpeed *= InputScale;
        }

        // 6. Set the character movement speed
        MoveComp->MaxWalkSpeed = FinalSpeed;
        CurrentTargetSpeed = BaseTargetSpeed; // Store base for reference
        SmoothedTargetSpeed = FinalSpeed;     // Store actual for debug
    } else {
        // Fallback to standard system
        MoveComp->MaxWalkSpeed = BaseTargetSpeed;
    }

    // === PHYSICS ADJUSTMENTS FOR WEIGHT ===

    // Get base physics values from curve or defaults
    FVector CurveValues = FVector::ZeroVector;
    if (MovementConfig.MovementCurve) {
        float MappedSpeed = GetMappedSpeed();
        CurveValues = MovementConfig.MovementCurve->GetVectorValue(MappedSpeed);
    }

    float BaseAccel = CurveValues.X > 0.f ? CurveValues.X : 2048.f;
    float BaseBraking = CurveValues.Y > 0.f ? CurveValues.Y : 2048.f;
    float BaseFriction = CurveValues.Z > 0.f ? CurveValues.Z : 8.f;

    // Modify physics based on movement weight
    float WeightMultiplier = FMath::Lerp(1.5f, 0.5f, MovementConfig.MovementWeight);    // Heavy = slower accel
    float FrictionMultiplier = FMath::Lerp(0.7f, 1.3f, MovementConfig.FrictionControl); // Heavy = more friction

    // Apply gait-specific modifiers
    switch (AllowedGait) {
    case EOHGait::Sprinting:
        BaseAccel *= 0.8f * WeightMultiplier; // Slower to reach top speed
        BaseBraking *= 0.6f;                  // Less braking for momentum
        BaseFriction *= 0.8f * FrictionMultiplier;
        break;
    case EOHGait::Running:
        BaseAccel *= 1.0f * WeightMultiplier;
        BaseBraking *= 0.8f;
        BaseFriction *= 1.0f * FrictionMultiplier;
        break;
    case EOHGait::Walking:
        BaseAccel *= 1.2f * WeightMultiplier; // More responsive at low speed
        BaseBraking *= 1.2f;
        BaseFriction *= 1.2f * FrictionMultiplier;
        break;
    }

    // Apply to movement component
    MoveComp->MaxAcceleration = BaseAccel;
    MoveComp->BrakingDecelerationWalking = BaseBraking;
    MoveComp->GroundFriction = BaseFriction;
}

// New function to calculate directional speed modifiers
void UOHMovementComponent::CalculateDirectionalSpeedModifiers(ACharacter* OwnerChar) {
    if (!OwnerChar)
        return;

    FVector CharacterForward = OwnerChar->GetActorForwardVector();
    FVector MovementDirection = CalculateMovementInputVector();

    if (!MovementDirection.IsNearlyZero()) {
        MovementDirection.Normalize();

        // Calculate angle between facing and movement (0-180)
        float DotProduct = FVector::DotProduct(CharacterForward, MovementDirection);
        CurrentMovementAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(DotProduct, -1.f, 1.f)));

        // Base directional multiplier
        if (CurrentMovementAngle < 45.f) {
            // Forward movement
            DirectionalSpeedMultiplier = 1.0f;
        } else if (CurrentMovementAngle > 135.f) {
            // Backward movement
            DirectionalSpeedMultiplier = MovementConfig.GetBackpedalSpeedMultiplier();
        } else if (FMath::Abs(CurrentMovementAngle - 90.f) < 20.f) {
            // Pure strafe (70-110 degrees)
            DirectionalSpeedMultiplier = MovementConfig.GetStrafeSpeedMultiplier();
        } else {
            // Diagonal movement
            DirectionalSpeedMultiplier = MovementConfig.GetDiagonalSpeedMultiplier();
        }

        // Apply custom curve if available
        if (MovementConfig.DirectionalSpeedCurve) {
            DirectionalSpeedMultiplier *= MovementConfig.DirectionalSpeedCurve->GetFloatValue(CurrentMovementAngle);
        }

        // === DIRECTION CHANGE PENALTY ===
        if (!LastMovementDirection.IsNearlyZero()) {
            float DirectionDot = FVector::DotProduct(MovementDirection, LastMovementDirection);
            float DirectionChangeAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(DirectionDot, -1.f, 1.f)));

            // Apply speed retention based on how sharp the direction change is
            if (DirectionChangeAngle > 90.f) {
                float ChangeMultiplier = FMath::GetMappedRangeValueClamped(
                    FVector2D(90.f, 180.f), FVector2D(1.0f, MovementConfig.GetSpeedRetentionOnDirectionChange()),
                    DirectionChangeAngle);
                DirectionalSpeedMultiplier *= ChangeMultiplier;
            }
        }

        // Smooth the direction change
        LastMovementDirection =
            FMath::VInterpTo(LastMovementDirection, MovementDirection, GetWorld()->GetDeltaSeconds(),
                             MovementConfig.GetDirectionChangeSharpness());
    } else {
        // No input - maintain last direction for momentum
        DirectionalSpeedMultiplier = 1.0f;
    }

    // === ROTATIONAL SPEED MODIFIER ===

    // Calculate angular velocity
    static FRotator LastRotation = OwnerChar->GetActorRotation();
    FRotator CurrentRotation = OwnerChar->GetActorRotation();
    float DeltaYaw = FMath::FindDeltaAngleDegrees(LastRotation.Yaw, CurrentRotation.Yaw);
    CurrentAngularVelocity = FMath::Abs(DeltaYaw) / GetWorld()->GetDeltaSeconds();
    LastRotation = CurrentRotation;

    // Base rotation penalty
    if (CurrentAngularVelocity > 90.f) // Turning faster than 90 deg/sec
    {
        RotationalSpeedMultiplier =
            FMath::GetMappedRangeValueClamped(FVector2D(90.f, 360.f), FVector2D(1.0f, 0.7f), CurrentAngularVelocity);
    } else {
        RotationalSpeedMultiplier = 1.0f;
    }

    // Apply custom curve if available
    if (MovementConfig.RotationalSpeedCurve) {
        RotationalSpeedMultiplier *= MovementConfig.RotationalSpeedCurve->GetFloatValue(CurrentAngularVelocity);
    }
}

void UOHMovementComponent::HandlePACPushback(FName BoneName, const FVector2D& PushbackVector, float ImpactForce) {
    if (!bAcceptPhysicsInput) {
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("HandlePACPushback: Physics input disabled"));
        }
        return;
    }

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character) {
        UE_LOG(LogTemp, Error, TEXT("HandlePACPushback: No character owner!"));
        return;
    }

    // Ensure we're initialized
    if (!bMovementInitialized) {
        EnsureInitialized();
    }

    // The pushback vector is already in character-relative space (Forward, Right)
    // We need to convert it back to world space for the impulse system
    FVector ForwardDir = Character->GetActorForwardVector();
    FVector RightDir = Character->GetActorRightVector();

    // Build world-space impulse from character-relative input
    FVector WorldPush = ForwardDir * PushbackVector.X + RightDir * PushbackVector.Y;
    WorldPush.Z = 0.0f; // Keep it horizontal

    // Scale the impulse
    WorldPush *= ImpactForce * PhysicsImpulseStrength;

    // Store combat pushback separately
    LastCombatPushback = WorldPush;
    LastCombatPushbackTime = GetWorld()->GetTimeSeconds();

    // Add to physics impulse accumulator
    AccumulatedPhysicsImpulse += WorldPush;

    // Cap maximum accumulated impulse
    float MaxImpulse = PhysicsImpulseStrength * 1000.0f;
    if (AccumulatedPhysicsImpulse.Size() > MaxImpulse) {
        AccumulatedPhysicsImpulse = AccumulatedPhysicsImpulse.GetClampedToMaxSize(MaxImpulse);
    }

    // Set flags
    bCombatPushbackActive = true;
    bForceImpulseBasedPhysics = true;

    // OPTIONAL: Apply immediate velocity for instant feedback
    if (Character->GetCharacterMovement()) {
        UCharacterMovementComponent* CharMove = Character->GetCharacterMovement();

        // Add a portion of the impulse as immediate velocity
        float ImmediateRatio = 0.3f; // 30% immediate, 70% over time
        FVector ImmediateVelocity = WorldPush.GetSafeNormal() * ImpactForce * ImmediateRatio;

        // Add to current velocity
        CharMove->Velocity += ImmediateVelocity;

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("Applied immediate velocity: %s"), *ImmediateVelocity.ToString());
        }
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("HandlePACPushback: CharRelative=(%.2f,%.2f) -> World=%s, Total=%s"),
               PushbackVector.X, PushbackVector.Y, *WorldPush.ToString(), *AccumulatedPhysicsImpulse.ToString());
    }

    // Fire delegate
    OnPushbackReceived.Broadcast(PushbackVector, ImpactForce * PhysicsImpulseStrength);
}

FVector2D UOHMovementComponent::ConvertImpulseToInput(const FVector& WorldImpulse) const {
    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character)
        return FVector2D::ZeroVector;

    // Convert world impulse to local character space
    FVector LocalImpulse = Character->GetActorTransform().InverseTransformVector(WorldImpulse);

    // Extract forward/right components
    FVector2D InputVector;
    InputVector.X = LocalImpulse.X; // Forward
    InputVector.Y = LocalImpulse.Y; // Right

    // Normalize to input range (-1 to 1)
    float MaxInputMagnitude = 1000.0f; // Tune this
    InputVector /= MaxInputMagnitude;

    // Clamp to valid input range
    if (InputVector.Size() > 1.0f) {
        InputVector = InputVector.GetSafeNormal();
    }

    return InputVector;
}
float UOHMovementComponent::GetMappedSpeed() const {
    // Returns normalized speed between 0 and 1 for curve evaluation

    float Speed = CalculateMovementSpeed();

    // Clamp between WalkSpeedThreshold and SprintSpeedThreshold
    float MinSpeed = MovementConfig.GetWalkSpeedThreshold();
    float MaxSpeed = MovementConfig.GetSprintSpeedThreshold();

    if (MaxSpeed <= MinSpeed) {
        return 0.f; // Avoid divide by zero
    }

    float ClampedSpeed = FMath::Clamp(Speed, MinSpeed, MaxSpeed);

    // Normalize [MinSpeed, MaxSpeed] -> [0, 1]
    float NormalizedSpeed = (ClampedSpeed - MinSpeed) / (MaxSpeed - MinSpeed);

    return NormalizedSpeed;
}

// Delegate handlers for input axis

void UOHMovementComponent::OnReceiveMoveForward(float ForwardValue) {
    CurrentForwardInput = ForwardValue;
    MovementInputVector.X = ForwardValue;
    InputIntentMagnitude = MovementInputVector.Size();
    OnMovementInputUpdated.Broadcast(MovementInputVector);
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
    if (MovementConfig.bEnableDebugOutput && GEngine) {
        GEngine->AddOnScreenDebugMessage(static_cast<uint64>(reinterpret_cast<PTRINT>(this)) + 11111, 0.05f,
                                         FColor::Yellow,
                                         FString::Printf(TEXT("MC: OnReceiveMoveForward = %.2f"), ForwardValue));
    }

#endif
}

void UOHMovementComponent::OnReceiveMoveRight(float RightValue) {
    CurrentRightInput = RightValue;
    MovementInputVector.Y = RightValue;
    InputIntentMagnitude = MovementInputVector.Size();
    OnMovementInputUpdated.Broadcast(MovementInputVector);
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT

    if (MovementConfig.bEnableDebugOutput && GEngine) {
        GEngine->AddOnScreenDebugMessage(static_cast<uint64>(reinterpret_cast<PTRINT>(this)) + 22222, 0.05f,
                                         FColor::Orange,
                                         FString::Printf(TEXT("MC: OnReceiveMoveRight = %.2f"), RightValue));
    }

#endif
}

static FVector2D ClampVector2D(const FVector2D& Vec, float MaxSize) {
    if (Vec.SizeSquared() > MaxSize * MaxSize) {
        return Vec.GetSafeNormal() * MaxSize;
    }
    return Vec;
}

void UOHMovementComponent::OnReceiveMoveVector(float ForwardValue, float RightValue) {
    CurrentForwardInput = ForwardValue;
    CurrentRightInput = RightValue;
    MovementInputVector = ClampVector2D(FVector2D(ForwardValue, RightValue), 1.f);
    InputIntentMagnitude = MovementInputVector.Size();
    OnMovementInputUpdated.Broadcast(MovementInputVector);
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
    if (MovementConfig.bEnableDebugOutput) {
        if (GEngine) {
            GEngine->AddOnScreenDebugMessage(
                static_cast<uint64>(reinterpret_cast<PTRINT>(this)) + 33333, 0.05f, FColor::Red,
                FString::Printf(TEXT("MC: OnReceiveMoveVector F=%.2f R=%.2f"), ForwardValue, RightValue));
        }
    }

#endif

    // --- Debug: Track Last Received Input ---
    LastReceivedInputVector = MovementInputVector;
    LastInputDebugTimestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f;
}

bool UOHMovementComponent::GetIsPlayerControlled() const {
    bool bIsPlayer = false;

    if (ACharacter* OwnerCharacter = Cast<ACharacter>(GetOwner())) {
        if (AController* Controller = OwnerCharacter->GetController()) {
            bIsPlayer = Controller->IsPlayerController();
        }
    }
    return bIsPlayer;
}

void UOHMovementComponent::SetMovementInputVector(const FVector2D& NewInput) {
    MovementInputVector = NewInput;

    // Manually clamp magnitude
    if (MovementInputVector.SizeSquared() > 1.f) {
        MovementInputVector.Normalize();
    }

    OnMovementInputUpdated.Broadcast(MovementInputVector);

    CurrentForwardInput = MovementInputVector.X;
    CurrentRightInput = MovementInputVector.Y;
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
    if (MovementConfig.bEnableDebugOutput) {
        if (GEngine) {
            GEngine->AddOnScreenDebugMessage(static_cast<uint64>(reinterpret_cast<PTRINT>(this)) + 33333, 0.05f,
                                             FColor::Green,
                                             FString::Printf(TEXT("MC: SetMovementInputVector F=%.2f R=%.2f"),
                                                             CurrentForwardInput, CurrentRightInput));
        }
    }

#endif
}

FRotator UOHMovementComponent::CalculateRotationToTarget_Internal() const {
    APawn* OwnerPawn = Cast<APawn>(GetOwner());
    if (!OwnerPawn) {
        return FRotator::ZeroRotator;
    }

    TargetActor = GetLockedTargetActor();
    if (!TargetActor) {
        return FRotator::ZeroRotator;
    }

    FVector OwnerLocation = OwnerPawn->GetActorLocation();
    FVector TargetLocation = TargetActor->GetActorLocation();

    return UKismetMathLibrary::FindLookAtRotation(OwnerLocation, TargetLocation);
}

float UOHMovementComponent::GetInputWorldAngleDegrees() const {
    // Optional: Returns the angle between the forward vector and the input vector in degrees
    if (ACharacter* OwnerChar = Cast<ACharacter>(GetOwner())) {
        FVector ActorForward = OwnerChar->GetActorForwardVector();
        FVector InputVec(MovementInputVector.X, MovementInputVector.Y, 0.f);

        if (!InputVec.IsNearlyZero()) {
            InputVec.Normalize();
            float Dot = FVector::DotProduct(ActorForward, InputVec);
            float AngleRadians = FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f));
            return FMath::RadiansToDegrees(AngleRadians);
        }
    }
    return 0.f;
}

FRotator UOHMovementComponent::CalculateTargetOffset_Internal() const {
    APawn* OwnerPawn = Cast<APawn>(GetOwner());
    if (!OwnerPawn)
        return FRotator::ZeroRotator;

    FRotator CurrentRotation = OwnerPawn->GetActorRotation();
    FRotator TargetRotation = CalculateRotationToTarget_Internal();

    FRotator DeltaRotation = UKismetMathLibrary::NormalizedDeltaRotator(TargetRotation, CurrentRotation);

    float Roll, Pitch, Yaw;
    UKismetMathLibrary::BreakRotator(DeltaRotation, Roll, Pitch, Yaw);

    float OffsetYaw = 180.f - Yaw;
    float NormalizedYaw = UKismetMathLibrary::NormalizeAxis(OffsetYaw);

    return FRotator(0.f, 0.f, NormalizedYaw);
}

float UOHMovementComponent::CalculateMovementInputAmount() const {
    return FMath::Clamp(MovementInputVector.Size(), 0.f, 1.f);
}

float UOHMovementComponent::CalculateMovementSpeed() const {
    if (const ACharacter* OwnerChar = Cast<ACharacter>(GetOwner())) {
        FVector Velocity = OwnerChar->GetVelocity();
        Velocity.Z = 0.f;
        return Velocity.Size();
    }
    return 0.f;
}

float UOHMovementComponent::CalculateMovementPlayRate(float Speed, float InputAmount) const {
    if (InputAmount < KINDA_SMALL_NUMBER) {
        return 1.f;
    }

    float MinSpeed = MovementConfig.GetWalkSpeedThreshold();
    float MaxSpeed = MovementConfig.GetSprintSpeedThreshold();

    return FMath::GetMappedRangeValueClamped(FVector2D(MinSpeed, MaxSpeed), FVector2D(0.5f, 1.5f), Speed);
}

// 1. Enhanced CalculateDesiredGait for virtual joystick
EOHGait UOHMovementComponent::CalculateDesiredGait(float DeltaTime) {
    const float InputAmount = CalculateMovementInputAmount();
    const float Speed = CalculateMovementSpeed();

    // Store for debugging
    LastCalculatedSpeed = Speed;
    LastBufferedInputAmount = BufferedInputAmount;

    // For virtual joystick, use raw input magnitude for intent
    float IntentMagnitude = InputAmount;

    // Apply deadzone
    if (IntentMagnitude < MovementConfig.GetJoystickDeadzone()) {
        IntentMagnitude = 0.f;
    }

    // Smooth the input with different rates for rise/fall
    if (IntentMagnitude > BufferedInputAmount) {
        BufferedInputAmount =
            FMath::FInterpTo(BufferedInputAmount, IntentMagnitude, DeltaTime, MovementConfig.GetInputBufferRiseRate());
    } else {
        BufferedInputAmount =
            FMath::FInterpTo(BufferedInputAmount, IntentMagnitude, DeltaTime, MovementConfig.GetInputBufferDecay());
    }

    // Smooth speed always
    BufferedSpeed = FMath::FInterpTo(BufferedSpeed, Speed, DeltaTime, MovementConfig.GetInputBufferRiseRate());

    // Virtual joystick intent-based gait selection
    EOHGait IntentGait = EOHGait::Walking;

    if (BufferedInputAmount > MovementConfig.GetJoystickRunZone()) {
        IntentGait = EOHGait::Sprinting;
    } else if (BufferedInputAmount > MovementConfig.GetJoystickWalkZone()) {
        IntentGait = EOHGait::Running;
    } else if (BufferedInputAmount > MovementConfig.GetJoystickDeadzone()) {
        IntentGait = EOHGait::Walking;
    }

    // Now check if we have enough speed to support the intent
    bool bSpeedSupportsIntent = true;

    if (IntentGait == EOHGait::Sprinting && BufferedSpeed < MovementConfig.GetRunSpeedThreshold()) {
        // Not fast enough for sprint yet
        bSpeedSupportsIntent = false;
        IntentGait = EOHGait::Running;
    }

    if (IntentGait == EOHGait::Running && BufferedSpeed < MovementConfig.GetWalkSpeedThreshold()) {
        // Not fast enough for run yet
        bSpeedSupportsIntent = false;
        IntentGait = EOHGait::Walking;
    }

    // Update timers based on intent and speed
    if (IntentGait == EOHGait::Sprinting && BufferedSpeed > MovementConfig.GetRunSpeedThreshold()) {
        BufferedTimeAboveSprintThreshold += DeltaTime;
        BufferedTimeAboveRunThreshold += DeltaTime; // Also accumulate run time
    } else if (IntentGait == EOHGait::Running && BufferedSpeed > MovementConfig.GetWalkSpeedThreshold()) {
        BufferedTimeAboveRunThreshold += DeltaTime;
        BufferedTimeAboveSprintThreshold = 0.f;
    } else {
        BufferedTimeAboveRunThreshold = 0.f;
        BufferedTimeAboveSprintThreshold = 0.f;
    }

    // Enhanced debug for virtual joystick
    if (MovementConfig.bEnableDebugOutput && GEngine) {
        GEngine->AddOnScreenDebugMessage(
            -1, 0.0f, FColor::Cyan,
            FString::Printf(TEXT("VJoy: Raw:%.2f Buff:%.2f Intent:%s Speed:%.0f SpeedOK:%s"), InputAmount,
                            BufferedInputAmount,
                            IntentGait == EOHGait::Sprinting ? TEXT("SPRINT")
                            : IntentGait == EOHGait::Running ? TEXT("RUN")
                                                             : TEXT("WALK"),
                            BufferedSpeed, bSpeedSupportsIntent ? TEXT("YES") : TEXT("NO")));
    }

    // Determine final gait with timing requirements
    if (IntentGait == EOHGait::Sprinting &&
        BufferedTimeAboveSprintThreshold >= MovementConfig.GetHoldTimeAboveSprintThreshold()) {
        return EOHGait::Sprinting;
    } else if (IntentGait >= EOHGait::Running &&
               BufferedTimeAboveRunThreshold >= MovementConfig.GetHoldTimeAboveRunThreshold()) {
        return EOHGait::Running;
    } else {
        return EOHGait::Walking;
    }
}

EOHGait UOHMovementComponent::GetAllowedGait() const {
    if (const ACharacter* OwnerChar = Cast<ACharacter>(GetOwner())) {
        // Disallow sprint if crouched or in air
        if (OwnerChar->bIsCrouched || CurrentMovementMode == MOVE_Falling) {
            return EOHGait::Walking;
        }

        // Get a forward vector of character (XY plane)
        const FVector Forward = OwnerChar->GetActorForwardVector().GetSafeNormal2D();
        // Get movement input vector (already clamped to max size 1)
        const FVector2D MovementInput2D = MovementInputVector;
        if (MovementInput2D.IsNearlyZero()) {
            return EOHGait::Walking; // No input means any sprint
        }

        // Convert input 2D to 3D vector on XY plane (assume X=Forward, Y=Right)
        FVector InputDir = FVector(MovementInput2D.X, MovementInput2D.Y, 0.f).GetSafeNormal();

        // Calculate angles between the forward and input direction in degrees
        float InputAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(Forward, InputDir)));

        // Define max allowed an angle to sprint (e.g., must be mostly forward)
        constexpr float MaxSprintAngle = 45.f;

        if (InputAngleDegrees > MaxSprintAngle) {
            // Input is too far sideways or backwards to sprint
            return EOHGait::Running;
        }

        // TODO: add other gameplay state restrictions here

        return EOHGait::Sprinting;
    }

    return EOHGait::Walking;
}

EOHGait UOHMovementComponent::GetActualGait(EOHGait DesiredGait, EOHGait AllowedGait) {
    // Clamp the actual gait so it doesn't exceed the allowed gait
    return (DesiredGait <= AllowedGait) ? DesiredGait : AllowedGait;
}

void UOHMovementComponent::InterpolateGait(EOHGait DesiredGait, float DeltaTime) {
    if (CurrentGait != DesiredGait) {
        // Fade out the old gait by interpolating GaitBlendAlpha towards 0
        GaitBlendAlpha = FMath::FInterpTo(GaitBlendAlpha, 0.f, DeltaTime, MovementConfig.GetGaitInterpSpeed());

        if (GaitBlendAlpha <= KINDA_SMALL_NUMBER) {
            // Switch gait and reset alpha to 1
            CurrentGait = DesiredGait;
            GaitBlendAlpha = 1.f;
            OnGaitChanged.Broadcast(CurrentGait);
        }
    } else {
        // When gait matches desired, fade alpha back to 1
        GaitBlendAlpha = FMath::FInterpTo(GaitBlendAlpha, 1.f, DeltaTime, MovementConfig.GetGaitInterpSpeed());
    }
}

void UOHMovementComponent::HandleMovementModeChanged(EMovementMode NewMovementMode, uint8 NewCustomMode) {
    CurrentMovementMode = NewMovementMode;
    CurrentCustomMovementMode = NewCustomMode;
}

bool UOHMovementComponent::ShouldPivot_Internal() const {
    FRotator TargetOffset = CalculateTargetOffset_Internal();
    return FMath::Abs(TargetOffset.Yaw) > MovementConfig.GetPivotThreshold();
}

// TODO: Implement according to your target locking system
AActor* UOHMovementComponent::GetLockedTargetActor() {
    // For example, if your character has a target pointer, you can do:
    // if (const ACharacter* OwnerChar = Cast<ACharacter>(GetOwner()))
    //     return OwnerChar->GetLockedTarget();
    // For now return nullptr; You need to replace this with your actual locking logic
    return nullptr;
}

// Call this from TickComponent after UpdateMovement
void UOHMovementComponent::UpdateStanceTracking(float DeltaTime) {
    if (!bEnableStanceTracking || !GetOwner())
        return;

    // Clear bone cache for fresh lookups
    BoneLocationCache.Empty();

    // Rate limiting for performance
    StanceUpdateAccumulator += DeltaTime;
    const float UpdateInterval = 1.0f / StanceUpdateRate;

    if (StanceUpdateAccumulator < UpdateInterval)
        return;

    // Update foot data
    UpdateFootData(LeftFootData, LeftFootBoneName, true, StanceUpdateAccumulator);
    UpdateFootData(RightFootData, RightFootBoneName, false, StanceUpdateAccumulator);

    // Update overall stance state
    UpdateStanceState(StanceUpdateAccumulator);

    // Calculate weight distribution
    CalculateWeightDistribution();

    // Debug visualization
#if WITH_EDITOR
    if (bDebugStanceTracking) {
        DrawDebug_StanceData();
    }
#endif

    // Reset accumulator
    StanceUpdateAccumulator = 0.0f;
}

void UOHMovementComponent::UpdateFootData(FOHFootStanceData& FootData, const FName& BoneName, bool bIsLeftFoot,
                                          float DeltaTime) {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar || !OwnerChar->GetMesh())
        return;

    // Get all bone transforms for kinematic chain
    UpdateKinematicChain(FootData.ChainData, bIsLeftFoot, OwnerChar->GetMesh());

    // Get foot transform and position
    FVector CurrentPos = FootData.ChainData.FootTransform.GetLocation();
    FVector LastPos = OHSafeMapUtils::SafeGet(LastBoneLocations, BoneName, CurrentPos);

    // Update basic foot data
    FootData.WorldPosition = CurrentPos;
    FootData.LocalPosition = OwnerChar->GetActorTransform().InverseTransformPosition(CurrentPos);
    FootData.Velocity = (CurrentPos - LastPos) / FMath::Max(DeltaTime, SMALL_NUMBER);

    // Enhanced ground detection with surface analysis
    FHitResult GroundHit;
    FootData.GroundDistance = TraceToGround(CurrentPos, GroundHit);
    FootData.bIsGrounded = FootData.GroundDistance < GroundedThreshold;

    if (FootData.bIsGrounded) {
        FootData.ChainData.SurfaceNormal = GroundHit.Normal;
        FootData.ChainData.SurfaceAngle =
            FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(GroundHit.Normal, FVector::UpVector)));
        FootData.ChainData.SurfaceType = GroundHit.PhysMaterial.IsValid()
                                             ? GroundHit.PhysMaterial->SurfaceType
                                             : TEnumAsByte<EPhysicalSurface>(EPhysicalSurface::SurfaceType_Default);
    }

    // Sliding detection
    UpdateSlidingDetection(FootData, DeltaTime);

    // Clipping detection with other leg
    FOHFootStanceData& OtherFoot = bIsLeftFoot ? RightFootData : LeftFootData;
    UpdateClippingDetection(FootData, OtherFoot);

    // Edge detection
    FootData.ChainData.bOnEdge = DetectNearbyEdge(CurrentPos, FootData.ChainData.SurfaceNormal);

    // Enhanced phase calculation using kinematic data
    EOHFootPhase NewPhase = CalculatePhaseFromKinematics(FootData, DeltaTime);
    if (NewPhase != FootData.CurrentPhase) {
        FootData.CurrentPhase = NewPhase;
        FootData.PhaseTime = 0.0f;

        // Track planting for slide detection
        if (NewPhase == EOHFootPhase::Plant || NewPhase == EOHFootPhase::Contact) {
            FootData.ChainData.PlantedPosition = CurrentPos;
            FootData.ChainData.TimeSincePlanted = 0.0f;
        }
    } else {
        FootData.PhaseTime += DeltaTime;
        if (FootData.bIsGrounded) {
            FootData.ChainData.TimeSincePlanted += DeltaTime;
        }
    }

    // Weight bearing with compression factor
    CalculateWeightBearingFromCompression(FootData, DeltaTime);

    // Cache position
    LastBoneLocations.Add(BoneName, CurrentPos);
}

EOHFootPhase UOHMovementComponent::CalculateFootPhase(const FOHFootStanceData& FootData, float DeltaTime) {
    // Phase detection based on velocity, ground contact, and position
    float VerticalVel = FootData.Velocity.Z;
    float HorizontalVel = FVector(FootData.Velocity.X, FootData.Velocity.Y, 0.0f).Size();

    // State machine for foot phases
    switch (FootData.CurrentPhase) {
    case EOHFootPhase::Contact:
        if (!FootData.bIsGrounded || VerticalVel > 20.0f)
            return EOHFootPhase::Lift;
        if (HorizontalVel > 100.0f && FootData.bIsForward)
            return EOHFootPhase::Push;
        break;

    case EOHFootPhase::Push:
        if (!FootData.bIsGrounded || VerticalVel > 50.0f)
            return EOHFootPhase::Lift;
        if (HorizontalVel < 20.0f)
            return EOHFootPhase::Contact;
        break;

    case EOHFootPhase::Lift:
        if (VerticalVel < -20.0f && !FootData.bIsGrounded)
            return EOHFootPhase::Swing;
        if (FootData.bIsGrounded)
            return EOHFootPhase::Contact;
        break;

    case EOHFootPhase::Swing:
        if (VerticalVel < -50.0f && FootData.GroundDistance < 30.0f)
            return EOHFootPhase::Reach;
        if (FootData.bIsGrounded)
            return EOHFootPhase::Plant;
        break;

    case EOHFootPhase::Reach:
        if (FootData.bIsGrounded || FootData.GroundDistance < 5.0f)
            return EOHFootPhase::Plant;
        break;

    case EOHFootPhase::Plant:
        if (FootData.PhaseTime > 0.1f && HorizontalVel < 20.0f)
            return EOHFootPhase::Contact;
        break;

    case EOHFootPhase::Pivot:
        if (!FootData.bIsPivoting)
            return EOHFootPhase::Lift;
        break;
    }

    return FootData.CurrentPhase;
}

void UOHMovementComponent::UpdateStanceState(float DeltaTime) {
    // Calculate cadence from recent steps
    if (RecentStepIntervals.Num() > 2) {
        float AverageInterval = 0.0f;
        for (float Interval : RecentStepIntervals) {
            AverageInterval += Interval;
        }
        AverageInterval /= RecentStepIntervals.Num();
        StanceState.StrideCadence = 1.0f / FMath::Max(AverageInterval, 0.1f);
    }

    // Calculate stride length
    FVector FootDelta = RightFootData.WorldPosition - LeftFootData.WorldPosition;
    StanceState.StrideLength = FootDelta.Size();
    StanceState.StanceWidth = FMath::Abs(FootDelta.Y); // Lateral distance

    // Determine stance type
    float ForwardDist = FootDelta.X; // Assuming X is forward
    float LateralDist = FootDelta.Y;

    if (FMath::Abs(ForwardDist) < 10.0f && FMath::Abs(LateralDist) < 20.0f) {
        StanceState.CurrentStance = EOHStanceType::Neutral;
    } else if (ForwardDist > 30.0f) {
        StanceState.CurrentStance = EOHStanceType::Forward;
    } else if (ForwardDist < -30.0f) {
        StanceState.CurrentStance = EOHStanceType::Backward;
    } else if (FMath::Abs(LateralDist) > 40.0f) {
        StanceState.CurrentStance = EOHStanceType::Wide;
    } else if (FMath::Abs(LateralDist) < 15.0f) {
        StanceState.CurrentStance = EOHStanceType::Narrow;
    } else if (CurrentSpeed > 100.0f) {
        StanceState.CurrentStance = EOHStanceType::Dynamic;
    } else {
        StanceState.CurrentStance = EOHStanceType::Staggered;
    }

    // Pivoting detection
    StanceState.bIsPivoting = (LeftFootData.bIsPivoting || RightFootData.bIsPivoting) && CurrentSpeed < 50.0f;

    if (StanceState.bIsPivoting) {
        // Calculate pivot angle based on character rotation change
        static float LastYaw = 0.0f;
        float CurrentYaw = GetOwner()->GetActorRotation().Yaw;
        StanceState.PivotAngle = FMath::FindDeltaAngleDegrees(LastYaw, CurrentYaw);
        LastYaw = CurrentYaw;
    }

    // Stability calculation
    float WeightBalance = FMath::Abs(LeftFootData.WeightBearing - RightFootData.WeightBearing);
    bool BothGrounded = LeftFootData.bIsGrounded && RightFootData.bIsGrounded;
    float SpeedFactor = FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 500.0f), FVector2D(1.0f, 0.5f), CurrentSpeed);

    StanceState.StabilityScore = (BothGrounded ? 0.5f : 0.0f) + (1.0f - WeightBalance) * 0.3f + SpeedFactor * 0.2f;
    StanceState.bIsStable = StanceState.StabilityScore > 0.7f;

    // Determine dominant foot
    if (LeftFootData.MovementContribution > RightFootData.MovementContribution + 0.2f) {
        bLeftFootDominant = true;
    } else if (RightFootData.MovementContribution > LeftFootData.MovementContribution + 0.2f) {
        bLeftFootDominant = false;
    }
    // Otherwise maintain current dominant foot
}

void UOHMovementComponent::CalculateWeightDistribution() {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return;

    // Get pelvis position as reference
    FVector PelvisPos = GetBoneWorldLocation(PelvisBoneName);

    // Calculate center of gravity (simplified)
    float TotalWeight = LeftFootData.WeightBearing + RightFootData.WeightBearing;
    if (TotalWeight > SMALL_NUMBER) {
        StanceState.CenterOfGravity = (LeftFootData.WorldPosition * LeftFootData.WeightBearing +
                                       RightFootData.WorldPosition * RightFootData.WeightBearing) /
                                      TotalWeight;
        StanceState.CenterOfGravity.Z = PelvisPos.Z; // Keep COG at pelvis height
    } else {
        StanceState.CenterOfGravity = PelvisPos;
    }

    // Weight distribution relative to character center
    FVector CharCenter = OwnerChar->GetActorLocation();
    FVector COGOffset = StanceState.CenterOfGravity - CharCenter;

    // Normalize to character space
    COGOffset = OwnerChar->GetActorTransform().InverseTransformVector(COGOffset);

    // X = forward/back, Y = left/right, Z = up/down
    StanceState.WeightDistribution = COGOffset / 50.0f; // Normalize to reasonable range
    StanceState.WeightDistribution = StanceState.WeightDistribution.GetClampedToMaxSize(1.0f);
}

bool UOHMovementComponent::IsFootForward(const FOHFootStanceData& FootData) const {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return false;

    // Get movement direction or facing direction
    FVector MoveDir = CalculateMovementInputVector();
    if (MoveDir.IsNearlyZero()) {
        MoveDir = OwnerChar->GetActorForwardVector();
    } else {
        MoveDir.Normalize();
    }

    // Check if foot is ahead of pelvis in movement direction
    FVector PelvisPos = GetBoneWorldLocation(PelvisBoneName);
    FVector FootToPelvis = FootData.WorldPosition - PelvisPos;
    FootToPelvis.Z = 0.0f; // Only consider horizontal plane

    float DotProduct = FVector::DotProduct(FootToPelvis.GetSafeNormal(), MoveDir);
    return DotProduct > 0.0f;
}

FVector UOHMovementComponent::GetBoneWorldLocation(const FName& BoneName) const {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar || !OwnerChar->GetMesh())
        return FVector::ZeroVector;

    // Use cache if available (for same frame)
    if (BoneLocationCache.Contains(BoneName)) {
        return BoneLocationCache[BoneName];
    }

    FVector Location = OwnerChar->GetMesh()->GetBoneLocation(BoneName);
    BoneLocationCache.Add(BoneName, Location);
    return Location;
}

FVector UOHMovementComponent::GetBoneVelocity(const FName& BoneName) const {
    // Could use physics velocities if available, or calculate from position delta
    FVector CurrentPos = GetBoneWorldLocation(BoneName);
    FVector LastPos = OHSafeMapUtils::SafeGet(LastBoneLocations, BoneName, CurrentPos);

    float DeltaTime = GetWorld()->GetDeltaSeconds();
    if (DeltaTime > SMALL_NUMBER) {
        return (CurrentPos - LastPos) / DeltaTime;
    }

    return FVector::ZeroVector;
}

// Original TraceToGround - preserved for backward compatibility
float UOHMovementComponent::TraceToGround(const FVector& StartPos) const {
    FHitResult DummyHit;
    return TraceToGround(StartPos, DummyHit);
}

float UOHMovementComponent::TraceToGround(const FVector& StartPos, FHitResult& OutHit) const {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return FootTraceDistance;

    FVector TraceStart = StartPos + FVector(0, 0, 10.0f);
    FVector TraceEnd = StartPos - FVector(0, 0, FootTraceDistance + SurfaceTraceExtension);

    FCollisionQueryParams QueryParams("StanceGroundTrace", false, OwnerChar);
    QueryParams.bReturnPhysicalMaterial = true;
    QueryParams.bTraceComplex = true;

    if (GetWorld()->LineTraceSingleByChannel(OutHit, TraceStart, TraceEnd, ECC_Visibility, QueryParams)) {
        // Store surface normal and angle
        FOHFootStanceData FootData = OutHit.Component == OwnerChar->GetMesh()
                                         ? (OutHit.BoneName == LeftFootBoneName ? LeftFootData : RightFootData)
                                         : LeftFootData; // Fallback
        FootData.ChainData.SurfaceNormal = OutHit.Normal;
        FootData.ChainData.SurfaceAngle =
            FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(OutHit.Normal, FVector::UpVector)));
        FootData.bIsGrounded = true;
        FootData.GroundDistance = FVector::Dist(StartPos, OutHit.Location);
        return FootData.GroundDistance;
    }

    return FootTraceDistance;
}

void UOHMovementComponent::UpdateKinematicChain(FOHKinematicChainData& ChainData, bool bIsLeftFoot,
                                                USkeletalMeshComponent* Mesh) {
    // Get all transforms
    FName FootBone = bIsLeftFoot ? LeftFootBoneName : RightFootBoneName;
    FName AnkleBone = bIsLeftFoot ? LeftAnkleBoneName : RightAnkleBoneName;
    FName KneeBone = bIsLeftFoot ? LeftKneeBoneName : RightKneeBoneName;
    FName HipBone = bIsLeftFoot ? LeftHipBoneName : RightHipBoneName;

    ChainData.FootTransform = Mesh->GetBoneTransform(Mesh->GetBoneIndex(FootBone));
    ChainData.AnkleTransform = Mesh->GetBoneTransform(Mesh->GetBoneIndex(AnkleBone));
    ChainData.KneeTransform = Mesh->GetBoneTransform(Mesh->GetBoneIndex(KneeBone));
    ChainData.HipTransform = Mesh->GetBoneTransform(Mesh->GetBoneIndex(HipBone));

    // Calculate orientations
    FRotator FootRot = ChainData.FootTransform.GetRotation().Rotator();
    ChainData.FootPitch = FootRot.Pitch;
    ChainData.FootRoll = FootRot.Roll;

    // Calculate leg metrics
    FVector HipPos = ChainData.HipTransform.GetLocation();
    FVector KneePos = ChainData.KneeTransform.GetLocation();
    FVector FootPos = ChainData.FootTransform.GetLocation();

    // Leg length and compression
    ChainData.LegLength = FVector::Dist(HipPos, FootPos);
    if (ChainData.RestLegLength < SMALL_NUMBER) {
        // Initialize rest length (assuming extended leg)
        float ThighLength = FVector::Dist(HipPos, KneePos);
        float ShinLength = FVector::Dist(KneePos, FootPos);
        ChainData.RestLegLength = ThighLength + ShinLength * 0.95f; // Slight bend
    }

    ChainData.LegCompression = 1.0f - (ChainData.LegLength / ChainData.RestLegLength);
    ChainData.LegCompression = FMath::Clamp(ChainData.LegCompression, 0.0f, 1.0f);

    // Knee angle (using law of cosines)
    float ThighLength = FVector::Dist(HipPos, KneePos);
    float ShinLength = FVector::Dist(KneePos, FootPos);

    if (ThighLength > SMALL_NUMBER && ShinLength > SMALL_NUMBER && ChainData.LegLength > SMALL_NUMBER) {
        float CosAngle =
            (ThighLength * ThighLength + ShinLength * ShinLength - ChainData.LegLength * ChainData.LegLength) /
            (2.0f * ThighLength * ShinLength);
        ChainData.KneeAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosAngle, -1.0f, 1.0f)));
    }

    // Knee direction (perpendicular to hip-foot line)
    FVector HipToFoot = (FootPos - HipPos).GetSafeNormal();
    FVector HipToKnee = (KneePos - HipPos).GetSafeNormal();
    ChainData.KneeDirection = (HipToKnee - (HipToFoot * FVector::DotProduct(HipToKnee, HipToFoot))).GetSafeNormal();
}

void UOHMovementComponent::UpdateSlidingDetection(FOHFootStanceData& FootData, float DeltaTime) {
    // Only check sliding when foot should be planted
    if (!FootData.bIsGrounded || FootData.CurrentPhase == EOHFootPhase::Swing ||
        FootData.CurrentPhase == EOHFootPhase::Lift) {
        FootData.ChainData.bIsSliding = false;
        FootData.ChainData.AccumulatedSlideDistance = 0.0f;
        return;
    }

    // Check velocity during planted phase
    float HorizontalSpeed = FVector(FootData.Velocity.X, FootData.Velocity.Y, 0.0f).Size();

    if (HorizontalSpeed > FootSlideThreshold && FootData.ChainData.TimeSincePlanted > 0.1f) {
        FootData.ChainData.bIsSliding = true;
        FootData.ChainData.SlideVelocity = HorizontalSpeed;
        FootData.ChainData.SlideDirection = FootData.Velocity.GetSafeNormal();

        // Accumulate slide distance
        float SlideDistance = FVector::Dist2D(FootData.WorldPosition, FootData.ChainData.PlantedPosition);
        FootData.ChainData.AccumulatedSlideDistance = SlideDistance;

        // Log significant slides
        if (SlideDistance > MaxAllowedSlideDistance) {
            UE_LOG(LogTemp, Warning, TEXT("Foot sliding detected! Distance: %.1fcm, Speed: %.1fcm/s"), SlideDistance,
                   HorizontalSpeed);
        }
    } else {
        FootData.ChainData.bIsSliding = false;
        FootData.ChainData.SlideVelocity = 0.0f;
    }
}

void UOHMovementComponent::UpdateClippingDetection(FOHFootStanceData& FootData, FOHFootStanceData& OtherFoot) {
    FootData.ChainData.bIsClipping = false;
    FootData.ChainData.ClippingBones.Empty();

    // Check foot-to-foot distance
    float FootDistance = FVector::Dist(FootData.WorldPosition, OtherFoot.WorldPosition);
    if (FootDistance < LegSeparationMin) {
        FootData.ChainData.bIsClipping = true;
        FootData.ChainData.ClipDepth = LegSeparationMin - FootDistance;
        FootData.ChainData.ClippingBones.Add(TEXT("Feet"));
    }

    // Check knee collision
    float KneeDistance =
        FVector::Dist(FootData.ChainData.KneeTransform.GetLocation(), OtherFoot.ChainData.KneeTransform.GetLocation());
    if (KneeDistance < KneeCollisionRadius * 2.0f) {
        FootData.ChainData.bIsClipping = true;
        FootData.ChainData.ClipDepth =
            FMath::Max(FootData.ChainData.ClipDepth, KneeCollisionRadius * 2.0f - KneeDistance);
        FootData.ChainData.ClippingBones.Add(TEXT("Knees"));
    }

    // Check if legs are crossing (knees on wrong sides)
    FVector HipCenter =
        (FootData.ChainData.HipTransform.GetLocation() + OtherFoot.ChainData.HipTransform.GetLocation()) * 0.5f;
    FVector ToLeftKnee = LeftFootData.ChainData.KneeTransform.GetLocation() - HipCenter;
    FVector ToRightKnee = RightFootData.ChainData.KneeTransform.GetLocation() - HipCenter;

    float CrossProduct = FVector::CrossProduct(ToLeftKnee, ToRightKnee).Z;
    if (CrossProduct < 0.0f) // Knees are crossed
    {
        FootData.ChainData.bIsClipping = true;
        FootData.ChainData.ClippingBones.Add(TEXT("LegsCrossed"));
    }
}

bool UOHMovementComponent::DetectNearbyEdge(const FVector& FootPos, const FVector& SurfaceNormal) {
    // Trace in cardinal directions to detect edges
    TArray<FVector> TraceDirections = {FVector::ForwardVector, FVector::BackwardVector, FVector::RightVector,
                                       FVector::LeftVector};

    for (const FVector& Dir : TraceDirections) {
        FVector TraceStart = FootPos + Dir * 20.0f;
        FVector TraceEnd = TraceStart - FVector::UpVector * EdgeDetectionDistance;

        FHitResult Hit;
        if (!GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_Visibility,
                                                  FCollisionQueryParams::DefaultQueryParam)) {
            // No ground found = edge
            return true;
        }
    }

    return false;
}

EOHFootPhase UOHMovementComponent::CalculatePhaseFromKinematics(const FOHFootStanceData& FootData, float DeltaTime) {
    // Use kinematic data to determine phase more accurately
    const FOHKinematicChainData& Chain = FootData.ChainData;

    // High compression + grounded = likely push phase
    if (FootData.bIsGrounded && Chain.LegCompression > 0.3f && FootData.Velocity.Z > 10.0f) {
        return EOHFootPhase::Push;
    }

    // Extension + high knee angle = swing phase
    if (!FootData.bIsGrounded && Chain.KneeAngle > 90.0f) {
        return EOHFootPhase::Swing;
    }

    // Approaching ground with extending leg = reach
    if (!FootData.bIsGrounded && FootData.GroundDistance < 30.0f && Chain.LegCompression < 0.2f &&
        FootData.Velocity.Z < 0.0f) {
        return EOHFootPhase::Reach;
    }

    // Use original phase detection as fallback
    return CalculateFootPhase(FootData, DeltaTime);
}

void UOHMovementComponent::CalculateWeightBearingFromCompression(FOHFootStanceData& FootData, float DeltaTime) {
    if (!FootData.bIsGrounded) {
        FootData.WeightBearing = FMath::FInterpTo(FootData.WeightBearing, 0.0f, DeltaTime, 10.0f);
        return;
    }

    // Weight bearing based on compression and phase
    float CompressionWeight = FootData.ChainData.LegCompression;
    float PhaseWeight = 0.5f;

    switch (FootData.CurrentPhase) {
    case EOHFootPhase::Push:
        PhaseWeight = 0.9f;
        break;
    case EOHFootPhase::Contact:
        PhaseWeight = 0.7f;
        break;
    case EOHFootPhase::Plant:
        PhaseWeight = 0.6f;
        break;
    }

    float TargetWeight = FMath::Max(CompressionWeight, PhaseWeight);
    FootData.WeightBearing = FMath::FInterpTo(FootData.WeightBearing, TargetWeight, DeltaTime, 5.0f);
}

#pragma region Debug

void UOHMovementComponent::DrawDebug_MovementData(float DeltaTime) {
    if (!MovementConfig.bEnableDebugOutput || !GEngine)
        return;

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character)
        return;

    FVector CharLocation = Character->GetActorLocation();

    FVector ActualVelocity = Character->GetCharacterMovement()->Velocity;

    if (bAcceptPhysicsInput) {
        // Draw accumulated impulse
        if (!AccumulatedPhysicsImpulse.IsNearlyZero()) {
            DrawDebugLine(GetWorld(), CharLocation,
                          CharLocation + AccumulatedPhysicsImpulse.GetClampedToMaxSize(200.0f), FColor::Orange, false,
                          -1, 0, 5.0f);

            DrawDebugString(GetWorld(), CharLocation + FVector(0, 0, 100),
                            FString::Printf(TEXT("Impulse: %.1f"), AccumulatedPhysicsImpulse.Size()), nullptr,
                            FColor::Orange, 0.0f);
        }

        // Draw actual velocity
        if (Character->GetCharacterMovement()) {
            FVector Velocity = Character->GetCharacterMovement()->Velocity;
            if (!Velocity.IsNearlyZero()) {
                DrawDebugLine(GetWorld(), CharLocation + FVector(0, 0, 50),
                              CharLocation + FVector(0, 0, 50) + Velocity * 0.1f, FColor::Cyan, false, -1, 0, 3.0f);
            }
        }
    }

    // 2. Visualize Forces
    float DebugScale = 0.1f;

    // Player input vector (GREEN)
    FVector PlayerInputWorld = Character->GetActorForwardVector() * MovementInputVector.X +
                               Character->GetActorRightVector() * MovementInputVector.Y;
    DrawDebugLine(GetWorld(), CharLocation, CharLocation + PlayerInputWorld * 200.f, FColor::Green, false, -1, 0, 3.0f);

    // Physics impulse (ORANGE)
    if (!AccumulatedPhysicsImpulse.IsNearlyZero()) {
        DrawDebugLine(GetWorld(), CharLocation + FVector(0, 0, 10),
                      CharLocation + FVector(0, 0, 10) + AccumulatedPhysicsImpulse * DebugScale, FColor::Orange, false,
                      -1, 0, 5.0f);

        // Impulse history trail
        FVector PrevPos = CharLocation;
        for (int32 i = 0; i < PhysicsImpulseHistory.Num(); i++) {
            float Alpha = static_cast<float>(i) / PhysicsImpulseHistory.Num();
            FColor TrailColor = FColor::MakeRedToGreenColorFromScalar(Alpha);
            FVector HistoryPos = PrevPos - PhysicsImpulseHistory[i] * DebugScale * 0.5f;
            DrawDebugLine(GetWorld(), PrevPos, HistoryPos, TrailColor, false, -1, 0, 2.0f);
            PrevPos = HistoryPos;
        }
    }

    // Actual velocity (CYAN)
    DrawDebugLine(GetWorld(), CharLocation + FVector(0, 0, 20),
                  CharLocation + FVector(0, 0, 20) + ActualVelocity * DebugScale, FColor::Cyan, false, -1, 0, 4.0f);

    // 3. Proximity Spacing Visualization
    if (bEnableProximitySpacing && TargetActor) {
        // Personal space bubble
        DrawDebugSphere(GetWorld(), CharLocation, PersonalSpaceRadius, 24, FColor::Red, false, -1, 0, 1.0f);

        // Optimal spacing ring
        DrawDebugCircle(GetWorld(), CharLocation, BaseSpacingDistance, 32, FColor::Green, false, -1, SDPG_World, 2.0f,
                        FVector(0, 0, 1));

        // Connection to target
        FVector TargetLoc = TargetActor->GetActorLocation();
        float Distance = FVector::Dist2D(CharLocation, TargetLoc);
        FColor DistanceColor = Distance < PersonalSpaceRadius   ? FColor::Red
                               : Distance < BaseSpacingDistance ? FColor::Yellow
                                                                : FColor::Green;

        DrawDebugLine(GetWorld(), CharLocation, TargetLoc, DistanceColor, false, -1, 0, 1.0f);

        // Distance text
        FVector MidPoint = (CharLocation + TargetLoc) * 0.5f + FVector(0, 0, 100);
        DrawDebugString(GetWorld(), MidPoint, FString::Printf(TEXT("Distance: %.1f"), Distance), nullptr, DistanceColor,
                        0.0f, true, 1.2f);
    }
}

void UOHMovementComponent::DrawDebug_StanceData() const {
#if WITH_EDITOR
    if (!GetWorld())
        return;

    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar || !OwnerChar->GetMesh())
        return;

    USkeletalMeshComponent* Mesh = OwnerChar->GetMesh();

    // === KINEMATIC CHAIN VISUALIZATION ===
    DrawDebug_KinematicChainData(LeftFootData, true, Mesh);
    DrawDebug_KinematicChainData(RightFootData, false, Mesh);

    // === SLIDING DETECTION ===
    if (LeftFootData.ChainData.bIsSliding) {
        DrawDebug_SlidingIndicator(LeftFootData, FColor::Red);
    }
    if (RightFootData.ChainData.bIsSliding) {
        DrawDebug_SlidingIndicator(RightFootData, FColor::Blue);
    }

    // === CLIPPING DETECTION ===
    if (LeftFootData.ChainData.bIsClipping || RightFootData.ChainData.bIsClipping) {
        DrawDebug_ClippingWarning();
    }

    // === LEG COMPRESSION VISUALIZATION ===
    DrawDebug_CompressionBars();

    // === ENVIRONMENT AWARENESS ===
    DrawDebug_EnvironmentInfo(LeftFootData);
    DrawDebug_EnvironmentInfo(RightFootData);

    // === ORIGINAL DEBUG ELEMENTS (Enhanced) ===
    DrawDebug_FootPhases();
    DrawDebug_BalanceAndWeightDistribution();
    PrintDebug_StanceData();

#endif
}

void UOHMovementComponent::DrawDebug_KinematicChainData(const FOHFootStanceData& FootData, bool bIsLeft,
                                                        USkeletalMeshComponent* Mesh) const {
#if WITH_EDITOR
    const FOHKinematicChainData& Chain = FootData.ChainData;
    FColor ChainColor = bIsLeft ? FColor::Red : FColor::Blue;
    FColor BrightChainColor = bIsLeft ? FColor(255, 100, 100) : FColor(100, 100, 255);

    // Get bone positions
    FVector FootPos = Chain.FootTransform.GetLocation();
    FVector AnklePos = Chain.AnkleTransform.GetLocation();
    FVector KneePos = Chain.KneeTransform.GetLocation();
    FVector HipPos = Chain.HipTransform.GetLocation();

    // Draw bone chain
    float LineThickness = FootData.WeightBearing * 3.0f + 1.0f;
    DrawDebugLine(GetWorld(), FootPos, AnklePos, ChainColor, false, DebugDrawDuration, 0, LineThickness);
    DrawDebugLine(GetWorld(), AnklePos, KneePos, ChainColor, false, DebugDrawDuration, 0, LineThickness);
    DrawDebugLine(GetWorld(), KneePos, HipPos, ChainColor, false, DebugDrawDuration, 0, LineThickness);

    // Joint spheres with size based on compression
    float KneeSize = FMath::Lerp(4.0f, 8.0f, Chain.LegCompression);
    DrawDebugSphere(GetWorld(), KneePos, KneeSize, 8, BrightChainColor, false, DebugDrawDuration);
    DrawDebugSphere(GetWorld(), HipPos, 6.0f, 8, ChainColor, false, DebugDrawDuration);

    // Knee angle visualization
    FVector KneeForward = Chain.KneeDirection * 20.0f;
    DrawDebugDirectionalArrow(GetWorld(), KneePos, KneePos + KneeForward, 10.0f, FColor::Yellow, false,
                              DebugDrawDuration, 0, 1.5f);

    // Knee angle text
    DrawDebugString(GetWorld(), KneePos + FVector(0, 0, 10), FString::Printf(TEXT("%.0f°"), Chain.KneeAngle), nullptr,
                    FColor::Yellow, DebugDrawDuration, true, 0.8f);

    // Foot orientation indicators
    FVector FootForward = Chain.FootTransform.GetRotation().GetForwardVector() * 15.0f;
    FVector FootRight = Chain.FootTransform.GetRotation().GetRightVector() * 10.0f;
    DrawDebugLine(GetWorld(), FootPos, FootPos + FootForward, FColor::Green, false, DebugDrawDuration, 0, 2.0f);
    DrawDebugLine(GetWorld(), FootPos, FootPos + FootRight, FColor::Red, false, DebugDrawDuration, 0, 2.0f);

    // Compression spring visualization
    if (Chain.LegCompression > 0.1f) {
        int32 Coils = 5;
        FVector SpringAxis = (HipPos - FootPos).GetSafeNormal();

        for (int32 i = 0; i < Coils; i++) {
            float SpringRadius = 5.0f;
            float T = static_cast<float>(i) / Coils;
            float NextT = static_cast<float>(i + 1) / Coils;

            FVector P1 = FMath::Lerp(FootPos, HipPos, T);
            FVector P2 = FMath::Lerp(FootPos, HipPos, NextT);

            float Angle1 = T * 360.0f * 3;
            float Angle2 = NextT * 360.0f * 3;

            FVector Offset1 =
                FVector(FMath::Cos(FMath::DegreesToRadians(Angle1)), FMath::Sin(FMath::DegreesToRadians(Angle1)), 0) *
                SpringRadius;
            FVector Offset2 =
                FVector(FMath::Cos(FMath::DegreesToRadians(Angle2)), FMath::Sin(FMath::DegreesToRadians(Angle2)), 0) *
                SpringRadius;

            DrawDebugLine(GetWorld(), P1 + Offset1, P2 + Offset2, FColor::Purple, false, DebugDrawDuration, 0, 2.0f);
        }
    }
#endif
}

void UOHMovementComponent::DrawDebug_SlidingIndicator(const FOHFootStanceData& FootData,
                                                      const FColor& FootColor) const {
#if WITH_EDITOR
    // Sliding trail
    if (FootData.ChainData.AccumulatedSlideDistance > 1.0f) {
        DrawDebugLine(GetWorld(), FootData.ChainData.PlantedPosition, FootData.WorldPosition, FColor::Orange, false,
                      DebugDrawDuration, 0, 3.0f);

        // Warning circle at planted position
        DrawDebugCircle(GetWorld(), FootData.ChainData.PlantedPosition, FootData.ChainData.AccumulatedSlideDistance, 16,
                        FColor::Orange, false, DebugDrawDuration, SDPG_World, 2.0f, FVector(0, 0, 1));
    }

    // Slide velocity arrow
    if (FootData.ChainData.SlideVelocity > 0.1f) {
        FVector SlideArrow = FootData.ChainData.SlideDirection * FootData.ChainData.SlideVelocity * 0.5f;
        DrawDebugDirectionalArrow(GetWorld(), FootData.WorldPosition, FootData.WorldPosition + SlideArrow, 15.0f,
                                  FColor::Orange, false, DebugDrawDuration, 0, 3.0f);

        // Slide info
        DrawDebugString(GetWorld(), FootData.WorldPosition + FVector(0, 0, 30),
                        FString::Printf(TEXT("SLIDE: %.1fcm @ %.0fcm/s"), FootData.ChainData.AccumulatedSlideDistance,
                                        FootData.ChainData.SlideVelocity),
                        nullptr, FColor::Orange, DebugDrawDuration, false, 1.0f);
    }
#endif
}

void UOHMovementComponent::DrawDebug_ClippingWarning() const {
#if WITH_EDITOR
    // Draw warning between clipping points
    if (LeftFootData.ChainData.bIsClipping && RightFootData.ChainData.bIsClipping) {
        FVector MidPoint = (LeftFootData.WorldPosition + RightFootData.WorldPosition) * 0.5f;

        // Flashing warning sphere
        float PulseScale = FMath::Sin(GetWorld()->GetTimeSeconds() * 10.0f) * 0.5f + 0.5f;
        float WarningRadius = FMath::Lerp(10.0f, 20.0f, PulseScale);

        DrawDebugSphere(GetWorld(), MidPoint, WarningRadius, 16, FColor::Red, false, DebugDrawDuration);

        // Clipping depth indicator
        float ClipDepth = FMath::Max(LeftFootData.ChainData.ClipDepth, RightFootData.ChainData.ClipDepth);
        DrawDebugString(GetWorld(), MidPoint + FVector(0, 0, 40), FString::Printf(TEXT("CLIPPING: %.1fcm"), ClipDepth),
                        nullptr, FColor::Red, DebugDrawDuration, false, 1.2f);

        // Draw X between feet if crossed
        if (LeftFootData.ChainData.ClippingBones.Contains(TEXT("LegsCrossed"))) {
            DrawDebugLine(GetWorld(), LeftFootData.ChainData.KneeTransform.GetLocation(),
                          RightFootData.ChainData.KneeTransform.GetLocation(), FColor::Red, false, DebugDrawDuration, 0,
                          4.0f);

            DrawDebugLine(GetWorld(), LeftFootData.WorldPosition, RightFootData.WorldPosition, FColor::Red, false,
                          DebugDrawDuration, 0, 4.0f);
        }
    }
#endif
}
void UOHMovementComponent::DrawDebug_CompressionBars() const {
#if WITH_EDITOR
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return;

    FVector BarBasePos = OwnerChar->GetActorLocation() + FVector(100, 0, 50);

    // Left leg compression bar
    float LeftBarHeight = LeftFootData.ChainData.LegCompression * 100.0f;
    FColor LeftBarColor =
        FMath::Lerp(FLinearColor(FColor::Green), FLinearColor(FColor::Red), LeftFootData.ChainData.LegCompression)
            .ToFColor(true);

    DrawDebugBox(GetWorld(), BarBasePos + FVector(0, -20, LeftBarHeight * 0.5f), FVector(10, 5, LeftBarHeight * 0.5f),
                 LeftBarColor, false, DebugDrawDuration);

    // Right leg compression bar
    float RightBarHeight = RightFootData.ChainData.LegCompression * 100.0f;
    FColor RightBarColor =
        FMath::Lerp(FLinearColor(FColor::Green), FLinearColor(FColor::Red), RightFootData.ChainData.LegCompression)
            .ToFColor(true);

    DrawDebugBox(GetWorld(), BarBasePos + FVector(0, 20, RightBarHeight * 0.5f), FVector(10, 5, RightBarHeight * 0.5f),
                 RightBarColor, false, DebugDrawDuration);

    // Labels
    DrawDebugString(GetWorld(), BarBasePos + FVector(0, -20, -10),
                    FString::Printf(TEXT("L: %.0f%%"), LeftFootData.ChainData.LegCompression * 100.0f), nullptr,
                    FColor::White, DebugDrawDuration, true, 0.8f);

    DrawDebugString(GetWorld(), BarBasePos + FVector(0, 20, -10),
                    FString::Printf(TEXT("R: %.0f%%"), RightFootData.ChainData.LegCompression * 100.0f), nullptr,
                    FColor::White, DebugDrawDuration, true, 0.8f);
#endif
}

void UOHMovementComponent::DrawDebug_BalanceAndWeightDistribution() const {
#if WITH_EDITOR
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar || !OwnerChar->GetMesh())
        return;

    // Get fresh positions
    FVector LeftFootPos = OwnerChar->GetMesh()->GetBoneLocation(LeftFootBoneName);
    FVector RightFootPos = OwnerChar->GetMesh()->GetBoneLocation(RightFootBoneName);
    FVector PelvisPos = OwnerChar->GetMesh()->GetBoneLocation(PelvisBoneName);

    // === WEIGHT DISTRIBUTION & BALANCE ===
    // Center of gravity with stability indicator
    FColor COGColor = StanceState.bIsStable ? FColor::Green : FColor::Orange;
    float COGRadius = FMath::Lerp(15.0f, 5.0f, StanceState.StabilityScore);
    DrawDebugSphere(GetWorld(), StanceState.CenterOfGravity, COGRadius, 16, COGColor, false, DebugDrawDuration);

    // Weight distribution vector
    FVector WeightArrowEnd = StanceState.CenterOfGravity + StanceState.WeightDistribution * 100.0f;
    DrawDebugDirectionalArrow(GetWorld(), StanceState.CenterOfGravity, WeightArrowEnd, 30.0f, FColor::Yellow, false,
                              DebugDrawDuration, 0, 3.0f);

    // Balance triangle between feet and COG
    TArray<FVector> BalanceTriangle = {LeftFootPos, RightFootPos, StanceState.CenterOfGravity};
    for (int32 i = 0; i < 3; i++) {
        DrawDebugLine(GetWorld(), BalanceTriangle[i], BalanceTriangle[(i + 1) % 3], COGColor, false, DebugDrawDuration,
                      0, 1.0f);
    }

    // Support polygon (base of support)
    DrawDebugLine(GetWorld(), LeftFootPos, RightFootPos, FColor::Cyan, false, DebugDrawDuration, 0, 2.0f);

    // Vertical projection of COG to ground
    FVector COGGroundProj = StanceState.CenterOfGravity;
    COGGroundProj.Z = FMath::Min(LeftFootPos.Z, RightFootPos.Z);
    DrawDebugLine(GetWorld(), StanceState.CenterOfGravity, COGGroundProj, FColor::Yellow, false, DebugDrawDuration, 0,
                  1.0f);
    DrawDebugSphere(GetWorld(), COGGroundProj, 5.0f, 8, FColor::Yellow, false, DebugDrawDuration);

    // Balance margin indicator (distance from COG projection to edge of support)
    FVector FootLine = RightFootPos - LeftFootPos;
    FVector ToCOG = COGGroundProj - LeftFootPos;
    float ProjectionOnLine = FVector::DotProduct(ToCOG, FootLine.GetSafeNormal());
    float LineLength = FootLine.Size();

    float BalanceMargin = FMath::Min(ProjectionOnLine, LineLength - ProjectionOnLine);
    float BalancePercentage = (BalanceMargin / (LineLength * 0.5f)) * 100.0f;

    DrawDebugString(GetWorld(), COGGroundProj + FVector(0, 0, 20),
                    FString::Printf(TEXT("Balance: %.0f%%"), BalancePercentage), nullptr, COGColor, DebugDrawDuration);
#endif
}

void UOHMovementComponent::DrawDebug_EnvironmentInfo(const FOHFootStanceData& FootData) const {
#if WITH_EDITOR
    if (!FootData.bIsGrounded)
        return;

    // Surface normal
    FVector NormalEnd = FootData.WorldPosition + FootData.ChainData.SurfaceNormal * 30.0f;
    DrawDebugDirectionalArrow(GetWorld(), FootData.WorldPosition, NormalEnd, 10.0f, FColor::Cyan, false,
                              DebugDrawDuration, 0, 1.0f);

    // Surface angle indicator
    if (FootData.ChainData.SurfaceAngle > 5.0f) {
        DrawDebugString(GetWorld(), FootData.WorldPosition + FVector(0, 0, -10),
                        FString::Printf(TEXT("%.0f°"), FootData.ChainData.SurfaceAngle), nullptr, FColor::Cyan,
                        DebugDrawDuration, true, 0.8f);
    }

    // Edge warning
    if (FootData.ChainData.bOnEdge) {
        DrawDebugCircle(GetWorld(), FootData.WorldPosition, 30.0f, 32, FColor::Yellow, false, DebugDrawDuration,
                        SDPG_World, 3.0f, FVector(0, 0, 1));

        DrawDebugString(GetWorld(), FootData.WorldPosition + FVector(0, 0, 40), TEXT("EDGE!"), nullptr, FColor::Yellow,
                        DebugDrawDuration, false, 1.2f);
    }
#endif
}

void UOHMovementComponent::DrawDebug_FootPhases() const {
#if WITH_EDITOR
    if (ACharacter* OwnerChar = Cast<ACharacter>(GetOwner())) {
        // Get fresh positions
        FVector LeftFootPos = OwnerChar->GetMesh()->GetBoneLocation(LeftFootBoneName);
        FVector RightFootPos = OwnerChar->GetMesh()->GetBoneLocation(RightFootBoneName);

        // Enhanced foot visualization with compression
        float LeftRadius =
            FMath::Lerp(3.0f, 12.0f, LeftFootData.WeightBearing * (1.0f + LeftFootData.ChainData.LegCompression));
        float RightRadius =
            FMath::Lerp(3.0f, 12.0f, RightFootData.WeightBearing * (1.0f + RightFootData.ChainData.LegCompression));

        FColor LeftColor = GetPhaseColor(LeftFootData.CurrentPhase);
        FColor RightColor = GetPhaseColor(RightFootData.CurrentPhase);

        // Modify color intensity based on sliding
        if (LeftFootData.ChainData.bIsSliding) {
            LeftColor = FMath::Lerp(FLinearColor(LeftColor), FLinearColor(FColor::Orange),
                                    0.5f)
                            .ToFColor(true); // true = sRGB
        }
        if (RightFootData.ChainData.bIsSliding) {
            RightColor = FMath::Lerp(FLinearColor(RightColor), FLinearColor(FColor::Orange), 0.5f).ToFColor(true);
        }

        DrawDebugSphere(GetWorld(), LeftFootPos, LeftRadius, 12, LeftColor, false, DebugDrawDuration);
        DrawDebugSphere(GetWorld(), RightFootPos, RightRadius, 12, RightColor, false, DebugDrawDuration);
    }

#endif
}
void UOHMovementComponent::PrintDebug_StanceData() const {
#if WITH_EDITOR
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar)
        return;

    FVector TextPos = OwnerChar->GetActorLocation() + FVector(0, 0, 250);

    // Kinematic chain info
    DrawDebugString(GetWorld(), TextPos,
                    FString::Printf(TEXT("Leg Length - L: %.0f/%.0f R: %.0f/%.0f"), LeftFootData.ChainData.LegLength,
                                    LeftFootData.ChainData.RestLegLength, RightFootData.ChainData.LegLength,
                                    RightFootData.ChainData.RestLegLength),
                    nullptr, FColor::White, DebugDrawDuration);

    // Sliding status
    if (LeftFootData.ChainData.bIsSliding || RightFootData.ChainData.bIsSliding) {
        DrawDebugString(GetWorld(), TextPos - FVector(0, 0, 20),
                        FString::Printf(TEXT("⚠ SLIDING - L: %.1fcm R: %.1fcm"),
                                        LeftFootData.ChainData.AccumulatedSlideDistance,
                                        RightFootData.ChainData.AccumulatedSlideDistance),
                        nullptr, FColor::Orange, DebugDrawDuration, false, 1.2f);
    }

    // Clipping status
    if (LeftFootData.ChainData.bIsClipping || RightFootData.ChainData.bIsClipping) {
        FString ClipInfo = TEXT("⚠ CLIPPING: ");
        if (LeftFootData.ChainData.ClippingBones.Num() > 0) {
            // Use JoinBy for direct FName to FString conversion and joining
            ClipInfo += FString::JoinBy(LeftFootData.ChainData.ClippingBones, TEXT(", "),
                                        [](const FName& Name) { return Name.ToString(); });
        }

        DrawDebugString(GetWorld(), TextPos - FVector(0, 0, 40), ClipInfo, nullptr, FColor::Red, DebugDrawDuration,
                        false, 1.2f);
    }
#endif
}
// Helper function to get phase-specific colors
FColor UOHMovementComponent::GetPhaseColor(EOHFootPhase Phase) {
    switch (Phase) {
    case EOHFootPhase::Contact:
        return FColor::Green;
    case EOHFootPhase::Push:
        return FColor::Yellow;
    case EOHFootPhase::Lift:
        return FColor::Orange;
    case EOHFootPhase::Swing:
        return FColor::Red;
    case EOHFootPhase::Reach:
        return FColor::Magenta;
    case EOHFootPhase::Plant:
        return FColor::Blue;
    case EOHFootPhase::Pivot:
        return FColor::Purple;
    default:
        return FColor::White;
    }
}

#pragma endregion

// Combat integration implementations
bool UOHMovementComponent::CanPerformKick(bool bLeftFoot) const {
    const FOHFootStanceData& FootData = bLeftFoot ? LeftFootData : RightFootData;
    const FOHFootStanceData& OtherFoot = bLeftFoot ? RightFootData : LeftFootData;

    // Can't kick if sliding or clipping
    if (FootData.ChainData.bIsSliding || FootData.ChainData.bIsClipping)
        return false;

    // Need other foot stable and grounded
    if (!OtherFoot.bIsGrounded || OtherFoot.WeightBearing < 0.6f)
        return false;

    // Check leg has room to extend
    if (FootData.ChainData.LegCompression > 0.7f)
        return false;

    // Check balance
    if (!StanceState.bIsStable || StanceState.StabilityScore < 0.5f)
        return false;

    return true;
}

float UOHMovementComponent::GetKickPower(bool bLeftFoot) const {
    const FOHFootStanceData& FootData = bLeftFoot ? LeftFootData : RightFootData;

    // Power based on compression (spring-loaded kick)
    float CompressionPower = FootData.ChainData.LegCompression;

    // Bonus for proper phase (push phase = optimal)
    float PhaseBonus = 0.0f;
    if (FootData.CurrentPhase == EOHFootPhase::Push)
        PhaseBonus = 0.2f;
    else if (FootData.CurrentPhase == EOHFootPhase::Contact)
        PhaseBonus = 0.1f;

    // Stability bonus
    float StabilityBonus = StanceState.StabilityScore * 0.2f;

    return FMath::Clamp(CompressionPower + PhaseBonus + StabilityBonus, 0.0f, 1.0f);
}

bool UOHMovementComponent::ValidateKinematicSetup() const {
    ACharacter* OwnerChar = Cast<ACharacter>(GetOwner());
    if (!OwnerChar || !OwnerChar->GetMesh()) {
        UE_LOG(LogTemp, Error, TEXT("ValidateKinematicSetup: No valid character/mesh"));
        return false;
    }

    USkeletalMeshComponent* Mesh = OwnerChar->GetMesh();

    // Validate all required bones exist
    TArray<TPair<FName, FName>> RequiredBones = {
        {TEXT("LeftFoot"), LeftFootBoneName},   {TEXT("RightFoot"), RightFootBoneName},
        {TEXT("LeftAnkle"), LeftAnkleBoneName}, {TEXT("RightAnkle"), RightAnkleBoneName},
        {TEXT("LeftKnee"), LeftKneeBoneName},   {TEXT("RightKnee"), RightKneeBoneName},
        {TEXT("LeftHip"), LeftHipBoneName},     {TEXT("RightHip"), RightHipBoneName},
        {TEXT("Pelvis"), PelvisBoneName}};

    bool bAllValid = true;
    for (const auto& BonePair : RequiredBones) {
        if (Mesh->GetBoneIndex(BonePair.Value) == INDEX_NONE) {
            UE_LOG(LogTemp, Error, TEXT("ValidateKinematicSetup: %s bone '%s' not found in skeleton"),
                   *BonePair.Key.ToString(), *BonePair.Value.ToString());
            bAllValid = false;
        }
    }

    if (!bAllValid) {
        UE_LOG(LogTemp, Error, TEXT("ValidateKinematicSetup: Missing required bones. Check bone name configuration."));

        // List available foot/leg bones for debugging
        const FReferenceSkeleton& RefSkeleton = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
        UE_LOG(LogTemp, Warning, TEXT("Available bones containing 'foot', 'knee', 'hip', or 'ankle':"));

        for (int32 i = 0; i < RefSkeleton.GetNum(); i++) {
            FString BoneName = RefSkeleton.GetBoneName(i).ToString().ToLower();
            if (BoneName.Contains("foot") || BoneName.Contains("knee") || BoneName.Contains("hip") ||
                BoneName.Contains("ankle") || BoneName.Contains("thigh") || BoneName.Contains("calf") ||
                BoneName.Contains("leg")) {
                UE_LOG(LogTemp, Warning, TEXT("  - %s"), *RefSkeleton.GetBoneName(i).ToString());
            }
        }
    }

    return bAllValid;
}

// OHMovementComponent.cpp - Collision integration
#pragma region COLLISION_IMPLEMENTATION

void UOHMovementComponent::UpdatePhysicsInput(float DeltaTime) {
    // This function now only handles the OLD physics input system decay
    // The NEW impulse system is handled in ProcessPhysicsImpulses

    // Only process if we're NOT using the impulse system
    if (!bForceImpulseBasedPhysics && !PhysicsInputVector.IsNearlyZero()) {
        // Apply decay
        float Decay = FMath::Exp(-PhysicsInputDecayRate * DeltaTime);

        // Different decay rates for X and Y
        float ForwardDecay = Decay;
        float LateralDecay = Decay * 1.2f; // Lateral decays faster

        PhysicsInputVector.X *= ForwardDecay;
        PhysicsInputVector.Y *= LateralDecay;

        // Apply friction-based decay when grounded
        if (CurrentMovementMode == MOVE_Walking) {
            float FrictionDecay = 1.0f - (MovementConfig.GetGroundFrictionWalking() * 0.01f * DeltaTime);
            PhysicsInputVector *= FrictionDecay;
        }

        // Clear small values
        if (PhysicsInputVector.Size() < 0.01f) {
            PhysicsInputVector = FVector2D::ZeroVector;
        }
    }
}

void UOHMovementComponent::ApplyPhysicsImpulse(const FVector& WorldImpulse, bool bIsCombatHit) {
    if (!bAcceptPhysicsInput) {
        return;
    }

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character) {
        return;
    }

    // Ensure we're initialized
    if (!bMovementInitialized) {
        EnsureInitialized();
    }

    // For combat hits, we want to override any existing gentle pushes
    if (bIsCombatHit) {
        // If existing impulse is small (likely proximity), clear it
        if (AccumulatedPhysicsImpulse.Size() < 300.0f) {
            AccumulatedPhysicsImpulse = FVector::ZeroVector;
        }
    }

    // Add to accumulated impulse
    AccumulatedPhysicsImpulse += WorldImpulse;

    // Different caps for different impulse types
    float MaxImpulse = bIsCombatHit ? (PhysicsImpulseStrength * 3000.0f) : (PhysicsImpulseStrength * 500.0f);

    if (AccumulatedPhysicsImpulse.Size() > MaxImpulse) {
        AccumulatedPhysicsImpulse = AccumulatedPhysicsImpulse.GetClampedToMaxSize(MaxImpulse);
    }

    // Store combat information
    if (bIsCombatHit) {
        LastCombatPushback = WorldImpulse;
        LastCombatPushbackTime = GetWorld()->GetTimeSeconds();
        bCombatPushbackActive = true;
        bForceImpulseBasedPhysics = true;

        // Apply immediate velocity feedback for combat
        if (Character->GetCharacterMovement()) {
            UCharacterMovementComponent* CharMove = Character->GetCharacterMovement();

            // Calculate immediate velocity based on impulse magnitude
            float ImpulseMagnitude = WorldImpulse.Size();
            float ImmediateVelocity = FMath::GetMappedRangeValueClamped(FVector2D(100.0f, 2000.0f),
                                                                        FVector2D(50.0f, 400.0f), ImpulseMagnitude);

            FVector VelocityBoost = WorldImpulse.GetSafeNormal() * ImmediateVelocity;
            CharMove->Velocity += VelocityBoost;

            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("Combat velocity boost: %s"), *VelocityBoost.ToString());
            }
        }
    }

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("ApplyPhysicsImpulse: %s, Combat=%s, Total=%s"), *WorldImpulse.ToString(),
               bIsCombatHit ? TEXT("Yes") : TEXT("No"), *AccumulatedPhysicsImpulse.ToString());
    }
}

FVector2D UOHMovementComponent::GetEffectiveMovementInput() const {
    if (!bAcceptPhysicsInput)
        return MovementInputVector;

    // During combat pushback, reduce player input influence
    FVector2D PlayerInput = MovementInputVector;

    if (bCombatPushbackActive) {
        float TimeSinceCombat = GetWorld()->GetTimeSeconds() - LastCombatPushbackTime;

        // First 0.3 seconds: heavily reduce player control
        if (TimeSinceCombat < 0.3f) {
            float ControlReduction =
                FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 0.3f), FVector2D(0.1f, 0.5f), // 10% to 50% control
                                                  TimeSinceCombat);
            PlayerInput *= ControlReduction;
        }
    }

    // Add physics input (already weighted internally)
    FVector2D EffectiveInput = PlayerInput + PhysicsInputVector;

    // Clamp to valid input range
    if (EffectiveInput.Size() > 1.0f) {
        // Preserve direction but clamp magnitude
        EffectiveInput = EffectiveInput.GetSafeNormal();
    }

    return EffectiveInput;
}

void UOHMovementComponent::UpdateProximitySpacing(float DeltaTime) {
    if (!bEnableProximitySpacing)
        return;

    // === PERFORMANCE OPTIMIZATION: STAGGERED UPDATES ===
    ProximityUpdateTimer += DeltaTime;
    if (ProximityUpdateTimer < ProximityUpdateInterval)
        return;
    ProximityUpdateTimer = 0.0f;

    // Skip proximity during active combat pushback to prevent interference
    if (bCombatPushbackActive) {
        return;
    }

    ACharacter* MyCharacter = Cast<ACharacter>(GetOwner());
    if (!MyCharacter)
        return;

    UCharacterMovementComponent* MyCharMove = MyCharacter->GetCharacterMovement();
    if (!MyCharMove)
        return;

    // === LEVERAGE OHCOMBATUTILS FOR OPPONENT DETECTION ===
    AActor* OpponentActor = UOHCombatUtils::FindMostLikelyOpponent(MyCharacter);
    ACharacter* NearestOpponent = Cast<ACharacter>(OpponentActor);

    if (!NearestOpponent)
        return;

    // === SPATIAL ANALYSIS USING UTILITY FUNCTIONS ===
    FVector MyLocation = MyCharacter->GetActorLocation();
    FVector OpponentLocation = NearestOpponent->GetActorLocation();
    float Distance = FVector::Dist2D(MyLocation, OpponentLocation);

    // Early exit for distant opponents
    if (Distance > PersonalSpaceRadius * 2.5f)
        return;

    // === UNIFIED COMBAT STATE ANALYSIS INTEGRATION ===
    FOHCombatAnalysis MyCombatState = UOHPACManager::AnalyzeCharacterCombatState(MyCharacter);
    FOHCombatAnalysis OpponentCombatState = UOHPACManager::AnalyzeCharacterCombatState(NearestOpponent);

    // Determine if either character is in active combat using unified analysis
    bool bEitherInActiveCombat = (MyCombatState.bIsAttacking && MyCombatState.TotalKineticEnergy > 1000.0f) ||
                                 (OpponentCombatState.bIsAttacking && OpponentCombatState.TotalKineticEnergy > 1000.0f);

    // Let combat impulses handle separation during high-intensity combat
    if (bEitherInActiveCombat) {
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning,
                   TEXT("Proximity: Deferring to combat impulses - My Energy: %.1f, Opponent Energy: %.1f"),
                   MyCombatState.TotalKineticEnergy, OpponentCombatState.TotalKineticEnergy);
        }
        return;
    }

    // === DYNAMIC SPACING CALCULATION USING COMBAT STATE ===
    float BaseSpacing = BaseSpacingDistance;

    // Adjust spacing based on combat readiness using utility functions
    if (MyCombatState.AttackConfidence > 0.3f || OpponentCombatState.AttackConfidence > 0.3f) {
        // Increase spacing if either character shows attack intent
        BaseSpacing *=
            FMath::Lerp(1.0f, 1.4f, FMath::Max(MyCombatState.AttackConfidence, OpponentCombatState.AttackConfidence));
    }

    // Apply proximity spacing only when encroaching personal space
    if (Distance < BaseSpacing) {
        // === SEPARATION VECTOR CALCULATION WITH DIRECTIONAL ANALYSIS ===
        FVector SeparationDirection = (MyLocation - OpponentLocation).GetSafeNormal();

        // Validate separation direction using OHCombatUtils
        if (SeparationDirection.IsNearlyZero()) {
            // Fallback: Use OHCombatUtils mirroring technique for consistent direction
            FVector MyForward = MyCharacter->GetActorForwardVector();
            SeparationDirection = UOHCombatUtils::MirrorVectorOverAxis(MyForward, FVector::UpVector);
            SeparationDirection.Z = 0.0f;
            SeparationDirection.Normalize();
        }

        // === PENETRATION DEPTH ANALYSIS ===
        float PenetrationDepth = BaseSpacing - Distance;
        float PenetrationRatio = FMath::Clamp(PenetrationDepth / BaseSpacing, 0.0f, 1.0f);

        // Calculate base push strength using curve integration
        float PushStrength = ProximityPushStrength * PenetrationRatio;

        if (SpacingCurve) {
            float CurveMultiplier = SpacingCurve->GetFloatValue(PenetrationRatio);
            if (FMath::IsFinite(CurveMultiplier) && CurveMultiplier > 0.0f) {
                PushStrength *= CurveMultiplier;
            }
        }

        // === COMBAT STATE MODULATION ===
        // Reduce proximity push during combat preparation to avoid interference
        if (MyCombatState.AttackConfidence > 0.1f) {
            float CombatReduction = FMath::Lerp(1.0f, 0.3f, MyCombatState.AttackConfidence);
            PushStrength *= CombatReduction;
        }

        // === LEVERAGE OHLOCOMOTIONUTILS FOR DIRECTIONAL BLENDING ===
        FVector CurrentVelocity = MyCharMove->Velocity;
        FVector DesiredSeparation = SeparationDirection * PushStrength * 250.0f;

        // Use OHLocomotionUtils for sophisticated direction blending
        FVector BlendedDirection = UOHLocomotionUtils::BlendDirectionalInput(CurrentVelocity.GetSafeNormal(),
                                                                             DesiredSeparation.GetSafeNormal(),
                                                                             0.3f // Responsive but not jarring
        );

        // === APPLY PHYSICS IMPULSE WITH COMBAT STATE AWARENESS ===
        FVector ProximityImpulse = BlendedDirection * PushStrength * 200.0f;

        // Apply gentle damping using OHCombatUtils for smooth movement
        if (!CurrentVelocity.IsNearlyZero()) {
            FVector DampedVelocity = UOHCombatUtils::ApplyLinearDrag(CurrentVelocity, 0.1f, DeltaTime);
            float DampingFactor = (DampedVelocity.Size() / FMath::Max(CurrentVelocity.Size(), 1.0f));
            ProximityImpulse *= (0.7f + DampingFactor * 0.3f);
        }

        // Route through unified physics impulse system (non-combat)
        ApplyPhysicsImpulse(ProximityImpulse, false);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning,
                   TEXT("Proximity Spacing: Dist=%.1f/%.1f, Penetration=%.1f%%, Push=%s, MyCombat=%.2f, "
                        "OpponentCombat=%.2f"),
                   Distance, BaseSpacing, PenetrationRatio * 100.0f, *ProximityImpulse.ToString(),
                   MyCombatState.AttackConfidence, OpponentCombatState.AttackConfidence);
        }
    }
}
void UOHMovementComponent::CalculateSmartSpacing(ACharacter* Opponent, float& OutPushX, float& OutPushY) {
    if (!Opponent || !GetOwner()) {
        OutPushX = OutPushY = 0.0f;
        return;
    }

    ACharacter* MyCharacter = Cast<ACharacter>(GetOwner());
    if (!MyCharacter) {
        OutPushX = OutPushY = 0.0f;
        return;
    }

    // Get positions
    FVector MyPos = MyCharacter->GetActorLocation();
    FVector OpponentPos = Opponent->GetActorLocation();
    FVector ToOpponent = OpponentPos - MyPos;
    float Distance = ToOpponent.Size2D();

    if (Distance < KINDA_SMALL_NUMBER) {
        OutPushX = OutPushY = 0.0f;
        return;
    }

    ToOpponent.Normalize();

    // Check combat states using chain data
    float MyAttackConfidence = 0.0f;
    float MyChainSpeed = 0.0f;
    float OpponentAttackConfidence = 0.0f;
    float OpponentChainSpeed = 0.0f;

    bool bImAttacking = IsCharacterInCombat(MyCharacter, MyAttackConfidence, MyChainSpeed);
    bool bOpponentAttacking = IsCharacterInCombat(Opponent, OpponentAttackConfidence, OpponentChainSpeed);

    // Dynamic spacing based on combat state
    float DesiredDistance = BaseSpacingDistance;

    if (bImAttacking && !bOpponentAttacking) {
        // I'm attacking - maintain closer distance
        DesiredDistance *= 0.8f;
    } else if (!bImAttacking && bOpponentAttacking) {
        // Opponent attacking - increase distance
        DesiredDistance *= 1.3f;
    } else if (bImAttacking && bOpponentAttacking) {
        // Both attacking - moderate distance
        DesiredDistance *= 1.0f;
    }

    // Calculate push based on distance violation
    float DistanceError = DesiredDistance - Distance;

    if (FMath::Abs(DistanceError) > 5.0f) // Dead zone of 5 units
    {
        // Base push direction
        FVector PushDirection;
        float PushMagnitude = 0.0f;

        if (DistanceError > 0) // Too close
        {
            PushDirection = -ToOpponent; // Push away
            PushMagnitude = DistanceError * ProximityPushStrength;
        } else // Too far
        {
            PushDirection = ToOpponent;                                               // Pull closer
            PushMagnitude = FMath::Abs(DistanceError) * ProximityPushStrength * 0.3f; // Reduced pull strength
        }

        // Apply curve if available
        if (SpacingCurve) {
            float NormalizedDistance = Distance / PersonalSpaceRadius;
            PushMagnitude *= SpacingCurve->GetFloatValue(NormalizedDistance);
        }

        // Reduce push during active combat to avoid interference
        if (bImAttacking || bOpponentAttacking) {
            PushMagnitude *= 0.3f;
        }

        // Add lateral component to avoid straight-line movement
        FVector RightVector = FVector::CrossProduct(ToOpponent, FVector::UpVector);
        RightVector.Normalize();

        // Bias lateral movement based on attack confidence
        if (bOpponentAttacking && OpponentAttackConfidence > 0.7f) {
            // High confidence opponent attack - add dodge component
            float DodgeDirection = (FMath::FRand() > 0.5f) ? 1.0f : -1.0f;
            PushDirection += RightVector * 0.5f * DodgeDirection;
            PushDirection.Normalize();
        }

        // Convert to character-relative coordinates
        FVector LocalPush = MyCharacter->GetActorTransform().InverseTransformVector(PushDirection);

        OutPushX = LocalPush.X * PushMagnitude;
        OutPushY = LocalPush.Y * PushMagnitude;

        // Debug logging
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("Smart Spacing: Distance=%.1f, Desired=%.1f, Error=%.1f, Push=(%.3f,%.3f)"),
                   Distance, DesiredDistance, DistanceError, OutPushX, OutPushY);
        }
    } else {
        OutPushX = OutPushY = 0.0f;
    }
}

void UOHMovementComponent::ProcessPhysicsImpulses(float DeltaTime) {
    if (!bAcceptPhysicsInput || AccumulatedPhysicsImpulse.IsNearlyZero()) {
        return;
    }

    ACharacter* Character = Cast<ACharacter>(GetOwner());
    if (!Character || !Character->GetCharacterMovement()) {
        AccumulatedPhysicsImpulse = FVector::ZeroVector;
        return;
    }

    UCharacterMovementComponent* CharMove = Character->GetCharacterMovement();

    // === ENHANCED IMPULSE DIRECTION VALIDATION ===
    FVector ImpulseDirection = AccumulatedPhysicsImpulse.GetSafeNormal();
    float ImpulseMagnitude = AccumulatedPhysicsImpulse.Size();

    // Validate impulse direction is meaningful
    if (ImpulseDirection.IsNearlyZero() || !ImpulseDirection.ContainsNaN()) {
        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("Invalid impulse direction detected, clearing: %s"),
                   *AccumulatedPhysicsImpulse.ToString());
        }
        AccumulatedPhysicsImpulse = FVector::ZeroVector;
        return;
    }

    // === CRITICAL FIX: ENSURE HORIZONTAL MOVEMENT ===
    // Ensure impulse primarily affects horizontal movement
    ImpulseDirection.Z = FMath::Clamp(ImpulseDirection.Z, -0.2f, 0.2f);
    ImpulseDirection.Normalize();

    // === COMBAT STATE ANALYSIS ===
    float TimeSinceCombat = GetWorld()->GetTimeSeconds() - LastCombatPushbackTime;
    bool bIsCombatImpact = bCombatPushbackActive && TimeSinceCombat < 1.5f;

    // === RELIABLE MOVEMENT APPLICATION ===
    if (bIsCombatImpact && ImpulseMagnitude > 200.0f) {
        // === DIRECT MOVEMENT FOR STRONG COMBAT IMPACTS ===
        float MovementSpeed = FMath::GetMappedRangeValueClamped(
            FVector2D(200.0f, 2000.0f), FVector2D(CharMove->MaxWalkSpeed * 0.6f, CharMove->MaxWalkSpeed * 1.8f),
            ImpulseMagnitude);

        // Apply time-based decay for natural recovery
        float RecoveryFactor =
            FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 0.8f), FVector2D(1.0f, 0.2f), TimeSinceCombat);
        MovementSpeed *= RecoveryFactor;

        // === CRITICAL: DIRECT MOVEMENT INPUT APPLICATION ===
        if (MovementSpeed > 50.0f) {
            float InputScale = MovementSpeed / FMath::Max(CharMove->MaxWalkSpeed, 1.0f);
            InputScale = FMath::Clamp(InputScale, 0.0f, 2.0f);

            // Direct movement application - this moves the character
            Character->AddMovementInput(ImpulseDirection, InputScale);

            if (bVerboseLogging) {
                UE_LOG(LogTemp, Warning, TEXT("Applied direct movement: Dir=%s, Scale=%.2f, Speed=%.1f"),
                       *ImpulseDirection.ToString(), InputScale, MovementSpeed);
            }
        }

        // === ENHANCED BLENDED INPUT FOR SMOOTH TRANSITIONS ===
        FVector LocalImpulse = Character->GetActorTransform().InverseTransformVector(AccumulatedPhysicsImpulse);
        float BlendInputScale =
            FMath::GetMappedRangeValueClamped(FVector2D(0.3f, 0.8f), FVector2D(0.6f, 0.1f), TimeSinceCombat);

        PhysicsInputVector = FVector2D(LocalImpulse.X, LocalImpulse.Y).GetSafeNormal() * BlendInputScale;

        // Faster decay for combat
        float DecayRate = PhysicsImpulseDecayRate * 2.0f;
        float DecayFactor = FMath::Exp(-DecayRate * DeltaTime);
        AccumulatedPhysicsImpulse *= DecayFactor;
    } else {
        // === GENTLE PROXIMITY PUSHES ===
        FVector LocalImpulse = Character->GetActorTransform().InverseTransformVector(AccumulatedPhysicsImpulse);
        float ProximityInputScale =
            FMath::GetMappedRangeValueClamped(FVector2D(10.0f, 300.0f), FVector2D(0.05f, 0.3f), ImpulseMagnitude);

        PhysicsInputVector = FVector2D(LocalImpulse.X, LocalImpulse.Y).GetSafeNormal() * ProximityInputScale;

        // Slower decay
        float DecayRate = PhysicsImpulseDecayRate * 0.5f;
        float DecayFactor = FMath::Exp(-DecayRate * DeltaTime);
        AccumulatedPhysicsImpulse *= DecayFactor;
    }

    // === TERMINATION CONDITIONS ===
    if (AccumulatedPhysicsImpulse.Size() < 30.0f) {
        AccumulatedPhysicsImpulse = FVector::ZeroVector;
        bCombatPushbackActive = false;
        bForceImpulseBasedPhysics = false;
        PhysicsInputVector = FVector2D::ZeroVector;

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Log, TEXT("Physics impulse processing complete"));
        }
    }
}

#pragma endregion

void UOHMovementComponent::EnsureInitialized() {
    if (bMovementInitialized) {
        return;
    }

    // Perform initialization
    bIsPlayerControlled = GetIsPlayerControlled();

    // Connect to PAC Manager if available
    if (UOHPACManager* PACManager = GetOwner()->FindComponentByClass<UOHPACManager>()) {
        PACManager->OnPushbackApplied.AddDynamic(this, &UOHMovementComponent::HandlePACPushback);

        if (bVerboseLogging) {
            UE_LOG(LogTemp, Warning, TEXT("MovementComponent connected to PAC pushback events"));
        }
    }

    // Initialize movement config
    MovementConfig.Update();

    // Set initialized flag
    bMovementInitialized = true;

    if (bVerboseLogging) {
        UE_LOG(LogTemp, Warning, TEXT("MovementComponent initialized successfully"));
    }
}

bool UOHMovementComponent::IsCharacterInCombat(ACharacter* Character, float& OutAttackConfidence,
                                               float& OutChainSpeed) {
    // === MIGRATION TO UNIFIED COMBAT ANALYSIS ===
    // This function is now a wrapper around the enhanced analysis system

    OutAttackConfidence = 0.0f;
    OutChainSpeed = 0.0f;

    if (!IsValid(Character)) {
        return false;
    }

    // Leverage unified combat analysis from Phase 1
    FOHCombatAnalysis CombatAnalysis = UOHPACManager::AnalyzeCharacterCombatState(Character);

    // Extract legacy interface values from enhanced analysis
    OutAttackConfidence = CombatAnalysis.AttackConfidence;
    OutChainSpeed = CombatAnalysis.MaxChainSpeed;

    // Enhanced combat detection logic using unified system
    bool bInCombat = CombatAnalysis.bIsAttacking && CombatAnalysis.AttackConfidence > 0.3f &&
                     CombatAnalysis.MaxChainSpeed > 200.0f; // Configurable threshold

    if (bVerboseLogging && bInCombat) {
        UE_LOG(LogTemp, Warning, TEXT("Combat Detected: %s - Confidence=%.2f, Speed=%.1f, Energy=%.1f, Quality=%.2f"),
               *Character->GetName(), CombatAnalysis.AttackConfidence, CombatAnalysis.MaxChainSpeed,
               CombatAnalysis.TotalKineticEnergy, CombatAnalysis.ChainMotionQuality);
    }

    return bInCombat;
}
