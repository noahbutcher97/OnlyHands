#include "Component/OHMovementComponent.h"
#include "Controller/OHPlayerController.h"
#include "Curves/CurveVector.h"
#include "FunctionLibrary/OHCombatUtils.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/Vector2D.h"

UOHMovementComponent::UOHMovementComponent() {
  PrimaryComponentTick.bCanEverTick = false;
  DirectionalSpeedCurve = nullptr;
  RotationalSpeedCurve = nullptr;
  MovementCurve = nullptr;
  InputBufferDecay = 5.f;
  InputBufferRiseRate = 10.f;
  RunHysteresis = 30.f;                // Reduced from 50
  SprintHysteresis = 30.f;             // Reduced from 50
  HoldTimeAboveRunThreshold = 0.15f;   // Reduced from 0.2
  HoldTimeAboveSprintThreshold = 0.2f; // Reduced from 0.3
  GaitInterpSpeed = 8.f; // Increased from 5 for snappier transitions
  InputIntentMagnitudeThreshold =
      0.25f; // Reduced from 0.3 for easier triggering
  PivotThreshold = 45.f;

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

  bDebugMovement = false;
  MovementInputVector = FVector2D::ZeroVector;
}

void UOHMovementComponent::BeginPlay() {
  Super::BeginPlay();
  UE_LOG(LogTemp, Warning, TEXT("MC: BeginPlay called for %s"),
         *GetNameSafe(this));

  bIsPlayerControlled = GetIsPlayerControlled();

  /*if (bIsPlayerControlled)
  {
          if (AOHPlayerController* PC = Cast<AOHPlayerController>(Controller))
          {
                  PC->OnMoveForwardInput.AddDynamic(this,
  &UOHMovementComponent::OnReceiveMoveForward);
                  PC->OnMoveRightInput.AddDynamic(this,
  &UOHMovementComponent::OnReceiveMoveRight);
                  PC->OnMoveInputVector.AddDynamic(this,
  &UOHMovementComponent::OnReceiveMoveVector); UE_LOG(LogTemp, Warning,
  TEXT("MC: Delegates bound to PlayerController!"));

          }
  }*/
}

void UOHMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);
}

// 5. Enhanced debug tick
void UOHMovementComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction) {
  if (!bMovementInitialized || !GetOwner() || !GetIsPlayerControlled()) {
    return;
  }

  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
  UpdateInputBuffers(DeltaTime);
  UpdateMovement(DeltaTime);
  ApplyMovementInput();

  if (bDebugMovement && GEngine) {
    // Main status
    GEngine->AddOnScreenDebugMessage(
        -1, 0.f, FColor::White,
        FString::Printf(TEXT("=== MOVEMENT DEBUG ===")));

    // Input status
    GEngine->AddOnScreenDebugMessage(
        -1, 0.f, FColor::Yellow,
        FString::Printf(
            TEXT("Input: Vec(%.2f,%.2f) Mag:%.2f Buffered:%.2f Intent:%s"),
            MovementInputVector.X, MovementInputVector.Y, InputIntentMagnitude,
            BufferedInputAmount,
            bLastInputIntentValid ? TEXT("VALID") : TEXT("INVALID")));

    // Speed status
    GEngine->AddOnScreenDebugMessage(
        -1, 0.f, FColor::Cyan,
        FString::Printf(TEXT("Speed: Current:%.0f Buffered:%.0f"),
                        LastCalculatedSpeed, BufferedSpeed));

    // Gait status
    const TCHAR *GaitNames[] = {TEXT("Walking"), TEXT("Running"),
                                TEXT("Sprinting")};
    GEngine->AddOnScreenDebugMessage(
        -1, 0.f, FColor::Green,
        FString::Printf(
            TEXT("Gait: %s (Alpha:%.2f) RunTimer:%.2f SprintTimer:%.2f"),
            GaitNames[static_cast<int32>(CurrentGait)], GaitBlendAlpha,
            BufferedTimeAboveRunThreshold, BufferedTimeAboveSprintThreshold));

    // Character movement component status
    if (ACharacter *Owner = Cast<ACharacter>(GetOwner())) {
      if (UCharacterMovementComponent *MoveComp =
              Owner->GetCharacterMovement()) {
        GEngine->AddOnScreenDebugMessage(
            -1, 0.f, FColor::Magenta,
            FString::Printf(TEXT("CharMove: MaxSpeed:%.0f ActualVel:%.0f"),
                            MoveComp->MaxWalkSpeed, MoveComp->Velocity.Size()));
      }
    }
  }
}

void UOHMovementComponent::FindTargetActor() {
  if (AActor *Opponent = UOHCombatUtils::FindMostLikelyOpponent(
          GetOwner(), TEXT("john") /*optional*/)) {
    SetTargetActor(Opponent);
    GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::Red,
                                     TEXT("Found target actor!"));
  }
}

void UOHMovementComponent::SetTargetActor(AActor *NewTarget) {
  if (NewTarget) {
    TargetActor = NewTarget;
  }
}

// Enhanced movement application with weight
void UOHMovementComponent::ApplyMovementInput() {
  FVector MoveVector = CalculateMovementInputVector();

  if (MoveVector.SizeSquared() > 1.f) {
    MoveVector = MoveVector.GetSafeNormal();
  }

  if (!MoveVector.IsNearlyZero()) {
    if (ACharacter *OwnerCharacter = Cast<ACharacter>(GetOwner())) {
      // Apply movement with weight consideration
      float InputScale = 1.0f;

      // Reduce input strength based on weight when changing direction
      if (!LastMovementDirection.IsNearlyZero()) {
        float DirChange =
            FVector::DotProduct(MoveVector, LastMovementDirection);
        if (DirChange < 0.5f) // Significant direction change
        {
          InputScale = FMath::Lerp(1.0f, 0.7f, MovementWeight);
        }
      }

      OwnerCharacter->AddMovementInput(MoveVector, InputScale, false);
    }
  }

#if WITH_EDITOR
  if (bDebugMovement) {
    if (AActor *Owner = GetOwner()) {
      FVector Start = Owner->GetActorLocation();

      // Draw movement vector
      FVector End = Start + MoveVector * 200.f * BufferedInputAmount;
      DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, -1, 0, 3.0f);

      // Draw facing direction
      FVector FacingEnd = Start + Owner->GetActorForwardVector() * 150.f;
      DrawDebugLine(GetWorld(), Start, FacingEnd, FColor::Blue, false, -1, 0,
                    2.0f);

      // Draw momentum direction
      if (!LastMovementDirection.IsNearlyZero()) {
        FVector MomentumEnd =
            Start + LastMovementDirection * 100.f *
                        (CurrentMomentumSpeed / SprintMaxSpeed);
        DrawDebugLine(GetWorld(), Start, MomentumEnd, FColor::Yellow, false, -1,
                      0, 2.0f);
      }
    }
  }
#endif
}

FVector UOHMovementComponent::CalculateMovementInputVector() const {
  // 1. Read current input vector (from InputPanel2D or controller)
  // Assume MovementInputVector (FVector2D) is up-to-date from input events
  FVector2D LocalInput = MovementInputVector;

  // 2. Clamp input vector to prevent diagonal speed boost
  if (LocalInput.SizeSquared() > 1.0f) {
    LocalInput.Normalize();
  }

  // 3. Get the control rotation (camera yaw only)
  FRotator ControlRotation =
      CalculateControlRotation(); // Your custom function!

  // 4. Convert input into world space using camera yaw
  // Unreal's forward: X, right: Y
  const FVector Forward = FRotationMatrix(ControlRotation)
                              .GetUnitAxis(EAxis::X); // Forward from camera yaw
  const FVector Right = FRotationMatrix(ControlRotation)
                            .GetUnitAxis(EAxis::Y); // Right from camera yaw

  // 5. Compose world-space movement direction
  FVector WorldMoveVector = (Forward * LocalInput.X) + (Right * LocalInput.Y);

  // 6. Zero out Z to prevent unintended vertical movement
  WorldMoveVector.Z = 0.f;

  // 7. Normalize if necessary (important for consistent movement speed)
  if (WorldMoveVector.SizeSquared() > 1.f) {
    WorldMoveVector.Normalize();
  }

  return WorldMoveVector;
}

FRotator UOHMovementComponent::CalculateControlRotation() const {
  FVector CameraLocation;
  FRotator CameraRotation;
  // Pass in the owning actor for best context
  if (UOHCombatUtils::GetPlayerCameraView(GetOwner(), CameraLocation,
                                          CameraRotation)) {
    // Only yaw for movement
    return FRotator(0.f, CameraRotation.Yaw, 0.f);
  }
  // Fallback to actor's rotation
  if (AActor *Owner = GetOwner()) {
    return FRotator(0.f, Owner->GetActorRotation().Yaw, 0.f);
  }
  return FRotator::ZeroRotator;
}

void UOHMovementComponent::UpdateMovement(float DeltaTime) {

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

  // 5. Calculate animation play rate based on speed and input (optional
  // fallback)
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
    BufferedInputAmount =
        FMath::FInterpTo(BufferedInputAmount, CurrentInputAmount, DeltaTime,
                         InputBufferRiseRate);
  } else {
    BufferedInputAmount = FMath::FInterpTo(
        BufferedInputAmount, CurrentInputAmount, DeltaTime, InputBufferDecay);
  }

  // Smooth buffered speed (always rise rate for smoothing)
  BufferedSpeed = FMath::FInterpTo(BufferedSpeed, CurrentSpeed, DeltaTime,
                                   InputBufferRiseRate);

  // Update run hold timer with hysteresis
  if (BufferedSpeed > RunSpeedThreshold + RunHysteresis) {
    BufferedTimeAboveRunThreshold += DeltaTime;
  } else if (BufferedSpeed < RunSpeedThreshold - RunHysteresis) {
    BufferedTimeAboveRunThreshold = 0.f;
  }

  // Update sprint hold timer with hysteresis
  if (BufferedSpeed > SprintSpeedThreshold + SprintHysteresis) {
    BufferedTimeAboveSprintThreshold += DeltaTime;
  } else if (BufferedSpeed < SprintSpeedThreshold - SprintHysteresis) {
    BufferedTimeAboveSprintThreshold = 0.f;
  }
}

// Override UpdateDynamicMovementSettings for momentum-based approach
void UOHMovementComponent::UpdateDynamicMovementSettings(EOHGait AllowedGait) {
  ACharacter *OwnerChar = Cast<ACharacter>(GetOwner());
  if (!OwnerChar)
    return;

  UCharacterMovementComponent *MoveComp = OwnerChar->GetCharacterMovement();
  if (!MoveComp)
    return;

  // Determine base target speed for current gait
  float BaseTargetSpeed = WalkMaxSpeed;

  switch (AllowedGait) {
  case EOHGait::Walking:
    BaseTargetSpeed = WalkMaxSpeed;
    break;
  case EOHGait::Running:
    BaseTargetSpeed = RunMaxSpeed;
    break;
  case EOHGait::Sprinting:
    BaseTargetSpeed = SprintMaxSpeed;
    break;
  }

  // === MOMENTUM-BASED SPEED SYSTEM ===
  if (bUseMomentumBasedMovement) {
    float DeltaTime = GetWorld()->GetDeltaSeconds();

    // 1. Check if we have input
    bool bHasInput = BufferedInputAmount > JoystickDeadzone;

    // 2. Build or decay momentum speed based on input
    if (bHasInput) {
      // Build speed when input detected
      CurrentMomentumSpeed =
          FMath::FInterpTo(CurrentMomentumSpeed, BaseTargetSpeed, DeltaTime,
                           InputSpeedBuildRate);
    } else {
      // Decay speed when no input
      CurrentMomentumSpeed = FMath::FInterpTo(CurrentMomentumSpeed, 0.f,
                                              DeltaTime, NoInputSpeedDecayRate);
    }

    // 3. Calculate directional modifiers
    CalculateDirectionalSpeedModifiers(OwnerChar);

    // 4. Apply all modifiers to get final speed
    float FinalSpeed = CurrentMomentumSpeed;
    FinalSpeed *= DirectionalSpeedMultiplier;
    FinalSpeed *= RotationalSpeedMultiplier;

    // 5. Apply input magnitude scaling (for analog feel)
    if (bHasInput && bUseInputMagnitudeForSpeed) {
      float InputScale = FMath::GetMappedRangeValueClamped(
          FVector2D(JoystickDeadzone, 1.0f), FVector2D(0.3f, 1.0f),
          BufferedInputAmount);
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
  if (MovementCurve) {
    float MappedSpeed = GetMappedSpeed();
    CurveValues = MovementCurve->GetVectorValue(MappedSpeed);
  }

  float BaseAccel = CurveValues.X > 0.f ? CurveValues.X : 2048.f;
  float BaseBraking = CurveValues.Y > 0.f ? CurveValues.Y : 2048.f;
  float BaseFriction = CurveValues.Z > 0.f ? CurveValues.Z : 8.f;

  // Modify physics based on movement weight
  float WeightMultiplier =
      FMath::Lerp(1.5f, 0.5f, MovementWeight); // Heavy = slower accel
  float FrictionMultiplier =
      FMath::Lerp(0.7f, 1.3f, MovementWeight); // Heavy = more friction

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

  // Debug output
  if (bDebugMovement && GEngine) {
    GEngine->AddOnScreenDebugMessage(
        -1, 0.0f, FColor::Green,
        FString::Printf(
            TEXT("Momentum: %.0f | Dir: %.2fx | Rot: %.2fx | Angle: %.0f°"),
            CurrentMomentumSpeed, DirectionalSpeedMultiplier,
            RotationalSpeedMultiplier, CurrentMovementAngle));
  }
}

// New function to calculate directional speed modifiers
void UOHMovementComponent::CalculateDirectionalSpeedModifiers(
    ACharacter *OwnerChar) {
  if (!OwnerChar)
    return;

  FVector CharacterForward = OwnerChar->GetActorForwardVector();
  FVector MovementDirection = CalculateMovementInputVector();

  if (!MovementDirection.IsNearlyZero()) {
    MovementDirection.Normalize();

    // Calculate angle between facing and movement (0-180)
    float DotProduct = FVector::DotProduct(CharacterForward, MovementDirection);
    CurrentMovementAngle = FMath::RadiansToDegrees(
        FMath::Acos(FMath::Clamp(DotProduct, -1.f, 1.f)));

    // Base directional multiplier
    if (CurrentMovementAngle < 45.f) {
      // Forward movement
      DirectionalSpeedMultiplier = 1.0f;
    } else if (CurrentMovementAngle > 135.f) {
      // Backward movement
      DirectionalSpeedMultiplier = BackpedalSpeedMultiplier;
    } else if (FMath::Abs(CurrentMovementAngle - 90.f) < 20.f) {
      // Pure strafe (70-110 degrees)
      DirectionalSpeedMultiplier = StrafeSpeedMultiplier;
    } else {
      // Diagonal movement
      DirectionalSpeedMultiplier = DiagonalSpeedMultiplier;
    }

    // Apply custom curve if available
    if (DirectionalSpeedCurve) {
      DirectionalSpeedMultiplier *=
          DirectionalSpeedCurve->GetFloatValue(CurrentMovementAngle);
    }

    // === DIRECTION CHANGE PENALTY ===
    if (!LastMovementDirection.IsNearlyZero()) {
      float DirectionDot =
          FVector::DotProduct(MovementDirection, LastMovementDirection);
      float DirectionChangeAngle = FMath::RadiansToDegrees(
          FMath::Acos(FMath::Clamp(DirectionDot, -1.f, 1.f)));

      // Apply speed retention based on how sharp the direction change is
      if (DirectionChangeAngle > 90.f) {
        float ChangeMultiplier = FMath::GetMappedRangeValueClamped(
            FVector2D(90.f, 180.f),
            FVector2D(1.0f, SpeedRetentionOnDirectionChange),
            DirectionChangeAngle);
        DirectionalSpeedMultiplier *= ChangeMultiplier;
      }
    }

    // Smooth the direction change
    LastMovementDirection = FMath::VInterpTo(
        LastMovementDirection, MovementDirection, GetWorld()->GetDeltaSeconds(),
        DirectionChangeSharpness);
  } else {
    // No input - maintain last direction for momentum
    DirectionalSpeedMultiplier = 1.0f;
  }

  // === ROTATIONAL SPEED MODIFIER ===

  // Calculate angular velocity
  static FRotator LastRotation = OwnerChar->GetActorRotation();
  FRotator CurrentRotation = OwnerChar->GetActorRotation();
  float DeltaYaw =
      FMath::FindDeltaAngleDegrees(LastRotation.Yaw, CurrentRotation.Yaw);
  CurrentAngularVelocity = FMath::Abs(DeltaYaw) / GetWorld()->GetDeltaSeconds();
  LastRotation = CurrentRotation;

  // Base rotation penalty
  if (CurrentAngularVelocity > 90.f) // Turning faster than 90 deg/sec
  {
    RotationalSpeedMultiplier = FMath::GetMappedRangeValueClamped(
        FVector2D(90.f, 360.f), FVector2D(1.0f, 0.7f), CurrentAngularVelocity);
  } else {
    RotationalSpeedMultiplier = 1.0f;
  }

  // Apply custom curve if available
  if (RotationalSpeedCurve) {
    RotationalSpeedMultiplier *=
        RotationalSpeedCurve->GetFloatValue(CurrentAngularVelocity);
  }
}

FOHMovementSettings UOHMovementComponent::GetTargetMovementSettings() const {
  FOHMovementSettings Settings;

  // Use your thresholds or any logic here
  Settings.WalkSpeed = WalkMaxSpeed;
  Settings.RunSpeed = RunMaxSpeed;
  Settings.SprintSpeed = SprintMaxSpeed;

  Settings.MaxAcceleration = 2048.f; // default fallback
  Settings.BrakingDecelerationWalking = 2048.f;
  Settings.GroundFriction = 8.f;

  Settings.MovementCurve = MovementCurve;

  return Settings;
}

float UOHMovementComponent::GetMappedSpeed() const {
  // Returns normalized speed between 0 and 1 for curve evaluation

  float Speed = CalculateMovementSpeed();

  // Clamp between WalkSpeedThreshold and SprintSpeedThreshold
  float MinSpeed = WalkSpeedThreshold;
  float MaxSpeed = SprintSpeedThreshold;

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
  if (bDebugMovement) {
    if (GEngine) {
      GEngine->AddOnScreenDebugMessage(
          static_cast<uint64>((PTRINT)this) + 11111, 0.05f, FColor::Yellow,
          FString::Printf(TEXT("MC: OnReceiveMoveForward = %.2f"),
                          ForwardValue));
    }
  }

#endif
}

void UOHMovementComponent::OnReceiveMoveRight(float RightValue) {
  CurrentRightInput = RightValue;
  MovementInputVector.Y = RightValue;
  InputIntentMagnitude = MovementInputVector.Size();
  OnMovementInputUpdated.Broadcast(MovementInputVector);
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT

  if (bDebugMovement) {
    if (GEngine) {
      GEngine->AddOnScreenDebugMessage(
          static_cast<uint64>((PTRINT)this) + 22222, 0.05f, FColor::Orange,
          FString::Printf(TEXT("MC: OnReceiveMoveRight = %.2f"), RightValue));
    }
  }

#endif
}

static FVector2D ClampVector2D(const FVector2D &Vec, float MaxSize) {
  if (Vec.SizeSquared() > MaxSize * MaxSize) {
    return Vec.GetSafeNormal() * MaxSize;
  }
  return Vec;
}

void UOHMovementComponent::OnReceiveMoveVector(float ForwardValue,
                                               float RightValue) {
  CurrentForwardInput = ForwardValue;
  CurrentRightInput = RightValue;
  MovementInputVector = ClampVector2D(FVector2D(ForwardValue, RightValue), 1.f);
  InputIntentMagnitude = MovementInputVector.Size();
  OnMovementInputUpdated.Broadcast(MovementInputVector);
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
  if (bDebugMovement) {
    if (GEngine) {
      GEngine->AddOnScreenDebugMessage(
          static_cast<uint64>((PTRINT)this) + 33333, 0.05f, FColor::Red,
          FString::Printf(TEXT("MC: OnReceiveMoveVector F=%.2f R=%.2f"),
                          ForwardValue, RightValue));
    }
  }

#endif

  // --- Debug: Track Last Received Input ---
  LastReceivedInputVector = MovementInputVector;
  LastInputDebugTimestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f;
}

bool UOHMovementComponent::GetIsPlayerControlled() const {
  bool bIsPlayer = false;

  if (ACharacter *OwnerCharacter = Cast<ACharacter>(GetOwner())) {
    if (AController *Controller = OwnerCharacter->GetController()) {
      bIsPlayer = Controller->IsPlayerController();
    }
  }
  return bIsPlayer;
}

void UOHMovementComponent::SetMovementInputVector(const FVector2D &NewInput) {
  MovementInputVector = NewInput;

  // Manually clamp magnitude
  if (MovementInputVector.SizeSquared() > 1.f) {
    MovementInputVector.Normalize();
  }

  OnMovementInputUpdated.Broadcast(MovementInputVector);

  CurrentForwardInput = MovementInputVector.X;
  CurrentRightInput = MovementInputVector.Y;
#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
  if (bDebugMovement) {
    if (GEngine) {
      GEngine->AddOnScreenDebugMessage(
          static_cast<uint64>((PTRINT)this) + 33333, 0.05f, FColor::Green,
          FString::Printf(TEXT("MC: SetMovementInputVector F=%.2f R=%.2f"),
                          CurrentForwardInput, CurrentRightInput));
    }
  }

#endif
}

FRotator UOHMovementComponent::CalculateRotationToTarget_Internal() const {
  APawn *OwnerPawn = Cast<APawn>(GetOwner());
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
  // Optional: Returns the angle between the forward vector and the input vector
  // in degrees
  if (ACharacter *OwnerChar = Cast<ACharacter>(GetOwner())) {
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
  APawn *OwnerPawn = Cast<APawn>(GetOwner());
  if (!OwnerPawn)
    return FRotator::ZeroRotator;

  FRotator CurrentRotation = OwnerPawn->GetActorRotation();
  FRotator TargetRotation = CalculateRotationToTarget_Internal();

  FRotator DeltaRotation = UKismetMathLibrary::NormalizedDeltaRotator(
      TargetRotation, CurrentRotation);

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
  if (const ACharacter *OwnerChar = Cast<ACharacter>(GetOwner())) {
    FVector Velocity = OwnerChar->GetVelocity();
    Velocity.Z = 0.f;
    return Velocity.Size();
  }
  return 0.f;
}

float UOHMovementComponent::CalculateMovementPlayRate(float Speed,
                                                      float InputAmount) const {
  if (InputAmount < KINDA_SMALL_NUMBER) {
    return 1.f;
  }

  float MinSpeed = WalkSpeedThreshold;
  float MaxSpeed = SprintSpeedThreshold;

  return FMath::GetMappedRangeValueClamped(FVector2D(MinSpeed, MaxSpeed),
                                           FVector2D(0.5f, 1.5f), Speed);
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
  if (IntentMagnitude < JoystickDeadzone) {
    IntentMagnitude = 0.f;
  }

  // Smooth the input with different rates for rise/fall
  if (IntentMagnitude > BufferedInputAmount) {
    BufferedInputAmount = FMath::FInterpTo(BufferedInputAmount, IntentMagnitude,
                                           DeltaTime, InputBufferRiseRate);
  } else {
    BufferedInputAmount = FMath::FInterpTo(BufferedInputAmount, IntentMagnitude,
                                           DeltaTime, InputBufferDecay);
  }

  // Smooth speed always
  BufferedSpeed =
      FMath::FInterpTo(BufferedSpeed, Speed, DeltaTime, InputBufferRiseRate);

  // Virtual joystick intent-based gait selection
  EOHGait IntentGait = EOHGait::Walking;

  if (BufferedInputAmount > JoystickRunZone) {
    IntentGait = EOHGait::Sprinting;
  } else if (BufferedInputAmount > JoystickWalkZone) {
    IntentGait = EOHGait::Running;
  } else if (BufferedInputAmount > JoystickDeadzone) {
    IntentGait = EOHGait::Walking;
  }

  // Now check if we have enough speed to support the intent
  bool bSpeedSupportsIntent = true;

  if (IntentGait == EOHGait::Sprinting && BufferedSpeed < RunSpeedThreshold) {
    // Not fast enough for sprint yet
    bSpeedSupportsIntent = false;
    IntentGait = EOHGait::Running;
  }

  if (IntentGait == EOHGait::Running && BufferedSpeed < WalkSpeedThreshold) {
    // Not fast enough for run yet
    bSpeedSupportsIntent = false;
    IntentGait = EOHGait::Walking;
  }

  // Update timers based on intent and speed
  if (IntentGait == EOHGait::Sprinting && BufferedSpeed > RunSpeedThreshold) {
    BufferedTimeAboveSprintThreshold += DeltaTime;
    BufferedTimeAboveRunThreshold += DeltaTime; // Also accumulate run time
  } else if (IntentGait == EOHGait::Running &&
             BufferedSpeed > WalkSpeedThreshold) {
    BufferedTimeAboveRunThreshold += DeltaTime;
    BufferedTimeAboveSprintThreshold = 0.f;
  } else {
    BufferedTimeAboveRunThreshold = 0.f;
    BufferedTimeAboveSprintThreshold = 0.f;
  }

  // Enhanced debug for virtual joystick
  if (bDebugMovement && GEngine) {
    GEngine->AddOnScreenDebugMessage(
        -1, 0.0f, FColor::Cyan,
        FString::Printf(
            TEXT("VJoy: Raw:%.2f Buff:%.2f Intent:%s Speed:%.0f SpeedOK:%s"),
            InputAmount, BufferedInputAmount,
            IntentGait == EOHGait::Sprinting ? TEXT("SPRINT")
            : IntentGait == EOHGait::Running ? TEXT("RUN")
                                             : TEXT("WALK"),
            BufferedSpeed, bSpeedSupportsIntent ? TEXT("YES") : TEXT("NO")));
  }

  // Determine final gait with timing requirements
  if (IntentGait == EOHGait::Sprinting &&
      BufferedTimeAboveSprintThreshold >= HoldTimeAboveSprintThreshold) {
    return EOHGait::Sprinting;
  } else if (IntentGait >= EOHGait::Running &&
             BufferedTimeAboveRunThreshold >= HoldTimeAboveRunThreshold) {
    return EOHGait::Running;
  } else {
    return EOHGait::Walking;
  }
}

EOHGait UOHMovementComponent::GetAllowedGait() const {
  if (const ACharacter *OwnerChar = Cast<ACharacter>(GetOwner())) {
    // Disallow sprint if crouched or in air
    if (OwnerChar->bIsCrouched || CurrentMovementMode == MOVE_Falling) {
      return EOHGait::Walking;
    }

    // Get a forward vector of character (XY plane)
    const FVector Forward =
        OwnerChar->GetActorForwardVector().GetSafeNormal2D();
    // Get movement input vector (already clamped to max size 1)
    const FVector2D MovementInput2D = MovementInputVector;
    if (MovementInput2D.IsNearlyZero()) {
      return EOHGait::Walking; // No input means any sprint
    }

    // Convert input 2D to 3D vector on XY plane (assume X=Forward, Y=Right)
    FVector InputDir =
        FVector(MovementInput2D.X, MovementInput2D.Y, 0.f).GetSafeNormal();

    // Calculate angles between the forward and input direction in degrees
    float InputAngleDegrees = FMath::RadiansToDegrees(
        FMath::Acos(FVector::DotProduct(Forward, InputDir)));

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

EOHGait UOHMovementComponent::GetActualGait(EOHGait DesiredGait,
                                            EOHGait AllowedGait) {
  // Clamp the actual gait so it doesn't exceed the allowed gait
  return (DesiredGait <= AllowedGait) ? DesiredGait : AllowedGait;
}

void UOHMovementComponent::InterpolateGait(EOHGait DesiredGait,
                                           float DeltaTime) {
  if (CurrentGait != DesiredGait) {
    // Fade out the old gait by interpolating GaitBlendAlpha towards 0
    GaitBlendAlpha =
        FMath::FInterpTo(GaitBlendAlpha, 0.f, DeltaTime, GaitInterpSpeed);

    if (GaitBlendAlpha <= KINDA_SMALL_NUMBER) {
      // Switch gait and reset alpha to 1
      CurrentGait = DesiredGait;
      GaitBlendAlpha = 1.f;
      OnGaitChanged.Broadcast(CurrentGait);
    }
  } else {
    // When gait matches desired, fade alpha back to 1
    GaitBlendAlpha =
        FMath::FInterpTo(GaitBlendAlpha, 1.f, DeltaTime, GaitInterpSpeed);
  }
}

void UOHMovementComponent::HandleMovementModeChanged(
    EMovementMode NewMovementMode, uint8 NewCustomMode) {
  CurrentMovementMode = NewMovementMode;
  CurrentCustomMovementMode = NewCustomMode;
}

bool UOHMovementComponent::ShouldPivot_Internal() const {
  FRotator TargetOffset = CalculateTargetOffset_Internal();
  return FMath::Abs(TargetOffset.Yaw) > PivotThreshold;
}

// TODO: Implement according to your target locking system
AActor *UOHMovementComponent::GetLockedTargetActor() {
  // For example, if your character has a target pointer, you can do:
  // if (const ACharacter* OwnerChar = Cast<ACharacter>(GetOwner()))
  //     return OwnerChar->GetLockedTarget();

  // For now return nullptr; You need to replace this with your actual locking
  // logic
  return nullptr;
}
