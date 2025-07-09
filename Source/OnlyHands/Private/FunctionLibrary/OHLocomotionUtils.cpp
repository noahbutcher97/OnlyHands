// Returns the character's movement angle relative to the screen/camera
// direction.
#include "FunctionLibrary/OHLocomotionUtils.h"

#include "Component/OHPhysicsManager.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "OHPhysicsStructs.h"

void UOHLocomotionUtils::ApplyJoystickMovementRelativeToActor(
    ACharacter *Character, const FVector2D &Input,
    const FRotator &ActorRotation, float Deadzone) {
  if (!Character || Input.SizeSquared() < Deadzone * Deadzone)
    return;

  FVector Forward = FRotationMatrix(ActorRotation).GetUnitAxis(EAxis::X);
  FVector Right = FRotationMatrix(ActorRotation).GetUnitAxis(EAxis::Y);

  Forward.Z = 0.f;
  Right.Z = 0.f;
  Forward.Normalize();
  Right.Normalize();

  FVector MoveDirection = Forward * Input.Y + Right * Input.X;
  Character->AddMovementInput(MoveDirection);
}

void UOHLocomotionUtils::GetLockOnMovementBasis(ACharacter *Character,
                                                AActor *LockOnTarget,
                                                FVector &ForwardVector,
                                                FVector &RightVector) {
  ForwardVector = FVector::ZeroVector;
  RightVector = FVector::ZeroVector;

  if (!Character || !LockOnTarget) {
    return;
  }

  // Calculate planar direction to target
  FVector ToTarget =
      LockOnTarget->GetActorLocation() - Character->GetActorLocation();
  ToTarget.Z = 0.0f;

  if (ToTarget.IsNearlyZero()) {
    return;
  }

  ToTarget.Normalize();

  // Rotation facing the target
  const FRotator FacingRotation = ToTarget.Rotation();

  // Basis vectors in world space
  ForwardVector = FRotationMatrix(FacingRotation).GetUnitAxis(EAxis::X);
  RightVector = FRotationMatrix(FacingRotation).GetUnitAxis(EAxis::Y);
}

float UOHLocomotionUtils::ComputeSmoothedQuadrantMovementAngle(
    const FVector &Velocity, const FRotator &FacingRotation,
    float PreviousAngle, float DeltaTime, float PredictionWeight,
    float BiasStrength) {
  if (Velocity.IsNearlyZero()) {
    return PreviousAngle;
  }

  // 1. Transform velocity to character local space
  FRotationMatrix FacingMatrix(FacingRotation);
  const FVector LocalVelocity =
      FacingMatrix.GetTransposed().TransformVector(Velocity);

  // 2. Get current input angle relative to character
  float RawAngle =
      FMath::Atan2(LocalVelocity.Y, LocalVelocity.X); // Y = right, X = forward
  float CurrentAngle = FMath::RadiansToDegrees(RawAngle);
  CurrentAngle =
      FMath::Fmod(CurrentAngle + 360.0f, 360.0f); // Normalize to [0, 360)

  // 3. Predict future angle delta
  const float SmoothedDelta =
      FMath::UnwindDegrees(CurrentAngle - PreviousAngle);
  const float PredictedAngle =
      CurrentAngle + (SmoothedDelta * PredictionWeight);

  // 4. Bias toward previous angle to reduce quadrant jitter
  const float FinalAngle =
      FMath::Lerp(PreviousAngle, PredictedAngle, 1.0f - BiasStrength);

  return FMath::Fmod(FinalAngle + 360.0f, 360.0f); // Ensure output is [0, 360)
}

// Static cache for smoothed angle per character
static TMap<ACharacter *, float> SmoothedAngleCache;
FPredictedMotionDirectionData
UOHLocomotionUtils::ComputeSmartAnimationDirectionAuto(
    ACharacter *Character, float DeltaTime, float StrafeBias,
    float PredictionTime, float SmoothingStrength,
    float PelvisTranslationThreshold) {
  FPredictedMotionDirectionData Result;
  if (!Character)
    return Result;

  const FRotator FacingRotation = Character->GetActorRotation();
  float &PreviousAngle = SmoothedAngleCache.FindOrAdd(Character);

  // === Adaptive smoothing under low FPS ===
  if (DeltaTime > 0.05f) // ~20 FPS or lower
  {
    PredictionTime *= 0.75f;
    SmoothingStrength = FMath::Clamp(SmoothingStrength * 1.2f, 0.f, 1.f);
  }

  // === Auto-detect movement mode and adjust dynamics ===
  if (const UCharacterMovementComponent *MoveComp =
          Character->GetCharacterMovement()) {
    switch (MoveComp->MovementMode) {
    case MOVE_Falling:
      PredictionTime *= 1.2f;
      SmoothingStrength *= 0.9f;
      break;
    case MOVE_Swimming:
    case MOVE_Custom:
      PredictionTime *= 0.5f;
      SmoothingStrength *= 1.2f;
      break;
    default:
      break;
    }
  }

  // === Get motion data from physics manager if available ===
  UOHPhysicsManager *PhysicsManager =
      Character->FindComponentByClass<UOHPhysicsManager>();

  FVector MoveDirWorld = FVector::ZeroVector;
  FVector FinalVel = FVector::ZeroVector;
  FVector FinalAccel = FVector::ZeroVector;

  if (PhysicsManager) {
    const FVector RootVel = PhysicsManager->GetVelocity(FName("root"));
    const FVector PelvisVel = PhysicsManager->GetVelocity(FName("pelvis"));

    const float RootMag = RootVel.Size();
    const float PelvisMag = PelvisVel.Size();

    const float BlendAlpha = FMath::Clamp(
        PelvisMag / (RootMag * PelvisTranslationThreshold + KINDA_SMALL_NUMBER),
        0.f, 1.f);

    FinalVel = FMath::Lerp(RootVel, PelvisVel, BlendAlpha);
    FinalAccel = FMath::Lerp(
        PhysicsManager->GetLinearAcceleration(FName("root")),
        PhysicsManager->GetLinearAcceleration(FName("pelvis")), BlendAlpha);

    const FVector PredictDelta =
        FinalVel * PredictionTime +
        0.5f * FinalAccel * FMath::Square(PredictionTime);
    MoveDirWorld = PredictDelta.GetSafeNormal();
  }

  // === Fallback: use character's velocity directly ===
  if (MoveDirWorld.IsNearlyZero()) {
    FinalVel = Character->GetVelocity();
    FinalAccel = FVector::ZeroVector;

    // Foot-lock heuristic: high accel, low speed → stay stable
    if (FinalVel.Size() < 10.f && FinalAccel.Size() > 150.f) {
      return Result;
    }

    if (!FinalVel.IsNearlyZero()) {
      MoveDirWorld = FinalVel.GetSafeNormal();
    } else {
      return Result; // idle
    }
  }

  // === Convert to local space ===
  const FVector LocalDir = FRotationMatrix(FacingRotation)
                               .GetTransposed()
                               .TransformVector(MoveDirWorld);
  float AngleRad = FMath::Atan2(LocalDir.Y, LocalDir.X);
  float AngleDeg =
      FMath::Fmod(FMath::RadiansToDegrees(AngleRad) + 360.f, 360.f);

  // === Pivot anticipation via curve arc detection ===
  const float ArcStrength = FVector::CrossProduct(FinalVel.GetSafeNormal(),
                                                  FinalAccel.GetSafeNormal())
                                .Z;
  if (FMath::Abs(ArcStrength) > 0.1f) {
    const float CurveOffset = FMath::Clamp(ArcStrength, -1.f, 1.f) * 20.f;
    AngleDeg += CurveOffset;
  }

  // === Apply directional strafe bias ===
  const float BiasTarget = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) / 2.f);
  AngleDeg = FMath::Lerp(AngleDeg, BiasTarget, FMath::Abs(StrafeBias));

  // === Smooth result ===
  const float Smoothed =
      FMath::Lerp(PreviousAngle, AngleDeg, 1.0f - SmoothingStrength);
  PreviousAngle = FMath::Fmod(Smoothed + 360.f, 360.f);
  Result.SmoothedAngle = PreviousAngle;

  // === Enrich output ===
  const float Radians = FMath::DegreesToRadians(Result.SmoothedAngle);
  Result.BlendFactors = FVector2D(FMath::Sin(Radians), FMath::Cos(Radians));
  Result.bMirror = (Result.BlendFactors.X < 0.f);

  // --- 8-Way direction
  if (PreviousAngle < 22.5f || PreviousAngle >= 337.5f)
    Result.Direction8 = EScreenDirection8::Forward;
  else if (PreviousAngle < 67.5f)
    Result.Direction8 = EScreenDirection8::ForwardRight;
  else if (PreviousAngle < 112.5f)
    Result.Direction8 = EScreenDirection8::Right;
  else if (PreviousAngle < 157.5f)
    Result.Direction8 = EScreenDirection8::BackwardRight;
  else if (PreviousAngle < 202.5f)
    Result.Direction8 = EScreenDirection8::Backward;
  else if (PreviousAngle < 247.5f)
    Result.Direction8 = EScreenDirection8::BackwardLeft;
  else if (PreviousAngle < 292.5f)
    Result.Direction8 = EScreenDirection8::Left;
  else
    Result.Direction8 = EScreenDirection8::ForwardLeft;

  // --- 4-Way direction
  if (PreviousAngle < 45.f || PreviousAngle >= 315.f)
    Result.Direction4 = EScreenDirection4::Forward;
  else if (PreviousAngle < 135.f)
    Result.Direction4 = EScreenDirection4::Right;
  else if (PreviousAngle < 225.f)
    Result.Direction4 = EScreenDirection4::Backward;
  else
    Result.Direction4 = EScreenDirection4::Left;

  // --- Quadrant
  if (PreviousAngle < 90.f)
    Result.Quadrant = EScreenDirectionQuadrant::ForwardRight;
  else if (PreviousAngle < 180.f)
    Result.Quadrant = EScreenDirectionQuadrant::BackwardRight;
  else if (PreviousAngle < 270.f)
    Result.Quadrant = EScreenDirectionQuadrant::BackwardLeft;
  else
    Result.Quadrant = EScreenDirectionQuadrant::ForwardLeft;

  return Result;
}

void UOHLocomotionUtils::ApplyStableCameraRelativeMovement(
    ACharacter *Character, const FVector2D &Input) {
  if (!Character || Input.SizeSquared() <= KINDA_SMALL_NUMBER)
    return;

  static FRotator LastStableRotation;
  static FVector2D LastInputDirection;

  // Only update rotation when the input vector direction changes significantly
  float AngleDelta = FMath::Acos(FVector2D::DotProduct(
      Input.GetSafeNormal(), LastInputDirection.GetSafeNormal()));
  bool bInputChanged =
      AngleDelta > FMath::DegreesToRadians(10.f); // ~10 degrees change

  if (bInputChanged) {
    if (AController *Controller = Character->GetController()) {
      LastStableRotation = Controller->GetControlRotation();
      LastInputDirection = Input;
    }
  }

  FVector Forward = FRotationMatrix(LastStableRotation).GetUnitAxis(EAxis::X);
  FVector Right = FRotationMatrix(LastStableRotation).GetUnitAxis(EAxis::Y);

  Forward.Z = 0.f;
  Right.Z = 0.f;
  Forward.Normalize();
  Right.Normalize();

  FVector MoveDirection = Forward * Input.Y + Right * Input.X;
  Character->AddMovementInput(MoveDirection);
}

void UOHLocomotionUtils::ApplyJoystickMovementWithBasis(
    APawn *Pawn, const FVector2D &Input, const FRotator &BasisRotation,
    FVector UpVector, float Deadzone) {
  if (!Pawn || Input.SizeSquared() < Deadzone * Deadzone)
    return;

  // Derive forward/right from basis, flatten using custom up vector
  FVector Forward = FRotationMatrix(BasisRotation).GetScaledAxis(EAxis::X);
  FVector Right = FRotationMatrix(BasisRotation).GetScaledAxis(EAxis::Y);

  // Remove vertical component using up vector
  Forward = FVector::VectorPlaneProject(Forward, UpVector).GetSafeNormal();
  Right = FVector::VectorPlaneProject(Right, UpVector).GetSafeNormal();

  FVector MoveDir = Forward * Input.Y + Right * Input.X;
  MoveDir.Normalize();

  Pawn->AddMovementInput(MoveDir);
}

void UOHLocomotionUtils::ApplyAutoCameraRelativeJoystickMovement(
    ACharacter *Character, const FVector2D &Input, float Deadzone) {
  if (!Character || Input.SizeSquared() < Deadzone * Deadzone)
    return;

  // Try to use control rotation
  FRotator MovementBasis = FRotator::ZeroRotator;

  if (AController *Controller = Character->GetController()) {
    MovementBasis = Controller->GetControlRotation();
  } else {
    // Fallback to actor rotation
    MovementBasis = Character->GetActorRotation();
  }

  // Flatten pitch/roll to keep movement on XY plane
  MovementBasis.Pitch = 0.f;
  MovementBasis.Roll = 0.f;

  FVector Forward = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::X);
  FVector Right = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::Y);

  Forward.Normalize();
  Right.Normalize();

  // Auto-detect dominant input axis to resolve input convention
  FVector MoveDir;
  if (FMath::Abs(Input.Y) >= FMath::Abs(Input.X)) {
    // Assume Y = forward
    MoveDir = Forward * Input.Y + Right * Input.X;
  } else {
    // Assume X = forward
    MoveDir = Forward * Input.X + Right * Input.Y;
  }

  Character->AddMovementInput(MoveDir);
}

void UOHLocomotionUtils::ApplySmartCameraRelativeJoystickMovement(
    ACharacter *Character, const FVector2D &Input, bool bFaceMovementDirection,
    bool bConstrainToXYPlane, float Deadzone) {
  if (!Character || Input.SizeSquared() < Deadzone * Deadzone)
    return;

  // Get movement basis
  FRotator MovementBasis = FRotator::ZeroRotator;
  if (AController *Controller = Character->GetController()) {
    MovementBasis = Controller->GetControlRotation();
  } else {
    MovementBasis = Character->GetActorRotation();
  }

  MovementBasis.Pitch = 0.f;
  MovementBasis.Roll = 0.f;

  FVector Forward = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::X);
  FVector Right = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::Y);

  Forward.Normalize();
  Right.Normalize();

  // Auto-resolve axis layout
  FVector MoveDir;
  if (FMath::Abs(Input.Y) >= FMath::Abs(Input.X)) {
    MoveDir = Forward * Input.Y + Right * Input.X;
  } else {
    MoveDir = Forward * Input.X + Right * Input.Y;
  }

  if (bConstrainToXYPlane) {
    MoveDir.Z = 0.f;
  }

  MoveDir.Normalize();

  Character->AddMovementInput(MoveDir);

  // Optional: rotate to face movement direction
  if (bFaceMovementDirection && MoveDir.SizeSquared() > KINDA_SMALL_NUMBER) {
    FRotator TargetRotation = MoveDir.Rotation();
    TargetRotation.Pitch = 0.f;
    TargetRotation.Roll = 0.f;

    Character->SetActorRotation(TargetRotation);
  }
}

void UOHLocomotionUtils::ApplyFullCameraRelativeJoystickMovement(
    ACharacter *Character, const FVector2D &Input, bool bFaceMovementDirection,
    bool bConstrainToXYPlane, float Deadzone, float TurnSpeed,
    int32 SnapToCardinalDirections) {
  if (!Character || Input.SizeSquared() < Deadzone * Deadzone)
    return;

  // Get movement basis
  FRotator MovementBasis = FRotator::ZeroRotator;
  if (AController *Controller = Character->GetController()) {
    MovementBasis = Controller->GetControlRotation();
  } else {
    MovementBasis = Character->GetActorRotation();
  }

  MovementBasis.Pitch = 0.f;
  MovementBasis.Roll = 0.f;

  FVector Forward = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::X);
  FVector Right = FRotationMatrix(MovementBasis).GetUnitAxis(EAxis::Y);

  Forward.Normalize();
  Right.Normalize();

  // Auto-resolve axis layout
  FVector MoveDir;
  if (FMath::Abs(Input.Y) >= FMath::Abs(Input.X)) {
    MoveDir = Forward * Input.Y + Right * Input.X;
  } else {
    MoveDir = Forward * Input.X + Right * Input.Y;
  }

  if (bConstrainToXYPlane) {
    MoveDir.Z = 0.f;
  }

  MoveDir.Normalize();
  Character->AddMovementInput(MoveDir);

  // Handle rotation
  if (bFaceMovementDirection && MoveDir.SizeSquared() > KINDA_SMALL_NUMBER) {
    FRotator TargetRotation = MoveDir.Rotation();

    // Snap to cardinal directions if requested
    if (SnapToCardinalDirections == 4 || SnapToCardinalDirections == 8) {
      float Step = 360.f / SnapToCardinalDirections;
      float Yaw = TargetRotation.Yaw;
      Yaw = FMath::RoundToFloat(Yaw / Step) * Step;
      TargetRotation = FRotator(0.f, Yaw, 0.f);
    }

    TargetRotation.Pitch = 0.f;
    TargetRotation.Roll = 0.f;

    FRotator NewRotation =
        FMath::RInterpTo(Character->GetActorRotation(), TargetRotation,
                         Character->GetWorld()->GetDeltaSeconds(), TurnSpeed);

    Character->SetActorRotation(NewRotation);
  }
}
// ===== Screen-Relative =====

FScreenRelativeDirectionData UOHLocomotionUtils::ComputeDirectionFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController,
    bool bNormalize) {
  if (!PlayerController || InputVector.IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector2D CamForward(CamRot.Vector().X, CamRot.Vector().Y);

  FScreenRelativeDirectionData Result;
  Result.Angle =
      ComputeScreenRelativeAngle(CamForward, InputVector, bNormalize);
  Result.WrappedAngle = FMath::Fmod(Result.Angle + 360.f, 360.f);
  Result.BlendFactors = ComputeBlendFactors(InputVector, CamRot, bNormalize);
  Result.Direction = MapAngleTo8WayDirection_Inline(Result.Angle);
  Result.Direction4 = MapAngleTo4WayDirection_Inline(Result.Angle);
  Result.Quadrant = MapAngleToQuadrant_Inline(Result.Angle);
  Result.ClampedStrafeDirection =
      FMath::Clamp(Result.BlendFactors.X, -1.f, 1.f);
  return Result;
}

FScreenRelativeDirectionData UOHLocomotionUtils::ComputeDirectionFromVelocity(
    const APlayerController *PlayerController, const ACharacter *Character,
    bool bNormalize) {
  if (!PlayerController || !Character ||
      Character->GetVelocity().IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FVector2D Velocity2D(Character->GetVelocity().X,
                             Character->GetVelocity().Y);
  return ComputeDirectionFromInput(Velocity2D, PlayerController, bNormalize);
}

FScreenRelativeDirectionData UOHLocomotionUtils::ComputeDirectionToTarget(
    const AActor *TargetActor, const APlayerController *PlayerController,
    const ACharacter *Character, bool bNormalize) {
  if (!PlayerController || !TargetActor || !Character)
    return FScreenRelativeDirectionData::Default();

  const FVector2D ToTarget =
      FVector2D(TargetActor->GetActorLocation() - Character->GetActorLocation())
          .GetSafeNormal();
  return ComputeDirectionFromInput(ToTarget, PlayerController, bNormalize);
}

// ===== Player-Relative =====

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputePlayerRelativeDirectionFromInput(
    const FVector2D &InputVector, const ACharacter *Character,
    bool bNormalize) {
  if (!Character || InputVector.IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FVector2D PlayerForward(Character->GetActorForwardVector().X,
                                Character->GetActorForwardVector().Y);

  FScreenRelativeDirectionData Result;
  Result.Angle =
      ComputeScreenRelativeAngle(PlayerForward, InputVector, bNormalize);
  Result.WrappedAngle = FMath::Fmod(Result.Angle + 360.f, 360.f);
  Result.BlendFactors = ComputeBlendFactors(
      InputVector, Character->GetActorRotation(), bNormalize);
  Result.Direction = MapAngleTo8WayDirection_Inline(Result.Angle);
  Result.Direction4 = MapAngleTo4WayDirection_Inline(Result.Angle);
  Result.Quadrant = MapAngleToQuadrant_Inline(Result.Angle);
  Result.ClampedStrafeDirection =
      FMath::Clamp(Result.BlendFactors.X, -1.f, 1.f);
  return Result;
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputePlayerRelativeDirectionFromVelocity(
    const ACharacter *Character, bool bNormalize) {
  if (!Character || Character->GetVelocity().IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FVector2D Velocity2D(Character->GetVelocity().X,
                             Character->GetVelocity().Y);
  return ComputePlayerRelativeDirectionFromInput(Velocity2D, Character,
                                                 bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputePlayerRelativeDirectionToTarget(
    const AActor *TargetActor, const ACharacter *Character, bool bNormalize) {
  if (!TargetActor || !Character)
    return FScreenRelativeDirectionData::Default();

  const FVector2D ToTarget =
      FVector2D(TargetActor->GetActorLocation() - Character->GetActorLocation())
          .GetSafeNormal();
  return ComputePlayerRelativeDirectionFromInput(ToTarget, Character,
                                                 bNormalize);
}

// ===== World-Relative =====

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeWorldRelativeDirectionFromInput(
    const FVector2D &InputVector, bool bNormalize) {
  if (InputVector.IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FVector2D WorldForward(1, 0); // +X axis

  FScreenRelativeDirectionData Result;
  Result.Angle =
      ComputeScreenRelativeAngle(WorldForward, InputVector, bNormalize);
  Result.WrappedAngle = FMath::Fmod(Result.Angle + 360.f, 360.f);
  Result.BlendFactors = bNormalize ? InputVector.GetSafeNormal() : InputVector;
  Result.Direction = MapAngleTo8WayDirection_Inline(Result.Angle);
  Result.Direction4 = MapAngleTo4WayDirection_Inline(Result.Angle);
  Result.Quadrant = MapAngleToQuadrant_Inline(Result.Angle);
  Result.ClampedStrafeDirection =
      FMath::Clamp(Result.BlendFactors.X, -1.f, 1.f);
  return Result;
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeWorldRelativeDirectionFromVelocity(
    const ACharacter *Character, bool bNormalize) {
  if (!Character || Character->GetVelocity().IsNearlyZero())
    return FScreenRelativeDirectionData::Default();

  const FVector2D Velocity2D(Character->GetVelocity().X,
                             Character->GetVelocity().Y);
  return ComputeWorldRelativeDirectionFromInput(Velocity2D, bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeWorldRelativeDirectionToTarget(
    const AActor *TargetActor, const ACharacter *Character, bool bNormalize) {
  if (!TargetActor || !Character)
    return FScreenRelativeDirectionData::Default();

  const FVector2D ToTarget =
      FVector2D(TargetActor->GetActorLocation() - Character->GetActorLocation())
          .GetSafeNormal();
  return ComputeWorldRelativeDirectionFromInput(ToTarget, bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeActorRelativeDirectionFromInput(
    const FVector2D &InputVector, const AActor *ReferenceActor,
    bool bNormalize) {
  if (!ReferenceActor || InputVector.IsNearlyZero()) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D Forward(ReferenceActor->GetActorForwardVector().X,
                          ReferenceActor->GetActorForwardVector().Y);
  const FRotator RefRot = ReferenceActor->GetActorRotation();

  FScreenRelativeDirectionData Result;
  Result.Angle = ComputeScreenRelativeAngle(Forward, InputVector, bNormalize);
  Result.WrappedAngle = FMath::Fmod(Result.Angle + 360.f, 360.f);
  Result.BlendFactors =
      ComputeBlendFactors_Inline(InputVector, RefRot, bNormalize);
  Result.Direction = MapAngleTo8WayDirection_Inline(Result.Angle);
  Result.Direction4 = MapAngleTo4WayDirection_Inline(Result.Angle);
  Result.Quadrant = MapAngleToQuadrant_Inline(Result.Angle);
  Result.ClampedStrafeDirection =
      FMath::Clamp(Result.BlendFactors.X, -1.f, 1.f);
  return Result;
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeActorRelativeDirectionFromVelocity(
    const ACharacter *SourceCharacter, const AActor *ReferenceActor,
    bool bNormalize) {
  if (!SourceCharacter || !ReferenceActor ||
      SourceCharacter->GetVelocity().IsNearlyZero()) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D Velocity2D(SourceCharacter->GetVelocity().X,
                             SourceCharacter->GetVelocity().Y);
  return ComputeActorRelativeDirectionFromInput(Velocity2D, ReferenceActor,
                                                bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeActorRelativeDirectionToTarget(
    const AActor *TargetActor, const AActor *ReferenceActor, bool bNormalize) {
  if (!TargetActor || !ReferenceActor) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D ToTarget = FVector2D(TargetActor->GetActorLocation() -
                                       ReferenceActor->GetActorLocation())
                                 .GetSafeNormal();
  return ComputeActorRelativeDirectionFromInput(ToTarget, ReferenceActor,
                                                bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeComponentRelativeDirectionFromInput(
    const FVector2D &InputVector, const USceneComponent *ReferenceComponent,
    bool bNormalize) {
  if (!ReferenceComponent || InputVector.IsNearlyZero()) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D Forward(ReferenceComponent->GetForwardVector().X,
                          ReferenceComponent->GetForwardVector().Y);
  const FRotator RefRot = ReferenceComponent->GetComponentRotation();

  FScreenRelativeDirectionData Result;
  Result.Angle = ComputeScreenRelativeAngle(Forward, InputVector, bNormalize);
  Result.WrappedAngle = FMath::Fmod(Result.Angle + 360.f, 360.f);
  Result.BlendFactors =
      ComputeBlendFactors_Inline(InputVector, RefRot, bNormalize);
  Result.Direction = MapAngleTo8WayDirection_Inline(Result.Angle);
  Result.Direction4 = MapAngleTo4WayDirection_Inline(Result.Angle);
  Result.Quadrant = MapAngleToQuadrant_Inline(Result.Angle);
  Result.ClampedStrafeDirection =
      FMath::Clamp(Result.BlendFactors.X, -1.f, 1.f);
  return Result;
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeComponentRelativeDirectionFromVelocity(
    const ACharacter *SourceCharacter,
    const USceneComponent *ReferenceComponent, bool bNormalize) {
  if (!SourceCharacter || !ReferenceComponent ||
      SourceCharacter->GetVelocity().IsNearlyZero()) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D Velocity2D(SourceCharacter->GetVelocity().X,
                             SourceCharacter->GetVelocity().Y);
  return ComputeComponentRelativeDirectionFromInput(
      Velocity2D, ReferenceComponent, bNormalize);
}

FScreenRelativeDirectionData
UOHLocomotionUtils::ComputeComponentRelativeDirectionToTarget(
    const AActor *TargetActor, const USceneComponent *ReferenceComponent,
    bool bNormalize) {
  if (!TargetActor || !ReferenceComponent) {
    return FScreenRelativeDirectionData::Default();
  }

  const FVector2D ToTarget =
      FVector2D(TargetActor->GetActorLocation() -
                ReferenceComponent->GetComponentLocation())
          .GetSafeNormal();
  return ComputeComponentRelativeDirectionFromInput(
      ToTarget, ReferenceComponent, bNormalize);
}

// ===========================================================================================
float UOHLocomotionUtils::ComputeScreenRelativeDirection(
    const FVector2D &InputVector, const APlayerController *PlayerController,
    const ACharacter *Character, bool bClampToCardinal) {
  if (!PlayerController || !Character || InputVector.IsNearlyZero()) {
    return 0.0f;
  }

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector CamForward = CamRot.Vector();
  const FVector CamRight = FRotationMatrix(CamRot).GetUnitAxis(EAxis::Y);

  const FVector2D Forward2D(CamForward.X, CamForward.Y);
  const FVector2D Right2D(CamRight.X, CamRight.Y);
  const FVector2D InputDir = InputVector.GetSafeNormal();

  const float ForwardDot =
      FVector2D::DotProduct(InputDir, Forward2D.GetSafeNormal());
  const float RightDot =
      FVector2D::DotProduct(InputDir, Right2D.GetSafeNormal());

  float Direction = FMath::Clamp(RightDot, -1.0f, 1.0f);

  if (bClampToCardinal) {
    if (FMath::Abs(Direction) > FMath::Abs(ForwardDot)) {
      Direction = FMath::Sign(Direction);
    } else {
      Direction = 0.0f;
    }
  }

  return Direction;
}

FVector2D UOHLocomotionUtils::GetScreenRelativeVector(
    const FVector2D &InputVector, const APlayerController *PlayerController) {
  if (!PlayerController || InputVector.IsNearlyZero()) {
    return FVector2D::ZeroVector;
  }

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector CamForward = CamRot.Vector();
  const FVector CamRight = FRotationMatrix(CamRot).GetUnitAxis(EAxis::Y);

  const FVector2D Forward2D(CamForward.X, CamForward.Y);
  const FVector2D Right2D(CamRight.X, CamRight.Y);
  const FVector2D InputDir = InputVector.GetSafeNormal();

  const float ForwardDot =
      FVector2D::DotProduct(InputDir, Forward2D.GetSafeNormal());
  const float RightDot =
      FVector2D::DotProduct(InputDir, Right2D.GetSafeNormal());

  return FVector2D(RightDot, ForwardDot);
}

// Returns the angle to the target relative to the screen
float UOHLocomotionUtils::GetScreenRelativeAngleToTarget(
    const AActor *TargetActor, const APlayerController *PlayerController,
    const ACharacter *Character) {
  if (!PlayerController || !TargetActor || !Character) {
    return 0.0f;
  }

  const FVector2D ToTarget =
      FVector2D(TargetActor->GetActorLocation() - Character->GetActorLocation())
          .GetSafeNormal();
  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector CamForward = CamRot.Vector();
  const FVector2D CamForward2D(CamForward.X, CamForward.Y);

  const float AngleRad =
      FMath::Atan2(CamForward2D.X * ToTarget.Y - CamForward2D.Y * ToTarget.X,
                   CamForward2D.X * ToTarget.X + CamForward2D.Y * ToTarget.Y);

  return FMath::RadiansToDegrees(AngleRad);
}

EScreenDirection4
UOHLocomotionUtils::GetScreenRelativeCardinalDirectionToTarget(
    const AActor *TargetActor, const APlayerController *PlayerController,
    const ACharacter *Character) {
  const float Angle =
      GetScreenRelativeAngleToTarget(TargetActor, PlayerController, Character);

  if (Angle >= -45.f && Angle < 45.f) {
    return EScreenDirection4::Forward;
  } else if (Angle >= 45.f && Angle < 135.f) {
    return EScreenDirection4::Right;
  } else if (Angle >= -135.f && Angle < -45.f) {
    return EScreenDirection4::Left;
  } else {
    return EScreenDirection4::Backward;
  }
}

EScreenDirection8 UOHLocomotionUtils::GetScreenRelative8WayDirectionToTarget(
    const AActor *TargetActor, const APlayerController *PlayerController,
    const ACharacter *Character) {
  const float Angle =
      GetScreenRelativeAngleToTarget(TargetActor, PlayerController, Character);
  return MapAngleTo8WayDirection_Inline(Angle);
}

// Blend Factors To Target
FVector2D UOHLocomotionUtils::GetScreenRelativeBlendFactorsToTarget(
    const AActor *TargetActor, const APlayerController *PlayerController,
    const ACharacter *Character) {
  if (!PlayerController || !TargetActor || !Character) {
    return FVector2D::ZeroVector;
  }

  const FVector ToTarget =
      (TargetActor->GetActorLocation() - Character->GetActorLocation())
          .GetSafeNormal2D();
  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();

  return ComputeBlendFactors(FVector2D(ToTarget.X, ToTarget.Y), CamRot);
}

FVector2D UOHLocomotionUtils::GetScreenRelativeBlendFactorsFromVelocity(
    const APlayerController *PlayerController, const ACharacter *Character) {
  if (!PlayerController || !Character) {
    return FVector2D::ZeroVector;
  }

  const FVector2D Velocity2D(Character->GetVelocity().X,
                             Character->GetVelocity().Y);
  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();

  return ComputeBlendFactors(Velocity2D, CamRot);
}

FVector2D UOHLocomotionUtils::GetScreenRelativeBlendFactorsFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController) {
  if (!PlayerController || InputVector.IsNearlyZero()) {
    return FVector2D::ZeroVector;
  }

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  return ComputeBlendFactors(InputVector, CamRot);
}

// Returns the character's movement angle relative to the screen/camera
// direction.
float UOHLocomotionUtils::GetScreenRelativeAngleFromVelocity(
    const APlayerController *PlayerController, const ACharacter *Character) {
  if (!PlayerController || !Character) {
    return 0.0f;
  }

  const FVector Velocity = Character->GetVelocity().GetSafeNormal2D();
  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector2D CamForward(
      FVector2D(CamRot.Vector().X, CamRot.Vector().Y).GetSafeNormal());
  const FVector2D Velocity2D(Velocity.X, Velocity.Y);

  return ComputeScreenRelativeAngle(CamForward, Velocity2D, false);
}

EScreenDirection8
UOHLocomotionUtils::GetScreenRelative8WayDirectionFromVelocity(
    const APlayerController *PlayerController, const ACharacter *Character) {
  const float Angle =
      GetScreenRelativeAngleFromVelocity(PlayerController, Character);
  return MapAngleTo8WayDirection(Angle);
}

float UOHLocomotionUtils::GetScreenRelativeAngleFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController) {
  if (!PlayerController || InputVector.IsNearlyZero()) {
    return 0.0f;
  }

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector2D CamForward(
      FVector2D(CamRot.Vector().X, CamRot.Vector().Y).GetSafeNormal());
  const FVector2D InputDir = InputVector.GetSafeNormal();

  return ComputeScreenRelativeAngle(CamForward, InputDir, false);
}

EScreenDirection8 UOHLocomotionUtils::GetScreenRelative8WayDirectionFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController) {
  const float Angle =
      GetScreenRelativeAngleFromInput(InputVector, PlayerController);
  return MapAngleTo8WayDirection(Angle);
}

FVector UOHLocomotionUtils::GetCameraRelativeMovementVectorFromInput(
    const FVector2D &InputVector, const FRotator &CameraRotation,
    bool bNormalize) {
  if (InputVector.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FVector2D Blend =
      ComputeBlendFactors_Inline(InputVector, CameraRotation, bNormalize);
  const FVector Forward = CameraRotation.Vector();
  const FVector Right = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::Y);
  return (Right * Blend.X + Forward * Blend.Y).GetSafeNormal();
}

FRotator UOHLocomotionUtils::GetCameraRelativeFacingRotationFromInput(
    const FVector2D &InputVector, const FRotator &CameraRotation,
    bool bNormalize) {
  const FVector MoveVector = GetCameraRelativeMovementVectorFromInput(
      InputVector, CameraRotation, bNormalize);
  return MoveVector.IsNearlyZero() ? FRotator::ZeroRotator
                                   : MoveVector.Rotation();
}

FVector UOHLocomotionUtils::GetActorRelativeMovementVectorFromInput(
    const FVector2D &InputVector, const AActor *ReferenceActor,
    bool bNormalize) {
  if (!ReferenceActor || InputVector.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FRotator Rot = ReferenceActor->GetActorRotation();
  const FVector2D Blend =
      ComputeBlendFactors_Inline(InputVector, Rot, bNormalize);
  const FVector Forward = Rot.Vector();
  const FVector Right = FRotationMatrix(Rot).GetUnitAxis(EAxis::Y);
  return (Right * Blend.X + Forward * Blend.Y).GetSafeNormal();
}

FRotator UOHLocomotionUtils::GetActorRelativeFacingRotationFromInput(
    const FVector2D &InputVector, const AActor *ReferenceActor,
    bool bNormalize) {
  const FVector MoveVector = GetActorRelativeMovementVectorFromInput(
      InputVector, ReferenceActor, bNormalize);
  return MoveVector.IsNearlyZero() ? FRotator::ZeroRotator
                                   : MoveVector.Rotation();
}

FVector UOHLocomotionUtils::GetComponentRelativeMovementVectorFromInput(
    const FVector2D &InputVector, const USceneComponent *ReferenceComponent,
    bool bNormalize) {
  if (!ReferenceComponent || InputVector.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FRotator Rot = ReferenceComponent->GetComponentRotation();
  const FVector2D Blend =
      ComputeBlendFactors_Inline(InputVector, Rot, bNormalize);
  const FVector Forward = Rot.Vector();
  const FVector Right = FRotationMatrix(Rot).GetUnitAxis(EAxis::Y);
  return (Right * Blend.X + Forward * Blend.Y).GetSafeNormal();
}

FRotator UOHLocomotionUtils::GetComponentRelativeFacingRotationFromInput(
    const FVector2D &InputVector, const USceneComponent *ReferenceComponent,
    bool bNormalize) {
  const FVector MoveVector = GetComponentRelativeMovementVectorFromInput(
      InputVector, ReferenceComponent, bNormalize);
  return MoveVector.IsNearlyZero() ? FRotator::ZeroRotator
                                   : MoveVector.Rotation();
}

FScreenRelativeDirectionData UOHLocomotionUtils::BuildScreenDirectionFromInput(
    const FVector2D &InputVector, const FRotator &ReferenceRotation,
    const AActor *ReferenceActor, const APlayerController *PlayerController,
    bool bAllowMirroring, bool bClamp) {
  FScreenRelativeDirectionData Result;

  if (InputVector.IsNearlyZero()) {
    return Result;
  }

  // Normalize the input vector
  const FVector2D InputDir = InputVector.GetSafeNormal();

  // Calculate the forward and right vectors from the reference rotation
  const FVector Forward3D = ReferenceRotation.Vector();
  const FVector Right3D =
      FRotationMatrix(ReferenceRotation).GetUnitAxis(EAxis::Y);
  const FVector2D Forward2D(Forward3D.X, Forward3D.Y);
  const FVector2D Right2D(Right3D.X, Right3D.Y);

  // Compute blend factors
  float ForwardBlend = FVector2D::DotProduct(InputDir, Forward2D);
  float RightBlend = FVector2D::DotProduct(InputDir, Right2D);

  // Compute angle in degrees [-180, 180]
  float AngleDeg =
      FMath::RadiansToDegrees(FMath::Atan2(RightBlend, ForwardBlend));

  // Compute wrapped angle in degrees [0, 360)
  float WrappedAngleDeg = FMath::Fmod(AngleDeg + 360.0f, 360.0f);

  // Determine if mirroring should be applied
  bool bShouldMirror = false;
  if (bAllowMirroring && ReferenceActor && PlayerController) {
    const FVector2D ActorForward =
        FVector2D(ReferenceActor->GetActorForwardVector().X,
                  ReferenceActor->GetActorForwardVector().Y)
            .GetSafeNormal();
    const FVector2D CameraForward =
        FVector2D(PlayerController->PlayerCameraManager->GetCameraRotation()
                      .Vector()
                      .X,
                  PlayerController->PlayerCameraManager->GetCameraRotation()
                      .Vector()
                      .Y)
            .GetSafeNormal();
    bShouldMirror = FVector2D::DotProduct(ActorForward, CameraForward) < 0.0f;
  }

  if (bShouldMirror) {
    RightBlend *= -1.0f;
    AngleDeg *= -1.0f;
    WrappedAngleDeg = FMath::Fmod(AngleDeg + 360.0f, 360.0f);
  }

  // Clamp blend factors if requested
  if (bClamp) {
    RightBlend = FMath::Clamp(RightBlend, -1.0f, 1.0f);
    ForwardBlend = FMath::Clamp(ForwardBlend, -1.0f, 1.0f);
  }

  // Populate the result struct
  Result.Angle = AngleDeg;
  Result.WrappedAngle = WrappedAngleDeg;
  Result.BlendFactors = FVector2D(RightBlend, ForwardBlend);
  Result.ClampedStrafeDirection = FMath::Clamp(RightBlend, -1.0f, 1.0f);

  // Determine discrete directions
  Result.Direction =
      UOHLocomotionUtils::MapAngleTo8WayDirection(WrappedAngleDeg);
  Result.Direction4 =
      UOHLocomotionUtils::MapAngleTo4WayDirection(WrappedAngleDeg);
  Result.Quadrant = UOHLocomotionUtils::MapAngleToQuadrant(WrappedAngleDeg);

  return Result;
}
///////////////////////////////////////////////////////////////////////////

FVector UOHLocomotionUtils::GetWorldMovementVectorFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController,
    bool bNormalize) {
  if (!PlayerController || InputVector.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FVector2D Blend = ComputeBlendFactors_Inline(
      InputVector, PlayerController->PlayerCameraManager->GetCameraRotation(),
      bNormalize);
  const FVector Forward =
      PlayerController->PlayerCameraManager->GetCameraRotation().Vector();
  const FVector Right =
      FRotationMatrix(
          PlayerController->PlayerCameraManager->GetCameraRotation())
          .GetUnitAxis(EAxis::Y);

  return (Right * Blend.X + Forward * Blend.Y).GetSafeNormal();
}

FRotator UOHLocomotionUtils::GetFacingRotationFromInput(
    const FVector2D &InputVector, const APlayerController *PlayerController,
    bool bNormalize) {
  const FVector MoveDir = GetWorldMovementVectorFromInput(
      InputVector, PlayerController, bNormalize);
  return MoveDir.IsNearlyZero() ? FRotator::ZeroRotator : MoveDir.Rotation();
}

FVector UOHLocomotionUtils::GetWorldMovementVectorFromBlendFactors(
    const FVector2D &BlendFactors, const APlayerController *PlayerController) {
  if (!PlayerController || BlendFactors.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FRotator CamRot =
      PlayerController->PlayerCameraManager->GetCameraRotation();
  const FVector Forward = CamRot.Vector();
  const FVector Right = FRotationMatrix(CamRot).GetUnitAxis(EAxis::Y);

  return (Right * BlendFactors.X + Forward * BlendFactors.Y).GetSafeNormal();
}

FRotator UOHLocomotionUtils::GetFacingRotationFromDirection(
    const FScreenRelativeDirectionData &DirectionData,
    const APlayerController *PlayerController) {
  return GetWorldMovementVectorFromBlendFactors(DirectionData.BlendFactors,
                                                PlayerController)
      .Rotation();
}

FVector UOHLocomotionUtils::GetStrafeMovementVectorFromBlendFactors(
    const FVector2D &BlendFactors, const FRotator &FacingRotation) {
  if (BlendFactors.IsNearlyZero()) {
    return FVector::ZeroVector;
  }

  const FVector Forward = FacingRotation.Vector();
  const FVector Right = FRotationMatrix(FacingRotation).GetUnitAxis(EAxis::Y);

  return (Right * BlendFactors.X + Forward * BlendFactors.Y).GetSafeNormal();
}

///////////////////////////////////////////////////////////

FVector2D UOHLocomotionUtils::GetOptimalAnimationBlendVector(
    const FScreenRelativeDirectionData &DirectionData, bool bNormalize,
    bool bClamp) {
  if (DirectionData.BlendFactors.IsNearlyZero()) {
    return FVector2D::ZeroVector;
  }

  FVector2D Blend = DirectionData.BlendFactors;

  if (bNormalize) {
    Blend.Normalize();
  }

  if (bClamp) {
    Blend.X = FMath::Clamp(Blend.X, -1.f, 1.f);
    Blend.Y = FMath::Clamp(Blend.Y, -1.f, 1.f);
  }

  return Blend;
}

FVector2D UOHLocomotionUtils::GetVisualConsistentAnimationBlendVector(
    const FScreenRelativeDirectionData &DirectionData,
    const AActor *ReferenceActor, const APlayerController *PlayerController,
    bool bNormalize, bool bClamp, bool bApplyMirroring) {
  if (!ReferenceActor || !PlayerController ||
      DirectionData.BlendFactors.IsNearlyZero()) {
    return FVector2D::ZeroVector;
  }

  FVector2D Adjusted = DirectionData.BlendFactors;

  if (bApplyMirroring) {
    const FVector ActorForward =
        ReferenceActor->GetActorForwardVector().GetSafeNormal2D();
    const FVector CameraForward =
        PlayerController->PlayerCameraManager->GetCameraRotation()
            .Vector()
            .GetSafeNormal2D();

    // Determine if actor is facing leftward on screen (from camera's
    // perspective)
    const float Dot =
        FVector2D::DotProduct(FVector2D(ActorForward.X, ActorForward.Y),
                              FVector2D(CameraForward.X, CameraForward.Y));
    const bool bFacingAwayFromCameraForward = Dot < 0.f;

    // If facing away, mirror the lateral direction
    if (bFacingAwayFromCameraForward) {
      Adjusted.X *= -1.f; // Mirror right/left
    }
  }

  if (bNormalize) {
    Adjusted.Normalize();
  }

  if (bClamp) {
    Adjusted.X = FMath::Clamp(Adjusted.X, -1.f, 1.f);
    Adjusted.Y = FMath::Clamp(Adjusted.Y, -1.f, 1.f);
  }

  return Adjusted;
}

float UOHLocomotionUtils::GetVisualConsistentAnimationDirectionAngle(
    const FScreenRelativeDirectionData &DirectionData,
    const AActor *ReferenceActor, const APlayerController *PlayerController,
    bool bApplyMirroring, bool bNormalize, bool bClamp) {
  if (!ReferenceActor || !PlayerController) {
    return 0.f;
  }

  float Angle = DirectionData.Angle;

  if (bApplyMirroring) {
    const FVector2D ActorForward =
        FVector2D(ReferenceActor->GetActorForwardVector().X,
                  ReferenceActor->GetActorForwardVector().Y)
            .GetSafeNormal();
    const FVector2D CameraForward =
        FVector2D(PlayerController->PlayerCameraManager->GetCameraRotation()
                      .Vector()
                      .X,
                  PlayerController->PlayerCameraManager->GetCameraRotation()
                      .Vector()
                      .Y)
            .GetSafeNormal();

    const float Dot = FVector2D::DotProduct(ActorForward, CameraForward);
    const bool bFacingAwayFromCamera = Dot < 0.f;

    if (bFacingAwayFromCamera) {
      Angle *= -1.f;
    }
  }

  if (bNormalize) {
    // Normalize to [-1, 1]
    Angle /= 180.f;
  }

  if (bClamp) {
    Angle = FMath::Clamp(Angle, bNormalize ? -1.f : -180.f,
                         bNormalize ? 1.f : 180.f);
  }

  return Angle;
}

float UOHLocomotionUtils::GetPredictedDirectionAngleFromBoneMotion(
    const FOHBoneData &BoneData, const AActor *ReferenceActor,
    const APlayerController *PlayerController, float PredictAheadTime,
    bool bApplyMirroring, bool bNormalize, bool bClamp) {
  if (!ReferenceActor || !PlayerController || !BoneData.HasMotionHistory()) {
    return 0.f;
  }

  const FVector Start = BoneData.GetCurrentPosition();
  const FVector End =
      Start + BoneData.GetBodyLinearVelocity() * PredictAheadTime;
  const FVector2D Delta = FVector2D((End - Start).GetSafeNormal2D());

  if (Delta.IsNearlyZero()) {
    return 0.f;
  }

  float AngleRad = FMath::Atan2(Delta.X, Delta.Y);
  float AngleDeg = FMath::RadiansToDegrees(AngleRad); // [-180, 180]

  if (bApplyMirroring) {
    const FVector2D ActorForward =
        FVector2D(ReferenceActor->GetActorForwardVector()).GetSafeNormal();
    const FVector2D CameraForward =
        FVector2D(
            PlayerController->PlayerCameraManager->GetCameraRotation().Vector())
            .GetSafeNormal();

    if (FVector2D::DotProduct(ActorForward, CameraForward) < 0.f) {
      AngleDeg *= -1.f;
    }
  }

  if (bNormalize) {
    AngleDeg /= 180.f;
  }

  if (bClamp) {
    AngleDeg = FMath::Clamp(AngleDeg, bNormalize ? -1.f : -180.f,
                            bNormalize ? 1.f : 180.f);
  }

  return AngleDeg;
}

FVector2D UOHLocomotionUtils::GetPredictedBlendVectorFromBoneMotion(
    const FOHBoneData &BoneData, const AActor *ReferenceActor,
    const APlayerController *PlayerController, float PredictAheadTime,
    bool bApplyMirroring, bool bNormalize, bool bClamp) {
  if (!ReferenceActor || !PlayerController || !BoneData.HasMotionHistory()) {
    return FVector2D::ZeroVector;
  }

  const FVector Start = BoneData.GetCurrentPosition();
  const FVector End =
      Start + BoneData.GetBodyLinearVelocity() * PredictAheadTime;
  FVector2D Delta = FVector2D((End - Start).GetSafeNormal2D());

  if (Delta.IsNearlyZero()) {
    return FVector2D::ZeroVector;
  }

  if (bApplyMirroring) {
    const FVector2D ActorForward =
        FVector2D(ReferenceActor->GetActorForwardVector()).GetSafeNormal();
    const FVector2D CameraForward =
        FVector2D(
            PlayerController->PlayerCameraManager->GetCameraRotation().Vector())
            .GetSafeNormal();

    if (FVector2D::DotProduct(ActorForward, CameraForward) < 0.f) {
      Delta.X *= -1.f;
    }
  }

  if (bNormalize) {
    Delta.Normalize();
  }

  if (bClamp) {
    Delta.X = FMath::Clamp(Delta.X, -1.f, 1.f);
    Delta.Y = FMath::Clamp(Delta.Y, -1.f, 1.f);
  }

  return Delta;
}

/**
 * Computes the screen-relative angle in degrees between two 2D vectors.
 * The result is in the range [-180, 180].
 *
 * @param FromVector The starting 2D vector.
 * @param ToVector The target 2D vector.
 * @param bNormalize If true, normalizes the input vectors before computing the
 * angle.
 * @return The angle in degrees between the FromVector and ToVector in a
 * screen-relative context.
 */
float UOHLocomotionUtils::ComputeScreenRelativeAngle(
    const FVector2D &FromVector, const FVector2D &ToVector, bool bNormalize) {
  if (FromVector.IsNearlyZero() || ToVector.IsNearlyZero()) {
    return 0.0f;
  }

  const FVector2D From = bNormalize ? FromVector.GetSafeNormal() : FromVector;
  const FVector2D To = bNormalize ? ToVector.GetSafeNormal() : ToVector;

  const float AngleRad = FMath::Atan2(FVector2D::CrossProduct(From, To),
                                      FVector2D::DotProduct(From, To));

  return FMath::RadiansToDegrees(AngleRad); // Range: [-180, 180]
}

float UOHLocomotionUtils::PredictMovementAngle(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    float &InOutPrevAngle,

    float BasePredictionTime, float SpeedPredictionScale,
    float BaseSmoothingAlpha, float SpeedSmoothScale,
    float PelvisTranslationThreshold, float FootLockSpeedThreshold,
    float FootLockAccelThreshold, float PivotScale, float PivotDeadzone,
    float StrafeBias, float JitterThreshold, float MaxAngleJumpPerFrame,

    float JerkTriggerScale, float MaxTrajectoryBlend) {
  if (!Character || DeltaTime <= 0.f)
    return InOutPrevAngle;

  // 1) Adaptive prediction time & smoothing α
  FVector CharVel = Character->GetVelocity();
  float Speed = CharVel.Size();

  float PredictionTime = BasePredictionTime + Speed * SpeedPredictionScale;
  float Alpha =
      FMath::Clamp(BaseSmoothingAlpha - Speed * SpeedSmoothScale, 0.f, 1.f);

  if (const auto *MoveComp = Character->GetCharacterMovement()) {
    switch (MoveComp->MovementMode) {
    case MOVE_Falling:
      PredictionTime *= 1.2f;
      Alpha *= 0.9f;
      break;
    case MOVE_Swimming:
    case MOVE_Custom:
      PredictionTime *= 0.5f;
      Alpha *= 1.2f;
      break;
    default:
      break;
    }
  }

  // 2) Acquire vel/acc (physics manager or fallback)
  FVector FinalVel = FVector::ZeroVector;
  FVector FinalAccel = FVector::ZeroVector;
  FVector MoveDir = FVector::ZeroVector;

  if (auto *PhysMgr = Character->FindComponentByClass<UOHPhysicsManager>()) {
    FVector RV = PhysMgr->GetVelocity(FName("root"));
    FVector PV = PhysMgr->GetVelocity(FName("pelvis"));

    float RM = FMath::Max(RV.Size(), KINDA_SMALL_NUMBER);
    float PM = PV.Size();
    float B = FMath::Clamp(
        PM / (RM * PelvisTranslationThreshold + KINDA_SMALL_NUMBER), 0.f, 1.f);

    FinalVel = FMath::Lerp(RV, PV, B);
    FinalAccel =
        FMath::Lerp(PhysMgr->GetLinearAcceleration(FName("root")),
                    PhysMgr->GetLinearAcceleration(FName("pelvis")), B);

    MoveDir = (FinalVel * PredictionTime +
               0.5f * FinalAccel * PredictionTime * PredictionTime)
                  .GetSafeNormal();
  }

  // 3) Fallback to Character::GetVelocity
  if (MoveDir.IsNearlyZero()) {
    FinalVel = CharVel;
    FinalAccel = FVector::ZeroVector;

    if (FinalVel.Size() < FootLockSpeedThreshold &&
        FinalAccel.Size() > FootLockAccelThreshold) {
      return InOutPrevAngle;
    }

    MoveDir = FinalVel.GetSafeNormal();
    if (MoveDir.IsNearlyZero())
      return InOutPrevAngle;
  }

  // 4) Raw screen-space angle [0..360)
  FVector LocalDir = FacingRotation.UnrotateVector(MoveDir);
  float TargetA = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LocalDir.Y, LocalDir.X)) + 360.f,
      360.f);

  // 5) Pivot anticipation
  {
    FVector Vn = FinalVel.GetSafeNormal();
    FVector An = FinalAccel.GetSafeNormal();
    float Arc = FVector::CrossProduct(Vn, An).Z;
    if (FMath::Abs(Arc) > PivotDeadzone)
      TargetA += Arc * PivotScale;
  }

  // 6) Strafe bias
  {
    float BiasT = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) * 0.5f);
    TargetA = FMath::Lerp(TargetA, BiasT, FMath::Abs(StrafeBias));
  }

  // 7) Trajectory-based blend when jerk is high
  if (auto *PhysMgr = Character->FindComponentByClass<UOHPhysicsManager>()) {
    if (const FOHBoneData *PelvisData = PhysMgr->GetBoneData(FName("pelvis"))) {
      const TArray<FOHMotionSample> &History = PelvisData->GetMotionHistory();
      FVector Jerk = UOHAlgoUtils::EstimateJerk(History, DeltaTime);
      float JerkMag = Jerk.Size();
      float TrajBlend =
          FMath::Clamp(JerkMag * JerkTriggerScale, 0.f, MaxTrajectoryBlend);

      if (TrajBlend > KINDA_SMALL_NUMBER) {
        TArray<FVector> CP =
            UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(
                *PelvisData, PredictionTime);
        const FVector P0 = CP[0], P1 = CP[1], P2 = CP[2];
        constexpr int NumSamples = 3;
        FVector2D SumDir(0, 0);

        for (int i = 0; i < NumSamples; ++i) {
          float T = (i + 1) / float(NumSamples + 1);
          FVector A = FMath::Lerp(P0, P1, T);
          FVector B = FMath::Lerp(P1, P2, T);
          FVector Pt = FMath::Lerp(A, B, T);

          FVector Dir = (Pt - P0).GetSafeNormal();
          FVector Loc = FacingRotation.UnrotateVector(Dir);
          float AngRad = FMath::Atan2(Loc.Y, Loc.X);
          SumDir.X += FMath::Cos(AngRad);
          SumDir.Y += FMath::Sin(AngRad);
        }

        if (!SumDir.IsNearlyZero()) {
          float AvgAngDeg = FMath::Fmod(
              FMath::RadiansToDegrees(FMath::Atan2(SumDir.Y, SumDir.X)) + 360.f,
              360.f);
          TargetA = FMath::Lerp(TargetA, AvgAngDeg, TrajBlend);
        }
      }
    }
  }

  // 8) Clamp & jitter-filter
  {
    float DeltaA = FMath::FindDeltaAngleDegrees(InOutPrevAngle, TargetA);
    if (FMath::Abs(DeltaA) > MaxAngleJumpPerFrame)
      TargetA = InOutPrevAngle + FMath::Clamp(DeltaA, -MaxAngleJumpPerFrame,
                                              MaxAngleJumpPerFrame);
    if (FMath::Abs(DeltaA) < JitterThreshold)
      TargetA = InOutPrevAngle;
  }

  // 9) Exponential smoothing
  float Smoothed = InOutPrevAngle + Alpha * FMath::FindDeltaAngleDegrees(
                                                InOutPrevAngle, TargetA);
  Smoothed = FMath::Fmod(Smoothed + 360.f, 360.f);

  InOutPrevAngle = Smoothed;
  return Smoothed;
}

float UOHLocomotionUtils::PredictMovementAngleFromHistory(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    float /*PredictionTime*/, float /*SpeedPredictionScale*/,
    float PivotDeadzone, float PivotScale, float StrafeBias) {
  if (!Character)
    return 0.f;

  // Grab physics manager and root‐bone data
  const UOHPhysicsManager *PhysMgr =
      Character->FindComponentByClass<UOHPhysicsManager>();
  FVector MoveDir = FVector::ZeroVector;

  if (PhysMgr) {
    const FOHBoneData *RootData = PhysMgr->GetBoneData(FName("root"));
    if (RootData && RootData->HasMotionHistory()) {
      // Use the full history Δ-displacement → robust average direction
      const auto &H = RootData->GetMotionHistory();
      if (H.Num() >= 2) {
        MoveDir = (H.Last().GetLocation() - H[0].GetLocation()).GetSafeNormal();
      }
    }

    // Fallback to the smoothed instantaneous velocity
    if (MoveDir.IsNearlyZero() && PhysMgr) {
      const FOHBoneData *RootData2 = PhysMgr->GetBoneData(FName("root"));
      if (RootData2) {
        MoveDir = RootData2->GetSmoothedLinearVelocity();
        // FOHBoneData::GetSmoothedLinearVelocity
        // :contentReference[oaicite:0]{index=0}
      }
    }
  }

  if (MoveDir.IsNearlyZero())
    return 0.f;

  // Transform into local (FacingRotation) space
  const FVector LocalDir = FacingRotation.UnrotateVector(MoveDir);
  float AngleDeg =
      FMath::RadiansToDegrees(FMath::Atan2(LocalDir.Y, LocalDir.X));
  AngleDeg = FMath::Fmod(AngleDeg + 360.f, 360.f);

  // Pivot anticipation: cross(vel, accel).Z
  if (PhysMgr) {
    const FOHBoneData *RD = PhysMgr->GetBoneData(FName("root"));
    if (RD) {
      FVector Accel = RD->GetSmoothedLinearAcceleration();
      // FOHBoneData::GetSmoothedLinearAcceleration
      // :contentReference[oaicite:1]{index=1}

      if (!Accel.IsNearlyZero()) {
        float Arc = FVector::CrossProduct(MoveDir, Accel.GetSafeNormal()).Z;
        if (FMath::Abs(Arc) > PivotDeadzone) {
          AngleDeg += FMath::Clamp(Arc, -1.f, 1.f) * PivotScale;
        }
      }
    }
  }

  // Apply directional strafe bias
  float BiasTarget = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) * 0.5f);
  AngleDeg = FMath::Lerp(AngleDeg, BiasTarget, FMath::Abs(StrafeBias));

  return AngleDeg;
}

float UOHLocomotionUtils::PredictMovementAngleWithBezierDebug(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    UWorld *World, float DrawDuration, FColor CurveColor, int32 NumSegments,
    float PredictionTime, float SpeedPredictionScale, float PivotDeadzone,
    float PivotScale, float JerkThreshold, float StrafeBias) {
  if (!Character || !World)
    return 0.f;

  // Fetch root data
  const UOHPhysicsManager *PhysMgr =
      Character->FindComponentByClass<UOHPhysicsManager>();
  const FOHBoneData *RD =
      PhysMgr ? PhysMgr->GetBoneData(FName("root")) : nullptr;
  if (!RD || !RD->HasMotionHistory())
    return 0.f;

  // -- Build a simple quadratic Bézier (P0→P1→P2):
  const auto &H = RD->GetMotionHistory();
  FVector P0 = H[0].GetLocation();
  FVector P2 = H.Last().GetLocation();

  // Middle control point based on speed‐weighted extrapolation of last segment
  FVector LastVel = RD->GetSmoothedLinearVelocity();
  FVector P1 = P2 + LastVel * (PredictionTime * SpeedPredictionScale);

  // Optionally thicken line if jerk is high
  float JerkMag = RD->GetJerkMagnitude();
  float LineWidth = (JerkMag > JerkThreshold) ? 4.f : 2.f;

  // Draw the curve
  FVector PrevPoint = P0;
  for (int32 i = 1; i <= NumSegments; ++i) {
    float t = static_cast<float>(i) / NumSegments;
    FVector A = FMath::Lerp(P0, P1, t);
    FVector B = FMath::Lerp(P1, P2, t);
    FVector Pt = FMath::Lerp(A, B, t);
    DrawDebugLine(World, PrevPoint, Pt, CurveColor, false, DrawDuration, 0,
                  LineWidth);
    PrevPoint = Pt;
  }

  // Compute tangent at end → final move direction
  FVector Tangent = (P2 - P1).GetSafeNormal();

  // Transform into local space & get angle
  const FVector LocalDir = FacingRotation.UnrotateVector(Tangent);
  float AngleDeg =
      FMath::RadiansToDegrees(FMath::Atan2(LocalDir.Y, LocalDir.X));
  AngleDeg = FMath::Fmod(AngleDeg + 360.f, 360.f);

  // Pivot anticipation
  FVector Accel = RD->GetSmoothedLinearAcceleration();
  if (!Accel.IsNearlyZero()) {
    float Arc = FVector::CrossProduct(Tangent, Accel.GetSafeNormal()).Z;
    if (FMath::Abs(Arc) > PivotDeadzone) {
      AngleDeg += FMath::Clamp(Arc, -1.f, 1.f) * PivotScale;
    }
  }

  // Strafe bias
  float BiasTarget = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) * 0.5f);
  AngleDeg = FMath::Lerp(AngleDeg, BiasTarget, FMath::Abs(StrafeBias));

  return AngleDeg;
}

float UOHLocomotionUtils::PredictMovementAngleMonster(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    UWorld *World, float DrawDuration, float PredictionTime,
    float SpeedPredictionScale, float AccelPredictionScale, float JerkScale,
    float CurvatureScale, float MaxBlend, float PivotDeadzone, float PivotScale,
    float StrafeBias, float RotationThreshold, float MoveThreshold,
    float ProcNoise, float MeasNoise) {
  // --- Kalman state (static across calls) ---
  static bool bInit = false;
  static float PrevAng = 0.f, PrevVar = 0.f, PrevAngVel = 0.f;
  if (!bInit) {
    PrevVar = MeasNoise;
    bInit = true;
  }

  if (!Character || DeltaTime <= 0.f)
    return PrevAng;

  // --- 1) Gather bone histories for centroid fit ---
  static const FName BoneNames[] = {TEXT("root"), TEXT("pelvis"),
                                    TEXT("spine_01"), TEXT("spine_02")};
  UOHPhysicsManager *PhysMgr =
      Character->FindComponentByClass<UOHPhysicsManager>();
  if (!PhysMgr)
    return PrevAng;

  TArray<const TArray<FOHMotionSample> *> Hists;
  for (auto &B : BoneNames) {
    const FOHBoneData *BD = PhysMgr->GetBoneData(B);
    if (BD && BD->HasMotionHistory())
      Hists.Add(&BD->GetMotionHistory());
  }
  if (Hists.Num() == 0)
    return PrevAng;

  // get the shortest history length
  int32 Count = INT_MAX;
  for (auto Hist : Hists)
    Count = FMath::Min(Count, Hist->Num());
  if (Count < 3)
    return PrevAng;

  // build centroids[] and times[] from the first history
  TArray<FVector> Centroids;
  Centroids.SetNum(Count);
  TArray<float> Times;
  Times.SetNum(Count);
  for (int32 i = 0; i < Count; ++i) {
    FVector SumPos(0.f);
    for (auto Hist : Hists)
      SumPos += (*Hist)[i].GetLocation();
    Centroids[i] = SumPos / Hists.Num();

    // time from the first bone's history
    Times[i] = (*Hists[0])[i].GetTimeStamp();
  }

  // --- 2) Rotation-only detection ---
  {
    // root-bone yaw at start/end
    const auto &RootHist = *Hists[0];
    float Y0 = RootHist[0].GetRotation().Rotator().Yaw;
    float Y1 = RootHist.Last().GetRotation().Rotator().Yaw;
    float dYaw = FMath::FindDeltaAngleDegrees(Y0, Y1);
    float dPos = FVector::Dist(Centroids.Last(), Centroids[0]);
    if (FMath::Abs(dYaw) > RotationThreshold && dPos < MoveThreshold)
      return FMath::Fmod(FacingRotation.Yaw + dYaw + 360.f, 360.f);
  }

  // --- 3) Raw direction (centroid Δ-pos or smoothed root vel) ---
  FVector RawDir;
  {
    FVector DP = Centroids.Last() - Centroids[0];
    if (DP.IsNearlyZero()) {
      const FOHBoneData *R = PhysMgr->GetBoneData(TEXT("root"));
      RawDir = R ? R->GetSmoothedLinearVelocity().GetSafeNormal()
                 : FVector::ZeroVector;
    } else {
      RawDir = DP.GetSafeNormal();
    }
    if (RawDir.IsNearlyZero())
      return PrevAng;
  }

  // --- 4) Dynamic horizon: speed + accel scaling ---
  float FinalPredTime = PredictionTime;
  {
    float Speed = Character->GetVelocity().Size();
    FinalPredTime += Speed * SpeedPredictionScale;

    const FOHBoneData *RD = PhysMgr->GetBoneData(TEXT("root"));
    float AccMag = RD ? RD->GetSmoothedLinearAcceleration().Size() : 0.f;
    FinalPredTime *= (1.f + AccMag * AccelPredictionScale);
  }

  // --- 5) Jerk & curvature weights ---
  const auto &RH = *Hists[0];
  float JM = UOHAlgoUtils::EstimateJerk(RH, DeltaTime).Size();
  float Curv = UOHAlgoUtils::EstimateCurvature(RH);

  float PolyBlend = FMath::Clamp(JM * JerkScale, 0.f, MaxBlend);
  float BezBlend = FMath::Clamp(Curv * CurvatureScale, 0.f, MaxBlend);

  // --- 6) 2nd-order least-squares fit → PolyAng ---
  float PolyAng = 0.f;
  {
    // build normal equations for sum(t^k), sum(x t^k), sum(y t^k)...
    double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    double SX0 = 0, SX1 = 0, SX2 = 0;
    double SY0 = 0, SY1 = 0, SY2 = 0;
    for (int i = 0; i < Count; ++i) {
      double t = Times[i], x = Centroids[i].X, y = Centroids[i].Y;
      double t2 = t * t, t3 = t2 * t, t4 = t2 * t2;
      S0 += 1;
      S1 += t;
      S2 += t2;
      S3 += t3;
      S4 += t4;
      SX0 += x;
      SX1 += x * t;
      SX2 += x * t2;
      SY0 += y;
      SY1 += y * t;
      SY2 += y * t2;
    }
    double Det = S0 * (S2 * S4 - S3 * S3) - S1 * (S1 * S4 - S3 * S2) +
                 S2 * (S1 * S3 - S2 * S2);
    if (FMath::Abs(Det) > KINDA_SMALL_NUMBER) {
      // Cramer's rule for derivative coeffs (a1,a2) in x and y
      double A1 = (SX0 * (S2 * S4 - S3 * S3) - S1 * (SX1 * S4 - S3 * SX2) +
                   S2 * (SX1 * S3 - S2 * SX2)) /
                  Det;
      double B1 = (SY0 * (S2 * S4 - S3 * S3) - S1 * (SY1 * S4 - S3 * SY2) +
                   S2 * (SY1 * S3 - S2 * SY2)) /
                  Det;
      double A2 = (S0 * (SX1 * S4 - S3 * SX2) - SX0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SX2 - SX1 * S2)) /
                  Det;
      double B2 = (S0 * (SY1 * S4 - S3 * SY2) - SY0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SY2 - SY1 * S2)) /
                  Det;
      double tP = Times.Last();
      double dx = A1 + 2.0 * A2 * tP;
      double dy = B1 + 2.0 * B2 * tP;
      if (FMath::Abs(dx) + FMath::Abs(dy) > SMALL_NUMBER) {
        FVector PD(dx, dy, 0.f);
        FVector L = FacingRotation.UnrotateVector(PD.GetSafeNormal());
        PolyAng = FMath::Fmod(
            FMath::RadiansToDegrees(FMath::Atan2(L.Y, L.X)) + 360.f, 360.f);
      }
    }
  }

  // --- 7) Quadratic Bézier & debug draw → BezAng ---
  TArray<FVector> CP =
      UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(
          *PhysMgr->GetBoneData(TEXT("root")), FinalPredTime);
  const FVector &P0 = CP[0], &P1 = CP[1], &P2 = CP[2];

  if (World && DrawDuration > 0.f) {
    FVector Prev = P0;
    float W = (JM > KINDA_SMALL_NUMBER) ? 3.f : 1.f;
    for (int32 i = 1; i <= 16; ++i) {
      float t = i / 16.f;
      FVector A = FMath::Lerp(P0, P1, t);
      FVector B = FMath::Lerp(P1, P2, t);
      FVector Pt = FMath::Lerp(A, B, t);
      DrawDebugLine(World, Prev, Pt, FColor::Yellow, false, DrawDuration, 0, W);
      Prev = Pt;
    }
  }
  FVector BezDir = (P2 - P1).GetSafeNormal();
  FVector LB = FacingRotation.UnrotateVector(BezDir);
  float BezAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LB.Y, LB.X)) + 360.f, 360.f);

  // --- 8) Build measured angle: raw→poly→bezier blend ---
  FVector LR = FacingRotation.UnrotateVector(RawDir);
  float RawAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LR.Y, LR.X)) + 360.f, 360.f);

  float Meas = RawAng;
  Meas = FMath::Lerp(Meas, PolyAng, PolyBlend);
  Meas = FMath::Lerp(Meas, BezAng, BezBlend);

  // --- 9) Pivot & strafe bias ---
  {
    const FOHBoneData *RD = PhysMgr->GetBoneData(TEXT("root"));
    FVector Accn =
        RD ? RD->GetSmoothedLinearAcceleration() : FVector::ZeroVector;
    if (!Accn.IsNearlyZero()) {
      float Arc = FVector::CrossProduct(RawDir, Accn.GetSafeNormal()).Z;
      if (FMath::Abs(Arc) > PivotDeadzone)
        Meas += FMath::Clamp(Arc, -1.f, 1.f) * PivotScale;
    }
    float BiasT = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) * 0.5f);
    Meas = FMath::Lerp(Meas, BiasT, FMath::Abs(StrafeBias));
  }
  Meas = FMath::Fmod(Meas + 360.f, 360.f);

  // --- 10) 5-point Savitzky–Golay smoothing on Meas ---
  static bool bSGInit = false;
  static float SGBuf[5];
  if (!bSGInit) {
    for (int i = 0; i < 5; ++i)
      SGBuf[i] = Meas;
    bSGInit = true;
  } else {
    for (int i = 0; i < 4; ++i)
      SGBuf[i] = SGBuf[i + 1];
    SGBuf[4] = Meas;
  }
  static const float SG[5] = {-3.f / 35.f, 12.f / 35.f, 17.f / 35.f,
                              12.f / 35.f, -3.f / 35.f};
  float SmMeas = 0.f;
  for (int i = 0; i < 5; ++i)
    SmMeas += SG[i] * SGBuf[i];
  SmMeas = FMath::Fmod(SmMeas + 360.f, 360.f);

  // --- 11) Kalman-style update ---
  float PredAng = PrevAng + PrevAngVel * DeltaTime;
  float PredVar = PrevVar + ProcNoise;
  float dA = FMath::FindDeltaAngleDegrees(PredAng, SmMeas);
  float K = PredVar / (PredVar + MeasNoise);
  float NewAng = PredAng + K * dA;
  float NewVar = (1.f - K) * PredVar;
  float NewVel = dA / DeltaTime;

  PrevAng = FMath::Fmod(NewAng + 360.f, 360.f);
  PrevVar = NewVar;
  PrevAngVel = NewVel;

  return PrevAng;
}

float UOHLocomotionUtils::PredictMovementAngleMonsterNoDebug(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    float PredictionTime, float SpeedPredictionScale,
    float AccelPredictionScale, float JerkScale, float CurvatureScale,
    float MaxBlend, float PivotDeadzone, float PivotScale, float StrafeBias,
    float RotationThreshold, float MoveThreshold, float ProcNoise,
    float MeasNoise) {

  // --- Kalman state (static across calls) ---
  static bool bInit = false;
  static float PrevAng = 0.f, PrevVar = 0.f, PrevAngVel = 0.f;
  if (!bInit) {
    PrevVar = MeasNoise;
    bInit = true;
  }

  if (!Character || DeltaTime <= 0.f)
    return PrevAng;

  // --- 1) Gather bone histories for centroid fit ---
  static const FName BoneNames[] = {TEXT("root"), TEXT("pelvis"),
                                    TEXT("spine_01"), TEXT("spine_02")};
  UOHPhysicsManager *PhysMgr =
      Character->FindComponentByClass<UOHPhysicsManager>();
  if (!PhysMgr)
    return PrevAng;

  TArray<const TArray<FOHMotionSample> *> Hists;
  for (auto &B : BoneNames) {
    const FOHBoneData *BD = PhysMgr->GetBoneData(B);
    if (BD && BD->HasMotionHistory())
      Hists.Add(&BD->GetMotionHistory());
  }
  if (Hists.Num() == 0)
    return PrevAng;

  int32 Count = INT_MAX;
  for (auto Hist : Hists)
    Count = FMath::Min(Count, Hist->Num());
  if (Count < 3)
    return PrevAng;

  TArray<FVector> Centroids;
  Centroids.SetNum(Count);
  TArray<float> Times;
  Times.SetNum(Count);
  for (int32 i = 0; i < Count; ++i) {
    FVector SumPos(0.f);
    for (auto Hist : Hists)
      SumPos += (*Hist)[i].GetLocation();
    Centroids[i] = SumPos / Hists.Num();
    Times[i] = (*Hists[0])[i].GetTimeStamp();
  }

  // --- 2) Rotation-only detection ---
  {
    const auto &RootHist = *Hists[0];
    float Y0 = RootHist[0].GetRotation().Rotator().Yaw;
    float Y1 = RootHist.Last().GetRotation().Rotator().Yaw;
    float dYaw = FMath::FindDeltaAngleDegrees(Y0, Y1);
    float dPos = FVector::Dist(Centroids.Last(), Centroids[0]);
    if (FMath::Abs(dYaw) > RotationThreshold && dPos < MoveThreshold)
      return FMath::Fmod(FacingRotation.Yaw + dYaw + 360.f, 360.f);
  }

  // --- 3) Raw direction from centroid Δ-pos or smoothed root vel ---
  FVector RawDir;
  {
    FVector DP = Centroids.Last() - Centroids[0];
    if (DP.IsNearlyZero()) {
      const FOHBoneData *R = PhysMgr->GetBoneData(TEXT("root"));
      RawDir = R ? R->GetSmoothedLinearVelocity().GetSafeNormal()
                 : FVector::ZeroVector;
    } else {
      RawDir = DP.GetSafeNormal();
    }
    if (RawDir.IsNearlyZero())
      return PrevAng;
  }

  // --- 4) Dynamic horizon (speed + accel) ---
  float FinalPredTime = PredictionTime;
  {
    float Speed = Character->GetVelocity().Size();
    FinalPredTime += Speed * SpeedPredictionScale;

    const FOHBoneData *RD = PhysMgr->GetBoneData(TEXT("root"));
    float AccMag = RD ? RD->GetSmoothedLinearAcceleration().Size() : 0.f;
    FinalPredTime *= (1.f + AccMag * AccelPredictionScale);
  }

  // --- 5) Jerk & curvature weights ---
  const auto &RH = *Hists[0];
  float JM = UOHAlgoUtils::EstimateJerk(RH, DeltaTime).Size();
  float Curv = UOHAlgoUtils::EstimateCurvature(RH);

  float PolyBlend = FMath::Clamp(JM * JerkScale, 0.f, MaxBlend);
  float BezBlend = FMath::Clamp(Curv * CurvatureScale, 0.f, MaxBlend);

  // --- 6) 2nd-order LS fit → PolyAng ---
  float PolyAng = 0.f;
  {
    double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    double SX0 = 0, SX1 = 0, SX2 = 0;
    double SY0 = 0, SY1 = 0, SY2 = 0;
    for (int i = 0; i < Count; ++i) {
      double t = Times[i], x = Centroids[i].X, y = Centroids[i].Y;
      double t2 = t * t, t3 = t2 * t, t4 = t2 * t2;
      S0 += 1;
      S1 += t;
      S2 += t2;
      S3 += t3;
      S4 += t4;
      SX0 += x;
      SX1 += x * t;
      SX2 += x * t2;
      SY0 += y;
      SY1 += y * t;
      SY2 += y * t2;
    }
    double Det = S0 * (S2 * S4 - S3 * S3) - S1 * (S1 * S4 - S3 * S2) +
                 S2 * (S1 * S3 - S2 * S2);
    if (FMath::Abs(Det) > KINDA_SMALL_NUMBER) {
      double A1 = (SX0 * (S2 * S4 - S3 * S3) - S1 * (SX1 * S4 - S3 * SX2) +
                   S2 * (SX1 * S3 - S2 * SX2)) /
                  Det;
      double B1 = (SY0 * (S2 * S4 - S3 * S3) - S1 * (SY1 * S4 - S3 * SY2) +
                   S2 * (SY1 * S3 - S2 * SY2)) /
                  Det;
      double A2 = (S0 * (SX1 * S4 - S3 * SX2) - SX0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SX2 - SX1 * S2)) /
                  Det;
      double B2 = (S0 * (SY1 * S4 - S3 * SY2) - SY0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SY2 - SY1 * S2)) /
                  Det;
      double tP = Times.Last();
      double dx = A1 + 2.0 * A2 * tP;
      double dy = B1 + 2.0 * B2 * tP;
      if (FMath::Abs(dx) + FMath::Abs(dy) > SMALL_NUMBER) {
        FVector PD(dx, dy, 0.f);
        FVector L = FacingRotation.UnrotateVector(PD.GetSafeNormal());
        PolyAng = FMath::Fmod(
            FMath::RadiansToDegrees(FMath::Atan2(L.Y, L.X)) + 360.f, 360.f);
      }
    }
  }

  // --- 7) Quadratic Bézier → BezAng ---
  TArray<FVector> CP =
      UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(
          *PhysMgr->GetBoneData(TEXT("root")), FinalPredTime);
  const FVector &P0 = CP[0], &P1 = CP[1], &P2 = CP[2];
  FVector BezDir = (P2 - P1).GetSafeNormal();
  FVector LB = FacingRotation.UnrotateVector(BezDir);
  float BezAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LB.Y, LB.X)) + 360.f, 360.f);

  // --- 8) Measured angle = raw→poly→bezier blend ---
  FVector LR = FacingRotation.UnrotateVector(RawDir);
  float RawAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LR.Y, LR.X)) + 360.f, 360.f);
  float Meas =
      FMath::Lerp(FMath::Lerp(RawAng, PolyAng, PolyBlend), BezAng, BezBlend);

  // --- 9) Pivot & strafe bias ---
  {
    const FOHBoneData *RD = PhysMgr->GetBoneData(TEXT("root"));
    FVector Accn =
        RD ? RD->GetSmoothedLinearAcceleration() : FVector::ZeroVector;
    if (!Accn.IsNearlyZero()) {
      float Arc = FVector::CrossProduct(RawDir, Accn.GetSafeNormal()).Z;
      if (FMath::Abs(Arc) > PivotDeadzone)
        Meas += FMath::Clamp(Arc, -1.f, 1.f) * PivotScale;
    }
    float BiasT = FMath::Lerp(180.f, 0.f, (StrafeBias + 1.f) * 0.5f);
    Meas = FMath::Fmod(FMath::Lerp(Meas, BiasT, FMath::Abs(StrafeBias)) + 360.f,
                       360.f);
  }

  // --- 10) Savitzky–Golay smoothing on Meas ---
  static bool bSGInit = false;
  static float SGBuf[5];
  if (!bSGInit) {
    for (int i = 0; i < 5; ++i)
      SGBuf[i] = Meas;
    bSGInit = true;
  } else {
    for (int i = 0; i < 4; ++i)
      SGBuf[i] = SGBuf[i + 1];
    SGBuf[4] = Meas;
  }
  static const float SG[5] = {-3.f / 35.f, 12.f / 35.f, 17.f / 35.f,
                              12.f / 35.f, -3.f / 35.f};
  float SmMeas = 0.f;
  for (int i = 0; i < 5; ++i)
    SmMeas += SG[i] * SGBuf[i];
  SmMeas = FMath::Fmod(SmMeas + 360.f, 360.f);

  // --- 11) Kalman-style update ---
  float PredAng = PrevAng + PrevAngVel * DeltaTime;
  float PredVar = PrevVar + ProcNoise;
  float dA = FMath::FindDeltaAngleDegrees(PredAng, SmMeas);
  float K = PredVar / (PredVar + MeasNoise);
  float NewAng = PredAng + K * dA;
  float NewVar = (1.f - K) * PredVar;
  float NewVel = dA / DeltaTime;

  PrevAng = FMath::Fmod(NewAng + 360.f, 360.f);
  PrevVar = NewVar;
  PrevAngVel = NewVel;

  return PrevAng;
}

#pragma region INLINE Wrappers

FVector2D
UOHLocomotionUtils::ComputeBlendFactors(const FVector2D &Direction,
                                        const FRotator &CameraRotation,
                                        bool bNormalize) {
  return ComputeBlendFactors_Inline(Direction, CameraRotation, bNormalize);
}

EScreenDirection8 UOHLocomotionUtils::MapAngleTo8WayDirection(float Angle) {
  return MapAngleTo8WayDirection_Inline(Angle);
}

EScreenDirection4 UOHLocomotionUtils::MapAngleTo4WayDirection(float Angle) {
  return MapAngleTo4WayDirection_Inline(Angle);
}

EScreenDirectionQuadrant UOHLocomotionUtils::MapAngleToQuadrant(float Angle) {
  return MapAngleToQuadrant_Inline(Angle);
}

float UOHLocomotionUtils::PredictMovementAngleMonsterExtended(
    ACharacter *Character, const FRotator &FacingRotation, float DeltaTime,
    float &OutBezierT, float PredictionTime, float SpeedPredictionScale,
    float AccelPredictionScale, float JerkScale, float CurvatureScale,
    float MaxBlend, float PivotDeadzone, float PivotScale, float StrafeBias,
    float RotationThreshold, float MoveThreshold, float SGCurvThreshold,
    float FootPlantSpeedThresh, float FootPlantAccelThresh,
    float FootPlantHoldTime, float TeleportThreshold, float VarMin,
    float VarMax, float ProcNoise, float MeasNoise) {
  // — Static filter & history state —
  static bool bInit = false;
  static float PrevAng = 0.f, PrevVar = 0.f, PrevVel = 0.f;
  static float LastPlantTime = -1e6f;
  static FVector DirHist[7] = {FVector(1, 0, 0)};
  static int32 DirCount = 1;

  if (!bInit) {
    PrevVar = MeasNoise;
    bInit = true;
  }
  if (!Character || DeltaTime <= 0.f)
    return PrevAng;

  // — 1) Physics manager & bone data —
  auto *Phys = Character->FindComponentByClass<UOHPhysicsManager>();
  if (!Phys)
    return PrevAng;
  const FOHBoneData *RootBD = Phys->GetBoneData(TEXT("root"));
  if (!RootBD || !RootBD->HasMotionHistory())
    return PrevAng;

  // — 2) Root history & teleport detect —
  const auto &H = RootBD->GetMotionHistory();
  int32 Count = H.Num();
  if (Count < 3)
    return PrevAng;

  FVector C0 = H[0].GetLocation();
  FVector CN = H.Last().GetLocation();
  if (FVector::Dist(C0, CN) > TeleportThreshold) {
    PrevVar = MeasNoise;
    PrevVel = 0.f;
  }

  // — 3) Foot-plant anchoring —
  auto DetectPlant = [&](const FName &BoneName) -> float {
    const FOHBoneData *BD = Phys->GetBoneData(BoneName);
    if (!BD || !BD->HasMotionHistory())
      return -1.f;
    for (int32 i = BD->GetMotionHistory().Num() - 1; i >= 0; --i) {
      const auto &S = BD->GetMotionHistory()[i];
      if (S.GetLinearVelocity().Size() < FootPlantSpeedThresh &&
          BD->GetSmoothedLinearAcceleration().Size() > FootPlantAccelThresh) {
        return S.GetTimeStamp();
      }
    }
    return -1.f;
  };
  LastPlantTime =
      FMath::Max(LastPlantTime, FMath::Max(DetectPlant(TEXT("foot_l")),
                                           DetectPlant(TEXT("foot_r"))));
  float Now = H.Last().GetTimeStamp();

  // — 4) In-place rotation detection —
  {
    float Y0 = H[0].GetRotation().Rotator().Yaw;
    float Y1 = H.Last().GetRotation().Rotator().Yaw;
    float dY = FMath::FindDeltaAngleDegrees(Y0, Y1);
    float dP = FVector::Dist(C0, CN);
    if (FMath::Abs(dY) > RotationThreshold && dP < MoveThreshold) {
      return FMath::Fmod(FacingRotation.Yaw + dY + 360.f, 360.f);
    }
  }

  // — 5) Raw direction vector —
  FVector RawDir = (CN - C0).GetSafeNormal();
  if (RawDir.IsNearlyZero())
    RawDir = RootBD->GetSmoothedLinearVelocity().GetSafeNormal();
  if (RawDir.IsNearlyZero())
    return PrevAng;

  // — 6) Dynamic prediction horizon & output fraction —
  float FinalT =
      PredictionTime + Character->GetVelocity().Size() * SpeedPredictionScale;
  FinalT *= 1.f + (RootBD->GetSmoothedLinearAcceleration().Size() *
                   AccelPredictionScale);
  OutBezierT = FMath::Clamp(FinalT / PredictionTime, 0.f, 1.f);

  // — 7) Jerk & curvature blends —
  float JM = UOHAlgoUtils::EstimateJerk(H, DeltaTime).Size();
  float Curv = UOHAlgoUtils::EstimateCurvature(H);
  float Pblend = FMath::Clamp(JM * JerkScale, 0.f, MaxBlend);
  float Bblend = FMath::Clamp(Curv * CurvatureScale, 0.f, MaxBlend);

  // — 8) Polynomial least-squares fit → PolyAng —
  float PolyAng = 0.f;
  {
    // build sums for normal equations
    double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    double SX0 = 0, SX1 = 0, SX2 = 0;
    double SY0 = 0, SY1 = 0, SY2 = 0;
    for (int i = 0; i < Count; ++i) {
      double t = H[i].GetTimeStamp(), x = H[i].GetLocation().X,
             y = H[i].GetLocation().Y;
      double t2 = t * t, t3 = t2 * t, t4 = t2 * t2;
      S0 += 1;
      S1 += t;
      S2 += t2;
      S3 += t3;
      S4 += t4;
      SX0 += x;
      SX1 += x * t;
      SX2 += x * t2;
      SY0 += y;
      SY1 += y * t;
      SY2 += y * t2;
    }
    double Det = S0 * (S2 * S4 - S3 * S3) - S1 * (S1 * S4 - S3 * S2) +
                 S2 * (S1 * S3 - S2 * S2);
    if (FMath::Abs(Det) > KINDA_SMALL_NUMBER) {
      double A1 = (SX0 * (S2 * S4 - S3 * S3) - S1 * (SX1 * S4 - S3 * SX2) +
                   S2 * (SX1 * S3 - S2 * SX2)) /
                  Det;
      double B1 = (SY0 * (S2 * S4 - S3 * S3) - S1 * (SY1 * S4 - S3 * SY2) +
                   S2 * (SY1 * S3 - S2 * SY2)) /
                  Det;
      double A2 = (S0 * (SX1 * S4 - S3 * SX2) - SX0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SX2 - SX1 * S2)) /
                  Det;
      double B2 = (S0 * (SY1 * S4 - S3 * SY2) - SY0 * (S1 * S4 - S3 * S2) +
                   S2 * (S1 * SY2 - SY1 * S2)) /
                  Det;
      double tP = H.Last().GetTimeStamp();
      double dx = A1 + 2.0 * A2 * tP, dy = B1 + 2.0 * B2 * tP;
      if (FMath::Abs(dx) + FMath::Abs(dy) > SMALL_NUMBER) {
        FVector D(dx, dy, 0.f);
        FVector L = FacingRotation.UnrotateVector(D.GetSafeNormal());
        PolyAng = FMath::Fmod(
            FMath::RadiansToDegrees(FMath::Atan2(L.Y, L.X)) + 360.f, 360.f);
      }
    }
  }

  // — 9) Quadratic Bezier extrapolation → BezAng —
  TArray<FVector> CP =
      UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(*RootBD,
                                                                FinalT);
  const FVector &P0 = CP[0], &P1 = CP[1], &P2 = CP[2];
  FVector BezDir = (P2 - P1).GetSafeNormal();
  FVector LB = FacingRotation.UnrotateVector(BezDir);
  float BezAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LB.Y, LB.X)) + 360.f, 360.f);

  // —10) Measured local-space direction blend →
  FVector LR = FacingRotation.UnrotateVector(RawDir);
  float RawAng = FMath::Fmod(
      FMath::RadiansToDegrees(FMath::Atan2(LR.Y, LR.X)) + 360.f, 360.f);
  float MeasAng =
      FMath::Lerp(FMath::Lerp(RawAng, PolyAng, Pblend), BezAng, Bblend);

  // —11) Pivot anticipation & strafe bias on angle—
  {
    FVector Acc = RootBD->GetSmoothedLinearAcceleration();
    float Arc = FVector::CrossProduct(RawDir, Acc.GetSafeNormal()).Z;
    if (FMath::Abs(Arc) > PivotDeadzone)
      MeasAng += FMath::Clamp(Arc, -1.f, 1.f) * PivotScale;
    float BT = FMath::Lerp(180.f, 0.f, (StrafeBias + 1) * 0.5f);
    MeasAng = FMath::Fmod(
        FMath::Lerp(MeasAng, BT, FMath::Abs(StrafeBias)) + 360.f, 360.f);
  }

  // —12) Per-axis SG smoothing of direction vector—
  FVector MeasDir(FMath::Cos(FMath::DegreesToRadians(MeasAng)),
                  FMath::Sin(FMath::DegreesToRadians(MeasAng)), 0.f);
  // shift buffer
  if (DirCount < 7)
    ++DirCount;
  for (int i = 0; i < DirCount - 1; ++i)
    DirHist[i] = DirHist[i + 1];
  DirHist[DirCount - 1] = MeasDir;
  // choose window by curvature
  int Win = (Curv > SGCurvThreshold ? 5 : 7);
  static const float SG5[5] = {-3.f / 35, 12.f / 35, 17.f / 35, 12.f / 35,
                               -3.f / 35};
  static const float SG7[7] = {-2.f / 21, 3.f / 21, 6.f / 21, 7.f / 21,
                               6.f / 21,  3.f / 21, -2.f / 21};
  FVector S(0, 0, 0);
  if (DirCount >= Win) {
    const float *Cfs = (Win == 5 ? SG5 : SG7);
    int start = DirCount - Win;
    for (int i = 0; i < Win; ++i)
      S += DirHist[start + i] * Cfs[i];
  } else
    S = MeasDir;
  FVector SmDir = S.GetSafeNormal();
  float SmAng = FMath::RadiansToDegrees(FMath::Atan2(SmDir.Y, SmDir.X));
  SmAng = FMath::Fmod(SmAng + 360.f, 360.f);

  // —13) Kalman-style filter & variance clamp & foot-plant hold —
  float PredAng = PrevAng + PrevVel * DeltaTime;
  float PredVar = FMath::Clamp(PrevVar + ProcNoise, VarMin, VarMax);
  float dA = FMath::FindDeltaAngleDegrees(PredAng, SmAng);
  float K = PredVar / (PredVar + MeasNoise);
  float NewAng = PredAng + K * dA;
  float NewVar = FMath::Clamp((1 - K) * PredVar, VarMin, VarMax);
  float NewVel = dA / DeltaTime;

  if (Now - LastPlantTime < FootPlantHoldTime) {
    // lock heading
    NewAng = PrevAng;
    NewVel = 0.f;
  }

  PrevAng = FMath::Fmod(NewAng + 360.f, 360.f);
  PrevVar = NewVar;
  PrevVel = NewVel;

  return PrevAng;
}

#pragma endregion
