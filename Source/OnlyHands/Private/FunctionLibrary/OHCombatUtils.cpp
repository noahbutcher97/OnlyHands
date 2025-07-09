// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "FunctionLibrary/OHCombatUtils.h"

#include "Animation/AnimComposite.h"
#include "Animation/AnimNotifies/AnimNotifyState.h"
#include "AnimationRuntime.h"
#include "BoneContainer.h"
#include "Component/OHPhysicsManager.h"
#include "Components/CapsuleComponent.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "EngineUtils.h"
#include "FunctionLibrary/OHCollisionUtils.h"
#include "OHPhysicsStructs.h"
// #include "FunctionLibrary/OHGraphUtils.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "GameFramework/Character.h"
#include "Kismet/KismetMathLibrary.h"
#include "Pawn/OHCombatCharacter.h"

FVector2D UOHCombatUtils::ProjectVectorTo2D(const FVector &Vector) {
  return FVector2D(Vector.X, Vector.Y);
}

FVector UOHCombatUtils::MirrorVectorOverAxis(const FVector &Input,
                                             const FVector &Axis) {
  const FVector NormalizedAxis = Axis.GetSafeNormal();
  return Input -
         2.f * FVector::DotProduct(Input, NormalizedAxis) * NormalizedAxis;
}

float UOHCombatUtils::PredictTravelTime(const FVector &From, const FVector &To,
                                        float Speed) {
  if (Speed <= 0.f) {
    return -1.f; // Invalid case: cannot travel if speed is 0 or negative
  }

  const float Distance = FVector::Dist(From, To);
  return Distance / Speed;
}

FVector UOHCombatUtils::ApplyLinearDrag(const FVector &Velocity,
                                        float DragCoefficient,
                                        float DeltaTime) {
  float DragFactor = 1.f - (DragCoefficient * DeltaTime);
  DragFactor = FMath::Clamp(DragFactor, 0.f, 1.f);
  return Velocity * DragFactor;
}

FVector UOHCombatUtils::ApplyFriction(FVector Velocity, float Friction,
                                      float DeltaTime) {
  const float Speed = Velocity.Size();
  const float Drop = Speed * Friction * DeltaTime;
  const float NewSpeed = FMath::Max(Speed - Drop, 0.0f);
  return (Speed > 0.f) ? Velocity.GetSafeNormal() * NewSpeed
                       : FVector::ZeroVector;
}

float UOHCombatUtils::GetRelativeFootHeight(USkeletalMeshComponent *Mesh,
                                            FName FootBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector FootWorld = Mesh->GetBoneLocation(FootBone);
  FVector RootWorld = Mesh->GetComponentLocation();

  return FootWorld.Z - RootWorld.Z;
}

float UOHCombatUtils::GetLegSeparationRatio(USkeletalMeshComponent *Mesh,
                                            FName LeftFootBone,
                                            FName RightFootBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector L = Mesh->GetBoneLocation(LeftFootBone);
  FVector R = Mesh->GetBoneLocation(RightFootBone);

  float Distance = FVector::Dist2D(L, R);
  float Reference = FMath::Max(1.f, Mesh->Bounds.SphereRadius);
  return Distance / Reference;
}

bool UOHCombatUtils::TraceGroundBelowBone(USkeletalMeshComponent *Mesh,
                                          FName BoneName, float MaxDistance,
                                          FHitResult &OutHit) {
  if (!Mesh) {
    return false;
  }

  FVector Start = Mesh->GetBoneLocation(BoneName);
  FVector End = Start - FVector(0, 0, MaxDistance);

  FCollisionQueryParams Params;
  Params.AddIgnoredActor(Mesh->GetOwner());

  return Mesh->GetWorld()->LineTraceSingleByChannel(OutHit, Start, End,
                                                    ECC_Visibility, Params);
}

bool UOHCombatUtils::IsFootApproachingGround(USkeletalMeshComponent *Mesh,
                                             FName FootBone,
                                             FVector BoneVelocity,
                                             float MaxDistance,
                                             float MinDescendSpeed) {
  if (!Mesh) {
    return false;
  }

  FVector FootWorld = Mesh->GetBoneLocation(FootBone);
  FVector RootWorld = Mesh->GetComponentLocation();

  float VerticalOffset = FootWorld.Z - RootWorld.Z;
  bool bCloseEnough = VerticalOffset <= MaxDistance;
  bool bDescending = BoneVelocity.Z < MinDescendSpeed;

  return bCloseEnough && bDescending;
}

bool UOHCombatUtils::IsFootNearGround(USkeletalMeshComponent *Mesh,
                                      FName FootBone, float MaxDistance) {
  if (!Mesh) {
    return false;
  }

  FVector FootWorld = Mesh->GetBoneLocation(FootBone);
  FVector RootWorld = Mesh->GetComponentLocation();

  float VerticalOffset = FootWorld.Z - RootWorld.Z;
  return VerticalOffset <= MaxDistance;
}

float UOHCombatUtils::GetKneeRaiseHeight(USkeletalMeshComponent *Mesh,
                                         FName FootBone, FName PelvisBone) {
  if (!Mesh) {
    return 0.f;
  }

  float FootZ = Mesh->GetBoneLocation(FootBone).Z;
  float PelvisZ = Mesh->GetBoneLocation(PelvisBone).Z;
  return FootZ - PelvisZ;
}

float UOHCombatUtils::GetStrideWidth(USkeletalMeshComponent *Mesh,
                                     FName LeftFootBone, FName RightFootBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector L = Mesh->GetBoneLocation(LeftFootBone);
  FVector R = Mesh->GetBoneLocation(RightFootBone);

  return FMath::Abs(L.Y - R.Y);
}

float UOHCombatUtils::GetStepSyncRatio(FVector BoneVelocity, float MaxSpeed) {
  float Speed = FMath::Abs(BoneVelocity.Z);
  return FMath::Clamp(Speed / MaxSpeed, 0.f, 1.f);
}

int32 UOHCombatUtils::GetStepLeadFoot(USkeletalMeshComponent *Mesh,
                                      FName LeftFootBone, FName RightFootBone) {
  if (!Mesh) {
    return 0;
  }

  float LZ = Mesh->GetBoneLocation(LeftFootBone).Z;
  float RZ = Mesh->GetBoneLocation(RightFootBone).Z;

  return (LZ > RZ) ? 0 : 1;
}

float UOHCombatUtils::GetTorsoLeanAngle(USkeletalMeshComponent *Mesh,
                                        FName SpineBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector SpineForward = Mesh->GetBoneQuaternion(SpineBone).GetForwardVector();
  float Angle = FMath::RadiansToDegrees(FMath::Acos(
      FVector::DotProduct(SpineForward.GetSafeNormal(), FVector::UpVector)));
  return SpineForward.Z > 0 ? -Angle : Angle; // Negative = forward lean
}

float UOHCombatUtils::GetUpperBodyTwist(USkeletalMeshComponent *Mesh,
                                        FName LowerSpine, FName UpperSpine) {
  if (!Mesh) {
    return 0.f;
  }

  FVector LowerForward = Mesh->GetBoneQuaternion(LowerSpine).GetForwardVector();
  FVector UpperForward = Mesh->GetBoneQuaternion(UpperSpine).GetForwardVector();

  float Angle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(
      LowerForward.GetSafeNormal2D(), UpperForward.GetSafeNormal2D())));
  return Angle;
}

float UOHCombatUtils::GetPelvisCompressionRatio(USkeletalMeshComponent *Mesh,
                                                FName PelvisBone,
                                                float StandingHeight) {
  if (!Mesh) {
    return 0.f;
  }

  FVector PelvisWorld = Mesh->GetBoneLocation(PelvisBone);
  FVector RootWorld = Mesh->GetComponentLocation();

  float CurrentHeight = FMath::Max(1.f, RootWorld.Z - PelvisWorld.Z);
  float Ratio = CurrentHeight / StandingHeight;
  return FMath::Clamp(Ratio, 0.f, 1.f);
}

float UOHCombatUtils::GetSpineArchAngle(USkeletalMeshComponent *Mesh,
                                        FName UpperSpine, FName HeadBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector UpperSpineLoc = Mesh->GetBoneLocation(UpperSpine);
  FVector HeadLoc = Mesh->GetBoneLocation(HeadBone);

  FVector Direction = (HeadLoc - UpperSpineLoc).GetSafeNormal();
  float Angle = FMath::RadiansToDegrees(
      FMath::Acos(FVector::DotProduct(Direction, FVector::UpVector)));
  return Angle;
}

float UOHCombatUtils::GetCumulativeLean(USkeletalMeshComponent *Mesh) {
  if (!Mesh)
    return 0.f;

  // 1. Normalize lean angle (ideal range: 0–45 deg)
  const float LeanDeg = FMath::Abs(GetTorsoLeanAngle(Mesh));
  const float LeanScore =
      FMath::Clamp(LeanDeg / 45.f, 0.f, 1.f); // 1 = max lean

  // 2. Normalize twist angle (ideal range: 0–60 deg)
  const float TwistDeg = GetUpperBodyTwist(Mesh);
  const float TwistScore = FMath::Clamp(TwistDeg / 60.f, 0.f, 1.f);

  // 3. Invert compression so lower pelvis = higher value
  const float CompressionRatio = GetPelvisCompressionRatio(Mesh);
  const float CompressionScore = FMath::Clamp(1.f - CompressionRatio, 0.f, 1.f);

  // Weighted blend
  const float Composite =
      LeanScore * 0.4f + TwistScore * 0.2f + CompressionScore * 0.4f;

  return FMath::Clamp(Composite, 0.f, 1.f);
}

bool UOHCombatUtils::IsUpperBodyGuardDropped(USkeletalMeshComponent *Mesh,
                                             FName ChestBone, FName LeftElbow,
                                             FName RightElbow) {
  if (!Mesh) {
    return false;
  }

  FVector Chest = Mesh->GetBoneLocation(ChestBone);
  FVector ElbowL = Mesh->GetBoneLocation(LeftElbow);
  FVector ElbowR = Mesh->GetBoneLocation(RightElbow);

  float DistL = FVector::Dist(ElbowL, Chest);
  float DistR = FVector::Dist(ElbowR, Chest);

  float Threshold = 40.f; // Customize as needed
  return (DistL > Threshold || DistR > Threshold);
}

bool UOHCombatUtils::IsCharacterLeaningForward(USkeletalMeshComponent *Mesh,
                                               FName SpineBone) {
  if (!Mesh) {
    return false;
  }

  FVector SpineForward = Mesh->GetBoneQuaternion(SpineBone).GetForwardVector();
  FVector ActorForward = Mesh->GetOwner()->GetActorForwardVector();

  float Dot = FVector::DotProduct(SpineForward.GetSafeNormal2D(),
                                  ActorForward.GetSafeNormal2D());
  return Dot > 0.5f;
}

float UOHCombatUtils::GetArmExtensionRatio(USkeletalMeshComponent *Mesh,
                                           FName ClavicleBone, FName HandBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector Clavicle = Mesh->GetBoneLocation(ClavicleBone);
  FVector Hand = Mesh->GetBoneLocation(HandBone);
  float Distance = FVector::Dist(Clavicle, Hand);

  float Reference = 60.f; // Typical max arm extension in cm
  return FMath::Clamp(Distance / Reference, 0.f, 1.f);
}

bool UOHCombatUtils::IsArmCrossedMidline(USkeletalMeshComponent *Mesh,
                                         FName HandBone, FName PelvisBone) {
  if (!Mesh) {
    return false;
  }

  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector Pelvis = Mesh->GetBoneLocation(PelvisBone);

  float LocalX = Mesh->GetComponentTransform().InverseTransformPosition(Hand).Y;
  float PelvisX =
      Mesh->GetComponentTransform().InverseTransformPosition(Pelvis).Y;

  return (LocalX * PelvisX < 0); // Opposite sides of centerline
}

float UOHCombatUtils::GetShoulderCompressionRatio(USkeletalMeshComponent *Mesh,
                                                  FName ClavicleLeft,
                                                  FName ClavicleRight) {
  if (!Mesh) {
    return 0.f;
  }

  FVector L = Mesh->GetBoneLocation(ClavicleLeft);
  FVector R = Mesh->GetBoneLocation(ClavicleRight);

  float Dist = FVector::Dist(L, R);
  float Reference = 35.f; // Typical neutral shoulder width

  return 1.f - FMath::Clamp(Dist / Reference, 0.f, 1.f);
}

float UOHCombatUtils::GetHandReachExtent(USkeletalMeshComponent *Mesh,
                                         FName HandBone, FName PelvisBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector Pelvis = Mesh->GetBoneLocation(PelvisBone);

  return FVector::Dist(Hand, Pelvis);
}

float UOHCombatUtils::GetCrossBodyReach(USkeletalMeshComponent *Mesh,
                                        FName HandBone, FName PelvisBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector Hand = Mesh->GetComponentTransform().InverseTransformPosition(
      Mesh->GetBoneLocation(HandBone));
  return FMath::Abs(Hand.Y);
}

FVector UOHCombatUtils::GetArmLineDirection(USkeletalMeshComponent *Mesh,
                                            FName ClavicleBone,
                                            FName HandBone) {
  if (!Mesh) {
    return FVector::ZeroVector;
  }

  FVector Clavicle = Mesh->GetBoneLocation(ClavicleBone);
  FVector Hand = Mesh->GetBoneLocation(HandBone);

  return (Hand - Clavicle).GetSafeNormal();
}

bool UOHCombatUtils::IsHandWithinGuardZone(USkeletalMeshComponent *Mesh,
                                           FName HandBone, FName ChestBone,
                                           float MaxDistance) {
  if (!Mesh) {
    return false;
  }

  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector Chest = Mesh->GetBoneLocation(ChestBone);

  float Distance = FVector::Dist(Hand, Chest);
  return Distance <= MaxDistance;
}

bool UOHCombatUtils::IsArmAlignedWithMovement(USkeletalMeshComponent *Mesh,
                                              FName ClavicleBone,
                                              FName HandBone,
                                              FVector ActorVelocity,
                                              float ThresholdDegrees) {
  if (!Mesh || ActorVelocity.IsNearlyZero()) {
    return false;
  }

  FVector Clavicle = Mesh->GetBoneLocation(ClavicleBone);
  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector ArmDirection = (Hand - Clavicle).GetSafeNormal();
  FVector VelocityDir = ActorVelocity.GetSafeNormal();

  float Dot = FVector::DotProduct(ArmDirection, VelocityDir);
  float Angle =
      FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));

  return Angle <= ThresholdDegrees;
}

float UOHCombatUtils::GetArmToFacingAngle(USkeletalMeshComponent *Mesh,
                                          FName ClavicleBone, FName HandBone) {
  if (!Mesh) {
    return 0.f;
  }

  FVector Clavicle = Mesh->GetBoneLocation(ClavicleBone);
  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector ArmVector = (Hand - Clavicle).GetSafeNormal();
  FVector Facing = Mesh->GetOwner()->GetActorForwardVector().GetSafeNormal();

  float Dot = FVector::DotProduct(ArmVector, Facing);
  return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
}

float UOHCombatUtils::GetStrikePrepAlignmentScore(USkeletalMeshComponent *Mesh,
                                                  FName ClavicleBone,
                                                  FName HandBone,
                                                  FVector ActorVelocity) {
  if (!Mesh) {
    return 0.f;
  }

  // Arm extension score
  float ExtensionRatio = GetArmExtensionRatio(
      Mesh, ClavicleBone, HandBone); // 0 = retracted, 1 = fully extended
  float ExtensionScore =
      1.f - FMath::Abs(ExtensionRatio - 0.7f); // Best score near 0.7

  // Lean score
  float LeanAngle =
      FMath::Abs(GetTorsoLeanAngle(Mesh)); // 0 = upright, 90 = forward lean
  float LeanScore =
      FMath::Clamp(LeanAngle / 45.f, 0.f, 1.f); // Best in range [20�45 deg]

  // Motion alignment score
  float MotionAngle = GetArmToFacingAngle(Mesh, ClavicleBone, HandBone);
  float MotionScore = 1.f - FMath::Clamp(MotionAngle / 90.f, 0.f,
                                         1.f); // Best aligned with facing

  float Composite =
      (ExtensionScore * 0.4f) + (LeanScore * 0.3f) + (MotionScore * 0.3f);
  return FMath::Clamp(Composite, 0.f, 1.f);
}

bool UOHCombatUtils::IsStrikeArcingTowardTarget(USkeletalMeshComponent *Mesh,
                                                FName ClavicleBone,
                                                FName HandBone, AActor *Target,
                                                float ThresholdDegrees) {
  if (!Mesh || !Target) {
    return false;
  }

  FVector Clavicle = Mesh->GetBoneLocation(ClavicleBone);
  FVector Hand = Mesh->GetBoneLocation(HandBone);
  FVector ArmVector = (Hand - Clavicle).GetSafeNormal();

  FVector TargetVector =
      (Target->GetActorLocation() - Clavicle).GetSafeNormal();

  float Dot = FVector::DotProduct(ArmVector, TargetVector);
  float Angle =
      FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));

  return Angle <= ThresholdDegrees;
}

FRotator
UOHCombatUtils::GetActorRotationRelativeToCamera(const AActor *TargetActor) {
  if (!TargetActor) {
    return FRotator::ZeroRotator;
  }

  FVector CameraLoc;
  FRotator CameraRot;
  if (!GetPlayerCameraView(TargetActor, CameraLoc, CameraRot)) {
    return FRotator::ZeroRotator;
  }

  const FRotator ActorRot = TargetActor->GetActorRotation();
  return UKismetMathLibrary::NormalizedDeltaRotator(ActorRot, CameraRot);
}

FVector UOHCombatUtils::GetActorDirectionFromCamera(const AActor *TargetActor) {
  if (!TargetActor) {
    return FVector::ZeroVector;
  }

  FVector CameraLoc;
  FRotator CameraRot;
  if (!GetPlayerCameraView(TargetActor, CameraLoc, CameraRot)) {
    return FVector::ZeroVector;
  }

  const FVector ActorLoc = TargetActor->GetActorLocation();
  return (ActorLoc - CameraLoc).GetSafeNormal();
}

float UOHCombatUtils::GetActorFacingAngleRelativeToCamera(
    const AActor *TargetActor) {
  if (!TargetActor) {
    return 0.f;
  }

  FVector CameraLoc;
  FRotator CameraRot;
  if (!GetPlayerCameraView(TargetActor, CameraLoc, CameraRot)) {
    return 0.f;
  }

  FVector ActorForward = TargetActor->GetActorForwardVector();
  FVector CameraForward = CameraRot.Vector();

  ActorForward.Z = 0.f;
  CameraForward.Z = 0.f;

  ActorForward.Normalize();
  CameraForward.Normalize();

  // Signed angle from camera forward to actor forward
  const float Angle = FMath::RadiansToDegrees(
      FMath::Atan2(FVector::CrossProduct(CameraForward, ActorForward).Z,
                   FVector::DotProduct(CameraForward, ActorForward)));

  return Angle;
}

float UOHCombatUtils::GetActorMovementDirectionRelativeToCamera(
    const AActor *TargetActor) {
  if (!TargetActor) {
    return 0.f;
  }

  const FVector Velocity = TargetActor->GetVelocity();
  if (Velocity.IsNearlyZero()) {
    return 0.f;
  }

  FVector CameraLoc;
  FRotator CameraRot;
  if (!GetPlayerCameraView(TargetActor, CameraLoc, CameraRot)) {
    return 0.f;
  }

  FVector CameraForward = CameraRot.Vector();
  FVector MoveDirection = Velocity;

  // Flatten
  CameraForward.Z = 0.f;
  MoveDirection.Z = 0.f;

  CameraForward.Normalize();
  MoveDirection.Normalize();

  const float Angle = FMath::RadiansToDegrees(
      FMath::Atan2(FVector::CrossProduct(CameraForward, MoveDirection).Z,
                   FVector::DotProduct(CameraForward, MoveDirection)));

  return Angle;
}

bool UOHCombatUtils::GetPlayerCameraView(const UObject *ContextObject,
                                         FVector &OutCameraLocation,
                                         FRotator &OutCameraRotation) {
  OutCameraLocation = FVector::ZeroVector;
  OutCameraRotation = FRotator::ZeroRotator;

  if (!ContextObject || !GEngine) {
    return false;
  }

  UWorld *World = GEngine->GetWorldFromContextObjectChecked(ContextObject);
  if (!World || !GEngine->GameViewport) {
    return false;
  }

  ULocalPlayer *LocalPlayer =
      GEngine->GameViewport->GetGameInstance()->FindLocalPlayerFromControllerId(
          0);
  if (!LocalPlayer) {
    return false;
  }

  APlayerController *PC = LocalPlayer->GetPlayerController(World);
  if (!PC || !PC->PlayerCameraManager) {
    return false;
  }

  OutCameraLocation = PC->PlayerCameraManager->GetCameraLocation();
  OutCameraRotation = PC->PlayerCameraManager->GetCameraRotation();
  return true;
}

void UOHCombatUtils::DebugDrawActorMovementDirection(const AActor *TargetActor,
                                                     float Duration,
                                                     FColor Color) {
  if (!TargetActor) {
    return;
  }

  const float MoveAngle =
      GetActorMovementDirectionRelativeToCamera(TargetActor);
  const FString Label =
      FString::Printf(TEXT("Move Direction Angle: %.1f°"), MoveAngle);

  const FVector Offset = FVector(0, 0, 120.f);
  DrawDebugString(TargetActor->GetWorld(),
                  TargetActor->GetActorLocation() + Offset, Label, nullptr,
                  Color, Duration, false);
}

bool UOHCombatUtils::IsActorWithinRange(const AActor *Source,
                                        const AActor *Target, float MaxDistance,
                                        float Radius) {
  if (!Source || !Target) {
    return false;
  }

  const FVector Start = Source->GetActorLocation();
  const FVector End = Target->GetActorLocation();

  const float Distance = FVector::Dist(Start, End);

  if (Radius > 0.f) {
    // Interprets range as a radius-adjusted band
    return Distance <= (MaxDistance + Radius);
  }

  return Distance <= MaxDistance;
}

EOHHitDirection UOHCombatUtils::ClassifyHitDirection(const FHitResult &Hit,
                                                     const AActor *TargetActor,
                                                     bool bUse8Directions) {
  if (!TargetActor || !Hit.bBlockingHit) {
    return EOHHitDirection::None;
  }

  // Calculate hit direction vector (from impact to trace origin)
  FVector HitDirection = (Hit.TraceStart - Hit.ImpactPoint).GetSafeNormal();
  FVector TargetForward = TargetActor->GetActorForwardVector();

  // Flatten to horizontal plane
  HitDirection.Z = 0.f;
  TargetForward.Z = 0.f;

  HitDirection.Normalize();
  TargetForward.Normalize();

  // Signed angle around up-axis (Yaw)
  float YawAngle = FMath::RadiansToDegrees(
      FMath::Atan2(FVector::CrossProduct(TargetForward, HitDirection).Z,
                   FVector::DotProduct(TargetForward, HitDirection)));
  YawAngle = FMath::UnwindDegrees(YawAngle);

  if (bUse8Directions) {
    if (YawAngle > -22.5f && YawAngle <= 22.5f) {
      return EOHHitDirection::Front;
    }
    if (YawAngle > 22.5f && YawAngle <= 67.5f) {
      return EOHHitDirection::FrontRight;
    }
    if (YawAngle > 67.5f && YawAngle <= 112.5f) {
      return EOHHitDirection::Right;
    }
    if (YawAngle > 112.5f && YawAngle <= 157.5f) {
      return EOHHitDirection::BackRight;
    }
    if (YawAngle > 157.5f || YawAngle <= -157.5f) {
      return EOHHitDirection::Back;
    }
    if (YawAngle > -157.5f && YawAngle <= -112.5f) {
      return EOHHitDirection::BackLeft;
    }
    if (YawAngle > -112.5f && YawAngle <= -67.5f) {
      return EOHHitDirection::Left;
    }
    if (YawAngle > -67.5f && YawAngle <= -22.5f) {
      return EOHHitDirection::FrontLeft;
    }
  } else {
    if (YawAngle > -45.f && YawAngle <= 45.f) {
      return EOHHitDirection::Front;
    }
    if (YawAngle > 45.f && YawAngle <= 135.f) {
      return EOHHitDirection::Right;
    }
    if (YawAngle < -45.f && YawAngle >= -135.f) {
      return EOHHitDirection::Left;
    }
    return EOHHitDirection::Back;
  }

  return EOHHitDirection::None;
}

void UOHCombatUtils::DebugDrawHitDirection(const FHitResult &Hit,
                                           const AActor *TargetActor,
                                           bool bUse8Directions, float Duration,
                                           float ArrowSize) {
  if (!TargetActor || !Hit.bBlockingHit) {
    return;
  }

  const FVector HitLocation = Hit.ImpactPoint;
  FVector TraceDirection = (Hit.TraceStart - Hit.ImpactPoint).GetSafeNormal();
  TraceDirection.Z = 0.f;

  const EOHHitDirection Dir =
      ClassifyHitDirection(Hit, TargetActor, bUse8Directions);

  FColor Color;
  switch (Dir) {
  case EOHHitDirection::Front:
    Color = FColor::Green;
    break;
  case EOHHitDirection::FrontRight:
    Color = FColor::Emerald;
    break;
  case EOHHitDirection::Right:
    Color = FColor::Blue;
    break;
  case EOHHitDirection::BackRight:
    Color = FColor::Cyan;
    break;
  case EOHHitDirection::Back:
    Color = FColor::Red;
    break;
  case EOHHitDirection::BackLeft:
    Color = FColor::Magenta;
    break;
  case EOHHitDirection::Left:
    Color = FColor::Yellow;
    break;
  case EOHHitDirection::FrontLeft:
    Color = FColor::Orange;
    break;
  default:
    Color = FColor::White;
    break;
  }

  DrawDebugDirectionalArrow(TargetActor->GetWorld(), HitLocation,
                            HitLocation + TraceDirection * ArrowSize,
                            ArrowSize * 0.25f, Color, false, Duration, 0, 2.f);

  DrawDebugString(TargetActor->GetWorld(), HitLocation + FVector(0, 0, 50.f),
                  UEnum::GetValueAsString(Dir), nullptr, Color, Duration,
                  false);
}

#pragma region Animation

FStrikeContactMetrics
UOHCombatUtils::ComputeStrikeContactMetrics(const FVector &ContactNormal,
                                            const FVector &Velocity,
                                            float PenetrationDepth) {
  FStrikeContactMetrics Metrics;

  const FVector Normal = ContactNormal.GetSafeNormal();
  const FVector Vel = Velocity;

  const float Dot = FVector::DotProduct(Normal, Vel.GetSafeNormal());
  const float AngleDeg =
      FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));

  Metrics.ContactNormal = Normal;
  Metrics.Velocity = Vel;
  Metrics.VelocityDotNormal = Dot;
  Metrics.ImpactAngleDegrees = AngleDeg;

  // Add scaled force (penetration × aligned speed)
  const float Speed = Vel.Size();
  Metrics.EstimatedForce = Speed * PenetrationDepth * FMath::Abs(Dot);

  return Metrics;
}

FVector UOHCombatUtils::GetBoneVelocitySafe(USkeletalMeshComponent *Mesh,
                                            FName BoneName, float DeltaTime) {
  if (!Mesh || DeltaTime <= 0.f) {
    return FVector::ZeroVector;
  }

  if (Mesh->IsSimulatingPhysics(BoneName)) {
    return Mesh->GetPhysicsLinearVelocity(BoneName);
  }

  // Fallback: approximate from previous frame location
  static TMap<TPair<USkeletalMeshComponent *, FName>, FVector> PrevPositions;

  const FVector Current = Mesh->GetBoneLocation(BoneName);
  TPair<USkeletalMeshComponent *, FName> Key(Mesh, BoneName);

  const FVector Prev = PrevPositions.FindRef(Key);
  PrevPositions.Add(Key, Current);

  return (Current - Prev) / DeltaTime;
}

void UOHCombatUtils::BeginDeferredStrikeSweep(FDeferredStrikeSweepState &State,
                                              USkeletalMeshComponent *Mesh,
                                              AActor *Owner,
                                              const TArray<FName> &BoneChain,
                                              float SampleInterval) {
  State.Mesh = Mesh;
  State.Owner = Owner;
  State.BoneChain = BoneChain;
  State.SampleInterval = SampleInterval;
  State.AccumulatedTime = 0.f;
  State.TimeSinceLastSample = 0.f;
  State.HitCharacters.Empty();
  State.Contacts.Empty();
}

void UOHCombatUtils::TickDeferredStrikeSweep(FDeferredStrikeSweepState &State,
                                             float DeltaTime) {
  if (!State.Mesh || !State.Owner) {
    return;
  }

  State.AccumulatedTime += DeltaTime;
  State.TimeSinceLastSample += DeltaTime;

  if (State.TimeSinceLastSample >= State.SampleInterval) {
    TArray<FStrikeContactMetrics> SweepContacts;
    TArray<ACharacter *> SweepHits;

    UOHCollisionUtils::SweepBoneChainWithMetrics(
        State.Mesh, State.BoneChain, State.Owner, SweepContacts, SweepHits,
        true, false, State.SampleInterval);

    for (ACharacter *Hit : SweepHits) {
      if (!State.HitCharacters.Contains(Hit)) {
        State.HitCharacters.Add(Hit);
      }
    }

    State.Contacts.Append(SweepContacts);
    State.TimeSinceLastSample = 0.f;
  }
}

bool UOHCombatUtils::GetActiveMontageBlendWeight(USkeletalMeshComponent *Mesh,
                                                 float &OutBlendWeight,
                                                 FName &OutCurrentSectionName) {
  OutBlendWeight = 0.f;
  OutCurrentSectionName = NAME_None;

  if (!Mesh || !Mesh->GetAnimInstance()) {
    return false;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  const FAnimMontageInstance *ActiveMontage =
      AnimInstance->GetActiveMontageInstance();

  if (!ActiveMontage || !ActiveMontage->Montage) {
    return false;
  }

  OutBlendWeight = ActiveMontage->GetWeight();

  // Optional: Get current section name
  const int32 SectionIndex =
      ActiveMontage->Montage->GetSectionIndexFromPosition(
          ActiveMontage->GetPosition());
  if (SectionIndex != INDEX_NONE) {
    OutCurrentSectionName =
        ActiveMontage->Montage->GetSectionName(SectionIndex);
  }

  return true;
}

FOHMontagePlaybackState
UOHCombatUtils::GetMontagePlaybackState(USkeletalMeshComponent *Mesh) {
  FOHMontagePlaybackState State;

  if (!Mesh || !Mesh->GetAnimInstance()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[MontagePlaybackState] Invalid mesh or anim instance."));
    return State;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  const FAnimMontageInstance *MontageInstance =
      AnimInstance->GetActiveMontageInstance();

  if (!MontageInstance || !MontageInstance->Montage) {
    UE_LOG(LogTemp, Warning, TEXT("[MontagePlaybackState] No active montage."));
    return State;
  }

  const UAnimMontage *Montage = MontageInstance->Montage;
  const float Position = MontageInstance->GetPosition();
  const int32 SectionIndex = Montage->GetSectionIndexFromPosition(Position);

  State.MontageName = Montage->GetFName();
  State.Position = Position;
  State.BlendWeight = MontageInstance->GetWeight();

  if (SectionIndex != INDEX_NONE) {
    State.SectionName = Montage->GetSectionName(SectionIndex);
    State.SectionLength = Montage->GetSectionLength(SectionIndex);

    const float SectionStart =
        Montage->CompositeSections[SectionIndex].GetTime();
    State.NormalizedTime =
        (State.SectionLength > 0.f)
            ? FMath::Clamp((Position - SectionStart) / State.SectionLength, 0.f,
                           1.f)
            : 0.f;
  } else {
    UE_LOG(LogTemp, Warning,
           TEXT("[MontagePlaybackState] Could not resolve section from "
                "playback position %.3f"),
           Position);
  }

  // Extract UAnimSequence and segment-relative info
  for (const FSlotAnimationTrack &Slot : Montage->SlotAnimTracks) {
    for (const FAnimSegment &Segment : Slot.AnimTrack.AnimSegments) {
      const float SegmentStart = Segment.StartPos;
      const float SegmentEnd = SegmentStart + Segment.GetLength();

      if (Position >= SegmentStart && Position <= SegmentEnd) {
        if (UAnimSequence *Seq =
                Cast<UAnimSequence>(Segment.GetAnimReference())) {
          State.CurrentAnimSequence = Seq;
          State.SegmentStartTime = SegmentStart;
          State.SegmentLength = Segment.GetLength();
          State.SegmentPlayTime = Position - SegmentStart;
          State.SegmentNormalizedTime =
              (State.SegmentLength > 0.f)
                  ? FMath::Clamp(State.SegmentPlayTime / State.SegmentLength,
                                 0.f, 1.f)
                  : 0.f;
          break;
        }
      }
    }
    if (State.CurrentAnimSequence)
      break;
  }

  if (!State.CurrentAnimSequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[MontagePlaybackState] Could not extract UAnimSequence from "
                "current segment."));
  }

  State.bIsValid = true;
  return State;
}

FOHAnimInstancePlaybackState
UOHCombatUtils::GetAnimInstancePlaybackState(USkeletalMeshComponent *Mesh) {
  FOHAnimInstancePlaybackState State;

  if (!Mesh)
    return State;

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  if (!AnimInstance)
    return State;

  State.bIsValid = true;

  // Check if a Montage is active
  const FAnimMontageInstance *MontageInstance =
      AnimInstance->GetActiveMontageInstance();
  if (MontageInstance && MontageInstance->Montage) {
    State.bIsMontagePlaying = true;
    State.ActiveMontage = MontageInstance->Montage;
    State.ActiveAsset = MontageInstance->Montage;
    State.PlayPosition = MontageInstance->GetPosition();
    State.PlayRate =
        MontageInstance->Montage->RateScale; // Not per-instance, but usable
    State.MontageBlendWeight = MontageInstance->GetWeight();

    const int32 SectionIndex =
        MontageInstance->Montage->GetSectionIndexFromPosition(
            State.PlayPosition);
    if (SectionIndex != INDEX_NONE) {
      State.CurrentMontageSection =
          MontageInstance->Montage->GetSectionName(SectionIndex);
    }
  }

  // If not a montage, we cannot safely access sequence state without engine
  // internals
  return State;
}

FOHSlotBlendState
UOHCombatUtils::GetSlotBlendState(USkeletalMeshComponent *Mesh,
                                  FName SlotName) {
  FOHSlotBlendState State;
  State.SlotName = SlotName;

  if (!Mesh || SlotName.IsNone())
    return State;

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  if (!AnimInstance)
    return State;

  const TArray<FAnimMontageInstance *> &MontageInstances =
      AnimInstance->MontageInstances;

  for (FAnimMontageInstance *MontageInstance : MontageInstances) {
    if (!MontageInstance || !MontageInstance->Montage)
      continue;

    for (const FSlotAnimationTrack &SlotTrack :
         MontageInstance->Montage->SlotAnimTracks) {
      if (SlotTrack.SlotName == SlotName) {
        State.bIsSlotActive = true;
        State.BlendWeight = MontageInstance->GetWeight();
        State.ActiveMontage = MontageInstance->Montage;
        return State; // Assuming one active montage per slot
      }
    }
  }

  return State;
}

FOHRootMotionState
UOHCombatUtils::GetPoseRootMotionState(USkeletalMeshComponent *Mesh,
                                       float DeltaTime) {
  FOHRootMotionState State;
  State.DeltaTime = DeltaTime;

  if (!Mesh || DeltaTime <= 0.f)
    return State;

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  if (!AnimInstance)
    return State;

  const FAnimMontageInstance *MontageInstance =
      AnimInstance->GetActiveMontageInstance();
  if (!MontageInstance || !MontageInstance->Montage)
    return State;

  const float CurrentTime = MontageInstance->GetPosition();
  const float FutureTime = CurrentTime + DeltaTime;

  // Extract root motion over delta time
  const FTransform DeltaTransform =
      MontageInstance->Montage->ExtractRootMotionFromTrackRange(CurrentTime,
                                                                FutureTime);
  State.RootMotionDelta = DeltaTransform;

  const FVector Translation = DeltaTransform.GetTranslation();
  const FRotator Rotation = DeltaTransform.GetRotation().Rotator();

  State.LinearVelocity = Translation / DeltaTime;
  State.RotationDelta = Rotation;
  State.bHasRootMotion =
      !Translation.IsNearlyZero() || !Rotation.IsNearlyZero();

  return State;
}

FOHMontageSectionWindow
UOHCombatUtils::GetAnimSectionTransitionWindow(USkeletalMeshComponent *Mesh,
                                               float CancelWindowStart,
                                               float CancelWindowEnd) {
  FOHMontageSectionWindow State;

  if (!Mesh)
    return State;

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  if (!AnimInstance)
    return State;

  const FAnimMontageInstance *MontageInstance =
      AnimInstance->GetActiveMontageInstance();
  if (!MontageInstance || !MontageInstance->Montage)
    return State;

  const float Position = MontageInstance->GetPosition();
  const UAnimMontage *Montage = MontageInstance->Montage;

  const int32 SectionIndex = Montage->GetSectionIndexFromPosition(Position);
  if (SectionIndex == INDEX_NONE)
    return State;

  const float SectionStart = Montage->CompositeSections[SectionIndex].GetTime();
  const float SectionLength = Montage->GetSectionLength(SectionIndex);
  const float LocalTime = Position - SectionStart;

  const float NormalizedTime =
      (SectionLength > 0.f) ? FMath::Clamp(LocalTime / SectionLength, 0.f, 1.f)
                            : 0.f;

  State.SectionName = Montage->GetSectionName(SectionIndex);
  State.NormalizedTime = NormalizedTime;
  State.RawPlayTime = LocalTime;
  State.SectionLength = SectionLength;

  State.bCanCancel = (NormalizedTime >= CancelWindowStart &&
                      NormalizedTime <= CancelWindowEnd);
  State.bCanChain = (NormalizedTime >= CancelWindowEnd);
  State.bIsLocked = (NormalizedTime < CancelWindowStart);

  return State;
}

void UOHCombatUtils::DrawPredictedRootMotionArc(
    const UObject *WorldContextObject, const FVector &StartLocation,
    const FOHRootMotionState &RootMotion, FColor Color, float Duration,
    float Thickness) {
  if (!WorldContextObject || !RootMotion.bHasRootMotion)
    return;

  UWorld *World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
  if (!World)
    return;

  const FVector EndLocation =
      StartLocation + RootMotion.RootMotionDelta.GetTranslation();

  // Draw trajectory line
  DrawDebugLine(World, StartLocation, EndLocation, Color, false, Duration, 0,
                Thickness);

  // Optional: draw direction arrow
  DrawDebugDirectionalArrow(World, StartLocation, EndLocation, 30.f, Color,
                            false, Duration, 0, Thickness);
}

bool UOHCombatUtils::GetMontageSectionTimeRange(UAnimMontage *Montage,
                                                FName SectionName,
                                                float &OutStartTime,
                                                float &OutEndTime) {
  OutStartTime = 0.f;
  OutEndTime = 0.f;

  if (!Montage || SectionName.IsNone()) {
    return false;
  }

  const int32 Index = Montage->GetSectionIndex(SectionName);
  if (!Montage->CompositeSections.IsValidIndex(Index)) {
    return false;
  }

  Montage->GetSectionStartAndEndTime(Index, OutStartTime, OutEndTime);
  //	OutStartTime = Montage->CompositeSections[Index].GetTime();
  //	OutEndTime = OutStartTime + Montage->GetSectionLength(Index);
  return true;
}

bool UOHCombatUtils::GetMontageSectionNormalizedTime(UAnimMontage *Montage,
                                                     FName SectionName,
                                                     float PlaybackTime,
                                                     float &OutNormalizedTime) {
  OutNormalizedTime = 0.f;

  if (!Montage || SectionName.IsNone())
    return false;

  const int32 SectionIndex = Montage->GetSectionIndex(SectionName);
  if (!Montage->CompositeSections.IsValidIndex(SectionIndex))
    return false;

  const float Start = Montage->CompositeSections[SectionIndex].GetTime();
  const float Length = Montage->GetSectionLength(SectionIndex);

  if (PlaybackTime < Start || PlaybackTime > Start + Length)
    return false;

  OutNormalizedTime =
      (Length > 0.f) ? FMath::Clamp((PlaybackTime - Start) / Length, 0.f, 1.f)
                     : 0.f;

  return true;
}

bool UOHCombatUtils::IsInMontageSectionWindow(
    const FOHMontagePlaybackState &PlaybackState, float WindowStart,
    float WindowEnd) {
  if (!PlaybackState.bIsValid) {
    UE_LOG(LogTemp, Warning,
           TEXT("[IsInMontageSectionWindow] PlaybackState is invalid."));
    return false;
  }

  if (WindowStart < 0.f || WindowEnd > 1.f || WindowEnd <= WindowStart) {
    UE_LOG(LogTemp, Warning,
           TEXT("[IsInMontageSectionWindow] Invalid window range: Start=%.2f "
                "End=%.2f"),
           WindowStart, WindowEnd);
    return false;
  }

  const float T = PlaybackState.NormalizedTime;
  const bool bInWindow = (T >= WindowStart && T <= WindowEnd);

  // Optional log for debugging
  // UE_LOG(LogTemp, Log, TEXT("[IsInMontageSectionWindow] NormalizedTime=%.3f
  // InWindow=%s"), T, bInWindow ? TEXT("YES") : TEXT("NO"));

  return bInWindow;
}

TArray<FInputWindow>
UOHCombatUtils::ExtractInputWindowsFromMontage(UAnimMontage *Montage,
                                               FName SectionName) {

  if (!Montage)
    return TArray<FInputWindow>();

  TArray<FInputWindow> OutWindows;

  struct FWindowTimeRange {
    float Start;
    float End;
    FName Name;
    FLinearColor Color;
  };

  // Step 1: Find the section
  int32 SectionIndex = Montage->GetSectionIndex(SectionName);
  if (SectionIndex == INDEX_NONE) {
    UE_LOG(LogTemp, Warning, TEXT("Invalid Montage section name: %s"),
           *SectionName.ToString());
    return OutWindows;
  }

  float SectionStartTime;
  float SectionEndTime;
  Montage->GetSectionStartAndEndTime(SectionIndex, SectionStartTime,
                                     SectionEndTime);

  TArray<FWindowTimeRange> Ranges;

  // Step 1: Extract all notify states of interest
  for (const FAnimNotifyEvent &NotifyEvent : Montage->Notifies) {
    const float NotifyStartTime = NotifyEvent.GetTriggerTime();
    const float NotifyEndTime = NotifyEvent.GetEndTriggerTime();

    if (NotifyStartTime >= SectionStartTime &&
        NotifyStartTime < SectionEndTime) {
      UInputWindowNotifyState *Notify =
          Cast<UInputWindowNotifyState>(NotifyEvent.NotifyStateClass);
      if (Notify) {
        FWindowTimeRange Range;
        Range.Start = NotifyStartTime;
        Range.End = NotifyEndTime;
        Range.Name = FName(Notify->WindowName);

        if (Range.Name == "Chain")
          Range.Color = FLinearColor::Green;
        else if (Range.Name == "Cancel")
          Range.Color = FLinearColor::Red;
        else
          Range.Color = FLinearColor::White;

        Ranges.Add(Range);
      }
    }
  }

  // Step 2: Sort by start time
  Ranges.Sort([](const FWindowTimeRange &A, const FWindowTimeRange &B) {
    return A.Start < B.Start;
  });

  float CurrentTime = SectionStartTime;

  // Step 3: Generate windows, including gaps
  for (const FWindowTimeRange &Range : Ranges) {
    if (Range.Start > CurrentTime) {
      // Add a "None" dead window before this range
      FInputWindow DeadWindow;
      DeadWindow.StartTime = CurrentTime - SectionStartTime;
      DeadWindow.EndTime = Range.Start - SectionStartTime;
      DeadWindow.WindowName = FName("None");
      DeadWindow.WindowColor = FLinearColor::White;
      OutWindows.Add(DeadWindow);
    }

    // Add the actual input window
    FInputWindow ActiveWindow;
    ActiveWindow.StartTime = Range.Start - SectionStartTime;
    ActiveWindow.EndTime = Range.End - SectionStartTime;
    ActiveWindow.WindowName = Range.Name;
    ActiveWindow.WindowColor = Range.Color;
    OutWindows.Add(ActiveWindow);

    CurrentTime = Range.End;
  }

  // Step 4: Final "None" window if needed
  if (CurrentTime < SectionEndTime) {
    FInputWindow DeadWindow;
    DeadWindow.StartTime = CurrentTime - SectionStartTime;
    DeadWindow.EndTime = SectionEndTime - SectionStartTime;
    DeadWindow.WindowName = FName("None");
    DeadWindow.WindowColor = FLinearColor::White;
    OutWindows.Add(DeadWindow);
  }

  return OutWindows;
}

bool UOHCombatUtils::IsOutsideMontageSectionWindow(
    const FOHMontagePlaybackState &State, float WindowStart, float WindowEnd) {
  return !IsInMontageSectionWindow(State, WindowStart, WindowEnd);
}

bool UOHCombatUtils::IsInMontageSegmentWindow(
    const FOHMontagePlaybackState &State, float SegmentStart,
    float SegmentEnd) {
  if (!State.bIsValid || SegmentEnd <= SegmentStart || SegmentStart < 0.f ||
      SegmentEnd > 1.f) {
    return false;
  }

  const float T = State.SegmentNormalizedTime;
  return (T >= SegmentStart && T <= SegmentEnd);
}

float UOHCombatUtils::GetRemainingSectionTime(
    const FOHMontagePlaybackState &State) {
  if (!State.bIsValid || State.SectionLength <= 0.f) {
    return 0.f;
  }

  const float LocalTime = State.NormalizedTime * State.SectionLength;
  return FMath::Clamp(State.SectionLength - LocalTime, 0.f,
                      State.SectionLength);
}

#pragma endregion

#pragma region StrikeComputation

bool UOHCombatUtils::IsStrikeContactValid(FVector Velocity,
                                          FVector ContactNormal,
                                          float EstimatedForce,
                                          float MinDotThreshold,
                                          float MinEstimatedForce) {
  const float Dot = FVector::DotProduct(Velocity.GetSafeNormal(),
                                        ContactNormal.GetSafeNormal());
  const bool bDirectional = Dot < -FMath::Abs(MinDotThreshold);
  const bool bForceful = EstimatedForce > MinEstimatedForce;
  return bDirectional && bForceful;
}

float UOHCombatUtils::ComputeContactScore(FVector Velocity,
                                          FVector ContactNormal,
                                          float EstimatedForce) {
  float Dot = FVector::DotProduct(Velocity.GetSafeNormal(),
                                  ContactNormal.GetSafeNormal());
  float VelocityScore = FMath::Clamp(FMath::Abs(Dot), 0.f, 1.f);
  float ForceScore = FMath::Clamp(EstimatedForce / 100.f, 0.f, 1.f);
  return (VelocityScore * 0.5f) + (ForceScore * 0.5f);
}

void UOHCombatUtils::SortContactsByImpactScore_Raw(
    TArray<FVector> &Velocities, TArray<FVector> &Normals,
    TArray<float> &Forces, TArray<int32> &OutSortedIndices) {
  const int32 Count = Velocities.Num();
  if (Count != Normals.Num() || Count != Forces.Num()) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("SortContactsByImpactScore_Raw: Mismatched input array lengths"));
    return;
  }

  OutSortedIndices.SetNum(Count);
  for (int32 i = 0; i < Count; ++i) {
    OutSortedIndices[i] = i;
  }

  OutSortedIndices.Sort([&](int32 A, int32 B) {
    const float ScoreA =
        ComputeContactScore(Velocities[A], Normals[A], Forces[A]);
    const float ScoreB =
        ComputeContactScore(Velocities[B], Normals[B], Forces[B]);
    return ScoreA > ScoreB;
  });
}

FVector UOHCombatUtils::ConvertWorldToActorLocal(const AActor *ReferenceActor,
                                                 const FVector &WorldLocation) {
  return ReferenceActor
             ? ReferenceActor->GetTransform().InverseTransformPosition(
                   WorldLocation)
             : FVector::ZeroVector;
}

bool UOHCombatUtils::IsVelocityAlignedWithImpactNormal(const FVector &Velocity,
                                                       const FVector &Normal,
                                                       float ThresholdDegrees) {
  const float Dot =
      FVector::DotProduct(Velocity.GetSafeNormal(), Normal.GetSafeNormal());
  const float Angle = FMath::Acos(Dot) * (180.f / PI);
  return Angle >= ThresholdDegrees;
}

void UOHCombatUtils::FilterRedundantHitsByActor(
    const TArray<AActor *> &HitActors, TArray<int32> &OutUniqueIndices) {
  TSet<AActor *> Seen;
  OutUniqueIndices.Reset();

  for (int32 i = 0; i < HitActors.Num(); ++i) {
    AActor *Actor = HitActors[i];
    if (Actor && !Seen.Contains(Actor)) {
      Seen.Add(Actor);
      OutUniqueIndices.Add(i);
    }
  }
}

void UOHCombatUtils::DrawDebugPath(const UObject *WorldContextObject,
                                   const TArray<FVector> &Points, FColor Color,
                                   float Duration, float Thickness, bool bLoop,
                                   bool bPersistentLines) {
  if (!WorldContextObject || Points.Num() < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] DrawDebugPath: Invalid input or insufficient points"));
    return;
  }

  UWorld *World = WorldContextObject->GetWorld();
  if (!World) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] DrawDebugPath: World is null"));
    return;
  }

  for (int32 i = 1; i < Points.Num(); ++i) {
    DrawDebugLine(World, Points[i - 1], Points[i], Color, bPersistentLines,
                  Duration, 0, Thickness);
  }

  if (bLoop) {
    DrawDebugLine(World, Points.Last(), Points[0], Color, bPersistentLines,
                  Duration, 0, Thickness);
  }
}

void UOHCombatUtils::DrawDebugStrikeCone(const UObject *WorldContextObject,
                                         const FVector &Origin,
                                         const FVector &Velocity, float Length,
                                         float AngleDegrees, FColor Color,
                                         float Duration, float Thickness) {
  if (!WorldContextObject || Velocity.IsNearlyZero())
    return;

  UWorld *World = WorldContextObject->GetWorld();
  if (!World)
    return;

  const FVector Forward = Velocity.GetSafeNormal();

  // Main line
  const FVector End = Origin + Forward * Length;
  DrawDebugLine(World, Origin, End, Color, false, Duration, 0, Thickness);

  // Cone rays
  const FVector Right =
      FVector::CrossProduct(Forward, FVector::UpVector).GetSafeNormal();
  const FVector Up = FVector::CrossProduct(Right, Forward).GetSafeNormal();

  const FVector Edge1 =
      Forward.RotateAngleAxis(AngleDegrees, Right).GetSafeNormal();
  const FVector Edge2 =
      Forward.RotateAngleAxis(-AngleDegrees, Right).GetSafeNormal();
  const FVector Edge3 =
      Forward.RotateAngleAxis(AngleDegrees, Up).GetSafeNormal();
  const FVector Edge4 =
      Forward.RotateAngleAxis(-AngleDegrees, Up).GetSafeNormal();

  DrawDebugLine(World, Origin, Origin + Edge1 * Length, Color, false, Duration,
                0, Thickness);
  DrawDebugLine(World, Origin, Origin + Edge2 * Length, Color, false, Duration,
                0, Thickness);
  DrawDebugLine(World, Origin, Origin + Edge3 * Length, Color, false, Duration,
                0, Thickness);
  DrawDebugLine(World, Origin, Origin + Edge4 * Length, Color, false, Duration,
                0, Thickness);
}

FVector
UOHCombatUtils::GetBoneLinearVelocity(const USkeletalMeshComponent *Mesh,
                                      FName BoneName, float DeltaTime) {
  if (!Mesh || !Mesh->DoesSocketExist(BoneName) ||
      DeltaTime <= KINDA_SMALL_NUMBER) {
    return FVector::ZeroVector;
  }

  const FVector CurrentPos = Mesh->GetSocketLocation(BoneName);

  // We'll store the previous position in a transient tag or a static cache
  static TMap<FName, FVector> LastPositions;

  FVector *PreviousPtr = LastPositions.Find(BoneName);
  FVector Velocity = FVector::ZeroVector;

  if (PreviousPtr) {
    Velocity = (CurrentPos - *PreviousPtr) / DeltaTime;
  }

  LastPositions.Add(BoneName, CurrentPos); // Update stored position
  return Velocity;
}

FVector UOHCombatUtils::ComputeBoneLinearVelocityFromSamples(
    const FVector &CurrentPos, const FVector &PreviousPos, float DeltaTime) {
  if (DeltaTime <= KINDA_SMALL_NUMBER) {
    return FVector::ZeroVector;
  }
  return (CurrentPos - PreviousPos) / DeltaTime;
}

FVector UOHCombatUtils::ComputeBoneVelocityMeshRelative(
    const USkeletalMeshComponent *Mesh, FName BoneName, float DeltaTime) {
  if (!Mesh || !Mesh->DoesSocketExist(BoneName) ||
      DeltaTime <= KINDA_SMALL_NUMBER) {
    return FVector::ZeroVector;
  }

  const FVector BoneWorldPos = Mesh->GetSocketLocation(BoneName);
  const FVector MeshWorldPos = Mesh->GetComponentLocation();

  const FVector BoneRelPos = BoneWorldPos - MeshWorldPos;

  static TMap<FName, FVector> LastRelativePositions;
  FVector *PrevRelPos = LastRelativePositions.Find(BoneName);

  FVector Velocity = FVector::ZeroVector;
  if (PrevRelPos) {
    Velocity = (BoneRelPos - *PrevRelPos) / DeltaTime;
  }

  LastRelativePositions.Add(BoneName, BoneRelPos);
  return Velocity;
}

FVector UOHCombatUtils::PredictBonePositionFromAnimation(
    const USkeletalMeshComponent *Mesh, FName BoneName, float TimeOffset) {
  if (!Mesh || !Mesh->GetAnimInstance() || !Mesh->DoesSocketExist(BoneName)) {
    return FVector::ZeroVector;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  const FAnimMontageInstance *MontageInstance =
      AnimInstance->GetActiveMontageInstance();

  if (!MontageInstance || !MontageInstance->Montage) {
    return Mesh->GetSocketLocation(BoneName); // Fallback
  }

  // Calculate time range for prediction
  const float CurrentTime = MontageInstance->GetPosition();
  const float FutureTime = CurrentTime + TimeOffset;

  // Sample root motion transform directly from the montage
  const FTransform RootMotionDelta =
      MontageInstance->Montage->ExtractRootMotionFromTrackRange(CurrentTime,
                                                                FutureTime);

  // Get the bone's current component-local offset
  const FTransform CurrentBoneTransform =
      Mesh->GetSocketTransform(BoneName, RTS_Component);
  const FVector LocalOffset = CurrentBoneTransform.GetLocation();

  // Predict future bone world position by applying root motion delta
  const FVector PredictedWorld =
      Mesh->GetComponentTransform().TransformPosition(
          RootMotionDelta.TransformPosition(LocalOffset));

  return PredictedWorld;
}

void UOHCombatUtils::DetectContactAlongStrikeArc(
    const UObject *WorldContextObject, const TArray<FVector> &ArcPoints,
    TArray<FHitResult> &OutHits, ECollisionChannel TraceChannel,
    float SegmentThickness) {
  OutHits.Reset();

  if (!WorldContextObject || ArcPoints.Num() < 2) {
    return;
  }

  UWorld *World = WorldContextObject->GetWorld();
  if (!World) {
    return;
  }

  for (int32 i = 1; i < ArcPoints.Num(); ++i) {
    const FVector Start = ArcPoints[i - 1];
    const FVector End = ArcPoints[i];

    FHitResult Hit;

    if (SegmentThickness <= 0.f) {
      World->LineTraceSingleByChannel(Hit, Start, End, TraceChannel);
    } else {
      FCollisionShape Sphere = FCollisionShape::MakeSphere(SegmentThickness);
      World->SweepSingleByChannel(Hit, Start, End, FQuat::Identity,
                                  TraceChannel, Sphere);
    }

    if (Hit.bBlockingHit) {
      OutHits.Add(Hit);
    }
  }
}

void UOHCombatUtils::SweepCapsuleAlongStrikeArc(
    const UObject *WorldContextObject, const TArray<FVector> &ArcPoints,
    float Radius, float HalfHeight, TArray<FHitResult> &OutHits,
    ECollisionChannel TraceChannel) {
  OutHits.Reset();

  if (!WorldContextObject || ArcPoints.Num() < 2) {
    return;
  }

  UWorld *World = WorldContextObject->GetWorld();
  if (!World) {
    return;
  }

  const FCollisionShape CapsuleShape =
      FCollisionShape::MakeCapsule(Radius, HalfHeight);

  for (int32 i = 1; i < ArcPoints.Num(); ++i) {
    const FVector Start = ArcPoints[i - 1];
    const FVector End = ArcPoints[i];

    FHitResult Hit;
    World->SweepSingleByChannel(Hit, Start, End, FQuat::Identity, TraceChannel,
                                CapsuleShape);

    if (Hit.bBlockingHit) {
      OutHits.Add(Hit);
    }
  }
}

bool UOHCombatUtils::DetectGJKContactFromStrikeArc(
    const TArray<FVector> &ArcPoints, const TArray<FVector> &TargetConvexPoints,
    FVector &OutContactPoint, FVector &OutPenetrationVector) {
  OutContactPoint = FVector::ZeroVector;
  OutPenetrationVector = FVector::ZeroVector;

  if (ArcPoints.Num() < 3 || TargetConvexPoints.Num() < 3) {
    return false;
  }

  // Step 1: Generate convex hull from arc points
  TArray<FVector> ArcHull;
  TArray<int32> ArcIndices;
  UOHCollisionUtils::ComputeConvexHullWithIndices(ArcPoints, ArcHull,
                                                  ArcIndices);

  if (ArcHull.Num() < 3) {
    return false;
  }

  constexpr int32 MaxIterations = 30;
  constexpr float Tolerance = 0.001f;

  // Step 2: Run GJK with simplex and support point output
  TArray<FVector> Simplex;
  TArray<FVector> SupportA;
  TArray<FVector> SupportB;

  bool bGJK = UOHCollisionUtils::GJK_Intersect(ArcHull, TargetConvexPoints,
                                               Simplex, SupportA, SupportB,
                                               MaxIterations, Tolerance);

  if (!bGJK) {
    return false;
  }

  // Step 3: Run EPA using the GJK simplex
  FVector Normal;
  float Depth;

  bool bEPA = UOHCollisionUtils::EPA_PenetrationDepth(
      ArcHull, TargetConvexPoints, Simplex, Normal, Depth, MaxIterations,
      Tolerance);

  if (!bEPA) {
    return false;
  }

  // Step 4: Compute penetration vector and contact point
  OutPenetrationVector = Normal * Depth;
  OutContactPoint =
      UOHCollisionUtils::ComputeAverageContactPoint(SupportA, SupportB);

  return true;
}
static bool SampleStrikeArc_Internal(UAnimSequence *Sequence,
                                     USkeletalMeshComponent *Mesh,
                                     FName BoneName,
                                     const TArray<float> &SampleTimes,
                                     TArray<FVector> &OutPoints) {
  OutPoints.Reset();

  if (!Sequence || !Mesh || BoneName.IsNone() || SampleTimes.Num() < 2)
    return false;

  for (float Time : SampleTimes) {
    FTransform BoneTransform;
    if (UOHCombatUtils::GetBoneTransformAtTime(Sequence, Mesh, BoneName, Time,
                                               BoneTransform)) {
      const FVector Pos = BoneTransform.GetLocation();
      if (!Pos.ContainsNaN()) {
        OutPoints.Add(Pos);
      }
    }
  }

  return OutPoints.Num() > 1;
}

void UOHCombatUtils::GenerateStrikeArcFromLiveMesh(
    const USkeletalMeshComponent *Mesh, FName BoneName, int32 NumSamples,
    float SampleInterval, TArray<FVector> &OutArc) {
  OutArc.Reset();

  if (!Mesh || !Mesh->DoesSocketExist(BoneName) || NumSamples < 2 ||
      SampleInterval <= 0.f) {
    return;
  }

  UWorld *World = Mesh->GetWorld();
  if (!World) {
    return;
  }

  for (int32 i = 0; i < NumSamples; ++i) {
    // Currently just using live bone position, optionally you can query motion
    // history here
    const FVector Pos = Mesh->GetSocketLocation(BoneName);
    OutArc.Add(Pos);
  }
}

bool UOHCombatUtils::GenerateStrikeArcFromAnimBase(
    UAnimSequenceBase *AnimBase, USkeletalMeshComponent *Mesh, FName BoneName,
    float StartTime, float EndTime, TArray<FVector> &OutPoints,
    float SampleRate) {
  if (!AnimBase || !Mesh) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] Invalid AnimBase or Mesh"));
    return false;
  }

  const float QueryTime = (StartTime > 0.f) ? StartTime : 0.01f;
  UAnimSequence *Sequence =
      ExtractSequenceFromAnimBaseAtTime(AnimBase, QueryTime);

  if (!Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] No valid sequence extracted from AnimBase at time %.2f"),
           QueryTime);
    return false;
  }

  const float ClampedStart =
      FMath::Clamp(StartTime, 0.f, Sequence->GetPlayLength());
  const float ClampedEnd =
      FMath::Clamp(EndTime, ClampedStart, Sequence->GetPlayLength());

  return GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName,
                                           ClampedStart, ClampedEnd, OutPoints,
                                           SampleRate);
}
bool UOHCombatUtils::GenerateStrikeArcFromAnimSequence(
    UAnimSequence *Sequence, USkeletalMeshComponent *Mesh, FName BoneName,
    float StartTime, float EndTime, TArray<FVector> &OutPoints,
    float SampleRate) {
  if (!Sequence || !Mesh || BoneName.IsNone() || StartTime >= EndTime ||
      SampleRate <= 0.f)
    return false;

  const float Interval = 1.f / FMath::Clamp(SampleRate, 1.f, 240.f);
  const int32 SampleCount = FMath::CeilToInt((EndTime - StartTime) / Interval);

  TArray<float> SampleTimes;
  SampleTimes.Reserve(SampleCount + 1);

  for (int32 i = 0; i <= SampleCount; ++i) {
    SampleTimes.Add(FMath::Clamp(StartTime + i * Interval, StartTime, EndTime));
  }

  return SampleStrikeArc_Internal(Sequence, Mesh, BoneName, SampleTimes,
                                  OutPoints);
}

bool UOHCombatUtils::GenerateStrikeArcFromAnimSequence_FixedResolution(
    UAnimSequence *Sequence, USkeletalMeshComponent *Mesh, FName BoneName,
    int32 NumSamples, float StartTime, float EndTime,
    TArray<FVector> &OutPoints) {
  if (!Sequence || !Mesh || BoneName.IsNone() || StartTime >= EndTime ||
      NumSamples < 2)
    return false;

  TArray<float> SampleTimes;
  SampleTimes.Reserve(NumSamples);

  const float Range = EndTime - StartTime;
  for (int32 i = 0; i < NumSamples; ++i) {
    const float Alpha = static_cast<float>(i) / (NumSamples - 1);
    SampleTimes.Add(StartTime + Alpha * Range);
  }

  return SampleStrikeArc_Internal(Sequence, Mesh, BoneName, SampleTimes,
                                  OutPoints);
}

bool UOHCombatUtils::GenerateStrikeArcFromComposite(
    UAnimComposite *Composite, USkeletalMeshComponent *Mesh, FName BoneName,
    float StartTime, float EndTime, TArray<FVector> &OutPoints,
    float SampleRate) {
  if (!Composite || !Mesh) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] Invalid Composite or Mesh"));
    return false;
  }

  const float QueryTime = (StartTime > 0.f) ? StartTime : 0.01f;
  UAnimSequence *Sequence =
      ExtractSequenceFromAnimBaseAtTime(Composite, QueryTime);

  if (!Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] No valid sequence extracted from Composite at time %.2f"),
           QueryTime);
    return false;
  }

  const float ClampedStart =
      FMath::Clamp(StartTime, 0.f, Sequence->GetPlayLength());
  const float ClampedEnd =
      FMath::Clamp(EndTime, ClampedStart, Sequence->GetPlayLength());

  return GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName,
                                           ClampedStart, ClampedEnd, OutPoints,
                                           SampleRate);
}

bool UOHCombatUtils::GenerateStrikeArcFromMontage(
    UAnimMontage *Montage, USkeletalMeshComponent *Mesh, FName BoneName,
    float StartTime, float EndTime, TArray<FVector> &OutPoints,
    float SampleRate) {
  if (!Montage || !Mesh) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] Invalid Montage or Mesh"));
    return false;
  }

  // Use slight offset to avoid ambiguous segment evaluation at 0
  const float QueryTime = (StartTime > 0.f) ? StartTime : 0.01f;
  UAnimSequence *Sequence =
      ExtractSequenceFromAnimBaseAtTime(Montage, QueryTime);

  if (!Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] No valid sequence extracted from Montage at time %.2f"),
           QueryTime);
    return false;
  }

  const float ClampedStart =
      FMath::Clamp(StartTime, 0.f, Sequence->GetPlayLength());
  const float ClampedEnd =
      FMath::Clamp(EndTime, ClampedStart, Sequence->GetPlayLength());

  return GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName,
                                           ClampedStart, ClampedEnd, OutPoints,
                                           SampleRate);
}

void UOHCombatUtils::VisualizeBoneMotionArcFromAnim(
    const UObject *WorldContextObject, UAnimSequenceBase *AnimBase,
    USkeletalMeshComponent *Mesh, FName BoneName, int32 NumSamples,
    FName MontageSectionName, FColor Color, float Duration, float Thickness,
    bool bLoop, bool bDrawVelocity, bool bDrawImpactNormals) {
  if (!WorldContextObject || !AnimBase || !Mesh || BoneName.IsNone() ||
      NumSamples < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("VisualizeBoneMotionArcFromAnim: Invalid input."));
    return;
  }

  float StartTime = 0.f;
  float EndTime = 0.f;

  // Try to get section timing if valid
  if (UAnimMontage *Montage = Cast<UAnimMontage>(AnimBase)) {
    if (!MontageSectionName.IsNone()) {
      const int32 SectionIndex = Montage->GetSectionIndex(MontageSectionName);
      if (SectionIndex != INDEX_NONE) {
        const FCompositeSection &Section =
            Montage->GetAnimCompositeSection(SectionIndex);
        StartTime = Section.GetTime();
        EndTime = StartTime + Montage->GetSectionLength(SectionIndex);
      } else {
        UE_LOG(LogTemp, Warning,
               TEXT("VisualizeBoneMotionArcFromAnim: Montage section '%s' not "
                    "found."),
               *MontageSectionName.ToString());
      }
    } else {
      EndTime = Montage->GetPlayLength();
    }
  } else {
    // Not a montage, fallback to full length
    EndTime = AnimBase->GetPlayLength();
  }

  // Sample arc
  TArray<FVector> ArcPoints;
  if (!UOHCombatUtils::GenerateStrikeArcFromAnimBase(
          AnimBase, Mesh, BoneName, StartTime, EndTime, ArcPoints,
          NumSamples /
              (EndTime - StartTime))) // Convert NumSamples to SampleRate
  {
    UE_LOG(LogTemp, Warning,
           TEXT("VisualizeBoneMotionArcFromAnim: Failed to generate arc."));
    return;
  }

  if (ArcPoints.Num() < 2) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("VisualizeBoneMotionArcFromAnim: Arc has insufficient points."));
    return;
  }

  // Draw main arc path
  DrawDebugPath(WorldContextObject, ArcPoints, Color, Duration, Thickness,
                bLoop);

  // Optional: Draw velocity vectors
  if (bDrawVelocity || bDrawImpactNormals) {
    for (int32 i = 1; i < ArcPoints.Num(); ++i) {
      const FVector Start = ArcPoints[i - 1];
      const FVector End = ArcPoints[i];
      const FVector Velocity = (End - Start).GetSafeNormal();

      if (bDrawVelocity) {
        DrawDebugLine(WorldContextObject->GetWorld(), Start,
                      Start + Velocity * 20.f, FColor::Yellow, false, Duration,
                      0, 1.5f);
      }

      if (bDrawImpactNormals && i < ArcPoints.Num() - 1) {
        // Estimate surface normal using forward difference
        const FVector Next = ArcPoints[i + 1];
        const FVector Dir1 = (End - Start).GetSafeNormal();
        const FVector Dir2 = (Next - End).GetSafeNormal();
        const FVector Tangent = (Dir1 + Dir2).GetSafeNormal();
        const FVector Normal =
            FVector::UpVector ^ Tangent; // crude right-angle estimate

        DrawDebugLine(WorldContextObject->GetWorld(), End, End + Normal * 20.f,
                      FColor::Cyan, false, Duration, 0, 1.0f);
      }
    }
  }
}

void UOHCombatUtils::VisualizePredictedArc(
    const UObject *WorldContextObject, USkeletalMeshComponent *Mesh,
    FName BoneName, float PredictionWindow, int32 NumSamples, FColor Color,
    float Duration, float Thickness, bool bDrawVelocity,
    bool bDrawImpactNormals) {
  if (!WorldContextObject || !Mesh || BoneName.IsNone() || NumSamples < 2 ||
      PredictionWindow <= 0.f) {
    UE_LOG(LogTemp, Warning,
           TEXT("VisualizePredictedArcFromNow: Invalid input."));
    return;
  }

  UAnimInstance *AnimInst = Mesh->GetAnimInstance();
  if (!AnimInst)
    return;

  // Get currently playing AnimSequence
  float LocalTime = 0.f;
  UAnimSequence *Sequence = nullptr;

  if (!UOHCombatUtils::ExtractActiveSequenceAndTimeFromMesh(Mesh, Sequence,
                                                            LocalTime) ||
      !Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("VisualizePredictedArcFromNow: Failed to extract active "
                "sequence."));
    return;
  }

  // Compute time bounds
  const float StartTime = LocalTime;
  const float EndTime =
      FMath::Min(StartTime + PredictionWindow, Sequence->GetPlayLength());

  TArray<FVector> ArcPoints;
  if (!GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName, StartTime,
                                         EndTime, ArcPoints,
                                         NumSamples / PredictionWindow)) {
    UE_LOG(LogTemp, Warning,
           TEXT("VisualizePredictedArcFromNow: Failed to generate arc."));
    return;
  }

  // Draw arc
  DrawDebugPath(WorldContextObject, ArcPoints, Color, Duration, Thickness);

  // Draw optional debug vectors
  for (int32 i = 1; i < ArcPoints.Num(); ++i) {
    const FVector Start = ArcPoints[i - 1];
    const FVector End = ArcPoints[i];
    const FVector Velocity = (End - Start).GetSafeNormal();

    if (bDrawVelocity) {
      DrawDebugLine(WorldContextObject->GetWorld(), Start,
                    Start + Velocity * 20.f, FColor::Yellow, false, Duration, 0,
                    1.5f);
    }

    if (bDrawImpactNormals && i < ArcPoints.Num() - 1) {
      const FVector Next = ArcPoints[i + 1];
      const FVector Dir1 = (End - Start).GetSafeNormal();
      const FVector Dir2 = (Next - End).GetSafeNormal();
      const FVector Tangent = (Dir1 + Dir2).GetSafeNormal();
      const FVector Normal = FVector::UpVector ^ Tangent;

      DrawDebugLine(WorldContextObject->GetWorld(), End, End + Normal * 20.f,
                    FColor::Cyan, false, Duration, 0, 1.0f);
    }
  }
}

FCombatStrikePredictionResult UOHCombatUtils::PredictStrikeCollision(
    const UObject *WorldContextObject, ACharacter *Attacker,
    UOHPhysicsManager *PhysicsManager, FName BoneName, float PredictionTime,
    int Steps, bool bEnableDebug) {
  FCombatStrikePredictionResult Result;

  if (!WorldContextObject || !Attacker || !PhysicsManager)
    return Result;

  UWorld *World = WorldContextObject->GetWorld();
  if (!World)
    return Result;

  const FOHBoneData *BoneData = PhysicsManager->GetBoneData(BoneName);
  if (!BoneData || !BoneData->IsValid())
    return Result;

  FVector StartPos = BoneData->GetCurrentPosition();
  FVector Velocity = BoneData->GetBodyLinearVelocity();
  FVector Accel = BoneData->GetLinearAcceleration();

  // === Find nearest enemy character ===
  ACharacter *ClosestTarget = nullptr;
  float ClosestDistSq = TNumericLimits<float>::Max();

  for (TActorIterator<ACharacter> It(World); It; ++It) {
    ACharacter *Candidate = *It;
    if (!Candidate || Candidate == Attacker)
      continue;

    float DistSq = FVector::DistSquared(Attacker->GetActorLocation(),
                                        Candidate->GetActorLocation());
    if (DistSq < ClosestDistSq) {
      ClosestTarget = Candidate;
      ClosestDistSq = DistSq;
    }
  }

  if (!ClosestTarget)
    return Result;
  USkeletalMeshComponent *TargetMesh = ClosestTarget->GetMesh();
  if (!TargetMesh)
    return Result;

  TArray<FName> TargetBones;
  if (UOHPhysicsManager *TargetPM =
          ClosestTarget->FindComponentByClass<UOHPhysicsManager>()) {
    TargetBones = TargetPM->GetTrackedBoneNames();
  }
  if (TargetBones.Num() == 0) {
    TargetMesh->GetBoneNames(TargetBones);
  }

  // === Predict bone trajectory ===
  const float StepSize = PredictionTime / Steps;

  float BestDistance = TNumericLimits<float>::Max();
  FVector BestPredictedPos;
  FName BestTargetBone = NAME_None;
  FVector BestTargetBoneLoc;
  float BestTime = 0.f;

  for (int32 Step = 1; Step <= Steps; ++Step) {
    float T = Step * StepSize;
    FVector Predicted = StartPos + Velocity * T + 0.5f * Accel * T * T;

    for (const FName &Bone : TargetBones) {
      FVector BoneLoc = TargetMesh->GetBoneLocation(Bone);
      float Dist = FVector::Dist(Predicted, BoneLoc);
      if (Dist < BestDistance) {
        BestDistance = Dist;
        BestPredictedPos = Predicted;
        BestTargetBone = Bone;
        BestTargetBoneLoc = BoneLoc;
        BestTime = T;
      }
    }

    if (bEnableDebug) {
      DrawDebugSphere(World, Predicted, 3.f, 8, FColor::Green, false, 1.0f);
    }
  }

  // === Result ===
  if (BestTargetBone.IsNone())
    return Result;

  Result.bHitPredictionValid = true;
  Result.PredictedStrikePoint = BestPredictedPos;
  Result.ClosestTargetBone = BestTargetBone;
  Result.ClosestTargetBoneLocation = BestTargetBoneLoc;
  Result.ClosestDistance = BestDistance;
  Result.TimeToClosestPoint = BestTime;
  Result.TargetCharacter = ClosestTarget;

  // === Final Debug Visualization ===
  if (bEnableDebug) {
    DrawDebugLine(World, BestPredictedPos, BestTargetBoneLoc, FColor::Yellow,
                  false, 2.f, 0, 1.5f);
    DrawDebugSphere(World, BestTargetBoneLoc, 5.f, 8, FColor::Blue, false, 2.f);
    DrawDebugString(
        World, BestPredictedPos + FVector(0, 0, 12),
        FString::Printf(TEXT("Target Bone: %s\nDist: %.1f\nTTC: %.2fs"),
                        *BestTargetBone.ToString(), BestDistance, BestTime),
        nullptr, FColor::White, 2.0f, false);
  }

  return Result;
}

FCombatStrikePredictionResult
UOHCombatUtils::AnalyzeStrikeTrajectoryAndTargeting(
    const UObject *WorldContextObject, ACharacter *Attacker,
    UOHPhysicsManager *PhysicsManager, FName StrikeBone, float PredictionTime,
    int Steps, bool bEnableDebug, FName DebugTag) {
  FCombatStrikePredictionResult Result;
  if (!WorldContextObject || !Attacker || !PhysicsManager ||
      StrikeBone.IsNone())
    return Result;

  UWorld *World = WorldContextObject->GetWorld();
  if (!World)
    return Result;

  // --- Validate attacker mesh ---
  const USkeletalMeshComponent *Mesh = Attacker->GetMesh();
  if (!Mesh)
    return Result;

  // --- Resolve strike bone chain from physics manager ---
  FName RootBone = "pelvis";
  TArray<FName> BoneChain = UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(
      Mesh, RootBone, StrikeBone);
  if (BoneChain.Num() == 0)
    BoneChain.Add(StrikeBone); // fallback to single bone

  // --- Aggregate OHBoneData motion from physics manager ---
  TArray<const FOHBoneData *> ValidBones;
  FVector TotalVelocity = FVector::ZeroVector;
  FVector TotalAcceleration = FVector::ZeroVector;
  FVector TotalPosition = FVector::ZeroVector;
  FVector TotalForward = FVector::ZeroVector;

  for (const FName &Bone : BoneChain) {
    const FOHBoneData *BoneData = PhysicsManager->GetBoneData(Bone);
    if (!BoneData || !BoneData->IsValid())
      continue;

    ValidBones.Add(BoneData);
    TotalVelocity += BoneData->GetBodyLinearVelocity();
    TotalAcceleration += BoneData->GetLinearAcceleration();
    TotalPosition += BoneData->GetCurrentPosition();
    TotalForward += BoneData->GetCurrentRotation().Vector();
  }

  if (ValidBones.Num() == 0) {
    UE_LOG(LogTemp, Warning,
           TEXT("Strike prediction failed: No valid OHBoneData for attacker."));
    return Result;
  }

  const FVector AvgVelocity = TotalVelocity / ValidBones.Num();
  const FVector AvgAccel = TotalAcceleration / ValidBones.Num();
  const FVector AvgStart = TotalPosition / ValidBones.Num();
  const FVector AvgForward = TotalForward.GetSafeNormal();

  // --- Find nearest character target (not self) ---
  ACharacter *ClosestTarget = nullptr;
  float ClosestDistSq = TNumericLimits<float>::Max();

  for (TActorIterator<ACharacter> It(World); It; ++It) {
    ACharacter *Candidate = *It;
    if (!Candidate || Candidate == Attacker)
      continue;

    const float DistSq = FVector::DistSquared(Attacker->GetActorLocation(),
                                              Candidate->GetActorLocation());
    if (DistSq < ClosestDistSq) {
      ClosestTarget = Candidate;
      ClosestDistSq = DistSq;
    }
  }

  if (!ClosestTarget || !ClosestTarget->GetMesh())
    return Result;

  // --- Get target bone data ---
  TArray<FName> TargetBones;
  if (UOHPhysicsManager *TargetPM =
          ClosestTarget->FindComponentByClass<UOHPhysicsManager>())
    TargetBones = TargetPM->GetTrackedBoneNames();

  if (TargetBones.Num() == 0)
    ClosestTarget->GetMesh()->GetBoneNames(TargetBones); // fallback

  // --- Sweep prediction steps ---
  const float StepSize = PredictionTime / Steps;
  float BestDistance = TNumericLimits<float>::Max();
  FVector BestPredictedPos;
  FVector BestTargetBoneLoc;
  FName BestTargetBone = NAME_None;
  float BestTime = 0.f;

  for (int32 Step = 1; Step <= Steps; ++Step) {
    const float T = Step * StepSize;
    const FVector Predicted =
        AvgStart + AvgVelocity * T + 0.5f * AvgAccel * T * T;

    for (const FName &Bone : TargetBones) {
      const FOHBoneData *TargetBoneData = nullptr;
      if (UOHPhysicsManager *TargetPM =
              ClosestTarget->FindComponentByClass<UOHPhysicsManager>())
        TargetBoneData = TargetPM->GetBoneData(Bone);

      const FVector BoneLoc =
          (TargetBoneData && TargetBoneData->IsValid())
              ? TargetBoneData->GetCurrentPosition()
              : ClosestTarget->GetMesh()->GetSocketLocation(Bone);

      const float Dist = FVector::Dist(Predicted, BoneLoc);
      if (Dist < BestDistance) {
        BestDistance = Dist;
        BestPredictedPos = Predicted;
        BestTargetBone = Bone;
        BestTargetBoneLoc = BoneLoc;
        BestTime = T;
      }
    }

    if (bEnableDebug) {
      DrawDebugSphere(World, Predicted, 4.f, 8, FColor::Green, false, 2.f);
    }
  }

  if (BestTargetBone.IsNone())
    return Result;

  // --- Directional scoring ---
  const FVector ToTarget = (BestTargetBoneLoc - AvgStart).GetSafeNormal();
  const float Dot = FVector::DotProduct(AvgForward, ToTarget);
  const float AngleDeg =
      FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
  const float DirScore = FMath::GetMappedRangeValueClamped(
      FVector2D(-1, 1), FVector2D(-1, 1), Dot);

  // --- Region classification ---
  FName Region = NAME_None;
  const FString BoneStr = BestTargetBone.ToString().ToLower();

  if (BoneStr.Contains("head") || BoneStr.Contains("neck"))
    Region = "Head";
  else if (BoneStr.Contains("spine") || BoneStr.Contains("pelvis"))
    Region = "Torso";
  else if (BoneStr.Contains("hand") || BoneStr.Contains("arm"))
    Region = "Arm";
  else if (BoneStr.Contains("leg") || BoneStr.Contains("foot"))
    Region = "Leg";

  // --- Result assignment ---
  Result.bHitPredictionValid = true;
  Result.PredictedStrikePoint = BestPredictedPos;
  Result.ClosestTargetBone = BestTargetBone;
  Result.ClosestTargetBoneLocation = BestTargetBoneLoc;
  Result.ClosestDistance = BestDistance;
  Result.TimeToClosestPoint = BestTime;
  Result.TargetCharacter = ClosestTarget;
  Result.DirectionalityScore = DirScore;
  Result.AngularDeviationDegrees = AngleDeg;
  Result.SkeletalRegion = Region;

  // --- Debug drawing ---
  if (bEnableDebug) {
    DrawDebugLine(World, BestPredictedPos, BestTargetBoneLoc, FColor::Yellow,
                  false, 2.f, 0, 1.5f);
    DrawDebugSphere(World, BestTargetBoneLoc, 6.f, 12, FColor::Blue, false,
                    2.f);
    DrawDebugDirectionalArrow(World, AvgStart, BestTargetBoneLoc, 150.f,
                              FColor::Red, false, 2.f, 0, 3.0f);
    DrawDebugString(
        World, BestPredictedPos + FVector(0, 0, 32),
        FString::Printf(TEXT("Bone: %s\nAngle: %.1f°\nScore: %.2f\nRegion: %s"),
                        *BestTargetBone.ToString(), AngleDeg, DirScore,
                        *Region.ToString()),
        nullptr, FColor::White, 2.5f, false);
  }

  return Result;
}

FCombatStrikePredictionResult UOHCombatUtils::AnalyzeAdvancedStrikeTrajectory3D(
    const UObject *WorldContextObject, ACharacter *Attacker,
    UOHPhysicsManager *PhysicsManager, FName StrikeBone, float PredictionTime,
    int Steps, bool bEnableDebug, FName DebugTag, FName OptionalChainStart,
    int32 MaxChainDepth, bool bIsFullBodyAnimation,
    float HistoricalBlendAlpha) {
  FCombatStrikePredictionResult Result;

  if (!WorldContextObject || !Attacker || !PhysicsManager ||
      StrikeBone.IsNone())
    return Result;

  UWorld *World = WorldContextObject->GetWorld();
  const USkeletalMeshComponent *Mesh = Attacker->GetMesh();
  if (!World || !Mesh)
    return Result;

  // --- 1. Bone Chain ---
  TArray<FName> BoneChain =
      OptionalChainStart.IsNone()
          ? UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, NAME_None,
                                                               StrikeBone)
          : UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(
                Mesh, OptionalChainStart, StrikeBone);

  if (MaxChainDepth > 0 && BoneChain.Num() > MaxChainDepth) {
    TArray<FName> TruncatedChain;
    for (int32 i = BoneChain.Num() - MaxChainDepth; i < BoneChain.Num(); ++i)
      TruncatedChain.Add(BoneChain[i]);
    BoneChain = TruncatedChain;
  }
  if (BoneChain.Num() == 0)
    BoneChain.Add(StrikeBone);

  // --- 2. Motion Aggregation ---
  TArray<const FOHBoneData *> ValidBones;
  FVector SumPos, SumVel, SumHistVel, SumAccel, SumForward, SumAngVel,
      SumAngAccel = FVector::ZeroVector;

  for (const FName &Bone : BoneChain) {
    if (!bIsFullBodyAnimation &&
        (Bone == "pelvis" || Bone.ToString().Contains(TEXT("root"))))
      continue;

    const FOHBoneData *BoneData = PhysicsManager->GetBoneData(Bone);
    if (!BoneData || !BoneData->IsValid())
      continue;

    ValidBones.Add(BoneData);
    SumPos += BoneData->GetCurrentPosition();
    SumVel += BoneData->GetSmoothedLinearVelocity();
    SumHistVel += BoneData->GetBlendedLinearVelocity(HistoricalBlendAlpha, 6);
    SumAccel += BoneData->GetLinearAcceleration();
    SumAngVel += BoneData->GetBodyAngularVelocity();
    SumAngAccel += BoneData->GetAngularAcceleration();
    SumForward += BoneData->GetCurrentRotation().Vector();

    if (bEnableDebug) {
      DrawDebugSphere(World, BoneData->GetCurrentPosition(), 3.f, 8,
                      FColor::Magenta, false, 2.f);
      DrawDebugString(World, BoneData->GetCurrentPosition() + FVector(0, 0, 10),
                      Bone.ToString(), nullptr, FColor::White, 2.f);
    }
  }
  if (ValidBones.Num() == 0)
    return Result;

  const FVector Start = SumPos / ValidBones.Num();
  const FVector LinearVel = (SumVel + SumHistVel) / (ValidBones.Num() * 2);
  const FVector LinearAccel = SumAccel / ValidBones.Num();
  const FVector AngularVel = SumAngVel / ValidBones.Num();
  const FVector AngularAccel = SumAngAccel / ValidBones.Num();
  const FVector Forward = SumForward.GetSafeNormal();

  // --- 3. Pose Deformation ---
  float FullExtension = FVector::Dist(Mesh->GetBoneLocation(BoneChain[0]),
                                      Mesh->GetBoneLocation(StrikeBone));
  float MaxChainLength = 0.f;
  for (int32 i = 1; i < BoneChain.Num(); ++i) {
    MaxChainLength += FVector::Dist(Mesh->GetBoneLocation(BoneChain[i - 1]),
                                    Mesh->GetBoneLocation(BoneChain[i]));
  }
  float ExtensionAlpha = FMath::Clamp(FullExtension / MaxChainLength, 0.f, 1.f);
  float ArcRadius = MaxChainLength * FMath::Lerp(0.5f, 1.2f, ExtensionAlpha);

  // --- 4. Find Target ---
  ACharacter *ClosestTarget = nullptr;
  float MinDistSq = FLT_MAX;
  for (TActorIterator<ACharacter> It(World); It; ++It) {
    ACharacter *Candidate = *It;
    if (!Candidate || Candidate == Attacker)
      continue;

    const float DistSq = FVector::DistSquared(Attacker->GetActorLocation(),
                                              Candidate->GetActorLocation());
    if (DistSq < MinDistSq) {
      ClosestTarget = Candidate;
      MinDistSq = DistSq;
    }
  }
  if (!ClosestTarget)
    return Result;

  USkeletalMeshComponent *TargetMesh = ClosestTarget->GetMesh();
  if (!TargetMesh)
    return Result;

  TArray<FName> TargetBones;
  if (UOHPhysicsManager *TargetPM =
          ClosestTarget->FindComponentByClass<UOHPhysicsManager>())
    TargetBones = TargetPM->GetTrackedBoneNames();
  if (TargetBones.Num() == 0)
    TargetMesh->GetBoneNames(TargetBones);

  // --- 5. Arc Simulation ---
  const float StepSize = PredictionTime / FMath::Max(1, Steps);
  float BestDist = FLT_MAX;
  FVector BestPos, BestTargetLoc;
  FName BestTargetBone = NAME_None;
  float BestTime = 0.f;

  const FVector TangentDir =
      FVector::CrossProduct(AngularVel, Forward).GetSafeNormal();

  for (int32 Step = 1; Step <= Steps; ++Step) {
    const float T = Step * StepSize;
    const FVector LinearPart =
        Start + LinearVel * T + 0.5f * LinearAccel * T * T;
    const float AngularSpeed = AngularVel.Size() + AngularAccel.Size() * T;
    const FVector ArcOffset =
        TangentDir * (ArcRadius * FMath::Sin(AngularSpeed * T));
    const FVector Predicted = LinearPart + ArcOffset;

    for (const FName &Bone : TargetBones) {
      const FVector BoneLoc = TargetMesh->GetBoneLocation(Bone);
      const float Dist = FVector::Dist(Predicted, BoneLoc);
      if (Dist < BestDist) {
        BestDist = Dist;
        BestPos = Predicted;
        BestTargetLoc = BoneLoc;
        BestTargetBone = Bone;
        BestTime = T;
      }
    }

    if (bEnableDebug) {
      FLinearColor LinearColor =
          FLinearColor::LerpUsingHSV(FLinearColor::Green, FLinearColor::Red,
                                     Step / static_cast<float>(Steps));
      FColor StepColor =
          LinearColor.ToFColor(true); // Convert to gamma-corrected FColor

      DrawDebugSphere(World, Predicted, 4.f, 10, StepColor, false, 2.f);
      DrawDebugLine(World, LinearPart, Predicted, FColor::Cyan, false, 2.f);
      DrawDebugString(World, Predicted + FVector(0, 0, 12),
                      FString::Printf(TEXT("T: %.2f"), T), nullptr, StepColor,
                      2.f);
    }
  }

  if (BestTargetBone.IsNone())
    return Result;

  // --- 6. Scoring + Region ---
  const FVector ToTarget = (BestTargetLoc - Start).GetSafeNormal();
  const float Dot = FVector::DotProduct(Forward, ToTarget);
  const float Angle = FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f));
  const float DirScore = FMath::GetMappedRangeValueClamped(
      FVector2D(-1, 1), FVector2D(-1, 1), Dot);

  FName Region = NAME_None;
  float ImpactWeight = 1.f;
  const FString BoneStr = BestTargetBone.ToString().ToLower();

  if (BoneStr.Contains("head") || BoneStr.Contains("neck")) {
    Region = "Head";
    ImpactWeight = 2.f;
  } else if (BoneStr.Contains("spine") || BoneStr.Contains("pelvis")) {
    Region = "Torso";
    ImpactWeight = 1.5f;
  } else if (BoneStr.Contains("hand") || BoneStr.Contains("arm")) {
    Region = "Arm";
    ImpactWeight = 0.75f;
  } else if (BoneStr.Contains("leg") || BoneStr.Contains("foot")) {
    Region = "Leg";
    ImpactWeight = 0.75f;
  }

  // --- 7. Populate Result ---
  Result.bHitPredictionValid = true;
  Result.PredictedStrikePoint = BestPos;
  Result.ClosestTargetBone = BestTargetBone;
  Result.ClosestTargetBoneLocation = BestTargetLoc;
  Result.ClosestDistance = BestDist;
  Result.TimeToClosestPoint = BestTime;
  Result.TargetCharacter = ClosestTarget;
  Result.DirectionalityScore = DirScore * ImpactWeight;
  Result.AngularDeviationDegrees = FMath::RadiansToDegrees(Angle);
  Result.SkeletalRegion = Region;

  // --- 8. Debug ---
  if (bEnableDebug) {
    DrawDebugLine(World, BestPos, BestTargetLoc, FColor::Yellow, false, 2.f, 0,
                  2.f);
    DrawDebugSphere(World, BestTargetLoc, 7.f, 14, FColor::Red, false, 2.f);
    DrawDebugString(
        World, BestPos + FVector(0, 0, 30),
        FString::Printf(
            TEXT("Target: %s\nRegion: %s\nScore: %.2f\nAngle: %.1f°"),
            *BestTargetBone.ToString(), *Region.ToString(),
            Result.DirectionalityScore, Result.AngularDeviationDegrees),
        nullptr, FColor::White, 2.f);
  }

  return Result;
}

FCombatStrikePredictionResult UOHCombatUtils::AnalyzeAdvancedStrikeTrajectory2D(
    const UObject *WorldContextObject, ACharacter *Attacker,
    UOHPhysicsManager *PhysicsManager, FName StrikeBone, float PredictionTime,
    int Steps, bool bEnableDebug, FName DebugTag, FName OptionalChainStart,
    int32 MaxChainDepth, bool bIsFullBodyAnimation,
    float HistoricalBlendAlpha) {
  FCombatStrikePredictionResult Result;
  if (!WorldContextObject || !Attacker || !PhysicsManager ||
      StrikeBone.IsNone())
    return Result;

  UWorld *World = WorldContextObject->GetWorld();
  const USkeletalMeshComponent *Mesh = Attacker->GetMesh();
  if (!World || !Mesh)
    return Result;

  // --- 1. Bone Chain ---
  TArray<FName> BoneChain =
      OptionalChainStart.IsNone()
          ? UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, NAME_None,
                                                               StrikeBone)
          : UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(
                Mesh, OptionalChainStart, StrikeBone);

  if (MaxChainDepth > 0 && BoneChain.Num() > MaxChainDepth) {
    TArray<FName> Truncated;
    for (int32 i = BoneChain.Num() - MaxChainDepth; i < BoneChain.Num(); ++i)
      Truncated.Add(BoneChain[i]);
    BoneChain = Truncated;
  }
  if (BoneChain.Num() == 0)
    BoneChain.Add(StrikeBone);

  // --- 2. Aggregate Bone Motion ---
  TArray<const FOHBoneData *> ValidBones;
  FVector SumPos, SumVel, SumHistVel, SumAccel, SumForward, SumAngVel,
      SumAngAccel = FVector::ZeroVector;

  for (const FName &Bone : BoneChain) {
    if (!bIsFullBodyAnimation &&
        (Bone == "pelvis" || Bone.ToString().Contains(TEXT("root"))))
      continue;

    const FOHBoneData *Data = PhysicsManager->GetBoneData(Bone);
    if (!Data || !Data->IsValid())
      continue;

    ValidBones.Add(Data);
    SumPos += Data->GetCurrentPosition();
    SumVel += Data->GetSmoothedLinearVelocity();
    SumHistVel += Data->GetBlendedLinearVelocity(HistoricalBlendAlpha, 6);
    SumAccel += Data->GetLinearAcceleration();
    SumAngVel += Data->GetBodyAngularVelocity();
    SumAngAccel += Data->GetAngularAcceleration();
    SumForward += Data->GetCurrentRotation().Vector();

    if (bEnableDebug) {
      DrawDebugSphere(World, Data->GetCurrentPosition(), 3.f, 8, FColor::Purple,
                      false, 2.f);
      DrawDebugString(World, Data->GetCurrentPosition() + FVector(0, 0, 10),
                      Bone.ToString(), nullptr, FColor::White, 2.f);
    }
  }
  if (ValidBones.Num() == 0)
    return Result;

  const FVector Start = SumPos / ValidBones.Num();
  const FVector Forward = SumForward.GetSafeNormal();
  const FVector LinearVel = (SumVel + SumHistVel) / (ValidBones.Num() * 2);
  const FVector LinearAccel = SumAccel / ValidBones.Num();
  const FVector AngularVel = SumAngVel / ValidBones.Num();
  const FVector AngularAccel = SumAngAccel / ValidBones.Num();

  // --- 3. Pose-Aware Arc Radius ---
  float FullExtension = FVector::Dist(Mesh->GetBoneLocation(BoneChain[0]),
                                      Mesh->GetBoneLocation(StrikeBone));
  float MaxChainLen = 0.f;
  for (int32 i = 1; i < BoneChain.Num(); ++i)
    MaxChainLen += FVector::Dist(Mesh->GetBoneLocation(BoneChain[i - 1]),
                                 Mesh->GetBoneLocation(BoneChain[i]));

  float ExtensionAlpha = FMath::Clamp(FullExtension / MaxChainLen, 0.f, 1.f);
  float ArcRadius = MaxChainLen * FMath::Lerp(0.5f, 1.2f, ExtensionAlpha);

  // --- 4. Find Closest Target ---
  ACharacter *ClosestTarget = nullptr;
  float MinDistSq = FLT_MAX;
  for (TActorIterator<ACharacter> It(World); It; ++It) {
    ACharacter *Other = *It;
    if (!Other || Other == Attacker)
      continue;

    float DistSq = FVector::DistSquared(Attacker->GetActorLocation(),
                                        Other->GetActorLocation());
    if (DistSq < MinDistSq) {
      MinDistSq = DistSq;
      ClosestTarget = Other;
    }
  }
  if (!ClosestTarget)
    return Result;

  USkeletalMeshComponent *TargetMesh = ClosestTarget->GetMesh();
  if (!TargetMesh)
    return Result;

  TArray<FName> TargetBones;
  if (UOHPhysicsManager *TargetPM =
          ClosestTarget->FindComponentByClass<UOHPhysicsManager>())
    TargetBones = TargetPM->GetTrackedBoneNames();
  if (TargetBones.Num() == 0)
    TargetMesh->GetBoneNames(TargetBones);

  // --- 5. 2D Arc Prediction ---
  const float StepSize = PredictionTime / FMath::Max(1, Steps);
  float BestDist = FLT_MAX;
  FVector BestPredicted;
  FVector BestTargetLoc;
  FName BestTargetBone = NAME_None;
  float BestTime = 0.f;

  const FVector Tangent =
      FVector::CrossProduct(AngularVel, Forward).GetSafeNormal();

  for (int32 Step = 1; Step <= Steps; ++Step) {
    const float T = Step * StepSize;
    FVector StepPos = Start + LinearVel * T + 0.5f * LinearAccel * T * T;
    StepPos += Tangent *
               (ArcRadius *
                FMath::Sin((AngularVel.Size() + AngularAccel.Size() * T) * T));

    // Project X and Z, keep actual Z height
    StepPos =
        FVector(StepPos.X, Start.Y,
                StepPos.Z); // maintains height, fixes “on the floor” issue

    for (const FName &Bone : TargetBones) {
      FVector BoneLoc = TargetMesh->GetBoneLocation(Bone);
      BoneLoc = FVector(BoneLoc.X, Start.Y, BoneLoc.Z);

      const float Dist = FVector::Dist(StepPos, BoneLoc);
      if (Dist < BestDist) {
        BestDist = Dist;
        BestPredicted = StepPos;
        BestTargetBone = Bone;
        BestTargetLoc = BoneLoc;
        BestTime = T;
      }
    }

    if (bEnableDebug) {
      DrawDebugSphere(World, StepPos, 4.f, 10, FColor::Orange, false, 2.f);
      DrawDebugString(World, StepPos + FVector(0, 0, 10),
                      FString::Printf(TEXT("T: %.2f"), T), nullptr,
                      FColor::White, 2.f);
    }
  }
  if (BestTargetBone.IsNone())
    return Result;

  // --- 6. Scoring ---
  const FVector ToTarget = (BestTargetLoc - Start).GetSafeNormal();
  const float Dot = FVector::DotProduct(Forward, ToTarget);
  const float Angle = FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f));
  const float DirScore = FMath::GetMappedRangeValueClamped(
      FVector2D(-1, 1), FVector2D(-1, 1), Dot);

  // --- 7. Region & Result ---
  FName Region = NAME_None;
  float ImpactWeight = 1.f;
  const FString BoneStr = BestTargetBone.ToString().ToLower();

  if (BoneStr.Contains("head") || BoneStr.Contains("neck")) {
    Region = "Head";
    ImpactWeight = 2.f;
  } else if (BoneStr.Contains("spine") || BoneStr.Contains("pelvis")) {
    Region = "Torso";
    ImpactWeight = 1.5f;
  } else if (BoneStr.Contains("hand") || BoneStr.Contains("arm")) {
    Region = "Arm";
    ImpactWeight = 0.75f;
  } else if (BoneStr.Contains("leg") || BoneStr.Contains("foot")) {
    Region = "Leg";
    ImpactWeight = 0.75f;
  }

  Result.bHitPredictionValid = true;
  Result.PredictedStrikePoint = BestPredicted;
  Result.ClosestTargetBone = BestTargetBone;
  Result.ClosestTargetBoneLocation = BestTargetLoc;
  Result.ClosestDistance = BestDist;
  Result.TimeToClosestPoint = BestTime;
  Result.TargetCharacter = ClosestTarget;
  Result.DirectionalityScore = DirScore * ImpactWeight;
  Result.AngularDeviationDegrees = FMath::RadiansToDegrees(Angle);
  Result.SkeletalRegion = Region;

  // --- 8. Debug Overlay ---
  if (bEnableDebug) {
    DrawDebugLine(World, BestPredicted, BestTargetLoc, FColor::Yellow, false,
                  2.f, 0, 2.f);
    DrawDebugSphere(World, BestTargetLoc, 6.f, 14, FColor::Red, false, 2.f);
    DrawDebugString(
        World, BestPredicted + FVector(0, 0, 30),
        FString::Printf(
            TEXT("Target: %s\nRegion: %s\nScore: %.2f\nAngle: %.1f°"),
            *BestTargetBone.ToString(), *Region.ToString(),
            Result.DirectionalityScore, Result.AngularDeviationDegrees),
        nullptr, FColor::White, 2.f);
  }

  return Result;
}

#pragma endregion

#pragma region Sampling

FOHBoneMotionSample UOHCombatUtils::SampleBoneMotionFromAnimBase(
    UAnimSequenceBase *AnimBase, USkeletalMeshComponent *Mesh, FName BoneName,
    int32 NumSamples, float StartTime, float EndTime, FName MontageSectionName,
    bool bDrawDebug, FColor DebugColor, float DebugDuration,
    float DebugThickness) {
  FOHBoneMotionSample Result =
      SampleBoneMotionInternal(AnimBase, Mesh, BoneName, NumSamples, StartTime,
                               EndTime, MontageSectionName);

  if (bDrawDebug && Mesh && Mesh->GetWorld()) {
    DrawDebugPath(Mesh, Result.Points, DebugColor, DebugDuration,
                  DebugThickness);
  }

  return Result;
}

FOHBoneMotionSample UOHCombatUtils::SampleBoneMotionFromAnimBasePure(
    UAnimSequenceBase *AnimBase, USkeletalMeshComponent *Mesh, FName BoneName,
    int32 NumSamples, float StartTime, float EndTime,
    FName MontageSectionName) {
  return SampleBoneMotionInternal(AnimBase, Mesh, BoneName, NumSamples,
                                  StartTime, EndTime, MontageSectionName);
}

FOHBoneMotionSample UOHCombatUtils::SampleBoneMotionInternal(
    UAnimSequenceBase *AnimBase, USkeletalMeshComponent *Mesh, FName BoneName,
    int32 NumSamples, float StartTime, float EndTime,
    FName MontageSectionName) {
  FOHBoneMotionSample Result;
  Result.BoneName = BoneName;

  if (!AnimBase || !Mesh || BoneName.IsNone() || NumSamples < 2) {
    UE_LOG(LogTemp, Warning, TEXT("[SampleBoneMotion] Invalid input."));
    return Result;
  }

  const USkeletalMesh *SkelMesh = Mesh->GetSkeletalMeshAsset();
  if (!SkelMesh || !SkelMesh->GetSkeleton()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[SampleBoneMotion] Mesh or skeleton not valid at runtime."));
    return Result;
  }

  const float AssetDuration = AnimBase->GetPlayLength();

  // Extract montage section timing if provided
  if (UAnimMontage *Montage = Cast<UAnimMontage>(AnimBase)) {
    if (!MontageSectionName.IsNone()) {
      const int32 SectionIndex = Montage->GetSectionIndex(MontageSectionName);
      if (Montage->CompositeSections.IsValidIndex(SectionIndex)) {
        StartTime = Montage->CompositeSections[SectionIndex].GetTime();
        EndTime = StartTime + Montage->GetSectionLength(SectionIndex);
      }
    } else if (StartTime <= 0.f && EndTime <= 0.f) {
      // Attempt to infer current playback section at runtime
      if (UAnimInstance *AnimInst = Mesh->GetAnimInstance()) {
        if (const FAnimMontageInstance *MontageInst =
                AnimInst->GetActiveMontageInstance()) {
          const float CurrentTime = MontageInst->GetPosition();
          const int32 SectionIndex =
              Montage->GetSectionIndexFromPosition(CurrentTime);
          if (Montage->CompositeSections.IsValidIndex(SectionIndex)) {
            StartTime = Montage->CompositeSections[SectionIndex].GetTime();
            EndTime = StartTime + Montage->GetSectionLength(SectionIndex);
          }
        }
      }
    }
  }

  if (StartTime < 0.f || EndTime < 0.f ||
      FMath::IsNearlyEqual(StartTime, EndTime)) {
    StartTime = 0.01f;
    EndTime = AssetDuration;
  }

  const double T0 = FMath::Clamp(static_cast<double>(StartTime), 0.0,
                                 static_cast<double>(AssetDuration));
  const double T1 = FMath::Clamp(static_cast<double>(EndTime), T0,
                                 static_cast<double>(AssetDuration));
  const double DeltaT = T1 - T0;

  if (FMath::IsNearlyZero(DeltaT)) {
    UE_LOG(LogTemp, Warning,
           TEXT("[SampleBoneMotion] Insufficient delta time (%.4f)"), DeltaT);
    return Result;
  }

  const float SampleInterval = DeltaT / static_cast<float>(NumSamples - 1);

  // Resolve to UAnimSequence
  UAnimSequence *Sequence = ExtractSequenceFromAnimBaseAtTime(AnimBase, T0);
  if (!Sequence) {
    UE_LOG(
        LogTemp, Warning,
        TEXT(
            "[SampleBoneMotion] Could not resolve sequence at time %.2f in %s"),
        T0, *GetNameSafe(AnimBase));
    return Result;
  }

  for (int32 i = 0; i < NumSamples; ++i) {
    const double Alpha =
        static_cast<double>(i) / static_cast<double>(NumSamples - 1);
    const double Time = T0 + Alpha * DeltaT;

    FTransform BoneTransform;
    if (UOHCombatUtils::GetBoneTransformAtTime(Sequence, Mesh, BoneName,
                                               static_cast<float>(Time),
                                               BoneTransform)) {
      const FVector Pos = BoneTransform.GetLocation();
      if (!Pos.ContainsNaN()) {
        Result.Points.Add(Pos);
      }
    }
  }

  Result.NumPoints = Result.Points.Num();
  Result.Duration = DeltaT;
  Result.SourceAnimation = Sequence;

  if (Result.NumPoints > 1) {
    UOHCombatUtils::SampleMotionDerivatives(
        Result.Points, SampleInterval, Result.Velocities, Result.Accelerations);
  }

  return Result;
}

void UOHCombatUtils::ScheduleDeferredBoneArcSample(
    UObject *WorldContext, UAnimSequenceBase *AnimBase,
    USkeletalMeshComponent *Mesh, FName BoneName, int32 NumSamples,
    FName MontageSectionName, FColor DebugColor, float Duration,
    float Thickness) {
  if (!WorldContext || !Mesh || !AnimBase || BoneName.IsNone())
    return;

  UWorld *World = WorldContext->GetWorld();
  if (!World)
    return;

  // Capture weak refs to pass safely to timer lambda
  TWeakObjectPtr<USkeletalMeshComponent> WeakMesh = Mesh;
  TWeakObjectPtr<UAnimSequenceBase> WeakAnim = AnimBase;

  FTimerDelegate TimerCallback;
  TimerCallback.BindLambda([=]() {
    if (!WeakMesh.IsValid() || !WeakAnim.IsValid())
      return;

    UOHCombatUtils::VisualizeBoneMotionArcFromAnim(
        WorldContext, WeakAnim.Get(), WeakMesh.Get(), BoneName, NumSamples,
        MontageSectionName, DebugColor, Duration, Thickness);
  });

  FTimerHandle Handle;
  World->GetTimerManager().SetTimer(Handle, TimerCallback,
                                    0.01f, // next frame
                                    false);
}

#if 0
FOHBoneMotionSample UOHCombatUtils::SampleBoneMotionInternal(
	UAnimSequenceBase* AnimBase,
	USkeletalMeshComponent* Mesh,
	FName BoneName,
	int32 NumSamples,
	float StartTime,
	float EndTime,
	FName MontageSectionName)
{
	FOHBoneMotionSample Result;
	Result.BoneName = BoneName;

	if (!AnimBase || !Mesh || BoneName.IsNone() || NumSamples < 2)
		return Result;

	const float AssetDuration = AnimBase->GetPlayLength();

	// Try to extract section bounds if a montage and section provided
	if (UAnimMontage* Montage = Cast<UAnimMontage>(AnimBase))
	{
		if (!MontageSectionName.IsNone())
		{
			const int32 SectionIndex = Montage->GetSectionIndex(MontageSectionName);
			if (Montage->CompositeSections.IsValidIndex(SectionIndex))
			{
				StartTime = Montage->CompositeSections[SectionIndex].GetTime();
				EndTime = StartTime + Montage->GetSectionLength(SectionIndex);
			}
		}
		else if (StartTime <= 0.f && EndTime <= 0.f)
		{
			// Use current playback section
			if (UAnimInstance* AnimInst = Mesh->GetAnimInstance())
			{
				if (const FAnimMontageInstance* MontageInst = AnimInst->GetActiveMontageInstance())
				{
					const float CurrentTime = MontageInst->GetPosition();
					const int32 SectionIndex = Montage->GetSectionIndexFromPosition(CurrentTime);
					if (Montage->CompositeSections.IsValidIndex(SectionIndex))
					{
						StartTime = Montage->CompositeSections[SectionIndex].GetTime();
						EndTime = StartTime + Montage->GetSectionLength(SectionIndex);
					}
				}
			}
		}
	}

	if (StartTime < 0.f || EndTime < 0.f || FMath::IsNearlyEqual(StartTime, EndTime))
	{
		StartTime = 0.01f;
		EndTime = AssetDuration;
	}

	// Clamp
	const double T0 = FMath::Clamp(static_cast<double>(StartTime), 0.0, static_cast<double>(AssetDuration));
	const double T1 = FMath::Clamp(static_cast<double>(EndTime), T0, static_cast<double>(AssetDuration));
	const double DeltaT = T1 - T0;

	if (FMath::IsNearlyZero(DeltaT))
		return Result;

	const float SampleInterval = DeltaT / static_cast<float>(NumSamples - 1);

	// Use updated recursive sequence extractor
	UAnimSequence* Sequence = ExtractSequenceFromAnimBaseAtTime(AnimBase, T0);
	if (!Sequence)
	{
		UE_LOG(LogTemp, Warning, TEXT("[SampleBoneMotion] Could not resolve sequence at %.2f in %s"),
			T0, *GetNameSafe(AnimBase));
		return Result;
	}

	for (int32 i = 0; i < NumSamples; ++i)
	{
		const double Alpha = static_cast<double>(i) / static_cast<double>(NumSamples - 1);
		const double Time = T0 + Alpha * DeltaT;

		FTransform BoneTransform;
		if (GetBoneTransformAtTime(Sequence, Mesh, BoneName, Time, BoneTransform))
		{
			const FVector Pos = BoneTransform.GetLocation();
			if (!Pos.ContainsNaN())
			{
				Result.Points.Add(Pos);
			}
		}
	}

	ComputeMotionDerivatives(Result.Points, SampleInterval, Result.Velocities, Result.Accelerations);
	Result.NumPoints = Result.Points.Num();
	Result.Duration = DeltaT;
	Result.SourceAnimation = Sequence;

	return Result;
}
#endif

bool UOHCombatUtils::GetBoneTransformAtTime(UAnimSequence *Sequence,
                                            USkeletalMeshComponent *Mesh,
                                            FName BoneName, double Time,
                                            FTransform &OutTransform) {
  OutTransform = FTransform::Identity;

  if (!Sequence || !Mesh || BoneName.IsNone()) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] Invalid input to GetBoneTransformAtTime"));
    return false;
  }

  USkeletalMesh *SkelMesh = Mesh->GetSkeletalMeshAsset();
  if (!SkelMesh || !SkelMesh->GetSkeleton()) {
    UE_LOG(LogTemp, Error, TEXT("[OH] Skeletal mesh or skeleton not ready"));
    return false;
  }

  const FReferenceSkeleton &RefSkeleton = SkelMesh->GetRefSkeleton();
  const int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneName);
  if (BoneIndex == INDEX_NONE) {
    UE_LOG(LogTemp, Error, TEXT("[OH] Bone '%s' not found in skeleton"),
           *BoneName.ToString());
    return false;
  }

  TArray<FBoneIndexType> RequiredBones;
  const int32 NumBones = RefSkeleton.GetNum();
  RequiredBones.Reserve(NumBones);
  for (int32 i = 0; i < NumBones; ++i) {
    RequiredBones.Add(i);
  }

  FBoneContainer BoneContainer;
  UE::Anim::FCurveFilterSettings CurveFilter;
  BoneContainer.InitializeTo(RequiredBones, CurveFilter, *SkelMesh);

  const FCompactPoseBoneIndex CompactIndex =
      BoneContainer.GetCompactPoseIndexFromSkeletonIndex(BoneIndex);
  if (!CompactIndex.IsValid()) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] Invalid compact pose index for bone '%s'"),
           *BoneName.ToString());
    return false;
  }

  FCompactPose Pose;
  Pose.ResetToRefPose(BoneContainer);

  FBlendedCurve Curve;
  Curve.InitFrom(BoneContainer);
  UE::Anim::FStackAttributeContainer Attributes;
  FAnimationPoseData PoseData(Pose, Curve, Attributes);

  const FAnimExtractContext ExtractContext(Time, false);
  Sequence->GetAnimationPose(PoseData, ExtractContext);

  OutTransform = Pose[CompactIndex];
  return true;
}

bool UOHCombatUtils::GetBoneTransformWorldAtTime(UAnimSequence *Sequence,
                                                 USkeletalMeshComponent *Mesh,
                                                 FName BoneName, double Time,
                                                 FTransform &OutTransform) {
  FTransform LocalTransform;
  if (!GetBoneTransformAtTime(Sequence, Mesh, BoneName, Time, LocalTransform)) {
    OutTransform = FTransform::Identity;
    return false;
  }

  OutTransform = LocalTransform * Mesh->GetComponentTransform();
  return true;
}

bool UOHCombatUtils::GetBoneTransformFromAnimBase(UAnimSequenceBase *AnimBase,
                                                  USkeletalMeshComponent *Mesh,
                                                  FName BoneName, double Time,
                                                  FTransform &OutTransform,
                                                  bool bWorldSpace) {
  UAnimSequence *Sequence = ExtractSequenceFromAnimBaseAtTime(AnimBase, Time);
  if (!Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] Failed to extract sequence from anim base at %.2f"),
           Time);
    OutTransform = FTransform::Identity;
    return false;
  }

  if (bWorldSpace) {
    return GetBoneTransformWorldAtTime(Sequence, Mesh, BoneName, Time,
                                       OutTransform);
  } else {
    return GetBoneTransformAtTime(Sequence, Mesh, BoneName, Time, OutTransform);
  }
}

bool UOHCombatUtils::GetBoneRotationAtTime(UAnimSequence *Sequence,
                                           USkeletalMeshComponent *Mesh,
                                           FName BoneName, double Time,
                                           FRotator &OutRotation,
                                           bool bWorldSpace) {
  FTransform BoneTransform;
  const bool bSuccess =
      bWorldSpace ? GetBoneTransformWorldAtTime(Sequence, Mesh, BoneName, Time,
                                                BoneTransform)
                  : GetBoneTransformAtTime(Sequence, Mesh, BoneName, Time,
                                           BoneTransform);

  if (!bSuccess) {
    OutRotation = FRotator::ZeroRotator;
    return false;
  }

  OutRotation = BoneTransform.GetRotation().Rotator();
  return true;
}

bool UOHCombatUtils::GetBoneVelocityAtTime(UAnimSequence *Sequence,
                                           USkeletalMeshComponent *Mesh,
                                           FName BoneName, double Time,
                                           FVector &OutVelocity,
                                           float SampleWindow) {
  OutVelocity = FVector::ZeroVector;

  if (!Sequence || !Mesh || BoneName.IsNone()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] Invalid input to GetBoneVelocityAtTime"));
    return false;
  }

  if (SampleWindow <= 0.f) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] SampleWindow must be > 0"));
    return false;
  }

  const float Duration = Sequence->GetPlayLength();
  const float T0 = FMath::Clamp(Time - SampleWindow * 0.5f, 0.f, Duration);
  const float T1 = FMath::Clamp(Time + SampleWindow * 0.5f, 0.f, Duration);

  FTransform X0, X1;
  if (!GetBoneTransformWorldAtTime(Sequence, Mesh, BoneName, T0, X0) ||
      !GetBoneTransformWorldAtTime(Sequence, Mesh, BoneName, T1, X1)) {
    return false;
  }

  const FVector P0 = X0.GetLocation();
  const FVector P1 = X1.GetLocation();

  if (P0.ContainsNaN() || P1.ContainsNaN()) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] Bone velocity sampling returned NaN"));
    return false;
  }

  OutVelocity = (P1 - P0) / SampleWindow;
  return true;
}

FTransform UOHCombatUtils::GetBoneComponentSpaceTransformAtTime(
    UAnimSequence *AnimSequence, const USkeletalMeshComponent *Mesh,
    FName BoneName, double Time) {
  if (!AnimSequence || !Mesh || !Mesh->GetSkeletalMeshAsset()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] [GetBoneComponentSpaceTransformAtTime] Invalid input."));
    return FTransform::Identity;
  }

  const USkeletalMesh *SkelMesh = Mesh->GetSkeletalMeshAsset();
  const FReferenceSkeleton &RefSkeleton = SkelMesh->GetRefSkeleton();

  const int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneName);
  if (BoneIndex == INDEX_NONE) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] [GetBoneComponentSpaceTransformAtTime] Bone '%s' not "
                "found in mesh."),
           *BoneName.ToString());
    return FTransform::Identity;
  }

  // Build required bone list
  TArray<FBoneIndexType> RequiredBones;
  const int32 NumBones = RefSkeleton.GetNum();
  RequiredBones.Reserve(NumBones);
  for (int32 i = 0; i < NumBones; ++i) {
    RequiredBones.Add(i);
  }

  // Safe asset for bone container
  UObject *BoneAsset = GetValidBoneAssetForContainer(Mesh);
  if (!BoneAsset) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] Failed to resolve valid asset for bone container."));
    return FTransform::Identity;
  }

  FBoneContainer BoneContainer;
  UE::Anim::FCurveFilterSettings CurveFilter;
  BoneContainer.InitializeTo(RequiredBones, CurveFilter, *BoneAsset);

  // Build pose
  FCompactPose Pose;
  FBlendedCurve Curve;
  UE::Anim::FStackAttributeContainer Attributes;

  Pose.ResetToRefPose(BoneContainer);
  Curve.InitFrom(BoneContainer);
  FAnimationPoseData PoseData(Pose, Curve, Attributes);

  const FDeltaTimeRecord DummyTimeRecord;
  const FAnimExtractContext ExtractContext(Time, false, DummyTimeRecord, false);
  AnimSequence->GetAnimationPose(PoseData, ExtractContext);

  // Get Compact index for target bone
  const FMeshPoseBoneIndex MeshPoseBoneIndex(BoneIndex);
  const FCompactPoseBoneIndex CompactIndex =
      BoneContainer.MakeCompactPoseIndex(MeshPoseBoneIndex);
  if (!Pose.IsValidIndex(CompactIndex)) {
    UE_LOG(LogTemp, Warning, TEXT("[OH] Invalid CompactPose index for '%s'."),
           *BoneName.ToString());
    return FTransform::Identity;
  }

  // Accumulate transforms up hierarchy
  FTransform BoneTransform = Pose[CompactIndex];
  int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);

  while (ParentIndex != INDEX_NONE) {
    const FMeshPoseBoneIndex ParentMeshIndex(ParentIndex);
    const FCompactPoseBoneIndex ParentCompactIndex =
        BoneContainer.MakeCompactPoseIndex(ParentMeshIndex);

    if (!Pose.IsValidIndex(ParentCompactIndex)) {
      break;
    }

    BoneTransform = BoneTransform * Pose[ParentCompactIndex];
    ParentIndex = RefSkeleton.GetParentIndex(ParentIndex);
  }

  // Return in component space
  return BoneTransform * Mesh->GetComponentTransform();
}

void UOHCombatUtils::ComputeMotionDerivatives(
    const TArray<FVector> &Points, float SampleDelta,
    TArray<FVector> &OutVelocities, TArray<FVector> &OutAccelerations) {
  const int32 Count = Points.Num();
  OutVelocities.SetNumUninitialized(Count);
  OutAccelerations.SetNumUninitialized(Count);

  for (int32 i = 0; i < Count; ++i) {
    if (i == 0) {
      OutVelocities[i] = (Points[i + 1] - Points[i]) / SampleDelta;
      OutAccelerations[i] = FVector::ZeroVector;
    } else if (i == Count - 1) {
      OutVelocities[i] = (Points[i] - Points[i - 1]) / SampleDelta;
      OutAccelerations[i] = FVector::ZeroVector;
    } else {
      OutVelocities[i] = (Points[i + 1] - Points[i - 1]) / (2.f * SampleDelta);
      OutAccelerations[i] = (Points[i + 1] - 2.f * Points[i] + Points[i - 1]) /
                            (SampleDelta * SampleDelta);
    }
  }
}

bool UOHCombatUtils::ResolveAnimSequenceAtTimeRecursive(
    UAnimSequenceBase *AnimBase, float Time, UAnimSequence *&OutSequence,
    float &OutLocalTime) {
  if (!AnimBase) {
    UE_LOG(LogTemp, Warning,
           TEXT("[ResolveAnimSequenceAtTimeRecursive] Null AnimBase."));
    return false;
  }

  if (UAnimSequence *Seq = Cast<UAnimSequence>(AnimBase)) {
    OutSequence = Seq;
    OutLocalTime = Time;
    return true;
  }

  const auto TrySegments = [&](const TArray<FAnimSegment> &Segments) -> bool {
    for (const FAnimSegment &Segment : Segments) {
      const float Start = Segment.StartPos;
      const float End = Start + Segment.GetLength();

      if (Time >= Start && Time <= End) {
        if (UAnimSequenceBase *Nested = Segment.GetAnimReference()) {
          return ResolveAnimSequenceAtTimeRecursive(Nested, Time - Start,
                                                    OutSequence, OutLocalTime);
        }
      }
    }
    return false;
  };

  if (UAnimComposite *Composite = Cast<UAnimComposite>(AnimBase)) {
    return TrySegments(Composite->AnimationTrack.AnimSegments);
  }

  if (UAnimMontage *Montage = Cast<UAnimMontage>(AnimBase)) {
    for (const FSlotAnimationTrack &Slot : Montage->SlotAnimTracks) {
      if (TrySegments(Slot.AnimTrack.AnimSegments)) {
        return true;
      }
    }
  }

  UE_LOG(LogTemp, Warning,
         TEXT("[ResolveAnimSequenceAtTimeRecursive] Could not resolve sequence "
              "at time %.3f in %s"),
         Time, *GetNameSafe(AnimBase));
  return false;
}

UAnimSequence *
UOHCombatUtils::ExtractSequenceFromAnimBaseAtTime(UAnimSequenceBase *AnimBase,
                                                  float Time) {
  if (!AnimBase)
    return nullptr;

  UAnimSequence *OutSequence = nullptr;
  float OutLocalTime = 0.f;

  const bool bResolved = ResolveAnimSequenceAtTimeRecursive(
      AnimBase, Time, OutSequence, OutLocalTime);
  if (!bResolved || !OutSequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("ExtractSequenceFromAnimBaseAtTime: No sequence found at time "
                "%.3f in %s"),
           Time, *GetNameSafe(AnimBase));
  }

  return OutSequence;
}

UAnimSequence *
UOHCombatUtils::ExtractSequenceFromAnimBase(UAnimSequenceBase *AnimBase) {
  if (!AnimBase)
    return nullptr;

  // Use a small positive time to avoid edge cases at exact zero
  constexpr float SafeStartTime = 0.01f;
  return ExtractSequenceFromAnimBaseAtTime(AnimBase, SafeStartTime);
}

UAnimSequence *
UOHCombatUtils::ExtractActiveSequenceFromMesh(USkeletalMeshComponent *Mesh) {
  if (!Mesh || !Mesh->GetAnimInstance()) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("ExtractActiveSequenceFromMesh: Invalid mesh or anim instance"));
    return nullptr;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  UAnimMontage *Montage = AnimInstance->GetCurrentActiveMontage();

  if (!Montage) {
    UE_LOG(LogTemp, Warning,
           TEXT("ExtractActiveSequenceFromMesh: No active montage"));
    return nullptr;
  }

  const float PlaybackTime = AnimInstance->Montage_GetPosition(Montage);
  UAnimSequence *Sequence =
      ExtractSequenceFromAnimBaseAtTime(Montage, PlaybackTime);

  if (!Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("ExtractActiveSequenceFromMesh: Could not extract sequence at "
                "time %.2f from %s"),
           PlaybackTime, *Montage->GetName());
  } else {
    UE_LOG(LogTemp, Log,
           TEXT("ExtractActiveSequenceFromMesh: Resolved '%s' at time %.2f"),
           *Sequence->GetName(), PlaybackTime);
  }

  return Sequence;
}

UAnimSequence *UOHCombatUtils::ExtractActiveSequenceFromMeshPure(
    USkeletalMeshComponent *Mesh) {
  if (!Mesh || !Mesh->GetAnimInstance()) {
    return nullptr;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  UAnimMontage *Montage = AnimInstance->GetCurrentActiveMontage();

  if (!Montage) {
    return nullptr;
  }

  const float PlaybackTime = AnimInstance->Montage_GetPosition(Montage);
  return ExtractSequenceFromAnimBaseAtTime(Montage, PlaybackTime);
}

bool UOHCombatUtils::ExtractActiveSequenceAndTimeFromMesh(
    USkeletalMeshComponent *Mesh, UAnimSequence *&OutSequence,
    float &OutLocalTime) {
  OutSequence = nullptr;
  OutLocalTime = 0.f;

  if (!Mesh || !Mesh->GetAnimInstance()) {
    return false;
  }

  UAnimInstance *AnimInstance = Mesh->GetAnimInstance();
  UAnimMontage *Montage = AnimInstance->GetCurrentActiveMontage();

  if (!Montage) {
    return false;
  }

  const float PlaybackTime = AnimInstance->Montage_GetPosition(Montage);
  return ResolveAnimSequenceAtTimeRecursive(Montage, PlaybackTime, OutSequence,
                                            OutLocalTime);
}

void UOHCombatUtils::StartTimedBoneArcSampling(
    UObject *WorldContextObject, USkeletalMeshComponent *Mesh, FName BoneName,
    float Duration, int32 NumSamples,
    TFunction<void(const TArray<FVector> &)> OnComplete) {
  if (!Mesh || BoneName.IsNone() || NumSamples < 2 || Duration <= 0.f)
    return;

  UWorld *World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
  if (!World)
    return;

  const float Interval = Duration / (NumSamples - 1);

  FOHAsyncBoneArcSampler *Sampler = new FOHAsyncBoneArcSampler();
  Sampler->Mesh = Mesh; // Now TWeakObjectPtr
  Sampler->BoneName = BoneName;
  Sampler->Duration = Duration;
  Sampler->Interval = Interval;
  Sampler->MaxSamples = NumSamples;
  Sampler->ArcPoints.Reserve(NumSamples);

  World->GetTimerManager().SetTimer(
      Sampler->TimerHandle,
      [Sampler, OnComplete, World]() {
        if (!Sampler->Mesh.IsValid() ||
            Sampler->ElapsedTime > Sampler->Duration) {
          World->GetTimerManager().ClearTimer(Sampler->TimerHandle);
          OnComplete(Sampler->ArcPoints);
          delete Sampler;
          return;
        }

        if (Sampler->Mesh->DoesSocketExist(Sampler->BoneName)) {
          const FVector Pos =
              Sampler->Mesh->GetSocketLocation(Sampler->BoneName);
          Sampler->ArcPoints.Add(Pos);
        }

        Sampler->ElapsedTime += Sampler->Interval;
      },
      Sampler->Interval,
      true // looping
  );
}

UObject *UOHCombatUtils::GetValidBoneAssetForContainer(
    const USkeletalMeshComponent *Mesh) {
  if (!Mesh)
    return nullptr;

  if (USkeletalMesh *SkelMesh = Mesh->GetSkeletalMeshAsset()) {
    return SkelMesh;
  }

  if (const UAnimInstance *AnimInstance = Mesh->GetAnimInstance()) {
    if (const UAnimMontage *Montage = AnimInstance->GetCurrentActiveMontage()) {
      if (const USkeleton *Skeleton = Montage->GetSkeleton()) {
        return const_cast<USkeleton *>(Skeleton);
      }
    }
  }

  return nullptr;
}

bool UOHCombatUtils::GetBoneWorldSpaceVelocityAtTime(
    UAnimSequence *Sequence, const USkeletalMeshComponent *Mesh, FName BoneName,
    float Time, FVector &OutVelocity, float SampleWindow) {
  OutVelocity = FVector::ZeroVector;

  if (!Sequence || !Mesh || BoneName.IsNone() || SampleWindow <= 0.f) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] Invalid input to GetBoneWorldSpaceVelocityAtTime"));
    return false;
  }

  const float Duration = Sequence->GetPlayLength();
  const float T0 = FMath::Clamp(Time - SampleWindow * 0.5f, 0.f, Duration);
  const float T1 = FMath::Clamp(Time + SampleWindow * 0.5f, 0.f, Duration);

  const FTransform X0 =
      GetBoneComponentSpaceTransformAtTime(Sequence, Mesh, BoneName, T0);
  const FTransform X1 =
      GetBoneComponentSpaceTransformAtTime(Sequence, Mesh, BoneName, T1);

  const FVector Pos0 = X0.GetLocation();
  const FVector Pos1 = X1.GetLocation();

  if (Pos0.ContainsNaN() || Pos1.ContainsNaN()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] Bone position NaN during velocity sampling"));
    return false;
  }

  OutVelocity = (Pos1 - Pos0) / FMath::Max(SampleWindow, KINDA_SMALL_NUMBER);
  return true;
}

void UOHCombatUtils::SampleMotionDerivatives(
    const TArray<FVector> &Positions, float SampleInterval,
    TArray<FVector> &OutVelocities, TArray<FVector> &OutAccelerations) {
  const int32 Count = Positions.Num();
  OutVelocities.Reset();
  OutAccelerations.Reset();

  if (Count < 2 || SampleInterval <= 0.f)
    return;

  // --- Velocity: finite difference
  for (int32 i = 1; i < Count; ++i) {
    const FVector V = (Positions[i] - Positions[i - 1]) / SampleInterval;
    OutVelocities.Add(V);
  }

  // Pad to match size
  OutVelocities.Insert(OutVelocities[0], 0);

  // --- Acceleration: second derivative
  for (int32 i = 1; i < OutVelocities.Num(); ++i) {
    const FVector A =
        (OutVelocities[i] - OutVelocities[i - 1]) / SampleInterval;
    OutAccelerations.Add(A);
  }

  // Pad to match size
  OutAccelerations.Insert(OutAccelerations[0], 0);
}

void UOHCombatUtils::VisualizeMotionDerivatives(
    const UObject *WorldContextObject, const TArray<FVector> &Points,
    const TArray<FVector> &Velocities, const TArray<FVector> &Accelerations,
    float Duration, float Scale, float Thickness) {
  if (!WorldContextObject || Points.Num() < 2 ||
      Velocities.Num() != Points.Num() || Accelerations.Num() != Points.Num()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] VisualizeMotionDerivatives: Mismatched or invalid input "
                "arrays."));
    return;
  }

  UWorld *World = WorldContextObject->GetWorld();
  if (!World)
    return;

  for (int32 i = 0; i < Points.Num(); ++i) {
    const FVector Origin = Points[i];

    // Draw velocity vector (yellow)
    const FVector V = Velocities[i] * Scale;
    DrawDebugLine(World, Origin, Origin + V, FColor::Yellow, false, Duration, 0,
                  Thickness);

    // Draw acceleration vector (cyan)
    const FVector A = Accelerations[i] * Scale;
    DrawDebugLine(World, Origin, Origin + A, FColor::Cyan, false, Duration, 0,
                  Thickness * 0.75f);
  }
}

FOHBoneMotionSample UOHCombatUtils::GenerateBoneMotionSample(
    UAnimSequenceBase *AnimBase, USkeletalMeshComponent *Mesh, FName BoneName,
    int32 NumSamples, FName MontageSectionName, float StartTime,
    float EndTime) {
  FOHBoneMotionSample Sample;
  Sample.BoneName = BoneName;

  if (!AnimBase) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] GenerateBoneMotionSample: Null AnimBase"));
    return Sample;
  }

  if (!Mesh || !Mesh->GetSkeletalMeshAsset()) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("[OH] GenerateBoneMotionSample: Invalid SkeletalMeshComponent"));
    return Sample;
  }

  if (BoneName.IsNone()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] GenerateBoneMotionSample: BoneName is None"));
    return Sample;
  }

  if (NumSamples < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] GenerateBoneMotionSample: NumSamples too low (%d)"),
           NumSamples);
    return Sample;
  }

  const bool bCallerProvidedRange =
      !FMath::IsNearlyZero(StartTime) || !FMath::IsNearlyZero(EndTime);
  const float MaxDuration = AnimBase->GetPlayLength();

  if (bCallerProvidedRange) {
    if (StartTime < 0.f || EndTime < 0.f || StartTime >= EndTime ||
        EndTime > MaxDuration) {
      UE_LOG(LogTemp, Warning,
             TEXT("[OH] GenerateBoneMotionSample: Invalid time range provided "
                  "(%.2f → %.2f), Max: %.2f"),
             StartTime, EndTime, MaxDuration);
      return Sample;
    }
  }

  Sample = SampleBoneMotionInternal(AnimBase, Mesh, BoneName, NumSamples,
                                    StartTime, EndTime, MontageSectionName);

  if (Sample.Points.Num() < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] GenerateBoneMotionSample: Insufficient motion points "
                "returned"));
  }

  return Sample;
}

void UOHCombatUtils::TestVisualizeBoneMotionArcFromMesh(
    USkeletalMeshComponent *Mesh, FName BoneName, float PredictDuration,
    int32 NumSamples, FColor DebugColor, float DebugLineDuration,
    float LineThickness, bool bLoop) {
  if (!Mesh || BoneName.IsNone()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] TestVisualizeBoneMotionArcFromMesh: Invalid input."));
    return;
  }

  UAnimSequence *Sequence = nullptr;
  float LocalTime = 0.f;

  if (!UOHCombatUtils::ExtractActiveSequenceAndTimeFromMesh(Mesh, Sequence,
                                                            LocalTime) ||
      !Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] Failed to extract active sequence from mesh."));
    return;
  }

  const float Duration = Sequence->GetPlayLength();
  const float T0 = FMath::Clamp(LocalTime, 0.f, Duration);
  const float T1 = FMath::Clamp(LocalTime + PredictDuration, T0, Duration);

  TArray<FVector> ArcPoints;
  const bool bSuccess = UOHCombatUtils::GenerateStrikeArcFromAnimSequence(
      Sequence, Mesh, BoneName, T0, T1, ArcPoints,
      NumSamples > 2 ? static_cast<float>(NumSamples) / (T1 - T0)
                     : 60.f); // auto sample rate

  if (!bSuccess || ArcPoints.Num() < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] TestVisualize: Failed to generate arc for bone %s."),
           *BoneName.ToString());
    return;
  }

  UE_LOG(LogTemp, Log,
         TEXT("[OH] Drawing test arc (%d pts) for bone %s from %.2f → %.2f"),
         ArcPoints.Num(), *BoneName.ToString(), T0, T1);

  UOHCombatUtils::DrawDebugPath(Mesh, ArcPoints, DebugColor, DebugLineDuration,
                                LineThickness, bLoop, true /* persistent */);
}

void UOHCombatUtils::TestDrawArcFromMesh(USkeletalMeshComponent *Mesh,
                                         FName BoneName, float Duration,
                                         FColor Color) {
  if (!Mesh || BoneName.IsNone())
    return;

  UAnimSequence *Sequence = nullptr;
  float LocalTime = 0.f;

  if (!ExtractActiveSequenceAndTimeFromMesh(Mesh, Sequence, LocalTime) ||
      !Sequence) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] TestDrawArcFromMesh: Failed to resolve sequence/time."));
    return;
  }

  const float PlayLen = Sequence->GetPlayLength();
  const float StartTime = FMath::Clamp(LocalTime, 0.f, PlayLen);
  const float EndTime = FMath::Clamp(LocalTime + 0.5f, StartTime, PlayLen);

  TArray<FVector> ArcPoints;
  const bool bSuccess = GenerateStrikeArcFromAnimSequence(
      Sequence, Mesh, BoneName, StartTime, EndTime, ArcPoints);

  if (!bSuccess || ArcPoints.Num() < 2) {
    UE_LOG(LogTemp, Warning,
           TEXT("[OH] TestDrawArcFromMesh: Arc gen failed or insufficient "
                "points."));
    return;
  }

  DrawDebugPath(Mesh, ArcPoints, Color, Duration, 2.0f, false, true);
}

void UOHCombatUtils::QuickDrawBoneArcFromMesh(USkeletalMeshComponent *Mesh,
                                              FName BoneName,
                                              float PredictDuration,
                                              FColor Color, float Duration,
                                              float Thickness) {
  if (!Mesh || BoneName.IsNone()) {
    UE_LOG(LogTemp, Error, TEXT("[OH] Mesh or BoneName invalid"));
    return;
  }

  UWorld *World = Mesh->GetWorld();
  if (!World) {
    UE_LOG(LogTemp, Error, TEXT("[OH] World is null"));
    return;
  }

  UAnimInstance *AnimInst = Mesh->GetAnimInstance();
  if (!AnimInst) {
    UE_LOG(LogTemp, Error, TEXT("[OH] AnimInstance is null"));
    return;
  }

  UAnimMontage *Montage = AnimInst->GetCurrentActiveMontage();
  if (!Montage) {
    UE_LOG(LogTemp, Error, TEXT("[OH] No active montage"));
    return;
  }

  float MontageTime = AnimInst->Montage_GetPosition(Montage);
  UAnimSequence *Sequence = nullptr;
  float LocalTime = 0.f;

  if (!ResolveAnimSequenceAtTimeRecursive(Montage, MontageTime, Sequence,
                                          LocalTime)) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] Could not resolve sequence from montage at %.2f"),
           MontageTime);
    return;
  }

  const float T0 = FMath::Clamp(LocalTime, 0.f, Sequence->GetPlayLength());
  const float T1 =
      FMath::Clamp(T0 + PredictDuration, T0 + 0.01f, Sequence->GetPlayLength());

  TArray<FVector> ArcPoints;
  bool bSuccess = GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName,
                                                    T0, T1, ArcPoints);

  if (!bSuccess || ArcPoints.Num() < 2) {
    UE_LOG(LogTemp, Error, TEXT("[OH] ArcPoints failed: %d pts"),
           ArcPoints.Num());
    for (int32 i = 0; i < ArcPoints.Num(); ++i)
      UE_LOG(LogTemp, Warning, TEXT("Point[%d] = %s"), i,
             *ArcPoints[i].ToString());
    return;
  }

  UE_LOG(LogTemp, Log,
         TEXT("[OH] Drawing arc from %.2f to %.2f with %d points"), T0, T1,
         ArcPoints.Num());

  for (const FVector &Pt : ArcPoints) {
    DrawDebugSphere(World, Pt, 6.f, 8, FColor::Yellow, true, Duration);
  }

  DrawDebugPath(Mesh, ArcPoints, Color, Duration, Thickness, false, true);
}

void UOHCombatUtils::ScheduleDeferredArcDraw(USkeletalMeshComponent *Mesh,
                                             FName BoneName,
                                             float PredictDuration,
                                             FColor DebugColor, float Duration,
                                             float Thickness) {
  if (!Mesh || BoneName.IsNone()) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] ScheduleDeferredArcDraw: Mesh or BoneName invalid"));
    return;
  }

  UWorld *World = Mesh->GetWorld();
  if (!World) {
    UE_LOG(LogTemp, Error,
           TEXT("[OH] ScheduleDeferredArcDraw: No valid world"));
    return;
  }

  FTimerHandle DummyTimer;
  World->GetTimerManager().SetTimer(
      DummyTimer,
      [=]() {
        UAnimInstance *AnimInst = Mesh->GetAnimInstance();
        if (!AnimInst) {
          UE_LOG(LogTemp, Error,
                 TEXT("[OH] No anim instance found for deferred arc"));
          return;
        }

        UAnimMontage *Montage = AnimInst->GetCurrentActiveMontage();
        if (!Montage) {
          UE_LOG(LogTemp, Error,
                 TEXT("[OH] No active montage during deferred arc"));
          return;
        }

        float MontageTime = AnimInst->Montage_GetPosition(Montage);
        UAnimSequence *Sequence = nullptr;
        float LocalTime = 0.f;

        if (!ResolveAnimSequenceAtTimeRecursive(Montage, MontageTime, Sequence,
                                                LocalTime) ||
            !Sequence) {
          UE_LOG(LogTemp, Error,
                 TEXT("[OH] Could not resolve sequence from montage"));
          return;
        }

        const float T0 =
            FMath::Clamp(LocalTime, 0.f, Sequence->GetPlayLength());
        const float T1 = FMath::Clamp(T0 + PredictDuration, T0 + 0.01f,
                                      Sequence->GetPlayLength());

        TArray<FVector> ArcPoints;
        if (!GenerateStrikeArcFromAnimSequence(Sequence, Mesh, BoneName, T0, T1,
                                               ArcPoints)) {
          UE_LOG(LogTemp, Warning, TEXT("[OH] Arc generation failed"));
          return;
        }

        if (ArcPoints.Num() >= 2) {
          DrawDebugPath(Mesh, ArcPoints, DebugColor, Duration, Thickness, false,
                        true);
          UE_LOG(LogTemp, Log, TEXT("[OH] Drew arc (%d points) for bone '%s'"),
                 ArcPoints.Num(), *BoneName.ToString());
        } else if (ArcPoints.Num() == 1) {
          DrawDebugSphere(World, ArcPoints[0], 10.f, 12, FColor::Red, true,
                          Duration);
          UE_LOG(LogTemp, Warning,
                 TEXT("[OH] Only one point in arc — drawing fallback sphere"));
        } else {
          UE_LOG(LogTemp, Error,
                 TEXT("[OH] Arc had no points — fallback draw"));
          DrawDebugBox(World, FVector(0, 0, 100), FVector(10, 10, 10),
                       FColor::Magenta, true, Duration);
        }
      },
      0.01f, false);
}

void UOHCombatUtils::GetBoneChainAngularMotion(
    UOHPhysicsManager *PhysicsManager, const TArray<FName> &BoneChain,
    FVector &OutAngularVelocity, FVector &OutAngularAcceleration) {
  OutAngularVelocity = FVector::ZeroVector;
  OutAngularAcceleration = FVector::ZeroVector;

  if (!PhysicsManager || BoneChain.Num() == 0)
    return;

  int32 Count = 0;

  for (const FName &BoneName : BoneChain) {
    const FOHBoneData *Data = PhysicsManager->GetBoneData(BoneName);
    if (!Data || !Data->IsValid())
      continue;

    OutAngularVelocity += Data->GetBodyAngularVelocity();
    OutAngularAcceleration += Data->GetAngularAcceleration();
    ++Count;
  }

  if (Count > 0) {
    OutAngularVelocity /= Count;
    OutAngularAcceleration /= Count;
  }
}

FVector UOHCombatUtils::GetBlendedChainVelocity(
    UOHPhysicsManager *PhysicsManager, const USkeletalMeshComponent *Mesh,
    const FName StartBone, const FName EndBone, float HistoryWeight,
    int NumSamples) {
  if (!PhysicsManager || !Mesh || EndBone.IsNone()) {
    return FVector::ZeroVector;
  }

  TArray<FName> BoneChain =
      StartBone.IsNone()
          ? UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, NAME_None,
                                                               EndBone)
          : UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, StartBone,
                                                               EndBone);

  if (BoneChain.Num() == 0) {
    BoneChain.Add(EndBone);
  }

  FVector Accumulated = FVector::ZeroVector;
  int32 ValidCount = 0;

  for (const FName &Bone : BoneChain) {
    const FOHBoneData *Data = PhysicsManager->GetBoneData(Bone);
    if (!Data || !Data->IsValid())
      continue;

    Accumulated += Data->GetBlendedLinearVelocity(HistoryWeight, NumSamples);
    ++ValidCount;
  }

  return (ValidCount > 0) ? Accumulated / ValidCount : FVector::ZeroVector;
}

FVector UOHCombatUtils::GetBlendedChainAcceleration(
    UOHPhysicsManager *PhysicsManager, const USkeletalMeshComponent *Mesh,
    const FName StartBone, const FName EndBone, float HistoryWeight,
    int NumSamples) {
  if (!PhysicsManager || !Mesh || EndBone.IsNone()) {
    return FVector::ZeroVector;
  }

  TArray<FName> BoneChain =
      StartBone.IsNone()
          ? UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, NAME_None,
                                                               EndBone)
          : UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, StartBone,
                                                               EndBone);

  if (BoneChain.Num() == 0) {
    BoneChain.Add(EndBone);
  }

  FVector Accumulated = FVector::ZeroVector;
  int32 ValidCount = 0;

  for (const FName &Bone : BoneChain) {
    const FOHBoneData *Data = PhysicsManager->GetBoneData(Bone);
    if (!Data || !Data->IsValid())
      continue;

    Accumulated +=
        Data->GetBlendedLinearAcceleration(HistoryWeight, NumSamples);
    ++ValidCount;
  }

  return (ValidCount > 0) ? Accumulated / ValidCount : FVector::ZeroVector;
}

void UOHCombatUtils::GetBlendedBoneChainMotion(
    UOHPhysicsManager *PhysicsManager, const TArray<FName> &BoneChain,
    float HistoryWeight, int NumSamples, FVector &OutVelocity,
    FVector &OutAcceleration, FVector &OutStartLocation, FVector &OutForward) {
  OutVelocity = FVector::ZeroVector;
  OutAcceleration = FVector::ZeroVector;
  OutStartLocation = FVector::ZeroVector;
  OutForward = FVector::ZeroVector;

  if (!PhysicsManager || BoneChain.Num() == 0)
    return;

  int32 ValidCount = 0;

  for (const FName &BoneName : BoneChain) {
    const FOHBoneData *BoneData = PhysicsManager->GetBoneData(BoneName);
    if (!BoneData || !BoneData->IsValid())
      continue;

    OutVelocity +=
        BoneData->GetBlendedLinearVelocity(HistoryWeight, NumSamples);
    OutAcceleration +=
        BoneData->GetBlendedLinearAcceleration(HistoryWeight, NumSamples);
    OutStartLocation += BoneData->GetCurrentPosition();
    OutForward += BoneData->GetCurrentRotation().Vector();
    ++ValidCount;
  }

  if (ValidCount > 0) {
    OutVelocity /= ValidCount;
    OutAcceleration /= ValidCount;
    OutStartLocation /= ValidCount;
    OutForward = OutForward.GetSafeNormal();
  }
}

void UOHCombatUtils::GetBlendedBoneChainMotionData(
    UOHPhysicsManager *PhysicsManager, const TArray<FName> &BoneChain,
    float HistoryWeight, int NumSamples, FVector &OutVelocity,
    FVector &OutAcceleration, FVector &OutStartLocation, FVector &OutForward,
    FVector &OutAngularVelocity, FVector &OutAngularAcceleration,
    float &OutJitterScore) {
  OutVelocity = FVector::ZeroVector;
  OutAcceleration = FVector::ZeroVector;
  OutStartLocation = FVector::ZeroVector;
  OutForward = FVector::ZeroVector;
  OutAngularVelocity = FVector::ZeroVector;
  OutAngularAcceleration = FVector::ZeroVector;
  OutJitterScore = 0.f;

  if (!PhysicsManager || BoneChain.Num() == 0)
    return;

  int32 ValidCount = 0;

  for (const FName &BoneName : BoneChain) {
    const FOHBoneData *BoneData = PhysicsManager->GetBoneData(BoneName);
    if (!BoneData || !BoneData->IsValid())
      continue;

    OutVelocity +=
        BoneData->GetBlendedLinearVelocity(HistoryWeight, NumSamples);
    OutAcceleration +=
        BoneData->GetBlendedLinearAcceleration(HistoryWeight, NumSamples);
    OutAngularVelocity += BoneData->GetBodyAngularVelocity();
    OutAngularAcceleration += BoneData->GetAngularAcceleration();
    OutStartLocation += BoneData->GetCurrentPosition();
    OutForward += BoneData->GetCurrentRotation().Vector();
    OutJitterScore += BoneData->GetTrajectoryShiftScore();

    ++ValidCount;
  }

  if (ValidCount > 0) {
    OutVelocity /= ValidCount;
    OutAcceleration /= ValidCount;
    OutStartLocation /= ValidCount;
    OutForward = OutForward.GetSafeNormal();
    OutAngularVelocity /= ValidCount;
    OutAngularAcceleration /= ValidCount;
    OutJitterScore /= ValidCount;
  }
}

void UOHCombatUtils::GetBlendedBoneChainMotionFromStrikeBone(
    UOHPhysicsManager *PhysicsManager, const USkeletalMeshComponent *Mesh,
    FName StrikeBone, FName ChainStart, int32 MaxDepth,
    bool bIsFullBodyAnimation, float HistoryWeight, int NumSamples,
    FVector &OutVelocity, FVector &OutAcceleration, FVector &OutStartLocation,
    FVector &OutForward, FVector &OutAngularVelocity,
    FVector &OutAngularAcceleration, float &OutJitterScore) {
  OutVelocity = FVector::ZeroVector;
  OutAcceleration = FVector::ZeroVector;
  OutStartLocation = FVector::ZeroVector;
  OutForward = FVector::ZeroVector;
  OutAngularVelocity = FVector::ZeroVector;
  OutAngularAcceleration = FVector::ZeroVector;
  OutJitterScore = 0.f;

  if (!PhysicsManager || !Mesh || StrikeBone.IsNone())
    return;

  // --- 1. Resolve bone chain ---
  TArray<FName> BoneChain =
      ChainStart.IsNone()
          ? UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, NAME_None,
                                                               StrikeBone)
          : UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(Mesh, ChainStart,
                                                               StrikeBone);

  if (MaxDepth > 0 && BoneChain.Num() > MaxDepth) {
    TArray<FName> TruncatedChain;
    for (int32 i = BoneChain.Num() - MaxDepth; i < BoneChain.Num(); ++i) {
      TruncatedChain.Add(BoneChain[i]);
    }
    BoneChain = TruncatedChain;
  }

  if (BoneChain.Num() == 0) {
    BoneChain.Add(StrikeBone);
  }

  // --- 2. Aggregate data from valid bone entries ---
  int32 ValidCount = 0;

  for (const FName &BoneName : BoneChain) {
    if (!bIsFullBodyAnimation &&
        (BoneName == "pelvis" || BoneName.ToString().Contains(TEXT("root"))))
      continue;

    const FOHBoneData *Data = PhysicsManager->GetBoneData(BoneName);
    if (!Data || !Data->IsValid())
      continue;

    OutVelocity += Data->GetBlendedLinearVelocity(HistoryWeight, NumSamples);
    OutAcceleration +=
        Data->GetBlendedLinearAcceleration(HistoryWeight, NumSamples);
    OutAngularVelocity += Data->GetBodyAngularVelocity();
    OutAngularAcceleration += Data->GetAngularAcceleration();
    OutStartLocation += Data->GetCurrentPosition();
    OutForward += Data->GetCurrentRotation().Vector();
    OutJitterScore += Data->GetTrajectoryShiftScore();

    ++ValidCount;
  }

  if (ValidCount > 0) {
    OutVelocity /= ValidCount;
    OutAcceleration /= ValidCount;
    OutAngularVelocity /= ValidCount;
    OutAngularAcceleration /= ValidCount;
    OutStartLocation /= ValidCount;
    OutForward = OutForward.GetSafeNormal();
    OutJitterScore /= ValidCount;
  }
}

#pragma endregion

#pragma region MotionWarping

void UOHCombatUtils::AddWarpTargetToNearestEnemy(
    UMotionWarpingComponent *MotionWarpingComponent, float SearchRadius,
    FName WarpTargetName) {
  if (!MotionWarpingComponent) {
    UE_LOG(LogTemp, Warning, TEXT("MotionWarpingComponent is null."));
    return;
  }

  AActor *Owner = MotionWarpingComponent->GetOwner();
  if (!Owner) {
    UE_LOG(LogTemp, Warning, TEXT("MotionWarpingComponent has no owner."));
    return;
  }

  UWorld *World = Owner->GetWorld();
  if (!World)
    return;

  FVector OwnerLocation = Owner->GetActorLocation();

  ACharacter *ClosestTarget = nullptr;
  float ClosestDistanceSqr = SearchRadius * SearchRadius;

  for (TActorIterator<ACharacter> It(World); It; ++It) {
    ACharacter *Candidate = *It;
    if (!Candidate || Candidate == Owner)
      continue;

    float DistSqr =
        FVector::DistSquared(OwnerLocation, Candidate->GetActorLocation());
    if (DistSqr < ClosestDistanceSqr) {
      ClosestDistanceSqr = DistSqr;
      ClosestTarget = Candidate;
    }
  }

  if (ClosestTarget) {
    FVector TargetLocation = ClosestTarget->GetActorLocation();

    // Set warp target
    MotionWarpingComponent->AddOrUpdateWarpTargetFromLocation(WarpTargetName,
                                                              TargetLocation);

    // Debug capsule around the target
    if (UCapsuleComponent *Capsule = ClosestTarget->GetCapsuleComponent()) {
      DrawDebugCapsule(World, TargetLocation,
                       Capsule->GetScaledCapsuleHalfHeight(),
                       Capsule->GetScaledCapsuleRadius(), FQuat::Identity,
                       FColor::Red, false, 2.0f, 0, 2.0f);
    }

    // Debug arrow from source to target

    DrawDebugDirectionalArrow(World, OwnerLocation, TargetLocation,
                              150.0f, // Arrow size
                              FColor::Blue, false, 2.0f, 0, 3.0f);
  }
}

float UOHCombatUtils::ComputeImpulseVulnerabilityScore(
    const FOHPhysicsGraphNode &Graph, FName BoneName,
    const FVector &ImpactNormal, const FVector &RootLocation) {
  const FOHBoneData *Bone = Graph.FindBoneData(BoneName);
  if (!Bone || !Bone->IsValid())
    return 0.f;

  const FVector BoneDirection =
      (Bone->GetCurrentPosition() - RootLocation).GetSafeNormal();
  const float Dot = FVector::DotProduct(BoneDirection, ImpactNormal);
  return FMath::Clamp((1.f + Dot) * 0.5f, 0.f,
                      1.f); // Simple vulnerability heuristic
}

float UOHCombatUtils::ComputeImpulseVulnerabilityScore(
    const FOHBoneData &BoneData, const FVector &ImpactDirection,
    const FVector &RootLocation) {
  if (!BoneData.IsValid()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[CombatUtils] Invalid bone data in vulnerability score "
                "computation."));
    return 0.0f;
  }

  const FVector BoneVector =
      (BoneData.GetCurrentPosition() - RootLocation).GetSafeNormal();
  const float Dot = FVector::DotProduct(BoneVector, ImpactDirection);
  const float Score =
      FMath::Clamp((1.0f + Dot) * 0.5f, 0.0f, 1.0f); // -1..1 to 0..1

#if WITH_EDITOR
  UE_LOG(LogTemp, Verbose,
         TEXT("[CombatUtils] Vulnerability Score: Dot = %.2f → Score = %.2f"),
         Dot, Score);
#endif

  return Score;
}

FVector UOHCombatUtils::ComputeImpulseVectorForBone(
    const FOHPhysicsGraphNode &Graph, FName BoneName, const FHitResult &Hit,
    const FVector &RootLocation, float BaseStrength,
    EOHImpulseMode ModeOverride, bool bAutoInferMode,
    EImpulseDirectionMode DirectionMode) {
  const FOHBoneData *Bone = Graph.FindBoneData(BoneName);
  if (!Bone || !Bone->IsValid()) {
    UE_LOG(LogTemp, Warning,
           TEXT("ComputeImpulseVectorForBone: Invalid bone %s"),
           *BoneName.ToString());
    return FVector::ZeroVector;
  }

  FVector ImpulseDirection;

  // === Impulse direction logic ===
  switch (DirectionMode) {
  case EImpulseDirectionMode::FromHitNormal:
    ImpulseDirection = Hit.ImpactNormal;
    break;

  case EImpulseDirectionMode::FromBoneToImpactPoint:
    ImpulseDirection =
        (Hit.ImpactPoint - Bone->GetCurrentPosition()).GetSafeNormal();
    break;

  case EImpulseDirectionMode::FromBoneVelocity:
    ImpulseDirection = Bone->GetBodyLinearVelocity().GetSafeNormal();
    break;

  default:
    ImpulseDirection = Hit.ImpactNormal;
    break;
  }

  // === Impulse strength scaling ===
  float Multiplier = ComputeImpulseVulnerabilityScore(
      Graph, BoneName, ImpulseDirection, RootLocation);
  float Strength = BaseStrength * Multiplier;

  // === Impulse application logic ===
  switch (ModeOverride) {
  case EOHImpulseMode::Directional:
    return ImpulseDirection * Strength;

  case EOHImpulseMode::Radial:
    return (Bone->GetCurrentPosition() - RootLocation).GetSafeNormal() *
           Strength;

  case EOHImpulseMode::VerticalOnly:
    return FVector::UpVector * Strength;

  case EOHImpulseMode::Auto:
    if (bAutoInferMode) {
      const bool bIsLeaf = Graph.GetChildrenOfBone(BoneName).Num() == 0;
      return bIsLeaf ? ImpulseDirection * Strength
                     : FVector::UpVector * Strength;
    }
    return ImpulseDirection * Strength;

  default:
    return ImpulseDirection * Strength;
  }
}

#pragma endregion

#pragma region ForceFeedback

FVector
UOHCombatUtils::GetImpulseDirectionFromHitNormal(const FHitResult &Hit) {
  return Hit.bBlockingHit ? -Hit.ImpactNormal.GetSafeNormal()
                          : FVector::ZeroVector;
}

FVector UOHCombatUtils::GetImpulseDirectionFromBoneToImpactPoint(
    const FOHBoneData &BoneData, const FHitResult &Hit) {
  if (!BoneData.IsValid() || !Hit.bBlockingHit)
    return FVector::ZeroVector;

  return (Hit.ImpactPoint - BoneData.GetCurrentPosition()).GetSafeNormal();
}

FVector UOHCombatUtils::GetImpulseDirectionFromHit(const FOHBoneData &BoneData,
                                                   const FHitResult &Hit,
                                                   EImpulseDirectionMode Mode) {
  return (Mode == EImpulseDirectionMode::FromBoneToImpactPoint)
             ? GetImpulseDirectionFromBoneToImpactPoint(BoneData, Hit)
             : GetImpulseDirectionFromHitNormal(Hit);
}

float UOHCombatUtils::ComputeImpulseStrengthFromNormal(
    const FOHBoneData &BoneData, const FHitResult &Hit, float BaseMultiplier) {
  const FVector Dir = GetImpulseDirectionFromHitNormal(Hit);
  return ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseMultiplier);
}

float UOHCombatUtils::ComputeImpulseStrengthFromImpactVector(
    const FOHBoneData &BoneData, const FHitResult &Hit, float BaseMultiplier) {
  const FVector Dir = GetImpulseDirectionFromBoneToImpactPoint(BoneData, Hit);
  return ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseMultiplier);
}

float UOHCombatUtils::ComputeImpulseStrengthFromBoneData(
    const FOHBoneData &BoneData, const FVector &OptionalStrikeDir,
    float BaseMultiplier) {
  if (!BoneData.IsValid() || !BoneData.HasMotionHistory()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[CombatUtils] Invalid or missing bone data — impulse strength "
                "defaulted to 0"));
    return 0.f;
  }

  const FOHMotionSample Sample = BoneData.GetLatestMotionSample();
  if (!Sample.IsValidSample()) {
    UE_LOG(LogTemp, Warning,
           TEXT("[CombatUtils] No valid motion sample in bone data"));
    return 0.f;
  }

  // Extract motion metrics
  const float LinearSpeed = Sample.GetLinearSpeed();
  const float AngularSpeed = Sample.GetAngularSpeed();
  const float LinearAccelMag = Sample.GetLinearAccelMagnitude();
  const float AngularAccelMag = Sample.GetAngularAccelMagnitude();

  // Optional directionality influence
  float DirectionScore = 1.f;
  if (!OptionalStrikeDir.IsNearlyZero()) {
    const FVector BoneVelocity = Sample.GetLinearVelocity();
    const float Dot = FVector::DotProduct(BoneVelocity.GetSafeNormal(),
                                          OptionalStrikeDir.GetSafeNormal());
    DirectionScore =
        FMath::Clamp(Dot, 0.f, 1.f); // Emphasize forward-aligned strikes
  }

  // Weighted scoring formula
  const float WeightedScore = (LinearSpeed * 0.5f) + (AngularSpeed * 0.2f) +
                              (LinearAccelMag * 0.2f) +
                              (AngularAccelMag * 0.1f);

  const float FinalStrength = WeightedScore * DirectionScore * BaseMultiplier;

#if WITH_EDITOR
  UE_LOG(LogTemp, Log,
         TEXT("[CombatUtils] ImpulseStrength: %.2f (Linear: %.2f, Angular: "
              "%.2f, Accel: %.2f, DirScore: %.2f)"),
         FinalStrength, LinearSpeed, AngularSpeed, LinearAccelMag,
         DirectionScore);
#endif

  return FinalStrength;
}

float UOHCombatUtils::ComputeImpulseStrengthFromHit(
    const FOHBoneData &BoneData, const FHitResult &Hit,
    const USkeletalMeshComponent *SkeletalMesh, float BaseMultiplier) {
  if (!BoneData.IsValid() || !Hit.bBlockingHit || !IsValid(SkeletalMesh)) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("[CombatUtils] Invalid input to ComputeImpulseStrengthFromHit"));
    return 0.0f;
  }

  // Anchor point for leverage: assume "pelvis"
  static const FName PelvisBone = FName("pelvis");
  const FVector RootLocation =
      SkeletalMesh->GetBoneLocation(PelvisBone, EBoneSpaces::WorldSpace);

  const FVector ImpactDir = Hit.ImpactNormal;
  const float VulnerabilityScore =
      ComputeImpulseVulnerabilityScore(BoneData, ImpactDir, RootLocation);

  const float FinalStrength = BaseMultiplier * VulnerabilityScore;

#if WITH_EDITOR
  UE_LOG(LogTemp, Log,
         TEXT("[CombatUtils] ImpulseStrength: Base = %.2f | Multiplier = %.2f "
              "| Final = %.2f"),
         BaseMultiplier, VulnerabilityScore, FinalStrength);
#endif

  return FinalStrength;
}

FVector UOHCombatUtils::ComputeImpulseVectorFromHit(
    const FOHBoneData &BoneData, const FHitResult &Hit, float BaseMultiplier,
    EImpulseDirectionMode Mode) {
  const FVector Dir = GetImpulseDirectionFromHit(BoneData, Hit, Mode);
  const float Strength =
      ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseMultiplier);
  const FVector FinalImpulse = Dir * Strength;

#if WITH_EDITOR
  UE_LOG(LogTemp, Log,
         TEXT("[CombatUtils] Impulse Vector (%s): Strength = %.2f | Direction "
              "= %s | Result = %s"),
         *UEnum::GetValueAsString(Mode), Strength, *Dir.ToString(),
         *FinalImpulse.ToString());
#endif

  return FinalImpulse;
}

FVector UOHCombatUtils::ComputeImpulseVectorFromHitNormal(
    const FOHBoneData &BoneData, const FHitResult &Hit, float BaseMultiplier) {
  const FVector Dir = GetImpulseDirectionFromHitNormal(Hit);
  const float Strength =
      ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseMultiplier);
  return Dir * Strength;
}

FVector UOHCombatUtils::ComputeImpulseVectorFromImpactVector(
    const FOHBoneData &BoneData, const FHitResult &Hit, float BaseMultiplier) {
  const FVector Dir = GetImpulseDirectionFromBoneToImpactPoint(BoneData, Hit);
  const float Strength =
      ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseMultiplier);
  return Dir * Strength;
}

EImpulseDirectionMode
UOHCombatUtils::InferBestImpulseDirectionMode(const FOHBoneData &BoneData,
                                              const FHitResult &Hit,
                                              float AlignmentThreshold) {
  if (!BoneData.IsValid() || !Hit.bBlockingHit)
    return EImpulseDirectionMode::FromHitNormal;

  const FVector BoneVelocity = BoneData.GetBodyLinearVelocity().GetSafeNormal();
  const FVector ToImpactPoint =
      (Hit.ImpactPoint - BoneData.GetCurrentPosition()).GetSafeNormal();

  const float Dot = FVector::DotProduct(BoneVelocity, ToImpactPoint);
  const bool bIsAligned = Dot > AlignmentThreshold;

#if WITH_EDITOR
  UE_LOG(LogTemp, Log,
         TEXT("[CombatUtils] Impulse Direction Auto-Detect: Dot = %.2f (%s)"),
         Dot, bIsAligned ? TEXT("BoneToImpact") : TEXT("HitNormal"));
#endif

  return bIsAligned ? EImpulseDirectionMode::FromBoneToImpactPoint
                    : EImpulseDirectionMode::FromHitNormal;
}

FVector UOHCombatUtils::ComputeSelfReactionImpulse(const FOHBoneData &BoneData,
                                                   const FHitResult &Hit,
                                                   const FVector &RootLocation,
                                                   float BaseStrength,
                                                   float Mass) {
  if (!BoneData.IsValid() || !Hit.bBlockingHit) {
    return FVector::ZeroVector;
  }

  const FVector BonePos = BoneData.GetCurrentPosition();
  const FVector BoneVel = BoneData.GetBodyLinearVelocity();
  const FVector AngularVel = BoneData.GetBodyAngularVelocity();

  // 1. Direction: Away from impact point
  const FVector Direction = -Hit.ImpactNormal.GetSafeNormal();

  // 2. Motion Amplification
  const float VelocityAlignment =
      FVector::DotProduct(BoneVel.GetSafeNormal(), -Hit.ImpactNormal);
  const float AngularMag = AngularVel.Size();
  const float DistanceFromRoot = FVector::Distance(RootLocation, BonePos);

  // 3. Normalize metrics
  const float VelocityFactor = FMath::Clamp(VelocityAlignment, 0.f, 1.f);
  const float AngularFactor =
      FMath::Clamp(AngularMag / 300.f, 0.f, 1.f); // Tunable
  const float LeverageFactor =
      FMath::Clamp(DistanceFromRoot / 100.f, 0.f, 2.f); // Tunable

  // 4. Composite multiplier
  const float Combined = (VelocityFactor * 0.4f) + (AngularFactor * 0.3f) +
                         (LeverageFactor * 0.3f);
  const float FinalStrength =
      BaseStrength * Mass * FMath::Clamp(Combined, 0.2f, 2.0f);

  // 5. Final impulse
  const FVector Impulse = Direction * FinalStrength;

#if WITH_EDITOR
  UE_LOG(LogTemp, Log,
         TEXT("[SelfImpulse] VelDot=%.2f | Ang=%.2f | Dist=%.2f | Final=%.2f"),
         VelocityFactor, AngularFactor, LeverageFactor, FinalStrength);
#endif

  return Impulse;
}

FVector UOHCombatUtils::ComputeImpulseVectorFromBone(
    const FOHBoneData &BoneData, const FHitResult &Hit,
    const FVector &RootLocation, float BaseStrength,
    EImpulseDirectionMode DirectionMode, bool bUseAutoInfer) {
  if (!BoneData.IsValid() || !Hit.bBlockingHit)
    return FVector::ZeroVector;

  const FVector Dir =
      (bUseAutoInfer)
          ? GetImpulseDirectionFromHit(
                BoneData, Hit, InferBestImpulseDirectionMode(BoneData, Hit))
          : GetImpulseDirectionFromHit(BoneData, Hit, DirectionMode);

  const float Strength =
      ComputeImpulseStrengthFromBoneData(BoneData, Dir, BaseStrength);
  return Dir * Strength;
}

#pragma endregion

AActor *UOHCombatUtils::FindFuzzyTarget(const AActor *SelfActor,
                                        const FString &ContextualHint) {
  if (!SelfActor) {
    UE_LOG(LogTemp, Warning,
           TEXT("[FindMostLikelyOpponent] SelfActor is null!"));
    return nullptr;
  }

  UWorld *World = SelfActor->GetWorld();
  if (!World) {
    UE_LOG(
        LogTemp, Warning,
        TEXT(
            "[FindMostLikelyOpponent] Could not get world from SelfActor [%s]"),
        *SelfActor->GetName());
    return nullptr;
  }

  // --- Collect all valid candidate actors ---
  TArray<AActor *> Candidates;
  for (TActorIterator<AActor> It(World); It; ++It) {
    AActor *Candidate = *It;
    if (!Candidate)
      continue;
    if (Candidate == SelfActor)
      continue; // Don't target self
    if (!IsValid(Candidate))
      continue; // Dead or being GC'd

    // Exclude destroyed or pending kill
    if (!IsValid(Candidate) || Candidate->IsActorBeingDestroyed()) {
      continue;
    }

    Candidates.Add(Candidate);
  }

  if (Candidates.Num() == 0) {
    UE_LOG(
        LogTemp, Warning,
        TEXT("[FindMostLikelyOpponent] No candidates found in world for [%s]"),
        *SelfActor->GetName());
    return nullptr;
  }

  // --- Score each candidate using a weighted heuristic ---
  TMap<AActor *, float> CandidateScores;

  const FVector SelfLoc = SelfActor->GetActorLocation();
  const FRotator SelfRot = SelfActor->GetActorRotation();
  const FVector SelfForward = SelfRot.Vector();

  for (AActor *Candidate : Candidates) {
    float Score = 0.0f;

    // 1 Distance: closer = better (inverse, clamp)
    const float Dist = FVector::Dist(SelfLoc, Candidate->GetActorLocation());
    Score += 1000.f / (Dist + 1.f);

    // 2 Field of View: favor in-front actors
    const FVector ToTarget =
        (Candidate->GetActorLocation() - SelfLoc).GetSafeNormal();
    const float Dot = FVector::DotProduct(SelfForward, ToTarget);
    Score += 100.f * FMath::Clamp(Dot, 0.f, 1.f);

    // 3 Name/Hint fuzzy logic (if ContextualHint is supplied)
    if (!ContextualHint.IsEmpty()) {
      const FString TargetName = Candidate->GetName().ToLower();
      float FuzzyScore = UOHAlgoUtils::JaroWinklerDistance(
          TargetName, ContextualHint.ToLower());
      Score += 50.f * FuzzyScore;
    }

    // 4️⃣ Prefer "character" types (add a bonus)
    if (Candidate->IsA(APawn::StaticClass())) {
      Score += 10.f;
    }
    if (Candidate->GetClass()->GetName().Contains(TEXT("CombatCharacter"))) {
      Score += 15.f;
    }

    // 5️⃣ [Future] Last-damaged-by or aggro could go here

    CandidateScores.Add(Candidate, Score);
  }

  // --- Use OHSafeMapUtils if you want to process scores more
  // defensively/efficiently --- Example: Get candidate with max score (safe,
  // future-proof, easy to extend)
  AActor *BestCandidate = nullptr;
  float BestScore = -FLT_MAX;
  for (const auto &Pair : CandidateScores) {
    if (Pair.Value > BestScore) {
      BestCandidate = Pair.Key;
      BestScore = Pair.Value;
    }
  }

  if (!BestCandidate) {
    UE_LOG(LogTemp, Warning,
           TEXT("[FindMostLikelyOpponent] No valid opponent found for [%s]"),
           *SelfActor->GetName());
  }

  return BestCandidate;
}

bool UOHCombatUtils::PredictiveCapsuleGroundTrace(
    USkeletalMeshComponent *Mesh, FName BoneName, float MaxDistance,
    float TraceForwardBias, float CapsuleRadius, float CapsuleHalfHeight,
    const FVector &OwnerVelocity, FHitResult &OutHit, bool bDrawDebug) {
  if (!Mesh)
    return false;

  FVector Start = Mesh->GetBoneLocation(BoneName);
  FVector VelocityDir = OwnerVelocity.GetSafeNormal();
  // Predict slightly forward in the movement direction
  FVector PredictedStart = Start + (VelocityDir * TraceForwardBias);

  FVector End = PredictedStart - FVector(0, 0, MaxDistance);

  FCollisionQueryParams Params;
  Params.AddIgnoredActor(Mesh->GetOwner());

  bool bHit = Mesh->GetWorld()->SweepSingleByChannel(
      OutHit, PredictedStart, End, FQuat::Identity, ECC_Visibility,
      FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight), Params);

  if (bDrawDebug) {
    FColor TraceColor = bHit ? FColor::Green : FColor::Red;
    DrawDebugCapsule(Mesh->GetWorld(), PredictedStart, CapsuleHalfHeight,
                     CapsuleRadius, FQuat::Identity, TraceColor, false, 0.05f,
                     0, 1.0f);
    DrawDebugLine(Mesh->GetWorld(), PredictedStart, End, TraceColor, false,
                  0.05f, 0, 1.0f);
    if (bHit) {
      DrawDebugPoint(Mesh->GetWorld(), OutHit.ImpactPoint, 12.f, FColor::Blue,
                     false, 0.1f);
      DrawDebugDirectionalArrow(Mesh->GetWorld(), OutHit.ImpactPoint,
                                OutHit.ImpactPoint + OutHit.ImpactNormal * 30.f,
                                16.f, FColor::Yellow, false, 0.1f, 0, 1.5f);
    }
  }
  return bHit;
}

AActor *UOHCombatUtils::FindMostLikelyOpponent(const AActor *SelfActor,
                                               const FString &ContextualHint) {
  if (!SelfActor)
    return nullptr;
  UWorld *World = SelfActor->GetWorld();
  if (!World)
    return nullptr;

  TArray<AActor *> Candidates;
  for (TActorIterator<AActor> It(World); It; ++It) {
    AActor *Candidate = *It;
    if (!Candidate)
      continue;
    if (Candidate == SelfActor)
      continue;
    if (!IsValid(Candidate))
      continue;

    // --- Skip hitboxes/child actors if their parent/owner is a Pawn/Character
    // (common in fighting games) ---
    if (Candidate->GetOwner() &&
        Candidate->GetOwner()->IsA(APawn::StaticClass()))
      continue;
    if (Candidate->GetAttachParentActor() &&
        Candidate->GetAttachParentActor()->IsA(APawn::StaticClass()))
      continue;

    // --- Optional: Skip actors that aren't Pawns (or Characters, or your base
    // combat class) --- If you want to *only* ever target pawns/characters,
    // uncomment below: if (!Candidate->IsA(APawn::StaticClass())) continue;

    // Or: if you have a base class like AOHCombatCharacter
    // if (!Candidate->IsA(AOHCombatCharacter::StaticClass())) continue;

    Candidates.Add(Candidate);
  }

  if (Candidates.Num() == 0)
    return nullptr;

  // --- Score each candidate ---
  TMap<AActor *, float> CandidateScores;
  const FVector SelfLoc = SelfActor->GetActorLocation();
  const FRotator SelfRot = SelfActor->GetActorRotation();
  const FVector SelfForward = SelfRot.Vector();

  for (AActor *Candidate : Candidates) {
    float Score = 0.0f;

    // Distance (closer = better)
    const float Dist = FVector::Dist(SelfLoc, Candidate->GetActorLocation());
    Score += 1000.f / (Dist + 1.f);

    // Field of view
    const FVector ToTarget =
        (Candidate->GetActorLocation() - SelfLoc).GetSafeNormal();
    const float Dot = FVector::DotProduct(SelfForward, ToTarget);
    Score += 100.f * FMath::Clamp(Dot, 0.f, 1.f);

    // Name/Hint fuzzy logic (optional)
    if (!ContextualHint.IsEmpty()) {
      const FString TargetName = Candidate->GetName().ToLower();
      float FuzzyScore = UOHAlgoUtils::JaroWinklerDistance(
          TargetName, ContextualHint.ToLower());
      Score += 50.f * FuzzyScore;
    }

    // Prefer pawns/characters even more!
    if (Candidate->IsA(APawn::StaticClass()))
      Score += 100.f; // Strong preference
    if (Candidate->IsA(AOHCombatCharacter::StaticClass()))
      Score += 200.f; // Top preference

    CandidateScores.Add(Candidate, Score);
  }

  // --- Pick the best ---
  AActor *BestCandidate = nullptr;
  float BestScore = -FLT_MAX;
  for (const auto &Pair : CandidateScores) {
    if (Pair.Value > BestScore) {
      BestCandidate = Pair.Key;
      BestScore = Pair.Value;
    }
  }
  return BestCandidate;
}
