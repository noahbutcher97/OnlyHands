// Fill out your copyright notice in the Description page of Project Settings.
#include "FunctionLibrary/OHAnimationUtils.h"
#include "Component/OHPhysicsManager.h"

FTransform
UOHAnimationUtils::GetCurrentAnimPoseTransform(USkeletalMeshComponent *Mesh,
                                               FName BoneName) {
  if (!Mesh)
    return FTransform::Identity;

  const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
  if (BoneIndex == INDEX_NONE)
    return FTransform::Identity;

  // This returns bone transform in component space from evaluated animation
  // pose
  return Mesh->GetSocketTransform(BoneName, RTS_Component);
}

float UOHAnimationUtils::GetTransformError(UOHPhysicsManager *Manager,
                                           FName BoneName) {
  if (!Manager)
    return 0.f;

  const FOHBoneData *Data = Manager->GetBoneData(BoneName);
  if (!Data)
    return 0.f;

  USkeletalMeshComponent *Mesh = Manager->GetSkeletalMeshComponent();
  if (!Mesh)
    return 0.f;

  const FVector TrackedPos = Data->GetCurrentPosition();
  const FVector AnimPos =
      GetCurrentAnimPoseTransform(Mesh, BoneName).GetLocation();

  return FVector::Dist(AnimPos, TrackedPos);
}

float UOHAnimationUtils::GetRotationDriftError(UOHPhysicsManager *Manager,
                                               FName BoneName) {
  if (!Manager)
    return 0.f;

  const FOHBoneData *Data = Manager->GetBoneData(BoneName);
  if (!Data)
    return 0.f;

  USkeletalMeshComponent *Mesh = Manager->GetSkeletalMeshComponent();
  if (!Mesh)
    return 0.f;

  const FQuat TrackedRot = Data->GetCurrentRotation();
  const FQuat AnimRot =
      GetCurrentAnimPoseTransform(Mesh, BoneName).GetRotation();

  // Return angular difference in degrees
  return FMath::RadiansToDegrees(TrackedRot.AngularDistance(AnimRot));
}

void UOHAnimationUtils::DrawPoseDeviation(UOHPhysicsManager *Manager,
                                          FName BoneName, FLinearColor Color,
                                          float Thickness) {
  if (!Manager)
    return;

  const FOHBoneData *Data = Manager->GetBoneData(BoneName);
  if (!Data)
    return;

  USkeletalMeshComponent *Mesh = Manager->GetSkeletalMeshComponent();
  if (!Mesh)
    return;

  const FVector TrackedPos = Data->GetCurrentPosition();
  const FVector AnimPos =
      GetCurrentAnimPoseTransform(Mesh, BoneName).GetLocation();

  DrawDebugLine(Mesh->GetWorld(), AnimPos, TrackedPos, Color.ToFColor(true),
                false, // persistent lines
                0.05f, // duration
                0, Thickness);
}

TArray<FName>
UOHAnimationUtils::GetBonesWithHighDeviation(UOHPhysicsManager *Manager,
                                             float PositionThreshold,
                                             float RotationThreshold) {
  TArray<FName> Result;

  if (!Manager)
    return Result;

  const USkeletalMeshComponent *Mesh = Manager->GetSkeletalMeshComponent();
  if (!Mesh)
    return Result;

  for (const TPair<FName, FOHBoneData> &Pair : Manager->GetTrackedBoneData()) {
    const FName Bone = Pair.Key;
    const float PosError = GetTransformError(Manager, Bone);
    const float RotError = GetRotationDriftError(Manager, Bone);

    if (PosError > PositionThreshold || RotError > RotationThreshold) {
      Result.Add(Bone);
    }
  }

  return Result;
}

// Utility function to get current root motion mode from an anim instance
static ERootMotionMode::Type
GetAnimInstanceRootMotionMode(const UAnimInstance *AnimInstance) {
  if (!AnimInstance) {
    return ERootMotionMode::NoRootMotionExtraction;
  }
  return AnimInstance->RootMotionMode;
}