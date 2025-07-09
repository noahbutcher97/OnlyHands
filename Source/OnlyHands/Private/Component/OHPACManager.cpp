#include "Component/OHPACManager.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"

UOHPACManager::UOHPACManager() { PrimaryComponentTick.bCanEverTick = false; }

void UOHPACManager::BeginPlay() {
  Super::BeginPlay();
  Initialize();
}

void UOHPACManager::Initialize() {
  SkeletalMeshComponent =
      GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
  PhysicalAnimationComponent =
      GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();

  BonesInSkeleton.Empty();
  if (SkeletalMeshComponent && SkeletalMeshComponent->GetSkeletalMeshAsset()) {
    const FReferenceSkeleton &RefSkeleton =
        SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
    for (int32 i = 0; i < RefSkeleton.GetNum(); ++i) {
      BonesInSkeleton.Add(RefSkeleton.GetBoneName(i));
    }
  }

  CachePhysicsAssetData();
}

void UOHPACManager::CachePhysicsAssetData() {
  AllPhysicsBodies.Empty();
  AllConstraintNames.Empty();
  BodyNameToBodySetup.Empty();
  ConstraintNameToConstraintSetup.Empty();
  BoneNameToBodyInstance.Empty();
  BodyInstanceToBoneName.Empty();
  ConstraintNameToConstraintInstance.Empty();
  ConstraintInstanceToConstraintName.Empty();
  BoneNameToConstraintInstances.Empty();
  BodyInstanceToConstraintInstances.Empty();

  if (!SkeletalMeshComponent || !SkeletalMeshComponent->GetPhysicsAsset())
    return;

  UPhysicsAsset *PhysicsAsset = SkeletalMeshComponent->GetPhysicsAsset();

  // Gather editor-time bodies and constraints
  for (USkeletalBodySetup *BodySetup : PhysicsAsset->SkeletalBodySetups) {
    if (BodySetup) {
      FName BodyName = BodySetup->BoneName;
      AllPhysicsBodies.Add(BodyName);
      BodyNameToBodySetup.Add(BodyName, BodySetup);
    }
  }

  for (UPhysicsConstraintTemplate *ConstraintTemplate :
       PhysicsAsset->ConstraintSetup) {
    if (ConstraintTemplate) {
      FName ConstraintName = ConstraintTemplate->DefaultInstance.JointName;
      AllConstraintNames.Add(ConstraintName);
      ConstraintNameToConstraintSetup.Add(ConstraintName, ConstraintTemplate);
    }
  }

  // Gather runtime bodies (BodyInstances)
  for (FBodyInstance *BodyInstance : SkeletalMeshComponent->Bodies) {
    if (BodyInstance && BodyInstance->IsValidBodyInstance()) {
      FName BoneName = BodyInstance->BodySetup.IsValid()
                           ? BodyInstance->BodySetup->BoneName
                           : NAME_None;
      BoneNameToBodyInstance.Add(BoneName, BodyInstance);
      BodyInstanceToBoneName.Add(BodyInstance, BoneName);
    }
  }

  // Gather runtime constraints (ConstraintInstances)
  for (FConstraintInstance *ConstraintInstance :
       SkeletalMeshComponent->Constraints) {
    if (ConstraintInstance) {
      FName JointName = ConstraintInstance->JointName;
      ConstraintNameToConstraintInstance.Add(JointName, ConstraintInstance);
      ConstraintInstanceToConstraintName.Add(ConstraintInstance, JointName);

      // Attach constraint to both bones/bodies
      FName Bone1 = ConstraintInstance->ConstraintBone1;
      FName Bone2 = ConstraintInstance->ConstraintBone2;

      BoneNameToConstraintInstances.FindOrAdd(Bone1).Add(ConstraintInstance);
      BoneNameToConstraintInstances.FindOrAdd(Bone2).Add(ConstraintInstance);

      FBodyInstance *Body1 = BoneNameToBodyInstance.FindRef(Bone1);
      FBodyInstance *Body2 = BoneNameToBodyInstance.FindRef(Bone2);

      if (Body1)
        BodyInstanceToConstraintInstances.FindOrAdd(Body1).Add(
            ConstraintInstance);
      if (Body2)
        BodyInstanceToConstraintInstances.FindOrAdd(Body2).Add(
            ConstraintInstance);
    }
  }
}

void UOHPACManager::SmartUpdatePhysicsSnapshot() {
  // Smart: Only recache if mesh/asset/component has changed, otherwise update
  // live constraint/body instance pointers This could be extended to be even
  // smarter with property diff checks
  CachePhysicsAssetData();
}

void UOHPACManager::VisualizePhysicsSnapshot(bool bDrawBodies,
                                             bool bDrawConstraints,
                                             bool bDrawBoneNames,
                                             float Duration) const {
  if (!SkeletalMeshComponent)
    return;

  const float BodyRadius = 2.5f;
  const float ConstraintRadius = 5.0f;
  const FColor BodyColor = FColor::Green;
  const FColor ConstraintColor = FColor::Red;
  const FColor LineColor = FColor::Yellow;
  const FColor BoneNameColor = FColor::White;

  // Draw bodies
  if (bDrawBodies) {
    for (const TPair<FName, FBodyInstance *> &Pair : BoneNameToBodyInstance) {
      FName BoneName = Pair.Key;
      const FBodyInstance *BodyInstance = Pair.Value;
      FVector Location = SkeletalMeshComponent->GetBoneLocation(BoneName);
      DrawDebugSphere(GetWorld(), Location, BodyRadius, 8, BodyColor, false,
                      Duration);
      if (bDrawBoneNames) {
        DrawDebugString(GetWorld(), Location + FVector(0, 0, 10),
                        BoneName.ToString(), nullptr, BoneNameColor, Duration,
                        false);
      }
    }
  }

  // Draw constraints
  if (bDrawConstraints) {
    for (const TPair<FName, FConstraintInstance *> &Pair :
         ConstraintNameToConstraintInstance) {
      const FConstraintInstance *Constraint = Pair.Value;
      if (!Constraint)
        continue;

      FName Bone1 = Constraint->ConstraintBone1;
      FName Bone2 = Constraint->ConstraintBone2;
      FVector Pos1 = SkeletalMeshComponent->GetBoneLocation(Bone1);
      FVector Pos2 = SkeletalMeshComponent->GetBoneLocation(Bone2);

      // Draw line between constrained bones
      DrawDebugLine(GetWorld(), Pos1, Pos2, LineColor, false, Duration, 0,
                    1.5f);
      DrawDebugSphere(GetWorld(), (Pos1 + Pos2) * 0.5f, ConstraintRadius, 8,
                      ConstraintColor, false, Duration);
      if (bDrawBoneNames) {
        FString ConStr = FString::Printf(TEXT("%s-%s"), *Bone1.ToString(),
                                         *Bone2.ToString());
        DrawDebugString(GetWorld(), (Pos1 + Pos2) * 0.5f + FVector(0, 0, 10),
                        ConStr, nullptr, ConstraintColor, Duration, false);
      }
    }
  }
}
