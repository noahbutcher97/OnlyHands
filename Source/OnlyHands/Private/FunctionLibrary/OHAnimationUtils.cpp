// Fill out your copyright notice in the Description page of Project Settings.
#include "FunctionLibrary/OHAnimationUtils.h"
#include "Component/OHPhysicsManager.h"

bool UOHAnimationUtils::GetContiguousBoneChain(const USkeletalMeshComponent* SkeletalMeshComp, FName RootBone,
                                               TArray<FName>& OutChain) {
    OutChain.Empty();

    if (!SkeletalMeshComp || !SkeletalMeshComp->GetSkeletalMeshAsset())
        return false;

    const USkeletalMesh* Mesh = SkeletalMeshComp->GetSkeletalMeshAsset();
    const FReferenceSkeleton& RefSkeleton = Mesh->GetRefSkeleton();

    int32 CurrentIdx = RefSkeleton.FindBoneIndex(RootBone);
    if (CurrentIdx == INDEX_NONE)
        return false;

    OutChain.Add(RootBone);

    while (true) {
        // Find children of CurrentIdx
        TArray<int32> ChildIndices;
        for (int32 BoneIdx = 0; BoneIdx < RefSkeleton.GetNum(); ++BoneIdx) {
            if (RefSkeleton.GetParentIndex(BoneIdx) == CurrentIdx)
                ChildIndices.Add(BoneIdx);
        }

        if (ChildIndices.Num() == 1) {
            // Continue chain
            CurrentIdx = ChildIndices[0];
            OutChain.Add(RefSkeleton.GetBoneName(CurrentIdx));
        } else {
            // End of chain (no children or branch)
            break;
        }
    }

    return OutChain.Num() >= 2; // At least root and one child
}

bool UOHAnimationUtils::GetContiguousBoneChainToRoot(const USkeletalMeshComponent* SkeletalMeshComp, FName TipBone,
                                                     TArray<FName>& OutChain) {
    OutChain.Empty();

    if (!SkeletalMeshComp || !SkeletalMeshComp->GetSkeletalMeshAsset())
        return false;

    const USkeletalMesh* Mesh = SkeletalMeshComp->GetSkeletalMeshAsset();
    const FReferenceSkeleton& RefSkeleton = Mesh->GetRefSkeleton();

    int32 CurrentIdx = RefSkeleton.FindBoneIndex(TipBone);
    if (CurrentIdx == INDEX_NONE)
        return false;

    // Collect bones from tip toward root
    TArray<FName> ChainReversed;
    ChainReversed.Add(RefSkeleton.GetBoneName(CurrentIdx));

    while (true) {
        int32 ParentIdx = RefSkeleton.GetParentIndex(CurrentIdx);
        if (ParentIdx == INDEX_NONE)
            break; // Reached skeleton root

        // Count the children of this parent
        int32 NumChildren = 0;
        for (int32 i = 0; i < RefSkeleton.GetNum(); ++i) {
            if (RefSkeleton.GetParentIndex(i) == ParentIdx)
                ++NumChildren;
        }

        // If parent has more than one child, this is a branch point
        if (NumChildren > 1)
            break;

        ChainReversed.Add(RefSkeleton.GetBoneName(ParentIdx));
        CurrentIdx = ParentIdx;
    }

    // Reverse to get root-to-tip order
    Algo::Reverse(ChainReversed);
    OutChain = ChainReversed;

    return OutChain.Num() >= 2;
}

float UOHAnimationUtils::CalculateChainLengthInReferencePose(const USkeletalMeshComponent* SkeletalMeshComp,
                                                             const TArray<FName>& BoneChain) {
    if (!SkeletalMeshComp || !SkeletalMeshComp->GetSkeletalMeshAsset()) {
        UE_LOG(LogTemp, Warning, TEXT("Invalid SkeletalMeshComponent or mesh"));
        return 0.0f;
    }

    const USkeletalMesh* Mesh = SkeletalMeshComp->GetSkeletalMeshAsset();
    const FReferenceSkeleton& RefSkeleton = Mesh->GetRefSkeleton();

    if (BoneChain.Num() < 2) {
        UE_LOG(LogTemp, Warning, TEXT("Bone chain too short"));
        return 0.0f;
    }

    TArray<FVector> RefPosePositions;
    RefPosePositions.SetNum(BoneChain.Num());

    // Compute component-space positions for each bone in the chain
    for (int32 i = 0; i < BoneChain.Num(); ++i) {
        int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneChain[i]);
        if (BoneIndex == INDEX_NONE) {
            UE_LOG(LogTemp, Warning, TEXT("Bone %s not found in skeleton!"), *BoneChain[i].ToString());
            return 0.0f;
        }
        // Start with this bone's local ref pose
        FTransform BoneTransform = RefSkeleton.GetRefBonePose()[BoneIndex];
        // Accumulate parent transforms to get component space
        int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
        while (ParentIndex != INDEX_NONE) {
            BoneTransform = BoneTransform * RefSkeleton.GetRefBonePose()[ParentIndex];
            ParentIndex = RefSkeleton.GetParentIndex(ParentIndex);
        }
        RefPosePositions[i] = BoneTransform.GetLocation();
    }

    // Sum the distances between each consecutive bone in the chain
    float ChainLength = 0.0f;
    for (int32 i = 1; i < RefPosePositions.Num(); ++i) {
        ChainLength += FVector::Dist(RefPosePositions[i - 1], RefPosePositions[i]);
    }
    return ChainLength;
}

float UOHAnimationUtils::CalculateChainRestLength(const USkeletalMeshComponent* SkeletalMeshComp,
                                                  const TArray<FName>& BoneChain) {
    if (!SkeletalMeshComp || !SkeletalMeshComp->GetSkeletalMeshAsset()) {
        UE_LOG(LogTemp, Warning, TEXT("Invalid SkeletalMeshComponent or mesh"));
        return 0.0f;
    }

    const USkeletalMesh* Mesh = SkeletalMeshComp->GetSkeletalMeshAsset();
    const FReferenceSkeleton& RefSkeleton = Mesh->GetRefSkeleton();

    if (BoneChain.Num() < 2) {
        UE_LOG(LogTemp, Warning, TEXT("Bone chain too short"));
        return 0.0f;
    }

    float ChainLength = 0.0f;

    for (int32 i = 1; i < BoneChain.Num(); ++i) {
        int32 ChildIndex = RefSkeleton.FindBoneIndex(BoneChain[i]);
        int32 ParentIndex = RefSkeleton.FindBoneIndex(BoneChain[i - 1]);
        if (ChildIndex == INDEX_NONE || ParentIndex == INDEX_NONE) {
            UE_LOG(LogTemp, Warning, TEXT("Bone %s or %s not found in skeleton!"), *BoneChain[i - 1].ToString(),
                   *BoneChain[i].ToString());
            return 0.0f;
        }

        const FTransform& ChildLocal = RefSkeleton.GetRefBonePose()[ChildIndex];
        float SegmentLength = ChildLocal.GetLocation().Size();
        ChainLength += SegmentLength;
    }

    return ChainLength;
}

FTransform UOHAnimationUtils::GetCurrentAnimPoseTransform(USkeletalMeshComponent* Mesh, FName BoneName) {
    if (!Mesh)
        return FTransform::Identity;

    const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
    if (BoneIndex == INDEX_NONE)
        return FTransform::Identity;

    // This returns bone transform in component space from evaluated animation pose
    return Mesh->GetSocketTransform(BoneName, RTS_Component);
}

float UOHAnimationUtils::GetTransformError(UOHPhysicsManager* Manager, FName BoneName) {
    if (!Manager)
        return 0.f;

    const FOHBoneData* Data = Manager->GetBoneData(BoneName);
    if (!Data)
        return 0.f;

    USkeletalMeshComponent* Mesh = Manager->GetSkeletalMeshComponent();
    if (!Mesh)
        return 0.f;

    const FVector TrackedPos = Data->GetCurrentPosition();
    const FVector AnimPos = GetCurrentAnimPoseTransform(Mesh, BoneName).GetLocation();

    return FVector::Dist(AnimPos, TrackedPos);
}

float UOHAnimationUtils::GetRotationDriftError(UOHPhysicsManager* Manager, FName BoneName) {
    if (!Manager)
        return 0.f;

    const FOHBoneData* Data = Manager->GetBoneData(BoneName);
    if (!Data)
        return 0.f;

    USkeletalMeshComponent* Mesh = Manager->GetSkeletalMeshComponent();
    if (!Mesh)
        return 0.f;

    const FQuat TrackedRot = Data->GetCurrentRotation();
    const FQuat AnimRot = GetCurrentAnimPoseTransform(Mesh, BoneName).GetRotation();

    // Return angular difference in degrees
    return FMath::RadiansToDegrees(TrackedRot.AngularDistance(AnimRot));
}

void UOHAnimationUtils::DrawPoseDeviation(UOHPhysicsManager* Manager, FName BoneName, FLinearColor Color,
                                          float Thickness) {
    if (!Manager)
        return;

    const FOHBoneData* Data = Manager->GetBoneData(BoneName);
    if (!Data)
        return;

    USkeletalMeshComponent* Mesh = Manager->GetSkeletalMeshComponent();
    if (!Mesh)
        return;

    const FVector TrackedPos = Data->GetCurrentPosition();
    const FVector AnimPos = GetCurrentAnimPoseTransform(Mesh, BoneName).GetLocation();

    DrawDebugLine(Mesh->GetWorld(), AnimPos, TrackedPos, Color.ToFColor(true),
                  false, // persistent lines
                  0.05f, // duration
                  0, Thickness);
}

TArray<FName> UOHAnimationUtils::GetBonesWithHighDeviation(UOHPhysicsManager* Manager, float PositionThreshold,
                                                           float RotationThreshold) {
    TArray<FName> Result;

    if (!Manager)
        return Result;

    const USkeletalMeshComponent* Mesh = Manager->GetSkeletalMeshComponent();
    if (!Mesh)
        return Result;

    for (const TPair<FName, FOHBoneData>& Pair : Manager->GetTrackedBoneData()) {
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
static ERootMotionMode::Type GetAnimInstanceRootMotionMode(const UAnimInstance* AnimInstance) {
    if (!AnimInstance) {
        return ERootMotionMode::NoRootMotionExtraction;
    }
    return AnimInstance->RootMotionMode;
}