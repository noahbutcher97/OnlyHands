// Fill out your copyright notice in the Description page of Project Settings.
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "OHPhysicsStructs.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "BonePose.h"             // FBoneTransform
#include "BoneContainer.h"        // FBoneContainer (if needed)
#include "CollisionShape.h"
#include "OHSkeletalMappings.h"
#include "Animation/OHAnimInstance.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "Kismet/KismetMathLibrary.h"

// OHSkeletalPhysicsUtils.cpp
#pragma region BoneResolution
FName UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone(EOHSkeletalBone Bone, const USkeletalMeshComponent* SkelComp)
{
	if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
	{
		return NAME_None;
	}

	const FReferenceSkeleton& RefSkel = SkelComp->GetSkeletalMeshAsset()->GetRefSkeleton();

	// Try ideal mapping first
	if (const FName* Ideal = OHSkeletalMappings::SkeletalBoneToFNameMap.Find(Bone))
	{
		if (RefSkel.FindBoneIndex(*Ideal) != INDEX_NONE)
		{
			return *Ideal;
		}
	}

	// Fallback fuzzy match
	TArray<FName> BoneNames;
	for (int32 i = 0; i < RefSkel.GetNum(); ++i)
	{
		BoneNames.Add(RefSkel.GetBoneName(i));
	}

	FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(FName(UEnum::GetValueAsString(Bone)), BoneNames);
	return FName(*Match.Candidate);
}

FOHResolvedBoneData UOHSkeletalPhysicsUtils::ResolveResolvedDataFromSkeletalBone(EOHSkeletalBone Bone, const USkeletalMeshComponent* SkelComp)
{
	FOHResolvedBoneData Result;
	Result.LogicalBone = Bone;
	if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
	{
		return Result;
	}

	const FReferenceSkeleton& RefSkel = SkelComp->GetSkeletalMeshAsset()->GetRefSkeleton();

	// Ideal match
	if (const FName* Ideal = OHSkeletalMappings::SkeletalBoneToFNameMap.Find(Bone))
	{
		if (RefSkel.FindBoneIndex(*Ideal) != INDEX_NONE)
		{
			Result.ResolvedBone = *Ideal;
			Result.MatchResult = FOHNameMatchResult(Ideal->ToString(), 0.f, TEXT("Direct"));
			Result.bResolved = true;
		}
	}

	// Fallback fuzzy match
	if (!Result.bResolved)
	{
		TArray<FName> BoneNames;
		for (int32 i = 0; i < RefSkel.GetNum(); ++i)
		{
			BoneNames.Add(RefSkel.GetBoneName(i));
		}

		FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(FName(UEnum::GetValueAsString(Bone)), BoneNames);
		Result.ResolvedBone = FName(*Match.Candidate);
		Result.MatchResult = Match;
		Result.bResolved = true;
	}

	Result.BodyPart = GetBodyPartFromBone(Bone);
	Result.BodyZone = GetBodyZoneFromBone(Bone);
	Result.FunctionalGroup = GetFunctionalGroupFromBone(Bone);

	return Result;
}

EOHSkeletalBone UOHSkeletalPhysicsUtils::GetRootBoneInBodyPartFromMesh(const USkeletalMeshComponent* SkelComp, EOHBodyPart BodyPart)
{
	if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
		return EOHSkeletalBone::None;

	const FReferenceSkeleton& RefSkel = SkelComp->GetSkeletalMeshAsset()->GetRefSkeleton();

	// Resolve actual bone names
	TArray<EOHSkeletalBone> LogicalBones;
	if (const TArray<EOHSkeletalBone>* Bones = OHSkeletalMappings::BodyPartToPrimaryBonesMap.Find(BodyPart))
	{
		LogicalBones = *Bones;
	}

	TMap<FName, EOHSkeletalBone> NameToLogical;
	TSet<int32> BoneIndices;

	for (EOHSkeletalBone Logical : LogicalBones)
	{
		FName Resolved = ResolveBoneNameFromSkeletalBone(Logical, SkelComp);
		int32 Index = RefSkel.FindBoneIndex(Resolved);
		if (Index != INDEX_NONE)
		{
			BoneIndices.Add(Index);
			NameToLogical.Add(Resolved, Logical);
		}
	}

	// Find root-most index (highest in hierarchy)
	int32 RootIndex = INDEX_NONE;
	for (int32 Index : BoneIndices)
	{
		int32 ParentIndex = RefSkel.GetParentIndex(Index);
		if (!BoneIndices.Contains(ParentIndex))
		{
			RootIndex = Index;
			break;
		}
	}

	if (RootIndex != INDEX_NONE)
	{
		return NameToLogical.FindChecked(RefSkel.GetBoneName(RootIndex));
	}

	return EOHSkeletalBone::None;
}

EOHSkeletalBone UOHSkeletalPhysicsUtils::GetEndBoneInBodyPartFromMesh(const USkeletalMeshComponent* SkelComp, EOHBodyPart BodyPart)
{
	if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
		return EOHSkeletalBone::None;

	const FReferenceSkeleton& RefSkel = SkelComp->GetSkeletalMeshAsset()->GetRefSkeleton();

	// Resolve actual bone names
	TArray<EOHSkeletalBone> LogicalBones;
	if (const TArray<EOHSkeletalBone>* Bones = OHSkeletalMappings::BodyPartToPrimaryBonesMap.Find(BodyPart))
	{
		LogicalBones = *Bones;
	}

	TMap<FName, EOHSkeletalBone> NameToLogical;
	TSet<int32> BoneIndices;

	for (EOHSkeletalBone Logical : LogicalBones)
	{
		FName Resolved = ResolveBoneNameFromSkeletalBone(Logical, SkelComp);
		int32 Index = RefSkel.FindBoneIndex(Resolved);
		if (Index != INDEX_NONE)
		{
			BoneIndices.Add(Index);
			NameToLogical.Add(Resolved, Logical);
		}
	}

	// Find deepest (leaf-most) bone in the region
	int32 DeepestIndex = INDEX_NONE;
	int32 MaxDepth = -1;

	for (int32 Index : BoneIndices)
	{
		int32 Depth = 0;
		int32 Walk = Index;

		while (Walk != INDEX_NONE)
		{
			Walk = RefSkel.GetParentIndex(Walk);
			Depth++;
		}

		if (Depth > MaxDepth)
		{
			MaxDepth = Depth;
			DeepestIndex = Index;
		}
	}

	if (DeepestIndex != INDEX_NONE)
	{
		return NameToLogical.FindChecked(RefSkel.GetBoneName(DeepestIndex));
	}

	return EOHSkeletalBone::None;
}

FName UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(EOHSkeletalBone Bone)
{
	if (const FName* Found = OHSkeletalMappings::SkeletalBoneToFNameMap.Find(Bone))
	{
		return *Found;
	}

	TArray<FName> AllNames;
	OHSkeletalMappings::SkeletalBoneToFNameMap.GenerateValueArray(AllNames);

	FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(FName(UEnum::GetValueAsString(Bone)), AllNames);
	return FName(*Match.Candidate);
}

FOHResolvedBoneData UOHSkeletalPhysicsUtils::ResolveResolvedDataFromSkeletalBone_Static(EOHSkeletalBone Bone)
{
	FOHResolvedBoneData Result;
	Result.LogicalBone = Bone;

	if (const FName* Found = OHSkeletalMappings::SkeletalBoneToFNameMap.Find(Bone))
	{
		Result.ResolvedBone = *Found;
		Result.MatchResult = FOHNameMatchResult(Found->ToString(), 0.f, TEXT("Direct"));
		Result.bResolved = true;
	}
	else
	{
		TArray<FName> AllNames;
		OHSkeletalMappings::SkeletalBoneToFNameMap.GenerateValueArray(AllNames);
		FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(FName(UEnum::GetValueAsString(Bone)), AllNames);

		Result.ResolvedBone = FName(*Match.Candidate);
		Result.MatchResult = Match;
		Result.bResolved = true;
	}

	Result.BodyPart = GetBodyPartFromBone(Bone);
	Result.BodyZone = GetBodyZoneFromBone(Bone);
	Result.FunctionalGroup = GetFunctionalGroupFromBone(Bone);

	return Result;
}


#pragma endregion


#pragma region Enums

EOHBodyZone UOHSkeletalPhysicsUtils::GetBodyZoneFromBone(EOHSkeletalBone Bone)
{
	if (const EOHBodyZone* Found = OHSkeletalMappings::BoneToZoneMap.Find(Bone))
	{
		return *Found;
	}
	return EOHBodyZone::None;
}

EOHBodyPart UOHSkeletalPhysicsUtils::GetBodyPartFromBone(EOHSkeletalBone Bone)
{
	if (const EOHBodyPart* Found = OHSkeletalMappings::BoneToBodyPartMap.Find(Bone))
	{
		return *Found;
	}
	return EOHBodyPart::None;
}


EOHFunctionalBoneGroup UOHSkeletalPhysicsUtils::GetFunctionalGroupFromBone(EOHSkeletalBone Bone)
{
	if (const EOHFunctionalBoneGroup* Found = OHSkeletalMappings::BoneToFunctionalGroupMap.Find(Bone))
	{
		return *Found;
	}
	return EOHFunctionalBoneGroup::None;
}


TArray<FName> UOHSkeletalPhysicsUtils::GetPrimaryBoneNamesFromBodyPart(EOHBodyPart BodyPart)
{
	TArray<FName> Result;

	if (const TArray<EOHSkeletalBone>* Bones = OHSkeletalMappings::BodyPartToPrimaryBonesMap.Find(BodyPart))
	{
		for (EOHSkeletalBone Bone : *Bones)
		{
			if (const FName* Name = OHSkeletalMappings::PrimaryBoneToFNameMap.Find(Bone))
			{
				Result.Add(*Name);
			}
		}
	}

	return Result;
}

EOHSkeletalBone UOHSkeletalPhysicsUtils::ResolveSkeletalBoneFromNameSmart(const FName& Input, const USkeletalMeshComponent* SkelComp, float ScoreThreshold)
{
	TArray<FName> AllBoneNames;
	OHSkeletalMappings::SkeletalBoneToFNameMap.GenerateValueArray(AllBoneNames);

	FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(Input, AllBoneNames);
	if (Match.Score >= ScoreThreshold)
	{
		const FName MatchedName(*Match.Candidate);
		for (const TPair<EOHSkeletalBone, FName>& Pair : OHSkeletalMappings::SkeletalBoneToFNameMap)
		{
			if (Pair.Value == MatchedName)
			{
				return Pair.Key;
			}
		}
	}
	return EOHSkeletalBone::None;
}
#pragma endregion






bool UOHSkeletalPhysicsUtils::GetBoneCollisionShape(
	UPhysicsAsset* PhysAsset,
	FName BoneName,
	FTransform& OutBoneTransform,
	FCollisionShape& OutShape)
{
	if (!PhysAsset)
		return false;

	const int32 BodyIndex = PhysAsset->FindBodyIndex(BoneName);
	if (BodyIndex == INDEX_NONE)
		return false;

	const USkeletalBodySetup* BodySetup = PhysAsset->SkeletalBodySetups[BodyIndex];
	if (!BodySetup || BodySetup->AggGeom.GetElementCount() == 0)
		return false;

	const FKSphylElem* Capsule = BodySetup->AggGeom.SphylElems.Num() > 0
		? &BodySetup->AggGeom.SphylElems[0]
		: nullptr;

	if (Capsule)
	{
		OutBoneTransform = Capsule->GetTransform();
		OutShape = FCollisionShape::MakeCapsule(Capsule->Radius, Capsule->Length * 0.5f);
		return true;
	}

	const FKSphereElem* Sphere = BodySetup->AggGeom.SphereElems.Num() > 0
		? &BodySetup->AggGeom.SphereElems[0]
		: nullptr;

	if (Sphere)
	{
		OutBoneTransform = Sphere->GetTransform();
		OutShape = FCollisionShape::MakeSphere(Sphere->Radius);
		return true;
	}

	return false;
}



bool TryGetBoneReferenceFromEnum_Internal(
	EOHSkeletalBone BoneEnum,
	const FBoneContainer& BoneContainer,
	FBoneReference& OutRef)
{
	if (BoneEnum == EOHSkeletalBone::None)
	{
		return false;
	}
	
	const FName BoneName = GetBoneNameFromEnum(BoneEnum);
	const int32 PoseIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);
	if (PoseIndex == INDEX_NONE)
	{
		return false;
	}

	OutRef.BoneName = BoneName;
	OutRef.Initialize(BoneContainer);
	return OutRef.IsValidToEvaluate(BoneContainer);
}





bool UOHSkeletalPhysicsUtils::TryGetPoseIndex(
	EOHSkeletalBone Bone,
	const TMap<EOHSkeletalBone, FCompactPoseBoneIndex>& Indices,
	FCompactPoseBoneIndex& OutIndex)
{
	if (const FCompactPoseBoneIndex* Found = Indices.Find(Bone))
	{
		OutIndex = *Found;
		return true;
	}
	OutIndex = FCompactPoseBoneIndex(INDEX_NONE);
	return false;
}


TArray<FConstraintInstance*> UOHSkeletalPhysicsUtils::GetAllParentConstraints(USkeletalMeshComponent* SkelMesh,
	FName BoneName)
{
	TArray<FConstraintInstance*> Result;
	if (!SkelMesh) return Result;
	UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset();
	if (!PhysAsset) return Result;

	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);

	while (MyIdx != INDEX_NONE)
	{
		int32 ParentIdx = RefSkel.GetParentIndex(MyIdx);
		if (ParentIdx == INDEX_NONE) break;
		FName ParentName = RefSkel.GetBoneName(ParentIdx);
		FName ThisName = RefSkel.GetBoneName(MyIdx);

		for (UPhysicsConstraintTemplate* ConstraintTemplate : PhysAsset->ConstraintSetup)
		{
			if (!ConstraintTemplate) continue;
			FConstraintInstance* CI = const_cast<FConstraintInstance*>(&ConstraintTemplate->DefaultInstance);
			if ((CI->ConstraintBone1 == ParentName && CI->ConstraintBone2 == ThisName) ||
				(CI->ConstraintBone1 == ThisName && CI->ConstraintBone2 == ParentName))
			{
				Result.Add(CI);
				break;
			}
		}
		MyIdx = ParentIdx;
	}
	return Result;
}

TArray<FName> UOHSkeletalPhysicsUtils::GetAllParentBoneNames(USkeletalMeshComponent* SkelMesh, FName BoneName)
{
	TArray<FName> OutNames;
	if (!SkelMesh) return OutNames;
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);
	while (true)
	{
		int32 ParentIdx = RefSkel.GetParentIndex(MyIdx);
		if (ParentIdx == INDEX_NONE) break;
		OutNames.Add(RefSkel.GetBoneName(ParentIdx));
		MyIdx = ParentIdx;
	}
	return OutNames;
}

float UOHSkeletalPhysicsUtils::ComputeBoneLength(const USkeletalMeshComponent* SkeletalMesh, FName BoneA, FName BoneB)
{
	if (!SkeletalMesh || BoneA.IsNone() || BoneB.IsNone())
	{
		return 0.f;
	}

	using FCacheKey = TTuple<const USkeletalMeshComponent*, FName, FName>;
	static TMap<FCacheKey, float> LengthCache;

	// Ensure consistent ordering of bone pair
	const bool bSwap = BoneA.FastLess(BoneB);
	const FName KeyA = bSwap ? BoneA : BoneB;
	const FName KeyB = bSwap ? BoneB : BoneA;

	const FCacheKey CacheKey(SkeletalMesh, KeyA, KeyB);

	if (const float* CachedLength = LengthCache.Find(CacheKey))
	{
		return *CachedLength;
	}

	const FVector PosA = SkeletalMesh->GetBoneLocation(BoneA);
	const FVector PosB = SkeletalMesh->GetBoneLocation(BoneB);
	const float Length = FVector::Dist(PosA, PosB);

	LengthCache.Add(CacheKey, Length);
	return Length;
}



TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetChildBonesByDepth(EOHSkeletalBone RootBone, int32 MaxDepth)
{
	TArray<EOHSkeletalBone> Result;
	if (MaxDepth <= 0)
	{
		return Result;
	}

	TQueue<TPair<EOHSkeletalBone, int32>> Queue;
	Queue.Enqueue(TPair<EOHSkeletalBone, int32>(RootBone, 0));

	while (!Queue.IsEmpty())
	{
		TPair<EOHSkeletalBone, int32> Current;
		Queue.Dequeue(Current);

		EOHSkeletalBone Bone = Current.Key;
		int32 Depth = Current.Value;

		if (Depth >= MaxDepth)
		{
			continue;
		}

		const TArray<EOHSkeletalBone> Children = GetChildBones(Bone);
		for (EOHSkeletalBone Child : Children)
		{
			Result.Add(Child);
			Queue.Enqueue(TPair<EOHSkeletalBone, int32>(Child, Depth + 1));
		}
	}

	return Result;
}


EOHSkeletalBone UOHSkeletalPhysicsUtils::GetDirectChildBone(EOHSkeletalBone ParentBone)
{
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (GetParentBone(Bone) == ParentBone)
		{
			return Bone; // First match only
		}
	}

	return EOHSkeletalBone::None;
}

int32 UOHSkeletalPhysicsUtils::GetBoneDepthRelativeTo(EOHSkeletalBone ParentBone, EOHSkeletalBone TargetBone)
{
	if (ParentBone == EOHSkeletalBone::None || TargetBone == EOHSkeletalBone::None)
	{
		return -1;
	}

	if (ParentBone == TargetBone)
	{
		return 0;
	}

	int32 Depth = 0;
	EOHSkeletalBone Current = TargetBone;

	while (Current != EOHSkeletalBone::None)
	{
		Current = GetParentBone(Current);
		Depth++;

		if (Current == ParentBone)
		{
			return Depth;
		}
	}

	return -1; // Not a descendant
}


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBoneLineage(EOHSkeletalBone TargetBone, EOHSkeletalBone StopAtBone)
{
	TArray<EOHSkeletalBone> Lineage;

	EOHSkeletalBone Current = TargetBone;

	while (Current != EOHSkeletalBone::None)
	{
		Lineage.Add(Current);

		if (Current == StopAtBone)
		{
			break;
		}

		Current = GetParentBone(Current);
	}

	return Lineage;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBoneLineageToRoot(EOHSkeletalBone TargetBone)
{
	return GetBoneLineage(TargetBone, EOHSkeletalBone::None);
}

int32 UOHSkeletalPhysicsUtils::CompareBoneDepth(EOHSkeletalBone A, EOHSkeletalBone B)
{
	if (A == B)
	{
		return 0;
	}

	const TArray<EOHSkeletalBone> LineageA = GetBoneLineageToRoot(A);
	const TArray<EOHSkeletalBone> LineageB = GetBoneLineageToRoot(B);

	const int32 DepthA = LineageA.Num();
	const int32 DepthB = LineageB.Num();

	if (DepthA > DepthB)
	{
		return -1; // A is deeper
	}
	if (DepthA < DepthB)
	{
		return 1; // B is deeper
	}
	return 0; // Same depth
}


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBoneChainBelow(const EOHSkeletalBone RootBone)
{
	static TMap<EOHSkeletalBone, TArray<EOHSkeletalBone>> BoneHierarchy = {
		{
			EOHSkeletalBone::Pelvis, {
				EOHSkeletalBone::Spine_01, EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03,
				EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L, EOHSkeletalBone::LowerArm_L,
				EOHSkeletalBone::Hand_L, EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
				EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
				EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L, EOHSkeletalBone::Foot_L,
				EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R, EOHSkeletalBone::Foot_R
			}
		},
		{
			EOHSkeletalBone::Clavicle_L, {
				EOHSkeletalBone::UpperArm_L, EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L
			}
		},
		{
			EOHSkeletalBone::UpperArm_L, {
				EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L
			}
		},
		{
			EOHSkeletalBone::Clavicle_R, {
				EOHSkeletalBone::UpperArm_R, EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R
			}
		},
		{
			EOHSkeletalBone::UpperArm_R, {
				EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R
			}
		},
		{
			EOHSkeletalBone::Thigh_L, {
				EOHSkeletalBone::Calf_L, EOHSkeletalBone::Foot_L
			}
		},
		{
			EOHSkeletalBone::Thigh_R, {
				EOHSkeletalBone::Calf_R, EOHSkeletalBone::Foot_R
			}
		},
		// ... extend as needed
	};

	TArray<EOHSkeletalBone> OutChain;
	OutChain.Add(RootBone);

	if (const TArray<EOHSkeletalBone>* Children = BoneHierarchy.Find(RootBone))
	{
		for (EOHSkeletalBone Child : *Children)
		{
			OutChain.Append(GetBoneChainBelow(Child)); // recurse
		}
	}

	return OutChain;
}


EOHSkeletalBone UOHSkeletalPhysicsUtils::GetParentBone(EOHSkeletalBone TargetBone)
{
	switch (TargetBone)
	{
	// Spine
	case EOHSkeletalBone::Spine_01: return EOHSkeletalBone::Pelvis;
	case EOHSkeletalBone::Spine_02: return EOHSkeletalBone::Spine_01;
	case EOHSkeletalBone::Spine_03: return EOHSkeletalBone::Spine_02;

	// Head
	case EOHSkeletalBone::Neck_01: return EOHSkeletalBone::Spine_03;
	case EOHSkeletalBone::Head: return EOHSkeletalBone::Neck_01;

	// Left Arm
	case EOHSkeletalBone::Clavicle_L: return EOHSkeletalBone::Spine_03;
	case EOHSkeletalBone::UpperArm_L: return EOHSkeletalBone::Clavicle_L;
	case EOHSkeletalBone::LowerArm_L: return EOHSkeletalBone::UpperArm_L;
	case EOHSkeletalBone::Hand_L: return EOHSkeletalBone::LowerArm_L;

	// Left Fingers
	case EOHSkeletalBone::Thumb_01_L: return EOHSkeletalBone::Hand_L;
	case EOHSkeletalBone::Thumb_02_L: return EOHSkeletalBone::Thumb_01_L;
	case EOHSkeletalBone::Thumb_03_L: return EOHSkeletalBone::Thumb_02_L;

	case EOHSkeletalBone::Index_01_L: return EOHSkeletalBone::Hand_L;
	case EOHSkeletalBone::Index_02_L: return EOHSkeletalBone::Index_01_L;
	case EOHSkeletalBone::Index_03_L: return EOHSkeletalBone::Index_02_L;

	case EOHSkeletalBone::Middle_01_L: return EOHSkeletalBone::Hand_L;
	case EOHSkeletalBone::Middle_02_L: return EOHSkeletalBone::Middle_01_L;
	case EOHSkeletalBone::Middle_03_L: return EOHSkeletalBone::Middle_02_L;

	case EOHSkeletalBone::Ring_01_L: return EOHSkeletalBone::Hand_L;
	case EOHSkeletalBone::Ring_02_L: return EOHSkeletalBone::Ring_01_L;
	case EOHSkeletalBone::Ring_03_L: return EOHSkeletalBone::Ring_02_L;

	case EOHSkeletalBone::Pinky_01_L: return EOHSkeletalBone::Hand_L;
	case EOHSkeletalBone::Pinky_02_L: return EOHSkeletalBone::Pinky_01_L;
	case EOHSkeletalBone::Pinky_03_L: return EOHSkeletalBone::Pinky_02_L;

	// Right Arm
	case EOHSkeletalBone::Clavicle_R: return EOHSkeletalBone::Spine_03;
	case EOHSkeletalBone::UpperArm_R: return EOHSkeletalBone::Clavicle_R;
	case EOHSkeletalBone::LowerArm_R: return EOHSkeletalBone::UpperArm_R;
	case EOHSkeletalBone::Hand_R: return EOHSkeletalBone::LowerArm_R;

	// Right Fingers
	case EOHSkeletalBone::Thumb_01_R: return EOHSkeletalBone::Hand_R;
	case EOHSkeletalBone::Thumb_02_R: return EOHSkeletalBone::Thumb_01_R;
	case EOHSkeletalBone::Thumb_03_R: return EOHSkeletalBone::Thumb_02_R;

	case EOHSkeletalBone::Index_01_R: return EOHSkeletalBone::Hand_R;
	case EOHSkeletalBone::Index_02_R: return EOHSkeletalBone::Index_01_R;
	case EOHSkeletalBone::Index_03_R: return EOHSkeletalBone::Index_02_R;

	case EOHSkeletalBone::Middle_01_R: return EOHSkeletalBone::Hand_R;
	case EOHSkeletalBone::Middle_02_R: return EOHSkeletalBone::Middle_01_R;
	case EOHSkeletalBone::Middle_03_R: return EOHSkeletalBone::Middle_02_R;

	case EOHSkeletalBone::Ring_01_R: return EOHSkeletalBone::Hand_R;
	case EOHSkeletalBone::Ring_02_R: return EOHSkeletalBone::Ring_01_R;
	case EOHSkeletalBone::Ring_03_R: return EOHSkeletalBone::Ring_02_R;

	case EOHSkeletalBone::Pinky_01_R: return EOHSkeletalBone::Hand_R;
	case EOHSkeletalBone::Pinky_02_R: return EOHSkeletalBone::Pinky_01_R;
	case EOHSkeletalBone::Pinky_03_R: return EOHSkeletalBone::Pinky_02_R;

	// Left Leg
	case EOHSkeletalBone::Thigh_L: return EOHSkeletalBone::Pelvis;
	case EOHSkeletalBone::Calf_L: return EOHSkeletalBone::Thigh_L;
	case EOHSkeletalBone::Foot_L: return EOHSkeletalBone::Calf_L;
	case EOHSkeletalBone::Ball_L: return EOHSkeletalBone::Foot_L;

	// Right Leg
	case EOHSkeletalBone::Thigh_R: return EOHSkeletalBone::Pelvis;
	case EOHSkeletalBone::Calf_R: return EOHSkeletalBone::Thigh_R;
	case EOHSkeletalBone::Foot_R: return EOHSkeletalBone::Calf_R;
	case EOHSkeletalBone::Ball_R: return EOHSkeletalBone::Foot_R;

	default:
		return EOHSkeletalBone::None;
	}
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup Group)
{
	using FB = EOHSkeletalBone;
	TArray<FB> Bones;

	switch (Group)
	{
	case EOHFunctionalBoneGroup::Cranial:
		Bones = { FB::Neck_01, FB::Head };
		break;

	case EOHFunctionalBoneGroup::Core:
		Bones = { FB::Pelvis, FB::Spine_01, FB::Spine_02 };
		break;

	case EOHFunctionalBoneGroup::LeftArm:
		Bones = { FB::Clavicle_L, FB::UpperArm_L, FB::LowerArm_L, FB::Hand_L };
		break;

	case EOHFunctionalBoneGroup::RightArm:
		Bones = { FB::Clavicle_R, FB::UpperArm_R, FB::LowerArm_R, FB::Hand_R };
		break;

	case EOHFunctionalBoneGroup::LeftLeg:
		Bones = { FB::Thigh_L, FB::Calf_L };
		break;

	case EOHFunctionalBoneGroup::RightLeg:
		Bones = { FB::Thigh_R, FB::Calf_R };
		break;

	case EOHFunctionalBoneGroup::LeftHand:
		Bones = { FB::Hand_L };
		break;

	case EOHFunctionalBoneGroup::RightHand:
		Bones = { FB::Hand_R };
		break;

	case EOHFunctionalBoneGroup::LeftFoot:
		Bones = { FB::Foot_L, FB::Ball_L };
		break;

	case EOHFunctionalBoneGroup::RightFoot:
		Bones = { FB::Foot_R, FB::Ball_R };
		break;

	case EOHFunctionalBoneGroup::Hands:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftHand);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightHand));
		break;

	case EOHFunctionalBoneGroup::Feet:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftFoot);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightFoot));
		break;

	case EOHFunctionalBoneGroup::Arms:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftArm);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightArm));
		break;

	case EOHFunctionalBoneGroup::Legs:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftLeg);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightLeg));
		break;

	case EOHFunctionalBoneGroup::LeftLimbs:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftArm);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LeftLeg));
		break;

	case EOHFunctionalBoneGroup::RightLimbs:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightArm);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::RightLeg));
		break;

	case EOHFunctionalBoneGroup::UpperBody:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Cranial);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Arms));
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Hands));
		break;

	case EOHFunctionalBoneGroup::LowerBody:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Legs);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Feet));
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::Core));
		break;

	case EOHFunctionalBoneGroup::FullBody:
		Bones = GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::UpperBody);
		Bones.Append(GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup::LowerBody));
		break;

	default:
		break;
	}

	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetChildBones(EOHSkeletalBone ParentBone)
{
	TArray<EOHSkeletalBone> Children;

	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (GetParentBone(Bone) == ParentBone)
		{
			Children.Add(Bone);
		}
	}

	return Children;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetDescendantBones(EOHSkeletalBone RootBone)
{
	TArray<EOHSkeletalBone> Descendants;
	TQueue<EOHSkeletalBone> BonesToVisit;
	BonesToVisit.Enqueue(RootBone);

	while (!BonesToVisit.IsEmpty())
	{
		EOHSkeletalBone CurrentBone;
		BonesToVisit.Dequeue(CurrentBone);

		const TArray<EOHSkeletalBone> Children = GetChildBones(CurrentBone);

		for (EOHSkeletalBone Child : Children)
		{
			Descendants.Add(Child);
			BonesToVisit.Enqueue(Child);
		}
	}
	
	return Descendants;
}


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBoneInfluenceChain(const EOHSkeletalBone Bone)
{
	TArray<EOHSkeletalBone> InfluenceChain;

	// Start with the target bone
	InfluenceChain.Add(Bone);

	// Define bone hierarchy and influence paths
	// This is a simplified implementation - in a real system this would use
	// a proper skeletal hierarchy definition

	// Hand and finger influence chains
	if (IsFingerBone(Bone))
	{
		// Add hand
		if (IsLeftSideBone(Bone))
		{
			InfluenceChain.Add(EOHSkeletalBone::Hand_L);
			InfluenceChain.Add(EOHSkeletalBone::LowerArm_L);
			InfluenceChain.Add(EOHSkeletalBone::UpperArm_L);
			InfluenceChain.Add(EOHSkeletalBone::Clavicle_L);
			// Connect to spine
			InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		}
		else // Right side
		{
			InfluenceChain.Add(EOHSkeletalBone::Hand_R);
			InfluenceChain.Add(EOHSkeletalBone::LowerArm_R);
			InfluenceChain.Add(EOHSkeletalBone::UpperArm_R);
			InfluenceChain.Add(EOHSkeletalBone::Clavicle_R);
			// Connect to spine
			InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		}
	}
	// Hand influence chains
	else if (Bone == EOHSkeletalBone::Hand_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::LowerArm_L);
		InfluenceChain.Add(EOHSkeletalBone::UpperArm_L);
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_L);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
	}
	else if (Bone == EOHSkeletalBone::Hand_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::LowerArm_R);
		InfluenceChain.Add(EOHSkeletalBone::UpperArm_R);
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_R);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
	}
	// Arm influence chains
	else if (Bone == EOHSkeletalBone::LowerArm_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::UpperArm_L);
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_L);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
	}
	else if (Bone == EOHSkeletalBone::LowerArm_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::UpperArm_R);
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_R);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
	}
	else if (Bone == EOHSkeletalBone::UpperArm_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_L);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::UpperArm_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::Clavicle_R);
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	// Leg influence chains
	else if (Bone == EOHSkeletalBone::Foot_L || Bone == EOHSkeletalBone::Ball_L)
	{
		if (Bone == EOHSkeletalBone::Ball_L)
		{
			InfluenceChain.Add(EOHSkeletalBone::Foot_L);
		}
		InfluenceChain.Add(EOHSkeletalBone::Calf_L);
		InfluenceChain.Add(EOHSkeletalBone::Thigh_L);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Foot_R || Bone == EOHSkeletalBone::Ball_R)
	{
		if (Bone == EOHSkeletalBone::Ball_R)
		{
			InfluenceChain.Add(EOHSkeletalBone::Foot_R);
		}
		InfluenceChain.Add(EOHSkeletalBone::Calf_R);
		InfluenceChain.Add(EOHSkeletalBone::Thigh_R);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Calf_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::Thigh_L);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Calf_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::Thigh_R);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Thigh_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Thigh_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	// Spine and head influence chains
	else if (Bone == EOHSkeletalBone::Head || Bone == EOHSkeletalBone::Neck_01)
	{
		if (Bone == EOHSkeletalBone::Head)
		{
			InfluenceChain.Add(EOHSkeletalBone::Neck_01);
		}
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Spine_03)
	{
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
	}
	else if (Bone == EOHSkeletalBone::Spine_02)
	{
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
	}
	else if (Bone == EOHSkeletalBone::Spine_01)
	{
		InfluenceChain.Add(EOHSkeletalBone::Pelvis);
	}
	// Clavicle influence chains
	else if (Bone == EOHSkeletalBone::Clavicle_L)
	{
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}
	else if (Bone == EOHSkeletalBone::Clavicle_R)
	{
		InfluenceChain.Add(EOHSkeletalBone::Spine_03);
		InfluenceChain.Add(EOHSkeletalBone::Spine_02);
		InfluenceChain.Add(EOHSkeletalBone::Spine_01);
	}

	return InfluenceChain;
}







		// ------------------------------- Physics Solver Functions --------------------------------- //

#pragma region PhysicsSolvers

void UOHSkeletalPhysicsUtils::SolveIK_TwoBone(
	FTransform& OutUpper,
	FTransform& OutLower,
	FTransform& OutEnd,
	const FVector& TargetPosition,
	const FVector& JointTarget,
	bool bAllowStretching,
	float StartStretchRatio,
	float MaxStretchScale,
	UWorld* WorldDebugContext,
	bool bDrawDebug)
{
	const FVector RootPos = OutUpper.GetLocation();
	const FVector ElbowPos = OutLower.GetLocation();
	const FVector WristPos = OutEnd.GetLocation();

	const float UpperLen = FVector::Distance(RootPos, ElbowPos);
	const float LowerLen = FVector::Distance(ElbowPos, WristPos);
	if (UpperLen <= KINDA_SMALL_NUMBER || LowerLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector DesiredDir = TargetPosition - RootPos;
	const float DesiredLength = DesiredDir.Size();
	if (DesiredLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	float StretchScale = 1.f;
	const float MaxReach = UpperLen + LowerLen;
	if (bAllowStretching && DesiredLength > MaxReach * StartStretchRatio)
	{
		StretchScale = FMath::Min(DesiredLength / MaxReach, MaxStretchScale);
	}

	const float A = UpperLen * StretchScale;
	const float B = LowerLen * StretchScale;
	const float C = DesiredLength;

	const float CosAngle = FMath::Clamp((A * A + C * C - B * B) / (2 * A * C), -1.f, 1.f);
	const float Angle = FMath::Acos(CosAngle);
	const FVector DirectionNorm = DesiredDir / DesiredLength;

	const FVector ToJoint = JointTarget - RootPos;
	const FVector RotationAxis = FVector::CrossProduct(ToJoint, DesiredDir).GetSafeNormal();

	// Fall back to a default axis if the cross product is nearly zero
	const FVector SafeRotationAxis = RotationAxis.IsNearlyZero()
		                                 ? ComputeFallbackBendAxis(OutUpper, OutLower)
		                                 : RotationAxis;

	const FQuat UpperRot = FQuat(SafeRotationAxis, Angle);
	const FVector NewElbowPos = RootPos + UpperRot.RotateVector(DirectionNorm * A);
	const FVector NewWristPos = TargetPosition;

	const FVector UpperDir = NewElbowPos - RootPos;
	const FVector LowerDir = NewWristPos - NewElbowPos;

	OutUpper.SetRotation(UpperDir.ToOrientationQuat());
	OutLower.SetRotation(LowerDir.ToOrientationQuat());
	OutLower.SetLocation(NewElbowPos);
	OutEnd.SetLocation(NewWristPos);

	if (bDrawDebug && WorldDebugContext)
	{
		DrawDebugLine(WorldDebugContext, RootPos, NewElbowPos, FColor::Blue, false, 0.1f, 0, 2.f);
		DrawDebugLine(WorldDebugContext, NewElbowPos, NewWristPos, FColor::Green, false, 0.1f, 0, 2.f);
		DrawDebugSphere(WorldDebugContext, NewElbowPos, 2.f, 8, FColor::Cyan, false, 0.1f);
		DrawDebugSphere(WorldDebugContext, NewWristPos, 2.f, 8, FColor::Yellow, false, 0.1f);
	}
}

void UOHSkeletalPhysicsUtils::SolveIK_ArmChain(
	FTransform& Clavicle,
	FTransform& UpperArm,
	FTransform& LowerArm,
	FTransform& Hand,
	const FVector& IKTarget,
	const FVector& ElbowHint,
	bool bUseClavicle,
	bool bAllowStretching,
	float MaxStretchScale,
	bool bEnableWristRotation,
	const FVector& WristLookAtAxis,
	UWorld* World,
	bool bDrawDebug)
{
	// Chain setup
	const FVector RootPos = bUseClavicle ? Clavicle.GetLocation() : UpperArm.GetLocation();
	const FVector MidPos = UpperArm.GetLocation();
	const FVector ElbowPos = LowerArm.GetLocation();
	const FVector WristPos = Hand.GetLocation();

	const float UpperLen = FVector::Distance(MidPos, ElbowPos);
	const float LowerLen = FVector::Distance(ElbowPos, WristPos);

	if (UpperLen <= KINDA_SMALL_NUMBER || LowerLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector DesiredDir = IKTarget - RootPos;
	const float DesiredLen = DesiredDir.Size();
	if (DesiredLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Stretching
	float StretchScale = 1.f;
	if (const float MaxReach = UpperLen + LowerLen; bAllowStretching && DesiredLen > MaxReach)
	{
		StretchScale = FMath::Min(DesiredLen / MaxReach, MaxStretchScale);
	}

	const float A = UpperLen * StretchScale;
	const float B = LowerLen * StretchScale;
	const float C = DesiredLen;

	// Law of Cosines
	const float CosAngle = FMath::Clamp((A * A + C * C - B * B) / (2 * A * C), -1.f, 1.f);
	const float Angle = FMath::Acos(CosAngle);
	const FVector DirectionNorm = DesiredDir / DesiredLen;

	// Elbow bend axis
	FVector ToHint = ElbowHint - RootPos;
	FVector RotationAxis = FVector::CrossProduct(DirectionNorm, ToHint).GetSafeNormal();

	if (RotationAxis.IsNearlyZero())
	{
		RotationAxis = ComputeFallbackBendAxis(UpperArm, LowerArm);
	}

	const FQuat UpperRot = FQuat(RotationAxis, Angle);
	const FVector NewElbow = RootPos + UpperRot.RotateVector(DirectionNorm * A);
	const FVector NewWrist = IKTarget;

	// Final vectors
	const FVector UpperDir = NewElbow - RootPos;
	const FVector LowerDir = NewWrist - NewElbow;

	UpperArm.SetLocation(RootPos);
	UpperArm.SetRotation(UpperDir.ToOrientationQuat());

	LowerArm.SetLocation(NewElbow);
	LowerArm.SetRotation(LowerDir.ToOrientationQuat());

	Hand.SetLocation(NewWrist);

	if (bEnableWristRotation)
	{
		const FVector WristForward = (NewWrist - NewElbow).GetSafeNormal();
		const FQuat WristRot = FRotationMatrix::MakeFromX(WristForward).ToQuat();
		Hand.SetRotation(WristRot);
	}

	// Optional clavicle retraction
	if (bUseClavicle)
	{
		const FVector ShoulderToTarget = NewWrist - Clavicle.GetLocation();
		Clavicle.SetRotation(ShoulderToTarget.ToOrientationQuat());
	}

	if (bDrawDebug && World)
	{
		DrawDebugLine(World, RootPos, NewElbow, FColor::Blue, false, 0.1f, 0, 2.f);
		DrawDebugLine(World, NewElbow, NewWrist, FColor::Green, false, 0.1f, 0, 2.f);
		DrawDebugSphere(World, NewElbow, 2.f, 8, FColor::Cyan, false, 0.1f);
		DrawDebugSphere(World, NewWrist, 2.f, 8, FColor::Yellow, false, 0.1f);
	}
}

FVector UOHSkeletalPhysicsUtils::ComputeFallbackBendAxis(const FTransform& Root, const FTransform& Mid)
{
	// Use local +Y (or mirrored -Y) as fallback bend axis
	const FVector LocalAxisY = Root.GetUnitAxis(EAxis::Y);
	const FVector MidVec = Mid.GetLocation() - Root.GetLocation();
	const FVector CrossResult = FVector::CrossProduct(MidVec, LocalAxisY).GetSafeNormal();

	// If cross product produces a valid result, use it
	if (!CrossResult.IsNearlyZero())
	{
		return CrossResult;
	}

	// If still zero, fall back to world up vector
	return FVector::UpVector;
}

int32 UOHSkeletalPhysicsUtils::ValidateAndSortBoneTransforms(
	TArray<FBoneTransform>& OutBoneTransforms,
	bool bSortByIndex,
	bool bLogWarnings)
{
	// Early out if empty
	if (OutBoneTransforms.Num() == 0)
	{
		return 0;
	}

	// Track and remove invalid bone transforms
	int32 InvalidCount = 0;
	for (int32 i = OutBoneTransforms.Num() - 1; i >= 0; --i)
	{
		const FBoneTransform& BT = OutBoneTransforms[i];

		// Check for invalid rotation
		if (!BT.Transform.GetRotation().IsNormalized())
		{
			if (bLogWarnings)
			{
				UE_LOG(LogTemp, Warning,
				       TEXT("ValidateAndSortBoneTransforms: Transform at index %d has invalid rotation"), i);
			}
			OutBoneTransforms.RemoveAt(i);
			InvalidCount++;
			continue;
		}

		// Check for NaN or invalid values in transform
		if (BT.Transform.ContainsNaN())
		{
			if (bLogWarnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("ValidateAndSortBoneTransforms: Transform at index %d contains NaN"), i);
			}
			OutBoneTransforms.RemoveAt(i);
			InvalidCount++;
			continue;
		}

		// Check for invalid bone index
		if (!BT.BoneIndex.IsValid())
		{
			if (bLogWarnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("ValidateAndSortBoneTransforms: Invalid bone index at index %d"), i);
			}
			OutBoneTransforms.RemoveAt(i);
			InvalidCount++;
		}
	}

	// Sort by bone index if requested and if there are transforms left
	if (bSortByIndex && OutBoneTransforms.Num() > 1)
	{
		// In-place sort by bone index
		OutBoneTransforms.Sort([](const FBoneTransform& A, const FBoneTransform& B)
		{
			return A.BoneIndex < B.BoneIndex;
		});

		// Verify sort correctness in debug builds
#if !UE_BUILD_SHIPPING
		bool bSortValid = true;
		for (int32 i = 1; i < OutBoneTransforms.Num(); ++i)
		{
			if (OutBoneTransforms[i].BoneIndex < OutBoneTransforms[i - 1].BoneIndex)
			{
				bSortValid = false;
				break;
			}
		}

		if (!bSortValid && bLogWarnings)
		{
			UE_LOG(LogTemp, Error,
			       TEXT("ValidateAndSortBoneTransforms: Sorting failed! This indicates a deeper issue."));
		}
#endif
	}

	return InvalidCount;
}


FVector UOHSkeletalPhysicsUtils::MirrorPointAcrossAxes(
	const FVector& Point,
	const FVector& Pivot,
	const FVector& MirrorAxis
)
{
	// Compute vector from pivot
	FVector Rel = Point - Pivot;

	// Build scale vector: if MirrorAxis component is non-zero, flip that axis
	FVector Scale(
		FMath::Abs(MirrorAxis.X) > KINDA_SMALL_NUMBER ? -1.f : 1.f,
		FMath::Abs(MirrorAxis.Y) > KINDA_SMALL_NUMBER ? -1.f : 1.f,
		FMath::Abs(MirrorAxis.Z) > KINDA_SMALL_NUMBER ? -1.f : 1.f
	);

	// Apply and reconstruct world-space point
	return Pivot + (Rel * Scale);
}


// ------------------------------- Math Utilities and Physics Computation ------------------------------ //


///////////////////////////////////////////////////////
// Chain length

float UOHSkeletalPhysicsUtils::ComputeBoneChainLength(
	USkeletalMeshComponent* SkelComp,
	const TArray<FName>& BoneNames
)
{
	if (!SkelComp)
	{
		return 0.f;
	}
	float Total = 0.f;
	TArray<FVector> Positions;
	GetBoneChainWorldPositions(SkelComp, BoneNames, Positions);
	for (int32 i = 1; i < Positions.Num(); ++i)
	{
		Total += FVector::Dist(Positions[i - 1], Positions[i]);
	}
	return Total;
}

void UOHSkeletalPhysicsUtils::GetBoneChainWorldPositions(
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames,
	TArray<FVector>& OutPositions
)
{
	OutPositions.Reset(BoneNames.Num());
	if (!SkelComp)
	{
		return;
	}
	for (auto& Name : BoneNames)
	{
		OutPositions.Add(SkelComp->GetBoneLocation(Name));
	}
}

FVector UOHSkeletalPhysicsUtils::GetClosestPointOnBoneChain(
	TArray<FVector> ChainPoints,
	FVector Point
)
{
	FVector BestPoint = Point;
	float BestDistSqr = FLT_MAX;
	for (int32 i = 1; i < ChainPoints.Num(); ++i)
	{
		FVector P = FMath::ClosestPointOnSegment(
			Point, ChainPoints[i - 1], ChainPoints[i]
		);
		float DS = FVector::DistSquared(P, Point);
		if (DS < BestDistSqr)
		{
			BestDistSqr = DS;
			BestPoint = P;
		}
	}
	return BestPoint;
}

///////////////////////////////////////////////////////
// Collision sweep

bool UOHSkeletalPhysicsUtils::SweepCapsuleAlongBoneChain(
	UWorld* World,
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames,
	float Radius,
	TArray<FHitResult>& OutHits
)
{
	if (!World || !SkelComp || BoneNames.Num() < 2)
	{
		return false;
	}
	bool bAnyHit = false;

	for (int32 i = 1; i < BoneNames.Num(); ++i)
	{
		FVector Start = SkelComp->GetBoneLocation(BoneNames[i - 1]);
		FVector End = SkelComp->GetBoneLocation(BoneNames[i]);

		FCollisionShape Capsule = FCollisionShape::MakeCapsule(Radius, 0.f);
		FHitResult Hit;
		if (World->SweepSingleByChannel(
			Hit, Start, End, FQuat::Identity,
			ECC_PhysicsBody, Capsule
		))
		{
			OutHits.Add(Hit);
			bAnyHit = true;
		}
	}

	return bAnyHit;
}

///////////////////////////////////////////////////////
// Center of mass

FVector UOHSkeletalPhysicsUtils::ComputeBoneChainCenterOfMass(
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames
)
{
	if (!SkelComp)
	{
		return FVector::ZeroVector;
	}
	FVector Sum(0, 0, 0);
	int32 Count = 0;

	for (auto& Name : BoneNames)
	{
		if (FBodyInstance* BI = SkelComp->GetBodyInstance(Name))
		{
			Sum += BI->GetCOMPosition();
			++Count;
		}
	}

	return (Count > 0) ? (Sum / Count) : SkelComp->GetComponentLocation();
}

///////////////////////////////////////////////////////
// Prediction

void UOHSkeletalPhysicsUtils::PredictBoneChainPositions(
	TArray<FVector> CurrentPositions,
	TArray<FVector> Velocities,
	float DeltaTime,
	TArray<FVector>& OutPredicted
)
{
	int32 N = FMath::Min(CurrentPositions.Num(), Velocities.Num());
	OutPredicted.Reset(N);
	for (int32 i = 0; i < N; ++i)
	{
		OutPredicted.Add(CurrentPositions[i] + Velocities[i] * DeltaTime);
	}
}

///////////////////////////////////////////////////////
// Swing/twist decomposition

void UOHSkeletalPhysicsUtils::DecomposeSwingTwist(
	FQuat Rotation,
	FVector TwistAxis,
	FQuat& OutSwing,
	FQuat& OutTwist
)
{
	FVector Ra = TwistAxis.GetSafeNormal();
	FQuat R = Rotation;
	// Project rotation axis onto twist axis
	FVector RotAxis(R.X, R.Y, R.Z);
	FVector Proj = FVector::DotProduct(RotAxis, Ra) * Ra;
	OutTwist = FQuat(Proj.X, Proj.Y, Proj.Z, R.W).GetNormalized();
	OutSwing = (R * OutTwist.Inverse()).GetNormalized();
}

///////////////////////////////////////////////////////
// BoneEnum velocity

FVector UOHSkeletalPhysicsUtils::ComputeBoneVelocity(
	USkeletalMeshComponent* SkelComp,
	const FName BoneName,
	const float DeltaTime
)
{
	if (!SkelComp || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}
	static TMap<FName, FVector> PrevPosMap;
	const FVector Current = SkelComp->GetBoneLocation(BoneName);
	const FVector Prev = PrevPosMap.FindRef(BoneName);
	const FVector Vel = (Current - Prev) / DeltaTime;
	PrevPosMap.Add(BoneName, Current);
	return Vel;
}

FVector UOHSkeletalPhysicsUtils::ComputeAngularVelocity(
	FQuat PrevRotation,
	FQuat CurrRotation,
	float DeltaTime
)
{
	if (DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}
	FQuat Delta = CurrRotation * PrevRotation.Inverse();
	// Extract axis-angle from Delta
	FVector Axis;
	float Angle;
	Delta.ToAxisAndAngle(Axis, Angle);
	return Axis * (Angle / DeltaTime);
}


// 1) Predict the time until a bone collides along its velocity vector
float UOHSkeletalPhysicsUtils::PredictBoneTimeToCollision(
	USkeletalMeshComponent* SkelComp,
	const FName BoneName,
	FVector Velocity,
	const float MaxTime,
	const ECollisionChannel Channel,
	FHitResult& OutHit
)
{
	if (!SkelComp || Velocity.IsNearlyZero())
	{
		OutHit = FHitResult();
		return MaxTime;
	}

	UWorld* World = SkelComp->GetWorld();
	const FVector Start = SkelComp->GetBoneLocation(BoneName);
	const FVector End = Start + Velocity.GetSafeNormal() * Velocity.Size() * MaxTime;

	bool bHit = World->LineTraceSingleByChannel(
		OutHit,
		Start, End,
		Channel,
		FCollisionQueryParams(NAME_None, false, SkelComp->GetOwner())
	);

	if (bHit && OutHit.Distance > 0.f && Velocity.Size() > 0.f)
	{
		// time = distance_along_ray / speed
		return FMath::Clamp(OutHit.Distance / Velocity.Size(), 0.f, MaxTime);
	}
	return MaxTime;
}

// 2) Compute a reaction impulse vector from a collision hit
FVector UOHSkeletalPhysicsUtils::ComputeReactiveImpulseFromHit(
	const FHitResult& Hit,
	float ImpactStrength
)
{
	// Impulse away from surface normal, scaled
	return Hit.ImpactNormal * ImpactStrength;
}

// 3) Test whether any bone in the chain overlaps any physics body
bool UOHSkeletalPhysicsUtils::IsBoneChainIntersecting(
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames,
	const float SphereRadius,
	const ECollisionChannel Channel
)
{
	if (!SkelComp)
	{
		return false;
	}
	UWorld* World = SkelComp->GetWorld();

	for (FName Bone : BoneNames)
	{
		FVector P = SkelComp->GetBoneLocation(Bone);
		TArray<FOverlapResult> Overlaps;
		bool bOverlap = World->OverlapAnyTestByChannel(
			P,
			FQuat::Identity,
			Channel,
			FCollisionShape::MakeSphere(SphereRadius),
			FCollisionQueryParams(NAME_None, false, SkelComp->GetOwner())
		);
		if (bOverlap)
		{
			return true;
		}
	}
	return false;
}

// 4) Find the closest point on a capsule defined by Start,End,Radius
FVector UOHSkeletalPhysicsUtils::ProjectPointOntoBoneCapsule(
	FVector Start,
	FVector End,
	const float Radius,
	FVector Point
)
{
	// Closest point on line segment
	FVector Closest = UKismetMathLibrary::FindClosestPointOnSegment(Point, Start, End);
	FVector Dir = (Point - Closest).GetSafeNormal();
	// Project out to capsule surface
	return Closest + Dir * Radius;
}

// 5) Blend weight based on distance to collision point
float UOHSkeletalPhysicsUtils::ComputeCollisionBlendWeight(
	FVector BonePos,
	FVector CollisionPos,
	float DistanceThreshold
)
{
	float Dist = FVector::Dist(BonePos, CollisionPos);
	if (DistanceThreshold <= KINDA_SMALL_NUMBER)
	{
		return 0.f;
	}
	return FMath::Clamp(1.f - (Dist / DistanceThreshold), 0.f, 1.f);
}

// 6) Approximate which segment of a moving sphere will collide first
void UOHSkeletalPhysicsUtils::PredictChainCollisionSegment(
	const TArray<FVector>& ChainPositions,
	FVector SphereCenter,
	float Radius,
	FVector Velocity,
	float MaxTime,
	float& OutHitTime,
	int32& OutSegmentIndex,
	FVector& OutHitPoint
)
{
	OutHitTime = MaxTime;
	OutSegmentIndex = -1;
	OutHitPoint = SphereCenter + Velocity * MaxTime;

	if (ChainPositions.Num() < 2 || Velocity.IsNearlyZero())
	{
		return;
	}

	constexpr int32 Steps = 20;
	for (int32 i = 1; i < ChainPositions.Num(); ++i)
	{
		// For each segment, sample along the sphere's path
		FVector A = ChainPositions[i - 1];
		FVector B = ChainPositions[i];
		for (int32 s = 1; s <= Steps; ++s)
		{
			float T = MaxTime * (static_cast<float>(s) / Steps);
			FVector Cpos = SphereCenter + Velocity * T;
			float Dist2 = FMath::PointDistToSegmentSquared(Cpos, A, B);
			if (Dist2 <= Radius * Radius && T < OutHitTime)
			{
				OutHitTime = T;
				OutSegmentIndex = i - 1;
				// Compute exact closest point
				OutHitPoint = FMath::ClosestPointOnSegment(Cpos, A, B);
				return;
			}
		}
	}
}

// 7) Generate capsule transforms (midpoint+half-length) for chain motion
void UOHSkeletalPhysicsUtils::GenerateBoneChainMotionCapsules(
	TArray<FVector> CurrentPositions,
	TArray<FVector> PredictedPositions,
	float Radius,
	TArray<FTransform>& OutCapsuleTransforms
)
{
	OutCapsuleTransforms.Reset();

	const int32 N = FMath::Min(CurrentPositions.Num(), PredictedPositions.Num());
	for (int32 i = 1; i < N; ++i)
	{
		const FVector Start = CurrentPositions[i - 1];
		const FVector End = PredictedPositions[i];
		const FVector Delta = End - Start;
		const float HalfLen = Delta.Size() * 0.5f;
		if (HalfLen <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		// Midpoint
		const FVector Midpoint = (Start + End) * 0.5f;

		// Rotation: align local Z (UpVector) to Delta direction
		const FQuat Rotation = FQuat::FindBetweenNormals(FVector::UpVector, Delta.GetSafeNormal());

		// Scale: (Radius, Radius, HalfLength)
		const FVector Scale(Radius, Radius, HalfLen);

		// Unreal scales a capsule by X=Radius, Y=HalfHeight, so we swap Z/Y
		const FVector FinalScale(Scale.X, Scale.Z, Scale.Y);

		OutCapsuleTransforms.Add(FTransform(Rotation, Midpoint, FinalScale));
	}
}

// 8) Build a map of bone masses from the PhysicsAsset
void UOHSkeletalPhysicsUtils::ComputeBoneMassMap(
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames,
	TMap<FName, float>& OutBoneMass
)
{
	OutBoneMass.Reset();
	if (!SkelComp)
	{
		return;
	}

	for (const FName& BoneName : BoneNames)
	{
		float Mass = 0.f;

		// Query the runtime BodyInstance for this bone
		if (FBodyInstance* BI = SkelComp->GetBodyInstance(BoneName))
		{
			// GetBodyMass() returns the mass in kg
			Mass = BI->GetBodyMass();
		}

		OutBoneMass.Add(BoneName, Mass);
	}
}

// 9) Blend reactive pose transforms into the output array
void UOHSkeletalPhysicsUtils::BlendReactivePose(
	const TArray<FBoneTransform>& ReactiveTransforms,
	TArray<FBoneTransform>& OutBoneTransforms,
	float BlendWeight
)
{
	const float Alpha = FMath::Clamp(BlendWeight, 0.f, 1.f);

	for (const FBoneTransform& RT : ReactiveTransforms)
	{
		bool bMerged = false;

		// Try to merge into an existing transform
		for (FBoneTransform& OT : OutBoneTransforms)
		{
			if (OT.BoneIndex == RT.BoneIndex)
			{
				const FTransform& A = OT.Transform;
				const FTransform& B = RT.Transform;

				// Blend location
				FVector Loc = FMath::Lerp(A.GetLocation(), B.GetLocation(), Alpha);
				// Static slerp: (quat1, quat2, alpha)
				FQuat Rot = FQuat::Slerp(A.GetRotation(), B.GetRotation(), Alpha);
				// Blend scale
				FVector Scale = FMath::Lerp(A.GetScale3D(), B.GetScale3D(), Alpha);

				OT.Transform = FTransform(Rot, Loc, Scale);
				bMerged = true;
				break;
			}
		}

		// If not found, add a new entry (only if alpha > 0)
		if (!bMerged && Alpha > 0.f)
		{
			const FTransform& B = RT.Transform;
			FVector Loc = B.GetLocation() * Alpha;
			FQuat Rot = FQuat::Slerp(FQuat::Identity, B.GetRotation(), Alpha);
			FVector Scale = FVector::OneVector + (B.GetScale3D() - FVector::OneVector) * Alpha;

			OutBoneTransforms.Add(FBoneTransform(RT.BoneIndex, FTransform(Rot, Loc, Scale)));
		}
	}
}

// Predictive target offset based on current velocity and a directional multiplier 
FVector UOHSkeletalPhysicsUtils::ComputeVelocityBasedTarget(
	FVector HandPosition,
	FVector Velocity,
	const float CompressionThreshold
)
{
	// Safe normalize velocity
	FVector Dir = Velocity.SizeSquared() > KINDA_SMALL_NUMBER
		              ? Velocity.GetSafeNormal()
		              : FVector::ZeroVector;
	return HandPosition - Dir * CompressionThreshold;
}


// Simple linear blend between two targets
FVector UOHSkeletalPhysicsUtils::BlendTargets(
	FVector A,
	FVector B,
	const float Alpha
)
{
	return FMath::Lerp(A, B, FMath::Clamp(Alpha, 0.f, 1.f));
}

// Move a point forward along its velocity for a given delta time
FVector UOHSkeletalPhysicsUtils::ApplyPredictiveOffset(
	FVector BaseTarget,
	FVector Velocity,
	float PredictiveOffset
)
{
	return BaseTarget + Velocity * PredictiveOffset;
}

// Mirror a vector across a normalized axis
FVector UOHSkeletalPhysicsUtils::MirrorVectorAcrossAxis(
	FVector Vector,
	FVector MirrorAxis
)
{
	// Flip components where MirrorAxis component is non-zero
	return FVector(
		FMath::Abs(MirrorAxis.X) > KINDA_SMALL_NUMBER ? -Vector.X : Vector.X,
		FMath::Abs(MirrorAxis.Y) > KINDA_SMALL_NUMBER ? -Vector.Y : Vector.Y,
		FMath::Abs(MirrorAxis.Z) > KINDA_SMALL_NUMBER ? -Vector.Z : Vector.Z
	);
}

// Evaluate a UCurveFloat safely, with a default fallback
float UOHSkeletalPhysicsUtils::EvaluateCurveAtAlpha(
	const UCurveFloat* Curve,
	float InputAlpha,
	float DefaultValue
)
{
	return (Curve != nullptr)
		       ? Curve->GetFloatValue(InputAlpha)
		       : DefaultValue;
}

// Clamp a normalized alpha
float UOHSkeletalPhysicsUtils::ClampAlpha(
	float Alpha,
	float MinAlpha,
	float MaxAlpha
)
{
	return FMath::Clamp(Alpha, MinAlpha, MaxAlpha);
}

// Compute a falloff factor (1 at zero distance, 0 at or beyond Radius)
float UOHSkeletalPhysicsUtils::ComputeFalloffAlpha(
	float Distance,
	float Radius
)
{
	if (Radius <= KINDA_SMALL_NUMBER)
	{
		return 1.f;
	}
	return FMath::Clamp(1.f - (Distance / Radius), 0.f, 1.f);
}

// Apply a dead-zone around zero alpha, remapping [DeadZone..1] → [0..1]
float UOHSkeletalPhysicsUtils::ComputeSoftZoneAlpha(
	float Alpha,
	float SoftZoneOffset
)
{
	if (SoftZoneOffset > KINDA_SMALL_NUMBER)
	{
		return FMath::Clamp((Alpha - SoftZoneOffset) / (1.f - SoftZoneOffset), 0.f, 1.f);
	}
	return Alpha;
}


// Depth of penetration for a point into a capsule defined by two endpoints
float UOHSkeletalPhysicsUtils::ComputeCapsulePenetrationDepth(
	FVector Point,
	FVector CapsuleStart,
	FVector CapsuleEnd,
	float Radius
)
{
	FVector Closest = FMath::ClosestPointOnSegment(Point, CapsuleStart, CapsuleEnd);
	float Dist = FVector::Dist(Point, Closest);
	return FMath::Max(0.f, Radius - Dist);
}

// Direction & magnitude to push a point out of a capsule
FVector UOHSkeletalPhysicsUtils::ComputePenetrationCorrection(
	FVector Point,
	FVector CapsuleStart,
	FVector CapsuleEnd,
	float Radius
)
{
	FVector Closest = FMath::ClosestPointOnSegment(Point, CapsuleStart, CapsuleEnd);
	FVector Dir = Point - Closest;
	float Len = Dir.Size();
	if (Len <= KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}
	float Depth = Radius - Len;
	return Depth > 0.f ? Dir.GetSafeNormal() * Depth : FVector::ZeroVector;
}


// Average Normal from a list of hit‐results
FVector UOHSkeletalPhysicsUtils::ComposeCollisionNormal(TArray<FHitResult> Hits)
{
	FVector Sum = FVector::ZeroVector;
	for (const FHitResult& H : Hits)
	{
		Sum += H.ImpactNormal;
	}
	return Hits.Num()
		       ? Sum.GetSafeNormal()
		       : FVector::UpVector;
}


// Reflect velocity about a collision normal with restitution
FVector UOHSkeletalPhysicsUtils::ComputeBounceVelocity(
	FVector VelocityIn,
	FVector Normal,
	float RestitutionCoefficient
)
{
	const FVector V = VelocityIn;
	const FVector N = Normal.GetSafeNormal();
	return V - (1.f + RestitutionCoefficient) * FVector::DotProduct(V, N) * N;
}

// Stubbed chain‐stiffness map (per-bone uniform stiffness)
void UOHSkeletalPhysicsUtils::ComputeChainStiffnessMap(
	UPhysicsAsset* /*PhysAsset*/,
	TArray<FName> BoneNames,
	float StiffnessPerBone,
	TMap<FName, float>& OutStiffnessMap
)
{
	OutStiffnessMap.Reset();
	for (const FName& Bone : BoneNames)
	{
		OutStiffnessMap.Add(Bone, StiffnessPerBone);
	}
}

// Simple sweep per-bone, returns first hit Time (0–1) or –1 if none
float UOHSkeletalPhysicsUtils::PredictCapsuleChainTimeToImpact(
	UWorld* World,
	USkeletalMeshComponent* SkelComp,
	TArray<FName> BoneNames,
	FVector CapsuleVelocity,
	float CapsuleRadius,
	float MaxTime
)
{
	if (!World || !SkelComp)
	{
		return -1.f;
	}

	const float Speed = CapsuleVelocity.Size();
	if (Speed <= KINDA_SMALL_NUMBER || BoneNames.Num() < 1)
	{
		return -1.f;
	}

	const FVector Dir = CapsuleVelocity / Speed;
	// We'll sweep a sphere along each bone's world position
	FCollisionShape Shape = FCollisionShape::MakeCapsule(CapsuleRadius, 0.f);

	float ClosestTime = MaxTime;
	bool bFoundHit = false;

	for (const FName& Bone : BoneNames)
	{
		// Start at the bone's current location
		const FVector Start = SkelComp->GetBoneLocation(Bone);
		// End = Start + Dir * Speed * MaxTime
		const FVector End = Start + Dir * Speed * MaxTime;

		FHitResult Hit;
		// SweepSingleByChannel will fill Hit.Time (0–1) and Hit.Distance
		const bool bHit = World->SweepSingleByChannel(
			Hit,
			Start,
			End,
			FQuat::Identity,
			ECC_PhysicsBody,
			Shape,
			FCollisionQueryParams(NAME_None, /*bTraceComplex=*/false, SkelComp->GetOwner())
		);

		if (bHit && Hit.Distance > KINDA_SMALL_NUMBER)
		{
			// Convert distance-to-impact into seconds: time = distance / speed
			const float HitTime = Hit.Distance / Speed;
			ClosestTime = FMath::Min(ClosestTime, HitTime);
			bFoundHit = true;
		}
	}

	// If we never hit anything, return -1; otherwise earliest impact time
	return bFoundHit ? ClosestTime : -1.f;
}

// Exponential decay curve for collisions
float UOHSkeletalPhysicsUtils::ComputeCollisionDecay(
	float Time,
	float DecayRate
)
{
	return FMath::Exp(-DecayRate * Time);
}


float UOHSkeletalPhysicsUtils::GetArmChainLength(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm,
                                                 FName Hand)
{
	if (!Mesh)
	{
		return 0.f;
	}

	const FVector A = Mesh->GetBoneLocation(UpperArm);
	const FVector B = Mesh->GetBoneLocation(LowerArm);
	const FVector C = Mesh->GetBoneLocation(Hand);

	return FVector::Distance(A, B) + FVector::Distance(B, C);
}

FVector UOHSkeletalPhysicsUtils::GetArmEffectorDirection(USkeletalMeshComponent* Mesh, FName UpperArm, FVector Effector)
{
	if (!Mesh)
	{
		return FVector::ForwardVector;
	}
	return (Effector - Mesh->GetBoneLocation(UpperArm)).GetSafeNormal();
}

float UOHSkeletalPhysicsUtils::GetArmCompressionAlpha(float Distance, float Min, float Max)
{
	if (FMath::IsNearlyEqual(Min, Max))
	{
		return 1.0f;
	}
	return FMath::Clamp((Distance - Min) / (Max - Min), 0.f, 1.f);
}

void UOHSkeletalPhysicsUtils::DebugDrawArmIK(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm, FName Hand,
                                             FVector EffectorTarget, float MinLen, float MaxLen, float CurrentAlpha,
                                             FLinearColor Color, float Duration)
{
	if (!Mesh)
	{
		return;
	}

	const FVector A = Mesh->GetBoneLocation(UpperArm);
	const FVector B = Mesh->GetBoneLocation(LowerArm);
	const FVector C = Mesh->GetBoneLocation(Hand);

	DrawDebugLine(Mesh->GetWorld(), A, B, FColor::Red, false, Duration, 0, 2.f);
	DrawDebugLine(Mesh->GetWorld(), B, C, FColor::Blue, false, Duration, 0, 2.f);
	DrawDebugLine(Mesh->GetWorld(), A, EffectorTarget, Color.ToFColor(true), false, Duration, 0, 1.f);

	const float CurrentLen = FVector::Distance(A, EffectorTarget);
	const FString Label = FString::Printf(
		TEXT("IK Len: %.1f [%.1f - %.1f]  Alpha: %.2f"), CurrentLen, MinLen, MaxLen, CurrentAlpha);

	DrawDebugString(Mesh->GetWorld(), A + FVector(0, 0, 20), Label, nullptr, Color.ToFColor(true), Duration, false);
}

FVector UOHSkeletalPhysicsUtils::PredictEffectorTarget(USkeletalMeshComponent* Mesh, FName StartBone, FVector Velocity,
                                                       float Distance)
{
	if (!Mesh)
	{
		return FVector::ZeroVector;
	}
	FVector Start = Mesh->GetBoneLocation(StartBone);
	return Start + Velocity.GetSafeNormal() * Distance;
}


float UOHSkeletalPhysicsUtils::PredictCompressionTrend(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm,
                                                       FName Hand, float DeltaTime)
{
	if (!Mesh || DeltaTime <= 0.f)
	{
		return 0.f;
	}

	const FVector Prev = Mesh->GetBoneLocation(Hand) - Mesh->GetBoneLocation(UpperArm);
	const FVector Curr = Prev + (Mesh->GetComponentVelocity() * DeltaTime);

	float PrevLen = Prev.Size();
	float CurrLen = Curr.Size();
	return (CurrLen - PrevLen) / DeltaTime;
}

bool UOHSkeletalPhysicsUtils::IsBoneOverlapping(USkeletalMeshComponent* Mesh, FName BoneName, float Radius)
{
	if (!Mesh)
	{
		return false;
	}

	FVector BoneLoc = Mesh->GetBoneLocation(BoneName);
	FCollisionShape Shape = FCollisionShape::MakeSphere(Radius);
	return Mesh->GetWorld()->OverlapAnyTestByChannel(BoneLoc, FQuat::Identity, ECC_Pawn, Shape);
}

bool UOHSkeletalPhysicsUtils::PredictBoneCollision(USkeletalMeshComponent* Mesh, FName BoneName, FVector Velocity,
                                                   float Distance, FHitResult& OutHit)
{
	if (!Mesh)
	{
		return false;
	}
	FVector Start = Mesh->GetBoneLocation(BoneName);
	FVector End = Start + Velocity.GetSafeNormal() * Distance;

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Mesh->GetOwner());
	return Mesh->GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, ECC_Pawn, Params);
}


bool UOHSkeletalPhysicsUtils::TracePunchCollision(USkeletalMeshComponent* Mesh, FName HandBone, FVector Direction,
                                                  float Length, FHitResult& OutHit)
{
	if (!Mesh)
	{
		return false;
	}
	FVector Start = Mesh->GetBoneLocation(HandBone);
	FVector End = Start + Direction.GetSafeNormal() * Length;

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Mesh->GetOwner());
	return Mesh->GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, ECC_Visibility, Params);
}


void UOHSkeletalPhysicsUtils::SyncIKFromArmPose(UObject* AnimInstance, FName UpperArm, FName LowerArm, FName Hand,
                                                float MinLength, float MaxLength)
{
	if (!AnimInstance)
	{
		return;
	}
	USkeletalMeshComponent* Mesh = Cast<USkeletalMeshComponent>(AnimInstance->GetOuter());
	if (!Mesh)
	{
		return;
	}

	FVector A = Mesh->GetBoneLocation(UpperArm);
	FVector C = Mesh->GetBoneLocation(Hand);
	float CurrentLen = FVector::Dist(A, C);
	float Alpha = GetArmCompressionAlpha(CurrentLen, MinLength, MaxLength);

	// Assuming UOHAnimInstance subclass with setter method
	if (UOHAnimInstance* OHAnim = Cast<UOHAnimInstance>(AnimInstance))
	{
		OHAnim->SetCompressionAlpha(Alpha);
		OHAnim->SetCompressionEffectorTarget(C);
	}
}


void UOHSkeletalPhysicsUtils::TrackImpactCamera(UObject* WorldContext, FVector ImpactLocation, float Duration)
{
	if (!WorldContext)
	{
		return;
	}
	UWorld* World = WorldContext->GetWorld();
	if (!World)
	{
		return;
	}

	DrawDebugSphere(World, ImpactLocation, 12.f, 16, FColor::Orange, false, Duration);
	DrawDebugString(World, ImpactLocation + FVector(0, 0, 20), TEXT("Impact!"), nullptr, FColor::Yellow, Duration);
}

FVector UOHSkeletalPhysicsUtils::ComputeOverstretchedEffector(FVector Origin, FVector Target, float StretchRatio)
{
	FVector Direction = (Target - Origin).GetSafeNormal();
	float Distance = FVector::Distance(Origin, Target);
	return Origin + Direction * (Distance * StretchRatio);
}

FTransform UOHSkeletalPhysicsUtils::OffsetTransform(const FTransform& Base, const FVector& TranslationOffset,
                                                    const FRotator& RotationOffset)
{
	FTransform Result = Base;
	Result.AddToTranslation(TranslationOffset);
	Result.ConcatenateRotation(RotationOffset.Quaternion());
	return Result;
}

FTransform UOHSkeletalPhysicsUtils::InterpTransform(const FTransform& A, const FTransform& B, float Alpha)
{
	return FTransform(
		FMath::Lerp(A.GetRotation(), B.GetRotation(), Alpha),
		FMath::Lerp(A.GetLocation(), B.GetLocation(), Alpha),
		FMath::Lerp(A.GetScale3D(), B.GetScale3D(), Alpha)
	);
}


float UOHSkeletalPhysicsUtils::GetChainLengthFromBoneEnums(
	USkeletalMeshComponent* Mesh,
	const TArray<EOHSkeletalBone>& BoneChain)
{
	if (!Mesh || BoneChain.Num() < 2)
	{
		return 0.f;
	}
	float Total = 0.f;
	for (int32 i = 0; i < BoneChain.Num() - 1; ++i)
	{
		FVector A = Mesh->GetBoneLocation(GetBoneNameFromEnum(BoneChain[i]));
		FVector B = Mesh->GetBoneLocation(GetBoneNameFromEnum(BoneChain[i + 1]));
		Total += FVector::Distance(A, B);
	}
	return Total;
}


float UOHSkeletalPhysicsUtils::GetLegCompressionAlpha(
	USkeletalMeshComponent* Mesh,
	EOHSkeletalBone Thigh,
	EOHSkeletalBone Calf,
	EOHSkeletalBone Foot,
	float MinLength,
	float MaxLength)
{
	const FName A = GetBoneNameFromEnum(Thigh);
	const FName B = GetBoneNameFromEnum(Calf);
	const FName C = GetBoneNameFromEnum(Foot);

	float Length = FVector::Dist(Mesh->GetBoneLocation(A), Mesh->GetBoneLocation(B)) +
		FVector::Dist(Mesh->GetBoneLocation(B), Mesh->GetBoneLocation(C));

	return FMath::Clamp((Length - MinLength) / (MaxLength - MinLength), 0.f, 1.f);
}


float UOHSkeletalPhysicsUtils::GetSpineBendRatio(
	USkeletalMeshComponent* Mesh,
	EOHSkeletalBone SpineStart,
	EOHSkeletalBone SpineMid,
	EOHSkeletalBone SpineEnd)
{
	FVector A = Mesh->GetBoneLocation(GetBoneNameFromEnum(SpineStart));
	FVector B = Mesh->GetBoneLocation(GetBoneNameFromEnum(SpineMid));
	FVector C = Mesh->GetBoneLocation(GetBoneNameFromEnum(SpineEnd));

	FVector DirStart = (B - A).GetSafeNormal();
	FVector DirEnd = (C - B).GetSafeNormal();

	return FVector::DotProduct(DirStart, DirEnd); // 1 = straight, 0 = 90°, -1 = folded
}
#pragma endregion 

void UOHSkeletalPhysicsUtils::DrawDebugBoneChainByName(const USkeletalMeshComponent* Mesh, const FName& ParentBoneName, const FName& ChildBoneName, FColor Color, float Duration)
{
	if (!Mesh) return;

	TArray<FName> BoneChain = GetBoneChainBetweenByName(Mesh, ParentBoneName, ChildBoneName);
	const UWorld* World = Mesh->GetWorld();

	for (int32 i = 0; i < BoneChain.Num(); ++i)
	{
		const FVector Pos = Mesh->GetSocketLocation(BoneChain[i]);
		DrawDebugSphere(World, Pos, 4.0f, 8, Color, false, Duration);

		// Draw lines to next bone in chain
		if (i + 1 < BoneChain.Num())
		{
			const FVector NextPos = Mesh->GetSocketLocation(BoneChain[i + 1]);
			DrawDebugLine(World, Pos, NextPos, Color, false, Duration, 0, 1.5f);
		}

		// Label bone
		const FString Label = BoneChain[i].ToString();
		DrawDebugString(World, Pos + FVector(0, 0, 6), Label, nullptr, Color, Duration, false);
	}
}

TArray<FName> UOHSkeletalPhysicsUtils::GetBoneChainReverseByName(const USkeletalMeshComponent* Mesh, const FName& StartingBone)
{
	TArray<FName> Chain;

	if (!Mesh || !Mesh->DoesSocketExist(StartingBone))
		return Chain;

	const USkeleton* Skeleton = Mesh->GetSkeletalMeshAsset() ? Mesh->GetSkeletalMeshAsset()->GetSkeleton() : nullptr;
	if (!Skeleton)
		return Chain;

	const FReferenceSkeleton& RefSkeleton = Skeleton->GetReferenceSkeleton();

	int32 CurrentIndex = RefSkeleton.FindBoneIndex(StartingBone);
	while (CurrentIndex != INDEX_NONE)
	{
		Chain.Add(RefSkeleton.GetBoneName(CurrentIndex));
		CurrentIndex = RefSkeleton.GetParentIndex(CurrentIndex);
	}

	return Chain;
}

TArray<FName> UOHSkeletalPhysicsUtils::GetBonesInChain(EOHSkeletalBone Bone)
{
	TArray<FName> BoneNames;

	// Reuse the existing lineage function to walk up to the “None” root.
	TArray<EOHSkeletalBone> Lineage = GetBoneLineageToRoot(Bone);
	for (EOHSkeletalBone EnumBone : Lineage)
	{
		// Convert the enum to its FName (lowercase, matching your skeleton naming)
		FName Name = ::GetBoneNameFromEnum(EnumBone);
		if (Name != NAME_None)
		{
			BoneNames.Add(Name);
		}
	}

	return BoneNames;
}



void UOHSkeletalPhysicsUtils::BuildValidatedBoneChain(
	EOHSkeletalBone EffectorEnum,
	const FBoneContainer& Bones,
	TArray<FName>& OutValidBoneNames
)
{
	OutValidBoneNames.Reset();

	// 1) Get the raw enum lineage (your existing GetBoneLineage)
	TArray<EOHSkeletalBone> Raw = GetBoneLineage(EffectorEnum, EOHSkeletalBone::None);

	// 2) Map each enum → FName and test if the skeleton actually has it
	for (EOHSkeletalBone E : Raw)
	{
		if (E == EOHSkeletalBone::None)
		{
			continue;
		}

		// Your existing mapping from enum to bone-name string
		FName BoneName = GetBoneNameFromEnum(E);
		if (BoneName == NAME_None)
		{
			continue;
		}

		// **The key change**: use GetPoseBoneIndexForBoneName
		const int32 PoseIndex = Bones.GetPoseBoneIndexForBoneName(BoneName);
		if (PoseIndex != INDEX_NONE)
		{
			OutValidBoneNames.Add(BoneName);
		}
	}
}


TArray<FName> UOHSkeletalPhysicsUtils::GetBoneChainBetweenByName(const USkeletalMeshComponent* Mesh, const FName& ParentBoneName, const FName& ChildBoneName)
{
	TArray<FName> Chain;

	if (!Mesh || !Mesh->DoesSocketExist(ParentBoneName) || !Mesh->DoesSocketExist(ChildBoneName))
	{
		return Chain;
	}

	const USkeleton* Skeleton = Mesh->GetSkeletalMeshAsset() ? Mesh->GetSkeletalMeshAsset()->GetSkeleton() : nullptr;
	if (!Skeleton)
	{
		return Chain;
	}

	const FReferenceSkeleton& RefSkeleton = Skeleton->GetReferenceSkeleton();

	int32 ParentIndex = RefSkeleton.FindBoneIndex(ParentBoneName);
	int32 ChildIndex = RefSkeleton.FindBoneIndex(ChildBoneName);

	if (ParentIndex == INDEX_NONE || ChildIndex == INDEX_NONE)
	{
		return Chain;
	}

	int32 CurrentIndex = ChildIndex;
	while (CurrentIndex != INDEX_NONE)
	{
		Chain.Insert(RefSkeleton.GetBoneName(CurrentIndex), 0);
		if (CurrentIndex == ParentIndex)
		{
			break;
		}
		CurrentIndex = RefSkeleton.GetParentIndex(CurrentIndex);
	}

	return Chain;
}



#pragma region PhysicsSimulation

bool UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBone(
    USkeletalMeshComponent* Mesh,
    FName BoneName,
    float BlendWeight)
{
    if (!Mesh)
        return false;
    if (BoneName.IsNone())
        return false;
    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (!BodyInst)
        return false;

    // Clamp and set
    float Clamped = FMath::Clamp(BlendWeight, 0.f, 1.f);
    BodyInst->PhysicsBlendWeight = Clamped;
    Mesh->RefreshBoneTransforms(); // Immediate visual update

    return true;
}

void UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBones(
    USkeletalMeshComponent* Mesh,
    const TArray<FName>& BoneNames,
    float BlendWeight)
{
    if (!Mesh || BoneNames.Num() == 0)
        return;

    float Clamped = FMath::Clamp(BlendWeight, 0.f, 1.f);
    for (const FName& BoneName : BoneNames)
    {
        if (BoneName.IsNone())
            continue;
        if (FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName))
            BodyInst->PhysicsBlendWeight = Clamped;
    }
    Mesh->RefreshBoneTransforms();
}

void UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBoneChain(
    USkeletalMeshComponent* Mesh,
    FName StartBone,
    FName EndBone,
    float BlendWeight)
{
    if (!Mesh || !Mesh->GetSkeletalMeshAsset())
        return;
    if (StartBone.IsNone() || EndBone.IsNone())
        return;

    const FReferenceSkeleton& RefSkel = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
    int32 StartIdx = Mesh->GetBoneIndex(StartBone);
    int32 EndIdx = Mesh->GetBoneIndex(EndBone);
    if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE)
        return;

    // Walk up from EndBone to StartBone
    TArray<int32> BoneIndices;
    int32 CurrIdx = EndIdx;
    while (CurrIdx != INDEX_NONE)
    {
        BoneIndices.Insert(CurrIdx, 0);
        if (CurrIdx == StartIdx)
            break;
        CurrIdx = RefSkel.GetParentIndex(CurrIdx);
    }
    if (BoneIndices.Num() == 0 || BoneIndices[0] != StartIdx)
        return; // Not a valid parent chain

    TArray<FName> ChainNames;
    for (int32 Idx : BoneIndices)
        ChainNames.Add(Mesh->GetBoneName(Idx));
    SetPhysicsBlendWeightForBones(Mesh, ChainNames, BlendWeight);
}

bool UOHSkeletalPhysicsUtils::EnablePhysicalAnimationForBone(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName BoneName,
    const FPhysicalAnimationData& Profile)
{
    if (!Mesh || !PAC)
        return false;
    if (BoneName.IsNone())
        return false;
    UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset();
    if (!PhysicsAsset)
        return false;
    if (PhysicsAsset->FindBodyIndex(BoneName) == INDEX_NONE)
        return false;
    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (!BodyInst)
        return false;

    PAC->ApplyPhysicalAnimationSettings(BoneName, Profile);
    BodyInst->SetInstanceSimulatePhysics(true);
    BodyInst->SetInstanceNotifyRBCollision(true);
    SetPhysicsBlendWeightForBone(Mesh, BoneName, 1.0f);
    BodyInst->WakeInstance();
    BodyInst->SetLinearVelocity(FVector::ZeroVector, false);
    BodyInst->SetAngularVelocityInRadians(FVector::ZeroVector, false);
    return true;
}

bool UOHSkeletalPhysicsUtils::DisablePhysicalAnimationForBone(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName BoneName)
{
    if (!Mesh || !PAC)
        return false;
    if (BoneName.IsNone())
        return false;
    UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset();
    if (!PhysicsAsset)
        return false;
    if (PhysicsAsset->FindBodyIndex(BoneName) == INDEX_NONE)
        return false;
    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (!BodyInst)
        return false;

    PAC->ApplyPhysicalAnimationSettings(BoneName, FPhysicalAnimationData());
    SetPhysicsBlendWeightForBone(Mesh, BoneName, 0.0f);
    BodyInst->SetInstanceSimulatePhysics(false);
    BodyInst->SetInstanceNotifyRBCollision(false);
    BodyInst->SetLinearVelocity(FVector::ZeroVector, false);
    BodyInst->SetAngularVelocityInRadians(FVector::ZeroVector, false);
    return true;
}

bool UOHSkeletalPhysicsUtils::EnablePhysicalAnimationForBones(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    const TArray<FName>& BoneNames,
    const FPhysicalAnimationData& Profile)
{
    if (!Mesh || !PAC || BoneNames.Num() == 0)
        return false;

    bool bAnySucceeded = false;
    for (const FName& BoneName : BoneNames)
    {
        if (EnablePhysicalAnimationForBone(Mesh, PAC, BoneName, Profile))
            bAnySucceeded = true;
    }
    return bAnySucceeded;
}

bool UOHSkeletalPhysicsUtils::DisablePhysicalAnimationForBones(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    const TArray<FName>& BoneNames)
{
    if (!Mesh || !PAC || BoneNames.Num() == 0)
        return false;

    bool bAnySucceeded = false;
    for (const FName& BoneName : BoneNames)
    {
        if (DisablePhysicalAnimationForBone(Mesh, PAC, BoneName))
            bAnySucceeded = true;
    }
    return bAnySucceeded;
}

bool UOHSkeletalPhysicsUtils::EnablePhysicalAnimationForBoneChain(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName StartBone,
    FName EndBone,
    const FPhysicalAnimationData& Profile)
{
    if (!Mesh || !Mesh->GetSkeletalMeshAsset() || !PAC)
        return false;
    if (StartBone.IsNone() || EndBone.IsNone())
        return false;

    const FReferenceSkeleton& RefSkel = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
    int32 StartIdx = Mesh->GetBoneIndex(StartBone);
    int32 EndIdx = Mesh->GetBoneIndex(EndBone);
    if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE)
        return false;

    // Walk up from EndBone to StartBone
    TArray<int32> BoneIndices;
    int32 CurrIdx = EndIdx;
    while (CurrIdx != INDEX_NONE)
    {
        BoneIndices.Insert(CurrIdx, 0);
        if (CurrIdx == StartIdx)
            break;
        CurrIdx = RefSkel.GetParentIndex(CurrIdx);
    }
    if (BoneIndices.Num() == 0 || BoneIndices[0] != StartIdx)
        return false;

    TArray<FName> ChainNames;
    for (int32 Idx : BoneIndices)
        ChainNames.Add(Mesh->GetBoneName(Idx));
    return EnablePhysicalAnimationForBones(Mesh, PAC, ChainNames, Profile);
}

bool UOHSkeletalPhysicsUtils::DisablePhysicalAnimationForBoneChain(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName StartBone,
    FName EndBone)
{
    if (!Mesh || !Mesh->GetSkeletalMeshAsset() || !PAC)
        return false;
    if (StartBone.IsNone() || EndBone.IsNone())
        return false;

    const FReferenceSkeleton& RefSkel = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
    int32 StartIdx = Mesh->GetBoneIndex(StartBone);
    int32 EndIdx = Mesh->GetBoneIndex(EndBone);
    if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE)
        return false;

    // Walk up from EndBone to StartBone
    TArray<int32> BoneIndices;
    int32 CurrIdx = EndIdx;
    while (CurrIdx != INDEX_NONE)
    {
        BoneIndices.Insert(CurrIdx, 0);
        if (CurrIdx == StartIdx)
            break;
        CurrIdx = RefSkel.GetParentIndex(CurrIdx);
    }
    if (BoneIndices.Num() == 0 || BoneIndices[0] != StartIdx)
        return false;

    TArray<FName> ChainNames;
    for (int32 Idx : BoneIndices)
        ChainNames.Add(Mesh->GetBoneName(Idx));
    return DisablePhysicalAnimationForBones(Mesh, PAC, ChainNames);
}

bool UOHSkeletalPhysicsUtils::EnablePhysicsSimulationForBone(
    USkeletalMeshComponent* Mesh,
    FName BoneName)
{
    if (!Mesh)
        return false;
    if (BoneName.IsNone())
        return false;
    UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset();
    if (!PhysicsAsset)
        return false;
    if (PhysicsAsset->FindBodyIndex(BoneName) == INDEX_NONE)
        return false;
    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (!BodyInst)
        return false;

    BodyInst->SetInstanceSimulatePhysics(true);
    BodyInst->SetInstanceNotifyRBCollision(true);
    SetPhysicsBlendWeightForBone(Mesh, BoneName, 1.0f); // Always use accessor!
    BodyInst->WakeInstance();
    BodyInst->SetLinearVelocity(FVector::ZeroVector, false);
    BodyInst->SetAngularVelocityInRadians(FVector::ZeroVector, false);

    return true;
}

bool UOHSkeletalPhysicsUtils::DisablePhysicsSimulationForBone(
    USkeletalMeshComponent* Mesh,
    FName BoneName)
{
    if (!Mesh)
        return false;
    if (BoneName.IsNone())
        return false;
    UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset();
    if (!PhysicsAsset)
        return false;
    if (PhysicsAsset->FindBodyIndex(BoneName) == INDEX_NONE)
        return false;
    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (!BodyInst)
        return false;

    BodyInst->SetInstanceSimulatePhysics(false);
    BodyInst->SetInstanceNotifyRBCollision(false);
    SetPhysicsBlendWeightForBone(Mesh, BoneName, 0.0f); // Always use accessor!
    BodyInst->SetLinearVelocity(FVector::ZeroVector, false);
    BodyInst->SetAngularVelocityInRadians(FVector::ZeroVector, false);

    return true;
}

void UOHSkeletalPhysicsUtils::BlendInPhysicalAnimationForBone(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName BoneName,
    const FPhysicalAnimationData& Profile,
    float BlendDuration,
    UCurveFloat* BlendCurve,
    UObject* WorldContextObject)
{
    if (!Mesh || !PAC || BoneName.IsNone()) return;

    PAC->ApplyPhysicalAnimationSettings(BoneName, Profile);

    FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
    if (BodyInst)
    {
        BodyInst->SetInstanceSimulatePhysics(true);
        BodyInst->SetInstanceNotifyRBCollision(true);
        BodyInst->WakeInstance();
    }

    UWorld* World = (WorldContextObject && WorldContextObject->GetWorld())
        ? WorldContextObject->GetWorld()
        : (Mesh->GetWorld());

    if (!World) return;

    float Elapsed = 0.f;
    FTimerHandle Handle;
    const float Step = 0.01f;

    World->GetTimerManager().SetTimer(Handle, [=, &Elapsed]() mutable
    {
        Elapsed += Step;
        float Alpha = FMath::Clamp(Elapsed / BlendDuration, 0.f, 1.f);
        float Blend = BlendCurve ? BlendCurve->GetFloatValue(Alpha) : Alpha;

        UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBone(Mesh, BoneName, Blend);

        if (Alpha >= 1.f)
        {
            UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBone(Mesh, BoneName, 1.0f);
            World->GetTimerManager().ClearTimer(Handle);
        }
    }, Step, true);
}

void UOHSkeletalPhysicsUtils::BlendOutPhysicalAnimationForBone(
    USkeletalMeshComponent* Mesh,
    UPhysicalAnimationComponent* PAC,
    FName BoneName,
    float BlendDuration,
    UCurveFloat* BlendCurve,
    UObject* WorldContextObject)
{
    if (!Mesh || !PAC || BoneName.IsNone()) return;

    UWorld* World = (WorldContextObject && WorldContextObject->GetWorld())
        ? WorldContextObject->GetWorld()
        : (Mesh->GetWorld());

    if (!World) return;

    float Elapsed = 0.f;
    FTimerHandle Handle;
    const float Step = 0.01f;

    World->GetTimerManager().SetTimer(Handle, [=, &Elapsed]() mutable
    {
        Elapsed += Step;
        float Alpha = FMath::Clamp(Elapsed / BlendDuration, 0.f, 1.f);
        float Blend = BlendCurve ? (1.0f - BlendCurve->GetFloatValue(Alpha)) : (1.0f - Alpha);

        UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBone(Mesh, BoneName, Blend);

        if (Alpha >= 1.f)
        {
            UOHSkeletalPhysicsUtils::SetPhysicsBlendWeightForBone(Mesh, BoneName, 0.0f);
            // Optional: remove physical animation profile and disable sim
            PAC->ApplyPhysicalAnimationSettings(BoneName, FPhysicalAnimationData());
            FBodyInstance* BodyInst = Mesh->GetBodyInstance(BoneName);
            if (BodyInst)
            {
                BodyInst->SetInstanceSimulatePhysics(false);
                BodyInst->SetInstanceNotifyRBCollision(false);
                BodyInst->SetLinearVelocity(FVector::ZeroVector, false);
                BodyInst->SetAngularVelocityInRadians(FVector::ZeroVector, false);
            }
            World->GetTimerManager().ClearTimer(Handle);
        }
    }, Step, true);
}

void UOHSkeletalPhysicsUtils::BlendInPhysicalAnimationForBodyPart(
	USkeletalMeshComponent* Mesh,
	UPhysicalAnimationComponent* PAC,
	EOHBodyPart BodyPart,
	const FPhysicalAnimationData& Profile,
	float BlendDuration,
	UCurveFloat* BlendCurve,
	UObject* WorldContextObject)
{
	TArray<EOHSkeletalBone> PartBones = GetBonesInBodyPart(BodyPart);
	for (EOHSkeletalBone BoneEnum : PartBones)
	{
		FName BoneName = GetBoneNameFromEnum(BoneEnum);
		if (BoneName != NAME_None)
			BlendInPhysicalAnimationForBone(Mesh, PAC, BoneName, Profile, BlendDuration, BlendCurve, WorldContextObject);
	}
}

void UOHSkeletalPhysicsUtils::BlendOutPhysicalAnimationForBodyPart(
	USkeletalMeshComponent* Mesh,
	UPhysicalAnimationComponent* PAC,
	EOHBodyPart BodyPart,
	float BlendDuration,
	UCurveFloat* BlendCurve,
	UObject* WorldContextObject)
{
	TArray<EOHSkeletalBone> PartBones = GetBonesInBodyPart(BodyPart);
	for (EOHSkeletalBone BoneEnum : PartBones)
	{
		FName BoneName = GetBoneNameFromEnum(BoneEnum);
		if (BoneName != NAME_None)
			BlendOutPhysicalAnimationForBone(Mesh, PAC, BoneName, BlendDuration, BlendCurve, WorldContextObject);
	}
}

void UOHSkeletalPhysicsUtils::ApplyPhysicalAnimationToBone(UPhysicalAnimationComponent* PhysicalAnimationComponent,
	FName BoneName, const FPhysicalAnimationData& Profile)
{
	if (!PhysicalAnimationComponent || !PhysicalAnimationComponent->GetSkeletalMesh())
	{
		UE_LOG(LogTemp, Warning, TEXT("PhysicalAnimationComponent invalid"));
		return;
	}

	USkeletalMeshComponent* SkelMesh = PhysicalAnimationComponent->GetSkeletalMesh();

	// Enable physics on the bone
	SkelMesh->SetAllBodiesBelowSimulatePhysics(BoneName, true, false);

	// Enable collision if needed
	SkelMesh->SetEnableBodyGravity(true, BoneName);
	SkelMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	// Apply the physical animation profile to the single bone
	PhysicalAnimationComponent->ApplyPhysicalAnimationSettings(BoneName, Profile);

	// Ensure physical animation is active
	PhysicalAnimationComponent->SetSkeletalMeshComponent(SkelMesh);
}

void UOHSkeletalPhysicsUtils::ApplyPhysicalAnimationToBoneChain(UPhysicalAnimationComponent* PhysicalAnimationComponent,
	FName RootBoneName, const FPhysicalAnimationData& Profile)
{
	if (!PhysicalAnimationComponent || !PhysicalAnimationComponent->GetSkeletalMesh())
	{
		UE_LOG(LogTemp, Warning, TEXT("PhysicalAnimationComponent invalid"));
		return;
	}

	USkeletalMeshComponent* SkelMesh = PhysicalAnimationComponent->GetSkeletalMesh();

	// Enable physics on all bones below RootBoneName
	SkelMesh->SetAllBodiesBelowSimulatePhysics(RootBoneName, true, false);

	// Enable collision if needed
	SkelMesh->SetEnableGravity(true);
	SkelMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

	// Apply the physical animation profile to all bones below the root
	PhysicalAnimationComponent->ApplyPhysicalAnimationSettingsBelow(RootBoneName, Profile, true);

	// Ensure physical animation is active
	PhysicalAnimationComponent->SetSkeletalMeshComponent(SkelMesh);
}

void UOHSkeletalPhysicsUtils::ApplyPhysicalAnimationProfileToBoneChain(
	UPhysicalAnimationComponent* PhysicalAnimationComponent, FName RootBoneName, FName ProfileName)
{
	if (!PhysicalAnimationComponent || !PhysicalAnimationComponent->GetSkeletalMesh())
	{
		UE_LOG(LogTemp, Warning, TEXT("PhysicalAnimationComponent invalid"));
		return;
	}

	USkeletalMeshComponent* SkelMesh = PhysicalAnimationComponent->GetSkeletalMesh();

	// Enable physics on all bones below RootBoneName
	SkelMesh->SetAllBodiesBelowSimulatePhysics(RootBoneName, true, false);

	// Enable collision if needed
	SkelMesh->SetEnableGravity(true);
	SkelMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

	// Apply the physical animation profile to all bones below the root
	PhysicalAnimationComponent->ApplyPhysicalAnimationProfileBelow(RootBoneName, ProfileName, true, true);

	// Ensure physical animation is active
	PhysicalAnimationComponent->SetSkeletalMeshComponent(SkelMesh);
}



#pragma endregion



#if 0

float UOHSkeletalPhysicsUtils::PredictChainLengthTrend(const TArray<FOHBoneState>& BoneStates)
{
	if (BoneStates.Num() < 2)
	{
		return 0.f;
	}
	float PrevLength = 0.f, CurrLength = 0.f;

	for (int32 i = 0; i < BoneStates.Num() - 1; ++i)
	{
		PrevLength += FVector::Dist(
			BoneStates[i].PreviousWorldTransform.GetLocation(),
			BoneStates[i + 1].PreviousWorldTransform.GetLocation()
		);
		CurrLength += FVector::Dist(
			BoneStates[i].CurrentWorldTransform.GetLocation(),
			BoneStates[i + 1].CurrentWorldTransform.GetLocation()
		);
	}

	return CurrLength - PrevLength;
}

void UOHSkeletalPhysicsUtils::PrintBoneMapDebug(const TMap<EOHSkeletalBone, FBoneReference>& BoneRefs)
{
	for (const auto& Pair : BoneRefs)
	{
		UE_LOG(LogTemp, Verbose, TEXT("BoneEnum: %s → %s"),
			*UEnum::GetValueAsString(Pair.Key),
			*Pair.Value.BoneName.ToString());
	}
}

TArray<FName> UOHSkeletalPhysicsUtils::BP_GetValidatedBoneChainFromComponent(
	USkeletalMeshComponent* SkelComp,
	EOHSkeletalBone EffectorEnum
)
{
	TArray<FName> OutChain;
	if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
	{
		return OutChain;
	}

	// Grab the reference skeleton (editor-time & runtime) 
	const FReferenceSkeleton& RefSkel = SkelComp->GetSkeletalMeshAsset()->GetRefSkeleton();
	// Build the raw enum lineage (including markers)
	TArray<EOHSkeletalBone> Raw = GetBoneLineage(EffectorEnum, EOHSkeletalBone::None);

	// Filter out markers & only keep bones actually in the ref-skeleton
	for (EOHSkeletalBone E : Raw)
	{
		if (E == EOHSkeletalBone::None)
		{
			continue;
		}

		FName BoneName = GetBoneNameFromEnum(E);
		if (BoneName == NAME_None)
		{
			continue;
		}

		if (RefSkel.FindBoneIndex(BoneName) != INDEX_NONE)
		{
			OutChain.Add(BoneName);
		}
	}

	return OutChain;
}



EOHBodyPart UOHSkeletalPhysicsUtils::GetBodyPartFromSkeletalBone(const EOHSkeletalBone Bone)
{
	return ::GetBodyPartForBone(Bone);
}

TArray<EOHBodyPart> UOHSkeletalPhysicsUtils::GetBodyPartsFromBones(TArray<EOHSkeletalBone> Bones)
{
	TArray<EOHBodyPart> BodyParts;
	for (const EOHSkeletalBone Bone : Bones)
	{
		BodyParts.Add(GetBodyPartFromBone(Bone));
	}
	return BodyParts;
}

EOHBodyRegion UOHSkeletalPhysicsUtils::GetRegionFromBone(const EOHSkeletalBone Bone)
{
	const EOHBodyPart Part = GetBodyPartFromBone(Bone);
	return GetBodyRegionForPart(Part);
}

TArray<EOHBodyRegion> UOHSkeletalPhysicsUtils::GetRegionsFromBones(TArray<EOHSkeletalBone> Bones)
{
	TArray<EOHBodyRegion> Regions;
	for (const EOHSkeletalBone Bone : Bones)
	{
		Regions.Add(GetRegionFromBone(Bone));
	}
	return Regions;
}

EOHBodyRegion UOHSkeletalPhysicsUtils::GetRegionFromBodyPart(const EOHBodyPart BodyPart)
{
	return GetBodyRegionForPart(BodyPart);
}

TArray<EOHBodyRegion> UOHSkeletalPhysicsUtils::GetRegionsFromBodyParts(TArray<EOHBodyPart> BodyParts)
{
	TArray<EOHBodyRegion> Regions;
	for (const EOHBodyPart BodyPart : BodyParts)
	{
		Regions.Add(GetRegionFromBodyPart(BodyPart));
	}
	return Regions;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesInRegion(const EOHBodyRegion Region)
{
	TArray<EOHSkeletalBone> BonesInRegion;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (GetRegionFromBone(Bone) == Region)
		{
			BonesInRegion.Add(Bone);
		}
	}
	return BonesInRegion;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesInRegions(TArray<EOHBodyRegion> Regions)
{
	TArray<EOHSkeletalBone> BonesInRegions;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		for (const EOHBodyRegion Region : Regions)
		{
			if (GetRegionFromBone(Bone) == Region)
			{
				BonesInRegions.Add(Bone);
			}
		}
	}
	return BonesInRegions;
}

TArray<EOHBodyPart> UOHSkeletalPhysicsUtils::GetBodyPartsInRegion(const EOHBodyRegion Region)
{
	TArray<EOHBodyPart> BodyParts;
	for (int32 i = static_cast<int32>(EOHBodyPart::Head); i <= static_cast<int32>(EOHBodyPart::Leg_Right); ++i)
	{
		const EOHBodyPart Part = static_cast<EOHBodyPart>(i);
		if (GetRegionFromBodyPart(Part) == Region)
		{
			BodyParts.Add(Part);
		}
	}
	return BodyParts;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesInBodyParts(TArray<EOHBodyPart> BodyParts)
{
	TArray<EOHSkeletalBone> BonesInBodyParts;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		// Use the range-based iteration for enum
		for (const EOHBodyPart BodyPart : BodyParts)
		{
			if (GetBodyPartForBone(Bone) == BodyPart)
			{
				BonesInBodyParts.Add(Bone);
			}
		}
	}
	return BonesInBodyParts;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesFromRegionAndBodyPart(
	const EOHBodyRegion Region, const EOHBodyPart BodyPart)
{
	TArray<EOHSkeletalBone> Bones;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (GetBodyPartFromBone(Bone) == BodyPart && GetRegionFromBone(Bone) == Region)
		{
			Bones.Add(Bone);
		}
	}
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesFromRegionAndBodyParts(
	const EOHBodyRegion Region, const TArray<EOHBodyPart> BodyParts)
{
	TArray<EOHSkeletalBone> Bones;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))

	{
		for (const EOHBodyPart BodyPart : BodyParts)
		{
			if (GetBodyPartFromBone(Bone) == BodyPart && GetRegionFromBone(Bone) == Region)
			{
				Bones.Add(Bone);
			}
		}
	}
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesFromRegionsAndBodyParts(const TArray<EOHBodyRegion>& Regions,
	const TArray<EOHBodyPart>& BodyParts)
{
	TArray<EOHSkeletalBone> Bones;
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		for (const EOHBodyPart BodyPart : BodyParts)
		{
			if (GetBodyPartFromBone(Bone) == BodyPart && Regions.Contains(GetRegionFromBone(Bone)))
			{
				Bones.Add(Bone);
			}
		}
	}
	return Bones;
}

EOHSkeletalBone UOHSkeletalPhysicsUtils::GetRootBoneEnumForBodyPart(const EOHBodyPart BodyPart)
{
	switch (BodyPart)
	{
	case EOHBodyPart::Head: return EOHSkeletalBone::Neck_01;
	case EOHBodyPart::Torso: return EOHSkeletalBone::Spine_01;
	case EOHBodyPart::Arm_Left: return EOHSkeletalBone::Clavicle_L;
	case EOHBodyPart::Arm_Right: return EOHSkeletalBone::Clavicle_R;
	case EOHBodyPart::Leg_Left: return EOHSkeletalBone::Thigh_L;
	case EOHBodyPart::Leg_Right: return EOHSkeletalBone::Thigh_R;
	default: return EOHSkeletalBone::FirstBone;
	}
}

EOHSkeletalBone UOHSkeletalPhysicsUtils::GetEndBoneEnumForBodyPart(EOHBodyPart BodyPart)
{
	switch (BodyPart)
	{
	case EOHBodyPart::Head: return EOHSkeletalBone::Head;
	case EOHBodyPart::Torso: return EOHSkeletalBone::Spine_03;
	case EOHBodyPart::Arm_Left: return EOHSkeletalBone::Hand_L;
	case EOHBodyPart::Arm_Right: return EOHSkeletalBone::Hand_R;
	case EOHBodyPart::Leg_Left: return EOHSkeletalBone::Ball_L;
	case EOHBodyPart::Leg_Right: return EOHSkeletalBone::Ball_R;
	default: return EOHSkeletalBone::LastBone;
	}
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesInBodyPart(const EOHBodyPart BodyPart)
{
	TArray<EOHSkeletalBone> BodyPartBones;

	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (GetBodyPartForBone(Bone) == BodyPart)
		{
			BodyPartBones.Add(Bone);
		}
	}

	return BodyPartBones;
}


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetAllFingerBones(const bool bLeftHand)
{
	TArray<EOHSkeletalBone> FingerBones;

	// Use the range-based iteration for enum
	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		// Call the FORCE INLINE helper functions from EOHPhysicsEnums.h
		if (bLeftHand && IsLeftFingerBone(Bone))
		{
			FingerBones.Add(Bone);
		}
		else if (!bLeftHand && IsRightFingerBone(Bone))
		{
			FingerBones.Add(Bone);
		}
	}

	return FingerBones;
}

bool UOHSkeletalPhysicsUtils::IsBoneInFingerByIndex(const EOHSkeletalBone Bone, const int32 FingerIndex)
{
	// First check if it's a finger bone at all
	if (!IsFingerBone(Bone))
	{
		return false;
	}

	// Then check if it belongs to the specified finger index
	return GetFingerIndexForBone(Bone) == FingerIndex;
}

int32 UOHSkeletalPhysicsUtils::GetFingerIndexForBone(EOHSkeletalBone Bone)
{
	// If not a finger bone, return -1
	if (!IsFingerBone(Bone))
	{
		return -1;
	}

	// Determine which finger this bone belongs to (0=thumb, 1=index, 2=middle, 3=ring, 4=pinky)
	FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));

	if (BoneName.Contains(TEXT("Thumb")))
	{
		return 0;
	}
	if (BoneName.Contains(TEXT("Index")))
	{
		return 1;
	}
	if (BoneName.Contains(TEXT("Middle")))
	{
		return 2;
	}
	if (BoneName.Contains(TEXT("Ring")))
	{
		return 3;
	}
	if (BoneName.Contains(TEXT("Pinky")) || BoneName.Contains(TEXT("Little")))
	{
		return 4;
	}

	// Shouldn't reach here if IsFingerBone is accurate
	return -1;
}

int32 UOHSkeletalPhysicsUtils::GetJointPositionInFinger(EOHSkeletalBone Bone)
{
	// If not a finger bone, return -1
	if (!IsFingerBone(Bone))
	{
		return -1;
	}

	// Determine position in finger based on naming pattern
	FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));

	// Check for common naming patterns in bone names
	if (BoneName.Contains(TEXT("_01")) || BoneName.Contains(TEXT("Proximal")))
	{
		return 0; // Proximal joint (closest to hand)
	}
	if (BoneName.Contains(TEXT("_02")) || BoneName.Contains(TEXT("Middle")))
	{
		return 1; // Middle joint
	}
	if (BoneName.Contains(TEXT("_03")) || BoneName.Contains(TEXT("Distal")))
	{
		return 2; // Distal joint (fingertip)
	}

	// If pattern doesn't match, do a fallback
	if (BoneName.EndsWith(TEXT("1_L")) || BoneName.EndsWith(TEXT("1_R")))
	{
		return 0;
	}
	if (BoneName.EndsWith(TEXT("2_L")) || BoneName.EndsWith(TEXT("2_R")))
	{
		return 1;
	}
	if (BoneName.EndsWith(TEXT("3_L")) || BoneName.EndsWith(TEXT("3_R")))
	{
		return 2;
	}

	// Can't determine position
	return -1;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesOfAnatomicalType(FString BoneType)
{
	TArray<EOHSkeletalBone> TypeBones;

	// Convert input to lowercase for case-insensitive comparison
	BoneType = BoneType.ToLower();

	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		// Match based on the requested bone type
		// Using the inline helpers from EOHPhysicsEnums.h where applicable

		if (BoneType == TEXT("finger") && IsFingerBone(Bone))
		{
			TypeBones.Add(Bone);
		}
		else if (BoneType == TEXT("arm") && IsArmBone(Bone))
		{
			TypeBones.Add(Bone);
		}
		else if (BoneType == TEXT("leg") && IsLegBone(Bone))
		{
			TypeBones.Add(Bone);
		}
		else if (BoneType == TEXT("spine") && IsSpineBone(Bone))
		{
			TypeBones.Add(Bone);
		}
		else if (BoneType == TEXT("head") && IsHeadBone(Bone))
		{
			TypeBones.Add(Bone);
		}
		// Additional types can be added here
	}

	return TypeBones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBonesOnBodySide(const bool bLeftSide)
{
	TArray<EOHSkeletalBone> SideBones;

	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		// Check if the bone is on the requested side
		if ((bLeftSide && IsLeftSideBone(Bone)) || (!bLeftSide && IsRightSideBone(Bone)))
		{
			SideBones.Add(Bone);
		}
	}

	return SideBones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetBoneChainBetween(const EOHSkeletalBone StartBone,
                                                                     const EOHSkeletalBone EndBone)
{
	TArray<EOHSkeletalBone> BoneChain;

	// This is a simplified implementation - a real system would use the actual
	// skeletal hierarchy to find the shortest path between bones

	// Add the start bone
	BoneChain.Add(StartBone);

	// Example: Chain from finger to hand to arm
	if (IsFingerBone(StartBone) && IsArmBone(EndBone))
	{
		// Determine side
		bool bIsLeft = IsLeftSideBone(StartBone);

		// Add hand
		BoneChain.Add(bIsLeft ? EOHSkeletalBone::Hand_L : EOHSkeletalBone::Hand_R);

		// Add a lower arm if needed
		if (EndBone != (bIsLeft ? EOHSkeletalBone::LowerArm_L : EOHSkeletalBone::LowerArm_R))
		{
			BoneChain.Add(bIsLeft ? EOHSkeletalBone::LowerArm_L : EOHSkeletalBone::LowerArm_R);

			// Add an upper arm if needed
			if (EndBone == (bIsLeft ? EOHSkeletalBone::UpperArm_L : EOHSkeletalBone::UpperArm_R))
			{
				BoneChain.Add(bIsLeft ? EOHSkeletalBone::UpperArm_L : EOHSkeletalBone::UpperArm_R);
			}
		}
	}

	// More specific chain definitions would be added for various bone combinations

	// Only include the end bone if we got a valid path
	if (BoneChain.Num() > 1 && BoneChain.Last() != EndBone)
	{
		BoneChain.Add(EndBone);
	}

	return BoneChain;
}



EOHSkeletalBone UOHSkeletalPhysicsUtils::GetMirroredBone(EOHSkeletalBone Bone)
{
	// Check if the bone is on a specific side
	if (!IsLeftSideBone(Bone) && !IsRightSideBone(Bone))
	{
		// Return the original bone if it's not a side-specific bone
		return Bone;
	}

	// Get the bone name
	FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));

	// Replace L with R or vice versa to get the mirror
	if (BoneName.EndsWith(TEXT("_L")))
	{
		BoneName = BoneName.LeftChop(2) + TEXT("_R");
	}
	else if (BoneName.EndsWith(TEXT("_R")))
	{
		BoneName = BoneName.LeftChop(2) + TEXT("_L");
	}
	else
	{
		// Handle other naming patterns if needed
		if (BoneName.Contains(TEXT("Left")))
		{
			BoneName = BoneName.Replace(TEXT("Left"), TEXT("Right"));
		}
		else if (BoneName.Contains(TEXT("Right")))
		{
			BoneName = BoneName.Replace(TEXT("Right"), TEXT("Left"));
		}
	}

	// Try to find the mirrored bone enum
	const UEnum* EnumPtr = StaticEnum<EOHSkeletalBone>();
	int64 EnumValue = EnumPtr->GetValueByName(FName(*BoneName));

	if (EnumValue != INDEX_NONE)
	{
		return static_cast<EOHSkeletalBone>(EnumValue);
	}

	// Return the original bone if the mirrored version not found
	return Bone;
}


FTransform UOHSkeletalPhysicsUtils::GetBoneWorldTransformSafe(USkeletalMeshComponent* SkeletalMesh,
                                                              const EOHSkeletalBone Bone)
{
	if (!SkeletalMesh)
	{
		return FTransform::Identity;
	}

	const FName BoneName = GetBoneNameFromEnum(Bone);

	if (SkeletalMesh->DoesSocketExist(BoneName))
	{
		return SkeletalMesh->GetSocketTransform(BoneName, RTS_World);
	}

	return FTransform::Identity;
}
#if 0
FBoneMetadata UOHSkeletalPhysicsUtils::GetBoneMetadata(const EOHSkeletalBone BoneEnum)
{
    FBoneMetadata Meta;
    Meta.bAllowSimulation = true;
    Meta.ImpulseStrengthMultiplier = 1.0f;
    Meta.ForceScale = 1.0f;
    Meta.MaxLinearSpeed = 3000.f;
    Meta.MaxAngularSpeedDegrees = 720.f;
    Meta.LinearSmoothingFactor = 0.25f;
    Meta.AngularSmoothingFactor = 0.25f;

    if (IsArmBone(BoneEnum)) {
        Meta.ImpulseStrengthMultiplier = 1.2f;
        Meta.Importance = 1.5f;
    } else if (IsFingerBone(BoneEnum)) {
        Meta.ImpulseStrengthMultiplier = 0.8f;
        Meta.Importance = 0.8f;
        Meta.MaxLinearSpeed = 1500.f; // Smaller bones, less speed
    } else if (IsHeadBone(BoneEnum)) {
        Meta.ImpulseStrengthMultiplier = 0.7f;
        Meta.Importance = 2.0f;
    }
    return Meta;
}

TArray<FBoneMetadata> UOHSkeletalPhysicsUtils::GetAllBoneMetadata()
{
    TArray<FBoneMetadata> Output;

    const UEnum* EnumPtr = StaticEnum<EOHSkeletalBone>();
    if (!EnumPtr) return Output;

    const int32 Count = EnumPtr->NumEnums();
    for (int32 i = 0; i < Count; ++i)
    {
        const int64 Value = EnumPtr->GetValueByIndex(i);
        if (Value < 0) continue;

        EOHSkeletalBone BoneEnum = static_cast<EOHSkeletalBone>(Value);
        if (!IsTrackedBoneValid(BoneEnum)) continue;

        Output.Add(GetBoneMetadata(BoneEnum));
    }

    return Output;
}
FTransform UOHSkeletalPhysicsUtils::GetBoneLocalTransformByEnum(const USkeletalMeshComponent* Component,
    EOHSkeletalBone BoneEnum, bool bIncludeScale)
{
    const FName BoneName = GetBoneNameFromEnum(BoneEnum);

    if (Component->DoesSocketExist(BoneName))
    {

        FTransform BoneTransform = Component->GetSocketTransform(BoneName, ERelativeTransformSpace::RTS_World);
        if (bIncludeScale)
        {
            BoneTransform.SetScale3D(Component->GetSocketTransform(BoneName).GetScale3D());
        }
        return BoneTransform;
    }
    return FTransform::Identity;
}
#endif


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetLeftArmBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L, EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,
		EOHSkeletalBone::Thumb_01_L, EOHSkeletalBone::Thumb_02_L, EOHSkeletalBone::Thumb_03_L,
		EOHSkeletalBone::Index_01_L, EOHSkeletalBone::Index_02_L, EOHSkeletalBone::Index_03_L,
		EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Middle_02_L, EOHSkeletalBone::Middle_03_L,
		EOHSkeletalBone::Ring_01_L, EOHSkeletalBone::Ring_02_L, EOHSkeletalBone::Ring_03_L,
		EOHSkeletalBone::Pinky_01_L, EOHSkeletalBone::Pinky_02_L, EOHSkeletalBone::Pinky_03_L
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetRightArmBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R, EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
		EOHSkeletalBone::Thumb_01_R, EOHSkeletalBone::Thumb_02_R, EOHSkeletalBone::Thumb_03_R,
		EOHSkeletalBone::Index_01_R, EOHSkeletalBone::Index_02_R, EOHSkeletalBone::Index_03_R,
		EOHSkeletalBone::Middle_01_R, EOHSkeletalBone::Middle_02_R, EOHSkeletalBone::Middle_03_R,
		EOHSkeletalBone::Ring_01_R, EOHSkeletalBone::Ring_02_R, EOHSkeletalBone::Ring_03_R,
		EOHSkeletalBone::Pinky_01_R, EOHSkeletalBone::Pinky_02_R, EOHSkeletalBone::Pinky_03_R
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetLeftLegBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L, EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetRightLegBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R, EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetSpineBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01, EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03,
		EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetFingerBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::Thumb_01_L, EOHSkeletalBone::Thumb_02_L, EOHSkeletalBone::Thumb_03_L,
		EOHSkeletalBone::Index_01_L, EOHSkeletalBone::Index_02_L, EOHSkeletalBone::Index_03_L,
		EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Middle_02_L, EOHSkeletalBone::Middle_03_L,
		EOHSkeletalBone::Ring_01_L, EOHSkeletalBone::Ring_02_L, EOHSkeletalBone::Ring_03_L,
		EOHSkeletalBone::Pinky_01_L, EOHSkeletalBone::Pinky_02_L, EOHSkeletalBone::Pinky_03_L,

		EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R, EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
		EOHSkeletalBone::Thumb_01_R, EOHSkeletalBone::Thumb_02_R, EOHSkeletalBone::Thumb_03_R,
		EOHSkeletalBone::Index_01_R, EOHSkeletalBone::Index_02_R, EOHSkeletalBone::Index_03_R,
		EOHSkeletalBone::Middle_01_R, EOHSkeletalBone::Middle_02_R, EOHSkeletalBone::Middle_03_R,
		EOHSkeletalBone::Ring_01_R, EOHSkeletalBone::Ring_02_R, EOHSkeletalBone::Ring_03_R,
		EOHSkeletalBone::Pinky_01_R, EOHSkeletalBone::Pinky_02_R, EOHSkeletalBone::Pinky_03_R
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetIKRootBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::IK_Root,
		EOHSkeletalBone::IK_Hand_Root,
		EOHSkeletalBone::IK_Foot_Root,
		EOHSkeletalBone::IK_Hand_Gun
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetIKLimbBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::IK_Hand_L,
		EOHSkeletalBone::IK_Hand_R,
		EOHSkeletalBone::IK_Foot_L,
		EOHSkeletalBone::IK_Foot_R
	};
	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetIKBones()
{
	static const TArray<EOHSkeletalBone> Bones = {
		EOHSkeletalBone::IK_Root,
		EOHSkeletalBone::IK_Hand_Root,
		EOHSkeletalBone::IK_Foot_Root,
		EOHSkeletalBone::IK_Hand_Gun,
		EOHSkeletalBone::IK_Hand_L,
		EOHSkeletalBone::IK_Hand_R,
		EOHSkeletalBone::IK_Foot_L,
		EOHSkeletalBone::IK_Foot_R
	};
	return Bones;
}


bool UOHSkeletalPhysicsUtils::IsTrackedBoneValid(const EOHSkeletalBone Bone)
{
	return Bone != EOHSkeletalBone::None;
}

bool UOHSkeletalPhysicsUtils::IsIKBone(const EOHSkeletalBone Bone)
{
	const TArray<EOHSkeletalBone>& IK = GetIKBones();
	return IK.Contains(Bone);
}

bool UOHSkeletalPhysicsUtils::IsSimulatableBone(const EOHSkeletalBone Bone)
{
	// Simulatable = not None, not IK, not marker
	return IsTrackedBoneValid(Bone)
		&& !IsIKBone(Bone)
		&& !IsFingerBone(Bone) // optional: skip small bones if not simulating them
		&& !IsBallBone(Bone) // Optional: skip ball for certain sims
		&& Bone != EOHSkeletalBone::Pelvis // Optional: skip root for certain sims
		&& Bone != EOHSkeletalBone::Root; // Optional: skip root for certain sims
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetDefaultSimulatableBones()
{
	TArray<EOHSkeletalBone> DefaultBones;

	for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
	     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
	{
		if (IsSimulatableBone(Bone))
		{
			DefaultBones.Add(Bone);
		}
	}

	return DefaultBones;
}


// --- Static Maps ---


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::BP_GetBonesFromBodyPart(const EOHBodyPart BodyPart)
{
	return GetBonesInBodyPart(BodyPart);
}

EOHBodyPart UOHSkeletalPhysicsUtils::BP_GetBodyPartFromBone(const EOHSkeletalBone Bone)
{
	const TMap<EOHSkeletalBone, EOHBodyPart>& Map = GetBoneToBodyPartMap();
	if (const EOHBodyPart* Part = Map.Find(Bone))
	{
		return *Part;
	}
	return EOHBodyPart::None;
}

EOHBodyRegion UOHSkeletalPhysicsUtils::BP_GetRegionFromBone(const EOHSkeletalBone Bone)
{
	const TMap<EOHSkeletalBone, EOHBodyRegion>& Map = GetBoneToRegionMap();
	if (const EOHBodyRegion* Region = Map.Find(Bone))
	{
		return *Region;
	}
	return EOHBodyRegion::None;
}

EOHBodyRegion UOHSkeletalPhysicsUtils::BP_GetRegionFromBodyPart(const EOHBodyPart BodyPart)
{
	const TMap<EOHBodyPart, EOHBodyRegion>& Map = GetBodyPartToRegionMap();
	if (const EOHBodyRegion* Region = Map.Find(BodyPart))
	{
		return *Region;
	}
	return EOHBodyRegion::None;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::BP_GetBonesInRegion(const EOHBodyRegion Region)
{
	return GetBonesInRegion(Region);
}

TArray<EOHBodyPart> UOHSkeletalPhysicsUtils::BP_GetBodyPartsInRegion(const EOHBodyRegion Region)
{
	return GetBodyPartsInRegion(Region);
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::BP_GetBonesFromRegionAndBodyPart(
	const EOHBodyRegion Region, const EOHBodyPart BodyPart)
{
	return GetBonesFromRegionAndBodyPart(Region, BodyPart);
}

bool UOHSkeletalPhysicsUtils::BP_IsShoulderBone(const EOHSkeletalBone Bone)
{
	return Bone == EOHSkeletalBone::Clavicle_L || Bone == EOHSkeletalBone::Clavicle_R;
}

bool UOHSkeletalPhysicsUtils::BP_IsHandBone(const EOHSkeletalBone Bone)
{
	return Bone == EOHSkeletalBone::Hand_L || Bone == EOHSkeletalBone::Hand_R;
}

bool UOHSkeletalPhysicsUtils::BP_IsFootBone(const EOHSkeletalBone Bone)
{
	return Bone == EOHSkeletalBone::Foot_L || Bone == EOHSkeletalBone::Foot_R;
}

bool UOHSkeletalPhysicsUtils::BP_IsPelvisBone(const EOHSkeletalBone Bone)
{
	return Bone == EOHSkeletalBone::Pelvis;
}

bool UOHSkeletalPhysicsUtils::ShouldSimulate(const FOHBoneState& State)
{
	return State.Role == EOHBoneSimulationRole::Simulated ||
		State.Role == EOHBoneSimulationRole::AttachedToSimulated;
}

bool UOHSkeletalPhysicsUtils::ShouldApplyPACDrive(const FOHBoneState& State)
{
	return FMath::IsNearlyEqual(State.PhysicsBlendWeight, 1.0f, 0.01f) &&
		State.Role == EOHBoneSimulationRole::Simulated &&
		State.ResolvedProfileData.OrientationStrength > 0.0f;
}




FName UOHSkeletalPhysicsUtils::GetBoneNameFromEnumSafe(EOHSkeletalBone Bone)
{
	return GetBoneNameFromEnum(Bone);
}



float UOHSkeletalPhysicsUtils::ComputeCompressionAlphaFromBoneEnums(
	USkeletalMeshComponent* Mesh,
	EOHSkeletalBone UpperArm,
	EOHSkeletalBone LowerArm,
	EOHSkeletalBone Hand,
	float MinLength,
	float MaxLength)
{
	if (!Mesh)
		return 0.f;

	const FVector Upper = Mesh->GetBoneLocation(GetBoneNameFromEnum(UpperArm));
	const FVector Lower = Mesh->GetBoneLocation(GetBoneNameFromEnum(LowerArm));
	const FVector End   = Mesh->GetBoneLocation(GetBoneNameFromEnum(Hand));

	const float Length = (Upper - Lower).Size() + (Lower - End).Size();
	const float Alpha = (Length - MinLength) / FMath::Max(MaxLength - MinLength, KINDA_SMALL_NUMBER);
	return FMath::Clamp(Alpha, 0.f, 1.f);
}

FVector UOHSkeletalPhysicsUtils::ComputeEffectorTargetFromVelocity(
	USkeletalMeshComponent* Mesh,
	EOHSkeletalBone StartBone,
	FVector Velocity,
	float ProjectDistance)
{
	if (!Mesh || StartBone == EOHSkeletalBone::None)
		return FVector::ZeroVector;

	const FVector Start = Mesh->GetBoneLocation(GetBoneNameFromEnum(StartBone));
	return Start + Velocity.GetSafeNormal() * ProjectDistance;
}

FTransform UOHSkeletalPhysicsUtils::GetOffsetBoneTransform(
	USkeletalMeshComponent* SK,
	EOHSkeletalBone BoneEnum,
	FVector Offset,
	FRotator RotationOffset)
{
	if (!SK || BoneEnum == EOHSkeletalBone::None)
		return FTransform::Identity;

	const FName BoneName = GetBoneNameFromEnum(BoneEnum);
	const FTransform BoneTransform = SK->GetBoneTransform(SK->GetBoneIndex(BoneName));
	
	FTransform OffsetTransform(RotationOffset, Offset);
	return OffsetTransform * BoneTransform;
}

float UOHSkeletalPhysicsUtils::PredictArmExtensionTrendFromState(
	const FOHBoneState& Upper,
	const FOHBoneState& Hand)
{
	const FVector CurrentDir = (Hand.CurrentWorldTransform.GetLocation() - Upper.CurrentWorldTransform.GetLocation()).GetSafeNormal();
	const FVector VelocityDir = Hand.LinearVelocity.GetSafeNormal();

	const float Dot = FVector::DotProduct(CurrentDir, VelocityDir);
	return FMath::Clamp(Dot, -1.f, 1.f); // +1 = extending, -1 = retracting
}

void UOHSkeletalPhysicsUtils::InitializeBoneReferenceMap(
	const USkeletalMeshComponent* Mesh,
	FOHBoneReferenceMap& OutMap)
{
	if (!Mesh || !Mesh->GetAnimInstance())
	{
		return;
	}

	const FBoneContainer& BoneContainer = Mesh->GetAnimInstance()->GetRequiredBones();
	OutMap.Initialize(BoneContainer);
}


void UOHSkeletalPhysicsUtils::GetPreviewBoneInfosSafe(const UObject* WorldContext, const FOHBoneContextMap& Context, TArray<FOHBonePreviewInfo>& OutInfos)
{
	OutInfos.Reset();
	for (const FOHBoneContextEntry& Entry : Context.GetContextEntries())
	{
		FOHBonePreviewInfo Info;
		Info.Bone = Entry.Bone;
		Info.Location = Entry.State.CurrentWorldTransform.GetLocation();
		Info.Velocity = Entry.State.LinearVelocity;

		OutInfos.Add(Info);
	}
}


// --------------------- Internal Methods  --------------------- //
#pragma region Internals_


bool UOHSkeletalPhysicsUtils::ResolveIKChainTransforms(
	FComponentSpacePoseContext& PoseContext,
	const FOHIKBoneIndexCache& ChainCache,
	FTransform& OutRoot,
	FTransform& OutMiddle,
	FTransform& OutTip)
{
	if (!ChainCache.IsValid())
	{
		return false;
	}

	FCSPose<FCompactPose>& CSPose = PoseContext.Pose;

	OutRoot = CSPose.GetComponentSpaceTransform(ChainCache.RootIndex);
	OutMiddle = CSPose.GetComponentSpaceTransform(ChainCache.MiddleIndex);
	OutTip = CSPose.GetComponentSpaceTransform(ChainCache.TipIndex);

	return true;
}

void UOHSkeletalPhysicsUtils::InitializeIKChainCache(
	const FOHIKChain& Chain,
	const FBoneContainer& BoneContainer,
	FOHIKBoneIndexCache& OutCache)
{
	auto GetIndex = [&](EOHSkeletalBone BoneEnum) -> FCompactPoseBoneIndex
	{
		const FName BoneName = GetBoneNameFromEnum(BoneEnum);
		return FCompactPoseBoneIndex(BoneContainer.GetPoseBoneIndexForBoneName(BoneName));
	};

	const FCompactPoseBoneIndex Root = GetIndex(Chain.Root);
	const FCompactPoseBoneIndex Mid = GetIndex(Chain.Middle);
	const FCompactPoseBoneIndex Tip = GetIndex(Chain.Tip);

	const bool bAllValid = Root != INDEX_NONE && Mid != INDEX_NONE && Tip != INDEX_NONE;
	if (bAllValid)
	{
		OutCache.RootIndex = Root;
		OutCache.MiddleIndex = Mid;
		OutCache.TipIndex = Tip;
		OutCache.bInitialized = true;
	}
	else
	{
		OutCache.bInitialized = false;
	}
}

bool UOHSkeletalPhysicsUtils::ValidateAndInitializeIKChain(
	const FOHIKChain& Chain,
	const FBoneContainer& BoneContainer,
	FBoneReference& OutRootRef,
	FBoneReference& OutMiddleRef,
	FBoneReference& OutTipRef)
{
	auto TryInit = [&](EOHSkeletalBone BoneEnum, FBoneReference& OutRef) -> bool
	{
		if (BoneEnum == EOHSkeletalBone::None)
		{
			return false;
		}

		const FName BoneName = GetBoneNameFromEnum(BoneEnum);
		const int32 PoseIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);

		if (PoseIndex == INDEX_NONE)
		{
			return false;
		}

		OutRef.BoneName = BoneName;
		OutRef.Initialize(BoneContainer);
		return OutRef.IsValidToEvaluate(BoneContainer);
	};

	return TryInit(Chain.Root, OutRootRef)
		&& TryInit(Chain.Middle, OutMiddleRef)
		&& TryInit(Chain.Tip, OutTipRef);
}

bool UOHSkeletalPhysicsUtils::ValidateReferenceMapAgainstExpectedBones(
	const FOHBoneReferenceMap& Map,
	const TArray<EOHSkeletalBone>& ExpectedBones,
	FName ContextLabel)
{
	bool bAllValid = true;

	if (!Map.IsInitialized())
	{
		UE_LOG(LogTemp, Error, TEXT("[OnlyHands] BoneReferenceMap is not initialized. [%s]"), *ContextLabel.ToString());
		return false;
	}

	for (EOHSkeletalBone Bone : ExpectedBones)
	{
		if (!Map.HasValidReference(Bone))
		{
			UE_LOG(LogTemp, Error,
				TEXT("[OnlyHands][%s] Missing BoneReference for: %s"),
				*ContextLabel.ToString(),
				*UEnum::GetValueAsString(Bone));
			bAllValid = false;
		}
	}

	if (bAllValid)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("[OnlyHands][%s] BoneReferenceMap contains all expected bones."), *ContextLabel.ToString());
	}

	return bAllValid;
}

void UOHSkeletalPhysicsUtils::AddToBoneMap(
	FOHBoneReferenceMap& Map,
	EOHSkeletalBone Bone,
	const FBoneReference& Ref,
	const FCompactPoseBoneIndex& Index)
{
	Map.AddEntry(Bone, Ref, Index);
}


TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetMissingBones(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& ExpectedBones)
{
	TArray<EOHSkeletalBone> Missing;

	if (!Map.IsInitialized()) return ExpectedBones;

	for (EOHSkeletalBone Bone : ExpectedBones)
	{
		if (!Map.HasValidReference(Bone))
		{
			Missing.Add(Bone);
		}
	}
	return Missing;
}



#pragma endregion


// --------------------- Private Methods  --------------------- //
#pragma region Private_
const TMap<EOHSkeletalBone, EOHBodyPart>& UOHSkeletalPhysicsUtils::GetBoneToBodyPartMap()
{
	static TMap<EOHSkeletalBone, EOHBodyPart> Map;
	if (Map.Num() == 0)
	{
		for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
		     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
		{
			Map.Add(Bone, GetBodyPartForBone(Bone));
		}
	}
	return Map;
}

const TMap<EOHSkeletalBone, EOHBodyRegion>& UOHSkeletalPhysicsUtils::GetBoneToRegionMap()
{
	static TMap<EOHSkeletalBone, EOHBodyRegion> Map;
	if (Map.Num() == 0)
	{
		for (EOHSkeletalBone Bone = EOHSkeletalBone::FirstBone; Bone <= EOHSkeletalBone::LastBone;
		     Bone = static_cast<EOHSkeletalBone>(static_cast<uint8>(Bone) + 1))
		{
			const EOHBodyPart Part = GetBodyPartForBone(Bone);
			Map.Add(Bone, GetBodyRegionForPart(Part));
		}
	}
	return Map;
}

const TMap<EOHBodyPart, EOHBodyRegion>& UOHSkeletalPhysicsUtils::GetBodyPartToRegionMap()
{
	static TMap<EOHBodyPart, EOHBodyRegion> Map;
	if (Map.Num() == 0)
	{
		Map.Add(EOHBodyPart::Head, EOHBodyRegion::UpperBody);
		Map.Add(EOHBodyPart::Torso, EOHBodyRegion::UpperBody);
		Map.Add(EOHBodyPart::Arm_Left, EOHBodyRegion::UpperBody);
		Map.Add(EOHBodyPart::Arm_Right, EOHBodyRegion::UpperBody);
		Map.Add(EOHBodyPart::Leg_Left, EOHBodyRegion::LowerBody);
		Map.Add(EOHBodyPart::Leg_Right, EOHBodyRegion::LowerBody);
	}
	return Map;
}

#pragma endregion


bool UOHSkeletalPhysicsUtils::IsBoneValidForMap(EOHSkeletalBone Bone, const FBoneContainer& BoneContainer)
{
	const FName BoneName = ResolveBoneName(Bone);
	return BoneName != NAME_None && BoneContainer.GetPoseBoneIndexForBoneName(BoneName) != INDEX_NONE;
}



TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetAllValidBones()
{
	TArray<EOHSkeletalBone> Bones;

	for (uint8 Index = static_cast<uint8>(EOHSkeletalBone::FirstBone);
		 Index <= static_cast<uint8>(EOHSkeletalBone::LastBone);
		 ++Index)
	{
		Bones.Add(static_cast<EOHSkeletalBone>(Index));
	}

	return Bones;
}

TArray<EOHSkeletalBone> UOHSkeletalPhysicsUtils::GetAllValidBones(const FBoneContainer& BoneContainer)
{
	TArray<EOHSkeletalBone> ValidBones;

	for (uint8 i = static_cast<uint8>(EOHSkeletalBone::FirstBone);
		 i <= static_cast<uint8>(EOHSkeletalBone::LastBone);
		 ++i)
	{
		const EOHSkeletalBone Bone = static_cast<EOHSkeletalBone>(i);
		if (IsBoneValidForMap(Bone, BoneContainer))
		{
			ValidBones.Add(Bone);
		}
	}

	return ValidBones;
}

bool UOHSkeletalPhysicsUtils::DoesMapContainBones(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& RequiredBones)
{
	for (EOHSkeletalBone Bone : RequiredBones)
	{
		if (!Map.HasValidReference(Bone))
		{
			return false;
		}
	}
	return true;
}

bool UOHSkeletalPhysicsUtils::DoesMapContainAnyBone(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& Bones)
{
	for (EOHSkeletalBone Bone : Bones)
	{
		if (Map.HasValidReference(Bone))
		{
			return true;
		}
	}
	return false;
}

FOHBoneState* UOHSkeletalPhysicsUtils::GetBoneStateFromContext(FOHBoneContextMap& Context, EOHSkeletalBone Bone)
{
	return Context.FindState(Bone);
}

const FOHBoneState* UOHSkeletalPhysicsUtils::GetBoneStateFromContext(const FOHBoneContextMap& Context, EOHSkeletalBone Bone)
{
	return Context.FindState(Bone);
}


bool UOHSkeletalPhysicsUtils::GetBoneStateFromContext(
	const FOHBoneContextMap& Context,
	EOHSkeletalBone Bone,
	FOHBoneState& OutState)
{
	const FOHBoneState* Found = Context.FindState(Bone);
	if (!Found)
	{
		return false;
	}

	OutState = *Found;
	return true;
}




// -- Create BoneEnum Referemce Map From AnimInstanceProxy
FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const USkeletalMeshComponent* MeshComponent)
{
	FOHBoneReferenceMap Map;

	if (!MeshComponent || !MeshComponent->GetSkeletalMeshAsset())
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: Invalid skeletal mesh component or mesh."));
		return Map;
	}

	const UAnimInstance* AnimInstance = MeshComponent->GetAnimInstance();
	if (!AnimInstance)
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: No AnimInstance on component."));
		return Map;
	}

	const FBoneContainer& BoneContainer = AnimInstance->GetRequiredBones();
	if (!BoneContainer.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: BoneContainer is invalid."));
		return Map;
	}

	Map.Initialize(BoneContainer); // Uses full auto-mapping
	return Map;
}

FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const USkeletalMeshComponent* MeshComp, const TArray<EOHSkeletalBone>& Bones)
{
	if (!MeshComp || !MeshComp->GetAnimInstance())
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: Invalid SkeletalMeshComponent."));
		return FOHBoneReferenceMap();
	}

	return CreateBoneReferenceMap(MeshComp->GetAnimInstance(), Bones);
}


FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const UAnimInstance* AnimInstance)
{
	if (!AnimInstance)
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: Null AnimInstance."));
		return FOHBoneReferenceMap();
	}

	const FBoneContainer& BoneContainer = AnimInstance->GetRequiredBones();
	return CreateBoneReferenceMap(BoneContainer);
}

FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const UAnimInstance* AnimInstance, const TArray<EOHSkeletalBone>& Bones)
{
	if (!AnimInstance)
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: Null AnimInstance."));
		return FOHBoneReferenceMap();
	}

	const FBoneContainer& BoneContainer = AnimInstance->GetRequiredBones();
	return CreateBoneReferenceMap(BoneContainer, Bones);
}


FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const FAnimInstanceProxy* AnimProxy)
{
	FOHBoneReferenceMap Map;

	if (!AnimProxy)
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: AnimProxy is null."));
		return Map;
	}

	const FBoneContainer& BoneContainer = AnimProxy->GetRequiredBones();
	if (!BoneContainer.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: BoneContainer from proxy is invalid."));
		return Map;
	}

	Map.Initialize(BoneContainer);
	return Map;
}


FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const FAnimInstanceProxy* AnimProxy, const TArray<EOHSkeletalBone>& Bones)
{
	if (!AnimProxy)
	{
		UE_LOG(LogTemp, Warning, TEXT("CreateBoneReferenceMap: AnimProxy is null."));
		return FOHBoneReferenceMap();
	}

	const FBoneContainer& BoneContainer = AnimProxy->GetRequiredBones();
	return CreateBoneReferenceMap(BoneContainer, Bones);
}

FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const FAnimationInitializeContext& Context)
{
	const FAnimInstanceProxy* Proxy = Context.AnimInstanceProxy;
	return CreateBoneReferenceMap(Proxy); // clean delegation
}

FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const FAnimationInitializeContext& Context, const TArray<EOHSkeletalBone>& Bones)
{
	const FAnimInstanceProxy* Proxy = Context.AnimInstanceProxy;
	return CreateBoneReferenceMap(Proxy, Bones);
}


FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(const FBoneContainer& BoneContainer)
{
	FOHBoneReferenceMap Map;

	if (!BoneContainer.IsValid())
	{
		UE_LOG(LogTemp, Verbose, TEXT("CreateBoneReferenceMap: BoneContainer is invalid."));
		return Map;
	}

	Map.Initialize(BoneContainer); // Uses full enum auto-mapping
	return Map;
}


FOHBoneReferenceMap UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(
	const FBoneContainer& BoneContainer,
	const TArray<EOHSkeletalBone>& Bones)
{
	FOHBoneReferenceMap Map;

	if (!BoneContainer.IsValid())
	{
		UE_LOG(LogTemp, Verbose, TEXT("CreateBoneReferenceMapForBones: Invalid BoneContainer."));
		return Map;
	}

	for (EOHSkeletalBone Bone : Bones)
	{
		FBoneReference Ref;
		FCompactPoseBoneIndex Index(INDEX_NONE);  // required manual init

		if (TryBuildBoneReference(Bone, BoneContainer, Ref, Index))
		{
			Map.AddEntry(Bone, Ref, Index);
		}
	}

	//  Use your existing internal mutator
	Map.GetMutableInitializedFlag() = Map.GetMappedBones().Num() > 0;
	return Map;
}







bool UOHSkeletalPhysicsUtils::TryGetBoneReferenceFromEnum(
	EOHSkeletalBone BoneEnum,
	const FBoneContainer& BoneContainer,
	FBoneReference& OutReference)
{
	if (BoneEnum == EOHSkeletalBone::None)
	{
		return false;
	}

	const FName BoneName = GetBoneNameFromEnum(BoneEnum);
	const int32 RawIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);

	if (RawIndex == INDEX_NONE)
	{
		return false;
	}

	OutReference.BoneName = BoneName;
	OutReference.Initialize(BoneContainer);

	return OutReference.IsValidToEvaluate(BoneContainer);
}

bool UOHSkeletalPhysicsUtils::TryGetBoneState(const FOHBoneContextMap& Context, EOHSkeletalBone BoneEnum, FOHBoneState& OutState)
{
	const FOHBoneState* Found = Context.FindState(BoneEnum);
	if (Found)
	{
		OutState = *Found;
		return true;
	}
	return false;
}

static const FOHBoneState* TryGetBoneStatePtr(const FOHBoneContextMap& Context, EOHSkeletalBone BoneEnum)
{
	return Context.FindState(BoneEnum);
}

bool UOHSkeletalPhysicsUtils::TryBuildBoneReference(
	EOHSkeletalBone BoneEnum,
	const FBoneContainer& BoneContainer,
	TMap<EOHSkeletalBone, FBoneReference>& OutRefs,
	TMap<EOHSkeletalBone, FCompactPoseBoneIndex>& OutIndices)
{
	const FName BoneName = ResolveBoneName(BoneEnum);
	if (BoneName == NAME_None)
		return false;

	const int32 PoseIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);
	if (PoseIndex == INDEX_NONE)
		return false;

	FBoneReference Ref;
	Ref.BoneName = BoneName;
	Ref.Initialize(BoneContainer);

	if (!Ref.IsValidToEvaluate(BoneContainer))
		return false;

	OutRefs.Add(BoneEnum, Ref);
	OutIndices.Add(BoneEnum, Ref.GetCompactPoseIndex(BoneContainer));
	return true;
}



bool UOHSkeletalPhysicsUtils::TryBuildBoneReference(
	EOHSkeletalBone Bone,
	const FBoneContainer& BoneContainer,
	FBoneReference& OutReference,
	FCompactPoseBoneIndex& OutIndex)
{
	const FName BoneName = ResolveBoneName(Bone);
	if (BoneName == NAME_None)
		return false;

	OutReference.BoneName = BoneName;
	OutReference.Initialize(BoneContainer);

	if (!OutReference.IsValidToEvaluate(BoneContainer))
		return false;

	const int32 RawIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);
	if (RawIndex == INDEX_NONE)
		return false;

	OutIndex = FCompactPoseBoneIndex(RawIndex);
	return true;
}


FName UOHSkeletalPhysicsUtils::ResolveBoneName(EOHSkeletalBone BoneEnum)
{
	return GetBoneNameFromEnum(BoneEnum);
}


#endif


