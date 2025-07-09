#include "Data/Struct/OHPhysicsStructs.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimTypes.h"
#include "Animation/AnimNodeBase.h"
#include "AnimationRuntime.h"
#include "FunctionLibrary/OHAlgoUtils.h"
#include "Utilities/OHSafeMapUtils.h"

// ===================== BoneName Resolver ===================== //

#pragma region BoneNameResolver
void FOHNameResolver::Resolve(const TSet<FName>& Inputs, const TArray<FName>& Candidates)
{
	ResolvedMap.Empty();
	MatchResults.Empty();

	for (const FName& Input : Inputs)
	{
	const FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(Input, Candidates);
		if (!Match.Candidate.IsEmpty())
		{
			const FName Resolved = FName(*Match.Candidate);
			ResolvedMap.Add(Input, Resolved);
			MatchResults.Add(Input, Match);
		}
	}
}

FName FOHNameResolver::GetResolved(FName Input) const
{
	if (const FName* Found = ResolvedMap.Find(Input))
	{
		return *Found;
	}
	return NAME_None;
}

FOHNameMatchResult FOHNameResolver::GetMatchResult(FName Input) const
{
	if (const FOHNameMatchResult* Found = MatchResults.Find(Input))
	{
		return *Found;
	}
	return FOHNameMatchResult();
}

bool FOHNameResolver::HasResolved(FName Input) const
{
	return ResolvedMap.Contains(Input);
}

#pragma endregion 

// ====================== Physics Graph Node Implementation ====================== //

#pragma region PhysicsGraphNode


bool FOHPhysicsGraphNode::AddBone(const FOHBoneData& Bone)
{
    if (!Bone.IsValid())
        return false;

    BoneMap.Add(Bone.GetBoneName(), Bone);
    RebuildWrappers();
    return true;
}

bool FOHPhysicsGraphNode::RemoveBone(const FName& BoneName)
{
    if (!BoneMap.Contains(BoneName))
        return false;

    BoneMap.Remove(BoneName);

    ConstraintLinks.RemoveAll([&](const FOHConstraintInstanceData& Constraint)
    {
        return Constraint.GetParentBone() == BoneName || Constraint.GetChildBone() == BoneName;
    });

    RebuildWrappers();
    return true;
}

bool FOHPhysicsGraphNode::AddConstraint(const FOHConstraintInstanceData& Constraint)
{
    if (!IsBoneValid(Constraint.GetParentBone()) || !IsBoneValid(Constraint.GetChildBone()))
        return false;

    int32 FoundIdx = ConstraintLinks.IndexOfByPredicate([&](const FOHConstraintInstanceData& Existing)
    {
        return Existing.GetConstraintName() == Constraint.GetConstraintName();
    });

    if (FoundIdx != INDEX_NONE)
        ConstraintLinks[FoundIdx] = Constraint;
    else
        ConstraintLinks.Add(Constraint);

    RebuildWrappers();
    return true;
}

bool FOHPhysicsGraphNode::RemoveConstraint(const FName& ConstraintName)
{
    int32 Removed = ConstraintLinks.RemoveAll([&](const FOHConstraintInstanceData& C)
    {
        return C.GetConstraintName() == ConstraintName;
    });
    if (Removed > 0)
    {
        RebuildWrappers();
        return true;
    }
    return false;
}
void FOHPhysicsGraphNode::RebuildWrappers()
{
	// 1) Clear out the new multimaps
	ParentToChildrenMultiMap.Reset();
	BoneToConstraintsMultiMap.Reset();

	// 2) Populate parent→children
	for (const auto& Pair : BoneMap)
	{
		const FName& Child  = Pair.Key;
		const FName  Parent = Pair.Value.GetParentBone();
		if (Parent != NAME_None)
		{
			ParentToChildrenMultiMap.Add(Parent, Child);
		}
	}

	// 3) Populate bone→constraint pointers
	for (FOHConstraintInstanceData& C : ConstraintLinks)
	{
		BoneToConstraintsMultiMap.Add(C.GetParentBone(), &C);
		BoneToConstraintsMultiMap.Add(C.GetChildBone(),  &C);
	}
}

bool FOHPhysicsGraphNode::ValidateGraph(bool bLog, bool bAutoRepair)
{
    bool bAllValid = true;

    // 1. Remove invalid constraints (referencing missing or invalid bones)
    int32 OldConstraintNum = ConstraintLinks.Num();

    OHSafeMapUtils::RemoveIfAnyMissingOrInvalid<FName, FOHBoneData, FOHConstraintInstanceData>(
        ConstraintLinks,
        BoneMap,
        [](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
        [](const FOHConstraintInstanceData& C) { return C.GetChildBone(); },
        [](const FOHBoneData& BoneData) { return BoneData.IsValid(); }
    );

    if (ConstraintLinks.Num() != OldConstraintNum)
    {
        bAllValid = false;
        if (bLog)
        {
            UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraphNode] Removed %d invalid constraints referencing missing or invalid bones."),
                OldConstraintNum - ConstraintLinks.Num());
        }
    }

    // 2. Find and log orphans (use TSet to ensure uniqueness)
    TSet<FName> ReferencedBones;
    for (const FOHConstraintInstanceData& Constraint : ConstraintLinks)
    {
        ReferencedBones.Add(Constraint.GetParentBone());
        ReferencedBones.Add(Constraint.GetChildBone());
    }

    TSet<FName> OrphanBones;
    for (const auto& BonePair : BoneMap)
    {
        const FName& BoneName = BonePair.Key;
        if (!ReferencedBones.Contains(BoneName))
        {
            OrphanBones.Add(BoneName);
            bAllValid = false;
            if (bLog)
                UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraphNode] Orphan bone: %s (not referenced by any constraint)"), *BoneName.ToString());
        }
    }

    // 3. Optionally, remove orphans (auto repair)
    if (bAutoRepair && OrphanBones.Num() > 0)
    {
        for (const FName& BoneName : OrphanBones)
            BoneMap.Remove(BoneName);
        if (bLog)
            UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraphNode] Removed %d orphan bones."), OrphanBones.Num());
    }

    // 4. Check for cycles
    if (HasCycles())
    {
        bAllValid = false;
        if (bLog)
            UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraphNode] Constraint graph contains at least one cycle!"));
    }

    // 5. Check islands (disconnected subgraphs)
    TArray<TSet<FName>> Islands = GetIslands();
    if (Islands.Num() > 1)
    {
        bAllValid = false;
        if (bLog)
            UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraphNode] Found %d disconnected islands (subgraphs) in the graph."), Islands.Num());
    }

    // 6. Log result
    if (bLog)
    {
        if (bAllValid)
        	{
        		UE_LOG(LogTemp, Log, TEXT("[PhysicsGraphNode] Graph validation PASSED: all bones and constraints are valid and connected."));
        	}
        else
        {
        	UE_LOG(LogTemp, Error, TEXT("[PhysicsGraphNode] Graph validation FAILED. See warnings above for issues."));
        }
    }
    return bAllValid;
}


TArray<TSet<FName>> FOHPhysicsGraphNode::GetIslands(bool bBidirectional) const
{
	// Build a set of every bone in the graph
	TSet<FName> AllBones;
	for (const auto& Pair : BoneMap)
	{
		AllBones.Add(Pair.Key);
	}

	return OHSafeMapUtils::BuildGraphComponentSets<FName>(
		AllBones,
		[this, bBidirectional](const FName& Node)
		{
			TArray<FName> Adj;

			// 1) Down‐links (children)
			ParentToChildrenMultiMap.MultiFind(Node, Adj, /*bMaintainOrder=*/true);

			// 2) Up‐links (parent) if bidirectional
			if (bBidirectional)
			{
				if (const FOHBoneData* BoneData = BoneMap.Find(Node))
				{
					FName Parent = BoneData->GetParentBone();
					if (Parent != NAME_None)
					{
						Adj.Add(Parent);
					}
				}
			}

			return Adj;
		}
	);
}


TSet<FName> FOHPhysicsGraphNode::CaptureReachableBones(const FName& StartBone) const
{
	return OHSafeMapUtils::CaptureGraphSnapshot<FName>(
		StartBone,
		[this](const FName& Node)
		{
			TArray<FName> Adj;
			ParentToChildrenMultiMap.MultiFind(Node, Adj, /*bMaintainOrder=*/true);
			return Adj;
		}
	);
}


TSet<FName> FOHPhysicsGraphNode::GetOrphanBones() const
{
    // Bones not referenced in any constraint
    TSet<FName> Orphans;
    TSet<FName> Referenced;
    for (const FOHConstraintInstanceData& Constraint : ConstraintLinks)
    {
        Referenced.Add(Constraint.GetParentBone());
        Referenced.Add(Constraint.GetChildBone());
    }
    for (const auto& Pair : BoneMap)
    {
        if (!Referenced.Contains(Pair.Key))
            Orphans.Add(Pair.Key);
    }
    return Orphans;
}


TSet<FName> FOHPhysicsGraphNode::CaptureConstraintNames() const
{
    TSet<FName> Names;
    for (const FOHConstraintInstanceData& C : ConstraintLinks)
        Names.Add(C.GetConstraintName());
    return Names;
}



bool FOHPhysicsGraphNode::IsConnected() const
{
    auto Islands = GetIslands();
    return (Islands.Num() == 1);
}

void FOHPhysicsGraphNode::InitializeBonesFromSource(const TArray<FOHBoneData*>& Sources)
{
	TArray<FOHBoneData> Result = OHSafeMapUtils::InitializeObjectsFromSourceArray<FOHBoneData, FOHBoneData>(
		Sources,
		[](FOHBoneData& Out, const FOHBoneData* Src)
		{
			if (!Src) return false;
			Out = *Src;
			return Out.IsValid();
		});
	BoneMap.Empty();
	for (const FOHBoneData& Bone : Result)
	{
		BoneMap.Add(Bone.GetBoneName(), Bone);
	}
}


void FOHPhysicsGraphNode::InitializeConstraintsFromSource(const TArray<FOHConstraintInstanceData*>& Sources)
{
	ConstraintLinks = OHSafeMapUtils::InitializeObjectsFromSourceArray<FOHConstraintInstanceData, FOHConstraintInstanceData>(
		Sources,
		[](FOHConstraintInstanceData& Out, const FOHConstraintInstanceData* Src)
		{
			if (!Src) return false;
			Out = *Src;
			return true;
		});
}

void FOHPhysicsGraphNode::RemoveInvalidConstraints()
{
    OHSafeMapUtils::RemoveIfAnyMissingOrInvalid<FName, FOHBoneData, FOHConstraintInstanceData>(
        ConstraintLinks,
        BoneMap,
        [](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
        [](const FOHConstraintInstanceData& C) { return C.GetChildBone(); },
        [](const FOHBoneData& B) { return B.IsValid(); }
    );
}

void FOHPhysicsGraphNode::RemoveOrphanBones()
{
    // Remove bones not referenced in any constraint
    TSet<FName> Referenced;
    for (const FOHConstraintInstanceData& Constraint : ConstraintLinks)
    {
        Referenced.Add(Constraint.GetParentBone());
        Referenced.Add(Constraint.GetChildBone());
    }
    TArray<FName> ToRemove;
    for (const auto& Pair : BoneMap)
        if (!Referenced.Contains(Pair.Key))
            ToRemove.Add(Pair.Key);
    for (const FName& BoneName : ToRemove)
        BoneMap.Remove(BoneName);
}


TArray<FName> FOHPhysicsGraphNode::GetInvalidBones(TArray<FString>* OutReasons) const
{
    TArray<FName> Invalid;
    for (const auto& Pair : BoneMap)
    {
        const FName& Bone = Pair.Key;
        FString Reason;
        bool bBad = false;

        // No parent or child at all? (Orphan)
        bool bIsOrphan = true;
        for (const auto& C : ConstraintLinks)
        {
            if (C.GetParentBone() == Bone || C.GetChildBone() == Bone)
            {
                bIsOrphan = false;
                break;
            }
        }
        if (bIsOrphan)
        {
            bBad = true;
            Reason = TEXT("Orphan (not referenced in any constraint)");
        }
        // Duplicate check not needed for TMap, but can add more validation as needed

        if (bBad)
        {
            Invalid.Add(Bone);
            if (OutReasons) OutReasons->Add(Reason);
        }
    }
    return Invalid;
}

TArray<FName> FOHPhysicsGraphNode::GetInvalidConstraints(TArray<FString>* OutReasons) const
{
    TArray<FName> Invalid;
    for (const FOHConstraintInstanceData& C : ConstraintLinks)
    {
        bool bBad = false;
        FString Reason;
        if (!IsBoneValid(C.GetParentBone()))
        {
            bBad = true;
            Reason = FString::Printf(TEXT("Parent bone '%s' not found."), *C.GetParentBone().ToString());
        }
        else if (!IsBoneValid(C.GetChildBone()))
        {
            bBad = true;
            Reason = FString::Printf(TEXT("Child bone '%s' not found."), *C.GetChildBone().ToString());
        }
        if (bBad)
        {
            Invalid.Add(C.GetConstraintName());
            if (OutReasons) OutReasons->Add(Reason);
        }
    }
    return Invalid;
}

TSet<FName> FOHPhysicsGraphNode::CaptureBoneSnapshot(const FName& RootBone) const
{
	return OHSafeMapUtils::CaptureGraphSnapshot<FName>(
		RootBone,
		[this](const FName& Node)
		{
			return GetChildrenOfBone(Node);
		}
	);
}

TSet<FName> FOHPhysicsGraphNode::CaptureConstraintSnapshot() const
{
	TSet<FName> Out;
	for (const FOHConstraintInstanceData& C : ConstraintLinks)
		Out.Add(C.GetConstraintName());
	return Out;
}

void FOHPhysicsGraphNode::DiffBoneSnapshots(const TSet<FName>& OldSnap, const TSet<FName>& NewSnap, TSet<FName>& OutRemoved, TSet<FName>& OutAdded) const
{
	OHSafeMapUtils::DiffGraphStates(OldSnap, NewSnap, OutRemoved, OutAdded);
}

TArray<FName> FOHPhysicsGraphNode::GetRootBones() const
{
    TSet<FName> AllBones, ChildBones;
    BoneMap.GetKeys(AllBones);
    for (const FOHConstraintInstanceData& C : ConstraintLinks)
        ChildBones.Add(C.GetChildBone());
    TArray<FName> Roots;
    for (const FName& Bone : AllBones)
        if (!ChildBones.Contains(Bone))
            Roots.Add(Bone);
    return Roots;
}

TArray<FName> FOHPhysicsGraphNode::GetLeafBones() const
{
    TSet<FName> AllBones, ParentBones;
    BoneMap.GetKeys(AllBones);
    for (const FOHConstraintInstanceData& C : ConstraintLinks)
        ParentBones.Add(C.GetParentBone());
    TArray<FName> Leaves;
    for (const FName& Bone : AllBones)
        if (!ParentBones.Contains(Bone))
            Leaves.Add(Bone);
    return Leaves;
}


bool FOHPhysicsGraphNode::HasCycles(TArray<FName>* OutCycle) const
{
	// Gather all nodes
	TSet<FName> AllBones;
	for (const auto& Pair : BoneMap)
	{
		AllBones.Add(Pair.Key);
	}

	// Try cycle detection from each root
	for (const FName& Root : AllBones)
	{
		TArray<FName> Cycle;
		bool bFound = OHSafeMapUtils::DetectCyclesInGraph<FName>(
			Root,
			[this](const FName& Node)
			{
				TArray<FName> Adj;
				ParentToChildrenMultiMap.MultiFind(Node, Adj, /*bMaintainOrder=*/true);
				return Adj;
			},
			Cycle
		);

		if (bFound)
		{
			if (OutCycle) *OutCycle = Cycle;
			return true;
		}
	}

	return false;
}

TArray<FName> FOHPhysicsGraphNode::FindShortestPath(FName Start, FName End) const
{
	return OHSafeMapUtils::FindShortestPathInGraph<FName>(
		Start, End,
		[this](const FName& Node)
		{
			TArray<FName> Adj;
			ParentToChildrenMultiMap.MultiFind(Node, Adj, /*bMaintainOrder=*/true);
			return Adj;
		}
	);
}


void FOHPhysicsGraphNode::PrintGraphState(const FString& Tag) const
{
    UE_LOG(LogTemp, Log, TEXT("[%s] BoneCount: %d, ConstraintCount: %d"), *Tag, BoneMap.Num(), ConstraintLinks.Num());
    UE_LOG(LogTemp, Log, TEXT("Roots: %s"), *FString::JoinBy(GetRootBones(), TEXT(", "), [](const FName& Name){ return Name.ToString(); }));
    UE_LOG(LogTemp, Log, TEXT("Leaves: %s"), *FString::JoinBy(GetLeafBones(), TEXT(", "), [](const FName& Name){ return Name.ToString(); }));
    UE_LOG(LogTemp, Log, TEXT("HasCycles: %s"), HasCycles() ? TEXT("YES") : TEXT("NO"));
    auto Islands = GetIslands();
    for (int32 i = 0; i < Islands.Num(); ++i)
    {
        UE_LOG(LogTemp, Log, TEXT("Island %d: %s"), i,
            *FString::JoinBy(Islands[i].Array(), TEXT(", "), [](const FName& N){ return N.ToString(); }));
    }

    // Print invalid bones/constraints
    TArray<FString> BoneReasons, ConstraintReasons;
    auto BadBones = GetInvalidBones(&BoneReasons);
    auto BadConstraints = GetInvalidConstraints(&ConstraintReasons);

    for (int32 i = 0; i < BadBones.Num(); ++i)
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid Bone: %s — %s"), *BadBones[i].ToString(),
            BoneReasons.IsValidIndex(i) ? *BoneReasons[i] : TEXT("Unknown reason"));
    }
    for (int32 i = 0; i < BadConstraints.Num(); ++i)
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid Constraint: %s — %s"), *BadConstraints[i].ToString(),
            ConstraintReasons.IsValidIndex(i) ? *ConstraintReasons[i] : TEXT("Unknown reason"));
    }
}


#pragma endregion

// ===================== BoneData Implementation ===================== //

#pragma region BoneData

// Update Kinematics

void FOHBoneData::UpdateKinematics(const FVector& NewPosition, const FQuat& NewRotation, float DeltaTime, float TimeStamp)
{
	if (DeltaTime <= KINDA_SMALL_NUMBER)
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateKinematics: DeltaTime too small — update aborted"));
		return;
	}
	if (!NewRotation.IsNormalized())
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateKinematics: Invalid rotation (not normalized) — update aborted"));
		return;
	}

	SetLastDeltaTime(DeltaTime);
	const float SafeDelta = GetLastDeltaTime();

	// --- Linear Motion ---
	const FVector OldPosition = GetCurrentPosition();
	const FVector RawLinearVelocity = (NewPosition - OldPosition) / SafeDelta;
	const FVector RawLinearAccel = (RawLinearVelocity - GetBodyLinearVelocity()) / SafeDelta;

	SetPreviousPosition(OldPosition);
	SetCurrentPosition(NewPosition);
	SetLinearVelocity(RawLinearVelocity);
	SetLinearAcceleration(RawLinearAccel);

	// --- Angular Motion ---
	const FQuat OldRotation = GetCurrentRotation();
	const FQuat DeltaQuat = NewRotation * OldRotation.Inverse();
	if (!DeltaQuat.IsNormalized())
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateKinematics: DeltaQuat not normalized — update aborted"));
		return;
	}

	FVector Axis;
	float Angle;
	DeltaQuat.ToAxisAndAngle(Axis, Angle);
	const FVector RawAngularVelocity = (Axis * Angle) / SafeDelta;
	const FVector RawAngularAccel = (RawAngularVelocity - GetBodyAngularVelocity()) / SafeDelta;

	SetPreviousRotation(OldRotation);
	SetCurrentRotation(NewRotation);
	SetAngularVelocity(RawAngularVelocity);
	SetAngularAcceleration(RawAngularAccel);

	// --- Store History Sample ---
	AddMotionSample(FTransform(NewRotation, NewPosition), TimeStamp, SafeDelta);

	// --- Stabilize Values with Historical Estimators ---
	if (MotionHistory.Num() >= GetDefaultSampleCount())
	{
		SetLinearVelocity(GetBlendedLinearVelocity());
		SetAngularVelocity(GetBlendedAngularVelocity());

		SetLinearAcceleration(GetEstimatedLinearAcceleration());
		SetAngularAcceleration(GetEstimatedAngularAcceleration());
	}
}


void FOHBoneData::UpdateFromComponent(float CurrentTime)
{

	if (!GetOwnerComponent())
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateFromComponent: Null mesh"));
		return;
	}
	USkeletalMeshComponent* Mesh = GetOwnerComponent();

	if (!Mesh->DoesSocketExist(BoneName))
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateFromComponent: Socket %s does not exist"), *BoneName.ToString());
		return;
	}

	const FVector NewPos = Mesh->GetSocketLocation(BoneName);
	const FQuat NewRot = Mesh->GetSocketQuaternion(BoneName);
	UpdateKinematics(NewPos, NewRot, LastDeltaTime, CurrentTime);
}


void FOHBoneData::UpdateFromAnimationPose(float TimeStamp)
{

	if (!GetOwnerComponent())
		return;
	USkeletalMeshComponent* Mesh = GetOwnerComponent();

	if (!Mesh->DoesSocketExist(BoneName))
		return;

	const int32 PoseBoneIndex = Mesh->GetBoneIndex(BoneName);
	if (PoseBoneIndex == INDEX_NONE)
		return;

	const FTransform BoneAnimTransform = Mesh->GetBoneTransform(PoseBoneIndex, FTransform::Identity);
	UpdateKinematics(BoneAnimTransform.GetLocation(), BoneAnimTransform.GetRotation(), LastDeltaTime, TimeStamp);
}


float FOHBoneData::GetLocalBoneCrowding() const
{
	if (!GetOwnerComponent()) return 0.f;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	TArray<FVector> Points;
	FTransform MyTM = SkelMesh->GetSocketTransform(BoneName, RTS_World);
	Points.Add(MyTM.GetLocation());
	FName Parent = GetParentBoneName();
	if (Parent != NAME_None)
		Points.Add(SkelMesh->GetSocketTransform(Parent, RTS_World).GetLocation());
	for (const FName& Child : FindChildBones())
		Points.Add(SkelMesh->GetSocketTransform(Child, RTS_World).GetLocation());
	float Sum = 0.f, Count = 0.f;
	for (int32 i = 1; i < Points.Num(); ++i)
	{
		Sum += FVector::Dist(Points[0], Points[i]);
		Count++;
	}
	return (Count > 0.f) ? (Sum / Count) : 0.f; // Smaller = more crowded
}


TArray<FName> FOHBoneData::GetBonesOfMaxDepthUp(int32 MaxDepth) const
{
	TArray<FName> OutBones;
	if (!GetOwnerComponent() || MaxDepth < 1) return OutBones;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);

	int32 Depth = 0;
	while (MyIdx != INDEX_NONE && Depth < MaxDepth)
	{
		int32 ParentIdx = RefSkel.GetParentIndex(MyIdx);
		if (ParentIdx == INDEX_NONE) break;
		OutBones.Add(RefSkel.GetBoneName(ParentIdx));
		MyIdx = ParentIdx;
		++Depth;
	}
	return OutBones;
}

TArray<FName> FOHBoneData::FindChildBones() const
{
	TArray<FName> OutChildren;
	if (!GetOwnerComponent()) return OutChildren;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	for (int32 i = 0; i < RefSkel.GetNum(); ++i)
		if (RefSkel.GetParentIndex(i) == MyIdx)
			OutChildren.Add(RefSkel.GetBoneName(i));
	return OutChildren;
}

TArray<FBodyInstance*> FOHBoneData::GetChildBodyInstances() const
{
	TArray<FBodyInstance*> OutBodies;
	if (!GetOwnerComponent()) return OutBodies;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	
	for (const FName& Child : FindChildBones())
	{
		if (FBodyInstance* BI = SkelMesh->GetBodyInstance(Child))
			OutBodies.Add(BI);
	}
	return OutBodies;
}

FName FOHBoneData::GetParentBoneName() const
{
	if (!GetOwnerComponent()) return NAME_None;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 ParentIdx = RefSkel.GetParentIndex(MyIdx);
	return (ParentIdx != INDEX_NONE) ? RefSkel.GetBoneName(ParentIdx) : NAME_None;
}

int32 FOHBoneData::GetParentBoneIndex() const
{
	if (!GetOwnerComponent()) return int32 { INDEX_NONE };
	USkeletalMeshComponent* SkeletalMesh = GetOwnerComponent();
	if (!SkeletalMesh || BoneName.IsNone()) return INDEX_NONE;
	int32 BoneIdx = SkeletalMesh->GetBoneIndex(BoneName);
	if (BoneIdx == INDEX_NONE) return INDEX_NONE;
	const FReferenceSkeleton& RefSkel = SkeletalMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	return RefSkel.GetParentIndex(BoneIdx);
}


TArray<FName> FOHBoneData::GetBonesOfMaxDepthDown(int32 MaxDepth) const
{
	TArray<FName> OutBones;
	if (!GetOwnerComponent()) return OutBones;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	if (!SkelMesh || MaxDepth < 1) return OutBones;
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);
	if (MyIdx == INDEX_NONE) return OutBones;

	// Breadth-first search
	struct FSearch
	{
		int32 BoneIdx;
		int32 Depth;
	};
	TArray<FSearch> Stack;
	Stack.Add({ MyIdx, 0 });

	while (Stack.Num())
	{
		FSearch Cur = Stack.Pop();
		if (Cur.Depth >= 1) // Don't include the root itself
			OutBones.Add(RefSkel.GetBoneName(Cur.BoneIdx));
		if (Cur.Depth >= MaxDepth) continue;

		for (int32 i = 0; i < RefSkel.GetNum(); ++i)
		{
			if (RefSkel.GetParentIndex(i) == Cur.BoneIdx)
				Stack.Add({ i, Cur.Depth + 1 });
		}
	}
	return OutBones;
}

float FOHBoneData::GetEstimatedBoneLength() const
{
	if (!GetOwnerComponent()) return 0.f;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	if (!SkelMesh) return 0.f;
	int32 BoneIdx = SkelMesh->GetBoneIndex(BoneName);
	if (BoneIdx == INDEX_NONE) return 0.f;
	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 ChildIdx = INDEX_NONE;
	// Find first child (simple heuristic; for robust, loop all children)
	for (int32 i = 0; i < RefSkel.GetNum(); ++i)
		if (RefSkel.GetParentIndex(i) == BoneIdx)
		{ ChildIdx = i; break; }
	if (ChildIdx == INDEX_NONE) return 0.f;
	FVector BonePos = RefSkel.GetRefBonePose()[BoneIdx].GetLocation();
	FVector ChildPos = RefSkel.GetRefBonePose()[ChildIdx].GetLocation();
	return FVector::Dist(BonePos, ChildPos);
}



FConstraintInstance* FOHBoneData::GetParentConstraintInstance() const
{
	USkeletalMeshComponent* SkeletalMesh = GetOwnerComponent();

	if (!SkeletalMesh || BoneName.IsNone()) return nullptr;
	FName ParentName = GetParentBoneName();
	if (ParentName.IsNone()) return nullptr;
	UPhysicsAsset* PhysAsset = SkeletalMesh->GetPhysicsAsset();
	if (!PhysAsset) return nullptr;
	for (UPhysicsConstraintTemplate* ConstraintTemplate : PhysAsset->ConstraintSetup)
	{
		if (!ConstraintTemplate) continue;
		if (FConstraintInstance* CI = &ConstraintTemplate->DefaultInstance; (CI->ConstraintBone1 == ParentName && CI->ConstraintBone2 == BoneName) ||
			(CI->ConstraintBone1 == BoneName && CI->ConstraintBone2 == ParentName))
		{
			return CI;
		}
	}
	return nullptr;
}
TArray<FConstraintInstance*> FOHBoneData::GetAllParentConstraints() const
{
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
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



TArray<FName> FOHBoneData::GetAllParentBoneNames() const
{
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
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

TArray<FConstraintInstance*> FOHBoneData::GetChildConstraintsDownToDepth(int32 MaxDepth) const
{
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	TArray<FConstraintInstance*> Result;
	if (!SkelMesh || MaxDepth < 1) return Result;
	UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset();
	if (!PhysAsset) return Result;

	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 RootIdx = SkelMesh->GetBoneIndex(BoneName);

	// Breadth-first search with depth tracking
	struct FSearch
	{
		int32 BoneIdx;
		int32 Depth;
	};
	TArray<FSearch> Stack;
	Stack.Add({ RootIdx, 0 });

	while (Stack.Num())
	{
		FSearch Cur = Stack.Pop();
		if (Cur.Depth >= MaxDepth) continue;
		// Find children of Cur.BoneIdx
		for (int32 i = 0; i < RefSkel.GetNum(); ++i)
		{
			if (RefSkel.GetParentIndex(i) == Cur.BoneIdx)
			{
				FName ChildName = RefSkel.GetBoneName(i);
				FName ParentName = RefSkel.GetBoneName(Cur.BoneIdx);

				for (UPhysicsConstraintTemplate* ConstraintTemplate : PhysAsset->ConstraintSetup)
				{
					if (!ConstraintTemplate) continue;
					FConstraintInstance* CI = const_cast<FConstraintInstance*>(&ConstraintTemplate->DefaultInstance);
					if ((CI->ConstraintBone1 == ParentName && CI->ConstraintBone2 == ChildName) ||
						(CI->ConstraintBone1 == ChildName && CI->ConstraintBone2 == ParentName))
					{
						Result.Add(CI);
						break;
					}
				}
				Stack.Add({ i, Cur.Depth + 1 });
			}
		}
	}
	return Result;
}


TArray<FConstraintInstance*> FOHBoneData::GetAllChildConstraints() const
{
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();

	TArray<FConstraintInstance*> Result;
	if (!SkelMesh) return Result;
	UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset();
	if (!PhysAsset) return Result;

	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 MyIdx = SkelMesh->GetBoneIndex(BoneName);
	int32 NumBones = RefSkel.GetNum();

	for (int32 i = 0; i < NumBones; ++i)
	{
		if (RefSkel.GetParentIndex(i) == MyIdx)
		{
			FName ChildName = RefSkel.GetBoneName(i);

			for (UPhysicsConstraintTemplate* ConstraintTemplate : PhysAsset->ConstraintSetup)
			{
				if (!ConstraintTemplate) continue;
				FConstraintInstance* CI = const_cast<FConstraintInstance*>(&ConstraintTemplate->DefaultInstance);
				if ((CI->ConstraintBone1 == BoneName && CI->ConstraintBone2 == ChildName) ||
					(CI->ConstraintBone1 == ChildName && CI->ConstraintBone2 == BoneName))
				{
					Result.Add(CI);
					break;
				}
			}
			// Optionally recurse down further for all descendants
		}
	}
	return Result;
}

TArray<FConstraintInstance*> FOHBoneData::GetParentConstraintsUpToDepth(int32 MaxDepth) const
{
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	TArray<FConstraintInstance*> Result;
	if (!SkelMesh || MaxDepth < 1) return Result;
	UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset();
	if (!PhysAsset) return Result;

	const FReferenceSkeleton& RefSkel = SkelMesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	int32 CurIdx = SkelMesh->GetBoneIndex(BoneName);
	int32 Depth = 0;

	while (CurIdx != INDEX_NONE && Depth < MaxDepth)
	{
		int32 ParentIdx = RefSkel.GetParentIndex(CurIdx);
		if (ParentIdx == INDEX_NONE) break;
		FName ParentName = RefSkel.GetBoneName(ParentIdx);
		FName ThisName = RefSkel.GetBoneName(CurIdx);

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
		CurIdx = ParentIdx;
		++Depth;
	}
	return Result;
}

// Motion Sampling
void FOHBoneData::AddMotionSample(const FTransform& NewTransform, float CurrentTime, float DeltaTime)
{
	if (DeltaTime <= KINDA_SMALL_NUMBER || !NewTransform.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("AddMotionSample: Invalid data — sample skipped"));
		return;
	}

	FOHMotionSample Sample = FOHMotionSample::CreateFromState(
		NewTransform,
		GetBodyLinearVelocity(),
		static_cast<FVector>(GetPhysicsBlendWeight()),
		GetLinearAcceleration(),
		GetAngularAcceleration(),
		CurrentTime
	);

	if (!Sample.IsValidSample())
	{
		UE_LOG(LogTemp, Warning, TEXT("AddMotionSample: Sample failed IsValidSample — sample skipped"));
		return;
	}

	Sample.ClampValues();

	MotionHistory.Add(Sample);
	if (MotionHistory.Num() > MaxSamples)
	{
		MotionHistory.RemoveAt(0);
	}
}


void FOHBoneData::TrimMotionHistoryByAge(float MaxAge, float CurrentTime)
{
	for (int32 i = MotionHistory.Num() - 1; i >= 0; --i)
	{
		if (CurrentTime - MotionHistory[i].GetTimeStamp() > MaxAge)
		{
			MotionHistory.RemoveAt(i);
		}
	}
}

FOHMotionSample FOHBoneData::GetLatestMotionSample() const
{
	return MotionHistory.Num() > 0 ? MotionHistory.Last() : FOHMotionSample();
}


FOHMotionSample FOHBoneData::GetClosestSampleByTime(float Time) const
{
	float BestDiff = TNumericLimits<float>::Max();
	FOHMotionSample Closest;

	for (const FOHMotionSample& Sample : MotionHistory)
	{
		const float Diff = FMath::Abs(Sample.GetTimeStamp() - Time);
		if (Diff < BestDiff)
		{
			BestDiff = Diff;
			Closest = Sample;
		}
	}

	return Closest;
}


FOHMotionSample FOHBoneData::GetPeakVelocitySample() const
{
	FOHMotionSample PeakSample;
	float MaxSpeed = -1.f;

	for (const FOHMotionSample& Sample : MotionHistory)
	{
		const float Speed = Sample.GetLinearSpeed();
		if (Speed > MaxSpeed)
		{
			MaxSpeed = Speed;
			PeakSample = Sample;
		}
	}

	return PeakSample;
}

// Smooth Forces



FVector FOHBoneData::GetSmoothedLinearVelocity() const
{
	if (MotionHistory.Num() < 2 || LastDeltaTime <= KINDA_SMALL_NUMBER)
	{
		return LinearVelocity;
	}

	const FVector& PosA = MotionHistory[1].GetLocation();
	const FVector& PosB = MotionHistory[0].GetLocation();

	return (PosB - PosA) / LastDeltaTime;
}

FVector FOHBoneData::GetSmoothedAngularVelocity() const
{
	if (MotionHistory.Num() < 2 || LastDeltaTime <= KINDA_SMALL_NUMBER)
	{
		return AngularVelocity;
	}

	const FQuat RotA = MotionHistory[1].GetRotation();
	const FQuat RotB = MotionHistory[0].GetRotation();
	const FQuat Delta = RotB * RotA.Inverse();

	if (!Delta.IsNormalized())
	{
		return AngularVelocity;
	}

	FVector Axis;
	float Angle;
	Delta.ToAxisAndAngle(Axis, Angle);

	return (Axis * Angle) / LastDeltaTime;
}

FVector FOHBoneData::GetSmoothedLinearAcceleration() const
{
	const FVector Current = GetSmoothedLinearVelocity();
	const FVector Previous = (PreviousPosition.IsZero() || LastDeltaTime <= KINDA_SMALL_NUMBER)
		? Current : (PreviousPosition - GetPositionBeforePrevious()) / LastDeltaTime;

	return (Current - Previous) / LastDeltaTime;
}

FVector FOHBoneData::GetSmoothedAngularAcceleration() const
{
	const FVector Current = GetSmoothedAngularVelocity();
	const FVector Previous = (MotionHistory.Num() >= 2) ? MotionHistory[MotionHistory.Num() - 2].GetAngularVelocity() : AngularVelocity;
	return (Current - Previous) / LastDeltaTime;
}

FVector FOHBoneData::GetAverageLinearVelocity(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return LinearVelocity;

	FVector Sum = FVector::ZeroVector;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Sum += MotionHistory[i].GetLinearVelocity();
	}

	return Count > 0 ? Sum / Count : LinearVelocity;
}

FVector FOHBoneData::GetAverageAngularVelocity(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return AngularVelocity;

	FVector Sum = FVector::ZeroVector;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Sum += MotionHistory[i].GetAngularVelocity();
	}

	return Count > 0 ? Sum / Count : AngularVelocity;
}

FVector FOHBoneData::GetAverageLinearAcceleration(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return LinearAcceleration;

	FVector Sum = FVector::ZeroVector;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Sum += MotionHistory[i].GetLinearAcceleration();
	}

	return Count > 0 ? Sum / Count : LinearAcceleration;
}

FVector FOHBoneData::GetAverageAngularAcceleration(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return AngularAcceleration;

	FVector Sum = FVector::ZeroVector;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Sum += MotionHistory[i].GetAngularAcceleration();
	}

	return Count > 0 ? Sum / Count : AngularAcceleration;
}

FVector FOHBoneData::GetAveragePosition(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return WorldPosition;

	FVector Sum = FVector::ZeroVector;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Sum += MotionHistory[i].GetLocation();
	}

	return Count > 0 ? Sum / Count : WorldPosition;
}

FQuat FOHBoneData::GetAverageRotation(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return CurrentRotation;

	TArray<FQuat> Rotations;
	Rotations.Reserve(SampleCount);

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Rotations.Num() < SampleCount; --i)
	{
		const FQuat& Rot = MotionHistory[i].GetRotation();
		if (Rot.IsNormalized())
		{
			Rotations.Add(Rot);
		}
	}

	if (Rotations.Num() == 0) return CurrentRotation;

	// Average using Slerp accumulation (more stable)
	FQuat Avg = Rotations[0];
	for (int32 i = 1; i < Rotations.Num(); ++i)
	{
		const float T = 1.f / (i + 1);
		Avg = FQuat::Slerp(Avg, Rotations[i], T).GetNormalized();
	}

	return Avg;
}

float FOHBoneData::GetAverageLinearSpeed(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return LinearVelocity.Size();

	float Total = 0.f;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Total += MotionHistory[i].GetLinearVelocity().Size();
	}

	return Count > 0 ? Total / Count : LinearVelocity.Size();
}

float FOHBoneData::GetAverageAngularSpeed(int32 SampleCount) const
{
	if (SampleCount <= 0) SampleCount = GetDefaultSampleCount();
	if (MotionHistory.Num() == 0) return AngularVelocity.Size();

	float Total = 0.f;
	int32 Count = 0;

	for (int32 i = MotionHistory.Num() - 1; i >= 0 && Count < SampleCount; --i, ++Count)
	{
		Total += MotionHistory[i].GetAngularVelocity().Size();
	}

	return Count > 0 ? Total / Count : AngularVelocity.Size();
}

FVector FOHBoneData::GetEstimatedLinearAcceleration(int32 SampleCount) const
{
	if (SampleCount <= 1 || MotionHistory.Num() < 2)
		return LinearAcceleration;

	SampleCount = FMath::Clamp(SampleCount, 2, MotionHistory.Num());

	const FOHMotionSample& Newest = MotionHistory.Last();
	const FOHMotionSample& Oldest = MotionHistory[MotionHistory.Num() - SampleCount];

	const float DeltaTime = Newest.GetTimeStamp() - Oldest.GetTimeStamp();
	if (DeltaTime <= KINDA_SMALL_NUMBER)
		return LinearAcceleration;

	const FVector DeltaVel = Newest.GetLinearVelocity() - Oldest.GetLinearVelocity();
	return DeltaVel / DeltaTime;
}

FVector FOHBoneData::GetEstimatedAngularAcceleration(int32 SampleCount) const
{
	if (SampleCount <= 1 || MotionHistory.Num() < 2)
		return AngularAcceleration;

	SampleCount = FMath::Clamp(SampleCount, 2, MotionHistory.Num());

	const FOHMotionSample& Newest = MotionHistory.Last();
	const FOHMotionSample& Oldest = MotionHistory[MotionHistory.Num() - SampleCount];

	const float DeltaTime = Newest.GetTimeStamp() - Oldest.GetTimeStamp();
	if (DeltaTime <= KINDA_SMALL_NUMBER)
		return AngularAcceleration;

	const FVector DeltaVel = Newest.GetAngularVelocity() - Oldest.GetAngularVelocity();
	return DeltaVel / DeltaTime;
}


// Blended Force Computation

FVector FOHBoneData::GetBlendedLinearVelocity(float HistoryWeight, int32 SampleCount) const
{
	const FVector Current = GetBodyLinearVelocity();
	const FVector Historical = GetAverageLinearVelocity(SampleCount);

	return FMath::Lerp(Current, Historical, FMath::Clamp(HistoryWeight, 0.f, 1.f));
}

FVector FOHBoneData::GetBlendedAngularVelocity(float HistoryWeight, int32 SampleCount) const
{
	const FVector Current = GetBodyAngularVelocity();
	const FVector Historical = GetAverageAngularVelocity(SampleCount);

	return FMath::Lerp(Current, Historical, FMath::Clamp(HistoryWeight, 0.f, 1.f));
}

FVector FOHBoneData::GetBlendedLinearAcceleration(float HistoryWeight, int32 SampleCount) const
{
	const FVector Current = GetLinearAcceleration();
	const FVector Historical = GetEstimatedLinearAcceleration(SampleCount);

	return FMath::Lerp(Current, Historical, FMath::Clamp(HistoryWeight, 0.f, 1.f));
}

FVector FOHBoneData::GetBlendedAngularAcceleration(float HistoryWeight, int32 SampleCount) const
{
	const FVector Current = GetAngularAcceleration();
	const FVector Historical = GetEstimatedAngularAcceleration(SampleCount);

	return FMath::Lerp(Current, Historical, FMath::Clamp(HistoryWeight, 0.f, 1.f));
}

FVector FOHBoneData::GetTimeWeightedLinearVelocity() const
{
	const TArray<FOHMotionSample>& History = GetMotionHistory();
	if (History.Num() < 2)
		return FVector::ZeroVector;

	FVector WeightedVelocity = FVector::ZeroVector;
	float TotalTime = 0.f;

	for (int32 i = 1; i < History.Num(); ++i)
	{
		const FOHMotionSample& Prev = History[i - 1];
		const FOHMotionSample& Curr = History[i];

		const float SegmentTime = Curr.GetTimeStamp() - Prev.GetTimeStamp();
		if (SegmentTime <= SMALL_NUMBER)
			continue;

		WeightedVelocity += Curr.GetLinearVelocity() * SegmentTime;
		TotalTime += SegmentTime;
	}

	return (TotalTime > 0.f) ? WeightedVelocity / TotalTime : FVector::ZeroVector;
}

FVector FOHBoneData::GetTimeWeightedAngularVelocity() const
{
	const TArray<FOHMotionSample>& History = GetMotionHistory();
	if (History.Num() < 2)
		return FVector::ZeroVector;

	FVector WeightedAngular = FVector::ZeroVector;
	float TotalTime = 0.f;

	for (int32 i = 1; i < History.Num(); ++i)
	{
		const FOHMotionSample& Prev = History[i - 1];
		const FOHMotionSample& Curr = History[i];

		const float SegmentTime = Curr.GetTimeStamp() - Prev.GetTimeStamp();
		if (SegmentTime <= SMALL_NUMBER)
			continue;

		WeightedAngular += Curr.GetAngularVelocity() * SegmentTime;
		TotalTime += SegmentTime;
	}
	return (TotalTime > 0.f) ? WeightedAngular / TotalTime : FVector::ZeroVector;
}


FVector FOHBoneData::GetTimeWeightedAcceleration() const
{
	const TArray<FOHMotionSample>& History = GetMotionHistory();
	if (History.Num() < 2)
		return FVector::ZeroVector;

	FVector WeightedAccel = FVector::ZeroVector;
	float TotalTime = 0.f;

	for (int32 i = 1; i < History.Num(); ++i)
	{
		const FOHMotionSample& Prev = History[i - 1];
		const FOHMotionSample& Curr = History[i];

		const float SegmentTime = Curr.GetTimeStamp() - Prev.GetTimeStamp();
		if (SegmentTime <= SMALL_NUMBER)
			continue;

		WeightedAccel += Curr.GetLinearAcceleration() * SegmentTime;
		TotalTime += SegmentTime;
	}
	return (TotalTime > 0.f) ? WeightedAccel / TotalTime : FVector::ZeroVector;
}


FVector FOHBoneData::GetTimeWeightedAngularAcceleration() const
{
	const TArray<FOHMotionSample>& History = GetMotionHistory();
	if (History.Num() < 2)
		return FVector::ZeroVector;

	FVector WeightedAccel = FVector::ZeroVector;
	float TotalTime = 0.f;

	for (int32 i = 1; i < History.Num(); ++i)
	{
		const FOHMotionSample& Prev = History[i - 1];
		const FOHMotionSample& Curr = History[i];

		const float SegmentTime = Curr.GetTimeStamp() - Prev.GetTimeStamp();
		if (SegmentTime <= SMALL_NUMBER)
			continue;

		WeightedAccel += Curr.GetAngularAcceleration() * SegmentTime;
		TotalTime += SegmentTime;
	}
	return (TotalTime > 0.f) ? WeightedAccel / TotalTime : FVector::ZeroVector;
}

// Derived Force Scores

float FOHBoneData::GetVelocityDeviationMagnitude() const
{
	const FVector AvgVel = GetAverageLinearVelocity();
	return (LinearVelocity - AvgVel).Size();
}

float FOHBoneData::GetVelocitySpikeScore(float NormalizationFactor) const
{
	const float Deviation = GetVelocityDeviationMagnitude();
	return FMath::Clamp(Deviation / NormalizationFactor, 0.f, 2.f);
}

float FOHBoneData::GetPoseDriftScore() const
{
	if (MotionHistory.Num() < 2)
		return 0.f;

	const FVector AvgPosition = GetAveragePosition();
	const FQuat AvgRotation = GetAverageRotation();

	const float PosDrift = FVector::Dist(WorldPosition, AvgPosition);
	const float RotDrift = FQuat::ErrorAutoNormalize(CurrentRotation, AvgRotation); // Angular difference in radians

	// Weighted blend — tweak sensitivity as needed
	return PosDrift * 0.1f + RotDrift * 50.f;
}

float FOHBoneData::GetStabilityScore() const
{
	const float Spike = GetVelocitySpikeScore();
	const float Drift = GetPoseDriftScore();
	return Spike + (Drift * 0.05f);
}

float FOHBoneData::GetAngularJitterScore(float NormalizationFactor) const
{
	const FVector Smoothed = GetSmoothedAngularVelocity();
	const float Deviation = (AngularVelocity - Smoothed).Size();
	return FMath::Clamp(Deviation / NormalizationFactor, 0.f, 2.f);
}

float FOHBoneData::GetLinearDampingRatio() const
{
	const float SpeedNow = LinearVelocity.Size();
	const float SpeedPrev = (PreviousPosition - WorldPosition).Size() / LastDeltaTime;
	if (SpeedPrev <= KINDA_SMALL_NUMBER) return 0.f;
	return 1.f - (SpeedNow / SpeedPrev);
}

float FOHBoneData::GetRotationalDriftScore() const
{
	const FQuat Delta = CurrentRotation * PreviousRotation.Inverse();
	return FMath::RadiansToDegrees(Delta.GetAngle());
}

float FOHBoneData::GetTrajectoryShiftScore() const
{
	if (LastDeltaTime <= KINDA_SMALL_NUMBER || LinearVelocity.IsNearlyZero() || MotionHistory.Num() < 2)
		return 0.f;

	const FOHMotionSample& Latest = GetLatestMotionSample();

	// Compare with previous sample's velocity direction
	const FOHMotionSample* Prev = nullptr;
	for (int32 i = MotionHistory.Num() - 2; i >= 0; --i)
	{
		if (MotionHistory[i].IsValidSample())
		{
			Prev = &MotionHistory[i];
			break;
		}
	}

	if (!Prev || Prev->GetLinearVelocity().IsNearlyZero())
		return 0.f;

	const FVector DirA = Latest.GetLinearVelocity().GetSafeNormal();
	const FVector DirB = Prev->GetLinearVelocity().GetSafeNormal();

	const float Dot = FVector::DotProduct(DirA, DirB);
	const float SafeDot = FMath::Clamp(Dot, -1.f, 1.f);
	const float Angle = FMath::RadiansToDegrees(FMath::Acos(SafeDot));

	return FMath::Clamp(Angle, 0.f, 180.f);
}

float FOHBoneData::GetImpulseSignatureScore() const
{
	static constexpr float Normalization = 2000.f;

	const FVector PrevAccel = (LinearVelocity - GetSmoothedLinearVelocity()) / GetLastDeltaTime();
	const FVector Jerk = LinearAcceleration - PrevAccel;

	return FMath::Clamp(Jerk.Size() / Normalization, 0.f, 3.f);
}

float FOHBoneData::GetCompositeInstabilityScore() const
{
	const float LinSpike = GetVelocitySpikeScore();
	const float AngJitter = GetAngularJitterScore();
	const float Drift = GetPoseDriftScore();
	const float Trajectory = GetTrajectoryShiftScore();

	return LinSpike * 0.5f + AngJitter * 0.5f + Drift * 0.05f + Trajectory * 0.02f;
}

float FOHBoneData::GetCurvatureScore() const
{
	const int32 Count = MotionHistory.Num();
	if (Count < 3)
		return 0.f;

	const FVector A = MotionHistory[Count - 3].GetLocation();
	const FVector B = MotionHistory[Count - 2].GetLocation();
	const FVector C = MotionHistory[Count - 1].GetLocation();

	const FVector Dir1 = (B - A).GetSafeNormal();
	const FVector Dir2 = (C - B).GetSafeNormal();

	if (Dir1.IsNearlyZero() || Dir2.IsNearlyZero())
		return 0.f;

	const float Dot = FVector::DotProduct(Dir1, Dir2);
	return 1.f - FMath::Clamp(Dot, 0.f, 1.f); // 0 = straight, 1 = curved
}

float FOHBoneData::GetDirectionalStabilityScore() const
{
	if (MotionHistory.Num() < 4)
		return 1.f;

	TArray<FVector> Dirs;
	for (int32 i = 1; i < MotionHistory.Num(); ++i)
	{
		const FVector Prev = MotionHistory[i - 1].GetLocation();
		const FVector Curr = MotionHistory[i].GetLocation();
		const FVector Dir = (Curr - Prev).GetSafeNormal();

		if (!Dir.IsNearlyZero())
			Dirs.Add(Dir);
	}

	if (Dirs.Num() < 2)
		return 1.f;

	float TotalDot = 0.f;
	int32 DotCount = 0;

	for (int32 i = 1; i < Dirs.Num(); ++i)
	{
		TotalDot += FVector::DotProduct(Dirs[i - 1], Dirs[i]);
		DotCount++;
	}

	return DotCount > 0 ? FMath::Clamp(TotalDot / DotCount, 0.f, 1.f) : 1.f;
}

float FOHBoneData::ComputeImpulseVulnerabilityScore(
    const FVector& ImpactNormal, 
    const FVector& RootLocation,
    const FOHPhysicsGraphNode* PhysicsGraph) const  // Add graph parameter
{
    const FVector BoneVelocity = GetBodyLinearVelocity();
    const FVector BoneAngular = GetBodyAngularVelocity();
    const FVector BonePos = GetCurrentPosition();

    const float VelocityAlignment = FVector::DotProduct(BoneVelocity.GetSafeNormal(), -ImpactNormal);
    const float AngularSpeed = BoneAngular.Size();
    const float DistFromRoot = FVector::Distance(RootLocation, BonePos);

    // OLD: Direct constraint access
    // float ConstraintStress = GetConstraintStressScore();
    
    // NEW: Graph-mediated constraint access
    float ConstraintStress = 0.f;
    if (PhysicsGraph)
    {
        ConstraintStress = PhysicsGraph->GetConstraintStrain(GetBoneName());
    }

    const float Score =
        (VelocityAlignment * 0.3f) +
        (FMath::Clamp(AngularSpeed / 300.f, 0.f, 1.f) * 0.2f) +
        (FMath::Clamp(DistFromRoot / 100.f, 0.f, 1.f) * 0.2f) +
        (FMath::Clamp(ConstraintStress, 0.f, 1.f) * 0.3f);  // Constraint stress now major factor

    return FMath::Clamp(Score, 0.f, 2.0f);
}

// ===== UPDATE BONE DATA METHOD SIGNATURES =====

// Update method signatures that previously accessed constraints directly:
FVector FOHBoneData::ComputeImpulseVector(
    const FHitResult& Hit,
    const FVector& RootLocation,
    float BaseStrength,
    EImpulseDirectionMode ModeOverride,
    bool bAutoInferMode,
    const FOHPhysicsGraphNode* PhysicsGraph) const  // Add graph parameter
{
    if (!IsValid() || !Hit.bBlockingHit)
        return FVector::ZeroVector;

    const FVector Direction = ComputeImpulseDirection(Hit, 
        bAutoInferMode ? InferBestImpulseDirectionMode(Hit) : ModeOverride);
    
    const float Multiplier = ComputeImpulseVulnerabilityScore(Hit.ImpactNormal, RootLocation, PhysicsGraph);
    
    return Direction * BaseStrength * Multiplier;
}

FVector FOHBoneData::ComputeImpulseDirection(const FHitResult& Hit, EImpulseDirectionMode Mode) const
{
	const FVector Default = -Hit.ImpactNormal.GetSafeNormal();

	switch (Mode)
	{
	case EImpulseDirectionMode::FromHitNormal:
		return Default;

	case EImpulseDirectionMode::FromBoneToImpactPoint:
		return (Hit.ImpactPoint - GetCurrentPosition()).GetSafeNormal();

	case EImpulseDirectionMode::FromBoneVelocity:
		{
			const FVector Vel = GetBodyLinearVelocity();
			return Vel.SizeSquared() > KINDA_SMALL_NUMBER ? Vel.GetSafeNormal() : Default;
		}

	default:
		return Default;
	}
}




EImpulseDirectionMode FOHBoneData::InferBestImpulseDirectionMode(const FHitResult& Hit) const
{
	const FVector BoneToImpact = (Hit.ImpactPoint - GetCurrentPosition());
	const FVector Velocity = GetBodyLinearVelocity();

	// If we’re moving fast enough, velocity direction is preferred
	if (Velocity.SizeSquared() > FMath::Square(25.f))
	{
		const float Alignment = FVector::DotProduct(Velocity.GetSafeNormal(), -Hit.ImpactNormal);
		if (Alignment > 0.7f)
		{
			return EImpulseDirectionMode::FromBoneVelocity;
		}
	}

	// If impact came directly at the bone center, use that
	if (BoneToImpact.Size() < 2.0f)
	{
		return EImpulseDirectionMode::FromHitNormal;
	}

	// Otherwise, fall back to bone-to-hit logic
	return EImpulseDirectionMode::FromBoneToImpactPoint;
}




float FOHBoneData::GetOscillationFrequencyHz() const
{
	if (MotionHistory.Num() < 3)
		return 0.f;

	// Naive estimate using sample spacing
	const float Period = 2.f * LastDeltaTime;
	return Period > 0.f ? 1.f / Period : 0.f;
}

float FOHBoneData::GetMaxDisplacement() const
{
	float MaxDist = 0.f;
	for (const FOHMotionSample& Sample : MotionHistory)
	{
		if (!Sample.IsValidSample()) continue;
		const float Dist = FVector::Dist(WorldPosition, Sample.GetLocation());
		MaxDist = FMath::Max(MaxDist, Dist);
	}
	return MaxDist;
}

float FOHBoneData::GetUndershootRatio(const FVector& TargetPosition) const
{
	const float Distance = FVector::Dist(WorldPosition, TargetPosition);
	const float VelocityTowardTarget = FVector::DotProduct(
		LinearVelocity, (TargetPosition - WorldPosition).GetSafeNormal());

	return VelocityTowardTarget < 5.f ? Distance : 0.f;
}

float FOHBoneData::GetJerkMagnitude() const
{
	const FVector PrevAccel = (LinearVelocity - GetSmoothedLinearVelocity()) / LastDeltaTime;
	return (LinearAcceleration - PrevAccel).Size();
}

FVector FOHBoneData::GetPositionBeforePrevious() const
{
	return MotionHistory.Num() >= 2 ? MotionHistory[MotionHistory.Num() - 2].GetLocation() : PreviousPosition;
}

float FOHBoneData::GetImpulseScaleFactor()
{
	if (!GetOwnerComponent()) return 1.f;
	USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
	if (!SkelMesh->GetBodyInstance(BoneName))
		return 1.f;

	const FBodyInstance* Body = SkelMesh->GetBodyInstance(BoneName);
	const float Mass = Body ? Body->GetBodyMass() : 1.f;
	const float Damping = Body ? FMath::Max(Body->AngularDamping, 1.f) : 1.f;

	// Lower mass or damping → higher impulse scale
	return 1.f / FMath::Max(Mass * Damping, 1.f);
}
void FOHBoneData::InitializeFromBodyInstance(FBodyInstance* InBodyInstance)
{
	if (!InBodyInstance) return;

	SetBodyInstance(InBodyInstance);

	SetBoneName(InBodyInstance->BodySetup.IsValid()
		? InBodyInstance->BodySetup->BoneName
		: NAME_None);

	SetCachedBodyMass(InBodyInstance->GetBodyMass());
	SetIsSimulating(InBodyInstance->IsInstanceSimulatingPhysics());
}

void FOHBoneData::InitializeFromBodyInstance(const FBodyInstance* BI, const FReferenceSkeleton& RefSkeleton)
{
	if (!BI || !BI->IsValidBodyInstance()) return;

	SetBoneName( BI->BodySetup.IsValid()? BI->BodySetup->BoneName : NAME_None);
	BoneName = BI->BodySetup.IsValid() ? BI->BodySetup->BoneName : NAME_None;
	SetBodyInstance(const_cast<FBodyInstance*>(BI));
	SetBodySetup(BI->GetBodySetup());
	SetBoneIndex(RefSkeleton.FindBoneIndex(BoneName));
	SetCachedBodyMass(BI->GetBodyMass()) ;
}

void FOHBoneData::InitializeFromBodySetup(const UBodySetup* Setup, const FReferenceSkeleton& RefSkeleton)
{
	if (!Setup) return;

	SetBoneName(Setup->BoneName);
	SetBodySetup(const_cast<UBodySetup*>(Setup));
	BoneIndex = RefSkeleton.FindBoneIndex(BoneName);

	// Approximate mass and position (tool-time only)
	SetCachedBodyMass(Setup->CalculateMass());

	// NOTE: No BodyInstance at this point (editor-only use case)
	BodyInstance = nullptr;
}

#pragma endregion

#pragma region ConstraintInstanceData

bool FOHConstraintInstanceData::InitializeFromTemplate(const UPhysicsConstraintTemplate* Template)
{
	if (!Template)
	{
		return false;
	}

	// Grab the built‐in default constraint instance
	const FConstraintInstance& CI = Template->DefaultInstance;

	// Core identity
	SetConstraintName( CI.JointName );
	SetParentBone(    CI.ConstraintBone1 );
	SetChildBone(     CI.ConstraintBone2 );

	// Angular limits (degrees)
	SetSwing1LimitDegrees( CI.GetAngularSwing1Limit() );
	SetSwing2LimitDegrees( CI.GetAngularSwing2Limit() );
	SetTwistLimitDegrees(  CI.GetAngularTwistLimit() );

	// Angular‐cone soft limit properties
	// (uses the ProfileInstance's cone‐limit soft settings)
	SetConeLimitStiffness( CI.ProfileInstance.ConeLimit.Stiffness );
	SetConeLimitDamping(   CI.ProfileInstance.ConeLimit.Damping );

	// Linear drives (we only use XDrive here; extend if needed)
	SetLinearDriveStiffness( CI.ProfileInstance.LinearDrive.XDrive.Stiffness );
	SetLinearDriveDamping(   CI.ProfileInstance.LinearDrive.XDrive.Damping );

	// Angular drives: swing = SlerpDrive, twist = TwistDrive
	SetAngularSwingStiffness( CI.ProfileInstance.AngularDrive.SlerpDrive.Stiffness );
	SetAngularSwingDamping(   CI.ProfileInstance.AngularDrive.SlerpDrive.Damping );

	SetAngularTwistStiffness( CI.ProfileInstance.AngularDrive.TwistDrive.Stiffness );
	SetAngularTwistDamping(   CI.ProfileInstance.AngularDrive.TwistDrive.Damping );

	// Drive‐enable flags
	SetPositionDriveEnabled( CI.ProfileInstance.LinearDrive.IsPositionDriveEnabled() );
	SetVelocityDriveEnabled( CI.ProfileInstance.LinearDrive.IsVelocityDriveEnabled() );

	// Initialize cached kinematic targets to zero—will be set when blending begins
	SetCachedPositionTarget(       FVector::ZeroVector );
	SetCachedVelocityTarget(       FVector::ZeroVector );
	SetCachedOrientationTarget(    FQuat::Identity );
	SetCachedAngularVelocityTarget(FVector::ZeroVector);

	return true;
}


void FOHConstraintInstanceData::UpdateFromLiveConstraint()
{
	if (!GetConstraintInstance())
	{
		UE_LOG(LogTemp, Warning, TEXT("[FOHConstraintInstanceData] Missing GetConstraintInstance() for %s"), *ConstraintName.ToString());
		return;
	}

	// === Cone Limits ===
	SetSwing1LimitDegrees(GetConstraintInstance()->ProfileInstance.ConeLimit.Swing1LimitDegrees);
	SetSwing2LimitDegrees(GetConstraintInstance()->ProfileInstance.ConeLimit.Swing2LimitDegrees);
	SetTwistLimitDegrees(GetConstraintInstance()->ProfileInstance.TwistLimit.TwistLimitDegrees);

	// === Linear Drive ===
	SetLinearDriveStiffness(GetConstraintInstance()->ProfileInstance.LinearDrive.XDrive.Stiffness);
	SetLinearDriveDamping(GetConstraintInstance()->ProfileInstance.LinearDrive.XDrive.Damping);

	// === Angular Drive: Swing and Twist ===
	SetAngularSwingStiffness(GetConstraintInstance()->ProfileInstance.AngularDrive.SwingDrive.Stiffness);
	SetAngularSwingDamping(GetConstraintInstance()->ProfileInstance.AngularDrive.SwingDrive.Damping);
	SetAngularTwistStiffness(GetConstraintInstance()->ProfileInstance.AngularDrive.TwistDrive.Stiffness);
	SetAngularTwistDamping(GetConstraintInstance()->ProfileInstance.AngularDrive.TwistDrive.Damping);
}

#pragma endregion

