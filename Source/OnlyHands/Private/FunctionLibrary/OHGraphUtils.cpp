#include "FunctionLibrary/OHGraphUtils.h"
#include "Data/Struct/OHPhysicsStructs.h"

bool UOHGraphUtils::IsBoneValid(const FOHBoneData& Bone, EValidationStrictness Strictness)
{
	// Example validation logic - adjust as per your actual criteria

	switch (Strictness)
	{
	case EValidationStrictness::CriticalOnly:
		return Bone.GetBoneName() != NAME_None;

	case EValidationStrictness::Standard:
		return Bone.GetBoneName() != NAME_None && Bone.GetHasSimulatedBody();

	case EValidationStrictness::Strict:
		return Bone.GetBoneName() != NAME_None
			&& Bone.GetHasSimulatedBody()
			&& Bone.GetCachedBodyMass() > 0.f
			&& Bone.GetCachedBoneLength() > KINDA_SMALL_NUMBER;

	default:
		return false;
	}
}


bool UOHGraphUtils::AddBoneIfNoConflict(
	TMap<FName, FOHBoneData>& BoneMap,
	const FOHBoneData& BoneData)
{
	const FName Name = BoneData.GetBoneName();
	if (Name.IsNone())
	{
		UE_LOG(LogTemp, Warning, TEXT("[OHGraphUtils] Skipped bone with no name."));
		return false;
	}

	return OHSafeMapUtils::TryInsertIfNoConflict<FName, FOHBoneData>(
		BoneMap,
		Name,
		BoneData,
		[](const FOHBoneData& Existing, const FOHBoneData& Incoming)
		{
			// Conflict is defined as mismatched simulation status
			const bool bMismatchSim = Existing.GetHasSimulatedBody() != Incoming.GetHasSimulatedBody();

			// Or: meaningful mass/length discrepancy
			const bool bMassDiff = !FMath::IsNearlyEqual(Existing.GetCachedBodyMass(), Incoming.GetCachedBodyMass(), 0.01f);
			const bool bLengthDiff = !FMath::IsNearlyEqual(Existing.GetCachedBoneLength(), Incoming.GetCachedBoneLength(), 0.01f);

			return bMismatchSim || bMassDiff || bLengthDiff;
		});
}


int32 UOHGraphUtils::InsertAllBonesIfValid(
	TMap<FName, FOHBoneData>& TargetMap,
	const TMap<FName, FOHBoneData>& SourceMap,
	EValidationStrictness Strictness)
{
	return OHSafeMapUtils::InsertAllValidated<FName, FOHBoneData>(
		TargetMap,
		SourceMap,
		[Strictness](const FOHBoneData& Bone)
		{
			return UOHGraphUtils::IsBoneValid(Bone, Strictness);
		});
}


int32 UOHGraphUtils::InsertAllConstraintsIfValid(
	TArray<FOHConstraintInstanceData>& TargetList,
	const TArray<FOHConstraintInstanceData>& SourceList,
	const TMap<FName, FOHBoneData>& BoneMap)
{
	int32 Count = 0;

	for (const FOHConstraintInstanceData& Constraint : SourceList)
	{
		const FName Parent = Constraint.GetParentBone();
		const FName Child = Constraint.GetChildBone();

		const bool bParentValid = BoneMap.Contains(Parent) && BoneMap[Parent].GetHasSimulatedBody();
		const bool bChildValid = BoneMap.Contains(Child) && BoneMap[Child].GetHasSimulatedBody();

		if (bParentValid && bChildValid)
		{
			TargetList.Add(Constraint);
			++Count;
		}
	}

	return Count;
}

TArray<FName> UOHGraphUtils::GetChildBones(const TMap<FName, FOHBoneData>& BoneMap, FName ParentBone)
{
	if (const FOHBoneData* Data = BoneMap.Find(ParentBone))
	{
		return Data->FindChildBones();
	}
	return {};
}

TArray<FOHConstraintInstanceData> UOHGraphUtils::GetConstraintsForBone(
	FName BoneName,
	const TArray<FOHConstraintInstanceData>& ConstraintList)
{
	TArray<FOHConstraintInstanceData> Result;
	for (const auto& Constraint : ConstraintList)
	{
		if (Constraint.GetParentBone() == BoneName || Constraint.GetChildBone() == BoneName)
		{
			Result.Add(Constraint);
		}
	}
	return Result;
}

TArray<FName> UOHGraphUtils::GetSimulatedBones(const TMap<FName, FOHBoneData>& BoneMap)
{
	TArray<FName> Result;
	for (const auto& Pair : BoneMap)
	{
		if (Pair.Value.GetHasSimulatedBody())
		{
			Result.Add(Pair.Key);
		}
	}
	return Result;
}

void UOHGraphUtils::RemoveInvalidConstraints(
	TArray<FOHConstraintInstanceData>& ConstraintList,
	const TMap<FName, FOHBoneData>& BoneMap)
{
	ConstraintList.RemoveAll([&](const FOHConstraintInstanceData& C)
	{
		const bool bValidParent = BoneMap.Contains(C.GetParentBone()) && BoneMap[C.GetParentBone()].GetHasSimulatedBody();
		const bool bValidChild = BoneMap.Contains(C.GetChildBone()) && BoneMap[C.GetChildBone()].GetHasSimulatedBody();
		return !(bValidParent && bValidChild);
	});
}

void UOHGraphUtils::NormalizeBoneMasses(TMap<FName, FOHBoneData>& BoneMap)
{
	OHSafeMapUtils::NormalizeMapValues<FName, FOHBoneData>(
		BoneMap,
		[](const FOHBoneData& Bone) { return Bone.GetCachedBodyMass(); },
		[](FOHBoneData& Bone, float Normalized) { Bone.SetCachedBodyMass(Normalized); }
	);
}

float UOHGraphUtils::ComputeBoneMapEntropy(const TMap<FName, FOHBoneData>& BoneMap)
{
	return OHSafeMapUtils::EntropyOfMapKeys(BoneMap);
}

void UOHGraphUtils::RemoveBoneMassOutliers(TMap<FName, FOHBoneData>& BoneMap, float ThresholdZ)
{
	OHSafeMapUtils::RemoveMapOutliers<FName, FOHBoneData>(
		BoneMap,
		[](const FOHBoneData& Bone) { return Bone.GetCachedBodyMass(); },
		ThresholdZ
	);
}

void UOHGraphUtils::LogGraphStats(
	const TMap<FName, FOHBoneData>& BoneMap,
	const TArray<FOHConstraintInstanceData>& Constraints,
	const FString& ContextTag)
{
	const int32 BoneCount = BoneMap.Num();
	const int32 ConstraintCount = Constraints.Num();

	int32 SimulatedCount = 0;
	for (const auto& Pair : BoneMap)
	{
		if (Pair.Value.GetIsSimulating())
		{
			++SimulatedCount;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("[%s] BoneCount: %d, Simulated: %d, Constraints: %d"),
		*ContextTag, BoneCount, SimulatedCount, ConstraintCount);
}

bool UOHGraphUtils::ValidateAllChainsStable(
	const TMap<FName, FOHBoneData>& BoneMap,
	const TArray<FOHConstraintInstanceData>& Constraints,
	int32 MaxDepth)
{
	auto ChildMap = OHSafeMapUtils::BuildParentToChildrenMap<FOHConstraintInstanceData>(
		Constraints,
		[](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
		[](const FOHConstraintInstanceData& C) { return C.GetChildBone(); }
	);

	TFunction<bool(const FName&, int32)> Recurse;
	Recurse = [&](const FName& Bone, int32 Depth) -> bool
	{
		if (Depth > MaxDepth) return false;
		const FOHBoneData* BoneData = BoneMap.Find(Bone);
		if (!BoneData || !BoneData->GetIsSimulating()) return false;

		const TArray<FName>* Children = ChildMap.Find(Bone);
		if (!Children) return true;

		for (const FName& Child : *Children)
		{
			if (!Recurse(Child, Depth + 1)) return false;
		}
		return true;
	};

	for (const auto& Pair : BoneMap)
	{
		if (Recurse(Pair.Key, 0) == false)
			return false;
	}
	return true;
}

#pragma region Graph Building



// ========================================================
// BuildPhysicsGraphFromComponent
// ========================================================
bool UOHGraphUtils::BuildPhysicsGraphFromComponent(
    USkeletalMeshComponent* SkeletalMeshComp,
    FOHPhysicsGraphNode&    OutGraph,
    const FString&          ContextTag,
    bool                    bRunAudit
)
{
    if (!SkeletalMeshComp)
    {
        UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMeshComponent is null"), *ContextTag);
        return false;
    }

    const UPhysicsAsset* PhysAsset = SkeletalMeshComp->GetPhysicsAsset();
    if (!PhysAsset)
    {
        UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMeshComponent has no PhysicsAsset"), *ContextTag);
        return false;
    }

    const USkeletalMesh* SkeletalMesh = SkeletalMeshComp->GetSkeletalMeshAsset();
    if (!SkeletalMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMesh asset is null"), *ContextTag);
        return false;
    }

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();

    // 1) Reset graph and cache pointers
    OutGraph.Reset();
    OutGraph.SetMeshComponent(SkeletalMeshComp);
    OutGraph.SetSkeleton(const_cast<USkeleton*>(SkeletalMesh->GetSkeleton()));
    OutGraph.SetPhysicsAsset(const_cast<UPhysicsAsset*>(PhysAsset));

    // 2) Build BoneMap
    auto& BoneMap = OutGraph.GetBoneMap();
    for (const FBodyInstance* BI : SkeletalMeshComp->Bodies)
    {
        if (!BI || !BI->IsValidBodyInstance()) continue;

        const FName BoneName = BI->BodySetup.IsValid()
            ? BI->BodySetup->BoneName
            : NAME_None;
        if (BoneName == NAME_None) continue;

        FOHBoneData Bone;
        Bone.InitializeFromBodyInstance(BI, RefSkeleton);
        BoneMap.Add(BoneName, Bone);
    }

    // 3) Build ConstraintLinks
    auto& Constraints = OutGraph.GetConstraintLinks();
    for (const UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
    {
        if (!Template) continue;

        FOHConstraintInstanceData Link;
        if (!Link.InitializeFromTemplate(Template))
            continue;

        // find the live FConstraintInstance on the component
        FConstraintInstance** FoundPtr = SkeletalMeshComp->Constraints.FindByPredicate(
            [Link](const FConstraintInstance* CI)
            {
                return CI && CI->JointName == Link.GetConstraintName();
            });
        FConstraintInstance* FoundCI = FoundPtr ? *FoundPtr : nullptr;

        Link.SetConstraintInstance(FoundCI);
        Link.SetConstraintTemplate(const_cast<UPhysicsConstraintTemplate*>(Template));
        Link.SetOwnerComponent(SkeletalMeshComp);

        Constraints.Add(Link);
    }

    // 4) Finalize: rebuild all adjacency multimaps
    OutGraph.RebuildWrappers();

    // 5) Optional audit + stats
    if (bRunAudit)
    {
        UOHGraphUtils::PruneInvalidConstraints(OutGraph);
        UOHGraphUtils::LogGraphStats(BoneMap, Constraints, ContextTag);
    }

    UE_LOG(LogTemp, Log, TEXT("[%s] Built physics graph: %d bones, %d constraints"),
        *ContextTag, BoneMap.Num(), Constraints.Num());

    return true;
}



#pragma endregion

void UOHGraphUtils::ClearPhysicsGraph(FOHPhysicsGraphNode& Graph)
{
	UE_LOG(LogTemp, Log, TEXT("[ClearPhysicsGraph] Resetting physics graph."));
	Graph.Reset();
	UE_LOG(LogTemp, Log, TEXT("[ClearPhysicsGraph] Graph fully reset."));
}


bool UOHGraphUtils::RefreshBonesOnlyFromComponent(
	USkeletalMeshComponent* SkeletalMeshComp,
	FOHPhysicsGraphNode& Graph,
	const FString& ContextTag,
	bool bRebindOwnerComponent)
{
	if (!SkeletalMeshComp)
	{
		UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMeshComponent is null"), *ContextTag);
		return false;
	}

	const USkeletalMesh* SkeletalMesh = SkeletalMeshComp->GetSkeletalMeshAsset();
	if (!SkeletalMesh)
	{
		UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMesh is null"), *ContextTag);
		return false;
	}

	if (bRebindOwnerComponent)
	{
		Graph.SetMeshComponent(SkeletalMeshComp);
		Graph.SetSkeleton(const_cast<USkeleton*>(SkeletalMesh->GetSkeleton()));
	}

	const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();
	TMap<FName, FOHBoneData>& BoneMap = Graph.GetBoneMap();

	int32 Refreshed = 0;
	int32 Missing = 0;

	for (auto& [BoneName, Bone] : BoneMap)
	{
		FBodyInstance* BI = SkeletalMeshComp->GetBodyInstance(BoneName);
		if (!BI || !BI->IsValidBodyInstance())
		{
			UE_LOG(LogTemp, Warning, TEXT("[%s] Missing or invalid body: %s"), *ContextTag, *BoneName.ToString());
			Bone.SetBodyInstance(nullptr);
			Missing++;
			continue;
		}

		// Refresh direct reference
		Bone.SetBodyInstance(BI);

		// Cached Mass
		const float Mass = BI->GetBodyMass();
		Bone.SetCachedBodyMass(Mass);

		// Cached World Transform
		const FTransform WorldTransform = BI->GetUnrealWorldTransform();

		// Cached Length (distance to parent ref-pose position)
		float Length = 0.f;
		if (int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneName); BoneIndex != INDEX_NONE && BoneIndex != 0)
		{
			const FTransform ParentRef = RefSkeleton.GetRefBonePose()[RefSkeleton.GetParentIndex(BoneIndex)];
			Length = (WorldTransform.GetLocation() - ParentRef.GetLocation()).Size();
		}
		Bone.SetCachedBoneLength(Length);

		Refreshed++;
	}

	UE_LOG(LogTemp, Log, TEXT("[%s] Bone refresh complete: %d refreshed, %d missing"), *ContextTag, Refreshed, Missing);
	return true;
}


bool UOHGraphUtils::RefreshConstraintsOnlyFromComponent(
    USkeletalMeshComponent*    SkeletalMeshComp,
    FOHPhysicsGraphNode&       Graph,
    const FString&             ContextTag,
    bool                       bRebuildFullConstraintList,
    bool                       bRebindOwnerComponent
)
{
    if (!SkeletalMeshComp)
    {
        UE_LOG(LogTemp, Error, TEXT("[%s] SkeletalMeshComponent is null"), *ContextTag);
        return false;
    }

    // Optionally update the cached component pointer
    if (bRebindOwnerComponent)
    {
        Graph.SetMeshComponent(SkeletalMeshComp);
    }

    // Reference to the graph's constraint array
    TArray<FOHConstraintInstanceData>& Constraints = Graph.GetConstraintLinks();
    int32 ReboundCount = 0;

    if (bRebuildFullConstraintList)
    {
        // Wipe and rebuild the entire list
        Constraints.Reset();

        const UPhysicsAsset* PhysAsset = SkeletalMeshComp->GetPhysicsAsset();
        if (!PhysAsset)
        {
            UE_LOG(LogTemp, Error, TEXT("[%s] No PhysicsAsset on SkeletalMeshComponent"), *ContextTag);
            return false;
        }

        for (const UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
        {
            if (!Template)
                continue;

            FOHConstraintInstanceData Entry;
            if (!Entry.InitializeFromTemplate(Template))
                continue;

            // Find the live constraint instance by joint name
            FConstraintInstance* FoundCI = SkeletalMeshComp->FindConstraintInstance(Entry.GetConstraintName());
            Entry.SetConstraintInstance(FoundCI);
            Entry.SetConstraintTemplate(const_cast<UPhysicsConstraintTemplate*>(Template));
            Entry.SetOwnerComponent(SkeletalMeshComp);

            Constraints.Add(Entry);
            if (FoundCI)
            {
                ++ReboundCount;
            }
        }
    }
    else
    {
        // Only rebind existing entries
        for (FOHConstraintInstanceData& Entry : Constraints)
        {
            FConstraintInstance* FoundCI = SkeletalMeshComp->FindConstraintInstance(Entry.GetConstraintName());
            Entry.SetConstraintInstance(FoundCI);
            Entry.SetOwnerComponent(SkeletalMeshComp);

            if (FoundCI)
            {
                ++ReboundCount;
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("[%s] Could not find constraint instance for '%s'"),
                    *ContextTag, *Entry.GetConstraintName().ToString());
            }
        }
    }

    // Rebuild our adjacency multimaps (parent→child & bone→constraint)
    Graph.RebuildWrappers();

    UE_LOG(LogTemp, Log, TEXT("[%s] Refreshed %d constraints (%s full rebuild)"),
        *ContextTag,
        ReboundCount,
        bRebuildFullConstraintList ? TEXT("with") : TEXT("without")
    );

    return true;
}
bool UOHGraphUtils::RefreshGraphFromComponent(
	USkeletalMeshComponent* SkeletalMeshComp,
	FOHPhysicsGraphNode& Graph,
	const FString& ContextTag,
	bool bRefreshBones,
	bool bRefreshConstraints,
	bool bRebindOwnerComponent,
	bool bFullConstraintRebuild)
{
	if (!SkeletalMeshComp)
	{
		UE_LOG(LogTemp, Error, TEXT("[%s] RefreshGraph failed: SkeletalMeshComponent is null"), *ContextTag);
		return false;
	}

	bool bSuccess = true;

	if (bRefreshBones)
	{
		const bool bBoneResult = RefreshBonesOnlyFromComponent(
			SkeletalMeshComp,
			Graph,
			ContextTag + TEXT("_Bones"),
			bRebindOwnerComponent
		);

		if (!bBoneResult)
		{
			UE_LOG(LogTemp, Warning, TEXT("[%s] Bone refresh failed"), *ContextTag);
			bSuccess = false;
		}
	}

	if (bRefreshConstraints)
	{
		const bool bConstraintResult = RefreshConstraintsOnlyFromComponent(
			SkeletalMeshComp,
			Graph,
			ContextTag + TEXT("_Constraints"),
			bFullConstraintRebuild,
			bRebindOwnerComponent
		);

		if (!bConstraintResult)
		{
			UE_LOG(LogTemp, Warning, TEXT("[%s] Constraint refresh failed"), *ContextTag);
			bSuccess = false;
		}
	}

	return bSuccess;
}

void UOHGraphUtils::ResetGraph(FOHPhysicsGraphNode& Graph)
{
	UE_LOG(LogTemp, Verbose, TEXT("ResetGraph: Clearing %d bones, %d constraints"),
		Graph.GetBoneMap().Num(),
		Graph.GetConstraintLinks().Num()
	);
	Graph.Reset();
}

bool UOHGraphUtils::SafeRebuildGraph(const USkeletalMeshComponent* SkelComp, FOHPhysicsGraphNode& Graph, bool bForceResetIfFailed)
{
	if (!SkelComp) return false;

	// Build directly into the existing graph (the function resets it internally)
	if (BuildPhysicsGraphFromComponent(const_cast<USkeletalMeshComponent*>(SkelComp), Graph, TEXT("SafeRebuild"), true))
	{
		return true;
	}

	if (bForceResetIfFailed)
	{
		Graph.Reset();
	}
	return false;
}

TSet<FName> UOHGraphUtils::GetSimulatedBoneNames(const FOHPhysicsGraphNode& Graph)
{
	TSet<FName> Result;
	for (const auto& Pair : Graph.GetBoneMap())
	{
		if (Pair.Value.GetIsSimulating())
		{
			Result.Add(Pair.Key);
		}
	}
	return Result;
}

TArray<FName> UOHGraphUtils::GetAllConstraintNames(const FOHPhysicsGraphNode& Graph)
{
	TArray<FName> Result;
	for (const auto& C : Graph.GetConstraintLinks())
	{
		Result.Add(C.GetConstraintName());
	}
	return Result;
}

TSet<FName> UOHGraphUtils::GetAllDescendants(const FOHPhysicsGraphNode& Graph, FName RootBone)
{
	return OHSafeMapUtils::TraverseWithNodeLimit<FName>(
		RootBone,
		[&](const FName& Node) -> TArray<FName>
		{
			return Graph.GetChildrenOfBone(Node);
		},
		[](const FName&) {},
		/*MaxNodesToVisit=*/ -1
	);
}


TMap<FName, TArray<FName>> UOHGraphUtils::BuildAdjacencyMap(const FOHPhysicsGraphNode& Graph)
{
	return OHSafeMapUtils::BuildBoneConstraintAdjacencyGraph<FOHConstraintInstanceData>(
		Graph.GetConstraintLinks(),
		[](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
		[](const FOHConstraintInstanceData& C) { return C.GetChildBone(); }
	);
}

TSet<FName> UOHGraphUtils::TraverseGraphBFS(const FOHPhysicsGraphNode& Graph, FName Start)
{
	const auto& BoneMap = Graph.GetBoneMap();

	TSet<FName> VisitedNodes;

	// Lambda to get child bones safely
	auto GetChildren = [&](const FName& Bone) -> TArray<FName>
	{
		const FOHBoneData* Data = BoneMap.Find(Bone);
		return Data ? Data->FindChildBones() : TArray<FName>{};
	};

	// Visitor lambda adds the visited bone to the set
	auto Visitor = [&](const FName& VisitedBone)
	{
		VisitedNodes.Add(VisitedBone);
	};

	// Call traversal with start node, get neighbors, visitor, and max node limit (512)
	OHSafeMapUtils::TraverseWithNodeLimit<FName>(
		Start,
		GetChildren,
		Visitor,
		512
	);

	return VisitedNodes;
}


TSet<FName> UOHGraphUtils::TraverseGraphBFS(
	const FOHPhysicsGraphNode& Graph,
	FName Start,
	int32 MaxNodesToVisit
)
{
	return OHSafeMapUtils::TraverseWithNodeLimit<FName>(
		Start,
		[&](const FName& Node) -> TArray<FName>
		{
			return Graph.GetChildrenOfBone(Node);
		},
		[](const FName&) {},
		MaxNodesToVisit
	);
}



void UOHGraphUtils::TraverseGraphDFS(const FOHPhysicsGraphNode& Graph, FName Start, TFunctionRef<void(const FName&)> Visitor)
{
	const auto& BoneMap = Graph.GetBoneMap();

	OHSafeMapUtils::DepthFirstVisit<FName>(
		Start,
		[&](const FName& Bone) -> TArray<FName>
		{
			const FOHBoneData* Data = BoneMap.Find(Bone);
			return Data ? Data->FindChildBones() : TArray<FName>{};
		},
		Visitor
	);
}

TArray<TSet<FName>> UOHGraphUtils::ExtractConnectedComponents(const FOHPhysicsGraphNode& Graph)
{
	TArray<TSet<FName>> Result;
	TSet<FName> Unvisited;
	Graph.GetBoneMap().GetKeys(Unvisited);  
	while (!Unvisited.IsEmpty())
	{
		const FName Start = *Unvisited.CreateConstIterator();
		TSet<FName> Component = TraverseGraphBFS(Graph, Start, 512);
		Result.Add(Component);
		Unvisited = Unvisited.Difference(Component);
	}
	return Result;
}

bool UOHGraphUtils::HasCycle(const FOHPhysicsGraphNode& Graph)
{
	// 1) Collect all “root” bones (those with no parent)
	TArray<FName> Roots;
	Roots.Reserve(Graph.GetBoneMap().Num());
	for (const auto& Pair : Graph.GetBoneMap())
	{
		const FName BoneName    = Pair.Key;
		const FOHBoneData& Data = Pair.Value;
		if (Data.GetParentBone() == NAME_None)
		{
			Roots.Add(BoneName);
		}
	}

	// 2) For each root, run cycle detection using Graph.GetChildrenOfBone()
	for (const FName& Root : Roots)
	{
		TArray<FName> CyclePath;
		bool bFoundCycle = OHSafeMapUtils::DetectCyclesInGraph<FName>(
			Root,
			[&](const FName& Node) -> TArray<FName>
			{
				// Pull direct children in O(1) from the multimap
				return Graph.GetChildrenOfBone(Node);
			},
			CyclePath
		);

		if (bFoundCycle)
		{
			return true;
		}
	}

	return false;
}



TMap<FName, FString> UOHGraphUtils::GetConstraintHealthReport(const FOHPhysicsGraphNode& Graph)
{
	const auto& Constraints = Graph.GetConstraintLinks();
	const auto& BoneMap = Graph.GetBoneMap();

	return OHSafeMapUtils::MapToReport<FOHConstraintInstanceData, FName>(
	Constraints,
	[](const FOHConstraintInstanceData& C) { return C.GetConstraintName(); },
	[&](const FOHConstraintInstanceData& C) -> FString
	{
		const FName A = C.GetParentBone();
		const FName B = C.GetChildBone();
		const FOHBoneData* DataA = BoneMap.Find(A);
		const FOHBoneData* DataB = BoneMap.Find(B);

		if (!DataA || !DataB)
			return TEXT("Error: Missing bone(s)");

		float MassA = DataA->GetCachedBodyMass();
		float MassB = DataB->GetCachedBodyMass();
		float Ratio = (MassB > 0.f) ? MassA / MassB : 0.f;
		float Distance = FVector::Dist(DataA->GetCurrentPosition(), DataB->GetCurrentPosition());

		FString Flags;
		if (Ratio > 10.f || Ratio < 0.1f)
			Flags += TEXT("[Unbalanced Mass] ");
		if (Distance > 100.f)
			Flags += TEXT("[High Separation] ");
		if (C.GetConeLimitStiffness() == 0.f && C.GetConeLimitDamping() == 0.f)
			Flags += TEXT("[Unconstrained] ");

		return FString::Printf(
			TEXT("%sParent: %s | Child: %s | Ratio: %.2f | Dist: %.1f | Stiff: %.2f | Damp: %.2f"),
			*Flags,
			*A.ToString(), *B.ToString(),
			Ratio, Distance, C.GetConeLimitStiffness(), C.GetConeLimitDamping()
		);
	});
}

void UOHGraphUtils::PruneInvalidConstraints(FOHPhysicsGraphNode& Graph)
{
	OHSafeMapUtils::RemoveIfAnyMissingOrInvalid<FName, FOHBoneData, FOHConstraintInstanceData>(
		Graph.GetConstraintLinks(),
		Graph.GetBoneMap(),
		[](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
		[](const FOHConstraintInstanceData& C) { return C.GetChildBone(); },
		[](const FOHBoneData& B) { return B.GetIsSimulating(); },
		[](const FOHConstraintInstanceData& C)
		{
			return C.GetConstraintInstance() != nullptr;
		}
	);
}

bool UOHGraphUtils::ValidateConstraintConnectivity(const FOHPhysicsGraphNode& Graph)
{
	const auto& BoneMap = Graph.GetBoneMap();

	for (const FOHConstraintInstanceData& C : Graph.GetConstraintLinks())
	{
		const FOHBoneData* A = BoneMap.Find(C.GetParentBone());
		const FOHBoneData* B = BoneMap.Find(C.GetChildBone());
		if (!A || !B || !A->GetHasSimulatedBody() || !B->GetHasSimulatedBody())
		{
			return false;
		}
	}
	return true;
}

bool UOHGraphUtils::HasMissingBodies(const FOHPhysicsGraphNode& Graph)
{
	for (const auto& Pair : Graph.GetBoneMap())
	{
		const FOHBoneData& Bone = Pair.Value;
		if (!Bone.GetBodyInstance() || !Bone.GetBodyInstance()->IsValidBodyInstance())
		{
			return true;
		}
	}
	return false;
}

void UOHGraphUtils::RebuildChildBoneLinks(FOHPhysicsGraphNode& Graph)
{
	auto& Map = Graph.GetBoneMap();

	for (auto& Pair : Map)
	{
		Pair.Value.GetMutableChildBones().Reset();
	}

	for (auto& Pair : Map)
	{
		const FName Parent = Pair.Value.GetParentBone();
		if (Map.Contains(Parent))
		{
			Map[Parent].AddChildBone(Pair.Key);
		}
	}
}


bool UOHGraphUtils::ValidateAndRepairBoneHierarchy(FOHPhysicsGraphNode& Graph)
{
	RebuildChildBoneLinks(Graph);
	return !HasCycle(Graph);
}

void UOHGraphUtils::AssignDefaultProfiles(FOHPhysicsGraphNode& Graph)
{
	for (auto& Pair : Graph.GetBoneMap())
	{
		FOHBoneData& Bone = Pair.Value;

		if (Bone.GetCachedBodyMass() <= 0.f)
		{
			Bone.SetCachedBodyMass(1.0f);
		}

		if (Bone.GetCachedBoneLength() <= 0.f)
		{
			Bone.SetCachedBoneLength(10.f);
		}
	}
}

void UOHGraphUtils::DrawPhysicsGraphOverlay(
	const FOHPhysicsGraphNode& Graph,
	const USkeletalMeshComponent* MeshComp,
	UWorld* World,
	float Duration)
{
	if (!MeshComp || !World) return;

	const FVector MeshScale = MeshComp->GetComponentScale();

	// Draw bone boxes
	for (const auto& Pair : Graph.GetBoneMap())
	{
		const FName BoneName = Pair.Key;
		const FOHBoneData& Bone = Pair.Value;

		const int32 BoneIndex = MeshComp->GetBoneIndex(BoneName);
		if (BoneIndex == INDEX_NONE) continue;

		const FTransform BoneTransform = MeshComp->GetBoneTransform(BoneIndex);
		const FVector Pos = BoneTransform.GetLocation();
		const float Mass = FMath::Max(1.f, Bone.GetCachedBodyMass());
		const float Length = FMath::Max(1.f, Bone.GetCachedBoneLength());

		const FVector BoxExtent = FVector(5.f, 5.f, Length * 0.5f) * MeshScale * FMath::LogX(2.0f, Mass + 2.0f);
		const FColor BoxColor = Bone.GetHasSimulatedBody() ? FColor::Green : FColor::Red;

		DrawDebugBox(World, Pos, BoxExtent, BoneTransform.GetRotation(), BoxColor, false, Duration, 0, 0.5f);
	}

	// Draw constraints
	for (const FOHConstraintInstanceData& Constraint : Graph.GetConstraintLinks())
	{
		const FName Parent = Constraint.GetParentBone();
		const FName Child = Constraint.GetChildBone();

		const int32 ParentIndex = MeshComp->GetBoneIndex(Parent);
		const int32 ChildIndex = MeshComp->GetBoneIndex(Child);

		if (ParentIndex == INDEX_NONE || ChildIndex == INDEX_NONE) continue;

		const FVector P0 = MeshComp->GetBoneTransform(ParentIndex).GetLocation();
		const FVector P1 = MeshComp->GetBoneTransform(ChildIndex).GetLocation();
		const FVector Midpoint = (P0 + P1) * 0.5f;

		const float Influence = CalculateConstraintInfluenceScore(Constraint, Graph, MeshComp);
		const FColor SphereColor = FColor::MakeRedToGreenColorFromScalar(Influence);

		DrawDebugLine(World, P0, P1, SphereColor, false, Duration, 0, 0.75f);
		DrawDebugSphere(World, Midpoint, 5.f, 12, SphereColor, false, Duration, 0, 1.0f);
	}
}

float UOHGraphUtils::CalculateConstraintInfluenceScore(
	const FOHConstraintInstanceData& Constraint,
	const FOHPhysicsGraphNode& Graph,
	const USkeletalMeshComponent* MeshComp)
{
	const FVector Center = [&]
	{
		const int32 P = MeshComp->GetBoneIndex(Constraint.GetParentBone());
		const int32 C = MeshComp->GetBoneIndex(Constraint.GetChildBone());
		return (MeshComp->GetBoneTransform(P).GetLocation() + MeshComp->GetBoneTransform(C).GetLocation()) * 0.5f;
	}();

	// === Spatial Crowding ===
	int32 NearbyConstraints = 0;
	for (const FOHConstraintInstanceData& Other : Graph.GetConstraintLinks())
	{
		if (&Other == &Constraint) continue;
		const int32 OP = MeshComp->GetBoneIndex(Other.GetParentBone());
		const int32 OC = MeshComp->GetBoneIndex(Other.GetChildBone());
		if (OP == INDEX_NONE || OC == INDEX_NONE) continue;

		const FVector OtherMid = (MeshComp->GetBoneTransform(OP).GetLocation() + MeshComp->GetBoneTransform(OC).GetLocation()) * 0.5f;
		if (FVector::DistSquared(Center, OtherMid) < FMath::Square(30.f)) ++NearbyConstraints;
	}
	const float SpatialCrowding = FMath::Clamp(static_cast<float>(NearbyConstraints) / 10.f, 0.f, 1.f);

	// === Topological Density ===
	int32 LinksIn = 0;
	for (const FOHConstraintInstanceData& Link : Graph.GetConstraintLinks())
	{
		if (Link.GetParentBone() == Constraint.GetParentBone() || Link.GetChildBone() == Constraint.GetParentBone()) ++LinksIn;
		if (Link.GetParentBone() == Constraint.GetChildBone() || Link.GetChildBone() == Constraint.GetChildBone()) ++LinksIn;
	}
	const float TopologicalCrowding = FMath::Clamp(static_cast<float>(LinksIn) / 12.f, 0.f, 1.f);

	// === Physical and Runtime Dynamics ===
	const FOHBoneData* PData = Graph.FindBoneData(Constraint.GetParentBone());
	const FOHBoneData* CData = Graph.FindBoneData(Constraint.GetChildBone());

	float Mass = 0.f, Inertia = 0.f, Damping = 0.f, Stiffness = 0.f;
	float VelocityDelta = 0.f, AccelDelta = 0.f;

	if (PData && CData)
	{
		if (const FBodyInstance* PBI = PData->GetBodyInstance())
		{
			Mass += PBI->GetBodyMass();
			Inertia += PBI->GetBodyInertiaTensor().Size();
		}
		if (const FBodyInstance* CBI = CData->GetBodyInstance())
		{
			Mass += CBI->GetBodyMass();
			Inertia += CBI->GetBodyInertiaTensor().Size();
		}

		VelocityDelta = FVector::Dist(PData->GetBodyLinearVelocity(), CData->GetBodyLinearVelocity());
		AccelDelta = FVector::Dist(PData->GetLinearAcceleration(), CData->GetLinearAcceleration());
	}

	if (const FConstraintInstance* CI = Constraint.GetConstraintInstance())
	{
		if (CI->ProfileInstance.LinearLimit.bSoftConstraint)
		{
			Damping = CI->ProfileInstance.LinearLimit.Damping / 100.f;
			Stiffness = CI->ProfileInstance.LinearLimit.Stiffness / 1000.f;
		}
	}

	const float BodyWeightFactor = FMath::Clamp(Mass / 200.f, 0.f, 1.f);
	const float InertiaFactor = FMath::Clamp(Inertia / 1000.f, 0.f, 1.f);
	const float DampingScore = FMath::Clamp(Damping, 0.f, 1.f);
	const float StiffnessScore = FMath::Clamp(Stiffness, 0.f, 1.f);
	const float VelocityInfluence = FMath::Clamp(VelocityDelta / 100.f, 0.f, 1.f);
	const float AccelInfluence = FMath::Clamp(AccelDelta / 100.f, 0.f, 1.f);

	// === Final Composite ===
	float Composite = 0.f;
	Composite += SpatialCrowding * 0.15f;
	Composite += TopologicalCrowding * 0.15f;
	Composite += BodyWeightFactor * 0.15f;
	Composite += InertiaFactor * 0.15f;
	Composite += DampingScore * 0.1f;
	Composite += StiffnessScore * 0.1f;
	Composite += VelocityInfluence * 0.125f;
	Composite += AccelInfluence * 0.125f;

	return FMath::Clamp(Composite, 0.f, 1.f);
}

// --- Cycle detection utility ---

static bool HasCycleUtil(
    const FName& Bone,
    const TMap<FName, TArray<FName>>& AdjacencyMap,
    TSet<FName>& Visited,
    TSet<FName>& RecStack)
{
    if (!Visited.Contains(Bone))
    {
        Visited.Add(Bone);
        RecStack.Add(Bone);

        if (const TArray<FName>* Neighbors = AdjacencyMap.Find(Bone))
        {
            for (const FName& Neighbor : *Neighbors)
            {
                if (!Visited.Contains(Neighbor) && HasCycleUtil(Neighbor, AdjacencyMap, Visited, RecStack))
                {
                    return true;
                }
                else if (RecStack.Contains(Neighbor))
                {
                    return true;
                }
            }
        }
    }
    RecStack.Remove(Bone);
    return false;
}

bool UOHGraphUtils::DetectCyclesInGraph(
    const TMap<FName, FOHBoneData>& BoneMap,
    const TArray<FOHConstraintInstanceData>& Constraints)
{
    // Build simple adjacency map for cycle detection
    TMap<FName, TArray<FName>> AdjacencyMap;
    for (const FOHConstraintInstanceData& Constraint : Constraints)
    {
        AdjacencyMap.FindOrAdd(Constraint.GetParentBone()).Add(Constraint.GetChildBone());
    }

    TSet<FName> Visited;
    TSet<FName> RecursionStack;

    for (const auto& Pair : BoneMap)
    {
        const FName& Bone = Pair.Key;
        if (!Visited.Contains(Bone))
        {
            if (HasCycleUtil(Bone, AdjacencyMap, Visited, RecursionStack))
            {
                UE_LOG(LogTemp, Warning, TEXT("[OHGraphUtils] Cycle detected involving bone: %s"), *Bone.ToString());
                return true;
            }
        }
    }
    return false;
}

void UOHGraphUtils::UpdateAdjacencyCaches(FOHPhysicsGraphNode& Graph)
{
	// Rebuild both parent→child and bone→constraint multimaps in one go
	Graph.RebuildWrappers();
}

void UOHGraphUtils::RefreshConstraintInstances(
	FOHPhysicsGraphNode&       Graph,
	USkeletalMeshComponent*    SkeletalMesh
)
{
	if (!SkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("RefreshConstraintInstances: SkeletalMeshComponent is null, skipping."));
		return;
	}

	// Walk every FOHConstraintInstanceData in the graph
	for (FOHConstraintInstanceData& Entry : Graph.GetConstraintLinks())
	{
		// Find the live FConstraintInstance by matching JointName == ConstraintName
		FConstraintInstance* FoundCI = SkeletalMesh->FindConstraintInstance(Entry.GetConstraintName());

		// Update the runtime pointer (or nullptr if not found)
		Entry.SetConstraintInstance(FoundCI);
	}

	// Finally, since constraint pointers changed, rebuild adjacency if needed
	Graph.RebuildWrappers();

	UE_LOG(LogTemp, Verbose,
		TEXT("RefreshConstraintInstances: Rebound %d/%d constraints."),
		// Count how many we actually found
		[&]()
		{
			int32 Bound=0;
			for (auto& E : Graph.GetConstraintLinks())
				if (E.GetConstraintInstance()) ++Bound;
			return Bound;
		}(),
		Graph.GetConstraintLinks().Num()
	);
}

bool UOHGraphUtils::DetectCyclesInGraph(const FOHPhysicsGraphNode& Graph)
{
	// Build a flat adjacency map: Bone → [ChildrenBones]
	TMap<FName, TArray<FName>> Flat;
	for (const auto& Pair : Graph.GetBoneMap())
	{
		const FName Bone = Pair.Key;
		Flat.Add(Bone, Graph.GetChildrenOfBone(Bone));
	}

	// AssertNoCycles returns true if no cycles are found, so invert it
	return !OHSafeMapUtils::AssertNoCycles(Flat, TEXT("FOHPhysicsGraphNode"));
}

bool UOHGraphUtils::ValidateGraph(
	FOHPhysicsGraphNode&        Graph,
	EValidationStrictness       Strictness
)
{
	// Grab the raw data
	const auto& BoneMap     = Graph.GetBoneMap();
	const auto& Constraints = Graph.GetConstraintLinks();

	// Find any bones that have no incoming or outgoing constraints
	TSet<FName> BonesWithIssues = OHSafeMapUtils::FindUnconstrainedBones<FOHConstraintInstanceData>(
		BoneMap,
		Constraints,
		/* ParentKeyFn = */ [](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
		/* ChildKeyFn  = */ [](const FOHConstraintInstanceData& C) { return C.GetChildBone(); }
	);

	if (Strictness == EValidationStrictness::Strict)
	{
		// In strict mode, fail if there are any unconstrained bones
		// or if we detect any cycles
		return BonesWithIssues.IsEmpty()
			&& !DetectCyclesInGraph(Graph);
	}
	else
	{
		// In lenient mode, always pass
		return true;
	}
}

void UOHGraphUtils::PruneInvalidGraphElements(FOHPhysicsGraphNode& Graph, EValidationStrictness Strictness)
{
	auto& BoneMap = Graph.GetBoneMap();
	auto& Constraints = Graph.GetConstraintLinks();

	TSet<FName> InvalidBones = OHSafeMapUtils::FindUnconstrainedBones<FOHConstraintInstanceData>(
		BoneMap,
		Constraints,
		[](const FOHConstraintInstanceData& C) { return C.GetParentBone(); },
		[](const FOHConstraintInstanceData& C) { return C.GetChildBone(); }
	);

	for (const FName& Name : InvalidBones)
	{
		BoneMap.Remove(Name);
	}

	Constraints.RemoveAll([&](const FOHConstraintInstanceData& C)
	{
		return !BoneMap.Contains(C.GetParentBone()) || !BoneMap.Contains(C.GetChildBone());
	});
}

void UOHGraphUtils::DrawPhysicsGraphOverlay(
    const FOHPhysicsGraphNode& Graph,
    const USkeletalMeshComponent* MeshComp,
    UWorld* World,
    float Duration,
    EPhysicsGraphOverlayMode OverlayMode,
    bool bDrawLabels)
{
    if (!MeshComp || !World) return;

    const bool bDrawFull = (OverlayMode == EPhysicsGraphOverlayMode::Full || OverlayMode == EPhysicsGraphOverlayMode::Both);
    const bool bDrawAnomalies = (OverlayMode == EPhysicsGraphOverlayMode::Anomalies || OverlayMode == EPhysicsGraphOverlayMode::Both);

    // === Full Graph Overlay ===
    if (bDrawFull)
    {
        // Draw bones
        for (const auto& Pair : Graph.GetBoneMap())
        {
            const FName BoneName = Pair.Key;
            const FOHBoneData& Bone = Pair.Value;
            const int32 BoneIndex = MeshComp->GetBoneIndex(BoneName);
            if (BoneIndex == INDEX_NONE) continue;

            const FTransform BoneTransform = MeshComp->GetBoneTransform(BoneIndex);
            const FVector Pos = BoneTransform.GetLocation();
            const float Mass = FMath::Max(1.f, Bone.GetCachedBodyMass());
            const float Length = FMath::Max(1.f, Bone.GetCachedBoneLength());
            const FVector BoxExtent = FVector(5.f, 5.f, Length * 0.5f);
            const FColor BoxColor = Bone.GetHasSimulatedBody() ? FColor::Green : FColor::Red;

            DrawDebugBox(World, Pos, BoxExtent, BoneTransform.GetRotation(), BoxColor, false, Duration, 0, 0.5f);

            if (bDrawLabels) {
                FString Label = FString::Printf(TEXT("%s\nM:%.1f L:%.1f"), *BoneName.ToString(), Mass, Length);
                DrawDebugString(World, Pos + FVector(0,0,Length), Label, nullptr, BoxColor, Duration, false);
            }
        }

        // Draw constraints
        for (const FOHConstraintInstanceData& Constraint : Graph.GetConstraintLinks())
        {
            const FName Parent = Constraint.GetParentBone();
            const FName Child = Constraint.GetChildBone();
            const int32 ParentIndex = MeshComp->GetBoneIndex(Parent);
            const int32 ChildIndex = MeshComp->GetBoneIndex(Child);
            if (ParentIndex == INDEX_NONE || ChildIndex == INDEX_NONE) continue;

            const FVector P0 = MeshComp->GetBoneTransform(ParentIndex).GetLocation();
            const FVector P1 = MeshComp->GetBoneTransform(ChildIndex).GetLocation();
            const FVector Midpoint = (P0 + P1) * 0.5f;
            const float Influence = CalculateConstraintInfluenceScore(Constraint, Graph, MeshComp);
            const FColor LineColor = FColor::MakeRedToGreenColorFromScalar(Influence);

            DrawDebugLine(World, P0, P1, LineColor, false, Duration, 0, 1.5f);
            DrawDebugSphere(World, Midpoint, 5.f, 10, LineColor, false, Duration);

            if (bDrawLabels) {
                FString Label = FString::Printf(TEXT("%s\nI:%.2f"), *Constraint.GetConstraintName().ToString(), Influence);
                DrawDebugString(World, Midpoint + FVector(0,0,10), Label, nullptr, LineColor, Duration, false);
            }
        }
    }

    // === Anomaly Overlay ===
    if (bDrawAnomalies)
    {
        // Orphans
        TSet<FName> Orphans;
        for (const auto& Pair : Graph.GetBoneMap())
        {
            bool bReferenced = false;
            for (const auto& Constraint : Graph.GetConstraintLinks())
            {
                if (Constraint.GetParentBone() == Pair.Key || Constraint.GetChildBone() == Pair.Key)
                {
                    bReferenced = true; break;
                }
            }
            if (!bReferenced)
                Orphans.Add(Pair.Key);
        }
        for (FName Orphan : Orphans)
        {
            int32 BoneIndex = MeshComp->GetBoneIndex(Orphan);
            if (BoneIndex != INDEX_NONE)
            {
                FVector Pos = MeshComp->GetBoneTransform(BoneIndex).GetLocation();
                DrawDebugSphere(World, Pos, 12, 16, FColor::Magenta, false, Duration);
                DrawDebugString(World, Pos + FVector(0,0,18), TEXT("ORPHAN"), nullptr, FColor::Magenta, Duration, false);
            }
        }

        // Cycles (example: highlight cycle path)
        TArray<FName> CyclePath;
        if (Graph.HasCycles(&CyclePath) && CyclePath.Num() > 1)
        {
            for (int32 i = 0; i < CyclePath.Num() - 1; ++i)
            {
                int32 IdxA = MeshComp->GetBoneIndex(CyclePath[i]);
                int32 IdxB = MeshComp->GetBoneIndex(CyclePath[i+1]);
                if (IdxA == INDEX_NONE || IdxB == INDEX_NONE) continue;
                FVector A = MeshComp->GetBoneTransform(IdxA).GetLocation();
                FVector B = MeshComp->GetBoneTransform(IdxB).GetLocation();
                DrawDebugLine(World, A, B, FColor::Orange, false, Duration, 0, 5.f);
            }
            // Close the cycle
            int32 IdxA = MeshComp->GetBoneIndex(CyclePath.Last());
            int32 IdxB = MeshComp->GetBoneIndex(CyclePath[0]);
            if (IdxA != INDEX_NONE && IdxB != INDEX_NONE)
            {
                FVector A = MeshComp->GetBoneTransform(IdxA).GetLocation();
                FVector B = MeshComp->GetBoneTransform(IdxB).GetLocation();
                DrawDebugLine(World, A, B, FColor::Orange, false, Duration, 0, 5.f);
            }
        }

        // Islands (disconnected subgraphs)
        TArray<TSet<FName>> Islands = Graph.GetIslands();
        if (Islands.Num() > 1)
        {
            const TArray<FColor> Palette = {
                FColor::Blue, FColor::Green, FColor::Cyan, FColor::Purple, FColor::Yellow
            };
            for (int32 i = 0; i < Islands.Num(); ++i)
            {
                FColor Color = Palette[i % Palette.Num()];
                for (const FName& Bone : Islands[i])
                {
                    int32 BoneIndex = MeshComp->GetBoneIndex(Bone);
                    if (BoneIndex != INDEX_NONE)
                    {
                        FVector Pos = MeshComp->GetBoneTransform(BoneIndex).GetLocation();
                        DrawDebugSphere(World, Pos, 7.f, 10, Color, false, Duration);
                        if (bDrawLabels)
                            DrawDebugString(World, Pos + FVector(0,0,12), FString::Printf(TEXT("Island %d"), i+1), nullptr, Color, Duration, false);
                    }
                }
            }
        }

        // TODO: Add more anomaly overlays (outliers, invalid constraints, etc) as needed
    }
}

// Overlay state (file-scope or static inside a manager class)
static EPhysicsGraphOverlayMode GPhysicsGraphOverlayMode = EPhysicsGraphOverlayMode::None;

// Accessor for console/PIE use
EPhysicsGraphOverlayMode UOHGraphUtils::GetPhysicsGraphOverlayMode()
{
	return GPhysicsGraphOverlayMode;
}
void UOHGraphUtils::SetPhysicsGraphOverlayMode(EPhysicsGraphOverlayMode Mode)
{
	GPhysicsGraphOverlayMode = Mode;
}
