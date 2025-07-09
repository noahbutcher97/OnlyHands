#include "Animation/Nodes/AnimNode_ArmCompressionResponse.h"
#include "Animation/AnimInstanceProxy.h"
#include "CoreMinimal.h"
#include "DrawDebugHelpers.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "Stats/Stats.h"

DEFINE_LOG_CATEGORY(LogOnlyHands);
DECLARE_STATS_GROUP(TEXT("OnlyHands"), STATGROUP_OnlyHands, STATCAT_Advanced);

DECLARE_CYCLE_STAT(
    TEXT("OnlyHands ArmCompression"), // What shows up in the profiler
    STAT_OnlyHands_ArmCompression, // The symbol you pass to SCOPE_CYCLE_COUNTER
    STATGROUP_OnlyHands            // The group you just declared
);

// FAnimNode_ArmCompressionResponse

void FAnimNode_ArmCompressionResponse::CacheBones_AnyThread(
    const FAnimationCacheBonesContext &Context) {
  Super::CacheBones_AnyThread(Context);

  // Get the required bones from the proxy, not Context.Pose
  const FBoneContainer &RequiredBones =
      Context.AnimInstanceProxy->GetRequiredBones();

  if (bUseEnumSetup) {
    TArray<FName> ValidNames;
    UOHSkeletalPhysicsUtils::BuildValidatedBoneChain(HandBoneEnum,
                                                     RequiredBones, ValidNames);

    if (ValidNames.Num() < 3) {
      UE_LOG(LogOnlyHands, Warning,
             TEXT("ArmCompressionResponse: enum %s yielded only %d valid "
                  "bones, disabling auto-setup"),
             *UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(
                  HandBoneEnum)
                  .ToString(),
             ValidNames.Num());
      bUseEnumSetup = false;
    } else {
      const int32 N = ValidNames.Num();
      UpperArmBone.BoneName = ValidNames[N - 3];
      LowerArmBone.BoneName = ValidNames[N - 2];
      HandBone.BoneName = ValidNames[N - 1];
    }
  }

  // Now initialize the three FBoneReferences against the real bone container
  InitializeAndValidateBoneRef(UpperArmBone, RequiredBones);
  InitializeAndValidateBoneRef(LowerArmBone, RequiredBones);
  InitializeAndValidateBoneRef(HandBone, RequiredBones);
}

bool FAnimNode_ArmCompressionResponse::IsValidToEvaluate(
    const USkeleton * /*Skeleton*/, const FBoneContainer &RequiredBones) {
  // Only valid when all three refs resolve
  return HandBone.IsValidToEvaluate(RequiredBones) &&
         LowerArmBone.IsValidToEvaluate(RequiredBones) &&
         UpperArmBone.IsValidToEvaluate(RequiredBones);
}

void FAnimNode_ArmCompressionResponse::EvaluateSkeletalControl_AnyThread(
    FComponentSpacePoseContext &Output,
    TArray<FBoneTransform> &OutBoneTransforms) {
  SCOPE_CYCLE_COUNTER(STAT_OnlyHands_ArmCompression);

  // 0) Quick pointer checks
  if (!Output.AnimInstanceProxy) {
    return;
  }
  USkeletalMeshComponent *SkelComp =
      Output.AnimInstanceProxy->GetSkelMeshComponent();
  if (!SkelComp) {
    return;
  }

  // 1) Reset & grab CSPose + bone container
  OutBoneTransforms.Reset();
  FCSPose<FCompactPose> &CSPose = Output.Pose;
  const FCompactPose &PoseCS = CSPose.GetPose();
  const FBoneContainer &BoneC = PoseCS.GetBoneContainer();

  // 2) Resolve our three indices
  const FCompactPoseBoneIndex iHand = HandBone.GetCompactPoseIndex(BoneC);
  const FCompactPoseBoneIndex iLowerArm =
      LowerArmBone.GetCompactPoseIndex(BoneC);
  const FCompactPoseBoneIndex iUpperArm =
      UpperArmBone.GetCompactPoseIndex(BoneC);
  if (!PoseCS.IsValidIndex(iHand) || !PoseCS.IsValidIndex(iLowerArm) ||
      !PoseCS.IsValidIndex(iUpperArm)) {
    UE_LOG(LogOnlyHands, Warning, TEXT("ArmCompression: invalid bone indices"));
    return;
  }

  // 3) Pull component-space transforms
  FTransform HandTM = CSPose.GetComponentSpaceTransform(iHand);
  FTransform LowerTM = CSPose.GetComponentSpaceTransform(iLowerArm);
  FTransform UpperTM = CSPose.GetComponentSpaceTransform(iUpperArm);

  // 4) Base target from velocity-driven compression
  const FVector Velocity = SkelComp->GetComponentVelocity();
  const FVector VelDir =
      Velocity.IsNearlyZero() ? FVector::ZeroVector : Velocity.GetSafeNormal();
  const FVector BaseTarget =
      HandTM.GetLocation() - VelDir * CompressionThreshold;

  // 5) Update our runtime override alpha
  const float DeltaTime = Output.AnimInstanceProxy->GetDeltaSeconds();
  CurrentEffectorAlpha =
      FMath::FInterpTo(CurrentEffectorAlpha, bUseEffectorOverride ? 1.f : 0.f,
                       DeltaTime, EffectorInterpSpeed);

  // 6) Blend Base→EffectorTarget
  FVector TargetPos =
      FMath::Lerp(BaseTarget, EffectorTarget, CurrentEffectorAlpha);

  // 7) Predictive offset
  TargetPos += Velocity * PredictiveOffset;

  // 8) Joint hint (with editor offset + fallback)
  FVector JointHint = LowerTM.GetLocation() + JointTargetOffset;
  if ((JointHint - UpperTM.GetLocation()).IsNearlyZero()) {
    // if it collapses onto the upper joint, pick a safe bend axis
    const FVector Axis =
        UOHSkeletalPhysicsUtils::ComputeFallbackBendAxis(UpperTM, LowerTM);
    const float Len = (LowerTM.GetLocation() - UpperTM.GetLocation()).Size();
    JointHint =
        UpperTM.GetLocation() + UpperTM.TransformVectorNoScale(Axis) * Len;
  }

  // 9) Mirror across your chosen axis if requested
  if (bMirror) {
    const FVector RootPos = UpperTM.GetLocation();
    FVector Rel = TargetPos - RootPos;
    const FVector Scale(
        FMath::Abs(MirrorHintAxis.X) > KINDA_SMALL_NUMBER ? -1.f : 1.f,
        FMath::Abs(MirrorHintAxis.Y) > KINDA_SMALL_NUMBER ? -1.f : 1.f,
        FMath::Abs(MirrorHintAxis.Z) > KINDA_SMALL_NUMBER ? -1.f : 1.f);
    TargetPos = RootPos + Rel * Scale;
  }

  // 10) Bail on NaN/Inf
  if (!FMath::IsFinite(TargetPos.SizeSquared()) ||
      !FMath::IsFinite(JointHint.SizeSquared())) {
    UE_LOG(LogOnlyHands, Warning, TEXT("ArmCompression: invalid target/hint"));
    return;
  }

  // 11) Compute max stretch scale
  const float BoneLen = (UpperTM.GetLocation() - LowerTM.GetLocation()).Size();
  const float SafeLen = FMath::Max(BoneLen, KINDA_SMALL_NUMBER);
  const float MaxStretch =
      bClampCompression ? (SafeLen + MaxCompressionDistance) / SafeLen : 1.f;

  // 12) Solve the two-bone IK
  UOHSkeletalPhysicsUtils::SolveIK_TwoBone(
      UpperTM, LowerTM, HandTM, TargetPos, JointHint, bClampCompression,
      /*StartStretchRatio=*/1.f, MaxStretch,
      bEnableDebug ? SkelComp->GetWorld() : nullptr, bEnableDebug);

  // 13) Push back for LocalBlendCSBoneTransforms
  OutBoneTransforms.Add(FBoneTransform(iUpperArm, UpperTM));
  OutBoneTransforms.Add(FBoneTransform(iLowerArm, LowerTM));
  OutBoneTransforms.Add(FBoneTransform(iHand, HandTM));

  // 14) Finally sort & strip out any dupes / invalids
  UOHSkeletalPhysicsUtils::ValidateAndSortBoneTransforms(
      OutBoneTransforms,
      /*bSortByIndex=*/true,
      /*bLogWarnings=*/bEnableDebug);
}

///////////////////////////////////////////////////////
// Enum-driven chain helper

void FAnimNode_ArmCompressionResponse::InitializeChainFromEnum(
    const FBoneContainer &RequiredBones) {
  // Build parent-lineage from the hand enum toward root
  TArray<EOHSkeletalBone> Lineage =
      UOHSkeletalPhysicsUtils::GetBoneLineageToRoot(HandBoneEnum);
  Algo::Reverse(Lineage);
  if (Lineage.Num() < 3) {
    UE_LOG(LogOnlyHands, Warning,
           TEXT("ArmCompressionResponse: HandBoneEnum '%s' yields too short "
                "chain (%d bones)"),
           *UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(
                HandBoneEnum)
                .ToString(),
           Lineage.Num());
    return;
  }

  // Last three are [ ..., UpperArm, LowerArm, Hand ]
  const int32 Last = Lineage.Num() - 1;
  UpperArmBone.BoneName =
      UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(
          Lineage[Last - 2]);
  LowerArmBone.BoneName =
      UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(
          Lineage[Last - 1]);
  HandBone.BoneName =
      UOHSkeletalPhysicsUtils::ResolveBoneNameFromSkeletalBone_Static(
          Lineage[Last]);

  // Immediately initialize so IsValidToEvaluate() will return true next frame
  UpperArmBone.Initialize(RequiredBones);
  LowerArmBone.Initialize(RequiredBones);
  HandBone.Initialize(RequiredBones);
}

#if 0

// --- Local Pose Safety Helpers (Scoped to this node) ---
namespace
{    
	FORCEINLINE bool IsAnimContextValid(const FComponentSpacePoseContext& Output)
	{
		return Output.AnimInstanceProxy && Output.AnimInstanceProxy->GetSkelMeshComponent();
	}

	FORCEINLINE bool IsPoseIndexSafe(const FCompactPose& Pose, FCompactPoseBoneIndex Index)
	{
		return Index.IsValid() && Pose.IsValidIndex(Index);
	}

	FORCEINLINE bool AreIndicesSafe(const FCompactPose& Pose,
		FCompactPoseBoneIndex A, FCompactPoseBoneIndex B, FCompactPoseBoneIndex C)
	{
		return IsPoseIndexSafe(Pose, A) && IsPoseIndexSafe(Pose, B) && IsPoseIndexSafe(Pose, C);
	}
}

#define LOCTEXT_NAMESPACE "ArmCompressionResponse"

#if WITH_EDITORONLY_DATA
void FAnimNode_ArmCompressionResponse::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
    // 1) Reset our chain‐built flag and clear any stale warning text
    bChainInitialized = false;

    // 2) Let the base class drive InitializeBoneReferences (and thus our override below)
    FAnimNode_SkeletalControlBase::CacheBones_AnyThread(Context);
}
#endif

void FAnimNode_ArmCompressionResponse::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{

    // 2) Only build + validate the chain once per cache pass
    if (bChainInitialized)
    {
        return;
    }
    bChainInitialized = true;

    // 3) Start fresh
    BoneRefs.Empty();
    ResolvedIndices.Empty();

    // 4) Early-out if no enum specified
    if (HandBoneEnum == EOHSkeletalBone::None)
    {
#if WITH_EDITOR
        if (GIsEditor)
        {
            UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: HandBoneEnum is None. Node disabled in editor."));
        }
#endif
        return;
    }

    // 5) Build the bone chain: enum-driven or manual
    if (bUseEnumSetup)
    {
        InitializeIKChainFromEnum(HandBoneEnum, RequiredBones);
    }
    else
    {
        BoneRefs.Add(HandBone);
        BoneRefs.Add(LowerArmBone);
        BoneRefs.Add(UpperArmBone);
    }

    // 6) Initialize & validate each reference
    for (FBoneReference& Ref : BoneRefs)
    {
        InitializeAndValidateBoneRef(Ref, RequiredBones);
    }

    // 7) Cache the compact-pose indices
    ResolvedIndices.Reserve(BoneRefs.Num());
    for (const FBoneReference& Ref : BoneRefs)
    {
        if (Ref.IsValidToEvaluate(RequiredBones))
        {
            ResolvedIndices.Add(Ref.GetCompactPoseIndex(RequiredBones));
        }
        else
        {
#if WITH_EDITOR
            if (GIsEditor)
            {
                UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: Invalid bone '%s'"), *Ref.BoneName.ToString());
            }
#endif
        }
    }

    // 8) Warn if the resulting chain length isn’t 3–4 bones
    const int32 ChainLen = ResolvedIndices.Num();
    if (ChainLen < 3 || ChainLen > 4)
    {
#if WITH_EDITOR
        if (GIsEditor)
        {
            UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: Invalid chain length %d"), ChainLen);
        }
#endif
    }
}


bool FAnimNode_ArmCompressionResponse::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return IsValidToEvaluate_Internal(RequiredBones);
}


bool FAnimNode_ArmCompressionResponse::IsValidToEvaluate_Internal(const FBoneContainer& RequiredBones) const
{
	if (BoneRefs.Num() < 3 || ResolvedIndices.Num() < 3)
	{
#if WITH_EDITOR
		if (GIsEditor)
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Invalid bone setup in editor preview (BoneRefs: %d, Indices: %d)"), BoneRefs.Num(), ResolvedIndices.Num());
		}
#endif
		return false;
	}

	for (int32 i = 0; i < BoneRefs.Num(); ++i)
	{
		if (!BoneRefs[i].IsValidToEvaluate(RequiredBones) || !ResolvedIndices[i].IsValid())
		{
#if WITH_EDITOR
			if (GIsEditor)
			{
				UE_LOG(LogTemp, Warning, TEXT("ArmCompression: BoneEnum or index invalid [%d]: %s"), i, *BoneRefs[i].BoneName.ToString());
			}
#endif
			return false;
		}
	}

	return true;
}

FName FAnimNode_ArmCompressionResponse::GetBoneNameFromEnum(EOHSkeletalBone BoneEnum)
{
	return UOHSkeletalPhysicsUtils::GetBoneNameFromEnum(BoneEnum);
}

EOHSkeletalBone FAnimNode_ArmCompressionResponse::GetParentBoneEnum(EOHSkeletalBone BoneEnum)
{
	return UOHSkeletalPhysicsUtils::GetParentBone(BoneEnum);
}



// Solve 2 BoneEnum IK

static void SolveIK_2Bone(
	FTransform& OutUpper,
	FTransform& OutLower,
	FTransform& OutEnd,
	const FVector& TargetPosition,
	const FVector& JointTarget,
	bool bAllowStretching,
	float StartStretchRatio,
	float MaxStretchScale,
	UWorld* WorldDebugContext,
	bool bDrawDebug
)
{
	const FVector RootPos  = OutUpper.GetLocation();
	const FVector ElbowPos = OutLower.GetLocation();
	const FVector WristPos = OutEnd.GetLocation();

	const float UpperLen = FVector::Distance(RootPos, ElbowPos);
	const float LowerLen = FVector::Distance(ElbowPos, WristPos);
	if (UpperLen <= KINDA_SMALL_NUMBER || LowerLen <= KINDA_SMALL_NUMBER) return;

	const FVector DesiredDir = TargetPosition - RootPos;
	const float DesiredLength = DesiredDir.Size();
	if (DesiredLength <= KINDA_SMALL_NUMBER) return;

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

	const FQuat UpperRot = FQuat(RotationAxis, Angle);
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

namespace ArmIK
{
	static FVector ComputeFallbackBendAxis(const FTransform& Root, const FTransform& Mid)
	{
		// Use local +Y (or mirrored -Y) as fallback bend axis
		const FVector LocalAxisY = Root.GetUnitAxis(EAxis::Y);
		const FVector MidVec = Mid.GetLocation() - Root.GetLocation();
		const FVector Fallback = FVector::CrossProduct(MidVec, LocalAxisY).GetSafeNormal();

		return Fallback.IsNearlyZero() ? FVector::UpVector : Fallback;
	}

	static void SolveIK_ArmChainGeneral(
		FTransform& OutRoot,
		FTransform& OutMid,
		FTransform& OutEnd,
		FTransform& OutEffector,
		const FVector& IKTarget,
		const FVector& ElbowHint,
		float StretchStartRatio,
		float MaxStretchScale,
		bool bEnableEffectorRotation,
		const FVector& EffectorLookAxis,
		UWorld* DebugWorld,
		bool bDrawDebug)
	{
		const FVector RootPos = OutRoot.GetLocation();
		const FVector MidPos = OutMid.GetLocation();
		const FVector EndPos = OutEnd.GetLocation();
		const FVector EffectorPos = OutEffector.GetLocation();

		const float UpperLen = FVector::Dist(RootPos, MidPos);
		const float LowerLen = FVector::Dist(MidPos, EndPos);
		const float EndLen   = FVector::Dist(EndPos, EffectorPos);

		if (UpperLen < KINDA_SMALL_NUMBER || LowerLen < KINDA_SMALL_NUMBER)
			return;

		const FVector DesiredDir = IKTarget - RootPos;
		const float DesiredLen = DesiredDir.Size();
		if (DesiredLen < KINDA_SMALL_NUMBER)
			return;

		// --- Stretching ---
		float StretchScale = 1.f;
		const float ChainLen = UpperLen + LowerLen;
		if (DesiredLen > ChainLen * StretchStartRatio)
		{
			StretchScale = FMath::Min(DesiredLen / ChainLen, MaxStretchScale);
		}

		const float A = UpperLen * StretchScale;
		const float B = LowerLen * StretchScale;
		const float C = DesiredLen;

		const FVector DirectionNorm = DesiredDir / DesiredLen;

		// --- Law of Cosines ---
		const float CosAngle = FMath::Clamp((A*A + C*C - B*B) / (2*A*C), -1.f, 1.f);
		const float Angle = FMath::Acos(CosAngle);

		FVector HintVec = ElbowHint - RootPos;
		FVector RotationAxis = FVector::CrossProduct(DirectionNorm, HintVec).GetSafeNormal();

		if (RotationAxis.IsNearlyZero())
		{
			RotationAxis = ArmIK::ComputeFallbackBendAxis(OutRoot, OutMid);
		}

		// --- Solve mid and end ---
		const FQuat UpperRot = FQuat(RotationAxis, Angle);
		const FVector NewMid = RootPos + UpperRot.RotateVector(DirectionNorm * A);
		const FVector NewEnd = IKTarget;

		const FVector UpperDir = NewMid - RootPos;
		const FVector LowerDir = NewEnd - NewMid;

		OutRoot.SetRotation(UpperDir.ToOrientationQuat());
		OutMid.SetRotation(LowerDir.ToOrientationQuat());

		OutMid.SetLocation(NewMid);
		OutEnd.SetLocation(NewEnd);

		// --- Optional effector orientation ---
		if (bEnableEffectorRotation)
		{
			const FVector Forward = (NewEnd - NewMid).GetSafeNormal();
			const FVector RefUp = FVector::UpVector;
			const FVector Right = FVector::CrossProduct(RefUp, Forward).GetSafeNormal();
			const FVector Up = FVector::CrossProduct(Forward, Right).GetSafeNormal();

			OutEffector.SetRotation(FMatrix(Forward, Right, Up, FVector::ZeroVector).ToQuat());
		}

		// --- Debug ---
		if (bDrawDebug && DebugWorld)
		{
			DrawDebugLine(DebugWorld, RootPos, NewMid, FColor::Blue, false, 0.1f, 0, 2.f);
			DrawDebugLine(DebugWorld, NewMid, NewEnd, FColor::Green, false, 0.1f, 0, 2.f);
			DrawDebugSphere(DebugWorld, NewMid, 3.f, 8, FColor::Cyan, false, 0.1f);
			DrawDebugSphere(DebugWorld, NewEnd, 3.f, 8, FColor::Yellow, false, 0.1f);
		}
	}
}


void FAnimNode_ArmCompressionResponse::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output,
	TArray<FBoneTransform>& OutBoneTransforms)
{

#if WITH_EDITOR
	// Skip evaluation if we're in editor and the AnimInstance isn't valid
	if (GIsEditor && (!Output.AnimInstanceProxy || !Output.AnimInstanceProxy->GetSkelMeshComponent()))
	{
		return;
	}
#endif

	if (!IsAnimContextValid(Output) || !IsValidToEvaluate_Internal(Output.AnimInstanceProxy->GetRequiredBones()))
	{
		return;
	}
	USkeletalMeshComponent* SkelMesh = Output.AnimInstanceProxy ? Output.AnimInstanceProxy->GetSkelMeshComponent() : nullptr;
	UWorld* World = SkelMesh ? SkelMesh->GetWorld() : nullptr;

	ensureMsgf(Output.AnimInstanceProxy, TEXT("ArmCompressionNode: AnimInstanceProxy is null"));
	ensureMsgf(SkelMesh, TEXT("ArmCompressionNode: SkeletalMeshComponent is null"));
	ensureMsgf(World, TEXT("ArmCompressionNode: World is null"));
	

	// === Safety: Bail on missing runtime context ===
	if (!Output.AnimInstanceProxy || !SkelMesh || !World)
	{
#if WITH_EDITOR
		if (bEnableDebug)
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Invalid context — Proxy, SkelMesh, or World is null."));
		}
#endif
		return;
	}

	if (!IsValidToEvaluate(nullptr, Output.AnimInstanceProxy->GetRequiredBones()))
	{
		if (ResolvedIndices.Num() < 3)
		{
#if WITH_EDITOR
			if (bEnableDebug)
			{
				UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Not enough valid ResolvedIndices."));
			}
#endif
			return;
		}

		for (const FCompactPoseBoneIndex& Index : ResolvedIndices)
		{
			if (!Index.IsValid())
			{
#if WITH_EDITOR
				if (bEnableDebug)
				{
					UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Invalid bone index detected in ResolvedIndices."));
				}
#endif
				return;
			}
		}
#if WITH_EDITOR
		if (bEnableDebug)
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Not valid to evaluate (missing BoneRefs)."));
		}
#endif
		return;
	}

	if (ResolvedIndices.Num() != BoneRefs.Num())
	{
#if WITH_EDITOR
		if (bEnableDebug)
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompression: ResolvedIndices mismatch — chain not initialized."));
		}
#endif
		return;
	}
	ensureMsgf(ResolvedIndices.Num() >= 3, TEXT("ArmCompressionNode: Expected at least 3 resolved indices but got %d"), ResolvedIndices.Num());

	for (int32 i = 0; i < ResolvedIndices.Num(); ++i)
	{
		ensureMsgf(ResolvedIndices[i].IsValid(), TEXT("ArmCompressionNode: ResolvedIndices[%d] is invalid (bone: %s)"),
			i, *BoneRefs[i].BoneName.ToString());
	}
	// === Resolve pose transforms ===
	TArray<FTransform> BoneTransformsCS;
	BoneTransformsCS.Reserve(ResolvedIndices.Num());

	
	const FCompactPose& Pose = Output.Pose.GetPose();
	for (int32 i = 0; i < ResolvedIndices.Num(); ++i)
	{
		ensureMsgf(Pose.IsValidIndex(ResolvedIndices[i]), TEXT("ArmCompressionNode: Pose index [%d] is invalid for bone [%s]"),
		i, *BoneRefs[i].BoneName.ToString());
		if (!Pose.IsValidIndex(ResolvedIndices[i]))
		{
#if WITH_EDITOR
			if (bEnableDebug)
			{
				UE_LOG(LogTemp, Warning, TEXT("ArmCompression: Invalid pose index [%d]"), i);
			}
#endif
			return;
		}

		BoneTransformsCS.Add(Output.Pose.GetComponentSpaceTransform(ResolvedIndices[i]));
	}
	
	ensureMsgf(BoneTransformsCS.Num() >= 3, TEXT("ArmCompressionNode: BoneTransformsCS contains insufficient transforms (count: %d)"), BoneTransformsCS.Num());

	const FTransform& RootCS     = BoneTransformsCS[0];
	const FTransform& JointCS    = BoneTransformsCS[1];
	const FTransform& EffectorCS = BoneTransformsCS.Last();

	const FVector OriginalEffectorPos = EffectorCS.GetLocation();

	// === Determine target effector location ===
	FVector PredictedEffector = OriginalEffectorPos;

#if WITH_EDITOR
	const bool bSimulatePreview =
		bAutoEnablePreviewVelocity && !bUseExternalVelocity && !IsRunningGame() && !IsRunningCommandlet();

	if (bSimulatePreview)
	{
		PredictedEffector = OriginalEffectorPos + PreviewOffsetDirection.GetSafeNormal() * PreviewSimulatedVelocity.Size();
	}
#endif

	if (bUseExternalVelocity && !ExternalPredictedTarget.IsZero())
	{
		PredictedEffector = ExternalPredictedTarget;
	}

	// === Smoothed effector target ===
	FVector EffectorOverride = EffectorTarget;

	if (bUseExternalEffectorReference)
	{
		EffectorOverride = ExternalEffectorReferenceTransform.GetLocation();
	}
	else
	{
		const FTransform RefTransform =
			EffectorSpace == BCS_ComponentSpace ? SkelMesh->GetComponentTransform() :
			EffectorSpace == BCS_ParentBoneSpace ? JointCS :
			EffectorSpace == BCS_BoneSpace ? EffectorCS :
			FTransform::Identity;

		EffectorOverride = RefTransform.TransformPosition(EffectorTarget);
	}

	if (bUseEffectorOverride)
	{
		SmoothedEffectorTarget = FMath::VInterpTo(
			SmoothedEffectorTarget,
			EffectorOverride,
			Output.AnimInstanceProxy->GetDeltaSeconds(),
			EffectorInterpSpeed
		);
		ensureMsgf(!SmoothedEffectorTarget.ContainsNaN(), TEXT("ArmCompressionNode: SmoothedEffectorTarget contains NaN!"));

	}
	else
	{
		SmoothedEffectorTarget = PredictedEffector;
	}

	// === Compute compression alpha ===
	float DynamicAlpha = 0.f;

	if (bUseExternalVelocity)
	{
		const float Speed = ExternalLinearVelocity.Size();
		if (Speed >= CompressionThreshold)
		{
			DynamicAlpha = FMath::Clamp((Speed - CompressionThreshold) / CompressionThreshold, 0.f, 1.f);
		}
	}

	DynamicAlpha = ApplySoftZoneOffset(DynamicAlpha);
	DynamicAlpha = ApplyCompressionResponseCurve(DynamicAlpha);

	float OverrideAlpha = CompressionBlendAlpha;

	if (bUseEffectorOverride && CompressionFalloffRadius > 0.f)
	{
		OverrideAlpha = ComputeFalloffScaledAlpha(SmoothedEffectorTarget, PredictedEffector, OverrideAlpha);
	}

	const float FinalAlpha = BlendCompressionSources(DynamicAlpha, OverrideAlpha);

	// === Clamp target movement ===
	const FVector FinalTarget = FMath::Lerp(PredictedEffector, SmoothedEffectorTarget, EffectorBlendAlpha);
	const FVector Delta = FinalTarget - OriginalEffectorPos;
	const float DeltaLen = Delta.Size();

	const FVector FinalTargetClamped = (MaxCompressionDistance > 0.f && DeltaLen > MaxCompressionDistance)
		? OriginalEffectorPos + Delta.GetSafeNormal() * MaxCompressionDistance
		: FinalTarget;

	const FVector BlendedEffector = FMath::Lerp(OriginalEffectorPos, FinalTargetClamped, FinalAlpha);

	// === Elbow hint with mirroring support ===
	const FVector ElbowHint = RootCS.TransformPosition(GetMirroredHintOffset());

	// === Solve IK ===
	SolveIK_2Bone(
		BoneTransformsCS[0],
		BoneTransformsCS[1],
		BoneTransformsCS[2],
		BlendedEffector,
		ElbowHint,
		bEnableStretching,
		1.f,
		MaxStretchScale,
		World,
		bEnableDebug
	);

	// === Output transforms ===
	OutBoneTransforms.Reset();              // clear any stale data
	OutBoneTransforms.Reserve(ResolvedIndices.Num());

	const int32 NumBones = Output.Pose.GetPose().GetNumBones();

	for (int32 i = 0; i < BoneTransformsCS.Num(); ++i)
	{
		const FCompactPoseBoneIndex CPIndex = ResolvedIndices[i];

		// -------- SAFETY CHECKS --------
		if (!CPIndex.IsValid() || CPIndex.GetInt() >= NumBones)
		{
#if WITH_EDITOR
			UE_LOG(LogTemp, Warning,
				TEXT("ArmCompressionNode: Skipping invalid bone index [%d] (PoseBones:%d) on pass %d"),
				CPIndex.GetInt(), NumBones, i);
#endif
			continue;   // never push an unsafe transform
		}

		OutBoneTransforms.Emplace(CPIndex, BoneTransformsCS[i]);
	}

	// Bail if we ended up with nothing after filtering
	if (OutBoneTransforms.Num() == 0)
	{
#if WITH_EDITOR
		UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: No valid bone transforms – evaluation skipped"));
#endif
		return;
	}


	// === Add Transforms ===
	
	OutBoneTransforms.Reserve(ResolvedIndices.Num());
	for (int32 i = 0; i < BoneTransformsCS.Num(); ++i)
	{
		OutBoneTransforms.Add(FBoneTransform(ResolvedIndices[i], BoneTransformsCS[i]));
	}

#if WITH_EDITOR
	if (bEnableDebug)
	{
		DrawDebugLine(World, OriginalEffectorPos, BlendedEffector, FColor::Green, false, 0.1f, 0, 2.f);
		DrawDebugSphere(World, BlendedEffector, 4.f, 12, FColor::Yellow, false, 0.1f, 0, 1.f);
		DrawDebugString(World, OriginalEffectorPos + FVector(0, 0, 10),
			FString::Printf(TEXT("Alpha: %.2f"), FinalAlpha), nullptr, FColor::White, 0.f, true);
	}
#endif
}

float FAnimNode_ArmCompressionResponse::ComputeFalloffScaledAlpha(
	const FVector& EffectorPos, const FVector& TargetPos, float RawAlpha) const
{
	if (CompressionFalloffRadius <= KINDA_SMALL_NUMBER)
	{
		return RawAlpha; // No falloff applied
	}

	const float Distance = FVector::Dist(EffectorPos, TargetPos);

	// Full compression if within 10% of falloff range
	if (Distance <= CompressionFalloffRadius * 0.1f)
	{
		return RawAlpha;
	}

	const float Falloff = 1.f - FMath::Clamp((Distance - (CompressionFalloffRadius * 0.1f)) / CompressionFalloffRadius, 0.f, 1.f);
	return RawAlpha * Falloff;
}



float FAnimNode_ArmCompressionResponse::ApplySoftZoneOffset(float RawAlpha) const
{
	if (SoftZoneOffset <= KINDA_SMALL_NUMBER)
	{
		// No deadzone applied
		return RawAlpha;
	}

	if (RawAlpha <= SoftZoneOffset)
	{
		// Inside deadzone → no compression
		return 0.f;
	}

	// Remap to [0,1] range after deadzone
	const float Adjusted = (RawAlpha - SoftZoneOffset) / (1.f - SoftZoneOffset);
	return FMath::Clamp(Adjusted, 0.f, 1.f);
}

float FAnimNode_ArmCompressionResponse::ApplyCompressionResponseCurve(float InputAlpha) const
{

	return InputAlpha;

	/*ddd
	UCurveFloat* CurveToUse = ResponseCurve;

	if (!CurveToUse)
	{
#if WITH_EDITOR
		if (GIsEditor && bEnableDebug)
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompression: ResponseCurve is null — using fallback curve."));
		}
#endif
		CurveToUse = GetDefaultResponseCurve();
	}

	const float CurveValue = CurveToUse->GetFloatValue(FMath::Clamp(InputAlpha, 0.f, 1.f));
	return FMath::Clamp(CurveValue, 0.f, 1.f);
	*/
}


float FAnimNode_ArmCompressionResponse::BlendCompressionSources(float DynamicAlpha, float OverrideAlpha) const
{
	const float Bias = FMath::Clamp(CompressionAlphaBlendBias, 0.f, 1.f);
	return FMath::Lerp(DynamicAlpha, OverrideAlpha, Bias);
}

FVector FAnimNode_ArmCompressionResponse::GetMirroredHintOffset() const
{
	return FVector(
		MirrorHintAxis.X != 0.f ? -JointTargetOffset.X : JointTargetOffset.X,
		MirrorHintAxis.Y != 0.f ? -JointTargetOffset.Y : JointTargetOffset.Y,
		MirrorHintAxis.Z != 0.f ? -JointTargetOffset.Z : JointTargetOffset.Z
	);
}

static void SolveIK_ArmChain(
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
	const FVector& WristLookAtAxis, // usually +X or -Y
	UWorld* World,
	bool bDrawDebug
)
{
	// Chain setup
	const FVector RootPos  = bUseClavicle ? Clavicle.GetLocation() : UpperArm.GetLocation();
	const FVector MidPos   = UpperArm.GetLocation();
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
	const float CosAngle = FMath::Clamp((A*A + C*C - B*B) / (2*A*C), -1.f, 1.f);
	const float Angle = FMath::Acos(CosAngle);
	const FVector DirectionNorm = DesiredDir / DesiredLen;

	// Elbow bend axis
	FVector ToHint = ElbowHint - RootPos;
	FVector RotationAxis = FVector::CrossProduct(DirectionNorm, ToHint).GetSafeNormal();

	if (RotationAxis.IsNearlyZero())
	{
		ensureMsgf(false, TEXT("ArmCompressionNode: Rotation axis is zero — fallback axis used"));
		RotationAxis = FVector::UpVector;
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



void FAnimNode_ArmCompressionResponse::InitializeIKChainFromEnum(
	const EOHSkeletalBone EffectorEnum,
	const FBoneContainer& RequiredBones)
{
	// Step 1: Walk up the hierarchy from effector → root
	TArray<EOHSkeletalBone> ChainEnums;
	BuildChainFromEffector(EffectorEnum, ChainEnums, /*MaxDepth=*/4);

	if (ChainEnums.Num() < 3 || ChainEnums.Num() > 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: Invalid chain length (%d) from %s."),
			ChainEnums.Num(),
			*UOHSkeletalPhysicsUtils::GetBoneNameFromEnum(EffectorEnum).ToString());
		return;
	}

	// Store chain enums
	BoneChainEnums = ChainEnums;

	// Step 2: Allocate and initialize BoneRefs
	BoneRefs.SetNum(ChainEnums.Num());

	for (int32 i = 0; i < ChainEnums.Num(); ++i)
	{
		const FName BoneName = UOHSkeletalPhysicsUtils::GetBoneNameFromEnum(ChainEnums[i]);
		BoneRefs[i].BoneName = BoneName;
		BoneRefs[i].Initialize(RequiredBones);
	}

	// Step 3: Manually construct ResolvedIndices using valid constructor
	ResolvedIndices.Empty();
	for (int32 i = 0; i < ChainEnums.Num(); ++i)
	{
		const FCompactPoseBoneIndex Index = BoneRefs[i].GetCompactPoseIndex(RequiredBones);
		ResolvedIndices.Add(Index);

		if (!Index.IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("ArmCompressionNode: Could not resolve CompactPose index for bone [%s] at position %d."),
				*BoneRefs[i].BoneName.ToString(), i);
		}
	}
}

void FAnimNode_ArmCompressionResponse::BuildChainFromEffector(
	const EOHSkeletalBone EffectorBone,
	TArray<EOHSkeletalBone>& OutChain,
	const int32 MaxDepth /* = 4 */)
{
	OutChain.Empty();

	EOHSkeletalBone Current = EffectorBone;
	int32 Depth = 0;

	while (Current != EOHSkeletalBone::None && Depth < MaxDepth)
	{
		OutChain.Insert(Current, 0); // Prepend to maintain root → tip order
		Current = UOHSkeletalPhysicsUtils::GetParentBone(Current);

		++Depth;
	}
}

/*UCurveFloat* FAnimNode_ArmCompressionResponse::GetDefaultResponseCurve()
{
	static UCurveFloat* DefaultCurve = nullptr;

	if (!DefaultCurve)
	{
		DefaultCurve = NewObject<UCurveFloat>();
		DefaultCurve->FloatCurve.AddKey(0.f, 0.f);
		DefaultCurve->FloatCurve.AddKey(1.f, 1.f);
		DefaultCurve->SetFlags(RF_Standalone); // Prevent GC during preview
	}

	return DefaultCurve;
}*/

#undef LOCTEXT_NAMESPACE
#endif
