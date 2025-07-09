#pragma once

#include "AnimNode_ArmCompressionResponse.generated.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"

// Declare log category here so we don’t touch other files
DECLARE_LOG_CATEGORY_EXTERN(LogOnlyHands, Log, All);

/**
 * Performs procedural arm compression via IK based on velocity, impact
 * feedback, or manual override. Minimal overrides: CacheBones,
 * IsValidToEvaluate, EvaluateSkeletalControl.
 */
USTRUCT(BlueprintInternalUseOnly)
struct ONLYHANDS_API FAnimNode_ArmCompressionResponse
    : public FAnimNode_SkeletalControlBase {
  GENERATED_BODY()

  /** Constructor: initialize all parameters to sensible defaults */
  FAnimNode_ArmCompressionResponse()
      : FAnimNode_SkeletalControlBase(), HandBone(), LowerArmBone(),
        UpperArmBone(), bUseEnumSetup(false),
        HandBoneEnum(EOHSkeletalBone::Hand_R), CompressionThreshold(120.f),
        MaxCompressionDistance(15.f), CompressionBlendAlpha(1.f),
        MinCompressionAlpha(0.f), bClampCompression(true),
        EffectorInterpSpeed(12.f), EffectorBlendAlpha(0.f),
        bUseEffectorOverride(false), EffectorTarget(FVector::ZeroVector)
        // referenced by SetEffectorInterpSpeed
        // :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}
        ,
        EffectorSpace(BCS_ComponentSpace)
        // referenced by SetEffectorBlendAlpha
        // :contentReference[oaicite:2]{index=2}:contentReference[oaicite:3]{index=3}
        ,
        PredictiveOffset(0.f)
        // referenced by SetEffectorPredictionOffset
        // :contentReference[oaicite:4]{index=4}:contentReference[oaicite:5]{index=5}
        ,
        bMirror(false)
        // referenced by SetMirror
        // :contentReference[oaicite:6]{index=6}:contentReference[oaicite:7]{index=7}
        ,
        MirrorHintAxis(FVector(0.f, 1.f, 0.f)), bEnableDebug(false) {}

  // ----- BoneEnum setup -----
  virtual void
  CacheBones_AnyThread(const FAnimationCacheBonesContext &Context) override;
  virtual bool IsValidToEvaluate(const USkeleton *Skeleton,
                                 const FBoneContainer &RequiredBones) override;
  virtual void EvaluateSkeletalControl_AnyThread(
      FComponentSpacePoseContext &Output,
      TArray<FBoneTransform> &OutBoneTransforms) override;

  // --- IK bones ---
  UPROPERTY(EditAnywhere, Category = "IK Bones")
  FBoneReference HandBone;

  UPROPERTY(EditAnywhere, Category = "IK Bones")
  FBoneReference LowerArmBone;

  UPROPERTY(EditAnywhere, Category = "IK Bones")
  FBoneReference UpperArmBone;

  // --- Auto setup via enum ---
  UPROPERTY(EditAnywhere, Category = "Auto Setup")
  bool bUseEnumSetup;

  UPROPERTY(EditAnywhere, Category = "Auto Setup",
            meta = (EditCondition = "bUseEnumSetup"))
  EOHSkeletalBone HandBoneEnum;

  // ----- Compression parameters -----
  UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin = "0.0"))
  float CompressionThreshold;

  UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin = "0.0"))
  float MaxCompressionDistance;

  UPROPERTY(EditAnywhere, Category = "Compression",
            meta = (ClampMin = "0.0", ClampMax = "1.0"))
  float CompressionBlendAlpha;

  UPROPERTY(EditAnywhere, Category = "Compression",
            meta = (ClampMin = "0.0", ClampMax = "1.0"))
  float MinCompressionAlpha;

  UPROPERTY(EditAnywhere, Category = "Compression")
  bool bClampCompression;

  /** Whether the two-bone chain may stretch past its rest length. */
  UPROPERTY(EditAnywhere, Category = "Compression")
  bool bAllowStretching = false;

  /** Maximum extra length (cm) the chain may stretch beyond its rest length. */
  UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin = "0.0"))
  float MaxStretchDistance = 10.f;

  /** Speed (units/sec) to interpolate into/out of the effector override. */
  UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin = "0.1"))
  float EffectorInterpSpeed = 5.f;

  UPROPERTY(EditAnywhere, Category = "Effector",
            meta = (ClampMin = "0.0", ClampMax = "1.0"))
  float EffectorBlendAlpha;

  float CurrentEffectorAlpha = 0.f; // internal

  // ----- Effector override -----
  UPROPERTY(EditAnywhere, Category = "Effector")
  bool bUseEffectorOverride;

  UPROPERTY(EditAnywhere, Category = "Effector",
            meta = (EditCondition = "bUseEffectorOverride"))
  FVector EffectorTarget;

  UPROPERTY(EditAnywhere, Category = "Effector",
            meta = (EditCondition = "bUseEffectorOverride"))
  TEnumAsByte<EBoneControlSpace> EffectorSpace;

  UPROPERTY(EditAnywhere, Category = "Effector")
  float PredictiveOffset;
  // ----- Hinting / mirroring -----
  UPROPERTY(EditAnywhere, Category = "Hint")
  bool bMirror;
  UPROPERTY(EditAnywhere, Category = "Hint")
  FVector MirrorHintAxis;

  UPROPERTY(EditAnywhere, Category = "Hint")
  FVector JointTargetOffset;

  // ----- Debug -----
  UPROPERTY(EditAnywhere, Category = "Debug")
  bool bEnableDebug;

protected:
  /** Build chain from enum if requested */
  void InitializeChainFromEnum(const FBoneContainer &RequiredBones);
};

#if 0
/**
 * Performs procedural arm compression via IK based on velocity, impact feedback, or manual override.
 * Supports dynamic anticipation, soft zone deadbands, curve-mapped response, and velocity blending.
 */
USTRUCT(BlueprintInternalUseOnly)
struct ONLYHANDS_API FAnimNode_ArmCompressionResponse : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Constructor: ensure the preview-warning text is always a valid, empty FText */
	FAnimNode_ArmCompressionResponse()
		: FAnimNode_SkeletalControlBase()
		, bUseEnumSetup(false)
		, HandBoneEnum(EOHSkeletalBone::Hand_R)
		, CompressionThreshold(120.f)
		, MaxCompressionDistance(15.f)
		, CompressionBlendAlpha(1.f)
		, MinCompressionAlpha(0.f)
		, bClampCompression(true)
		, bUseEffectorOverride(false)
		, EffectorTarget(FVector::ZeroVector)
		, EffectorSpace(BCS_ComponentSpace)
		, EffectorInterpSpeed(12.f)
		, EffectorBlendAlpha(0.f)
		, PredictiveOffset(0.f)
		, bUseExternalVelocity(false)
		, ExternalLinearVelocity(FVector::ZeroVector)
		, ExternalPredictedTarget(FVector::ZeroVector)
		, VelocityCorrectionSource(EOHMotionCorrectionSource::Pelvis)
		, bUseExternalEffectorReference(false)
		, ExternalEffectorReferenceTransform(FTransform::Identity)
		, EffectorReferenceBone()
		, EffectorReferenceSpace(BCS_ComponentSpace)
		, EffectorReferenceOffset(FTransform::Identity)
		, CompressionFalloffRadius(0.f)
		, SoftZoneOffset(0.f)
		, CompressionAlphaBlendBias(0.f)
		, JointTargetOffset(FVector(0.f,10.f,0.f))
		, bMirror(false)
		, MirrorHintAxis(FVector(0.f,1.f,0.f))
		, bEnableWristRotation(true)
		, WristRotationAxis(FVector(1.f,0.f,0.f))
		, bEnableStretching(false)
		, MaxStretchScale(1.25f)
		, bEnableDebug(false)
		, PreviewSimulatedVelocity(FVector(0.f,0.f,-120.f))
		, PreviewOffsetDirection(FVector(0.f,0.f,-1.f))
		, bAutoEnablePreviewVelocity(true)
		, SmoothedEffectorTarget(FVector::ZeroVector)
		, Index_Hand(INDEX_NONE)
		, Index_LowerArm(INDEX_NONE)
		, Index_UpperArm(INDEX_NONE)
		, bChainInitialized(false)
	{
#if WITH_EDITORONLY_DATA
		ClearValidationVisualWarnings();
#endif
	}

	// ----------------------------- FAnimNode Overrides ----------------------------- //

	/** Validate bone references at runtime (non-editor path) */
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

	/** Main runtime IK evaluation */
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;

#if WITH_EDITORONLY_DATA
	/** Wipe stale warnings & reset chain before any cache pass */
	virtual void CacheBones_AnyThread(const FAnimationCacheBonesContext& Context) override;

	/** Build & validate the bone chain for editor-preview */
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
#endif

	// ----------------------------- Validation Helpers ----------------------------- //

	/** Non-virtual helper to test RequiredBones validity without editor logging */
	bool IsValidToEvaluate_Internal(const FBoneContainer& RequiredBones) const;

	float ApplyCompressionResponseCurve(float InputAlpha) const;

	// ----------------------------- IK BoneEnum Chain ----------------------------- //

	/** End effector (hand) */
	UPROPERTY(EditAnywhere, Category = "IK Bones")
	FBoneReference HandBone;

	/** Middle joint (forearm) */
	UPROPERTY(EditAnywhere, Category = "IK Bones")
	FBoneReference LowerArmBone;

	/** Root joint (upper arm) */
	UPROPERTY(EditAnywhere, Category = "IK Bones")
	FBoneReference UpperArmBone;

	/** If true, builds the chain automatically from the EOHSkeletalBone enum */
	UPROPERTY(EditAnywhere, Category = "Auto Setup", meta = (InlineEditConditionToggle))
	bool bUseEnumSetup;

	/** Enum value defining the hand bone for chain construction */
	UPROPERTY(EditAnywhere, Category = "Auto Setup", meta = (EditCondition = "bUseEnumSetup"))
	EOHSkeletalBone HandBoneEnum;

	// ----------------------------- Compression Settings ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin="1.0"))
	float CompressionThreshold;

	UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin="0.0"))
	float MaxCompressionDistance;

	UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin="0.0", ClampMax="1.0"))
	float CompressionBlendAlpha;

	UPROPERTY(EditAnywhere, Category = "Compression", meta = (ClampMin="0.0", ClampMax="1.0"))
	float MinCompressionAlpha;

	UPROPERTY(EditAnywhere, Category = "Compression")
	bool bClampCompression;

	// ----------------------------- Effector Control ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "Effector")
	bool bUseEffectorOverride;

	UPROPERTY(EditAnywhere, Category = "Effector", meta = (EditCondition="bUseEffectorOverride"))
	FVector EffectorTarget;

	UPROPERTY(EditAnywhere, Category = "Effector", meta = (EditCondition="bUseEffectorOverride"))
	TEnumAsByte<EBoneControlSpace> EffectorSpace;

	UPROPERTY(EditAnywhere, Category = "Effector")
	float EffectorInterpSpeed;

	UPROPERTY(EditAnywhere, Category = "Effector", meta = (ClampMin="0.0", ClampMax="1.0"))
	float EffectorBlendAlpha;

	UPROPERTY(EditAnywhere, Category = "Effector")
	float PredictiveOffset;

	// ----------------------------- External Input ----------------------------- //

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	bool bUseExternalVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	FVector ExternalLinearVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	FVector ExternalPredictedTarget;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	EOHMotionCorrectionSource VelocityCorrectionSource;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	bool bUseExternalEffectorReference;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	FTransform ExternalEffectorReferenceTransform;

	UPROPERTY(EditAnywhere, Category = "Compression|External")
	FBoneReference EffectorReferenceBone;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	TEnumAsByte<EBoneControlSpace> EffectorReferenceSpace;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Compression|External")
	FTransform EffectorReferenceOffset;

	// ----------------------------- Advanced Settings ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "Compression|Advanced")
	float CompressionFalloffRadius;

	UPROPERTY(EditAnywhere, Category = "Compression|Advanced", meta = (ClampMin="0.0", ClampMax="1.0"))
	float SoftZoneOffset;

	UPROPERTY(EditAnywhere, Category = "Compression|Advanced", meta = (ClampMin="0.0", ClampMax="1.0"))
	float CompressionAlphaBlendBias;

	// ----------------------------- Hinting ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "Hint")
	FVector JointTargetOffset;

	UPROPERTY(EditAnywhere, Category = "Hint")
	bool bMirror;

	UPROPERTY(EditAnywhere, Category = "Hint")
	FVector MirrorHintAxis;

	// ----------------------------- Wrist & Stretch ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "IK|Wrist")
	bool bEnableWristRotation;

	UPROPERTY(EditAnywhere, Category = "IK|Wrist")
	FVector WristRotationAxis;

	UPROPERTY(EditAnywhere, Category = "Stretch")
	bool bEnableStretching;

	UPROPERTY(EditAnywhere, Category = "Stretch", meta = (ClampMin="1.0", ClampMax="2.0"))
	float MaxStretchScale;

	// ----------------------------- Debug & Preview ----------------------------- //

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bEnableDebug;

	UPROPERTY(EditAnywhere, Category = "Preview", meta = (ClampMin="0.0"))
	FVector PreviewSimulatedVelocity;

	UPROPERTY(EditAnywhere, Category = "Preview", meta = (ClampMin="0.0"))
	FVector PreviewOffsetDirection;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bAutoEnablePreviewVelocity;

protected:
	// Smoothed override target
	FVector SmoothedEffectorTarget;

	// Cached pose indices
	FCompactPoseBoneIndex Index_Hand;
	FCompactPoseBoneIndex Index_LowerArm;
	FCompactPoseBoneIndex Index_UpperArm;

	// Internal chain data
	TArray<EOHSkeletalBone>      BoneChainEnums;
	TArray<FBoneReference>       BoneRefs;
	TArray<FCompactPoseBoneIndex> ResolvedIndices;
	bool bChainInitialized;

	// Helpers for enum-driven chain
	static FName GetBoneNameFromEnum(EOHSkeletalBone BoneEnum);
	static EOHSkeletalBone GetParentBoneEnum(EOHSkeletalBone BoneEnum);
	static void BuildChainFromEffector(EOHSkeletalBone EffectorBone, TArray<EOHSkeletalBone>& OutChain, int32 MaxDepth = 4);
	void InitializeIKChainFromEnum(EOHSkeletalBone EffectorEnum, const FBoneContainer& RequiredBones);

	// Additional runtime helpers
	float ComputeFalloffScaledAlpha(const FVector& EffectorPos, const FVector& TargetPos, float RawAlpha) const;
	float ApplySoftZoneOffset(float RawAlpha) const;
	float BlendCompressionSources(float DynamicAlpha, float OverrideAlpha) const;
	FVector GetMirroredHintOffset() const;
};

#endif
