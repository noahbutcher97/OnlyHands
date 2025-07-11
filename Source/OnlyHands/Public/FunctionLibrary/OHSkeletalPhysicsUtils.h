// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "OHPhysicsStructs.h"
#include "Animation/AnimTypes.h"
#include "Components/SkeletalMeshComponent.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "CollisionShape.h"
#include "Engine/EngineTypes.h"
#include "OHSkeletalPhysicsUtils.generated.h"

#pragma region Macros_
// BoneEnum Reference Helper Macros
// ----------------------------
/** Shorthand for creating a bone reference map from any supported context (AnimInstance, BoneContainer, etc.) */
#define OH_CREATE_BONE_MAP(Context) UOHSkeletalPhysicsUtils::CreateBoneReferenceMap(Context)
/** Shorthand for creating a bone reference map from any supported context (AnimInstance, BoneContainer, etc.) */
#define OH_CREATE_BONE_MAP_SAFE(Context) UOHSkeletalPhysicsUtils::CreateBoneReferenceMapSafe(Context)
/** Safely retrieve a compact pose index from a bone reference map */
#define OH_GET_POSE_INDEX(Map, Bone) (Map.GetPoseIndex(Bone))
/** Safely retrieve a compact pose index from a bone reference map */
#define OH_GET_POSE_INDEX_SAFE(Map, Bone) (Map.HasValidReference(Bone) ? Map.GetPoseIndex(Bone) : INDEX_NONE)
/** Safely retrieve a bone reference struct */
#define OH_GET_BONE_REF(Map, Bone) (Map.GetReference(Bone))
/** Safely retrieve a bone reference struct */
#define OH_GET_BONE_REF_SAFE(Map, Bone) (Map.HasValidReference(Bone) ? Map.GetReference(Bone) : FBoneReference())
/** Check if a given bone has a valid reference in the map */
#define OH_HAS_REF(Map, Bone) (Map.HasValidReference(Bone))
/** Check if a given bone has a valid reference in the map */
#define OH_HAS_REF_SAFE(Map, Bone) (Map.HasValidReference(Bone))
/** Guard clause macro for checking map validity */
#define OH_ENSURE_BONE_MAP_VALID(Map, ReturnValue)                                                                     \
    if (!Map.IsInitialized()) {                                                                                        \
        UE_LOG(LogTemp, Warning, TEXT("BoneEnum map is not initialized."));                                            \
        return ReturnValue;                                                                                            \
        \                                                                                                              \
    }
/** Guard clause macro for checking map validity */
#define OH_ENSURE_BONE_MAP_VALID_NO_RETURN(Map)                                                                        \
    if (!Map.IsInitialized()) {                                                                                        \
        UE_LOG(LogTemp, Warning, TEXT("BoneEnum map is not initialized."));                                            \
        return;                                                                                                        \
        \                                                                                                              \
    }
/** Debug print of bone map if enabled */
#define OH_PRINT_BONE_MAP(Map) (Map.PrintDebugSummary())
/** Safely retrieve a bone reference struct */
#define OH_GET_BONE_REF(Map, Bone) (Map.GetReference(Bone))
/** Safely retrieve a bone reference struct */
#define OH_GET_BONE_REF_SAFE(Map, Bone) (Map.HasValidReference(Bone) ? Map.GetReference(Bone) : FBoneReference())
/** Safely retrieve a bone reference struct */
#define OH_GET_BONE_REF_SAFE_NO_RETURN(Map, Bone)                                                                      \
    (Map.HasValidReference(Bone) ? Map.GetReference(Bone) : FBoneReference())
// --------------------
// Debugging & Logging
// --------------------
/** Verbose conditional log macro */
#define OH_LOG_VERBOSE(Format, ...) UE_LOG(LogTemp, Verbose, TEXT(Format), ##__VA_ARGS__)
/** Lightweight warning for validation */
#define OH_LOG_WARN_IF(Condition, Format, ...)                                                                         \
    if (Condition) {                                                                                                   \
        UE_LOG(LogTemp, Warning, TEXT(Format), ##__VA_ARGS__);                                                         \
    }
// --------------------
// Enum safety
// --------------------
/** Enum string helper */
#define OH_ENUM_STRING(EnumVal) (*UEnum::GetValueAsString(EnumVal))
/** Enum string helper */
#define OH_ENUM_STRING_SAFE(EnumVal) (EnumVal ? (*UEnum::GetValueAsString(EnumVal)) : TEXT("Unknown"))

#pragma endregion

/**
 * Specialized function library for skeletal physics and advanced bone analysis in OnlyHands
 *
 * This library focuses exclusively on:
 * - Skeletal physics operations (influence chains, force propagation)
 * - Complex bone relationships and hierarchies
 * - Group-based bone queries and enumeration (BodyParts, fingers, chains)
 * - Structural analysis of the skeleton
 * - Multi-bone operations and queries
 *
 * For simpler utility functions or single-bone classification, use OHHelperUtils or the
 * FORCE INLINE functions in EOHPhysicsEnums.h instead.
 */
UCLASS(meta = (BlueprintThreadSafe))
class ONLYHANDS_API UOHSkeletalPhysicsUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

  public:
#pragma region BoneResolution

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static FName ResolveBoneNameFromSkeletalBone(EOHSkeletalBone Bone, const USkeletalMeshComponent* SkelComp);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static FOHResolvedBoneData ResolveResolvedDataFromSkeletalBone(EOHSkeletalBone Bone,
                                                                   const USkeletalMeshComponent* SkelComp);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static FName ResolveBoneNameFromSkeletalBone_Static(EOHSkeletalBone Bone);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static FOHResolvedBoneData ResolveResolvedDataFromSkeletalBone_Static(EOHSkeletalBone Bone);

#pragma endregion

#pragma region Enums

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHBodyZone GetBodyZoneFromBone(EOHSkeletalBone Bone);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHBodyPart GetBodyPartFromBone(EOHSkeletalBone Bone);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHFunctionalBoneGroup GetFunctionalGroupFromBone(EOHSkeletalBone Bone);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static TArray<FName> GetPrimaryBoneNamesFromBodyPart(EOHBodyPart BodyPart);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHSkeletalBone ResolveSkeletalBoneFromNameSmart(const FName& Input, const USkeletalMeshComponent* SkelComp,
                                                            float ScoreThreshold = 0.65f);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHSkeletalBone GetRootBoneInBodyPartFromMesh(const USkeletalMeshComponent* SkelComp, EOHBodyPart BodyPart);

    UFUNCTION(BlueprintPure, Category = "OH|Skeletal")
    static EOHSkeletalBone GetEndBoneInBodyPartFromMesh(const USkeletalMeshComponent* SkelComp, EOHBodyPart BodyPart);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|BoneEnum Utility")
    static bool IsBallBone(EOHSkeletalBone Bone) {
        return Bone == EOHSkeletalBone::Ball_L || Bone == EOHSkeletalBone::Ball_R;
    }

#pragma endregion

#pragma region Bone Queries

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal Physics|Structure")
    static TArray<FName> GetBoneChainBetweenByName(const USkeletalMeshComponent* Mesh, const FName& ParentBoneName,
                                                   const FName& ChildBoneName);

    /**
     * Returns the full bone chain (as bone names) from the specified skeletal-enum up to the root of the bone
     * hierarchy. E.g. passing Hand_R will give { Hand_R, LowerArm_R, UpperArm_R, Clavicle_R, …, Pelvis }.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static TArray<FName> GetBonesInChain(EOHSkeletalBone Bone);

    /**
     * Builds a bone-name chain from EffectorEnum up toward root,
     * dropping any entries that don’t resolve in the given BoneContainer.
     */
    // C++-only helper — not blueprint-exposed
    static void BuildValidatedBoneChain(EOHSkeletalBone EffectorEnum, const FBoneContainer& Bones,
                                        TArray<FName>& OutValidBoneNames);

#pragma endregion

    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalUtils")
    static TArray<FName> GetBoneChainReverseByName(const USkeletalMeshComponent* Mesh, const FName& StartingBone);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
    static void DrawDebugBoneChainByName(const USkeletalMeshComponent* Mesh, const FName& ParentBoneName,
                                         const FName& ChildBoneName, FColor Color = FColor::Cyan,
                                         float Duration = 2.0f);

    /** Returns all bones that are hierarchically below the given root bone (inclusive). */
    UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Structure")
    static TArray<EOHSkeletalBone> GetBoneChainBelow(EOHSkeletalBone RootBone);

    /**
     * Gets the influence chain for force propagation through connected bones
     * @param Bone - The origin bone where force is applied
     * @return Ordered an array of bones that should be affected, starting with the origin bone
     */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Forces")
    static TArray<EOHSkeletalBone> GetBoneInfluenceChain(EOHSkeletalBone Bone);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static EOHSkeletalBone GetParentBone(EOHSkeletalBone TargetBone);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|BoneEnum Utility")
    static TArray<EOHSkeletalBone> GetBonesInFunctionalBoneGroup(EOHFunctionalBoneGroup Group);
    /**
     * Returns all direct children of the given skeletal bone enum.
     * This is based on the hardcoded OnlyHands anatomical hierarchy.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static TArray<EOHSkeletalBone> GetChildBones(EOHSkeletalBone ParentBone);

    /**
     * Returns all descendant bones below the specified root bone, recursively.
     * This includes all direct and indirect children.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static TArray<EOHSkeletalBone> GetDescendantBones(EOHSkeletalBone RootBone);

    /**
     * Returns the first direct child of the specified bone.
     * Returns EOHSkeletalBone::None if there are no children.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static EOHSkeletalBone GetDirectChildBone(EOHSkeletalBone ParentBone);

    /**
     * Returns all child bones of the specified root bone up to a given depth.
     * Depth of 1 returns only direct children, depth of 2 includes grandchildren, etc.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure",
              meta = (DisplayName = "Get Child Bones By Depth",
                      ToolTip = "Returns all children up to a specified depth below a given bone."))
    static TArray<EOHSkeletalBone> GetChildBonesByDepth(EOHSkeletalBone RootBone, int32 MaxDepth);

    /**
     * Returns the depth between a parent and target bone in the skeletal hierarchy.
     * If the parent is not an ancestor of the target, returns -1.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static int32 GetBoneDepthRelativeTo(EOHSkeletalBone ParentBone, EOHSkeletalBone TargetBone);

    /**
     * Returns the full bone lineage from the target bone up to the specified ancestor or root.
     * Includes the target bone itself. Stops at ParentBone or when reaching the root.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static TArray<EOHSkeletalBone> GetBoneLineage(EOHSkeletalBone TargetBone, EOHSkeletalBone StopAtBone);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure",
              meta = (DisplayName = "Get BoneEnum Lineage (Full)"))
    static TArray<EOHSkeletalBone> GetBoneLineageToRoot(EOHSkeletalBone TargetBone);
    /**
     * Compares the hierarchy depth of two bones.
     * Returns:
     *   -1 if A is deeper than B
     *    0 if same depth
     *    1 if B is deeper than A
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Structure")
    static int32 CompareBoneDepth(EOHSkeletalBone A, EOHSkeletalBone B);

    //------------------------------- Physics Solvers --------------------------------- //

    /**
     * Solves a two-bone IK chain to position the end effector at the target location.
     *
     * @param OutUpper          Upper bone transform (shoulder/hip) - modified in-place
     * @param OutLower          Lower bone transform (elbow/knee) - modified in-place
     * @param OutEnd            End effector transform (wrist/ankle) - modified in-place
     * @param TargetPosition    Target world position for the end effector
     * @param JointTarget       Hint position for the middle joint (elbow/knee)
     * @param bAllowStretching  Whether to allow chain stretching to reach targets beyond max distance
     * @param StartStretchRatio Ratio of max reach at which stretching begins (0-1)
     * @param MaxStretchScale   Maximum stretch multiplier
     * @param WorldDebugContext World context for debug visualization (optional)
     * @param bDrawDebug        Whether to draw debug information
     */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|IK")
    static void SolveIK_TwoBone(UPARAM(ref) FTransform& OutUpper, UPARAM(ref) FTransform& OutLower,
                                UPARAM(ref) FTransform& OutEnd, const FVector& TargetPosition,
                                const FVector& JointTarget, bool bAllowStretching = false,
                                float StartStretchRatio = 1.0f, float MaxStretchScale = 1.25f,
                                UWorld* WorldDebugContext = nullptr, bool bDrawDebug = false);

    /**
     * Solves an arm IK chain with optional clavicle retraction.
     *
     * @param Clavicle           Clavicle/shoulder transform - modified in-place
     * @param UpperArm           Upper arm transform - modified in-place
     * @param LowerArm           Lower arm/forearm transform - modified in-place
     * @param Hand               Hand/wrist transform - modified in-place
     * @param IKTarget           Target world position for the hand
     * @param ElbowHint          Hint position for the elbow joint
     * @param bUseClavicle       Whether to adjust the clavicle/shoulder
     * @param bAllowStretching   Whether to allow chain stretching
     * @param MaxStretchScale    Maximum stretch multiplier
     * @param bEnableWristRotation Whether to adjust wrist rotation to target
     * @param WristLookAtAxis    Local axis of the wrist that should align with target
     * @param World              World context for debug visualization
     * @param bDrawDebug         Whether to draw debug information
     */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|IK")
    static void SolveIK_ArmChain(UPARAM(ref) FTransform& Clavicle, UPARAM(ref) FTransform& UpperArm,
                                 UPARAM(ref) FTransform& LowerArm, UPARAM(ref) FTransform& Hand,
                                 const FVector& IKTarget, const FVector& ElbowHint, bool bUseClavicle = false,
                                 bool bAllowStretching = false, float MaxStretchScale = 1.25f,
                                 bool bEnableWristRotation = true,
                                 const FVector& WristLookAtAxis = FVector(1.0f, 0.0f, 0.0f), UWorld* World = nullptr,
                                 bool bDrawDebug = false);

    // ------------------------------- Solver Utility Functions -------------------------------- //
    /**
     * Utility function to compute fallback elbow bend axis when hint direction is unreliable.
     *
     * @param Root  Root bone transform (shoulder/hip)
     * @param Mid   Middle joint transform (elbow/knee)
     * @return      A safe bend axis perpendicular to the bone direction
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IK|Utility")
    static FVector ComputeFallbackBendAxis(const FTransform& Root, const FTransform& Mid);

    /**
     * Validates bone transforms for a standard IK chain to ensure they meet engine requirements.
     * Particularly useful before submitting transforms to FCSPose::LocalBlendCSBoneTransforms
     *
     * @param OutBoneTransforms Bone transforms array to validate/modify
     * @param bSortByIndex      Whether to sort transforms by bone index
     * @param bLogWarnings      Whether to log warnings for invalid transforms
     * @return                  Number of invalid transforms removed (0 if all valid)
     */
    int32 static ValidateAndSortBoneTransforms(TArray<FBoneTransform>& OutBoneTransforms, bool bSortByIndex = true,
                                               bool bLogWarnings = false);

    /**
     * Mirrors a point around a given pivot, flipping only the components
     * along axes where MirrorAxis has a significant (non-zero) component.
     *
     * @param Point       The world-space point to mirror.
     * @param Pivot       The origin about which to mirror.
     * @param MirrorAxis  A unit vector whose X/Y/Z components indicate which axes to flip.
     * @return            The mirrored point.
     */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|IK")
    static FVector MirrorPointAcrossAxes(const FVector& Point, const FVector& Pivot, const FVector& MirrorAxis);
    /**
     * Compute a velocity-based target: HandPosition minus the velocity direction scaled by Threshold.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static FVector ComputeVelocityBasedTarget(FVector HandPosition, FVector Velocity, float CompressionThreshold);

    /**
     * Linearly blend between two targets A and B by Alpha (clamped 0–1).
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static FVector BlendTargets(FVector A, FVector B, float Alpha);

    /**
     * Apply a predictive offset along Velocity: BaseTarget + Velocity * PredictiveOffset.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static FVector ApplyPredictiveOffset(FVector BaseTarget, FVector Velocity, float PredictiveOffset);

    /**
     * Mirror a vector across the specified axes. Any non‐zero component in MirrorAxis flips that component.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static FVector MirrorVectorAcrossAxis(FVector Vector, FVector MirrorAxis);

    /**
     * If Curve is valid, returns Curve->GetFloatValue(InputAlpha); otherwise returns DefaultValue.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static float EvaluateCurveAtAlpha(const UCurveFloat* Curve, float InputAlpha, float DefaultValue = 1.0f);

    /**
     * Clamp Alpha into [MinAlpha, MaxAlpha].
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static float ClampAlpha(float Alpha, float MinAlpha, float MaxAlpha);

    /**
     * Computes a simple falloff alpha: 1 − (Distance / FalloffRadius), clamped to [0,1].
     * If FalloffRadius ≤ 0, returns 1.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static float ComputeFalloffAlpha(float Distance, float FalloffRadius);

    /**
     * Applies a “soft zone” offset: remaps Alpha such that below SoftZoneOffset → 0, above 1 → 1, linearly in between.
     * If SoftZoneOffset ≤ 0, returns Alpha unchanged.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|IKHelpers")
    static float ComputeSoftZoneAlpha(float Alpha, float SoftZoneOffset);
    /**
     * Compute the total rest‐pose length of a bone chain by summing distances between successive bones.
     * @param SkelComp  The skeletal mesh component
     * @param BoneNames Ordered list of bone names (e.g. {"upperarm_r","lowerarm_r","hand_r"})
     * @return          Sum of segment lengths in component space
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float ComputeBoneChainLength(USkeletalMeshComponent* SkelComp, const TArray<FName>& BoneNames);

    /**
     * Sample world‐space positions for a bone chain.
     * @param SkelComp   The skeletal mesh component
     * @param BoneNames  Ordered list of bone names
     * @param OutPositions Filled with one FVector per bone
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static void GetBoneChainWorldPositions(USkeletalMeshComponent* SkelComp, TArray<FName> BoneNames,
                                           TArray<FVector>& OutPositions);

    /**
     * Find the closest point on a polyline (chain of points) to a given world point.
     * @param ChainPoints  World positions of the chain vertices
     * @param Point        World point to project
     * @return             Closest world‐space point on any segment
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector GetClosestPointOnBoneChain(TArray<FVector> ChainPoints, FVector Point);

    /**
     * Perform a capsule sweep along each bone‐segment in the chain to collect collision hits.
     * @param World       The world context (for sweeps)
     * @param SkelComp    The skeletal mesh component
     * @param BoneNames   Ordered list of bone names defining the chain
     * @param Radius      Radius of the capsule
     * @param OutHits     Array to accumulate hit results
     * @return            True if any hit was found
     */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|SkeletalPhysics")
    static bool SweepCapsuleAlongBoneChain(UWorld* World, USkeletalMeshComponent* SkelComp, TArray<FName> BoneNames,
                                           float Radius, TArray<FHitResult>& OutHits);

    /**
     * Approximate the center of mass of a bone chain by averaging each bone’s body‐instance COM.
     * @param SkelComp   The skeletal mesh component
     * @param BoneNames  List of bone names to include
     * @return           World‐space COM of the chain
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputeBoneChainCenterOfMass(USkeletalMeshComponent* SkelComp, TArray<FName> BoneNames);

    /**
     * Predict future positions for a chain given current positions and velocities via simple Euler integration.
     * @param CurrentPositions   World‐space positions at time T
     * @param Velocities         World‐space velocities (units/sec)
     * @param DeltaTime          Time step (sec)
     * @param OutPredicted       World‐space positions at T+DeltaTime
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static void PredictBoneChainPositions(TArray<FVector> CurrentPositions, TArray<FVector> Velocities, float DeltaTime,
                                          TArray<FVector>& OutPredicted);

    /**
     * Decompose a rotation into swing (cone) and twist (around TwistAxis).
     * @param Rotation   Input world or local rotation
     * @param TwistAxis  Axis (in same space) to extract twist about (unit vector)
     * @param OutSwing   Rotation with twist removed
     * @param OutTwist   Pure twist rotation about TwistAxis
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static void DecomposeSwingTwist(FQuat Rotation, FVector TwistAxis, FQuat& OutSwing, FQuat& OutTwist);

    /**
     * Compute linear velocity of a bone by sampling its world positions over a timestep.
     * @param SkelComp   The skeletal mesh component
     * @param BoneName   Name of the bone
     * @param DeltaTime  Time difference between samples
     * @return           Velocity = (CurrentPos - PrevPos) / DeltaTime
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputeBoneVelocity(USkeletalMeshComponent* SkelComp, FName BoneName, float DeltaTime);

    /**
     * Compute angular velocity (axis‐angle) between two rotations.
     * @param PrevRotation  Rotation at time T
     * @param CurrRotation  Rotation at time T+Δ
     * @param DeltaTime     Time difference Δ
     * @return              Angular velocity vector in radians/sec
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputeAngularVelocity(FQuat PrevRotation, FQuat CurrRotation, float DeltaTime);

    /**
     * Ray‐casts from a bone along its current velocity to predict
     * the earliest time (in seconds) until collision.
     * @param SkelComp    The skeletal mesh component
     * @param BoneName    The bone whose collision to predict
     * @param Velocity    Current world‐space velocity of the bone
     * @param MaxTime     How far into the future to test (sec)
     * @param Channel     Collision channel to use
     * @param OutHit      Hit result if a collision occurs
     * @return            Time to collision, or MaxTime if none
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float PredictBoneTimeToCollision(USkeletalMeshComponent* SkelComp, FName BoneName, FVector Velocity,
                                            float MaxTime, ECollisionChannel Channel, FHitResult& OutHit);

    /**
     * Given a predicted collision hit, compute a per‐bone response impulse
     * that points away from the surface normal, scaled by ImpactStrength.
     * @param Hit             The collision hit
     * @param ImpactStrength  Scale of the impulse
     * @return                World‐space impulse vector
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputeReactiveImpulseFromHit(const FHitResult& Hit, float ImpactStrength);

    /**
     * Samples a bone chain’s swept volume by casting spheres at each step
     * and returns true if any overlap is found.
     * @param SkelComp     The skeletal mesh component
     * @param BoneNames    Ordered list of chain bones
     * @param SphereRadius Radius of each test sphere
     * @param Channel      Collision channel
     * @return             True if any bone‐sphere overlaps a body
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static bool IsBoneChainIntersecting(USkeletalMeshComponent* SkelComp, TArray<FName> BoneNames, float SphereRadius,
                                        ECollisionChannel Channel);

    /**
     * Projects a target point onto the closest capsule along a bone segment,
     * useful for sliding‐against‐bone collisions.
     * @param Start      World start of bone segment
     * @param End        World end of bone segment
     * @param Radius     Capsule radius
     * @param Point      World point to project
     * @return           Closest world‐space point on the capsule surface
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ProjectPointOntoBoneCapsule(FVector Start, FVector End, float Radius, FVector Point);

    /**
     * Computes a blend weight in [0,1] for a bone based on its proximity
     * to a collision point, using linear fall‐off over DistanceThreshold.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float ComputeCollisionBlendWeight(FVector BonePos, FVector CollisionPos, float DistanceThreshold);

    /**
     * Given a bone chain’s world positions and a moving sphere (center+radius+velocity),
     * predicts which segment it will first collide with and at what time.
     * @return tuple (HitTime, SegmentIndex, HitPoint). If no hit, HitTime=MaxTime, SegmentIndex=-1.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static void PredictChainCollisionSegment(const TArray<FVector>& ChainPositions, FVector SphereCenter, float Radius,
                                             FVector Velocity, float MaxTime, float& OutHitTime, int32& OutSegmentIndex,
                                             FVector& OutHitPoint);

    /**
     * Computes penetration depth of a point inside a capsule segment—
     * useful to drive spring forces in procedural response.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float ComputeCapsulePenetrationDepth(FVector Point, FVector CapsuleStart, FVector CapsuleEnd, float Radius);

    /**
     * Generates a corrective delta‐translation for a bone to resolve penetration,
     * sliding it out along the capsule’s inward normal.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputePenetrationCorrection(FVector Point, FVector CapsuleStart, FVector CapsuleEnd, float Radius);

    /**
     * Given a chain of predicted collision hits, compute a composite collision normal
     * by summing and normalizing all hit normals.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComposeCollisionNormal(TArray<FHitResult> Hits);

    /**
     * Maps a collision normal and incoming bone velocity to an outgoing bounce velocity,
     * clamped by RestitutionCoefficient.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static FVector ComputeBounceVelocity(FVector VelocityIn, FVector Normal, float RestitutionCoefficient);

    /**
     * Computes chain stiffness weights per bone based on mass and chain length,
     * to drive procedural spring strengths.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static void ComputeChainStiffnessMap(UPhysicsAsset* PhysAsset, TArray<FName> BoneNames, float BaseStiffness,
                                         TMap<FName, float>& OutStiffnessMap);

    /**
     * Predicts the time‐of‐impact along a bone chain when hit by a moving capsule,
     * returning the earliest impact time or MaxTime if none.
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float PredictCapsuleChainTimeToImpact(UWorld* World, USkeletalMeshComponent* SkelComp,
                                                 TArray<FName> BoneNames, FVector CapsuleVelocity, float CapsuleRadius,
                                                 float MaxTime);

    /**
     * For a reactive effect, computes an exponential decay of collision response over Time,
     * using DecayRate (e.g. spring/damper fall‐off).
     */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|SkeletalPhysics")
    static float ComputeCollisionDecay(float Time, float DecayRate);

    // IK Arm Metrics and Debug

    /** Computes arm chain length using skeletal space */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK")
    static float GetArmChainLength(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm, FName Hand);

    /** Computes current effector direction (unit vector) */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK")
    static FVector GetArmEffectorDirection(USkeletalMeshComponent* Mesh, FName UpperArm, FVector Effector);

    /** Computes normalized extension [0..1] between min/max */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK")
    static float GetArmCompressionAlpha(float Distance, float Min, float Max);

    /** Draws debug line and lengths for IK */
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Debug")
    static void DebugDrawArmIK(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm, FName Hand,
                               FVector EffectorTarget, float MinLen, float MaxLen, float CurrentAlpha,
                               FLinearColor Color = FLinearColor::Yellow, float Duration = 1.0f);

    // Predict IK target from velocity or local aim offset
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Targeting")
    static FVector PredictEffectorTarget(USkeletalMeshComponent* Mesh, FName StartBone, FVector Velocity,
                                         float Distance = 60.f);

    // Determine likely compression change based on arm extension velocity
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|Prediction")
    static float PredictCompressionTrend(USkeletalMeshComponent* Mesh, FName UpperArm, FName LowerArm, FName Hand,
                                         float DeltaTime);

    // Simple collision check around a bone
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Collision")
    static bool IsBoneOverlapping(USkeletalMeshComponent* Mesh, FName BoneName, float Radius = 5.f);

    // Forward prediction based on velocity
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Collision")
    static bool PredictBoneCollision(USkeletalMeshComponent* Mesh, FName BoneName, FVector Velocity, float Distance,
                                     FHitResult& OutHit);

    // Trace from hand to world (punch prediction)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Collision")
    static bool TracePunchCollision(USkeletalMeshComponent* Mesh, FName HandBone, FVector Direction, float Length,
                                    FHitResult& OutHit);

    // Drive IK node inputs from current pose
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Helpers")
    static void SyncIKFromArmPose(UObject* AnimInstance, FName UpperArm, FName LowerArm, FName Hand, float MinLength,
                                  float MaxLength);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Tracking")
    static void TrackImpactCamera(UObject* WorldContext, FVector ImpactLocation, float Duration = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Transform")
    static FTransform OffsetTransform(const FTransform& Base, const FVector& TranslationOffset,
                                      const FRotator& RotationOffset);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Transform")
    static FTransform InterpTransform(const FTransform& A, const FTransform& B, float Alpha);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK|Stretch")
    static FVector ComputeOverstretchedEffector(FVector Origin, FVector Target, float StretchRatio = 1.2f);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|General")
    static float GetChainLengthFromBoneEnums(USkeletalMeshComponent* Mesh, const TArray<EOHSkeletalBone>& BoneChain);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|Leg")
    static float GetLegCompressionAlpha(USkeletalMeshComponent* Mesh, EOHSkeletalBone Thigh, EOHSkeletalBone Calf,
                                        EOHSkeletalBone Foot, float MinLength, float MaxLength);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|Spine")
    static float GetSpineBendRatio(USkeletalMeshComponent* Mesh, EOHSkeletalBone SpineStart, EOHSkeletalBone SpineMid,
                                   EOHSkeletalBone SpineEnd);

#pragma region PhysicsSimulation

    // Robustly sets physics blend weight for a single bone (0=animation, 1=physics)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool SetPhysicsBlendWeightForBone(USkeletalMeshComponent* Mesh, FName BoneName, float BlendWeight);

    // Robustly sets blend weight for a set of bones (arbitrary, not validated as chain)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static void SetPhysicsBlendWeightForBones(USkeletalMeshComponent* Mesh, const TArray<FName>& BoneNames,
                                              float BlendWeight);

    // Robustly sets blend weight for a contiguous chain of bones (inclusive)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static void SetPhysicsBlendWeightForBoneChain(USkeletalMeshComponent* Mesh, FName StartBone, FName EndBone,
                                                  float BlendWeight);

    // Enable/disable physical animation and simulation for a single bone
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool EnablePhysicalAnimationForBone(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                               FName BoneName, const FPhysicalAnimationData& Profile);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool DisablePhysicalAnimationForBone(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                FName BoneName);

    // Enable/disable for arbitrary sets of bones
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool EnablePhysicalAnimationForBones(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                const TArray<FName>& BoneNames, const FPhysicalAnimationData& Profile);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool DisablePhysicalAnimationForBones(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                 const TArray<FName>& BoneNames);

    // Enable/disable for contiguous chain of bones (walked via RefSkeleton)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool EnablePhysicalAnimationForBoneChain(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                    FName StartBone, FName EndBone,
                                                    const FPhysicalAnimationData& Profile);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool DisablePhysicalAnimationForBoneChain(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                     FName StartBone, FName EndBone);

    // Minimal physics sim enable/disable for a single bone (no PAC/profile, just sim state)
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool EnablePhysicsSimulationForBone(USkeletalMeshComponent* Mesh, FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test")
    static bool DisablePhysicsSimulationForBone(USkeletalMeshComponent* Mesh, FName BoneName);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test", meta = (WorldContext = "WorldContextObject"))
    static void BlendInPhysicalAnimationForBone(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                FName BoneName, const FPhysicalAnimationData& Profile,
                                                float BlendDuration = 0.2f, UCurveFloat* BlendCurve = nullptr,
                                                UObject* WorldContextObject = nullptr);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test", meta = (WorldContext = "WorldContextObject"))
    static void BlendOutPhysicalAnimationForBone(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                 FName BoneName, float BlendDuration = 0.2f,
                                                 UCurveFloat* BlendCurve = nullptr,
                                                 UObject* WorldContextObject = nullptr);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test", meta = (WorldContext = "WorldContextObject"))
    static void BlendInPhysicalAnimationForBodyPart(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                    EOHBodyPart BodyPart, const FPhysicalAnimationData& Profile,
                                                    float BlendDuration = 0.2f, UCurveFloat* BlendCurve = nullptr,
                                                    UObject* WorldContextObject = nullptr);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Test", meta = (WorldContext = "WorldContextObject"))
    static void BlendOutPhysicalAnimationForBodyPart(USkeletalMeshComponent* Mesh, UPhysicalAnimationComponent* PAC,
                                                     EOHBodyPart BodyPart, float BlendDuration = 0.2f,
                                                     UCurveFloat* BlendCurve = nullptr,
                                                     UObject* WorldContextObject = nullptr);

    static void ApplyPhysicalAnimationToBone(UPhysicalAnimationComponent* PhysicalAnimationComponent, FName BoneName,
                                             const FPhysicalAnimationData& Profile);

    static void ApplyPhysicalAnimationToBoneChain(UPhysicalAnimationComponent* PhysicalAnimationComponent,
                                                  FName RootBoneName, const FPhysicalAnimationData& Profile);

    static void ApplyPhysicalAnimationProfileToBoneChain(UPhysicalAnimationComponent* PhysicalAnimationComponent,
                                                         FName RootBoneName, FName ProfileName);

#pragma endregion

    /** Utility: Checks if a bone is valid for use in a bone map */

    // ------------------ Internal Functions ------------------ //

    // Tries to build and add a valid reference

    static bool TryGetPoseIndex(EOHSkeletalBone Bone, const TMap<EOHSkeletalBone, FCompactPoseBoneIndex>& Indices,
                                FCompactPoseBoneIndex& OutIndex);

#pragma region BoneHierarchy

    static TArray<FConstraintInstance*> GetAllParentConstraints(USkeletalMeshComponent* SkelMesh, FName BoneName);

    static TArray<FName> GetAllParentBoneNames(USkeletalMeshComponent* SkelMesh, FName BoneName);

    float ComputeBoneLength(const USkeletalMeshComponent* SkeletalMesh, FName BoneA, FName BoneB);

#pragma endregion

#pragma region Internals_DerivedCalculations_

    /**
     * Computes the tube of motion for a bone chain over DeltaTime,
     * returning an array of capsule primitives (Start, End, Radius) for collision tests.
     */
    static void GenerateBoneChainMotionCapsules(TArray<FVector> CurrentPositions, TArray<FVector> PredictedPositions,
                                                float Radius, TArray<FTransform>& OutCapsuleTransforms);

    /**
     * Compute an approximate mass for each bone in BoneNames by querying
     * the runtime BodyInstance on the SkeletalMeshComponent.
     */
    static void ComputeBoneMassMap(USkeletalMeshComponent* SkelComp, TArray<FName> BoneNames,
                                   TMap<FName, float>& OutBoneMass);

    /**
     * Blends a reactive pose sample into the current pose delta‐array according to BlendWeight.
     */
    static void BlendReactivePose(const TArray<FBoneTransform>& ReactiveTransforms,
                                  TArray<FBoneTransform>& OutBoneTransforms, float BlendWeight);

    /**
     * Samples the PhysicsAsset to find the world‐space capsule or sphere representing the bone’s body,
     * returning its shape parameters.
     */
    static bool GetBoneCollisionShape(UPhysicsAsset* PhysAsset, FName BoneName, FTransform& OutBoneTransform,
                                      FCollisionShape& OutShape);

#pragma endregion

    // ------------------ Private Functions ------------------ //
#pragma region Private_

  private:
#pragma endregion
};

#if 0
UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|General")
static float PredictChainLengthTrend(
	const TArray<FOHBoneState>& BoneStates
);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK")
	static FVector ComputeEffectorTargetFromVelocity(
		USkeletalMeshComponent* Mesh,
		EOHSkeletalBone StartBone,
		FVector Velocity,
		float ProjectDistance
	);


	UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|Transform")
	static FTransform GetOffsetBoneTransform(
		USkeletalMeshComponent* SK,
		EOHSkeletalBone BoneEnum,
		FVector Offset,
		FRotator RotationOffset
	);



	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal IK")
	static float ComputeCompressionAlphaFromBoneEnums(
		USkeletalMeshComponent* Mesh,
		EOHSkeletalBone UpperArm,
		EOHSkeletalBone LowerArm,
		EOHSkeletalBone Hand,
		float MinLength,
		float MaxLength
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal IK|Prediction")
	static float PredictArmExtensionTrendFromState(const FOHBoneState& Upper, const FOHBoneState& Hand);

	static bool TryBuildBoneReference(EOHSkeletalBone BoneEnum, const FBoneContainer& BoneContainer, TMap<EOHSkeletalBone, FBoneReference>& OutRefs, TMap<EOHSkeletalBone, FCompactPoseBoneIndex>& OutIndices);


	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Utility")
	static FName GetBoneNameFromEnumSafe(EOHSkeletalBone BoneEnum);
	/** Utility: Attempts to build a valid FBoneReference and CompactPoseBoneIndex */
	static bool TryBuildBoneReference(EOHSkeletalBone Bone, const FBoneContainer& BoneContainer, FBoneReference& OutReference, FCompactPoseBoneIndex& OutIndex);
	
	static void PrintBoneMapDebug(const TMap<EOHSkeletalBone, FBoneReference>& BoneRefs);

	/** Returns all valid EOHSkeletalBone enum values from FirstBone to LastBone */
	static TArray<EOHSkeletalBone> GetAllValidBones();

	/** Returns all valid EOHSkeletalBone enum values from FirstBone to LastBone */
	static TArray<EOHSkeletalBone> GetAllValidBones(const FBoneContainer& BoneContainer);
			
	UFUNCTION(BlueprintPure, Category="OnlyHands|SkeletalPhysics")
	static TArray<FName> BP_GetValidatedBoneChainFromComponent(
		USkeletalMeshComponent* SkelComp,
		EOHSkeletalBone EffectorEnum
	);



	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get Bones From Body Part"))
	static TArray<EOHSkeletalBone> BP_GetBonesFromBodyPart(EOHBodyPart BodyPart);

	/** Fast lookup: Get body part from bone */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get Body Part From BoneEnum"))
	static EOHBodyPart BP_GetBodyPartFromBone(EOHSkeletalBone Bone);

	/** Fast lookup: Get body region from bone */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get Body Region From BoneEnum"))
	static EOHBodyRegion BP_GetRegionFromBone(EOHSkeletalBone Bone);
	/** Fast lookup: Get body region from body part */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get Body Region From BodyPart"))
	static EOHBodyRegion BP_GetRegionFromBodyPart(EOHBodyPart BodyPart);

	/** Get all bones in a given region */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup", meta=(DisplayName="Get Bones In Region"))
	static TArray<EOHSkeletalBone> BP_GetBonesInRegion(EOHBodyRegion Region);

	/** Get all body parts in a given region */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get BodyParts In Region"))
	static TArray<EOHBodyPart> BP_GetBodyPartsInRegion(EOHBodyRegion Region);

	/** Get all bones for a region and body part */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Lookup",
		meta=(DisplayName="Get Bones From Region And BodyPart"))
	static TArray<EOHSkeletalBone> BP_GetBonesFromRegionAndBodyPart(EOHBodyRegion Region, EOHBodyPart BodyPart);
	/** Check if a bone is a shoulder (left or right) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Classification")
	static bool BP_IsShoulderBone(EOHSkeletalBone Bone);

	/** Check if a bone is a hand bone (left or right hand) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Classification")
	static bool BP_IsHandBone(const EOHSkeletalBone Bone);

	/** Check if a bone is a foot bone (left or right foot) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Classification")
	static bool BP_IsFootBone(const EOHSkeletalBone Bone);

	/** Check if a bone is a pelvis or hip */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal|Classification")
	static bool BP_IsPelvisBone(const EOHSkeletalBone Bone);

	/**
 * Get all finger bones on the specified hand
 * @param bLeftHand - Whether to get fingers, on the left hand (true) or right hand (false)
 * @return Array of all finger bones on the specified hand
 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Fingers")
	static TArray<EOHSkeletalBone> GetAllFingerBones(bool bLeftHand);

	/**
	 * Determines if a bone belongs to a specific finger
	 * @param Bone - The bone to check
	 * @param FingerIndex - Index of the finger (0=thumb, 1=index, 2=middle, 3=rings, 4=pinky)
	 * @return Whether the bone belongs to the specified finger
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Fingers")
	static bool IsBoneInFingerByIndex(EOHSkeletalBone Bone, int32 FingerIndex);

	/**
	 * Identifies which finger a bone belongs to
	 * @param Bone - The bone to analyze
	 * @return Finger index (0=thumb, 1=index, 2=middle, 3=ring, 4=pinky, -1=not a finger)
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Fingers")
	static int32 GetFingerIndexForBone(EOHSkeletalBone Bone);

	/**
	 * Determines the joint position within a finger
	 * @param Bone - The bone to analyze
	 * @return Joint index (0=proximal, 1=middle, 2=distal, -1=not a finger)
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Fingers")
	static int32 GetJointPositionInFinger(EOHSkeletalBone Bone);


	/**
	 * Gets all bones of a specific anatomical type
	 * @param BoneType - Type name as string (e.g., "spine", "finger", "arm")
	 * @return Array of all bones matching the specified type
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetBonesOfAnatomicalType(FString BoneType);

	/**
	 * Get all bones on the specified side of the body
	 * @param bLeftSide - Whether to get bones on the left side (true) or right side (false)
	 * @return Array of all bones on the specified side of the body
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetBonesOnBodySide(bool bLeftSide);


	/**
	 * Get all bones in a connected chain between two bones
	 * @param StartBone - The starting bone in the chain
	 * @param EndBone - The ending bone in the chain
	 * @return Array of bones in the connection path, or empty if no valid path exists
	 */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|Structure")
	static TArray<EOHSkeletalBone> GetBoneChainBetween(EOHSkeletalBone StartBone, EOHSkeletalBone EndBone);



	UFUNCTION(BlueprintPure, Category = "OHPhysics|Eligibility")
	static bool ShouldSimulate(const FOHBoneState& State);

	UFUNCTION(BlueprintPure, Category = "OHPhysics|Eligibility")
	static bool ShouldApplyPACDrive(const FOHBoneState& State);

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static FName GetBoneNameFromEnum(const EOHSkeletalBone Bone)
	{
		return ::GetBoneNameFromEnum(Bone);
	}

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static EOHSkeletalBone GetEnumFromBoneName(FName BoneName)
	{
		return ::GetEnumFromBoneName(BoneName);
	}

	/** Returns the anatomical body BodyPart of a given skeletal bone */
	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static EOHBodyPart GetBodyPartForBone(EOHSkeletalBone Bone)
	{
		return ::GetBodyPartForBone(Bone);
	}


	/**
	 * Gets the corresponding bone on the opposite side of the body
	 * @param Bone - The source bone to mirror
	 * @return The mirrored bone on the opposite side, or the original if no mirror exists
	 */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Skeletal Physics|Mirroring")
	static EOHSkeletalBone GetMirroredBone(EOHSkeletalBone Bone);

	/** Returns the world transform of a bone or socket safely */
	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static FTransform GetBoneWorldTransformSafe(USkeletalMeshComponent* SkeletalMesh, EOHSkeletalBone Bone);
	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static FBoneMetadata GetBoneMetadata(EOHSkeletalBone BoneEnum);

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static FTransform GetBoneLocalTransformByEnum(const USkeletalMeshComponent* Component, EOHSkeletalBone BoneEnum, bool bIncludeScale);
	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static TArray<FBoneMetadata> GetAllBoneMetadata();
	// ----------------  BoneEnum Region Accessors  ---------------- //
	
	/** Get Region from BodyPart */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static EOHBodyRegion GetRegionFromBodyPart(EOHBodyPart BodyPart);

	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHBodyRegion> GetRegionsFromBodyParts(TArray<EOHBodyPart> BodyParts);

	/** Get Region from BoneEnum */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static EOHBodyRegion GetRegionFromBone(EOHSkeletalBone Bone);

	/** Get Regions from Bones */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHBodyRegion> GetRegionsFromBones(TArray<EOHSkeletalBone> Bones);

	/** Get BodyPart from BoneEnum */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static EOHBodyPart GetBodyPartFromSkeletalBone(EOHSkeletalBone Bone);

	/** Get BodyParts from Bones */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHBodyPart> GetBodyPartsFromBones(TArray<EOHSkeletalBone> Bones);

	/** Get Bones in BodyPart */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Skeletal Physics|BodyParts")
	static TArray<EOHSkeletalBone> GetBonesInBodyPart(EOHBodyPart BodyPart);

	/** Get BodyParts in Region */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHBodyPart> GetBodyPartsInRegion(EOHBodyRegion Region);

	/** Get Bones in BodyPart */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHSkeletalBone> GetBonesInBodyParts(TArray<EOHBodyPart> BodyParts);

	/** Get Bones in Region */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHSkeletalBone> GetBonesInRegion(EOHBodyRegion Region);

	/** Get Bones in Region */
	UFUNCTION(BlueprintPure, Category = "Physics|Mapping")
	static TArray<EOHSkeletalBone> GetBonesInRegions(TArray<EOHBodyRegion> Regions);

	/** Get Bones from Region and BodyPart */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal")
	static TArray<EOHSkeletalBone>
	GetBonesFromRegionAndBodyPart(const EOHBodyRegion Region, const EOHBodyPart BodyPart);

	/** Get Bones from Region and BodyPart */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal")
	static TArray<EOHSkeletalBone> GetBonesFromRegionAndBodyParts(const EOHBodyRegion Region,
	                                                              const TArray<EOHBodyPart> BodyParts);

	/** Get Bones from Region and BodyPart */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal")
	static TArray<EOHSkeletalBone> GetBonesFromRegionsAndBodyParts(const TArray<EOHBodyRegion>& Regions,
	                                                               const TArray<EOHBodyPart>& BodyParts);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal")
	static EOHSkeletalBone GetRootBoneEnumForBodyPart(EOHBodyPart BodyPart);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Physics|Skeletal")
	static EOHSkeletalBone GetEndBoneEnumForBodyPart(EOHBodyPart BodyPart);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Reference")
	EOHSkeletalBone Bone;

	// ───────────── BoneEnum Group Accessors ───────────── //

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetLeftArmBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetRightArmBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetLeftLegBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetRightLegBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetSpineBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetFingerBones();


	// ───────────── IK BoneEnum Groups ───────────── //
	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetIKRootBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetIKLimbBones();

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Groups")
	static TArray<EOHSkeletalBone> GetIKBones(); // Full set

	// ───────────── BoneEnum Analysis Utilities ─────────────

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static bool IsTrackedBoneValid(EOHSkeletalBone RefBone);

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static bool IsIKBone(EOHSkeletalBone RefBone);

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Utility")
	static bool IsSimulatableBone(EOHSkeletalBone RefBone);

	UFUNCTION(BlueprintPure, Category = "Skeletal Physics|Defaults")
	static TArray<EOHSkeletalBone> GetDefaultSimulatableBones();


	UFUNCTION(BlueprintPure, Category = "OnlyHands|BoneEnum Utility")
	static bool ShouldHaveBlendState(const EOHSkeletalBone InBone)
	{
		return InBone != EOHSkeletalBone::None
			&& !IsIKBone(InBone)
			&& !IsFingerBone(InBone)
			&& InBone != EOHSkeletalBone::Pelvis
			&& InBone != EOHSkeletalBone::Root
			&& !IsBallBone(InBone);
	}

	/** Initializes a bone reference map for the given skeletal mesh component */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|IK")
	static void InitializeBoneReferenceMap(
		const USkeletalMeshComponent* Mesh,
		FOHBoneReferenceMap& OutMap
	);

		/** Builds and initializes a bone reference map from a mesh or proxy. */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Physics|Utility")
	static FOHBoneReferenceMap CreateBoneReferenceMap(const USkeletalMeshComponent* MeshComponent);
	

	/** Safely gets a bone name from EOHSkeletalBone */
	static FName GetBoneNameSafe(EOHSkeletalBone Bone)
	{
		return GetBoneNameFromEnum(Bone); // Wrap your existing enum-to-name logic
	}
	
	static FOHBoneContextMap CreateContextMapFromMesh(USkeletalMeshComponent* Mesh)
	{
		FOHBoneContextMap Context;
		Context.InitializeFromMesh(Mesh);
		return Context;
	}

	
	static bool TryGetPoseIndex(
		const FName& BoneName,
		const FBoneContainer& BoneContainer,
		FCompactPoseBoneIndex& OutIndex)
	{
		const int32 RawIndex = BoneContainer.GetPoseBoneIndexForBoneName(BoneName);
		if (RawIndex == INDEX_NONE)
		{
			OutIndex = FCompactPoseBoneIndex(INDEX_NONE);
			return false;
		}

		OutIndex = FCompactPoseBoneIndex(RawIndex);
		return true;
	}
	
	/** Get simplified per-bone debug state for visual debugging */
	static void GetPreviewBoneInfosSafe(const UObject* WorldContext, const FOHBoneContextMap& Context, TArray<FOHBonePreviewInfo>& OutInfos);

	/** Copy-out version with existence check */
	static bool TryGetBoneState(const FOHBoneContextMap& Context, EOHSkeletalBone BoneEnum, FOHBoneState& OutState);

	/** Verifies that the reference map contains valid entries for all bones */
	static bool DoesMapContainBones(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& RequiredBones);

	static bool DoesMapContainAnyBone(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& Bones);

	static TArray<EOHSkeletalBone> GetMissingBones(const FOHBoneReferenceMap& Map, const TArray<EOHSkeletalBone>& ExpectedBones);


	/** Get modifiable pointer to bone state inside a context */
	static FOHBoneState* GetBoneStateFromContext(FOHBoneContextMap& Context, EOHSkeletalBone Bone);

	/** Const version */
	static const FOHBoneState* GetBoneStateFromContext(const FOHBoneContextMap& Context, EOHSkeletalBone Bone);
	
	/** Safe access to a bone state from context; returns false if not found */
	static bool GetBoneStateFromContext(const FOHBoneContextMap& Context, EOHSkeletalBone BoneEnum, FOHBoneState& OutState);

#pragma region Internals_
	/** 
	 * Resolves the current transforms of an IK chain using its cached bone indices.
	 * @return true if all bone indices were valid.
	 */
	static bool ResolveIKChainTransforms(
		FComponentSpacePoseContext& PoseContext,
		const FOHIKBoneIndexCache& ChainCache,
		FTransform& OutRoot,
		FTransform& OutMiddle,
		FTransform& OutTip);

	/**
	 * Initializes the cache indices for a given FOHIKChain.
	 * Safe for use in InitializeBoneReferences().
	 */
	static void InitializeIKChainCache(const FOHIKChain& Chain, const FBoneContainer& BoneContainer, FOHIKBoneIndexCache& OutCache);

	/**
	 * Validates and initializes FBoneReferences from a given FOHIKChain.
	 */
	static bool ValidateAndInitializeIKChain(const FOHIKChain& Chain, const FBoneContainer& BoneContainer, FBoneReference& OutRootRef, FBoneReference& OutMiddleRef, FBoneReference& OutTipRef);

	/** Validates that all expected bones are present in the reference map. Logs any missing bones. */
	static bool ValidateReferenceMapAgainstExpectedBones(
		const FOHBoneReferenceMap& Map,
		const TArray<EOHSkeletalBone>& ExpectedBones,
		FName ContextLabel = NAME_None);

	static bool TryGetBoneReferenceFromEnum(EOHSkeletalBone BoneEnum, const FBoneContainer& BoneContainer,FBoneReference& OutReference);
	
	// === Bone Reference Map Generation ===
	static FOHBoneReferenceMap CreateBoneReferenceMap(const FBoneContainer& BoneContainer);
	static FOHBoneReferenceMap CreateBoneReferenceMap(const FBoneContainer& BoneContainer, const TArray<EOHSkeletalBone>& Bones);

	static FOHBoneReferenceMap CreateBoneReferenceMap(const FAnimInstanceProxy* AnimProxy);
	static FOHBoneReferenceMap CreateBoneReferenceMap(const FAnimInstanceProxy* AnimProxy, const TArray<EOHSkeletalBone>& Bones);

	static FOHBoneReferenceMap CreateBoneReferenceMap(const FAnimationInitializeContext& Context);
	static FOHBoneReferenceMap CreateBoneReferenceMap(const FAnimationInitializeContext& Context, const TArray<EOHSkeletalBone>& Bones);

	static FOHBoneReferenceMap CreateBoneReferenceMap(const UAnimInstance* AnimInstance);
	static FOHBoneReferenceMap CreateBoneReferenceMap(const UAnimInstance* AnimInstance, const TArray<EOHSkeletalBone>& Bones);

	static FOHBoneReferenceMap CreateBoneReferenceMap(const USkeletalMeshComponent* MeshComp, const TArray<EOHSkeletalBone>& Bones);

	/** Utility: Checks if a bone is valid for use in a bone map */
	static bool IsBoneValidForMap(EOHSkeletalBone Bone, const FBoneContainer& BoneContainer);



	/** Utility: Adds a bone reference + index to a given bone map */
	static void AddToBoneMap(
		FOHBoneReferenceMap& Map,
		EOHSkeletalBone Bone,
		const FBoneReference& Ref,
		const FCompactPoseBoneIndex& Index);

	// Resolves a bone's FName from its enum
	static FName ResolveBoneName(EOHSkeletalBone BoneEnum);

private:
	static const TMap<EOHSkeletalBone, EOHBodyPart>& GetBoneToBodyPartMap();
	static const TMap<EOHSkeletalBone, EOHBodyRegion>& GetBoneToRegionMap();
	static const TMap<EOHBodyPart, EOHBodyRegion>& GetBodyPartToRegionMap();
	static TMap<FName, TArray<EOHSkeletalBone>> CustomBoneRegions;
#endif
