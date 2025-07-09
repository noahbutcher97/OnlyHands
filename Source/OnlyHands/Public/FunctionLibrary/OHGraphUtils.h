
#pragma once

#include "CoreMinimal.h"
#include "Data/Struct/OHPhysicsStructs.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Utilities/OHSafeMapUtils.h"

#include "OHGraphUtils.generated.h"

/**
 * UOHGraphUtils
 *
 * Static utilities for building, validating, and querying the OHPhysicsGraph
 * structure. Leverages OHSafeMapUtils for robust collection operations and
 * validation logic.
 */
UCLASS()
class ONLYHANDS_API UOHGraphUtils : public UBlueprintFunctionLibrary {
  GENERATED_BODY()

public:
  /** Validates a bone based on strictness level */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|GraphUtils")
  static bool IsBoneValid(const FOHBoneData &Bone,
                          EValidationStrictness Strictness);

  /** Inserts bone unless an irreconcilable conflict exists with current entry
   */
  static bool AddBoneIfNoConflict(TMap<FName, FOHBoneData> &BoneMap,
                                  const FOHBoneData &BoneData);

  /** Inserts all bones from source into target if they pass validation at the
   * given strictness level */
  static int32 InsertAllBonesIfValid(TMap<FName, FOHBoneData> &TargetMap,
                                     const TMap<FName, FOHBoneData> &SourceMap,
                                     EValidationStrictness Strictness);

  /** Inserts all structurally valid constraints into the graph */
  static int32 InsertAllConstraintsIfValid(
      TArray<FOHConstraintInstanceData> &TargetList,
      const TArray<FOHConstraintInstanceData> &SourceList,
      const TMap<FName, FOHBoneData> &BoneMap);

  /** Gets the immediate children of a bone from its cached FOHBoneData field */
  static TArray<FName> GetChildBones(const TMap<FName, FOHBoneData> &BoneMap,
                                     FName ParentBone);

  /** Gets all constraints referencing a specific bone as parent or child */
  static TArray<FOHConstraintInstanceData> GetConstraintsForBone(
      FName BoneName, const TArray<FOHConstraintInstanceData> &ConstraintList);

  /** Gets all bone names that are flagged as having simulated bodies */
  static TArray<FName>
  GetSimulatedBones(const TMap<FName, FOHBoneData> &BoneMap);

  /** Removes any constraints that reference non-existent or non-simulated bones
   */
  static void
  RemoveInvalidConstraints(TArray<FOHConstraintInstanceData> &ConstraintList,
                           const TMap<FName, FOHBoneData> &BoneMap);

  /** Normalizes cached body mass across bones using min-max scaling to [0,1] */
  static void NormalizeBoneMasses(TMap<FName, FOHBoneData> &BoneMap);

  /** Computes the Shannon entropy of the bone name key distribution (graph
   * complexity metric) */
  static float ComputeBoneMapEntropy(const TMap<FName, FOHBoneData> &BoneMap);

  /** Removes bone entries whose mass is an outlier using Z-score deviation */
  static void RemoveBoneMassOutliers(TMap<FName, FOHBoneData> &BoneMap,
                                     float ThresholdZ = 2.0f);

  /** Logs runtime stats and health checks for a given physics graph */
  static void
  LogGraphStats(const TMap<FName, FOHBoneData> &BoneMap,
                const TArray<FOHConstraintInstanceData> &Constraints,
                const FString &ContextTag = TEXT("PhysicsGraph"));

  /** Validates that every path from root to leaf contains only valid, simulated
   * bones */
  static bool
  ValidateAllChainsStable(const TMap<FName, FOHBoneData> &BoneMap,
                          const TArray<FOHConstraintInstanceData> &Constraints,
                          int32 MaxDepth = 16);

  // Build full graph
  static bool BuildPhysicsGraphFromComponent(
      USkeletalMeshComponent *SkeletalMeshComp, FOHPhysicsGraphNode &OutGraph,
      const FString &ContextTag, bool bRunAudit = false);

  // Fully rebuilds a graph node from skeletal mesh and physics asset.

  static void ClearPhysicsGraph(FOHPhysicsGraphNode &Graph);
  bool RefreshBonesOnlyFromComponent(USkeletalMeshComponent *SkeletalMeshComp,
                                     FOHPhysicsGraphNode &Graph,
                                     const FString &ContextTag,
                                     bool bRebindOwnerComponent);
  bool RefreshConstraintsOnlyFromComponent(
      USkeletalMeshComponent *SkeletalMeshComp, FOHPhysicsGraphNode &Graph,
      const FString &ContextTag, bool bRebuildFullConstraintList,
      bool bRebindOwnerComponent);
  bool RefreshGraphFromComponent(USkeletalMeshComponent *SkeletalMeshComp,
                                 FOHPhysicsGraphNode &Graph,
                                 const FString &ContextTag, bool bRefreshBones,
                                 bool bRefreshConstraints,
                                 bool bRebindOwnerComponent,
                                 bool bFullConstraintRebuild);

  // Resets the graph cleanly and logs state.
  static void ResetGraph(FOHPhysicsGraphNode &Graph);

  // Rebuilds graph defensively, maintaining existing state when partial rebuild
  // fails.
  static bool SafeRebuildGraph(const USkeletalMeshComponent *SkelComp,
                               FOHPhysicsGraphNode &Graph,
                               bool bForceResetIfFailed = true);

  // Returns bone names with simulated bodies only.
  static TSet<FName> GetSimulatedBoneNames(const FOHPhysicsGraphNode &Graph);

  // Returns all constraint names or labels.
  static TArray<FName> GetAllConstraintNames(const FOHPhysicsGraphNode &Graph);

  // Gets all children recursively for a given root bone.
  static TSet<FName> GetAllDescendants(const FOHPhysicsGraphNode &Graph,
                                       FName RootBone);

  // Builds adjacency map for constraints between simulated bodies.
  static TMap<FName, TArray<FName>>
  BuildAdjacencyMap(const FOHPhysicsGraphNode &Graph);

  // Gathers all connected components (islands of connected bodies).
  static TArray<TSet<FName>>
  ExtractConnectedComponents(const FOHPhysicsGraphNode &Graph);

  // Detects cycles in the graph based on constraints.
  static bool HasCycle(const FOHPhysicsGraphNode &Graph);

  // Validates that all constraints reference valid, simulated bone pairs.
  static bool ValidateConstraintConnectivity(const FOHPhysicsGraphNode &Graph);

  // Returns true if any body is missing a valid BodyInstance pointer.
  static bool HasMissingBodies(const FOHPhysicsGraphNode &Graph);

  // Returns a detailed map of constraint health (mass ratio, distance, etc).
  static TMap<FName, FString>
  GetConstraintHealthReport(const FOHPhysicsGraphNode &Graph);

  // Removes all constraints referencing missing or invalid bodies.
  static void PruneInvalidConstraints(FOHPhysicsGraphNode &Graph);

  // Rebuilds child lists on all bones from constraints only.
  static void RebuildChildBoneLinks(FOHPhysicsGraphNode &Graph);

  // Ensures all bones have coherent Parent–Child linkages.
  static bool ValidateAndRepairBoneHierarchy(FOHPhysicsGraphNode &Graph);

  // Assigns default PAC or inertial tuning to bones lacking values.
  static void AssignDefaultProfiles(FOHPhysicsGraphNode &Graph);

  static float
  CalculateConstraintInfluenceScore(const FOHConstraintInstanceData &Constraint,
                                    const FOHPhysicsGraphNode &Graph,
                                    const USkeletalMeshComponent *MeshComp);

  static void DrawPhysicsGraphOverlay(const FOHPhysicsGraphNode &Graph,
                                      const USkeletalMeshComponent *MeshComp,
                                      UWorld *World, float Duration = 1.f);

  /** Detect if the physics graph contains any cycles */
  static bool
  DetectCyclesInGraph(const TMap<FName, FOHBoneData> &BoneMap,
                      const TArray<FOHConstraintInstanceData> &Constraints);

  /** Updates adjacency caches for faster graph queries */
  static void UpdateAdjacencyCaches(FOHPhysicsGraphNode &Graph);

  /** Refreshes runtime FConstraintInstance* pointers from the skeletal mesh */
  static void RefreshConstraintInstances(FOHPhysicsGraphNode &Graph,
                                         USkeletalMeshComponent *SkeletalMesh);

  // Runs a BFS from a root bone to collect reachable nodes.
  static TSet<FName> TraverseGraphBFS(const FOHPhysicsGraphNode &Graph,
                                      FName RootBone);

  // Breadth-first traversal: returns visited bones up to limit
  UFUNCTION(BlueprintCallable, Category = "OnlyHands|Graph")
  static TSet<FName> TraverseGraphBFS(const FOHPhysicsGraphNode &Graph,
                                      FName Start, int32 MaxNodesToVisit = 512);

  // Runs DFS from a root bone, calling visitor lambda per node.
  static void TraverseGraphDFS(const FOHPhysicsGraphNode &Graph, FName Start,
                               TFunctionRef<void(const FName &)> Visitor);

  // Detect cycles in a graph: returns true if cycle found
  UFUNCTION(BlueprintCallable, Category = "OnlyHands|Graph")
  static bool DetectCyclesInGraph(const FOHPhysicsGraphNode &Graph);

  // Validate graph bones and constraints; returns success status
  UFUNCTION(BlueprintCallable, Category = "OnlyHands|Graph")
  static bool ValidateGraph(FOHPhysicsGraphNode &Graph,
                            EValidationStrictness Strictness);

  // Example: prune invalid bones or constraints from graph
  UFUNCTION(BlueprintCallable, Category = "OnlyHands|Graph")
  static void PruneInvalidGraphElements(FOHPhysicsGraphNode &Graph,
                                        EValidationStrictness Strictness);

  UFUNCTION(BlueprintCallable, Category = "OnlyHands|Graph|Debug")
  static void DrawPhysicsGraphOverlay(const FOHPhysicsGraphNode &Graph,
                                      const USkeletalMeshComponent *MeshComp,
                                      UWorld *World, float Duration,
                                      EPhysicsGraphOverlayMode OverlayMode,
                                      bool bDrawLabels);
  static EPhysicsGraphOverlayMode GetPhysicsGraphOverlayMode();
  static void SetPhysicsGraphOverlayMode(EPhysicsGraphOverlayMode Mode);
};

#pragma region Console Commands
static FAutoConsoleCommand Cmd_ShowPhysicsGraphOverlay(
    TEXT("ShowPhysicsGraphOverlay"),
    TEXT("Set OnlyHands physics graph overlay: 0=None, 1=Full, 2=Anomalies, "
         "3=Both"),
    FConsoleCommandWithArgsDelegate::CreateLambda(
        [](const TArray<FString> &Args) {
          if (Args.Num() == 0) {
            UE_LOG(LogTemp, Log, TEXT("Current overlay mode: %d"),
                   (int32)UOHGraphUtils::GetPhysicsGraphOverlayMode());
            return;
          }
          int32 Mode = FCString::Atoi(*Args[0]);
          Mode = FMath::Clamp(Mode, 0, 3);
          UOHGraphUtils::SetPhysicsGraphOverlayMode(
              static_cast<EPhysicsGraphOverlayMode>(Mode));
          UE_LOG(LogTemp, Log, TEXT("Physics Graph Overlay set to mode %d"),
                 Mode);
        }));
#pragma endregion