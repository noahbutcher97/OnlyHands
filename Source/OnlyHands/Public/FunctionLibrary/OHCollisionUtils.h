#pragma once
#include "CoreMinimal.h"
#include "OHPhysicsStructs.h"
#include "Components/SplineMeshComponent.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "OHCollisionUtils.generated.h"

USTRUCT(BlueprintType)
struct FStrikePredictionResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	bool bWillHit = false;

	UPROPERTY(BlueprintReadOnly)
	FVector PredictedImpactPoint = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	float TimeUntilImpact = -1.f;

	UPROPERTY(BlueprintReadOnly)
	float DistanceAtImpact = -1.f;

	UPROPERTY(BlueprintReadOnly)
	FVector AttackerForward = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector FutureTargetPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector DirectionToTarget = FVector::ZeroVector;
};
UCLASS()
class ONLYHANDS_API UOHCollisionUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
#pragma region CollisionDetection
#pragma region AABB
	// AABB
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|AABB")
	static bool AABB_Overlap(
		const FVector& CenterA, const FVector& ExtentsA,
		const FVector& CenterB, const FVector& ExtentsB
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|AABB")
	static FVector ComputeAABBDistanceOrPenetration(
		const FVector& CenterA, const FVector& ExtentsA,
		const FVector& CenterB, const FVector& ExtentsB
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|AABB")
	static FVector ClosestPointOnAABB(
		const FVector& Point,
		const FVector& BoxCenter,
		const FVector& Extents
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|GJK")
	static FVector SupportAABB(
		const FVector& Direction,
		const FVector& Center,
		const FVector& Extents
	);

#pragma endregion


#pragma region SAT
	// SAT - Separating Axis Theorem
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|SAT")
	static bool SAT_BoxIntersect(
		const FVector& CenterA, const FRotator& RotationA, const FVector& ExtentsA,
		const FVector& CenterB, const FRotator& RotationB, const FVector& ExtentsB
	);

#pragma endregion

#pragma region Minkowski
	// Minkowski
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Minkowski")
	static TArray<FVector> ComputeMinkowskiSum(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB
	);


	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Minkowski")
	static TArray<FVector> ComputeMinkowskiDifference(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB
	);

#pragma endregion

#pragma region OBB

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|OBB")
	static FVector SupportOBB(
		const FVector& Direction,
		const FVector& BoxCenter,
		const FRotator& BoxRotation,
		const FVector& BoxExtents
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|OBB")
	static FVector ClosestPointOnOBB(
		const FVector& Point,
		const FVector& BoxCenter,
		const FRotator& BoxRotation,
		const FVector& BoxExtents
	);

	// Point in Box
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|OBB")
	static bool IsPointInOBB(
		const FVector& Point,
		const FVector& BoxCenter,
		const FRotator& BoxRotation,
		const FVector& BoxExtents
	);

#pragma endregion

#pragma region GJK
	// GJK
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|GJK")
	static bool GJK_Intersect(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB,
		int32 MaxIterations = 20,
		float Tolerance = 0.001f
	);

	static bool GJK_Intersect(
	const TArray<FVector>& ShapeA,
	const TArray<FVector>& ShapeB,
	TArray<FVector>& OutSimplex,
	int32 MaxIterations = 30,
	float Tolerance = 0.001f);

	static bool GJK_Intersect(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB,
		TArray<FVector>& OutSimplex,
		TArray<FVector>& OutSupportA,
		TArray<FVector>& OutSupportB,
		int32 MaxIterations = 30,
		float Tolerance = 0.001f);
	
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|GJK")
	static FVector GetFarthestPointInDirection(
		const TArray<FVector>& Points,
		const FVector& Direction
	);

	static FVector TripleCrossProduct(const FVector& A, const FVector& B, const FVector& C);

	static bool ProcessSimplex(TArray<FVector>& Simplex, FVector& OutDirection);

#pragma endregion


#pragma region EPA
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|EPA")
	static bool EPA_PenetrationDepth(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB,
		const TArray<FVector>& InitialSimplex,
		FVector& OutNormal,
		float& OutPenetrationDepth,
		int32 MaxIterations = 30,
		float Tolerance = 0.001f
	);
#pragma endregion


#pragma endregion

#pragma region ShapeGeneration

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Shapes")
	static TArray<FVector> GenerateBoxHull(
		const FVector& Center,
		const FVector& Extents,
		const FRotator& Rotation
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Shapes")
	static TArray<FVector> GenerateSphereHull(
		const FVector& Center,
		float Radius,
		int32 Subdivisions = 3
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Shapes")
	static TArray<FVector> GenerateCapsuleHull(
		const FVector& Center,
		const FRotator& Rotation,
		float HalfHeight,
		float Radius,
		int32 HemisphereResolution = 4,
		int32 RingResolution = 8
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Hull")
	static TArray<FVector> ComputeConvexHull(const TArray<FVector>& InputPoints);

public:
	static void ComputeConvexHull_Internal(
		const TArray<FVector>& InPoints,
		TArray<FVector>& OutHullVertices,
		int32 MaxIterations = 128,
		float Epsilon = KINDA_SMALL_NUMBER
		);

	static void ComputeConvexHullWithIndices(
		const TArray<FVector>& InPoints,
		TArray<FVector>& OutVertices,
		TArray<int32>& OutIndices
	);

	UFUNCTION(BlueprintPure, Category = "Combat|Prediction")
	static FVector ComputeAverageContactPoint(
		const TArray<FVector>& SupportA,
		const TArray<FVector>& SupportB);
	
#pragma endregion

#pragma region MeshData

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static TArray<FVector> GetStaticMeshComponentVertices(UStaticMeshComponent* StaticComp, bool bWorldSpace = true);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static TArray<FVector> GetSkeletalMeshComponentVertices(USkeletalMeshComponent* SkelComp, bool bWorldSpace = true);
	
	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static TArray<FVector> GetSplineMeshComponentVertices(USplineMeshComponent* SplineComp, bool bWorldSpace = true);
	
	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static TArray<FVector> GetMeshComponentVertices(UMeshComponent* MeshComp, bool bWorldSpace = true);
	
	// AABB in world or local space
	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static void GetMeshComponentAABB(
		UMeshComponent* MeshComp,
		FVector& OutCenter,
		FVector& OutExtents,
		bool bWorldSpace = true
	);

	// OBB (returns center, rotation, extents)
	UFUNCTION(BlueprintCallable, Category="OnlyHands|Collision|Generation")
	static void GetMeshComponentOBB(
		UMeshComponent* MeshComp,
		FVector& OutCenter,
		FRotator& OutRotation,
		FVector& OutExtents,
		bool bWorldSpace = true
	);

#pragma endregion

	
#pragma region Debug


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Collision", meta = (WorldContext = "WorldContextObject"))
	static void DebugDrawSATBox(
		const UObject* WorldContextObject,
		const FVector& Center,
		const FRotator& Rotation,
		const FVector& Extents,
		const FColor Color = FColor::Cyan,
		float Duration = 2.0f,
		float LineThickness = 1.5f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Collision", meta=(WorldContext="WorldContextObject"))
	static void DebugDrawAABB(
		const UObject* WorldContextObject,
		const FVector& Center,
		const FVector& Extents,
		const FColor Color = FColor::Red,
		float Duration = 2.0f,
		float LineThickness = 1.5f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Collision", meta=(WorldContext="WorldContextObject"))
	static void DrawDebugOBB(
		const UObject* WorldContextObject,
		const FVector& Center,
		const FRotator& Rotation,
		const FVector& Extents,
		const FColor Color = FColor::White,
		float Duration = 2.0f,
		float LineThickness = 1.5f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|GJK", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugSimplex(
		const UObject* WorldContextObject,
		const TArray<FVector>& SimplexPoints,
		const FColor Color = FColor::Green,
		float Duration = 2.0f,
		float PointSize = 10.0f,
		float LineThickness = 1.5f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Minkowski", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugPointCloud(
		const UObject* WorldContextObject,
		const TArray<FVector>& Points,
		const FColor Color = FColor::White,
		float PointSize = 6.0f,
		float Duration = 2.0f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Hull", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugConvexHull(
		const UObject* WorldContextObject,
		const TArray<FVector>& HullPoints,
		const FColor Color = FColor::Green,
		float PointSize = 8.f,
		float LineThickness = 1.5f,
		float Duration = 2.0f,
		bool bDrawEdgesIfSafe = true
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Mesh", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugConvexMeshFromFaces(
		const UObject* WorldContextObject,
		const TArray<FVector>& Vertices,
		const TArray<int32>& Indices,
		const FColor Color = FColor::Green,
		float LineThickness = 1.5f,
		float Duration = 2.0f
	);

	
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|GJK", meta = (WorldContext = "WorldContextObject"))
	static void DebugConvexIntersectionCheck(
		const UObject* WorldContextObject,
		const TArray<FVector>& PointsA,
		const TArray<FVector>& PointsB,
		const FColor ColorA,
		const FColor ColorB,
		float Duration = 2.0f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Trail", meta = (WorldContext = "WorldContextObject"))
	static void DrawConvexHullFromMotionHistory(
		const UObject* WorldContextObject,
		const TArray<FVector>& HistoryPoints,
		const FColor HullColor = FColor::Magenta,
		float Duration = 1.0f
	);


	/** Draws a 2D arc (fan) in the XY plane to visualize a strike or cone angle */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
	static void DrawDebugStrikeArc(
		const UObject* WorldContextObject,
		const FVector& Origin,
		const FVector& Forward,
		float Radius,
		float ArcAngleDegrees,
		FColor Color = FColor::Red,
		float Duration = 1.0f,
		bool bPersistentLines = false,
		int32 Segments = 32
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Vision", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugCone(
		const UObject* WorldContextObject,
		const FVector& Origin,
		const FVector& Direction,
		float Length,
		float ConeAngleDegrees,
		const FColor Color = FColor::Blue,
		int32 Segments = 32,
		float Duration = 1.0f,
		bool bPersistent = false
	);



	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Trajectory", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugTrajectory(
		const UObject* WorldContextObject,
		const FVector& Start,
		const FVector& Velocity,
		float StepSize,
		int32 MaxSteps,
		const FColor Color = FColor::Green,
		float Duration = 1.0f,
		bool bPersistent = false,
		FVector Gravity = FVector(0, 0, -980.f)
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug|Prediction", meta = (WorldContext = "WorldContextObject"))
	static void DrawDebugConvexSweep(
		const UObject* WorldContextObject,
		const TArray<FVector>& ShapePoints,
		const FVector& Velocity,
		float DeltaTime,
		const FColor StartColor,
		const FColor EndColor,
		float Duration = 1.0f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Debug")
	static void DrawDebugConvexFaceNormals(
		const UObject* WorldContextObject,
		const TArray<FVector>& Vertices,
		const TArray<int32>& Indices,
		float NormalLength = 10.f,
		FColor Color = FColor::Green,
		float Duration = 1.f,
		float Thickness = 1.f);
	
#pragma endregion

#pragma region Tests
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromBoxBounds(
		const UObject* WorldContextObject,
		const AActor* SourceActor,
		const FColor HullColor = FColor::White,
		float Duration = 2.0f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromSockets(
		const UObject* WorldContextObject,
		USkeletalMeshComponent* MeshComp,
		const TArray<FName>& SocketNames,
		const FColor HullColor = FColor::Cyan,
		float Duration = 2.0f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromImpactPoints(
		const UObject* WorldContextObject,
		const TArray<FVector>& ImpactPoints,
		const FColor HullColor = FColor::White,
		float Duration = 2.0f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromMotionTrail(
		const UObject* WorldContextObject,
		USceneComponent* TargetComponent,
		int32 NumSamples = 8,
		float SimulatedDuration = 0.2f,
		const FColor HullColor = FColor::Yellow,
		float Duration = 2.0f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromBoneCluster(
		const UObject* WorldContextObject,
		USkeletalMeshComponent* MeshComp,
		const TArray<FName>& BoneOrSocketNames,
		const FColor HullColor = FColor::Cyan,
		float Duration = 2.0f
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|IK|CollisionTest", meta = (WorldContext = "WorldContextObject"))
	static void GenerateConvexHullFromBodyPartChain(
		const UObject* WorldContextObject,
		USkeletalMeshComponent* MeshComp,
		EOHBodyPart BodyPart,
		const FColor HullColor = FColor::Blue,
		float Duration = 2.0f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Test", meta = (WorldContext = "WorldContextObject"))
	static void RunStrikePredictionDebug(
		const UObject* WorldContextObject,
		AActor* Attacker,
		AActor* Target,
		float ArcAngle = 90.f,
		float Radius = 200.f,
		float PredictTime = 0.3f
	);


	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Test", meta = (WorldContext = "WorldContextObject"))
	static void RunStrikePredictionLive(
		const UObject* WorldContextObject,
		AActor* Attacker,
		AActor* Target,
		float Radius = 200.f,
		float ArcAngle = 90.f,
		float PredictTime = 0.2f,
		bool bShowTrajectory = true,
		bool bShowArc = true,
		bool bShowResult = true
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|AI|Reaction")
	static bool ShouldDodgePredictedStrike(
		AActor* Attacker,
		AActor* SelfActor,
		float ArcAngleDegrees = 90.f,
		float Radius = 200.f,
		float PredictTime = 0.2f
	);
	
#pragma endregion

#pragma region Sampling

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Test")
	static TArray<FVector> SampleMotionPoints(
		USceneComponent* TargetComponent,
		int32 NumSamples,
		float DurationSeconds
	);

	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Collision|Trail")
	static void UpdateMotionHistory(
		TArray<FVector>& HistoryBuffer,
		const FVector& NewSample,
		float MaxDistance = 200.f,
		int32 MaxSamples = 12
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|HitDetection")
	float ComputeContactConfidence(const FHitResult& Hit, const FOHBoneData& BoneData);
	
#pragma endregion

#pragma region Detection
	/** Returns a normalized direction vector from OriginActor to the nearest ACharacter (excluding self). Returns false if none found. */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static bool GetDirectionToClosestCharacter(AActor* OriginActor, FVector& OutDirection, ACharacter*& OutClosestCharacter);

	/** Returns all ACharacters within a given radius from OriginActor (excluding self) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static TArray<ACharacter*> GetCharactersInRange(AActor* OriginActor, float Radius);


	/** Returns the closest ACharacter within a given radius from OriginActor (excluding self). Returns false if none found. */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static bool GetClosestCharacterInRange(AActor* OriginActor, float Radius, ACharacter*& OutClosestCharacter);

	/** Uses a physics-based sphere overlap to find the closest ACharacter within a radius from OriginActor. Returns false if none found. */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static bool GetClosestCharacterInTraceRadius(
		AActor* OriginActor,
		float Radius,
		ACharacter*& OutClosestCharacter,
		ECollisionChannel TraceChannel = ECC_Pawn
	);

	/** Returns normalized direction vector to the closest character within a radius using a sphere trace. Returns false if none found. */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static bool GetDirectionToClosestCharacterInRadiusTrace(
		AActor* OriginActor,
		float Radius,
		FVector& OutDirection,
		ACharacter*& OutClosestCharacter,
		ECollisionChannel TraceChannel = ECC_Pawn
	);

	/** Finds closest character in radius using overlap + line trace, returns direction + hit. Optionally filters by visibility and draws debug. */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Proximity")
	static bool GetDirectionAndHitToClosestCharacterInRadiusTrace(
		AActor* OriginActor,
		float Radius,
		FVector& OutDirection,
		ACharacter*& OutClosestCharacter,
		FHitResult& OutHit,
		ECollisionChannel OverlapChannel = ECC_Pawn,
		ECollisionChannel TraceChannel = ECC_Visibility,
		bool bRequireLineOfSight = false,
		bool bDrawDebug = false,
		FColor DebugColor = FColor::Green,
		float DebugDuration = 2.0f
	);


	// AABB-based
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Targeting")
	static bool GetDirectionToClosestCharacterUsingAABB(AActor* OriginActor, float Radius, FVector& OutDirection, ACharacter*& OutClosestCharacter);

	// OBB-based
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Targeting")
	static bool GetClosestCharacterUsingOBBIntersection(AActor* OriginActor, FVector Extents, FRotator Rotation, ACharacter*& OutClosestCharacter);

	// GJK-based
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Collision|Targeting")
	static bool GetClosestCharacterUsingGJKIntersection(const TArray<FVector>& AttackHull, AActor* OriginActor, ACharacter*& OutClosestCharacter);



	/** Uses AABB overlap to find nearest character within box range */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Gameplay|Detection")
	static bool DetectTargetUsingAABB(
		AActor* OriginActor,
		float Radius,
		ACharacter*& OutClosestCharacter,
		FVector& OutDirection
	);

	/** Uses SAT OBB intersect to find intersecting character */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Gameplay|Detection")
	static bool DetectTargetUsingOBB(
		AActor* OriginActor,
		const FVector& OriginExtents,
		const FRotator& OriginRotation,
		ACharacter*& OutClosestCharacter,
		FVector& OutDirection
	);

	/** Uses GJK intersection with generated convex hulls */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Gameplay|Detection")
	static bool DetectTargetUsingGJK(
		const TArray<FVector>& AttackHull,
		AActor* OriginActor,
		ACharacter*& OutClosestCharacter,
		FVector& OutDirection
	);

	/** Uses GJK + EPA to find hit and get penetration direction/depth */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Gameplay|Detection")
	static bool DetectTargetWithEPA(
		const TArray<FVector>& AttackHull,
		AActor* OriginActor,
		ACharacter*& OutClosestCharacter,
		FVector& OutNormal,
		float& OutPenetrationDepth
	);

	/** Detect using sampled motion trail and convex hull generation */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Gameplay|Detection")
	static bool DetectTargetFromTrailHull(
		USceneComponent* TrailComponent,
		float Duration,
		int32 NumSamples,
		AActor* OriginActor,
		ACharacter*& OutClosestCharacter,
		FVector& OutDirection
	);

#pragma endregion

#pragma region PhysicsHelpers


	/** Computes a world-space impulse vector to separate two AABBs (penetration resolution) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|PhysicsSolver|AABB")
	static bool ComputeAABBSeparationImpulse(
		const FVector& CenterA,
		const FVector& ExtentsA,
		const FVector& CenterB,
		const FVector& ExtentsB,
		FVector& OutImpulse
	);

	/** Solves for EPA-based contact correction between convex shapes */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|PhysicsSolver|Convex")
	static bool ComputeEPAPenetrationCorrection(
		const TArray<FVector>& ShapeA,
		const TArray<FVector>& ShapeB,
		FVector& OutCorrectionNormal,
		float& OutPenetrationDepth
	);

	/** Applies world-space linear impulse to a bone using physical animation */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|PhysicsSolver|Bones")
	static void ApplyImpulseToBone(
		USkeletalMeshComponent* Mesh,
		FName BoneName,
		const FVector& Impulse,
		bool bVelChange = true
	);


#pragma endregion

#pragma region PhysicsSolver_Extensions

	/** Safely applies angular torque to a simulating bone to rotate it toward a desired world-space rotation */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|PhysicsSolver|Torque")
	static void ApplySafeAngularTorque(
		USkeletalMeshComponent* Mesh,
		FName BoneName,
		const FRotator& TargetRotation,
		float TorqueStrength
	);

	/** Applies spring-damper linear force to a bone toward a target position (e.g. stabilizing limbs) */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|PhysicsSolver|Spring")
	static void ApplySafeSpringForce(
		USkeletalMeshComponent* Mesh,
		FName BoneName,
		const FVector& TargetPosition,
		float SpringStrength,
		float Damping
	);

	/** Uses torque to rotate a simulating bone to look at a target world location */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|PhysicsSolver|Orientation")
	static void ApplyLookAtTorque(
		USkeletalMeshComponent* Mesh,
		FName BoneName,
		const FVector& TargetLocation,
		float TorqueStrength
	);

	/** Adjusts orientation/position stiffness based on hit strength, useful for procedural flinching */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|PhysicsSolver|Response")
	static void AdjustPhysicalStiffness(
		UPhysicalAnimationComponent* PhysAnim,
		FName BoneName,
		float HitMagnitude,
		float MaxExpectedForce,
		float MinStiffness,
		float MaxStiffness
	);

#pragma endregion

#pragma region Prediction

	/** Projects a future position based on initial position, velocity, and time */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static FVector PredictFuturePosition(
		const FVector& StartPosition,
		const FVector& Velocity,
		float Time
	);

	/** Predicts time until two actors with velocity vectors intersect within a given radius */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static float PredictTimeToCollision(
		const FVector& PositionA,
		const FVector& VelocityA,
		const FVector& PositionB,
		const FVector& VelocityB,
		float CombinedRadius
	);

	/** Predicts if AABBs will overlap after DeltaTime */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static bool PredictAABBOverlap(
		const FVector& CenterA,
		const FVector& ExtentsA,
		const FVector& VelocityA,
		const FVector& CenterB,
		const FVector& ExtentsB,
		const FVector& VelocityB,
		float DeltaTime
	);

	/** Predicts the intercept point between a moving target and a chaser */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static FVector ComputeInterceptPoint(
		const FVector& ChaserPosition,
		float ChaserSpeed,
		const FVector& TargetPosition,
		const FVector& TargetVelocity
	);






	/** Performs a convex hull sweep by offsetting one shape by velocity and checking GJK */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static bool PredictConvexSweepOverlap(
		const TArray<FVector>& ShapeA,
		const FVector& VelocityA,
		const TArray<FVector>& ShapeB,
		float DeltaTime
	);

	/** Computes an aim direction that leads a moving target for projectile interception */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static FVector ComputeLeadAimDirection(
		const FVector& Origin,
		const FVector& TargetLocation,
		const FVector& TargetVelocity,
		float ProjectileSpeed
	);

	/** Predicts if a target lies within a conical area from origin */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static bool IsTargetWithinCone(
		const FVector& Origin,
		const FVector& Direction,
		const FVector& TargetLocation,
		float ConeAngleDegrees,
		float MaxDistance
	);

	/** Predicts if a target lies within a strike arc (2D fan shape) */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Prediction")
	static bool IsTargetWithinStrikeArc(
		const FVector& Origin,
		const FVector& Forward,
		const FVector& TargetLocation,
		float ArcAngleDegrees,
		float Radius
	);

	UFUNCTION(BlueprintPure, Category = "OnlyHands|AI|Reaction")
	static FStrikePredictionResult EvaluateStrikePrediction(
		AActor* Attacker,
		AActor* Target,
		float Radius = 200.f,
		float ArcAngleDegrees = 90.f,
		float PredictTime = 0.2f
	);


	
#pragma endregion


#pragma region Animation

/** Uses custom convex hull + GJK/EPA to detect contact during anim notify using a bone chain */
	UFUNCTION(BlueprintCallable, Category = "OnlyHands|Animation|Collision")
	static bool SweepBoneChainWithMetrics(
		USkeletalMeshComponent* MeshComp,
		const TArray<FName>& BoneNames,
		AActor* SelfActor,
		TArray<FStrikeContactMetrics>& OutContacts,
		TArray<ACharacter*>& OutHitCharacters,
		bool bUseEPA = true,
		bool bDrawDebug = false,
		float DeltaTime = 0.016f
	);




#pragma endregion
	
};



