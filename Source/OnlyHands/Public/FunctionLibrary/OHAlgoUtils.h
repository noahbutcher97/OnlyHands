#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Data/Struct/OHPhysicsStructs.h"
#include "OHAlgoUtils.generated.h"







UCLASS(BlueprintType)
class ONLYHANDS_API UOHAlgoUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Returns a stringified, lower-cased name of any enum value */
	template <typename TEnum>
	static FString GetEnumName(TEnum EnumVal)
	{
		return StaticEnum<TEnum>()->GetNameStringByValue(static_cast<int64>(EnumVal)).ToLower();
	}

	/** Scores how semantically similar two enum values are based on naming */
	template <typename TEnumA, typename TEnumB>
	static float ScoreEnumSimilarity(TEnumA A, TEnumB B)
	{
		const FString AStr = GetEnumName(A);
		const FString BStr = GetEnumName(B);

		if (AStr.Equals(BStr)) return 2.0f;
		if (AStr.Contains(BStr) || BStr.Contains(AStr)) return 1.5f;
		return 0.0f;
	}

	/** Returns the number of parent bones above a given bone in the hierarchy */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo")
	static int32 GetBoneHierarchyDepth(const USkeletalMeshComponent* Mesh, FName Bone);


#pragma region IntegrationUtils

	/** Integrates using Semi-Implicit Euler method */
	static FOHMotionSample IntegrateSemiImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime);

	/** Verlet method requiring previous position */
	static FOHMotionSample IntegrateVerletSample(const FOHMotionSample& Current, const FVector& PreviousPosition, float DeltaTime);

	/** Velocity Verlet method requiring next acceleration */
	static FOHMotionSample IntegrateVelocityVerletSample(const FOHMotionSample& Current, const FVector& NextAcceleration, float DeltaTime);

	/** RK4 using externally supplied acceleration function */
	static FOHMotionSample IntegrateRK4Sample(
		const FOHMotionSample& Current,
		TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
		float DeltaTime);


	/** Implicit Euler Integrator */
	static FOHMotionSample IntegrateImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime);

	/** Heun’s Method (Improved Euler) */
	static FOHMotionSample IntegrateHeunSample(
		const FOHMotionSample& Current,
		TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
		float DeltaTime);

	/** Adaptive Step RK4 (fixed min/max dt) */
	static FOHMotionSample IntegrateAdaptiveRK4Sample(
		const FOHMotionSample& Current,
		TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
		float& InOutDeltaTime,
		float MinStep = 0.0001f,
		float MaxStep = 0.033f,
		float Tolerance = 0.01f);

	/** Critically Damped Spring Integrator */
	static FOHMotionSample IntegrateCriticallyDampedSpringSample(
		const FOHMotionSample& Current,
		const FVector& TargetPosition,
		float AngularFreq,
		float DeltaTime);

	/** Exponential Integration */
	static FOHMotionSample IntegrateExponentialSample(
		const FOHMotionSample& Current,
		float DampingRatio,
		float DeltaTime);

	/** Spring-based positional blend (targeting) */
	static FOHMotionSample IntegrateSpringBlendSample(
		const FOHMotionSample& Current,
		const FVector& TargetPosition,
		float BlendAlpha,
		float SpringCoeff,
		float DeltaTime);

#pragma endregion

#pragma region SolverUtils

	/** Computes the Baumgarte stabilization term for constraint error resolution */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Solvers")
	static float ComputeBaumgarteTerm(float ConstraintError, float Beta, float DeltaTime);

	/** Iteratively projects a bone chain toward a target position using spring-like correction */
	static void ProjectChainToTargetPosition(
		const TArray<FName>& ChainBones,
		TMap<FName, FOHBoneData>& BoneMap,
		const FVector& TargetPosition,
		float Stiffness = 1.0f,
		int32 Iterations = 3);

	/** Iteratively rotates a bone chain toward a target rotation */
	static void ProjectChainToTargetRotation(
		const TArray<FName>& ChainBones,
		TMap<FName, FOHBoneData>& BoneMap,
		const FQuat& TargetRotation,
		float Stiffness = 1.0f,
		int32 Iterations = 3);

	/** Projects a chain toward both a target position and target rotation */
	static void ProjectChainToTargetTransform(
		const TArray<FName>& ChainBones,
		TMap<FName, FOHBoneData>& BoneMap,
		const FTransform& Target,
		float PosStiffness = 1.0f,
		float RotStiffness = 1.0f,
		int32 Iterations = 3);

	/** Projects a chain to a target transform and pushes the result as motion samples */
	static void SolveAndPushChainToTargetTransform(
		const TArray<FName>& ChainBones,
		TMap<FName, FOHBoneData>& BoneMap,
		const FTransform& Target,
		float PosStiffness,
		float RotStiffness,
		int32 Iterations,
		float TimeStamp);



#pragma endregion


#pragma region MotionEstimators

	UFUNCTION(BlueprintPure, Category = "OH|Physics|Motion")
	static FVector ComputeLinearJerkFromSamples(
		const FOHMotionSample& Newer,
		const FOHMotionSample& Older);

	UFUNCTION(BlueprintPure, Category = "OH|Physics|Motion")
	static FVector ComputeAngularJerkFromSamples(
		const FOHMotionSample& Newer,
		const FOHMotionSample& Older);

	/** Estimates jerk (d/dt of acceleration) from a series of motion samples */
	static FVector EstimateJerk(const TArray<FOHMotionSample>& Samples, float DeltaTime);

	/** Estimates curvature of a trajectory from recent motion samples */
	static float EstimateCurvature(const TArray<FOHMotionSample>& Samples);

	/** Exponential moving average velocity smoother */
	static FVector SmoothVelocityEMA(const TArray<FOHMotionSample>& Samples, float Alpha);

	/** Detects if velocity reversed direction (zero crossing) */
	static bool IsVelocityZeroCrossing(const TArray<FOHMotionSample>& Samples);

	/** Measures deviation from a straight-line trajectory */
	static float GetTrajectoryDeviation(const TArray<FOHMotionSample>& Samples);

#pragma endregion


#pragma region MotionPredictors

	UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Prediction")
	static TArray<FVector> GetCubicBezierControlPointsFromBoneData(
		const FOHBoneData& BoneData,
		float PredictTime = 0.3f);

	UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Prediction")
	static TArray<FVector> GetQuadraticBezierControlPointsFromBoneData(
		const FOHBoneData& BoneData,
		float PredictTime = 0.3f);

	
	/** Predicts future position using linear extrapolation */
	static FVector PredictLinear(const FOHMotionSample& Current, float PredictTime);

	/** Predicts future position using quadratic extrapolation (includes acceleration) */
	static FVector PredictQuadratic(const FOHMotionSample& Current, float PredictTime);

	/** Predicts future position with velocity damping */
	static FVector PredictDamped(const FOHMotionSample& Current, float PredictTime, float DampingRatio = 3.0f);

	/** Predicts a future position along the curved path from recent samples */
	static FVector PredictCurvedFromSamples(const TArray<FOHMotionSample>& Samples, float PredictTime);

	/** Samples a point along a quadratic Bezier defined by three points */
	static FVector SampleBezierQuadratic(const FVector& P0, const FVector& P1, const FVector& P2, float T);

	/** Samples a point along a cubic Bezier defined by four control points */
	static FVector SampleBezierCubic(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& P3, float T);

	/** Estimates time to reach a target point under linear velocity */
	static float EstimateTimeToReachTargetLinear(const FOHMotionSample& Current, const FVector& TargetPosition);

	/** Estimates time until stop under linear damping */
	static float EstimateTimeToStopDamped(const FOHMotionSample& Current, float DampingRatio = 3.0f);


#pragma endregion

#pragma region FeedbackControllers

	/** Simple stateless PID control output */
	static FVector PIDControl(
		const FVector& Error,
		const FVector& ErrorIntegral,
		const FVector& ErrorDerivative,
		float Kp,
		float Ki,
		float Kd,
		float MaxOutput = BIG_NUMBER);

#pragma endregion


#pragma region ControlAndStabilization

	/** Returns critically damped spring acceleration */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Control")
	static FVector ComputeCriticallyDampedSpring(const FVector& Pos, const FVector& Vel, const FVector& TargetPos, float Stiffness, float Damping);

	/** Applies Baumgarte stabilization */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Stabilization")
	static float ComputeBaumgarteStabilization(float ConstraintError, float Beta, float DeltaTime);

	/** Applies a deadzone filter */
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Filter")
	static FVector ApplyDeadzone(const FVector& Value, float Threshold);

#pragma endregion

#pragma region NameResolution

	static int32 LevenshteinDistance(const FString& A, const FString& B);
	static int32 DamerauLevenshteinDistance(const FString& A, const FString& B);
	static float JaroWinklerDistance(const FString& A, const FString& B);
	static float JaccardIndex(const FString& A, const FString& B);
	static FOHNameMatchResult FindBestNameMatch(const FString& Target, const TArray<FString>& Candidates);


	/** Returns a phonetic Soundex code from input string */
	static FString SoundexCode(const FString& Input);

	/** Returns normalized bigram overlap score between two strings */
	static float BigramOverlap(const FString& A, const FString& B);

	/** Simplified Metaphone-like phonetic encoding */
	static FString MetaphoneCode(const FString& Input);

	/** Computes approximate TF-IDF cosine similarity between two terms */
	static float CosineTFIDFSimilarity(const FString& A, const FString& B, const TArray<FString>& Corpus);


	static EOHNameMatchingStrategy DetermineBestMatchingStrategy(
	const FString& Target,
	const TArray<FString>& Candidates);
	
	static FOHNameMatchResult FindBestNameMatchUsingStrategy(
	const FString& Target,
	const TArray<FString>& Candidates,
	EOHNameMatchingStrategy Strategy);

	/** Matches multiple target strings to the best candidates using the selected strategy */
	static TArray<FOHNameMatchResult> BatchMatchTargetsToCandidates(
		const TArray<FString>& Targets,
		const TArray<FString>& Candidates,
		EOHNameMatchingStrategy Strategy);

	static TMap<FString, FOHNameMatchResult> MatchTargetAcrossSets(
		const FString& Target,
		const TMap<FString, TArray<FString>>& CandidateSetsByLabel,
		EOHNameMatchingStrategy Strategy);

	static TMap<FString, FOHNameMatchResult> BatchAutoMatch(
		const TArray<FString>& Targets,
		const TArray<FString>& Candidates);

	
	static TArray<FOHNameMatchResult> ScoreMatchesAgainstCandidates(
		const FString& Target,
		const TArray<FString>& Candidates,
		EOHNameMatchingStrategy Strategy);

	static TArray<FOHNameMatchResult> RankFNameToEnumMap(
		const FString& Target,
		const TMap<FName, EOHSkeletalBone>& Map,
		EOHNameMatchingStrategy Strategy,
		float MinScoreThreshold = 0.0f);

	// Matching score helpers
	static float GetNormalizedLevenshteinScore(const FString& A, const FString& B);
	
	static float GetContainsScore(const FString& A, const FString& B);
	
	static float GetPrefixScore(const FString& A, const FString& B);

	static float GetSuffixScore(const FString& A, const FString& B);
	
	static float GetExactScore(const FString& A, const FString& B);

	static FOHNameMatchResult FindBestNameMatchAutoStrategy(const FString& Target, const TArray<FString>& Candidates);

	static FOHNameMatchResult FindBestNameMatchAutoStrategy(const FName& InTarget, const TArray<FName>& Candidates);

	static const TArray<EOHNameMatchingStrategy>& GetAllMatchingStrategies();

	
private:
	static float GetStrategyWeight(EOHNameMatchingStrategy Strategy);
	static bool CandidatesAreTokenized(const TArray<FString>& Candidates);
	static bool CandidatesUseNamingConvention(const TArray<FString>& Candidates);
	
#pragma endregion

#pragma region DistanceFormulas

	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Distance")
	static float ManhattanDistance(const FVector& A, const FVector& B);

	
	UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Distance")
	static float EuclideanDistance(const FVector& A, const FVector& B);


#pragma endregion

#pragma region Debug

	UFUNCTION(BlueprintCallable, Category = "OH|Locomotion|Debug")
	void DrawDebugQuadraticBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps, FLinearColor Color, float Thickness, float Duration);
	
	UFUNCTION(BlueprintCallable, Category = "OH|Locomotion|Debug")
	void DrawDebugCubicBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps, FLinearColor Color, float Thickness, float Duration);


#pragma endregion
};





