

#include "FunctionLibrary/OHAlgoUtils.h"
#include "OHPhysicsStructs.h"
#include "Components/SkeletalMeshComponent.h"
#include "Animation/Skeleton.h"

int32 UOHAlgoUtils::GetBoneHierarchyDepth(const USkeletalMeshComponent* Mesh, FName Bone) {
    if (!Mesh || Bone.IsNone())
        return 0;

    const int32 Index = Mesh->GetBoneIndex(Bone);
    if (Index == INDEX_NONE)
        return 0;

    const FReferenceSkeleton& Ref = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
    int32 Depth = 0;
    int32 Parent = Ref.GetParentIndex(Index);

    while (Parent != INDEX_NONE) {
        ++Depth;
        Parent = Ref.GetParentIndex(Parent);
    }
    return Depth;
}

#pragma region IntegrationUtils

FOHMotionSample UOHAlgoUtils::IntegrateSemiImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime) {
    const FVector NewVel = Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime;
    const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateVerletSample(const FOHMotionSample& Current, const FVector& PreviousPosition,
                                                    float DeltaTime) {
    const FVector NewPos =
        2 * Current.GetLocation() - PreviousPosition + Current.GetLinearAcceleration() * FMath::Square(DeltaTime);
    const FVector NewVel = (NewPos - PreviousPosition) / (2 * DeltaTime);

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateVelocityVerletSample(const FOHMotionSample& Current,
                                                            const FVector& NextAcceleration, float DeltaTime) {
    const FVector NewPos = Current.GetLocation() + Current.GetLinearVelocity() * DeltaTime +
                           0.5f * Current.GetLinearAcceleration() * FMath::Square(DeltaTime);
    const FVector NewVel =
        Current.GetLinearVelocity() + 0.5f * (Current.GetLinearAcceleration() + NextAcceleration) * DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), NextAcceleration,
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateRK4Sample(const FOHMotionSample& Current,
                                                 TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                 float DeltaTime) {
    const FVector x0 = Current.GetLocation();
    const FVector v0 = Current.GetLinearVelocity();
    const FVector a0 = AccelFunc(x0, v0);

    const FVector v1 = v0 + 0.5f * a0 * DeltaTime;
    const FVector x1 = x0 + 0.5f * v0 * DeltaTime;
    const FVector a1 = AccelFunc(x1, v1);

    const FVector v2 = v0 + 0.5f * a1 * DeltaTime;
    const FVector x2 = x0 + 0.5f * v1 * DeltaTime;
    const FVector a2 = AccelFunc(x2, v2);

    const FVector v3 = v0 + a2 * DeltaTime;
    const FVector x3 = x0 + v2 * DeltaTime;
    const FVector a3 = AccelFunc(x3, v3);

    const FVector FinalPos = x0 + (DeltaTime / 6.f) * (v0 + 2.f * v1 + 2.f * v2 + v3);
    const FVector FinalVel = v0 + (DeltaTime / 6.f) * (a0 + 2.f * a1 + 2.f * a2 + a3);
    const FVector FinalAccel = AccelFunc(FinalPos, FinalVel);

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), FinalPos), FinalVel,
                                            Current.GetAngularVelocity(), FinalAccel, Current.GetAngularAcceleration(),
                                            Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime) {
    const FVector NewPos =
        Current.GetLocation() + (Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime) * DeltaTime;
    const FVector NewVel = Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateCriticallyDampedSpringSample(const FOHMotionSample& Current,
                                                                    const FVector& TargetPosition, float AngularFreq,
                                                                    float DeltaTime) {
    const float Omega = FMath::Clamp(AngularFreq, 1.f, 100.f);
    const float Exp = FMath::Exp(-Omega * DeltaTime);
    const FVector Displacement = Current.GetLocation() - TargetPosition;
    const FVector NewPos = TargetPosition + (Displacement + Current.GetLinearVelocity() / Omega) * Exp;
    const FVector NewVel = (NewPos - Current.GetLocation()) / DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateAdaptiveRK4Sample(const FOHMotionSample& Current,
                                                         TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                         float& InOutDeltaTime, float MinStep, float MaxStep,
                                                         float Tolerance) {
    // Perform two small steps
    const float HalfStep = InOutDeltaTime * 0.5f;
    FOHMotionSample Mid = IntegrateRK4Sample(Current, AccelFunc, HalfStep);
    FOHMotionSample Small = IntegrateRK4Sample(Mid, AccelFunc, HalfStep);

    // One large step
    FOHMotionSample Large = IntegrateRK4Sample(Current, AccelFunc, InOutDeltaTime);

    const float Error = FVector::Dist(Small.GetLocation(), Large.GetLocation());
    if (Error > Tolerance && InOutDeltaTime > MinStep)
        InOutDeltaTime *= 0.5f;
    else if (Error < Tolerance * 0.25f && InOutDeltaTime < MaxStep)
        InOutDeltaTime *= 2.0f;

    return Small;
}

FOHMotionSample UOHAlgoUtils::IntegrateHeunSample(const FOHMotionSample& Current,
                                                  TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                  float DeltaTime) {
    const FVector x0 = Current.GetLocation();
    const FVector v0 = Current.GetLinearVelocity();
    const FVector a0 = AccelFunc(x0, v0);

    const FVector v1 = v0 + a0 * DeltaTime;
    const FVector x1 = x0 + v0 * DeltaTime;
    const FVector a1 = AccelFunc(x1, v1);

    const FVector FinalPos = x0 + 0.5f * (v0 + v1) * DeltaTime;
    const FVector FinalVel = v0 + 0.5f * (a0 + a1) * DeltaTime;
    const FVector FinalAccel = AccelFunc(FinalPos, FinalVel);

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), FinalPos), FinalVel,
                                            Current.GetAngularVelocity(), FinalAccel, Current.GetAngularAcceleration(),
                                            Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateExponentialSample(const FOHMotionSample& Current, float DampingRatio,
                                                         float DeltaTime) {
    const float ExpFactor = FMath::Exp(-FMath::Abs(DampingRatio) * DeltaTime);
    const FVector NewVel = Current.GetLinearVelocity() * ExpFactor;
    const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateSpringBlendSample(const FOHMotionSample& Current, const FVector& TargetPosition,
                                                         float BlendAlpha, float SpringCoeff, float DeltaTime) {
    const FVector BlendedTarget = FMath::Lerp(Current.GetLocation(), TargetPosition, BlendAlpha);
    const FVector SpringForce = -SpringCoeff * (Current.GetLocation() - BlendedTarget);
    const FVector NewVel = Current.GetLinearVelocity() + SpringForce * DeltaTime;
    const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;

    return FOHMotionSample::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                            Current.GetAngularVelocity(),
                                            SpringForce, // reuse as proxy accel
                                            Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
}

#pragma endregion

#pragma region SolverUtils

float UOHAlgoUtils::ComputeBaumgarteTerm(float ConstraintError, float Beta, float DeltaTime) {
    return -(Beta / DeltaTime) * ConstraintError;
}

void UOHAlgoUtils::ProjectChainToTargetPosition(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                                const FVector& TargetPosition, float Stiffness, int32 Iterations) {
    if (ChainBones.Num() < 2 || Stiffness <= 0.f || Iterations <= 0)
        return;

    for (int32 It = 0; It < Iterations; ++It) {
        FVector Goal = TargetPosition;

        for (int32 i = ChainBones.Num() - 1; i >= 0; --i) {
            FName Bone = ChainBones[i];
            if (!BoneMap.Contains(Bone))
                continue;

            FOHBoneData& Data = BoneMap[Bone];
            const FVector Pos = Data.GetCurrentPosition();
            const FVector ToGoal = Goal - Pos;

            const FVector Correction = ToGoal * FMath::Clamp(Stiffness, 0.f, 1.f);
            Data.SetCurrentPosition(Pos + Correction);

            Goal = Data.GetCurrentPosition(); // propagate up
        }
    }
}

void UOHAlgoUtils::ProjectChainToTargetRotation(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                                const FQuat& TargetRotation, float Stiffness, int32 Iterations) {
    if (ChainBones.Num() < 2 || Stiffness <= 0.f || Iterations <= 0)
        return;

    for (int32 It = 0; It < Iterations; ++It) {
        FQuat Goal = TargetRotation;

        for (int32 i = ChainBones.Num() - 1; i >= 0; --i) {
            FName Bone = ChainBones[i];
            if (!BoneMap.Contains(Bone))
                continue;

            FOHBoneData& Data = BoneMap[Bone];
            const FQuat Rot = Data.GetCurrentRotation();

            const FQuat Delta = Goal * Rot.Inverse();
            const FQuat Adjusted = FQuat::Slerp(FQuat::Identity, Delta, FMath::Clamp(Stiffness, 0.f, 1.f));
            Data.SetCurrentRotation((Adjusted * Rot).GetNormalized());

            Goal = Data.GetCurrentRotation(); // propagate up
        }
    }
}

void UOHAlgoUtils::ProjectChainToTargetTransform(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                                 const FTransform& Target, float PosStiffness, float RotStiffness,
                                                 int32 Iterations) {
    if (ChainBones.Num() < 2 || Iterations <= 0)
        return;

    for (int32 It = 0; It < Iterations; ++It) {
        FVector GoalPos = Target.GetLocation();
        FQuat GoalRot = Target.GetRotation();

        for (int32 i = ChainBones.Num() - 1; i >= 0; --i) {
            FName Bone = ChainBones[i];
            if (!BoneMap.Contains(Bone))
                continue;

            FOHBoneData& Data = BoneMap[Bone];

            // --- Position ---
            const FVector Pos = Data.GetCurrentPosition();
            const FVector ToGoal = GoalPos - Pos;
            const FVector PosCorrection = ToGoal * FMath::Clamp(PosStiffness, 0.f, 1.f);
            Data.SetCurrentPosition(Pos + PosCorrection);
            GoalPos = Data.GetCurrentPosition();

            // --- Rotation ---
            const FQuat Rot = Data.GetCurrentRotation();
            const FQuat Delta = GoalRot * Rot.Inverse();
            const FQuat Adjusted = FQuat::Slerp(FQuat::Identity, Delta, FMath::Clamp(RotStiffness, 0.f, 1.f));
            Data.SetCurrentRotation((Adjusted * Rot).GetNormalized());
            GoalRot = Data.GetCurrentRotation();
        }
    }
}

void UOHAlgoUtils::SolveAndPushChainToTargetTransform(const TArray<FName>& ChainBones,
                                                      TMap<FName, FOHBoneData>& BoneMap, const FTransform& Target,
                                                      float PosStiffness, float RotStiffness, int32 Iterations,
                                                      float TimeStamp) {
    if (ChainBones.Num() < 2 || Iterations <= 0)
        return;

    // Step 1: Solve
    ProjectChainToTargetTransform(ChainBones, BoneMap, Target, PosStiffness, RotStiffness, Iterations);

    // Step 2: Push solved transforms into each bone
    for (const FName& Bone : ChainBones) {
        if (!BoneMap.Contains(Bone))
            continue;

        FOHBoneData& BoneData = BoneMap[Bone];
        const FVector Pos = BoneData.GetCurrentPosition();
        const FQuat Rot = BoneData.GetCurrentRotation();

        const FOHMotionSample SolvedSample = FOHMotionSample::CreateFromState(
            FTransform(Rot, Pos), BoneData.GetBodyLinearVelocity(), BoneData.GetBodyAngularVelocity(),
            BoneData.GetLinearAcceleration(), BoneData.GetAngularAcceleration(), TimeStamp);

        BoneData.PushMotionSample(SolvedSample);
    }
}

#pragma endregion

#pragma region MotionEstimators

FVector UOHAlgoUtils::ComputeLinearJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older) {
    const float DeltaTime = FMath::Max(Newer.GetTimeStamp() - Older.GetTimeStamp(), KINDA_SMALL_NUMBER);
    const FVector DeltaAccel = Newer.GetLinearAcceleration() - Older.GetLinearAcceleration();
    return DeltaAccel / DeltaTime;
}

FVector UOHAlgoUtils::ComputeAngularJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older) {
    const float DeltaTime = FMath::Max(Newer.GetTimeStamp() - Older.GetTimeStamp(), KINDA_SMALL_NUMBER);
    const FVector DeltaAccel = Newer.GetAngularAcceleration() - Older.GetAngularAcceleration();
    return DeltaAccel / DeltaTime;
}

FVector UOHAlgoUtils::EstimateJerk(const TArray<FOHMotionSample>& Samples, float DeltaTime) {
    if (Samples.Num() < 3 || DeltaTime <= 0.f)
        return FVector::ZeroVector;

    const FVector A0 = Samples[Samples.Num() - 3].GetLinearAcceleration();
    const FVector A1 = Samples[Samples.Num() - 2].GetLinearAcceleration();
    const FVector A2 = Samples[Samples.Num() - 1].GetLinearAcceleration();

    const FVector J0 = (A1 - A0) / DeltaTime;
    const FVector J1 = (A2 - A1) / DeltaTime;

    return (J1 - J0) / DeltaTime; // dJ/dt
}

float UOHAlgoUtils::EstimateCurvature(const TArray<FOHMotionSample>& Samples) {
    if (Samples.Num() < 3)
        return 0.f;

    const FVector P0 = Samples[Samples.Num() - 3].GetLocation();
    const FVector P1 = Samples[Samples.Num() - 2].GetLocation();
    const FVector P2 = Samples[Samples.Num() - 1].GetLocation();

    const FVector A = P1 - P0;
    const FVector B = P2 - P1;

    const float Cross = FVector::CrossProduct(A, B).Size();

    const float Denominator = A.Size() * B.Size();
    if (Denominator <= SMALL_NUMBER)
        return 0.f;

    return Cross / FMath::Square(Denominator);
}

FVector UOHAlgoUtils::SmoothVelocityEMA(const TArray<FOHMotionSample>& Samples, float Alpha) {
    if (Samples.Num() < 1)
        return FVector::ZeroVector;

    FVector Smoothed = Samples[0].GetLinearVelocity();
    for (int32 i = 1; i < Samples.Num(); ++i) {
        Smoothed = FMath::Lerp(Smoothed, Samples[i].GetLinearVelocity(), Alpha);
    }
    return Smoothed;
}

bool UOHAlgoUtils::IsVelocityZeroCrossing(const TArray<FOHMotionSample>& Samples) {
    if (Samples.Num() < 2)
        return false;

    const FVector V0 = Samples[Samples.Num() - 2].GetLinearVelocity();
    const FVector V1 = Samples[Samples.Num() - 1].GetLinearVelocity();

    return (V0.X * V1.X < 0) || (V0.Y * V1.Y < 0) || (V0.Z * V1.Z < 0);
}

float UOHAlgoUtils::GetTrajectoryDeviation(const TArray<FOHMotionSample>& Samples) {
    if (Samples.Num() < 3)
        return 0.f;

    const FVector Start = Samples[0].GetLocation();
    const FVector End = Samples.Last().GetLocation();
    const FVector Line = End - Start;

    float MaxDeviation = 0.f;
    for (int32 i = 1; i < Samples.Num() - 1; ++i) {
        const FVector Point = Samples[i].GetLocation();
        const FVector ToLine = FVector::PointPlaneProject(Point, Start, Line.GetSafeNormal());
        MaxDeviation = FMath::Max(MaxDeviation, FVector::Dist(Point, ToLine));
    }
    return MaxDeviation;
}

#pragma endregion

#pragma region MotionPredictors

FVector UOHAlgoUtils::PredictLinear(const FOHMotionSample& Current, float PredictTime) {
    return Current.GetLocation() + Current.GetLinearVelocity() * PredictTime;
}

FVector UOHAlgoUtils::PredictQuadratic(const FOHMotionSample& Current, float PredictTime) {
    return Current.GetLocation() + Current.GetLinearVelocity() * PredictTime +
           0.5f * Current.GetLinearAcceleration() * FMath::Square(PredictTime);
}

FVector UOHAlgoUtils::PredictDamped(const FOHMotionSample& Current, float PredictTime, float DampingRatio) {
    const float Decay = FMath::Exp(-FMath::Abs(DampingRatio) * PredictTime);
    const FVector DampedVel = Current.GetLinearVelocity() * Decay;
    return Current.GetLocation() + DampedVel * PredictTime;
}

FVector UOHAlgoUtils::PredictCurvedFromSamples(const TArray<FOHMotionSample>& Samples, float PredictTime) {
    if (Samples.Num() < 3)
        return FVector::ZeroVector;

    const FVector P0 = Samples[Samples.Num() - 3].GetLocation();
    const FVector P1 = Samples[Samples.Num() - 2].GetLocation();
    const FVector P2 = Samples[Samples.Num() - 1].GetLocation();

    // Estimate next point using velocity direction
    const FVector Forward = (P2 - P1).GetSafeNormal();
    const FVector EstControl = P2 + Forward * (P2 - P1).Size();

    // Scale T relative to sample spacing
    const float T = FMath::Clamp(PredictTime / (Samples[1].GetTimeStamp() - Samples[0].GetTimeStamp()), 0.f, 1.f);
    return SampleBezierQuadratic(P0, EstControl, P2, T);
}

FVector UOHAlgoUtils::SampleBezierQuadratic(const FVector& P0, const FVector& P1, const FVector& P2, float T) {
    const FVector A = FMath::Lerp(P0, P1, T);
    const FVector B = FMath::Lerp(P1, P2, T);
    return FMath::Lerp(A, B, T);
}

FVector UOHAlgoUtils::SampleBezierCubic(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& P3,
                                        float T) {
    const FVector A = FMath::Lerp(P0, P1, T);
    const FVector B = FMath::Lerp(P1, P2, T);
    const FVector C = FMath::Lerp(P2, P3, T);

    const FVector AB = FMath::Lerp(A, B, T);
    const FVector BC = FMath::Lerp(B, C, T);

    return FMath::Lerp(AB, BC, T);
}

TArray<FVector> UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(const FOHBoneData& BoneData,
                                                                          float PredictTime) {
    const FVector P0 = BoneData.GetCurrentPosition();
    const FVector P2 = PredictQuadratic(BoneData.GetLatestMotionSample(), PredictTime);
    const FVector P1 = P0 + BoneData.GetBodyLinearVelocity() * (PredictTime * 0.5f);

    return {P0, P1, P2};
}

TArray<FVector> UOHAlgoUtils::GetCubicBezierControlPointsFromBoneData(const FOHBoneData& BoneData, float PredictTime) {
    const FVector P0 = BoneData.GetCurrentPosition();
    const FVector P3 = PredictQuadratic(BoneData.GetLatestMotionSample(), PredictTime);
    const FVector P1 = P0 + BoneData.GetBodyLinearVelocity() * (PredictTime / 3.f);
    const FVector P2 = P3 - BoneData.GetBodyLinearVelocity() * (PredictTime / 3.f);

    return {P0, P1, P2, P3};
}

float UOHAlgoUtils::EstimateTimeToReachTargetLinear(const FOHMotionSample& Current, const FVector& TargetPosition) {
    const FVector Dir = TargetPosition - Current.GetLocation();
    const float Speed = Current.GetLinearVelocity().Size();
    if (Speed <= SMALL_NUMBER)
        return -1.f;

    const float Distance = FVector::DotProduct(Dir, Current.GetLinearVelocity().GetSafeNormal());
    return Distance / Speed;
}

float UOHAlgoUtils::EstimateTimeToStopDamped(const FOHMotionSample& Current, float DampingRatio) {
    const float Speed = Current.GetLinearVelocity().Size();
    if (Speed <= SMALL_NUMBER || DampingRatio <= SMALL_NUMBER)
        return 0.f;

    // Solve v(t) = v0 * exp(-λt) → t = ln(v0 / v(t)) / λ
    // Assume threshold v(t) ~ 1 unit/s
    const float Threshold = 1.f;
    return FMath::Max(0.f, FMath::Loge(Speed / Threshold) / DampingRatio);
}

#pragma endregion

#pragma region FeedbackControllers

FVector UOHAlgoUtils::PIDControl(const FVector& Error, const FVector& ErrorIntegral, const FVector& ErrorDerivative,
                                 float Kp, float Ki, float Kd, float MaxOutput) {
    const FVector Output = Kp * Error + Ki * ErrorIntegral + Kd * ErrorDerivative;

    return Output.GetClampedToMaxSize(MaxOutput);
}

#pragma endregion

#pragma region ControlAndStabilization

FVector UOHAlgoUtils::ComputeCriticallyDampedSpring(const FVector& Pos, const FVector& Vel, const FVector& TargetPos,
                                                    float Stiffness, float Damping) {
    FVector Displacement = TargetPos - Pos;
    return Stiffness * Displacement - Damping * Vel;
}

float UOHAlgoUtils::ComputeBaumgarteStabilization(float ConstraintError, float Beta, float DeltaTime) {
    return -(Beta / DeltaTime) * ConstraintError;
}

FVector UOHAlgoUtils::ApplyDeadzone(const FVector& Value, float Threshold) {
    return FVector(FMath::Abs(Value.X) > Threshold ? Value.X : 0.f, FMath::Abs(Value.Y) > Threshold ? Value.Y : 0.f,
                   FMath::Abs(Value.Z) > Threshold ? Value.Z : 0.f);
}

#pragma endregion

#pragma region NameResolution

int32 UOHAlgoUtils::LevenshteinDistance(const FString& A, const FString& B) {
    const int32 LenA = A.Len(), LenB = B.Len();
    TArray<TArray<int32>> DP;
    DP.SetNum(LenA + 1);
    for (int32 i = 0; i <= LenA; ++i)
        DP[i].SetNumZeroed(LenB + 1);

    for (int32 i = 0; i <= LenA; ++i)
        DP[i][0] = i;
    for (int32 j = 0; j <= LenB; ++j)
        DP[0][j] = j;

    for (int32 i = 1; i <= LenA; ++i)
        for (int32 j = 1; j <= LenB; ++j)
            DP[i][j] =
                FMath::Min3(DP[i - 1][j] + 1, DP[i][j - 1] + 1, DP[i - 1][j - 1] + (A[i - 1] == B[j - 1] ? 0 : 1));

    return DP[LenA][LenB];
}

int32 UOHAlgoUtils::DamerauLevenshteinDistance(const FString& A, const FString& B) {
    const int32 LenA = A.Len(), LenB = B.Len();
    TArray<TArray<int32>> DP;
    DP.SetNum(LenA + 1);
    for (int32 i = 0; i <= LenA; ++i)
        DP[i].SetNumZeroed(LenB + 1);

    for (int32 i = 0; i <= LenA; ++i)
        DP[i][0] = i;
    for (int32 j = 0; j <= LenB; ++j)
        DP[0][j] = j;

    for (int32 i = 1; i <= LenA; ++i)
        for (int32 j = 1; j <= LenB; ++j) {
            int Cost = (A[i - 1] == B[j - 1]) ? 0 : 1;
            DP[i][j] = FMath::Min3(DP[i - 1][j] + 1, DP[i][j - 1] + 1, DP[i - 1][j - 1] + Cost);

            if (i > 1 && j > 1 && A[i - 1] == B[j - 2] && A[i - 2] == B[j - 1])
                DP[i][j] = FMath::Min(DP[i][j], DP[i - 2][j - 2] + Cost);
        }

    return DP[LenA][LenB];
}

float UOHAlgoUtils::JaroWinklerDistance(const FString& A, const FString& B) {
    const int32 LenA = A.Len(), LenB = B.Len();
    if (LenA == 0 || LenB == 0)
        return 0.f;

    const int MatchDistance = FMath::Max(LenA, LenB) / 2 - 1;
    TArray<bool> AMatched, BMatched;
    AMatched.Init(false, LenA);
    BMatched.Init(false, LenB);

    int Matches = 0;
    for (int32 i = 0; i < LenA; ++i) {
        const int32 Start = FMath::Max(0, i - MatchDistance);
        const int32 End = FMath::Min(i + MatchDistance + 1, LenB);

        for (int32 j = Start; j < End; ++j) {
            if (BMatched[j] || A[i] != B[j])
                continue;
            AMatched[i] = BMatched[j] = true;
            ++Matches;
            break;
        }
    }
    if (Matches == 0)
        return 0.f;

    int Transpositions = 0, K = 0;
    for (int32 i = 0; i < LenA; ++i)
        if (AMatched[i]) {
            while (!BMatched[K])
                ++K;
            if (A[i] != B[K])
                ++Transpositions;
            ++K;
        }

    const float m = Matches;
    float J = (m / LenA + m / LenB + (m - Transpositions / 2.0f) / m) / 3.0f;

    // Winkler prefix scale
    int Prefix = 0;
    for (int32 i = 0; i < FMath::Min3(4, LenA, LenB); ++i)
        if (A[i] == B[i])
            ++Prefix;
        else
            break;

    return J + Prefix * 0.1f * (1.f - J);
}

float UOHAlgoUtils::JaccardIndex(const FString& A, const FString& B) {
    TSet<TCHAR> SetA, SetB;
    for (TCHAR C : A)
        SetA.Add(C);
    for (TCHAR C : B)
        SetB.Add(C);

    TSet<TCHAR> Intersection = SetA.Intersect(SetB);
    TSet<TCHAR> Union = SetA.Union(SetB);

    return Union.Num() > 0 ? static_cast<float>(Intersection.Num()) / Union.Num() : 0.f;
}

FOHNameMatchResult UOHAlgoUtils::FindBestNameMatch(const FString& Target, const TArray<FString>& Candidates) {
    FOHNameMatchResult Best;
    float MaxScore = -1.f;

    for (const FString& C : Candidates) {
        const float Score = JaroWinklerDistance(Target, C); // Use best-for-now
        if (Score > MaxScore) {
            MaxScore = Score;
            Best = FOHNameMatchResult(C, Score, TEXT("JaroWinkler"));
        }
    }
    return Best;
}

FString UOHAlgoUtils::SoundexCode(const FString& Input) {
    if (Input.IsEmpty())
        return FString();

    const TCHAR FirstLetter = FChar::ToUpper(Input[0]);
    FString Encoded = TEXT("");
    Encoded.AppendChar(FirstLetter);

    auto Encode = [](TCHAR C) -> TCHAR {
        switch (FChar::ToUpper(C)) {
        case 'B':
        case 'F':
        case 'P':
        case 'V':
            return '1';
        case 'C':
        case 'G':
        case 'J':
        case 'K':
        case 'Q':
        case 'S':
        case 'X':
        case 'Z':
            return '2';
        case 'D':
        case 'T':
            return '3';
        case 'L':
            return '4';
        case 'M':
        case 'N':
            return '5';
        case 'R':
            return '6';
        default:
            return '0';
        }
    };

    TCHAR LastCode = Encode(Input[0]);
    for (int32 i = 1; i < Input.Len(); ++i) {
        const TCHAR Code = Encode(Input[i]);
        if (Code != '0' && Code != LastCode) {
            Encoded.AppendChar(Code);
            LastCode = Code;
        }
    }

    while (Encoded.Len() < 4)
        Encoded.AppendChar('0');
    if (Encoded.Len() > 4)
        Encoded = Encoded.Left(4);

    return Encoded;
}

float UOHAlgoUtils::BigramOverlap(const FString& A, const FString& B) {
    TSet<FString> BigramsA, BigramsB;

    for (int32 i = 0; i < A.Len() - 1; ++i)
        BigramsA.Add(A.Mid(i, 2));

    for (int32 i = 0; i < B.Len() - 1; ++i)
        BigramsB.Add(B.Mid(i, 2));

    TSet<FString> Intersection = BigramsA.Intersect(BigramsB);
    TSet<FString> Union = BigramsA.Union(BigramsB);

    return Union.Num() > 0 ? static_cast<float>(Intersection.Num()) / Union.Num() : 0.f;
}

FString UOHAlgoUtils::MetaphoneCode(const FString& Input) {
    FString Result;
    FString Word = Input.ToUpper();

    auto IsVowel = [](TCHAR C) { return C == 'A' || C == 'E' || C == 'I' || C == 'O' || C == 'U'; };

    auto Append = [&](TCHAR C) {
        if (Result.IsEmpty() || Result[Result.Len() - 1] != C)
            Result.AppendChar(C);
    };

    if (Word.IsEmpty())
        return Result;

    // Preserve first letter if vowel
    if (IsVowel(Word[0]))
        Result.AppendChar(Word[0]);

    for (int32 i = 0; i < Word.Len(); ++i) {
        const TCHAR C = Word[i];
        const TCHAR Next = (i + 1 < Word.Len()) ? Word[i + 1] : '\0';

        // Handle common phoneme pairs
        if (C == 'P' && Next == 'H') {
            Append('F');
            ++i;
            continue;
        }
        if (C == 'C' && Next == 'H') {
            Append('X');
            ++i;
            continue;
        }
        if (C == 'S' && Next == 'H') {
            Append('X');
            ++i;
            continue;
        }
        if (C == 'G' && Next == 'H') {
            Append('F');
            ++i;
            continue;
        }
        if (C == 'T' && Next == 'H') {
            Append('0');
            ++i;
            continue;
        }

        switch (C) {
        case 'B':
        case 'F':
        case 'P':
        case 'V':
            Append('1');
            break;
        case 'C':
        case 'G':
        case 'J':
        case 'K':
        case 'Q':
        case 'S':
        case 'X':
        case 'Z':
            Append('2');
            break;
        case 'D':
        case 'T':
            Append('3');
            break;
        case 'L':
            Append('4');
            break;
        case 'M':
        case 'N':
            Append('5');
            break;
        case 'R':
            Append('6');
            break;
        default:
            break; // ignore vowels and silent letters
        }
    }

    return Result.Left(4);
}

float UOHAlgoUtils::CosineTFIDFSimilarity(const FString& A, const FString& B, const TArray<FString>& Corpus) {
    TMap<FString, float> TfA, TfB, Idf;

    TSet<FString> StopWords = {TEXT("the"), TEXT("is"), TEXT("a"), TEXT("an"), TEXT("of"), TEXT("in"), TEXT("and")};

    auto Tokenize = [&](const FString& In) -> TArray<FString> {
        TArray<FString> Raw;
        In.ToLower().ParseIntoArray(Raw, TEXT(" "), true);
        TArray<FString> Cleaned;

        for (const FString& Token : Raw) {
            FString Clean = Token;
            Clean.RemoveSpacesInline();
            if (!StopWords.Contains(Clean) && !Clean.IsEmpty())
                Cleaned.Add(Clean);
        }
        return Cleaned;
    };

    TArray<FString> TokensA = Tokenize(A), TokensB = Tokenize(B);

    for (const FString& Token : TokensA)
        TfA.FindOrAdd(Token) += 1.f;
    for (const FString& Token : TokensB)
        TfB.FindOrAdd(Token) += 1.f;

    for (const TPair<FString, float>& Pair : TfA) {
        const FString& Token = Pair.Key;
        int32 DocsWith = 0;
        for (const FString& Doc : Corpus)
            if (Doc.Contains(Token))
                ++DocsWith;

        const float DF = FMath::Max(1.f, static_cast<float>(DocsWith));
        Idf.Add(Token, FMath::Loge(Corpus.Num() / DF));
    }

    float Dot = 0.f, MagA = 0.f, MagB = 0.f;
    for (const auto& KVP : Idf) {
        const FString& Token = KVP.Key;
        const float IDF = KVP.Value;

        const float ValA = TfA.FindRef(Token) * IDF;
        const float ValB = TfB.FindRef(Token) * IDF;

        Dot += ValA * ValB;
        MagA += ValA * ValA;
        MagB += ValB * ValB;
    }

    return (MagA > 0 && MagB > 0) ? Dot / (FMath::Sqrt(MagA) * FMath::Sqrt(MagB)) : 0.f;
}

EOHNameMatchingStrategy UOHAlgoUtils::DetermineBestMatchingStrategy(const FString& Target,
                                                                    const TArray<FString>& Candidates) {
    if (Target.IsEmpty() || Candidates.Num() == 0)
        return EOHNameMatchingStrategy::Levenshtein;

    EOHNameMatchingStrategy BestStrategy = EOHNameMatchingStrategy::Levenshtein;
    float BestWeightedScore = 0.f;

    for (int32 EnumIndex = 0; EnumIndex < StaticEnum<EOHNameMatchingStrategy>()->NumEnums(); ++EnumIndex) {
        EOHNameMatchingStrategy Strategy = static_cast<EOHNameMatchingStrategy>(EnumIndex);
        const TArray<FOHNameMatchResult> Scored = ScoreMatchesAgainstCandidates(Target, Candidates, Strategy);

        if (Scored.Num() == 0)
            continue;

        const float TopScore = Scored[0].Score;
        const float Weighted = TopScore * GetStrategyWeight(Strategy);

        if (Weighted > BestWeightedScore) {
            BestWeightedScore = Weighted;
            BestStrategy = Strategy;
        }
    }

    // Optionally override to force simpler matchers if candidates are trivial
    if (BestWeightedScore < 0.6f) {
        if (CandidatesUseNamingConvention(Candidates))
            return EOHNameMatchingStrategy::Prefix;

        if (CandidatesAreTokenized(Candidates))
            return EOHNameMatchingStrategy::Contains;
    }

    return BestStrategy;
}

FOHNameMatchResult UOHAlgoUtils::FindBestNameMatchUsingStrategy(const FString& Target,
                                                                const TArray<FString>& Candidates,
                                                                EOHNameMatchingStrategy Strategy) {
    FOHNameMatchResult Best;
    float MaxScore = -1.f;

    for (const FString& C : Candidates) {
        float Score = 0.f;

        switch (Strategy) {
        case EOHNameMatchingStrategy::Levenshtein:
            Score = 1.f - static_cast<float>(LevenshteinDistance(Target, C)) /
                              FMath::Max(1, FMath::Max(Target.Len(), C.Len()));
            Best.AlgorithmUsed = TEXT("Levenshtein");
            break;

        case EOHNameMatchingStrategy::DamerauLevenshtein:
            Score = 1.f - static_cast<float>(DamerauLevenshteinDistance(Target, C)) /
                              FMath::Max(1, FMath::Max(Target.Len(), C.Len()));
            Best.AlgorithmUsed = TEXT("Damerau-Levenshtein");
            break;

        case EOHNameMatchingStrategy::JaroWinkler:
            Score = JaroWinklerDistance(Target, C);
            Best.AlgorithmUsed = TEXT("Jaro-Winkler");
            break;

        case EOHNameMatchingStrategy::Jaccard:
            Score = JaccardIndex(Target, C);
            Best.AlgorithmUsed = TEXT("Jaccard");
            break;

        case EOHNameMatchingStrategy::Bigram:
            Score = BigramOverlap(Target, C);
            Best.AlgorithmUsed = TEXT("Bigram");
            break;

        case EOHNameMatchingStrategy::Soundex:
            Score = SoundexCode(Target) == SoundexCode(C) ? 1.f : 0.f;
            Best.AlgorithmUsed = TEXT("Soundex");
            break;

        case EOHNameMatchingStrategy::Metaphone:
            Score = MetaphoneCode(Target) == MetaphoneCode(C) ? 1.f : 0.f;
            Best.AlgorithmUsed = TEXT("Metaphone");
            break;

        case EOHNameMatchingStrategy::CosineTFIDF:
            Score = CosineTFIDFSimilarity(Target, C, Candidates);
            Best.AlgorithmUsed = TEXT("TF-IDF Cosine");
            break;

        case EOHNameMatchingStrategy::Contains:
            Score = GetContainsScore(Target, C);
            Best.AlgorithmUsed = TEXT("Contains");
            break;

        case EOHNameMatchingStrategy::Prefix:
            Score = GetPrefixScore(Target, C);
            Best.AlgorithmUsed = TEXT("Prefix");
            break;

        case EOHNameMatchingStrategy::Suffix:
            Score = GetSuffixScore(Target, C);
            Best.AlgorithmUsed = TEXT("Suffix");
            break;

        case EOHNameMatchingStrategy::Exact:
            Score = GetExactScore(Target, C);
            Best.AlgorithmUsed = TEXT("Exact");
            break;

        default:
            break;
        }

        if (Score > MaxScore) {
            MaxScore = Score;
            Best.Candidate = C;
            Best.Score = Score;
        }
    }

    return Best;
}

TArray<FOHNameMatchResult> UOHAlgoUtils::BatchMatchTargetsToCandidates(const TArray<FString>& Targets,
                                                                       const TArray<FString>& Candidates,
                                                                       EOHNameMatchingStrategy Strategy) {
    TArray<FOHNameMatchResult> Results;
    Results.Reserve(Targets.Num());

    for (const FString& Target : Targets) {
        FOHNameMatchResult Match = FindBestNameMatchUsingStrategy(Target, Candidates, Strategy);
        Results.Add(Match);
    }

    return Results;
}

TMap<FString, FOHNameMatchResult>
UOHAlgoUtils::MatchTargetAcrossSets(const FString& Target, const TMap<FString, TArray<FString>>& CandidateSetsByLabel,
                                    EOHNameMatchingStrategy Strategy) {
    TMap<FString, FOHNameMatchResult> Results;

    for (const auto& Pair : CandidateSetsByLabel) {
        const FString& Label = Pair.Key;
        const TArray<FString>& Candidates = Pair.Value;

        FOHNameMatchResult Match = FindBestNameMatchUsingStrategy(Target, Candidates, Strategy);
        Results.Add(Label, Match);
    }

    return Results;
}

TMap<FString, FOHNameMatchResult> UOHAlgoUtils::BatchAutoMatch(const TArray<FString>& Targets,
                                                               const TArray<FString>& Candidates) {
    TMap<FString, FOHNameMatchResult> Results;

    for (const FString& Target : Targets) {
        EOHNameMatchingStrategy BestStrategy = DetermineBestMatchingStrategy(Target, Candidates);
        FOHNameMatchResult Match = FindBestNameMatchUsingStrategy(Target, Candidates, BestStrategy);
        Results.Add(Target, Match);
    }

    return Results;
}

TArray<FOHNameMatchResult> UOHAlgoUtils::ScoreMatchesAgainstCandidates(const FString& Target,
                                                                       const TArray<FString>& Candidates,
                                                                       EOHNameMatchingStrategy Strategy) {
    TArray<FOHNameMatchResult> Results;

    for (const FString& Candidate : Candidates) {
        float Score = 0.f;

        switch (Strategy) {
        case EOHNameMatchingStrategy::Levenshtein:
            Score = GetNormalizedLevenshteinScore(Target, Candidate);
            break;

        case EOHNameMatchingStrategy::Contains:
            Score = GetContainsScore(Target, Candidate);
            break;

        case EOHNameMatchingStrategy::Prefix:
            Score = GetPrefixScore(Target, Candidate);
            break;

        default:
            continue;
        }

        Results.Emplace(Candidate, Score,
                        StaticEnum<EOHNameMatchingStrategy>()->GetNameStringByValue(static_cast<int64>(Strategy)));
    }

    Results.Sort([](const FOHNameMatchResult& A, const FOHNameMatchResult& B) { return A.Score > B.Score; });

    return Results;
}

TArray<FOHNameMatchResult> UOHAlgoUtils::RankFNameToEnumMap(const FString& Target,
                                                            const TMap<FName, EOHSkeletalBone>& Map,
                                                            EOHNameMatchingStrategy Strategy, float MinScoreThreshold) {
    TArray<FString> Candidates;
    for (const auto& Pair : Map) {
        Candidates.Add(Pair.Key.ToString().ToLower());
    }

    TArray<FOHNameMatchResult> Ranked = ScoreMatchesAgainstCandidates(Target, Candidates, Strategy);
    Ranked.RemoveAll([=](const FOHNameMatchResult& R) { return R.Score < MinScoreThreshold; });
    return Ranked;
}

float UOHAlgoUtils::GetNormalizedLevenshteinScore(const FString& A, const FString& B) {
    const int32 Distance = LevenshteinDistance(A, B);
    const int32 MaxLen = FMath::Max(A.Len(), B.Len());
    return MaxLen > 0 ? 1.0f - static_cast<float>(Distance) / static_cast<float>(MaxLen) : 0.f;
}

float UOHAlgoUtils::GetContainsScore(const FString& A, const FString& B) {
    const FString ALower = A.ToLower();
    const FString BLower = B.ToLower();

    if (BLower.Contains(ALower))
        return 0.9f;
    if (ALower.Contains(BLower))
        return 0.8f;
    return 0.f;
}

float UOHAlgoUtils::GetPrefixScore(const FString& A, const FString& B) {
    const FString ALower = A.ToLower();
    const FString BLower = B.ToLower();
    return BLower.StartsWith(ALower) ? 1.f : 0.f;
}

float UOHAlgoUtils::GetSuffixScore(const FString& A, const FString& B) {
    const FString ALower = A.ToLower();
    const FString BLower = B.ToLower();
    return BLower.EndsWith(ALower) ? 1.f : 0.f;
}

float UOHAlgoUtils::GetExactScore(const FString& A, const FString& B) {
    return A.Equals(B, ESearchCase::IgnoreCase) ? 1.f : 0.f;
}

float UOHAlgoUtils::GetStrategyWeight(EOHNameMatchingStrategy Strategy) {
    switch (Strategy) {
    case EOHNameMatchingStrategy::Prefix:
        return 1.25f;
    case EOHNameMatchingStrategy::Suffix:
        return 1.2f;
    case EOHNameMatchingStrategy::Contains:
        return 1.1f;
    case EOHNameMatchingStrategy::Levenshtein:
        return 0.95f;
    case EOHNameMatchingStrategy::DamerauLevenshtein:
        return 0.9f;
    case EOHNameMatchingStrategy::JaroWinkler:
        return 0.85f;
    case EOHNameMatchingStrategy::Jaccard:
        return 0.8f;
    case EOHNameMatchingStrategy::Bigram:
        return 0.75f;
    case EOHNameMatchingStrategy::Soundex:
        return 0.5f;
    case EOHNameMatchingStrategy::Metaphone:
        return 0.5f;
    case EOHNameMatchingStrategy::CosineTFIDF:
        return 0.4f;
    default:
        return 0.1f;
    }
}

const TArray<EOHNameMatchingStrategy>& UOHAlgoUtils::GetAllMatchingStrategies() {
    static const TArray<EOHNameMatchingStrategy> AllStrategies = {
        EOHNameMatchingStrategy::Exact,       EOHNameMatchingStrategy::Prefix,
        EOHNameMatchingStrategy::Suffix,      EOHNameMatchingStrategy::Contains,
        EOHNameMatchingStrategy::Levenshtein, EOHNameMatchingStrategy::DamerauLevenshtein,
        EOHNameMatchingStrategy::JaroWinkler, EOHNameMatchingStrategy::Jaccard,
        EOHNameMatchingStrategy::Bigram,      EOHNameMatchingStrategy::Soundex,
        EOHNameMatchingStrategy::Metaphone,   EOHNameMatchingStrategy::CosineTFIDF};

    return AllStrategies;
}

bool UOHAlgoUtils::CandidatesAreTokenized(const TArray<FString>& Candidates) {
    int32 Tokenized = 0;
    for (const FString& C : Candidates) {
        if (C.Contains(TEXT("_")) || C.Len() < 6)
            Tokenized++;
    }
    return static_cast<float>(Tokenized) / static_cast<float>(Candidates.Num()) > 0.6f;
}

bool UOHAlgoUtils::CandidatesUseNamingConvention(const TArray<FString>& Candidates) {
    for (const FString& C : Candidates) {
        if (C.StartsWith("ik_") || C.EndsWith("_l") || C.EndsWith("_r"))
            return true;
    }
    return false;
}

FOHNameMatchResult UOHAlgoUtils::FindBestNameMatchAutoStrategy(const FString& Target,
                                                               const TArray<FString>& Candidates) {
    FOHNameMatchResult Best;
    Best.Score = -1.f;

    if (Target.IsEmpty() || Candidates.Num() == 0)
        return Best;

    for (int32 EnumIndex = 0; EnumIndex < StaticEnum<EOHNameMatchingStrategy>()->NumEnums(); ++EnumIndex) {
        const EOHNameMatchingStrategy Strategy = static_cast<EOHNameMatchingStrategy>(EnumIndex);
        const FOHNameMatchResult Match = FindBestNameMatchUsingStrategy(Target, Candidates, Strategy);
        const float Weighted = Match.Score * GetStrategyWeight(Strategy);

        if (Weighted > Best.Score) {
            Best = Match;
            Best.AlgorithmUsed =
                StaticEnum<EOHNameMatchingStrategy>()->GetNameStringByValue(static_cast<int64>(Strategy));
        }
    }

    // Optional override for heuristics
    if (Best.Score < 0.6f) {
        if (CandidatesUseNamingConvention(Candidates)) {
            FOHNameMatchResult Alt =
                FindBestNameMatchUsingStrategy(Target, Candidates, EOHNameMatchingStrategy::Prefix);
            if (Alt.Score > Best.Score) {
                Best = Alt;
                Best.AlgorithmUsed = TEXT("Prefix [Heuristic Override]");
            }
        } else if (CandidatesAreTokenized(Candidates)) {
            FOHNameMatchResult Alt =
                FindBestNameMatchUsingStrategy(Target, Candidates, EOHNameMatchingStrategy::Contains);
            if (Alt.Score > Best.Score) {
                Best = Alt;
                Best.AlgorithmUsed = TEXT("Contains [Heuristic Override]");
            }
        }
    }

    return Best;
}

FOHNameMatchResult UOHAlgoUtils::FindBestNameMatchAutoStrategy(const FName& InTarget, const TArray<FName>& Candidates) {
    TArray<FString> StringCandidates;
    for (const FName& Candidate : Candidates) {
        StringCandidates.Add(Candidate.ToString());
    }

    return FindBestNameMatchAutoStrategy(InTarget.ToString(), StringCandidates);
}

#pragma endregion

#pragma region DistanceFormulas
// Example heuristic functions for FVector nodes
float UOHAlgoUtils::ManhattanDistance(const FVector& A, const FVector& B) {
    return FMath::Abs(A.X - B.X) + FMath::Abs(A.Y - B.Y) + FMath::Abs(A.Z - B.Z);
}

float UOHAlgoUtils::EuclideanDistance(const FVector& A, const FVector& B) {
    return FVector::Dist(A, B);
}

#pragma endregion

#pragma region Debug

void UOHAlgoUtils::DrawDebugQuadraticBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps,
                                            FLinearColor Color, float Thickness, float Duration) {
    if (Points.Num() != 3 || !World)
        return;

    for (int32 i = 0; i < Steps; ++i) {
        const float T0 = static_cast<float>(i) / Steps;
        const float T1 = static_cast<float>(i + 1) / Steps;

        const FVector P0 = SampleBezierQuadratic(Points[0], Points[1], Points[2], T0);
        const FVector P1 = SampleBezierQuadratic(Points[0], Points[1], Points[2], T1);

        DrawDebugLine(World, P0, P1, Color.ToFColor(true), false, Duration, 0, Thickness);
    }
}

void UOHAlgoUtils::DrawDebugCubicBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps,
                                        FLinearColor Color, float Thickness, float Duration) {
    if (Points.Num() != 4 || !World)
        return;

    for (int32 i = 0; i < Steps; ++i) {
        const float T0 = static_cast<float>(i) / Steps;
        const float T1 = static_cast<float>(i + 1) / Steps;

        const FVector P0 = SampleBezierCubic(Points[0], Points[1], Points[2], Points[3], T0);
        const FVector P1 = SampleBezierCubic(Points[0], Points[1], Points[2], Points[3], T1);

        DrawDebugLine(World, P0, P1, Color.ToFColor(true), false, Duration, 0, Thickness);
    }
}

#pragma endregion