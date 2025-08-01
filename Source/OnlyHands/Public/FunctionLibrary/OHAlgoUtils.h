#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Data/Struct/OHPhysicsStructs.h"
#include "OHAlgoUtils.generated.h"

#pragma region Structs
// RK4 integration for high accuracy

#pragma endregion

#pragma region NameSpaces

namespace OHPhysicsConstants {
constexpr float DefaultContactTime = 0.01f;   // 10ms typical impact duration
constexpr float DefaultRestitution = 0.3f;    // Coefficient of restitution for flesh
constexpr float DefaultFriction = 0.4f;       // Kinetic friction for skin
constexpr float GravityAcceleration = 981.0f; // cm/s^2 in Unreal units
constexpr float AirDensity = 0.001225f;       // g/cm^3 at sea level
} // namespace OHPhysicsConstants

#pragma endregion

UCLASS(BlueprintType)
class ONLYHANDS_API UOHAlgoUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

  public:
#pragma region AlgorithmTemplates

    /** Returns a stringified, lower-cased name of any enum value */
    template <typename TEnum> static FString GetEnumName(TEnum EnumVal) {
        return StaticEnum<TEnum>()->GetNameStringByValue(static_cast<int64>(EnumVal)).ToLower();
    }

    /** Scores how semantically similar two enum values are based on naming */
    template <typename TEnumA, typename TEnumB> static float ScoreEnumSimilarity(TEnumA A, TEnumB B) {
        const FString AStr = GetEnumName(A);
        const FString BStr = GetEnumName(B);

        if (AStr.Equals(BStr)) {
            return 2.0f;
        }
        if (AStr.Contains(BStr) || BStr.Contains(AStr)) {
            return 1.5f;
        }
        return 0.0f;
    }

    // -------------------- Motion Estimators --------------------

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector ComputeLinearJerkFromSamples(const SampleType& Newer, const SampleType& Older) {
        const float Dt = FMath::Max(Newer.GetTimeStamp() - Older.GetTimeStamp(), KINDA_SMALL_NUMBER);
        return (Newer.GetLinearAcceleration() - Older.GetLinearAcceleration()) / Dt;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector ComputeLinearJerkFromSamples(const FVector& NewerAccel, float NewerTime,
                                                            const FVector& OlderAccel, float OlderTime) {
        const float Dt = FMath::Max(NewerTime - OlderTime, KINDA_SMALL_NUMBER);
        return (NewerAccel - OlderAccel) / Dt;
    }

    // Jerk calculation for FOHMotionFrameSample
    static FORCEINLINE FVector ComputeLinearJerkFromSamples(const FOHMotionFrameSample& Newer,
                                                            const FOHMotionFrameSample& Older,
                                                            bool bUseLocalSpace = false) {
        const float Dt = FMath::Max(Newer.TimeStamp - Older.TimeStamp, KINDA_SMALL_NUMBER);
        if (bUseLocalSpace) {
            return (Newer.LocalLinearAcceleration - Older.LocalLinearAcceleration) / Dt;
        } else {
            return (Newer.WorldLinearAcceleration - Older.WorldLinearAcceleration) / Dt;
        }
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Motion")
    static FVector ComputeLinearJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector ComputeAngularJerkFromSamples(const SampleType& Newer, const SampleType& Older) {
        const float Dt = FMath::Max(Newer.GetTimeStamp() - Older.GetTimeStamp(), KINDA_SMALL_NUMBER);
        return (Newer.GetAngularAcceleration() - Older.GetAngularAcceleration()) / Dt;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector ComputeAngularJerkFromSamples(const FVector& NewerAngAccel, float NewerTime,
                                                             const FVector& OlderAngAccel, float OlderTime) {
        const float Dt = FMath::Max(NewerTime - OlderTime, KINDA_SMALL_NUMBER);
        return (NewerAngAccel - OlderAngAccel) / Dt;
    }

    // Angular jerk for FOHMotionFrameSample
    static FORCEINLINE FVector ComputeAngularJerkFromSamples(const FOHMotionFrameSample& Newer,
                                                             const FOHMotionFrameSample& Older) {
        const float Dt = FMath::Max(Newer.TimeStamp - Older.TimeStamp, KINDA_SMALL_NUMBER);
        return (Newer.AngularAcceleration - Older.AngularAcceleration) / Dt;
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Motion")
    static FVector ComputeAngularJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector EstimateJerk(const TArray<SampleType>& Samples, float DeltaTime) {
        if (Samples.Num() < 3 || DeltaTime <= 0.f) {
            return FVector::ZeroVector;
        }

        const FVector A0 = Samples[Samples.Num() - 3].GetLinearAcceleration();
        const FVector A1 = Samples[Samples.Num() - 2].GetLinearAcceleration();
        const FVector A2 = Samples[Samples.Num() - 1].GetLinearAcceleration();

        const FVector J0 = (A1 - A0) / DeltaTime;
        const FVector J1 = (A2 - A1) / DeltaTime;
        return (J1 - J0) / DeltaTime;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector EstimateJerk(const TArray<FVector>& Accelerations, const TArray<float>& Times) {
        int32 N = Accelerations.Num();
        if (N < 3 || Times.Num() < 3) {
            return FVector::ZeroVector;
        }

        const FVector& A0 = Accelerations[N - 3];
        const FVector& A1 = Accelerations[N - 2];
        const FVector& A2 = Accelerations[N - 1];
        const float Dt0 = FMath::Max(Times[N - 2] - Times[N - 3], KINDA_SMALL_NUMBER);
        const float Dt1 = FMath::Max(Times[N - 1] - Times[N - 2], KINDA_SMALL_NUMBER);
        const FVector J0 = (A1 - A0) / Dt0;
        const FVector J1 = (A2 - A1) / Dt1;
        const float AvgDt = (Dt0 + Dt1) * 0.5f;
        return (J1 - J0) / AvgDt;
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector EstimateJerk(const TArray<FOHMotionSample>& Samples, float DeltaTime);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE float EstimateCurvature(const TArray<SampleType>& Samples) {
        if (Samples.Num() < 3) {
            return 0.f;
        }
        const FVector P0 = Samples[Samples.Num() - 3].GetLocation();
        const FVector P1 = Samples[Samples.Num() - 2].GetLocation();
        const FVector P2 = Samples[Samples.Num() - 1].GetLocation();
        const FVector A = P1 - P0;
        const FVector B = P2 - P1;
        const float Cross = FVector::CrossProduct(A, B).Size();
        const float Denom = A.Size() * B.Size();
        if (Denom <= SMALL_NUMBER) {
            return 0.f;
        }
        return Cross / FMath::Square(Denom);
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE float EstimateCurvature(const TArray<FVector>& Positions) {
        if (Positions.Num() < 3) {
            return 0.f;
        }
        const FVector& P0 = Positions[Positions.Num() - 3];
        const FVector& P1 = Positions[Positions.Num() - 2];
        const FVector& P2 = Positions[Positions.Num() - 1];
        const FVector A = P1 - P0;
        const FVector B = P2 - P1;
        const float Cross = FVector::CrossProduct(A, B).Size();
        const float Denom = A.Size() * B.Size();
        if (Denom <= SMALL_NUMBER) {
            return 0.f;
        }
        return Cross / FMath::Square(Denom);
    }

    // Curvature estimation for FOHBoneMotionData
    static float EstimateCurvature(const FOHBoneMotionData& MotionData, int32 SampleWindow = 5);

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static float EstimateCurvature(const TArray<FOHMotionSample>& Samples);

    // Estimate curvature from FOHBoneMotionData
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Algo|Analysis")
    static float EstimateCurvatureFromBoneData(const FOHBoneMotionData& BoneData) {
        if (BoneData.GetHistoryDepth() < 3)
            return 0.0f;

        // Get three positions
        FVector P0 = BoneData.GetHistoricalSample(2)->GetPosition(EOHReferenceSpace::WorldSpace);
        ;
        FVector P1 = BoneData.GetHistoricalSample(1)->GetPosition(EOHReferenceSpace::WorldSpace);
        ;
        FVector P2 = BoneData.GetLatestSample()->GetPosition(EOHReferenceSpace::WorldSpace);
        ;

        // Calculate vectors
        FVector V1 = P1 - P0;
        FVector V2 = P2 - P1;

        // If either vector is too small, no meaningful curvature
        if (V1.IsNearlyZero() || V2.IsNearlyZero())
            return 0.0f;

        // Calculate angle between vectors
        V1.Normalize();
        V2.Normalize();

        float DotProduct = FVector::DotProduct(V1, V2);
        float Angle = FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f));

        // Calculate arc length approximation
        float ArcLength = (P1 - P0).Size() + (P2 - P1).Size();

        // Curvature = angle change / arc length
        return ArcLength > KINDA_SMALL_NUMBER ? Angle / ArcLength : 0.0f;
    }

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector SmoothVelocityEMA(const TArray<SampleType>& Samples, float Alpha) {
        if (Samples.Num() < 1) {
            return FVector::ZeroVector;
        }
        FVector Smoothed = Samples[0].GetLinearVelocity();
        for (int32 i = 1; i < Samples.Num(); ++i) {
            Smoothed = FMath::Lerp(Smoothed, Samples[i].GetLinearVelocity(), Alpha);
        }
        return Smoothed;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector SmoothVelocityEMA(const TArray<FVector>& Velocities, float Alpha) {
        if (Velocities.Num() < 1) {
            return FVector::ZeroVector;
        }
        FVector Smoothed = Velocities[0];
        for (int32 i = 1; i < Velocities.Num(); ++i) {
            Smoothed = FMath::Lerp(Smoothed, Velocities[i], Alpha);
        }
        return Smoothed;
    }

    // Velocity smoothing for FOHBoneMotionData
    static FVector SmoothVelocityEMA(const FOHBoneMotionData& MotionData, float Alpha = 0.3f);

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector SmoothVelocityEMA(const TArray<FOHMotionSample>& Samples, float Alpha);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE bool IsVelocityZeroCrossing(const TArray<SampleType>& Samples) {
        if (Samples.Num() < 2) {
            return false;
        }
        const FVector V0 = Samples[Samples.Num() - 2].GetLinearVelocity();
        const FVector V1 = Samples[Samples.Num() - 1].GetLinearVelocity();
        return (V0.X * V1.X < 0) || (V0.Y * V1.Y < 0) || (V0.Z * V1.Z < 0);
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE bool IsVelocityZeroCrossing(const TArray<FVector>& Velocities) {
        if (Velocities.Num() < 2) {
            return false;
        }
        const FVector& V0 = Velocities[Velocities.Num() - 2];
        const FVector& V1 = Velocities[Velocities.Num() - 1];
        return (V0.X * V1.X < 0) || (V0.Y * V1.Y < 0) || (V0.Z * V1.Z < 0);
    }

    // Zero crossing detection for FOHBoneMotionData
    static bool IsVelocityZeroCrossing(const FOHBoneMotionData& MotionData);

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static bool IsVelocityZeroCrossing(const TArray<FOHMotionSample>& Samples);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE float GetTrajectoryDeviation(const TArray<SampleType>& Samples) {
        if (Samples.Num() < 3) {
            return 0.f;
        }
        const FVector Start = Samples[0].GetLocation();
        const FVector End = Samples.Last().GetLocation();
        const FVector Line = End - Start;
        float MaxDev = 0.f;
        for (int32 i = 1; i < Samples.Num() - 1; ++i) {
            const FVector Pt = Samples[i].GetLocation();
            const FVector ToLine = FVector::PointPlaneProject(Pt, Start, Line.GetSafeNormal());
            MaxDev = FMath::Max(MaxDev, FVector::Dist(Pt, ToLine));
        }
        return MaxDev;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE float GetTrajectoryDeviation(const TArray<FVector>& Positions) {
        if (Positions.Num() < 3) {
            return 0.f;
        }
        const FVector& Start = Positions[0];
        const FVector& End = Positions.Last();
        const FVector Line = End - Start;
        float MaxDev = 0.f;
        for (int32 i = 1; i < Positions.Num() - 1; ++i) {
            const FVector& Pt = Positions[i];
            const FVector ToLine = FVector::PointPlaneProject(Pt, Start, Line.GetSafeNormal());
            MaxDev = FMath::Max(MaxDev, FVector::Dist(Pt, ToLine));
        }
        return MaxDev;
    }
    // Trajectory deviation for FOHBoneMotionData
    static float GetTrajectoryDeviation(const FOHBoneMotionData& MotionData, int32 MaxSamples = 10);

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static float GetTrajectoryDeviation(const TArray<FOHMotionSample>& Samples);

    // -------------------- Motion Predictors --------------------

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector PredictLinear(const SampleType& Current, float PredictTime) {
        return Current.GetLocation() + Current.GetLinearVelocity() * PredictTime;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector PredictLinear(const FVector& Position, const FVector& Velocity, float PredictTime) {
        return Position + Velocity * PredictTime;
    }
    // Linear prediction for FOHMotionFrameSample
    static FORCEINLINE FVector PredictLinear(const FOHMotionFrameSample& Sample, float PredictTime,
                                             bool bUseLocalVelocity) {
        FVector Velocity = bUseLocalVelocity ? Sample.LocalLinearVelocity : Sample.WorldLinearVelocity;
        return Sample.WorldPosition + Velocity * PredictTime;
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector PredictLinear(const FOHMotionSample& Current, float PredictTime);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector PredictQuadratic(const SampleType& Current, float PredictTime) {

        return Current.GetLocation() + Current.GetLinearVelocity() * PredictTime +
               0.5f * Current.GetLinearAcceleration() * FMath::Square(PredictTime);
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector PredictQuadratic(const FVector& Position, const FVector& Velocity,
                                                const FVector& Acceleration, float PredictTime) {
        return Position + Velocity * PredictTime + 0.5f * Acceleration * FMath::Square(PredictTime);
    }

    // Quadratic prediction for FOHMotionFrameSample
    static FORCEINLINE FVector PredictQuadratic(const FOHMotionFrameSample& Sample, float PredictTime,
                                                bool bUseLocalSpace = false) {
        const FVector& Velocity = bUseLocalSpace ? Sample.LocalLinearVelocity : Sample.WorldLinearVelocity;
        const FVector& Acceleration = bUseLocalSpace ? Sample.LocalLinearAcceleration : Sample.WorldLinearAcceleration;
        return Sample.WorldPosition + Velocity * PredictTime + 0.5f * Acceleration * FMath::Square(PredictTime);
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector PredictQuadratic(const FOHMotionSample& Current, float PredictTime);

    // 1. Template version (works for any SampleType with GetLocation())
    template <typename SampleType>
    static FORCEINLINE FVector PredictDamped(const SampleType& Current, float PredictTime, float DampingRatio = 3.0f) {
        const float Decay = FMath::Exp(-FMath::Abs(DampingRatio) * PredictTime);
        const FVector DampedVel = Current.GetLinearVelocity() * Decay;
        return Current.GetLocation() + DampedVel * PredictTime;
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector PredictDamped(const FVector& Position, const FVector& Velocity, float PredictTime,
                                             float DampingRatio = 3.0f) {
        const float Decay = FMath::Exp(-FMath::Abs(DampingRatio) * PredictTime);
        const FVector DampedVel = Velocity * Decay;
        return Position + DampedVel * PredictTime;
    }

    // Damped prediction for FOHMotionFrameSample
    static FORCEINLINE FVector PredictDamped(const FOHMotionFrameSample& Sample, float PredictTime, float DampingFactor,
                                             bool bUseLocalVelocity) {
        FVector Velocity = bUseLocalVelocity ? Sample.LocalLinearVelocity : Sample.WorldLinearVelocity;

        // Apply exponential damping to velocity over time
        float DampingCoeff = FMath::Exp(-DampingFactor * PredictTime);
        FVector DampedVelocity = Velocity * DampingCoeff;

        // Integrate damped velocity
        FVector Displacement = Velocity * (1.0f - DampingCoeff) / DampingFactor;

        return Sample.WorldPosition + Displacement;
    }
    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector PredictDamped(const FOHMotionSample& Current, float PredictTime, float DampingRatio);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE FVector PredictCurvedFromSamples(const TArray<SampleType>& Samples, float PredictTime) {
        // Example: Quadratic Bezier using last three positions
        if (Samples.Num() < 3) {
            return FVector::ZeroVector;
        }
        const FVector P0 = Samples[Samples.Num() - 3].GetLocation();
        const FVector P1 = Samples[Samples.Num() - 2].GetLocation();
        const FVector P2 = Samples[Samples.Num() - 1].GetLocation();
        float T = FMath::Clamp(PredictTime, 0.f, 1.f);
        return FMath::Lerp(FMath::Lerp(P0, P1, T), FMath::Lerp(P1, P2, T), T);
    }

    // 2. Raw data version (array of FVectors)
    static FORCEINLINE FVector PredictCurvedFromSamples(const TArray<FVector>& Positions, float PredictTime) {
        if (Positions.Num() < 3) {
            return FVector::ZeroVector;
        }
        const FVector& P0 = Positions[Positions.Num() - 3];
        const FVector& P1 = Positions[Positions.Num() - 2];
        const FVector& P2 = Positions[Positions.Num() - 1];
        float T = FMath::Clamp(PredictTime, 0.f, 1.f);
        return FMath::Lerp(FMath::Lerp(P0, P1, T), FMath::Lerp(P1, P2, T), T);
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FVector PredictCurvedFromSamples(const TArray<FOHMotionSample>& Samples, float PredictTime);

    // Curved prediction for FOHBoneMotionData
    static FVector PredictCurvedFromMotionData(const FOHBoneMotionData& MotionData, float PredictTime);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE float EstimateTimeToReachTargetLinear(const SampleType& Current, const FVector& TargetPosition) {
        const FVector Dir = TargetPosition - Current.GetLocation();
        const float Speed = Current.GetLinearVelocity().Size();
        if (Speed <= SMALL_NUMBER) {
            return -1.f;
        }
        const float Distance = FVector::DotProduct(Dir, Current.GetLinearVelocity().GetSafeNormal());
        return Distance / Speed;
    }

    // 2. FOHMotionSample overload for Blueprint/legacy API
    static FORCEINLINE float EstimateTimeToReachTargetLinear(const FVector& Position, const FVector& Velocity,
                                                             const FVector& TargetPosition) {
        const FVector Dir = TargetPosition - Position;
        const float Speed = Velocity.Size();
        if (Speed <= SMALL_NUMBER) {
            return -1.f;
        }
        const float Distance = FVector::DotProduct(Dir, Velocity.GetSafeNormal());
        return Distance / Speed;
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static float EstimateTimeToReachTargetLinear(const FOHMotionSample& Current, const FVector& TargetPosition);

    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE float EstimateTimeToStopDamped(const SampleType& Current, float DampingRatio = 3.0f) {
        const float Speed = Current.GetLinearVelocity().Size();
        if (Speed <= SMALL_NUMBER || DampingRatio <= SMALL_NUMBER) {
            return 0.f;
        }
        constexpr float Threshold = 1.f;
        return FMath::Max(0.f, FMath::Loge(Speed / Threshold) / DampingRatio);
    }

    // 2. FOHMotionSample overload for Blueprint/legacy API
    static FORCEINLINE float EstimateTimeToStopDamped(const FVector& Velocity, float DampingRatio = 3.0f) {
        const float Speed = Velocity.Size();
        if (Speed <= SMALL_NUMBER || DampingRatio <= SMALL_NUMBER) {
            return 0.f;
        }
        constexpr float Threshold = 1.f;
        return FMath::Max(0.f, FMath::Loge(Speed / Threshold) / DampingRatio);
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static float EstimateTimeToStopDamped(const FOHMotionSample& Current, float DampingRatio);

    // ----- Semi-Implicit Euler -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateSemiImplicitEulerSample(const SampleType& Current, float DeltaTime) {
        const FVector NewVel = Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime;
        const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;
        // This assumes SampleType provides a static CreateFromState or similar constructor.
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    // 2. Raw-data version for position, velocity, acceleration

    static FORCEINLINE void IntegrateSemiImplicitEuler(const FVector& Position, const FVector& Velocity,
                                                       const FVector& Acceleration, float DeltaTime,
                                                       FVector& OutPosition, FVector& OutVelocity) {
        OutVelocity = Velocity + Acceleration * DeltaTime;
        OutPosition = Position + OutVelocity * DeltaTime;
    }

    // Semi-implicit Euler for FOHMotionFrameSample
    static FOHMotionFrameSample IntegrateSemiImplicitEuler(const FOHMotionFrameSample& Current, float DeltaTime);

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FOHMotionSample IntegrateSemiImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime);

    // ----- Implicit Euler -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateImplicitEulerSample(const SampleType& Current, float DeltaTime) {
        const FVector NewVel = Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime;
        const FVector NewPos = Current.GetLocation() +
                               (Current.GetLinearVelocity() + Current.GetLinearAcceleration() * DeltaTime) * DeltaTime;
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    // 2. Raw-data version for position, velocity, acceleration
    static FORCEINLINE void IntegrateImplicitEuler(const FVector& Position, const FVector& Velocity,
                                                   const FVector& Acceleration, float DeltaTime, FVector& OutPosition,
                                                   FVector& OutVelocity) {
        OutVelocity = Velocity + Acceleration * DeltaTime;
        OutPosition = Position + (Velocity + Acceleration * DeltaTime) * DeltaTime;
    }

    // 3. FOHMotionSample overload for Blueprint/legacy API
    static FOHMotionSample IntegrateImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime);

    // ----- Heun’s Method (Improved Euler) -----
    // 1. Template version (any SampleType with GetLocation() and GetLinearVelocity())
    template <typename SampleType, typename AccelFuncType>
    static FORCEINLINE SampleType IntegrateHeunSample(const SampleType& Current, AccelFuncType AccelFunc,
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

        return SampleType::CreateFromState(FTransform(Current.GetRotation(), FinalPos), FinalVel,
                                           Current.GetAngularVelocity(), FinalAccel, Current.GetAngularAcceleration(),
                                           Current.GetTimeStamp() + DeltaTime);
    }

    // 2. Raw-data version (FVector and lambda)
    template <typename AccelFuncType>
    static FORCEINLINE void IntegrateHeun(const FVector& Position, const FVector& Velocity, AccelFuncType AccelFunc,
                                          float DeltaTime, FVector& OutPosition, FVector& OutVelocity) {
        const FVector a0 = AccelFunc(Position, Velocity);
        const FVector v1 = Velocity + a0 * DeltaTime;
        const FVector x1 = Position + Velocity * DeltaTime;
        const FVector a1 = AccelFunc(x1, v1);

        OutPosition = Position + 0.5f * (Velocity + v1) * DeltaTime;
        OutVelocity = Velocity + 0.5f * (a0 + a1) * DeltaTime;
    }

    // 3. Struct/legacy overload (for FOHMotionSample)
    static FOHMotionSample IntegrateHeunSample(const FOHMotionSample& Current,
                                               TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                               float DeltaTime);

    // ----- Verlet -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateVerletSample(const SampleType& Current, const FVector& PreviousPosition,
                                                        float DeltaTime) {
        const FVector NewPos =
            2 * Current.GetLocation() - PreviousPosition + Current.GetLinearAcceleration() * FMath::Square(DeltaTime);
        const FVector NewVel = (NewPos - PreviousPosition) / (2 * DeltaTime);
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    // Verlet: previous and current position, current acceleration
    static FORCEINLINE void IntegrateVerlet(const FVector& CurrentPosition, const FVector& PreviousPosition,
                                            const FVector& Acceleration, float DeltaTime, FVector& OutPosition,
                                            FVector& OutVelocity) {
        OutPosition = 2 * CurrentPosition - PreviousPosition + Acceleration * FMath::Square(DeltaTime);
        OutVelocity = (OutPosition - PreviousPosition) / (2 * DeltaTime);
    }

    // Overload For Legacy API
    static FOHMotionSample IntegrateVerletSample(const FOHMotionSample& Current, const FVector& PreviousPosition,
                                                 float DeltaTime);

    // ----- Velocity Verlet -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateVelocityVerletSample(const SampleType& Current,
                                                                const FVector& NextAcceleration, float DeltaTime) {
        const FVector NewPos = Current.GetLocation() + Current.GetLinearVelocity() * DeltaTime +
                               0.5f * Current.GetLinearAcceleration() * FMath::Square(DeltaTime);
        const FVector NewVel =
            Current.GetLinearVelocity() + 0.5f * (Current.GetLinearAcceleration() + NextAcceleration) * DeltaTime;
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), NextAcceleration,
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    // Velocity Verlet: position, velocity, acceleration, next acceleration
    static FORCEINLINE void IntegrateVelocityVerlet(const FVector& Position, const FVector& Velocity,
                                                    const FVector& Acceleration, const FVector& NextAcceleration,
                                                    float DeltaTime, FVector& OutPosition, FVector& OutVelocity) {
        OutPosition = Position + Velocity * DeltaTime + 0.5f * Acceleration * FMath::Square(DeltaTime);
        OutVelocity = Velocity + 0.5f * (Acceleration + NextAcceleration) * DeltaTime;
    }

    // Velocity Verlet for FOHMotionFrameSample
    static FOHMotionFrameSample IntegrateVelocityVerlet(const FOHMotionFrameSample& Current,
                                                        const FVector& NextAcceleration, float DeltaTime);

    // Overload For Legacy API
    static FOHMotionSample IntegrateVelocityVerletSample(const FOHMotionSample& Current,
                                                         const FVector& NextAcceleration, float DeltaTime);

    // 1. Template version (works for any StateType with Position and Velocity members)
    template <typename StateType>
    static FORCEINLINE StateType IntegrateRK4(const StateType& InitialState, float DeltaTime,
                                              const FVector& ConstantAcceleration) {
        // K1
        StateType K1;
        K1.Position = InitialState.Velocity;
        K1.Velocity = ConstantAcceleration;

        // K2
        StateType State2;
        State2.Position = InitialState.Position + K1.Position * (DeltaTime * 0.5f);
        State2.Velocity = InitialState.Velocity + K1.Velocity * (DeltaTime * 0.5f);
        StateType K2;
        K2.Position = State2.Velocity;
        K2.Velocity = ConstantAcceleration;

        // K3
        StateType State3;
        State3.Position = InitialState.Position + K2.Position * (DeltaTime * 0.5f);
        State3.Velocity = InitialState.Velocity + K2.Velocity * (DeltaTime * 0.5f);
        StateType K3;
        K3.Position = State3.Velocity;
        K3.Velocity = ConstantAcceleration;

        // K4
        StateType State4;
        State4.Position = InitialState.Position + K3.Position * DeltaTime;
        State4.Velocity = InitialState.Velocity + K3.Velocity * DeltaTime;
        StateType K4;
        K4.Position = State4.Velocity;
        K4.Velocity = ConstantAcceleration;

        // Final state
        StateType Result = InitialState;
        Result.Position = InitialState.Position +
                          (K1.Position + 2.0f * K2.Position + 2.0f * K3.Position + K4.Position) * (DeltaTime / 6.0f);
        Result.Velocity = InitialState.Velocity +
                          (K1.Velocity + 2.0f * K2.Velocity + 2.0f * K3.Velocity + K4.Velocity) * (DeltaTime / 6.0f);
        return Result;
    }

    // 2. Raw-data version (position/velocity/acceleration only)
    static FORCEINLINE void IntegrateRK4(const FVector& Position, const FVector& Velocity,
                                         const FVector& ConstantAcceleration, float DeltaTime, FVector& OutPosition,
                                         FVector& OutVelocity) {
        // K1
        const FVector K1_V = Velocity;
        const FVector K1_A = ConstantAcceleration;

        // K2
        const FVector K2_V = Velocity + K1_A * (DeltaTime * 0.5f);
        const FVector K2_A = ConstantAcceleration;

        // K3
        const FVector K3_V = Velocity + K2_A * (DeltaTime * 0.5f);
        const FVector K3_A = ConstantAcceleration;

        // K4
        const FVector K4_V = Velocity + K3_A * DeltaTime;
        const FVector K4_A = ConstantAcceleration;

        OutPosition = Position + (K1_V + 2.0f * K2_V + 2.0f * K3_V + K4_V) * (DeltaTime / 6.0f);
        OutVelocity = Velocity + (K1_A + 2.0f * K2_A + 2.0f * K3_A + K4_A) * (DeltaTime / 6.0f);
    }

    // 3. Legacy overload for Blueprint or struct
    static FRK4State IntegrateRK4(const FRK4State& InitialState, float DeltaTime, const FVector& ConstantAcceleration);

    // RK4 Integration specifically for FRK4State
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Algo|Physics")
    static FRK4State IntegrateRK4State(const FRK4State& CurrentState, const FVector& ForceAcceleration,
                                       float TimeStep) {
        FRK4State Result;

        // RK4 integration for position and velocity
        FVector k1_v = ForceAcceleration;
        FVector k1_x = CurrentState.Velocity;

        FVector k2_v = ForceAcceleration; // Assuming constant force
        FVector k2_x = CurrentState.Velocity + k1_v * TimeStep * 0.5f;

        FVector k3_v = ForceAcceleration;
        FVector k3_x = CurrentState.Velocity + k2_v * TimeStep * 0.5f;

        FVector k4_v = ForceAcceleration;
        FVector k4_x = CurrentState.Velocity + k3_v * TimeStep;

        // Update position and velocity
        Result.Position = CurrentState.Position + (k1_x + 2.0f * k2_x + 2.0f * k3_x + k4_x) * (TimeStep / 6.0f);
        Result.Velocity = CurrentState.Velocity + (k1_v + 2.0f * k2_v + 2.0f * k3_v + k4_v) * (TimeStep / 6.0f);

        // For now, rotation remains constant (can be enhanced later)
        Result.Rotation = CurrentState.Rotation;
        Result.AngularVelocity = CurrentState.AngularVelocity;

        return Result;
    }

    // ----- RK4 with acceleration function -----

    // Template version (for any SampleType with GetLocation/GetLinearVelocity)
    template <typename SampleType, typename AccelFuncType>
    static FORCEINLINE SampleType IntegrateRK4Sample(const SampleType& Current, AccelFuncType AccelFunc,
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

        return SampleType::CreateFromState(FTransform(Current.GetRotation(), FinalPos), FinalVel,
                                           Current.GetAngularVelocity(), FinalAccel, Current.GetAngularAcceleration(),
                                           Current.GetTimeStamp() + DeltaTime);
    }

    // Legacy/struct overload
    static FOHMotionSample IntegrateRK4Sample(const FOHMotionSample& Current,
                                              TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                              float DeltaTime);

    // ----- Adaptive Step RK4 -----

    template <typename SampleType, typename AccelFuncType>
    static FORCEINLINE SampleType IntegrateAdaptiveRK4Sample(const SampleType& Current, AccelFuncType AccelFunc,
                                                             float& InOutDeltaTime, float MinStep = 0.0001f,
                                                             float MaxStep = 0.033f, float Tolerance = 0.01f) {
        const float HalfStep = InOutDeltaTime * 0.5f;
        SampleType Mid = IntegrateRK4Sample(Current, AccelFunc, HalfStep);
        SampleType Small = IntegrateRK4Sample(Mid, AccelFunc, HalfStep);
        SampleType Large = IntegrateRK4Sample(Current, AccelFunc, InOutDeltaTime);

        const float Error = FVector::Dist(Small.GetLocation(), Large.GetLocation());
        if (Error > Tolerance && InOutDeltaTime > MinStep) {
            InOutDeltaTime *= 0.5f;
        } else if (Error < Tolerance * 0.25f && InOutDeltaTime < MaxStep) {
            InOutDeltaTime *= 2.0f;
        }
        return Small;
    }

    // Legacy/struct overload
    static FOHMotionSample IntegrateAdaptiveRK4Sample(const FOHMotionSample& Current,
                                                      TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                      float& InOutDeltaTime, float MinStep = 0.0001f,
                                                      float MaxStep = 0.033f, float Tolerance = 0.01f);
    // ----- Critically Damped Spring -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateCriticallyDampedSpringSample(const SampleType& Current,
                                                                        const FVector& TargetPosition,
                                                                        float AngularFreq, float DeltaTime) {
        const float Omega = FMath::Clamp(AngularFreq, 1.f, 100.f);
        const float Exp = FMath::Exp(-Omega * DeltaTime);
        const FVector Displacement = Current.GetLocation() - TargetPosition;
        const FVector NewPos = TargetPosition + (Displacement + Current.GetLinearVelocity() / Omega) * Exp;
        const FVector NewVel = (NewPos - Current.GetLocation()) / DeltaTime;
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    // Overload For Legacy API
    static FOHMotionSample IntegrateCriticallyDampedSpringSample(const FOHMotionSample& Current,
                                                                 const FVector& TargetPosition, float AngularFreq,
                                                                 float DeltaTime);

    // Critically Damped Spring specifically for FOHMotionFrameSample
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Algo|Physics")
    static FOHMotionFrameSample IntegrateCriticallyDampedSpringMotionSample(const FOHMotionFrameSample& CurrentSample,
                                                                            const FVector& TargetPosition,
                                                                            float SpringFrequency, float TimeStep) {
        FOHMotionFrameSample Result = CurrentSample;

        // Calculate spring forces
        FVector Displacement = TargetPosition - CurrentSample.WorldPosition;
        FVector SpringForce = Displacement * SpringFrequency * SpringFrequency;
        FVector DampingForce = -CurrentSample.WorldLinearVelocity * 2.0f * SpringFrequency;
        FVector TotalAcceleration = SpringForce + DampingForce;

        // Update velocities and position
        Result.WorldLinearVelocity = CurrentSample.WorldLinearVelocity + TotalAcceleration * TimeStep;
        Result.WorldPosition = CurrentSample.WorldPosition + Result.WorldLinearVelocity * TimeStep;
        Result.WorldLinearAcceleration = TotalAcceleration;

        // Update other fields
        Result.TimeStamp = CurrentSample.TimeStamp + TimeStep;
        Result.DeltaTime = TimeStep;
        Result.WorldSpeed = Result.WorldLinearVelocity.Size();

        // Keep other fields the same for now
        Result.WorldRotation = CurrentSample.WorldRotation;
        Result.AngularVelocity = CurrentSample.AngularVelocity;
        Result.ComponentTransform = CurrentSample.ComponentTransform;
        Result.ReferenceTransform = CurrentSample.ReferenceTransform;

        // Update local velocities
        if (!Result.ComponentTransform.Equals(FTransform::Identity)) {
            Result.LocalLinearVelocity = Result.ComponentTransform.InverseTransformVector(Result.WorldLinearVelocity);
            Result.LocalLinearAcceleration =
                Result.ComponentTransform.InverseTransformVector(Result.WorldLinearAcceleration);
            Result.LocalSpeed = Result.LocalLinearVelocity.Size();
        }

        return Result;
    }

    // ----- Exponential Integration -----
    template <typename SampleType>
    // 1. Template version (works for any SampleType with GetLocation())
    static FORCEINLINE SampleType IntegrateExponentialSample(const SampleType& Current, float DampingRatio,
                                                             float DeltaTime) {
        const float ExpFactor = FMath::Exp(-FMath::Abs(DampingRatio) * DeltaTime);
        const FVector NewVel = Current.GetLinearVelocity() * ExpFactor;
        const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), Current.GetLinearAcceleration(),
                                           Current.GetAngularAcceleration(), Current.GetTimeStamp() + DeltaTime);
    }

    static FOHMotionSample IntegrateExponentialSample(const FOHMotionSample& Current, float DampingRatio,
                                                      float DeltaTime);

    // ----- Spring Blend -----
    template <typename SampleType>
    static FORCEINLINE SampleType IntegrateSpringBlendSample(const SampleType& Current, const FVector& TargetPosition,
                                                             float BlendAlpha, float SpringCoeff, float DeltaTime) {
        const FVector BlendedTarget = FMath::Lerp(Current.GetLocation(), TargetPosition, BlendAlpha);
        const FVector SpringForce = -SpringCoeff * (Current.GetLocation() - BlendedTarget);
        const FVector NewVel = Current.GetLinearVelocity() + SpringForce * DeltaTime;
        const FVector NewPos = Current.GetLocation() + NewVel * DeltaTime;
        return SampleType::CreateFromState(FTransform(Current.GetRotation(), NewPos), NewVel,
                                           Current.GetAngularVelocity(), SpringForce, Current.GetAngularAcceleration(),
                                           Current.GetTimeStamp() + DeltaTime);
    }

    static FOHMotionSample IntegrateSpringBlendSample(const FOHMotionSample& Current, const FVector& TargetPosition,
                                                      float BlendAlpha, float SpringCoeff, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Algo|Filtering")
    static FOHKalmanState KalmanPredict(const FOHKalmanState& CurrentState, const FVector& Measurement,
                                        float ProcessNoise, float MeasurementNoise, float DeltaTime = 0.016f) {
        FOHKalmanState NewState = CurrentState;
        NewState.ProcessNoise = ProcessNoise;
        NewState.MeasurementNoise = MeasurementNoise;

        // Predict
        NewState.Predict(DeltaTime);

        // Update with measurement
        NewState.Update(Measurement, DeltaTime);

        return NewState;
    }
    // Simple Kalman filter specifically for position tracking
    UFUNCTION(BlueprintCallable, Category = "OnlyHands|Algo|Filtering")
    static FOHKalmanState PredictKalmanState(const FOHKalmanState& CurrentState, const FVector& MeasuredPosition,
                                             float DeltaTime) {
        FOHKalmanState NewState = CurrentState;

        // Predict step
        NewState.Predict(DeltaTime);

        // Update step
        NewState.Update(MeasuredPosition, DeltaTime);

        return NewState;
    }
#pragma endregion

#pragma region SolverUtils

    /** Computes the Baumgarte stabilization term for constraint error resolution */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Solvers")
    static float ComputeBaumgarteTerm(float ConstraintError, float Beta, float DeltaTime);

    /** Iteratively projects a bone chain toward a target position using spring-like correction */
    static void ProjectChainToTargetPosition(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                             const FVector& TargetPosition, float Stiffness = 1.0f,
                                             int32 Iterations = 3);

    /** Iteratively rotates a bone chain toward a target rotation */
    static void ProjectChainToTargetRotation(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                             const FQuat& TargetRotation, float Stiffness = 1.0f, int32 Iterations = 3);

    /** Projects a chain toward both a target position and target rotation */
    static void ProjectChainToTargetTransform(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                              const FTransform& Target, float PosStiffness = 1.0f,
                                              float RotStiffness = 1.0f, int32 Iterations = 3);

    /** Projects a chain to a target transform and pushes the result as motion samples */
    static void SolveAndPushChainToTargetTransform(const TArray<FName>& ChainBones, TMap<FName, FOHBoneData>& BoneMap,
                                                   const FTransform& Target, float PosStiffness, float RotStiffness,
                                                   int32 Iterations, float TimeStamp);

#pragma endregion

#pragma region PhysicsOperations
    // Apply drag to velocity
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Physics", meta = (DisplayName = "Apply Drag"))
    static FVector ApplyDrag(const FVector& Velocity, float DragCoefficient = 0.47f, float CrossSectionalArea = 50.0f,
                             float Mass = 5.0f, float DeltaTime = 0.016f);

    // Calculate distance-based falloff
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Impulse", meta = (DisplayName = "Calculate Distance Falloff"))
    static float CalculateDistanceFalloff(float Distance, float FalloffRadius, float FalloffExponent = 2.0f);

    // Coulomb friction model
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Physics", meta = (DisplayName = "Calculate Friction Force"))
    static FVector CalculateFrictionForce(const FVector& RelativeVelocity, const FVector& NormalForce,
                                          float StaticFrictionCoeff = 0.6f, float KineticFrictionCoeff = 0.4f,
                                          float StaticThreshold = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Analysis", meta = (DisplayName = "Get Contact Bistangential"))
    static void GetContactBistangential(const FHitResult& Hit, FVector& OutTangentU, FVector& OutTangentV,
                                        FVector& OutNormal);

    UFUNCTION(BlueprintPure, Category = "OH|Physics|Contact", meta = (DisplayName = "Calculate Hertz Contact Area"))
    static float CalculateHertzContactArea(float AppliedForce, float BodyRadius, float ElasticModulus = 1000000.0f);

    // Calculate contact pressure from force and area
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Contact", meta = (DisplayName = "Calculate Contact Pressure"))
    static float CalculateContactPressure(float AppliedForce, float ContactArea);

    UFUNCTION(BlueprintPure, Category = "OH|Physics|Force", meta = (DisplayName = "Calculate Impact Force"))
    static float CalculateImpactForce(float Mass, float Velocity, float ContactDuration = 0.01f);

    UFUNCTION(BlueprintPure, Category = "OH|Physics|Velocity", meta = (DisplayName = "Decompose Velocity"))
    static void DecomposeVelocity(const FVector& Velocity, const FVector& Normal, FVector& OutNormalComponent,
                                  FVector& OutTangentialComponent, float& OutNormalSpeed, float& OutTangentialSpeed);

    // Calculate shear components from velocity
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Shear", meta = (DisplayName = "Calculate Shear Components"))
    static void CalculateShearComponents(const FVector& Velocity, const FVector& SurfaceNormal,
                                         FVector& OutShearDirection, float& OutShearMagnitude);

    // Calculate velocity reflection
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Velocity", meta = (DisplayName = "Calculate Velocity Reflection"))
    static FVector CalculateVelocityReflection(const FVector& IncomingVelocity, const FVector& SurfaceNormal,
                                               float Restitution = 1.0f);

    UFUNCTION(BlueprintPure, Category = "OH|Physics|Energy", meta = (DisplayName = "Calculate Kinetic Energy"))
    static float CalculateKineticEnergy(float Mass, float Velocity);

    // Calculate kinetic energy from vector velocity
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Energy", meta = (DisplayName = "Calculate Kinetic Energy Vector"))
    static float CalculateKineticEnergyVector(float Mass, const FVector& Velocity);

    // Calculate rotational kinetic energy
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Energy", meta = (DisplayName = "Calculate Rotational Energy"))
    static float CalculateRotationalEnergy(float MomentOfInertia, const FVector& AngularVelocity);

    // Calculate elastic energy fraction
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Energy", meta = (DisplayName = "Calculate Elastic Energy Fraction"))
    static float CalculateElasticEnergyFraction(float Restitution);

    // Distribute impact energy
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Energy", meta = (DisplayName = "Distribute Impact Energy"))
    static void DistributeImpactEnergy(float TotalEnergy, float Restitution, float& OutElasticEnergy,
                                       float& OutPlasticEnergy, float& OutHeatEnergy, float& OutSoundEnergy);

    // Calculate momentum
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Momentum", meta = (DisplayName = "Calculate Momentum"))
    static FVector CalculateMomentum(float Mass, const FVector& Velocity);

    // Calculate impulse magnitude from collision parameters
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Collision")
    static float CalculateImpulseMagnitude(float RelativeNormalVelocity, float ReducedMass, float Restitution);

    // Calculate impulse magnitude with friction
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Collision")
    static FVector CalculateImpulseWithFriction(const FVector& RelativeVelocity, const FVector& CollisionNormal,
                                                float Mass1, float Mass2, float Restitution, float StaticFriction,
                                                float KineticFriction);

    // ==================== Material Property Derivation ====================

    // Get material properties for common biological tissues
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Materials")
    static FMaterialPhysicsProperties GetBiologicalMaterialProperties(EOHBiologicalMaterial MaterialType);

    // Derive restitution coefficient from material properties
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Materials")
    static float DeriveRestitutionCoefficient(float YoungsModulus1, float YoungsModulus2, float PoissonsRatio1 = 0.3f,
                                              float PoissonsRatio2 = 0.3f);

    // Derive friction coefficients from surface properties
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Materials")
    static void DeriveFrictionCoefficients(EPhysicalSurface Surface1, EPhysicalSurface Surface2,
                                           float& OutStaticFriction, float& OutKineticFriction);

    // Calculate drag coefficient from shape
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Drag")
    static float CalculateDragCoefficient(EOHShapeType ShapeType, float AspectRatio = 1.0f);

    // Calculate cross-sectional area from bone bounds
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Geometry")
    static float CalculateCrossSectionalArea(const FVector& BoundsExtent, const FVector& VelocityDirection);

    // ==================== Mass and Inertia Estimation ====================

    // Estimate bone mass from volume and tissue density
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Mass")
    static float EstimateBoneMassFromVolume(const FVector& BoundsExtent, EOHBiologicalMaterial TissueType,
                                            float BonePercentage = 0.2f // Percentage that is bone vs soft tissue
    );

    // Calculate reduced mass for collision
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Mass")
    static float CalculateReducedMass(float Mass1, float Mass2);

    // ==================== Validation and Calibration ====================

    // Validate physics parameters against expected ranges
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Validation")
    static bool ValidatePhysicsParameters(float Restitution, float StaticFriction, float KineticFriction,
                                          FString& OutWarnings);

    // Auto-calibrate coefficients from observed motion
    UFUNCTION(BlueprintCallable, Category = "OH|Physics|Calibration")
    static void CalibrateCoefficientsFromMotion(const TArray<FVector>& PreImpactVelocities,
                                                const TArray<FVector>& PostImpactVelocities,
                                                const TArray<FVector>& ImpactNormals, float& OutRestitution,
                                                float& OutFriction);
    // Calculate impulse from momentum change
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Impulse", meta = (DisplayName = "Calculate Impulse From Momentum"))
    static FVector CalculateImpulseFromMomentum(const FVector& InitialMomentum, const FVector& FinalMomentum);

    // Calculate coefficient of restitution from velocities
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Material", meta = (DisplayName = "Calculate Restitution"))
    static float CalculateRestitutionFromVelocities(float RelativeVelocityBefore, float RelativeVelocityAfter);

    // Calculate moment of inertia for simple shapes
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Angular", meta = (DisplayName = "Calculate Moment Of Inertia"))
    static float CalculateMomentOfInertia(float Mass, float Radius,
                                          float InertiaCoefficient = 0.4f); // 0.4 for solid sphere, 0.5 for cylinder

    // Calculate torque from force and lever arm
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Angular", meta = (DisplayName = "Calculate Torque"))
    static FVector CalculateTorque(const FVector& Force, const FVector& LeverArm);

    // Calculate angular momentum
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Angular", meta = (DisplayName = "Calculate Angular Momentum"))
    static FVector CalculateAngularMomentum(float MomentOfInertia, const FVector& AngularVelocity);

    // Calculate angular impulse
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Angular", meta = (DisplayName = "Calculate Angular Impulse"))
    static FVector CalculateAngularImpulse(const FVector& Torque, float Duration);

    // Calculate collision impulse magnitude
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Collision",
              meta = (DisplayName = "Calculate Collision Impulse Magnitude"))
    static float CalculateCollisionImpulseMagnitude(float RelativeNormalVelocity, float ReducedMass, float Restitution);

    // Calculate post-collision velocities
    UFUNCTION(BlueprintPure, Category = "OH|Physics|Collision",
              meta = (DisplayName = "Calculate Post Collision Velocities"))
    static void CalculatePostCollisionVelocities(const FVector& Velocity1, const FVector& Velocity2, float Mass1,
                                                 float Mass2, const FVector& CollisionNormal, float Restitution,
                                                 FVector& OutVelocity1, FVector& OutVelocity2);

    UFUNCTION(BlueprintPure, Category = "OH|Combat|Analysis", meta = (DisplayName = "Analyze Contact Patch"))
    static void AnalyzeContactPatch(const FHitResult& Hit, float& OutContactArea, float& OutPressure,
                                    FVector& OutShearDirection, float& OutShearMagnitude);

    // Calculate energy distribution from impact
    UFUNCTION(BlueprintPure, Category = "OH|Combat|Analysis",
              meta = (DisplayName = "Calculate Impact Energy Distribution"))
    static void CalculateImpactEnergyDistribution(const FHitResult& Hit, float& OutKineticEnergyTransfer,
                                                  float& OutRotationalEnergy, float& OutDeformationEnergy,
                                                  float& OutSoundEnergy, float& OutHeatEnergy);

#pragma endregion
#pragma region MotionEstimators

#pragma endregion

#pragma region MotionPredictors

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Prediction")
    static TArray<FVector> GetCubicBezierControlPointsFromBoneData(const FOHBoneData& BoneData,
                                                                   float PredictTime = 0.3f);

    static TArray<FVector> GetCubicBezierControlPointsFromBoneData(const FOHBoneMotionData& BoneData,
                                                                   float PredictTime);

    UFUNCTION(BlueprintPure, Category = "OH|Locomotion|Prediction")
    static TArray<FVector> GetQuadraticBezierControlPointsFromBoneData(const FOHBoneData& BoneData,
                                                                       float PredictTime = 0.3f);

    static TArray<FVector> GetQuadraticBezierControlPointsFromBoneData(const FOHBoneMotionData& BoneData,
                                                                       float PredictTime);

    /** Samples a point along a quadratic Bezier defined by three points */
    static FVector SampleBezierQuadratic(const FVector& P0, const FVector& P1, const FVector& P2, float T);

    /** Samples a point along a cubic Bezier defined by four control points */
    static FVector SampleBezierCubic(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& P3,
                                     float T);

#pragma endregion

#pragma region FeedbackControllers

    /** Simple stateless PID control output */
    static FVector PIDControl(const FVector& Error, const FVector& ErrorIntegral, const FVector& ErrorDerivative,
                              float Kp, float Ki, float Kd, float MaxOutput = BIG_NUMBER);

#pragma endregion

#pragma region ControlAndStabilization

    /** Returns critically damped spring acceleration */
    UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Control")
    static FVector ComputeCriticallyDampedSpring(const FVector& Pos, const FVector& Vel, const FVector& TargetPos,
                                                 float Stiffness, float Damping);

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

    static EOHNameMatchingStrategy DetermineBestMatchingStrategy(const FString& Target,
                                                                 const TArray<FString>& Candidates);

    static FOHNameMatchResult FindBestNameMatchUsingStrategy(const FString& Target, const TArray<FString>& Candidates,
                                                             EOHNameMatchingStrategy Strategy);

    /** Matches multiple target strings to the best candidates using the selected strategy */
    static TArray<FOHNameMatchResult> BatchMatchTargetsToCandidates(const TArray<FString>& Targets,
                                                                    const TArray<FString>& Candidates,
                                                                    EOHNameMatchingStrategy Strategy);

    static TMap<FString, FOHNameMatchResult>
    MatchTargetAcrossSets(const FString& Target, const TMap<FString, TArray<FString>>& CandidateSetsByLabel,
                          EOHNameMatchingStrategy Strategy);

    static TMap<FString, FOHNameMatchResult> BatchAutoMatch(const TArray<FString>& Targets,
                                                            const TArray<FString>& Candidates);

    static TArray<FOHNameMatchResult> ScoreMatchesAgainstCandidates(const FString& Target,
                                                                    const TArray<FString>& Candidates,
                                                                    EOHNameMatchingStrategy Strategy);

    static TArray<FOHNameMatchResult> RankFNameToEnumMap(const FString& Target, const TMap<FName, EOHSkeletalBone>& Map,
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

  public:
#pragma region DistanceFormulas

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Distance")
    static float ManhattanDistance(const FVector& A, const FVector& B);

    UFUNCTION(BlueprintPure, Category = "OnlyHands|Algo|Distance")
    static float EuclideanDistance(const FVector& A, const FVector& B);

#pragma endregion

    // ==================== Advanced Analysis ====================

    // Motion quality score combining multiple metrics
    static float CalculateMotionQuality(const FOHBoneMotionData& MotionData);

    // Predict time to target using motion data
    static float EstimateTimeToReachTarget(const FOHBoneMotionData& MotionData, const FVector& TargetPosition);

    // Multi-frame jerk estimation
    static FVector EstimateJerkFromMotionData(const FOHBoneMotionData& MotionData, int32 SampleWindow = 3);

    // Get bezier control points from motion data
    static TArray<FVector> GetBezierControlPointsFromBoneData(const FOHBoneMotionData& MotionData, float PredictTime,
                                                              bool bCubic = true);

#pragma region Debug

    UFUNCTION(BlueprintCallable, Category = "OH|Locomotion|Debug")
    void DrawDebugQuadraticBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps, FLinearColor Color,
                                  float Thickness, float Duration);

    UFUNCTION(BlueprintCallable, Category = "OH|Locomotion|Debug")
    void DrawDebugCubicBezier(const UWorld* World, const TArray<FVector>& Points, int32 Steps, FLinearColor Color,
                              float Thickness, float Duration);

#pragma endregion

  private:
    // Internal material database
    static TMap<EOHBiologicalMaterial, FMaterialPhysicsProperties> GetMaterialDatabase();
    static TMap<TPair<EPhysicalSurface, EPhysicalSurface>, TPair<float, float>> GetFrictionDatabase();
    static TPair<float, float> GetFrictionCoefficient(EPhysicalSurface SurfaceA, EPhysicalSurface SurfaceB);
};
