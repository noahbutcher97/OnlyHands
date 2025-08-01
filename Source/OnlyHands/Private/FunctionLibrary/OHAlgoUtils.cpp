

#include "FunctionLibrary/OHAlgoUtils.h"
#include "OHPhysicsStructs.h"
#include "Components/SkeletalMeshComponent.h"
#include "Animation/Skeleton.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"

#pragma region IntegrationUtils

FOHMotionFrameSample UOHAlgoUtils::IntegrateSemiImplicitEuler(const FOHMotionFrameSample& Current, float DeltaTime) {
    FOHMotionFrameSample Next = Current;
    Next.TimeStamp += DeltaTime;
    Next.DeltaTime = DeltaTime;

    // Semi-implicit Euler: velocity first, then position
    Next.WorldLinearVelocity = Current.WorldLinearVelocity + Current.WorldLinearAcceleration * DeltaTime;
    Next.LocalLinearVelocity = Current.LocalLinearVelocity + Current.LocalLinearAcceleration * DeltaTime;
    Next.AngularVelocity = Current.AngularVelocity + Current.AngularAcceleration * DeltaTime;

    // Update position using new velocity
    Next.WorldPosition = Current.WorldPosition + Next.WorldLinearVelocity * DeltaTime;

    // Update rotation using new angular velocity
    FVector AngularDisplacement = Next.AngularVelocity * DeltaTime;
    float Angle = AngularDisplacement.Size();
    if (Angle > KINDA_SMALL_NUMBER) {
        FQuat DeltaRotation(AngularDisplacement.GetSafeNormal(), Angle);
        Next.WorldRotation = DeltaRotation * Current.WorldRotation;
        Next.WorldRotation.Normalize();
    } else {
        Next.WorldRotation = Current.WorldRotation;
    }

    // Update derived values
    Next.WorldSpeed = Next.WorldLinearVelocity.Size();
    Next.LocalSpeed = Next.LocalLinearVelocity.Size();
    Next.AngularSpeed = Next.AngularVelocity.Size();

    return Next;
}

FOHMotionSample UOHAlgoUtils::IntegrateSemiImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime) {
    return IntegrateSemiImplicitEulerSample<FOHMotionSample>(Current, DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateImplicitEulerSample(const FOHMotionSample& Current, float DeltaTime) {
    return IntegrateImplicitEulerSample<FOHMotionSample>(Current, DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateVerletSample(const FOHMotionSample& Current, const FVector& PreviousPosition,
                                                    float DeltaTime) {
    return IntegrateVerletSample<FOHMotionSample>(Current, PreviousPosition, DeltaTime);
}

FOHMotionFrameSample UOHAlgoUtils::IntegrateVelocityVerlet(const FOHMotionFrameSample& Current,
                                                           const FVector& NextAcceleration, float DeltaTime) {
    FOHMotionFrameSample Next = Current;
    Next.TimeStamp += DeltaTime;
    Next.DeltaTime = DeltaTime;

    // Velocity Verlet integration
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
    Next.WorldPosition = Current.WorldPosition + Current.WorldLinearVelocity * DeltaTime +
                         Current.WorldLinearAcceleration * (0.5f * DeltaTime * DeltaTime);

    // v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
    Next.WorldLinearVelocity =
        Current.WorldLinearVelocity + (Current.WorldLinearAcceleration + NextAcceleration) * (0.5f * DeltaTime);

    Next.WorldLinearAcceleration = NextAcceleration;

    // Update derived values
    Next.WorldSpeed = Next.WorldLinearVelocity.Size();
    Next.LocalSpeed = Next.LocalLinearVelocity.Size();
    Next.AngularSpeed = Next.AngularVelocity.Size();

    return Next;
}

FOHMotionSample UOHAlgoUtils::IntegrateVelocityVerletSample(const FOHMotionSample& Current,
                                                            const FVector& NextAcceleration, float DeltaTime) {
    return IntegrateVelocityVerletSample<FOHMotionSample>(Current, NextAcceleration, DeltaTime);
}

FRK4State UOHAlgoUtils::IntegrateRK4(const FRK4State& InitialState, float DeltaTime,
                                     const FVector& ConstantAcceleration) {
    return IntegrateRK4<FRK4State>(InitialState, DeltaTime, ConstantAcceleration);
}

FOHMotionSample UOHAlgoUtils::IntegrateRK4Sample(const FOHMotionSample& Current,
                                                 TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                 float DeltaTime) {
    return IntegrateRK4Sample<FOHMotionSample, TFunction<FVector(const FVector&, const FVector&)>>(Current, AccelFunc,
                                                                                                   DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateAdaptiveRK4Sample(const FOHMotionSample& Current,
                                                         TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                         float& InOutDeltaTime, float MinStep, float MaxStep,
                                                         float Tolerance) {
    return IntegrateAdaptiveRK4Sample<FOHMotionSample, TFunction<FVector(const FVector&, const FVector&)>>(
        Current, AccelFunc, InOutDeltaTime, MinStep, MaxStep, Tolerance);
}

FOHMotionSample UOHAlgoUtils::IntegrateCriticallyDampedSpringSample(const FOHMotionSample& Current,
                                                                    const FVector& TargetPosition, float AngularFreq,
                                                                    float DeltaTime) {
    return IntegrateCriticallyDampedSpringSample<FOHMotionSample>(Current, TargetPosition, AngularFreq, DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateHeunSample(const FOHMotionSample& Current,
                                                  TFunction<FVector(const FVector&, const FVector&)> AccelFunc,
                                                  float DeltaTime) {
    return IntegrateHeunSample<FOHMotionSample, TFunction<FVector(const FVector&, const FVector&)>>(Current, AccelFunc,
                                                                                                    DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateExponentialSample(const FOHMotionSample& Current, float DampingRatio,
                                                         float DeltaTime) {
    return IntegrateExponentialSample<FOHMotionSample>(Current, DampingRatio, DeltaTime);
}

FOHMotionSample UOHAlgoUtils::IntegrateSpringBlendSample(const FOHMotionSample& Current, const FVector& TargetPosition,
                                                         float BlendAlpha, float SpringCoeff, float DeltaTime) {
    return IntegrateSpringBlendSample<FOHMotionSample>(Current, TargetPosition, BlendAlpha, SpringCoeff, DeltaTime);
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

FVector UOHAlgoUtils::ApplyDrag(const FVector& Velocity, float DragCoefficient, float CrossSectionalArea, float Mass,
                                float DeltaTime) {
    float Speed = Velocity.Size();
    if (Speed < KINDA_SMALL_NUMBER)
        return Velocity;

    // F_drag = 0.5 * ρ * C_d * A * v²
    const float AirDensity = 0.001225f; // g/cm³
    float DragForce = 0.5f * AirDensity * DragCoefficient * CrossSectionalArea * Speed * Speed;

    // Drag acceleration opposes motion
    FVector DragAccel = -Velocity.GetSafeNormal() * (DragForce / Mass);

    // Apply using semi-implicit Euler
    return Velocity + DragAccel * DeltaTime;
}

float UOHAlgoUtils::CalculateDistanceFalloff(float Distance, float FalloffRadius, float FalloffExponent) {
    if (Distance <= 0.0f)
        return 1.0f;
    if (Distance >= FalloffRadius)
        return 0.0f;

    float NormalizedDist = Distance / FalloffRadius;
    return FMath::Pow(1.0f - NormalizedDist, FalloffExponent);
}

#pragma endregion

#pragma region MotionEstimators

FVector UOHAlgoUtils::CalculateFrictionForce(const FVector& RelativeVelocity, const FVector& NormalForce,
                                             float StaticFrictionCoeff, float KineticFrictionCoeff,
                                             float StaticThreshold) {
    float NormalMagnitude = NormalForce.Size();
    if (NormalMagnitude < KINDA_SMALL_NUMBER)
        return FVector::ZeroVector;

    FVector VelocityDirection = RelativeVelocity.GetSafeNormal();
    float Speed = RelativeVelocity.Size();

    // Static vs kinetic friction
    if (Speed < StaticThreshold) {
        // Static friction (opposes any applied force up to maximum)
        return -VelocityDirection * FMath::Min(Speed, StaticFrictionCoeff * NormalMagnitude);
    } else {
        // Kinetic friction (constant magnitude opposing motion)
        return -VelocityDirection * KineticFrictionCoeff * NormalMagnitude;
    }
}

void UOHAlgoUtils::GetContactBistangential(const FHitResult& Hit, FVector& OutTangentU, FVector& OutTangentV,
                                           FVector& OutNormal) {
    OutNormal = Hit.ImpactNormal;

    // Find the least aligned world axis to create a stable tangent
    FVector WorldUp = FVector::UpVector;
    FVector WorldRight = FVector::RightVector;
    FVector WorldForward = FVector::ForwardVector;

    float UpDot = FMath::Abs(FVector::DotProduct(OutNormal, WorldUp));
    float RightDot = FMath::Abs(FVector::DotProduct(OutNormal, WorldRight));
    float ForwardDot = FMath::Abs(FVector::DotProduct(OutNormal, WorldForward));

    // Use the axis least aligned with normal to avoid degenerate cases
    FVector TangentBase;
    if (UpDot <= RightDot && UpDot <= ForwardDot) {
        TangentBase = WorldUp;
    } else if (RightDot <= ForwardDot) {
        TangentBase = WorldRight;
    } else {
        TangentBase = WorldForward;
    }

    // Gram-Schmidt orthogonalization
    OutTangentU = (TangentBase - OutNormal * FVector::DotProduct(TangentBase, OutNormal)).GetSafeNormal();
    OutTangentV = FVector::CrossProduct(OutNormal, OutTangentU);
}

float UOHAlgoUtils::CalculateHertzContactArea(float AppliedForce, float BodyRadius, float ElasticModulus) {
    // Hertz contact: A = π * sqrt(F * R / E)
    if (AppliedForce <= 0 || BodyRadius <= 0 || ElasticModulus <= 0)
        return 0.0f;

    float ContactArea = PI * FMath::Sqrt(AppliedForce * BodyRadius / ElasticModulus);
    return FMath::Clamp(ContactArea, 1.0f, 1000.0f); // cm²
}

float UOHAlgoUtils::CalculateContactPressure(float AppliedForce, float ContactArea) {
    if (ContactArea <= 0)
        return 0.0f;

    return AppliedForce / ContactArea;
}

float UOHAlgoUtils::CalculateImpactForce(float Mass, float Velocity, float ContactDuration) {
    if (ContactDuration <= 0)
        return 0.0f;

    // F = ma = m(Δv/Δt) = m*v/t for complete stop
    return Mass * Velocity / ContactDuration;
}

void UOHAlgoUtils::DecomposeVelocity(const FVector& Velocity, const FVector& Normal, FVector& OutNormalComponent,
                                     FVector& OutTangentialComponent, float& OutNormalSpeed,
                                     float& OutTangentialSpeed) {
    // Project velocity onto normal
    OutNormalSpeed = FVector::DotProduct(Velocity, Normal);
    OutNormalComponent = Normal * OutNormalSpeed;

    // Tangential is remainder
    OutTangentialComponent = Velocity - OutNormalComponent;
    OutTangentialSpeed = OutTangentialComponent.Size();
}

void UOHAlgoUtils::CalculateShearComponents(const FVector& Velocity, const FVector& SurfaceNormal,
                                            FVector& OutShearDirection, float& OutShearMagnitude) {
    FVector NormalComp, TangentialComp;
    float NormalSpeed, TangentialSpeed;

    DecomposeVelocity(Velocity, SurfaceNormal, NormalComp, TangentialComp, NormalSpeed, TangentialSpeed);

    OutShearDirection = TangentialComp.GetSafeNormal();
    OutShearMagnitude = TangentialSpeed;
}

FVector UOHAlgoUtils::CalculateVelocityReflection(const FVector& IncomingVelocity, const FVector& SurfaceNormal,
                                                  float Restitution) {
    // v' = v - (1 + e) * n * (v · n)
    float NormalSpeed = FVector::DotProduct(IncomingVelocity, SurfaceNormal);
    return IncomingVelocity - (1.0f + Restitution) * SurfaceNormal * NormalSpeed;
}

float UOHAlgoUtils::CalculateKineticEnergy(float Mass, float Velocity) {
    // KE = 1/2 * m * v²
    return 0.5f * Mass * Velocity * Velocity;
}

float UOHAlgoUtils::CalculateKineticEnergyVector(float Mass, const FVector& Velocity) {
    // KE = 1/2 * m * |v|²
    return 0.5f * Mass * Velocity.SizeSquared();
}

float UOHAlgoUtils::CalculateRotationalEnergy(float MomentOfInertia, const FVector& AngularVelocity) {
    // KE_rot = 1/2 * I * ω²
    return 0.5f * MomentOfInertia * AngularVelocity.SizeSquared();
}

float UOHAlgoUtils::CalculateElasticEnergyFraction(float Restitution) {
    // Elastic energy fraction = e²
    return Restitution * Restitution;
}

void UOHAlgoUtils::DistributeImpactEnergy(float TotalEnergy, float Restitution, float& OutElasticEnergy,
                                          float& OutPlasticEnergy, float& OutHeatEnergy, float& OutSoundEnergy) {
    float ElasticFraction = CalculateElasticEnergyFraction(Restitution);
    float PlasticFraction = 1.0f - ElasticFraction;

    // Elastic energy (returned as kinetic)
    OutElasticEnergy = TotalEnergy * ElasticFraction;

    // Plastic deformation (60% of inelastic)
    OutPlasticEnergy = TotalEnergy * PlasticFraction * 0.6f;

    // Heat (30% of inelastic)
    OutHeatEnergy = TotalEnergy * PlasticFraction * 0.3f;

    // Sound (10% of inelastic)
    OutSoundEnergy = TotalEnergy * PlasticFraction * 0.1f;
}

FVector UOHAlgoUtils::CalculateMomentum(float Mass, const FVector& Velocity) {
    // p = m * v
    return Mass * Velocity;
}

float UOHAlgoUtils::CalculateImpulseMagnitude(float RelativeNormalVelocity, float ReducedMass, float Restitution) {
    // J = m_reduced * (1 + e) * v_rel
    return ReducedMass * (1.0f + Restitution) * RelativeNormalVelocity;
}

FVector UOHAlgoUtils::CalculateImpulseWithFriction(const FVector& RelativeVelocity, const FVector& CollisionNormal,
                                                   float Mass1, float Mass2, float Restitution, float StaticFriction,
                                                   float KineticFriction) {
    // Calculate normal impulse
    float VrelNormal = FVector::DotProduct(RelativeVelocity, CollisionNormal);
    if (VrelNormal >= 0.0f)
        return FVector::ZeroVector; // Separating

    float ReducedMass = CalculateReducedMass(Mass1, Mass2);
    float ImpulseMagnitude = ReducedMass * (1.0f + Restitution) * -VrelNormal;
    FVector NormalImpulse = CollisionNormal * ImpulseMagnitude;

    // Calculate tangential (friction) impulse
    FVector VrelTangent = RelativeVelocity - CollisionNormal * VrelNormal;
    float VrelTangentMag = VrelTangent.Size();

    if (VrelTangentMag > KINDA_SMALL_NUMBER) {
        FVector TangentDir = VrelTangent / VrelTangentMag;

        // Coulomb friction model
        float MaxStaticFriction = StaticFriction * ImpulseMagnitude;
        float TangentialImpulse = ReducedMass * VrelTangentMag;

        FVector FrictionImpulse;
        if (TangentialImpulse <= MaxStaticFriction) {
            // Static friction - stops tangential motion
            FrictionImpulse = -TangentDir * TangentialImpulse;
        } else {
            // Kinetic friction
            FrictionImpulse = -TangentDir * KineticFriction * ImpulseMagnitude;
        }

        return NormalImpulse + FrictionImpulse;
    }

    return NormalImpulse;
}

FMaterialPhysicsProperties UOHAlgoUtils::GetBiologicalMaterialProperties(EOHBiologicalMaterial MaterialType) {
    static TMap<EOHBiologicalMaterial, FMaterialPhysicsProperties> MaterialDB = GetMaterialDatabase();

    if (const FMaterialPhysicsProperties* Props = MaterialDB.Find(MaterialType)) {
        return *Props;
    }

    // Return default skin properties
    return MaterialDB[EOHBiologicalMaterial::Skin];
}

TMap<EOHBiologicalMaterial, FMaterialPhysicsProperties> UOHAlgoUtils::GetMaterialDatabase() {
    TMap<EOHBiologicalMaterial, FMaterialPhysicsProperties> Database;

    // Skin - Based on biomechanical studies
    FMaterialPhysicsProperties Skin;
    Skin.Density = 1100.0f;      // kg/m³
    Skin.YoungsModulus = 0.1e6f; // 0.1 MPa (varies 0.05-0.15 MPa)
    Skin.PoissonsRatio = 0.46f;
    Skin.StaticFriction = 0.61f; // Dry skin on various surfaces
    Skin.KineticFriction = 0.42f;
    Skin.Restitution = 0.05f;    // Very low - skin absorbs impact
    Skin.DragCoefficient = 0.7f; // Rough surface
    Database.Add(EOHBiologicalMaterial::Skin, Skin);

    // Muscle - Active tissue properties
    FMaterialPhysicsProperties Muscle;
    Muscle.Density = 1060.0f;       // kg/m³
    Muscle.YoungsModulus = 0.01e6f; // 10 kPa (relaxed state)
    Muscle.PoissonsRatio = 0.45f;
    Muscle.StaticFriction = 0.5f;
    Muscle.KineticFriction = 0.35f;
    Muscle.Restitution = 0.1f;
    Muscle.DragCoefficient = 0.6f;
    Database.Add(EOHBiologicalMaterial::Muscle, Muscle);

    // Fat tissue
    FMaterialPhysicsProperties Fat;
    Fat.Density = 920.0f;         // kg/m³
    Fat.YoungsModulus = 0.002e6f; // 2 kPa - very soft
    Fat.PoissonsRatio = 0.48f;    // Nearly incompressible
    Fat.StaticFriction = 0.4f;
    Fat.KineticFriction = 0.3f;
    Fat.Restitution = 0.15f; // Some bounce
    Fat.DragCoefficient = 0.5f;
    Database.Add(EOHBiologicalMaterial::Fat, Fat);

    // Bone - Cortical bone properties
    FMaterialPhysicsProperties Bone;
    Bone.Density = 1900.0f;     // kg/m³
    Bone.YoungsModulus = 20e9f; // 20 GPa
    Bone.PoissonsRatio = 0.3f;
    Bone.StaticFriction = 0.3f;
    Bone.KineticFriction = 0.2f;
    Bone.Restitution = 0.3f; // Higher than soft tissue
    Bone.DragCoefficient = 0.4f;
    Database.Add(EOHBiologicalMaterial::Bone, Bone);

    // Cartilage
    FMaterialPhysicsProperties Cartilage;
    Cartilage.Density = 1100.0f;      // kg/m³
    Cartilage.YoungsModulus = 0.7e6f; // 0.7 MPa
    Cartilage.PoissonsRatio = 0.4f;
    Cartilage.StaticFriction = 0.02f;   // Very low - synovial fluid
    Cartilage.KineticFriction = 0.002f; // Extremely low when lubricated
    Cartilage.Restitution = 0.1f;
    Cartilage.DragCoefficient = 0.3f;
    Database.Add(EOHBiologicalMaterial::Cartilage, Cartilage);

    // Add more materials...

    return Database;
}

TMap<TPair<EPhysicalSurface, EPhysicalSurface>, TPair<float, float>> UOHAlgoUtils::GetFrictionDatabase() {
    static TMap<TPair<EPhysicalSurface, EPhysicalSurface>, TPair<float, float>> FrictionDB;

    if (FrictionDB.Num() == 0) {
        // Initialize friction coefficient database
        // Format: {SurfaceA, SurfaceB} -> {StaticFriction, KineticFriction}

        auto AddFriction = [&](EPhysicalSurface A, EPhysicalSurface B, float StaticFriction, float KineticFriction) {
            FrictionDB.Add({A, B}, {StaticFriction, KineticFriction});
            if (A != B) // Add symmetric entry
            {
                FrictionDB.Add({B, A}, {StaticFriction, KineticFriction});
            }
        };

        // Skin (Default) interactions
        AddFriction(SurfaceType_Default, SurfaceType_Default, 0.4f, 0.3f); // Skin on skin
        AddFriction(SurfaceType_Default, SurfaceType1, 0.6f, 0.4f);        // Skin on concrete
        AddFriction(SurfaceType_Default, SurfaceType2, 0.8f, 0.6f);        // Skin on metal
        AddFriction(SurfaceType_Default, SurfaceType3, 0.3f, 0.2f);        // Skin on water
        AddFriction(SurfaceType_Default, SurfaceType4, 0.9f, 0.7f);        // Skin on wood
        AddFriction(SurfaceType_Default, SurfaceType5, 0.7f, 0.5f);        // Skin on grass
        AddFriction(SurfaceType_Default, SurfaceType6, 0.1f, 0.05f);       // Skin on ice

        // Concrete interactions
        AddFriction(SurfaceType1, SurfaceType1, 1.0f, 0.8f); // Concrete on concrete
        AddFriction(SurfaceType1, SurfaceType2, 0.7f, 0.5f); // Concrete on metal
        AddFriction(SurfaceType1, SurfaceType4, 0.6f, 0.4f); // Concrete on wood

        // Metal interactions
        AddFriction(SurfaceType2, SurfaceType2, 0.8f, 0.6f);  // Metal on metal
        AddFriction(SurfaceType2, SurfaceType4, 0.5f, 0.3f);  // Metal on wood
        AddFriction(SurfaceType2, SurfaceType6, 0.1f, 0.05f); // Metal on ice

        // Water interactions (low friction)
        AddFriction(SurfaceType3, SurfaceType3, 0.1f, 0.05f);  // Water on water
        AddFriction(SurfaceType3, SurfaceType4, 0.2f, 0.1f);   // Water on wood
        AddFriction(SurfaceType3, SurfaceType6, 0.05f, 0.02f); // Water on ice

        // Wood interactions
        AddFriction(SurfaceType4, SurfaceType4, 0.7f, 0.5f); // Wood on wood
        AddFriction(SurfaceType4, SurfaceType5, 0.8f, 0.6f); // Wood on grass

        // Grass interactions
        AddFriction(SurfaceType5, SurfaceType5, 0.9f, 0.7f); // Grass on grass
        AddFriction(SurfaceType5, SurfaceType6, 0.4f, 0.2f); // Grass on ice

        // Ice interactions (very low friction)
        AddFriction(SurfaceType6, SurfaceType6, 0.1f, 0.02f); // Ice on ice
    }

    return FrictionDB;
}

TPair<float, float> UOHAlgoUtils::GetFrictionCoefficient(EPhysicalSurface SurfaceA, EPhysicalSurface SurfaceB) {
    TMap<TPair<EPhysicalSurface, EPhysicalSurface>, TPair<float, float>> FrictionDB = GetFrictionDatabase();

    TPair<EPhysicalSurface, EPhysicalSurface> SurfacePair = {SurfaceA, SurfaceB};

    if (TPair<float, float>* Found = FrictionDB.Find(SurfacePair)) {
        return *Found;
    }

    // Default friction values if not found in database
    return {0.5f, 0.3f}; // {Static, Kinetic}
}

float UOHAlgoUtils::DeriveRestitutionCoefficient(float YoungsModulus1, float YoungsModulus2, float PoissonsRatio1,
                                                 float PoissonsRatio2) {
    // Based on Hertz contact theory and energy dissipation
    // Effective modulus
    float E1_eff = YoungsModulus1 / (1.0f - PoissonsRatio1 * PoissonsRatio1);
    float E2_eff = YoungsModulus2 / (1.0f - PoissonsRatio2 * PoissonsRatio2);
    float E_eff = 1.0f / (1.0f / E1_eff + 1.0f / E2_eff);

    // Empirical formula for biological materials
    // Higher stiffness = higher restitution
    float StiffnessRatio = E_eff / 1e6f; // Normalize to MPa
    float BaseRestitution = FMath::Clamp(0.05f * FMath::Loge(StiffnessRatio + 1.0f), 0.0f, 0.95f);

    // Account for viscoelastic damping in biological materials
    float DampingFactor = 0.8f; // Biological materials have high damping

    return BaseRestitution * DampingFactor;
}

void UOHAlgoUtils::DeriveFrictionCoefficients(EPhysicalSurface Surface1, EPhysicalSurface Surface2,
                                              float& OutStaticFriction, float& OutKineticFriction) {
    static TMap<TPair<EPhysicalSurface, EPhysicalSurface>, TPair<float, float>> FrictionDB = GetFrictionDatabase();

    // Make pair ordered for consistent lookup
    TPair<EPhysicalSurface, EPhysicalSurface> Key = Surface1 <= Surface2
                                                        ? TPair<EPhysicalSurface, EPhysicalSurface>(Surface1, Surface2)
                                                        : TPair<EPhysicalSurface, EPhysicalSurface>(Surface2, Surface1);

    if (const TPair<float, float>* Coeffs = FrictionDB.Find(Key)) {
        OutStaticFriction = Coeffs->Key;
        OutKineticFriction = Coeffs->Value;
    } else {
        // Default values
        OutStaticFriction = 0.6f;
        OutKineticFriction = 0.4f;
    }
}

float UOHAlgoUtils::CalculateDragCoefficient(EOHShapeType ShapeType, float AspectRatio) {
    // Based on fluid dynamics data
    switch (ShapeType) {
    case EOHShapeType::Sphere:
        return 0.47f;

    case EOHShapeType::Cylinder:
        // Depends on aspect ratio and orientation
        return FMath::Lerp(0.82f, 1.15f, FMath::Clamp((AspectRatio - 1.0f) / 10.0f, 0.0f, 1.0f));

    case EOHShapeType::Box:
        return 1.05f;

    case EOHShapeType::Ellipsoid:
        // Streamlined shape
        return 0.2f + 0.3f * (1.0f - 1.0f / AspectRatio);

    case EOHShapeType::Human_Limb:
        // Approximated as cylinder with rounded ends
        return 0.65f;

    case EOHShapeType::Human_Torso:
        // Complex shape, higher drag
        return 0.9f;

    case EOHShapeType::Human_Head:
        // Nearly spherical
        return 0.5f;

    default:
        return 0.7f; // Conservative estimate
    }
}

float UOHAlgoUtils::CalculateCrossSectionalArea(const FVector& BoundsExtent, const FVector& VelocityDirection) {
    // Project bounds onto plane perpendicular to velocity
    FVector VelDir = VelocityDirection.GetSafeNormal();

    // Get two perpendicular vectors in the plane
    FVector Tangent, Bitangent;
    VelDir.FindBestAxisVectors(Tangent, Bitangent);

    // Project extent onto plane
    float Width = FMath::Abs(FVector::DotProduct(BoundsExtent * 2.0f, Tangent));
    float Height = FMath::Abs(FVector::DotProduct(BoundsExtent * 2.0f, Bitangent));

    // Approximate as ellipse (more accurate than rectangle for body parts)
    return PI * Width * Height * 0.25f;
}

float UOHAlgoUtils::EstimateBoneMassFromVolume(const FVector& BoundsExtent, EOHBiologicalMaterial TissueType,
                                               float BonePercentage) {
    // Calculate volume (assuming ellipsoid)
    float Volume = (4.0f / 3.0f) * PI * BoundsExtent.X * BoundsExtent.Y * BoundsExtent.Z;

    // Convert from cm³ to m³
    Volume *= 1e-6f;

    // Get tissue density
    FMaterialPhysicsProperties TissueProps = GetBiologicalMaterialProperties(TissueType);
    FMaterialPhysicsProperties BoneProps = GetBiologicalMaterialProperties(EOHBiologicalMaterial::Bone);

    // Weighted average density
    float AverageDensity = TissueProps.Density * (1.0f - BonePercentage) + BoneProps.Density * BonePercentage;

    // Mass = Density * Volume
    return AverageDensity * Volume;
}

float UOHAlgoUtils::CalculateReducedMass(float Mass1, float Mass2) {
    if (Mass1 <= 0.0f || Mass2 <= 0.0f)
        return 0.0f;

    // Handle infinite mass case (static object)
    if (Mass2 > 1e6f)
        return Mass1;
    if (Mass1 > 1e6f)
        return Mass2;

    return (Mass1 * Mass2) / (Mass1 + Mass2);
}

bool UOHAlgoUtils::ValidatePhysicsParameters(float Restitution, float StaticFriction, float KineticFriction,
                                             FString& OutWarnings) {
    bool bValid = true;
    OutWarnings.Empty();

    // Validate restitution
    if (Restitution < 0.0f || Restitution > 1.0f) {
        OutWarnings += FString::Printf(TEXT("Restitution %.2f out of range [0,1]\n"), Restitution);
        bValid = false;
    }

    // Validate friction coefficients
    if (StaticFriction < 0.0f) {
        OutWarnings += FString::Printf(TEXT("Static friction %.2f cannot be negative\n"), StaticFriction);
        bValid = false;
    }

    if (KineticFriction < 0.0f) {
        OutWarnings += FString::Printf(TEXT("Kinetic friction %.2f cannot be negative\n"), KineticFriction);
        bValid = false;
    }

    if (KineticFriction > StaticFriction) {
        OutWarnings += TEXT("Kinetic friction should not exceed static friction\n");
        bValid = false;
    }

    // Warn about unusual values
    if (StaticFriction > 2.0f) {
        OutWarnings += TEXT("Static friction > 2.0 is unusually high\n");
    }

    return bValid;
}

void UOHAlgoUtils::CalibrateCoefficientsFromMotion(const TArray<FVector>& PreImpactVelocities,
                                                   const TArray<FVector>& PostImpactVelocities,
                                                   const TArray<FVector>& ImpactNormals, float& OutRestitution,
                                                   float& OutFriction) {
    if (PreImpactVelocities.Num() != PostImpactVelocities.Num() || PreImpactVelocities.Num() != ImpactNormals.Num() ||
        PreImpactVelocities.Num() == 0) {
        return;
    }

    float TotalRestitution = 0.0f;
    float TotalFriction = 0.0f;
    int32 ValidSamples = 0;

    for (int32 i = 0; i < PreImpactVelocities.Num(); i++) {
        const FVector& V1 = PreImpactVelocities[i];
        const FVector& V2 = PostImpactVelocities[i];
        const FVector& Normal = ImpactNormals[i].GetSafeNormal();

        // Calculate normal components
        float V1n = FVector::DotProduct(V1, Normal);
        float V2n = FVector::DotProduct(V2, Normal);

        if (V1n < -KINDA_SMALL_NUMBER) // Approaching
        {
            // Restitution = -V2n / V1n
            float e = FMath::Clamp(-V2n / V1n, 0.0f, 1.0f);
            TotalRestitution += e;

            // Calculate tangential components for friction
            FVector V1t = V1 - Normal * V1n;
            FVector V2t = V2 - Normal * V2n;

            if (V1t.Size() > KINDA_SMALL_NUMBER) {
                // Estimate friction from tangential velocity change
                float FrictionEffect = 1.0f - (V2t.Size() / V1t.Size());
                TotalFriction += FMath::Clamp(FrictionEffect, 0.0f, 1.0f);
            }

            ValidSamples++;
        }
    }

    if (ValidSamples > 0) {
        OutRestitution = TotalRestitution / ValidSamples;
        OutFriction = TotalFriction / ValidSamples * 0.6f; // Approximate static from kinetic
    }
}

FVector UOHAlgoUtils::CalculateImpulseFromMomentum(const FVector& InitialMomentum, const FVector& FinalMomentum) {
    // J = Δp = p_f - p_i
    return FinalMomentum - InitialMomentum;
}

float UOHAlgoUtils::CalculateRestitutionFromVelocities(float RelativeVelocityBefore, float RelativeVelocityAfter) {
    if (FMath::IsNearlyZero(RelativeVelocityBefore))
        return 0.0f;

    // e = -v_after / v_before
    return -RelativeVelocityAfter / RelativeVelocityBefore;
}

float UOHAlgoUtils::CalculateMomentOfInertia(float Mass, float Radius, float InertiaCoefficient) {
    // I = k * m * r²
    return InertiaCoefficient * Mass * Radius * Radius;
}

FVector UOHAlgoUtils::CalculateTorque(const FVector& Force, const FVector& LeverArm) {
    // τ = r × F
    return FVector::CrossProduct(LeverArm, Force);
}

FVector UOHAlgoUtils::CalculateAngularMomentum(float MomentOfInertia, const FVector& AngularVelocity) {
    // L = I * ω
    return MomentOfInertia * AngularVelocity;
}

FVector UOHAlgoUtils::CalculateAngularImpulse(const FVector& Torque, float Duration) {
    // J_angular = τ * Δt
    return Torque * Duration;
}

float UOHAlgoUtils::CalculateCollisionImpulseMagnitude(float RelativeNormalVelocity, float ReducedMass,
                                                       float Restitution) {
    if (RelativeNormalVelocity <= 0)
        return 0.0f;

    // J = (1 + e) * μ * v_rel
    return (1.0f + Restitution) * ReducedMass * RelativeNormalVelocity;
}

void UOHAlgoUtils::CalculatePostCollisionVelocities(const FVector& Velocity1, const FVector& Velocity2, float Mass1,
                                                    float Mass2, const FVector& CollisionNormal, float Restitution,
                                                    FVector& OutVelocity1, FVector& OutVelocity2) {
    // Calculate relative velocity
    FVector RelativeVelocity = Velocity1 - Velocity2;
    float RelativeNormalSpeed = FVector::DotProduct(RelativeVelocity, CollisionNormal);

    // Calculate impulse
    float ReducedMass = CalculateReducedMass(Mass1, Mass2);
    float ImpulseMagnitude = CalculateCollisionImpulseMagnitude(RelativeNormalSpeed, ReducedMass, Restitution);
    FVector Impulse = CollisionNormal * ImpulseMagnitude;

    // Apply impulse
    OutVelocity1 = Velocity1 - Impulse / Mass1;
    OutVelocity2 = Velocity2 + Impulse / Mass2;
}

void UOHAlgoUtils::AnalyzeContactPatch(const FHitResult& Hit, float& OutContactArea, float& OutPressure,
                                       FVector& OutShearDirection, float& OutShearMagnitude) {
    // Get striking body data
    float StrikingMass = 5.0f;
    FVector Velocity = FVector::ZeroVector;
    float BodyRadius = 10.0f; // Default estimate

    if (USkeletalMeshComponent* SkelMesh = Cast<USkeletalMeshComponent>(Hit.GetComponent())) {
        if (Hit.BoneName != NAME_None) {
            FBodyInstance* Body = SkelMesh->GetBodyInstance(Hit.BoneName);
            if (Body && Body->IsValidBodyInstance()) {
                StrikingMass = Body->GetBodyMass();
                Velocity = Body->GetUnrealWorldVelocity();

                // Estimate body radius from physics asset
                if (UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset()) {
                    int32 BodyIndex = PhysAsset->FindBodyIndex(Hit.BoneName);
                    if (BodyIndex != INDEX_NONE && PhysAsset->SkeletalBodySetups.IsValidIndex(BodyIndex)) {
                        UBodySetup* BodySetup = PhysAsset->SkeletalBodySetups[BodyIndex];
                        FBox Bounds = BodySetup->AggGeom.CalcAABB(FTransform::Identity);
                        BodyRadius = Bounds.GetExtent().GetAbsMin();
                    }
                }
            }
        }
    }

    // Estimate contact area using Hertz contact theory
    // A = π * sqrt(F * R / E)
    // Use extracted functions
    float Force = CalculateImpactForce(StrikingMass, Velocity.Size());
    OutContactArea = CalculateHertzContactArea(Force, BodyRadius);
    OutPressure = CalculateContactPressure(Force, OutContactArea);
    CalculateShearComponents(Velocity, Hit.ImpactNormal, OutShearDirection, OutShearMagnitude);
}

void UOHAlgoUtils::CalculateImpactEnergyDistribution(const FHitResult& Hit, float& OutKineticEnergyTransfer,
                                                     float& OutRotationalEnergy, float& OutDeformationEnergy,
                                                     float& OutSoundEnergy, float& OutHeatEnergy) {
    // Initialize
    OutKineticEnergyTransfer = 0.0f;
    OutRotationalEnergy = 0.0f;
    OutDeformationEnergy = 0.0f;
    OutSoundEnergy = 0.0f;
    OutHeatEnergy = 0.0f;

    // Get physics data
    float Mass = 5.0f;
    FVector LinearVel = FVector::ZeroVector;
    FVector AngularVel = FVector::ZeroVector;
    float Restitution = 0.3f;

    if (Hit.PhysMaterial.IsValid()) {
        Restitution = Hit.PhysMaterial->Restitution;
    }

    if (USkeletalMeshComponent* SkelMesh = Cast<USkeletalMeshComponent>(Hit.GetComponent())) {
        if (Hit.BoneName != NAME_None) {
            FBodyInstance* Body = SkelMesh->GetBodyInstance(Hit.BoneName);
            if (Body && Body->IsValidBodyInstance()) {
                Mass = Body->GetBodyMass();
                LinearVel = Body->GetUnrealWorldVelocity();
                AngularVel = Body->GetUnrealWorldAngularVelocityInRadians();
            }
        }
    }

    // Calculate energies using functions
    float LinearKE = CalculateKineticEnergyVector(Mass, LinearVel);
    float MomentOfInertia = CalculateMomentOfInertia(Mass, 10.0f); // Estimated radius
    float AngularKE = CalculateRotationalEnergy(MomentOfInertia, AngularVel);
    float TotalEnergy = LinearKE + AngularKE;

    // Distribute energy
    float ElasticEnergy, PlasticEnergy;
    DistributeImpactEnergy(TotalEnergy, Restitution, ElasticEnergy, PlasticEnergy, OutHeatEnergy, OutSoundEnergy);
    OutKineticEnergyTransfer = ElasticEnergy * 0.7f;
    OutRotationalEnergy = ElasticEnergy * 0.3f;
    OutDeformationEnergy = PlasticEnergy;
}

FVector UOHAlgoUtils::ComputeLinearJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older) {
    return ComputeLinearJerkFromSamples<FOHMotionSample>(Newer, Older);
}

FVector UOHAlgoUtils::ComputeAngularJerkFromSamples(const FOHMotionSample& Newer, const FOHMotionSample& Older) {
    return ComputeAngularJerkFromSamples<FOHMotionSample>(Newer, Older);
}

FVector UOHAlgoUtils::EstimateJerk(const TArray<FOHMotionSample>& Samples, float DeltaTime) {
    return EstimateJerk<FOHMotionSample>(Samples, DeltaTime);
}

float UOHAlgoUtils::EstimateCurvature(const FOHBoneMotionData& MotionData, int32 SampleWindow) {
    if (MotionData.MotionHistory.NumFrames() < 3)
        return 0.0f;

    // Get three consecutive samples
    const FOHMotionFrameSample* S0 =
        MotionData.MotionHistory.GetLatest(FMath::Min(2, MotionData.MotionHistory.NumFrames() - 1));
    const FOHMotionFrameSample* S1 = MotionData.MotionHistory.GetLatest(1);
    const FOHMotionFrameSample* S2 = MotionData.MotionHistory.GetLatest(0);

    if (!S0 || !S1 || !S2)
        return 0.0f;

    // Calculate curvature using velocity and acceleration
    const FVector& Vel = S2->WorldLinearVelocity;
    const FVector& Accel = S2->WorldLinearAcceleration;

    float VelMag = Vel.Size();
    if (VelMag < KINDA_SMALL_NUMBER)
        return 0.0f;

    // Curvature = |v × a| / |v|³
    FVector Cross = FVector::CrossProduct(Vel, Accel);
    return Cross.Size() / FMath::Pow(VelMag, 3.0f);
}

float UOHAlgoUtils::EstimateCurvature(const TArray<FOHMotionSample>& Samples) {
    return EstimateCurvature<FOHMotionSample>(Samples);
}

FVector UOHAlgoUtils::SmoothVelocityEMA(const TArray<FOHMotionSample>& Samples, float Alpha) {
    return SmoothVelocityEMA<FOHMotionSample>(Samples, Alpha);
}

FVector UOHAlgoUtils::SmoothVelocityEMA(const FOHBoneMotionData& MotionData, float Alpha) {
    if (MotionData.GetHistoryDepth() < 2) {
        return MotionData.GetLatestSample() ? MotionData.GetLatestSample()->WorldLinearVelocity : FVector::ZeroVector;
    }

    FVector SmoothedVelocity = FVector::ZeroVector;
    float Weight = 1.0f;
    float TotalWeight = 0.0f;

    // Apply exponential moving average over recent samples
    int32 SampleCount = FMath::Min(5, MotionData.GetHistoryDepth());
    for (int32 i = 0; i < SampleCount; i++) {
        if (const FOHMotionFrameSample* Sample = MotionData.GetHistoricalSample(i)) {
            SmoothedVelocity += Sample->WorldLinearVelocity * Weight;
            TotalWeight += Weight;
            Weight *= (1.0f - Alpha); // Exponential decay
        }
    }

    return TotalWeight > 0.0f ? SmoothedVelocity / TotalWeight : FVector::ZeroVector;
}

bool UOHAlgoUtils::IsVelocityZeroCrossing(const TArray<FOHMotionSample>& Samples) {
    return IsVelocityZeroCrossing<FOHMotionSample>(Samples);
}

bool UOHAlgoUtils::IsVelocityZeroCrossing(const FOHBoneMotionData& MotionData) {
    if (MotionData.GetHistoryDepth() < 3)
        return false;

    const FOHMotionFrameSample* Current = MotionData.GetHistoricalSample(0);
    const FOHMotionFrameSample* Previous1 = MotionData.GetHistoricalSample(1);
    const FOHMotionFrameSample* Previous2 = MotionData.GetHistoricalSample(2);

    if (!Current || !Previous1 || !Previous2)
        return false;

    // Check for velocity magnitude crossing near zero
    float CurrentSpeed = Current->LocalLinearVelocity.Size();
    float Previous1Speed = Previous1->LocalLinearVelocity.Size();
    float Previous2Speed = Previous2->LocalLinearVelocity.Size();

    const float ZeroThreshold = 10.0f; // cm/s

    // Detect transition through low velocity
    return (Previous2Speed > ZeroThreshold && CurrentSpeed < ZeroThreshold) ||
           (Previous2Speed < ZeroThreshold && CurrentSpeed > ZeroThreshold) ||
           (Previous1Speed < ZeroThreshold && (Previous2Speed > ZeroThreshold || CurrentSpeed > ZeroThreshold));
}

float UOHAlgoUtils::GetTrajectoryDeviation(const TArray<FOHMotionSample>& Samples) {
    return GetTrajectoryDeviation<FOHMotionSample>(Samples);
}

#pragma endregion

#pragma region MotionPredictors

FVector UOHAlgoUtils::PredictLinear(const FOHMotionSample& Current, float PredictTime) {
    return PredictLinear<FOHMotionSample>(Current, PredictTime);
}

FVector UOHAlgoUtils::PredictQuadratic(const FOHMotionSample& Current, float PredictTime) {
    return PredictQuadratic<FOHMotionSample>(Current, PredictTime);
}

FVector UOHAlgoUtils::PredictDamped(const FOHMotionSample& Current, float PredictTime, float DampingRatio) {
    return PredictDamped<FOHMotionSample>(Current, PredictTime, DampingRatio);
}

FVector UOHAlgoUtils::PredictCurvedFromSamples(const TArray<FOHMotionSample>& Samples, float PredictTime) {
    return PredictCurvedFromSamples<FOHMotionSample>(Samples, PredictTime);
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

TArray<FVector> UOHAlgoUtils::GetQuadraticBezierControlPointsFromBoneData(const FOHBoneMotionData& BoneData,
                                                                          float PredictTime) {
    const FVector P0 = BoneData.GetCurrentPosition();
    const FVector P2 = PredictQuadratic(*BoneData.GetLatestSample(), PredictTime);
    const FVector P1 = P0 + BoneData.GetVelocity(EOHReferenceSpace::WorldSpace) * (PredictTime * 0.5f);

    return {P0, P1, P2};
}

TArray<FVector> UOHAlgoUtils::GetCubicBezierControlPointsFromBoneData(const FOHBoneData& BoneData, float PredictTime) {
    const FVector P0 = BoneData.GetCurrentPosition();
    const FVector P3 = PredictQuadratic(BoneData.GetLatestMotionSample(), PredictTime);
    const FVector P1 = P0 + BoneData.GetBodyLinearVelocity() * (PredictTime / 3.f);
    const FVector P2 = P3 - BoneData.GetBodyLinearVelocity() * (PredictTime / 3.f);

    return {P0, P1, P2, P3};
}

TArray<FVector> UOHAlgoUtils::GetCubicBezierControlPointsFromBoneData(const FOHBoneMotionData& BoneData,
                                                                      float PredictTime) {
    const FVector P0 = BoneData.GetCurrentPosition();
    const FVector P3 = PredictQuadratic(*BoneData.GetLatestSample(), PredictTime);
    const FVector P1 = P0 + BoneData.GetVelocity(EOHReferenceSpace::WorldSpace) * (PredictTime / 3.f);
    const FVector P2 = P3 - BoneData.GetVelocity(EOHReferenceSpace::WorldSpace) * (PredictTime / 3.f);

    return {P0, P1, P2, P3};
}

float UOHAlgoUtils::EstimateTimeToReachTargetLinear(const FOHMotionSample& Current, const FVector& TargetPosition) {
    return EstimateTimeToReachTargetLinear<FOHMotionSample>(Current, TargetPosition);
}

float UOHAlgoUtils::EstimateTimeToStopDamped(const FOHMotionSample& Current, float DampingRatio) {
    return EstimateTimeToStopDamped<FOHMotionSample>(Current, DampingRatio);
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

float UOHAlgoUtils::CalculateMotionQuality(const FOHBoneMotionData& MotionData) {
    if (MotionData.GetHistoryDepth() < 3)
        return 0.0f;

    // Multi-metric quality assessment
    float VelocityConsistency = 0.0f;
    float AccelerationStability = 0.0f;
    float DirectionalStability = 0.0f;

    int32 SampleCount = FMath::Min(10, MotionData.GetHistoryDepth());
    int32 ValidComparisons = 0;

    // Calculate velocity variance (local space for character-relative motion)
    for (int32 i = 0; i < SampleCount - 1; i++) {
        const FOHMotionFrameSample* Current = MotionData.GetHistoricalSample(i);
        const FOHMotionFrameSample* Next = MotionData.GetHistoricalSample(i + 1);

        if (Current && Next) {
            // Velocity consistency (lower variance = higher quality)
            FVector VelDiff = Current->LocalLinearVelocity - Next->LocalLinearVelocity;
            VelocityConsistency += VelDiff.SizeSquared();

            // Acceleration stability
            FVector AccelDiff = Current->LocalLinearAcceleration - Next->LocalLinearAcceleration;
            AccelerationStability += AccelDiff.SizeSquared();

            // Directional stability
            if (!Current->LocalLinearVelocity.IsNearlyZero() && !Next->LocalLinearVelocity.IsNearlyZero()) {
                float DotProduct = FVector::DotProduct(Current->LocalLinearVelocity.GetSafeNormal(),
                                                       Next->LocalLinearVelocity.GetSafeNormal());
                DirectionalStability += FMath::Clamp(DotProduct, 0.0f, 1.0f);
            }

            ValidComparisons++;
        }
    }

    if (ValidComparisons == 0)
        return 0.0f;

    // Normalize metrics
    VelocityConsistency = FMath::Clamp(1.0f - (VelocityConsistency / (ValidComparisons * 10000.0f)), 0.0f, 1.0f);
    AccelerationStability = FMath::Clamp(1.0f - (AccelerationStability / (ValidComparisons * 50000.0f)), 0.0f, 1.0f);
    DirectionalStability = DirectionalStability / ValidComparisons;

    // Weighted combination
    return (VelocityConsistency * 0.4f + AccelerationStability * 0.3f + DirectionalStability * 0.3f);
}

float UOHAlgoUtils::EstimateTimeToReachTarget(const FOHBoneMotionData& MotionData, const FVector& TargetPosition) {
    const FOHMotionFrameSample* Latest = MotionData.GetLatestSample();
    if (!Latest)
        return -1.0f;

    FVector ToTarget = TargetPosition - Latest->WorldPosition;
    float Distance = ToTarget.Size();

    if (Distance < 1.0f)
        return 0.0f; // Already at target

    FVector CurrentVelocity = Latest->WorldLinearVelocity;
    FVector CurrentAcceleration = Latest->WorldLinearAcceleration;

    // Project velocity onto target direction
    FVector TargetDirection = ToTarget.GetSafeNormal();
    float VelocityTowardTarget = FVector::DotProduct(CurrentVelocity, TargetDirection);
    float AccelerationTowardTarget = FVector::DotProduct(CurrentAcceleration, TargetDirection);

    // If moving away from target, estimate when direction might change
    if (VelocityTowardTarget <= 0.0f) {
        if (AccelerationTowardTarget > 0.0f) {
            // Time to stop and reverse
            float TimeToStop = -VelocityTowardTarget / AccelerationTowardTarget;
            float DistanceWhileStopping =
                VelocityTowardTarget * TimeToStop + 0.5f * AccelerationTowardTarget * TimeToStop * TimeToStop;
            float RemainingDistance = Distance + FMath::Abs(DistanceWhileStopping);

            // Estimate time from stopping point
            float TimeFromStop = FMath::Sqrt(2.0f * RemainingDistance / FMath::Max(AccelerationTowardTarget, 1.0f));
            return TimeToStop + TimeFromStop;
        } else {
            return -1.0f; // Moving away with no acceleration toward target
        }
    }

    // Quadratic formula for motion toward target
    // Distance = VelocityTowardTarget * t + 0.5 * AccelerationTowardTarget * t^2
    if (FMath::Abs(AccelerationTowardTarget) > KINDA_SMALL_NUMBER) {
        float Discriminant = VelocityTowardTarget * VelocityTowardTarget + 2.0f * AccelerationTowardTarget * Distance;
        if (Discriminant >= 0.0f) {
            float T1 = (-VelocityTowardTarget + FMath::Sqrt(Discriminant)) / AccelerationTowardTarget;
            float T2 = (-VelocityTowardTarget - FMath::Sqrt(Discriminant)) / AccelerationTowardTarget;

            // Return the positive, smaller time
            if (T1 > 0.0f && T2 > 0.0f)
                return FMath::Min(T1, T2);
            else if (T1 > 0.0f)
                return T1;
            else if (T2 > 0.0f)
                return T2;
        }
    }

    // Fallback to constant velocity
    return Distance / FMath::Max(VelocityTowardTarget, 1.0f);
}

FVector UOHAlgoUtils::EstimateJerkFromMotionData(const FOHBoneMotionData& MotionData, int32 SampleWindow) {
    if (MotionData.GetHistoryDepth() < SampleWindow + 1)
        return FVector::ZeroVector;

    TArray<FVector> Accelerations;
    TArray<float> DeltaTimes;

    // Collect accelerations from sample window
    for (int32 i = 0; i < SampleWindow; i++) {
        const FOHMotionFrameSample* Current = MotionData.GetHistoricalSample(i);
        const FOHMotionFrameSample* Next = MotionData.GetHistoricalSample(i + 1);

        if (Current && Next && Current->DeltaTime > KINDA_SMALL_NUMBER) {
            FVector Acceleration = (Current->WorldLinearVelocity - Next->WorldLinearVelocity) / Current->DeltaTime;
            Accelerations.Add(Acceleration);
            DeltaTimes.Add(Current->DeltaTime);
        }
    }

    if (Accelerations.Num() < 2)
        return FVector::ZeroVector;

    // Calculate weighted average jerk
    FVector TotalJerk = FVector::ZeroVector;
    float TotalWeight = 0.0f;

    for (int32 i = 0; i < Accelerations.Num() - 1; i++) {
        FVector Jerk = (Accelerations[i] - Accelerations[i + 1]) / DeltaTimes[i];
        float Weight = 1.0f / (i + 1); // More recent samples have higher weight

        TotalJerk += Jerk * Weight;
        TotalWeight += Weight;
    }

    return TotalWeight > 0.0f ? TotalJerk / TotalWeight : FVector::ZeroVector;
}

TArray<FVector> UOHAlgoUtils::GetBezierControlPointsFromBoneData(const FOHBoneMotionData& MotionData, float PredictTime,
                                                                 bool bCubic) {
    TArray<FVector> ControlPoints;
    const FOHMotionFrameSample* Latest = MotionData.GetLatestSample();

    if (!Latest)
        return ControlPoints;

    FVector CurrentPos = Latest->WorldPosition;
    FVector CurrentVel = Latest->WorldLinearVelocity;
    FVector CurrentAccel = Latest->WorldLinearAcceleration;

    if (bCubic) {
        // Cubic Bezier (4 control points)
        ControlPoints.Add(CurrentPos); // P0 - start point

        // P1 - control point based on velocity
        ControlPoints.Add(CurrentPos + CurrentVel * (PredictTime / 3.0f));

        // P2 - control point based on acceleration
        FVector MidPoint =
            CurrentPos + CurrentVel * (PredictTime * 0.5f) + CurrentAccel * (PredictTime * PredictTime * 0.125f);
        ControlPoints.Add(MidPoint);

        // P3 - predicted end point
        FVector EndPoint = CurrentPos + CurrentVel * PredictTime + CurrentAccel * (PredictTime * PredictTime * 0.5f);
        ControlPoints.Add(EndPoint);
    } else {
        // Quadratic Bezier (3 control points)
        ControlPoints.Add(CurrentPos); // P0 - start point

        // P1 - control point
        ControlPoints.Add(CurrentPos + CurrentVel * (PredictTime * 0.5f) +
                          CurrentAccel * (PredictTime * PredictTime * 0.125f));

        // P2 - end point
        ControlPoints.Add(CurrentPos + CurrentVel * PredictTime + CurrentAccel * (PredictTime * PredictTime * 0.5f));
    }

    return ControlPoints;
}

float UOHAlgoUtils::GetTrajectoryDeviation(const FOHBoneMotionData& MotionData, int32 MaxSamples) {
    if (MotionData.GetHistoryDepth() < 3)
        return 0.0f;

    int32 SampleCount = FMath::Min(MaxSamples, MotionData.GetHistoryDepth());
    if (SampleCount < 3)
        return 0.0f;

    // Get start and end points for trajectory line
    const FOHMotionFrameSample* Start = MotionData.GetHistoricalSample(SampleCount - 1);
    const FOHMotionFrameSample* End = MotionData.GetHistoricalSample(0);

    if (!Start || !End)
        return 0.0f;

    FVector TrajectoryLine = End->WorldPosition - Start->WorldPosition;
    float TrajectoryLength = TrajectoryLine.Size();

    if (TrajectoryLength < KINDA_SMALL_NUMBER)
        return 0.0f;

    FVector TrajectoryDirection = TrajectoryLine / TrajectoryLength;

    // Calculate deviation of intermediate points from straight line
    float TotalDeviation = 0.0f;
    int32 ValidPoints = 0;

    for (int32 i = 1; i < SampleCount - 1; i++) {
        const FOHMotionFrameSample* Sample = MotionData.GetHistoricalSample(i);
        if (!Sample)
            continue;

        // Project point onto trajectory line
        FVector ToPoint = Sample->WorldPosition - Start->WorldPosition;
        float ProjectionLength = FVector::DotProduct(ToPoint, TrajectoryDirection);
        FVector ProjectedPoint = Start->WorldPosition + TrajectoryDirection * ProjectionLength;

        // Calculate perpendicular distance
        float Deviation = FVector::Dist(Sample->WorldPosition, ProjectedPoint);
        TotalDeviation += Deviation;
        ValidPoints++;
    }

    return ValidPoints > 0 ? TotalDeviation / ValidPoints : 0.0f;
}

FVector UOHAlgoUtils::PredictCurvedFromMotionData(const FOHBoneMotionData& MotionData, float PredictTime) {
    TArray<FVector> BezierPoints = GetBezierControlPointsFromBoneData(MotionData, PredictTime, true);

    if (BezierPoints.Num() >= 4) {
        // Sample the cubic Bezier curve at t=1 (end point)
        return SampleBezierCubic(BezierPoints[0], BezierPoints[1], BezierPoints[2], BezierPoints[3], 1.0f);
    } else if (BezierPoints.Num() >= 3) {
        // Sample the quadratic Bezier curve at t=1
        return SampleBezierQuadratic(BezierPoints[0], BezierPoints[1], BezierPoints[2], 1.0f);
    }

    // Fallback to linear prediction
    const FOHMotionFrameSample* Latest = MotionData.GetLatestSample();
    return Latest ? PredictLinear(*Latest, PredictTime, false) : FVector::ZeroVector;
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