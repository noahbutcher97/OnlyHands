// Fill out your copyright notice in the Description page of Project Settings.
#include "Animation/OHAnimInstance_Base.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "TimerManager.h"
#include "FunctionLibrary/OHCombatUtils.h"

UOHAnimInstance_Base::UOHAnimInstance_Base()
{
    // Explicitly initialize buffers (resets indices)
    LeftFootBuffer = FFootTraceBuffer();
    RightFootBuffer = FFootTraceBuffer();

    // Initialize all "last known" positions to zero just in case
    LockedLeftFootWorldPos = FVector::ZeroVector;
    LockedRightFootWorldPos = FVector::ZeroVector;
    DampedLeftEffector = FVector::ZeroVector;
    DampedRightEffector = FVector::ZeroVector;
}



void UOHAnimInstance_Base::NativeInitializeAnimation()
{
    Super::NativeInitializeAnimation();
}

// --- Critical damping smoothing ---
static FVector CriticallyDampedSmoothing(const FVector& Current, const FVector& Target, float DeltaTime, float Spring = 80.f, float Damping = 1.2f)
{
    // Standard 2nd order critically damped response
    float Stiffness = Spring;
    float Damp = Damping * 2.f * FMath::Sqrt(Spring);
    FVector Velocity = (Target - Current) * Stiffness * DeltaTime;
    return Current + Velocity / (1.f + Damp * DeltaTime);
}


bool UOHAnimInstance_Base::CapsuleGroundTrace(const USkeletalMeshComponent* Mesh, const FName& BoneName, float MaxDistance,
    float ForwardBias, float CapsuleRadius, float CapsuleHalfHeight, const FVector& Velocity, FHitResult& OutHit,
    bool bDrawDebug)
{
    if (!Mesh) return false;
    FVector Start = Mesh->GetBoneLocation(BoneName);
    FVector VelocityDir = Velocity.IsNearlyZero() ? FVector::ZeroVector : Velocity.GetSafeNormal();
    FVector PredictedStart = Start + (VelocityDir * ForwardBias);
    FVector End = PredictedStart - FVector(0, 0, MaxDistance);

    FCollisionQueryParams Params;
    Params.AddIgnoredActor(Mesh->GetOwner());

    bool bHit = Mesh->GetWorld()->SweepSingleByChannel(
        OutHit, PredictedStart, End, FQuat::Identity, ECC_Visibility,
        FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight), Params);

    if (bDrawDebug)
    {
        FColor TraceColor = bHit ? FColor::Green : FColor::Red;
        DrawDebugCapsule(Mesh->GetWorld(), PredictedStart, CapsuleHalfHeight, CapsuleRadius, FQuat::Identity, TraceColor, false, 0.08f, 0, 1.2f);
        DrawDebugLine(Mesh->GetWorld(), PredictedStart, End, TraceColor, false, 0.08f, 0, 1.2f);
        if (bHit)
        {
            DrawDebugPoint(Mesh->GetWorld(), OutHit.ImpactPoint, 12.f, FColor::Blue, false, 0.12f);
            DrawDebugDirectionalArrow(Mesh->GetWorld(), OutHit.ImpactPoint, OutHit.ImpactPoint + OutHit.ImpactNormal * 30.f, 16.f, FColor::Yellow, false, 0.12f, 0, 1.5f);
        }
    }
    return bHit;
    
}

bool UOHAnimInstance_Base::SphereGroundTrace(const USkeletalMeshComponent* Mesh, const FName& BoneName, float MaxDistance,
    float ForwardBias, float SphereRadius, const FVector& Velocity, FHitResult& OutHit, bool bDrawDebug)
{
    if (!Mesh) return false;
    FVector Start = Mesh->GetBoneLocation(BoneName) + (Velocity.IsNearlyZero() ? FVector::ZeroVector : Velocity.GetSafeNormal()) * ForwardBias;
    FVector End = Start - FVector(0, 0, MaxDistance);
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(Mesh->GetOwner());
    
    bool bHit = Mesh->GetWorld()->SweepSingleByChannel(
        OutHit, Start, End, FQuat::Identity, ECC_Visibility,
        FCollisionShape::MakeSphere(SphereRadius), Params);

    if (bDrawDebug)
    {
        FColor TraceColor = bHit ? FColor::Green : FColor::Red;
        DrawDebugSphere(Mesh->GetWorld(), bHit ? OutHit.ImpactPoint : End, SphereRadius, 8, TraceColor, false, 0.12f);
        DrawDebugLine(Mesh->GetWorld(), Start, End, TraceColor, false, 0.08f, 0, 1.2f);
    }
    return bHit;
}

bool UOHAnimInstance_Base::SafeDoubleTrace(const USkeletalMeshComponent* Mesh, FName Bone, float MaxDist, float Bias,
    const float Radius, const FVector& Velocity, FHitResult& OutMain, FHitResult& OutBackup, float BackupDepth)
{
    bool bMain = SphereGroundTrace(Mesh, Bone, MaxDist, Bias, Radius, Velocity, OutMain, false);
    bool bBackup = SphereGroundTrace(Mesh, Bone, MaxDist + BackupDepth, Bias, Radius, Velocity, OutBackup, false);

    // Use backup if the main misses but backup is close and lower
    if (!bMain && bBackup && FMath::Abs(OutMain.ImpactPoint.Z - OutBackup.ImpactPoint.Z) < 10.f)
    {
        OutMain = OutBackup; bMain = true;
    }
    return bMain;
}

float UOHAnimInstance_Base::Median(const float* Arr, int32 Count)
{
    if (Count < 1) return 0.f;
    TArray<float> Copy;
    Copy.Reserve(Count);
    for (int32 i = 0; i < Count; ++i) Copy.Add(Arr[i]);
    Copy.Sort();
    return Copy[Count / 2];
}

FVector UOHAnimInstance_Base::CatmullRomInterp(const TArray<FVector>& P, float T)
{
    {
        if (P.Num() < 3) return (P.Num() == 2) ? FMath::Lerp(P[0], P[1], T) : (P.Num() ? P[0] : FVector::ZeroVector);
        const FVector& P0 = P[0], &P1 = P[1], &P2 = P[2];
        float t2 = T * T;
        float t3 = t2 * T;
        return 0.5f * ((2 * P1) +
            (-P0 + P2) * T +
            (2 * P0 - 5 * P1 + 4 * P2 - P0) * t2 +
            (-P0 + 3 * P1 - 3 * P2 + P0) * t3);
    }
}

float UOHAnimInstance_Base::FindHighestNearbyFloorZ(const FVector& Center, UWorld* World, float Radius,
    float ProbeDepth)
{
    float MaxZ = Center.Z - ProbeDepth;
    FHitResult Hit;
    for (int32 i = 0; i < 8; ++i)
    {
        FVector Offset = FVector(FMath::Cos(i * PI/4) * Radius, FMath::Sin(i * PI/4) * Radius, 0.f);
        FVector Start = Center + Offset + FVector(0,0,ProbeDepth*0.5f);
        FVector End = Center + Offset - FVector(0,0,ProbeDepth);
        if (World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            MaxZ = FMath::Max(MaxZ, Hit.ImpactPoint.Z);
    }
    return MaxZ;
}

EFootPhase UOHAnimInstance_Base::DetectFootPhase(const FFootTraceBuffer& Buffer) const
{
    FVector Vel = Buffer.GetVelocity();
    float VerticalSpeed = Vel.Z;
    float HorizontalSpeed = FVector(Vel.X, Vel.Y, 0).Size();

    // Use your new thresholds!
    if (HorizontalSpeed > PhaseSwingHorizontalSpeed && VerticalSpeed > PhaseSwingVerticalSpeed)
        return EFootPhase::Swing;
    if (FMath::Abs(VerticalSpeed) < PhasePlantVerticalSpeed && HorizontalSpeed < PhasePlantHorizontalSpeed)
        return EFootPhase::Plant;
    if (VerticalSpeed < PhaseLiftVerticalSpeed)
        return EFootPhase::Lift;
    return EFootPhase::Unknown;
}

float UOHAnimInstance_Base::GetHeelToeBlendAlpha(EFootPhase Phase, const FFootTraceBuffer& Buffer)
{
    switch (Phase)
    {
    case EFootPhase::Swing: return 1.f;
    case EFootPhase::Lift:  return 0.7f;
    case EFootPhase::Plant: return 0.f;
    default: return 0.3f;
    }
}

FVector UOHAnimInstance_Base::CriticallyDampedSmoothing(const FVector& Current, const FVector& Target, float DeltaTime, float Spring,
    float Damping)
{
    float Stiffness = Spring;
    float Damp = Damping * 2.f * FMath::Sqrt(Spring);
    FVector Velocity = (Target - Current) * Stiffness * DeltaTime;
    return Current + Velocity / (1.f + Damp * DeltaTime);
}
void UOHAnimInstance_Base::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    // --- 0. Early outs & handle pointers ---
    APawn* OwnerPawn = TryGetPawnOwner();
    if (!OwnerPawn) return;
    USkeletalMeshComponent* Mesh = GetSkelMeshComponent();
    if (!Mesh) return;
    UWorld* World = Mesh->GetWorld();
    if (!World) return;

    // --- 1. Auto foot size detection (first tick only) ---
    if (!bMeasuredFootLength)
    {
        FVector Ankle = Mesh->GetBoneLocation(FName("foot_l"));
        FVector Toe   = Mesh->GetBoneLocation(FName("ball_l"));
        AutoFootLength = (Toe - Ankle).Size2D();
        if (IKFootSphereRadius <= 0.f) IKFootSphereRadius = FMath::Clamp(AutoFootLength * 0.28f, 4.5f, 10.f);
        if (IKFootCapsuleHalfHeight <= 0.f) IKFootCapsuleHalfHeight = FMath::Clamp(AutoFootLength * 0.48f, 7.0f, 16.f);
        bMeasuredFootLength = true;
    }

    // --- 2. Gather bone world positions ---
    FVector LeftAnkleWS  = Mesh->GetBoneLocation(FName("foot_l"));
    FVector LeftToeWS    = Mesh->GetBoneLocation(FName("ball_l"));
    FVector RightAnkleWS = Mesh->GetBoneLocation(FName("foot_r"));
    FVector RightToeWS   = Mesh->GetBoneLocation(FName("ball_r"));
    FVector PelvisWorld  = Mesh->GetBoneLocation(FName("pelvis"));
    float TimeNow = World->TimeSeconds;

    // --- 3. Update foot trace buffers ---
    LeftFootBuffer.AddSample(LeftAnkleWS, TimeNow);
    RightFootBuffer.AddSample(RightAnkleWS, TimeNow);

    // --- 4. Velocity and stride direction ---
    FVector CharVel = OwnerPawn->GetVelocity();
    float Speed = CharVel.Size();
    FVector MoveDir = CharVel.IsNearlyZero() ? OwnerPawn->GetActorForwardVector() : CharVel.GetSafeNormal();

    // --- 5. Detect stride phase & intent (per foot, robust stride thresholds) ---
    EFootPhase LeftPhase  = DetectFootPhase(LeftFootBuffer);
    EFootPhase RightPhase = DetectFootPhase(RightFootBuffer);
    float LeftSpeed  = LeftFootBuffer.GetVelocity().Size();
    float RightSpeed = RightFootBuffer.GetVelocity().Size();

    // --- 6. Update phase buffers with time/position/speed for stride-aware blending ---
    LeftPhaseBuffer.Add(LeftPhase, TimeNow, LeftAnkleWS, LeftSpeed);
    RightPhaseBuffer.Add(RightPhase, TimeNow, RightAnkleWS, RightSpeed);

    // --- 7. Predict stride phase and duration for ghost/anticipatory blending ---
    EFootPhase PredictedLeftPhase  = LeftPhaseBuffer.PredictNextPhase();
    EFootPhase PredictedRightPhase = RightPhaseBuffer.PredictNextPhase();
    float PredictPhaseTimeL = LeftPhaseBuffer.PredictPhaseChangeTime(TimeNow);
    float PredictPhaseTimeR = RightPhaseBuffer.PredictPhaseChangeTime(TimeNow);
    FVector AvgStrideVelL = LeftPhaseBuffer.GetAvgStrideVelocity();
    FVector AvgStrideVelR = RightPhaseBuffer.GetAvgStrideVelocity();

    // --- 8. Confidence (stride stability), for adaptive smoothing ---
    float PhaseConfidenceL = LeftPhaseBuffer.GetPhaseConfidence();
    float PhaseConfidenceR = RightPhaseBuffer.GetPhaseConfidence();
    LeftFootStrideConfidence = PhaseConfidenceL;
    RightFootStrideConfidence = PhaseConfidenceR;

    // --- 9. Heel-toe blend for realistic roll ---
    float HeelToeAlphaLeft  = GetHeelToeBlendAlpha(LeftPhase, LeftFootBuffer);
    float HeelToeAlphaRight = GetHeelToeBlendAlpha(RightPhase, RightFootBuffer);

    // --- 10. Trace bias for predictive steps ---
    float BiasLeft  = (LeftPhase  == EFootPhase::Plant) ? IKFootForwardBias : IKFootForwardBias * 2.f;
    float BiasRight = (RightPhase == EFootPhase::Plant) ? IKFootForwardBias : IKFootForwardBias * 2.f;
    if (Speed < IdleFreezeSpeed) BiasLeft = BiasRight = FMath::Min(IKFootForwardBias, 12.f);

    // --- 11. Dual ankle/toe trace (safe, backup), environment-aware ---
    FHitResult LAnkleHit, LAnkleBackup, LToeHit, LToeBackup;
    FHitResult RAnkleHit, RAnkleBackup, RToeHit, RToeBackup;
    bool bLAnkleHit = SafeDoubleTrace(Mesh, FName("foot_l"), IKFootProbeDepth, BiasLeft, IKFootSphereRadius, CharVel, LAnkleHit, LAnkleBackup, FloorProbeDepth);
    bool bLToeHit   = SafeDoubleTrace(Mesh, FName("ball_l"), IKFootProbeDepth, BiasLeft, IKFootSphereRadius, CharVel, LToeHit, LToeBackup, FloorProbeDepth);
    bool bRAnkleHit = SafeDoubleTrace(Mesh, FName("foot_r"), IKFootProbeDepth, BiasRight, IKFootSphereRadius, CharVel, RAnkleHit, RAnkleBackup, FloorProbeDepth);
    bool bRToeHit   = SafeDoubleTrace(Mesh, FName("ball_r"), IKFootProbeDepth, BiasRight, IKFootSphereRadius, CharVel, RToeHit, RToeBackup, FloorProbeDepth);

    // --- 12. Robust floor Z blend/median for anti-pop & anti-jitter ---
    float LAnkleZ = bLAnkleHit ? LAnkleHit.ImpactPoint.Z : LeftAnkleWS.Z;
    float LToeZ   = bLToeHit   ? LToeHit.ImpactPoint.Z   : LeftToeWS.Z;
    float RAnkleZ = bRAnkleHit ? RAnkleHit.ImpactPoint.Z : RightAnkleWS.Z;
    float RToeZ   = bRToeHit   ? RToeHit.ImpactPoint.Z   : RightToeWS.Z;

    float BlendL = FMath::Lerp(LAnkleZ, LToeZ, HeelToeAlphaLeft);
    float MaxL   = FMath::Max(LAnkleZ, LToeZ);
    float LFootGroundZ = (FMath::Abs(LAnkleZ - LToeZ) > MaxStairHeight)
        ? FMath::Min(LAnkleZ, LToeZ)
        : FMath::Lerp(BlendL, MaxL, 0.6f);

    float BlendR = FMath::Lerp(RAnkleZ, RToeZ, HeelToeAlphaRight);
    float MaxR   = FMath::Max(RAnkleZ, RToeZ);
    float RFootGroundZ = (FMath::Abs(RAnkleZ - RToeZ) > MaxStairHeight)
        ? FMath::Min(RAnkleZ, RToeZ)
        : FMath::Lerp(BlendR, MaxR, 0.6f);

    // --- 13. Temporal median smoothing (outlier protection) ---
    LFootZBuffer[LFootZBufIndex] = LFootGroundZ;
    if (LFootZBufValidSamples < FootZBufferSize) LFootZBufValidSamples++;
    LFootZBufIndex = (LFootZBufIndex + 1) % FootZBufferSize;
    float MedianLFootZ = Median(LFootZBuffer, LFootZBufValidSamples);

    RFootZBuffer[RFootZBufIndex] = RFootGroundZ;
    if (RFootZBufValidSamples < FootZBufferSize) RFootZBufValidSamples++;
    RFootZBufIndex = (RFootZBufIndex + 1) % FootZBufferSize;
    float MedianRFootZ = Median(RFootZBuffer, RFootZBufValidSamples);

    if (bLFootFirstUpdate || LFootZBufValidSamples < 2) { SmoothedLFootGroundZ = MedianLFootZ; bLFootFirstUpdate = false; }
    else { SmoothedLFootGroundZ = FMath::FInterpTo(SmoothedLFootGroundZ, MedianLFootZ, DeltaSeconds, OffsetInterpSpeed); }
    if (bRFootFirstUpdate || RFootZBufValidSamples < 2) { SmoothedRFootGroundZ = MedianRFootZ; bRFootFirstUpdate = false; }
    else { SmoothedRFootGroundZ = FMath::FInterpTo(SmoothedRFootGroundZ, MedianRFootZ, DeltaSeconds, OffsetInterpSpeed); }

    // --- 14. Heel-toe blend world targets (placement) ---
    FVector LTargetWS = FMath::Lerp(LeftAnkleWS, LeftToeWS, HeelToeAlphaLeft);
    FVector RTargetWS = FMath::Lerp(RightAnkleWS, RightToeWS, HeelToeAlphaRight);
    LTargetWS.Z = SmoothedLFootGroundZ + LeftFootGroundOffset;
    RTargetWS.Z = SmoothedRFootGroundZ + RightFootGroundOffset;

    // --- 15. Trajectory spline smoothing (Catmull-Rom) ---
    if (LeftEffectorSpline.Num() < SplineMax)  { for(int i=LeftEffectorSpline.Num();i<SplineMax;++i)  LeftEffectorSpline.Add(LTargetWS); }
    if (RightEffectorSpline.Num() < SplineMax) { for(int i=RightEffectorSpline.Num();i<SplineMax;++i) RightEffectorSpline.Add(RTargetWS); }
    LeftEffectorSpline.RemoveAt(0);  LeftEffectorSpline.Add(LTargetWS);
    RightEffectorSpline.RemoveAt(0); RightEffectorSpline.Add(RTargetWS);

    float SplineT = 0.7f * GlobalIKSmoothing + 0.25f;
    FVector LSplineEffector = CatmullRomInterp(LeftEffectorSpline, SplineT);
    FVector RSplineEffector = CatmullRomInterp(RightEffectorSpline, SplineT);

    // --- 16. Adaptive stride smoothing (stride confidence based) ---
    float AdaptiveSmoothL = FMath::Lerp(AdaptiveSmoothingMin, AdaptiveSmoothingMax, 1.f - PhaseConfidenceL);
    float AdaptiveSmoothR = FMath::Lerp(AdaptiveSmoothingMin, AdaptiveSmoothingMax, 1.f - PhaseConfidenceR);
    float AdaptiveInterpSpeedL = FMath::Lerp(AdaptiveInterpSpeedMin, AdaptiveInterpSpeedMax, 1.f - PhaseConfidenceL);
    float AdaptiveInterpSpeedR = FMath::Lerp(AdaptiveInterpSpeedMin, AdaptiveInterpSpeedMax, 1.f - PhaseConfidenceR);

    // --- 17. Sticky plant blend/fade (supports plant fade timing) ---
    if (PredictedLeftPhase == EFootPhase::Plant)  { LeftPlantAlpha = 1.f; LastLeftPlantTime = TimeNow; }
    else LeftPlantAlpha  = FMath::FInterpTo(LeftPlantAlpha,  0.f, DeltaSeconds, PlantFadeSpeed);
    if (PredictedRightPhase == EFootPhase::Plant) { RightPlantAlpha = 1.f; LastRightPlantTime = TimeNow; }
    else RightPlantAlpha = FMath::FInterpTo(RightPlantAlpha, 0.f, DeltaSeconds, PlantFadeSpeed);

    // --- 18. Plant lock/hysteresis ---
    if (LeftPhase == EFootPhase::Plant && bLAnkleHit)
    {
        LastValidLeftImpact = LAnkleHit.ImpactPoint;
        LastValidLeftNormal = bLToeHit
            ? ((LAnkleHit.ImpactNormal + LToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : LAnkleHit.ImpactNormal;
        LeftFootLockFrames = LockHysteresisFrames;
    }
    else if (LeftFootLockFrames > 0) { LeftFootLockFrames--; }
    else { LastValidLeftImpact = LTargetWS; LastValidLeftNormal = FVector::UpVector; }

    if (RightPhase == EFootPhase::Plant && bRAnkleHit)
    {
        LastValidRightImpact = RAnkleHit.ImpactPoint;
        LastValidRightNormal = bRToeHit
            ? ((RAnkleHit.ImpactNormal + RToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : RAnkleHit.ImpactNormal;
        RightFootLockFrames = LockHysteresisFrames;
    }
    else if (RightFootLockFrames > 0) { RightFootLockFrames--; }
    else { LastValidRightImpact = RTargetWS; LastValidRightNormal = FVector::UpVector; }

    // --- 19. Ghost step/stride anticipation (lookahead for both feet, using respective PredictPhaseTime) ---
    NewLockedLEffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidLeftImpact);
    NewLockedREffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidRightImpact);
    FRotator NewLockedLRot = LastValidLeftNormal.Rotation();
    FRotator NewLockedRRot = LastValidRightNormal.Rotation();

    float GhostStepLookaheadTimeL = FMath::Clamp(PredictPhaseTimeL * 0.4f, 0.06f, 0.14f);
    float GhostStepLookaheadTimeR = FMath::Clamp(PredictPhaseTimeR * 0.4f, 0.06f, 0.14f);
    float GhostStepBlend = 0.28f;
    FVector PredictedStep_L = LeftAnkleWS  + AvgStrideVelL * GhostStepLookaheadTimeL;
    FVector PredictedStep_R = RightAnkleWS + AvgStrideVelR * GhostStepLookaheadTimeR;
    if (LeftPhase == EFootPhase::Swing || LeftPhase == EFootPhase::Lift)
        NewLockedLEffector = FMath::Lerp(NewLockedLEffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_L), GhostStepBlend);
    if (RightPhase == EFootPhase::Swing || RightPhase == EFootPhase::Lift)
        NewLockedREffector = FMath::Lerp(NewLockedREffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_R), GhostStepBlend);

    // --- 20. Biomechanical pivot/side-step logic (context from PelvisWorld and MoveDir) ---
    static FVector LastPelvisWorld = FVector::ZeroVector;
    float SideStepAngleThreshold = 35.f;
    FVector PelvisMoveDir = (PelvisWorld - LastPelvisWorld).GetSafeNormal();
    float PelvisSpeed = (PelvisWorld - LastPelvisWorld).Size() / FMath::Max(DeltaSeconds, 0.0001f);
    if (PelvisSpeed > 1.f)
    {
        float SideAngle = FMath::RadiansToDegrees(acosf(FVector::DotProduct(MoveDir, PelvisMoveDir)));
        if (SideAngle > SideStepAngleThreshold)
        {
            float SideStepBlend = 0.34f;
            FVector OutDir = FVector::CrossProduct(PelvisMoveDir, FVector::UpVector).GetSafeNormal();
            NewLockedLEffector  += OutDir * FMath::Sign(FVector::DotProduct(AvgStrideVelL, OutDir)) * SideStepBlend * AutoFootLength;
            NewLockedREffector += OutDir * FMath::Sign(FVector::DotProduct(AvgStrideVelR, OutDir)) * SideStepBlend * AutoFootLength;
        }
    }
    LastPelvisWorld = PelvisWorld;

    // --- 21. CLAMP EFFECTOR RANGE TO PREVENT ROOT/PELVIS TILT (DROP-IN SAFE) ---
    {
        // Get pelvis in component space
        FVector PelvisCS = Mesh->GetComponentTransform().InverseTransformPosition(PelvisWorld);

        float MaxForwardReach = AutoFootLength * 1.2f;
        float MaxSideReach    = AutoFootLength * 0.8f;
        float MinBackReach    = -AutoFootLength * 0.4f;

        FVector RelLeft = NewLockedLEffector - PelvisCS;
        RelLeft.X = FMath::Clamp(RelLeft.X, MinBackReach, MaxForwardReach);
        RelLeft.Y = FMath::Clamp(RelLeft.Y, -MaxSideReach, MaxSideReach);
        NewLockedLEffector = PelvisCS + RelLeft;

        FVector RelRight = NewLockedREffector - PelvisCS;
        RelRight.X = FMath::Clamp(RelRight.X, MinBackReach, MaxForwardReach);
        RelRight.Y = FMath::Clamp(RelRight.Y, -MaxSideReach, MaxSideReach);
        NewLockedREffector = PelvisCS + RelRight;
    }

    // --- 22. Environment awareness: highest floor probe ---
    float LHFloorZ = FindHighestNearbyFloorZ(LSplineEffector, World, FloorPlaneRadius, FloorProbeDepth);
    float RHFloorZ = FindHighestNearbyFloorZ(RSplineEffector, World, FloorPlaneRadius, FloorProbeDepth);
    if (FMath::Abs(LSplineEffector.Z - LHFloorZ) > MaxStairHeight)
        LSplineEffector.Z = FMath::FInterpTo(LSplineEffector.Z, LHFloorZ, DeltaSeconds, 6.f);
    if (FMath::Abs(RSplineEffector.Z - RHFloorZ) > MaxStairHeight)
        RSplineEffector.Z = FMath::FInterpTo(RSplineEffector.Z, RHFloorZ, DeltaSeconds, 6.f);

    // --- 23. Physically-inspired critically-damped smoothing ---
    DampedLeftEffector  = CriticallyDampedSmoothing(DampedLeftEffector, NewLockedLEffector, DeltaSeconds, 80.f, AdaptiveInterpSpeedL);
    DampedRightEffector = CriticallyDampedSmoothing(DampedRightEffector, NewLockedREffector, DeltaSeconds, 80.f, AdaptiveInterpSpeedR);

    // --- 24. Effector output for FABRIK/TwoBoneIK nodes ---
    LeftFootIKEffector  = DampedLeftEffector;
    RightFootIKEffector = DampedRightEffector;

    // --- 25. Ankle joint limits ---
    FRotator AnklePitchLimits = FRotator(45.f, 0.f, 0.f);
    FRotator AnkleYawLimits   = FRotator(0.f, 25.f, 0.f);
    FRotator AnkleRollLimits  = FRotator(0.f, 0.f, 25.f);

    NewLockedLRot.Pitch  = FMath::Clamp(NewLockedLRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    NewLockedLRot.Yaw    = FMath::Clamp(NewLockedLRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    NewLockedLRot.Roll   = FMath::Clamp(NewLockedLRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    NewLockedRRot.Pitch  = FMath::Clamp(NewLockedRRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    NewLockedRRot.Yaw    = FMath::Clamp(NewLockedRRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    NewLockedRRot.Roll   = FMath::Clamp(NewLockedRRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    LeftFootIKRotation  = FMath::RInterpTo(LeftFootIKRotation,  NewLockedLRot, DeltaSeconds, AdaptiveInterpSpeedL * 1.12f);
    RightFootIKRotation = FMath::RInterpTo(RightFootIKRotation, NewLockedRRot, DeltaSeconds, AdaptiveInterpSpeedR * 1.12f);

    // --- 26. Z offsets for pose modify bone (component space) ---
    LeftFootIKOffset  = DampedLeftEffector.Z  - LeftAnkleWS.Z;
    RightFootIKOffset = DampedRightEffector.Z - RightAnkleWS.Z;

    // --- 27. Pelvis offset (ground both feet) ---
    float TargetPelvis = FMath::Min(LeftFootIKOffset, RightFootIKOffset);
    PelvisIKOffset = FMath::FInterpTo(PelvisIKOffset, TargetPelvis, DeltaSeconds, PelvisInterpSpeed);

    // --- 28. Debug: foot delta, slope, stride info ---
    FootDeltaZ = FMath::Abs(LeftFootIKOffset - RightFootIKOffset);

    // Optionally expose stride phase, confidence, ghost lookahead time for debug:
    LeftStridePhase = (int32)LeftPhase;
    RightStridePhase = (int32)RightPhase;
    LeftFootStrideConfidence = PhaseConfidenceL;
    LeftFootStrideConfidence = PhaseConfidenceR;
    LeftGhostLookaheadTime = GhostStepLookaheadTimeL;
    RightGhostLookaheadTime = GhostStepLookaheadTimeR;

    #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
if (GEngine && Mesh)
{
    const FTransform& MeshCompXform = Mesh->GetComponentTransform();

    // --- Draw effector output (world space) ---
    FVector LeftEffectorWS  = MeshCompXform.TransformPosition(DampedLeftEffector);
    FVector RightEffectorWS = MeshCompXform.TransformPosition(DampedRightEffector);

    DrawDebugSphere(Mesh->GetWorld(), LeftEffectorWS, 3.f, 12, FColor::Green, false, 0.12f, 0, 1.5f);
    DrawDebugSphere(Mesh->GetWorld(), RightEffectorWS, 3.f, 12, FColor::Blue, false, 0.12f, 0, 1.5f);

    // --- Draw component origin/root for reference ---
    DrawDebugSphere(Mesh->GetWorld(), MeshCompXform.GetLocation(), 4.f, 12, FColor::White, false, 0.12f, 0, 2.f);

    // --- Draw ground contact points for each foot (traces) ---
    DrawDebugSphere(Mesh->GetWorld(), LAnkleHit.ImpactPoint, 2.f, 8, FColor::Cyan, false, 0.12f, 0, 0.8f);
    DrawDebugSphere(Mesh->GetWorld(), LToeHit.ImpactPoint, 2.f, 8, FColor::Magenta, false, 0.12f, 0, 0.8f);
    DrawDebugSphere(Mesh->GetWorld(), RAnkleHit.ImpactPoint, 2.f, 8, FColor::Cyan, false, 0.12f, 0, 0.8f);
    DrawDebugSphere(Mesh->GetWorld(), RToeHit.ImpactPoint, 2.f, 8, FColor::Magenta, false, 0.12f, 0, 0.8f);

    // --- Draw ghost (predicted) step location ---
    DrawDebugSphere(Mesh->GetWorld(), PredictedStep_L, 3.f, 8, FColor::Yellow, false, 0.12f, 0, 1.5f);
    DrawDebugSphere(Mesh->GetWorld(), PredictedStep_R, 3.f, 8, FColor::Orange, false, 0.12f, 0, 1.5f);

    // --- Draw Spline Effector Positions ---
    DrawDebugSphere(Mesh->GetWorld(), MeshCompXform.TransformPosition(LSplineEffector), 2.f, 8, FColor::Emerald, false, 0.12f, 0, 1.0f);
    DrawDebugSphere(Mesh->GetWorld(), MeshCompXform.TransformPosition(RSplineEffector), 2.f, 8, FColor::Turquoise, false, 0.12f, 0, 1.0f);

    // --- Draw Locked Effector Positions ---
    DrawDebugSphere(Mesh->GetWorld(), MeshCompXform.TransformPosition(NewLockedLEffector), 2.2f, 10, FColor::Purple, false, 0.12f, 0, 1.2f);
    DrawDebugSphere(Mesh->GetWorld(), MeshCompXform.TransformPosition(NewLockedREffector), 2.2f, 10, FColor::Silver, false, 0.12f, 0, 1.2f);

    // --- Draw Highest Floor Probes ---
    FVector LHProbe = LSplineEffector; LHProbe.Z = LHFloorZ;
    FVector RHProbe = RSplineEffector; RHProbe.Z = RHFloorZ;
    DrawDebugBox(Mesh->GetWorld(), MeshCompXform.TransformPosition(LHProbe), FVector(2,2,2), FColor::Cyan, false, 0.12f, 0, 0.8f);
    DrawDebugBox(Mesh->GetWorld(), MeshCompXform.TransformPosition(RHProbe), FVector(2,2,2), FColor::Cyan, false, 0.12f, 0, 0.8f);

    // --- Draw Pelvis world ---
    DrawDebugSphere(Mesh->GetWorld(), PelvisWorld, 2.f, 12, FColor::Red, false, 0.12f, 0, 0.9f);

    // --- Annotate all spheres for quick ID ---
    DrawDebugString(Mesh->GetWorld(), LeftEffectorWS + FVector(0,0,8), TEXT("LeftEffector"), nullptr, FColor::Green, 0.12f, false);
    DrawDebugString(Mesh->GetWorld(), RightEffectorWS + FVector(0,0,8), TEXT("RightEffector"), nullptr, FColor::Blue, 0.12f, false);
    DrawDebugString(Mesh->GetWorld(), MeshCompXform.GetLocation() + FVector(0,0,10), TEXT("Root/Component"), nullptr, FColor::White, 0.12f, false);
    DrawDebugString(Mesh->GetWorld(), PredictedStep_L + FVector(0,0,10), TEXT("Ghost L"), nullptr, FColor::Yellow, 0.12f, false);
    DrawDebugString(Mesh->GetWorld(), PredictedStep_R + FVector(0,0,10), TEXT("Ghost R"), nullptr, FColor::Orange, 0.12f, false);

    // --- Display stride debug info as text in the world ---
    FString DebugInfoL = FString::Printf(TEXT("StridePhase:%d\nConf:%.2f\nGhost:%.2f"),
        LeftStridePhase, LeftFootStrideConfidence, LeftGhostLookaheadTime);
    FString DebugInfoR = FString::Printf(TEXT("StridePhase:%d\nConf:%.2f\nGhost:%.2f"),
        RightStridePhase, RightFootStrideConfidence, RightGhostLookaheadTime);

    DrawDebugString(Mesh->GetWorld(), LeftEffectorWS + FVector(0,10,20), DebugInfoL, nullptr, FColor::Green, 0.12f, false);
    DrawDebugString(Mesh->GetWorld(), RightEffectorWS + FVector(0,-10,20), DebugInfoR, nullptr, FColor::Blue, 0.12f, false);

    // --- Log values for additional tuning (optional) ---
    UE_LOG(LogTemp, Log, TEXT("[StrideIK] LEffComp:%s  REffComp:%s  LEffWorld:%s  REffWorld:%s"), 
        *DampedLeftEffector.ToString(), *DampedRightEffector.ToString(),
        *LeftEffectorWS.ToString(), *RightEffectorWS.ToString());
    UE_LOG(LogTemp, Log, TEXT("[StrideIK] LStridePhase:%d  RStridePhase:%d  LConf:%.2f  RConf:%.2f  LGhost:%.2f  RGhost:%.2f"),
        LeftStridePhase, RightStridePhase, LeftFootStrideConfidence, RightFootStrideConfidence, LeftGhostLookaheadTime, RightGhostLookaheadTime);
}
#endif
}
#if 0
void UOHAnimInstance_Base::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    // --- 0. Early outs & pointer guards ---
    APawn* OwnerPawn = TryGetPawnOwner();
    if (!OwnerPawn) return;
    USkeletalMeshComponent* Mesh = GetSkelMeshComponent();
    if (!Mesh) return;
    UWorld* World = Mesh->GetWorld();
    if (!World) return;

    // --- 1. Auto foot size detection (first tick only) ---
    if (!bMeasuredFootLength)
    {
        FVector Ankle = Mesh->GetBoneLocation(FName("foot_l"));
        FVector Toe   = Mesh->GetBoneLocation(FName("ball_l"));
        AutoFootLength = (Toe - Ankle).Size2D();
        if (IKFootSphereRadius <= 0.f) IKFootSphereRadius = FMath::Clamp(AutoFootLength * 0.28f, 4.5f, 10.f);
        if (IKFootCapsuleHalfHeight <= 0.f) IKFootCapsuleHalfHeight = FMath::Clamp(AutoFootLength * 0.48f, 7.0f, 16.f);
        bMeasuredFootLength = true;
    }

    // --- 2. Gather bone world positions ---
    FVector LeftAnkleWS  = Mesh->GetBoneLocation(FName("foot_l"));
    FVector LeftToeWS    = Mesh->GetBoneLocation(FName("ball_l"));
    FVector RightAnkleWS = Mesh->GetBoneLocation(FName("foot_r"));
    FVector RightToeWS   = Mesh->GetBoneLocation(FName("ball_r"));
    FVector PelvisWorld  = Mesh->GetBoneLocation(FName("pelvis"));
    float TimeNow = World->TimeSeconds;

    // --- 3. Update foot trace buffers ---
    LeftFootBuffer.AddSample(LeftAnkleWS, TimeNow);
    RightFootBuffer.AddSample(RightAnkleWS, TimeNow);

    // --- 4. Velocity and stride direction ---
    FVector CharVel = OwnerPawn->GetVelocity();
    float Speed = CharVel.Size();
    FVector MoveDir = CharVel.IsNearlyZero() ? OwnerPawn->GetActorForwardVector() : CharVel.GetSafeNormal();

    // --- 5. Detect stride phase & intent (per foot) ---
    EFootPhase LeftPhase  = DetectFootPhase(LeftFootBuffer);
    EFootPhase RightPhase = DetectFootPhase(RightFootBuffer);
    float LeftSpeed  = LeftFootBuffer.GetVelocity().Size();
    float RightSpeed = RightFootBuffer.GetVelocity().Size();

    // --- 6. Update phase buffers ---
    LeftPhaseBuffer.Add(LeftPhase, TimeNow, LeftAnkleWS, LeftSpeed);
    RightPhaseBuffer.Add(RightPhase, TimeNow, RightAnkleWS, RightSpeed);

    // --- 7. Predict stride phase/timing ---
    EFootPhase PredictedLeftPhase  = LeftPhaseBuffer.PredictNextPhase();
    EFootPhase PredictedRightPhase = RightPhaseBuffer.PredictNextPhase();
    float PredictPhaseTimeL = LeftPhaseBuffer.PredictPhaseChangeTime(TimeNow);
    float PredictPhaseTimeR = RightPhaseBuffer.PredictPhaseChangeTime(TimeNow);
    FVector AvgStrideVelL = LeftPhaseBuffer.GetAvgStrideVelocity();
    FVector AvgStrideVelR = RightPhaseBuffer.GetAvgStrideVelocity();

    // --- 8. Confidence (stride stability), for adaptive smoothing ---
    float PhaseConfidenceL = LeftPhaseBuffer.GetPhaseConfidence();
    float PhaseConfidenceR = RightPhaseBuffer.GetPhaseConfidence();
    LeftFootStrideConfidence = PhaseConfidenceL;
    RightFootStrideConfidence = PhaseConfidenceR;

    // EXPOSE for blueprints or debug
    LeftStridePhase = static_cast<int32>(LeftPhase);
    RightStridePhase = static_cast<int32>(RightPhase);
    LeftFootStrideConfidence = PhaseConfidenceL;
    RightFootStrideConfidence = PhaseConfidenceR;
    LeftGhostLookaheadTime = PredictPhaseTimeL;
    RightGhostLookaheadTime = PredictPhaseTimeR;

    // --- 9. Heel-toe blend for realistic roll ---
    float HeelToeAlphaLeft  = GetHeelToeBlendAlpha(LeftPhase, LeftFootBuffer);
    float HeelToeAlphaRight = GetHeelToeBlendAlpha(RightPhase, RightFootBuffer);

    // --- 10. Trace bias for predictive steps ---
    float BiasLeft  = (LeftPhase  == EFootPhase::Plant) ? IKFootForwardBias : IKFootForwardBias * 2.f;
    float BiasRight = (RightPhase == EFootPhase::Plant) ? IKFootForwardBias : IKFootForwardBias * 2.f;
    if (Speed < IdleFreezeSpeed) BiasLeft = BiasRight = FMath::Min(IKFootForwardBias, 12.f);

    // --- 11. Dual ankle/toe trace (safe, backup) ---
    FHitResult LAnkleHit, LAnkleBackup, LToeHit, LToeBackup;
    FHitResult RAnkleHit, RAnkleBackup, RToeHit, RToeBackup;
    bool bLAnkleHit = SafeDoubleTrace(Mesh, FName("foot_l"), IKFootProbeDepth, BiasLeft, IKFootSphereRadius, CharVel, LAnkleHit, LAnkleBackup, FloorProbeDepth);
    bool bLToeHit   = SafeDoubleTrace(Mesh, FName("ball_l"), IKFootProbeDepth, BiasLeft, IKFootSphereRadius, CharVel, LToeHit, LToeBackup, FloorProbeDepth);
    bool bRAnkleHit = SafeDoubleTrace(Mesh, FName("foot_r"), IKFootProbeDepth, BiasRight, IKFootSphereRadius, CharVel, RAnkleHit, RAnkleBackup, FloorProbeDepth);
    bool bRToeHit   = SafeDoubleTrace(Mesh, FName("ball_r"), IKFootProbeDepth, BiasRight, IKFootSphereRadius, CharVel, RToeHit, RToeBackup, FloorProbeDepth);

    // --- 12. Robust floor Z blend/median for anti-pop & anti-jitter ---
    float LAnkleZ = bLAnkleHit ? LAnkleHit.ImpactPoint.Z : LeftAnkleWS.Z;
    float LToeZ   = bLToeHit   ? LToeHit.ImpactPoint.Z   : LeftToeWS.Z;
    float RAnkleZ = bRAnkleHit ? RAnkleHit.ImpactPoint.Z : RightAnkleWS.Z;
    float RToeZ   = bRToeHit   ? RToeHit.ImpactPoint.Z   : RightToeWS.Z;

    float BlendL = FMath::Lerp(LAnkleZ, LToeZ, HeelToeAlphaLeft);
    float MaxL   = FMath::Max(LAnkleZ, LToeZ);
    float LFootGroundZ = (FMath::Abs(LAnkleZ - LToeZ) > MaxStairHeight)
        ? FMath::Min(LAnkleZ, LToeZ)
        : FMath::Lerp(BlendL, MaxL, 0.6f);

    float BlendR = FMath::Lerp(RAnkleZ, RToeZ, HeelToeAlphaRight);
    float MaxR   = FMath::Max(RAnkleZ, RToeZ);
    float RFootGroundZ = (FMath::Abs(RAnkleZ - RToeZ) > MaxStairHeight)
        ? FMath::Min(RAnkleZ, RToeZ)
        : FMath::Lerp(BlendR, MaxR, 0.6f);

    // --- 13. Temporal median smoothing (outlier protection) ---
    LFootZBuffer[LFootZBufIndex] = LFootGroundZ;
    if (LFootZBufValidSamples < FootZBufferSize) LFootZBufValidSamples++;
    LFootZBufIndex = (LFootZBufIndex + 1) % FootZBufferSize;
    float MedianLFootZ = Median(LFootZBuffer, LFootZBufValidSamples);

    RFootZBuffer[RFootZBufIndex] = RFootGroundZ;
    if (RFootZBufValidSamples < FootZBufferSize) RFootZBufValidSamples++;
    RFootZBufIndex = (RFootZBufIndex + 1) % FootZBufferSize;
    float MedianRFootZ = Median(RFootZBuffer, RFootZBufValidSamples);

    if (bLFootFirstUpdate || LFootZBufValidSamples < 2) { SmoothedLFootGroundZ = MedianLFootZ; bLFootFirstUpdate = false; }
    else { SmoothedLFootGroundZ = FMath::FInterpTo(SmoothedLFootGroundZ, MedianLFootZ, DeltaSeconds, OffsetInterpSpeed); }
    if (bRFootFirstUpdate || RFootZBufValidSamples < 2) { SmoothedRFootGroundZ = MedianRFootZ; bRFootFirstUpdate = false; }
    else { SmoothedRFootGroundZ = FMath::FInterpTo(SmoothedRFootGroundZ, MedianRFootZ, DeltaSeconds, OffsetInterpSpeed); }

    // --- 14. Heel-toe blend world targets (placement) ---
    FVector LTargetWS = FMath::Lerp(LeftAnkleWS, LeftToeWS, HeelToeAlphaLeft);
    FVector RTargetWS = FMath::Lerp(RightAnkleWS, RightToeWS, HeelToeAlphaRight);
    LTargetWS.Z = SmoothedLFootGroundZ + LeftFootGroundOffset;
    RTargetWS.Z = SmoothedRFootGroundZ + RightFootGroundOffset;

    // --- 15. Trajectory spline smoothing (Catmull-Rom) ---
    if (LeftEffectorSpline.Num() < SplineMax)  { for(int i=LeftEffectorSpline.Num();i<SplineMax;++i)  LeftEffectorSpline.Add(LTargetWS); }
    if (RightEffectorSpline.Num() < SplineMax) { for(int i=RightEffectorSpline.Num();i<SplineMax;++i) RightEffectorSpline.Add(RTargetWS); }
    LeftEffectorSpline.RemoveAt(0);  LeftEffectorSpline.Add(LTargetWS);
    RightEffectorSpline.RemoveAt(0); RightEffectorSpline.Add(RTargetWS);

    float SplineT = 0.7f * GlobalIKSmoothing + 0.25f;
    FVector LSplineEffector = CatmullRomInterp(LeftEffectorSpline, SplineT);
    FVector RSplineEffector = CatmullRomInterp(RightEffectorSpline, SplineT);

    // --- 16. Adaptive stride smoothing (stride confidence based, **APPLIED**) ---
    float AdaptiveSmoothL = FMath::Lerp(AdaptiveSmoothingMin, AdaptiveSmoothingMax, 1.f - PhaseConfidenceL);
    float AdaptiveSmoothR = FMath::Lerp(AdaptiveSmoothingMin, AdaptiveSmoothingMax, 1.f - PhaseConfidenceR);
    float AdaptiveInterpSpeedL = FMath::Lerp(AdaptiveInterpSpeedMin, AdaptiveInterpSpeedMax, 1.f - PhaseConfidenceL);
    float AdaptiveInterpSpeedR = FMath::Lerp(AdaptiveInterpSpeedMin, AdaptiveInterpSpeedMax, 1.f - PhaseConfidenceR);

    // Adaptive anti-jitter: Blend to locked/target using AdaptiveSmooth
    LSplineEffector = FMath::Lerp(LSplineEffector, NewLockedLEffector, AdaptiveSmoothL);
    RSplineEffector = FMath::Lerp(RSplineEffector, NewLockedREffector, AdaptiveSmoothR);

    // --- 17. Sticky plant blend/fade (supports plant fade timing) ---
    if (PredictedLeftPhase == EFootPhase::Plant)  { LeftPlantAlpha = 1.f; LastLeftPlantTime = TimeNow; }
    else LeftPlantAlpha  = FMath::FInterpTo(LeftPlantAlpha,  0.f, DeltaSeconds, PlantFadeSpeed);
    if (PredictedRightPhase == EFootPhase::Plant) { RightPlantAlpha = 1.f; LastRightPlantTime = TimeNow; }
    else RightPlantAlpha = FMath::FInterpTo(RightPlantAlpha, 0.f, DeltaSeconds, PlantFadeSpeed);

    // --- 18. Plant lock/hysteresis ---
    if (LeftPhase == EFootPhase::Plant && bLAnkleHit)
    {
        LastValidLeftImpact = LAnkleHit.ImpactPoint;
        LastValidLeftNormal = bLToeHit
            ? ((LAnkleHit.ImpactNormal + LToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : LAnkleHit.ImpactNormal;
        LeftFootLockFrames = LockHysteresisFrames;
    }
    else if (LeftFootLockFrames > 0) { LeftFootLockFrames--; }
    else { LastValidLeftImpact = LTargetWS; LastValidLeftNormal = FVector::UpVector; }

    if (RightPhase == EFootPhase::Plant && bRAnkleHit)
    {
        LastValidRightImpact = RAnkleHit.ImpactPoint;
        LastValidRightNormal = bRToeHit
            ? ((RAnkleHit.ImpactNormal + RToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : RAnkleHit.ImpactNormal;
        RightFootLockFrames = LockHysteresisFrames;
    }
    else if (RightFootLockFrames > 0) { RightFootLockFrames--; }
    else { LastValidRightImpact = RTargetWS; LastValidRightNormal = FVector::UpVector; }

    NewLockedLEffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidLeftImpact);
    NewLockedREffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidRightImpact);
    FRotator NewLockedLRot = LastValidLeftNormal.Rotation();
    FRotator NewLockedRRot = LastValidRightNormal.Rotation();

    // --- 19. Ghost step/stride anticipation (lookahead for both feet, using respective PredictPhaseTime) ---
    float GhostStepLookaheadTimeL = FMath::Clamp(PredictPhaseTimeL * 0.4f, 0.06f, 0.14f);
    float GhostStepLookaheadTimeR = FMath::Clamp(PredictPhaseTimeR * 0.4f, 0.06f, 0.14f);
    float GhostStepBlend = 0.28f;
    FVector PredictedStep_L = LeftAnkleWS  + AvgStrideVelL * GhostStepLookaheadTimeL;
    FVector PredictedStep_R = RightAnkleWS + AvgStrideVelR * GhostStepLookaheadTimeR;
    if (LeftPhase == EFootPhase::Swing || LeftPhase == EFootPhase::Lift)
        NewLockedLEffector = FMath::Lerp(NewLockedLEffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_L), GhostStepBlend);
    if (RightPhase == EFootPhase::Swing || RightPhase == EFootPhase::Lift)
        NewLockedREffector = FMath::Lerp(NewLockedREffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_R), GhostStepBlend);

    // --- 20. Biomechanical pivot/side-step logic (context from PelvisWorld and MoveDir) ---
    static FVector LastPelvisWorld = FVector::ZeroVector;
    float SideStepAngleThreshold = 35.f;
    FVector PelvisMoveDir = (PelvisWorld - LastPelvisWorld).GetSafeNormal();
    float PelvisSpeed = (PelvisWorld - LastPelvisWorld).Size() / FMath::Max(DeltaSeconds, 0.0001f);
    if (PelvisSpeed > 1.f)
    {
        float SideAngle = FMath::RadiansToDegrees(acosf(FVector::DotProduct(MoveDir, PelvisMoveDir)));
        if (SideAngle > SideStepAngleThreshold)
        {
            float SideStepBlend = 0.34f;
            FVector OutDir = FVector::CrossProduct(PelvisMoveDir, FVector::UpVector).GetSafeNormal();
            NewLockedLEffector  += OutDir * FMath::Sign(FVector::DotProduct(AvgStrideVelL, OutDir)) * SideStepBlend * AutoFootLength;
            NewLockedREffector += OutDir * FMath::Sign(FVector::DotProduct(AvgStrideVelR, OutDir)) * SideStepBlend * AutoFootLength;
        }
    }
    LastPelvisWorld = PelvisWorld;

    // --- FINAL ASSIGNMENT to UPROPERTY for Blueprint access & debug ---
    this->NewLockedLEffector = NewLockedLEffector;
    this->NewLockedREffector = NewLockedREffector;
    
    // --- 21. Idle dampening (sticky feet logic) ---
    float IdleDampMult = 1.0f;
    if (TimeNow - LastLeftPlantTime > 0.7f && TimeNow - LastRightPlantTime < 0.7f)
        IdleDampMult = 0.14f;
    if (TimeNow - LastRightPlantTime > 0.7f && TimeNow - LastLeftPlantTime < 0.7f)
        IdleDampMult = 0.14f;

    // --- 22. MaxEffectorWorldJump (guard band anti-pop) ---
    static FVector LastLeftEffectorWS = LSplineEffector, LastRightEffectorWS = RSplineEffector;
    float LDelta = (LSplineEffector - LastLeftEffectorWS).Size();
    float RDelta = (RSplineEffector - LastRightEffectorWS).Size();
    if (LDelta > MaxEffectorWorldJump)
        LSplineEffector = LastLeftEffectorWS + (LSplineEffector - LastLeftEffectorWS).GetClampedToMaxSize(MaxEffectorWorldJump);
    if (RDelta > MaxEffectorWorldJump)
        RSplineEffector = LastRightEffectorWS + (RSplineEffector - LastRightEffectorWS).GetClampedToMaxSize(MaxEffectorWorldJump);
    LastLeftEffectorWS = LSplineEffector;
    LastRightEffectorWS = RSplineEffector;

    // --- 23. Environment awareness: highest floor probe ---
    float LHFloorZ = FindHighestNearbyFloorZ(LSplineEffector, World, FloorPlaneRadius, FloorProbeDepth);
    float RHFloorZ = FindHighestNearbyFloorZ(RSplineEffector, World, FloorPlaneRadius, FloorProbeDepth);
    if (FMath::Abs(LSplineEffector.Z - LHFloorZ) > MaxStairHeight)
        LSplineEffector.Z = FMath::FInterpTo(LSplineEffector.Z, LHFloorZ, DeltaSeconds, 6.f);
    if (FMath::Abs(RSplineEffector.Z - RHFloorZ) > MaxStairHeight)
        RSplineEffector.Z = FMath::FInterpTo(RSplineEffector.Z, RHFloorZ, DeltaSeconds, 6.f);

    // --- 24. Physically-inspired critically-damped smoothing ---
    DampedLeftEffector  = CriticallyDampedSmoothing(DampedLeftEffector, LSplineEffector, DeltaSeconds, 80.f, AdaptiveInterpSpeedL * IdleDampMult);
    DampedRightEffector = CriticallyDampedSmoothing(DampedRightEffector, RSplineEffector, DeltaSeconds, 80.f, AdaptiveInterpSpeedR * IdleDampMult);

    // --- 25. Effector output for FABRIK/TwoBoneIK nodes ---
    LeftFootIKEffector  = DampedLeftEffector;
    RightFootIKEffector = DampedRightEffector;

    // --- 26. Ankle joint limits ---
    FRotator AnklePitchLimits = FRotator(45.f, 0.f, 0.f);
    FRotator AnkleYawLimits   = FRotator(0.f, 25.f, 0.f);
    FRotator AnkleRollLimits  = FRotator(0.f, 0.f, 25.f);

    NewLockedLRot.Pitch  = FMath::Clamp(NewLockedLRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    NewLockedLRot.Yaw    = FMath::Clamp(NewLockedLRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    NewLockedLRot.Roll   = FMath::Clamp(NewLockedLRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    NewLockedRRot.Pitch  = FMath::Clamp(NewLockedRRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    NewLockedRRot.Yaw    = FMath::Clamp(NewLockedRRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    NewLockedRRot.Roll   = FMath::Clamp(NewLockedRRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    LeftFootIKRotation  = FMath::RInterpTo(LeftFootIKRotation,  NewLockedLRot, DeltaSeconds, AdaptiveInterpSpeedL * 1.12f);
    RightFootIKRotation = FMath::RInterpTo(RightFootIKRotation, NewLockedRRot, DeltaSeconds, AdaptiveInterpSpeedR * 1.12f);

    // --- 27. Z offsets for pose modify bone (component space) ---
    LeftFootIKOffset  = DampedLeftEffector.Z  - LeftAnkleWS.Z;
    RightFootIKOffset = DampedRightEffector.Z - RightAnkleWS.Z;

    // --- 28. Pelvis offset (ground both feet) ---
   // float TargetPelvis = FMath::Min(LeftFootIKOffset, RightFootIKOffset);
   // PelvisIKOffset = FMath::FInterpTo(PelvisIKOffset, TargetPelvis, DeltaSeconds, PelvisInterpSpeed);

    
    // --- 29. Debug/visualization output ---
    FootDeltaZ = FMath::Abs(LeftFootIKOffset - RightFootIKOffset);
    // 👇 Expose for Blueprint, live debug, or logs:
    // LeftStridePhase, RightStridePhase, LeftStridePhaseConfidence, RightStridePhaseConfidence, LeftGhostLookaheadTime, RightGhostLookaheadTime
}
#endif
#if 0
void UOHAnimInstance_Base::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    APawn* OwnerPawn = TryGetPawnOwner();
    if (!OwnerPawn) return;
    USkeletalMeshComponent* Mesh = GetSkelMeshComponent();
    if (!Mesh) return;

    // --- Foot length (auto-measure once) ---
    if (!bMeasuredFootLength)
    {
        FVector Ankle = Mesh->GetBoneLocation(FName("foot_l"));
        FVector Toe = Mesh->GetBoneLocation(FName("ball_l"));
        AutoFootLength = (Toe - Ankle).Size2D();
        bMeasuredFootLength = true;
    }

    // --- Bone positions (world) ---
    FVector LeftAnkleWS  = Mesh->GetBoneLocation(FName("foot_l"));
    FVector LeftToeWS    = Mesh->GetBoneLocation(FName("ball_l"));
    FVector RightAnkleWS = Mesh->GetBoneLocation(FName("foot_r"));
    FVector RightToeWS   = Mesh->GetBoneLocation(FName("ball_r"));
    FVector PelvisWorld  = Mesh->GetBoneLocation(FName("pelvis"));

    float TimeNow = GetWorld()->TimeSeconds;

    // --- Foot buffer update (for stride/ghost/pivot) ---
    LeftFootBuffer.AddSample(LeftAnkleWS, TimeNow);
    RightFootBuffer.AddSample(RightAnkleWS, TimeNow);

    // --- Movement / stride ---
    FVector CharVel = OwnerPawn->GetVelocity();
    float Speed = CharVel.Size();
    FVector MoveDir = CharVel.IsNearlyZero() ? OwnerPawn->GetActorForwardVector() : CharVel.GetSafeNormal();

    // --- Phase detection ---
    EFootPhase LeftPhase = DetectFootPhase(LeftFootBuffer);
    EFootPhase RightPhase = DetectFootPhase(RightFootBuffer);

    float HeelToeAlphaLeft = GetHeelToeBlendAlpha(LeftPhase, LeftFootBuffer);
    float HeelToeAlphaRight = GetHeelToeBlendAlpha(RightPhase, RightFootBuffer);

    // --- Trace bias ---
    float BiasLeft  = (LeftPhase == EFootPhase::Plant)  ? 18.f : 36.f;
    float BiasRight = (RightPhase == EFootPhase::Plant) ? 18.f : 36.f;
    if (Speed < IdleFreezeSpeed) { BiasLeft = BiasRight = 12.f; }

    // --- Dual traces (ankle/toe) ---
    FHitResult LAnkleHit, LToeHit, RAnkleHit, RToeHit;
    bool bLAnkleHit = SphereGroundTrace(Mesh, FName("foot_l"), 60.f, BiasLeft, 5.f, CharVel, LAnkleHit, false);
    bool bLToeHit   = SphereGroundTrace(Mesh, FName("ball_l"), 60.f, BiasLeft, 5.f, CharVel, LToeHit, false);
    bool bRAnkleHit = SphereGroundTrace(Mesh, FName("foot_r"), 60.f, BiasRight, 5.f, CharVel, RAnkleHit, false);
    bool bRToeHit   = SphereGroundTrace(Mesh, FName("ball_r"), 60.f, BiasRight, 5.f, CharVel, RToeHit, false);

    // --- Jitter-proof Z: blend+max+clamp+median ---
    float LAnkleZ = bLAnkleHit ? LAnkleHit.ImpactPoint.Z : LeftAnkleWS.Z;
    float LToeZ   = bLToeHit   ? LToeHit.ImpactPoint.Z   : LeftToeWS.Z;
    float RAnkleZ = bRAnkleHit ? RAnkleHit.ImpactPoint.Z : RightAnkleWS.Z;
    float RToeZ   = bRToeHit   ? RToeHit.ImpactPoint.Z   : RightToeWS.Z;

    float BlendL = FMath::Lerp(LAnkleZ, LToeZ, HeelToeAlphaLeft);
    float MaxL   = FMath::Max(LAnkleZ, LToeZ);
    float LFootGroundZ = (FMath::Abs(LAnkleZ - LToeZ) > MaxStairHeight)
        ? FMath::Min(LAnkleZ, LToeZ)
        : FMath::Lerp(BlendL, MaxL, 0.6f);

    float BlendR = FMath::Lerp(RAnkleZ, RToeZ, HeelToeAlphaRight);
    float MaxR   = FMath::Max(RAnkleZ, RToeZ);
    float RFootGroundZ = (FMath::Abs(RAnkleZ - RToeZ) > MaxStairHeight)
        ? FMath::Min(RAnkleZ, RToeZ)
        : FMath::Lerp(BlendR, MaxR, 0.6f);

    // --- Temporal median filtering for Z ---
    LFootZBuffer[LFootZBufIndex] = LFootGroundZ;
    if (LFootZBufValidSamples < FootZBufferSize) LFootZBufValidSamples++;
    LFootZBufIndex = (LFootZBufIndex + 1) % FootZBufferSize;
    float MedianLFootZ = Median(LFootZBuffer, LFootZBufValidSamples);

    RFootZBuffer[RFootZBufIndex] = RFootGroundZ;
    if (RFootZBufValidSamples < FootZBufferSize) RFootZBufValidSamples++;
    RFootZBufIndex = (RFootZBufIndex + 1) % FootZBufferSize;
    float MedianRFootZ = Median(RFootZBuffer, RFootZBufValidSamples);

    // --- Z startup safety ---
    if (bLFootFirstUpdate || LFootZBufValidSamples < 2) { SmoothedLFootGroundZ = MedianLFootZ; bLFootFirstUpdate = false; }
    else { SmoothedLFootGroundZ = FMath::FInterpTo(SmoothedLFootGroundZ, MedianLFootZ, DeltaSeconds, OffsetInterpSpeed); }
    if (bRFootFirstUpdate || RFootZBufValidSamples < 2) { SmoothedRFootGroundZ = MedianRFootZ; bRFootFirstUpdate = false; }
    else { SmoothedRFootGroundZ = FMath::FInterpTo(SmoothedRFootGroundZ, MedianRFootZ, DeltaSeconds, OffsetInterpSpeed); }

    // --- Heel-toe world target ---
    FVector LTargetWS = FMath::Lerp(LeftAnkleWS, LeftToeWS, HeelToeAlphaLeft);
    FVector RTargetWS = FMath::Lerp(RightAnkleWS, RightToeWS, HeelToeAlphaRight);
    LTargetWS.Z = SmoothedLFootGroundZ + LeftFootGroundOffset;
    RTargetWS.Z = SmoothedRFootGroundZ + RightFootGroundOffset;

    // --- Plant lock/hysteresis/sticky foot ---
    bool bLeftPlanted = (LeftPhase == EFootPhase::Plant);
    bool bRightPlanted = (RightPhase == EFootPhase::Plant);

    if (bLeftPlanted && bLAnkleHit)
    {
        LastValidLeftImpact = LAnkleHit.ImpactPoint;
        LastValidLeftNormal = bLToeHit
            ? ((LAnkleHit.ImpactNormal + LToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : LAnkleHit.ImpactNormal;
        LeftFootLockFrames = LockHysteresisFrames;
    }
    else if (LeftFootLockFrames > 0) { LeftFootLockFrames--; }
    else
    {
        LastValidLeftImpact = LTargetWS;
        LastValidLeftNormal = FVector::UpVector;
    }

    if (bRightPlanted && bRAnkleHit)
    {
        LastValidRightImpact = RAnkleHit.ImpactPoint;
        LastValidRightNormal = bRToeHit
            ? ((RAnkleHit.ImpactNormal + RToeHit.ImpactNormal) * 0.5f).GetSafeNormal()
            : RAnkleHit.ImpactNormal;
        RightFootLockFrames = LockHysteresisFrames;
    }
    else if (RightFootLockFrames > 0) { RightFootLockFrames--; }
    else
    {
        LastValidRightImpact = RTargetWS;
        LastValidRightNormal = FVector::UpVector;
    }

    FVector NewLockedLEffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidLeftImpact);
    FVector NewLockedREffector = Mesh->GetComponentTransform().InverseTransformPosition(LastValidRightImpact);
    FRotator NewLockedLRot = LastValidLeftNormal.Rotation();
    FRotator NewLockedRRot = LastValidRightNormal.Rotation();

    // --- Super-sticky idle (for idle/freeze) ---
    if (Speed < IdleFreezeSpeed)
    {
        LockedLeftEffector = FMath::Lerp(LockedLeftEffector, NewLockedLEffector, 1.f - StickyStandingBlend);
        LockedRightEffector = FMath::Lerp(LockedRightEffector, NewLockedREffector, 1.f - StickyStandingBlend);
        LockedLeftRot = FMath::RInterpTo(LockedLeftRot, NewLockedLRot, DeltaSeconds, NormalInterpSpeed * 2.0f);
        LockedRightRot = FMath::RInterpTo(LockedRightRot, NewLockedRRot, DeltaSeconds, NormalInterpSpeed * 2.0f);
    }
    else
    {
        LockedLeftEffector = FMath::VInterpConstantTo(LockedLeftEffector, NewLockedLEffector, DeltaSeconds, MaxFootMoveDelta / FMath::Max(DeltaSeconds, 0.016f));
        LockedRightEffector = FMath::VInterpConstantTo(LockedRightEffector, NewLockedREffector, DeltaSeconds, MaxFootMoveDelta / FMath::Max(DeltaSeconds, 0.016f));
        LockedLeftRot = FMath::RInterpConstantTo(LockedLeftRot, NewLockedLRot, DeltaSeconds, MaxFootRotDelta / FMath::Max(DeltaSeconds, 0.016f));
        LockedRightRot = FMath::RInterpConstantTo(LockedRightRot, NewLockedRRot, DeltaSeconds, MaxFootRotDelta / FMath::Max(DeltaSeconds, 0.016f));
    }

    // --- Dynamic step anticipation ("ghost step") ---
    float GhostStepLookaheadTime = 0.08f;
    float GhostStepBlend = 0.25f;
    FVector LFootVel = LeftFootBuffer.GetVelocity();
    FVector RFootVel = RightFootBuffer.GetVelocity();
    FVector PredictedStep_L = LeftAnkleWS + LFootVel * GhostStepLookaheadTime;
    FVector PredictedStep_R = RightAnkleWS + RFootVel * GhostStepLookaheadTime;
    if (LeftPhase == EFootPhase::Swing || LeftPhase == EFootPhase::Lift)
        LockedLeftEffector = FMath::Lerp(LockedLeftEffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_L), GhostStepBlend);
    if (RightPhase == EFootPhase::Swing || RightPhase == EFootPhase::Lift)
        LockedRightEffector = FMath::Lerp(LockedRightEffector, Mesh->GetComponentTransform().InverseTransformPosition(PredictedStep_R), GhostStepBlend);

    // --- Procedural side-step & pivot ---
    static FVector LastPelvisWorld = FVector::ZeroVector;
    float SideStepBlend = 0.35f;
    float SideStepAngleThreshold = 40.f;
    FVector PelvisMoveDir = (PelvisWorld - LastPelvisWorld).GetSafeNormal();
    float PelvisSpeed = (PelvisWorld - LastPelvisWorld).Size() / FMath::Max(DeltaSeconds, 0.0001f);
    if (PelvisSpeed > 1.f)
    {
        float SideAngle_L = FMath::RadiansToDegrees(acosf(FVector::DotProduct(MoveDir, PelvisMoveDir)));
        if (SideAngle_L > SideStepAngleThreshold)
        {
            FVector OutDir = FVector::CrossProduct(PelvisMoveDir, FVector::UpVector).GetSafeNormal();
            LockedLeftEffector  += OutDir * FMath::Sign(FVector::DotProduct(LFootVel, OutDir)) * SideStepBlend * AutoFootLength;
            LockedRightEffector += OutDir * FMath::Sign(FVector::DotProduct(RFootVel, OutDir)) * SideStepBlend * AutoFootLength;
        }
    }
    LastPelvisWorld = PelvisWorld;

    // --- Ankle biomechanical limits ---
    FRotator AnklePitchLimits = FRotator(45.f, 0.f, 0.f);
    FRotator AnkleYawLimits   = FRotator(0.f, 25.f, 0.f);
    FRotator AnkleRollLimits  = FRotator(0.f, 0.f, 25.f);
    LockedLeftRot.Pitch  = FMath::Clamp(LockedLeftRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    LockedLeftRot.Yaw    = FMath::Clamp(LockedLeftRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    LockedLeftRot.Roll   = FMath::Clamp(LockedLeftRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);
    LockedRightRot.Pitch = FMath::Clamp(LockedRightRot.Pitch, -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    LockedRightRot.Yaw   = FMath::Clamp(LockedRightRot.Yaw,   -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    LockedRightRot.Roll  = FMath::Clamp(LockedRightRot.Roll,  -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    // --- Final output for FABRIC nodes ---
    LeftFootIKEffector = LockedLeftEffector;
    RightFootIKEffector = LockedRightEffector;
    LeftFootIKRotation = LockedLeftRot;
    RightFootIKRotation = LockedRightRot;
    LeftFootIKOffset = LockedLeftEffector.Z;
    RightFootIKOffset = LockedRightEffector.Z;

    // --- Pelvis offset for keeping feet on the ground ---
    float TargetPelvis = FMath::Min(LeftFootIKOffset, RightFootIKOffset);
    PelvisIKOffset = FMath::FInterpTo(PelvisIKOffset, TargetPelvis, DeltaSeconds, PelvisInterpSpeed);

    FootDeltaZ = FMath::Abs(LeftFootIKOffset - RightFootIKOffset);
}
#endif

#if 0
void UOHAnimInstance_Base::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    APawn* OwnerPawn = TryGetPawnOwner();
    if (!OwnerPawn) return;
    USkeletalMeshComponent* Mesh = GetSkelMeshComponent();
    if (!Mesh) return;

    // --- Bone/world positions
    FVector LeftFootWorld = Mesh->GetBoneLocation(FName("foot_l"));
    FVector RightFootWorld = Mesh->GetBoneLocation(FName("foot_r"));
    FVector LeftToeWorld = Mesh->GetBoneLocation(FName("ball_l"));
    FVector RightToeWorld = Mesh->GetBoneLocation(FName("ball_r"));
    FVector PelvisWorld = Mesh->GetBoneLocation(FName("pelvis"));

    // --- Character movement (predict stride, expect planting)
    FVector CharVel = OwnerPawn->GetVelocity();
    FVector MoveDir = CharVel.IsNearlyZero() ? OwnerPawn->GetActorForwardVector() : CharVel.GetSafeNormal();

    // --- Bone velocity for stride phase
    FVector LeftFootVel = (LeftFootWorld - LastLeftFootPos) / FMath::Max(DeltaSeconds, 0.0001f);
    FVector RightFootVel = (RightFootWorld - LastRightFootPos) / FMath::Max(DeltaSeconds, 0.0001f);

    float LeftFootMoveComp = FVector::DotProduct(LeftFootVel, MoveDir);
    float RightFootMoveComp = FVector::DotProduct(RightFootVel, MoveDir);

    float LeftFootRel = FVector::DotProduct((LeftFootWorld - PelvisWorld), MoveDir);
    float RightFootRel = FVector::DotProduct((RightFootWorld - PelvisWorld), MoveDir);

    const float PlantVelThreshold = 5.0f;
    const float PlantDistThreshold = 15.0f;

    bLeftFootPlanted = (FMath::Abs(LeftFootMoveComp) < PlantVelThreshold) && (FMath::Abs(LeftFootRel) < PlantDistThreshold);
    bRightFootPlanted = (FMath::Abs(RightFootMoveComp) < PlantVelThreshold) && (FMath::Abs(RightFootRel) < PlantDistThreshold);

    // --- Predictive bias for anticipation (step boost)
    float BaseBias = 18.f;
    float StepBoost = 18.f;
    float PredictBiasLeft = bLeftFootPlanted ? BaseBias : BaseBias + StepBoost;
    float PredictBiasRight = bRightFootPlanted ? BaseBias : BaseBias + StepBoost;

    // --- Capsule ground traces (for robust contact)
    FHitResult LeftFootHit, RightFootHit, LeftToeHit, RightToeHit;
    bool bLeftHit = CapsuleGroundTrace(
        Mesh, FName("foot_l"), 60.f, PredictBiasLeft, 8.f, 6.f, CharVel, LeftFootHit, true);
    bool bRightHit = CapsuleGroundTrace(
        Mesh, FName("foot_r"), 60.f, PredictBiasRight, 8.f, 6.f, CharVel, RightFootHit, true);
    // Trace toes
    bool bLeftToeHit = CapsuleGroundTrace(
        Mesh, FName("ball_l"), 60.f, PredictBiasLeft, 8.f, 6.f, CharVel, LeftToeHit, false);
    bool bRightToeHit = CapsuleGroundTrace(
        Mesh, FName("ball_r"), 60.f, PredictBiasRight, 8.f, 6.f, CharVel, RightToeHit, false);

    // --- Foot lock logic (procedural world-lock)
    float TargetLeftOffset = 0.f, TargetRightOffset = 0.f;
    FRotator TargetLeftRot = FRotator::ZeroRotator, TargetRightRot = FRotator::ZeroRotator;
    FVector TargetLeftEffector = FVector::ZeroVector, TargetRightEffector = FVector::ZeroVector;

    // LEFT FOOT
    // --- Toe-corrected rotation math
    FVector LAnkleToToe = LeftToeWorld - LeftFootWorld;
    float LToeTargetZ = bLeftToeHit ? LeftToeHit.ImpactPoint.Z : LeftToeWorld.Z;
    float LAnkleTargetZ = bLeftHit ? LeftFootHit.ImpactPoint.Z : LeftFootWorld.Z;
    float LToeDeltaZ = LToeTargetZ - LAnkleTargetZ;
    float LToeDistance = LAnkleToToe.Size2D();
    float LPitchRadians = (LToeDistance > 1e-3f) ? FMath::Atan2(LToeDeltaZ, LToeDistance) : 0.f;
    float LPitchDegrees = FMath::RadiansToDegrees(LPitchRadians);
    FRotator LBaseRot = bLeftHit ? LeftFootHit.ImpactNormal.Rotation() : FRotator::ZeroRotator;
    FRotator LToeCorrection = FRotator(LPitchDegrees, 0.f, 0.f);
    FRotator LFinalRot = (LBaseRot.Quaternion() * LToeCorrection.Quaternion()).Rotator();
    LFinalRot.Pitch = FMath::ClampAngle(LFinalRot.Pitch, -45.f, 45.f);

    if (bLeftFootPlanted && bLeftHit)
    {
        if (LockedLeftFootWorldPos.IsZero())
            LockedLeftFootWorldPos = LeftFootHit.ImpactPoint;
        TargetLeftOffset = LockedLeftFootWorldPos.Z - LeftFootWorld.Z;
        TargetLeftRot = LFinalRot;
        TargetLeftEffector = Mesh->GetComponentTransform().InverseTransformPosition(LockedLeftFootWorldPos);
    }
    else if (bLeftHit)
    {
        LockedLeftFootWorldPos = FVector::ZeroVector;
        TargetLeftOffset = LeftFootHit.ImpactPoint.Z - LeftFootWorld.Z;
        TargetLeftRot = LFinalRot;
        TargetLeftEffector = Mesh->GetComponentTransform().InverseTransformPosition(LeftFootHit.ImpactPoint);
    }

    // RIGHT FOOT
    FVector RAnkleToToe = RightToeWorld - RightFootWorld;
    float RToeTargetZ = bRightToeHit ? RightToeHit.ImpactPoint.Z : RightToeWorld.Z;
    float RAnkleTargetZ = bRightHit ? RightFootHit.ImpactPoint.Z : RightFootWorld.Z;
    float RToeDeltaZ = RToeTargetZ - RAnkleTargetZ;
    float RToeDistance = RAnkleToToe.Size2D();
    float RPitchRadians = (RToeDistance > 1e-3f) ? FMath::Atan2(RToeDeltaZ, RToeDistance) : 0.f;
    float RPitchDegrees = FMath::RadiansToDegrees(RPitchRadians);
    FRotator RBaseRot = bRightHit ? RightFootHit.ImpactNormal.Rotation() : FRotator::ZeroRotator;
    FRotator RToeCorrection = FRotator(RPitchDegrees, 0.f, 0.f);
    FRotator RFinalRot = (RBaseRot.Quaternion() * RToeCorrection.Quaternion()).Rotator();
    RFinalRot.Pitch = FMath::ClampAngle(RFinalRot.Pitch, -45.f, 45.f);

    if (bRightFootPlanted && bRightHit)
    {
        if (LockedRightFootWorldPos.IsZero())
            LockedRightFootWorldPos = RightFootHit.ImpactPoint;
        TargetRightOffset = LockedRightFootWorldPos.Z - RightFootWorld.Z;
        TargetRightRot = RFinalRot;
        TargetRightEffector = Mesh->GetComponentTransform().InverseTransformPosition(LockedRightFootWorldPos);
    }
    else if (bRightHit)
    {
        LockedRightFootWorldPos = FVector::ZeroVector;
        TargetRightOffset = RightFootHit.ImpactPoint.Z - RightFootWorld.Z;
        TargetRightRot = RFinalRot;
        TargetRightEffector = Mesh->GetComponentTransform().InverseTransformPosition(RightFootHit.ImpactPoint);
    }

    // --- Interpolate for smoothness
    const float InterpSpeed = 15.f;
    LeftFootIKOffset = FMath::FInterpTo(LeftFootIKOffset, TargetLeftOffset, DeltaSeconds, InterpSpeed);
    RightFootIKOffset = FMath::FInterpTo(RightFootIKOffset, TargetRightOffset, DeltaSeconds, InterpSpeed);
    LeftFootIKRotation = FMath::RInterpTo(LeftFootIKRotation, TargetLeftRot, DeltaSeconds, InterpSpeed);
    RightFootIKRotation = FMath::RInterpTo(RightFootIKRotation, TargetRightRot, DeltaSeconds, InterpSpeed);
    LeftFootIKEffector = FMath::VInterpTo(LeftFootIKEffector, TargetLeftEffector, DeltaSeconds, InterpSpeed);
    RightFootIKEffector = FMath::VInterpTo(RightFootIKEffector, TargetRightEffector, DeltaSeconds, InterpSpeed);

    // Pelvis offset: keep feet on the ground
    float TargetPelvis = FMath::Min(LeftFootIKOffset, RightFootIKOffset);
    PelvisIKOffset = FMath::FInterpTo(PelvisIKOffset, TargetPelvis, DeltaSeconds, 10.f);

    // Slope and stair/step info
    FootDeltaZ = FMath::Abs(TargetLeftOffset - TargetRightOffset);
    LeftFootSlopeAngle = bLeftHit ? FMath::RadiansToDegrees(acosf(FVector::DotProduct(LeftFootHit.ImpactNormal, FVector::UpVector))) : 0.f;
    RightFootSlopeAngle = bRightHit ? FMath::RadiansToDegrees(acosf(FVector::DotProduct(RightFootHit.ImpactNormal, FVector::UpVector))) : 0.f;

    // Save for next update
    LastLeftFootPos = LeftFootWorld;
    LastRightFootPos = RightFootWorld;
}
#endif

#if 0
void UOHAnimInstance_Base::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    APawn* OwnerPawn = TryGetPawnOwner();
    if (!OwnerPawn) return;
    USkeletalMeshComponent* Mesh = GetSkelMeshComponent();
    if (!Mesh) return;

    // --- Measure foot length (auto, once) ---
    if (!bMeasuredFootLength)
    {
        FVector Ankle = Mesh->GetBoneLocation(FName("foot_l"));
        FVector Toe = Mesh->GetBoneLocation(FName("ball_l"));
        AutoFootLength = (Toe - Ankle).Size2D();
        bMeasuredFootLength = true;
    }

    // --- Sample world positions
    FVector LeftAnkleWS  = Mesh->GetBoneLocation(FName("foot_l"));
    FVector LeftToeWS    = Mesh->GetBoneLocation(FName("ball_l"));
    FVector RightAnkleWS = Mesh->GetBoneLocation(FName("foot_r"));
    FVector RightToeWS   = Mesh->GetBoneLocation(FName("ball_r"));
    FVector PelvisWorld  = Mesh->GetBoneLocation(FName("pelvis"));

    float TimeNow = GetWorld()->TimeSeconds;

    // --- Update foot buffers ---
    LeftFootBuffer.AddSample(LeftAnkleWS, TimeNow);
    RightFootBuffer.AddSample(RightAnkleWS, TimeNow);

    // --- Character movement ---
    FVector CharVel = OwnerPawn->GetVelocity();
    FVector MoveDir = CharVel.IsNearlyZero() ? OwnerPawn->GetActorForwardVector() : CharVel.GetSafeNormal();

    // --- Phase detection ---
    EFootPhase LeftPhase = DetectFootPhase(LeftFootBuffer);
    EFootPhase RightPhase = DetectFootPhase(RightFootBuffer);

    float HeelToeAlphaLeft = GetHeelToeBlendAlpha(LeftPhase, LeftFootBuffer);
    float HeelToeAlphaRight = GetHeelToeBlendAlpha(RightPhase, RightFootBuffer);

    // --- Traces for foot and toe (robust contact) ---
    float BiasLeft = (LeftPhase == EFootPhase::Plant) ? 18.f : 36.f;
    float BiasRight = (RightPhase == EFootPhase::Plant) ? 18.f : 36.f;

    FHitResult LAnkleHit, LToeHit, RAnkleHit, RToeHit;
    bool bLAnkleHit = CapsuleGroundTrace(Mesh, FName("foot_l"), 60.f, BiasLeft, 8.f, 6.f, CharVel, LAnkleHit, false);
    bool bLToeHit   = CapsuleGroundTrace(Mesh, FName("ball_l"), 60.f, BiasLeft, 8.f, 6.f, CharVel, LToeHit, false);
    bool bRAnkleHit = CapsuleGroundTrace(Mesh, FName("foot_r"), 60.f, BiasRight, 8.f, 6.f, CharVel, RAnkleHit, false);
    bool bRToeHit   = CapsuleGroundTrace(Mesh, FName("ball_r"), 60.f, BiasRight, 8.f, 6.f, CharVel, RToeHit, false);

    // --- Heel-toe roll target ---
    FVector LTargetWS = FMath::Lerp(LeftAnkleWS, LeftToeWS, HeelToeAlphaLeft);
    FVector RTargetWS = FMath::Lerp(RightAnkleWS, RightToeWS, HeelToeAlphaRight);

    float LTargetZ = FMath::Lerp(
        bLAnkleHit ? LAnkleHit.ImpactPoint.Z : LeftAnkleWS.Z,
        bLToeHit ? LToeHit.ImpactPoint.Z : LeftToeWS.Z,
        HeelToeAlphaLeft
    );
    float RTargetZ = FMath::Lerp(
        bRAnkleHit ? RAnkleHit.ImpactPoint.Z : RightAnkleWS.Z,
        bRToeHit ? RToeHit.ImpactPoint.Z : RightToeWS.Z,
        HeelToeAlphaRight
    );

    // --- Plant lock logic (keep foot in place on plant) ---
    if (LeftPhase == EFootPhase::Plant && bLAnkleHit)
    {
        if (LockedLeftFootWorldPos.IsZero())
            LockedLeftFootWorldPos = LAnkleHit.ImpactPoint;
        LTargetWS = LockedLeftFootWorldPos;
        LTargetZ = LockedLeftFootWorldPos.Z;
    }
    else
    {
        LockedLeftFootWorldPos = FVector::ZeroVector;
    }
    if (RightPhase == EFootPhase::Plant && bRAnkleHit)
    {
        if (LockedRightFootWorldPos.IsZero())
            LockedRightFootWorldPos = RAnkleHit.ImpactPoint;
        RTargetWS = LockedRightFootWorldPos;
        RTargetZ = LockedRightFootWorldPos.Z;
    }
    else
    {
        LockedRightFootWorldPos = FVector::ZeroVector;
    }

    // --- Dynamic Step Anticipation ("Ghost Step") ---
    float GhostStepLookaheadTime = 0.08f; // ~5 frames at 60fps
    float GhostStepBlend = 0.25f;
    FVector LFootVel = LeftFootBuffer.GetVelocity();
    FVector RFootVel = RightFootBuffer.GetVelocity();
    FVector PredictedStep_L = LeftAnkleWS + LFootVel * GhostStepLookaheadTime;
    FVector PredictedStep_R = RightAnkleWS + RFootVel * GhostStepLookaheadTime;
    if (LeftPhase == EFootPhase::Swing || LeftPhase == EFootPhase::Lift)
        LTargetWS = FMath::Lerp(LTargetWS, PredictedStep_L, GhostStepBlend);
    if (RightPhase == EFootPhase::Swing || RightPhase == EFootPhase::Lift)
        RTargetWS = FMath::Lerp(RTargetWS, PredictedStep_R, GhostStepBlend);

    // --- Procedural Side-Step and Pivot Detection ---
    static FVector LastPelvisWorld = FVector::ZeroVector;
    float SideStepBlend = 0.35f;   // Tweak as needed
    float SideStepAngleThreshold = 40.f;
    FVector PelvisMoveDir = (PelvisWorld - LastPelvisWorld).GetSafeNormal();
    float PelvisSpeed = (PelvisWorld - LastPelvisWorld).Size() / FMath::Max(DeltaSeconds, 0.0001f);
    if (PelvisSpeed > 1.f)
    {
        float SideAngle_L = FMath::RadiansToDegrees(acosf(FVector::DotProduct(MoveDir, PelvisMoveDir)));
        if (SideAngle_L > SideStepAngleThreshold)
        {
            FVector OutDir = FVector::CrossProduct(PelvisMoveDir, FVector::UpVector).GetSafeNormal();
            LTargetWS += OutDir * FMath::Sign(FVector::DotProduct(LFootVel, OutDir)) * SideStepBlend * AutoFootLength;
            RTargetWS += OutDir * FMath::Sign(FVector::DotProduct(RFootVel, OutDir)) * SideStepBlend * AutoFootLength;
        }
    }
    LastPelvisWorld = PelvisWorld;

    // --- Offset calculation (Z) for pose modify bone
    float TargetLeftOffset = LTargetZ - LeftAnkleWS.Z;
    float TargetRightOffset = RTargetZ - RightAnkleWS.Z;

    // --- Rotation (uses normal at blended point) ---
    FRotator TargetLeftRot = bLAnkleHit ? LAnkleHit.ImpactNormal.Rotation() : FRotator::ZeroRotator;
    FRotator TargetRightRot = bRAnkleHit ? RAnkleHit.ImpactNormal.Rotation() : FRotator::ZeroRotator;

    // --- Biomechanical Ankle Limit Constraints ---
    FRotator AnklePitchLimits = FRotator(45.f, 0.f, 0.f);  // +/- degrees
    FRotator AnkleYawLimits   = FRotator(0.f, 25.f, 0.f);
    FRotator AnkleRollLimits  = FRotator(0.f, 0.f, 25.f);
    TargetLeftRot.Pitch  = FMath::Clamp(TargetLeftRot.Pitch,  -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    TargetLeftRot.Yaw    = FMath::Clamp(TargetLeftRot.Yaw,    -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    TargetLeftRot.Roll   = FMath::Clamp(TargetLeftRot.Roll,   -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);
    TargetRightRot.Pitch = FMath::Clamp(TargetRightRot.Pitch, -AnklePitchLimits.Pitch,  AnklePitchLimits.Pitch);
    TargetRightRot.Yaw   = FMath::Clamp(TargetRightRot.Yaw,   -AnkleYawLimits.Yaw,      AnkleYawLimits.Yaw);
    TargetRightRot.Roll  = FMath::Clamp(TargetRightRot.Roll,  -AnkleRollLimits.Roll,    AnkleRollLimits.Roll);

    // --- Effector calculation in Component Space
    FVector LEffectorCS = Mesh->GetComponentTransform().InverseTransformPosition(LTargetWS);
    FVector REffectorCS = Mesh->GetComponentTransform().InverseTransformPosition(RTargetWS);

    // --- Physically inspired damping for realism
    DampedLeftEffector = CriticallyDampedSmoothing(DampedLeftEffector, LEffectorCS, DeltaSeconds, 80.f, 1.2f);
    DampedRightEffector = CriticallyDampedSmoothing(DampedRightEffector, REffectorCS, DeltaSeconds, 80.f, 1.2f);

    // --- Output variables for AnimGraph
    LeftFootIKEffector = DampedLeftEffector;
    RightFootIKEffector = DampedRightEffector;
    LeftFootIKRotation = FMath::RInterpTo(LeftFootIKRotation, TargetLeftRot, DeltaSeconds, 15.f);
    RightFootIKRotation = FMath::RInterpTo(RightFootIKRotation, TargetRightRot, DeltaSeconds, 15.f);
    LeftFootIKOffset = FMath::FInterpTo(LeftFootIKOffset, TargetLeftOffset, DeltaSeconds, 15.f);
    RightFootIKOffset = FMath::FInterpTo(RightFootIKOffset, TargetRightOffset, DeltaSeconds, 15.f);

    // Pelvis offset for keeping feet on ground
    float TargetPelvis = FMath::Min(LeftFootIKOffset, RightFootIKOffset);
    PelvisIKOffset = FMath::FInterpTo(PelvisIKOffset, TargetPelvis, DeltaSeconds, 10.f);

    FootDeltaZ = FMath::Abs(TargetLeftOffset - TargetRightOffset);
}
#endif

