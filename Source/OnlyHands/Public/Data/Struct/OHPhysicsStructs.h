
#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Utilities/OHSafeMapUtils.h"
#include "BonePose.h"
#include "Components/SkeletalMeshComponent.h"
#include "OnlyHands/Game/Enums/E_FightData.h"
#include "OnlyHands/Game/Structures/S_FightStructure.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "OHPhysicsStructs.generated.h"

#if 0
#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Curves/CurveFloat.h"
#include "Utilities/OHSafeMapUtils.h"
#include "OHPrimaryStructs.generated.h"

// Forward declarations
class UOHPACManager;


// ================================================================================
// LEVEL 1: FOHMotionFrameSample - Atomic Motion Data
// ================================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHMotionFrameSample
{
    GENERATED_BODY()

    // === CORE SPATIAL STATE ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Position")
    FVector WorldPosition = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Position")
    FQuat WorldRotation = FQuat::Identity;

    // === TEMPORAL DATA ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Time")
    float TimeStamp = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Time")
    float DeltaTime = 0.016f;

    // === MULTI-SPACE VELOCITY ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Velocity")
    FVector WorldLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Velocity")
    FVector LocalLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Velocity")
    FVector ComponentLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Velocity")
    FVector BoneLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Velocity")
    FVector AngularVelocity = FVector::ZeroVector;

    // === MULTI-SPACE ACCELERATION ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Acceleration")
    FVector WorldLinearAcceleration = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Acceleration")
    FVector LocalLinearAcceleration = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Acceleration")
    FVector ComponentLinearAcceleration = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Acceleration")
    FVector BoneLinearAcceleration = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Acceleration")
    FVector AngularAcceleration = FVector::ZeroVector;

    // === JERK (3rd DERIVATIVE) ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Jerk")
    FVector WorldLinearJerk = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Jerk")
    FVector LocalLinearJerk = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Jerk")
    FVector ComponentLinearJerk = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Jerk")
    FVector BoneLinearJerk = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Jerk")
    FVector AngularJerk = FVector::ZeroVector;

    // === REFERENCE TRANSFORMS ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Reference")
    FTransform ComponentTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Reference")
    FTransform BoneReferenceTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Reference")
    FName ReferenceBoneName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Reference")
    FVector ReferenceVelocity = FVector::ZeroVector;

    // === SAMPLE QUALITY ===
    UPROPERTY(BlueprintReadOnly, Category = "Sample|Quality")
    float SampleQuality = 1.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Quality")
    bool bIsInterpolated = false;

    UPROPERTY(BlueprintReadOnly, Category = "Sample|Quality")
    bool bHasValidData = true;

    // === FACTORY METHODS ===

    /**
     * Create sample from raw motion data with full multi-space calculation
     */
    static FOHMotionFrameSample CreateFromMotionData(
        const FVector& WorldPos,
        const FQuat& WorldRot,
        const FVector& ComponentVelocity,
        const FTransform& ComponentTransform,
        const FTransform& BoneRefTransform,
        FName RefBoneName,
        const FVector& ReferenceVelocity,
        float TimeStamp,
        const FOHMotionFrameSample* PreviousSample = nullptr)
    {
        FOHMotionFrameSample Sample;
        
        // Core data
        Sample.WorldPosition = WorldPos;
        Sample.WorldRotation = WorldRot;
        Sample.TimeStamp = TimeStamp;
        Sample.ComponentTransform = ComponentTransform;
        Sample.BoneReferenceTransform = BoneRefTransform;
        Sample.ReferenceBoneName = RefBoneName;
        Sample.ReferenceVelocity = ReferenceVelocity;
        
        // Calculate velocities in all spaces
        Sample.WorldLinearVelocity = ComponentVelocity;
        Sample.LocalLinearVelocity = ComponentVelocity - ReferenceVelocity;
        Sample.ComponentLinearVelocity = ComponentTransform.InverseTransformVector(ComponentVelocity);
        Sample.BoneLinearVelocity = BoneRefTransform.InverseTransformVector(ComponentVelocity);
        
        // Calculate accelerations and jerk if we have previous sample
        if (PreviousSample && PreviousSample->bHasValidData)
        {
            Sample.DeltaTime = TimeStamp - PreviousSample->TimeStamp;
            
            if (Sample.DeltaTime > KINDA_SMALL_NUMBER)
            {
                // Accelerations
                Sample.WorldLinearAcceleration = (Sample.WorldLinearVelocity - PreviousSample->WorldLinearVelocity) / Sample.DeltaTime;
                Sample.LocalLinearAcceleration = (Sample.LocalLinearVelocity - PreviousSample->LocalLinearVelocity) / Sample.DeltaTime;
                Sample.ComponentLinearAcceleration = (Sample.ComponentLinearVelocity - PreviousSample->ComponentLinearVelocity) / Sample.DeltaTime;
                Sample.BoneLinearAcceleration = (Sample.BoneLinearVelocity - PreviousSample->BoneLinearVelocity) / Sample.DeltaTime;
                
                // Angular
                FQuat DeltaRot = WorldRot * PreviousSample->WorldRotation.Inverse();
                FVector Axis;
                float Angle;
                DeltaRot.ToAxisAndAngle(Axis, Angle);
                Sample.AngularVelocity = (Axis * Angle) / Sample.DeltaTime;
                Sample.AngularAcceleration = (Sample.AngularVelocity - PreviousSample->AngularVelocity) / Sample.DeltaTime;
                
                // Jerk calculations
                Sample.WorldLinearJerk = (Sample.WorldLinearAcceleration - PreviousSample->WorldLinearAcceleration) / Sample.DeltaTime;
                Sample.LocalLinearJerk = (Sample.LocalLinearAcceleration - PreviousSample->LocalLinearAcceleration) / Sample.DeltaTime;
                Sample.ComponentLinearJerk = (Sample.ComponentLinearAcceleration - PreviousSample->ComponentLinearAcceleration) / Sample.DeltaTime;
                Sample.BoneLinearJerk = (Sample.BoneLinearAcceleration - PreviousSample->BoneLinearAcceleration) / Sample.DeltaTime;
                Sample.AngularJerk = (Sample.AngularAcceleration - PreviousSample->AngularAcceleration) / Sample.DeltaTime;
            }
        }
        
        return Sample;
    }

    /**
     * Get velocity in specified reference space
     */
    FVector GetVelocity(EOHReferenceSpace Space) const
    {
        switch (Space)
        {
            case EOHReferenceSpace::WorldSpace:
                return WorldLinearVelocity;
            case EOHReferenceSpace::LocalSpace:
                return LocalLinearVelocity;
            case EOHReferenceSpace::ComponentSpace:
                return ComponentLinearVelocity;
            case EOHReferenceSpace::BoneSpace:
                return BoneLinearVelocity;
            default:
                return WorldLinearVelocity;
        }
    }

    /**
     * Get acceleration in specified reference space
     */
    FVector GetAcceleration(EOHReferenceSpace Space) const
    {
        switch (Space)
        {
            case EOHReferenceSpace::WorldSpace:
                return WorldLinearAcceleration;
            case EOHReferenceSpace::LocalSpace:
                return LocalLinearAcceleration;
            case EOHReferenceSpace::ComponentSpace:
                return ComponentLinearAcceleration;
            case EOHReferenceSpace::BoneSpace:
                return BoneLinearAcceleration;
            default:
                return WorldLinearAcceleration;
        }
    }

    /**
     * Get jerk in specified reference space
     */
    FVector GetJerk(EOHReferenceSpace Space) const
    {
        switch (Space)
        {
            case EOHReferenceSpace::WorldSpace:
                return WorldLinearJerk;
            case EOHReferenceSpace::LocalSpace:
                return LocalLinearJerk;
            case EOHReferenceSpace::ComponentSpace:
                return ComponentLinearJerk;
            case EOHReferenceSpace::BoneSpace:
                return BoneLinearJerk;
            default:
                return WorldLinearJerk;
        }
    }

    /**
     * Compare with another sample
     */
    float GetSimilarityTo(const FOHMotionFrameSample& Other, EOHReferenceSpace Space) const
    {
        FVector VelDiff = GetVelocity(Space) - Other.GetVelocity(Space);
        FVector AccDiff = GetAcceleration(Space) - Other.GetAcceleration(Space);
        
        float VelSimilarity = FMath::Exp(-VelDiff.Size() * 0.001f);
        float AccSimilarity = FMath::Exp(-AccDiff.Size() * 0.0001f);
        
        return (VelSimilarity * 0.7f + AccSimilarity * 0.3f);
    }

    /**
     * Interpolate between samples
     */
    static FOHMotionFrameSample Interpolate(const FOHMotionFrameSample& A, const FOHMotionFrameSample& B, float Alpha)
    {
        FOHMotionFrameSample Result;
        
        Result.TimeStamp = FMath::Lerp(A.TimeStamp, B.TimeStamp, Alpha);
        Result.WorldPosition = FMath::Lerp(A.WorldPosition, B.WorldPosition, Alpha);
        Result.WorldRotation = FQuat::Slerp(A.WorldRotation, B.WorldRotation, Alpha);
        
        // Interpolate all velocity spaces
        Result.WorldLinearVelocity = FMath::Lerp(A.WorldLinearVelocity, B.WorldLinearVelocity, Alpha);
        Result.LocalLinearVelocity = FMath::Lerp(A.LocalLinearVelocity, B.LocalLinearVelocity, Alpha);
        Result.ComponentLinearVelocity = FMath::Lerp(A.ComponentLinearVelocity, B.ComponentLinearVelocity, Alpha);
        Result.BoneLinearVelocity = FMath::Lerp(A.BoneLinearVelocity, B.BoneLinearVelocity, Alpha);
        
        // Interpolate accelerations
        Result.WorldLinearAcceleration = FMath::Lerp(A.WorldLinearAcceleration, B.WorldLinearAcceleration, Alpha);
        Result.LocalLinearAcceleration = FMath::Lerp(A.LocalLinearAcceleration, B.LocalLinearAcceleration, Alpha);
        Result.ComponentLinearAcceleration = FMath::Lerp(A.ComponentLinearAcceleration, B.ComponentLinearAcceleration, Alpha);
        Result.BoneLinearAcceleration = FMath::Lerp(A.BoneLinearAcceleration, B.BoneLinearAcceleration, Alpha);
        
        Result.bIsInterpolated = true;
        Result.SampleQuality = FMath::Min(A.SampleQuality, B.SampleQuality) * 0.9f;
        
        return Result;
    }
};

// ================================================================================
// LEVEL 2: FOHBoneMotionData - Per-Bone Historical Motion Tracking
// ================================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHBoneMotionData
{
    GENERATED_BODY()

    // === IDENTIFICATION ===
    UPROPERTY(BlueprintReadOnly, Category = "Bone|Identity")
    FName BoneName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Identity")
    FName ParentBoneName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Identity")
    TArray<FName> ChildBoneNames;

    
    // === MOTION HISTORY ===
    UPROPERTY(BlueprintReadOnly, Category = "Bone|History")
    OHSafeMapUtils::TRollingBuffer<FOHMotionFrameSample, 16> MotionHistory;

    // === STRIKE DETECTION ===
    UPROPERTY(BlueprintReadOnly, Category = "Bone|Strike")
    float PeakAcceleration = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Strike")
    float PeakJerk = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Strike")
    float AccelerationBuildupTime = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Strike")
    bool bLikelyStrike = false;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Strike")
    float StrikeConfidence = 0.0f;

    // === MOTION QUALITY ===
    UPROPERTY(BlueprintReadOnly, Category = "Bone|Quality")
    float MotionQuality = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Quality")
    float MotionConsistency = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Bone|Quality")
    mutable EOHReferenceSpace OptimalAnalysisSpace = EOHReferenceSpace::WorldSpace;

    // === CACHED CALCULATIONS ===
    mutable float CachedMotionQuality = -1.0f;
    mutable EOHReferenceSpace CachedQualitySpace = EOHReferenceSpace::WorldSpace;
    mutable bool bMotionQualityCacheValid = false;

    // === INITIALIZATION ===

    void Initialize(FName InBoneName, FName InParentBone = NAME_None)
    {
        BoneName = InBoneName;
        ParentBoneName = InParentBone;
        PeakAcceleration = 0.0f;
        PeakJerk = 0.0f;
        AccelerationBuildupTime = 0.0f;
        bLikelyStrike = false;
        StrikeConfidence = 0.0f;
        InvalidateCaches();
    }

    // === SAMPLE MANAGEMENT ===

    /**
     * Add new motion sample with full multi-space calculation
     */
    void AddMotionSample(const FVector& WorldPos, const FQuat& WorldRot,
                        const FVector& ComponentVelocity, const FTransform& ComponentTransform,
                        const FTransform& BoneRefTransform, FName RefBoneName,
                        const FVector& ReferenceVelocity, float TimeStamp)
    {
        const FOHMotionFrameSample* PreviousSample = GetLatestSample();
        
        FOHMotionFrameSample NewSample = FOHMotionFrameSample::CreateFromMotionData(
            WorldPos, WorldRot, ComponentVelocity, ComponentTransform,
            BoneRefTransform, RefBoneName, ReferenceVelocity, TimeStamp, PreviousSample
        );
        
        // Update strike detection metrics
        UpdateStrikeMetrics(NewSample);
        
        // Add to history
        MotionHistory.Add(NewSample);
        
        // Invalidate caches
        InvalidateCaches();
    }

    /**
     * Add pre-calculated sample directly
     */
    void AddSample(const FOHMotionFrameSample& Sample)
    {
        UpdateStrikeMetrics(Sample);
        MotionHistory.Add(Sample);
        InvalidateCaches();
    }

    // === INTERSAMPLE COMMUNICATION ===

    /**
     * Compare motion with another bone using arbitrary reference space
     */
    float GetMotionSimilarityTo(const FOHBoneMotionData& OtherBone, EOHReferenceSpace Space) const
    {
        int32 MinSamples = FMath::Min(GetHistoryDepth(), OtherBone.GetHistoryDepth());
        if (MinSamples < 2) return 0.0f;
        
        float TotalSimilarity = 0.0f;
        int32 ValidComparisons = 0;
        
        for (int32 i = 0; i < MinSamples; ++i)
        {
            const FOHMotionFrameSample* ThisSample = MotionHistory.GetLatest(i);
            const FOHMotionFrameSample* OtherSample = OtherBone.MotionHistory.GetLatest(i);
            
            if (ThisSample && OtherSample && ThisSample->bHasValidData && OtherSample->bHasValidData)
            {
                TotalSimilarity += ThisSample->GetSimilarityTo(*OtherSample, Space);
                ValidComparisons++;
            }
        }
        
        return ValidComparisons > 0 ? (TotalSimilarity / ValidComparisons) : 0.0f;
    }

    /**
     * Calculate relative velocity to another bone
     */
    FVector GetRelativeVelocityTo(const FOHBoneMotionData& OtherBone, EOHReferenceSpace Space) const
    {
        const FOHMotionFrameSample* ThisSample = GetLatestSample();
        const FOHMotionFrameSample* OtherSample = OtherBone.GetLatestSample();
        
        if (ThisSample && OtherSample)
        {
            return ThisSample->GetVelocity(Space) - OtherSample->GetVelocity(Space);
        }
        
        return FVector::ZeroVector;
    }

    /**
     * Check phase synchronization with another bone
     */
    float GetPhaseSynchronization(const FOHBoneMotionData& OtherBone, float TimeWindow = 0.5f) const
    {
        float CurrentTime = GetLatestSample() ? GetLatestSample()->TimeStamp : 0.0f;
        
        TArray<FOHMotionFrameSample> ThisSamples = GetSamplesInTimeRange(CurrentTime - TimeWindow, CurrentTime);
        TArray<FOHMotionFrameSample> OtherSamples = OtherBone.GetSamplesInTimeRange(CurrentTime - TimeWindow, CurrentTime);
        
        if (ThisSamples.Num() < 2 || OtherSamples.Num() < 2) return 0.0f;
        
        float TotalSync = 0.0f;
        int32 Count = FMath::Min(ThisSamples.Num(), OtherSamples.Num());
        
        for (int32 i = 0; i < Count; ++i)
        {
            FVector ThisDir = ThisSamples[i].WorldLinearVelocity.GetSafeNormal();
            FVector OtherDir = OtherSamples[i].WorldLinearVelocity.GetSafeNormal();
            TotalSync += FVector::DotProduct(ThisDir, OtherDir);
        }
        
        return TotalSync / Count;
    }

    /**
     * Use another bone as reference transform
     */
    void SetReferenceBone(const FOHBoneMotionData& ReferenceBone)
    {
        if (FOHMotionFrameSample* Latest = MotionHistory.GetLatestMutable())
        {
            if (const FOHMotionFrameSample* RefSample = ReferenceBone.GetLatestSample())
            {
                Latest->ReferenceBoneName = ReferenceBone.BoneName;
                Latest->BoneReferenceTransform = FTransform(RefSample->WorldRotation, RefSample->WorldPosition);
                Latest->ReferenceVelocity = RefSample->WorldLinearVelocity;
                
                // Recalculate bone-space velocities
                Latest->BoneLinearVelocity = Latest->BoneReferenceTransform.InverseTransformVector(Latest->WorldLinearVelocity);
                Latest->BoneLinearAcceleration = Latest->BoneReferenceTransform.InverseTransformVector(Latest->WorldLinearAcceleration);
                Latest->BoneLinearJerk = Latest->BoneReferenceTransform.InverseTransformVector(Latest->WorldLinearJerk);
            }
        }
    }

    // === TEMPORAL ANALYSIS ===

    /**
     * Get samples within time range
     */
    TArray<FOHMotionFrameSample> GetSamplesInTimeRange(float StartTime, float EndTime) const
    {
        TArray<FOHMotionFrameSample> Result;
        
        for (int32 i = 0; i < MotionHistory.NumFrames(); ++i)
        {
            const FOHMotionFrameSample* Sample = MotionHistory.GetLatest(i);
            if (Sample && Sample->TimeStamp >= StartTime && Sample->TimeStamp <= EndTime)
            {
                Result.Add(*Sample);
            }
        }
        
        return Result;
    }

    /**
     * Get interpolated sample at specific time
     */
    FOHMotionFrameSample GetInterpolatedSampleAtTime(float TargetTime) const
    {
        // Find bracketing samples
        const FOHMotionFrameSample* Before = nullptr;
        const FOHMotionFrameSample* After = nullptr;
        
        for (int32 i = 0; i < MotionHistory.NumFrames() - 1; ++i)
        {
            const FOHMotionFrameSample* Current = MotionHistory.GetLatest(i);
            const FOHMotionFrameSample* Next = MotionHistory.GetLatest(i + 1);
            
            if (Current && Next)
            {
                if (Next->TimeStamp <= TargetTime && Current->TimeStamp >= TargetTime)
                {
                    Before = Next;
                    After = Current;
                    break;
                }
            }
        }
        
        if (Before && After)
        {
            float Alpha = (TargetTime - Before->TimeStamp) / (After->TimeStamp - Before->TimeStamp);
            return FOHMotionFrameSample::Interpolate(*Before, *After, Alpha);
        }
        
        // Return latest if interpolation not possible
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? *Latest : FOHMotionFrameSample();
    }

    // === MOTION QUALITY ASSESSMENT ===

    /**
     * Calculate motion quality in specified reference space
     */
    float CalculateMotionQuality(EOHReferenceSpace Space) const
    {
        if (bMotionQualityCacheValid && CachedQualitySpace == Space)
        {
            return CachedMotionQuality;
        }
        
        if (GetHistoryDepth() < 3) return 0.0f;
        
        float TotalQuality = 0.0f;
        float WeightSum = 0.0f;
        
        for (int32 i = 0; i < FMath::Min(8, GetHistoryDepth()); ++i)
        {
            const FOHMotionFrameSample* Sample = MotionHistory.GetLatest(i);
            if (Sample && Sample->bHasValidData)
            {
                float Weight = 1.0f / (i + 1.0f); // Recent samples weighted more
                
                // Quality based on velocity magnitude and consistency
                FVector Vel = Sample->GetVelocity(Space);
                float Speed = Vel.Size();
                float SpeedQuality = FMath::Clamp(Speed / 1000.0f, 0.0f, 1.0f);
                
                // Acceleration consistency
                FVector Accel = Sample->GetAcceleration(Space);
                float AccelQuality = 1.0f - FMath::Clamp(Accel.Size() / 10000.0f, 0.0f, 1.0f);
                
                float SampleQuality = (SpeedQuality * 0.6f + AccelQuality * 0.4f) * Sample->SampleQuality;
                
                TotalQuality += SampleQuality * Weight;
                WeightSum += Weight;
            }
        }
        
        CachedMotionQuality = WeightSum > 0.0f ? (TotalQuality / WeightSum) : 0.0f;
        CachedQualitySpace = Space;
        bMotionQualityCacheValid = true;
        
        return CachedMotionQuality;
    }

    /**
     * Determine optimal reference space for analysis
     */
    EOHReferenceSpace DetermineOptimalSpace() const
    {
        float BestClarity = 0.0f;
        EOHReferenceSpace BestSpace = EOHReferenceSpace::WorldSpace;
        
        EOHReferenceSpace Spaces[] = {
            EOHReferenceSpace::WorldSpace,
            EOHReferenceSpace::LocalSpace,
            EOHReferenceSpace::ComponentSpace,
            EOHReferenceSpace::BoneSpace
        };
        
        for (EOHReferenceSpace Space : Spaces)
        {
            float Clarity = CalculateMotionQuality(Space);
            if (Clarity > BestClarity)
            {
                BestClarity = Clarity;
                BestSpace = Space;
            }
        }
        
        OptimalAnalysisSpace = BestSpace;
        return BestSpace;
    }

    // === STRIKE DETECTION ===

    /**
     * Update strike detection metrics
     */
    void UpdateStrikeMetrics(const FOHMotionFrameSample& NewSample)
    {
        float AccelMag = NewSample.WorldLinearAcceleration.Size();
        float JerkMag = NewSample.WorldLinearJerk.Size();
        
        // Update peaks
        if (AccelMag > PeakAcceleration)
        {
            PeakAcceleration = AccelMag;
            AccelerationBuildupTime = 0.0f;
        }
        else
        {
            AccelerationBuildupTime += NewSample.DeltaTime;
        }
        
        PeakJerk = FMath::Max(PeakJerk, JerkMag);
        
        // Multi-factor strike detection
        bool AccelThreshold = PeakAcceleration > 800.0f;
        bool JerkThreshold = PeakJerk > 5000.0f;
        bool TimeWindow = AccelerationBuildupTime < 0.2f;
        
        bLikelyStrike = AccelThreshold && (JerkThreshold || TimeWindow);
        
        // Calculate confidence
        if (bLikelyStrike)
        {
            float AccelScore = FMath::Clamp(PeakAcceleration / 2000.0f, 0.0f, 1.0f);
            float JerkScore = FMath::Clamp(PeakJerk / 10000.0f, 0.0f, 1.0f);
            float TimeScore = FMath::Clamp(1.0f - (AccelerationBuildupTime / 0.2f), 0.0f, 1.0f);
            
            StrikeConfidence = (AccelScore * 0.4f + JerkScore * 0.4f + TimeScore * 0.2f);
        }
        else
        {
            StrikeConfidence *= 0.9f; // Decay confidence
        }
    }

    /**
     * Validate strike with temporal analysis
     */
    bool ValidateStrike(float TimeWindow = 0.2f) const
    {
        if (!bLikelyStrike) return false;
        
        float CurrentTime = GetLatestSample() ? GetLatestSample()->TimeStamp : 0.0f;
        TArray<FOHMotionFrameSample> RecentSamples = GetSamplesInTimeRange(CurrentTime - TimeWindow, CurrentTime);
        
        if (RecentSamples.Num() < 3) return false;
        
        // Check direction consistency
        FVector AvgDirection = FVector::ZeroVector;
        for (const FOHMotionFrameSample& Sample : RecentSamples)
        {
            if (Sample.WorldLinearVelocity.Size() > 100.0f)
            {
                AvgDirection += Sample.WorldLinearVelocity.GetSafeNormal();
            }
        }
        AvgDirection.Normalize();
        
        float DirectionConsistency = 0.0f;
        for (const FOHMotionFrameSample& Sample : RecentSamples)
        {
            if (Sample.WorldLinearVelocity.Size() > 100.0f)
            {
                FVector Dir = Sample.WorldLinearVelocity.GetSafeNormal();
                DirectionConsistency += FVector::DotProduct(Dir, AvgDirection);
            }
        }
        DirectionConsistency /= RecentSamples.Num();
        
        return DirectionConsistency > 0.7f && StrikeConfidence > 0.6f;
    }

    // === ACCESSORS ===

    const FOHMotionFrameSample* GetLatestSample() const { return MotionHistory.GetLatest(0); }
    FOHMotionFrameSample* GetLatestMutable() { return MotionHistory.GetLatestMutable(0); }
    int32 GetHistoryDepth() const { return MotionHistory.NumFrames(); }
    
    FVector GetVelocity(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const
    {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->GetVelocity(Space) : FVector::ZeroVector;
    }
    
    FVector GetAcceleration(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const
    {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->GetAcceleration(Space) : FVector::ZeroVector;
    }
    
    FVector GetJerk(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const
    {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->GetJerk(Space) : FVector::ZeroVector;
    }
    
    FVector GetPosition() const
    {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->WorldPosition : FVector::ZeroVector;
    }
    
    void InvalidateCaches() const
    {
        bMotionQualityCacheValid = false;
    }
};

// ================================================================================
// LEVEL 3: FOHCombatChainData - Chain-Level Motion Aggregation
// ================================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHCombatChainData
{
    GENERATED_BODY()

    // === CHAIN IDENTITY ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Identity")
    FName ChainName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Identity")
    FName RootBone = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Identity")
    TArray<FName> ChainBones;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Identity")
    EOHBodyPart BodyPart = EOHBodyPart::None;

    // === BONE MOTION DATA ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Motion")
    TArray<FOHBoneMotionData> BoneMotionArray;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Motion")
    TMap<FName, int32> BoneIndexMap; // Maps bone name to array index

    // === CHAIN PROPERTIES ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Properties")
    float ChainLength = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Properties")
    float ChainTotalMass = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Properties")
    FVector ChainCenterOfMass = FVector::ZeroVector;

    // === CHAIN MOTION METRICS ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Metrics")
    FVector ChainMomentum = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Metrics")
    FVector ChainAngularMomentum = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Metrics")
    float ChainKineticEnergy = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Metrics")
    float MotionCoherence = 0.0f;

    // === ATTACK DETECTION ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Attack")
    bool bIsAttacking = false;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Attack")
    float AttackConfidence = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Attack")
    float TimeInAttack = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Attack")
    FVector AttackDirection = FVector::ZeroVector;

    // === MULTI-SPACE ANALYSIS ===
    UPROPERTY(BlueprintReadOnly, Category = "Chain|Space")
    EOHReferenceSpace OptimalAnalysisSpace = EOHReferenceSpace::WorldSpace;

    UPROPERTY(BlueprintReadOnly, Category = "Chain|Space")
    float CrossSpaceCoherence = 0.0f;

    // === INITIALIZATION ===

    void Initialize(FName InChainName, const TArray<FName>& InChainBones, EOHBodyPart InBodyPart)
    {
        ChainName = InChainName;
        ChainBones = InChainBones;
        BodyPart = InBodyPart;
        RootBone = ChainBones.Num() > 0 ? ChainBones[0] : NAME_None;
        
        // Initialize bone motion data array
        BoneMotionArray.Empty();
        BoneIndexMap.Empty();
        
        for (int32 i = 0; i < ChainBones.Num(); ++i)
        {
            FOHBoneMotionData BoneData;
            BoneData.Initialize(ChainBones[i], i > 0 ? ChainBones[i-1] : NAME_None);
            
            // Set up parent-child relationships
            if (i > 0)
            {
                BoneMotionArray[i-1].ChildBoneNames.Add(ChainBones[i]);
            }
            
            BoneMotionArray.Add(BoneData);
            BoneIndexMap.Add(ChainBones[i], i);
        }
        
        ResetMetrics();
    }

    // === SAMPLE MANAGEMENT ===

    /**
     * Update motion data for specific bone
     */
    void UpdateBoneMotion(FName BoneName, const FVector& WorldPos, const FQuat& WorldRot,
                          const FVector& ComponentVelocity, const FTransform& ComponentTransform,
                          const FVector& ReferenceVelocity, float TimeStamp)
    {
        if (int32* IndexPtr = BoneIndexMap.Find(BoneName))
        {
            FOHBoneMotionData& BoneData = BoneMotionArray[*IndexPtr];
            
            // Use parent bone as reference if available
            FTransform BoneRefTransform = FTransform::Identity;
            FName RefBoneName = NAME_None;
            
            if (BoneData.ParentBoneName != NAME_None)
            {
                if (int32* ParentIndexPtr = BoneIndexMap.Find(BoneData.ParentBoneName))
                {
                    const FOHBoneMotionData& ParentData = BoneMotionArray[*ParentIndexPtr];
                    if (const FOHMotionFrameSample* ParentSample = ParentData.GetLatestSample())
                    {
                        BoneRefTransform = FTransform(ParentSample->WorldRotation, ParentSample->WorldPosition);
                        RefBoneName = BoneData.ParentBoneName;
                    }
                }
            }
            
            BoneData.AddMotionSample(WorldPos, WorldRot, ComponentVelocity, ComponentTransform,
                                    BoneRefTransform, RefBoneName, ReferenceVelocity, TimeStamp);
        }
    }

    /**
     * Update all chain metrics from bone data
     */
    void UpdateChainMetrics(float CurrentTime)
    {
        ResetMetrics();
        
        int32 ValidBones = 0;
        
        // Aggregate bone metrics
        for (FOHBoneMotionData& BoneData : BoneMotionArray)
        {
            if (const FOHMotionFrameSample* Sample = BoneData.GetLatestSample())
            {
                if (Sample->bHasValidData)
                {
                    float BoneMass = EstimateBoneMass(BoneData.BoneName);
                    
                    // Update chain momentum
                    ChainMomentum += Sample->WorldLinearVelocity * BoneMass;
                    
                    // Update center of mass
                    ChainCenterOfMass += Sample->WorldPosition * BoneMass;
                    ChainTotalMass += BoneMass;
                    
                    // Update kinetic energy
                    ChainKineticEnergy += 0.5f * BoneMass * Sample->WorldLinearVelocity.SizeSquared();
                    
                    // Update angular momentum
                    FVector R = Sample->WorldPosition - ChainCenterOfMass;
                    ChainAngularMomentum += FVector::CrossProduct(R, Sample->WorldLinearVelocity * BoneMass);
                    
                    ValidBones++;
                }
            }
        }
        
        // Normalize center of mass
        if (ChainTotalMass > KINDA_SMALL_NUMBER)
        {
            ChainCenterOfMass /= ChainTotalMass;
        }
        
        // Calculate chain length
        UpdateChainLength();
        
        // Calculate motion coherence
        UpdateMotionCoherence();
        
        // Determine optimal analysis space
        DetermineOptimalSpace();
        
        // Update attack detection
        UpdateAttackDetection(CurrentTime);
    }

    // === CHAIN-TO-CHAIN COMPARISON ===

    /**
     * Compare this chain with another chain
     */
    float GetChainSimilarity(const FOHCombatChainData& OtherChain, EOHReferenceSpace Space) const
    {
        float TotalSimilarity = 0.0f;
        int32 ComparisonCount = 0;
        
        // Compare matching bones
        for (const FOHBoneMotionData& ThisBone : BoneMotionArray)
        {
            for (const FOHBoneMotionData& OtherBone : OtherChain.BoneMotionArray)
            {
                if (ThisBone.BoneName == OtherBone.BoneName)
                {
                    TotalSimilarity += ThisBone.GetMotionSimilarityTo(OtherBone, Space);
                    ComparisonCount++;
                    break;
                }
            }
        }
        
        return ComparisonCount > 0 ? (TotalSimilarity / ComparisonCount) : 0.0f;
    }

    /**
     * Calculate closing rate to another chain
     */
    float GetClosingRate(const FOHCombatChainData& OtherChain) const
    {
        FVector RelativeVelocity = GetChainVelocity() - OtherChain.GetChainVelocity();
        FVector ToTarget = (OtherChain.ChainCenterOfMass - ChainCenterOfMass).GetSafeNormal();
        return FVector::DotProduct(RelativeVelocity, ToTarget);
    }

    // === CHAIN-TO-BONE COMPARISON ===

    /**
     * Get closest bone in chain to external bone
     */
    const FOHBoneMotionData* GetClosestBoneTo(const FOHBoneMotionData& ExternalBone) const
    {
        const FOHBoneMotionData* ClosestBone = nullptr;
        float MinDistance = FLT_MAX;
        
        FVector ExternalPos = ExternalBone.GetPosition();
        
        for (const FOHBoneMotionData& ChainBone : BoneMotionArray)
        {
            float Distance = FVector::Dist(ChainBone.GetPosition(), ExternalPos);
            if (Distance < MinDistance)
            {
                MinDistance = Distance;
                ClosestBone = &ChainBone;
            }
        }
        
        return ClosestBone;
    }

    /**
     * Check if chain is approaching a specific bone
     */
    bool IsApproachingBone(const FOHBoneMotionData& TargetBone, float& OutTimeToImpact) const
    {
        const FOHBoneMotionData* ClosestChainBone = GetClosestBoneTo(TargetBone);
        if (!ClosestChainBone) return false;
        
        FVector RelativeVel = ClosestChainBone->GetVelocity() - TargetBone.GetVelocity();
        FVector ToTarget = (TargetBone.GetPosition() - ClosestChainBone->GetPosition());
        
        float ClosingSpeed = FVector::DotProduct(RelativeVel, ToTarget.GetSafeNormal());
        
        if (ClosingSpeed > 0.0f)
        {
            OutTimeToImpact = ToTarget.Size() / ClosingSpeed;
            return true;
        }
        
        OutTimeToImpact = -1.0f;
        return false;
    }

    // === TEMPORAL ANALYSIS ===

    /**
     * Get chain state at specific time through interpolation
     */
    void GetChainStateAtTime(float TargetTime, TArray<FOHMotionFrameSample>& OutSamples) const
    {
        OutSamples.Empty();
        
        for (const FOHBoneMotionData& BoneData : BoneMotionArray)
        {
            FOHMotionFrameSample InterpolatedSample = BoneData.GetInterpolatedSampleAtTime(TargetTime);
            OutSamples.Add(InterpolatedSample);
        }
    }

    // === ACCESSORS ===

    FOHBoneMotionData* GetBoneMotionData(FName BoneName)
    {
        if (int32* IndexPtr = BoneIndexMap.Find(BoneName))
        {
            return &BoneMotionArray[*IndexPtr];
        }
        return nullptr;
    }

    const FOHBoneMotionData* GetBoneMotionData(FName BoneName) const
    {
        if (const int32* IndexPtr = BoneIndexMap.Find(BoneName))
        {
            return &BoneMotionArray[*IndexPtr];
        }
        return nullptr;
    }

    FVector GetChainVelocity() const
    {
        return ChainTotalMass > KINDA_SMALL_NUMBER ? (ChainMomentum / ChainTotalMass) : FVector::ZeroVector;
    }

    bool HasValidMotionData() const
    {
        for (const FOHBoneMotionData& BoneData : BoneMotionArray)
        {
            if (BoneData.GetHistoryDepth() > 0)
            {
                return true;
            }
        }
        return false;
    }

private:
    // === INTERNAL CALCULATIONS ===

    void ResetMetrics()
    {
        ChainMomentum = FVector::ZeroVector;
        ChainAngularMomentum = FVector::ZeroVector;
        ChainCenterOfMass = FVector::ZeroVector;
        ChainTotalMass = 0.0f;
        ChainKineticEnergy = 0.0f;
        MotionCoherence = 0.0f;
    }

    void UpdateChainLength()
    {
        ChainLength = 0.0f;
        
        for (int32 i = 0; i < ChainBones.Num() - 1; ++i)
        {
            const FOHBoneMotionData* Current = GetBoneMotionData(ChainBones[i]);
            const FOHBoneMotionData* Next = GetBoneMotionData(ChainBones[i + 1]);
            
            if (Current && Next)
            {
                FVector Pos1 = Current->GetPosition();
                FVector Pos2 = Next->GetPosition();
                ChainLength += FVector::Dist(Pos1, Pos2);
            }
        }
    }

    void UpdateMotionCoherence()
    {
        if (BoneMotionArray.Num() < 2)
        {
            MotionCoherence = 1.0f;
            return;
        }
        
        float TotalCoherence = 0.0f;
        int32 ComparisonCount = 0;
        
        // Compare adjacent bones
        for (int32 i = 0; i < BoneMotionArray.Num() - 1; ++i)
        {
            const FOHBoneMotionData& Current = BoneMotionArray[i];
            const FOHBoneMotionData& Next = BoneMotionArray[i + 1];
            
            float Similarity = Current.GetMotionSimilarityTo(Next, OptimalAnalysisSpace);
            TotalCoherence += Similarity;
            ComparisonCount++;
        }
        
        MotionCoherence = ComparisonCount > 0 ? (TotalCoherence / ComparisonCount) : 0.0f;
    }

    void DetermineOptimalSpace()
    {
        TMap<EOHReferenceSpace, float> SpaceScores;
        
        EOHReferenceSpace Spaces[] = {
            EOHReferenceSpace::WorldSpace,
            EOHReferenceSpace::LocalSpace,
            EOHReferenceSpace::ComponentSpace,
            EOHReferenceSpace::BoneSpace
        };
        
        for (EOHReferenceSpace Space : Spaces)
        {
            float TotalQuality = 0.0f;
            
            for (const FOHBoneMotionData& BoneData : BoneMotionArray)
            {
                TotalQuality += BoneData.CalculateMotionQuality(Space);
            }
            
            SpaceScores.Add(Space, TotalQuality);
        }
        
        // Find best space
        float BestScore = 0.0f;
        for (const auto& Pair : SpaceScores)
        {
            if (Pair.Value > BestScore)
            {
                BestScore = Pair.Value;
                OptimalAnalysisSpace = Pair.Key;
            }
        }
        
        // Calculate cross-space coherence
        float TotalCoherence = 0.0f;
        int32 Count = 0;
        
        for (const auto& Pair1 : SpaceScores)
        {
            for (const auto& Pair2 : SpaceScores)
            {
                if (Pair1.Key != Pair2.Key)
                {
                    float Diff = FMath::Abs(Pair1.Value - Pair2.Value);
                    TotalCoherence += 1.0f - FMath::Clamp(Diff, 0.0f, 1.0f);
                    Count++;
                }
            }
        }
        
        CrossSpaceCoherence = Count > 0 ? (TotalCoherence / Count) : 0.0f;
    }

    void UpdateAttackDetection(float CurrentTime)
    {
        // Check if any bones are striking
        int32 StrikingBones = 0;
        float TotalStrikeConfidence = 0.0f;
        
        for (const FOHBoneMotionData& BoneData : BoneMotionArray)
        {
            if (BoneData.bLikelyStrike)
            {
                StrikingBones++;
                TotalStrikeConfidence += BoneData.StrikeConfidence;
            }
        }
        
        // Chain is attacking if multiple bones show strike pattern
        float ChainStrikeRatio = static_cast<float>(StrikingBones) / BoneMotionArray.Num();
        
        if (ChainStrikeRatio > 0.3f && TotalStrikeConfidence > 0.5f)
        {
            if (!bIsAttacking)
            {
                bIsAttacking = true;
                TimeInAttack = 0.0f;
                
                // Calculate attack direction from chain momentum
                AttackDirection = ChainMomentum.GetSafeNormal();
            }
            
            TimeInAttack += 0.016f; // Assuming 60fps
            AttackConfidence = FMath::Min(TotalStrikeConfidence / StrikingBones, 1.0f);
        }
        else
        {
            if (bIsAttacking && TimeInAttack > 0.5f)
            {
                // Attack ended
                bIsAttacking = false;
                AttackConfidence *= 0.9f; // Decay
            }
        }
    }

    static float EstimateBoneMass(FName BoneName)
    {
        FString BoneString = BoneName.ToString().ToLower();
        
        if (BoneString.Contains(TEXT("hand")) || BoneString.Contains(TEXT("foot")))
            return 2.0f;
        else if (BoneString.Contains(TEXT("head")))
            return 5.0f;
        else if (BoneString.Contains(TEXT("spine")) || BoneString.Contains(TEXT("pelvis")))
            return 15.0f;
        else if (BoneString.Contains(TEXT("thigh")) || BoneString.Contains(TEXT("upperarm")))
            return 8.0f;
        else if (BoneString.Contains(TEXT("calf")) || BoneString.Contains(TEXT("lowerarm")))
            return 4.0f;
        else
            return 3.0f;
    }
};

#endif

// DECLARE_LOG_CATEGORY_EXTERN(LogPACManager, Log, All);
USTRUCT(BlueprintType)
struct FAttackResult : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_MomentumPoints::Enm_MomentumPoints> AddPointsMovementum;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_MomentumPoints::Enm_MomentumPoints> ReducePointsMovementum;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<TEnumAsByte<Enm_AttackSideBody::Enm_AttackSideBody>> Side_Need_Defense;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_HitBodyParts::Enm_HitBodyParts> Hit_Body_Part;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Damage;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float BlockDamage;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ShakeCameraHit;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OnHit;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OnBlock;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_AnimationRef Hit_AnmRef;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_AnimationRef Death_AnmRef;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_HitStun HitStun;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_HitSFX HitEffects;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_Parry Parry_System;
};

// ==================== Physics Coefficients from Real Data ====================

// Material properties database
USTRUCT(BlueprintType)
struct FMaterialPhysicsProperties {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Density = 1000.0f; // kg/m³

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float YoungsModulus = 1e6f; // Pa (N/m²)

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float PoissonsRatio = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StaticFriction = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float KineticFriction = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Restitution = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float DragCoefficient = 0.47f; // Sphere default
};

USTRUCT(BlueprintType)
struct FPhysicsImpactData {
    GENERATED_BODY()

    UPROPERTY()
    FVector ImpactLocation;

    UPROPERTY()
    FVector ImpactNormal;

    UPROPERTY()
    FVector RelativeVelocity;

    UPROPERTY()
    float ImpactMagnitude = 0.0f;

    UPROPERTY()
    FName BoneName;

    UPROPERTY()
    float Timestamp = 0.0f;

    UPROPERTY()
    AActor* OtherActor = nullptr;
};

USTRUCT(BlueprintType)
struct FSimpleConstraintProfile {
    GENERATED_BODY()

    // --------- Angular (Cone) Limit -----------
    UPROPERTY(BlueprintReadOnly) EOHAngularConstraintMotion Swing1Motion = EOHAngularConstraintMotion::ACM_Limited;
    UPROPERTY(BlueprintReadOnly) EOHAngularConstraintMotion Swing2Motion = EOHAngularConstraintMotion::ACM_Limited;
    UPROPERTY(BlueprintReadOnly) float Swing1LimitDegrees = 0.f;
    UPROPERTY(BlueprintReadOnly) float Swing2LimitDegrees = 0.f;
    UPROPERTY(BlueprintReadOnly) float SwingStiffness = 0.f;
    UPROPERTY(BlueprintReadOnly) float SwingDamping = 0.f;
    UPROPERTY(BlueprintReadOnly) bool bSwingSoftConstraint = false;
    UPROPERTY(BlueprintReadOnly) float SwingRestitution = 0.f;
    UPROPERTY(BlueprintReadOnly) float SwingContactDistance = 0.f;

    // --------- Angular (Twist) Limit ----------
    UPROPERTY(BlueprintReadOnly) EOHAngularConstraintMotion TwistMotion = EOHAngularConstraintMotion::ACM_Limited;
    UPROPERTY(BlueprintReadOnly) float TwistLimitDegrees = 0.f;
    UPROPERTY(BlueprintReadOnly) float TwistStiffness = 0.f;
    UPROPERTY(BlueprintReadOnly) float TwistDamping = 0.f;
    UPROPERTY(BlueprintReadOnly) bool bTwistSoftConstraint = false;
    UPROPERTY(BlueprintReadOnly) float TwistRestitution = 0.f;
    UPROPERTY(BlueprintReadOnly) float TwistContactDistance = 0.f;

    // --------- Linear Limit ------------------
    UPROPERTY(BlueprintReadOnly) EOHLinearConstraintMotion LinearXMotion = EOHLinearConstraintMotion::LCM_Locked;
    UPROPERTY(BlueprintReadOnly) EOHLinearConstraintMotion LinearYMotion = EOHLinearConstraintMotion::LCM_Locked;
    UPROPERTY(BlueprintReadOnly) EOHLinearConstraintMotion LinearZMotion = EOHLinearConstraintMotion::LCM_Locked;
    UPROPERTY(BlueprintReadOnly) float LinearLimit = 0.f;
    UPROPERTY(BlueprintReadOnly) float LinearStiffness = 0.f;
    UPROPERTY(BlueprintReadOnly) float LinearDamping = 0.f;
    UPROPERTY(BlueprintReadOnly) bool bLinearSoftConstraint = false;
    UPROPERTY(BlueprintReadOnly) float LinearRestitution = 0.f;
    UPROPERTY(BlueprintReadOnly) float LinearContactDistance = 0.f;

    // --------- Projection --------------------
    UPROPERTY(BlueprintReadOnly) bool bEnableProjection = false;
    UPROPERTY(BlueprintReadOnly) float ProjectionLinearTolerance = 1.f;
    UPROPERTY(BlueprintReadOnly) float ProjectionAngularTolerance = 10.f;

    // --------- Collision ---------------------
    UPROPERTY(BlueprintReadOnly) bool bDisableCollision = false;

    // --------- Breakable Constraints ---------
    UPROPERTY(BlueprintReadOnly) bool bLinearBreakable = false;
    UPROPERTY(BlueprintReadOnly) float LinearBreakThreshold = 300.f;

    UPROPERTY(BlueprintReadOnly) bool bAngularBreakable = false;
    UPROPERTY(BlueprintReadOnly) float AngularBreakThreshold = 300.f;

    // --------- Drive Settings (for completeness, but rarely set at profile level) ----
    // Not included in FConstraintProfileProperties by default, but you could add if needed.
};

USTRUCT(BlueprintType)
struct FSimpleConstraintDriveParams {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly) bool bAngularOrientationDrive = false;
    UPROPERTY(BlueprintReadOnly) bool bEnableSwingDrive = false;
    UPROPERTY(BlueprintReadOnly) bool bEnableTwistDrive = false;

    UPROPERTY(BlueprintReadOnly) bool bLinearPositionDrive = false;
    UPROPERTY(BlueprintReadOnly) bool bLinearVelocityDrive = false;
    UPROPERTY(BlueprintReadOnly) bool bAngularVelocityDrive = false;

    UPROPERTY(BlueprintReadOnly) FVector LinearPositionTarget = FVector::ZeroVector;
    UPROPERTY(BlueprintReadOnly) FVector LinearVelocityTarget = FVector::ZeroVector;
    UPROPERTY(BlueprintReadOnly) FQuat AngularOrientationTarget = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly) FVector AngularVelocityTarget = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly) float LinearDriveForceLimit = 0.f;
    UPROPERTY(BlueprintReadOnly) float LinearDriveStiffness = 0.f;
    UPROPERTY(BlueprintReadOnly) float LinearDriveDamping = 0.f;

    UPROPERTY(BlueprintReadOnly) float AngularDriveStiffness = 0.f;
    UPROPERTY(BlueprintReadOnly) float AngularDriveDamping = 0.f;
    UPROPERTY(BlueprintReadOnly) float AngularDriveForceLimit = 0.f;

    UPROPERTY(BlueprintReadOnly) EOHAngularDriveMode AngularDriveMode = EOHAngularDriveMode::SLERP;
};

USTRUCT(BlueprintType)
struct FSimpleConstraintInstanceState {
    GENERATED_BODY()

    // --- General State ---
    UPROPERTY(BlueprintReadOnly) bool bConstraintEnabled = true;
    UPROPERTY(BlueprintReadOnly) bool bDisableCollision = false;

    // --- World/Local Space Reference Frames ---
    UPROPERTY(BlueprintReadOnly) FVector ConstraintPos1 = FVector::ZeroVector; // Local position in Body1 (child)
    UPROPERTY(BlueprintReadOnly) FVector ConstraintPos2 = FVector::ZeroVector; // Local position in Body2 (parent)
    UPROPERTY(BlueprintReadOnly) FQuat ConstraintQuat1 = FQuat::Identity;      // Local orientation in Body1 (child)
    UPROPERTY(BlueprintReadOnly) FQuat ConstraintQuat2 = FQuat::Identity;      // Local orientation in Body2 (parent)

    // --- Runtime Offset ---
    UPROPERTY(BlueprintReadOnly) FRotator AngularRotationOffset = FRotator::ZeroRotator; // Offset (degrees)

    // --- Projection (Solver Correction) ---
    UPROPERTY(BlueprintReadOnly) bool bEnableProjection = false;
    UPROPERTY(BlueprintReadOnly) float ProjectionLinearTolerance = 1.f;
    UPROPERTY(BlueprintReadOnly) float ProjectionAngularTolerance = 10.f;

    // --- Breakable State ---
    UPROPERTY(BlueprintReadOnly) bool bLinearBreakable = false;
    UPROPERTY(BlueprintReadOnly) float LinearBreakThreshold = 300.f;
    UPROPERTY(BlueprintReadOnly) bool bAngularBreakable = false;
    UPROPERTY(BlueprintReadOnly) float AngularBreakThreshold = 300.f;

    // --- Runtime Status (Live) ---
    UPROPERTY(BlueprintReadOnly) bool bIsBroken = false;    // If constraint has broken this frame (requires logic)
    UPROPERTY(BlueprintReadOnly) float CurrentTwist = 0.f;  // Current twist angle (degrees)
    UPROPERTY(BlueprintReadOnly) float CurrentSwing1 = 0.f; // Current swing1 angle (degrees)
    UPROPERTY(BlueprintReadOnly) float CurrentSwing2 = 0.f; // Current swing2 angle (degrees)

    // --- Solver Iterations (Advanced, may be unsupported) ---
    UPROPERTY(BlueprintReadOnly) int32 LinearBreakSolverIterationCount = 0;
    UPROPERTY(BlueprintReadOnly) int32 AngularBreakSolverIterationCount = 0;

    // --- Constraint Names/IDs ---
    UPROPERTY(BlueprintReadOnly) FName ConstraintBone1 = NAME_None; // Child bone
    UPROPERTY(BlueprintReadOnly) FName ConstraintBone2 = NAME_None; // Parent bone
    UPROPERTY(BlueprintReadOnly) FName ConstraintName = NAME_None;

    // --- World Space Anchors (for debugging) ---
    UPROPERTY(BlueprintReadOnly) FVector RefFrame1_World = FVector::ZeroVector;
    UPROPERTY(BlueprintReadOnly) FVector RefFrame2_World = FVector::ZeroVector;
};

// --- Begin runtime support structs for constraint sampling and readback ---
USTRUCT(BlueprintType)
struct FConstraintRuntimeReadback {
    GENERATED_BODY()

  private:
    /** Last known orientation drive strength (spring) */
    UPROPERTY(Transient)
    float OrientationStrength = 0.f;

    /** Last known angular velocity drive strength (damping) */
    UPROPERTY(Transient)
    float AngularVelocityStrength = 0.f;

    /** Last known position drive strength */
    UPROPERTY(Transient)
    float PositionStrength = 0.f;

    /** Last known velocity drive strength */
    UPROPERTY(Transient)
    float VelocityStrength = 0.f;

    /** Last known local-simulation flag */
    UPROPERTY(Transient)
    bool bLocalSimulation = false;

  public:
    FORCEINLINE void SetOrientationStrength(float InStrength) {
        OrientationStrength = InStrength;
    }
    FORCEINLINE float GetOrientationStrength() const {
        return OrientationStrength;
    }

    FORCEINLINE void SetAngularVelocityStrength(float InStrength) {
        AngularVelocityStrength = InStrength;
    }
    FORCEINLINE float GetAngularVelocityStrength() const {
        return AngularVelocityStrength;
    }

    FORCEINLINE void SetPositionStrength(float InStrength) {
        PositionStrength = InStrength;
    }
    FORCEINLINE float GetPositionStrength() const {
        return PositionStrength;
    }

    FORCEINLINE void SetVelocityStrength(float InStrength) {
        VelocityStrength = InStrength;
    }
    FORCEINLINE float GetVelocityStrength() const {
        return VelocityStrength;
    }

    FORCEINLINE void SetLocalSimulation(bool bInLocalSimulation) {
        bLocalSimulation = bInLocalSimulation;
    }
    FORCEINLINE bool GetLocalSimulation() const {
        return bLocalSimulation;
    }

    FORCEINLINE void SetAll(float InOrientationStrength, float InAngularVelocityStrength, float InPositionStrength,
                            float InVelocityStrength, bool InbLocalSimulation) {
        SetOrientationStrength(InOrientationStrength);
        SetAngularVelocityStrength(InAngularVelocityStrength);
        SetPositionStrength(InPositionStrength);
        SetVelocityStrength(InVelocityStrength);
        SetLocalSimulation(InbLocalSimulation);
    }

    FORCEINLINE void SetAll(const FConstraintRuntimeReadback& InReadback) {
        SetOrientationStrength(InReadback.GetOrientationStrength());
        SetAngularVelocityStrength(InReadback.GetAngularVelocityStrength());
        SetPositionStrength(InReadback.GetPositionStrength());
        SetVelocityStrength(InReadback.GetVelocityStrength());
        SetLocalSimulation(InReadback.GetLocalSimulation());
    }
};

USTRUCT(BlueprintType)
struct FConstraintRuntimeState {
    GENERATED_BODY()

  private:
    /** Pointer to the live constraint instance in the physics scene */
    FConstraintInstance* ConstraintInstance = nullptr;

    /** Current sampled strain (force magnitude  torque magnitude) */
    UPROPERTY(Transient)
    float CurrentStrain = 0.f;

    /** Previous frame’s strain, for jitter calculation */
    float PrevStrain = 0.f;

    /** Smoothed jitter metric based on strain delta */
    UPROPERTY(Transient)
    float JitterMetric = 0.f;

    /** Last readback of drive parameters for exponential smoothing */
    UPROPERTY(Transient)
    FConstraintRuntimeReadback LastReadback;

  public:
    FORCEINLINE void SetConstraintInstance(FConstraintInstance* InInstance) {
        ConstraintInstance = InInstance;
    }
    FORCEINLINE FConstraintInstance* GetConstraintInstance() const {
        return ConstraintInstance;
    }

    FORCEINLINE void SetStrain(float InStrain) {
        CurrentStrain = InStrain;
    }
    FORCEINLINE float GetStrain() const {
        return CurrentStrain;
    }

    FORCEINLINE void SetPrevStrain(float InStrain) {
        PrevStrain = InStrain;
    }
    FORCEINLINE float GetPrevStrain() const {
        return PrevStrain;
    }

    FORCEINLINE void SetJitterMetric(float InMetric) {
        JitterMetric = InMetric;
    }
    FORCEINLINE float GetJitterMetric() const {
        return JitterMetric;
    }

    FORCEINLINE void SetLastReadback(const FConstraintRuntimeReadback& InReadback) {
        LastReadback = InReadback;
    }
    FORCEINLINE const FConstraintRuntimeReadback& GetLastReadback() const {
        return LastReadback;
    }
};
// --- End runtime support structs ---

#pragma region Physics_Structs
// Forward declarations
#pragma region Macros
// Debug-safe macro for setting and validating BoneState keys
#if !UE_BUILD_SHIPPING
#define ENSURE_BONESTATE_KEY(StateRef, BoneEnum)                                                                       \
    StateRef.SetBoneEnum(BoneEnum);                                                                                    \
    StateRef.ValidateKeyMatch(BoneEnum)
#else
#define ENSURE_BONESTATE_KEY(StateRef, BoneEnum) StateRef.SetBoneEnum(BoneEnum)
#endif
#define GET_OH_REF_SAFE(Map, BoneEnum) (Map.HasValidReference(BoneEnum) ? Map.GetReference(BoneEnum) : nullptr)
#define GET_OH_INDEX_SAFE(Map, BoneEnum)                                                                               \
    (Map.HasValidReference(BoneEnum) ? Map.GetPoseIndex(BoneEnum) : FCompactPoseBoneIndex(INDEX_NONE))

#pragma endregion

#pragma region PhysicsGraph_Structs

#pragma region MotionSample

USTRUCT(BlueprintType)
struct FOHMotionSample {
    GENERATED_BODY()

  private:
    UPROPERTY()
    FTransform Transform = FTransform::Identity;

    UPROPERTY()
    float TimeStamp = 0.f;

    UPROPERTY()
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector LinearAcceleration = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularAcceleration = FVector::ZeroVector;

  public:
    // Default constructor (made public and safe)
    FOHMotionSample() = default;

    // Full constructor
    FOHMotionSample(const FTransform& InTransform, float InTimeStamp, const FVector& InLinearVelocity,
                    const FVector& InAngularVelocity, const FVector& InLinearAcceleration,
                    const FVector& InAngularAcceleration)
        : Transform(InTransform), TimeStamp(InTimeStamp), LinearVelocity(InLinearVelocity),
          AngularVelocity(InAngularVelocity), LinearAcceleration(InLinearAcceleration),
          AngularAcceleration(InAngularAcceleration) {}

    // -- Static Factory --
    static FOHMotionSample CreateFromState(const FTransform& Transform, const FVector& LinearVel,
                                           const FVector& AngularVel, const FVector& LinAccel, const FVector& AngAccel,
                                           float TimeStamp) {
        FOHMotionSample Sample;
        Sample.SetTransform(Transform);
        Sample.SetLinearVelocity(LinearVel);
        Sample.SetAngularVelocity(AngularVel);
        Sample.SetLinearAcceleration(LinAccel);
        Sample.SetAngularAcceleration(AngAccel);
        Sample.SetTimeStamp(TimeStamp);
        return Sample;
    }

    // Getters  (inline)
    FORCEINLINE FVector GetLocation() const {
        return Transform.GetLocation();
    }
    FORCEINLINE FQuat GetRotation() const {
        return Transform.GetRotation();
    }
    FORCEINLINE const FTransform& GetTransform() const {
        return Transform;
    }
    FORCEINLINE float GetTimeStamp() const {
        return TimeStamp;
    }
    FORCEINLINE const FVector& GetLinearVelocity() const {
        return LinearVelocity;
    }
    FORCEINLINE const FVector& GetAngularVelocity() const {
        return AngularVelocity;
    }
    FORCEINLINE const FVector& GetLinearAcceleration() const {
        return LinearAcceleration;
    }
    FORCEINLINE const FVector& GetAngularAcceleration() const {
        return AngularAcceleration;
    }
    FORCEINLINE float GetLinearAccelMagnitude() const {
        return LinearAcceleration.Size();
    }
    FORCEINLINE float GetAngularAccelMagnitude() const {
        return AngularAcceleration.Size();
    }

    // Setterss (InLine)
    FORCEINLINE void SetLocation(const FVector& InLocation) {
        Transform.SetLocation(InLocation);
    }
    FORCEINLINE void SetRotation(const FQuat& InRotation) {
        Transform.SetRotation(InRotation);
    }
    FORCEINLINE void SetTransform(const FTransform& T) {
        Transform = T;
    }
    FORCEINLINE void SetTimeStamp(float Time) {
        TimeStamp = Time;
    }
    FORCEINLINE void SetLinearVelocity(const FVector& V) {
        LinearVelocity = V;
    }
    FORCEINLINE void SetAngularVelocity(const FVector& V) {
        AngularVelocity = V;
    }
    FORCEINLINE void SetLinearAcceleration(const FVector& A) {
        LinearAcceleration = A;
    }
    FORCEINLINE void SetAngularAcceleration(const FVector& A) {
        AngularAcceleration = A;
    }

    // Utility
    FORCEINLINE bool HasNaNs() const {
        return Transform.ContainsNaN() || LinearVelocity.ContainsNaN() || AngularVelocity.ContainsNaN() ||
               LinearAcceleration.ContainsNaN() || AngularAcceleration.ContainsNaN();
    }

    FORCEINLINE bool IsValidSample() const {
        return Transform.IsValid() && TimeStamp >= 0.f && !HasNaNs();
    }
    FORCEINLINE float GetLinearSpeed() const {
        return FMath::Clamp(LinearVelocity.Size(), 0.f, 100000.f); // Safety clamp
    }

    FORCEINLINE float GetAngularSpeed() const {
        return FMath::Clamp(AngularVelocity.Size(), 0.f, 100000.f); // Safety clamp
    }

    FORCEINLINE void ClampValues(float MaxSpeed = 5000.f, float MaxAccel = 10000.f) {
        LinearVelocity = LinearVelocity.GetClampedToMaxSize(MaxSpeed);
        AngularVelocity = AngularVelocity.GetClampedToMaxSize(MaxSpeed);
        LinearAcceleration = LinearAcceleration.GetClampedToMaxSize(MaxAccel);
        AngularAcceleration = AngularAcceleration.GetClampedToMaxSize(MaxAccel);
    }
};
#pragma endregion

#pragma region ConstraintInstanceData

USTRUCT(BlueprintType)
struct FOHConstraintInstanceData {
    GENERATED_BODY()

  private:
    // === Core Identity Properties ===
    /** Name identifier (can be the PhysicsAsset constraint name) */
    UPROPERTY()
    FName ConstraintName;

    /** Bone that owns Body1 (usually the parent bone in hierarchy) */
    UPROPERTY()
    FName ParentBone;

    /** Bone that owns Body2 (the child bone) */
    UPROPERTY()
    FName ChildBone;

    // === Angular Limit Properties ===
    UPROPERTY()
    float Swing1LimitDegrees = 0.f;

    UPROPERTY()
    float Swing2LimitDegrees = 0.f;

    UPROPERTY()
    float TwistLimitDegrees = 0.f;

    // === Angular ConeLimitStiffness/Damping Properties ===
    UPROPERTY()
    float ConeLimitStiffness = 0.f;

    UPROPERTY()
    float ConeLimitDamping = 0.f;

    // === Linear Drive Properties ===
    UPROPERTY()
    float LinearDriveStiffness = 0.f;

    UPROPERTY()
    float LinearDriveDamping = 0.f;

    // === Angular Drive Properties ===
    UPROPERTY()
    float AngularSwingStiffness = 0.f;

    UPROPERTY()
    float AngularSwingDamping = 0.f;

    UPROPERTY()
    float AngularTwistStiffness = 0.f;

    UPROPERTY()
    float AngularTwistDamping = 0.f;

    // === Drive Enable Flags ===
    UPROPERTY()
    bool bPositionDriveEnabled = false;

    UPROPERTY()
    bool bVelocityDriveEnabled = false;

    UPROPERTY()
    bool bOrientationDriveEnabled = false;

    UPROPERTY()
    bool bAngularVelocityDriveEnabled = false;
    // === Cached Targets ===
    UPROPERTY()
    FVector CachedPositionTarget = FVector::ZeroVector;

    UPROPERTY()
    FVector CachedVelocityTarget = FVector::ZeroVector;

    // === Orientation & Angular‐Velocity Targets (for strain calcs) ===
    /** Desired bone orientation when kinematic (world‐space quaternion) */
    UPROPERTY(Transient)
    FQuat CachedOrientationTarget = FQuat::Identity;

    /** Desired angular velocity when kinematic (degrees/sec) */
    UPROPERTY(Transient)
    FVector CachedAngularVelocityTarget = FVector::ZeroVector;
    // === References & Templates ===
    UPROPERTY(Transient)
    USkeletalMeshComponent* CachedOwnerComponent = nullptr;

    UPROPERTY()
    UPhysicsConstraintTemplate* ConstraintTemplate = nullptr;

    // === Runtime Sampling & Readback State ===
    /** Holds the live pointer, strain/jitter metrics, and last-readback drives */
    UPROPERTY(Transient)
    FConstraintRuntimeState RuntimeState;
    // === Constraint State ===

  public:
    FORCEINLINE bool IsValidConstraintInstance() const {
        return RuntimeState.GetConstraintInstance() != nullptr;
    }

#pragma region ACCESSORS

#pragma region Identity

    // =====================================================================
    // === IDENTITY & CORE ACCESSORS ===
    // =====================================================================

    // --- Constraint Name Getters ---
    FORCEINLINE FName GetIdentifier() const {
        return ConstraintName;
    }
    FORCEINLINE FName GetConstraintName() const {
        return ConstraintName;
    }
    FORCEINLINE FName GetParentBone() const {
        return ParentBone;
    }
    FORCEINLINE FName GetChildBone() const {
        return ChildBone;
    }

    // --- Constraint Name Setters ---
    FORCEINLINE void SetConstraintName(FName Name) {
        ConstraintName = Name;
    }
    FORCEINLINE void SetParentBone(FName Bone) {
        ParentBone = Bone;
    }
    FORCEINLINE void SetChildBone(FName Bone) {
        ChildBone = Bone;
    }

    // --- Runtime State Accessors ---
    /** Mutable access to the runtime‐sampling struct */
    FORCEINLINE FConstraintRuntimeState& GetRuntimeState() {
        return RuntimeState;
    }
    /** Const access to the runtime‐sampling struct */
    FORCEINLINE const FConstraintRuntimeState& GetRuntimeState() const {
        return RuntimeState;
    }
    FORCEINLINE void SetRuntimeState(const FConstraintRuntimeState& InState) {
        RuntimeState = InState;
    }

    FORCEINLINE FConstraintInstance* GetConstraintInstance() const {
        return RuntimeState.GetConstraintInstance();
    }
    FORCEINLINE void SetConstraintInstance(FConstraintInstance* Instance) {
        RuntimeState.SetConstraintInstance(Instance);
    }

    // --- Cached Owner Component Accessors ---
    FORCEINLINE USkeletalMeshComponent* GetOwnerComponent() const {
        return CachedOwnerComponent;
    }
    FORCEINLINE void SetOwnerComponent(USkeletalMeshComponent* Comp) {
        CachedOwnerComponent = Comp;
    }

    // --- Constraint Template Accessors ---
    FORCEINLINE UPhysicsConstraintTemplate* GetConstraintTemplate() const {
        return ConstraintTemplate;
    }
    FORCEINLINE void SetConstraintTemplate(UPhysicsConstraintTemplate* Template) {
        ConstraintTemplate = Template;
    }

#pragma endregion
#pragma region Angular Limits

    // =====================================================================
    // === ANGULAR LIMITS & SETTINGS ===
    // =====================================================================
    // --- Angular Limit Getters
    FORCEINLINE float GetSwing1LimitDegrees() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.ConeLimit.Swing1LimitDegrees
                                       : Swing1LimitDegrees;
    }
    FORCEINLINE float GetSwing2LimitDegrees() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.ConeLimit.Swing2LimitDegrees
                                       : Swing2LimitDegrees;
    }
    FORCEINLINE float GetTwistLimitDegrees() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.TwistLimit.TwistLimitDegrees
                                       : TwistLimitDegrees;
    }

    // --- Angular Limit Setters
    FORCEINLINE void SetSwing1LimitDegrees(float Val) {
        Swing1LimitDegrees = Val;
    }
    FORCEINLINE void SetSwing2LimitDegrees(float Val) {
        Swing2LimitDegrees = Val;
    }
    FORCEINLINE void SetTwistLimitDegrees(float Val) {
        TwistLimitDegrees = Val;
    }

#pragma endregion

#pragma region Cone Limit Drive
    // =====================================================================
    // === Cone Limit Drive Properties ===
    // =====================================================================

    // --- Cone Limit Drive Getters
    FORCEINLINE float GetConeLimitDamping(bool bUseLive = true) const {
        return (bUseLive && GetConstraintInstance()) ? GetConstraintInstance()->ProfileInstance.ConeLimit.Damping
                                                     : ConeLimitDamping;
    }

    FORCEINLINE float GetConeLimitStiffness(bool bUseLive = true) const {
        return (bUseLive && GetConstraintInstance()) ? GetConstraintInstance()->ProfileInstance.ConeLimit.Stiffness
                                                     : ConeLimitStiffness;
    }

    // --- Cone Limit Drive Setters
    FORCEINLINE void SetConeLimitStiffness(float InVal, bool bWriteThrough = true) {
        ConeLimitStiffness = InVal;
        if (bWriteThrough && GetConstraintInstance()) {
            GetConstraintInstance()->ProfileInstance.ConeLimit.Stiffness = InVal;
        }
    }

    FORCEINLINE void SetConeLimitDamping(float InVal, bool bWriteThrough = true) {
        ConeLimitDamping = InVal;
        if (bWriteThrough && GetConstraintInstance()) {
            GetConstraintInstance()->ProfileInstance.ConeLimit.Damping = InVal;
        }
    }

#pragma endregion

#pragma region Linear Drive
    // =====================================================================
    // === LINEAR DRIVE PROPERTIES ===
    // ====================================================================

    // --- Linear Drive Getters
    FORCEINLINE float GetLinearDriveStiffness() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.LinearDrive.XDrive.Stiffness
                                       : LinearDriveStiffness;
    }
    FORCEINLINE float GetLinearDriveDamping() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.LinearDrive.XDrive.Damping
                                       : LinearDriveDamping;
    }

    // --- Linear Drive Setters
    FORCEINLINE void SetLinearDriveStiffness(float Val) {
        LinearDriveStiffness = Val;
    }
    FORCEINLINE void SetLinearDriveDamping(float Val) {
        LinearDriveDamping = Val;
    }

#pragma endregion
#pragma region Angular Drive
    // =====================================================================
    // === ANGULAR DRIVE PROPERTIES ===
    // ====================================================================

    FORCEINLINE bool IsAngularOrientationDriveEnabled(bool bUseLive = true) const {
        return (bUseLive && GetConstraintInstance())
                   ? GetConstraintInstance()->ProfileInstance.AngularDrive.IsOrientationDriveEnabled()
                   : bOrientationDriveEnabled; // Reuse or separate if needed
    }

    FORCEINLINE void SetAngularOrientationDriveEnabled(bool bEnabled, bool bWriteThrough = true) {
        bPositionDriveEnabled = bEnabled;
        if (bWriteThrough && GetConstraintInstance()) {
            GetConstraintInstance()->ProfileInstance.AngularDrive.AngularDriveMode =
                IsAngularOrientationDriveEnabled() ? EAngularDriveMode::SLERP : EAngularDriveMode::TwistAndSwing;
        }
    }

    // --- Angular Swing Getters
    FORCEINLINE float GetAngularSwingStiffness(bool bUseLive = true) const {
        return (bUseLive && GetConstraintInstance())
                   ? GetConstraintInstance()->ProfileInstance.AngularDrive.SwingDrive.Stiffness
                   : AngularSwingStiffness;
    }
    FORCEINLINE float GetAngularSwingDamping() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.AngularDrive.SwingDrive.Damping
                                       : AngularSwingDamping;
    }

    // --- Angular Twist Getters
    FORCEINLINE float GetAngularTwistStiffness() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.AngularDrive.TwistDrive.Stiffness
                                       : AngularTwistStiffness;
    }
    FORCEINLINE float GetAngularTwistDamping() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.AngularDrive.TwistDrive.Damping
                                       : AngularTwistDamping;
    }

    // --- Angular Swing Setters
    FORCEINLINE void SetAngularSwingStiffness(float Val, bool bWriteToLive = true) {
        AngularSwingStiffness = Val;
        if (bWriteToLive && GetConstraintInstance())
            GetConstraintInstance()->ProfileInstance.AngularDrive.SwingDrive.Stiffness = Val;
    }
    FORCEINLINE void SetAngularSwingDamping(float Val) {
        AngularSwingDamping = Val;
    }

    // --- Angular Twist Setters
    FORCEINLINE void SetAngularTwistStiffness(float Val) {
        AngularTwistStiffness = Val;
    }
    FORCEINLINE void SetAngularTwistDamping(float Val) {
        AngularTwistDamping = Val;
    }

#pragma endregion

#pragma region Drive State

    FORCEINLINE bool IsPositionDriveEnabled(bool bUseLive = true) const {
        return (bUseLive && GetConstraintInstance())
                   ? GetConstraintInstance()->ProfileInstance.LinearDrive.IsPositionDriveEnabled()
                   : bPositionDriveEnabled;
    }
    FORCEINLINE bool IsVelocityDriveEnabled() const {
        return bVelocityDriveEnabled;
    }

    FORCEINLINE void SetPositionDriveEnabled(bool bEnabled) {
        bPositionDriveEnabled = bEnabled;
    }
    FORCEINLINE void SetVelocityDriveEnabled(bool bEnabled) {
        bVelocityDriveEnabled = bEnabled;
    }

    FORCEINLINE FVector GetCachedPositionTarget() const {
        return CachedPositionTarget;
    }
    FORCEINLINE FVector GetCachedVelocityTarget() const {
        return CachedVelocityTarget;
    }

    FORCEINLINE void SetCachedPositionTarget(FVector Target) {
        CachedPositionTarget = Target;
    }
    FORCEINLINE void SetCachedVelocityTarget(FVector Target) {
        CachedVelocityTarget = Target;
    }

    // --- Cached‐target accessors for robust encapsulation ---
    FORCEINLINE void SetCachedOrientationTarget(const FQuat& InQ) {
        CachedOrientationTarget = InQ;
    }
    FORCEINLINE FQuat GetCachedOrientationTarget() const {
        return CachedOrientationTarget;
    }

    FORCEINLINE void SetCachedAngularVelocityTarget(const FVector& InV) {
        CachedAngularVelocityTarget = InV;
    }
    FORCEINLINE FVector GetCachedAngularVelocityTarget() const {
        return CachedAngularVelocityTarget;
    }

#pragma endregion

#pragma endregion

    FORCEINLINE float GetMaxDriveForce() const {
        return GetConstraintInstance() ? GetConstraintInstance()->ProfileInstance.LinearDrive.XDrive.MaxForce : 0.f;
    }

    FORCEINLINE bool IsOverDriven(float Threshold = 1000.f) const {
        return GetPositionStrain() > 5.f && GetMaxDriveForce() > Threshold;
    }

    // === OPTIMIZED SPATIAL ANALYSIS (No parameters needed) ===

    float GetBoneToConstraintAnchorDistance() const {
        if (!GetOwnerComponent() || !GetConstraintInstance())
            return -1.f;
        USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
        FConstraintInstance* CI = GetConstraintInstance();
        FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame2);
        FTransform BoneTM = SkelMesh->GetSocketTransform(GetChildBone(), RTS_World);

        return FVector::Dist(BoneTM.GetLocation(), ConstraintTM.GetLocation());
    }

    float GetConstraintAnchorToBoneDistance() const {
        if (!GetOwnerComponent() || !GetConstraintInstance())
            return -1.f;

        USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
        FConstraintInstance* CI = GetConstraintInstance();
        FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame1);
        FTransform BoneTM = SkelMesh->GetSocketTransform(GetParentBone(), RTS_World);

        return FVector::Dist(BoneTM.GetLocation(), ConstraintTM.GetLocation());
    }

    FVector GetConstraintAnchorOffset() const {
        if (!CachedOwnerComponent || !GetConstraintInstance())
            return FVector::ZeroVector;

        FConstraintInstance* CI = GetConstraintInstance();
        FTransform ParentFrame = CI->GetRefFrame(EConstraintFrame::Frame1);
        FTransform ChildFrame = CI->GetRefFrame(EConstraintFrame::Frame2);

        return ChildFrame.GetLocation() - ParentFrame.GetLocation();
    }

    float GetConstraintLeverArm() const {
        FVector Offset = GetConstraintAnchorOffset();
        return Offset.Size();
    }

    // === ENHANCED ALIGNMENT ANALYSIS ===

    float GetAlignmentError() const {
        if (!GetOwnerComponent() || !GetConstraintInstance())
            return 0.f;
        USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
        FConstraintInstance* CI = GetConstraintInstance();
        FTransform BoneTM = SkelMesh->GetSocketTransform(GetChildBone(), RTS_World);
        FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame2);

        FVector BoneDir = BoneTM.GetUnitAxis(EAxis::X);
        FVector ConstraintDir = ConstraintTM.GetUnitAxis(EAxis::X);

        return FMath::Acos(FVector::DotProduct(BoneDir, ConstraintDir));
    }

    // === INTELLIGENT TUNING RECOMMENDATIONS ===

    float GetRecommendedSwingLimit(float BaseBoneLength) const {
        if (!GetOwnerComponent())
            return 25.f;

        // Spatial factor from bone length
        float SpatialFactor = FMath::Clamp(BaseBoneLength / 15.f, 0.5f, 2.f);

        // Anchoring factor from constraint geometry
        float LeverArm = GetConstraintLeverArm();
        float AnchorFactor = FMath::Clamp(LeverArm / 10.f, 0.7f, 1.3f);

        // Stress-based adjustment
        float StressFactor = 1.0f;
        float CurrentStrain = GetCurrentStrain();
        if (CurrentStrain > 1.2f) {
            StressFactor = 0.8f; // Tighter limits for overstressed constraints
        } else if (CurrentStrain < 0.5f) {
            StressFactor = 1.2f; // Can afford looser limits
        }

        return 25.f * SpatialFactor * AnchorFactor * StressFactor;
    }

    float GetRecommendedTwistLimit(float BaseBoneLength) const {
        return GetRecommendedSwingLimit(BaseBoneLength) * 0.6f;
    }

    // === RUNTIME VALIDATION WITH CACHED COMPONENT ===

    bool ValidateConstraintGeometry() const {
        if (!GetOwnerComponent() || !GetConstraintInstance())
            return false;

        USkeletalMeshComponent* SkelMesh = GetOwnerComponent();
        // Verify both bones exist in the skeletal mesh
        const int32 ParentIndex = SkelMesh->GetBoneIndex(GetParentBone());
        const int32 ChildIndex = SkelMesh->GetBoneIndex(GetChildBone());

        if (ParentIndex == INDEX_NONE || ChildIndex == INDEX_NONE) {
            UE_LOG(LogTemp, Error, TEXT("[Constraint] Invalid bone indices: Parent=%s (%d), Child=%s (%d)"),
                   *GetParentBone().ToString(), ParentIndex, *GetChildBone().ToString(), ChildIndex);
            return false;
        }

        // Verify constraint frames are properly positioned
        float AnchorDistance = GetBoneToConstraintAnchorDistance();
        if (AnchorDistance > 50.f) // Suspiciously far anchor
        {
            UE_LOG(LogTemp, Warning, TEXT("[Constraint] Anchor distance unusually large: %.2f for %s"), AnchorDistance,
                   *GetChildBone().ToString());
        }

        return true;
    }

    // === PERFORMANCE METRICS ===

    struct FConstraintPerformanceMetrics {
        float AnchorDistance = 0.f;
        float LeverArm = 0.f;
        float AlignmentError = 0.f;
        float CurrentStrain = 0.f;
        bool bIsValid = false;
    };

    FConstraintPerformanceMetrics GetPerformanceMetrics() const {
        FConstraintPerformanceMetrics Metrics;

        if (GetOwnerComponent() || !GetConstraintInstance()) {
            return Metrics; // Invalid metrics
        }

        Metrics.AnchorDistance = GetBoneToConstraintAnchorDistance();
        Metrics.LeverArm = GetConstraintLeverArm();
        Metrics.AlignmentError = GetAlignmentError();
        Metrics.CurrentStrain = GetCurrentStrain();
        Metrics.bIsValid = true;

        return Metrics;
    }

    /** How far the child bone is from your cached target position */
    float GetPositionStrain() const {
        if (!CachedOwnerComponent) {
            return 0.f;
        }
        // GetBoneLocation returns world‐space location of the named bone
        const FVector CurrentPosition = CachedOwnerComponent->GetBoneLocation(ChildBone);
        return (CachedPositionTarget - CurrentPosition).Size();
    }

    /** Difference between desired velocity and actual physics velocity */
    float GetVelocityStrain() const {
        if (!CachedOwnerComponent) {
            return 0.f;
        }
        // Uses the physics‐engine linear velocity for the named bone
        const FVector CurrentVelocity = CachedOwnerComponent->GetPhysicsLinearVelocity(ChildBone);
        return (CachedVelocityTarget - CurrentVelocity).Size();
    }

    // --- Strain methods ---
    /**
     * Angular‐orientation strain: the absolute angle (deg) between current and target orientation
     */
    float GetOrientationStrain() const {
        if (!CachedOwnerComponent) {
            return 0.f;
        }
        // World‐space bone quaternion
        const FQuat CurrentQ = CachedOwnerComponent->GetBoneQuaternion(ChildBone);
        // Error quaternion: target * inverse(current)
        const FQuat ErrQ = GetCachedOrientationTarget() * CurrentQ.Inverse();
        FVector Axis;
        float AngleRad;
        ErrQ.ToAxisAndAngle(Axis, AngleRad);
        return FMath::Abs(FMath::RadiansToDegrees(AngleRad));
    }

    /**
     * Angular‐velocity strain: magnitude difference between desired and actual angular velocity
     */
    float GetAngularVelocityStrain() const {
        if (!CachedOwnerComponent) {
            return 0.f;
        }
        // Degrees/sec from physics engine
        const FVector CurrentAV = CachedOwnerComponent->GetPhysicsAngularVelocityInDegrees(ChildBone);
        return (GetCachedAngularVelocityTarget() - CurrentAV).Size();
    }

    float GetCurrentStrain() const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float Swing1 = CI->GetCurrentSwing1();
            float Swing2 = CI->GetCurrentSwing2();
            float Twist = CI->GetCurrentTwist();

            float Swing1Limit = CI->ProfileInstance.ConeLimit.Swing1LimitDegrees;
            float Swing2Limit = CI->ProfileInstance.ConeLimit.Swing2LimitDegrees;
            float TwistLimit = CI->ProfileInstance.TwistLimit.TwistLimitDegrees;

            float MaxSwingStrain = FMath::Max(FMath::Abs(Swing1) / FMath::Max(Swing1Limit, 1.f),
                                              FMath::Abs(Swing2) / FMath::Max(Swing2Limit, 1.f));
            float TwistStrain = FMath::Abs(Twist) / FMath::Max(TwistLimit, 1.f);

            return FMath::Max(MaxSwingStrain, TwistStrain);
        }
        return 0.f;
    }

    // Angular bias detection
    float GetAngularBias() const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float Swing1 = CI->GetCurrentSwing1();
            float Swing2 = CI->GetCurrentSwing2();
            float Twist = CI->GetCurrentTwist();

            float Swing1Limit = CI->ProfileInstance.ConeLimit.Swing1LimitDegrees;
            float Swing2Limit = CI->ProfileInstance.ConeLimit.Swing2LimitDegrees;
            float TwistLimit = CI->ProfileInstance.TwistLimit.TwistLimitDegrees;

            float Swing1Bias = (Swing1Limit != 0.f) ? Swing1 / Swing1Limit : 0.f;
            float Swing2Bias = (Swing2Limit != 0.f) ? Swing2 / Swing2Limit : 0.f;
            float TwistBias = (TwistLimit != 0.f) ? Twist / TwistLimit : 0.f;

            return FMath::Max3(FMath::Abs(Swing1Bias), FMath::Abs(Swing2Bias), FMath::Abs(TwistBias));
        }
        return 0.f;
    }

    // Human-readable bias reporting
    FString GetAngularBiasLabel() const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float Swing1 = CI->GetCurrentSwing1();
            float Swing2 = CI->GetCurrentSwing2();
            float Twist = CI->GetCurrentTwist();

            float Swing1Limit = CI->ProfileInstance.ConeLimit.Swing1LimitDegrees;
            float Swing2Limit = CI->ProfileInstance.ConeLimit.Swing2LimitDegrees;
            float TwistLimit = CI->ProfileInstance.TwistLimit.TwistLimitDegrees;

            float Swing1Bias = (Swing1Limit != 0.f) ? FMath::Abs(Swing1 / Swing1Limit) : 0.f;
            float Swing2Bias = (Swing2Limit != 0.f) ? FMath::Abs(Swing2 / Swing2Limit) : 0.f;
            float TwistBias = (TwistLimit != 0.f) ? FMath::Abs(Twist / TwistLimit) : 0.f;

            if (Swing1Bias > Swing2Bias && Swing1Bias > TwistBias)
                return FString::Printf(TEXT("Swing1 Bias: %.2f"), Swing1Bias);
            if (Swing2Bias > TwistBias)
                return FString::Printf(TEXT("Swing2 Bias: %.2f"), Swing2Bias);
            return FString::Printf(TEXT("Twist Bias: %.2f"), TwistBias);
        }
        return FString("No Constraint");
    }

    // Available angular motion before hitting limits
    FVector GetAngularSlack() const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float Swing1 = CI->GetCurrentSwing1();
            float Swing2 = CI->GetCurrentSwing2();
            float Twist = CI->GetCurrentTwist();

            float Swing1Limit = CI->ProfileInstance.ConeLimit.Swing1LimitDegrees;
            float Swing2Limit = CI->ProfileInstance.ConeLimit.Swing2LimitDegrees;
            float TwistLimit = CI->ProfileInstance.TwistLimit.TwistLimitDegrees;

            return FVector(FMath::Max(0.f, Swing1Limit - FMath::Abs(Swing1)),
                           FMath::Max(0.f, Swing2Limit - FMath::Abs(Swing2)),
                           FMath::Max(0.f, TwistLimit - FMath::Abs(Twist)));
        }
        return FVector::ZeroVector;
    }

    // Breakage and stability analysis
    FORCEINLINE bool IsConstraintBreakable() const {
        if (FConstraintInstance* CI = GetConstraintInstance())
            return CI->IsAngularBreakable();
        return false;
    }

    FORCEINLINE float GetConstraintBreakThreshold() const {
        if (FConstraintInstance* CI = GetConstraintInstance())
            return CI->GetAngularBreakThreshold();
        return -1.f;
    }

    FORCEINLINE void SetConstraintBreakable(bool bBreakable, float Threshold) const {
        if (FConstraintInstance* CI = GetConstraintInstance())
            CI->SetAngularBreakable(bBreakable, Threshold);
    }

    // Advanced constraint health metrics
    FORCEINLINE float GetConstraintStressScore(float MaxSwingAngle = 45.f) const {
        float Strain = GetCurrentStrain();
        return Strain / FMath::Max(MaxSwingAngle, 1.f);
    }

    FORCEINLINE static float GetConstraintFatigueRatio(float WearTime, float CriticalTime = 3.0f) {
        return FMath::Clamp(WearTime / CriticalTime, 0.f, 1.f);
    }

    float GetRigidityScore() const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float ConstraintDamping = CI->ProfileInstance.ConeLimit.Damping;
            float ConstraintStiffness = CI->ProfileInstance.ConeLimit.Stiffness;
            FVector Slack = GetAngularSlack();

            return (ConeLimitStiffness + ConeLimitDamping) / (Slack.Size() + 1.0f);
        }
        return 0.f;
    }

    // Predictive time-to-limit analysis
    float PredictTimeToConstraintLimit(const FVector& AngularVelocity, float MinThreshold = 1.0f) const {
        if (FConstraintInstance* CI = GetConstraintInstance()) {
            float Swing1 = CI->GetCurrentSwing1();
            float Swing2 = CI->GetCurrentSwing2();
            float Twist = CI->GetCurrentTwist();

            float Swing1Limit = FMath::Abs(CI->ProfileInstance.ConeLimit.Swing1LimitDegrees);
            float Swing2Limit = FMath::Abs(CI->ProfileInstance.ConeLimit.Swing2LimitDegrees);
            float TwistLimit = FMath::Abs(CI->ProfileInstance.TwistLimit.TwistLimitDegrees);

            // Project angular velocity onto constraint axes
            FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame2);
            FVector LocalAVel = ConstraintTM.InverseTransformVectorNoScale(AngularVelocity);

            float TimeToSwing1 = (Swing1Limit - FMath::Abs(Swing1)) / (FMath::Abs(LocalAVel.Y) + KINDA_SMALL_NUMBER);
            float TimeToSwing2 = (Swing2Limit - FMath::Abs(Swing2)) / (FMath::Abs(LocalAVel.Z) + KINDA_SMALL_NUMBER);
            float TimeToTwist = (TwistLimit - FMath::Abs(Twist)) / (FMath::Abs(LocalAVel.X) + KINDA_SMALL_NUMBER);

            float MinTime = FMath::Min3(TimeToSwing1, TimeToSwing2, TimeToTwist);
            return (MinTime < 0.f || MinTime > 5.f) ? -1.f : MinTime;
        }
        return -1.f;
    }

    // Auto-tuning based on current constraint state
    FORCEINLINE void AutoTuneFromStrain(float BaseDamp = 10.f, float BaseStiff = 2000.f) {
        float Strain = GetCurrentStrain();
        float TuneFactor = FMath::Clamp(Strain, 1.f, 2.f);

        SetConeLimitDamping(BaseDamp * TuneFactor);
        SetConeLimitStiffness(BaseStiff * TuneFactor);
    }

    static FOHConstraintInstanceData FromPhysicsConstraint(const FConstraintInstance* Instance, const FName& ParentBone,
                                                           const FName& ChildBone) {
        FOHConstraintInstanceData Data;
        Data.SetConstraintName(Instance->JointName);
        Data.SetParentBone(ParentBone);
        Data.SetChildBone(ChildBone);
        Data.SetSwing1LimitDegrees(Instance->ProfileInstance.ConeLimit.Swing1LimitDegrees);
        Data.SetSwing2LimitDegrees(Instance->ProfileInstance.ConeLimit.Swing2LimitDegrees);
        Data.SetTwistLimitDegrees(Instance->ProfileInstance.TwistLimit.TwistLimitDegrees);
        Data.SetConeLimitDamping(Instance->ProfileInstance.ConeLimit.Stiffness);
        Data.SetConeLimitStiffness(Instance->ProfileInstance.ConeLimit.Damping);
        Data.SetConstraintInstance(const_cast<FConstraintInstance*>(Instance));
        return Data;
    }

    bool InitializeFromTemplate(const UPhysicsConstraintTemplate* Template);
    /** Updates cached constraint parameters from the live constraint instance */
    void UpdateFromLiveConstraint();

    //  Not included:
    // - Transform of bones or bodies (access through bones)
    // - Whether bone is simulated (access through FOHBoneData)
    // - PAC properties (stored and modified elsewhere)
};

#pragma endregion
#pragma endregion

#pragma region BoneData

USTRUCT(BlueprintType)
struct FOHBoneData {
    GENERATED_BODY()

  private:
    UPROPERTY()
    FName BoneName;

    UPROPERTY()
    int32 BoneIndex = INDEX_NONE;

    UPROPERTY()
    FVector WorldPosition = FVector::ZeroVector;

    UPROPERTY()
    FVector PreviousPosition = FVector::ZeroVector;

    UPROPERTY()
    FQuat CurrentRotation = FQuat::Identity;

    UPROPERTY()
    FQuat PreviousRotation = FQuat::Identity;

    UPROPERTY()
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector LinearAcceleration = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY()
    FVector AngularAcceleration = FVector::ZeroVector;

    UPROPERTY()
    float LastDeltaTime = 1.0f;

    UPROPERTY()
    TArray<FOHMotionSample> MotionHistory;

    UPROPERTY()
    int32 MaxSamples = 10;

    UPROPERTY()
    bool bIsSimulating = false;

    /** Parent bone in the skeleton hierarchy */
    UPROPERTY()
    FName ParentBone;

    /** Immediate children in the hierarchy (not filtered by simulation status) */
    UPROPERTY()
    TArray<FName> ChildBones;

    /** True if this bone is associated with a simulating body */
    UPROPERTY()
    bool bHasSimulatedBody = false;

    UPROPERTY()
    UBodySetup* BodySetup = nullptr;

    /** Cached pointer to the body instance — runtime use only */
    FBodyInstance* BodyInstance = nullptr;

    UPROPERTY(Transient)
    USkeletalMeshComponent* OwnerComponent = nullptr;

    UPROPERTY()
    TArray<FName> AttachedConstraints; // Constraint names (or indices)

    /** Cached mass of this bone’s body, if it exists */
    UPROPERTY()
    float CachedBodyMass = 0.0f;

    /** Cached length to parent (distance in component space) */
    UPROPERTY()
    float CachedBoneLength = 0.0f;

    UPROPERTY()
    float CachedLinearDamping = -1.0f;

    UPROPERTY()
    float CachedAngularDamping = -1.0f;

    UPROPERTY()
    float CachedPhysicsBlendWeight = -1.0f;

    UPROPERTY()
    FPhysicalAnimationData LastAppliedPACSettings;

    UPROPERTY()
    bool bPACProfileDirty = false; // Optional: tracks if we need to reapply

    UPROPERTY(Transient)
    bool bBoneDirty = false;

  public:
    FORCEINLINE bool IsBoneDirty() const {
        return bBoneDirty;
    }
    FORCEINLINE void SetBoneDirty(bool bDirty = true) {
        bBoneDirty = bDirty;
    }

    FORCEINLINE void SetBodyInstance(FBodyInstance* InInstance) {
        BodyInstance = InInstance;
    }
    // Returns the body instance for this bone, or nullptr if invalid
    FORCEINLINE FBodyInstance* GetBodyInstance() const {
        if (!BodyInstance) {
            USkeletalMeshComponent* SkeletalMesh = GetOwnerComponent();
            return SkeletalMesh ? SkeletalMesh->GetBodyInstance(GetBoneName()) : nullptr;
        }
        return BodyInstance;
    }

    FORCEINLINE void SetBoneIndex(int32 InIndex) {
        BoneIndex = InIndex;
    }
    FORCEINLINE int32 GetBoneIndex() const {
        return BoneIndex;
    }

    FORCEINLINE void SetOwnerComponent(USkeletalMeshComponent* InComponent) {
        OwnerComponent = InComponent;
    }
    FORCEINLINE USkeletalMeshComponent* GetOwnerComponent() const {
        return OwnerComponent;
    }

    FORCEINLINE UBodySetup* GetBodySetup() const {
        return BodySetup;
    }
    FORCEINLINE void SetBodySetup(UBodySetup* Setup) {
        BodySetup = Setup;
    }

    FName GetParentConstraint(const TMap<FName, FOHConstraintInstanceData>& ConstraintMap) const {
        for (const FName& Name : AttachedConstraints) {
            if (const FOHConstraintInstanceData* C = ConstraintMap.Find(Name)) {
                if (C->GetChildBone() == GetBoneName())
                    return C->GetConstraintName();
            }
        }
        return NAME_None;
    }

    FName GetChildConstraint(const TMap<FName, FOHConstraintInstanceData>& ConstraintMap) const {
        for (const FName& Name : AttachedConstraints) {
            if (const FOHConstraintInstanceData* C = ConstraintMap.Find(Name)) {
                if (C->GetParentBone() == GetBoneName())
                    return C->GetConstraintName();
            }
        }
        return NAME_None;
    }

    bool IsLeafBone(const TMap<FName, FOHConstraintInstanceData>& ConstraintMap) const {
        int32 Outgoing = 0;
        for (const FName& Name : AttachedConstraints) {
            if (const FOHConstraintInstanceData* C = ConstraintMap.Find(Name))
                if (C->GetParentBone() == GetBoneName())
                    ++Outgoing;
        }
        return Outgoing == 0;
    }

    // === Accessors ===
    FORCEINLINE FName GetBoneName() const {
        return BoneName;
    }
    FORCEINLINE FName GetParentBone() const {
        return ParentBone;
    }
    FORCEINLINE const TArray<FName>& GetChildBones() const {
        return ChildBones;
    }
    TArray<FName> FindChildBones() const;

    // Add mutable getter
    FORCEINLINE TArray<FName>& GetMutableChildBones() {
        return ChildBones;
    }

    FORCEINLINE bool GetHasSimulatedBody() const {
        return bHasSimulatedBody;
    }
    FORCEINLINE float GetCachedBodyMass() const {
        return CachedBodyMass;
    }
    FORCEINLINE float GetCachedBoneLength() const {
        return CachedBoneLength;
    }

    // === Mutators ===
    FORCEINLINE void SetBoneName(FName InName) {
        BoneName = InName;
    }
    FORCEINLINE void SetParentBone(FName InParent) {
        ParentBone = InParent;
    }
    FORCEINLINE void AddChildBone(FName InChild) {
        ChildBones.Add(InChild);
    }

    FORCEINLINE void SetHasSimulatedBody(bool bSim) {
        bHasSimulatedBody = bSim;
    }
    FORCEINLINE void SetCachedBodyMass(float InMass) {
        CachedBodyMass = InMass;
    }
    FORCEINLINE void SetCachedBoneLength(float InLength) {
        CachedBoneLength = InLength;
    }
    /** Default number of history samples used in smoothing and analysis */
    static FORCEINLINE int32 GetDefaultSampleCount() {
        return 5;
    }

    // === Validity ===
    FORCEINLINE bool IsValid() const {
        return !GetBoneName().IsNone();
    }

    // === Position ===
    FORCEINLINE FVector GetCurrentPosition() const {
        return WorldPosition;
    }
    FORCEINLINE void SetCurrentPosition(const FVector& Pos) {
        WorldPosition = Pos;
    }

    FORCEINLINE FVector GetPreviousPosition() const {
        return PreviousPosition;
    }
    FORCEINLINE void SetPreviousPosition(const FVector& Pos) {
        PreviousPosition = Pos;
    }

    // === Rotation ===
    FORCEINLINE FQuat GetCurrentRotation() const {
        return CurrentRotation;
    }
    FORCEINLINE void SetCurrentRotation(const FQuat& Rot) {
        CurrentRotation = Rot;
    }

    FORCEINLINE FQuat GetPreviousRotation() const {
        return PreviousRotation;
    }
    FORCEINLINE void SetPreviousRotation(const FQuat& Rot) {
        PreviousRotation = Rot;
    }

    // === Velocity / Acceleration ===
    FORCEINLINE FVector GetLinearVelocity() const {
        return LinearVelocity;
    }
    FORCEINLINE void SetLinearVelocity(const FVector& V) {
        LinearVelocity = V;
    }

    FORCEINLINE FVector GetLinearAcceleration() const {
        return LinearAcceleration;
    }
    FORCEINLINE void SetLinearAcceleration(const FVector& A) {
        LinearAcceleration = A;
    }

    FORCEINLINE FVector GetAngularVelocity() const {
        return AngularVelocity;
    }
    FORCEINLINE void SetAngularVelocity(const FVector& AV) {
        AngularVelocity = AV;
    }

    FORCEINLINE FVector GetAngularAcceleration() const {
        return AngularAcceleration;
    }
    FORCEINLINE void SetAngularAcceleration(const FVector& AA) {
        AngularAcceleration = AA;
    }

    // === Delta Time ===
    FORCEINLINE float GetLastDeltaTime() const {
        return LastDeltaTime;
    }
    FORCEINLINE void SetLastDeltaTime(float DT) {
        LastDeltaTime = FMath::Max(DT, KINDA_SMALL_NUMBER);
    }

    // === Simulation State ===
    FORCEINLINE bool GetIsSimulating() const {
        return bIsSimulating;
    }

    bool GetIsSimulating() {
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->IsInstanceSimulatingPhysics();
        return bIsSimulating;
    }
    void SetIsSimulating(bool bSimulate) {
        bool bSet = false;
        if (FBodyInstance* Body = GetBodyInstance()) {
            Body->SetInstanceSimulatePhysics(bSimulate);
            bSet = true;
        }
        bIsSimulating = bSimulate;
        if (!bSet)
            UE_LOG(LogTemp, Warning, TEXT("SetIsSimulating: Set only cache, engine body missing for %s"),
                   *GetBoneName().ToString());
    }

    // === Motion History ===
    FORCEINLINE const TArray<FOHMotionSample>& GetMotionHistory() const {
        return MotionHistory;
    }

    FORCEINLINE bool HasMotionHistory() const {
        return MotionHistory.Num() > 0;
    }

    FORCEINLINE void ResetMotionHistory() {
        MotionHistory.Reset();
    }

    FORCEINLINE float GetMotionHistoryDuration() const {
        if (MotionHistory.Num() < 2)
            return 0.f;
        return MotionHistory.Last().GetTimeStamp() - MotionHistory[0].GetTimeStamp();
    }

    FORCEINLINE FOHMotionSample GetMotionSampleAt(int32 Index) const {
        return MotionHistory.IsValidIndex(Index) ? MotionHistory[Index] : FOHMotionSample();
    }

    FORCEINLINE void AddMotionSampleFromCurrentState(float Time) {
        const FTransform CurrentTransform = FTransform(CurrentRotation, WorldPosition);
        AddMotionSample(CurrentTransform, Time, LastDeltaTime);
    }

    FORCEINLINE int32 GetMaxSamples() const {
        return MaxSamples;
    }

    FORCEINLINE void PushMotionSample(const FOHMotionSample& Sample) {
        if (!Sample.IsValidSample()) {
            UE_LOG(LogTemp, Warning, TEXT("PushMotionSample: Invalid sample ignored."));
            return;
        }
        MotionHistory.Add(Sample);
        if (MotionHistory.Num() > MaxSamples) {
            MotionHistory.RemoveAt(0);
        }
    }

    FORCEINLINE void SetMaxSamples(int32 Count) {
        MaxSamples = FMath::Clamp(Count, 1, 100);
        if (MotionHistory.Num() > MaxSamples) {
            MotionHistory.SetNum(MaxSamples);
        }
    }

    // Get Body Instance Linear Velocity
    FVector GetBodyLinearVelocity() const {
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->GetUnrealWorldVelocity();
        return FVector::ZeroVector;
    }
    // Get Body Instance Angular Velocity
    FVector GetBodyAngularVelocity() const {
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->GetUnrealWorldAngularVelocityInRadians();
        return FVector::ZeroVector;
    }
    // Get Body Instance World Transform
    FTransform GetBodyWorldTransform() const {
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->GetUnrealWorldTransform();
        return FTransform::Identity;
    }

    bool IsAwake() const {
        if (!GetOwnerComponent())
            return false;
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->IsInstanceAwake();
        return GetOwnerComponent()->GetBodyInstance(GetBoneName())->IsInstanceAwake();
    }

    // Gets current linear damping from the physics body (returns -1 if not found)
    float GetLinearDamping(bool bPreferCached = true) const {
        if (bPreferCached && CachedLinearDamping >= 0.f)
            return CachedLinearDamping;

        if (FBodyInstance* Body = GetBodyInstance())
            return Body->LinearDamping;

        return -1.f;
    }

    // Sets new linear damping for this bone's physics body
    void SetLinearDamping(float NewValue) {
        if (!FMath::IsFinite(NewValue)) {
            UE_LOG(LogTemp, Warning, TEXT("SetLinearDamping rejected NaN/Inf value"));
            return;
        }

        if (!FMath::IsNearlyEqual(CachedLinearDamping, NewValue)) {
            CachedLinearDamping = NewValue;

            if (FBodyInstance* Body = GetBodyInstance()) {
                Body->LinearDamping = NewValue;
            }

            SetBoneDirty(true);
        }
    }

    float GetAngularDamping(bool bPreferCached = true) const {
        if (bPreferCached && CachedAngularDamping >= 0.f)
            return CachedAngularDamping;

        if (FBodyInstance* Body = GetBodyInstance())
            return Body->AngularDamping;

        return -1.f;
    }

    void SetAngularDamping(float NewValue) {
        if (!FMath::IsFinite(NewValue)) {
            UE_LOG(LogTemp, Warning, TEXT("SetAngularDamping rejected NaN/Inf value"));
            return;
        }

        if (!FMath::IsNearlyEqual(CachedAngularDamping, NewValue)) {
            CachedAngularDamping = NewValue;

            if (FBodyInstance* Body = GetBodyInstance()) {
                Body->AngularDamping = NewValue;
            }

            SetBoneDirty(true);
        }
    }

    float GetPhysicsBlendWeight(bool bPreferCached = true) const {
        if (bPreferCached && CachedPhysicsBlendWeight >= 0.f)
            return CachedPhysicsBlendWeight;

        if (FBodyInstance* Body = GetBodyInstance())
            return Body->PhysicsBlendWeight;

        return -1.f; // fallback if neither cached nor live is valid
    }

    void SetPhysicsBlendWeight(float NewValue) {
        if (!FMath::IsNearlyEqual(CachedPhysicsBlendWeight, NewValue)) {
            CachedPhysicsBlendWeight = NewValue;

            if (FBodyInstance* Body = GetBodyInstance()) {
                Body->PhysicsBlendWeight = NewValue;
            }

            SetBoneDirty(true);
        }
    }

    void CachePACProfile(const FPhysicalAnimationData& Profile) {
        LastAppliedPACSettings = Profile;
        bPACProfileDirty = false;
    }

    const FPhysicalAnimationData& GetCachedPACProfile() const {
        return LastAppliedPACSettings;
    }
    // --- Mass and Inertia ---

    float GetMass() const {
        if (CachedBodyMass > 0.f)
            return CachedBodyMass;
        if (FBodyInstance* Body = GetBodyInstance())
            return Body->GetBodyMass();
        return GetOwnerComponent() ? GetOwnerComponent()->GetBodyInstance(GetBoneName())->GetBodyMass() : -1.0f;
    }

    float GetBoneToConstraintAnchorDistance() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance()) {
            FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame2); // Frame2 is typically child bone
            FTransform BoneTM = GetOwnerComponent()->GetSocketTransform(BoneName, RTS_World);
            return FVector::Dist(BoneTM.GetLocation(), ConstraintTM.GetLocation());
        }
        return -1.f;
    }

    float GetConstraintAnchorToBoneDistance() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance()) {
            FTransform ConstraintTM = CI->GetRefFrame(EConstraintFrame::Frame1); // Frame1 is typically parent bone
            FTransform BoneTM = GetOwnerComponent()->GetSocketTransform(BoneName, RTS_World);
            return FVector::Dist(BoneTM.GetLocation(), ConstraintTM.GetLocation());
        }
        return -1.f;
    }

    float GetRecommendedSwingLimitDegrees() const {
        // Heuristic: longer bones, or those with farther anchor, can use higher limits
        float BoneLength = GetEstimatedBoneLength(); // Implement using ref skeleton or mesh vertices
        float SpatialFactor = FMath::Clamp(BoneLength / 15.f, 0.5f, 2.f);
        // Optionally combine with bone's typical motion stats (from your analytics)
        return 25.f * SpatialFactor; // Example: 25 degrees is default, scaled for long/short bones
    }

    void Wake() const {
        if (FBodyInstance* Body = GetBodyInstance())
            Body->WakeInstance();
    }

    void Sleep() const {
        if (FBodyInstance* Body = GetBodyInstance())
            Body->PutInstanceToSleep();
    }

    bool IsConstraintBreakable() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            return CI->IsAngularBreakable();
        return false;
    }
    float GetConstraintBreakThreshold() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            return CI->GetAngularBreakThreshold();
        return -1.f;
    }
    void SetConstraintBreakable(bool bBreakable, float Threshold) const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            CI->SetAngularBreakable(bBreakable, Threshold);
    }

    bool IsProjectionEnabled() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            return CI->IsProjectionEnabled();
        return false;
    }

    float GetParentSwingLimitStiffness() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            return CI->ProfileInstance.ConeLimit.Stiffness;
        return -1.f;
    }
    void SetParentSwingLimitStiffness(float Stiffness) const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            CI->ProfileInstance.ConeLimit.Stiffness = Stiffness;
    }

    void SetParentConstraintAngularDriveParams(float Stiffness, float Damping, float MaxForce) const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            CI->SetAngularDriveParams(Stiffness, Damping, MaxForce);
    }

    static float GetConstraintFatigueRatio(float WearTime, float CriticalTime = 3.0f) {
        return FMath::Clamp(WearTime / CriticalTime, 0.f, 1.f);
    }

    EAngularDriveMode::Type GetParentConstraintDriveMode() const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            return CI->GetAngularDriveMode();
        return EAngularDriveMode::SLERP; // default/fallback
    }
    void SetConstraintDriveMode(EAngularDriveMode::Type Mode) const {
        if (FConstraintInstance* CI = GetParentConstraintInstance())
            CI->SetAngularDriveMode(Mode);
    }

    // --------------- CPP Declarations -------------- //
    // Skeletal Hierarchy
    FORCEINLINE void SetOwningComponent(USkeletalMeshComponent* InComp) {
        OwnerComponent = InComp;
    }
    FORCEINLINE FBodyInstance* GetBodyInstance() {
        return OwnerComponent ? OwnerComponent->GetBodyInstance(BoneName) : nullptr;
    }
    // void ApplyImpulse(const FVector& Direction, float Magnitude, bool bVelChange = true);
    // void SetSimBlendWeight(float Alpha);

    TArray<FConstraintInstance*> GetAllParentConstraints() const;
    TArray<FName> GetAllParentBoneNames() const;

    TArray<FConstraintInstance*> GetChildConstraintsDownToDepth(int32 MaxDepth) const;
    TArray<FConstraintInstance*> GetAllChildConstraints() const;
    TArray<FConstraintInstance*> GetParentConstraintsUpToDepth(int32 MaxDepth) const;
    TArray<FName> GetBonesOfMaxDepthDown(int32 MaxDepth) const;
    TArray<FName> GetBonesOfMaxDepthUp(int32 MaxDepth) const;
    // Returns names of all direct children of this bone
    // Get all BodyInstances and Constraints for children
    TArray<FBodyInstance*> GetChildBodyInstances() const;
    // Get parent bone name
    FName GetParentBoneName() const;
    int32 GetParentBoneIndex() const;
    float GetEstimatedBoneLength() const;
    // --- Angular bias: How much is the bone "favoring" one side of its allowed motion? (0=centered, >0=biased) ---
    // Get the constraint instance linking this bone to its parent
    FConstraintInstance* GetParentConstraintInstance() const;

    // === Derivations ===
    // Returns average distance from this bone to its immediate neighbors (parent + all children)
    float GetLocalBoneCrowding() const;
    float GetVelocityDeviationMagnitude() const;
    float GetVelocitySpikeScore(float NormalizationFactor = 50.f) const;
    float GetPoseDriftScore() const;
    float GetStabilityScore() const;
    float GetAngularJitterScore(float NormalizationFactor = 10.f) const;
    float GetLinearDampingRatio() const;
    float GetRotationalDriftScore() const;
    float GetTrajectoryShiftScore() const;
    float GetImpulseSignatureScore() const;
    float GetCompositeInstabilityScore() const;
    float GetCurvatureScore() const;
    float GetDirectionalStabilityScore() const;
    float ComputeImpulseVulnerabilityScore(const FVector& ImpactNormal, const FVector& RootLocation,
                                           const FOHPhysicsGraphNode* PhysicsGraph) const;
    FVector ComputeImpulseDirection(const FHitResult& Hit, EImpulseDirectionMode Mode) const;
    FVector ComputeImpulseVector(const FHitResult& Hit, const FVector& RootLocation, float BaseStrength,
                                 EImpulseDirectionMode ModeOverride, bool bAutoInferMode,
                                 const FOHPhysicsGraphNode* PhysicsGraph) const;
    EImpulseDirectionMode InferBestImpulseDirectionMode(const FHitResult& Hit) const;
    float GetOscillationFrequencyHz() const;
    float GetMaxDisplacement() const;
    float GetUndershootRatio(const FVector& TargetPosition) const;
    float GetJerkMagnitude() const;
    FVector GetPositionBeforePrevious() const;
    float GetImpulseScaleFactor();
    void InitializeFromBodyInstance(FBodyInstance* InBodyInstance);
    void InitializeFromBodyInstance(const FBodyInstance* BI, const FReferenceSkeleton& RefSkeleton);
    void InitializeFromBodySetup(const UBodySetup* Setup, const FReferenceSkeleton& RefSkeleton);

    // Motion Samples
    void AddMotionSample(const FTransform& NewTransform, float CurrentTime, float DeltaTime);
    void TrimMotionHistoryByAge(float MaxAge, float CurrentTime);
    FOHMotionSample GetLatestMotionSample() const;
    FOHMotionSample GetPeakVelocitySample() const;
    FOHMotionSample GetClosestSampleByTime(float Time) const;

    // === Smoothed  ===
    FVector GetSmoothedLinearVelocity() const;
    FVector GetSmoothedAngularVelocity() const;
    FVector GetSmoothedLinearAcceleration() const;
    FVector GetSmoothedAngularAcceleration() const;

    // Averages
    FVector GetAverageLinearVelocity(int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetAverageAngularVelocity(int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetAverageLinearAcceleration(int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetAverageAngularAcceleration(int32 SampleCount = GetDefaultSampleCount()) const;

    // Transform
    FVector GetAveragePosition(int32 SampleCount = GetDefaultSampleCount()) const;
    FQuat GetAverageRotation(int32 SampleCount = GetDefaultSampleCount()) const;

    // Speed
    float GetAverageLinearSpeed(int32 SampleCount = GetDefaultSampleCount()) const;
    float GetAverageAngularSpeed(int32 SampleCount = GetDefaultSampleCount()) const;

    // Estimates
    FVector GetEstimatedLinearAcceleration(int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetEstimatedAngularAcceleration(int32 SampleCount = GetDefaultSampleCount()) const;

    // Blended
    FVector GetBlendedLinearVelocity(float HistoryWeight = 0.5f, int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetBlendedLinearAcceleration(float HistoryWeight = 0.5f, int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetBlendedAngularVelocity(float HistoryWeight = 0.5f, int32 SampleCount = GetDefaultSampleCount()) const;
    FVector GetBlendedAngularAcceleration(float HistoryWeight = 0.5f,
                                          int32 SampleCount = GetDefaultSampleCount()) const;

    FVector GetTimeWeightedLinearVelocity() const;
    FVector GetTimeWeightedAngularVelocity() const;
    FVector GetTimeWeightedAcceleration() const;
    FVector GetTimeWeightedAngularAcceleration() const;

    // === Kinematic Update ===
    void UpdateKinematics(const FVector& NewPosition, const FQuat& NewRotation, float DeltaTime, float TimeStamp);
    void UpdateFromComponent(float CurrentTime);
    void UpdateFromAnimationPose(float TimeStamp);
};

#pragma endregion
#pragma region BoneNameArrayWrapper

/*USTRUCT(BlueprintType)
struct FOHBoneNameArrayWrapper
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FName> Bones;
};
*/
#pragma endregion

#pragma region ConstraintInstanceArrayWrapper
/*
USTRUCT(BlueprintType)
struct FOHConstraintInstanceArrayWrapper
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FOHConstraintInstanceData> Constraints;
};
*/
#pragma endregion
#pragma region PhysicsGraphNode

USTRUCT(BlueprintType)
struct FOHPhysicsGraphNode {
    GENERATED_BODY()

  private:
    UPROPERTY()
    TMap<FName, FOHBoneData> BoneMap;
    UPROPERTY()
    TArray<FOHConstraintInstanceData> ConstraintLinks;

    //  UPROPERTY()
    // TMap<FName, FOHBoneNameArrayWrapper> ParentToChildrenMap;

    // UPROPERTY()
    //  TMap<FName, FOHConstraintInstanceArrayWrapper> BoneToConstraintsMap;

    UPROPERTY(Transient)
    USkeletalMeshComponent* MeshComponent = nullptr;
    UPROPERTY()
    USkeleton* Skeleton = nullptr;
    UPROPERTY()
    UPhysicsAsset* PhysicsAsset = nullptr;

    // ─── New: fast, one-to-many lookup containers (plain C++ members) ───────────────
    TMultiMap<FName, FName> ParentToChildrenMultiMap;
    TMultiMap<FName, FOHConstraintInstanceData*> BoneToConstraintsMultiMap;

  public:
    // === Data Accessors ===
    // -- BoneMap --
    FORCEINLINE TMap<FName, FOHBoneData> GetBoneMap() const {
        return BoneMap;
    }
    FORCEINLINE TMap<FName, FOHBoneData>& GetBoneMap() {
        return BoneMap;
    }

    // -- ConstraintLinks --
    FORCEINLINE const TArray<FOHConstraintInstanceData>& GetConstraintLinks() const {
        return ConstraintLinks;
    }
    FORCEINLINE TArray<FOHConstraintInstanceData>& GetConstraintLinks() {
        return ConstraintLinks;
    }
    FORCEINLINE TArray<FOHConstraintInstanceData> GetAllConstraints() const {
        return ConstraintLinks;
    }

    // -- ParentToChildrenMap --
    /* FORCEINLINE const TMap<FName, FOHBoneNameArrayWrapper>& GetParentToChildrenMap() const { return
     ParentToChildrenMap; } FORCEINLINE TMap<FName, FOHBoneNameArrayWrapper>& GetParentToChildrenMap() { return
     ParentToChildrenMap; } FORCEINLINE void SetParentToChildrenMap(const TMap<FName, FOHBoneNameArrayWrapper>& InMap) {
     ParentToChildrenMap = InMap; }*/
    /// Returns all child bones for the given parent (O(1) via TMultiMap)
    /// Raw, O(1) parent→children via TMultiMap
    FORCEINLINE TArray<FName> GetChildrenOfBone(FName ParentBone) const {
        TArray<FName> Out;
        ParentToChildrenMultiMap.MultiFind(ParentBone, Out, /*bMaintainOrder=*/true);
        return Out;
    }
    FORCEINLINE TArray<FName> GetAllBoneNames() const {
        TArray<FName> Names;
        BoneMap.GetKeys(Names);
        return Names;
    }

    // -- BoneToConstraintsMap --
    /* FORCEINLINE const TMap<FName, FOHConstraintInstanceArrayWrapper>& GetBoneToConstraintsMap() const { return
     BoneToConstraintsMap; } FORCEINLINE TMap<FName, FOHConstraintInstanceArrayWrapper>& GetBoneToConstraintsMap() {
     return BoneToConstraintsMap; } FORCEINLINE void SetBoneToConstraintsMap(const TMap<FName,
     FOHConstraintInstanceArrayWrapper>& InMap) { BoneToConstraintsMap = InMap; }*/
    /// Returns all constraints attached to the given bone as child or parent
    /// Raw, O(1) bone→constraints via TMultiMap
    FORCEINLINE TArray<FOHConstraintInstanceData> GetConstraintsOfBone(FName BoneName) const {
        TArray<FOHConstraintInstanceData> Result;
        TArray<FOHConstraintInstanceData*> Ptrs;
        BoneToConstraintsMultiMap.MultiFind(BoneName, Ptrs, /*bMaintainOrder=*/true);
        Result.Reserve(Ptrs.Num());
        for (auto* C : Ptrs)
            Result.Add(*C);
        return Result;
    }

    // -- Bone Validation and Find --
    FORCEINLINE bool IsBoneValid(const FName& BoneName) const {
        return BoneMap.Contains(BoneName);
    }
    FORCEINLINE FOHBoneData* FindBoneData(const FName& BoneName) {
        return BoneMap.Find(BoneName);
    }
    FORCEINLINE const FOHBoneData* FindBoneData(const FName& BoneName) const {
        return BoneMap.Find(BoneName);
    }

    // -- Component and Asset Pointers --
    FORCEINLINE USkeletalMeshComponent* GetMeshComponent() const {
        return MeshComponent;
    }
    FORCEINLINE USkeleton* GetSkeleton() const {
        return Skeleton;
    }
    FORCEINLINE UPhysicsAsset* GetPhysicsAsset() const {
        return PhysicsAsset;
    }

    // -- Setters (not usually needed in BP, but callable for C++ tools) --
    FORCEINLINE void SetMeshComponent(USkeletalMeshComponent* InMesh) {
        MeshComponent = InMesh;
    }
    FORCEINLINE void SetSkeleton(USkeleton* InSkeleton) {
        Skeleton = InSkeleton;
    }
    FORCEINLINE void SetPhysicsAsset(UPhysicsAsset* InAsset) {
        PhysicsAsset = InAsset;
    }

    // === Reset ===

    // New
    void Reset() {
        // 1. Log for debug clarity
        UE_LOG(LogTemp, Log, TEXT("[ClearPhysicsGraph] Resetting physics graph."));

        // 2. Clear and reset all maps
        GetBoneMap().Reset();
        // GetBoneToConstraintsMap().Reset();
        // GetParentToChildrenMap().Reset();

        // 3. Clear wrapper arrays
        GetConstraintLinks().Reset();

        // 4. Reset skeletal references
        SetMeshComponent(nullptr);
        SetSkeleton(nullptr);
        SetPhysicsAsset(nullptr);
    }

    /// Return the constraint data where this bone is the *child* (i.e. the incoming joint).
    /// Bones (except the root) appear exactly once as a child in ConstraintLinks.
    /// Mutable overload: return a non-const pointer so you can modify the entry.
    FORCEINLINE FOHConstraintInstanceData* FindConstraintByChildBone(FName ChildBone) {
        for (FOHConstraintInstanceData& C : ConstraintLinks) {
            if (C.GetChildBone() == ChildBone) {
                return &C;
            }
        }
        return nullptr;
    }

    /// Existing const overload: read-only access.
    FORCEINLINE const FOHConstraintInstanceData* FindConstraintByChildBone(FName ChildBone) const {
        for (const FOHConstraintInstanceData& C : ConstraintLinks) {
            if (C.GetChildBone() == ChildBone) {
                return &C;
            }
        }
        return nullptr;
    }

    // === Incremental Edit API ===
    bool AddBone(const FOHBoneData& Bone);
    bool RemoveBone(const FName& BoneName);
    bool AddConstraint(const FOHConstraintInstanceData& Constraint);
    bool RemoveConstraint(const FName& ConstraintName);

    // === Consistency/Wrapper Rebuild ===
    void RebuildWrappers();
    /** Returns a list of invalid bones (orphans, missing, or duplicate) */
    TArray<FName> GetInvalidBones(TArray<FString>* OutReasons = nullptr) const;
    /** Returns a list of invalid constraints, with reason strings. */
    TArray<FName> GetInvalidConstraints(TArray<FString>* OutReasons = nullptr) const;
    // === High-Level Queries ===
    TArray<FName> GetRootBones() const;
    TArray<FName> GetLeafBones() const;

    /** Computes a set of orphan bones (bones not referenced by any constraint). */
    TSet<FName> GetOrphanBones() const;
    /** Validate the entire graph, optionally auto-repair invalid constraints, and log issues. Returns true if valid. */
    bool ValidateGraph(bool bLog = true, bool bAutoRepair = false);
    /** Snapshots current graph state for diff/undo. */
    TSet<FName> CaptureReachableBones(const FName& StartBone) const;
    /** Snapshots constraints for diff/undo. */
    TSet<FName> CaptureConstraintNames() const;
    /** Diffs previous and current states, reporting additions/removals. */

    /** Returns true if the graph is fully connected (single island) */
    bool IsConnected() const;

    // == Bulk Initialization ==
    void InitializeBonesFromSource(const TArray<FOHBoneData*>& Sources);
    void InitializeConstraintsFromSource(const TArray<FOHConstraintInstanceData*>& Sources);

    // == Snapshot/Diff ==
    TSet<FName> CaptureBoneSnapshot(const FName& RootBone) const;
    TSet<FName> CaptureConstraintSnapshot() const;
    static void DiffBoneSnapshots(const TSet<FName>& OldSnap, const TSet<FName>& NewSnap, TSet<FName>& OutRemoved,
                                  TSet<FName>& OutAdded);

    // == Components (Islands) ==
    /** Computes islands (disconnected subgraphs) in the current graph. */
    TArray<TSet<FName>> GetIslands(bool bBidirectional = false) const;

    // == Cycles ==
    bool HasCycles(TArray<FName>* OutCycle = nullptr) const;

    // == Shortest Path ==
    TArray<FName> FindShortestPath(FName Start, FName End) const;

    // == Utilities ==
    void RemoveInvalidConstraints();
    void RemoveOrphanBones();

    // === Debug Print ===
    void PrintGraphState(const FString& Tag = TEXT("PhysicsGraph")) const;

    // Direct constraint analytics access
    const FOHConstraintInstanceData* GetConstraintForBone(FName BoneName) const {
        // Find constraint where this bone is the child
        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            if (Constraint.GetChildBone() == BoneName)
                return &Constraint;
        }
        return nullptr;
    }

    FOHConstraintInstanceData* GetConstraintForBone(FName BoneName) {
        for (FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            if (Constraint.GetChildBone() == BoneName)
                return &Constraint;
        }
        return nullptr;
    }

    // Convenience accessors for common constraint analytics
    float GetConstraintStrain(FName BoneName) const {
        if (const FOHConstraintInstanceData* Constraint = GetConstraintForBone(BoneName))
            return Constraint->GetCurrentStrain();
        return 0.f;
    }

    float GetConstraintBias(FName BoneName) const {
        if (const FOHConstraintInstanceData* Constraint = GetConstraintForBone(BoneName))
            return Constraint->GetAngularBias();
        return 0.f;
    }

    FVector GetConstraintSlack(FName BoneName) const {
        if (const FOHConstraintInstanceData* Constraint = GetConstraintForBone(BoneName))
            return Constraint->GetAngularSlack();
        return FVector::ZeroVector;
    }

    // Chain-level analytics
    float GetChainStabilityScore(FName RootBone, int32 Depth = 3) const {
        TSet<FName> VisitedBones = CaptureReachableBones(RootBone);

        float TotalStrain = 0.f;
        int32 ConstraintCount = 0;

        for (const FName& Bone : VisitedBones) {
            if (const FOHConstraintInstanceData* Constraint = GetConstraintForBone(Bone)) {
                TotalStrain += Constraint->GetCurrentStrain();
                ConstraintCount++;
            }
        }

        return ConstraintCount > 0 ? TotalStrain / ConstraintCount : 0.f;
    }

    // Find overstressed constraints across the entire graph
    TArray<FName> GetOverstressedConstraints(float Threshold = 1.5f) const {
        TArray<FName> OverstressedBones;

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            if (Constraint.GetCurrentStrain() > Threshold) {
                OverstressedBones.Add(Constraint.GetChildBone());
            }
        }

        return OverstressedBones;
    }

    // Get constraint stress distribution for visualization
    TMap<FName, float> GetConstraintStressDistribution() const {
        TMap<FName, float> StressMap;

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            StressMap.Add(Constraint.GetChildBone(), Constraint.GetCurrentStrain());
        }

        return StressMap;
    }

    void InitializeConstraintComponents(USkeletalMeshComponent* OwnerComponent) {
        if (!OwnerComponent) {
            UE_LOG(LogTemp, Error, TEXT("[PhysicsGraph] Cannot initialize constraints: null OwnerComponent"));
            return;
        }

        // Cache owner component in all constraints
        for (FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            Constraint.SetOwnerComponent(OwnerComponent);
        }

        // Cache in graph level too
        MeshComponent = OwnerComponent;

        UE_LOG(LogTemp, Log, TEXT("[PhysicsGraph] Initialized %d constraints with OwnerComponent"),
               ConstraintLinks.Num());
    }

    // === VALIDATION WITH CACHED COMPONENTS ===

    bool ValidateAllConstraintGeometry() const {
        if (!MeshComponent) {
            UE_LOG(LogTemp, Error, TEXT("[PhysicsGraph] No MeshComponent cached for validation"));
            return false;
        }

        bool bAllValid = true;
        int32 ValidCount = 0;

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            if (Constraint.ValidateConstraintGeometry()) {
                ValidCount++;
            } else {
                bAllValid = false;
                UE_LOG(LogTemp, Warning, TEXT("[PhysicsGraph] Constraint validation failed: %s->%s"),
                       *Constraint.GetParentBone().ToString(), *Constraint.GetChildBone().ToString());
            }
        }

        UE_LOG(LogTemp, Log, TEXT("[PhysicsGraph] Constraint validation: %d/%d valid"), ValidCount,
               ConstraintLinks.Num());

        return bAllValid;
    }

    // === OPTIMIZED ANALYTICS (No parameter passing) ===

    TMap<FName, float> GetConstraintAnchorDistances() const {
        TMap<FName, float> Distances;

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            float Distance = Constraint.GetBoneToConstraintAnchorDistance();
            if (Distance >= 0.f) {
                Distances.Add(Constraint.GetChildBone(), Distance);
            }
        }

        return Distances;
    }

    TArray<FName> GetPoorlyAnchoredConstraints(float MaxDistance = 5.0f) const {
        TArray<FName> ProblematicBones;

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            float Distance = Constraint.GetBoneToConstraintAnchorDistance();
            if (Distance > MaxDistance) {
                ProblematicBones.Add(Constraint.GetChildBone());
            }
        }

        return ProblematicBones;
    }

    // === PERFORMANCE MONITORING ===

    void LogConstraintPerformanceReport() const {
        UE_LOG(LogTemp, Log, TEXT("=== Constraint Performance Report ==="));

        for (const FOHConstraintInstanceData& Constraint : ConstraintLinks) {
            auto Metrics = Constraint.GetPerformanceMetrics();
            if (Metrics.bIsValid) {
                UE_LOG(LogTemp, Log, TEXT("[%s] Anchor:%.1f | Lever:%.1f | Align:%.2f° | Strain:%.2f"),
                       *Constraint.GetChildBone().ToString(), Metrics.AnchorDistance, Metrics.LeverArm,
                       FMath::RadiansToDegrees(Metrics.AlignmentError), Metrics.CurrentStrain);
            }
        }
    }
};

#pragma endregion

#pragma endregion

#pragma region BoneNameResolution_Structs
#pragma region NameMatchResult

USTRUCT(BlueprintType)
struct FOHNameMatchResult {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FString Candidate;

    UPROPERTY(BlueprintReadOnly)
    float Score = 0.f;

    UPROPERTY(BlueprintReadOnly)
    FString AlgorithmUsed;

    FOHNameMatchResult() = default;

    FOHNameMatchResult(const FString& InCandidate, float InScore, const FString& InAlgorithmUsed)
        : Candidate(InCandidate), Score(InScore), AlgorithmUsed(InAlgorithmUsed) {}
};

#pragma endregion

#pragma region NameResolver
/**
 * Name resolver for resolving names to other names.
 *
 * This is used for resolving names to other names, such as resolving
 * a bone name to a bone name in another skeleton.
 *
 * This is used for resolving names to other names, such as resolving
 * a bone name to a bone name in another skeleton.
 */
USTRUCT(BlueprintType)
struct FOHNameResolver {
    GENERATED_BODY()

    UPROPERTY()
    TMap<FName, FName> ResolvedMap;

    UPROPERTY()
    TMap<FName, FOHNameMatchResult> MatchResults;

    void Resolve(const TSet<FName>& Inputs, const TArray<FName>& Candidates);
    FName GetResolved(FName Input) const;
    FOHNameMatchResult GetMatchResult(FName Input) const;
    bool HasResolved(FName Input) const;
};

#pragma endregion

#pragma region ResolvedBoneData
USTRUCT(BlueprintType)
struct FOHResolvedBoneData {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    EOHSkeletalBone LogicalBone = EOHSkeletalBone::None;

    UPROPERTY(BlueprintReadOnly)
    FName ResolvedBone = NAME_None;

    UPROPERTY(BlueprintReadOnly)
    EOHBodyZone BodyZone = EOHBodyZone::None;

    UPROPERTY(BlueprintReadOnly)
    EOHBodyPart BodyPart = EOHBodyPart::None;

    UPROPERTY(BlueprintReadOnly)
    EOHFunctionalBoneGroup FunctionalGroup = EOHFunctionalBoneGroup::None;

    UPROPERTY(BlueprintReadOnly)
    FOHNameMatchResult MatchResult;

    UPROPERTY(BlueprintReadOnly)
    bool bResolved = false;
};

#pragma endregion

#pragma endregion

#pragma region BlendTiming_Structs

// ============================= Active Physics Blend ============================= //

#pragma region ActivePhysicsBlend
USTRUCT()
struct FActivePhysicsBlend {
    GENERATED_BODY()

    UPROPERTY()
    FName RootBone;

    UPROPERTY()
    float Elapsed = 0.0f;

    UPROPERTY()
    float BlendInDuration = 0.1f;

    UPROPERTY()
    float HoldDuration = 0.0f;

    UPROPERTY()
    float BlendOutDuration = 0.3f;

    UPROPERTY()
    float BlendAlpha = 0.0f;

    UPROPERTY()
    EOHBlendPhases Phase = EOHBlendPhases::BlendIn;

    UPROPERTY()
    FName ReactionTag = NAME_None;

    /** Total time this blend will run (sum of In, Hold, Out) */
    UPROPERTY()
    float TotalBlendTime = 0.f;

    UPROPERTY()
    float StartAlpha = 0.f; // New: captures starting alpha when blend begins

    UPROPERTY()
    EOHBlendEasingType EasingType = EOHBlendEasingType::Linear;

    UPROPERTY()
    int32 PauseCount = 0; // ✅ replaces bPaused

    FORCEINLINE bool IsPaused() const {
        return PauseCount > 0;
    }
};
#pragma endregion

// ===================== Blend Operation ===================== //

#pragma region BlendOperation
USTRUCT(BlueprintType)
struct ONLYHANDS_API FBlendOperation {
    GENERATED_BODY()

    // Target bone for the blend operation
    UPROPERTY(EditAnywhere, Category = "Blend|Target")
    EOHSkeletalBone Bone = EOHSkeletalBone::None;

    /** If set, this bone will blend toward the transform of another bone */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proxy")
    EOHSkeletalBone ProxyFollowSourceBone = EOHSkeletalBone::None;

    // Timestamp when the blend operation started (in seconds)
    UPROPERTY(EditAnywhere, Category = "Blend|Timing")
    double StartTime = 0.0;

    // Duration of the blend in seconds
    UPROPERTY(EditAnywhere, Category = "Blend|Timing")
    float Duration = 0.25f;

    // Starting blend weight (0.0 = full animation, 1.0 = full physics)
    UPROPERTY(EditAnywhere, Category = "Blend|Weight")
    float StartWeight = 0.0f;

    // Target blend weight (0.0 = full animation, 1.0 = full physics)
    UPROPERTY(EditAnywhere, Category = "Blend|Weight")
    float TargetWeight = 1.0f;

    // Type of blend operation
    UPROPERTY(EditAnywhere, Category = "Blend|Type")
    EOHTransitionTypes BlendType = EOHTransitionTypes::FadeIn;

    // Optional curve to control the blend interpolation
    UPROPERTY(EditAnywhere, Category = "Blend|Curves")
    UCurveFloat* BlendCurve = nullptr;

    // Optional strength override to apply during the blend (-1 means not used)
    UPROPERTY(EditAnywhere, Category = "Blend|Physics")
    float StrengthOverride = -1.0f;

    // Optional curve for dynamic strength adjustment during blend
    UPROPERTY(EditAnywhere, Category = "Blend|Curves")
    UCurveFloat* StrengthCurve = nullptr;

    // Whether to deactivate physics when done
    UPROPERTY(EditAnywhere, Category = "Blend|Physics")
    bool bDeactivateOnComplete = false;

    /** If true, the bone's profile will be removed after this blend completes. */
    UPROPERTY(EditAnywhere, Category = "Blend|Lifecycle")
    bool bRemoveProfileAfter = false;

    // Current state of this blend operation
    UPROPERTY(VisibleAnywhere, Category = "Blend|State")
    EOHBoneSimulationState CurrentState = EOHBoneSimulationState::Blending;

    // ID to track this operation (useful for callbacks or cancellation)
    UPROPERTY(VisibleAnywhere, Category = "Blend|Utility")
    int32 BlendOperationId = INDEX_NONE;

    /** Optional: tracks which profile (if any) initiated this operation */
    UPROPERTY()
    FName ProfileNameOverride = NAME_None;

    /** When blending to simulated, apply transform propagation downward to child bones. */
    UPROPERTY()
    bool bPropagateAttachedRoleToChildren = false;

    /** Optional curve to control how force strength blends over time independently of simulation */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UCurveFloat* ForceStrengthCurve = nullptr;
    // Default constructor (using member initializers above)
    FBlendOperation() = default;

    // Static helper to create a fade-in blend operation
    static FBlendOperation CreateFadeIn(EOHSkeletalBone Bone, float Duration, float CurrentWeight,
                                        FName ProfileNameOverride = NAME_None) {
        FBlendOperation Op;
        Op.Bone = Bone;
        Op.Duration = FMath::Max(Duration, 0.01f); // Ensure safe duration
        Op.BlendType = EOHTransitionTypes::FadeIn;
        Op.StartWeight = FMath::Clamp(CurrentWeight, 0.0f, 1.0f);
        Op.TargetWeight = 1.0f;
        Op.ProfileNameOverride = ProfileNameOverride;
        Op.BlendOperationId = HashCombine(GetTypeHash(Bone), GetTypeHash(FPlatformTime::Seconds()));

        return Op;
    }

    // Static helper to create a fade-out blend operation
    static FBlendOperation CreateFadeOut(EOHSkeletalBone Bone, float Duration, float CurrentWeight,
                                         FName ProfileNameOverride = NAME_None) {
        FBlendOperation Op;
        Op.Bone = Bone;
        Op.Duration = FMath::Max(Duration, 0.01f);
        Op.BlendType = EOHTransitionTypes::FadeOut;
        Op.StartWeight = FMath::Clamp(CurrentWeight, 0.0f, 1.0f);
        Op.TargetWeight = 0.0f;
        Op.ProfileNameOverride = ProfileNameOverride;
        Op.BlendOperationId = HashCombine(GetTypeHash(Bone), GetTypeHash(FPlatformTime::Seconds()));

        return Op;
    }

    // Calculate the current blend weight based on elapsed time
    float CalculateCurrentWeight(double CurrentTime) const {
        const double ElapsedTime = CurrentTime - StartTime;
        const float Alpha = FMath::Clamp(static_cast<float>(ElapsedTime / Duration), 0.0f, 1.0f);

        // Use blend curve if provided
        if (BlendCurve) {
            const float CurveAlpha = BlendCurve->GetFloatValue(Alpha);
            return FMath::Lerp(StartWeight, TargetWeight, CurveAlpha);
        }

        // Otherwise use linear interpolation
        return FMath::Lerp(StartWeight, TargetWeight, Alpha);
    }

    // Calculate the current strength multiplier (if applicable)
    float CalculateStrengthMultiplier(double CurrentTime) const {
        // If there's a direct override and no curve, use it
        if (StrengthOverride >= 0.0f && !StrengthCurve) {
            return StrengthOverride;
        }

        // If there's a strength curve, evaluate it
        if (StrengthCurve) {
            const double ElapsedTime = CurrentTime - StartTime;
            const float Alpha = FMath::Clamp(static_cast<float>(ElapsedTime / Duration), 0.0f, 1.0f);

            // If there's also a strength override, use it as a base multiplier
            float Multiplier = StrengthOverride >= 0.0f ? StrengthOverride : 1.0f;

            return Multiplier * StrengthCurve->GetFloatValue(Alpha);
        }

        // No override or curve, return -1 to indicate default strength should be used
        return -1.0f;
    }

    // Check if the blend operation is complete
    bool IsComplete(double CurrentTime) const {
        return (CurrentTime - StartTime) >= Duration;
    }

    // Update the state based on progress
    void UpdateState(double CurrentTime) {
        if (IsComplete(CurrentTime)) {
            CurrentState = FMath::IsNearlyZero(TargetWeight) ? EOHBoneSimulationState::Kinematic
                                                             : EOHBoneSimulationState::Simulating;
        } else {
            CurrentState = EOHBoneSimulationState::Blending;
        }
    }
};

#pragma endregion

#pragma endregion

#pragma region Animation_Structs

// ==================== Montage Section Window ==================== //

#pragma region MontageSectionWindow
USTRUCT(BlueprintType)
struct FOHMontageSectionWindow {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    FName SectionName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float NormalizedTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    bool bCanCancel = false;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    bool bCanChain = false;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    bool bIsLocked = false;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float RawPlayTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float SectionLength = 0.f;
};
#pragma endregion

// =================== Pose Root Motion State =================== //

#pragma region PoseRootMotionState
USTRUCT(BlueprintType)
struct FOHRootMotionState {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "RootMotion")
    bool bHasRootMotion = false;

    UPROPERTY(BlueprintReadOnly, Category = "RootMotion")
    FTransform RootMotionDelta = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "RootMotion")
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "RootMotion")
    FRotator RotationDelta = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "RootMotion")
    float DeltaTime = 0.f;
};
#pragma endregion

// ==================== Slot Blend State ==================== //

#pragma region SlotBlendState
USTRUCT(BlueprintType)
struct FOHSlotBlendState {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    FName SlotName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    bool bIsSlotActive = false;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    float BlendWeight = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    UAnimMontage* ActiveMontage = nullptr;
};
#pragma endregion

// ========================== Anim Instance Playback State ========================== //

#pragma region AnimInstancePlaybackState
USTRUCT(BlueprintType)
struct FOHAnimInstancePlaybackState {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    bool bIsValid = false;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    bool bIsMontagePlaying = false;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    UAnimMontage* ActiveMontage = nullptr;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    UAnimationAsset* ActiveAsset = nullptr;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    float PlayPosition = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    float PlayRate = 1.f;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    float MontageBlendWeight = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Animation")
    FName CurrentMontageSection = NAME_None;
};

#pragma endregion

// ========================== Montage Playback State ========================== //

#pragma region MontagePlaybackState

USTRUCT(BlueprintType)
struct FOHMontagePlaybackState {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    bool bIsValid = false;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    FName MontageName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    FName SectionName = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float Position = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float SectionLength = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float NormalizedTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    float BlendWeight = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage")
    UAnimSequence* CurrentAnimSequence = nullptr;

    // NEW: Segment-level playback info
    UPROPERTY(BlueprintReadOnly, Category = "Montage|Segment")
    float SegmentStartTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage|Segment")
    float SegmentLength = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage|Segment")
    float SegmentPlayTime = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "Montage|Segment")
    float SegmentNormalizedTime = 0.f;
};
#pragma endregion

#pragma endregion

#pragma region MotionSampling_Structs

#pragma region BoneMotionSample
USTRUCT(BlueprintType)
struct FOHBoneMotionSample {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> Points;

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> Velocities;

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> Accelerations;

    UPROPERTY(BlueprintReadOnly)
    int32 NumPoints = 0;

    UPROPERTY(BlueprintReadOnly)
    float Duration = 0.f;

    UPROPERTY(BlueprintReadOnly)
    FName BoneName = NAME_None;

    UPROPERTY(BlueprintReadOnly)
    UAnimSequence* SourceAnimation = nullptr;
};
#pragma endregion

#pragma region BoneTrail
USTRUCT(BlueprintType)
struct FOHBoneTrail {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FName BoneName;

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> PositionHistory;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 MaxSamples = 10;

    FOHBoneTrail() : BoneName(NAME_None), MaxSamples(10) {}

    void AddSample(const FVector& Position) {
        if (MaxSamples <= 0)
            return;

        PositionHistory.Insert(Position, 0);

        if (PositionHistory.Num() > MaxSamples) {
            PositionHistory.SetNum(MaxSamples, true);
        }
    }

    const TArray<FVector>& GetTrail() const {
        return PositionHistory;
    }

    bool IsValid() const {
        return BoneName != NAME_None && PositionHistory.Num() > 1;
    }
};
#pragma endregion

#pragma region BoneArcSampler

USTRUCT()
struct FOHAsyncBoneArcSampler {
    GENERATED_BODY()

    TWeakObjectPtr<USkeletalMeshComponent> Mesh;
    FName BoneName = NAME_None;
    float Duration = 0.f;
    float ElapsedTime = 0.f;
    float Interval = 0.016f; // ~60 FPS
    int32 MaxSamples = 0;

    TArray<FVector> ArcPoints;
    FTimerHandle TimerHandle;
};
#pragma endregion
#pragma endregion

#pragma region StrikeEvaluation_Structs
// ======================= Strike Contact Metrics ======================= //

#pragma region StrikeContactMetrics
USTRUCT(BlueprintType)
struct FStrikeContactMetrics {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    AActor* HitActor = nullptr;

    UPROPERTY(BlueprintReadOnly)
    FName StrikingBone;

    UPROPERTY(BlueprintReadOnly)
    FVector ContactLocation = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    FVector ContactNormal = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    FVector Velocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    float VelocityDotNormal = 0.f;

    UPROPERTY(BlueprintReadOnly)
    float ImpactAngleDegrees = -1.f;

    UPROPERTY(BlueprintReadOnly)
    float EstimatedForce = 0.f;
};

#pragma endregion

// ======================= Deferred Strike State ======================= //

#pragma region DeferredStrikeSweepState

USTRUCT(BlueprintType)
struct FDeferredStrikeSweepState {
    GENERATED_BODY()

    UPROPERTY()
    USkeletalMeshComponent* Mesh = nullptr;

    UPROPERTY()
    AActor* Owner = nullptr;

    UPROPERTY()
    TArray<FName> BoneChain;

    UPROPERTY()
    float AccumulatedTime = 0.f;

    UPROPERTY()
    float SampleInterval = 0.016f;

    UPROPERTY()
    float TimeSinceLastSample = 0.f;

    UPROPERTY()
    TArray<ACharacter*> HitCharacters;

    UPROPERTY()
    TArray<FStrikeContactMetrics> Contacts;
};

#pragma endregion

#pragma endregion

// ========================= Physics Bone Link ====================== //

#pragma region PhysicsBoneLink

USTRUCT(BlueprintType)
struct FOHPhysicsBoneLink {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    FName Parent;

    UPROPERTY(EditAnywhere)
    TArray<FName> Children;

    UPROPERTY(EditAnywhere)
    float DampingMultiplier = 1.0f;

    UPROPERTY(EditAnywhere)
    float AngularConstraintTolerance = 15.f; // Degrees

    UPROPERTY(EditAnywhere)
    float PropagationAttenuation = 0.5f;
};

#pragma endregion
