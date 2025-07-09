#pragma once
#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "OHAnimInstance_Base.generated.h"

UENUM()
enum class EFootPhase : uint8
{
	Unknown,
	Swing,   // Foot is moving forward or up (in air)
	Plant,   // Foot is on the ground and not moving much
	Lift     // Foot is leaving the ground
};

USTRUCT(BlueprintType)
struct FFootTraceBuffer
{
	GENERATED_BODY()

	// How many samples to keep (tweakable if you want!)
	static constexpr int32 BufferSize = 8;

	FVector Positions[BufferSize]{};
	float Times[BufferSize]{};
	int32 Index = 0;
	int32 Valid = 0;

	// --- Add a new sample ---
	void AddSample(const FVector& Pos, float Time)
	{
		Positions[Index] = Pos;
		Times[Index] = Time;
		Index = (Index + 1) % BufferSize;
		if (Valid < BufferSize) ++Valid;
	}

	// --- Get velocity (instantaneous, last two samples) ---
	FVector GetVelocity() const
	{
		if (Valid < 2) return FVector::ZeroVector;
		int32 Prev = (Index - 1 + BufferSize) % BufferSize;
		int32 Old  = (Index - 2 + BufferSize) % BufferSize;
		float Dt = Times[Prev] - Times[Old];
		return (Dt > KINDA_SMALL_NUMBER) ? (Positions[Prev] - Positions[Old]) / Dt : FVector::ZeroVector;
	}

	// --- Get average position (smoothed) ---
	FVector GetAvgPosition() const
	{
		if (Valid < 1) return FVector::ZeroVector;
		FVector Sum = FVector::ZeroVector;
		for (int i = 0; i < Valid; ++i) Sum += Positions[i];
		return Sum / float(Valid);
	}

	// --- Get oldest sample ---
	FVector GetOldestPosition() const
	{
		if (Valid < 1) return FVector::ZeroVector;
		int32 Oldest = (Index + BufferSize - Valid) % BufferSize;
		return Positions[Oldest];
	}

	// --- Reset buffer (useful on teleport, etc) ---
	void Reset()
	{
		for (int i = 0; i < BufferSize; ++i)
		{
			Positions[i] = FVector::ZeroVector;
			Times[i] = 0.f;
		}
		Index = 0;
		Valid = 0;
	}
};

// --- A helper for phase, timing, and stride/plant prediction ---
static constexpr int32 PhaseBufferSize = 12;

USTRUCT()
struct FFootPhaseSample
{
	GENERATED_BODY()
	EFootPhase Phase = EFootPhase::Unknown;
	float Time = 0.f;
	FVector Pos = FVector::ZeroVector;
	float Speed = 0.f;

	FFootPhaseSample() {}
	FFootPhaseSample(EFootPhase InPhase, float InTime, FVector InPos, float InSpeed)
		: Phase(InPhase), Time(InTime), Pos(InPos), Speed(InSpeed) {}
};

USTRUCT()
struct FFootPhaseBuffer
{
    GENERATED_BODY()
    static constexpr int32 BufferSize = 8;

    FFootPhaseSample Buffer[BufferSize];
    int32 Index = 0;
    int32 Valid = 0;

    void Add(EFootPhase Phase, float Time, const FVector& Pos = FVector::ZeroVector, float Speed = 0.f)
    {
        Buffer[Index] = FFootPhaseSample(Phase, Time, Pos, Speed);
        Index = (Index + 1) % BufferSize;
        if (Valid < BufferSize) ++Valid;
    }

    // Latest sample
    FFootPhaseSample Latest() const
    {
        int32 Prev = (Index - 1 + BufferSize) % BufferSize;
        return Buffer[Prev];
    }

    // How long (seconds) has the current phase lasted
    float TimeInCurrentPhase(float CurrentTime) const
    {
        return FMath::Max(0.f, CurrentTime - Latest().Time);
    }

    // Frames the current phase has lasted
    int32 FramesInCurrentPhase() const
    {
        if (Valid < 2) return 1;
        EFootPhase Cur = Latest().Phase;
        int32 Count = 1;
        for (int32 i = 1; i < Valid; ++i)
        {
            int32 CheckIdx = (Index - 1 - i + BufferSize) % BufferSize;
            if (Buffer[CheckIdx].Phase == Cur) ++Count;
            else break;
        }
        return Count;
    }

    // Recent phase transitions, newest first
    TArray<FFootPhaseSample> RecentPhaseChanges(int32 Num = 3) const
    {
        TArray<FFootPhaseSample> Out;
        if (Valid == 0) return Out;
        EFootPhase LastPhase = Latest().Phase;
        for (int32 i = 1, Changes = 0; i < Valid && Changes < Num; ++i)
        {
            int32 CheckIdx = (Index - 1 - i + BufferSize) % BufferSize;
            if (Buffer[CheckIdx].Phase != LastPhase)
            {
                Out.Add(Buffer[CheckIdx]);
                LastPhase = Buffer[CheckIdx].Phase;
                ++Changes;
            }
        }
        return Out;
    }

    // Predict next phase using the last cycle(s)
    EFootPhase PredictNextPhase() const
    {
        if (Valid < 3) return Latest().Phase;
        TArray<FFootPhaseSample> Changes = RecentPhaseChanges(3);
        if (Changes.Num() < 2) return Latest().Phase;
        // For a common gait: Plant -> Lift -> Swing -> Plant ...
        const EFootPhase Last = Changes[0].Phase;
        const EFootPhase Prev = Changes[1].Phase;

        if (Last == EFootPhase::Plant && Prev == EFootPhase::Swing) return EFootPhase::Lift;
        if (Last == EFootPhase::Swing && Prev == EFootPhase::Lift) return EFootPhase::Plant;
        if (Last == EFootPhase::Lift && Prev == EFootPhase::Plant) return EFootPhase::Swing;
        // If only two, oscillate Plant <-> Swing
        if ((Last == EFootPhase::Plant && Prev == EFootPhase::Swing) ||
            (Last == EFootPhase::Swing && Prev == EFootPhase::Plant))
            return (Last == EFootPhase::Plant) ? EFootPhase::Swing : EFootPhase::Plant;
        // Default
        return Latest().Phase;
    }

    // Predict time until next phase change (averaged from history)
    float PredictPhaseChangeTime(float CurrentTime) const
    {
        TArray<FFootPhaseSample> Changes = RecentPhaseChanges(3);
        if (Changes.Num() < 2) return 0.2f; // fallback
        float AvgDelta = 0.f;
        int32 Found = 0;
        for (int32 i = 1; i < Changes.Num(); ++i)
        {
            float Dt = Changes[i-1].Time - Changes[i].Time;
            if (Dt > 0.01f) { AvgDelta += Dt; ++Found; }
        }
        return (Found > 0) ? AvgDelta / Found : 0.25f;
    }

    // Average stride velocity (over last Plant->Swing->Plant or similar)
    FVector GetAvgStrideVelocity() const
    {
        TArray<FFootPhaseSample> Changes = RecentPhaseChanges(2);
        if (Changes.Num() < 2) return FVector::ZeroVector;
        const FFootPhaseSample& Newer = Changes[0];
        const FFootPhaseSample& Older = Changes[1];
        float Dt = Newer.Time - Older.Time;
        if (Dt <= 0.0001f) return FVector::ZeroVector;
        return (Newer.Pos - Older.Pos) / Dt;
    }

    // Confidence: what percent of recent buffer is in the same phase
    float GetPhaseConfidence() const
    {
        if (Valid < 2) return 1.f;
        EFootPhase LatestPhase = Latest().Phase;
        int32 StableFrames = 1;
        for (int32 i = 1; i < Valid; ++i)
        {
            int32 CheckIdx = (Index - 1 - i + BufferSize) % BufferSize;
            if (Buffer[CheckIdx].Phase == LatestPhase) StableFrames++;
            else break;
        }
        return static_cast<float>(StableFrames) / static_cast<float>(Valid);
    }

    // Time since last phase transition
    float TimeSinceLastTransition(float CurrentTime) const
    {
        TArray<FFootPhaseSample> Changes = RecentPhaseChanges(1);
        if (Changes.Num() < 1) return 0.f;
        return FMath::Max(0.f, CurrentTime - Changes[0].Time);
    }

    void Reset()
    {
        for (int32 i = 0; i < BufferSize; ++i)
            Buffer[i] = FFootPhaseSample(EFootPhase::Plant, 0.f, FVector::ZeroVector, 0.f);
        Index = 0;
        Valid = 0;
    }
};


UCLASS()
class ONLYHANDS_API UOHAnimInstance_Base : public UAnimInstance
{
	GENERATED_BODY()

public:
	UOHAnimInstance_Base();
	

	UFUNCTION(BlueprintPure, Category = "Animation")
	ERootMotionMode::Type GetRootMotionModePublic() const { return RootMotionMode; }

	// --- Stride phase thresholds (used in DetectFootPhase) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|PhaseDetection")
	float PhaseSwingVerticalSpeed = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|PhaseDetection")
	float PhaseSwingHorizontalSpeed = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|PhaseDetection")
	float PhasePlantVerticalSpeed = 2.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|PhaseDetection")
	float PhasePlantHorizontalSpeed = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|PhaseDetection")
	float PhaseLiftVerticalSpeed = -5.0f;

	// --- Plant fade/blend time (speed of foot planting/unplanting) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float PlantFadeSpeed = 8.0f;

	// --- Adaptive smoothing (context-driven) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float AdaptiveSmoothingMin = 0.40f; // more responsive

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float AdaptiveSmoothingMax = 0.93f; // ultra sticky

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float AdaptiveInterpSpeedMin = 7.0f; // responsive

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float AdaptiveInterpSpeedMax = 2.0f; // very smooth

	// --- Spline trajectory memory (frames used for smoothing path) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	int32 SplineMax = 3;

	// --- Max effector jump (to prevent hard pops when ground changes sharply) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float MaxEffectorWorldJump = 7.0f;

	// --- Offsets for floor mesh (per foot, easily tweaked if shoes or mesh change)
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float LeftFootGroundOffset = 0.f;

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float RightFootGroundOffset = 0.f;

// --- Foot trace/IK smoothing (actual movement per frame)
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float MaxFootMoveDelta = 2.5f; // [cm/frame] effector movement clamp

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float MaxFootRotDelta = 6.f; // [deg/frame] rotation clamp

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
int32 LockHysteresisFrames = 6; // frames to "hold" after loss of trace

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float OffsetInterpSpeed = 15.f; // For Z offset smoothing (FInterpTo)

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float NormalInterpSpeed = 15.f; // For normal smoothing (VInterpTo)

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float PelvisInterpSpeed = 10.f; // For pelvis Z

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float StickyStandingBlend = 0.95f; // [0..1], higher = more glue when idle

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float MaxStairHeight = 30.f; // [cm] max heel/toe Z diff before pop

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
float IdleFreezeSpeed = 2.0f; // [cm/s] feet "stick" at low velocity

// --- Foot shape/size for collision (adjust for each character!)
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float IKFootSphereRadius = 5.0f; // [cm] shoe width

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float IKFootCapsuleHalfHeight = 8.0f; // [cm] shoe/foot length

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float IKFootForwardBias = 18.0f; // [cm] for predictive traces (dodges/steps)

UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Tuning")
float IKFootProbeDepth = 60.0f; // [cm] trace down distance


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float GlobalIKSmoothing = 0.75f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float FloorProbeDepth = 10.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="IK|Config")
	float FloorPlaneRadius = 16.f;


	
// --- Outputs: never mark as EditAnywhere! ---
UPROPERTY(BlueprintReadOnly, Category="IK") float LeftFootIKOffset = 0.f;
UPROPERTY(BlueprintReadOnly, Category="IK") float RightFootIKOffset = 0.f;
UPROPERTY(BlueprintReadOnly, Category="IK") FVector LeftFootIKEffector = FVector::ZeroVector;
UPROPERTY(BlueprintReadOnly, Category="IK") FVector RightFootIKEffector = FVector::ZeroVector;
UPROPERTY(BlueprintReadOnly, Category="IK") FRotator LeftFootIKRotation = FRotator::ZeroRotator;
UPROPERTY(BlueprintReadOnly, Category="IK") FRotator RightFootIKRotation = FRotator::ZeroRotator;
UPROPERTY(BlueprintReadOnly, Category="IK") float PelvisIKOffset = 0.f;

// --- Debug (for animation blueprints or debug UI) ---
UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float FootDeltaZ = 0.f;
UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float LeftFootSlopeAngle = 0.f;
UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float RightFootSlopeAngle = 0.f;
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float LeftFootStrideConfidence = 1.f;
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float RightFootStrideConfidence = 1.f;
	// --- Stride phase state ---
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") int32 LeftStridePhase = 0;
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") int32 RightStridePhase = 0;
	// --- Stride ghost step lookahead time (for visualization/tuning) ---
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float LeftGhostLookaheadTime = 0.0f;
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") float RightGhostLookaheadTime = 0.0f;
	// --- World effector debug values (component/local space) ---
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug") FVector NewLockedLEffector;
	UPROPERTY(BlueprintReadOnly, Category="IK|Debug")FVector NewLockedREffector;
	
protected:
	
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

private:

    // --- FOOT BUFFERING
    UPROPERTY()
    FFootTraceBuffer LeftFootBuffer;
    UPROPERTY()
    FFootTraceBuffer RightFootBuffer;

	// --- PHASE BUFFERING (robust stride/intent/anticipation)
	UPROPERTY()
	FFootPhaseBuffer LeftPhaseBuffer;
	UPROPERTY()
	FFootPhaseBuffer RightPhaseBuffer;

    // --- SPLINE SMOOTHING (stores several frames of effectors)
    TArray<FVector> LeftEffectorSpline;
    TArray<FVector> RightEffectorSpline;

    // --- Z BUFFERING / JITTER REDUCTION ---
    static constexpr int32 FootZBufferSize = 6;
    float LFootZBuffer[FootZBufferSize] = {};
    float RFootZBuffer[FootZBufferSize] = {};
    int32 LFootZBufIndex = 0, RFootZBufIndex = 0, LFootZBufValidSamples = 0, RFootZBufValidSamples = 0;
    bool bLFootFirstUpdate = true, bRFootFirstUpdate = true;
    float SmoothedLFootGroundZ = 0.f, SmoothedRFootGroundZ = 0.f;
    //float LastLFootZ = 0.f, LastRFootZ = 0.f;

    // --- LOCKED POSITIONS / DAMPING ---
    //FVector LastLeftFootPos = FVector::ZeroVector;
    //FVector LastRightFootPos = FVector::ZeroVector;
	//bool bLeftFootPlanted = false;
	//bool bRightFootPlanted = false;
	//FVector LockedLeftEffector = FVector::ZeroVector;
	//FVector LockedRightEffector = FVector::ZeroVector;
	
    FVector LockedLeftFootWorldPos = FVector::ZeroVector;
    FVector LockedRightFootWorldPos = FVector::ZeroVector;
    FVector DampedLeftEffector = FVector::ZeroVector;
    FVector DampedRightEffector = FVector::ZeroVector;


    // --- NORMAL SMOOTHING ---
   // FVector SmoothedLGroundNormal = FVector::UpVector;
    //FVector SmoothedRGroundNormal = FVector::UpVector;

    // --- PLANT LOGIC
    FVector LastValidLeftImpact = FVector::ZeroVector;
    FVector LastValidRightImpact = FVector::ZeroVector;
    FVector LastValidLeftNormal = FVector::UpVector;
    FVector LastValidRightNormal = FVector::UpVector;
    int32 LeftFootLockFrames = 0, RightFootLockFrames = 0;
    FRotator LockedLeftRot = FRotator::ZeroRotator;
    FRotator LockedRightRot = FRotator::ZeroRotator;

    // --- PLANT FADE
    float LeftPlantAlpha = 1.f, RightPlantAlpha = 1.f;
    float LastLeftPlantTime = 0.f, LastRightPlantTime = 0.f;

    // --- FOOT LENGTH ---
    bool bMeasuredFootLength = false;
    float AutoFootLength = 18.f;

    // --- OFFSET AUTO-DETECT
    bool bAutoDetectedFootOffsets = false;

	// --- HELPERS (cpp)
	// --- Capsule-based predictive trace under a bone (for dynamic dodge/slide, most robust for high speed)
	static bool CapsuleGroundTrace(
	const USkeletalMeshComponent* Mesh, const FName& BoneName, float MaxDistance, float ForwardBias,
	float CapsuleRadius, float CapsuleHalfHeight, const FVector& Velocity,
	FHitResult& OutHit, bool bDrawDebug = false);

	// --- Sphere-based ground trace (simpler, less edge artifacts for small feet/shoes)
	static bool SphereGroundTrace(
	const USkeletalMeshComponent* Mesh, const FName& BoneName, float MaxDistance, float ForwardBias,
	float SphereRadius, const FVector& Velocity, FHitResult& OutHit, bool bDrawDebug = false);


	// --- Dual-trace with backup (avoids foot "dangling" on steps/gaps)
	static bool SafeDoubleTrace(
		const USkeletalMeshComponent* Mesh, FName Bone, float MaxDist, float Bias, float Radius, const FVector& Velocity, 
		FHitResult& OutMain, FHitResult& OutBackup, float BackupDepth = 8.f);


	// --- Median for temporal smoothing (robust outlier rejection)
	static float Median(const float* Arr, int32 Count);


	// --- Simple Catmull-Rom for 3-point splines (anti-snapping for foot effector)
	static FVector CatmullRomInterp(const TArray<FVector>& P, float T);


	// --- Find highest valid floor nearby (8 directions, for floor-awareness/blending)
	static float FindHighestNearbyFloorZ(const FVector& Center, UWorld* World, float Radius = 15.f, float ProbeDepth = 12.f);

	// --- Procedural phase detection (with fallback to Unknown)
	// Foot phase detection with UPROPERTY thresholds (per instance, not static)
	UFUNCTION(BlueprintCallable, Category="IK|Phase")
	EFootPhase DetectFootPhase(const FFootTraceBuffer& Buffer) const;
	
	// --- Heel-toe blend alpha for smooth rolling
	static float GetHeelToeBlendAlpha(EFootPhase Phase, const FFootTraceBuffer& Buffer);

	// --- Critically-damped spring smoothing (jitterless!)
	static FVector CriticallyDampedSmoothing(const FVector& Current, const FVector& Target, float DeltaTime, float Spring = 80.f, float Damping = 1.2f);

};