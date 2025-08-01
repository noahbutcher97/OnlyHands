#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Utilities/OHSafeMapUtils.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "OHMovementComponent.generated.h"

UENUM(BlueprintType)
enum class EOHGait : uint8 { Walking, Running, Sprinting };

// Enums for stance tracking
UENUM(BlueprintType)
enum class EOHFootPhase : uint8 {
    Contact UMETA(DisplayName = "Contact"),      // Foot touching ground, bearing weight
    Push UMETA(DisplayName = "Push"),            // Foot pushing off ground
    Lift UMETA(DisplayName = "Lift"),            // Foot lifting from ground
    Swing UMETA(DisplayName = "Swing"),          // Foot swinging through air
    Reach UMETA(DisplayName = "Reach"),          // Foot reaching for next contact
    Plant UMETA(DisplayName = "Plant"),          // Foot planting down
    Pivot UMETA(DisplayName = "Pivot"),          // Foot rotating while planted
    Transition UMETA(DisplayName = "Transition") // Between phases
};

UENUM(BlueprintType)
enum class EOHStanceType : uint8 {
    Neutral UMETA(DisplayName = "Neutral"),     // Standard stance
    Forward UMETA(DisplayName = "Forward"),     // Leading foot forward
    Backward UMETA(DisplayName = "Backward"),   // Trailing foot back
    Wide UMETA(DisplayName = "Wide"),           // Feet spread wide
    Narrow UMETA(DisplayName = "Narrow"),       // Feet close together
    Staggered UMETA(DisplayName = "Staggered"), // Offset stance
    Dynamic UMETA(DisplayName = "Dynamic")      // In motion/transitioning
};

USTRUCT(BlueprintType)
struct FOHMovementHistoryFrame {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FTransform Transform; // Position + Rotation (root)

    UPROPERTY(BlueprintReadOnly)
    FVector2D InputVector; // Player movement input (normalized)

    UPROPERTY(BlueprintReadOnly)
    FVector Velocity; // World-space velocity (root)

    UPROPERTY(BlueprintReadOnly)
    FVector Acceleration; // Linear acceleration (Δvelocity / Δt)

    UPROPERTY(BlueprintReadOnly)
    FVector Jerk; // Linear jerk (Δacceleration / Δt)

    UPROPERTY(BlueprintReadOnly)
    float TurnRate = 0.f; // deg/sec: how fast facing changes between frames

    UPROPERTY(BlueprintReadOnly)
    float InputMagnitude = 0.f; // magnitude of InputVector

    UPROPERTY(BlueprintReadOnly)
    float Alignment = 1.f; // dot(Velocity, Facing). 1 = aligned, -1 = opposite

    UPROPERTY(BlueprintReadOnly)
    float Curvature = 0.f; // change in velocity direction per unit distance

    UPROPERTY(BlueprintReadOnly)
    float AbsoluteTime = 0.f; // World time or tick time (for cross-system sync)

    UPROPERTY(BlueprintReadOnly)
    float DeltaTime; // Time between frames

    FOHMovementHistoryFrame()
        : Transform(FTransform::Identity), InputVector(FVector2D::ZeroVector), Velocity(FVector::ZeroVector),
          Acceleration(FVector::ZeroVector), Jerk(FVector::ZeroVector), DeltaTime(0.f) {}

    // === Frame Calculation Helper ===

    /** Static factory for history frame with auto acceleration/jerk. */
    static FOHMovementHistoryFrame Create(const FTransform& CurrentTransform, const FVector2D& CurrentInput,
                                          const FVector& CurrentVelocity, float InDeltaTime,
                                          const FOHMovementHistoryFrame* LastFrame // nullptr allowed
    ) {
        FOHMovementHistoryFrame Frame;
        Frame.Transform = CurrentTransform;
        Frame.InputVector = CurrentInput;
        Frame.Velocity = CurrentVelocity;
        Frame.DeltaTime = InDeltaTime;

        if (LastFrame) {
            Frame.Acceleration = (Frame.Velocity - LastFrame->Velocity) / FMath::Max(InDeltaTime, KINDA_SMALL_NUMBER);
            Frame.Jerk = (Frame.Acceleration - LastFrame->Acceleration) / FMath::Max(InDeltaTime, KINDA_SMALL_NUMBER);
        } else {
            Frame.Acceleration = FVector::ZeroVector;
            Frame.Jerk = FVector::ZeroVector;
        }

        return Frame;
    }

    // Update the buffer with the latest frame
    template <typename BufferType>
    static const FOHMovementHistoryFrame* UpdateBuffer(BufferType& Buffer, const FTransform& CurrentTransform,
                                                       const FVector2D& CurrentInput, const FVector& CurrentVelocity,
                                                       float DeltaTime) {
        const FOHMovementHistoryFrame* LastFrame = Buffer.GetLatest();
        FOHMovementHistoryFrame Frame = Create(CurrentTransform, CurrentInput, CurrentVelocity, DeltaTime, LastFrame);
        Buffer.Add(Frame);
        return Buffer.GetLatest();
    }

    template <typename BufferType> static void ClearBuffer(BufferType& Buffer) {
        Buffer.Num = 0;
        Buffer.Start = 0;
    }

    /** Remove the Nth latest frame (N = 0 is latest, 1 is previous, etc.) */
    template <typename BufferType> static void RemoveFrame(BufferType& Buffer, int32 N = 0) {
        if (N >= Buffer.Num)
            return;
        int32 RemoveIndex = (Buffer.Start + Buffer.Num - 1 - N + Buffer.Capacity) % Buffer.Capacity;
        // Move all newer frames down
        for (int32 i = RemoveIndex; i != (Buffer.Start + Buffer.Num - 1) % Buffer.Capacity;
             i = (i + 1) % Buffer.Capacity) {
            int32 Next = (i + 1) % Buffer.Capacity;
            Buffer.Data[i] = Buffer.Data[Next];
        }
        --Buffer.Num;
    }

    // --- Simple accessors ---
    FVector GetPosition() const {
        return Transform.GetLocation();
    }
    FRotator GetRotation() const {
        return Transform.Rotator();
    }
    FVector GetForward() const {
        return Transform.GetRotation().GetForwardVector();
    }
    FVector GetRight() const {
        return Transform.GetRotation().GetRightVector();
    }
    FVector GetUp() const {
        return Transform.GetRotation().GetUpVector();
    }

    float GetSpeed() const {
        return Velocity.Size();
    }
    float GetAccelerationMagnitude() const {
        return Acceleration.Size();
    }
    float GetJerkMagnitude() const {
        return Jerk.Size();
    }

    // --- Static Frame Finders ---

    /** Find the frame closest to a specified time ago (from now). */
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetFrameClosestToTime(const BufferType& Buffer, float SecondsAgo) {
        float Accum = 0.f;
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Accum += F->DeltaTime;
                if (Accum >= SecondsAgo)
                    return F;
            }
        }
        return nullptr; // If no frame is old enough
    }
    /** Find last frame where there was significant forward input (X axis > threshold) */
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetLastFrameWithForwardInput(const BufferType& Buffer,
                                                                       float Threshold = 0.1f) {
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                if (FMath::Abs(F->InputVector.X) > Threshold)
                    return F;
            }
        }
        return nullptr;
    }

    /** Find last frame where there was significant lateral input (Y axis > threshold) */
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetLastFrameWithLateralInput(const BufferType& Buffer,
                                                                       float Threshold = 0.1f) {
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                if (FMath::Abs(F->InputVector.Y) > Threshold)
                    return F;
            }
        }
        return nullptr;
    }

    /** Find last frame where the input direction was very different from movement direction */
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetLastFrameWithInputMovementMismatch(const BufferType& Buffer,
                                                                                float AngleThresholdDeg = 60.f) {
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                FVector InputDir(F->InputVector, 0.f);
                InputDir.Normalize();
                FVector MoveDir = F->Velocity.GetSafeNormal();
                if (!InputDir.IsNearlyZero() && !MoveDir.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(InputDir, MoveDir);
                    float Angle = FMath::Acos(Dot) * 180.f / PI;
                    if (Angle > AngleThresholdDeg)
                        return F;
                }
            }
        }
        return nullptr;
    }

    /** Find the last frame with a significant direction change (change in velocity direction over AngleThresholdDeg) */
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetLastDirectionChange(const BufferType& Buffer,
                                                                 float AngleThresholdDeg = 45.f) {
        for (int32 i = 1; i < Buffer.NumFrames(); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(DirA, DirB);
                    float Angle = FMath::Acos(Dot) * 180.f / PI;
                    if (Angle > AngleThresholdDeg)
                        return Curr;
                }
            }
        }
        return nullptr;
    }

    // Last frame where movement stopped
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetLastFrameWhereStopped(const BufferType& Buffer,
                                                                   float SpeedThreshold = 5.f) {
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i))
                if (F->GetSpeed() < SpeedThreshold)
                    return F;
        }
        return nullptr;
    }

    // Frame closest to a specified world location
    template <typename BufferType>
    static const FOHMovementHistoryFrame* GetFrameClosestToPosition(const BufferType& Buffer, const FVector& Position) {
        float ClosestDistSqr = FLT_MAX;
        const FOHMovementHistoryFrame* Closest = nullptr;
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                float DistSqr = FVector::DistSquared(F->GetPosition(), Position);
                if (DistSqr < ClosestDistSqr) {
                    ClosestDistSqr = DistSqr;
                    Closest = F;
                }
            }
        }
        return Closest;
    }

    // Frame where facing direction matched a target rotation within X degrees
    template <typename BufferType>
    static const FOHMovementHistoryFrame*
    GetLastFrameWithFacingDirection(const BufferType& Buffer, const FRotator& TargetRot, float ToleranceDeg = 10.f) {
        for (int32 i = 0; i < Buffer.NumFrames(); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                float Angle = FMath::Abs(FMath::FindDeltaAngleDegrees(F->GetRotation().Yaw, TargetRot.Yaw));
                if (Angle < ToleranceDeg)
                    return F;
            }
        }
        return nullptr;
    }

    template <typename BufferType>
    static const FOHMovementHistoryFrame*
    GetFirstFrameAfterStopWithInput(const BufferType& Buffer, float StopSpeedThreshold = 5.f,
                                    float InputThreshold = 0.1f, int32 MaxFramesBack = 10) {
        bool bWasStopped = false;
        for (int32 i = 0; i < FMath::Min(MaxFramesBack, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                if (F->GetSpeed() < StopSpeedThreshold)
                    bWasStopped = true;
                else if (bWasStopped && F->InputVector.Size() > InputThreshold)
                    return F;
            }
        }
        return nullptr;
    }

    // --- Static Analytics For Buffers ---
    // Get average speed over last N frames
    template <typename BufferType> static float GetAverageSpeed(const BufferType& Buffer, int32 Frames) {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Sum += F->GetSpeed();
                ++Count;
            }
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : 0.f;
    }

    // Get average acceleration over last N frames
    template <typename BufferType> static FVector GetAverageVelocity(const BufferType& Buffer, int32 Frames) {
        FVector Sum = FVector::ZeroVector;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Sum += F->Velocity;
                ++Count;
            }
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : FVector::ZeroVector;
    }

    // Get average acceleration over last N frames
    template <typename BufferType> static FVector GetAverageAcceleration(const BufferType& Buffer, int32 Frames) {
        FVector Sum = FVector::ZeroVector;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Sum += F->Acceleration;
                ++Count;
            }
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : FVector::ZeroVector;
    }

    // Get Max jerk over last N frames
    template <typename BufferType> static float GetMaxJerkMagnitude(const BufferType& Buffer, int32 Frames) {
        float Max = 0.f;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Max = FMath::Max(Max, F->GetJerkMagnitude());
            }
        }
        return Max;
    }

    // Get average turn rate (deg/sec) over N frames
    template <typename BufferType> static float GetAverageTurnRate(const BufferType& Buffer, int32 Frames) {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 1; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev && Curr->DeltaTime > KINDA_SMALL_NUMBER) {
                float DeltaYaw =
                    FMath::Abs(FMath::FindDeltaAngleDegrees(Curr->GetRotation().Yaw, Prev->GetRotation().Yaw));
                Sum += DeltaYaw / Curr->DeltaTime;
                ++Count;
            }
        }
        return Count > 0 ? Sum / Count : 0.f;
    }

    // Get average alignment (velocity-to-facing) over N frames
    template <typename BufferType> static float GetAverageInputAlignment(const BufferType& Buffer, int32 Frames) {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                FVector VelNorm = F->Velocity.GetSafeNormal();
                FVector Forward = F->GetForward();
                if (!VelNorm.IsNearlyZero() && !Forward.IsNearlyZero()) {
                    Sum += FVector::DotProduct(VelNorm, Forward);
                    ++Count;
                }
            }
        }
        return Count > 0 ? Sum / Count : 0.f;
    }

    // Get rolling curvature (turn per unit distance)
    template <typename BufferType> static float GetRollingCurvature(const BufferType& Buffer, int32 Frames) {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 1; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                float Dist = FVector::Dist(Curr->GetPosition(), Prev->GetPosition());
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero() && Dist > KINDA_SMALL_NUMBER) {
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(DirA, DirB)));
                    Sum += Angle / Dist;
                    ++Count;
                }
            }
        }
        return Count > 0 ? Sum / Count : 0.f;
    }

    // Get peak (max) acceleration over last N frames
    template <typename BufferType> static float GetPeakAcceleration(const BufferType& Buffer, int32 Frames) {
        float Max = 0.f;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Max = FMath::Max(Max, F->GetAccelerationMagnitude());
            }
        }
        return Max;
    }

    // Get total distance traveled over last N frames
    template <typename BufferType> static float GetDistanceTraveled(const BufferType& Buffer, int32 Frames) {
        float Dist = 0.f;
        FVector PrevPos;
        bool bFirst = true;
        for (int32 i = FMath::Min(Frames, Buffer.NumFrames()) - 1; i >= 0; --i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                FVector Pos = F->GetPosition();
                if (!bFirst)
                    Dist += FVector::Dist(Pos, PrevPos);
                bFirst = false;
                PrevPos = Pos;
            }
        }
        return Dist;
    }

    template <typename BufferType>
    static bool WasMovingTowardDirection(const BufferType& Buffer, const FVector& WorldDirection,
                                         float AngleToleranceDeg = 30.f, int32 FramesToCheck = 10) {
        FVector TargetDir = WorldDirection.GetSafeNormal();
        for (int32 i = 0; i < FMath::Min(FramesToCheck, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                FVector VelDir = F->Velocity.GetSafeNormal();
                if (!VelDir.IsNearlyZero()) {
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(VelDir, TargetDir)));
                    if (Angle < AngleToleranceDeg)
                        return true;
                }
            }
        }
        return false;
    }

    // Rolling average (useful for smoothing, predictive filters)
    template <typename BufferType, typename Getter>
    static float GetRollingAverage(const BufferType& Buffer, Getter GetValue, int32 Frames) {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                Sum += GetValue(*F);
                ++Count;
            }
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : 0.f;
    }

    // Rolling min/max
    template <typename BufferType, typename Getter>
    static float GetRollingMin(const BufferType& Buffer, Getter GetValue, int32 Frames) {
        float MinValue = TNumericLimits<float>::Max();
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                MinValue = FMath::Min(MinValue, GetValue(*F));
            }
        }
        return MinValue;
    }

    template <typename BufferType, typename Getter>
    static float GetRollingMax(const BufferType& Buffer, Getter GetValue, int32 Frames) {
        float MaxValue = TNumericLimits<float>::Lowest();
        for (int32 i = 0; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            if (const FOHMovementHistoryFrame* F = Buffer.GetLatest(i)) {
                MaxValue = FMath::Max(MaxValue, GetValue(*F));
            }
        }
        return MaxValue;
    }

    // Predict next velocity (basic Euler)
    template <typename BufferType> static FVector PredictNextVelocity(const BufferType& Buffer) {
        if (Buffer.NumFrames() < 2)
            return FVector::ZeroVector;
        const FOHMovementHistoryFrame* Last = Buffer.GetLatest(0);
        const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(1);
        if (Last && Prev)
            return Last->Velocity + Last->Acceleration * Last->DeltaTime;
        return FVector::ZeroVector;
    }

    // Detect recent pivot/shuffle (large change in input or velocity direction)
    template <typename BufferType>
    static bool WasRecentPivot(const BufferType& Buffer, float InputAngleThreshold = 90.f,
                               float VelocityAngleThreshold = 45.f, int32 FramesToCheck = 5) {
        for (int32 i = 1; i < FMath::Min(FramesToCheck, Buffer.NumFrames()); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                // Input pivot
                FVector2D CurrInput = Curr->InputVector.GetSafeNormal();
                FVector2D PrevInput = Prev->InputVector.GetSafeNormal();
                if (!CurrInput.IsNearlyZero() && !PrevInput.IsNearlyZero()) {
                    float Dot = FVector2D::DotProduct(CurrInput, PrevInput);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(Dot));
                    if (Angle > InputAngleThreshold)
                        return true;
                }
                // Velocity pivot
                FVector CurrVel = Curr->Velocity.GetSafeNormal();
                FVector PrevVel = Prev->Velocity.GetSafeNormal();
                if (!CurrVel.IsNearlyZero() && !PrevVel.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(CurrVel, PrevVel);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(Dot));
                    if (Angle > VelocityAngleThreshold)
                        return true;
                }
            }
        }
        return false;
    }
    // Returns frame index (0 = newest) if a pivot > AngleThresholdDeg occurred within TimeWindowSec
    template <typename BufferType>
    static int32 FindRecentPivotWithinTime(const BufferType& Buffer, float AngleThresholdDeg = 45.f,
                                           float TimeWindowSec = 0.25f) {
        float AccumTime = 0.f;
        FVector LastDir = FVector::ZeroVector;
        for (int32 i = 1; i < Buffer.NumFrames(); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                AccumTime += Curr->DeltaTime;
                if (AccumTime > TimeWindowSec)
                    break;

                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(DirA, DirB);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
                    if (Angle > AngleThresholdDeg)
                        return i - 1; // Found recent pivot frame
                }
            }
        }
        return -1;
    }

    template <typename BufferType>
    static bool DetectMicroStutter(const BufferType& Buffer, float MinAngle = 10.f, int32 MinStutters = 2,
                                   float TimeWindowSec = 0.2f) {
        int32 StutterCount = 0;
        float AccumTime = 0.f;

        for (int32 i = 1; i < Buffer.NumFrames(); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                AccumTime += Curr->DeltaTime;
                if (AccumTime > TimeWindowSec)
                    break;

                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(DirA, DirB);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
                    if (Angle > MinAngle)
                        StutterCount++;
                }
            }
        }
        return StutterCount >= MinStutters;
    }

    template <typename BufferType>
    static bool DetectInputWobble(const BufferType& Buffer, float MinAngle = 30.f, int32 MinWobbles = 2,
                                  float TimeWindowSec = 0.2f) {
        int32 WobbleCount = 0;
        float AccumTime = 0.f;

        for (int32 i = 1; i < Buffer.NumFrames(); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                AccumTime += Curr->DeltaTime;
                if (AccumTime > TimeWindowSec)
                    break;

                FVector2D DirA = Curr->InputVector.GetSafeNormal();
                FVector2D DirB = Prev->InputVector.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Dot = FVector2D::DotProduct(DirA, DirB);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
                    if (Angle > MinAngle)
                        WobbleCount++;
                }
            }
        }
        return WobbleCount >= MinWobbles;
    }

    template <typename BufferType>
    static float GetLastPivotAngle(const BufferType& Buffer, float MinAngle = 30.f, float TimeWindowSec = 0.3f) {
        float AccumTime = 0.f;
        for (int32 i = 1; i < Buffer.NumFrames(); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                AccumTime += Curr->DeltaTime;
                if (AccumTime > TimeWindowSec)
                    break;

                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Dot = FVector::DotProduct(DirA, DirB);
                    float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
                    if (Angle > MinAngle)
                        return Angle;
                }
            }
        }
        return 0.f; // No significant pivot found
    }
    template <typename BufferType>
    static float GetAverageDirectionChangeRateDegPerSec(const BufferType& Buffer, int32 Frames) {
        float SumDeg = 0.f;
        float TotalTime = 0.f;
        for (int32 i = 1; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                float DeltaT = Curr->DeltaTime;
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero() && DeltaT > KINDA_SMALL_NUMBER) {
                    float Angle =
                        FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(DirA, DirB), -1.f, 1.f)));
                    SumDeg += Angle;
                    TotalTime += DeltaT;
                }
            }
        }
        return TotalTime > 0.f ? SumDeg / TotalTime : 0.f;
    }
    template <typename BufferType> static float GetDirectionChangeVarianceDeg(const BufferType& Buffer, int32 Frames) {
        TArray<float> AngleDeltas;
        float Mean = 0.f;
        for (int32 i = 1; i < FMath::Min(Frames, Buffer.NumFrames()); ++i) {
            const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(i - 1);
            const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(i);
            if (Curr && Prev) {
                FVector DirA = Curr->Velocity.GetSafeNormal();
                FVector DirB = Prev->Velocity.GetSafeNormal();
                if (!DirA.IsNearlyZero() && !DirB.IsNearlyZero()) {
                    float Angle =
                        FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(DirA, DirB), -1.f, 1.f)));
                    AngleDeltas.Add(Angle);
                    Mean += Angle;
                }
            }
        }
        if (AngleDeltas.Num() == 0)
            return 0.f;
        Mean /= AngleDeltas.Num();
        float Variance = 0.f;
        for (float V : AngleDeltas)
            Variance += FMath::Square(V - Mean);
        return Variance / AngleDeltas.Num();
    }

    // Returns ramp factor (0 = hard pivot, 1 = aligned), can be used to scale speed or acceleration
    template <typename BufferType>
    static float GetDirectionalRampFactor(const BufferType& Buffer, int32 FramesToCheck = 2,
                                          float MaxSlowAngle = 120.f) {
        if (Buffer.NumFrames() < FramesToCheck)
            return 1.f;
        const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(0);
        const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(1);
        if (Curr && Prev) {
            FVector PrevDir = Prev->Velocity.GetSafeNormal();
            FVector CurrInputDir(Curr->InputVector, 0.f);
            CurrInputDir.Normalize();
            if (!PrevDir.IsNearlyZero() && !CurrInputDir.IsNearlyZero()) {
                float Dot = FVector::DotProduct(PrevDir, CurrInputDir);
                float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.f, 1.f)));
                // Linear map: 0° → 1, MaxSlowAngle° → 0
                return FMath::Clamp(1.f - (Angle / MaxSlowAngle), 0.f, 1.f);
            }
        }
        return 1.f;
    }

    // Axis-aware ramp: fwd/back/side
    template <typename BufferType>
    static float GetAxisWeightedRamp(const BufferType& Buffer, float FwdWeight = 1.0f, float SideWeight = 0.6f,
                                     float BackWeight = 0.3f) {
        if (Buffer.NumFrames() < 2)
            return 1.f;
        const FOHMovementHistoryFrame* Curr = Buffer.GetLatest(0);
        const FOHMovementHistoryFrame* Prev = Buffer.GetLatest(1);
        if (Curr && Prev) {
            FVector PrevDir = Prev->Velocity.GetSafeNormal();
            FVector CurrInputDir(Curr->InputVector, 0.f);
            CurrInputDir.Normalize();
            if (!PrevDir.IsNearlyZero() && !CurrInputDir.IsNearlyZero()) {
                float Angle = FMath::RadiansToDegrees(
                    FMath::Acos(FMath::Clamp(FVector::DotProduct(PrevDir, CurrInputDir), -1.f, 1.f)));
                // Forward (<45°), Side (45–135°), Back (>135°)
                if (Angle < 45.f)
                    return FwdWeight;
                else if (Angle < 135.f)
                    return SideWeight;
                else
                    return BackWeight;
            }
        }
        return 1.f;
    }
};

// Enhanced foot data with kinematic chain tracking
USTRUCT(BlueprintType)
struct FOHKinematicChainData {
    GENERATED_BODY()

    // Bone transforms
    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform FootTransform;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform AnkleTransform;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform KneeTransform;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform HipTransform;

    // Bone orientations
    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    float FootPitch = 0.0f; // Toe up/down

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    float FootRoll = 0.0f; // Inside/outside edge

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    float KneeAngle = 0.0f; // Flexion angle

    // Chain metrics
    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    float LegLength = 0.0f; // Current hip-to-foot distance

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    float LegCompression = 0.0f; // 0=extended, 1=fully compressed

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FVector KneeDirection; // Which way knee points

    // Sliding detection
    UPROPERTY(BlueprintReadOnly, Category = "Sliding")
    bool bIsSliding = false;

    UPROPERTY(BlueprintReadOnly, Category = "Sliding")
    float SlideVelocity = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Sliding")
    FVector SlideDirection;

    UPROPERTY(BlueprintReadOnly, Category = "Sliding")
    float AccumulatedSlideDistance = 0.0f;

    // Clipping detection
    UPROPERTY(BlueprintReadOnly, Category = "Clipping")
    bool bIsClipping = false;

    UPROPERTY(BlueprintReadOnly, Category = "Clipping")
    float ClipDepth = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Clipping")
    TArray<FName> ClippingBones;

    // Environment awareness
    UPROPERTY(BlueprintReadOnly, Category = "Environment")
    FVector SurfaceNormal = FVector::UpVector;

    UPROPERTY(BlueprintReadOnly, Category = "Environment")
    float SurfaceAngle = 0.0f; // Angle from horizontal

    UPROPERTY(BlueprintReadOnly, Category = "Environment")
    TEnumAsByte<EPhysicalSurface> SurfaceType = EPhysicalSurface::SurfaceType_Default;

    UPROPERTY(BlueprintReadOnly, Category = "Environment")
    bool bOnEdge = false; // Near a ledge

    // Rest pose reference
    float RestLegLength = 0.0f;
    FVector PlantedPosition; // Position when last planted
    float TimeSincePlanted = 0.0f;
    static constexpr int32 KinematicHistoryCapacity = 10; // Adjust as needed

    // In FOHKinematicChainData:
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> LegLengthHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> LegCompressionHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> FootPitchHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> FootRollHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> KneeAngleHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> SurfaceAngleHistory;
    OHSafeMapUtils::TRollingBuffer<float, KinematicHistoryCapacity> SlideVelocityHistory;
    OHSafeMapUtils::TRollingBuffer<FVector, KinematicHistoryCapacity> FootPositionHistory;

    void AddHistorySample() {
        LegLengthHistory.Add(LegLength);
        LegCompressionHistory.Add(LegCompression);
        FootPitchHistory.Add(FootPitch);
        FootRollHistory.Add(FootRoll);
        KneeAngleHistory.Add(KneeAngle);
        SurfaceAngleHistory.Add(SurfaceAngle);
        SlideVelocityHistory.Add(SlideVelocity);
        FootPositionHistory.Add(FootTransform.GetLocation());
    }
};

USTRUCT(BlueprintType)
struct FOHFootStanceData {
    GENERATED_BODY()

    // Position and velocity
    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    FVector WorldPosition = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    FVector LocalPosition = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    FVector Velocity = FVector::ZeroVector;

    // Phase and timing
    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    EOHFootPhase CurrentPhase = EOHFootPhase::Contact;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float PhaseTime = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float StridePhase = 0.0f; // 0-1 normalized stride cycle

    // Ground interaction
    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float GroundDistance = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsGrounded = false;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsPivoting = false;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float WeightBearing = 0.0f; // 0-1 how much weight on this foot

    // Movement analysis
    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsForward = false; // Relative to movement direction

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsDriving = false; // Is this foot driving movement

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float MovementContribution = 0.0f; // How much this foot contributes to movement

    // Chain data for bone FK/IK etc.
    FOHKinematicChainData ChainData;

    // === Rolling History Buffers ===
    // Use 16 or 32 based on your smoothing needs
    OHSafeMapUtils::TRollingBuffer<FVector, 16> PositionHistory;
    OHSafeMapUtils::TRollingBuffer<float, 16> PhaseHistory;

    // --- Utility for easy adding, replaces old UpdateHistory ---
    void AddHistorySample(const FVector& NewPos, float NewPhase) {
        PositionHistory.Add(NewPos);
        PhaseHistory.Add(NewPhase);
    }

    // Example of a smoothing function using the buffer
    FVector GetSmoothedPosition(int32 NumFrames = 5) const {
        FVector Sum = FVector::ZeroVector;
        int32 Count = FMath::Min(NumFrames, PositionHistory.NumFrames());
        for (int32 i = 0; i < Count; ++i) {
            Sum += PositionHistory.GetLatest(i) ? *PositionHistory.GetLatest(i) : FVector::ZeroVector;
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : WorldPosition;
    }

    // Get average phase (for smoothing transitions)
    float GetAveragePhase(int32 NumFrames = 5) const {
        float Sum = 0.f;
        int32 Count = FMath::Min(NumFrames, PhaseHistory.NumFrames());
        for (int32 i = 0; i < Count; ++i) {
            Sum += PhaseHistory.GetLatest(i) ? *PhaseHistory.GetLatest(i) : 0.f;
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : StridePhase;
    }
    void ClearHistory() {
        PositionHistory.Num = 0;
        PositionHistory.Start = 0;
        PhaseHistory.Num = 0;
        PhaseHistory.Start = 0;
    }
};

// Overall stance data
USTRUCT(BlueprintType)
struct FOHStanceState {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    EOHStanceType CurrentStance = EOHStanceType::Neutral;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float StrideCadence = 0.0f; // Steps per second

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float StrideLength = 0.0f; // Distance between steps

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float StanceWidth = 0.0f; // Lateral distance between feet

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    FVector CenterOfGravity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    FVector WeightDistribution = FVector::ZeroVector; // X=forward/back, Y=left/right, Z=up/down

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsStable = true;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float StabilityScore = 1.0f; // 0-1 how stable the stance is

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    bool bIsPivoting = false;

    UPROPERTY(BlueprintReadOnly, Category = "Stance")
    float PivotAngle = 0.0f;
    static constexpr int32 StanceHistoryCapacity = 20;

    OHSafeMapUtils::TRollingBuffer<float, StanceHistoryCapacity> StrideLengthHistory;
    OHSafeMapUtils::TRollingBuffer<float, StanceHistoryCapacity> StrideCadenceHistory;
    OHSafeMapUtils::TRollingBuffer<float, StanceHistoryCapacity> StabilityScoreHistory;

    void AddHistorySample() {
        StrideLengthHistory.Add(StrideLength);
        StrideCadenceHistory.Add(StrideCadence);
        StabilityScoreHistory.Add(StabilityScore);
    }
    float GetAverageStrideLength(int32 Frames = 10) const {
        float Sum = 0.f;
        int32 Count = 0;
        for (int32 i = 0; i < FMath::Min(Frames, StrideLengthHistory.NumFrames()); ++i) {
            Sum += StrideLengthHistory.GetLatest(i) ? *StrideLengthHistory.GetLatest(i) : 0.f;
            ++Count;
        }
        return Count > 0 ? Sum / static_cast<float>(Count) : 0.f;
    }
};

USTRUCT(BlueprintType)
struct FOHMovementConfig {
    GENERATED_BODY()

    // ==== USER-FACING PROPERTIES (UPROPERTY) ====
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    bool bUseMomentumBasedMovement = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    bool bUseInputMagnitudeForSpeed = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Debug")
    bool bEnableDebugOutput = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    UCurveVector* MovementCurve = nullptr; // X = input, Y/Z/W = walk/run/sprint

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    UCurveFloat* DirectionalSpeedCurve = nullptr; // For strafing/backpedal

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    UCurveFloat* RotationalSpeedCurve = nullptr; // For turn-in-place or pivots

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    float SpeedFactor = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float FrictionControl = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    float GaitFlexibility = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Behavior")
    float InputResponsiveness = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Input")
    float JoystickSensitivity = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Scaling")
    float OverallMaxSpeed = 600.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float MovementWeight = 0.9f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float InertiaFactor = 0.5f;

    // ---- SUGGESTED NEW USER FIELDS ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float CharacterMass = 80.f; // (kg) Used for force/accel calcs, defaults to human mass

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Curves")
    UCurveFloat* AccelerationCurve = nullptr; // (Optional: modulate accel based on input/gait)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Curves")
    UCurveFloat* BrakingCurve = nullptr; // (Optional: for designer braking profile)

    // Rotation Parameters

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    bool bUseRotationBlendedMovementInput = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float RotationBlendedMovementWeight = 0.3f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float DeadzoneAngle = 10.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float MaxOffsetAngle = 22.5f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float PullStrength = 9.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Rotation")
    float HardSnapStrength = 20.0f;

    // ======== Accessors ========
    FORCEINLINE float GetMaxWalkSpeed() const {
        return MaxWalkSpeed;
    }
    FORCEINLINE float GetMaxRunSpeed() const {
        return MaxRunSpeed;
    }
    FORCEINLINE float GetMaxSprintSpeed() const {
        return MaxSprintSpeed;
    }
    FORCEINLINE float GetWalkSpeedThreshold() const {
        return WalkSpeedThreshold;
    }
    FORCEINLINE float GetRunSpeedThreshold() const {
        return RunSpeedThreshold;
    }
    FORCEINLINE float GetSprintSpeedThreshold() const {
        return SprintSpeedThreshold;
    }
    FORCEINLINE float GetMaxAccelerationWalking() const {
        return MaxAccelerationWalking;
    }
    FORCEINLINE float GetMaxAccelerationRunning() const {
        return MaxAccelerationRunning;
    }
    FORCEINLINE float GetMaxAccelerationSprinting() const {
        return MaxAccelerationSprinting;
    }
    FORCEINLINE float GetBrakingDecelerationWalking() const {
        return BrakingDecelerationWalking;
    }
    FORCEINLINE float GetBrakingDecelerationRunning() const {
        return BrakingDecelerationRunning;
    }
    FORCEINLINE float GetBrakingDecelerationSprinting() const {
        return BrakingDecelerationSprinting;
    }
    FORCEINLINE float GetGroundFrictionWalking() const {
        return GroundFrictionWalking;
    }
    FORCEINLINE float GetGroundFrictionRunning() const {
        return GroundFrictionRunning;
    }
    FORCEINLINE float GetGroundFrictionSprinting() const {
        return GroundFrictionSprinting;
    }
    FORCEINLINE float GetMaxFriction() const {
        return MaxFriction;
    }
    FORCEINLINE float GetMinFriction() const {
        return MinFriction;
    }
    FORCEINLINE float GetNoInputSpeedDecayRate() const {
        return NoInputSpeedDecayRate;
    }
    FORCEINLINE float GetInputSpeedBuildRate() const {
        return InputSpeedBuildRate;
    }
    FORCEINLINE float GetSpeedRetentionOnDirectionChange() const {
        return SpeedRetentionOnDirectionChange;
    }
    FORCEINLINE float GetStrafingThreshold() const {
        return StrafingThreshold;
    }
    FORCEINLINE float GetDirectionChangeSharpness() const {
        return DirectionChangeSharpness;
    }
    FORCEINLINE float GetPivotThreshold() const {
        return PivotThreshold;
    }
    FORCEINLINE float GetGaitSpeedTransitionRate() const {
        return GaitSpeedTransitionRate;
    }
    FORCEINLINE float GetGaitAccelerationMultiplier() const {
        return GaitAccelerationMultiplier;
    }
    FORCEINLINE float GetGaitDecelerationMultiplier() const {
        return GaitDecelerationMultiplier;
    }
    FORCEINLINE float GetRunHysteresis() const {
        return RunHysteresis;
    }
    FORCEINLINE float GetSprintHysteresis() const {
        return SprintHysteresis;
    }
    FORCEINLINE float GetInputBufferRiseRate() const {
        return InputBufferRiseRate;
    }
    FORCEINLINE float GetInputBufferDecay() const {
        return InputBufferDecay;
    }
    FORCEINLINE float GetHoldTimeAboveRunThreshold() const {
        return HoldTimeAboveRunThreshold;
    }
    FORCEINLINE float GetHoldTimeAboveSprintThreshold() const {
        return HoldTimeAboveSprintThreshold;
    }
    FORCEINLINE float GetGaitInterpSpeed() const {
        return GaitInterpSpeed;
    }
    FORCEINLINE float GetBackpedalSpeedMultiplier() const {
        return BackpedalSpeedMultiplier;
    }
    FORCEINLINE float GetStrafeSpeedMultiplier() const {
        return StrafeSpeedMultiplier;
    }
    FORCEINLINE float GetDiagonalSpeedMultiplier() const {
        return DiagonalSpeedMultiplier;
    }
    FORCEINLINE float GetJoystickDeadzone() const {
        return JoystickDeadzone;
    }
    FORCEINLINE float GetJoystickWalkZone() const {
        return JoystickWalkZone;
    }
    FORCEINLINE float GetJoystickRunZone() const {
        return JoystickRunZone;
    }
    FORCEINLINE float GetInputSmoothingBlendSpeed() const {
        return InputSmoothingBlendSpeed;
    }
    FORCEINLINE float GetInputIntentMagnitudeThreshold() const {
        return InputIntentMagnitudeThreshold;
    }
    FORCEINLINE float GetMomentumBasedMovement() const {
        return bUseMomentumBasedMovement;
    }
    FORCEINLINE float GetMomentumBasedMovementWeight() const {
        return MovementWeight;
    }
    // ==== PRIVATE DERIVED FIELDS ====
  private:
    float MaxWalkSpeed = 0.f;
    float MaxRunSpeed = 0.f;
    float MaxSprintSpeed = 0.f;

    float WalkSpeedThreshold = 0.f;
    float RunSpeedThreshold = 0.f;
    float SprintSpeedThreshold = 0.f;

    float MaxAccelerationWalking = 0.f;
    float MaxAccelerationRunning = 0.f;
    float MaxAccelerationSprinting = 0.f;

    float BrakingDecelerationWalking = 0.f;
    float BrakingDecelerationRunning = 0.f;
    float BrakingDecelerationSprinting = 0.f;

    float GroundFrictionWalking = 0.f;
    float GroundFrictionRunning = 0.f;
    float GroundFrictionSprinting = 0.f;

    float MaxFriction = 0.f;
    float MinFriction = 0.f;

    float NoInputSpeedDecayRate = 0.f;
    float InputSpeedBuildRate = 0.f;

    float SpeedRetentionOnDirectionChange = 0.f;
    float StrafingThreshold = 0.f;
    float DirectionChangeSharpness = 0.f;
    float PivotThreshold = 0.f;

    float GaitSpeedTransitionRate = 0.f;
    float GaitAccelerationMultiplier = 0.f;
    float GaitDecelerationMultiplier = 0.f;
    float RunHysteresis = 0.f;
    float SprintHysteresis = 0.f;

    float InputBufferRiseRate = 0.f;
    float InputBufferDecay = 0.f;
    float HoldTimeAboveRunThreshold = 0.f;
    float HoldTimeAboveSprintThreshold = 0.f;

    float GaitInterpSpeed = 0.f;
    float BackpedalSpeedMultiplier = 0.f;
    float StrafeSpeedMultiplier = 0.f;
    float DiagonalSpeedMultiplier = 0.f;

    float JoystickDeadzone = 0.f;
    float JoystickWalkZone = 0.f;
    float JoystickRunZone = 0.f;

    float InputSmoothingBlendSpeed = 0.f;
    float InputIntentMagnitudeThreshold = 0.f;

  public:
    // --------- Update function -----------
    void Update(float DeltaTime = 1.f / 60.f);
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGaitChanged, EOHGait, NewGait);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMovementInputUpdated, FVector2D, MovementInput);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPhysicsInputReceived, FVector2D, PhysicsInput, FVector2D,
                                             EffectiveMovement);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPushbackReceived, const FVector2D&, PushbackVector, float, Magnitude);

/**
 * Custom movement component for character control in a game.
 */
UCLASS(Blueprintable, BlueprintType, ClassGroup = (State), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHMovementComponent : public UActorComponent {
    GENERATED_BODY()

  public:
    UOHMovementComponent();

    /** Direct reference to the movement configuration */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Config")
    FOHMovementConfig MovementConfig;

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Config")
    bool bVerboseLogging = true;

    // References
    UPROPERTY(BlueprintReadWrite, Category = "OH|Target")
    mutable AActor* TargetActor = nullptr;

    UFUNCTION(BlueprintCallable, Category = "OH|Target")
    void FindTargetActor();
    ACharacter* FindTargetCharacter();

    UFUNCTION(BlueprintCallable, Category = "OH|Target")
    void SetTargetActor(AActor* NewTarget);

    // === Runtime State ===
    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentMomentumSpeed = 0.f; // Current momentum-based speed

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float DirectionalSpeedMultiplier = 1.0f; // Current direction-based speed modifier

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float RotationalSpeedMultiplier = 1.0f; // Current rotation-based speed modifier

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentMovementAngle = 0.f; // Angle between movement and facing direction

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    FVector LastMovementDirection = FVector::ZeroVector; // For direction change detection

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentAngularVelocity = 0.f; // Current turning speed

    UPROPERTY(BlueprintReadWrite, Category = "OH|Initialization")
    bool bMovementInitialized = false; // Track whether movement has been initialized

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float CurrentTargetSpeed = 0.f; // Target speed based on gait

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    float SmoothedTargetSpeed = 0.f; // Smoothly interpolated target speed

    UPROPERTY(BlueprintReadOnly, Category = "OH|Runtime|State")
    EOHGait PreviousGait = EOHGait::Walking; // Track gait changes

    // --- Movement Input ---
    // Apply movement input to CharacterMovementComponent
    UFUNCTION()
    void ApplyMovementInput();

    // Default version: always uses CalculateFacingRotation
    UFUNCTION()
    void ApplyRotation(float DeltaTime, bool bBlend = true, float BlendSpeed = 10.f);

    // Overload: direct application
    void ApplyRotation(const FRotator& TargetRotation, bool bBlend, float BlendSpeed, float DeltaTime);

    // Calculate movement input vector from input magnitude and direction
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FVector CalculateMovementInputVector() const;

    // --- Movement State ---
    // Calculate control rotation based on Camera Perspective
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FRotator CalculateControlRotation() const;

    // Calculate control rotation based on Camera Perspective
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    FRotator CalculateFacingRotation(float DeltaTime) const;

    UFUNCTION(BlueprintPure, Category = "Movement|Collision")
    FVector2D GetEffectiveMovementInput() const;

    UFUNCTION(BlueprintCallable, Category = "Movement|Input")
    void ApplyPhysicsImpulse(const FVector& WorldImpulse, bool bIsCombatHit);

    // --- Movement Settings ---
    // Calculate movement settings based on current gait
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void CalculateDirectionalSpeedModifiers(ACharacter* OwnerChar);

    // Update the HandlePACPushback declaration to ensure it's properly exposed
    UFUNCTION(BlueprintCallable, Category = "Movement|Combat")
    void HandlePACPushback(FName BoneName, const FVector2D& PushbackVector, float ImpactForce);
    FVector2D ConvertImpulseToInput(const FVector& WorldImpulse) const;

    // Movement input vector from external input sources (clamped 0-1)
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Input")
    FVector2D MovementInputVector;

    // Buffered smoothed input magnitude and speed for intent detection
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Buffering")
    float BufferedInputAmount;

    // Buffered smoothed speed for intent detection
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Buffering")
    float BufferedSpeed;

    // Current interpolated gait state and blend alpha for smooth transitions
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Gait")
    EOHGait CurrentGait;

    // Current interpolated gait state and blend alpha for smooth transitions
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Gait")
    float GaitBlendAlpha;

    // --- Movement State ---
    // Current movement speed (used to Calculate Gait)
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    float CurrentSpeed;

    // Movement play rate for animation
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    float MovementPlayRate;

    // Input intent magnitude from input panel (0 to 1), used in calculations
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|Input")
    float InputIntentMagnitude;

    // Movement mode tracking from CharacterMovementComponent
    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    TEnumAsByte<EMovementMode> CurrentMovementMode;

    UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
    uint8 CurrentCustomMovementMode;

    // Public setter to update input vector (to be called from player controller or input system)
    UFUNCTION(BlueprintCallable, Category = "OH|Movement|Input")
    void SetMovementInputVector(const FVector2D& NewInput);

    // Desired gait calculated from input and speed
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    EOHGait CalculateDesiredGait(float Deltatime);

    // Allowed gait from external constraints (e.g., stamina, crouching)
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    EOHGait GetAllowedGait() const;

    // Actual gait combining desired and allowed gait
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    static EOHGait GetActualGait(EOHGait DesiredGait, EOHGait AllowedGait);

    // Smooth interpolation of gait changes
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void InterpolateGait(EOHGait TargetGait, float DeltaTime);

    // Update movement state per tick
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateMovement(float DeltaTime);

    // Update input buffering values from input vector
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateInputBuffers(float DeltaTime);

    // Updates character movement component properties dynamically based on allowed gait
    UFUNCTION(BlueprintCallable, Category = "OH|Movement")
    void UpdateDynamicMovementSettings(EOHGait AllowedGait);

    // Returns normalized speed [0-1] for curve evaluation
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float GetMappedSpeed() const;

    // Calculates current horizontal movement speed ignoring vertical velocity
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementSpeed() const;

    // Calculates input magnitude (0-1) from input vector
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementInputAmount() const;

    // Calculates movement play rate (for animation speed scaling) based on speed and input
    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    float CalculateMovementPlayRate(float Speed, float InputAmount) const;

    UFUNCTION(BlueprintPure, Category = "OH|Movement")
    static bool ShouldRotateToTarget();

    UFUNCTION(BlueprintPure, Category = "OH|Movement|Direction")
    float GetInputWorldAngleDegrees() const;

    // Calculates a custom rotation looking from owner to locked target (implement in cpp)
    UFUNCTION(BlueprintPure, Category = "OH|Movement|Pivot")
    FRotator CalculateRotationToTarget_Internal() const;

    // Calculates the offset rotator between owner rotation and target rotation (implement in cpp)
    UFUNCTION(BlueprintPure, Category = "OH|Movement|Pivot")
    FRotator CalculateTargetOffset_Internal() const;

    // Movement mode changed handler, to be called when CharacterMovement changes movement mode
    UFUNCTION()
    void HandleMovementModeChanged(EMovementMode NewMovementMode, uint8 NewCustomMode);

    // Returns whether the character should perform a pivot based on the pivot threshold
    UFUNCTION(BlueprintCallable, Category = "OH|Movement|Pivot")
    bool ShouldPivot_Internal() const;

    UFUNCTION(BlueprintPure, Category = "OH|Movement|AI")
    bool GetIsPlayerControlled() const;
#pragma region Delegates
    // Input delegate handlers (bind these in your player controller)
    UFUNCTION()
    void OnReceiveMoveForward(float ForwardValue);

    UFUNCTION()
    void OnReceiveMoveRight(float RightValue);

    UFUNCTION()
    void OnReceiveMoveVector(float ForwardValue, float RightValue);

    // Delegate called when gait changes
    UPROPERTY(BlueprintAssignable, Category = "OH|Movement|Gait")
    FOnGaitChanged OnGaitChanged;

    // Delegate called when movement input updates
    UPROPERTY(BlueprintAssignable, Category = "OH|Movement|Input")
    FOnMovementInputUpdated OnMovementInputUpdated;

    UPROPERTY(BlueprintAssignable, Category = "Movement|Events")
    FOnPushbackReceived OnPushbackReceived;

#pragma endregion

  private:
    // Buffered timers to track intent hold durations
    float BufferedTimeAboveRunThreshold;
    float BufferedTimeAboveSprintThreshold;

    // Cached axis values
    float CurrentForwardInput;
    float CurrentRightInput;

    bool bIsPlayerControlled = false;

    // Helper to get locked target actor (implement this to suit your locking logic)
    static AActor* GetLockedTargetActor();

    // --- For Debug Input Reception ---
    FVector2D LastReceivedInputVector;
    float LastInputDebugTimestamp = 0.f;
    bool bLastInputIntentValid = false;
    float LastBufferedInputAmount = 0.f;
    float LastCalculatedSpeed = 0.f;

    // --- Stance Tracking ---
#pragma region StanceTracking
  public:
    // Configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    bool bEnableStanceTracking = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float StanceUpdateRate = 60.0f; // Updates per second

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float FootTraceDistance = 100.0f; // How far to trace for ground

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float GroundedThreshold = 10.0f; // Distance to consider foot grounded

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float PivotVelocityThreshold = 50.0f; // Below this velocity, foot is pivoting

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Config")
    float PhaseTransitionSpeed = 5.0f; // How fast phases transition

    // Runtime data
    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHFootStanceData LeftFootData = FOHFootStanceData();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHFootStanceData RightFootData = FOHFootStanceData();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    FOHStanceState StanceState = FOHStanceState();

    UPROPERTY(BlueprintReadOnly, Category = "Stance|State")
    bool bLeftFootDominant = true;

    // Bone names
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftFootBoneName = "ball_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightFootBoneName = "ball_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName PelvisBoneName = "pelvis";

    // Additional bone names for kinematic chain
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftAnkleBoneName = "foot_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightAnkleBoneName = "foot_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftKneeBoneName = "calf_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightKneeBoneName = "calf_r";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName LeftHipBoneName = "thigh_l";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Setup")
    FName RightHipBoneName = "thigh_r";

    // Sliding detection thresholds
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Sliding")
    float FootSlideThreshold = 5.0f; // cm/s when planted

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Sliding")
    float MaxAllowedSlideDistance = 10.0f; // cm before correction

    // Clipping detection
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Clipping")
    float LegSeparationMin = 15.0f; // Minimum distance between feet

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Clipping")
    float KneeCollisionRadius = 8.0f; // Knee capsule radius

    // Environment detection
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Environment")
    float EdgeDetectionDistance = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Environment")
    float SurfaceTraceExtension = 30.0f; // Extra trace beyond foot

    // Debug
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Debug")
    bool bDebugStanceTracking = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance|Debug")
    float DebugDrawDuration = 0.0f; // 0 = single frame

    // Functions
    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateStanceTracking(float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateFootData(FOHFootStanceData& FootData, const FName& BoneName, bool bIsLeftFoot, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    EOHFootPhase CalculateFootPhase(const FOHFootStanceData& FootData, float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void UpdateStanceState(float DeltaTime);

    UFUNCTION(BlueprintCallable, Category = "Stance")
    void CalculateWeightDistribution();

    UFUNCTION(BlueprintPure, Category = "Stance")
    bool IsFootForward(const FOHFootStanceData& FootData) const;

    UFUNCTION(BlueprintPure, Category = "Stance")
    float GetStrideCadence() const {
        return StanceState.StrideCadence;
    }

    UFUNCTION(BlueprintPure, Category = "Stance")
    EOHStanceType GetCurrentStance() const {
        return StanceState.CurrentStance;
    }

    UFUNCTION(BlueprintPure, Category = "Stance")
    bool IsStancePivoting() const {
        return StanceState.bIsPivoting;
    }

  private:
    // Internal helpers
    FVector GetBoneWorldLocation(const FName& BoneName) const;
    FVector GetBoneVelocity(const FName& BoneName) const;
    float TraceToGround(const FVector& StartPos) const;
    float TraceToGround(const FVector& StartPos, FHitResult& OutHit) const;
    void UpdateKinematicChain(FOHKinematicChainData& ChainData, bool bIsLeftFoot, USkeletalMeshComponent* Mesh);
    void UpdateSlidingDetection(FOHFootStanceData& FootData, float DeltaTime);
    void UpdateClippingDetection(FOHFootStanceData& FootData, FOHFootStanceData& OtherFoot);
    bool DetectNearbyEdge(const FVector& FootPos, const FVector& SurfaceNormal);
    EOHFootPhase CalculatePhaseFromKinematics(const FOHFootStanceData& FootData, float DeltaTime);
    static void CalculateWeightBearingFromCompression(FOHFootStanceData& FootData, float DeltaTime);
    void DrawDebug_MovementData(float DeltaTime);
    void PrintDebug_MovementInput();
    void PrintDebug_MovementData();
    void DrawDebug_StanceData() const;
    void DrawDebug_KinematicChainData(const FOHFootStanceData& FootData, bool bIsLeft,
                                      USkeletalMeshComponent* Mesh) const;
    void DrawDebug_SlidingIndicator(const FOHFootStanceData& FootData, const FColor& FootColor) const;
    void DrawDebug_ClippingWarning() const;
    void DrawDebug_CompressionBars() const;
    void DrawDebug_BalanceAndWeightDistribution() const;
    void DrawDebug_EnvironmentInfo(const FOHFootStanceData& FootData) const;
    void DrawDebug_FootPhases() const;
    void PrintDebug_StanceData() const;
    static FColor GetPhaseColor(EOHFootPhase Phase);
    bool CanPerformKick(bool bLeftFoot) const;
    float GetKickPower(bool bLeftFoot) const;
    bool ValidateKinematicSetup() const;

    // Timing
    float StanceUpdateAccumulator = 0.0f;

    // History tracking
    float LastLeftStepTime = 0.0f;
    float LastRightStepTime = 0.0f;
    TArray<float> RecentStepIntervals;

    // Bone location cache
    mutable TMap<FName, FVector> BoneLocationCache;
    mutable TMap<FName, FVector> LastBoneLocations;

    OHSafeMapUtils::TRollingBuffer<FOHMovementHistoryFrame, 16> MovementHistory;
#pragma endregion

#pragma region COLLISION_INTEGRATION

  public:
    // Configuration

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    bool bUseImpulseBasedPhysics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    bool bForceImpulseBasedPhysics = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float PhysicsImpulseStrength = 500.0f; // World units per second

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    float PhysicsImpulseDecayRate = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Physics")
    UCurveFloat* PhysicsImpulseDecayCurve = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    bool bAcceptPhysicsInput = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float PhysicsInputWeight = 0.3f; // How much physics affects movement vs player input

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float PhysicsInputDecayRate = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Collision")
    float MaxPhysicsInputMagnitude = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    bool bEnableProximitySpacing = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    float PersonalSpaceRadius = 80.0f; // Start pushing at this distance

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    float ProximityPushStrength = 0.5f; // How strong the push is

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing",
              meta = (ClampMin = "50.0", ClampMax = "300.0"))
    float BaseSpacingDistance = 120.0f; // Optimal fighting range

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    UCurveFloat* SpacingCurve = nullptr; // Distance to push strength mapping

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Spacing")
    bool bDebugProximitySystem = false;

    // Runtime State
    UPROPERTY(BlueprintReadOnly, Category = "Movement|Collision")
    FVector2D PhysicsInputVector = FVector2D::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Movement|Collision")
    FVector2D LastReceivedPhysicsInput = FVector2D::ZeroVector;

    UPROPERTY()
    FVector AccumulatedPhysicsImpulse;

  private:
    /**
     * Updates the physics input based on delta time.
     *
     * This method handles the old physics input system, which is a 2D vector that blends with player input.
     * It applies decay to the physics input vector and handles friction-based decay when grounded.
     * Small values are cleared from the physics input vector. Additionally, it can convert remaining physics input to
     * impulse for unified handling if bUseImpulseBasedPhysics is enabled.
     *
     * @param DeltaTime The time elapsed since the last frame in seconds.
     */
    void UpdatePhysicsInput(float DeltaTime);

    // Spacing system internals
    UPROPERTY(Transient)
    TWeakObjectPtr<ACharacter> CurrentOpponent;

    float ProximityUpdateTimer = 0.0f;

    const float ProximityUpdateInterval = 0.1f; // Update spacing every 100ms
    // Helper functions

    void CalculateSmartSpacing(ACharacter* Opponent, float& OutPushX, float& OutPushY);

    void UpdateProximitySpacing(float DeltaTime);

    void ProcessPhysicsImpulses(float DeltaTime);

    // Visualization
    TArray<FVector> PhysicsImpulseHistory;
    const int32 MaxImpulseHistorySize = 10;

  public:
#pragma endregion

    // Combat awareness
    FVector LastCombatPushback;
    float LastCombatPushbackTime = 0.0f;
    bool bCombatPushbackActive = false;

    // Add initialization function
    UFUNCTION(BlueprintCallable, Category = "Movement|Setup")
    void EnsureInitialized();

  protected:
    // Single new variable for stance state (should already exist)
    FOHStanceState CurrentStanceState = FOHStanceState();

  private:
    bool IsCharacterInCombat(ACharacter* Character, float& OutAttackConfidence, float& OutChainSpeed);

    // Combat state tracking
    float LastCombatStateTime = 0.0f;
    bool bRecentlyHit = false;
    float HitRecoveryTime = 0.0f;
};