#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "UObject/ObjectMacros.h"
#include "UObject/ScriptMacros.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Utilities/OHSafeMapUtils.h"
#include "OHPrimaryStructs.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogPACManager, Log, All);

// Forward declarations
struct FOHMotionFrameSample;
struct FOHBoneMotionData;
struct FOHCombatChainData;
struct FOHCombatAnalysis;
struct FOHKalmanState;
struct FRK4State;
struct FOHMovementHistoryFrame;
struct FOHKinematicChainData;
struct FOHFootStanceData;
struct FOHStanceState;
struct FOHBlendFloatState;
// ================================================================================
// OHPhysicsStructs.h - HIERARCHICAL MOTION TRACKING SYSTEM
// OnlyHands Project - 3-Level Motion Architecture
// ================================================================================

// ============================================================================
// Enhanced FOHMotionFrameSample - Self-Aware Motion Data Structure
// OnlyHands Project - Unreal Engine 5.3.2
//
// ENHANCEMENT SUMMARY:
// - Added comprehensive validation system with detailed error reporting
// - Implemented relative motion analysis between samples
// - Added dynamic reference space recalculation capabilities
// - Integrated debug and diagnostic self-reporting
// - Enhanced temporal analysis support for combat detection
//
// BACKWARD COMPATIBILITY:
// All existing methods preserved. New functionality accessed via new methods.
// ============================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHMotionFrameSample {
    GENERATED_BODY()

    // === CORE SPATIAL STATE (PRESERVED - NO CHANGES) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Spatial")
    FVector WorldPosition = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Spatial")
    FQuat WorldRotation = FQuat::Identity;

    // === WORLD SPACE MOTION (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|World")
    FVector WorldLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|World")
    FVector WorldLinearAcceleration = FVector::ZeroVector;

    // === LOCAL SPACE MOTION (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Local")
    FVector LocalLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Local")
    FVector LocalLinearAcceleration = FVector::ZeroVector;

    // === COMPONENT SPACE MOTION (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Component")
    FVector ComponentLinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Component")
    FVector ComponentLinearAcceleration = FVector::ZeroVector;

    // === ANGULAR MOTION (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Angular")
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Angular")
    FVector AngularAcceleration = FVector::ZeroVector;

    // === TIMING AND REFERENCE DATA (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Timing")
    float TimeStamp = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Timing")
    float DeltaTime = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Reference")
    FVector ReferenceVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Reference")
    FTransform ReferenceTransform = FTransform::Identity;

    // === COMPONENT SPACE DATA (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Transform")
    FTransform ComponentTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Transform")
    FVector ComponentPosition = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Transform")
    FQuat ComponentRotation = FQuat::Identity;

    // === SPEED PROPERTIES (PRESERVED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Speed")
    float WorldSpeed = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Speed")
    float LocalSpeed = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Speed")
    float ComponentSpeed = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Speed")
    float AngularSpeed = 0.0f;

    // === ENHANCED: SELF-AWARENESS PROPERTIES ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Validation")
    EOHReferenceSpace PrimarySpace = EOHReferenceSpace::WorldSpace;

    // ============================================================================
    // ENHANCED: CENTRALIZED VALIDATION SYSTEM
    // ============================================================================

    /**
     * Comprehensive validation with detailed error reporting
     * @param OutErrorMessage - Optional detailed error description for debugging
     * @return True if sample contains valid, usable motion data
     */
    bool IsValid(FString* OutErrorMessage = nullptr) const {
        // Validation result accumulator
        TArray<FString> ErrorMessages;
        bool bIsValidSample = true;

        // === CORE SPATIAL VALIDATION ===
        if (WorldPosition.ContainsNaN()) {
            ErrorMessages.Add(TEXT("WorldPosition contains NaN values"));
            bIsValidSample = false;
        }

        if (!WorldRotation.IsNormalized()) {
            ErrorMessages.Add(TEXT("WorldRotation is not normalized"));
            if (WorldRotation.ContainsNaN()) {
                ErrorMessages.Add(TEXT("WorldRotation contains NaN values"));
            }
            bIsValidSample = false;
        }

        // === VELOCITY VALIDATION ===
        if (WorldLinearVelocity.ContainsNaN()) {
            ErrorMessages.Add(TEXT("WorldLinearVelocity contains NaN values"));
            bIsValidSample = false;
        }

        if (LocalLinearVelocity.ContainsNaN()) {
            ErrorMessages.Add(TEXT("LocalLinearVelocity contains NaN values"));
            bIsValidSample = false;
        }

        if (ComponentLinearVelocity.ContainsNaN()) {
            ErrorMessages.Add(TEXT("ComponentLinearVelocity contains NaN values"));
            bIsValidSample = false;
        }

        // === ACCELERATION VALIDATION ===
        if (WorldLinearAcceleration.ContainsNaN()) {
            ErrorMessages.Add(TEXT("WorldLinearAcceleration contains NaN values"));
            bIsValidSample = false;
        }

        if (LocalLinearAcceleration.ContainsNaN()) {
            ErrorMessages.Add(TEXT("LocalLinearAcceleration contains NaN values"));
            bIsValidSample = false;
        }

        if (ComponentLinearAcceleration.ContainsNaN()) {
            ErrorMessages.Add(TEXT("ComponentLinearAcceleration contains NaN values"));
            bIsValidSample = false;
        }

        // === ANGULAR MOTION VALIDATION ===
        if (AngularVelocity.ContainsNaN()) {
            ErrorMessages.Add(TEXT("AngularVelocity contains NaN values"));
            bIsValidSample = false;
        }

        if (AngularAcceleration.ContainsNaN()) {
            ErrorMessages.Add(TEXT("AngularAcceleration contains NaN values"));
            bIsValidSample = false;
        }

        // === TIMING VALIDATION ===
        if (FMath::IsNaN(TimeStamp) || TimeStamp < 0.0f) {
            ErrorMessages.Add(TEXT("TimeStamp is invalid (NaN or negative)"));
            bIsValidSample = false;
        }

        if (FMath::IsNaN(DeltaTime) || DeltaTime < 0.0f) {
            ErrorMessages.Add(TEXT("DeltaTime is invalid (NaN or negative)"));
            bIsValidSample = false;
        }

        // === TRANSFORM VALIDATION ===
        if (ComponentTransform.ContainsNaN()) {
            ErrorMessages.Add(TEXT("ComponentTransform contains NaN values"));
            bIsValidSample = false;
        }

        if (ReferenceTransform.ContainsNaN()) {
            ErrorMessages.Add(TEXT("ReferenceTransform contains NaN values"));
            bIsValidSample = false;
        }

        // === SPEED CONSISTENCY VALIDATION ===
        float CalculatedWorldSpeed = WorldLinearVelocity.Size();
        if (FMath::Abs(WorldSpeed - CalculatedWorldSpeed) > 1.0f) // 1cm/s tolerance
        {
            ErrorMessages.Add(FString::Printf(TEXT("WorldSpeed inconsistency: stored=%.2f, calculated=%.2f"),
                                              WorldSpeed, CalculatedWorldSpeed));
            // Note: This is a warning, not a critical failure
        }

        // === CONSTRUCT ERROR MESSAGE ===
        if (OutErrorMessage && ErrorMessages.Num() > 0) {
            *OutErrorMessage = FString::Printf(TEXT("FOHMotionFrameSample validation failed (%d errors): %s"),
                                               ErrorMessages.Num(), *FString::Join(ErrorMessages, TEXT("; ")));
        }

        return bIsValidSample;
    }

    /**
     * Quick validation check without error message generation (performance optimized)
     * @return True if sample data is valid for use in calculations
     */
    FORCEINLINE bool IsValidFast() const {
        return !WorldPosition.ContainsNaN() && WorldRotation.IsNormalized() && !WorldLinearVelocity.ContainsNaN() &&
               !WorldLinearAcceleration.ContainsNaN() && !FMath::IsNaN(TimeStamp) && !FMath::IsNaN(DeltaTime);
    }

    // ============================================================================
    // ENHANCED: RELATIVE MOTION ANALYSIS SYSTEM
    // ============================================================================

    /**
     * Calculate relative velocity between this sample and another sample
     * @param Other - Other motion sample for comparison
     * @param Space - Reference space for velocity comparison
     * @return Relative velocity vector (this sample relative to other)
     */
    FVector CalculateRelativeVelocity(const FOHMotionFrameSample& Other, EOHReferenceSpace Space) const {
        // Validation check
        if (!IsValidFast() || !Other.IsValidFast()) {
            return FVector::ZeroVector;
        }

        FVector ThisVelocity = GetVelocity(Space);
        FVector OtherVelocity = Other.GetVelocity(Space);

        // Return velocity difference (this - other)
        FVector RelativeVel = ThisVelocity - OtherVelocity;
        return RelativeVel.ContainsNaN() ? FVector::ZeroVector : RelativeVel;
    }

    /**
     * Calculate relative speed between samples (magnitude of relative velocity)
     * @param Other - Other motion sample for comparison
     * @param Space - Reference space for speed comparison
     * @return Relative speed magnitude
     */
    FORCEINLINE float CalculateRelativeSpeed(const FOHMotionFrameSample& Other, EOHReferenceSpace Space) const {
        return CalculateRelativeVelocity(Other, Space).Size();
    }

    /**
     * Calculate closing rate between samples (positive = approaching, negative = separating)
     * @param Other - Other motion sample for comparison
     * @param Space - Reference space for calculation
     * @return Closing rate (positive values indicate samples are getting closer)
     */
    float CalculateClosingRate(const FOHMotionFrameSample& Other, EOHReferenceSpace Space) const {
        if (!IsValidFast() || !Other.IsValidFast()) {
            return 0.0f;
        }

        // Get positions in specified space
        FVector ThisPos = GetPosition(Space);
        FVector OtherPos = Other.GetPosition(Space);

        // Calculate separation vector
        FVector SeparationVector = OtherPos - ThisPos;
        if (SeparationVector.IsNearlyZero()) {
            return 0.0f; // Same position
        }

        // Get relative velocity
        FVector RelativeVelocity = CalculateRelativeVelocity(Other, Space);

        // Project relative velocity onto separation vector
        // Positive dot product means approaching (closing), negative means separating
        FVector SeparationDirection = SeparationVector.GetSafeNormal();
        float ClosingRate = FVector::DotProduct(RelativeVelocity, SeparationDirection);

        return FMath::IsNaN(ClosingRate) ? 0.0f : ClosingRate;
    }

    /**
     * Calculate comprehensive relative motion state between samples
     * @param Other - Other motion sample for comparison
     * @param Space - Reference space for analysis
     * @param OutRelativeVelocity - Relative velocity vector
     * @param OutRelativeSpeed - Relative speed magnitude
     * @param OutClosingRate - Rate of approach/separation
     * @param OutSeparationDistance - Current distance between samples
     * @return True if calculation successful, false if invalid input
     */
    bool CalculateRelativeMotion(const FOHMotionFrameSample& Other, EOHReferenceSpace Space,
                                 FVector& OutRelativeVelocity, float& OutRelativeSpeed, float& OutClosingRate,
                                 float& OutSeparationDistance) const {
        // Initialize outputs
        OutRelativeVelocity = FVector::ZeroVector;
        OutRelativeSpeed = 0.0f;
        OutClosingRate = 0.0f;
        OutSeparationDistance = 0.0f;

        // Validation
        if (!IsValidFast() || !Other.IsValidFast()) {
            return false;
        }

        // Calculate all relative motion metrics
        OutRelativeVelocity = CalculateRelativeVelocity(Other, Space);
        OutRelativeSpeed = OutRelativeVelocity.Size();
        OutClosingRate = CalculateClosingRate(Other, Space);

        FVector ThisPos = GetPosition(Space);
        FVector OtherPos = Other.GetPosition(Space);
        OutSeparationDistance = FVector::Dist(ThisPos, OtherPos);

        return true;
    }

    // ============================================================================
    // ENHANCED: REFERENCE SPACE MANAGEMENT SYSTEM
    // ============================================================================

    /**
     * Update all reference space calculations with new transform data
     * @param NewComponentTransform - Updated component transform
     * @param NewReferenceVelocity - Updated reference frame velocity
     */
    void UpdateReferenceSpaces(const FTransform& NewComponentTransform,
                               const FVector& NewReferenceVelocity = FVector::ZeroVector) {
        // Validate input transforms
        if (NewComponentTransform.ContainsNaN()) {
            UE_LOG(LogTemp, Warning,
                   TEXT("FOHMotionFrameSample::UpdateReferenceSpaces: Invalid ComponentTransform provided"));
            return;
        }

        // Update reference data
        ComponentTransform = NewComponentTransform;
        ReferenceVelocity = NewReferenceVelocity;
        ReferenceTransform = NewComponentTransform;

        // Recalculate component space data
        UpdateComponentSpaceData(NewComponentTransform);

        // Recalculate local space velocity with new reference
        LocalLinearVelocity = WorldLinearVelocity - NewReferenceVelocity;
        if (!WorldRotation.IsIdentity()) {
            LocalLinearVelocity = WorldRotation.Inverse().RotateVector(LocalLinearVelocity);
        }

        // Update speed properties
        UpdateSpeedProperties();
    }

    /**
     * Recalculate component space data from world space using provided transform
     * @param InComponentTransform - Component transform for space conversion
     */
    void UpdateComponentSpaceData(const FTransform& InComponentTransform) {
        if (InComponentTransform.ContainsNaN()) {
            return; // Skip update if transform is invalid
        }

        // Transform position and rotation to component space
        ComponentPosition = InComponentTransform.InverseTransformPosition(WorldPosition);
        ComponentRotation = InComponentTransform.InverseTransformRotation(WorldRotation);

        // Transform velocities to component space
        ComponentLinearVelocity = InComponentTransform.InverseTransformVector(WorldLinearVelocity);
        ComponentLinearAcceleration = InComponentTransform.InverseTransformVector(WorldLinearAcceleration);
    }

    /**
     * Update all speed properties from current velocity data
     */
    void UpdateSpeedProperties() {
        WorldSpeed = WorldLinearVelocity.Size();
        LocalSpeed = LocalLinearVelocity.Size();
        ComponentSpeed = ComponentLinearVelocity.Size();
        AngularSpeed = AngularVelocity.Size();
    }

    // ============================================================================
    // ENHANCED: DEBUG AND DIAGNOSTIC SYSTEM
    // ============================================================================

    /**
     * Generate comprehensive debug string for logging and diagnostics
     * @param bVerbose - Include detailed motion data in output
     * @return Formatted debug information string
     */
    FString GetDebugString(bool bVerbose = false) const {
        FString DebugInfo;

        // Basic info
        DebugInfo += FString::Printf(TEXT("MotionSample[T=%.3f, dT=%.4f] "), TimeStamp, DeltaTime);

        // Validation status
        FString ValidationError;
        bool bValid = IsValid(&ValidationError);
        DebugInfo += FString::Printf(TEXT("Valid=%s "), bValid ? TEXT("✓") : TEXT("✗"));

        if (!bValid) {
            DebugInfo += FString::Printf(TEXT("Error='%s' "), *ValidationError);
        }

        // Motion summary
        DebugInfo += FString::Printf(TEXT("Speeds[W=%.1f,L=%.1f,C=%.1f,A=%.2f] "), WorldSpeed, LocalSpeed,
                                     ComponentSpeed, AngularSpeed);

        if (bVerbose) {
            // Detailed motion data
            DebugInfo += FString::Printf(TEXT("\n  WorldPos=%s WorldVel=%s"), *WorldPosition.ToString(),
                                         *WorldLinearVelocity.ToString());
            DebugInfo += FString::Printf(TEXT("\n  LocalVel=%s ComponentVel=%s"), *LocalLinearVelocity.ToString(),
                                         *ComponentLinearVelocity.ToString());
            DebugInfo += FString::Printf(TEXT("\n  WorldAccel=%s AngularVel=%s"), *WorldLinearAcceleration.ToString(),
                                         *AngularVelocity.ToString());
        }

        return DebugInfo;
    }

    /**
     * Simplified logging method using your project's standard LogPACManager category
     * @param bVerbose - Include detailed motion data
     * @param Verbosity - Log verbosity level (Log, Warning, Error, VeryVerbose)
     */
    void LogDebugInfo(bool bVerbose = false, ELogVerbosity::Type Verbosity = ELogVerbosity::Log) const {
        FString DebugInfo = GetDebugString(bVerbose);

        // Use standard UE_LOG with LogPACManager (your project's log category)
        switch (Verbosity) {
        case ELogVerbosity::Error:
            UE_LOG(LogPACManager, Error, TEXT("[MotionSample] %s"), *DebugInfo);
            break;
        case ELogVerbosity::Warning:
            UE_LOG(LogPACManager, Warning, TEXT("[MotionSample] %s"), *DebugInfo);
            break;
        case ELogVerbosity::VeryVerbose:
            UE_LOG(LogPACManager, VeryVerbose, TEXT("[MotionSample] %s"), *DebugInfo);
            break;
        case ELogVerbosity::Log:
        default:
            UE_LOG(LogPACManager, Log, TEXT("[MotionSample] %s"), *DebugInfo);
            break;
        }
    }

    /**
     * Quick debug logging for development (uses LogPACManager with Warning level)
     * @param bVerbose - Include detailed motion data
     */
    void LogDebugInfoSimple(bool bVerbose = false) const {
        FString DebugInfo = GetDebugString(bVerbose);
        UE_LOG(LogPACManager, Warning, TEXT("[MotionSample] %s"), *DebugInfo);
    }

    // ============================================================================
    // ENHANCED: TEMPORAL ANALYSIS SUPPORT
    // ============================================================================

    /**
     * Calculate jerk (rate of change of acceleration) between this and previous sample
     * @param PreviousSample - Previous motion sample for differential calculation
     * @param Space - Reference space for jerk calculation
     * @return Jerk vector in specified space
     */
    FVector CalculateJerk(const FOHMotionFrameSample& PreviousSample, EOHReferenceSpace Space) const {
        if (DeltaTime <= KINDA_SMALL_NUMBER || !IsValidFast() || !PreviousSample.IsValidFast()) {
            return FVector::ZeroVector;
        }

        FVector CurrentAccel = GetAcceleration(Space);
        FVector PreviousAccel = PreviousSample.GetAcceleration(Space);

        if (CurrentAccel.ContainsNaN() || PreviousAccel.ContainsNaN()) {
            return FVector::ZeroVector;
        }

        FVector Jerk = (CurrentAccel - PreviousAccel) / DeltaTime;
        return Jerk.ContainsNaN() ? FVector::ZeroVector : Jerk;
    }

    /**
     * Detect significant motion direction change between samples
     * @param PreviousSample - Previous motion sample for comparison
     * @param Space - Reference space for analysis
     * @param AngleThreshold - Minimum angle change in degrees to consider significant
     * @return True if significant direction change detected
     */
    bool DetectDirectionChange(const FOHMotionFrameSample& PreviousSample, EOHReferenceSpace Space,
                               float AngleThreshold = 45.0f) const {
        if (!IsValidFast() || !PreviousSample.IsValidFast()) {
            return false;
        }

        FVector CurrentVel = GetVelocity(Space);
        FVector PreviousVel = PreviousSample.GetVelocity(Space);

        // Need significant velocity to detect direction change
        if (CurrentVel.IsNearlyZero(1.0f) || PreviousVel.IsNearlyZero(1.0f)) {
            return false;
        }

        // Calculate angle between velocity vectors
        float DotProduct = FVector::DotProduct(CurrentVel.GetSafeNormal(), PreviousVel.GetSafeNormal());
        float AngleRadians = FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f));
        float AngleDegrees = FMath::RadiansToDegrees(AngleRadians);

        return AngleDegrees > AngleThreshold;
    }

    // ============================================================================
    // PRESERVED API: All Existing Methods Unchanged
    // ============================================================================

    /**
     * Get velocity in specified reference space with comprehensive validation
     * @param Space - Reference space for velocity calculation
     * @return Velocity vector in specified space, zero if invalid
     */
    FORCEINLINE FVector GetVelocity(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return WorldLinearVelocity.ContainsNaN() ? FVector::ZeroVector : WorldLinearVelocity;
        case EOHReferenceSpace::LocalSpace:
            return LocalLinearVelocity.ContainsNaN() ? FVector::ZeroVector : LocalLinearVelocity;
        case EOHReferenceSpace::ComponentSpace:
            return ComponentLinearVelocity.ContainsNaN() ? FVector::ZeroVector : ComponentLinearVelocity;
        default:
            return WorldLinearVelocity.ContainsNaN() ? FVector::ZeroVector : WorldLinearVelocity;
        }
    }

    /**
     * Get acceleration in specified reference space with comprehensive validation
     * @param Space - Reference space for acceleration calculation
     * @return Acceleration vector in specified space, zero if invalid
     */
    FORCEINLINE FVector GetAcceleration(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return WorldLinearAcceleration.ContainsNaN() ? FVector::ZeroVector : WorldLinearAcceleration;
        case EOHReferenceSpace::LocalSpace:
            return LocalLinearAcceleration.ContainsNaN() ? FVector::ZeroVector : LocalLinearAcceleration;
        case EOHReferenceSpace::ComponentSpace:
            return ComponentLinearAcceleration.ContainsNaN() ? FVector::ZeroVector : ComponentLinearAcceleration;
        default:
            return WorldLinearAcceleration.ContainsNaN() ? FVector::ZeroVector : WorldLinearAcceleration;
        }
    }

    /**
     * Get position in specified reference space
     * @param Space - Reference space for position calculation
     * @return Position vector in specified space, zero if invalid
     */
    FORCEINLINE FVector GetPosition(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return WorldPosition.ContainsNaN() ? FVector::ZeroVector : WorldPosition;
        case EOHReferenceSpace::ComponentSpace:
            return ComponentPosition.ContainsNaN() ? FVector::ZeroVector : ComponentPosition;
        case EOHReferenceSpace::LocalSpace:
            // Local space position requires reference bone context - fallback to component
            return ComponentPosition.ContainsNaN() ? FVector::ZeroVector : ComponentPosition;
        default:
            return WorldPosition.ContainsNaN() ? FVector::ZeroVector : WorldPosition;
        }
    }

    /**
     * Get motion speed in specified reference space
     * @param Space - Reference space for speed calculation
     * @return Speed magnitude in specified space
     */
    FORCEINLINE float GetSpeed(EOHReferenceSpace Space) const {
        return GetVelocity(Space).Size();
    }

    // === LEGACY COMPATIBILITY ACCESSORS (PRESERVED) ===
    FORCEINLINE FVector GetLocation() const {
        return WorldPosition;
    }
    FORCEINLINE FVector GetLinearVelocity() const {
        return WorldLinearVelocity;
    }
    FORCEINLINE FVector GetAngularVelocity() const {
        return AngularVelocity;
    }
    FORCEINLINE FVector GetLinearAcceleration() const {
        return WorldLinearAcceleration;
    }
    FORCEINLINE FVector GetAngularAcceleration() const {
        return AngularAcceleration;
    }
    FORCEINLINE float GetTimeStamp() const {
        return TimeStamp;
    }
    FORCEINLINE FQuat GetRotation() const {
        return WorldRotation;
    }
    FORCEINLINE FTransform GetTransform() const {
        return FTransform(WorldRotation, WorldPosition);
    }

    void RecalculateReferenceSpaces(const FTransform& NewComponentTransform, const FTransform& NewReferenceTransform) {
        ComponentTransform = NewComponentTransform;
        ReferenceTransform = NewReferenceTransform; // Or ReferenceTransform based on your naming

        // Recalculate local space velocities
        if (!ComponentTransform.Equals(FTransform::Identity)) {
            ComponentLinearVelocity = ComponentTransform.InverseTransformVector(WorldLinearVelocity);
            ComponentLinearAcceleration = ComponentTransform.InverseTransformVector(WorldLinearAcceleration);
        }

        if (!ReferenceTransform.Equals(FTransform::Identity)) {
            LocalLinearVelocity = ReferenceTransform.InverseTransformVector(WorldLinearVelocity);
            LocalLinearAcceleration = ReferenceTransform.InverseTransformVector(WorldLinearAcceleration);
        }
    }
    // ============================================================================
    // ENHANCED FACTORY METHODS (Enhanced validation, preserved API)
    // ============================================================================

    /**
     * Create motion sample from kinematic state with comprehensive validation
     * Enhanced version of existing factory method with improved error handling
     */
    static FOHMotionFrameSample CreateFromState(const FVector& Position, const FQuat& Rotation,
                                                const FVector& LinearVel, const FVector& AngularVel,
                                                const FVector& LinearAccel = FVector::ZeroVector,
                                                const FVector& AngularAccel = FVector::ZeroVector, float Time = 0.0f,
                                                const FTransform& ComponentTransform = FTransform::Identity,
                                                const FVector& ReferenceVelocity = FVector::ZeroVector) {
        FOHMotionFrameSample Sample;

        // === ENHANCED: Input validation before assignment ===
        if (Position.ContainsNaN() || !Rotation.IsNormalized() || LinearVel.ContainsNaN()) {
            UE_LOG(LogTemp, Warning,
                   TEXT("FOHMotionFrameSample::CreateFromState: Invalid input data detected, returning "
                        "zero-initialized sample"));
            return Sample; // Return default-initialized sample
        }

        // === CORE SPATIAL STATE INITIALIZATION ===
        Sample.WorldPosition = Position;
        Sample.WorldRotation = Rotation;
        Sample.TimeStamp = Time;
        Sample.DeltaTime = 0.016f; // Default frame time

        // === WORLD SPACE MOTION ===
        Sample.WorldLinearVelocity = LinearVel;
        Sample.WorldLinearAcceleration = LinearAccel;
        Sample.AngularVelocity = AngularVel;
        Sample.AngularAcceleration = AngularAccel;

        // === REFERENCE FRAME SETUP ===
        Sample.ComponentTransform = ComponentTransform;
        Sample.ReferenceTransform = ComponentTransform;
        Sample.ReferenceVelocity = ReferenceVelocity;

        // === MULTI-SPACE CALCULATIONS ===
        Sample.UpdateComponentSpaceData(ComponentTransform);

        // Local space velocity calculation
        Sample.LocalLinearVelocity = LinearVel - ReferenceVelocity;
        if (!Rotation.IsIdentity()) {
            Sample.LocalLinearVelocity = Rotation.Inverse().RotateVector(Sample.LocalLinearVelocity);
        }

        // Local space acceleration (simplified - assumes reference frame is not accelerating)
        Sample.LocalLinearAcceleration = LinearAccel;
        if (!Rotation.IsIdentity()) {
            Sample.LocalLinearAcceleration = Rotation.Inverse().RotateVector(Sample.LocalLinearAcceleration);
        }

        // === SPEED PROPERTIES UPDATE ===
        Sample.UpdateSpeedProperties();

        return Sample;
    }

    /**
     * Create validated motion sample that guarantees valid output
     * @return Valid motion sample or zero-initialized sample if input invalid
     */
    static FOHMotionFrameSample CreateValidated(const FVector& Position, const FQuat& Rotation,
                                                const FVector& LinearVel, const FVector& AngularVel,
                                                float Time = 0.0f) {
        // Pre-validate all inputs
        if (Position.ContainsNaN() || !Rotation.IsNormalized() || LinearVel.ContainsNaN() || AngularVel.ContainsNaN() ||
            FMath::IsNaN(Time)) {
            // Return safe default sample
            return FOHMotionFrameSample();
        }

        return CreateFromState(Position, Rotation, LinearVel, AngularVel, FVector::ZeroVector, FVector::ZeroVector,
                               Time);
    }

    // === CONSTRUCTOR (Enhanced with validation) ===
    FOHMotionFrameSample() {
        // Initialize all fields to safe defaults
        WorldPosition = FVector::ZeroVector;
        WorldRotation = FQuat::Identity;
        WorldLinearVelocity = FVector::ZeroVector;
        WorldLinearAcceleration = FVector::ZeroVector;
        LocalLinearVelocity = FVector::ZeroVector;
        LocalLinearAcceleration = FVector::ZeroVector;
        ComponentLinearVelocity = FVector::ZeroVector;
        ComponentLinearAcceleration = FVector::ZeroVector;
        AngularVelocity = FVector::ZeroVector;
        AngularAcceleration = FVector::ZeroVector;
        TimeStamp = 0.0f;
        DeltaTime = 0.0f;
        ReferenceTransform = FTransform::Identity;
        ReferenceVelocity = FVector::ZeroVector;
        ComponentTransform = FTransform::Identity;
        ComponentPosition = FVector::ZeroVector;
        ComponentRotation = FQuat::Identity;
        WorldSpeed = 0.0f;
        LocalSpeed = 0.0f;
        ComponentSpeed = 0.0f;
        AngularSpeed = 0.0f;
        PrimarySpace = EOHReferenceSpace::WorldSpace;
    }
};

/// ============================================================================
// Enhanced FOHBoneMotionData - Self-Aware Bone Motion Tracking System
// OnlyHands Project - Unreal Engine 5.3.2
//
// ENHANCEMENT SUMMARY:
// - Added comprehensive bone-to-bone relative motion analysis
// - Implemented performance-optimized caching system with invalidation
// - Enhanced strike detection integration with motion validation
// - Added temporal analysis framework for combat pattern recognition
// - Integrated enhanced FOHMotionFrameSample validation pipeline
//
// BACKWARD COMPATIBILITY:
// All existing methods and data members preserved. Enhanced functionality
// accessed via new methods with clear naming conventions.
// ============================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHBoneMotionData {
    GENERATED_BODY()

    // =============== CORE PROPERTIES (PRESERVED - NO CHANGES) ===============

    UPROPERTY(BlueprintReadOnly, Category = "Motion Data|Core")
    FName BoneName = NAME_None;

    // Enhanced rolling buffer with comprehensive sample tracking
    static constexpr int32 HistoryCapacity = 30;
    OHSafeMapUtils::TRollingBuffer<FOHMotionFrameSample, HistoryCapacity> MotionHistory;

    // Strike detection metrics (PRESERVED)
    UPROPERTY(BlueprintReadOnly, Category = "Motion Data|Strike Analysis")
    float PeakAcceleration = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Data|Strike Analysis")
    float AccelerationBuildupTime = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Data|Strike Analysis")
    bool bLikelyStrike = false;

    // =============== ENHANCED: CACHING AND PERFORMANCE SYSTEM ===============

  private:
    // Cached motion quality data with invalidation tracking
    mutable float CachedMotionQuality = -1.0f;
    mutable EOHReferenceSpace CachedQualitySpace = EOHReferenceSpace::WorldSpace;
    mutable bool bMotionQualityCacheValid = false;

    // Cached relative motion data for performance optimization
    mutable TMap<FName, FVector> CachedRelativeVelocities;
    mutable TMap<FName, float> CachedRelativeSpeeds;
    mutable bool bRelativeMotionCacheValid = false;

    // Sample validation tracking
    mutable int32 LastValidatedSampleIndex = -1;
    mutable bool bLastSampleValid = false;

    // Strike metrics cache
    mutable bool bStrikeMetricsCacheValid = false;
    mutable float CachedJerkMagnitude = 0.0f;
    mutable float CachedMotionConsistency = 0.0f;

  public:
    // =============== CONSTRUCTOR (Enhanced with validation) ===============

    FOHBoneMotionData() {
        // Preserved initialization
        BoneName = NAME_None;
        PeakAcceleration = 0.0f;
        AccelerationBuildupTime = 0.0f;
        bLikelyStrike = false;

        // Enhanced cache initialization
        InvalidateAllCaches();
    }

    // ============================================================================
    // ENHANCED: COMPREHENSIVE RELATIVE MOTION ANALYSIS SYSTEM
    // ============================================================================

    /**
     * Calculate relative velocity between this bone and another bone
     * Uses enhanced FOHMotionFrameSample methods for direct comparison
     * @param Other - Other bone motion data for comparison
     * @param Space - Reference space for velocity comparison (World, Local, Component)
     * @return Relative velocity vector (this bone velocity - other bone velocity)
     */
    FVector CalculateRelativeVelocity(const FOHBoneMotionData& Other, EOHReferenceSpace Space) const {
        // Validate both motion data sets
        const FOHMotionFrameSample* ThisSample = GetLatestSample();
        const FOHMotionFrameSample* OtherSample = Other.GetLatestSample();

        if (!ThisSample || !OtherSample) {
            return FVector::ZeroVector;
        }

        // Use enhanced FOHMotionFrameSample relative motion calculation
        return ThisSample->CalculateRelativeVelocity(*OtherSample, Space);
    }

    /**
     * Calculate relative speed between bones (magnitude of relative velocity)
     * @param Other - Other bone motion data for comparison
     * @param Space - Reference space for speed comparison
     * @return Relative speed magnitude
     */
    FORCEINLINE float CalculateRelativeSpeed(const FOHBoneMotionData& Other, EOHReferenceSpace Space) const {
        return CalculateRelativeVelocity(Other, Space).Size();
    }

    /**
     * Calculate closing rate between bones (positive = approaching, negative = separating)
     * Critical for combat impact force calculations
     * @param Other - Other bone motion data for comparison
     * @param Space - Reference space for calculation
     * @return Closing rate (positive = bones getting closer, negative = separating)
     */
    float CalculateClosingRate(const FOHBoneMotionData& Other, EOHReferenceSpace Space) const {
        const FOHMotionFrameSample* ThisSample = GetLatestSample();
        const FOHMotionFrameSample* OtherSample = Other.GetLatestSample();

        if (!ThisSample || !OtherSample) {
            return 0.0f;
        }

        // Use enhanced FOHMotionFrameSample closing rate calculation
        return ThisSample->CalculateClosingRate(*OtherSample, Space);
    }

    /**
     * Comprehensive relative motion analysis between bones
     * @param Other - Other bone motion data for comparison
     * @param Space - Reference space for analysis
     * @param OutRelativeVelocity - Relative velocity vector
     * @param OutRelativeSpeed - Relative speed magnitude
     * @param OutClosingRate - Rate of approach/separation
     * @param OutSeparationDistance - Current distance between bones
     * @return True if calculation successful, false if insufficient data
     */
    bool CalculateRelativeMotion(const FOHBoneMotionData& Other, EOHReferenceSpace Space, FVector& OutRelativeVelocity,
                                 float& OutRelativeSpeed, float& OutClosingRate, float& OutSeparationDistance) const {
        const FOHMotionFrameSample* ThisSample = GetLatestSample();
        const FOHMotionFrameSample* OtherSample = Other.GetLatestSample();

        if (!ThisSample || !OtherSample) {
            OutRelativeVelocity = FVector::ZeroVector;
            OutRelativeSpeed = 0.0f;
            OutClosingRate = 0.0f;
            OutSeparationDistance = 0.0f;
            return false;
        }

        // Use enhanced FOHMotionFrameSample comprehensive relative motion
        return ThisSample->CalculateRelativeMotion(*OtherSample, Space, OutRelativeVelocity, OutRelativeSpeed,
                                                   OutClosingRate, OutSeparationDistance);
    }

    /**
     * Smart motion significance detection for combat system optimization
     * @param Other - Other bone motion data for comparison
     * @param Threshold - Minimum relative speed to consider significant (cm/s)
     * @param Space - Reference space for threshold comparison
     * @return True if bones have significant relative motion above threshold
     */
    bool HasSignificantRelativeMotion(const FOHBoneMotionData& Other, float Threshold,
                                      EOHReferenceSpace Space = EOHReferenceSpace::LocalSpace) const {
        // Quick validation
        if (!HasValidMotionData() || !Other.HasValidMotionData()) {
            return false;
        }

        // Check cache validity for performance optimization
        FName OtherBoneName = Other.GetBoneName();

        if (bRelativeMotionCacheValid) {
            if (const float* CachedSpeed = CachedRelativeSpeeds.Find(OtherBoneName)) {
                return *CachedSpeed > Threshold;
            }
        }

        // Calculate and cache relative speed
        float RelativeSpeed = CalculateRelativeSpeed(Other, Space);

        // Update cache if not locked
        if (!bRelativeMotionCacheValid) {
            CachedRelativeSpeeds.Add(OtherBoneName, RelativeSpeed);
        }

        return RelativeSpeed > Threshold;
    }

    /**
     * Batch relative motion analysis for multiple bones (performance optimized)
     * @param OtherBones - Array of bone motion data for comparison
     * @param Space - Reference space for analysis
     * @param Threshold - Minimum significance threshold
     * @param OutSignificantBones - Bones with significant relative motion
     * @return Number of bones with significant relative motion
     */
    int32 AnalyzeRelativeMotionBatch(const TArray<const FOHBoneMotionData*>& OtherBones, EOHReferenceSpace Space,
                                     float Threshold, TArray<FName>& OutSignificantBones) const {
        OutSignificantBones.Reset();

        if (!HasValidMotionData()) {
            return 0;
        }

        for (const FOHBoneMotionData* OtherBone : OtherBones) {
            if (OtherBone && HasSignificantRelativeMotion(*OtherBone, Threshold, Space)) {
                OutSignificantBones.Add(OtherBone->GetBoneName());
            }
        }

        return OutSignificantBones.Num();
    }

    // ============================================================================
    // ENHANCED: PERFORMANCE-OPTIMIZED CACHING SYSTEM
    // ============================================================================

    /**
     * Enhanced motion quality calculation with comprehensive caching
     * @param Space - Reference space for quality analysis
     * @param bForceRecalculate - Force cache invalidation and recalculation
     * @return Motion quality score (0.0 to 1.0), cached for performance
     */
    float GetMotionQuality(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace,
                           bool bForceRecalculate = false) const {
        // Check cache validity
        if (!bForceRecalculate && bMotionQualityCacheValid && CachedQualitySpace == Space) {
            return CachedMotionQuality;
        }

        // Recalculate motion quality using enhanced analysis
        float CalculatedQuality = CalculateMotionQualityInternal(Space);

        // Update cache
        CachedMotionQuality = CalculatedQuality;
        CachedQualitySpace = Space;
        bMotionQualityCacheValid = true;

        return CalculatedQuality;
    }

    /**
     * Update all motion metrics with comprehensive caching and validation
     * Call this after adding new motion samples for optimal performance
     */
    void UpdateMotionMetrics() {
        // Invalidate caches when new data is added
        InvalidateAllCaches();

        // Update strike detection metrics using validated data
        UpdateStrikeDetectionMetrics();

        // Pre-calculate commonly used metrics for performance
        GetMotionQuality(EOHReferenceSpace::LocalSpace); // Cache local space quality
        GetMotionQuality(EOHReferenceSpace::WorldSpace); // Cache world space quality
    }

    /**
     * Invalidate all cached calculations (call when new samples added)
     */
    void InvalidateAllCaches() const {
        bMotionQualityCacheValid = false;
        bRelativeMotionCacheValid = false;
        bStrikeMetricsCacheValid = false;
        CachedRelativeVelocities.Reset();
        CachedRelativeSpeeds.Reset();
        LastValidatedSampleIndex = -1;
    }

    // ============================================================================
    // ENHANCED: VALIDATION AND STRIKE DETECTION INTEGRATION
    // ============================================================================

    /**
     * Enhanced motion sample addition with comprehensive validation
     * Integrates with enhanced FOHMotionFrameSample validation system
     * @param WorldPos - World space position
     * @param WorldRot - World space rotation
     * @param ComponentVelocity - Character movement component velocity
     * @param ReferenceVelocity - Reference frame velocity for local space calculation
     * @param ComponentTransform - Component transform for space calculations
     * @param DeltaTime - Time delta for velocity/acceleration calculation
     * @param TimeStamp - Sample timestamp
     * @return True if sample was successfully added, false if validation failed
     */
    bool AddMotionSampleValidated(const FVector& WorldPos, const FQuat& WorldRot, const FVector& ComponentVelocity,
                                  const FVector& ReferenceVelocity, const FTransform& ComponentTransform,
                                  float DeltaTime, float TimeStamp) {
        // Create enhanced motion sample with comprehensive validation
        FOHMotionFrameSample NewSample = FOHMotionFrameSample::CreateFromState(
            WorldPos, WorldRot, ComponentVelocity, FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector,
            TimeStamp, ComponentTransform, ReferenceVelocity);

        // Validate sample before adding
        FString ValidationError;
        if (!NewSample.IsValid(&ValidationError)) {
            UE_LOG(LogPACManager, Warning, TEXT("AddMotionSampleValidated failed for bone '%s': %s"),
                   *BoneName.ToString(), *ValidationError);
            return false;
        }

        // Add validated sample to history
        MotionHistory.Add(NewSample);

        // Invalidate caches and update metrics
        InvalidateAllCaches();
        UpdateMotionMetrics();

        return true;
    }

    /**
     * Validate current strike detection with motion data integrity
     * @param MinVelocityThreshold - Minimum velocity for valid strike
     * @param MinAccelerationThreshold - Minimum acceleration for valid strike
     * @return True if current motion constitutes a valid strike
     */
    bool IsValidStrike(float MinVelocityThreshold = 300.0f, float MinAccelerationThreshold = 1000.0f) const {
        const FOHMotionFrameSample* LatestSample = GetLatestSample();
        if (!LatestSample || !LatestSample->IsValidFast()) {
            return false;
        }

        // Check velocity threshold
        float CurrentSpeed = LatestSample->GetSpeed(EOHReferenceSpace::LocalSpace);
        if (CurrentSpeed < MinVelocityThreshold) {
            return false;
        }

        // Check acceleration threshold
        float CurrentAccelMagnitude = LatestSample->GetAcceleration(EOHReferenceSpace::LocalSpace).Size();
        if (CurrentAccelMagnitude < MinAccelerationThreshold) {
            return false;
        }

        // Validate strike metrics are consistent with motion data
        return bLikelyStrike && PeakAcceleration > MinAccelerationThreshold;
    }

    /**
     * Comprehensive motion data validation check
     * @return True if motion data contains valid samples and metrics
     */
    bool HasValidMotionData() const {
        // Check if we have any samples
        if (MotionHistory.NumFrames() == 0) {
            return false;
        }

        // Check if latest sample is valid (cached for performance)
        const FOHMotionFrameSample* LatestSample = GetLatestSample();
        if (!LatestSample) {
            return false;
        }

        // Use cached validation if available
        int32 CurrentSampleIndex = MotionHistory.NumFrames() - 1;
        if (LastValidatedSampleIndex == CurrentSampleIndex) {
            return bLastSampleValid;
        }

        // Validate and cache result
        bLastSampleValid = LatestSample->IsValidFast();
        LastValidatedSampleIndex = CurrentSampleIndex;

        return bLastSampleValid;
    }

    // ============================================================================
    // ENHANCED: TEMPORAL ANALYSIS AND PATTERN DETECTION
    // ============================================================================

    /**
     * Analyze motion trend over specified time window
     * @param TimeWindow - Time window for analysis (seconds)
     * @param Space - Reference space for trend analysis
     * @param OutTrendVelocity - Average velocity trend over window
     * @param OutTrendAcceleration - Average acceleration trend over window
     * @param OutConsistencyScore - Motion consistency score (0.0 to 1.0)
     * @return True if sufficient data for trend analysis
     */
    bool GetMotionTrend(float TimeWindow, EOHReferenceSpace Space, FVector& OutTrendVelocity,
                        FVector& OutTrendAcceleration, float& OutConsistencyScore) const {
        OutTrendVelocity = FVector::ZeroVector;
        OutTrendAcceleration = FVector::ZeroVector;
        OutConsistencyScore = 0.0f;

        if (MotionHistory.NumFrames() < 3) {
            return false;
        }

        // Find samples within time window
        float CurrentTime = GetLatestSample() ? GetLatestSample()->GetTimeStamp() : 0.0f;
        float StartTime = CurrentTime - TimeWindow;

        TArray<const FOHMotionFrameSample*> WindowSamples;
        for (int32 i = 0; i < MotionHistory.NumFrames(); i++) {
            const FOHMotionFrameSample* Sample = MotionHistory.GetLatest(i);
            if (Sample && Sample->GetTimeStamp() >= StartTime) {
                WindowSamples.Add(Sample);
            }
        }

        if (WindowSamples.Num() < 3) {
            return false;
        }

        // Calculate trend metrics
        FVector VelocitySum = FVector::ZeroVector;
        FVector AccelerationSum = FVector::ZeroVector;
        int32 ValidSamples = 0;

        for (const FOHMotionFrameSample* Sample : WindowSamples) {
            if (Sample->IsValidFast()) {
                VelocitySum += Sample->GetVelocity(Space);
                AccelerationSum += Sample->GetAcceleration(Space);
                ValidSamples++;
            }
        }

        if (ValidSamples == 0) {
            return false;
        }

        OutTrendVelocity = VelocitySum / ValidSamples;
        OutTrendAcceleration = AccelerationSum / ValidSamples;

        // Calculate consistency score using velocity direction stability
        OutConsistencyScore = CalculateMotionConsistencyOverWindow(WindowSamples, Space);

        return true;
    }

    /**
     * Detect motion stability over specified window for prediction confidence
     * @param TimeWindow - Time window for stability analysis (seconds)
     * @param Space - Reference space for analysis
     * @return Stability score (0.0 = chaotic, 1.0 = perfectly stable)
     */
    float GetMotionStability(float TimeWindow, EOHReferenceSpace Space) const {
        if (MotionHistory.NumFrames() < 5) {
            return 0.0f;
        }

        // Use cached consistency if available
        if (bStrikeMetricsCacheValid) {
            return CachedMotionConsistency;
        }

        // Find samples within window
        float CurrentTime = GetLatestSample() ? GetLatestSample()->GetTimeStamp() : 0.0f;
        float StartTime = CurrentTime - TimeWindow;

        float StabilitySum = 0.0f;
        int32 ValidComparisons = 0;

        for (int32 i = 0; i < MotionHistory.NumFrames() - 1; i++) {
            const FOHMotionFrameSample* Current = MotionHistory.GetLatest(i);
            const FOHMotionFrameSample* Previous = MotionHistory.GetLatest(i + 1);

            if (Current && Previous && Current->GetTimeStamp() >= StartTime && Previous->GetTimeStamp() >= StartTime) {
                if (Current->IsValidFast() && Previous->IsValidFast()) {
                    FVector CurrentVel = Current->GetVelocity(Space);
                    FVector PreviousVel = Previous->GetVelocity(Space);

                    if (!CurrentVel.IsNearlyZero(1.0f) && !PreviousVel.IsNearlyZero(1.0f)) {
                        float DirectionStability =
                            FVector::DotProduct(CurrentVel.GetSafeNormal(), PreviousVel.GetSafeNormal());
                        StabilitySum += FMath::Clamp(DirectionStability, 0.0f, 1.0f);
                        ValidComparisons++;
                    }
                }
            }
        }

        float Stability = ValidComparisons > 0 ? (StabilitySum / ValidComparisons) : 0.0f;

        // Cache result
        CachedMotionConsistency = Stability;

        return Stability;
    }

    /**
     * Advanced jerk calculation with caching for performance
     * @param Space - Reference space for jerk calculation
     * @param bUseCache - Use cached result if available
     * @return Jerk magnitude (rate of change of acceleration)
     */
    float CalculateJerkMagnitude(EOHReferenceSpace Space, bool bUseCache = true) const {
        if (bUseCache && bStrikeMetricsCacheValid) {
            return CachedJerkMagnitude;
        }

        if (MotionHistory.NumFrames() < 2) {
            return 0.0f;
        }

        const FOHMotionFrameSample* Current = GetLatestSample();
        const FOHMotionFrameSample* Previous = GetHistoricalSample(1);

        if (!Current || !Previous || !Current->IsValidFast() || !Previous->IsValidFast()) {
            return 0.0f;
        }

        // Use enhanced FOHMotionFrameSample jerk calculation
        FVector Jerk = Current->CalculateJerk(*Previous, Space);
        float JerkMagnitude = Jerk.Size();

        // Cache result
        CachedJerkMagnitude = JerkMagnitude;

        return JerkMagnitude;
    }

    // ============================================================================
    // ENHANCED: DEBUG AND DIAGNOSTIC SYSTEM
    // ============================================================================

    /**
     * Comprehensive debug information with motion analysis
     * @param bIncludeRelativeMotion - Include relative motion cache data
     * @param bIncludeTemporalAnalysis - Include trend and stability analysis
     * @return Formatted debug string
     */
    FString GetDebugString(bool bIncludeRelativeMotion = false, bool bIncludeTemporalAnalysis = false) const {
        FString DebugInfo;

        // Basic bone info
        DebugInfo += FString::Printf(TEXT("BoneMotionData[%s] "), *BoneName.ToString());
        DebugInfo += FString::Printf(TEXT("Samples=%d Valid=%s "), MotionHistory.NumFrames(),
                                     HasValidMotionData() ? TEXT("✓") : TEXT("✗"));

        // Latest motion data
        if (const FOHMotionFrameSample* Latest = GetLatestSample()) {
            DebugInfo += FString::Printf(
                TEXT("Speed[W=%.1f,L=%.1f,C=%.1f] "), Latest->GetSpeed(EOHReferenceSpace::WorldSpace),
                Latest->GetSpeed(EOHReferenceSpace::LocalSpace), Latest->GetSpeed(EOHReferenceSpace::ComponentSpace));
        }

        // Strike detection status
        DebugInfo +=
            FString::Printf(TEXT("Strike[%s Peak=%.1f] "), bLikelyStrike ? TEXT("YES") : TEXT("NO"), PeakAcceleration);

        // Cache status
        DebugInfo += FString::Printf(TEXT("Cache[Q=%s RM=%s SM=%s] "), bMotionQualityCacheValid ? TEXT("✓") : TEXT("✗"),
                                     bRelativeMotionCacheValid ? TEXT("✓") : TEXT("✗"),
                                     bStrikeMetricsCacheValid ? TEXT("✓") : TEXT("✗"));

        if (bIncludeRelativeMotion && CachedRelativeSpeeds.Num() > 0) {
            DebugInfo += TEXT("\n  RelativeMotion: ");
            for (const auto& Pair : CachedRelativeSpeeds) {
                DebugInfo += FString::Printf(TEXT("[%s:%.1f] "), *Pair.Key.ToString(), Pair.Value);
            }
        }

        if (bIncludeTemporalAnalysis) {
            float Stability = GetMotionStability(0.5f, EOHReferenceSpace::LocalSpace);
            float Quality = GetMotionQuality(EOHReferenceSpace::LocalSpace);
            DebugInfo += FString::Printf(TEXT("\n  Temporal: Stability=%.2f Quality=%.2f"), Stability, Quality);
        }

        return DebugInfo;
    }

    /**
     * Log comprehensive debug information
     * @param bVerbose - Include detailed analysis data
     * @param Verbosity - Log verbosity level
     */
    void LogDebugInfo(bool bVerbose = false, ELogVerbosity::Type Verbosity = ELogVerbosity::Log) const {
        FString DebugInfo = GetDebugString(bVerbose, bVerbose);

        switch (Verbosity) {
        case ELogVerbosity::Error:
            UE_LOG(LogPACManager, Error, TEXT("[BoneMotion] %s"), *DebugInfo);
            break;
        case ELogVerbosity::Warning:
            UE_LOG(LogPACManager, Warning, TEXT("[BoneMotion] %s"), *DebugInfo);
            break;
        case ELogVerbosity::VeryVerbose:
            UE_LOG(LogPACManager, VeryVerbose, TEXT("[BoneMotion] %s"), *DebugInfo);
            break;
        default:
            UE_LOG(LogPACManager, Log, TEXT("[BoneMotion] %s"), *DebugInfo);
            break;
        }
    }

    // ============================================================================
    // PRESERVED API: All Existing Methods Unchanged
    // ============================================================================

    /**
     * Initialize motion data for specific bone (PRESERVED)
     */
    void Initialize(FName InBoneName) {
        BoneName = InBoneName;
        ClearMotionHistory();
        PeakAcceleration = 0.0f;
        AccelerationBuildupTime = 0.0f;
        bLikelyStrike = false;
        InvalidateAllCaches();
    }

    void ClearMotionHistory() {
        MotionHistory.Num = 0;   // Reset element count
        MotionHistory.Start = 0; // Reset circular buffer start
    }

    /**
     * Original motion sample addition method (PRESERVED)
     */
    void AddMotionSample(const FVector& WorldPos, const FQuat& WorldRot, const FVector& ComponentVelocity,
                         const FVector& ReferenceVelocity, const FTransform& ComponentTransform, float DeltaTime,
                         float TimeStamp) {
        // Create motion sample using original method
        FOHMotionFrameSample NewSample = FOHMotionFrameSample::CreateFromState(
            WorldPos, WorldRot, ComponentVelocity, FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector,
            TimeStamp, ComponentTransform, ReferenceVelocity);

        // Add to history and update metrics
        MotionHistory.Add(NewSample);
        InvalidateAllCaches();
    }

    /**
     * Get latest motion sample with validation (PRESERVED)
     */
    FORCEINLINE const FOHMotionFrameSample* GetLatestSample() const {
        return MotionHistory.NumFrames() > 0 ? MotionHistory.GetLatest(0) : nullptr;
    }

    /**
     * Get historical sample with bounds checking (PRESERVED)
     */
    FORCEINLINE const FOHMotionFrameSample* GetHistoricalSample(int32 FramesBack) const {
        return (FramesBack >= 0 && FramesBack < MotionHistory.NumFrames()) ? MotionHistory.GetLatest(FramesBack)
                                                                           : nullptr;
    }

    /**
     * Get motion history depth (PRESERVED)
     */
    FORCEINLINE int32 GetHistoryDepth() const {
        return MotionHistory.NumFrames();
    }

    /**
     * Get current world position with validation (PRESERVED)
     */
    FORCEINLINE FVector GetCurrentPosition() const {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->WorldPosition : FVector::ZeroVector;
    }

    /**
     * Unified velocity access with explicit reference space specification (PRESERVED)
     */
    FORCEINLINE FVector GetVelocity(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->GetVelocity(Space) : FVector::ZeroVector;
    }

    /**
     * Unified acceleration access with explicit reference space specification (PRESERVED)
     */
    FORCEINLINE FVector GetAcceleration(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->GetAcceleration(Space) : FVector::ZeroVector;
    }

    /**
     * Enhanced speed calculation with reference space awareness (PRESERVED)
     */
    FORCEINLINE float GetSpeed(EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace) const {
        return GetVelocity(Space).Size();
    }

    /**
     * Get angular velocity with validation (PRESERVED)
     */
    FORCEINLINE FVector GetAngularVelocity() const {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        return Latest ? Latest->AngularVelocity : FVector::ZeroVector;
    }

    /**
     * Get bone name (PRESERVED)
     */
    FORCEINLINE FName GetBoneName() const {
        return BoneName;
    }

    // ============================================================================
    // PRESERVED: LEGACY BOOLEAN-BASED API FOR BACKWARD COMPATIBILITY
    // ============================================================================

    /**
     * Legacy velocity access (PRESERVED)
     */
    FORCEINLINE FVector GetVelocity(bool bUseLocalSpace) const {
        return GetVelocity(bUseLocalSpace ? EOHReferenceSpace::LocalSpace : EOHReferenceSpace::WorldSpace);
    }

    /**
     * Legacy acceleration access (PRESERVED)
     */
    FORCEINLINE FVector GetAcceleration(bool bUseLocalSpace) const {
        return GetAcceleration(bUseLocalSpace ? EOHReferenceSpace::LocalSpace : EOHReferenceSpace::WorldSpace);
    }

    // ============================================================================
    // PRESERVED: EXISTING ADVANCED ANALYSIS METHODS
    // ============================================================================

    /**
     * Calculate jerk using advanced multi-sample estimation (PRESERVED)
     */
    FVector CalculateJerk(bool bUseLocalSpace = false) const {
        if (MotionHistory.NumFrames() < 2) {
            return FVector::ZeroVector;
        }

        const FOHMotionFrameSample* Current = GetLatestSample();
        const FOHMotionFrameSample* Previous = GetHistoricalSample(1);

        if (!Current || !Previous) {
            return FVector::ZeroVector;
        }

        EOHReferenceSpace Space = bUseLocalSpace ? EOHReferenceSpace::LocalSpace : EOHReferenceSpace::WorldSpace;
        return Current->CalculateJerk(*Previous, Space);
    }

    /**
     * Check if velocity is transitioning through zero (PRESERVED)
     */
    bool IsInVelocityTransition() const {
        if (MotionHistory.NumFrames() < 3) {
            return false;
        }

        const FOHMotionFrameSample* Current = GetHistoricalSample(0);
        const FOHMotionFrameSample* Previous1 = GetHistoricalSample(1);
        const FOHMotionFrameSample* Previous2 = GetHistoricalSample(2);

        if (!Current || !Previous1 || !Previous2) {
            return false;
        }

        float CurrentSpeed = Current->GetSpeed(EOHReferenceSpace::LocalSpace);
        float Previous1Speed = Previous1->GetSpeed(EOHReferenceSpace::LocalSpace);
        float Previous2Speed = Previous2->GetSpeed(EOHReferenceSpace::LocalSpace);

        const float ZeroThreshold = 10.0f; // cm/s

        return (Previous2Speed > ZeroThreshold && CurrentSpeed < ZeroThreshold) ||
               (Previous2Speed < ZeroThreshold && CurrentSpeed > ZeroThreshold) ||
               (Previous1Speed < ZeroThreshold && (Previous2Speed > ZeroThreshold || CurrentSpeed > ZeroThreshold));
    }

    // === ADDED: MISSING PREDICTION METHODS FOR ERROR RESOLUTION ===
    FVector PredictFuturePosition(float PredictTime, EOHReferenceSpace Space = EOHReferenceSpace::WorldSpace,
                                  bool bUseAcceleration = true) const {
        const FOHMotionFrameSample* Latest = GetLatestSample();
        if (!Latest)
            return FVector::ZeroVector;

        FVector CurrentPos = Latest->WorldPosition;
        FVector Velocity = Latest->GetVelocity(Space);

        if (bUseAcceleration && MotionHistory.NumFrames() >= 2) {
            FVector Acceleration = Latest->GetAcceleration(Space);
            return CurrentPos + Velocity * PredictTime + 0.5f * Acceleration * PredictTime * PredictTime;
        }

        return CurrentPos + Velocity * PredictTime;
    }

    TArray<FVector> GetBezierPredictionPath(float PredictTime, bool bCubic = true, int32 NumSamples = 20) const {
        // Get control points from UOHAlgoUtils
        TArray<FVector> ControlPoints = GetBezierControlPointsFromBoneData(PredictTime, bCubic);

        TArray<FVector> Path;
        Path.Reserve(NumSamples + 1);

        if (bCubic && ControlPoints.Num() >= 4) {
            // Sample cubic Bezier curve
            for (int32 i = 0; i <= NumSamples; i++) {
                float t = static_cast<float>(i) / static_cast<float>(NumSamples);
                FVector Point =
                    SampleBezierCubic(ControlPoints[0], ControlPoints[1], ControlPoints[2], ControlPoints[3], t);
                Path.Add(Point);
            }
        } else if (ControlPoints.Num() >= 3) {
            // Sample quadratic Bezier curve
            for (int32 i = 0; i <= NumSamples; i++) {
                float t = static_cast<float>(i) / static_cast<float>(NumSamples);
                FVector Point = SampleBezierQuadratic(ControlPoints[0], ControlPoints[1], ControlPoints[2], t);
                Path.Add(Point);
            }
        } else {
            // Fallback to linear interpolation
            Path.Add(GetCurrentPosition());
            if (ControlPoints.Num() > 0) {
                Path.Add(ControlPoints.Last());
            }
        }

        return Path;
    }

    bool DetectSuddenDirectionChange(float AngleThreshold = 45.0f, int32 SamplesToCheck = 3) const {
        if (MotionHistory.NumFrames() < SamplesToCheck)
            return false;

        int32 StartIdx = FMath::Max(0, MotionHistory.NumFrames() - SamplesToCheck);

        for (int32 i = StartIdx; i < MotionHistory.NumFrames() - 1; i++) {
            const FOHMotionFrameSample& Sample1 = MotionHistory[i];
            const FOHMotionFrameSample& Sample2 = MotionHistory[i + 1];

            FVector V1 = Sample1.WorldLinearVelocity;
            FVector V2 = Sample2.WorldLinearVelocity;

            if (!V1.IsNearlyZero() && !V2.IsNearlyZero()) {
                float Dot = FVector::DotProduct(V1.GetSafeNormal(), V2.GetSafeNormal());
                float Angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.0f, 1.0f)));
                if (Angle > AngleThreshold) {
                    return true;
                }
            }
        }

        return false;
    }

    TArray<FVector> GetBezierControlPointsFromBoneData(float PredictTime, bool bCubic) const {
        TArray<FVector> ControlPoints;
        const FOHMotionFrameSample* Latest = MotionHistory.GetLatest();

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
            FVector EndPoint =
                CurrentPos + CurrentVel * PredictTime + CurrentAccel * (PredictTime * PredictTime * 0.5f);
            ControlPoints.Add(EndPoint);
        } else {
            // Quadratic Bezier (3 control points)
            ControlPoints.Add(CurrentPos); // P0 - start point

            // P1 - control point
            ControlPoints.Add(CurrentPos + CurrentVel * (PredictTime * 0.5f) +
                              CurrentAccel * (PredictTime * PredictTime * 0.125f));

            // P2 - end point
            ControlPoints.Add(CurrentPos + CurrentVel * PredictTime +
                              CurrentAccel * (PredictTime * PredictTime * 0.5f));
        }

        return ControlPoints;
    }

    static FVector SampleBezierCubic(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& P3,
                                     float T) {
        const FVector A = FMath::Lerp(P0, P1, T);
        const FVector B = FMath::Lerp(P1, P2, T);
        const FVector C = FMath::Lerp(P2, P3, T);

        const FVector AB = FMath::Lerp(A, B, T);
        const FVector BC = FMath::Lerp(B, C, T);

        return FMath::Lerp(AB, BC, T);
    }

    static FVector SampleBezierQuadratic(const FVector& P0, const FVector& P1, const FVector& P2, float T) {
        const FVector A = FMath::Lerp(P0, P1, T);
        const FVector B = FMath::Lerp(P1, P2, T);
        return FMath::Lerp(A, B, T);
    }

  private:
    // ============================================================================
    // ENHANCED: INTERNAL CALCULATION METHODS
    // ============================================================================

    /**
     * Internal motion quality calculation with multi-factor analysis
     * @param Space - Reference space for quality analysis
     * @return Motion quality score (0.0 to 1.0)
     */
    float CalculateMotionQualityInternal(EOHReferenceSpace Space) const {
        if (MotionHistory.NumFrames() < 3) {
            return 0.0f;
        }

        float VelocityConsistency = 0.0f;
        float AccelerationStability = 0.0f;
        float DirectionalStability = 0.0f;
        int32 ValidComparisons = 0;

        // Analyze motion consistency using enhanced sample validation
        int32 SampleCount = FMath::Min(10, MotionHistory.NumFrames());
        for (int32 i = 0; i < SampleCount - 1; i++) {
            const FOHMotionFrameSample* Current = GetHistoricalSample(i);
            const FOHMotionFrameSample* Next = GetHistoricalSample(i + 1);

            if (Current && Next && Current->IsValidFast() && Next->IsValidFast()) {
                FVector CurrentVel = Current->GetVelocity(Space);
                FVector NextVel = Next->GetVelocity(Space);

                // Velocity consistency (lower variance = higher quality)
                FVector VelDiff = CurrentVel - NextVel;
                VelocityConsistency += VelDiff.SizeSquared();

                // Acceleration stability
                FVector CurrentAccel = Current->GetAcceleration(Space);
                FVector NextAccel = Next->GetAcceleration(Space);
                FVector AccelDiff = CurrentAccel - NextAccel;
                AccelerationStability += AccelDiff.SizeSquared();

                // Directional stability
                if (!CurrentVel.IsNearlyZero(1.0f) && !NextVel.IsNearlyZero(1.0f)) {
                    float DotProduct = FVector::DotProduct(CurrentVel.GetSafeNormal(), NextVel.GetSafeNormal());
                    DirectionalStability += FMath::Clamp(DotProduct, 0.0f, 1.0f);
                }

                ValidComparisons++;
            }
        }

        if (ValidComparisons == 0) {
            return 0.0f;
        }

        // Normalize metrics
        VelocityConsistency = FMath::Clamp(1.0f - (VelocityConsistency / (ValidComparisons * 10000.0f)), 0.0f, 1.0f);
        AccelerationStability =
            FMath::Clamp(1.0f - (AccelerationStability / (ValidComparisons * 50000.0f)), 0.0f, 1.0f);
        DirectionalStability = ValidComparisons > 0 ? (DirectionalStability / ValidComparisons) : 0.0f;

        // Weighted combination
        float QualityScore =
            (VelocityConsistency * 0.4f) + (AccelerationStability * 0.3f) + (DirectionalStability * 0.3f);
        return FMath::Clamp(QualityScore, 0.0f, 1.0f);
    }

    /**
     * Update strike detection metrics using validated motion data
     */
    void UpdateStrikeDetectionMetrics() {
        if (MotionHistory.NumFrames() < 2) {
            return;
        }

        const FOHMotionFrameSample* Latest = GetLatestSample();
        if (!Latest || !Latest->IsValidFast()) {
            return;
        }

        // Update peak acceleration tracking
        float CurrentAccelMagnitude = Latest->GetAcceleration(EOHReferenceSpace::LocalSpace).Size();
        if (CurrentAccelMagnitude > PeakAcceleration) {
            PeakAcceleration = CurrentAccelMagnitude;
            AccelerationBuildupTime = Latest->GetTimeStamp();
        }

        // Update strike likelihood using jerk analysis
        float CurrentJerk = CalculateJerkMagnitude(EOHReferenceSpace::LocalSpace, false);
        float CurrentSpeed = Latest->GetSpeed(EOHReferenceSpace::LocalSpace);

        // Strike detection criteria: high acceleration + high jerk + significant speed
        bLikelyStrike = (CurrentAccelMagnitude > 1000.0f) && (CurrentJerk > 5000.0f) && (CurrentSpeed > 200.0f);

        // Mark strike metrics cache as valid
        bStrikeMetricsCacheValid = true;
    }

    /**
     * Calculate motion consistency over sample window
     * @param WindowSamples - Samples to analyze
     * @param Space - Reference space for analysis
     * @return Consistency score (0.0 to 1.0)
     */
    static float CalculateMotionConsistencyOverWindow(const TArray<const FOHMotionFrameSample*>& WindowSamples,
                                                      EOHReferenceSpace Space) {
        if (WindowSamples.Num() < 2) {
            return 0.0f;
        }

        float ConsistencySum = 0.0f;
        int32 ValidComparisons = 0;

        for (int32 i = 0; i < WindowSamples.Num() - 1; i++) {
            const FOHMotionFrameSample* Current = WindowSamples[i];
            const FOHMotionFrameSample* Next = WindowSamples[i + 1];

            if (Current && Next && Current->IsValidFast() && Next->IsValidFast()) {
                FVector CurrentVel = Current->GetVelocity(Space);
                FVector NextVel = Next->GetVelocity(Space);

                if (!CurrentVel.IsNearlyZero(1.0f) && !NextVel.IsNearlyZero(1.0f)) {
                    float DirectionConsistency =
                        FVector::DotProduct(CurrentVel.GetSafeNormal(), NextVel.GetSafeNormal());
                    ConsistencySum += FMath::Clamp(DirectionConsistency, 0.0f, 1.0f);
                    ValidComparisons++;
                }
            }
        }

        return ValidComparisons > 0 ? (ConsistencySum / ValidComparisons) : 0.0f;
    }
};

// ============================================================================
// Enhanced FOHCombatChainData - Self-Aware Combat Chain Intelligence System
// OnlyHands Project - Unreal Engine 5.3.2
//
// ENHANCEMENT SUMMARY:
// - Added comprehensive chain-to-chain relative motion analysis
// - Implemented combat coordination intelligence for multi-limb attacks
// - Enhanced performance through smart motion significance detection
// - Integrated validation pipeline using enhanced FOHBoneMotionData methods
// - Added temporal combat pattern analysis and attack effectiveness metrics
//
// BACKWARD COMPATIBILITY:
// All existing properties and methods preserved. Enhanced functionality
// accessed via new methods leveraging enhanced FOHBoneMotionData capabilities.
// ============================================================================

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHCombatChainData {
    GENERATED_BODY()

    // =============== CHAIN DEFINITION (PRESERVED - NO CHANGES) ===============

    UPROPERTY(BlueprintReadWrite, Category = "Chain Definition")
    FName RootBone = NAME_None; // e.g., "hand_r" or "foot_l"

    UPROPERTY(BlueprintReadWrite, Category = "Chain Definition")
    TArray<FName> ChainBones; // Ordered from distal to proximal

    UPROPERTY(BlueprintReadWrite, Category = "Chain Definition")
    TArray<float> BoneMasses; // Mass estimates for each bone

    UPROPERTY(BlueprintReadWrite, Category = "Chain Definition")
    TArray<float> BoneRadii; // Collision radii for each bone

    // =============== MOTION DATA INTEGRATION (PRESERVED) ===============

    UPROPERTY(BlueprintReadWrite, Category = "Motion Data")
    TMap<FName, FOHBoneMotionData> ChainMotionData; // Full motion data per bone

    UPROPERTY(BlueprintReadWrite, Category = "Motion Data")
    TArray<FOHMotionFrameSample> CurrentFrameSamples; // Latest frame samples

    // =============== REFERENCE SPACE CONFIGURATION (PRESERVED) ===============

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Detection")
    EOHReferenceSpace MotionAnalysisSpace = EOHReferenceSpace::LocalSpace;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Detection")
    FName MotionReferenceFrameBone = "pelvis";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motion Detection")
    float MotionDetectionThreshold = 200.0f;

    // =============== CHAIN METRICS (PRESERVED) ===============

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    float ChainLength = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    FVector ChainCenterOfMass = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    float ChainTotalMass = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    FVector ChainMomentum = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    FVector ChainAngularMomentum = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Chain Metrics")
    float ChainKineticEnergy = 0.0f;

    // =============== MULTI-SPACE MOMENTUM TRACKING (PRESERVED) ===============

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainMomentumWorld = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainMomentumLocal = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainMomentumComponent = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainVelocityWorld = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainVelocityLocal = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Multi-Space Analysis")
    FVector ChainVelocityComponent = FVector::ZeroVector;

    // =============== COMBAT STATE TRACKING (PRESERVED) ===============

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    bool bIsAttacking = false;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float AttackConfidence = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float TimeInAttack = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float EffectiveStrikeRadius = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float ChainMotionQuality = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float MotionCoherence = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat State")
    float TrajectoryConfidence = 0.0f;

    // =============== ATTACK DIRECTION TRACKING ===============

    /**
     * AttackDirection - Instantaneous attack direction vector
     *
     * Updated every frame during active attacks based on the current velocity
     * of the primary striking bone. This represents the raw, unfiltered direction
     * of motion and can be noisy during complex movements.
     *
     * Use Cases:
     * - Immediate hit reaction direction
     * - Visual effects alignment
     * - Raw motion data for analysis
     *
     * Reference Space: Determined by MotionAnalysisSpace
     */
    UPROPERTY(BlueprintReadOnly, Category = "Combat State|Direction")
    FVector AttackDirection = FVector::ZeroVector;

    /**
     * AttackPrincipalDirection - Coherence-weighted principal attack vector
     *
     * Calculated using weighted averaging of all chain bone velocities above
     * motion threshold. This provides a stable, noise-filtered direction that
     * represents the true intent of the attack motion.
     *
     * Calculation includes:
     * - Velocity-weighted bone contributions
     * - Motion coherence validation
     * - Temporal smoothing via frame history
     *
     * Use Cases:
     * - Attack pattern classification
     * - Predictive targeting
     * - Combo system direction matching
     * - Distinguishing feints from committed strikes
     *
     * Reference Space: Determined by MotionAnalysisSpace
     */
    UPROPERTY(BlueprintReadOnly, Category = "Combat State|Direction")
    FVector AttackPrincipalDirection = FVector::ZeroVector;

    // =============== ENHANCED COMBAT METRICS (PRESERVED) ===============

    UPROPERTY(BlueprintReadOnly, Category = "Enhanced Combat")
    float AttackJerkMagnitude = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Enhanced Combat")
    float AttackCurvature = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Enhanced Combat")
    float AttackIntensity = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Enhanced Combat")
    float AttackDirectionalStability = 0.0f;

    // =============== ENHANCED: CHAIN COORDINATION SYSTEM ===============

  private:
    // Cached chain coordination data for performance optimization
    mutable TMap<FName, float> CachedChainCoordinationScores;
    mutable TMap<FName, FVector> CachedRelativeChainVelocities;
    mutable TMap<FName, float> CachedChainClosingRates;
    mutable bool bChainCoordinationCacheValid = false;

    // Smart processing flags
    mutable bool bHasValidMotionData = false;
    mutable float LastMotionValidationTime = 0.0f;

    // Coordination analysis cache
    mutable float CachedOverallCoordinationScore = -1.0f;
    mutable bool bCoordinationCacheValid = false;

  public:
    // ============================================================================
    // ENHANCED: CHAIN-TO-CHAIN RELATIVE MOTION ANALYSIS SYSTEM
    // ============================================================================

    /**
     * Calculate relative velocity between this chain and another chain
     * Leverages enhanced FOHBoneMotionData relative motion capabilities
     * @param Other - Other combat chain for comparison
     * @param Space - Reference space for velocity comparison
     * @return Relative velocity vector (this chain velocity - other chain velocity)
     */
    FVector CalculateRelativeChainVelocity(const FOHCombatChainData& Other, EOHReferenceSpace Space) const {
        // Validate both chains have motion data
        if (!HasValidChainMotionData() || !Other.HasValidChainMotionData()) {
            return FVector::ZeroVector;
        }

        // Check cache for performance optimization
        FName OtherChainKey = Other.GetRootBone();
        if (bChainCoordinationCacheValid) {
            if (const FVector* CachedVelocity = CachedRelativeChainVelocities.Find(OtherChainKey)) {
                return *CachedVelocity;
            }
        }

        // Get chain velocities using existing multi-space system
        FVector ThisChainVelocity = GetChainVelocity(Space);
        FVector OtherChainVelocity = Other.GetChainVelocity(Space);

        // Calculate relative velocity
        FVector RelativeVelocity = ThisChainVelocity - OtherChainVelocity;

        // Validate result
        if (RelativeVelocity.ContainsNaN()) {
            RelativeVelocity = FVector::ZeroVector;
        }

        // Cache result for performance
        CachedRelativeChainVelocities.Add(OtherChainKey, RelativeVelocity);

        return RelativeVelocity;
    }

    /**
     * Calculate relative speed between chains (magnitude of relative velocity)
     * @param Other - Other combat chain for comparison
     * @param Space - Reference space for speed comparison
     * @return Relative speed magnitude
     */
    FORCEINLINE float CalculateRelativeChainSpeed(const FOHCombatChainData& Other, EOHReferenceSpace Space) const {
        return CalculateRelativeChainVelocity(Other, Space).Size();
    }

    /**
     * Calculate closing rate between chains using center of mass analysis
     * Critical for multi-chain combat impact calculations
     * @param Other - Other combat chain for comparison
     * @param Space - Reference space for calculation
     * @return Closing rate (positive = chains approaching, negative = separating)
     */
    float CalculateChainClosingRate(const FOHCombatChainData& Other, EOHReferenceSpace Space) const {
        if (!HasValidChainMotionData() || !Other.HasValidChainMotionData()) {
            return 0.0f;
        }

        // Check cache for performance
        FName OtherChainKey = Other.GetRootBone();
        if (bChainCoordinationCacheValid) {
            if (const float* CachedRate = CachedChainClosingRates.Find(OtherChainKey)) {
                return *CachedRate;
            }
        }

        // Get chain center of mass for both chains
        FVector ThisCenterOfMass = GetChainCenterOfMass();
        FVector OtherCenterOfMass = Other.GetChainCenterOfMass();

        if (ThisCenterOfMass.ContainsNaN() || OtherCenterOfMass.ContainsNaN()) {
            return 0.0f;
        }

        // Calculate separation vector
        FVector SeparationVector = OtherCenterOfMass - ThisCenterOfMass;
        if (SeparationVector.IsNearlyZero()) {
            return 0.0f; // Same position
        }

        // Get relative velocity
        FVector RelativeVelocity = CalculateRelativeChainVelocity(Other, Space);

        // Project relative velocity onto separation vector
        FVector SeparationDirection = SeparationVector.GetSafeNormal();
        float ClosingRate = FVector::DotProduct(RelativeVelocity, SeparationDirection);

        // Cache result
        CachedChainClosingRates.Add(OtherChainKey, ClosingRate);

        return FMath::IsNaN(ClosingRate) ? 0.0f : ClosingRate;
    }

    /**
     * Comprehensive relative motion analysis between chains
     * @param Other - Other combat chain for comparison
     * @param Space - Reference space for analysis
     * @param OutRelativeVelocity - Relative velocity vector
     * @param OutRelativeSpeed - Relative speed magnitude
     * @param OutClosingRate - Rate of approach/separation
     * @param OutSeparationDistance - Current distance between chain centers
     * @return True if calculation successful, false if insufficient data
     */
    bool CalculateRelativeChainMotion(const FOHCombatChainData& Other, EOHReferenceSpace Space,
                                      FVector& OutRelativeVelocity, float& OutRelativeSpeed, float& OutClosingRate,
                                      float& OutSeparationDistance) const {
        // Initialize outputs
        OutRelativeVelocity = FVector::ZeroVector;
        OutRelativeSpeed = 0.0f;
        OutClosingRate = 0.0f;
        OutSeparationDistance = 0.0f;

        // Validation
        if (!HasValidChainMotionData() || !Other.HasValidChainMotionData()) {
            return false;
        }

        // Calculate all relative motion metrics
        OutRelativeVelocity = CalculateRelativeChainVelocity(Other, Space);
        OutRelativeSpeed = OutRelativeVelocity.Size();
        OutClosingRate = CalculateChainClosingRate(Other, Space);

        // Calculate separation distance using center of mass
        FVector ThisCenter = GetChainCenterOfMass();
        FVector OtherCenter = Other.GetChainCenterOfMass();
        OutSeparationDistance = FVector::Dist(ThisCenter, OtherCenter);

        return true;
    }

    /**
     * Smart motion significance detection for combat optimization
     * @param Other - Other combat chain for comparison
     * @param Threshold - Minimum relative speed to consider significant (cm/s)
     * @param Space - Reference space for threshold comparison
     * @return True if chains have significant relative motion above threshold
     */
    bool HasSignificantRelativeMotion(const FOHCombatChainData& Other, float Threshold,
                                      EOHReferenceSpace Space = EOHReferenceSpace::LocalSpace) const {
        // Quick validation checks
        if (!HasValidChainMotionData() || !Other.HasValidChainMotionData()) {
            return false;
        }

        // Performance optimization: Check if either chain is below motion threshold
        if (!HasSignificantMotion() || !Other.HasSignificantMotion()) {
            return false;
        }

        // Calculate relative speed and compare to threshold
        float RelativeSpeed = CalculateRelativeChainSpeed(Other, Space);
        return RelativeSpeed > Threshold;
    }

    // ============================================================================
    // ENHANCED: COMBAT COORDINATION INTELLIGENCE SYSTEM
    // ============================================================================

    /**
     * Calculate coordination score between chains for combination attack detection
     * @param Other - Other combat chain for coordination analysis
     * @return Coordination score (0.0 = independent, 1.0 = perfectly coordinated)
     */
    float GetChainCoordinationScore(const FOHCombatChainData& Other) const {
        if (!HasValidChainMotionData() || !Other.HasValidChainMotionData()) {
            return 0.0f;
        }

        // Check cache for performance
        FName OtherChainKey = Other.GetRootBone();
        if (bChainCoordinationCacheValid) {
            if (const float* CachedScore = CachedChainCoordinationScores.Find(OtherChainKey)) {
                return *CachedScore;
            }
        }

        float CoordinationScore = 0.0f;

        // === VELOCITY COHERENCE ANALYSIS ===
        FVector ThisVelocity = GetChainVelocity(MotionAnalysisSpace);
        FVector OtherVelocity = Other.GetChainVelocity(MotionAnalysisSpace);

        if (!ThisVelocity.IsNearlyZero(1.0f) && !OtherVelocity.IsNearlyZero(1.0f)) {
            float VelocityCoherence = FVector::DotProduct(ThisVelocity.GetSafeNormal(), OtherVelocity.GetSafeNormal());
            CoordinationScore += FMath::Clamp(VelocityCoherence, 0.0f, 1.0f) * 0.4f;
        }

        // === TIMING SYNCHRONIZATION ANALYSIS ===
        float TimingDifference = FMath::Abs(TimeInAttack - Other.TimeInAttack);
        float TimingSynchronization = FMath::Clamp(1.0f - (TimingDifference / 0.5f), 0.0f, 1.0f);
        CoordinationScore += TimingSynchronization * 0.3f;

        // === ATTACK CONFIDENCE CORRELATION ===
        float ConfidenceProduct = AttackConfidence * Other.AttackConfidence;
        CoordinationScore += ConfidenceProduct * 0.2f;

        // === MOTION QUALITY CORRELATION ===
        float QualityCorrelation = FMath::Min(ChainMotionQuality, Other.ChainMotionQuality);
        CoordinationScore += QualityCorrelation * 0.1f;

        // Cache result
        CachedChainCoordinationScores.Add(OtherChainKey, CoordinationScore);

        return FMath::Clamp(CoordinationScore, 0.0f, 1.0f);
    }

    /**
     * Analyze coordination with multiple chains for complex attack pattern detection
     * @param OtherChains - Array of other combat chains for analysis
     * @param OutCoordinatedChains - Chains with significant coordination (score > 0.5)
     * @param OutAverageCoordination - Average coordination score across all chains
     * @return Number of significantly coordinated chains
     */
    int32 AnalyzeCombatCoordination(const TArray<const FOHCombatChainData*>& OtherChains,
                                    TArray<FName>& OutCoordinatedChains, float& OutAverageCoordination) const {
        OutCoordinatedChains.Reset();
        OutAverageCoordination = 0.0f;

        if (!HasValidChainMotionData() || OtherChains.Num() == 0) {
            return 0;
        }

        float TotalCoordination = 0.0f;
        int32 ValidComparisons = 0;
        int32 SignificantCoordinations = 0;

        for (const FOHCombatChainData* OtherChain : OtherChains) {
            if (OtherChain && OtherChain->HasValidChainMotionData()) {
                float CoordinationScore = GetChainCoordinationScore(*OtherChain);
                TotalCoordination += CoordinationScore;
                ValidComparisons++;

                // Threshold for significant coordination
                if (CoordinationScore > 0.5f) {
                    OutCoordinatedChains.Add(OtherChain->GetRootBone());
                    SignificantCoordinations++;
                }
            }
        }

        OutAverageCoordination = ValidComparisons > 0 ? (TotalCoordination / ValidComparisons) : 0.0f;

        return SignificantCoordinations;
    }

    // ============================================================================
    // ENHANCED: CHAIN COMBAT EFFECTIVENESS SYSTEM
    // ============================================================================

    /**
     * Calculate chain combat potential with multi-chain context awareness
     * @param TargetChain - Target chain for impact assessment (optional)
     * @return Combat potential score incorporating kinetic energy, coordination, and attack quality
     */
    float CalculateChainCombatPotential(const FOHCombatChainData* TargetChain = nullptr) const {
        if (!HasValidChainMotionData()) {
            return 0.0f;
        }

        float CombatPotential = 0.0f;

        // === BASE KINETIC ENERGY COMPONENT ===
        float NormalizedKineticEnergy = FMath::Clamp(ChainKineticEnergy / 10000.0f, 0.0f, 1.0f);
        CombatPotential += NormalizedKineticEnergy * 0.3f;

        // === ATTACK CONFIDENCE COMPONENT ===
        CombatPotential += AttackConfidence * 0.25f;

        // === MOTION QUALITY COMPONENT ===
        CombatPotential += ChainMotionQuality * 0.2f;

        // === JERK AND INTENSITY COMPONENT ===
        float NormalizedJerk = FMath::Clamp(AttackJerkMagnitude / 20000.0f, 0.0f, 1.0f);
        float NormalizedIntensity = FMath::Clamp(AttackIntensity / 100.0f, 0.0f, 1.0f);
        CombatPotential += (NormalizedJerk + NormalizedIntensity) * 0.125f;

        // === TARGET-SPECIFIC ADJUSTMENTS ===
        if (TargetChain && TargetChain->HasValidChainMotionData()) {
            // Closing rate bonus for approaching targets
            float ClosingRate = CalculateChainClosingRate(*TargetChain, MotionAnalysisSpace);
            if (ClosingRate > 0.0f) // Approaching target
            {
                float ClosingBonus = FMath::Clamp(ClosingRate / 1000.0f, 0.0f, 0.2f);
                CombatPotential += ClosingBonus;
            }

            // Size differential consideration
            float SizeDifferential = FMath::Abs(ChainLength - TargetChain->ChainLength);
            float SizeAdvantage = FMath::Clamp(1.0f - (SizeDifferential / 100.0f), 0.0f, 0.1f);
            CombatPotential += SizeAdvantage;
        }

        return FMath::Clamp(CombatPotential, 0.0f, 1.0f);
    }

    /**
     * Calculate optimal impact force for collision with target chain
     * Uses closing rate and chain momentum for physics-based force calculation
     * @param TargetChain - Target chain for impact calculation
     * @param ImpactEfficiencyFactor - Efficiency of force transfer (0.0 to 1.0)
     * @return Optimal impact force magnitude
     */
    float CalculateOptimalImpactForce(const FOHCombatChainData& TargetChain,
                                      float ImpactEfficiencyFactor = 0.7f) const {
        if (!HasValidChainMotionData() || !TargetChain.HasValidChainMotionData()) {
            return 0.0f;
        }

        // Calculate closing rate for relative momentum
        float ClosingRate = CalculateChainClosingRate(TargetChain, MotionAnalysisSpace);
        if (ClosingRate <= 0.0f) {
            return 0.0f; // No impact if not approaching
        }

        // Use reduced mass for collision calculation
        float ThisMass = GetChainTotalMass();
        float TargetMass = TargetChain.GetChainTotalMass();

        if (ThisMass <= KINDA_SMALL_NUMBER || TargetMass <= KINDA_SMALL_NUMBER) {
            return 0.0f;
        }

        float ReducedMass = (ThisMass * TargetMass) / (ThisMass + TargetMass);

        // Calculate impact force using momentum transfer
        float BaseImpactForce = ReducedMass * ClosingRate;

        // Apply efficiency factor and attack quality modifiers
        float QualityModifier = FMath::Lerp(0.5f, 1.5f, ChainMotionQuality);
        float ConfidenceModifier = FMath::Lerp(0.7f, 1.3f, AttackConfidence);

        float OptimalForce = BaseImpactForce * ImpactEfficiencyFactor * QualityModifier * ConfidenceModifier;

        return FMath::Clamp(OptimalForce, 0.0f, 50000.0f); // Safety clamp
    }

    // ============================================================================
    // ENHANCED: SMART PROCESSING AND VALIDATION INTEGRATION
    // ============================================================================

    /**
     * Enhanced chain motion data validation using FOHBoneMotionData capabilities
     * @return True if chain contains valid motion data for all bones
     */
    bool HasValidChainMotionData() const {
        // Cache validation result for performance
        float CurrentTime = FPlatformTime::Seconds();
        if (CurrentTime - LastMotionValidationTime < 0.016f) // Cache for one frame
        {
            return bHasValidMotionData;
        }

        if (ChainBones.Num() == 0) {
            bHasValidMotionData = false;
            LastMotionValidationTime = CurrentTime;
            return false;
        }

        // Validate motion data for each bone in chain
        int32 ValidBones = 0;
        for (const FName& BoneName : ChainBones) {
            if (const FOHBoneMotionData* BoneData = GetBoneMotionData(BoneName)) {
                if (BoneData->HasValidMotionData()) {
                    ValidBones++;
                }
            }
        }

        // Require at least 50% of bones to have valid motion data
        bHasValidMotionData = (ValidBones >= FMath::Max(1, ChainBones.Num() / 2));
        LastMotionValidationTime = CurrentTime;

        return bHasValidMotionData;
    }

    /**
     * Enhanced UpdateChainMetrics with validation integration and selective processing
     * Integrates with enhanced FOHBoneMotionData validation and caching systems
     * @param CurrentTime - Current timestamp for time-based calculations
     * @param bForceUpdate - Force recalculation even if motion is below threshold
     */
    void UpdateChainMetricsEnhanced(float CurrentTime, bool bForceUpdate = false) {
        // Smart processing: Skip expensive calculations for inactive chains
        if (!bForceUpdate && !HasSignificantMotion()) {
            // Clear trajectory data for inactive chains
            PredictedTrajectory.Empty();
            BezierControlPoints.Empty();
            TrajectoryConfidence = 0.0f;
            InvalidateCoordinationCache();
            return;
        }

        // Invalidate caches when updating metrics
        InvalidateCoordinationCache();

        // Call original UpdateChainMetrics for core functionality
        UpdateChainMetrics(CurrentTime);

        // === ENHANCED: VALIDATION-INTEGRATED BONE PROCESSING ===
        int32 ValidatedBones = 0;
        for (const FName& BoneName : ChainBones) {
            if (const FOHBoneMotionData* BoneData = GetBoneMotionData(BoneName)) {
                // Use enhanced validation from FOHBoneMotionData
                if (BoneData->HasValidMotionData()) {
                    ValidatedBones++;

                    // Update motion metrics using enhanced bone capabilities
                    float BoneJerk = BoneData->CalculateJerkMagnitude(MotionAnalysisSpace);
                    AttackJerkMagnitude = FMath::Max(AttackJerkMagnitude, BoneJerk);
                }
            }
        }

        // === ENHANCED: MOTION QUALITY VALIDATION ===
        if (ValidatedBones < ChainBones.Num() / 2) {
            // Insufficient valid data - reduce confidence
            AttackConfidence *= 0.5f;
            ChainMotionQuality *= 0.3f;
        }

        // === ENHANCED: UPDATE ATTACK EFFECTIVENESS METRICS ===
        UpdateAttackEffectivenessMetrics();
    }

    /**
     * Batch relative motion analysis for multiple chains (performance optimized)
     * @param OtherChains - Array of other combat chains for analysis
     * @param Space - Reference space for analysis
     * @param Threshold - Minimum significance threshold
     * @param OutSignificantChains - Chains with significant relative motion
     * @return Number of chains with significant relative motion
     */
    int32 AnalyzeRelativeMotionBatch(const TArray<const FOHCombatChainData*>& OtherChains, EOHReferenceSpace Space,
                                     float Threshold, TArray<FName>& OutSignificantChains) const {
        OutSignificantChains.Reset();

        if (!HasValidChainMotionData()) {
            return 0;
        }

        for (const FOHCombatChainData* OtherChain : OtherChains) {
            if (OtherChain && HasSignificantRelativeMotion(*OtherChain, Threshold, Space)) {
                OutSignificantChains.Add(OtherChain->GetRootBone());
            }
        }

        return OutSignificantChains.Num();
    }

    // ============================================================================
    // ENHANCED: DEBUG AND DIAGNOSTIC SYSTEM
    // ============================================================================

    /**
     * Comprehensive debug information with chain coordination analysis
     * @param bIncludeCoordination - Include coordination cache data
     * @param bIncludeRelativeMotion - Include relative motion analysis
     * @return Formatted debug string
     */
    FString GetDebugString(bool bIncludeCoordination = false, bool bIncludeRelativeMotion = false) const {
        FString DebugInfo;

        // Basic chain info
        DebugInfo += FString::Printf(TEXT("CombatChain[%s] "), *RootBone.ToString());
        DebugInfo += FString::Printf(TEXT("Bones=%d Valid=%s "), ChainBones.Num(),
                                     HasValidChainMotionData() ? TEXT("✓") : TEXT("✗"));

        // Combat state
        DebugInfo += FString::Printf(TEXT("Attack[%s Conf=%.2f T=%.2f] "), bIsAttacking ? TEXT("YES") : TEXT("NO"),
                                     AttackConfidence, TimeInAttack);

        // Motion metrics
        DebugInfo += FString::Printf(TEXT("Motion[Q=%.2f Coh=%.2f Energy=%.0f] "), ChainMotionQuality, MotionCoherence,
                                     ChainKineticEnergy);

        // Multi-space velocities
        DebugInfo += FString::Printf(TEXT("Vel[W=%.1f,L=%.1f,C=%.1f] "), ChainVelocityWorld.Size(),
                                     ChainVelocityLocal.Size(), ChainVelocityComponent.Size());

        // Cache status
        DebugInfo += FString::Printf(TEXT("Cache[Coord=%s] "), bChainCoordinationCacheValid ? TEXT("✓") : TEXT("✗"));

        if (bIncludeCoordination && CachedChainCoordinationScores.Num() > 0) {
            DebugInfo += TEXT("\n  Coordination: ");
            for (const auto& Pair : CachedChainCoordinationScores) {
                DebugInfo += FString::Printf(TEXT("[%s:%.2f] "), *Pair.Key.ToString(), Pair.Value);
            }
        }

        if (bIncludeRelativeMotion && CachedRelativeChainVelocities.Num() > 0) {
            DebugInfo += TEXT("\n  RelativeMotion: ");
            for (const auto& Pair : CachedRelativeChainVelocities) {
                DebugInfo += FString::Printf(TEXT("[%s:%.1f] "), *Pair.Key.ToString(), Pair.Value.Size());
            }
        }

        return DebugInfo;
    }

    /**
     * Log comprehensive debug information
     * @param bVerbose - Include detailed coordination and relative motion data
     * @param Verbosity - Log verbosity level
     */
    void LogDebugInfo(bool bVerbose = false, ELogVerbosity::Type Verbosity = ELogVerbosity::Log) const {
        FString DebugInfo = GetDebugString(bVerbose, bVerbose);

        switch (Verbosity) {
        case ELogVerbosity::Error:
            UE_LOG(LogPACManager, Error, TEXT("[CombatChain] %s"), *DebugInfo);
            break;
        case ELogVerbosity::Warning:
            UE_LOG(LogPACManager, Warning, TEXT("[CombatChain] %s"), *DebugInfo);
            break;
        case ELogVerbosity::VeryVerbose:
            UE_LOG(LogPACManager, VeryVerbose, TEXT("[CombatChain] %s"), *DebugInfo);
            break;
        default:
            UE_LOG(LogPACManager, Log, TEXT("[CombatChain] %s"), *DebugInfo);
            break;
        }
    }

    // ============================================================================
    // PRESERVED API: All Existing Methods Unchanged
    // ============================================================================

    /**
     * Get bone motion data from chain (PRESERVED)
     */
    FORCEINLINE const FOHBoneMotionData* GetBoneMotionData(FName BoneName) const {
        return ChainMotionData.Find(BoneName);
    }

    /**
     * Get chain velocity in specified reference space (PRESERVED)
     */
    FORCEINLINE FVector GetChainVelocity(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return ChainVelocityWorld;
        case EOHReferenceSpace::LocalSpace:
            return ChainVelocityLocal;
        case EOHReferenceSpace::ComponentSpace:
            return ChainVelocityComponent;
        default:
            return ChainVelocityWorld;
        }
    }

    /**
     * Get chain momentum in specified reference space (PRESERVED)
     */
    FORCEINLINE FVector GetChainMomentum(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return ChainMomentumWorld;
        case EOHReferenceSpace::LocalSpace:
            return ChainMomentumLocal;
        case EOHReferenceSpace::ComponentSpace:
            return ChainMomentumComponent;
        default:
            return ChainMomentum;
        }
    }

    /**
     * Get chain center of mass (PRESERVED)
     */
    FORCEINLINE FVector GetChainCenterOfMass() const {
        return ChainCenterOfMass;
    }

    /**
     * Get chain total mass (PRESERVED)
     */
    FORCEINLINE float GetChainTotalMass() const {
        return ChainTotalMass;
    }

    /**
     * Get root bone name (PRESERVED)
     */
    FORCEINLINE FName GetRootBone() const {
        return RootBone;
    }

    /**
     * Check if chain has significant motion (PRESERVED)
     */
    FORCEINLINE bool HasSignificantMotion() const {
        return GetEffectiveMotionSpeed() > MotionDetectionThreshold;
    }

    /**
     * Get effective motion speed (PRESERVED)
     */
    FORCEINLINE float GetEffectiveMotionSpeed() const {
        return FMath::Max3(ChainVelocityWorld.Size(), ChainVelocityLocal.Size(), ChainVelocityComponent.Size());
    }

    // =============== ATTACK DIRECTION HELPERS ===============

    /**
     * Get the most reliable attack direction based on current state
     * Returns principal direction if available and valid, otherwise raw direction
     */
    FORCEINLINE FVector GetReliableAttackDirection() const {
        if (!AttackPrincipalDirection.IsNearlyZero() && AttackConfidence > 0.5f) {
            return AttackPrincipalDirection;
        }
        return AttackDirection;
    }

    /**
     * Calculate angular difference between raw and principal directions
     * Useful for detecting feints or direction changes
     */
    FORCEINLINE float GetAttackDirectionDeviation() const {
        if (AttackDirection.IsNearlyZero() || AttackPrincipalDirection.IsNearlyZero()) {
            return 0.0f;
        }

        float DotProduct =
            FVector::DotProduct(AttackDirection.GetSafeNormal(), AttackPrincipalDirection.GetSafeNormal());
        return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f)));
    }

    /**
     * Check if attack direction is stable (low deviation between raw and principal)
     * @param MaxDeviationDegrees - Maximum allowed deviation in degrees
     */
    FORCEINLINE bool HasStableAttackDirection(float MaxDeviationDegrees = 15.0f) const {
        return GetAttackDirectionDeviation() <= MaxDeviationDegrees;
    }

    /**
     * Enhanced UpdateChainMetrics with comprehensive multi-space motion analysis (PRESERVED)
     */
    void UpdateChainMetrics(float CurrentTime) {
        if (ChainBones.Num() == 0)
            return;

        // Reset all metrics
        ChainTotalMass = 0.0f;
        ChainCenterOfMass = FVector::ZeroVector;
        ChainMomentum = FVector::ZeroVector;
        ChainAngularMomentum = FVector::ZeroVector;
        ChainKineticEnergy = 0.0f;
        ChainMotionQuality = 0.0f;

        // Reset multi-space fields
        ChainMomentumWorld = FVector::ZeroVector;
        ChainMomentumLocal = FVector::ZeroVector;
        ChainMomentumComponent = FVector::ZeroVector;
        ChainVelocityWorld = FVector::ZeroVector;
        ChainVelocityLocal = FVector::ZeroVector;
        ChainVelocityComponent = FVector::ZeroVector;

        // Process each bone in chain
        CurrentFrameSamples.SetNum(ChainBones.Num());
        int32 ValidBones = 0;

        for (int32 i = 0; i < ChainBones.Num(); i++) {
            const FName& BoneName = ChainBones[i];
            const FOHBoneMotionData* MotionData = GetBoneMotionData(BoneName);

            if (!MotionData || !BoneMasses.IsValidIndex(i))
                continue;

            if (const FOHMotionFrameSample* LatestSample = MotionData->GetLatestSample()) {
                CurrentFrameSamples[i] = *LatestSample;

                // Multi-space velocity extraction
                FVector VelocityWorld = MotionData->GetVelocity(EOHReferenceSpace::WorldSpace);
                FVector VelocityLocal = MotionData->GetVelocity(EOHReferenceSpace::LocalSpace);
                FVector VelocityComponent = MotionData->GetVelocity(EOHReferenceSpace::ComponentSpace);

                if (!VelocityWorld.ContainsNaN() && !VelocityLocal.ContainsNaN() && !VelocityComponent.ContainsNaN()) {
                    float BoneMass = BoneMasses[i];

                    // Accumulate multi-space momentum
                    ChainMomentumWorld += VelocityWorld * BoneMass;
                    ChainMomentumLocal += VelocityLocal * BoneMass;
                    ChainMomentumComponent += VelocityComponent * BoneMass;

                    // Accumulate mass and center of mass
                    ChainTotalMass += BoneMass;
                    ChainCenterOfMass += LatestSample->WorldPosition * BoneMass;

                    // Accumulate kinetic energy
                    float BoneKineticEnergy = 0.5f * BoneMass * VelocityWorld.SizeSquared();
                    ChainKineticEnergy += BoneKineticEnergy;

                    ValidBones++;
                }
            }
        }

        // Finalize calculations
        if (ValidBones > 0 && ChainTotalMass > KINDA_SMALL_NUMBER) {
            ChainCenterOfMass /= ChainTotalMass;

            // Calculate average velocities
            ChainVelocityWorld = ChainMomentumWorld / ChainTotalMass;
            ChainVelocityLocal = ChainMomentumLocal / ChainTotalMass;
            ChainVelocityComponent = ChainMomentumComponent / ChainTotalMass;

            // Set legacy momentum field for backward compatibility
            ChainMomentum = ChainMomentumWorld;
        }

        // Update other chain properties
        UpdateChainLength();
    }

    /**
     * Enhanced motion coherence calculation with reference space awareness (PRESERVED)
     */
    void UpdateMotionCoherence() {
        if (ChainBones.Num() < 2) {
            MotionCoherence = 0.0f;
            return;
        }

        float CoherenceSum = 0.0f;
        int32 ValidPairs = 0;

        for (int32 i = 0; i < ChainBones.Num() - 1; i++) {
            const FOHBoneMotionData* Current = GetBoneMotionData(ChainBones[i]);
            const FOHBoneMotionData* Next = GetBoneMotionData(ChainBones[i + 1]);

            if (Current && Next) {
                FVector V1 = Current->GetVelocity(MotionAnalysisSpace);
                FVector V2 = Next->GetVelocity(MotionAnalysisSpace);

                if (!V1.IsNearlyZero() && !V2.IsNearlyZero() && !V1.ContainsNaN() && !V2.ContainsNaN()) {
                    float Coherence = FVector::DotProduct(V1.GetSafeNormal(), V2.GetSafeNormal());
                    CoherenceSum += FMath::Clamp(Coherence, 0.0f, 1.0f);
                    ValidPairs++;
                }
            }
        }

        MotionCoherence = ValidPairs > 0 ? (CoherenceSum / ValidPairs) : 0.0f;
    }

    /**
     * Update chain length calculation (PRESERVED)
     */
    void UpdateChainLength() {
        ChainLength = 0.0f;
        for (int32 i = 0; i < ChainBones.Num() - 1; i++) {
            const FOHBoneMotionData* Current = GetBoneMotionData(ChainBones[i]);
            const FOHBoneMotionData* Next = GetBoneMotionData(ChainBones[i + 1]);

            if (Current && Next) {
                FVector Pos1 = Current->GetCurrentPosition();
                FVector Pos2 = Next->GetCurrentPosition();

                if (!Pos1.ContainsNaN() && !Pos2.ContainsNaN()) {
                    ChainLength += FVector::Dist(Pos1, Pos2);
                }
            }
        }
    }

    FName GetPrimaryStrikingBone() const {
        if (ChainBones.Num() == 0)
            return NAME_None;

        // Find the bone with highest velocity
        FName FastestBone = ChainBones[0];
        float MaxSpeed = 0.0f;

        for (const FName& BoneName : ChainBones) {
            if (const FOHBoneMotionData* MotionData = GetBoneMotionData(BoneName)) {
                float Speed = MotionData->GetSpeed(MotionAnalysisSpace);
                if (Speed > MaxSpeed) {
                    MaxSpeed = Speed;
                    FastestBone = BoneName;
                }
            }
        }

        // For limbs, prefer the distal bone
        if (RootBone.ToString().Contains(TEXT("hand")) || RootBone.ToString().Contains(TEXT("foot"))) {
            return RootBone;
        }

        return FastestBone;
    }

    void RecalculateReferenceSpaces(const FTransform& ComponentTransform, const FTransform& ReferenceTransform) {
        // Update samples in CurrentFrameSamples
        for (auto& Sample : CurrentFrameSamples) {
            Sample.RecalculateReferenceSpaces(ComponentTransform, ReferenceTransform);
        }

        // Update samples in motion data's rolling buffer
        for (auto& Pair : ChainMotionData) {
            for (int32 i = 0; i < Pair.Value.MotionHistory.NumFrames(); i++) {
                FOHMotionFrameSample& Sample = Pair.Value.MotionHistory[i];
                Sample.RecalculateReferenceSpaces(ComponentTransform, ReferenceTransform);
            }
        }
    }
    // =============== CONSTRUCTOR (Enhanced with cache initialization) ===============

    FOHCombatChainData() {
        // Original defaults maintained
        RootBone = NAME_None;
        ChainLength = 0.0f;
        ChainTotalMass = 0.0f;
        bIsAttacking = false;
        AttackConfidence = 0.0f;
        TimeInAttack = 0.0f;
        EffectiveStrikeRadius = 0.0f;
        ChainMotionQuality = 0.0f;
        MotionCoherence = 0.0f;
        TrajectoryConfidence = 0.0f;
        ChainKineticEnergy = 0.0f;

        // Enhanced defaults
        MotionAnalysisSpace = EOHReferenceSpace::LocalSpace;
        MotionReferenceFrameBone = "pelvis";
        MotionDetectionThreshold = 200.0f;
        AttackJerkMagnitude = 0.0f;
        AttackCurvature = 0.0f;
        AttackIntensity = 0.0f;
        AttackDirectionalStability = 0.0f;
        PreviousUpdateTime = 0.0f;

        // Initialize attack direction fields
        AttackDirection = FVector::ZeroVector;
        AttackPrincipalDirection = FVector::ZeroVector;

        // Initialize caches
        InvalidateCoordinationCache();
    }

    const TArray<FVector>& GetPredictedTrajectory() const {
        return PredictedTrajectory;
    }
    const TArray<FVector>& GetBezierControlPoints() const {
        return BezierControlPoints;
    }

    void SetPredictedTrajectory(const TArray<FVector>& InTrajectory) {
        PredictedTrajectory = InTrajectory;
    }
    void SetBezierControlPoints(const TArray<FVector>& InControlPoints) {
        BezierControlPoints = InControlPoints;
    }

    void ClearPredictedTrajectory() {
        PredictedTrajectory.Empty();
    }
    void ClearBezierControlPoints() {
        BezierControlPoints.Empty();
    }

  private:
    // ============================================================================
    // ENHANCED: INTERNAL COORDINATION CACHE MANAGEMENT
    // ============================================================================

    /**
     * Invalidate coordination cache when chain data changes
     */
    void InvalidateCoordinationCache() const {
        bChainCoordinationCacheValid = false;
        bCoordinationCacheValid = false;
        CachedChainCoordinationScores.Reset();
        CachedRelativeChainVelocities.Reset();
        CachedChainClosingRates.Reset();
        CachedOverallCoordinationScore = -1.0f;
    }

    /**
     * Update attack effectiveness metrics using enhanced bone data
     */
    void UpdateAttackEffectivenessMetrics() {
        // Calculate attack intensity using validated bone data
        float IntensitySum = 0.0f;
        int32 ValidBones = 0;

        for (const FName& BoneName : ChainBones) {
            if (const FOHBoneMotionData* BoneData = GetBoneMotionData(BoneName)) {
                if (BoneData->HasValidMotionData()) {
                    float BoneSpeed = BoneData->GetSpeed(MotionAnalysisSpace);
                    float BoneAccel = BoneData->GetAcceleration(MotionAnalysisSpace).Size();

                    // Intensity calculation: weighted speed and acceleration
                    float BoneIntensity = (BoneSpeed * 0.6f) + (BoneAccel * 0.4f);
                    IntensitySum += BoneIntensity;
                    ValidBones++;
                }
            }
        }

        AttackIntensity = ValidBones > 0 ? (IntensitySum / ValidBones) / 100.0f : 0.0f; // Normalize to 0-1 range

        // Calculate directional stability using motion coherence
        AttackDirectionalStability = FMath::Clamp(MotionCoherence * 1.2f, 0.0f, 1.0f);
    }

    // Time tracking for cache validation
    float PreviousUpdateTime = 0.0f;

    // PRESERVED: Prediction trajectory storage
    // UPROPERTY(BlueprintReadOnly, Category = "Prediction")
    TArray<FVector> PredictedTrajectory;

    // UPROPERTY(BlueprintReadOnly, Category = "Prediction")
    TArray<FVector> BezierControlPoints;
};

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHCombatAnalysis {
    GENERATED_BODY()

    // === YOUR EXISTING CORE METRICS (MAINTAINED) ===
    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    bool bIsAttacking = false;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    float AttackConfidence = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    float MaxChainSpeed = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    float TotalKineticEnergy = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    FName PrimaryStrikingBone = NAME_None;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    FVector AttackDirection = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    TArray<FName> ActiveChains;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    float ChainMotionQuality = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Combat Analysis")
    float AttackIntensity = 0.0f;

    // === ENHANCED: REFERENCE SPACE-AWARE MOTION METRICS ===
    UPROPERTY(BlueprintReadOnly, Category = "Motion Analysis|Multi-Space")
    float MaxChainSpeedWorld = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Analysis|Multi-Space")
    float MaxChainSpeedLocal = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Analysis|Multi-Space")
    float MaxChainSpeedComponent = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Analysis|Multi-Space")
    EOHReferenceSpace PrimaryMotionSpace = EOHReferenceSpace::LocalSpace;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Analysis|Multi-Space")
    float EffectiveMotionSpeed = 0.0f; // Speed in primary motion space

    // === ENHANCED: KINEMATIC ANALYSIS METRICS ===
    UPROPERTY(BlueprintReadOnly, Category = "Kinematic Analysis")
    float PeakAcceleration = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic Analysis")
    float PeakJerkMagnitude = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic Analysis")
    float AverageChainMass = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic Analysis")
    float MomentumMagnitude = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic Analysis")
    float AngularMomentumMagnitude = 0.0f;

    // === ENHANCED: MOTION QUALITY AND CONSISTENCY METRICS ===
    UPROPERTY(BlueprintReadOnly, Category = "Quality Analysis")
    float MotionConsistencyScore = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Quality Analysis")
    float CoordinationFactor = 0.0f; // Multi-bone coordination assessment

    UPROPERTY(BlueprintReadOnly, Category = "Quality Analysis")
    int32 ActiveBoneCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Quality Analysis")
    float TrajectoryConfidence = 0.0f;

    // === ENHANCED: ATTACK CLASSIFICATION AND CHARACTERIZATION ===
    UPROPERTY(BlueprintReadOnly, Category = "Attack Classification")
    bool bIsCommittedAttack = false;

    UPROPERTY(BlueprintReadOnly, Category = "Attack Classification")
    bool bIsFeintOrGlancing = false;

    UPROPERTY(BlueprintReadOnly, Category = "Attack Classification")
    float AttackCurvature = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Attack Classification")
    float TimeInAttack = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Attack Classification")
    float ImpactPotential = 0.0f; // Predicted impact force potential

    // === ENHANCED: SPATIAL AND DIRECTIONAL ANALYSIS ===
    UPROPERTY(BlueprintReadOnly, Category = "Spatial Analysis")
    FVector AttackPrincipalDirection = FVector::ZeroVector; // Primary attack vector in analysis space

    UPROPERTY(BlueprintReadOnly, Category = "Spatial Analysis")
    FVector ChainCenterOfMass = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Spatial Analysis")
    float EffectiveStrikeRadius = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Spatial Analysis")
    float ChainLength = 0.0f;

    // === ENHANCED API: SYSTEMATIC MOTION ANALYSIS ACCESS ===

    // Get maximum speed across all reference spaces
    FORCEINLINE float GetMaxSpeedAnySpace() const {
        return FMath::Max3(MaxChainSpeedWorld, MaxChainSpeedLocal, MaxChainSpeedComponent);
    }

    // Get speed in specified reference space
    float GetMaxSpeedInSpace(EOHReferenceSpace Space) const {
        switch (Space) {
        case EOHReferenceSpace::WorldSpace:
            return MaxChainSpeedWorld;
        case EOHReferenceSpace::LocalSpace:
            return MaxChainSpeedLocal;
        case EOHReferenceSpace::ComponentSpace:
            return MaxChainSpeedComponent;
        default:
            return MaxChainSpeed; // Fallback to existing field
        }
    }

    // Enhanced normalized intensity calculation
    FORCEINLINE float GetNormalizedIntensity() const {
        return FMath::Clamp(AttackIntensity / 100.0f, 0.0f, 1.0f);
    }

    // Get normalized impact potential for physics calculations
    FORCEINLINE float GetNormalizedImpactPotential() const {
        return FMath::Clamp(ImpactPotential / 100.0f, 0.0f, 1.0f);
    }

    // Enhanced validity check with comprehensive state validation
    bool IsValid() const {
        return !PrimaryStrikingBone.IsNone() && AttackConfidence > KINDA_SMALL_NUMBER && ActiveChains.Num() > 0 &&
               TotalKineticEnergy > KINDA_SMALL_NUMBER;
    }

    // Get motion effectiveness rating combining speed, quality, and consistency
    float GetMotionEffectiveness() const {
        if (!bIsAttacking)
            return 0.0f;

        float SpeedRating = FMath::Clamp(EffectiveMotionSpeed / 500.0f, 0.0f, 1.0f);
        float QualityRating = FMath::Clamp(ChainMotionQuality, 0.0f, 1.0f);
        float ConsistencyRating = FMath::Clamp(MotionConsistencyScore, 0.0f, 1.0f);
        float CoordinationRating = FMath::Clamp(CoordinationFactor, 0.0f, 1.0f);

        return (SpeedRating * 0.3f + QualityRating * 0.3f + ConsistencyRating * 0.2f + CoordinationRating * 0.2f);
    }

    // Determine attack classification based on motion analysis
    FString GetAttackClassification() const {
        if (!bIsAttacking)
            return TEXT("No Attack");

        if (bIsCommittedAttack) {
            return FString::Printf(TEXT("Committed Strike (%.1fs)"), TimeInAttack);
        } else if (bIsFeintOrGlancing) {
            return TEXT("Feint/Glancing");
        } else if (AttackConfidence > 0.7f && PeakJerkMagnitude > 10000.0f) {
            return TEXT("Power Strike");
        } else if (AttackConfidence > 0.6f) {
            return TEXT("Standard Attack");
        } else if (AttackConfidence > 0.3f) {
            return TEXT("Tentative Motion");
        } else {
            return TEXT("Preparation");
        }
    }

    // Get attack intensity category for UI/feedback systems
    FString GetAttackIntensityCategory() const {
        float NormalizedIntensity = GetNormalizedIntensity();

        if (NormalizedIntensity > 0.8f)
            return TEXT("Devastating");
        if (NormalizedIntensity > 0.6f)
            return TEXT("Heavy");
        if (NormalizedIntensity > 0.4f)
            return TEXT("Moderate");
        if (NormalizedIntensity > 0.2f)
            return TEXT("Light");
        return TEXT("Minimal");
    }

    // Calculate predicted impact force based on kinematic analysis
    float CalculatePredictedImpactForce() const {
        if (!bIsAttacking || TotalKineticEnergy < KINDA_SMALL_NUMBER)
            return 0.0f;

        // Base force from kinetic energy
        float BaseForce = FMath::Sqrt(TotalKineticEnergy * 2.0f * AverageChainMass);

        // Modulate by attack confidence and quality
        float ConfidenceMultiplier = FMath::Lerp(0.5f, 1.5f, AttackConfidence);
        float QualityMultiplier = FMath::Lerp(0.7f, 1.3f, ChainMotionQuality);

        // Apply jerk factor for striking intensity
        float JerkMultiplier = FMath::Clamp(PeakJerkMagnitude / 15000.0f, 0.5f, 2.0f);

        return BaseForce * ConfidenceMultiplier * QualityMultiplier * JerkMultiplier;
    }

    // Enhanced reset with all new fields
    void Reset() {
        // === YOUR ORIGINAL RESET PATTERN MAINTAINED ===
        bIsAttacking = false;
        AttackConfidence = 0.0f;
        MaxChainSpeed = 0.0f;
        TotalKineticEnergy = 0.0f;
        PrimaryStrikingBone = NAME_None;
        AttackDirection = FVector::ZeroVector;
        ActiveChains.Empty();
        ChainMotionQuality = 0.0f;
        AttackIntensity = 0.0f;

        // === ENHANCED: RESET NEW FIELDS ===
        MaxChainSpeedWorld = 0.0f;
        MaxChainSpeedLocal = 0.0f;
        MaxChainSpeedComponent = 0.0f;
        EffectiveMotionSpeed = 0.0f;
        PeakAcceleration = 0.0f;
        PeakJerkMagnitude = 0.0f;
        AverageChainMass = 0.0f;
        MomentumMagnitude = 0.0f;
        AngularMomentumMagnitude = 0.0f;
        MotionConsistencyScore = 0.0f;
        CoordinationFactor = 0.0f;
        ActiveBoneCount = 0;
        TrajectoryConfidence = 0.0f;
        bIsCommittedAttack = false;
        bIsFeintOrGlancing = false;
        AttackCurvature = 0.0f;
        TimeInAttack = 0.0f;
        ImpactPotential = 0.0f;
        AttackPrincipalDirection = FVector::ZeroVector;
        ChainCenterOfMass = FVector::ZeroVector;
        EffectiveStrikeRadius = 0.0f;
        ChainLength = 0.0f;
        PrimaryMotionSpace = EOHReferenceSpace::LocalSpace;
    }

    // === CONSTRUCTOR MAINTAINING YOUR PATTERN ===
    FOHCombatAnalysis() {
        Reset();
    }
};

USTRUCT(BlueprintType)
struct ONLYHANDS_API FOHKalmanState {
    GENERATED_BODY()

    UPROPERTY()
    FVector Position = FVector::ZeroVector;

    UPROPERTY()
    FVector Velocity = FVector::ZeroVector;

    UPROPERTY()
    FMatrix CovarianceP = FMatrix::Identity; // 6x6 for position and velocity

    // Process noise covariance
    UPROPERTY()
    float ProcessNoise = 0.1f;

    // Measurement noise covariance
    UPROPERTY()
    float MeasurementNoise = 0.5f;

    // Initialize from motion data
    void InitializeFromMotionData(const FOHBoneMotionData& MotionData) {
        Position = MotionData.GetCurrentPosition();
        Velocity = MotionData.GetVelocity(false);
        CovarianceP = FMatrix::Identity * 10.0f; // Initial uncertainty
    }

    // Predict step
    void Predict(float DeltaTime) {
        // State transition: x = x + v*dt, v = v
        Position += Velocity * DeltaTime;

        // Update covariance (simplified for 3D position/velocity)
        float dt2 = DeltaTime * DeltaTime;
        CovarianceP.M[0][0] += ProcessNoise * dt2;
        CovarianceP.M[1][1] += ProcessNoise * dt2;
        CovarianceP.M[2][2] += ProcessNoise * dt2;
        CovarianceP.M[3][3] += ProcessNoise;
        CovarianceP.M[4][4] += ProcessNoise;
        CovarianceP.M[5][5] += ProcessNoise;
    }

    // Update step with measurement
    void Update(const FVector& MeasuredPosition, float DeltaTime) {
        // Innovation
        FVector Innovation = MeasuredPosition - Position;

        // Innovation covariance (simplified)
        float S = CovarianceP.M[0][0] + CovarianceP.M[1][1] + CovarianceP.M[2][2] + MeasurementNoise;

        // Kalman gain
        float K = CovarianceP.M[0][0] / S;

        // Update state
        Position += Innovation * K;
        Velocity += Innovation * (K / DeltaTime);

        // Update covariance
        CovarianceP *= (1.0f - K);
    }
};

USTRUCT(BlueprintType)
struct FRK4State {
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FVector Position = FVector::ZeroVector;

    UPROPERTY(BlueprintReadWrite)
    FVector Velocity = FVector::ZeroVector;

    // Add rotation support for full 6DOF physics
    UPROPERTY(BlueprintReadWrite)
    FQuat Rotation = FQuat::Identity;

    UPROPERTY(BlueprintReadWrite)
    FVector AngularVelocity = FVector::ZeroVector;

    // ============== ACCESSOR METHODS FOR TEMPLATE COMPATIBILITY ==============

    FVector GetLocation() const {
        return Position;
    }
    FVector GetVelocity() const {
        return Velocity;
    }

    // Add missing accessors that the template expects
    FVector GetLinearVelocity() const {
        return Velocity;
    }
    FQuat GetRotation() const {
        return Rotation;
    }
    FVector GetAngularVelocity() const {
        return AngularVelocity;
    }

    // For acceleration (RK4 doesn't store it, so return zero)
    static FVector GetLinearAcceleration() {
        return FVector::ZeroVector;
    }
    static FVector GetAngularAcceleration() {
        return FVector::ZeroVector;
    }

    // Time stamp (RK4 typically doesn't track time internally)
    static float GetTimeStamp() {
        return 0.0f;
    }

    // Factory method for creating from components
    static FRK4State CreateFromState(const FVector& Pos, const FQuat& Rot, const FVector& LinVel,
                                     const FVector& AngVel) {
        FRK4State State;
        State.Position = Pos;
        State.Rotation = Rot;
        State.Velocity = LinVel;
        State.AngularVelocity = AngVel;
        return State;
    }
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
struct ONLYHANDS_API FOHKinematicChainData {
    GENERATED_BODY()

    // Bone transforms
    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform FootTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform AnkleTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform KneeTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Kinematic")
    FTransform HipTransform = FTransform::Identity;

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
    FVector KneeDirection = FVector::ZeroVector; // Which way knee points

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
    TArray<FName> ClippingBones = TArray<FName>();

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
    FVector PlantedPosition = FVector::ZeroVector; // Position when last planted
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
struct ONLYHANDS_API FOHFootStanceData {
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
struct ONLYHANDS_API FOHStanceState {
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
struct ONLYHANDS_API FOHBlendFloatState {
    GENERATED_BODY()
    UPROPERTY(BlueprintReadOnly, Category = "Blend")
    float StartValue = 1.0f;
    UPROPERTY(BlueprintReadOnly, Category = "Blend")
    float TargetValue = 1.0f;
    UPROPERTY(BlueprintReadOnly, Category = "Blend")
    float Duration = 0.f;
    UPROPERTY(BlueprintReadOnly, Category = "Blend")
    float StartTime = 0.f;
    UPROPERTY(BlueprintReadOnly, Category = "Blend")
    bool bBlending = false;

    void StartBlend(float InCurrent, float InTarget, float InDuration, float WorldTime) {
        StartValue = InCurrent;
        TargetValue = InTarget;
        Duration = InDuration;
        StartTime = WorldTime;
        bBlending = true;
    }

    // Returns true if blending is still active, sets OutValue to current blend value
    bool Tick(float WorldTime, float& OutValue) {
        if (!bBlending) {
            OutValue = TargetValue;
            return false;
        }
        float Alpha = Duration > 0.f ? FMath::Clamp((WorldTime - StartTime) / Duration, 0.f, 1.f) : 1.f;
        OutValue = FMath::Lerp(StartValue, TargetValue, Alpha);
        if (Alpha >= 1.f) {
            bBlending = false;
            OutValue = TargetValue;
            return false;
        }
        return true;
    }
};
