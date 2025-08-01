
#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Utilities/OHSafeMapUtils.h"
#include "BonePose.h"
#include "Components/SkeletalMeshComponent.h"
// #include "FunctionLibrary/OHAlgoUtils.h"
#include "OnlyHands/Game/Enums/E_FightData.h"
#include "OnlyHands/Game/Structures/S_FightStructure.h"
#include "PhysicsEngine/PhysicalAnimationComponent.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "OHPhysicsStructs.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogPACManager, Log, All);

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
    FVector GetLinearAcceleration() const {
        return FVector::ZeroVector;
    }
    FVector GetAngularAcceleration() const {
        return FVector::ZeroVector;
    }

    // Time stamp (RK4 typically doesn't track time internally)
    float GetTimeStamp() const {
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
    FTransform ReferenceTransform = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Motion Sample|Reference")
    FVector ReferenceVelocity = FVector::ZeroVector;

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
    float CalculateMotionConsistencyOverWindow(const TArray<const FOHMotionFrameSample*>& WindowSamples,
                                               EOHReferenceSpace Space) const {
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

            const FOHMotionFrameSample* LatestSample = MotionData->GetLatestSample();
            if (LatestSample) {
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

        // Initialize caches
        InvalidateCoordinationCache();
    }

    TArray<FVector> GetPredictedTrajectory() const {
        return PredictedTrajectory;
    }

    TArray<FVector> GetBezierControlPoints() const {
        return BezierControlPoints;
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
struct FOHKalmanState {
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

struct FOHBlendFloatState {
    float StartValue = 1.0f;
    float TargetValue = 1.0f;
    float Duration = 0.f;
    float StartTime = 0.f;
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
