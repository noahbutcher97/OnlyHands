#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "OHMovementComponent.generated.h"

UENUM(BlueprintType)
enum class EOHGait : uint8
{
	Walking,
	Running,
	Sprinting
};

USTRUCT(BlueprintType)
struct FOHMovementSettings
{
	GENERATED_BODY()
	
	// Movement speeds for each gait
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Speeds")
	float WalkSpeed = 200.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Speeds")
	float RunSpeed = 450.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Speeds")
	float SprintSpeed = 650.f;

	// Default max acceleration
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Acceleration")
	float MaxAcceleration = 2048.f;

	// Default braking deceleration
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Braking")
	float BrakingDecelerationWalking = 2048.f;

	// Default ground friction
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Friction")
	float GroundFriction = 8.f;

	// Curve vector mapping normalized speed [0..1] to (Acceleration, BrakingDeceleration, GroundFriction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Curves")
	UCurveVector* MovementCurve = nullptr;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMovementInputUpdated, FVector2D, MovementInput);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGaitChanged, EOHGait, NewGait);

UCLASS(Blueprintable, BlueprintType, ClassGroup=(State), meta=(BlueprintSpawnableComponent))
class ONLYHANDS_API UOHMovementComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UOHMovementComponent();

	
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;



	// References
	UPROPERTY(BlueprintReadWrite, Category="OH|Target")
	mutable AActor* TargetActor = nullptr;

	UFUNCTION(BlueprintCallable, Category="OH|Target")
	void FindTargetActor();

	UFUNCTION(BlueprintCallable, Category="OH|Target")
	void SetTargetActor(AActor* NewTarget);

	// Virtual Joystick specific settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Input")
	bool bUseInputMagnitudeForSpeed = true;  // Scale movement speed by input magnitude
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Input")
	float JoystickDeadzone = 0.15f;  // Minimum input to register movement
    
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Input")
	float JoystickWalkZone = 0.5f;   // Input magnitude below this = walk
    
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Input")
	float JoystickRunZone = 0.85f;   // Input magnitude below this = run, above = sprint intent
    


	
	// === Momentum Settings ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config")
    bool bUseMomentumBasedMovement = true;  // Enable weighted, momentum-based movement
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|InputBuffering")
    float NoInputSpeedDecayRate = 4.0f;  // How fast to decay speed when no input
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|InputBuffering")
    float InputSpeedBuildRate = 2.5f;  // How fast to build speed when input detected

	
	// === Movement Speed Curves ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Curves")
    UCurveFloat* DirectionalSpeedCurve;  // Curve mapping strafe angle (0-180) to speed multiplier
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Curves")
    UCurveFloat* RotationalSpeedCurve;  // Curve mapping angular velocity to speed multiplier

	// Movement curve property to assign in blueprint or code
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Curves")
	UCurveVector* MovementCurve;

	// === Speed Multiplier Settings ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Multipliers")
    float BackpedalSpeedMultiplier = 0.75f;  // Speed reduction when moving backward
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Multipliers")
    float StrafeSpeedMultiplier = 0.85f;  // Speed reduction for pure strafe (90 degrees)
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Multipliers")
    float DiagonalSpeedMultiplier = 0.9f;  // Speed for 45-degree movement
    
    // === Weight & Momentum ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Weight")
    float MovementWeight = 0.85f;  // 0-1, higher = more weight/momentum
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Weight")
    float DirectionChangeSharpness = 3.0f;  // How quickly direction changes (lower = heavier)
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Weight")
    float SpeedRetentionOnDirectionChange = 0.7f;  // Speed multiplier when changing direction sharply
    
    // === Runtime State ===
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    float CurrentMomentumSpeed = 0.f;  // Current momentum-based speed
    
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    float DirectionalSpeedMultiplier = 1.0f;  // Current direction-based speed modifier
    
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    float RotationalSpeedMultiplier = 1.0f;  // Current rotation-based speed modifier
    
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    float CurrentMovementAngle = 0.f;  // Angle between movement and facing direction
    
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    FVector LastMovementDirection = FVector::ZeroVector;  // For direction change detection
    
    UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
    float CurrentAngularVelocity = 0.f;  // Current turning speed


	

    
	// Smooth gait transition settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Gait")
	float GaitSpeedTransitionRate = 2.0f;  // How fast to transition between gait speeds
	
	// Multiplier for Acceleration 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Gait")
	float GaitAccelerationMultiplier = 1.5f;  // Multiply acceleration when transitioning up
    
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Config|Gait")
	float GaitDecelerationMultiplier = 0.7f;  // Multiply deceleration when transitioning down

	
	UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
	float CurrentTargetSpeed = 0.f;  // Target speed based on gait
    
	UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
	float SmoothedTargetSpeed = 0.f;  // Smoothly interpolated target speed
    
	UPROPERTY(BlueprintReadOnly, Category="OH|Runtime|State")
	EOHGait PreviousGait = EOHGait::Walking;  // Track gait changes

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Initialization")
	bool bMovementInitialized = false;  // Track whether movement has been initialized


	// --- Movement Input ---
	// Apply movement input to CharacterMovementComponent
	UFUNCTION( )
	void ApplyMovementInput();

	// Calculate movement input vector from input magnitude and direction
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	FVector CalculateMovementInputVector() const;



	// --- Movement State ---
	// Calculate control rotation based on Camera Perspective 
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	FRotator CalculateControlRotation() const;
	
	// --- Movement Settings ---
	// Calculate movement settings based on current gait
	UFUNCTION(BlueprintCallable, Category="OH|Movement")
	void CalculateDirectionalSpeedModifiers(ACharacter* OwnerChar);
	
	// Movement input vector from external input sources (clamped 0-1)
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Input")
	FVector2D MovementInputVector;

	
	// --- Movement Settings Asset/Instance ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Config")
	FOHMovementSettings MovementSettings;
	
	// Buffered smoothed input magnitude and speed for intent detection
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Buffering")
	float BufferedInputAmount;
	
	// Buffered smoothed speed for intent detection
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Buffering")
	float BufferedSpeed;

	// Current interpolated gait state and blend alpha for smooth transitions
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Gait")
	EOHGait CurrentGait;

	// Current interpolated gait state and blend alpha for smooth transitions
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Gait")
	float GaitBlendAlpha;




	// --- Movement State ---
	// Current movement speed (used to Calculate Gait)
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|State")
	float CurrentSpeed;

	// Speed thresholds for walking and running gaits (used to Calculate Gait)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Thresholds")
	float WalkSpeedThreshold = 50.f;
	
	// Speeds for sprint and run gaits (used to Calculate Gait)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Thresholds")
	float RunSpeedThreshold = 200.f;

	// Speeds for sprint and run gaits (used to Calculate Gait)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Thresholds")
	float SprintSpeedThreshold = 400.f;



	// Max Walk speed (used to set CharacterMovementComponent values)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OH|Gait|Speeds")
	float WalkMaxSpeed = 150.f;

	// Max Run speed (used to set CharacterMovementComponent values)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OH|Gait|Speeds")
	float RunMaxSpeed = 450.f;

	// Max Sprint speed (used to set CharacterMovementComponent values)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OH|Gait|Speeds")
	float SprintMaxSpeed = 650.f;

	
	// Parameters for tuning smoothing and intent detection
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Buffering")
	float InputBufferDecay;        // How fast buffer decays when input drops (per second)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Buffering")
	float InputBufferRiseRate;    // How fast buffer rises to new input (per second)


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Hysteresis")
	float RunHysteresis;          // Hysteresis offset for run speed threshold

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Gait|Hysteresis")
	float SprintHysteresis;       // Hysteresis offset for sprint speed threshold

	

	// Input intent magnitude from input panel (0 to 1), used in calculations
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|Input")
	float InputIntentMagnitude;
	
	// Delegate called when gait changes
	UPROPERTY(BlueprintAssignable, Category="OH|Movement|Gait")
	FOnGaitChanged OnGaitChanged;

	// Delegate called when movement input updates
	UPROPERTY(BlueprintAssignable, Category="OH|Movement|Input")
	FOnMovementInputUpdated OnMovementInputUpdated;

	// Movement mode tracking from CharacterMovementComponent
	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|State")
	TEnumAsByte<EMovementMode> CurrentMovementMode;

	UPROPERTY(BlueprintReadOnly, Category="OH|Movement|State")
	uint8 CurrentCustomMovementMode;
	
	// --- Movement Debug ---
	// Debug draw movement input vector
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Debug")
	bool bDebugMovement;


	
	// Pivot threshold angle in degrees, configurable
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="OH|Movement|Pivot")
	float PivotThreshold;

	// Public setter to update input vector (to be called from player controller or input system)
	UFUNCTION(BlueprintCallable, Category="OH|Movement|Input")
	void SetMovementInputVector(const FVector2D& NewInput);
	
	// Desired gait calculated from input and speed
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	EOHGait CalculateDesiredGait(float Deltatime);

	// Allowed gait from external constraints (e.g., stamina, crouching)
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	EOHGait GetAllowedGait() const;

	// Actual gait combining desired and allowed gait
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	static EOHGait GetActualGait(EOHGait DesiredGait, EOHGait AllowedGait);

	// Smooth interpolation of gait changes
	UFUNCTION(BlueprintCallable, Category="OH|Movement")
	void InterpolateGait(EOHGait TargetGait, float DeltaTime);

	// Update movement state per tick
	UFUNCTION(BlueprintCallable, Category="OH|Movement")
	void UpdateMovement(float DeltaTime);

	// Update input buffering values from input vector
	UFUNCTION(BlueprintCallable, Category="OH|Movement")
	void UpdateInputBuffers(float DeltaTime);

	// Updates character movement component properties dynamically based on allowed gait
	UFUNCTION(BlueprintCallable, Category="OH|Movement")
	void UpdateDynamicMovementSettings(EOHGait AllowedGait);

	// Returns movement settings based on the current state / context
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	FOHMovementSettings GetTargetMovementSettings() const;

	// Returns normalized speed [0-1] for curve evaluation
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	float GetMappedSpeed() const;
	
	// Calculates current horizontal movement speed ignoring vertical velocity
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	float CalculateMovementSpeed() const;

	// Calculates input magnitude (0-1) from input vector
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	float CalculateMovementInputAmount() const;

	// Calculates movement play rate (for animation speed scaling) based on speed and input
	UFUNCTION(BlueprintPure, Category="OH|Movement")
	float CalculateMovementPlayRate(float Speed, float InputAmount) const;
	
	UFUNCTION(BlueprintPure, Category="OH|Movement|Direction")
	float GetInputWorldAngleDegrees() const;

	// Calculates a custom rotation looking from owner to locked target (implement in cpp)
	UFUNCTION(BlueprintPure, Category="OH|Movement|Pivot")
	FRotator CalculateRotationToTarget_Internal() const;
	
	// Calculates the offset rotator between owner rotation and target rotation (implement in cpp)
	UFUNCTION(BlueprintPure, Category="OH|Movement|Pivot")
	FRotator CalculateTargetOffset_Internal() const;
	
	// Movement mode changed handler, to be called when CharacterMovement changes movement mode
	UFUNCTION()
	void HandleMovementModeChanged(EMovementMode NewMovementMode, uint8 NewCustomMode);
	
	// Configurable thresholds and timing for gait buffering and intent detection
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Movement|Gait|Buffering")
	float InputIntentMagnitudeThreshold = 0.3f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Movement|Gait|Buffering")
	float HoldTimeAboveRunThreshold = 0.2f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Movement|Gait|Buffering")
	float HoldTimeAboveSprintThreshold = 0.3f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Movement|Gait|Interpolation")
	float GaitInterpSpeed = 5.f;
	
	// Movement play rate for animation
	UPROPERTY(BlueprintReadOnly, Category = "OH|Movement|State")
	float MovementPlayRate;
	
	// Returns whether the character should perform a pivot based on the pivot threshold 
	UFUNCTION(BlueprintCallable, Category = "OH|Movement|Pivot")
	bool ShouldPivot_Internal() const;
	
	// Input delegate handlers (bind these in your player controller)
	UFUNCTION()
	void OnReceiveMoveForward(float ForwardValue);

	UFUNCTION()
	void OnReceiveMoveRight(float RightValue);

	UFUNCTION()
	void OnReceiveMoveVector(float ForwardValue, float RightValue);

	UFUNCTION(BlueprintPure, Category="OH|Movement|AI")
	bool GetIsPlayerControlled() const;
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


};