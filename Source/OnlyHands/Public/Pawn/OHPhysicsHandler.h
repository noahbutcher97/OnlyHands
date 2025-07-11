#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsHandleComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Camera/CameraComponent.h"
#include "Engine/LocalPlayer.h" // <-- Add this for mouse lock modes!
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"
#include "OHPhysicsHandler.generated.h"

UCLASS()
class ONLYHANDS_API AOHPhysicsHandler : public APawn {
    GENERATED_BODY()

  public:
    AOHPhysicsHandler();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics")
    UPhysicsHandleComponent* PhysicsHandle;

    // How far we can reach to grab
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grabbing")
    float GrabDistance = 1000.f;

    // What object types the grab trace should hit
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grabbing")
    TArray<TEnumAsByte<EObjectTypeQuery>> GrabObjectTypes;

    // Sensitivity for mouse rotation (editable in editor)
    UPROPERTY(EditAnywhere, Category = "Grabbing|Rotation")
    float MouseRotationSensitivity = 2.0f;

    UPROPERTY(EditAnywhere, Category = "Grabbing|MouseDrag")
    double MouseDragSensitivity = 1.0f;

    // 1. Find the closest actor to a location (optionally within a distance)
    static AActor* FindClosestActor(const TArray<AActor*>& Actors, const FVector& ToLocation, float MaxDistance,
                                    float& OutDistance);

    // 2. Find the closest simulating physics UPrimitiveComponent on an actor
    static bool FindClosestSimulatingComponent(AActor* Actor, const FVector& ClickLocation, float MaxDistance,
                                               UPrimitiveComponent*& OutComponent, FVector& OutClosestPoint,
                                               float& OutDistance);

    // 3. Find the closest simulating FBodyInstance in a skeletal mesh (and bone name)
    static bool FindClosestSimulatingBodyInstance(USkeletalMeshComponent* SkelMesh, const FVector& ClickLocation,
                                                  float MaxDistance, const FBodyInstance*& OutBodyInstance,
                                                  FName& OutBoneName, FVector& OutBodyCOM, float& OutBodyDistance);
    static bool HasAnySimulatingBody(USkeletalMeshComponent* SkelMesh);
    static bool IsComponentOrAnyBodySimulatingPhysics(UPrimitiveComponent* Prim);

  protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    // Camera for mouse/world interaction
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera)
    UCameraComponent* Camera;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Movement)
    UFloatingPawnMovement* MovementComponent;

    bool bIsGrabbingWithMouse = false;
    float GrabbedDistance = 0.f;
    FVector2D GrabStartScreenPos = FVector2D::ZeroVector;
    FVector GrabStartWorldPos = FVector::ZeroVector;
    FVector GrabbedOffset = FVector::ZeroVector;            // offset from grab point to object COM
    FVector2D AccumulatedGrabDelta = FVector2D::ZeroVector; // Mouse axis delta since grab
    FVector CurrentGrabTargetLocation = FVector::ZeroVector;
    FVector2D PreviousMousePosition;
    bool bDraggingObject = false; // Set this to true in Grab(), false in Release()

  private:
    // Currently grabbed component
    UPROPERTY()
    UPrimitiveComponent* GrabbedComponent;

    // Currently highlighted component
    UPROPERTY()
    UPrimitiveComponent* HighlightedComponent;

    // For holding additional rotation while holding object
    FRotator HeldRotationOffset = FRotator::ZeroRotator;

    // RMB rotation drag state
    bool bIsRotatingHeldObject = false;

    // Shift held for roll rotation
    bool bIsShiftDown = false;

    // Last mouse position for possible future use (not strictly needed now)
    FVector2D LastMousePosition;

    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);

    // Camera look state
    bool bIsCameraLookActive = false;

    void StartCameraLook();
    void StopCameraLook();
    void CameraLookYaw(float AxisValue);
    void CameraLookPitch(float AxisValue);

    // Core grab logic
    void Grab();
    void Release();
    void MoveGrabbedComponent();
    void UpdateGrabbedTargetFromScreen();
    void UpdateHighlight();
    bool TryGrabUnderCursor(const TArray<TEnumAsByte<EObjectTypeQuery>>& ObjectTypes, float MaxGrabDistance);

    void OnGrabMoveX(float AxisValue);

    void OnGrabMoveY(float AxisValue);

    // Distance for grabbing
    FVector GetGrabTargetLocation() const;
};
