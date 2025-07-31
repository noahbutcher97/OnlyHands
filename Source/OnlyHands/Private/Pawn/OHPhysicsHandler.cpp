#include "Pawn/OHPhysicsHandler.h"
#include "Camera/CameraComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Controller/OHPhysicsHandleController.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"



AOHPhysicsHandler::AOHPhysicsHandler()
{
    PrimaryActorTick.bCanEverTick = true;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    RootComponent = Camera;
    Camera->bUsePawnControlRotation = true;
    MovementComponent = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("FloatingPawnMovement"));
    PhysicsHandle = CreateDefaultSubobject<UPhysicsHandleComponent>(TEXT("PhysicsHandle"));
    GrabbedComponent = nullptr;
    HighlightedComponent = nullptr;
}



void AOHPhysicsHandler::BeginPlay()
{
    Super::BeginPlay();
}

void AOHPhysicsHandler::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    //MoveGrabbedComponent();
    UpdateHighlight();
}


void AOHPhysicsHandler::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // Movement
    PlayerInputComponent->BindAxis("MoveForward", this, &AOHPhysicsHandler::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &AOHPhysicsHandler::MoveRight);
    PlayerInputComponent->BindAxis("MoveUp", this, &AOHPhysicsHandler::MoveUp);

    // Camera look (only when RMB held)
    PlayerInputComponent->BindAxis("Turn", this, &AOHPhysicsHandler::CameraLookYaw);
    PlayerInputComponent->BindAxis("LookUp", this, &AOHPhysicsHandler::CameraLookPitch);

    // Grabbing
    PlayerInputComponent->BindKey(EKeys::LeftMouseButton, IE_Pressed, this, &AOHPhysicsHandler::Grab);
    PlayerInputComponent->BindKey(EKeys::LeftMouseButton, IE_Released, this, &AOHPhysicsHandler::Release);

    // Camera look mode (RMB)
    PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Pressed, this, &AOHPhysicsHandler::StartCameraLook);
    PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Released, this, &AOHPhysicsHandler::StopCameraLook);

    PlayerInputComponent->BindAxis("MouseMoveX", this, &AOHPhysicsHandler::OnGrabMoveX);
    PlayerInputComponent->BindAxis("MouseMoveY", this, &AOHPhysicsHandler::OnGrabMoveY);
}

void AOHPhysicsHandler::StartCameraLook()
{
    bIsCameraLookActive = true;
    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        PC->bShowMouseCursor = false;
        FInputModeGameOnly InputMode;
        InputMode.SetConsumeCaptureMouseDown(true);
        PC->SetInputMode(InputMode);
    }
    if (GEngine)
        GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Green, TEXT("Camera Look Mode: ON (RMB Down)"));
}

void AOHPhysicsHandler::StopCameraLook()
{
    bIsCameraLookActive = false;
    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        PC->bShowMouseCursor = true;
        FInputModeGameAndUI InputMode;
        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        PC->SetInputMode(InputMode);
    }
    if (GEngine)
        GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Red, TEXT("Camera Look Mode: OFF (RMB Up)"));
}


void AOHPhysicsHandler::CameraLookYaw(float AxisValue)
{
    if (GEngine)
    {
        FString Msg = FString::Printf(TEXT("CameraLookYaw: %f | RMB active: %d"), AxisValue, static_cast<int>(bIsCameraLookActive));
        GEngine->AddOnScreenDebugMessage(-1, 0.02f, FColor::Cyan, Msg);
    }

    if (bIsCameraLookActive && AxisValue != 0.f)
    {
        AddControllerYawInput(AxisValue);
    }
}

void AOHPhysicsHandler::CameraLookPitch(float AxisValue)
{
    if (GEngine)
    {
        FString Msg = FString::Printf(TEXT("CameraLookPitch: %f | RMB active: %d"), AxisValue, static_cast<int>(bIsCameraLookActive));
        GEngine->AddOnScreenDebugMessage(-1, 0.02f, FColor::Cyan, Msg);
    }

    if (bIsCameraLookActive && AxisValue != 0.f)
    {
        AddControllerPitchInput(AxisValue);
    }
}

void AOHPhysicsHandler::MoveForward(float Value)
{
    UE_LOG(LogTemp, Warning, TEXT("MoveForward: %f"), Value);
    if (Value != 0.f)
        AddMovementInput(Camera->GetForwardVector(), Value);
}

void AOHPhysicsHandler::MoveRight(float Value)
{
    if (Value != 0.f)
        AddMovementInput(Camera->GetRightVector(), Value);
}

void AOHPhysicsHandler::MoveUp(float Value)
{
    if (Value != 0.f)
        AddMovementInput(Camera->GetUpVector(), Value);
}

void AOHPhysicsHandler::OnGrabMoveX(float AxisValue)
{
    if (bIsGrabbingWithMouse && FMath::Abs(AxisValue) > KINDA_SMALL_NUMBER)
    {
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 0.01f, FColor::Red, FString::Printf(TEXT("MouseMoveX: %f"), AxisValue));
        AccumulatedGrabDelta.X += AxisValue * MouseDragSensitivity;
        UpdateGrabbedTargetFromScreen();
    }
}

void AOHPhysicsHandler::OnGrabMoveY(float AxisValue)
{
    if (bIsGrabbingWithMouse && FMath::Abs(AxisValue) > KINDA_SMALL_NUMBER)
    {
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 0.01f, FColor::Green, FString::Printf(TEXT("MouseMoveY: %f"), AxisValue));

        AccumulatedGrabDelta.Y += AxisValue * MouseDragSensitivity;
        UpdateGrabbedTargetFromScreen();
    }
}
void AOHPhysicsHandler::Grab()
{
    bool bDidGrab = TryGrabUnderCursor(GrabObjectTypes, GrabDistance);
    if (bDidGrab)
    {
        // Store mouse and world positions for mouse-dragging logic
        if (APlayerController* PC = Cast<APlayerController>(GetController()))
        {
            PC->bShowMouseCursor = false;
            FInputModeGameOnly InputMode;
            InputMode.SetConsumeCaptureMouseDown(true);
            PC->SetInputMode(InputMode);

            // Store start mouse pos and world pos for dragging
            PC->GetMousePosition(GrabStartScreenPos.X, GrabStartScreenPos.Y);

            if (PhysicsHandle && PhysicsHandle->GrabbedComponent)
            {
                GrabStartWorldPos = PhysicsHandle->GrabbedComponent->GetComponentLocation();
            }

            // Reset drag delta
            AccumulatedGrabDelta = FVector2D::ZeroVector;
            bIsGrabbingWithMouse = true;

            if (GEngine)
                GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Yellow, TEXT("Grabbed - Mouse locked!"));
        }
    }
}



void AOHPhysicsHandler::Release()
{
    if (PhysicsHandle && PhysicsHandle->GrabbedComponent)
    {
        PhysicsHandle->ReleaseComponent();
        GrabbedComponent = nullptr;
        HeldRotationOffset = FRotator::ZeroRotator;
        bIsRotatingHeldObject = false;

        // Reset mouse drag state
        bIsGrabbingWithMouse = false;
        AccumulatedGrabDelta = FVector2D::ZeroVector;

        if (APlayerController* PC = Cast<APlayerController>(GetController()))
        {
            PC->bShowMouseCursor = true;
            FInputModeGameAndUI InputMode;
            InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
            PC->SetInputMode(InputMode);

            if (GEngine)
                GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Yellow, TEXT("Released - Mouse unlocked!"));
        }
    }
}

void AOHPhysicsHandler::MoveGrabbedComponent()
{
    if (PhysicsHandle && PhysicsHandle->GrabbedComponent && bIsGrabbingWithMouse)
    {
        APlayerController* PC = Cast<APlayerController>(GetController());
        if (!PC) return;

        FVector2D MousePos;
        PC->GetMousePosition(MousePos.X, MousePos.Y);

        // Project from mouse screen pos to world
        FVector WorldOrigin, WorldDir;
        PC->DeprojectScreenPositionToWorld(MousePos.X, MousePos.Y, WorldOrigin, WorldDir);

        // Calculate the target location at the current grab distance
        FVector TargetLoc = WorldOrigin + WorldDir * GrabbedDistance;
        TargetLoc += GrabbedOffset;

        // Interpolate for smoothing
        CurrentGrabTargetLocation = FMath::VInterpTo(
            CurrentGrabTargetLocation,
            TargetLoc,
            GetWorld()->GetDeltaSeconds(),
            20.0f
        );

        // Set the handle
        PhysicsHandle->SetTargetLocationAndRotation(
            CurrentGrabTargetLocation,
            PhysicsHandle->GrabbedComponent->GetComponentRotation()
        );
    }
}

void AOHPhysicsHandler::UpdateGrabbedTargetFromScreen()
{
    if (!PhysicsHandle || !PhysicsHandle->GrabbedComponent || !bIsGrabbingWithMouse) return;

    FVector2D NewScreenPos = GrabStartScreenPos + AccumulatedGrabDelta;
    FVector WorldOrigin, WorldDirection;

    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        if (PC->DeprojectScreenPositionToWorld(NewScreenPos.X, NewScreenPos.Y, WorldOrigin, WorldDirection))
        {
            // Move object to the new world position at the original grab distance
            FVector TargetLoc = WorldOrigin + WorldDirection * GrabbedDistance;
            TargetLoc += GrabbedOffset;

            // Smoothing (optional)
            CurrentGrabTargetLocation = FMath::VInterpTo(
                CurrentGrabTargetLocation,
                TargetLoc,
                GetWorld()->GetDeltaSeconds(),
                20.0f
            );

            PhysicsHandle->SetTargetLocationAndRotation(
                CurrentGrabTargetLocation,
                PhysicsHandle->GrabbedComponent->GetComponentRotation()
            );
        }
    }
}


FVector AOHPhysicsHandler::GetTargetLocation() const
{
    if (!Camera) return FVector::ZeroVector;

    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC) return FVector::ZeroVector;

    FVector WorldOrigin, WorldDirection;
    PC->DeprojectMousePositionToWorld(WorldOrigin, WorldDirection);
    return WorldOrigin + WorldDirection * GrabDistance;
}

// -------- HIGHLIGHTING --------
void AOHPhysicsHandler::UpdateHighlight()
{
    if (!GetWorld() || !GetController() || (PhysicsHandle && PhysicsHandle->GrabbedComponent))
    {
        if (HighlightedComponent)
        {
            HighlightedComponent->SetRenderCustomDepth(false);
            HighlightedComponent = nullptr;
        }
        return;
    }

    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC) return;

    // Use a trace with GrabObjectTypes (BP-configurable) for more flexibility:
    FHitResult HitResult;
    PC->GetHitResultUnderCursorForObjects(GrabObjectTypes, false, HitResult);

    // Use your robust helper to find the closest simulating component, even if the hit is a non-sim root
    UPrimitiveComponent* ClosestSimComp = nullptr;
    FVector ClosestPoint = FVector::ZeroVector;
    float ClosestDist = 0.f;

    if (HitResult.GetActor() && FindClosestSimulatingComponent(
        HitResult.GetActor(),
        HitResult.ImpactPoint,
        GrabDistance, // Or some "highlight" distance
        ClosestSimComp,
        ClosestPoint,
        ClosestDist
    ))
    {
        if (HighlightedComponent != ClosestSimComp)
        {
            if (HighlightedComponent)
                HighlightedComponent->SetRenderCustomDepth(false);

            HighlightedComponent = ClosestSimComp;
            HighlightedComponent->SetRenderCustomDepth(true);
        }
    }
    else
    {
        if (HighlightedComponent)
        {
            HighlightedComponent->SetRenderCustomDepth(false);
            HighlightedComponent = nullptr;
        }
    }
}


bool AOHPhysicsHandler::TryGrabUnderCursor(const TArray<TEnumAsByte<EObjectTypeQuery>>& ObjectTypes, float MaxGrabDistance)
{

    if (!PhysicsHandle)
    {
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, TEXT("No PhysicsHandle component!"));
        return false;
    }
    
    APlayerController* PC = Cast<APlayerController>(GetController());
    if (!PC)
    {
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, TEXT("No PlayerController found."));
        return false;
    }

    // Trace under the cursor for specified object types
    FHitResult HitResult;
    PC->GetHitResultUnderCursorForObjects(ObjectTypes, false, HitResult);

    // No hit? Print and exit
    if (!HitResult.GetActor())
    {
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, TEXT("No actor under cursor!"));
        return false;
    }

    float ActualDistance = FVector::Dist(Camera->GetComponentLocation(), HitResult.ImpactPoint);
    if (ActualDistance > MaxGrabDistance)
    {
        FString Msg = FString::Printf(TEXT("Hit '%s' but too far: %.0f / %.0f"), *HitResult.GetActor()->GetName(), ActualDistance, MaxGrabDistance);
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, Msg);
        return false;
    }

    // Find the closest simulating component on the actor
    UPrimitiveComponent* ClosestComponent = nullptr;
    FVector ClosestCompPoint = FVector::ZeroVector;
    float ClosestCompDist = 0.f;
    bool bFoundComponent = FindClosestSimulatingComponent(
        HitResult.GetActor(),
        HitResult.ImpactPoint,
        MaxGrabDistance,
        ClosestComponent,
        ClosestCompPoint,
        ClosestCompDist
    );

    if (!bFoundComponent || !ClosestComponent)
    {
        FString Msg = FString::Printf(TEXT("'%s' has no simulating physics component near click."), *HitResult.GetActor()->GetName());
        if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, Msg);
        return false;
    }

    // If it's a skeletal mesh, find the closest simulating bone
    if (USkeletalMeshComponent* SkelMesh = Cast<USkeletalMeshComponent>(ClosestComponent))
    {
        const FBodyInstance* ClosestBody = nullptr;
        FName ClosestBone = NAME_None;
        FVector ClosestCOM = FVector::ZeroVector;
        float ClosestBodyDist = 0.f;

        if (FindClosestSimulatingBodyInstance(
                SkelMesh, HitResult.ImpactPoint, MaxGrabDistance,
                ClosestBody, ClosestBone, ClosestCOM, ClosestBodyDist))
        {
            // Grab bone at COM
            PhysicsHandle->GrabComponentAtLocationWithRotation(
                SkelMesh,
                ClosestBone,
                ClosestCOM,
                SkelMesh->GetComponentRotation()
            );

            FString Msg = FString::Printf(
                TEXT("Grabbed bone '%s' (%.1f units) on '%s'"),
                *ClosestBone.ToString(), ClosestBodyDist, *SkelMesh->GetName());
            if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Green, Msg);

            GrabbedComponent = SkelMesh;
            HeldRotationOffset = FRotator::ZeroRotator;

            // Enhanced: Store drag data
            GrabbedDistance = (ClosestCOM - Camera->GetComponentLocation()).Size();
            GrabbedOffset = ClosestCOM - HitResult.ImpactPoint;
            bIsGrabbingWithMouse = true;
            if (PC)
            {
                PC->GetMousePosition(GrabStartScreenPos.X, GrabStartScreenPos.Y);
            }
            GrabStartWorldPos = ClosestCOM;
            return true
            ;
        }
        else
        {
            FString Msg = FString::Printf(TEXT("'%s' has no simulating bone near click."), *SkelMesh->GetName());
            if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, Msg);
            return false;
        }
    }

    // For static mesh/primitive components
    PhysicsHandle->GrabComponentAtLocationWithRotation(
        ClosestComponent,
        NAME_None,
        ClosestCompPoint,
        ClosestComponent->GetComponentRotation()
    );

    FString Msg = FString::Printf(
        TEXT("Grabbed component '%s' (%.1f units)"),
        *ClosestComponent->GetName(), ClosestCompDist);
    if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Green, Msg);

    GrabbedComponent = ClosestComponent;
    HeldRotationOffset = FRotator::ZeroRotator;

    // Enhanced: Store drag data
    GrabbedDistance = (ClosestCompPoint - Camera->GetComponentLocation()).Size();
    GrabbedOffset = ClosestCompPoint - HitResult.ImpactPoint;
    bIsGrabbingWithMouse = true;
    if (PC)
    {
        PC->GetMousePosition(GrabStartScreenPos.X, GrabStartScreenPos.Y);
    }
    GrabStartWorldPos = ClosestCompPoint;
    return true;
}

// 1. Closest Actor
AActor* AOHPhysicsHandler::FindClosestActor(const TArray<AActor*>& Actors, const FVector& ToLocation, float MaxDistance, float& OutDistance)
{
    float ClosestDist = TNumericLimits<float>::Max();
    AActor* ClosestActor = nullptr;

    for (AActor* Actor : Actors)
    {
        if (!Actor) continue;
        float Dist = FVector::Dist(Actor->GetActorLocation(), ToLocation);
        if (Dist < ClosestDist && Dist <= MaxDistance)
        {
            ClosestDist = Dist;
            ClosestActor = Actor;
        }
    }
    OutDistance = ClosestDist;
    return ClosestActor;
}

bool AOHPhysicsHandler::FindClosestSimulatingComponent(
    AActor* Actor, 
    const FVector& ClickLocation, 
    float MaxDistance,
    UPrimitiveComponent*& OutComponent, 
    FVector& OutClosestPoint, 
    float& OutDistance)
{
    OutComponent = nullptr;
    OutDistance = TNumericLimits<float>::Max();
    OutClosestPoint = FVector::ZeroVector;

    if (!Actor) return false;

    TArray<UPrimitiveComponent*> Prims;
    Actor->GetComponents<UPrimitiveComponent>(Prims);

    for (UPrimitiveComponent* Prim : Prims)
    {
        if (!IsComponentOrAnyBodySimulatingPhysics(Prim))
            continue;

        FVector OutPoint;
        Prim->GetClosestPointOnCollision(ClickLocation, OutPoint);
        float Dist = FVector::Dist(OutPoint, ClickLocation);
        if (Dist < OutDistance && Dist <= MaxDistance)
        {
            OutComponent = Prim;
            OutClosestPoint = OutPoint;
            OutDistance = Dist;
        }
    }
    return OutComponent != nullptr;
}

bool AOHPhysicsHandler::FindClosestSimulatingBodyInstance(
    USkeletalMeshComponent* SkelMesh,
    const FVector& ClickLocation,
    float MaxDistance,
    const FBodyInstance*& OutBodyInstance,
    FName& OutBoneName,
    FVector& OutBodyCOM,
    float& OutBodyDistance)
{
    OutBodyInstance = nullptr;
    OutBoneName = NAME_None;
    OutBodyCOM = FVector::ZeroVector;
    OutBodyDistance = TNumericLimits<float>::Max();

    if (!SkelMesh)
        return false;

    float ClosestDist = TNumericLimits<float>::Max();
    FName ClosestBone = NAME_None;
    FVector ClosestCOM = FVector::ZeroVector;
    const FBodyInstance* ClosestSimBody = nullptr;

    float ClosestAnyDist = TNumericLimits<float>::Max();
    FName ClosestAnyBone = NAME_None;
    FVector ClosestAnyCOM = FVector::ZeroVector;

    // Iterate all bodies, check for closest simulating AND closest overall
    for (const FBodyInstance* Body : SkelMesh->Bodies)
    {
        if (!Body || !Body->IsValidBodyInstance())
            continue;

        FName Bone = Body->BodySetup.IsValid() ? Body->BodySetup->BoneName : NAME_None;
        FVector COM = Body->GetCOMPosition();
        float Dist = FVector::Dist(COM, ClickLocation);

        // Track closest simulating
        if (Body->IsInstanceSimulatingPhysics() && Dist < ClosestDist && Dist <= MaxDistance)
        {
            ClosestSimBody = Body;
            ClosestDist = Dist;
            ClosestBone = Bone;
            ClosestCOM = COM;
        }

        // Track closest (any)
        if (Dist < ClosestAnyDist)
        {
            ClosestAnyDist = Dist;
            ClosestAnyBone = Bone;
            ClosestAnyCOM = COM;
        }
    }

    // Did we find a simulating body?
    if (ClosestSimBody)
    {
        OutBodyInstance = ClosestSimBody;
        OutBoneName = ClosestBone;
        OutBodyCOM = ClosestCOM;
        OutBodyDistance = ClosestDist;
        if (GEngine)
        {
            FString Msg = FString::Printf(TEXT("Simulating body found: %s (dist=%.2f)"), *OutBoneName.ToString(), OutBodyDistance);
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Green, Msg);
        }
        return true;
    }
    else
    {
        // No simulating bodies, but report closest anyway
        OutBodyInstance = nullptr;
        OutBoneName = ClosestAnyBone;
        OutBodyCOM = ClosestAnyCOM;
        OutBodyDistance = ClosestAnyDist;
        if (GEngine)
        {
            FString Msg = FString::Printf(TEXT("No Simulating bodies but closest bone is: %s (dist=%.2f)"),
                                          *OutBoneName.ToString(), OutBodyDistance);
            GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Yellow, Msg);
        }
        return false;
    }
}

bool AOHPhysicsHandler::HasAnySimulatingBody(USkeletalMeshComponent* SkelMesh)
{
    if (!SkelMesh) return false;
    for (const FBodyInstance* Body : SkelMesh->Bodies)
    {
        if (Body && Body->IsValidBodyInstance() && Body->IsInstanceSimulatingPhysics())
        {
            return true;
        }
    }
    return false;
}

bool AOHPhysicsHandler::IsComponentOrAnyBodySimulatingPhysics(UPrimitiveComponent* Prim)
{
    if (auto* SkelMesh = Cast<USkeletalMeshComponent>(Prim))
        return HasAnySimulatingBody(SkelMesh);
    else
        return Prim->IsSimulatingPhysics();
}