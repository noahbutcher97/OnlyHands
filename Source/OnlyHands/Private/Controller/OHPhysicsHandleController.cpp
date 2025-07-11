// Fill out your copyright notice in the Description page of Project Settings.


#include "Controller/OHPhysicsHandleController.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

AOHPhysicsHandleController::AOHPhysicsHandleController()
{
    PrimaryActorTick.bCanEverTick = true;
    // Always show the mouse cursor
    bShowMouseCursor = true;

    // Enable mouse click and mouse over events (needed for GetHitResultUnderCursor)
    bEnableClickEvents = true;
    bEnableMouseOverEvents = true;
}

void AOHPhysicsHandleController::BeginPlay()
{
    Super::BeginPlay();

    FInputModeGameAndUI InputMode;
    InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock); // or EMouseLockMode::LockAlways if you prefer
    SetInputMode(InputMode);// Ensure our handle is ready
}

void AOHPhysicsHandleController::SetupInputComponent()
{
    Super::SetupInputComponent();
    
}

void AOHPhysicsHandleController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void AOHPhysicsHandleController::SetCameraLookMode(bool bEnable)
{
    if (bEnable)
    {
        // Enter camera look mode: hide cursor, lock/capture mouse, game-only input
        bShowMouseCursor = false;
        FInputModeGameOnly InputMode;
        InputMode.SetConsumeCaptureMouseDown(true); // Optional, ensures capture is clean
        SetInputMode(InputMode);
    }
    else
    {
        // Exit camera look mode: show cursor, allow UI, unlock mouse
        bShowMouseCursor = true;
        FInputModeGameAndUI InputMode;
        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        SetInputMode(InputMode);
    }
}