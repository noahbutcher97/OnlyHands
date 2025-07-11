// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "OHPlayerController.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FFloatAxisInputDelegate, float, AxisValue);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FVector2DAxisInputDelegate, float, ForwardValue, float, RightValue);
UCLASS()
class ONLYHANDS_API AOHPlayerController : public APlayerController {
    GENERATED_BODY()

  public:
    virtual void AcknowledgePossession(APawn* NewPawn) override;

    // Axis delegates for separate axis inputs
    UPROPERTY(BlueprintAssignable, Category = "Input")
    FFloatAxisInputDelegate OnMoveForwardInput;

    UPROPERTY(BlueprintAssignable, Category = "Input")
    FFloatAxisInputDelegate OnMoveRightInput;

    // Delegate for combined 2D input vector (forward, right)
    UPROPERTY(BlueprintAssignable, Category = "Input")
    FVector2DAxisInputDelegate OnMoveInputVector;

  protected:
    virtual void SetupInputComponent() override;

  private:
    // Internal handlers for axis bindings
    void HandleMoveForward(float Value);
    void HandleMoveRight(float Value);

    // Cached last values to broadcast combined vector only on change
    float CachedForwardValue = 0.f;
    float CachedRightValue = 0.f;

    void BroadcastCombinedInput();
};
