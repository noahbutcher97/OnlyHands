// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/LocalPlayer.h"
#include "GameFramework/PlayerController.h"
#include "OHPhysicsHandleController.generated.h"
#include "PhysicsEngine/PhysicsHandleComponent.h"

UCLASS()
class ONLYHANDS_API AOHPhysicsHandleController : public APlayerController {
  GENERATED_BODY()

  AOHPhysicsHandleController();

public:
  UFUNCTION(BlueprintCallable)
  void SetCameraLookMode(bool bEnable);

protected:
  virtual void BeginPlay() override;
  virtual void Tick(float DeltaTime) override;

  // Input
  virtual void SetupInputComponent() override;

private:
};