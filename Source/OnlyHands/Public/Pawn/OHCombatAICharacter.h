// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "AbilitySystemInterface.h"
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "OHCombatAICharacter.generated.h"

UCLASS()
class ONLYHANDS_API AOHCombatAICharacter : public ACharacter,
                                           public IAbilitySystemInterface {
  GENERATED_BODY()

public:
  // Sets default values for this character's properties
  AOHCombatAICharacter();

public:
  // Called every frame
  virtual void Tick(float DeltaTime) override;

  // Called to bind functionality to input
  virtual void SetupPlayerInputComponent(
      class UInputComponent *PlayerInputComponent) override;

  virtual class UAbilitySystemComponent *
  GetAbilitySystemComponent() const override;

  virtual class UOHCombatAttributeSet *GetAttributeSet() const;

  // virtual void OnMovementModeChanged(EMovementMode PrevMovementMode, uint8
  // PreviousCustomMode = 0) override;

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

  UPROPERTY()
  class UOHCombatAbilitySystemComponent *AbilitySystemComponent;

  UPROPERTY()
  class UOHCombatAttributeSet *CombatAttributeSet;
};
