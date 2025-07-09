// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawn/OHCombatAICharacter.h"
#include "AbilitySystem/OHCombatAbilitySystemComponent.h"
#include "AbilitySystem/OHCombatAttributeSet.h"

// Sets default values
AOHCombatAICharacter::AOHCombatAICharacter() {
  // Create ability system component, and set it to be explicitly replicated
  AbilitySystemComponent =
      CreateDefaultSubobject<UOHCombatAbilitySystemComponent>(
          TEXT("AbilitySystemComponent"));
  AbilitySystemComponent->SetIsReplicated(true);

  AbilitySystemComponent->SetReplicationMode(
      EGameplayEffectReplicationMode::Minimal);

  // Create the attribute set, this replicates by default
  // Adding it as a subobject of the owning actor of an AbilitySystemComponent
  // automatically registers the AttributeSet with the AbilitySystemComponent
  CombatAttributeSet =
      CreateDefaultSubobject<UOHCombatAttributeSet>(TEXT("CombatAttributeSet"));

  // Set this character to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AOHCombatAICharacter::BeginPlay() {
  Super::BeginPlay();

  if (AbilitySystemComponent)
    AbilitySystemComponent->InitAbilityActorInfo(this, this);
}

// Called every frame
void AOHCombatAICharacter::Tick(float DeltaTime) { Super::Tick(DeltaTime); }

// Called to bind functionality to input
void AOHCombatAICharacter::SetupPlayerInputComponent(
    UInputComponent *PlayerInputComponent) {
  Super::SetupPlayerInputComponent(PlayerInputComponent);
}

UAbilitySystemComponent *
AOHCombatAICharacter::GetAbilitySystemComponent() const {
  return AbilitySystemComponent;
}

UOHCombatAttributeSet *AOHCombatAICharacter::GetAttributeSet() const {
  return CombatAttributeSet;
}
