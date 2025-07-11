// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "AbilitySystemInterface.h"
#include "AbilitySystemComponent.h"
#include "Component/OHMovementComponent.h"
#include "OHCombatCharacter.generated.h"

UCLASS()
class ONLYHANDS_API AOHCombatCharacter : public ACharacter, public IAbilitySystemInterface
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	AOHCombatCharacter();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	virtual void PossessedBy(AController* NewController) override;

	virtual void UnPossessed() override;
	
	virtual void OnRep_PlayerState() override;

	virtual class UAbilitySystemComponent* GetAbilitySystemComponent() const override;

	virtual class UOHCombatAttributeSet* GetAttributeSet() const;
	void EnsureMovementComponentExists();

	virtual void OnMovementModeChanged(EMovementMode PrevMovementMode, uint8 PreviousCustomMode = 0) override;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Movement")
	UOHMovementComponent* OHMovementComponent;
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UAbilitySystemComponent* AbilitySystemComponent;

	UOHCombatAttributeSet* AttributeSet;



};
