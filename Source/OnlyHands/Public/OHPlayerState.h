// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerState.h"
#include "AbilitySystem/OHCombatAttributeSet.h"
#include "AbilitySystem/OHCombatAbilitySystemComponent.h"
#include "AbilitySystemInterface.h"
#include "OHPlayerState.generated.h"

/**
 * 
 */
UCLASS()
class ONLYHANDS_API AOHPlayerState : public APlayerState, public IAbilitySystemInterface
{
	GENERATED_BODY()

public:
	AOHPlayerState();

	// Implement IAbilitySystemInterface
	class UAbilitySystemComponent* GetAbilitySystemComponent() const override;

	class UOHCombatAttributeSet* GetAttributeSet() const;

protected:

	UPROPERTY()
	class UOHCombatAbilitySystemComponent* AbilitySystemComponent;

	UPROPERTY()
	class UOHCombatAttributeSet* CombatAttributeSet;

	
};
