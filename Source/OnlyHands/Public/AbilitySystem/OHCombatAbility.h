// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Abilities/GameplayAbility.h"
#include "OHCombatAbility.generated.h"

/**
 * 
 */
UCLASS()
class ONLYHANDS_API UOHCombatAbility : public UGameplayAbility
{
	GENERATED_BODY()

public:
	// Tells an ability to activate immediately when its granted. Used for passive abilities and abilities forced on others.
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Ability")
	bool ActivateAbilityOnGranted = false;

	// If an ability is marked as 'ActivateAbilityOnGranted', activate them immediately when given here
	// Epic's comment: Projects may want to initiate passives or do other "BeginPlay" type of logic here.
	virtual void OnAvatarSet(const FGameplayAbilityActorInfo* ActorInfo, const FGameplayAbilitySpec& Spec) override;
	
};
