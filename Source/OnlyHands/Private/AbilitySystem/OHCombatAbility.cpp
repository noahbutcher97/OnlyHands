// Fill out your copyright notice in the Description page of Project Settings.

#include "AbilitySystem/OHCombatAbility.h"
#include "AbilitySystemComponent.h"

void UOHCombatAbility::OnAvatarSet(const FGameplayAbilityActorInfo *ActorInfo,
                                   const FGameplayAbilitySpec &Spec) {
  Super::OnAvatarSet(ActorInfo, Spec);

  if (ActivateAbilityOnGranted) {
    ActorInfo->AbilitySystemComponent->TryActivateAbility(Spec.Handle, false);
  }
}
