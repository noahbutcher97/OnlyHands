// Fill out your copyright notice in the Description page of Project Settings.

#include "Component/OHAttackComponent.h"

// Sets default values for this component's properties
UOHAttackComponent::UOHAttackComponent() {
  // Set this component to be initialized when the game starts, and to be ticked
  // every frame.  You can turn these features off to improve performance if you
  // don't need them.
  PrimaryComponentTick.bCanEverTick = false;

  // ...
}

/*
void UOHAttackComponent::TryAttack()
{
        return;
    if (!bIsAttacking)
    {
        bIsAttacking = true;
        ComboStep++;
        // TODO: Trigger attack animation
        GetWorld()->GetTimerManager().SetTimer(ResetHandle, this,
&UOHAttackComponent::Reset, AttackCooldown, false);
    }
}

void UOHAttackComponent::Reset()
{
        return;

    bIsAttacking = false;
    ComboStep = 0;
}

*/