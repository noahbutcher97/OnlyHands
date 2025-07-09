// Fill out your copyright notice in the Description page of Project Settings.

#include "Component/OHDamageComponent.h"

// Sets default values for this component's properties
UOHDamageComponent::UOHDamageComponent() {
  // Set this component to be initialized when the game starts, and to be ticked
  // every frame.  You can turn these features off to improve performance if you
  // don't need them.
  PrimaryComponentTick.bCanEverTick = false;

  // ...
}
/*
void UOHDamageComponent::ApplyHitReaction(float Force)
{
    if (Force > 0.0f)
    {
        bIsStaggered = true;
        // TODO: Trigger montage, particles, or notify animation graph
    }
}

void UOHDamageComponent::Reset()
{
    bIsStaggered = false;
}

*/