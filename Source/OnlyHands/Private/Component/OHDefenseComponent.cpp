// Fill out your copyright notice in the Description page of Project Settings.


#include "Component/OHDefenseComponent.h"

// Sets default values for this component's properties
UOHDefenseComponent::UOHDefenseComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = false;

	// ...
}

/*
void UOHDefenseComponent::TryParry(float CurrentTime)
{
    if (CurrentTime >= ParryWindowStart && CurrentTime <= ParryWindowEnd)
    {
        bIsParrying = true;
        // TODO: Trigger parry animation or reaction
    }
    else
    {
        bIsParrying = false;
        // TODO: Failed parry feedback
    }
}

void UOHDefenseComponent::Reset()
{
    bIsParrying = false;
}

*/