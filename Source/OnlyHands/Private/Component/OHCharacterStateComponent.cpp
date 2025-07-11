// Fill out your copyright notice in the Description page of Project Settings.


#include "Component/OHCharacterStateComponent.h"

// Sets default values for this component's properties
UOHCharacterStateComponent::UOHCharacterStateComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = false;

	// ...
}
/*

void UOHCharacterStateComponent::SetState(FName NewState)
{
    if (NewState != CurrentState)
    {
        CurrentState = NewState;
        // TODO: Broadcast state change event
    }
    return;
}

FName UOHCharacterStateComponent::GetCurrentState() const
{
    // CurrentState;
}

void UOHCharacterStateComponent::Reset()
{
    CurrentState = NAME_None;
}

*/