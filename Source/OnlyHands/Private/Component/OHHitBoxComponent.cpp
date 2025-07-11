// Fill out your copyright notice in the Description page of Project Settings.

#include "Component/OHHitBoxComponent.h"

// Sets default values for this component's properties
UOHHitBoxComponent::UOHHitBoxComponent() {
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these
    // features off to improve performance if you don't need them.
    PrimaryComponentTick.bCanEverTick = false;

    // ...
}

/*
// Called when the game starts
void UOHHitBoxComponent::ActivateHitbox()
{
    FVector Origin = GetComponentLocation();
    FQuat Rotation = GetComponentQuat();

    TArray<FOverlapResult> Overlaps;
    FCollisionShape Shape = FCollisionShape::MakeBox(BoxExtent);

    GetWorld()->OverlapMultiByChannel(
        Overlaps,
        Origin,
        Rotation,
        ECC_Pawn,
        Shape
    );

    for (const FOverlapResult& Hit : Overlaps)
    {
        AActor* HitActor = Hit.GetActor();
        if (HitActor && HitActor != GetOwner())
        {
            // TODO: Notify DamageComponent on target
        }
    }

    bIsActive = true;
}


// Called every frame
void UOHHitBoxComponent::Reset()
{
    bIsActive = false;
}

*/