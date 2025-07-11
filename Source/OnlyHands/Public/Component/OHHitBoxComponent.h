// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "OHHitBoxComponent.generated.h"

UCLASS(Blueprintable, BlueprintType, ClassGroup = (State), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHHitBoxComponent : public USceneComponent {
    GENERATED_BODY()

  public:
    // Sets default values for this component's properties
    UOHHitBoxComponent();

    /*:
        void ActivateHitbox();
        void Reset();

    protected:
        UPROPERTY(EditAnywhere)
        FVector BoxExtent = FVector(50.0f, 50.0f, 50.0f);

    private:
        bool bIsActive = false;
        */
};
