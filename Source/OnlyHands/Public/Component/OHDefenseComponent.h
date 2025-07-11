// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OHDefenseComponent.generated.h"

UCLASS(Blueprintable, BlueprintType, ClassGroup = (State), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHDefenseComponent : public UActorComponent {
    GENERATED_BODY()

  public:
    // Sets default values for this component's properties
    UOHDefenseComponent();
    /*
        UFUNCTION(BlueprintCallable, Category = "Combat")
        void TryParry(float CurrentTime);

        UFUNCTION(BlueprintCallable, Category = "Combat")
        void Reset();

    protected:
        UPROPERTY(EditAnywhere)
        float ParryWindowStart = 0.2f;

        UPROPERTY(EditAnywhere)
        float ParryWindowEnd = 0.6f;

    private:
        UPROPERTY(VisibleAnywhere, Category = "Combat")
        bool bIsParrying = false;
            */
};
