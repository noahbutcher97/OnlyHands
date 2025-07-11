// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OHStatusComponent.generated.h"

UCLASS(Blueprintable, BlueprintType, ClassGroup = (State), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHStatusComponent : public UActorComponent {
    GENERATED_BODY()

  public:
    // Sets default values for this component's properties
    UOHStatusComponent();

    /*
        UFUNCTION(BlueprintCallable, Category = "Status")
        void TakeDamage(float Amount);
        UFUNCTION(BlueprintCallable, Category = "Status")
        void GainStamina(float Amount);
        UFUNCTION(BlueprintCallable, Category = "Status")
        void Reset();

    protected:
        UPROPERTY(EditAnywhere)
        float MaxStamina = 100.0f;

        UPROPERTY(EditAnywhere)
        float MaxHealth = 100.0f;

    private:
        UPROPERTY(VisibleAnywhere, Category = "Status")
        float CurrentStamina;
        UPROPERTY(VisibleAnywhere, Category = "Status")
        float CurrentHealth;
        UPROPERTY(VisibleAnywhere, Category = "Status")
        float MomentumLevel;
        */
};
