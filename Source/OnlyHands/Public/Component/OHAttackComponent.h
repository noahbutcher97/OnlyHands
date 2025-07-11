// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OHAttackComponent.generated.h"
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Combat), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHAttackComponent : public UActorComponent {
    GENERATED_BODY()

  public:
    UOHAttackComponent();

    //    UFUNCTION(BlueprintCallable, Category = "Combat")
    //    void TryAttack();
    //
    //    UFUNCTION(BlueprintCallable, Category = "Combat")
    //    void Reset();

  protected:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Combat")
    float AttackCooldown = 1.0f;

  private:
    UPROPERTY(VisibleAnywhere, Category = "Combat")
    int32 ComboStep = 0;

    FTimerHandle ResetHandle;

    UPROPERTY(VisibleAnywhere, Category = "Combat")
    bool bIsAttacking = false;
};