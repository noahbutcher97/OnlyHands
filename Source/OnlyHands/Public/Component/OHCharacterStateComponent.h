// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OHCharacterStateComponent.generated.h"


UCLASS(Blueprintable, BlueprintType,ClassGroup = (Combat), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHCharacterStateComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UOHCharacterStateComponent();

/*
	UFUNCTION(BlueprintCallable, Category = "State")
    void SetState(FName NewState);

	UFUNCTION(BlueprintCallable, Category = "State")
    FName GetCurrentState() const;

    UFUNCTION(BlueprintCallable, Category = "State")
    void Reset();
*/
private:
   // UPROPERTY(VisibleAnywhere, Category = "State")
    //FName CurrentState;
};
