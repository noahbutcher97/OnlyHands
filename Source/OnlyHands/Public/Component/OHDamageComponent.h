// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OHDamageComponent.generated.h"


UCLASS(Blueprintable, BlueprintType, ClassGroup = (Combat), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHDamageComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UOHDamageComponent();

	/*
public:
	UFUNCTION(BlueprintCallable, Category = "Combat")
	void ApplyHitReaction(float Force);
	
	UFUNCTION(BlueprintCallable, Category = "Combat")
	void Reset();

private:
	UPROPERTY(VisibleAnywhere, Category = "Combat")
	bool bIsStaggered = false;
	*/	
};
