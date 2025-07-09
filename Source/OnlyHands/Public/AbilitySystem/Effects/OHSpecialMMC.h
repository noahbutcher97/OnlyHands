// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameplayEffectTypes.h"
#include "GameplayModMagnitudeCalculation.h"
#include "OHSpecialMMC.generated.h"

/**
 *
 */
UCLASS()
class ONLYHANDS_API UOHSpecialMMC : public UGameplayModMagnitudeCalculation {
  GENERATED_BODY()

  UOHSpecialMMC();

  FGameplayEffectAttributeCaptureDefinition SpecialBarDef;
  FGameplayEffectAttributeCaptureDefinition MaxSpecialBarDef;
  FGameplayEffectAttributeCaptureDefinition SpecialDef;

  float
  CalculateBaseMagnitude_Implementation(const FGameplayEffectSpec &Spec) const;
};
