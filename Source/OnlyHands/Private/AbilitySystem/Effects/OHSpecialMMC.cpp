// Fill out your copyright notice in the Description page of Project Settings.


#include "AbilitySystem/Effects/OHSpecialMMC.h"
#include "AbilitySystem/OHCombatAttributeSet.h"

UOHSpecialMMC::UOHSpecialMMC()
{
	SpecialBarDef.AttributeToCapture = UOHCombatAttributeSet::GetSpecialBarAttribute();
	SpecialBarDef.AttributeSource = EGameplayEffectAttributeCaptureSource::Source;
	SpecialBarDef.bSnapshot = true;

	MaxSpecialBarDef.AttributeToCapture = UOHCombatAttributeSet::GetMaxSpecialBarAttribute();
	MaxSpecialBarDef.AttributeSource = EGameplayEffectAttributeCaptureSource::Source;
	MaxSpecialBarDef.bSnapshot = true;

	SpecialDef.AttributeToCapture = UOHCombatAttributeSet::GetSpecialAttribute();
	SpecialDef.AttributeSource = EGameplayEffectAttributeCaptureSource::Source;
	SpecialDef.bSnapshot = true;

	RelevantAttributesToCapture.Add(SpecialBarDef);
	RelevantAttributesToCapture.Add(MaxSpecialBarDef);
	RelevantAttributesToCapture.Add(SpecialDef);
}

float UOHSpecialMMC::CalculateBaseMagnitude_Implementation(const FGameplayEffectSpec& Spec) const
{
	const FGameplayTagContainer* SourceTags = Spec.CapturedSourceTags.GetAggregatedTags();
	const FGameplayTagContainer* TargetTags = Spec.CapturedTargetTags.GetAggregatedTags();

	FAggregatorEvaluateParameters EvaluationParameters;
	EvaluationParameters.SourceTags = SourceTags;
	EvaluationParameters.TargetTags = TargetTags;

	float SpecialBar = 0.f;
	GetCapturedAttributeMagnitude(SpecialBarDef, Spec, EvaluationParameters, SpecialBar);

	float MaxSpecialBar = 0.f;
	GetCapturedAttributeMagnitude(MaxSpecialBarDef, Spec, EvaluationParameters, MaxSpecialBar);

	float Special = 0.f;
	GetCapturedAttributeMagnitude(SpecialDef, Spec, EvaluationParameters, Special);

	if (MaxSpecialBar == 0.f)
		MaxSpecialBar = 10000.f;// dont divide by zero, large denominator = small number.
	if (Special == 0.f)
		Special = 1.f;

	float returnVal = SpecialBar / MaxSpecialBar;

	return returnVal * Special;
}
