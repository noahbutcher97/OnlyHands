// Fill out your copyright notice in the Description page of Project Settings.

#include "AbilitySystem/OHCombatAttributeSet.h"
#include "GameplayEffectExtension.h"
#include "Net/UnrealNetwork.h"

UOHCombatAttributeSet::UOHCombatAttributeSet() {}

void UOHCombatAttributeSet::PreAttributeChange(
    const FGameplayAttribute &Attribute, float &NewValue) {
  Super::PreAttributeChange(Attribute, NewValue);
}

void UOHCombatAttributeSet::PostGameplayEffectExecute(
    const FGameplayEffectModCallbackData &Data) {
  Super::PostGameplayEffectExecute(Data);

  // Clamp values after effects applied.
  if (Data.EvaluatedData.Attribute == GetHealthAttribute()) {
    SetHealth(FMath::Clamp(GetHealth(), 0.0f, GetMaxHealth()));
  } else if (Data.EvaluatedData.Attribute == GetSpecialBarAttribute()) {
    SetSpecialBar(FMath::Clamp(GetSpecialBar(), 0.0f, GetMaxSpecialBar()));
  } else if (Data.EvaluatedData.Attribute == GetStaminaAttribute()) {
    SetStamina(FMath::Clamp(GetStamina(), 0.0f, GetMaxStamina()));
  } else if (Data.EvaluatedData.Attribute == GetMomentumAttribute()) {
    SetMomentum(FMath::Clamp(GetMomentum(), 0.0f, GetMaxMomentum()));
  } else if (Data.EvaluatedData.Attribute == GetStaggerResistanceAttribute()) {
    SetStaggerResistance(
        FMath::Clamp(GetStaggerResistance(), 0.0f, GetMaxStaggerResistance()));
  } else if (Data.EvaluatedData.Attribute == GetAttackSpeedAttribute()) {
    SetAttackSpeed(FMath::Clamp(GetAttackSpeed(), 0.0f, GetMaxAttackSpeed()));
  }
}

void UOHCombatAttributeSet::GetLifetimeReplicatedProps(
    TArray<FLifetimeProperty> &OutLifetimeProps) const {
  Super::GetLifetimeReplicatedProps(OutLifetimeProps);

  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Health, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxHealth, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Attack, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Defence, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Speed, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Special, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, SpecialBar, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxSpecialBar,
                                 COND_None, REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Stamina, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxStamina, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, Momentum, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxMomentum, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, CritChance, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, StaggerResistance,
                                 COND_None, REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxStaggerResistance,
                                 COND_None, REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, AttackSpeed, COND_None,
                                 REPNOTIFY_Always);
  DOREPLIFETIME_CONDITION_NOTIFY(UOHCombatAttributeSet, MaxAttackSpeed,
                                 COND_None, REPNOTIFY_Always);
}

bool UOHCombatAttributeSet::SetAttributeDefaultValue(
    FGameplayAttribute Attribute, float Value) {
  if (Attribute.GetAttributeSetClass() != UOHCombatAttributeSet::StaticClass())
    return false;

  if (Attribute == GetHealthAttribute()) {
    Health.SetBaseValue(Value);
    Health.SetCurrentValue(Value);
  } else if (Attribute == GetMaxHealthAttribute()) {
    MaxHealth.SetBaseValue(Value);
    MaxHealth.SetCurrentValue(Value);
  } else if (Attribute == GetAttackAttribute()) {
    Attack.SetBaseValue(Value);
    Attack.SetCurrentValue(Value);
  } else if (Attribute == GetDefenceAttribute()) {
    Defence.SetBaseValue(Value);
    Defence.SetCurrentValue(Value);
  } else if (Attribute == GetSpeedAttribute()) {
    Speed.SetBaseValue(Value);
    Speed.SetCurrentValue(Value);
  } else if (Attribute == GetSpecialAttribute()) {
    Special.SetBaseValue(Value);
    Special.SetCurrentValue(Value);
  } else if (Attribute == GetSpecialBarAttribute()) {
    SpecialBar.SetBaseValue(Value);
    SpecialBar.SetCurrentValue(Value);
  } else if (Attribute == GetMaxSpecialBarAttribute()) {
    MaxSpecialBar.SetBaseValue(Value);
    MaxSpecialBar.SetCurrentValue(Value);
  } else if (Attribute == GetStaminaAttribute()) {
    Stamina.SetBaseValue(Value);
    Stamina.SetCurrentValue(Value);
  } else if (Attribute == GetMaxStaminaAttribute()) {
    MaxStamina.SetBaseValue(Value);
    MaxStamina.SetCurrentValue(Value);
  } else if (Attribute == GetMomentumAttribute()) {
    Momentum.SetBaseValue(Value);
    Momentum.SetCurrentValue(Value);
  } else if (Attribute == GetMaxMomentumAttribute()) {
    MaxMomentum.SetBaseValue(Value);
    MaxMomentum.SetCurrentValue(Value);
  } else if (Attribute == GetCritChanceAttribute()) {
    CritChance.SetBaseValue(Value);
    CritChance.SetCurrentValue(Value);
  } else if (Attribute == GetStaggerResistanceAttribute()) {
    StaggerResistance.SetBaseValue(Value);
    StaggerResistance.SetCurrentValue(Value);
  } else if (Attribute == GetMaxStaggerResistanceAttribute()) {
    MaxStaggerResistance.SetBaseValue(Value);
    MaxStaggerResistance.SetCurrentValue(Value);
  } else if (Attribute == GetAttackSpeedAttribute()) {
    AttackSpeed.SetBaseValue(Value);
    AttackSpeed.SetCurrentValue(Value);
  } else if (Attribute == GetMaxAttackSpeedAttribute()) {
    MaxAttackSpeed.SetBaseValue(Value);
    MaxAttackSpeed.SetCurrentValue(Value);
  }

  return true;
}

void UOHCombatAttributeSet::OnRep_Health(
    const FGameplayAttributeData &OldHealth) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Health, OldHealth);
}

void UOHCombatAttributeSet::OnRep_MaxHealth(
    const FGameplayAttributeData &OldMaxHealth) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxHealth, OldMaxHealth);
}

void UOHCombatAttributeSet::OnRep_Attack(
    const FGameplayAttributeData &OldAttack) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Attack, OldAttack);
}

void UOHCombatAttributeSet::OnRep_Defence(
    const FGameplayAttributeData &OldDefence) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Defence, OldDefence);
}

void UOHCombatAttributeSet::OnRep_Speed(
    const FGameplayAttributeData &OldSpeed) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Speed, OldSpeed);
}

void UOHCombatAttributeSet::OnRep_Special(
    const FGameplayAttributeData &OldSpecial) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Special, OldSpecial);
}

void UOHCombatAttributeSet::OnRep_SpecialBar(
    const FGameplayAttributeData &OldSpecialBar) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, SpecialBar, OldSpecialBar);
}

void UOHCombatAttributeSet::OnRep_MaxSpecialBar(
    const FGameplayAttributeData &OldMaxSpecialBar) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxSpecialBar,
                              OldMaxSpecialBar);
}

void UOHCombatAttributeSet::OnRep_Stamina(
    const FGameplayAttributeData &OldStamina) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Stamina, OldStamina);
}

void UOHCombatAttributeSet::OnRep_MaxStamina(
    const FGameplayAttributeData &OldMaxStamina) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxStamina, OldMaxStamina);
}

void UOHCombatAttributeSet::OnRep_Momentum(
    const FGameplayAttributeData &OldMomentum) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, Momentum, OldMomentum);
}

void UOHCombatAttributeSet::OnRep_MaxMomentum(
    const FGameplayAttributeData &OldMaxMomentum) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxMomentum,
                              OldMaxMomentum);
}

void UOHCombatAttributeSet::OnRep_CritChance(
    const FGameplayAttributeData &OldCritChance) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, CritChance, OldCritChance);
}

void UOHCombatAttributeSet::OnRep_StaggerResistance(
    const FGameplayAttributeData &OldStaggerResistance) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, StaggerResistance,
                              OldStaggerResistance);
}

void UOHCombatAttributeSet::OnRep_MaxStaggerResistance(
    const FGameplayAttributeData &OldMaxStaggerResistance) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxStaggerResistance,
                              OldMaxStaggerResistance);
}

void UOHCombatAttributeSet::OnRep_AttackSpeed(
    const FGameplayAttributeData &OldAttackSpeed) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, AttackSpeed,
                              OldAttackSpeed);
}

void UOHCombatAttributeSet::OnRep_MaxAttackSpeed(
    const FGameplayAttributeData &OldMaxAttackSpeed) {
  GAMEPLAYATTRIBUTE_REPNOTIFY(UOHCombatAttributeSet, MaxAttackSpeed,
                              OldMaxAttackSpeed);
}
