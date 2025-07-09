// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "AbilitySystemComponent.h"
#include "AttributeSet.h"
#include "CoreMinimal.h"
#include "OHCombatAttributeSet.generated.h"

#define ATTRIBUTE_ACCESSORS(ClassName, PropertyName)                           \
  GAMEPLAYATTRIBUTE_PROPERTY_GETTER(ClassName, PropertyName)                   \
  GAMEPLAYATTRIBUTE_VALUE_GETTER(PropertyName)                                 \
  GAMEPLAYATTRIBUTE_VALUE_SETTER(PropertyName)                                 \
  GAMEPLAYATTRIBUTE_VALUE_INITTER(PropertyName)

/**
 *
 */
UCLASS()
class ONLYHANDS_API UOHCombatAttributeSet : public UAttributeSet {
  GENERATED_BODY()

public:
  UOHCombatAttributeSet();

  // AttributeSet Overrides
  virtual void PreAttributeChange(const FGameplayAttribute &Attribute,
                                  float &NewValue) override;
  virtual void PostGameplayEffectExecute(
      const FGameplayEffectModCallbackData &Data) override;

  virtual void GetLifetimeReplicatedProps(
      TArray<FLifetimeProperty> &OutLifetimeProps) const override;

  UFUNCTION(BlueprintCallable, Category = "Attributes")
  bool SetAttributeDefaultValue(FGameplayAttribute Attribute, float Value);

  UPROPERTY(BlueprintReadOnly, Category = "Health",
            ReplicatedUsing = OnRep_Health)
  FGameplayAttributeData Health;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Health)

  UPROPERTY(BlueprintReadOnly, Category = "Health",
            ReplicatedUsing = OnRep_MaxHealth)
  FGameplayAttributeData MaxHealth;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxHealth)

  UPROPERTY(BlueprintReadOnly, Category = "Attack",
            ReplicatedUsing = OnRep_Attack)
  FGameplayAttributeData Attack;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Attack)

  UPROPERTY(BlueprintReadOnly, Category = "Defence",
            ReplicatedUsing = OnRep_Defence)
  FGameplayAttributeData Defence;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Defence)

  UPROPERTY(BlueprintReadOnly, Category = "Speed",
            ReplicatedUsing = OnRep_Speed)
  FGameplayAttributeData Speed;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Speed)

  UPROPERTY(BlueprintReadOnly, Category = "Special",
            ReplicatedUsing = OnRep_Special)
  FGameplayAttributeData Special;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Special)

  UPROPERTY(BlueprintReadOnly, Category = "SpecialBar",
            ReplicatedUsing = OnRep_Special)
  FGameplayAttributeData SpecialBar;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, SpecialBar)

  UPROPERTY(BlueprintReadOnly, Category = "SpecialBar",
            ReplicatedUsing = OnRep_Special)
  FGameplayAttributeData MaxSpecialBar;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxSpecialBar)

  UPROPERTY(BlueprintReadOnly, Category = "Stamina",
            ReplicatedUsing = OnRep_Stamina)
  FGameplayAttributeData Stamina;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Stamina)

  UPROPERTY(BlueprintReadOnly, Category = "Stamina",
            ReplicatedUsing = OnRep_MaxStamina)
  FGameplayAttributeData MaxStamina;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxStamina)

  UPROPERTY(BlueprintReadOnly, Category = "Momentum",
            ReplicatedUsing = OnRep_Momentum)
  FGameplayAttributeData Momentum;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, Momentum)

  UPROPERTY(BlueprintReadOnly, Category = "Momentum",
            ReplicatedUsing = OnRep_MaxMomentum)
  FGameplayAttributeData MaxMomentum;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxMomentum)

  UPROPERTY(BlueprintReadOnly, Category = "CritChance",
            ReplicatedUsing = OnRep_CritChance)
  FGameplayAttributeData CritChance;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, CritChance)

  UPROPERTY(BlueprintReadOnly, Category = "StaggerResistance",
            ReplicatedUsing = OnRep_StaggerResistance)
  FGameplayAttributeData StaggerResistance;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, StaggerResistance)

  UPROPERTY(BlueprintReadOnly, Category = "StaggerResistance",
            ReplicatedUsing = OnRep_MaxStaggerResistance)
  FGameplayAttributeData MaxStaggerResistance;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxStaggerResistance)

  UPROPERTY(BlueprintReadOnly, Category = "AttackSpeed",
            ReplicatedUsing = OnRep_AttackSpeed)
  FGameplayAttributeData AttackSpeed;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, AttackSpeed)

  UPROPERTY(BlueprintReadOnly, Category = "AttackSpeed",
            ReplicatedUsing = OnRep_MaxAttackSpeed)
  FGameplayAttributeData MaxAttackSpeed;
  ATTRIBUTE_ACCESSORS(UOHCombatAttributeSet, MaxAttackSpeed)

protected:
  UFUNCTION()
  virtual void OnRep_Health(const FGameplayAttributeData &OldHealth);

  UFUNCTION()
  virtual void OnRep_MaxHealth(const FGameplayAttributeData &OldMaxHealth);

  UFUNCTION()
  virtual void OnRep_Attack(const FGameplayAttributeData &OldAttack);

  UFUNCTION()
  virtual void OnRep_Defence(const FGameplayAttributeData &OldDefence);

  UFUNCTION()
  virtual void OnRep_Speed(const FGameplayAttributeData &OldSpeed);

  UFUNCTION()
  virtual void OnRep_Special(const FGameplayAttributeData &OldSpecial);

  UFUNCTION()
  virtual void OnRep_SpecialBar(const FGameplayAttributeData &OldSpecialBar);

  UFUNCTION()
  virtual void
  OnRep_MaxSpecialBar(const FGameplayAttributeData &OldMaxSpecialBar);

  UFUNCTION()
  virtual void OnRep_Stamina(const FGameplayAttributeData &OldStamina);

  UFUNCTION()
  virtual void OnRep_MaxStamina(const FGameplayAttributeData &OldMaxStamina);

  UFUNCTION()
  virtual void OnRep_Momentum(const FGameplayAttributeData &OldMomentum);

  UFUNCTION()
  virtual void OnRep_MaxMomentum(const FGameplayAttributeData &OldMaxMomentum);

  UFUNCTION()
  virtual void OnRep_CritChance(const FGameplayAttributeData &OldCritChance);

  UFUNCTION()
  virtual void
  OnRep_StaggerResistance(const FGameplayAttributeData &OldStaggerResistance);

  UFUNCTION()
  virtual void OnRep_MaxStaggerResistance(
      const FGameplayAttributeData &OldMaxStaggerResistance);

  UFUNCTION()
  virtual void OnRep_AttackSpeed(const FGameplayAttributeData &OldAttackSpeed);

  UFUNCTION()
  virtual void
  OnRep_MaxAttackSpeed(const FGameplayAttributeData &OldMaxAttackSpeed);
};
