// Fill out your copyright notice in the Description page of Project Settings.

#include "AbilitySystem/AsyncAttributeChangedEvent.h"

UAsyncAttributeChangedEvent*
UAsyncAttributeChangedEvent::ListenForAttributeChange(UAbilitySystemComponent* AbilitySystemComponent,
                                                      FGameplayAttribute Attribute) {
    UAsyncAttributeChangedEvent* WaitForAttributeChangedTask = NewObject<UAsyncAttributeChangedEvent>();
    WaitForAttributeChangedTask->ASC = AbilitySystemComponent;
    WaitForAttributeChangedTask->AttributeToListenFor = Attribute;

    if (!IsValid(AbilitySystemComponent) || !Attribute.IsValid()) {
        WaitForAttributeChangedTask->RemoveFromRoot();
        return nullptr;
    }

    AbilitySystemComponent->GetGameplayAttributeValueChangeDelegate(Attribute).AddUObject(
        WaitForAttributeChangedTask, &UAsyncAttributeChangedEvent::AttributeChanged);

    return WaitForAttributeChangedTask;
}

UAsyncAttributeChangedEvent*
UAsyncAttributeChangedEvent::ListenForAttributesChange(UAbilitySystemComponent* AbilitySystemComponent,
                                                       TArray<FGameplayAttribute> Attributes) {
    UAsyncAttributeChangedEvent* WaitForAttributeChangedTask = NewObject<UAsyncAttributeChangedEvent>();
    WaitForAttributeChangedTask->ASC = AbilitySystemComponent;
    WaitForAttributeChangedTask->AttributesToListenFor = Attributes;

    if (!IsValid(AbilitySystemComponent) || Attributes.Num() < 1) {
        WaitForAttributeChangedTask->RemoveFromRoot();
        return nullptr;
    }

    for (FGameplayAttribute Attribute : Attributes) {
        AbilitySystemComponent->GetGameplayAttributeValueChangeDelegate(Attribute).AddUObject(
            WaitForAttributeChangedTask, &UAsyncAttributeChangedEvent::AttributeChanged);
    }

    return WaitForAttributeChangedTask;
}

void UAsyncAttributeChangedEvent::EndTask() {
    if (IsValid(ASC)) {
        ASC->GetGameplayAttributeValueChangeDelegate(AttributeToListenFor).RemoveAll(this);

        for (FGameplayAttribute Attribute : AttributesToListenFor) {
            ASC->GetGameplayAttributeValueChangeDelegate(Attribute).RemoveAll(this);
        }
    }

    SetReadyToDestroy();
    MarkAsGarbage();
}

void UAsyncAttributeChangedEvent::AttributeChanged(const FOnAttributeChangeData& Data) {
    OnAttributeChanged.Broadcast(Data.Attribute, Data.NewValue, Data.OldValue);
}
