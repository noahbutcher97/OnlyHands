// Fill out your copyright notice in the Description page of Project Settings.

#include "Controller/OHPlayerController.h"
#include "Pawn/OHCombatCharacter.h"

void AOHPlayerController::AcknowledgePossession(APawn* NewPawn) {
    Super::AcknowledgePossession(NewPawn);

    AOHCombatCharacter* CharacterPawn = Cast<AOHCombatCharacter>(NewPawn);

    if (CharacterPawn) {
        CharacterPawn->GetAbilitySystemComponent()->InitAbilityActorInfo(CharacterPawn, CharacterPawn);
    }
}

void AOHPlayerController::SetupInputComponent() {
    Super::SetupInputComponent();

    // Bind your existing axis mappings here
    InputComponent->BindAxis("MoveForward", this, &AOHPlayerController::HandleMoveForward);
    InputComponent->BindAxis("MoveRight", this, &AOHPlayerController::HandleMoveRight);
}

void AOHPlayerController::HandleMoveForward(float Value) {
    CachedForwardValue = Value;
    OnMoveForwardInput.Broadcast(Value);
    BroadcastCombinedInput();
    /*#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(
                (uint64)((PTRINT)this)+12345, // unique key for forward
                0.05f, // Short duration
                FColor::Green,
                FString::Printf(TEXT("Controller Forward Axis: %.2f"), Value)
            );
        }
    #endif*/
}

void AOHPlayerController::HandleMoveRight(float Value) {
    CachedRightValue = Value;
    OnMoveRightInput.Broadcast(Value);
    BroadcastCombinedInput();
    /*#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(
                (uint64)((PTRINT)this)+12346, // unique key for right
                0.05f,
                FColor::Cyan,
                FString::Printf(TEXT("Controller Right Axis: %.2f"), Value)
            );
        }
    #endif*/
}

void AOHPlayerController::BroadcastCombinedInput() {
    OnMoveInputVector.Broadcast(CachedForwardValue, CachedRightValue);
}
