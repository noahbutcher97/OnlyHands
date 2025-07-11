// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawn/OHCombatCharacter.h"
#include "OHPlayerState.h"
#include "AbilitySystemComponent.h"
#include "Component/OHMovementComponent.h"
#include "Controller/OHPlayerController.h"
#include "GameFramework/CharacterMovementComponent.h"

// Sets default values
AOHCombatCharacter::AOHCombatCharacter() {
    // Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need
    // it.
    PrimaryActorTick.bCanEverTick = true;
    AbilitySystemComponent = CreateDefaultSubobject<UOHCombatAbilitySystemComponent>(TEXT("AbilitySystemComponent"));
    AbilitySystemComponent->SetIsReplicated(true);

    // Mixed mode means we only replicated the GEs to ourselves, not the GEs to simulated proxies. If another
    // GDPlayerState (Hero) receives a GE, the Server won't tell us about it. Attributes, GameplayTags, and GameplayCues
    // will still replicate to us.
    AbilitySystemComponent->SetReplicationMode(EGameplayEffectReplicationMode::Mixed);

    // Create the attribute set, this replicates by default
    // Adding it as a subobject of the owning actor of an AbilitySystemComponent
    // automatically registers the AttributeSet with the AbilitySystemComponent
    AttributeSet = CreateDefaultSubobject<UOHCombatAttributeSet>(TEXT("CombatAttributeSet"));
}

// Called when the game starts or when spawned
void AOHCombatCharacter::BeginPlay() {
    Super::BeginPlay();
}

// Called every frame
void AOHCombatCharacter::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void AOHCombatCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) {
    Super::SetupPlayerInputComponent(PlayerInputComponent);
}

auto AOHCombatCharacter::PossessedBy(AController* NewController) -> void {
    Super::PossessedBy(NewController);

    EnsureMovementComponentExists();

    // If player controlled, bind delegates now
    if (AOHPlayerController* PC = Cast<AOHPlayerController>(NewController)) {
        if (OHMovementComponent) {
            // Unbind first to prevent duplicates (if previously bound)
            PC->OnMoveForwardInput.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveForward);
            PC->OnMoveRightInput.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveRight);
            PC->OnMoveInputVector.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveVector);

            // Bind new
            PC->OnMoveForwardInput.AddDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveForward);
            PC->OnMoveRightInput.AddDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveRight);
            PC->OnMoveInputVector.AddDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveVector);
        }
    }
    if (AbilitySystemComponent) {
        // Init ASC on server
        AbilitySystemComponent->InitAbilityActorInfo(this, this);
    }

    SetOwner(NewController);
}

void AOHCombatCharacter::UnPossessed() {
    Super::UnPossessed();
    if (AOHPlayerController* PC = Cast<AOHPlayerController>(GetController())) {
        if (OHMovementComponent) {
            PC->OnMoveForwardInput.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveForward);
            PC->OnMoveRightInput.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveRight);
            PC->OnMoveInputVector.RemoveDynamic(OHMovementComponent, &UOHMovementComponent::OnReceiveMoveVector);
        }
    }
}

void AOHCombatCharacter::OnRep_PlayerState() {
    Super::OnRep_PlayerState();

    //	AOHPlayerState* PS = GetPlayerState<AOHPlayerState>();
    //
    //	if (PS)
    //	{
    //		// Init ASC on Client
    //		PS->GetAbilitySystemComponent()->InitAbilityActorInfo(PS, this);
    //		AbilitySystemComponent = PS->GetAbilitySystemComponent();
    //	}
}

UAbilitySystemComponent* AOHCombatCharacter::GetAbilitySystemComponent() const {
    return AbilitySystemComponent;
}

UOHCombatAttributeSet* AOHCombatCharacter::GetAttributeSet() const {
    return AttributeSet;
}

void AOHCombatCharacter::EnsureMovementComponentExists() {
    if (!OHMovementComponent) {
        OHMovementComponent = FindComponentByClass<UOHMovementComponent>();
        if (!OHMovementComponent) {
            OHMovementComponent =
                NewObject<UOHMovementComponent>(this, UOHMovementComponent::StaticClass(), TEXT("OHMovementComponent"));
            OHMovementComponent->RegisterComponent();
            OHMovementComponent->InitializeComponent();
        }
    }
}

void AOHCombatCharacter::OnMovementModeChanged(EMovementMode PrevMovementMode, uint8 PreviousCustomMode) {
    Super::OnMovementModeChanged(PrevMovementMode, PreviousCustomMode);

    // Notify movement component
    if (OHMovementComponent) {
        OHMovementComponent->HandleMovementModeChanged(GetCharacterMovement()->MovementMode,
                                                       GetCharacterMovement()->CustomMovementMode);
    }
}