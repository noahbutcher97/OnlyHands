#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "OHInputComponent.generated.h"

UCLASS(Blueprintable, BlueprintType, ClassGroup = (State),
       meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHInputComponent : public UActorComponent {
  GENERATED_BODY()

public:
  // Sets default values for this component's properties
  UOHInputComponent();
};
