// InputWindowNotifyState.h
#pragma once

#include "Animation/AnimNotifies/AnimNotifyState.h"
#include "CoreMinimal.h"
#include "InputWindowNotifyState.generated.h"

UCLASS()
class ONLYHANDS_API UInputWindowNotifyState : public UAnimNotifyState {
  GENERATED_BODY()

public:
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input Window")
  FName WindowName;

  virtual FString GetNotifyName_Implementation() const override {
    return FString::Printf(TEXT("%s"), *WindowName.ToString());
  }
};
