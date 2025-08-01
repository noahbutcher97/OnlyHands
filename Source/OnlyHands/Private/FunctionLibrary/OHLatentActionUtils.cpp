// Fill out your copyright notice in the Description page of Project Settings.

<<<<<<< HEAD
== == == =

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
#include "FunctionLibrary/OHLatentActionUtils.h"

#include "Kismet/KismetMathLibrary.h"

#pragma region Latent Action Classes
             < < < < < < < HEAD class FLatentRotateComponentTo : public FPendingLatentAction {
  public:
    TWeakObjectPtr<USceneComponent> Pivot;
    FRotator StartRotation;
    FRotator TargetRotation;
    float Duration;
    float EaseAlpha;
    float Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentRotateComponentTo(USceneComponent* InPivot, FRotator InTarget, float InDuration, float InEaseAlpha,
                             const FLatentActionInfo& LatentInfo)
        : Pivot(InPivot), TargetRotation(InTarget), Duration(FMath::Max(0.01f, InDuration)),
          EaseAlpha(FMath::Clamp(InEaseAlpha, 0.0f, 10.0f)), ExecutionFunction(LatentInfo.ExecutionFunction),
          OutputLink(LatentInfo.Linkage), CallbackTarget(LatentInfo.CallbackTarget) {
        if (Pivot.IsValid()) {
            StartRotation = Pivot->GetRelativeRotation();
        }
    }

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!Pivot.IsValid()) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);

        // Optionally ease (lerp or smoothstep)
        float ActualAlpha =
            (EaseAlpha == 1.0f) ? Alpha : UKismetMathLibrary::Ease(0.f, 1.f, Alpha, EEasingFunc::EaseInOut, EaseAlpha);

        FRotator NewRot = FMath::Lerp(StartRotation, TargetRotation, ActualAlpha);
        Pivot->SetRelativeRotation(NewRot);

        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f && Pivot.IsValid()) {
            Pivot->SetRelativeRotation(TargetRotation); // Snap to target
        }
    }
};

// 2. Latent action class for moving a component
class FLatentMoveComponentTo : public FPendingLatentAction {
  public:
    TWeakObjectPtr<USceneComponent> Target;
    FVector Start, End;
    float Duration, Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentMoveComponentTo(USceneComponent* InTarget, FVector InEnd, float InDuration,
                           const FLatentActionInfo& LatentInfo)
        : Target(InTarget), End(InEnd), Duration(FMath::Max(InDuration, 0.01f)),
          ExecutionFunction(LatentInfo.ExecutionFunction), OutputLink(LatentInfo.Linkage),
          CallbackTarget(LatentInfo.CallbackTarget) {
        Start = Target.IsValid() ? Target->GetComponentLocation() : FVector::ZeroVector;
    }

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!Target.IsValid()) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);
        FVector NewLoc = FMath::Lerp(Start, End, Alpha);
        Target->SetWorldLocation(NewLoc);
        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f && Target.IsValid())
            Target->SetWorldLocation(End);
    }
};

// 3. Latent action class for interpolating a float
class FLatentInterpFloatTo : public FPendingLatentAction {
  public:
    float Start, End, Duration, Elapsed = 0.f;
    float* ResultPtr;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentInterpFloatTo(float* InResultPtr, float InStart, float InEnd, float InDuration,
                         const FLatentActionInfo& LatentInfo)
        : Start(InStart), End(InEnd), Duration(FMath::Max(InDuration, 0.01f)), ResultPtr(InResultPtr),
          ExecutionFunction(LatentInfo.ExecutionFunction), OutputLink(LatentInfo.Linkage),
          CallbackTarget(LatentInfo.CallbackTarget) {}

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!ResultPtr) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);
        *ResultPtr = FMath::Lerp(Start, End, Alpha);
        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f)
            *ResultPtr = End;
    }
};

// 4. Latent action class for delay
class FLatentDelayAction : public FPendingLatentAction {
  public:
    float Duration, Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentDelayAction(float InDuration, const FLatentActionInfo& LatentInfo)
        : Duration(FMath::Max(InDuration, 0.01f)), ExecutionFunction(LatentInfo.ExecutionFunction),
          OutputLink(LatentInfo.Linkage), CallbackTarget(LatentInfo.CallbackTarget) {}

    virtual void UpdateOperation(FLatentResponse& Response) override {
        Elapsed += Response.ElapsedTime();
        Response.DoneIf(Elapsed >= Duration);
    }
};

== == == = class FLatentRotateComponentTo : public FPendingLatentAction {
  public:
    TWeakObjectPtr<USceneComponent> Pivot;
    FRotator StartRotation;
    FRotator TargetRotation;
    float Duration;
    float EaseAlpha;
    float Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentRotateComponentTo(USceneComponent* InPivot, FRotator InTarget, float InDuration, float InEaseAlpha,
                             const FLatentActionInfo& LatentInfo)
        : Pivot(InPivot), TargetRotation(InTarget), Duration(FMath::Max(0.01f, InDuration)),
          EaseAlpha(FMath::Clamp(InEaseAlpha, 0.0f, 10.0f)), ExecutionFunction(LatentInfo.ExecutionFunction),
          OutputLink(LatentInfo.Linkage), CallbackTarget(LatentInfo.CallbackTarget) {
        if (Pivot.IsValid()) {
            StartRotation = Pivot->GetRelativeRotation();
        }
    }

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!Pivot.IsValid()) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);

        // Optionally ease (lerp or smoothstep)
        float ActualAlpha =
            (EaseAlpha == 1.0f) ? Alpha : UKismetMathLibrary::Ease(0.f, 1.f, Alpha, EEasingFunc::EaseInOut, EaseAlpha);

        FRotator NewRot = FMath::Lerp(StartRotation, TargetRotation, ActualAlpha);
        Pivot->SetRelativeRotation(NewRot);

        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f && Pivot.IsValid()) {
            Pivot->SetRelativeRotation(TargetRotation); // Snap to target
        }
    }
};

// 2. Latent action class for moving a component
class FLatentMoveComponentTo : public FPendingLatentAction {
  public:
    TWeakObjectPtr<USceneComponent> Target;
    FVector Start, End;
    float Duration, Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentMoveComponentTo(USceneComponent* InTarget, FVector InEnd, float InDuration,
                           const FLatentActionInfo& LatentInfo)
        : Target(InTarget), End(InEnd), Duration(FMath::Max(InDuration, 0.01f)),
          ExecutionFunction(LatentInfo.ExecutionFunction), OutputLink(LatentInfo.Linkage),
          CallbackTarget(LatentInfo.CallbackTarget) {
        Start = Target.IsValid() ? Target->GetComponentLocation() : FVector::ZeroVector;
    }

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!Target.IsValid()) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);
        FVector NewLoc = FMath::Lerp(Start, End, Alpha);
        Target->SetWorldLocation(NewLoc);
        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f && Target.IsValid())
            Target->SetWorldLocation(End);
    }
};

// 3. Latent action class for interpolating a float
class FLatentInterpFloatTo : public FPendingLatentAction {
  public:
    float Start, End, Duration, Elapsed = 0.f;
    float* ResultPtr;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentInterpFloatTo(float* InResultPtr, float InStart, float InEnd, float InDuration,
                         const FLatentActionInfo& LatentInfo)
        : Start(InStart), End(InEnd), Duration(FMath::Max(InDuration, 0.01f)), ResultPtr(InResultPtr),
          ExecutionFunction(LatentInfo.ExecutionFunction), OutputLink(LatentInfo.Linkage),
          CallbackTarget(LatentInfo.CallbackTarget) {}

    virtual void UpdateOperation(FLatentResponse& Response) override {
        if (!ResultPtr) {
            Response.DoneIf(true);
            return;
        }
        Elapsed += Response.ElapsedTime();
        float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);
        *ResultPtr = FMath::Lerp(Start, End, Alpha);
        Response.DoneIf(Alpha >= 1.f);
        if (Alpha >= 1.f)
            *ResultPtr = End;
    }
};

// 4. Latent action class for delay
class FLatentDelayAction : public FPendingLatentAction {
  public:
    float Duration, Elapsed = 0.f;
    FName ExecutionFunction;
    int32 OutputLink;
    FWeakObjectPtr CallbackTarget;

    FLatentDelayAction(float InDuration, const FLatentActionInfo& LatentInfo)
        : Duration(FMath::Max(InDuration, 0.01f)), ExecutionFunction(LatentInfo.ExecutionFunction),
          OutputLink(LatentInfo.Linkage), CallbackTarget(LatentInfo.CallbackTarget) {}

    virtual void UpdateOperation(FLatentResponse& Response) override {
        Elapsed += Response.ElapsedTime();
        Response.DoneIf(Elapsed >= Duration);
    }
};


>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
#pragma endregion

#pragma region Latent Actions

<<<<<<< HEAD void UOHLatentActionUtils::RotateComponentTo(UObject* WorldContextObject, USceneComponent* Pivot,
                                               FRotator TargetRotation, float Duration, float EaseAlpha,
                                               FLatentActionInfo LatentInfo) {
    if (!WorldContextObject || !Pivot)
        return;

    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World)
        return;

    FLatentActionManager& LatentManager = World->GetLatentActionManager();
    if (LatentManager.FindExistingAction<FLatentRotateComponentTo>(LatentInfo.CallbackTarget, LatentInfo.UUID) ==
        nullptr) {
        LatentManager.AddNewAction(
            LatentInfo.CallbackTarget, LatentInfo.UUID,
            new FLatentRotateComponentTo(Pivot, TargetRotation, Duration, EaseAlpha, LatentInfo));
    }
}

void UOHLatentActionUtils::MoveComponentTo(UObject* WorldContextObject, USceneComponent* Target, FVector TargetLocation,
                                           float Duration, FLatentActionInfo LatentInfo) {
    if (!WorldContextObject || !Target)
        return;
    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World)
        return;
    FLatentActionManager& LatentManager = World->GetLatentActionManager();
    if (!LatentManager.FindExistingAction<FLatentMoveComponentTo>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
        LatentManager.AddNewAction(LatentInfo.CallbackTarget, LatentInfo.UUID,
                                   new FLatentMoveComponentTo(Target, TargetLocation, Duration, LatentInfo));
    }
}

void UOHLatentActionUtils::InterpFloatTo(UObject* WorldContextObject, float StartValue, float TargetValue,
                                         float Duration, FLatentActionInfo LatentInfo, float& Result) {
    if (!WorldContextObject)
        return;
    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World)
        return;
    FLatentActionManager& LatentManager = World->GetLatentActionManager();
    if (!LatentManager.FindExistingAction<FLatentInterpFloatTo>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
        LatentManager.AddNewAction(LatentInfo.CallbackTarget, LatentInfo.UUID,
                                   new FLatentInterpFloatTo(&Result, StartValue, TargetValue, Duration, LatentInfo));
    }
}

void UOHLatentActionUtils::LatentDelay(UObject* WorldContextObject, float Duration, FLatentActionInfo LatentInfo) {
    if (!WorldContextObject)
        return;
    UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
    if (!World)
        return;
    FLatentActionManager& LatentManager = World->GetLatentActionManager();
    if (!LatentManager.FindExistingAction<FLatentDelayAction>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
        LatentManager.AddNewAction(LatentInfo.CallbackTarget, LatentInfo.UUID,
                                   new FLatentDelayAction(Duration, LatentInfo));
    }
    == == == =

                 void UOHLatentActionUtils::RotateComponentTo(UObject * WorldContextObject, USceneComponent * Pivot,
                                                              FRotator TargetRotation, float Duration, float EaseAlpha,
                                                              FLatentActionInfo LatentInfo) {
        if (!WorldContextObject || !Pivot)
            return;

        UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
        if (!World)
            return;

        FLatentActionManager& LatentManager = World->GetLatentActionManager();
        if (LatentManager.FindExistingAction<FLatentRotateComponentTo>(LatentInfo.CallbackTarget, LatentInfo.UUID) ==
            nullptr) {
            LatentManager.AddNewAction(
                LatentInfo.CallbackTarget, LatentInfo.UUID,
                new FLatentRotateComponentTo(Pivot, TargetRotation, Duration, EaseAlpha, LatentInfo));
        }
    }

    void UOHLatentActionUtils::MoveComponentTo(UObject * WorldContextObject, USceneComponent * Target,
                                               FVector TargetLocation, float Duration, FLatentActionInfo LatentInfo) {
        if (!WorldContextObject || !Target)
            return;
        UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
        if (!World)
            return;
        FLatentActionManager& LatentManager = World->GetLatentActionManager();
        if (!LatentManager.FindExistingAction<FLatentMoveComponentTo>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
            LatentManager.AddNewAction(LatentInfo.CallbackTarget, LatentInfo.UUID,
                                       new FLatentMoveComponentTo(Target, TargetLocation, Duration, LatentInfo));
        }
    }

    void UOHLatentActionUtils::InterpFloatTo(UObject * WorldContextObject, float StartValue, float TargetValue,
                                             float Duration, FLatentActionInfo LatentInfo, float& Result) {
        if (!WorldContextObject)
            return;
        UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
        if (!World)
            return;
        FLatentActionManager& LatentManager = World->GetLatentActionManager();
        if (!LatentManager.FindExistingAction<FLatentInterpFloatTo>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
            LatentManager.AddNewAction(
                LatentInfo.CallbackTarget, LatentInfo.UUID,
                new FLatentInterpFloatTo(&Result, StartValue, TargetValue, Duration, LatentInfo));
        }
    }

    void UOHLatentActionUtils::LatentDelay(UObject * WorldContextObject, float Duration, FLatentActionInfo LatentInfo) {
        if (!WorldContextObject)
            return;
        UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
        if (!World)
            return;
        FLatentActionManager& LatentManager = World->GetLatentActionManager();
        if (!LatentManager.FindExistingAction<FLatentDelayAction>(LatentInfo.CallbackTarget, LatentInfo.UUID)) {
            LatentManager.AddNewAction(LatentInfo.CallbackTarget, LatentInfo.UUID,
                                       new FLatentDelayAction(Duration, LatentInfo));
        }
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    }

#pragma endregion