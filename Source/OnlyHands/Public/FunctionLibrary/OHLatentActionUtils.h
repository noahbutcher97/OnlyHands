// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "OHLatentActionUtils.generated.h"

/**
<<<<<<< HEAD
 *
 */
UCLASS()
class ONLYHANDS_API UOHLatentActionUtils : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

    UFUNCTION(BlueprintCallable, Category = "SecurityCamera|Utils",
              meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
    static void RotateComponentTo(UObject* WorldContextObject, USceneComponent* Pivot, FRotator TargetRotation,
                                  float Duration, float EaseAlpha, FLatentActionInfo LatentInfo);

    /** Moves a scene component smoothly to a target world location over time (latent/Blueprint-friendly) */
    UFUNCTION(BlueprintCallable, Category = "Latent|Transform",
              meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
    static void MoveComponentTo(UObject* WorldContextObject, USceneComponent* Target, FVector TargetLocation,
                                float Duration, FLatentActionInfo LatentInfo);

    /** Interpolates a float value from start to target over time (latent/Blueprint-friendly) */
    UFUNCTION(BlueprintCallable, Category = "Latent|Lerp",
              meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
    static void InterpFloatTo(UObject* WorldContextObject, float StartValue, float TargetValue, float Duration,
                              FLatentActionInfo LatentInfo, float& Result);

    /** Delay utility: latent action that completes after a set duration */
    UFUNCTION(BlueprintCallable, Category = "Latent|Utility",
              meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
    static void LatentDelay(UObject* WorldContextObject, float Duration, FLatentActionInfo LatentInfo);
    == == == = **/ UCLASS() class ONLYHANDS_API UOHLatentActionUtils : public UBlueprintFunctionLibrary {
        GENERATED_BODY()

        UFUNCTION(BlueprintCallable, Category = "SecurityCamera|Utils",
                  meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
        static void RotateComponentTo(UObject* WorldContextObject, USceneComponent* Pivot, FRotator TargetRotation,
                                      float Duration, float EaseAlpha, FLatentActionInfo LatentInfo);

        /** Moves a scene component smoothly to a target world location over time (latent/Blueprint-friendly) */
        UFUNCTION(BlueprintCallable, Category = "Latent|Transform",
                  meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
        static void MoveComponentTo(UObject* WorldContextObject, USceneComponent* Target, FVector TargetLocation,
                                    float Duration, FLatentActionInfo LatentInfo);

        /** Interpolates a float value from start to target over time (latent/Blueprint-friendly) */
        UFUNCTION(BlueprintCallable, Category = "Latent|Lerp",
                  meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
        static void InterpFloatTo(UObject* WorldContextObject, float StartValue, float TargetValue, float Duration,
                                  FLatentActionInfo LatentInfo, float& Result);

        /** Delay utility: latent action that completes after a set duration */
        UFUNCTION(BlueprintCallable, Category = "Latent|Utility",
                  meta = (Latent, LatentInfo = "LatentInfo", WorldContext = "WorldContextObject"))
        static void LatentDelay(UObject* WorldContextObject, float Duration, FLatentActionInfo LatentInfo);

	
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
    };
