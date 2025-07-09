#pragma once

#include "CoreMinimal.h"
#include "OHHapticsStructs.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "OHMobileHapticsUtils.generated.h"


UCLASS()
class ONLYHANDS_API UOHMobileHapticsUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticPatternEx(
		UObject* WorldContext,
		const TArray<FOHHapticEventEx>& Events,
		bool bLeftHand = false);


	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticSimple(
		UObject* WorldContext,
		float Intensity = 1.0f,
		float Sharpness = 1.0f,
		float Duration = 0.1f,
		bool bTransient = true,
		bool bLeftHand = false);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticPatternWithCurves(
		UObject* WorldContext,
		const TArray<FOHHapticEventCurve>& Events,
		float SampleRate = 60.f, // Samples per second
		bool bLeftHand = false);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticCurve(
		UObject* WorldContext,
		UCurveFloat* IntensityCurve,
		float Duration,
		float SampleRate = 60.f);

	// Only in the .cpp file or as a private/protected static method in the class
	static void PlayMobileHapticCurve_Internal(
		UObject* WorldContext,
		UCurveFloat* IntensityCurve,
		float Duration,
		float SampleRate,
		EHapticCurveSamplingMode SamplingMode);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticCurve_FixedRate(
		UObject* WorldContext,
		UCurveFloat* IntensityCurve,
		float Duration,
		float SampleRate = 60.f);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticCurve_Keys(
		UObject* WorldContext,
		UCurveFloat* IntensityCurve,
		float Duration);

	UFUNCTION(BlueprintCallable, Category="OnlyHands|Mobile|Haptics", meta=(WorldContext="WorldContext"))
	static void PlayMobileHapticCurve_Auto(
		UObject* WorldContext,
		UCurveFloat* IntensityCurve,
		float Duration,
		float SampleRate = 60.f);
};


