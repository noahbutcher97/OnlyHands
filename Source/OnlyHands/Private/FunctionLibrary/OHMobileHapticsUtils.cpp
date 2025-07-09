

#include "FunctionLibrary/OHMobileHapticsUtils.h"
#include "Curves/CurveFloat.h"
#include "Data/Struct/OHHapticsStructs.h"
#include "Kismet/GameplayStatics.h"

#if PLATFORM_IOS
extern "C" void PlayiOSCoreHapticPatternEx(const FOHHapticEventEx *events,
                                           int numEvents);
#endif

void UOHMobileHapticsUtils::PlayMobileHapticPatternEx(
    UObject *WorldContext, const TArray<FOHHapticEventEx> &Events,
    bool bLeftHand) {
#if PLATFORM_IOS
  PlayiOSCoreHapticPatternEx(Events.GetData(), Events.Num());
#else
  // On non-iOS, fallback: play first event as a force feedback rumble
  if (Events.Num() > 0) {
    const FOHHapticEventEx &First = Events[0];
    float Strength = (First.IntensityCurve.Values.Num() > 0)
                         ? First.IntensityCurve.Values[0]
                         : 1.f;
    if (APlayerController *PC =
            UGameplayStatics::GetPlayerController(WorldContext, 0)) {
      PC->PlayDynamicForceFeedback(Strength, First.Duration, true, true, true,
                                   true);
    }
  }
#endif
}

#if PLATFORM_IOS
extern "C" void PlayiOSCoreHapticSimple(float intensity, float sharpness,
                                        float duration, bool bTransient);
#endif

void UOHMobileHapticsUtils::PlayMobileHapticSimple(
    UObject *WorldContext, float Intensity, float Sharpness, float Duration,
    bool bTransient, bool bLeftHand) {
#if PLATFORM_IOS
  PlayiOSCoreHapticSimple(Intensity, Sharpness, Duration, bTransient);
#else
  if (APlayerController *PC =
          UGameplayStatics::GetPlayerController(WorldContext, 0)) {
    PC->PlayDynamicForceFeedback(Intensity, Duration, true, true, true, true);
  }
#endif
}

#if PLATFORM_IOS
extern "C" void PlayiOSCoreHapticPatternEx(const FOHHapticEventEx *events,
                                           int numEvents);
#endif

namespace {
// Helper: Bake UCurveFloat into a key array for iOS native
void BakeCurveToArray(UCurveFloat *Curve, float Duration, float SampleRate,
                      TArray<float> &OutTimes, TArray<float> &OutValues) {
  if (!Curve || Duration <= 0.f || SampleRate < 1.f)
    return;

  int32 NumSamples = FMath::Max(2, FMath::RoundToInt(Duration * SampleRate));
  OutTimes.Reset(NumSamples);
  OutValues.Reset(NumSamples);

  for (int32 i = 0; i < NumSamples; ++i) {
    float Alpha = float(i) / float(NumSamples - 1);
    float Time = Alpha * Duration;
    OutTimes.Add(Time);
    OutValues.Add(Curve->GetFloatValue(Time));
  }
}
} // namespace

void UOHMobileHapticsUtils::PlayMobileHapticPatternWithCurves(
    UObject *WorldContext, const TArray<FOHHapticEventCurve> &Events,
    float SampleRate, bool bLeftHand) {
#if PLATFORM_IOS
  // Convert FOHHapticEventCurve to FOHHapticEventEx
  TArray<FOHHapticEventEx> BakedEvents;
  BakedEvents.Reserve(Events.Num());
  for (const FOHHapticEventCurve &Src : Events) {
    FOHHapticEventEx Baked;
    Baked.StartTime = Src.StartTime;
    Baked.Duration = Src.Duration;
    Baked.bTransient = Src.bTransient;
    // Sample curves into arrays
    BakeCurveToArray(Src.IntensityCurve, Src.Duration, SampleRate,
                     Baked.IntensityCurve.Times, Baked.IntensityCurve.Values);
    BakeCurveToArray(Src.SharpnessCurve, Src.Duration, SampleRate,
                     Baked.SharpnessCurve.Times, Baked.SharpnessCurve.Values);
    BakedEvents.Add(Baked);
  }
  PlayMobileHapticPatternEx(WorldContext, BakedEvents, bLeftHand);
#else
  // On non-iOS, fallback: play first event as a force feedback rumble
  if (Events.Num() > 0) {
    const FOHHapticEventCurve &First = Events[0];
    float Strength =
        (First.IntensityCurve) ? First.IntensityCurve->GetFloatValue(0.f) : 1.f;
    if (APlayerController *PC =
            UGameplayStatics::GetPlayerController(WorldContext, 0)) {
      PC->PlayDynamicForceFeedback(Strength, First.Duration, true, true, true,
                                   true);
    }
  }
#endif
}

#if PLATFORM_IOS
extern "C" void PlayiOSCoreHapticPatternEx(const FOHHapticEventEx *events,
                                           int numEvents);
#endif

void UOHMobileHapticsUtils::PlayMobileHapticCurve(UObject *WorldContext,
                                                  UCurveFloat *IntensityCurve,
                                                  float Duration,
                                                  float SampleRate) {
  if (!IntensityCurve || Duration <= 0.f)
    return;

#if PLATFORM_IOS
  // Bake the curve into times/values arrays
  TArray<float> Times;
  TArray<float> Values;

  int32 NumSamples = FMath::Max(2, FMath::RoundToInt(Duration * SampleRate));
  Times.Reserve(NumSamples);
  Values.Reserve(NumSamples);

  for (int32 i = 0; i < NumSamples; ++i) {
    float Alpha = float(i) / float(NumSamples - 1);
    float Time = Alpha * Duration;
    Times.Add(Time);
    Values.Add(IntensityCurve->GetFloatValue(Time));
  }

  // Fill the event struct (sharpness flat for simplicity, can extend as needed)
  FOHHapticEventEx HapticEvent;
  HapticEvent.StartTime = 0.f;
  HapticEvent.Duration = Duration;
  HapticEvent.bTransient = false;
  HapticEvent.IntensityCurve.Times = Times;
  HapticEvent.IntensityCurve.Values = Values;
  HapticEvent.SharpnessCurve.Times = {0.f, Duration};
  HapticEvent.SharpnessCurve.Values = {1.f, 1.f};

  TArray<FOHHapticEventEx> Events = {HapticEvent};
  PlayMobileHapticPatternEx(WorldContext, Events, false);

#else
  // Fallback: play as a simple rumble at first key’s value
  float Strength = (IntensityCurve->FloatCurve.Keys.Num() > 0)
                       ? IntensityCurve->FloatCurve.Keys[0].Value
                       : 1.f;
  if (APlayerController *PC =
          UGameplayStatics::GetPlayerController(WorldContext, 0)) {
    PC->PlayDynamicForceFeedback(Strength, Duration, true, true, true, true);
  }
#endif
}

#if PLATFORM_IOS
extern "C" void PlayiOSCoreHapticPatternEx(const FOHHapticEventEx *events,
                                           int numEvents);
#endif

namespace {
// Fixed rate resampling
void BakeCurveToArray_Fixed(UCurveFloat *Curve, float Duration,
                            float SampleRate, TArray<float> &OutTimes,
                            TArray<float> &OutValues) {
  int32 NumSamples = FMath::Max(2, FMath::RoundToInt(Duration * SampleRate));
  OutTimes.Reset(NumSamples);
  OutValues.Reset(NumSamples);
  for (int32 i = 0; i < NumSamples; ++i) {
    float Alpha = float(i) / float(NumSamples - 1);
    float Time = Alpha * Duration;
    OutTimes.Add(Time);
    OutValues.Add(Curve->GetFloatValue(Time));
  }
}

// Curve keyframes only
void BakeCurveToArray_Keys(UCurveFloat *Curve, float Duration,
                           TArray<float> &OutTimes, TArray<float> &OutValues) {
  const FRichCurve &RichCurve = Curve->FloatCurve;
  OutTimes.Reset(RichCurve.Keys.Num());
  OutValues.Reset(RichCurve.Keys.Num());
  for (const FRichCurveKey &Key : RichCurve.Keys) {
    float T = FMath::Clamp(Key.Time, 0.f, Duration);
    OutTimes.Add(T);
    OutValues.Add(Key.Value);
  }
  // Guarantee a key at end
  if (OutTimes.Num() == 0 || OutTimes.Last() < Duration) {
    OutTimes.Add(Duration);
    OutValues.Add(Curve->GetFloatValue(Duration));
  }
}

// Adaptive: if <= 5 keys, use keys, else use fixed
void BakeCurveToArray_Auto(UCurveFloat *Curve, float Duration, float SampleRate,
                           TArray<float> &OutTimes, TArray<float> &OutValues) {
  const int32 KeyCount = Curve ? Curve->FloatCurve.Keys.Num() : 0;
  if (KeyCount > 0 && KeyCount <= 5) {
    BakeCurveToArray_Keys(Curve, Duration, OutTimes, OutValues);
  } else {
    BakeCurveToArray_Fixed(Curve, Duration, SampleRate, OutTimes, OutValues);
  }
}
} // namespace

void UOHMobileHapticsUtils::PlayMobileHapticCurve_Internal(
    UObject *WorldContext, UCurveFloat *IntensityCurve, float Duration,
    float SampleRate, EHapticCurveSamplingMode SamplingMode) {
  if (!IntensityCurve || Duration <= 0.f)
    return;

#if PLATFORM_IOS
  TArray<float> Times;
  TArray<float> Values;
  switch (SamplingMode) {
  case EHapticCurveSamplingMode::CurveKeys:
    BakeCurveToArray_Keys(IntensityCurve, Duration, Times, Values);
    break;
  case EHapticCurveSamplingMode::FixedRate:
    BakeCurveToArray_Fixed(IntensityCurve, Duration, SampleRate, Times, Values);
    break;
  case EHapticCurveSamplingMode::Auto:
  default:
    BakeCurveToArray_Auto(IntensityCurve, Duration, SampleRate, Times, Values);
    break;
  }

  FOHHapticEventEx HapticEvent;
  HapticEvent.StartTime = 0.f;
  HapticEvent.Duration = Duration;
  HapticEvent.bTransient = false;
  HapticEvent.IntensityCurve.Times = Times;
  HapticEvent.IntensityCurve.Values = Values;
  HapticEvent.SharpnessCurve.Times = {0.f, Duration};
  HapticEvent.SharpnessCurve.Values = {1.f, 1.f};

  TArray<FOHHapticEventEx> Events = {HapticEvent};
  PlayMobileHapticPatternEx(WorldContext, Events, false);

#else
  float Strength = (IntensityCurve->FloatCurve.Keys.Num() > 0)
                       ? IntensityCurve->FloatCurve.Keys[0].Value
                       : 1.f;
  if (APlayerController *PC =
          UGameplayStatics::GetPlayerController(WorldContext, 0)) {
    PC->PlayDynamicForceFeedback(Strength, Duration, true, true, true, true);
  }
#endif
}

void UOHMobileHapticsUtils::PlayMobileHapticCurve_FixedRate(
    UObject *WorldContext, UCurveFloat *IntensityCurve, float Duration,
    float SampleRate) {
  PlayMobileHapticCurve_Internal(WorldContext, IntensityCurve, Duration,
                                 SampleRate,
                                 EHapticCurveSamplingMode::FixedRate);
}

void UOHMobileHapticsUtils::PlayMobileHapticCurve_Keys(
    UObject *WorldContext, UCurveFloat *IntensityCurve, float Duration) {
  PlayMobileHapticCurve_Internal(WorldContext, IntensityCurve, Duration, 60.f,
                                 EHapticCurveSamplingMode::CurveKeys);
}

void UOHMobileHapticsUtils::PlayMobileHapticCurve_Auto(
    UObject *WorldContext, UCurveFloat *IntensityCurve, float Duration,
    float SampleRate) {
  PlayMobileHapticCurve_Internal(WorldContext, IntensityCurve, Duration,
                                 SampleRate, EHapticCurveSamplingMode::Auto);
}