#pragma once
#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
#include "OHHapticsStructs.generated.h"

UENUM(BlueprintType)
enum class EHapticCurveSamplingMode : uint8 {
    FixedRate UMETA(DisplayName = "Fixed Rate"),
    CurveKeys UMETA(DisplayName = "Curve Keys"),
    Auto UMETA(DisplayName = "Auto (Adaptive)")
};

USTRUCT(BlueprintType)
struct FOHHapticParameterCurve {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<float> Times;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<float> Values;
};

USTRUCT(BlueprintType)
struct FOHHapticEventEx {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartTime = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Duration = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FOHHapticParameterCurve IntensityCurve;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FOHHapticParameterCurve SharpnessCurve;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bTransient = true;
};

USTRUCT(BlueprintType)
struct FOHHapticEventCurve {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics")
    float StartTime = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics")
    float Duration = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics")
    TObjectPtr<UCurveFloat> IntensityCurve = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics")
    TObjectPtr<UCurveFloat> SharpnessCurve = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics")
    bool bTransient = true;
};
