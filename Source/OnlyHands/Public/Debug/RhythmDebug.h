#pragma once

#include "Components/ContentWidget.h"
#include "Templates/SharedPointer.h"
#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include "Engine/Texture2D.h"

#include "Debug/SRhythmDebug.h"

#include "RhythmDebug.generated.h"


UCLASS()
class ONLYHANDS_API URhythmDebug : public UWidget
{
	GENERATED_UCLASS_BODY()
public:

	TSharedPtr<SRhythmDebug> RhythmSlateRef;
	
	virtual TSharedRef<SWidget> RebuildWidget() override;

	virtual void ReleaseSlateResources(bool bReleaseChildren) override;

	UFUNCTION(BlueprintCallable)
	void UpdateParameters();

	UFUNCTION(BlueprintCallable)
	void SetInputWindows(TArray<FInputWindow> _InputWindows);

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<FInputWindow> InputWindows;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float TimeScale;

#if WITH_EDITOR
	virtual const FText GetPaletteCategory() override;

#endif
	
};
