#pragma once

#include "../Game/Structures/S_FightStructure.h"
#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Framework/Application/SlateApplication.h"
#include "Input/Reply.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SLeafWidget.h"

class FPaintArgs;
class FSlateWindowElementList;
class URhythmDebug;

class ONLYHANDS_API SRhythmDebug : public SLeafWidget {
public:
  SLATE_BEGIN_ARGS(SRhythmDebug) {}

  SLATE_END_ARGS()

  void Construct(const FArguments &InArgs);

  virtual FVector2D ComputeDesiredSize(float) const override;
  virtual int32 OnPaint(const FPaintArgs &Args,
                        const FGeometry &AllottedGeometry,
                        const FSlateRect &MyCullingRect,
                        FSlateWindowElementList &OutDrawElements, int32 LayerId,
                        const FWidgetStyle &InWidgetStyle,
                        bool bParentEnabled) const override;
  virtual void Tick(const FGeometry &AllottedGeometry,
                    const double InCurrentTime,
                    const float InDeltaTime) override;
  virtual bool SupportsKeyboardFocus() const override;

  TArray<FInputWindow> InputWindows;

  float CurrentTime;

  float TimeScale;

  /* Reference to widget. */
  URhythmDebug *OwningWidget;

  /* Center of widget, set at runtime. */
  FVector2D WidgetCenter = FVector2D(0.f, 0.f);

  FVector2D LineOffset = FVector2D(0.f, 0.f);
};
