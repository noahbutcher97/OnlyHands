#include "Debug/SRhythmDebug.h"
// #include "GenericPlatform/GenericPlatformInputDeviceMapper.h"

#include "Debug/RhythmDebug.h"

bool SRhythmDebug::SupportsKeyboardFocus() const {
    return false;
}

void SRhythmDebug::Construct(const FArguments& InArgs) {}

/*
        -----------------TICK---------------------
*/
void SRhythmDebug::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) {
    if (WidgetCenter == FVector2D::ZeroVector) {
        float x = AllottedGeometry.GetLocalSize().X * 0.5f;
        float y = AllottedGeometry.GetLocalSize().Y * 0.5f;
        WidgetCenter.X = x;
        WidgetCenter.Y = y;
        LineOffset = WidgetCenter;
    }

    CurrentTime += InDeltaTime;

    //	LineOffset = LineOffset + FVector2D(-200.f * InDeltaTime, 0.f);
    //	CreateTimelineFromWindows();
}

FVector2D SRhythmDebug::ComputeDesiredSize(float) const {
    return FVector2D(100, 100);
}

int32 SRhythmDebug::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                            FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                            bool bParentEnabled) const {
    int RetLayerId = LayerId;

    //	FLinearColor LineColor = FLinearColor::White;
    //	TArray<FVector2D> LinePoints;
    //
    //	for (const FLineSegment& LineSegment : LineSegments)
    //	{
    //		LinePoints.Empty();
    //		LineColor = LineSegment.WindowColor;
    //		for (const FVector2D& Position : LineSegment.Positions)
    //		{
    //			LinePoints.Add(Position);
    //		}
    //
    //		FSlateDrawElement::MakeLines(
    //			OutDrawElements,
    //			LayerId,
    //			AllottedGeometry.ToPaintGeometry(),
    //			LinePoints,
    //			ESlateDrawEffect::None,
    //			LineColor,
    //			true, // bAntialias
    //			5.f  // Thickness
    //		);
    //	}

    const float PixelsPerSecond = 100.0f / TimeScale; // reverse scale
    const float MidX = WidgetCenter.X;

    const FVector2D CursorSize = FVector2D(10.f, 100.f);

    FSlateDrawElement::MakeBox(OutDrawElements, LayerId,
                               AllottedGeometry.ToPaintGeometry(WidgetCenter - (CursorSize / 2), CursorSize),
                               FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, FLinearColor::White);

    for (const FInputWindow& Window : InputWindows) {
        const float StartOffset = (Window.StartTime - CurrentTime) * PixelsPerSecond;
        const float EndOffset = (Window.EndTime - CurrentTime) * PixelsPerSecond;

        FVector2D StartPoint = FVector2D(MidX + StartOffset, WidgetCenter.Y - 10.f);
        FVector2D EndPoint = FVector2D(MidX + EndOffset, WidgetCenter.Y + 10.f);

        FSlateDrawElement::MakeBox(
            OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(StartPoint, EndPoint - StartPoint),
            FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Window.WindowColor);
    }

    return RetLayerId;
}
