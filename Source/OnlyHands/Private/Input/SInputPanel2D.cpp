#include "Input/SInputPanel2D.h"
// #include "GenericPlatform/GenericPlatformInputDeviceMapper.h"

#include "Input/InputPanel2D.h"

bool SInputPanel2D::SupportsKeyboardFocus() const {
    return false;
}

void SInputPanel2D::Construct(const FArguments& InArgs) {
    DeadZone = FVector2D(75.0f, 75.0f);
    SwipeEventFrameCount = 3;
    FramesSinceTouch = 0;
    bHoldSwipeTrigger = false;
    bTapGate = AutoTapFrames >= 0 ? true : false;
}

FReply SInputPanel2D::OnTouchStarted(const FGeometry& MyGeometry, const FPointerEvent& Event) {
    // 4 finger touch for pause menu
    if (Event.GetPointerIndex() == 3)
        OwningWidget->OnPause.Broadcast();

    // None -> Touch
    StateTransition(EInputState::Touch);

    // Store touch position in widget local space, setup initial conditions
    TouchPosition = MyGeometry.AbsoluteToLocal(Event.GetScreenSpacePosition());
    InitialTouchPosition = TouchPosition;
    SwipePosition = TouchPosition;

    // Tap vector relative to widget center, calculate quadrant for sending tap event.
    ESwipeDirection TapDirection = GetSwipeDirection(TouchPosition, WidgetCenter);

    // Broadcast on touch event and quarant direction tap originated
    OwningWidget->OnTouch.Broadcast(TapDirection);

    // Initial timestamps
    TouchStartTimestamp = t;
    TouchTimeStamp = t;
    SwipeTimeStamp = TouchTimeStamp;
    FramesSinceTouch = 0;

    // Ensure swipe positions history is empty
    SwipePositions.Empty();

    return FReply::Handled().CaptureMouse(SharedThis(this));
}

FReply SInputPanel2D::OnTouchMoved(const FGeometry& MyGeometry, const FPointerEvent& Event) {
    // New position when touch moved
    FVector2D swipePosition = MyGeometry.AbsoluteToLocal(Event.GetScreenSpacePosition());
    FVector2D deltaPosition = swipePosition - SwipePosition;

    // Calculate swipe velocity since last frame
    float swipeTimeStamp = t;
    float deltaTime = swipeTimeStamp - SwipeTimeStamp;

    FVector2D swipeVelocity = deltaPosition / deltaTime;
    // GEngine->AddOnScreenDebugMessage(20, 2.0f, FColor::Purple, FString::Printf(TEXT("VELOCITY :: %s"),
    // *FString::SanitizeFloat(swipeVelocity.Length())));

    SwipePosition = swipePosition;
    SwipeTimeStamp = swipeTimeStamp;

    // Start Swipe
    if ((InputState == EInputState::Touch || InputState == EInputState::Hold || InputState == EInputState::Nudge) &&
        swipeVelocity.Length() > MinSwipeVelocity && IsPositionOutsideDeadZone(SwipePosition) && bInputAccept) {
        SwipeStartTimestamp = t;
        StateTransition(EInputState::Swipe);
    }

    if (SwipePositions.Num() > 0) {
        FVector2D PrevSwipePos = SwipePositions[SwipePositions.Num() - 1];
        FVector2D NextSwipePos = SwipePosition;
        float PrevSwipeVectorLength = (PrevSwipePos - InitialTouchPosition).Length();
        float NextSwipeVectorLength = (NextSwipePos - InitialTouchPosition).Length();
        GEngine->AddOnScreenDebugMessage(20, 2.0f, FColor::Cyan,
                                         FString::Printf(TEXT("PREV :: %s, NEXT :: %s"),
                                                         *FString::SanitizeFloat(PrevSwipeVectorLength),
                                                         *FString::SanitizeFloat(NextSwipeVectorLength)));

        // Check for cancel swipe
        if (NextSwipeVectorLength < PrevSwipeVectorLength) {
            OwningWidget->OnCancelSwipe.Broadcast();
            GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Orange, FString::Printf(TEXT("----SWIPE CANCEL-----")));

            StateTransition(EInputState::Hold);
            SwipePositions.Empty();
        }
    }

    // Touch -> Swipe
    // Hold -> Swipe
    // Nudge -> Swipe
    // continue swipe
    if (InputState == EInputState::Swipe && IsPositionOutsideDeadZone(SwipePosition)) {
        SwipePositions.Add(swipePosition);

        // "auto" swipe trigger while finger remains held on screen.
        if (SwipePositions.Num() > 0) {
            if (SwipePositions.Num() == 1) {
                ESwipeDirection SwipeDirection = GetSwipeDirection(SwipePosition, InitialTouchPosition);
                OwningWidget->InputDelay = FMath::TruncToInt(1000 * (SwipeTimeStamp - SwipeStartTimestamp));
                OwningWidget->OnStartSwipe.Broadcast(SwipeDirection, EInputType::ShortSwipe, swipeVelocity);
            } else if (SwipePositions.Num() > 1) {
                float swipeVectorLength = (SwipePosition - InitialTouchPosition).Length();

                FVector2D PrevSwipePos = SwipePositions[SwipePositions.Num() - 2];
                FVector2D NextSwipePos = SwipePositions[SwipePositions.Num() - 1];
                float PrevSwipeVectorLength = (PrevSwipePos - InitialTouchPosition).Length();
                float NextSwipeVectorLength =
                    (SwipePositions[SwipePositions.Num() - 1] - InitialTouchPosition).Length();

                // Check for cancel swipe
                if (NextSwipeVectorLength < PrevSwipeVectorLength) {
                    OwningWidget->OnCancelSwipe.Broadcast();
                    GEngine->AddOnScreenDebugMessage(20, 2.0f, FColor::Orange,
                                                     FString::Printf(TEXT("----SWIPE CANCEL-----")));

                    StateTransition(EInputState::Hold);
                    SwipePositions.Empty();
                }
                // Trigger swipe event while touch is still held on screen.
                else if (bHoldSwipeTrigger && swipeVectorLength > LongSwipeThreshold) {
                    TriggerSwipeEvent();
                    //					//Swipe -> Hold
                    //					StateTransition(EInputState::Hold);
                    //					TouchPosition = swipePosition;
                }
            }
            bInputAccept = false;
        }
    }

    // Touch -> Nudge
    // Swipe -> Nudge
    if (swipeVelocity.Length() > MinNudgeVelocity && swipeVelocity.Length() < MinSwipeVelocity) {
        // Cancel Swipe
        if (InputState == EInputState::Swipe) {
            // OwningWidget->OnCancelSwipe.Broadcast();
            // SwipePositions.Empty();
        }
        StateTransition(EInputState::Nudge);
    }

    if (!IsPositionOutsideDeadZone(SwipePosition))
        bInputAccept = true;

    // Virtual joystick
    FVector2D rawAxis = SwipePosition - InitialTouchPosition;
    float angle = FMath::Atan2(rawAxis.Y, rawAxis.X);
    FVector2D JoystickLimitMod = JoystickLimit;
    FVector2D joystickLimit = FVector2D(JoystickLimitMod.X * FMath::Cos(angle), JoystickLimitMod.Y * FMath::Sin(angle));

    // GEngine->AddOnScreenDebugMessage(20, 2.0f, FColor::Cyan, FString::Printf(TEXT("AXIS :: %s, %s"),
    // *FString::SanitizeFloat(rawAxis.X), *FString::SanitizeFloat(rawAxis.Y)));
    if (rawAxis.Length() < joystickLimit.Length())
        JoystickPosition = SwipePosition;
    else
        JoystickPosition = InitialTouchPosition + joystickLimit;
    float div = joystickLimit.Length();
    AxisOutput = FVector2D(FMath::Clamp(rawAxis.X / div, -1.f, 1.f), FMath::Clamp(rawAxis.Y / div, -1.f, 1.f));
    OwningWidget->OnInputChange.Broadcast(AxisOutput);

    // moving slowly, reset touch position
    //	if (swipeVelocity.Size() < MinSwipeVelocity)
    //	{
    //		SwipePositions.Empty();
    //		TouchPosition = swipePosition;
    //		TouchTimeStamp = t;
    //	}

    // Broadcast raw input vector for processing in BP
    OwningWidget->OnRawInputVector.Broadcast(SwipePosition - InitialTouchPosition);

    return FReply::Handled();
}

void SInputPanel2D::TriggerSwipeEvent() {
    if (SwipePositions.Num() == 0)
        return;

    float swipeLength = (SwipePositions[SwipePositions.Num() - 1] - InitialTouchPosition).Length();

    EInputType inputType = EInputType::None;

    // Check swipe intensity
    if (swipeLength > ShortSwipeThreshold)
        inputType = EInputType::ShortSwipe;

    if (swipeLength > ShortSwipeThreshold &&
        CheckCurve(SwipePositions[0], SwipePositions[SwipePositions.Num() - 1], InitialTouchPosition, CurveTolerance))
        inputType = EInputType::LongSwipe;

    // Check swipe direction
    ESwipeDirection swipeDirection = GetSwipeDirection(SwipePositions[0], InitialTouchPosition);

    // If swipe intensity check passed, broadcast swipe event
    if (inputType != EInputType::None) {
        OwningWidget->OnCommitSwipe.Broadcast(swipeDirection, inputType);
        // SwipePositions.Empty();
    }
}

FReply SInputPanel2D::OnTouchEnded(const FGeometry& MyGeometry, const FPointerEvent& Event) {
    ReleasePosition = MyGeometry.AbsoluteToLocal(Event.GetScreenSpacePosition());

    // Touch -> Tap
    if (InputState == EInputState::Touch && FramesSinceTouch < HoldFrames && OwningWidget->AutoTriggerTapFrames < 0) {
        ESwipeDirection tapDirection = GetSwipeDirection(ReleasePosition, WidgetCenter);
        FString tapTime = FString::SanitizeFloat(1000 * (t - TouchStartTimestamp));
        OwningWidget->OnTap.Broadcast(tapDirection);
        OwningWidget->InputDelay = FMath::TruncToInt(1000 * (t - TouchStartTimestamp));

        StateTransition(EInputState::Tap);
    }

    if (InputState == EInputState::Swipe) {
        if (IsPositionOutsideDeadZone(ReleasePosition) && !bHoldSwipeTrigger)
            TriggerSwipeEvent();
    }

    // Cancel swipe on touch ended ? Makes you have to swipe farther and more consistently.
    // OwningWidget->OnCancelSwipe.Broadcast();

    // Any -> Release
    StateTransition(EInputState::Release);

    AxisOutput = FVector2D::ZeroVector;
    TouchPosition = FVector2D::ZeroVector;
    InitialTouchPosition = FVector2D::ZeroVector;
    SwipePosition = FVector2D::ZeroVector;
    SwipePositions.Empty();
    bTapGate = OwningWidget->AutoTriggerTapFrames >= 0 ? true : false;

    return FReply::Handled().ReleaseMouseCapture();
}

/*
        -----------------TICK---------------------
*/
void SInputPanel2D::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) {
    if (WidgetCenter == FVector2D::ZeroVector) {
        float x = AllottedGeometry.GetLocalSize().X * 0.5f;
        float y = AllottedGeometry.GetLocalSize().Y * 0.5f;
        WidgetCenter.X = x;
        WidgetCenter.Y = y;
    }

    if (InputState == EInputState::Touch) {
        FramesSinceTouch++;

        // Touch -> Hold
        if (FramesSinceTouch >= HoldFrames)
            StateTransition(EInputState::Hold);

        // Touch -> Tap
        if (FramesSinceTouch >= OwningWidget->AutoTriggerTapFrames && bTapGate) {
            ESwipeDirection TapDirection = GetSwipeDirection(SwipePosition, WidgetCenter);
            OwningWidget->OnTap.Broadcast(TapDirection);
            bTapGate = false;
            StateTransition(EInputState::Tap);
        }
    }

    // Swipe -> Hold
    //	if (InputState == EInputState::Swipe && SwipeTimeStamp + SwipeTimeOut <= t)
    //		StateTransition(EInputState::Hold);

    // Release -> None
    if (InputState == EInputState::Release)
        StateTransition(EInputState::None);

    // Virtual joystick input axis
    ForwardGamepadAxis(OwningWidget->JoystickAxisX.GetFName(), AxisOutput.X);
    ForwardGamepadAxis(OwningWidget->JoystickAxisY.GetFName(), -1 * AxisOutput.Y);

    t = t + InDeltaTime;
}

FVector2D SInputPanel2D::ComputeDesiredSize(float) const {
    return FVector2D(100, 100);
}

int32 SInputPanel2D::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                             FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                             bool bParentEnabled) const {
    int RetLayerId = LayerId;

    FVector2D VisualSize = FVector2D(JoystickThumbRadius * 2.f, JoystickThumbRadius * 2.f);
    FVector2D JoystickBoundsSize = JoystickLimit * 2.f;

    // Draw virtual joystick thumb and backgroud images
    if (InputState != EInputState::None && InputState != EInputState::Tap && InputState != EInputState::Touch) {
        FSlateDrawElement::MakeBox(
            OutDrawElements, RetLayerId++,
            AllottedGeometry.ToPaintGeometry(VisualSize, FSlateLayoutTransform(JoystickPosition - (VisualSize * 0.5f))),
            &(OwningWidget->JoystickImage));

        FSlateDrawElement::MakeBox(
            OutDrawElements, RetLayerId++,
            AllottedGeometry.ToPaintGeometry(JoystickBoundsSize,
                                             FSlateLayoutTransform(InitialTouchPosition - (JoystickBoundsSize * 0.5f))),
            &(OwningWidget->JoystickBoundsImage));
    }

    // Ghosted backgroud indicating tap quadrants
    if (InputState != EInputState::Tap && InputState != EInputState::None && InputState != EInputState::Touch) {
        FSlateDrawElement::MakeBox(
            OutDrawElements, RetLayerId++,
            AllottedGeometry.ToPaintGeometry(
                OwningWidget->QuadrantGhost.ImageSize,
                FSlateLayoutTransform(InitialTouchPosition -
                                      (FVector2D(OwningWidget->QuadrantGhost.ImageSize * 0.5f)))),
            &(OwningWidget->QuadrantGhost), ESlateDrawEffect::None,
            OwningWidget->QuadrantGhost.TintColor.GetColor(InWidgetStyle));
    }

    if (bHelpersEnabled) {
        if (InputState != EInputState::Swipe && InputState != EInputState::Hold && InputState != EInputState::Nudge) {
            // Make tap quadrant diagonal helper lines
            float lineLength = 300.f;
            FVector2D TapQuadLineEndPoint0;
            FVector2D TapQuadLineEndPoint1;
            FVector2D TapQuadLineEndPoint2;
            FVector2D TapQuadLineEndPoint3;
            if (bUseDiagonalSwipeInputs) {
                TapQuadLineEndPoint0 = FVector2D(WidgetCenter.X + lineLength, WidgetCenter.Y);
                TapQuadLineEndPoint1 = FVector2D(WidgetCenter.X, WidgetCenter.Y + lineLength);
                TapQuadLineEndPoint2 = FVector2D(WidgetCenter.X, WidgetCenter.Y - lineLength);
                TapQuadLineEndPoint3 = FVector2D(WidgetCenter.X - lineLength, WidgetCenter.Y);
            } else {
                TapQuadLineEndPoint0 = FVector2D(WidgetCenter.X + lineLength, WidgetCenter.Y + lineLength);
                TapQuadLineEndPoint1 = FVector2D(WidgetCenter.X - lineLength, WidgetCenter.Y + lineLength);
                TapQuadLineEndPoint2 = FVector2D(WidgetCenter.X + lineLength, WidgetCenter.Y - lineLength);
                TapQuadLineEndPoint3 = FVector2D(WidgetCenter.X - lineLength, WidgetCenter.Y - lineLength);
            }
            TArray<FVector2D> TapQuadLinePoints0;
            TArray<FVector2D> TapQuadLinePoints1;
            TArray<FVector2D> TapQuadLinePoints2;
            TArray<FVector2D> TapQuadLinePoints3;
            TapQuadLinePoints0.Add(WidgetCenter);
            TapQuadLinePoints0.Add(TapQuadLineEndPoint0);
            TapQuadLinePoints1.Add(WidgetCenter);
            TapQuadLinePoints1.Add(TapQuadLineEndPoint1);
            TapQuadLinePoints2.Add(WidgetCenter);
            TapQuadLinePoints2.Add(TapQuadLineEndPoint2);
            TapQuadLinePoints3.Add(WidgetCenter);
            TapQuadLinePoints3.Add(TapQuadLineEndPoint3);

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
                                         TapQuadLinePoints0, ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
                                         TapQuadLinePoints1, ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
                                         TapQuadLinePoints2, ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
                                         TapQuadLinePoints3, ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );
        }

        if (InputState == EInputState::Hold || InputState == EInputState::Swipe || InputState == EInputState::Nudge) {
            // Make DeadZone circle
            const FVector2D Center = InitialTouchPosition;
            float Radius = DeadZone.X;
            const int32 NumSegments = 32;
            FLinearColor CircleColor = FLinearColor::White;
            TArray<FVector2D> CirclePoints;

            for (int32 i = 0; i <= NumSegments; ++i) {
                float Angle = (2 * PI * i) / NumSegments;
                CirclePoints.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), CirclePoints,
                                         ESlateDrawEffect::None, CircleColor,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // Make short swipe threshold circle
            Radius = ShortSwipeThreshold;
            CircleColor = FLinearColor::Yellow;
            CirclePoints.Empty();

            for (int32 i = 0; i <= NumSegments; ++i) {
                float Angle = (2 * PI * i) / NumSegments;
                CirclePoints.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), CirclePoints,
                                         ESlateDrawEffect::None, CircleColor,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // Make long swipe threshold circle
            Radius = LongSwipeThreshold;
            CircleColor = FLinearColor::Red;
            CirclePoints.Empty();

            for (int32 i = 0; i <= NumSegments; ++i) {
                float Angle = (2 * PI * i) / NumSegments;
                CirclePoints.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), CirclePoints,
                                         ESlateDrawEffect::None, CircleColor,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // Make Joystick limit circle
            Radius = JoystickLimit.X;
            CircleColor = FLinearColor::White;
            CirclePoints.Empty();

            for (int32 i = 0; i <= NumSegments; ++i) {
                float Angle = (2 * PI * i) / NumSegments;
                CirclePoints.Add(InitialTouchPosition + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), CirclePoints,
                                         ESlateDrawEffect::None, CircleColor,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // Make Joystick circle
            Radius = JoystickThumbRadius;
            CircleColor = FLinearColor::White;
            CirclePoints.Empty();

            for (int32 i = 0; i <= NumSegments; ++i) {
                float Angle = (2 * PI * i) / NumSegments;
                CirclePoints.Add(JoystickPosition + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), CirclePoints,
                                         ESlateDrawEffect::None, CircleColor,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // SwipePositions debug
            TArray<FVector2D> SwipeLines;
            for (int32 i = 0; i < SwipePositions.Num(); i++) {
                SwipeLines.Add(SwipePositions[i]);
            }

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), SwipeLines,
                                         ESlateDrawEffect::None, FColor::Green,
                                         true, // bAntialias
                                         5.0f  // Thickness
            );

            // Make diagonal helper lines
            float lineLength = 300.f;
            FVector2D GhostLineEndPoint0;
            FVector2D GhostLineEndPoint1;
            FVector2D GhostLineEndPoint2;
            FVector2D GhostLineEndPoint3;
            if (bUseDiagonalSwipeInputs) {
                GhostLineEndPoint0 = FVector2D(Center.X + lineLength, Center.Y);
                GhostLineEndPoint1 = FVector2D(Center.X, Center.Y + lineLength);
                GhostLineEndPoint2 = FVector2D(Center.X, Center.Y - lineLength);
                GhostLineEndPoint3 = FVector2D(Center.X - lineLength, Center.Y);
            } else {
                GhostLineEndPoint0 = FVector2D(Center.X + lineLength, Center.Y + lineLength);
                GhostLineEndPoint1 = FVector2D(Center.X - lineLength, Center.Y + lineLength);
                GhostLineEndPoint2 = FVector2D(Center.X + lineLength, Center.Y - lineLength);
                GhostLineEndPoint3 = FVector2D(Center.X - lineLength, Center.Y - lineLength);
            }
            TArray<FVector2D> GhostLinePoints0;
            TArray<FVector2D> GhostLinePoints1;
            TArray<FVector2D> GhostLinePoints2;
            TArray<FVector2D> GhostLinePoints3;
            GhostLinePoints0.Add(Center);
            GhostLinePoints0.Add(GhostLineEndPoint0);
            GhostLinePoints1.Add(Center);
            GhostLinePoints1.Add(GhostLineEndPoint1);
            GhostLinePoints2.Add(Center);
            GhostLinePoints2.Add(GhostLineEndPoint2);
            GhostLinePoints3.Add(Center);
            GhostLinePoints3.Add(GhostLineEndPoint3);

            const FVector2D GhostLineLongSwipeEndPoint0 = FVector2D(Center.X + lineLength, Center.Y + lineLength);
            const FVector2D GhostLineLongSwipeEndPoint1 = FVector2D(Center.X - lineLength, Center.Y + lineLength);
            const FVector2D GhostLineLongSwipeEndPoint2 = FVector2D(Center.X + lineLength, Center.Y - lineLength);
            const FVector2D GhostLineLongSwipeEndPoint3 = FVector2D(Center.X - lineLength, Center.Y - lineLength);

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), GhostLinePoints0,
                                         ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), GhostLinePoints1,
                                         ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), GhostLinePoints2,
                                         ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            FSlateDrawElement::MakeLines(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), GhostLinePoints3,
                                         ESlateDrawEffect::None, FLinearColor::Gray,
                                         true, // bAntialias
                                         0.5f  // Thickness
            );

            // Draw polygon
            //	FSlateResourceHandle ResourceHandle =
            //FSlateApplication::Get().GetRenderer()->GetResourceHandle(*GhostMat);
            //
            //	TArray<FSlateVertex> Vertices;
            //	TArray<SlateIndex> Indices;
            //
            //	FVector2f P0(Center.X, Center.Y);
            //	FVector2f P1(GhostLineEndPoint0.X, GhostLineEndPoint0.Y);
            //	FVector2f P2(GhostLineEndPoint1.X, GhostLineEndPoint1.Y);
            //
            //	FColor FillColor = FColor::Green;
            //
            //	Vertices.Add(FSlateVertex::Make<ESlateVertexRounding::Disabled>(AllottedGeometry.ToPaintGeometry().GetAccumulatedRenderTransform(),
            //P0, FVector2f::ZeroVector, FillColor));
            //	Vertices.Add(FSlateVertex::Make<ESlateVertexRounding::Disabled>(AllottedGeometry.ToPaintGeometry().GetAccumulatedRenderTransform(),
            //P1, FVector2f::ZeroVector, FillColor));
            //	Vertices.Add(FSlateVertex::Make<ESlateVertexRounding::Disabled>(AllottedGeometry.ToPaintGeometry().GetAccumulatedRenderTransform(),
            //P2, FVector2f::ZeroVector, FillColor));
            //
            //	Indices.Add(0);
            //	Indices.Add(1);
            //	Indices.Add(2);
            //
            //	FSlateDrawElement::MakeCustomVerts(
            //		OutDrawElements,
            //		LayerId,
            //		ResourceHandle,
            //		Vertices,
            //		Indices,
            //		nullptr, 0, 0
            //	);
        }
    }

    return RetLayerId;
}

bool SInputPanel2D::IsPositionOutsideDeadZone(FVector2D PositionToCheck) {
    bool xCheck = PositionToCheck.X <= InitialTouchPosition.X - DeadZone.X ||
                  PositionToCheck.X >= InitialTouchPosition.X + DeadZone.X;
    bool yCheck = PositionToCheck.Y <= InitialTouchPosition.Y - DeadZone.Y ||
                  PositionToCheck.Y >= InitialTouchPosition.Y + DeadZone.Y;
    return xCheck || yCheck;
}

bool SInputPanel2D::ForwardGamepadAxis(const FGamepadKeyNames::Type KeyName, float AnalogValue) {
    const FInputDeviceId PrimaryInputDevice = IPlatformInputDeviceMapper::Get().GetPrimaryInputDeviceForUser(
        FSlateApplicationBase::SlateAppPrimaryPlatformUser);
    return FSlateApplication::Get().OnControllerAnalog(KeyName, FSlateApplicationBase::SlateAppPrimaryPlatformUser,
                                                       PrimaryInputDevice, AnalogValue);
    return false;
}

ESwipeDirection SInputPanel2D::GetSwipeDirection(FVector2D _SwipePosition, FVector2D OriginPosition) const {
    FVector2D tapVector = _SwipePosition - OriginPosition;
    float Angle = FMath::Atan2(tapVector.Y, tapVector.X);
    ESwipeDirection tapDirection;
    if (bUseDiagonalSwipeInputs)
        tapDirection = AngleToDiagonalSwipeDirection(Angle);
    else
        tapDirection = AngleToSwipeDirection(Angle);
    return tapDirection;
}

void SInputPanel2D::StateTransition(EInputState NewState) {
    InputState = NewState;
    OwningWidget->OnInputStateChange.Broadcast(InputState);
    GEngine->AddOnScreenDebugMessage(1, 2.0f, FColor::Silver, *UEnum::GetValueAsString(InputState));
}

void SInputPanel2D::CancelSwipe() {
    OwningWidget->OnCancelSwipe.Broadcast();
}

bool SInputPanel2D::CheckCurve(FVector2D StartPosition, FVector2D EndPosition, FVector2D OriginPosition,
                               float Tolerance) {
    FVector2D StartVector = StartPosition - OriginPosition;
    FVector2D EndVector = EndPosition - OriginPosition;
    bool EndVectorResult = EndVector.Normalize();
    bool StartVectorResult = StartVector.Normalize();

    float DotProd = EndVector.Dot(StartVector);
    // GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Orange, FString::Printf(TEXT("DOT PRODUCT :: %s"),
    // *FString::SanitizeFloat(DotProd)));
    return DotProd <= Tolerance;
}
