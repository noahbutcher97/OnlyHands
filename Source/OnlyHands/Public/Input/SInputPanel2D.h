#pragma once

#include "CoreMinimal.h"
#include "Input/Reply.h"
#include "Runtime/Launch/Resources/Version.h"
#include  "Framework/Application/SlateApplication.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SLeafWidget.h"

class FPaintArgs;
class FSlateWindowElementList;
class UInputPanel2D;
class InputPanel2D;


struct FTouchIndex
{
	int PointerIndex = -1;
	FVector2D Location;
};

UENUM(BlueprintType)
enum class EInputState : uint8 {
	None,
	Touch,
	Tap,
	Swipe,
	Nudge,
	Hold,
	Release
};

UENUM(BlueprintType)
enum ESwipeDirection
{
	Left,
	Up,
	Right,
	Down,
	DownRight,
	DownLeft,
	UpRight,
	UpLeft
};


class ONLYHANDS_API SInputPanel2D : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SInputPanel2D)
		{
		}

	SLATE_END_ARGS()


	void Construct(const FArguments& InArgs);

	virtual FVector2D ComputeDesiredSize(float) const override;
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
	                      FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
	                      bool bParentEnabled) const override;
	virtual FReply OnTouchStarted(const FGeometry& MyGeometry, const FPointerEvent& Event) override;
	virtual FReply OnTouchMoved(const FGeometry& MyGeometry, const FPointerEvent& Event) override;
	void TriggerSwipeEvent();
	virtual FReply OnTouchEnded(const FGeometry& MyGeometry, const FPointerEvent& Event) override;
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;
	virtual bool SupportsKeyboardFocus() const override;

	bool IsPositionOutsideDeadZone(FVector2D PositionToCheck);

	bool ForwardGamepadAxis(const FGamepadKeyNames::Type KeyName, float AnalogValue);

	virtual ESwipeDirection GetSwipeDirection(const FVector2D _SwipePosition, const FVector2D OriginPosition) const;

	bool CheckCurve(FVector2D StartPosition, FVector2D EndPosition, FVector2D OriginPosition, float Tolerance);

	void StateTransition(EInputState NewState);

	void CancelSwipe();

public:

	/* Show helper gizmos on screen. */
	bool bHelpersEnabled = false;

	FSlateBrush* GhostMat;
	/* Reference to widget. */
	UInputPanel2D* OwningWidget;

	FVector2D DeadZone = FVector2D(10.0f,10.0f);

	/* Frame delay to register finger held on screen. Consequently also the time frame to press and release for tap recognition. */
	int HoldFrames = 2;

	int AutoTapFrames = -1;

//----------Swipe Params-----------
	/* Minimum speed to move finger for swipe registration. */
	float MinSwipeVelocity = 1000.f;
	/* Distance to move finger for short swipe. */
	float ShortSwipeThreshold = 100.f;
	/* Distance to move finger for long swipe. */
	float LongSwipeThreshold = 200.f;
	/* Time to complete swipe, ie time to switch from swipe to hold state if no swipe is triggered. */
	float SwipeTimeOut = 0.016666f;
//----------Swipe Params-----------

	/* Toggle diagonal vs vert/horiz swipe and tap inputs. */
	bool bUseDiagonalSwipeInputs;

	/* Bounds of virtual joystick. */
	FVector2D JoystickLimit;

	/* Radius of virtual joystick thumb widget. */
	float JoystickThumbRadius = 40.f;

	float CurveTolerance = 0.8f;

	int CurveRankThreshold = 3;

	float MinNudgeVelocity = 200.f;

//-------------Experimental-------------
	/* Enables triggering swipe while finger held, hold->swipe->hold->swipe->... Default behavior: swipe is triggered on release. */
	bool bHoldSwipeTrigger = true;
	/* Frames to trigger swipe while finger held. */
	int SwipeEventFrameCount = 3;
//-------------Experimental-------------

	/* Center of widget, set at runtime. */
	FVector2D WidgetCenter = FVector2D(0.f, 0.f);

protected:



	EInputState InputState;
	/* Modified position when moving held finger. */
	FVector2D TouchPosition;
	/* Initial position on touch start. */
	FVector2D InitialTouchPosition;
	float TouchTimeStamp;
	FVector2D SwipePosition;
	/* Position on touch end. */
	FVector2D ReleasePosition;
	/* List of positions recorded during on touch move event. */
	TArray<FVector2D> SwipePositions;
	/* Frame counter to determine touch -> hold transition. */
	int FramesSinceTouch = 0;

	FColor GhostColor = FColor::White;

	float t = 0.f;

	float SwipeTimeStamp;

	float TouchStartTimestamp;

	float SwipeStartTimestamp;

	FVector2D JoystickPosition;

	FVector2D AxisOutput;

	bool bInputAccept = true;

	bool bTapGate;

private:


};
