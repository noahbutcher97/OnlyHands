#pragma once

#include "Components/ContentWidget.h"
#include "Templates/SharedPointer.h"
#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include "Engine/Texture2D.h"

#include "SInputPanel2D.h"

#include "InputPanel2D.generated.h"


UENUM(BlueprintType)
enum EInputType
{
	None,
	Tap,
	ShortSwipe,
	LongSwipe
};

static ESwipeDirection AngleToDiagonalSwipeDirection(float Angle)
{
	// Normalize the angle to the range [0, 2*PI]
	while (Angle < 0.0f)
	{
		Angle += 2 * PI;
	}

	while (Angle >= 2 * PI)
	{
		Angle -= 2 * PI;
	}

	// Determine the swipe direction based on angle ranges
	if (Angle >= 0.0f && Angle < PI / 2)
	{
		return ESwipeDirection::DownRight;
	}
	else if (Angle >= PI / 2 && Angle < PI)
	{
		return ESwipeDirection::DownLeft;
	}
	else if (Angle >= PI && Angle < 3 * PI / 2)
	{
		return ESwipeDirection::UpLeft;
	}
	else
	{
		return ESwipeDirection::UpRight;
	}
}
static ESwipeDirection AngleToSwipeDirection(float Angle)
{
	// Normalize the angle to the range [0, 2*PI]
	while (Angle < 0.0f)
	{
		Angle += 2 * PI;
	}

	while (Angle >= 2 * PI)
	{
		Angle -= 2 * PI;
	}

	// Determine the swipe direction based on angle ranges
	if (Angle >= 0.0f && Angle < PI / 4 || (Angle >= 7 * PI / 4 && Angle < 2 * PI))
	{
		return ESwipeDirection::Right;
	}
//	else if (Angle >= PI / 8 && Angle < 3 * PI / 8)
//	{
//		return ESwipeDirection::DownRight;
//	}
	else if (Angle >= 5 * PI / 4 && Angle < 7 * PI / 4)
	{
		return ESwipeDirection::Up;
	}
//	else if (Angle >= 5 * PI / 8 && Angle < 7 * PI / 8)
//	{
//		return ESwipeDirection::DownLeft;
//	}
	else if (Angle >= 3 * PI / 4 && Angle < 5 * PI / 4)
	{
		return ESwipeDirection::Left;
	}
//	else if (Angle >= 9 * PI / 8 && Angle < 11 * PI / 8)
//	{
//		return ESwipeDirection::UpLeft;
//	}
//	else if (Angle >= PI / 4 && Angle < 3 * PI / 4)
//	{
//		return ESwipeDirection::Up;
//	}
	else
	{
		return ESwipeDirection::Down;
	}
//	else
//	{
//		return ESwipeDirection::UpRight;
//	}
}

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnStartSwipe, ESwipeDirection, SwipeDirection, EInputType, InputType, FVector2D, SwipeVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnDuringSwipe, ESwipeDirection, SwipeDirection, EInputType, InputType, FVector2D, SwipeVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnCommitSwipe, ESwipeDirection, SwipeDirection, EInputType, InputType);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnCancelSwipe);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnTap, ESwipeDirection, SwipeDirection);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnTouch, ESwipeDirection, SwipeDirection);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMoveTouch, FVector2D, Output);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRawInputVector, FVector2D, RawInputVector);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnInputStateChange, EInputState, InputState);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnSwipeVelocity, EInputState, InputState);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnNudgeTouch);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnPauseCommand);
//	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnJoystickfloatValueChange,float,Output);
//	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnJoystickboolChange,bool,Output,bool,IsDoubleTap);
UCLASS()
class ONLYHANDS_API UInputPanel2D : public UWidget
{
	GENERATED_UCLASS_BODY()
public:

	UPROPERTY(BlueprintAssignable)
	FOnStartSwipe OnStartSwipe;

	UPROPERTY(BlueprintAssignable)
	FOnDuringSwipe OnDuringSwipe;

	UPROPERTY(BlueprintAssignable)
	FOnCommitSwipe OnCommitSwipe;

	UPROPERTY(BlueprintAssignable)
	FOnCancelSwipe OnCancelSwipe;

	UPROPERTY(BlueprintAssignable)
	FOnTap OnTap;

	UPROPERTY(BlueprintAssignable)
	FOnInputStateChange OnInputStateChange;

	UPROPERTY(BlueprintAssignable)
	FOnMoveTouch OnInputChange;

	UPROPERTY(BlueprintAssignable)
	FOnNudgeTouch OnNudge;

	UPROPERTY(BlueprintAssignable)
	FOnTouch OnTouch;

	UPROPERTY(BlueprintAssignable)
	FOnPauseCommand OnPause;

	UPROPERTY(BlueprintAssignable)
	FOnRawInputVector OnRawInputVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Helper gizmos for swipes. "))
	bool bHelpersEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FSlateBrush GhostMat;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FSlateBrush JoystickImage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FSlateBrush JoystickBoundsImage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FSlateBrush QuadrantGhost;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="If >= 0: sends tap event after this many frames while touch is held."))
	int AutoTriggerTapFrames = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Number of frames delay for start hold. Also max number of frames for tap."))
	int HoldFrames = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Min swipe velocity to start registering swipes. "))
	float MinSwipeVelocity = 1000.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Min swipe distance to start registering swipes. "))
	float DeadZone = 50.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Swipe distance needed to trigger short swipe."))
	float ShortSwipeDistance = 150.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Swipe distance needed to trigger long swipe."))
	float LongSwipeDistance = 300.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Swipe distance needed to trigger long swipe."))
	float SwipeTimeOut = 0.016666f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Change input mode for diagonal swipes and taps. "))
	bool bUseDiagonalSwipeInputs = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Joystick bounds."))
	FVector2D JoystickLimit;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Swipe distance needed to trigger long swipe."))
	float JoystickThumbSize = 40.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Change input mode for diagonal swipes and taps. "))
	bool bHoldSwipeTrigger = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Number of frames delay for start hold. Also max number of frames for tap."))
	int HoldSwipeEventFrames = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="0.0-1.0, how hard it is to draw curve, closer to 1 means easier"))
	float CurveTolerance = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="How sharp the curve must be"))
	int CurveRankThreshold = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Input Parameters", meta=(ToolTip="Touch move velocity to trigger nudge."))
	float MinNudgeVelocity = 200.f;

	UPROPERTY(EditAnywhere, Category="Axis Events", meta=(ToolTip="Joystick horizontal axis."))
	FKey JoystickAxisX;
	UPROPERTY(EditAnywhere, Category="Axis Events", meta=(ToolTip="Joystick vertical axis."))
	FKey JoystickAxisY;

	UPROPERTY(BlueprintReadOnly)
	int InputDelay = 10000;

	TSharedPtr<SInputPanel2D> InputPanelRef;
	
	virtual TSharedRef<SWidget> RebuildWidget() override;

	virtual void ReleaseSlateResources(bool bReleaseChildren) override;

	UFUNCTION(BlueprintCallable)
	void UpdateParameters();
//
#if WITH_EDITOR
	virtual const FText GetPaletteCategory() override;

#endif
	
};
