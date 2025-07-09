
#include "Input/InputPanel2D.h"
#include "Slate/DeferredCleanupSlateBrush.h"
#define LOCTEXT_NAMESPACE "UMG"

UInputPanel2D::UInputPanel2D(const FObjectInitializer& ObjectInitializer): Super(ObjectInitializer)
{
}

void UInputPanel2D::ReleaseSlateResources(bool bReleaseChildren)
{
Super::ReleaseSlateResources(bReleaseChildren);

   InputPanelRef.Reset();
    
}

void UInputPanel2D::UpdateParameters()
{
    InputPanelRef->GhostMat = &GhostMat;
    InputPanelRef->HoldFrames = HoldFrames;
    InputPanelRef->DeadZone = FVector2D(DeadZone, DeadZone);
    InputPanelRef->ShortSwipeThreshold = ShortSwipeDistance;
    InputPanelRef->LongSwipeThreshold = LongSwipeDistance;
    InputPanelRef->bHelpersEnabled = bHelpersEnabled;
    InputPanelRef->bUseDiagonalSwipeInputs = bUseDiagonalSwipeInputs;
    InputPanelRef->MinSwipeVelocity = MinSwipeVelocity;
    InputPanelRef->SwipeTimeOut = SwipeTimeOut;
	InputPanelRef->JoystickLimit = JoystickLimit;
    InputPanelRef->JoystickThumbRadius = JoystickThumbSize;
    InputPanelRef->bHoldSwipeTrigger = bHoldSwipeTrigger;
    InputPanelRef->SwipeEventFrameCount = HoldSwipeEventFrames;
    InputPanelRef->CurveTolerance = CurveTolerance;
    InputPanelRef->CurveRankThreshold = CurveRankThreshold;
    InputPanelRef->MinNudgeVelocity = MinNudgeVelocity;
}

TSharedRef<SWidget> UInputPanel2D::RebuildWidget()
{

    InputPanelRef = SNew(SInputPanel2D);
    InputPanelRef->OwningWidget=this;

    InputPanelRef->GhostMat = &GhostMat;
    InputPanelRef->HoldFrames = HoldFrames;
    InputPanelRef->DeadZone = FVector2D(DeadZone, DeadZone);
    InputPanelRef->ShortSwipeThreshold = ShortSwipeDistance;
    InputPanelRef->LongSwipeThreshold = LongSwipeDistance;
    InputPanelRef->bHelpersEnabled = bHelpersEnabled;
    InputPanelRef->bUseDiagonalSwipeInputs = bUseDiagonalSwipeInputs;
    InputPanelRef->MinSwipeVelocity = MinSwipeVelocity;
    InputPanelRef->SwipeTimeOut = SwipeTimeOut;
	InputPanelRef->JoystickLimit = JoystickLimit;
    InputPanelRef->JoystickThumbRadius = JoystickThumbSize;
    InputPanelRef->bHoldSwipeTrigger = bHoldSwipeTrigger;
    InputPanelRef->SwipeEventFrameCount = HoldSwipeEventFrames;
    InputPanelRef->CurveTolerance = CurveTolerance;
    InputPanelRef->CurveRankThreshold = CurveRankThreshold;
    InputPanelRef->MinNudgeVelocity = MinNudgeVelocity;
    InputPanelRef->AutoTapFrames = AutoTriggerTapFrames;

    return InputPanelRef.ToSharedRef();
}

#if WITH_EDITOR
const FText UInputPanel2D::GetPaletteCategory()
{
    return LOCTEXT("Mobile", "Mobile");
}
#endif 
