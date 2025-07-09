
#include "Debug/RhythmDebug.h"
#include "Slate/DeferredCleanupSlateBrush.h"
#define LOCTEXT_NAMESPACE "UMG"

URhythmDebug::URhythmDebug(const FObjectInitializer& ObjectInitializer): Super(ObjectInitializer)
{
}

void URhythmDebug::ReleaseSlateResources(bool bReleaseChildren)
{
Super::ReleaseSlateResources(bReleaseChildren);

   RhythmSlateRef.Reset();

}

void URhythmDebug::UpdateParameters()
{
}

void URhythmDebug::SetInputWindows(TArray<FInputWindow> _InputWindows)
{
    this->InputWindows = _InputWindows;
    RhythmSlateRef->InputWindows = InputWindows;

}

TSharedRef<SWidget> URhythmDebug::RebuildWidget()
{

    RhythmSlateRef = SNew(SRhythmDebug);
    RhythmSlateRef->OwningWidget=this;
    RhythmSlateRef->InputWindows = this->InputWindows;
    RhythmSlateRef->TimeScale = this->TimeScale;

    return RhythmSlateRef.ToSharedRef();
}

#if WITH_EDITOR
const FText URhythmDebug::GetPaletteCategory()
{
    return LOCTEXT("Mobile", "Mobile");
}
#endif 
