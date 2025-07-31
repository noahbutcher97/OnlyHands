#include "Animation/Nodes/AnimGraphNode_ArmCompressionResponse.h"


#if WITH_EDITOR


#define LOCTEXT_NAMESPACE "ArmCompressionResponse"

FText UAnimGraphNode_ArmCompressionResponse::GetControllerDescription() const
{
    return LOCTEXT("ArmCompressionResponse", "Arm Compression Response");
}

FText UAnimGraphNode_ArmCompressionResponse::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
    return GetControllerDescription();
}


FText UAnimGraphNode_ArmCompressionResponse::GetTooltipText() const
{
    return LOCTEXT("ArmCompressionResponse_Tooltip", "Procedural arm compression IK based on velocity or target.");
}

#if WITH_EDITOR
void UAnimGraphNode_ArmCompressionResponse::ValidateAnimNodeDuringCompilation(
        USkeleton* ForSkeleton, FCompilerResultsLog& MessageLog)
{

}

bool UAnimGraphNode_ArmCompressionResponse::ShowVisualWarning() const
{
    // Never show warnings
    return false;
}

FText UAnimGraphNode_ArmCompressionResponse::GetVisualWarningTooltipText() const
{
    // Return an empty, valid FText that won't try to rebuild or cause crashes
    return FText::GetEmpty();
}
#endif

#undef LOCTEXT_NAMESPACE

#endif