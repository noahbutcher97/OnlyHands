#pragma once
#include "AnimGraphNode_SkeletalControlBase.h"
#include "OnlyHands/Public/Animation/Nodes/AnimNode_ArmCompressionResponse.h"
#include "AnimGraphNode_ArmCompressionResponse.generated.h"

UCLASS(meta = (DisplayName = "Arm Compression Response", BlueprintInternalUseOnly = "true"))
class ONLYHANDSEDITOR_API UAnimGraphNode_ArmCompressionResponse : public UAnimGraphNode_SkeletalControlBase {
    GENERATED_BODY()

  public:
    virtual FText GetControllerDescription() const override;
    virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
    virtual FText GetTooltipText() const override;

    // Critical override that prevents crash - disables all validation
#if WITH_EDITOR
    virtual void ValidateAnimNodeDuringCompilation(USkeleton* ForSkeleton, FCompilerResultsLog& MessageLog) override;

    // Crucial overrides to prevent Slate UI from accessing validation text
    virtual bool ShowVisualWarning() const override;
    virtual FText GetVisualWarningTooltipText() const override;
#endif

  protected:
    UPROPERTY(EditAnywhere, Category = Settings)
    FAnimNode_ArmCompressionResponse Node;

    virtual const FAnimNode_SkeletalControlBase* GetNode() const override {
        return &Node;
    }
};