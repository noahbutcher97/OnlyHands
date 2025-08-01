
#pragma once
#include "AnimationGraphSchema.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "Animation/Nodes/OHAnimNode_ArmStretchIK.h"
#include "OHAnimGraphNode_ArmStretchIK.generated.h"


UCLASS(meta = (Keywords = "IK Arm Stretch"))
class ONLYHANDSEDITOR_API UOHAnimGraphNode_ArmStretchIK : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings")
	FOHAnimNode_ArmStretchIK Node;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override{ return &Node; }

	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override
	{
		return FText::FromString(TEXT("Arm Stretch IK"));
	}

	virtual FText GetTooltipText() const override
	{
		return FText::FromString(TEXT("Custom Arm Stretch IK Node"));
	}

	virtual FString GetNodeCategory() const override
	{
		return TEXT("OnlyHands|IK");
	}

	virtual bool IsCompatibleWithGraph(const UEdGraph* TargetGraph) const override
	{
		return TargetGraph && TargetGraph->GetSchema()->IsA(UAnimationGraphSchema::StaticClass());
	}
};

