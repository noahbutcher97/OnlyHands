#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimNodeBase.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "OHAnimNode_ArmStretchIK.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct ONLYHANDS_API FOHAnimNode_ArmStretchIK : public FAnimNode_SkeletalControlBase {
    GENERATED_BODY()

  public:
    FOHAnimNode_ArmStretchIK();

  protected:
    // FAnimNode_SkeletalControlBase interface
    virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output,
                                                   TArray<FBoneTransform>& OutBoneTransforms) override;
    virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
    virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
};
