#pragma once

#include "Interface/IOHPhysicsBehaviorStrategy.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"

/**
 * A hardcoded strategy that determines physics targets based on body part heuristics.
 */
class FOHAnatomicalHeuristicStrategy : public IOHPhysicsBehaviorStrategy
{
public:
    virtual FOHResolvedPhysicsTargets Resolve(const FBoneState& State, EOHSkeletalBone Bone) const override
    {
        FOHResolvedPhysicsTargets Out;

        const EOHBodyPart Part = UOHSkeletalPhysicsUtils::GetBodyPartForBone(Bone);

        if (Part == EOHBodyPart::Arm_Left || Part == EOHBodyPart::Arm_Right)
        {
            Out.OrientationStrength = 80.f;
            Out.PositionStrength = 60.f;
            Out.ProxyBlendAlpha = 0.5f;
            Out.TargetLinearDamping = 0.25f;
            Out.TargetAngularDamping = 0.4f;
        }
        else if (Part == EOHBodyPart::Leg_Left || Part == EOHBodyPart::Leg_Right)
        {
            Out.OrientationStrength = 70.f;
            Out.PositionStrength = 70.f;
            Out.ProxyBlendAlpha = 0.3f;
            Out.TargetLinearDamping = 0.6f;
            Out.TargetAngularDamping = 0.7f;
        }
        else
        {
            Out.OrientationStrength = 100.f;
            Out.PositionStrength = 90.f;
            Out.ProxyBlendAlpha = 0.1f;
            Out.TargetLinearDamping = 1.0f;
            Out.TargetAngularDamping = 1.2f;
        }

        return Out;
    }

    virtual FString GetStrategyName() const override { return TEXT("AnatomicalHeuristic"); }
};