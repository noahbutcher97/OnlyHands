#pragma once

#include "Interface/IOHPhysicsBehaviorStrategy.h"
#include "OHPhysicsStructs.h"

/**
 * Returns a single shared physics profile for all bones.
 * Useful for testing how PAC parameters affect different body parts uniformly.
 */
class FOHDebugStrategy : public IOHPhysicsBehaviorStrategy {
  public:
    FOHResolvedPhysicsTargets DebugTargets;

    FOHDebugStrategy() {
        DebugTargets.OrientationStrength = 100.f;
        DebugTargets.PositionStrength = 75.f;
        DebugTargets.VelocityStrength = 50.f;
        DebugTargets.AngularVelocityStrength = 30.f;
        DebugTargets.MaxLinearForce = 1000.f;
        DebugTargets.MaxAngularForce = 500.f;
        DebugTargets.TargetLinearDamping = 0.5f;
        DebugTargets.TargetAngularDamping = 0.7f;
        DebugTargets.ProxyBlendAlpha = 0.35f;
    }

    virtual FOHResolvedPhysicsTargets Resolve(const FBoneState& BoneState, EOHSkeletalBone Bone) const override {
        return DebugTargets;
    }

    virtual FString GetStrategyName() const override {
        return TEXT("DebugStrategy");
    }
};
