#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "OHPhysicsStructs.h"

/**
 * Interface for defining per-bone physics behavior strategies.
 * Each implementation returns a resolved target pose/force/damping for a given bone.
 */
class IOHPhysicsBehaviorStrategy {
  public:
    virtual ~IOHPhysicsBehaviorStrategy() {}

    /**
     * Resolves a set of physics target values for a bone.
     * @param BoneState - The current procedural bone state
     * @param Bone - The skeletal bone enum
     * @return FOHResolvedPhysicsTargets - target physics values for this frame
     */
    virtual FOHResolvedPhysicsTargets Resolve(const FOHBoneState& BoneState, EOHSkeletalBone Bone) const = 0;

    /**
     * Used to identify this strategy at runtime (for logging/debug).
     */
    virtual FString GetStrategyName() const = 0;
};