// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EOHPhysicsEnums.generated.h"

UENUM(BlueprintType)
enum class EOHImpulseMode : uint8 {
    Directional UMETA(DisplayName = "Directional"),    // Applies impulse in hit direction
    Radial UMETA(DisplayName = "Radial"),              // Impulse radiates from a center point
    VerticalOnly UMETA(DisplayName = "Vertical Only"), // Ignores lateral, uses upward force
    Custom UMETA(DisplayName = "Custom"),              // User-defined application logic
    Auto UMETA(DisplayName = "Auto Infer")             // Let system infer best mode from graph or hit
};

UENUM(BlueprintType)
enum class EOHMutationResult : uint8 {
    SuccessDirect UMETA(DisplayName = "Success (Direct)"),
    SuccessFallback UMETA(DisplayName = "Success (Fallback)"),
    Failure UMETA(DisplayName = "Failure"),
    NoChange UMETA(DisplayName = "No Change")
};

UENUM(BlueprintType)
enum class EPhysicsGraphOverlayMode : uint8 {
    None UMETA(DisplayName = "None"),
    Full UMETA(DisplayName = "Full Graph"),
    Anomalies UMETA(DisplayName = "Anomalies Only"),
    Both UMETA(DisplayName = "Full + Anomalies")
};

UENUM(BlueprintType)
enum class EOHNameMatchingStrategy : uint8 {
    Exact,
    Prefix,
    Suffix,
    Contains,
    Levenshtein,
    DamerauLevenshtein,
    Bigram,
    Jaccard,
    JaroWinkler,
    CosineTFIDF,
    Soundex,
    Metaphone

};

UENUM(BlueprintType)
enum class EOHPhysicsProfile : uint8 { Light, Medium, Heavy };

UENUM()
enum class EValidationStrictness : uint8 { CriticalOnly, Standard, Strict };

UENUM(BlueprintType)
enum class EOHResolutionContext : uint8 { Trackable, Simulatable };

UENUM(BlueprintType)
enum class EOHBlendEasingType : uint8 {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
};

// Old
UENUM(BlueprintType)
enum class EOHBoneSimulationRole : uint8 {
    AnimationOnly UMETA(DisplayName = "Animation Only"),
    Simulated UMETA(DisplayName = "Simulated"),
    AttachedToSimulated UMETA(DisplayName = "Attached to Simulated BoneEnum")
};

// Enum to track bone simulation state
UENUM(BlueprintType)
enum class EOHBoneSimulationState : uint8 {
    Kinematic UMETA(DisplayName = "Kinematic"),  // Fully animation-driven
    Blending UMETA(DisplayName = "Blending"),    // Partially simulating (during fade)
    Simulating UMETA(DisplayName = "Simulating") // Fully physics-driven
};

// Enum to track the type of blend operation
UENUM(BlueprintType)
enum class EOHBlendType : uint8 {
    None UMETA(DisplayName = "None"),      // No Blending
    FadeIn UMETA(DisplayName = "FadeIn"),  // Blending from animation to physics
    FadeOut UMETA(DisplayName = "FadeOut") // Blending from physics to animation
};

// New
UENUM(BlueprintType)
enum class EOHBlendState : uint8 {
    None UMETA(DisplayName = "None"),          // No Blending
    Delayed UMETA(DisplayName = "Delayed"),    // Waiting to Blend
    Blending UMETA(DisplayName = "Blending"),  // Blending between animation and physics
    Completed UMETA(DisplayName = "Completed") // Blend completed
};

UENUM(BlueprintType)
enum class EOHBlendDirection : uint8 {
    None UMETA(DisplayName = "None"),         // No Blend Direction
    Forward UMETA(DisplayName = "Forward"),   // Blend Forward
    Backward UMETA(DisplayName = "Backward"), // Blend Backward
};

UENUM(BlueprintType)
enum class EOHDriveType : uint8 {
    Kinematic UMETA(DisplayName = "Kinematic"),                       // Driven Entirely By Animation Transform
    Simulated UMETA(DisplayName = "Simulated"),                       // Driven Entirely By Physics Transform
    AttachedToSimulated UMETA(DisplayName = "Attached To Simulated"), // Driven By Parent Physics Transform
};

UENUM(BlueprintType)
enum class EOHForceApplicationSource : uint8 {
    None UMETA(DisplayName = "None"),                            // No Force Application
    PhysicalAnimation UMETA(DisplayName = "Physical Animation"), // Force Applied By Physical Animation Component
    Chaos UMETA(DisplayName = "Chaos"),                          // Force Applied by world chaos simulation
    Custom UMETA(DisplayName = "Custom")                         // Force Applied by custom logic
};

UENUM(BlueprintType)
enum class FollowMode : uint8 { None, Self, Target, Proxy, Custom };

UENUM(BlueprintType)
enum class EOHMotionCorrectionSource : uint8 {
    None UMETA(DisplayName = "None"),
    Pelvis UMETA(DisplayName = "Pelvis"),
    Root UMETA(DisplayName = "Root"),
    PelvisAndRoot UMETA(DisplayName = "Pelvis and Root"),
    Parent UMETA(DisplayName = "Parent BoneEnum"),
    Child UMETA(DisplayName = "First Child BoneEnum"),
    Custom UMETA(DisplayName = "Custom")
};

UENUM(BlueprintType)
enum class EOHFunctionalBoneGroup : uint8 {
    None UMETA(DisplayName = "None"),

    //  Functional groups for gameplay systems
    Cranial UMETA(DisplayName = "Cranial"), // Head + neck
    Core UMETA(DisplayName = "Core"),       // Pelvis + lower spine

    //  Arm-related groups
    Arms UMETA(DisplayName = "All Arms"),   // Both arms
    Hands UMETA(DisplayName = "All Hands"), // Both hands
    LeftArm UMETA(DisplayName = "Left Arm"),
    RightArm UMETA(DisplayName = "Right Arm"),
    LeftHand UMETA(DisplayName = "Left Hand"),
    RightHand UMETA(DisplayName = "Right Hand"),

    //  Leg-related groups
    Legs UMETA(DisplayName = "All Legs"), // Both legs
    Feet UMETA(DisplayName = "All Feet"), // Both feet
    LeftLeg UMETA(DisplayName = "Left Leg"),
    RightLeg UMETA(DisplayName = "Right Leg"),
    LeftFoot UMETA(DisplayName = "Left Foot"),
    RightFoot UMETA(DisplayName = "Right Foot"),

    //  Full body composites
    UpperBody UMETA(DisplayName = "Upper Body"),   // Arms, chest, head
    LowerBody UMETA(DisplayName = "Lower Body"),   // Legs, pelvis
    LeftLimbs UMETA(DisplayName = "Left Limbs"),   // Arm_L + Leg_L
    RightLimbs UMETA(DisplayName = "Right Limbs"), // Arm_R + Leg_R
    FullBody UMETA(DisplayName = "Full Body")
};

UENUM(BlueprintType)
enum class EOHBodyZone : uint8 {
    None UMETA(DisplayName = "None"),

    // Core
    Torso_Upper UMETA(DisplayName = "Torso - Upper"), // spine_02, spine_03
    Torso_Lower UMETA(DisplayName = "Torso - Lower"), // pelvis, spine_01
    Cranial UMETA(DisplayName = "Cranial"),           // neck_01, head

    // Left Arm
    Arm_L_Upper UMETA(DisplayName = "Upper Arm - Left"), // clavicle_l, upperarm_l
    Arm_L_Lower UMETA(DisplayName = "Lower Arm - Left"), // lowerarm_l
    Hand_L UMETA(DisplayName = "Hand - Left"),           // hand_l + fingers

    // Right Arm
    Arm_R_Upper UMETA(DisplayName = "Upper Arm - Right"), // clavicle_r, upperarm_r
    Arm_R_Lower UMETA(DisplayName = "Lower Arm - Right"), // lowerarm_r
    Hand_R UMETA(DisplayName = "Hand - Right"),           // hand_r + fingers

    // Left Leg
    Leg_L_Upper UMETA(DisplayName = "Thigh - Left"), // thigh_l
    Leg_L_Lower UMETA(DisplayName = "Calf - Left"),  // calf_l
    Foot_L UMETA(DisplayName = "Foot - Left"),       // foot_l, ball_l

    // Right Leg
    Leg_R_Upper UMETA(DisplayName = "Thigh - Right"), // thigh_r
    Leg_R_Lower UMETA(DisplayName = "Calf - Right"),  // calf_r
    Foot_R UMETA(DisplayName = "Foot - Right")        // foot_r, ball_r
};

// Fine-grained body parts (specific anatomical areas)
UENUM(BlueprintType)
enum class EOHBodyPart : uint8 {
    None UMETA(DisplayName = "None"),
    Pelvis UMETA(DisplayName = "Pelvis"),
    // Upper Body
    Torso UMETA(DisplayName = "Torso"),
    Head UMETA(DisplayName = "Head"),
    Arm_Left UMETA(DisplayName = "Left Arm"),
    Arm_Right UMETA(DisplayName = "Right Arm"),

    // Lower Body
    Leg_Left UMETA(DisplayName = "Left Leg"),
    Leg_Right UMETA(DisplayName = "Right Leg"),
};

// Coarse-grained body regions (broad control zones)
UENUM(BlueprintType)
enum class EOHBodyRegion : uint8 {
    None UMETA(DisplayName = "None"),
    UpperBody UMETA(DisplayName = "Upper Body"),
    LowerBody UMETA(DisplayName = "Lower Body"),
};

UENUM(BlueprintType,
      meta = (HiddenItems =
                  "None,EndOfSpine,EndOfHead,EndOfArm_L,EndOfHand_L,FirstFinger_L,LastFinger_L,EndOfArm_R,EndOfHand_R,"
                  "FirstFinger_R,LastFinger_R,EndOfLeg_L,EndOfLeg_R,FirstIKBone,LastIKBone,FirstBone,LastBone"))
enum class EOHSkeletalBone : uint8 {
    // Optional placeholder
    None UMETA(Hidden),
    // ----------------------- Root -----------------------
    Root UMETA(DisplayName = "root"),

    // ----------------------- Spine & Core -----------------------
    Pelvis UMETA(DisplayName = "pelvis"),
    Spine_01 UMETA(DisplayName = "spine_01"),
    Spine_02 UMETA(DisplayName = "spine_02"),
    Spine_03 UMETA(DisplayName = "spine_03"),

    EndOfSpine = Spine_03,

    // ----------------------- Head -----------------------
    Neck_01 UMETA(DisplayName = "neck_01"),
    Head UMETA(DisplayName = "head"),

    EndOfHead = Head UMETA(Hidden),

    // ----------------------- Left Arm -----------------------
    Clavicle_L UMETA(DisplayName = "clavicle_l"),
    UpperArm_L UMETA(DisplayName = "upperarm_l"),
    LowerArm_L UMETA(DisplayName = "lowerarm_l"),
    Hand_L UMETA(DisplayName = "hand_l"),

    Thumb_01_L UMETA(DisplayName = "thumb_01_l"),
    Thumb_02_L UMETA(DisplayName = "thumb_02_l"),
    Thumb_03_L UMETA(DisplayName = "thumb_03_l"),

    Index_01_L UMETA(DisplayName = "index_01_l"),
    Index_02_L UMETA(DisplayName = "index_02_l"),
    Index_03_L UMETA(DisplayName = "index_03_l"),

    Middle_01_L UMETA(DisplayName = "middle_01_l"),
    Middle_02_L UMETA(DisplayName = "middle_02_l"),
    Middle_03_L UMETA(DisplayName = "middle_03_l"),

    Ring_01_L UMETA(DisplayName = "ring_01_l"),
    Ring_02_L UMETA(DisplayName = "ring_02_l"),
    Ring_03_L UMETA(DisplayName = "ring_03_l"),

    Pinky_01_L UMETA(DisplayName = "pinky_01_l"),
    Pinky_02_L UMETA(DisplayName = "pinky_02_l"),
    Pinky_03_L UMETA(DisplayName = "pinky_03_l"),

    EndOfArm_L = Hand_L UMETA(Hidden),
    EndOfHand_L = Pinky_03_L UMETA(Hidden),
    FirstFinger_L = Thumb_01_L UMETA(Hidden),
    LastFinger_L = Pinky_03_L UMETA(Hidden),

    // ----------------------- Right Arm -----------------------
    Clavicle_R UMETA(DisplayName = "clavicle_r"),
    UpperArm_R UMETA(DisplayName = "upperarm_r"),
    LowerArm_R UMETA(DisplayName = "lowerarm_r"),
    Hand_R UMETA(DisplayName = "hand_r"),

    Thumb_01_R UMETA(DisplayName = "thumb_01_r"),
    Thumb_02_R UMETA(DisplayName = "thumb_02_r"),
    Thumb_03_R UMETA(DisplayName = "thumb_03_r"),

    Index_01_R UMETA(DisplayName = "index_01_r"),
    Index_02_R UMETA(DisplayName = "index_02_r"),
    Index_03_R UMETA(DisplayName = "index_03_r"),

    Middle_01_R UMETA(DisplayName = "middle_01_r"),
    Middle_02_R UMETA(DisplayName = "middle_02_r"),
    Middle_03_R UMETA(DisplayName = "middle_03_r"),

    Ring_01_R UMETA(DisplayName = "ring_01_r"),
    Ring_02_R UMETA(DisplayName = "ring_02_r"),
    Ring_03_R UMETA(DisplayName = "ring_03_r"),

    Pinky_01_R UMETA(DisplayName = "pinky_01_r"),
    Pinky_02_R UMETA(DisplayName = "pinky_02_r"),
    Pinky_03_R UMETA(DisplayName = "pinky_03_r"),

    EndOfArm_R = Hand_R UMETA(Hidden),
    EndOfHand_R = Pinky_03_R UMETA(Hidden),
    FirstFinger_R = Thumb_01_R UMETA(Hidden),
    LastFinger_R = Pinky_03_R UMETA(Hidden),

    // ----------------------- Left Leg -----------------------
    Thigh_L UMETA(DisplayName = "thigh_l"),
    Calf_L UMETA(DisplayName = "calf_l"),
    Foot_L UMETA(DisplayName = "foot_l"),
    Ball_L UMETA(DisplayName = "ball_l"),

    EndOfLeg_L = Ball_L UMETA(Hidden),

    // ----------------------- Right Leg -----------------------
    Thigh_R UMETA(DisplayName = "thigh_r"),
    Calf_R UMETA(DisplayName = "calf_r"),
    Foot_R UMETA(DisplayName = "foot_r"),
    Ball_R UMETA(DisplayName = "ball_r"),

    EndOfLeg_R = Ball_R UMETA(Hidden),

    // ----------------------- IK Bones -----------------------
    IK_Root UMETA(DisplayName = "ik_root"),
    IK_Hand_L UMETA(DisplayName = "ik_hand_l"),
    IK_Hand_R UMETA(DisplayName = "ik_hand_r"),
    IK_Foot_L UMETA(DisplayName = "ik_foot_l"),
    IK_Foot_R UMETA(DisplayName = "ik_foot_r"),
    IK_Hand_Gun UMETA(DisplayName = "ik_hand_gun"),
    IK_Hand_Root UMETA(DisplayName = "ik_hand_root"),
    IK_Foot_Root UMETA(DisplayName = "ik_foot_root"),

    FirstIKBone = IK_Root UMETA(Hidden),
    LastIKBone = IK_Foot_Root UMETA(Hidden),

    // ----------------------- Enum Range Support -----------------------
    FirstBone = Root UMETA(Hidden),
    LastBone = IK_Foot_Root UMETA(Hidden),
};

ENUM_RANGE_BY_FIRST_AND_LAST(EOHSkeletalBone, EOHSkeletalBone::FirstBone, EOHSkeletalBone::LastBone);

// ----------------------- Simple BoneEnum Classification Helpers -----------------------

/** Check if a bone is in a specific inclusive range */
FORCEINLINE bool IsInBoneRange(const EOHSkeletalBone Bone, const EOHSkeletalBone First, const EOHSkeletalBone Last) {
    return (Bone >= First && Bone <= Last);
}

/** Check if a bone is a core/spine bone */
FORCEINLINE bool IsSpineBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Spine_01, EOHSkeletalBone::EndOfSpine);
}

/** Check if a bone is a head bone */
FORCEINLINE bool IsHeadBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Neck_01, EOHSkeletalBone::EndOfHead);
}

/** Check if a bone is part of the left arm (excluding fingers) */
FORCEINLINE bool IsLeftArmBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::EndOfArm_L);
}

/** Check if a bone is part of the right arm (excluding fingers) */
FORCEINLINE bool IsRightArmBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::EndOfArm_R);
}

/** Check if a bone is part of either arm (excluding fingers) */
FORCEINLINE bool IsArmBone(const EOHSkeletalBone Bone) {
    return IsLeftArmBone(Bone) || IsRightArmBone(Bone);
}

/** Check if a bone is a left finger bone */
FORCEINLINE bool IsLeftFingerBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::FirstFinger_L, EOHSkeletalBone::LastFinger_L);
}

/** Check if a bone is a right finger bone */
FORCEINLINE bool IsRightFingerBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::FirstFinger_R, EOHSkeletalBone::LastFinger_R);
}

/** Check if a bone is a finger bone on either hand */
FORCEINLINE bool IsFingerBone(const EOHSkeletalBone Bone) {
    return IsLeftFingerBone(Bone) || IsRightFingerBone(Bone);
}

/** Check if a bone is part of the left leg */
FORCEINLINE bool IsLeftLegBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Thigh_L, EOHSkeletalBone::EndOfLeg_L);
}

/** Check if a bone is part of the right leg */
FORCEINLINE bool IsRightLegBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::Thigh_R, EOHSkeletalBone::EndOfLeg_R);
}

/** Check if a bone is part of either leg */
FORCEINLINE bool IsLegBone(const EOHSkeletalBone Bone) {
    return IsLeftLegBone(Bone) || IsRightLegBone(Bone);
}

/** Check if a bone is an IK bone */
FORCEINLINE bool IsIKBone(const EOHSkeletalBone Bone) {
    return IsInBoneRange(Bone, EOHSkeletalBone::FirstIKBone, EOHSkeletalBone::LastIKBone);
}

/** Check if a bone is on the left side of the body */
FORCEINLINE bool IsLeftSideBone(const EOHSkeletalBone Bone) {
    return IsLeftArmBone(Bone) || IsLeftFingerBone(Bone) || IsLeftLegBone(Bone) || Bone == EOHSkeletalBone::IK_Hand_L ||
           Bone == EOHSkeletalBone::IK_Foot_L;
}

/** Check if a bone is on the right side of the body */
FORCEINLINE bool IsRightSideBone(const EOHSkeletalBone Bone) {
    return IsRightArmBone(Bone) || IsRightFingerBone(Bone) || IsRightLegBone(Bone) ||
           Bone == EOHSkeletalBone::IK_Hand_R || Bone == EOHSkeletalBone::IK_Foot_R;
}

FORCEINLINE FName GetBoneNameFromEnum(EOHSkeletalBone Bone) {
    const UEnum* EnumPtr = StaticEnum<EOHSkeletalBone>();
    return EnumPtr ? FName(EnumPtr->GetNameStringByValue(static_cast<int64>(Bone)).ToLower()) : NAME_None;
}

FORCEINLINE EOHSkeletalBone GetEnumFromBoneName(FName BoneName) {
    const UEnum* EnumPtr = StaticEnum<EOHSkeletalBone>();
    if (!EnumPtr) {
        return EOHSkeletalBone::None;
    }

    const FString BoneStr = BoneName.ToString();
    for (int32 i = 0; i < EnumPtr->NumEnums(); ++i) {
        if (EnumPtr->GetNameStringByIndex(i).Equals(BoneStr, ESearchCase::IgnoreCase)) {
            return static_cast<EOHSkeletalBone>(EnumPtr->GetValueByIndex(i));
        }
    }
    return EOHSkeletalBone::None;
}

// Map bone ➔ body part
FORCEINLINE EOHBodyPart GetBodyPartForBone(const EOHSkeletalBone Bone) {
    if (IsHeadBone(Bone)) {
        return EOHBodyPart::Head;
    }
    if (IsSpineBone(Bone)) {
        return EOHBodyPart::Torso;
    }
    if (Bone == EOHSkeletalBone::Pelvis) {
        return EOHBodyPart::Pelvis;
    }
    if (IsLeftArmBone(Bone) || IsLeftFingerBone(Bone)) {
        return EOHBodyPart::Arm_Left;
    }
    if (IsRightArmBone(Bone) || IsRightFingerBone(Bone)) {
        return EOHBodyPart::Arm_Right;
    }
    if (IsLeftLegBone(Bone)) {
        return EOHBodyPart::Leg_Left;
    }
    if (IsRightLegBone(Bone)) {
        return EOHBodyPart::Leg_Right;
    }
    return EOHBodyPart::None;
}

// Map body part ➔ body region
FORCEINLINE EOHBodyRegion GetBodyRegionForPart(const EOHBodyPart Part) {
    switch (Part) {
    case EOHBodyPart::Head:
    case EOHBodyPart::Torso:
    case EOHBodyPart::Arm_Left:
    case EOHBodyPart::Arm_Right:
        return EOHBodyRegion::UpperBody;

    case EOHBodyPart::Leg_Left:
    case EOHBodyPart::Leg_Right:
    case EOHBodyPart::Pelvis:
        return EOHBodyRegion::LowerBody;

    default:
        return EOHBodyRegion::None;
    }
}

/** Map a BoneEnum directly to a Body Region */
FORCEINLINE EOHBodyRegion GetBodyRegionForBone(const EOHSkeletalBone Bone) {
    return GetBodyRegionForPart(GetBodyPartForBone(Bone));
}

/** Get all bones belonging to a BodyPart */
FORCEINLINE TArray<EOHSkeletalBone> GetBonesInBodyPart(const EOHBodyPart BodyPart) {
    TArray<EOHSkeletalBone> Bones;
    for (uint8 BoneIndex = static_cast<uint8>(EOHSkeletalBone::FirstBone);
         BoneIndex <= static_cast<uint8>(EOHSkeletalBone::LastBone); ++BoneIndex) {
        const EOHSkeletalBone Bone = static_cast<EOHSkeletalBone>(BoneIndex);
        if (GetBodyPartForBone(Bone) == BodyPart) {
            Bones.Add(Bone);
        }
    }
    return Bones;
}

/** Get all bones belonging to a BodyRegion */
FORCEINLINE TArray<EOHSkeletalBone> GetBonesInBodyRegion(const EOHBodyRegion Region) {
    TArray<EOHSkeletalBone> Bones;
    for (uint8 BoneIndex = static_cast<uint8>(EOHSkeletalBone::FirstBone);
         BoneIndex <= static_cast<uint8>(EOHSkeletalBone::LastBone); ++BoneIndex) {
        const EOHSkeletalBone Bone = static_cast<EOHSkeletalBone>(BoneIndex);
        if (GetBodyRegionForBone(Bone) == Region) {
            Bones.Add(Bone);
        }
    }
    return Bones;
}

/** Get all BodyParts belonging to a BodyRegion */
FORCEINLINE TArray<EOHBodyPart> GetBodyPartsInBodyRegion(const EOHBodyRegion Region) {
    TArray<EOHBodyPart> Parts;
    for (uint8 PartIndex = static_cast<uint8>(EOHBodyPart::Head);
         PartIndex <= static_cast<uint8>(EOHBodyPart::Leg_Right); ++PartIndex) {
        const EOHBodyPart Part = static_cast<EOHBodyPart>(PartIndex);
        if (GetBodyRegionForPart(Part) == Region) {
            Parts.Add(Part);
        }
    }
    return Parts;
}

/** Internal cached mappings */
namespace OHCachedMappings {
static TMap<EOHSkeletalBone, EOHBodyPart> BoneToPartMap;
static TMap<EOHSkeletalBone, EOHBodyRegion> BoneToRegionMap;
static TMap<EOHBodyPart, EOHBodyRegion> PartToRegionMap;
static TMap<EOHBodyRegion, TArray<EOHBodyPart>> RegionToPartsMap;
static TMap<EOHBodyPart, TArray<EOHSkeletalBone>> PartToBonesMap;
static TMap<EOHBodyRegion, TArray<EOHSkeletalBone>> RegionToBonesMap;

FORCEINLINE void BuildMappings() {
    BoneToPartMap.Empty();
    BoneToRegionMap.Empty();
    PartToRegionMap.Empty();
    RegionToPartsMap.Empty();
    PartToBonesMap.Empty();
    RegionToBonesMap.Empty();

    // Fill Part ➔ Region mapping
    for (uint8 PartIndex = static_cast<uint8>(EOHBodyPart::Head);
         PartIndex <= static_cast<uint8>(EOHBodyPart::Leg_Right); ++PartIndex) {
        const EOHBodyPart Part = static_cast<EOHBodyPart>(PartIndex);
        const EOHBodyRegion Region = GetBodyRegionForPart(Part);
        PartToRegionMap.Add(Part, Region);

        RegionToPartsMap.FindOrAdd(Region).Add(Part);
    }

    // Fill BoneEnum ➔ Part ➔ Region mapping
    for (uint8 BoneIndex = static_cast<uint8>(EOHSkeletalBone::FirstBone);
         BoneIndex <= static_cast<uint8>(EOHSkeletalBone::LastBone); ++BoneIndex) {
        const EOHSkeletalBone Bone = static_cast<EOHSkeletalBone>(BoneIndex);
        const EOHBodyPart Part = GetBodyPartForBone(Bone);
        const EOHBodyRegion Region = GetBodyRegionForPart(Part);

        BoneToPartMap.Add(Bone, Part);
        BoneToRegionMap.Add(Bone, Region);
        PartToBonesMap.FindOrAdd(Part).Add(Bone);
        RegionToBonesMap.FindOrAdd(Region).Add(Bone);
    }
}
} // namespace OHCachedMappings

UENUM(BlueprintType)
enum class EOHRelativeDirection : uint8 { Forward, Backward, Left, Right, None };

UENUM(BlueprintType)
enum class EOHHitDirection : uint8 {
    None UMETA(DisplayName = "None"),
    Front UMETA(DisplayName = "Front"),
    FrontRight UMETA(DisplayName = "Front Right"),
    Right UMETA(DisplayName = "Right"),
    BackRight UMETA(DisplayName = "Back Right"),
    Back UMETA(DisplayName = "Back"),
    BackLeft UMETA(DisplayName = "Back Left"),
    Left UMETA(DisplayName = "Left"),
    FrontLeft UMETA(DisplayName = "Front Left")
};

UENUM()
enum class EOHBlendPhase : uint8 {
    BlendIn,
    Hold,
    BlendOut,
};

UENUM(BlueprintType)
enum class EImpulseDirectionMode : uint8 {
    FromHitNormal UMETA(DisplayName = "From Hit Normal (Default)"),
    FromBoneToImpactPoint UMETA(DisplayName = "From Bone to Impact Point"),
    FromBoneVelocity UMETA(DisplayName = "From Bone Velocity Direction")
};

UENUM(BlueprintType)
enum class EOHMotionTimeDomain : uint8 {
    GameWorldTime UMETA(DisplayName = "World Time"),
    AnimationTime UMETA(DisplayName = "Animation Time"),
    PhysicsTime UMETA(DisplayName = "Physics Time")
};

UENUM(BlueprintType)
enum class EDriveAccessMode : uint8 { Cached UMETA(DisplayName = "Cached"), Live UMETA(DisplayName = "Live") };