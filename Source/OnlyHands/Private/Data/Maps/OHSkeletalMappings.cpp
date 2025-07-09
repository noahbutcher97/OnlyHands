
#include "Data/Maps/OHSkeletalMappings.h"
#include "FunctionLibrary/OHAlgoUtils.h"

namespace OHSkeletalMappings {
// ==========================================
// Core String Conversions
// ==========================================

// SkeletalBone → FName mapping
const TMap<EOHSkeletalBone, FName> SkeletalBoneToFNameMap = {
    {EOHSkeletalBone::None, FName("None")},
    {EOHSkeletalBone::Root, FName("root")},
    {EOHSkeletalBone::Spine_01, FName("spine_01")},
    {EOHSkeletalBone::Spine_02, FName("spine_02")},
    {EOHSkeletalBone::Spine_03, FName("spine_03")},
    {EOHSkeletalBone::Pelvis, FName("pelvis")},
    {EOHSkeletalBone::Neck_01, FName("neck_01")},
    {EOHSkeletalBone::Head, FName("head")},

    {EOHSkeletalBone::Clavicle_L, FName("clavicle_l")},
    {EOHSkeletalBone::UpperArm_L, FName("upperarm_l")},
    {EOHSkeletalBone::LowerArm_L, FName("lowerarm_l")},
    {EOHSkeletalBone::Hand_L, FName("hand_l")},

    {EOHSkeletalBone::Clavicle_R, FName("clavicle_r")},
    {EOHSkeletalBone::UpperArm_R, FName("upperarm_r")},
    {EOHSkeletalBone::LowerArm_R, FName("lowerarm_r")},
    {EOHSkeletalBone::Hand_R, FName("hand_r")},

    {EOHSkeletalBone::Thigh_L, FName("thigh_l")},
    {EOHSkeletalBone::Calf_L, FName("calf_l")},
    {EOHSkeletalBone::Foot_L, FName("foot_l")},
    {EOHSkeletalBone::Ball_L, FName("ball_l")},

    {EOHSkeletalBone::Thigh_R, FName("thigh_r")},
    {EOHSkeletalBone::Calf_R, FName("calf_r")},
    {EOHSkeletalBone::Foot_R, FName("foot_r")},
    {EOHSkeletalBone::Ball_R, FName("ball_r")},

    // Fingers - Left
    {EOHSkeletalBone::Thumb_01_L, FName("thumb_01_l")},
    {EOHSkeletalBone::Thumb_02_L, FName("thumb_02_l")},
    {EOHSkeletalBone::Thumb_03_L, FName("thumb_03_l")},
    {EOHSkeletalBone::Index_01_L, FName("index_01_l")},
    {EOHSkeletalBone::Index_02_L, FName("index_02_l")},
    {EOHSkeletalBone::Index_03_L, FName("index_03_l")},
    {EOHSkeletalBone::Middle_01_L, FName("middle_01_l")},
    {EOHSkeletalBone::Middle_02_L, FName("middle_02_l")},
    {EOHSkeletalBone::Middle_03_L, FName("middle_03_l")},
    {EOHSkeletalBone::Ring_01_L, FName("ring_01_l")},
    {EOHSkeletalBone::Ring_02_L, FName("ring_02_l")},
    {EOHSkeletalBone::Ring_03_L, FName("ring_03_l")},
    {EOHSkeletalBone::Pinky_01_L, FName("pinky_01_l")},
    {EOHSkeletalBone::Pinky_02_L, FName("pinky_02_l")},
    {EOHSkeletalBone::Pinky_03_L, FName("pinky_03_l")},

    // Fingers - Right
    {EOHSkeletalBone::Thumb_01_R, FName("thumb_01_r")},
    {EOHSkeletalBone::Thumb_02_R, FName("thumb_02_r")},
    {EOHSkeletalBone::Thumb_03_R, FName("thumb_03_r")},
    {EOHSkeletalBone::Index_01_R, FName("index_01_r")},
    {EOHSkeletalBone::Index_02_R, FName("index_02_r")},
    {EOHSkeletalBone::Index_03_R, FName("index_03_r")},
    {EOHSkeletalBone::Middle_01_R, FName("middle_01_r")},
    {EOHSkeletalBone::Middle_02_R, FName("middle_02_r")},
    {EOHSkeletalBone::Middle_03_R, FName("middle_03_r")},
    {EOHSkeletalBone::Ring_01_R, FName("ring_01_r")},
    {EOHSkeletalBone::Ring_02_R, FName("ring_02_r")},
    {EOHSkeletalBone::Ring_03_R, FName("ring_03_r")},
    {EOHSkeletalBone::Pinky_01_R, FName("pinky_01_r")},
    {EOHSkeletalBone::Pinky_02_R, FName("pinky_02_r")},
    {EOHSkeletalBone::Pinky_03_R, FName("pinky_03_r")},

    // IK Bones
    {EOHSkeletalBone::IK_Root, FName("ik_root")},
    {EOHSkeletalBone::IK_Hand_L, FName("ik_hand_l")},
    {EOHSkeletalBone::IK_Hand_R, FName("ik_hand_r")},
    {EOHSkeletalBone::IK_Foot_L, FName("ik_foot_l")},
    {EOHSkeletalBone::IK_Foot_R, FName("ik_foot_r")},
    {EOHSkeletalBone::IK_Hand_Gun, FName("ik_hand_gun")},
    {EOHSkeletalBone::IK_Hand_Root, FName("ik_hand_root")},
    {EOHSkeletalBone::IK_Foot_Root, FName("ik_foot_root")}};

// FName → SkeletalBone mapping (reverse lookup)
const TMap<FName, EOHSkeletalBone> FNameToSkeletalBoneMap = {
    {FName("None"), EOHSkeletalBone::None},
    {FName("root"), EOHSkeletalBone::Root},
    {FName("spine_01"), EOHSkeletalBone::Spine_01},
    {FName("spine_02"), EOHSkeletalBone::Spine_02},
    {FName("spine_03"), EOHSkeletalBone::Spine_03},
    {FName("pelvis"), EOHSkeletalBone::Pelvis},
    {FName("neck_01"), EOHSkeletalBone::Neck_01},
    {FName("head"), EOHSkeletalBone::Head},

    {FName("clavicle_l"), EOHSkeletalBone::Clavicle_L},
    {FName("upperarm_l"), EOHSkeletalBone::UpperArm_L},
    {FName("lowerarm_l"), EOHSkeletalBone::LowerArm_L},
    {FName("hand_l"), EOHSkeletalBone::Hand_L},

    {FName("clavicle_r"), EOHSkeletalBone::Clavicle_R},
    {FName("upperarm_r"), EOHSkeletalBone::UpperArm_R},
    {FName("lowerarm_r"), EOHSkeletalBone::LowerArm_R},
    {FName("hand_r"), EOHSkeletalBone::Hand_R},

    {FName("thigh_l"), EOHSkeletalBone::Thigh_L},
    {FName("calf_l"), EOHSkeletalBone::Calf_L},
    {FName("foot_l"), EOHSkeletalBone::Foot_L},
    {FName("ball_l"), EOHSkeletalBone::Ball_L},

    {FName("thigh_r"), EOHSkeletalBone::Thigh_R},
    {FName("calf_r"), EOHSkeletalBone::Calf_R},
    {FName("foot_r"), EOHSkeletalBone::Foot_R},
    {FName("ball_r"), EOHSkeletalBone::Ball_R},

    // Fingers - Left
    {FName("thumb_01_l"), EOHSkeletalBone::Thumb_01_L},
    {FName("thumb_02_l"), EOHSkeletalBone::Thumb_02_L},
    {FName("thumb_03_l"), EOHSkeletalBone::Thumb_03_L},
    {FName("index_01_l"), EOHSkeletalBone::Index_01_L},
    {FName("index_02_l"), EOHSkeletalBone::Index_02_L},
    {FName("index_03_l"), EOHSkeletalBone::Index_03_L},
    {FName("middle_01_l"), EOHSkeletalBone::Middle_01_L},
    {FName("middle_02_l"), EOHSkeletalBone::Middle_02_L},
    {FName("middle_03_l"), EOHSkeletalBone::Middle_03_L},
    {FName("ring_01_l"), EOHSkeletalBone::Ring_01_L},
    {FName("ring_02_l"), EOHSkeletalBone::Ring_02_L},
    {FName("ring_03_l"), EOHSkeletalBone::Ring_03_L},
    {FName("pinky_01_l"), EOHSkeletalBone::Pinky_01_L},
    {FName("pinky_02_l"), EOHSkeletalBone::Pinky_02_L},
    {FName("pinky_03_l"), EOHSkeletalBone::Pinky_03_L},

    // Fingers - Right
    {FName("thumb_01_r"), EOHSkeletalBone::Thumb_01_R},
    {FName("thumb_02_r"), EOHSkeletalBone::Thumb_02_R},
    {FName("thumb_03_r"), EOHSkeletalBone::Thumb_03_R},
    {FName("index_01_r"), EOHSkeletalBone::Index_01_R},
    {FName("index_02_r"), EOHSkeletalBone::Index_02_R},
    {FName("index_03_r"), EOHSkeletalBone::Index_03_R},
    {FName("middle_01_r"), EOHSkeletalBone::Middle_01_R},
    {FName("middle_02_r"), EOHSkeletalBone::Middle_02_R},
    {FName("middle_03_r"), EOHSkeletalBone::Middle_03_R},
    {FName("ring_01_r"), EOHSkeletalBone::Ring_01_R},
    {FName("ring_02_r"), EOHSkeletalBone::Ring_02_R},
    {FName("ring_03_r"), EOHSkeletalBone::Ring_03_R},
    {FName("pinky_01_r"), EOHSkeletalBone::Pinky_01_R},
    {FName("pinky_02_r"), EOHSkeletalBone::Pinky_02_R},
    {FName("pinky_03_r"), EOHSkeletalBone::Pinky_03_R},

    // IK Bones
    {FName("ik_root"), EOHSkeletalBone::IK_Root},
    {FName("ik_hand_l"), EOHSkeletalBone::IK_Hand_L},
    {FName("ik_hand_r"), EOHSkeletalBone::IK_Hand_R},
    {FName("ik_foot_l"), EOHSkeletalBone::IK_Foot_L},
    {FName("ik_foot_r"), EOHSkeletalBone::IK_Foot_R},
    {FName("ik_hand_gun"), EOHSkeletalBone::IK_Hand_Gun},
    {FName("ik_hand_root"), EOHSkeletalBone::IK_Hand_Root},
    {FName("ik_foot_root"), EOHSkeletalBone::IK_Foot_Root}};

// Primary bones only (no fingers/IK)
const TMap<EOHSkeletalBone, FName> PrimaryBoneToFNameMap = {
    {EOHSkeletalBone::Pelvis, FName("pelvis")},
    {EOHSkeletalBone::Spine_01, FName("spine_01")},
    {EOHSkeletalBone::Spine_02, FName("spine_02")},
    {EOHSkeletalBone::Spine_03, FName("spine_03")},
    {EOHSkeletalBone::Neck_01, FName("neck_01")},
    {EOHSkeletalBone::Head, FName("head")},
    {EOHSkeletalBone::Clavicle_L, FName("clavicle_l")},
    {EOHSkeletalBone::UpperArm_L, FName("upperarm_l")},
    {EOHSkeletalBone::LowerArm_L, FName("lowerarm_l")},
    {EOHSkeletalBone::Hand_L, FName("hand_l")},
    {EOHSkeletalBone::Clavicle_R, FName("clavicle_r")},
    {EOHSkeletalBone::UpperArm_R, FName("upperarm_r")},
    {EOHSkeletalBone::LowerArm_R, FName("lowerarm_r")},
    {EOHSkeletalBone::Hand_R, FName("hand_r")},
    {EOHSkeletalBone::Thigh_L, FName("thigh_l")},
    {EOHSkeletalBone::Calf_L, FName("calf_l")},
    {EOHSkeletalBone::Foot_L, FName("foot_l")},
    {EOHSkeletalBone::Ball_L, FName("ball_l")},
    {EOHSkeletalBone::Thigh_R, FName("thigh_r")},
    {EOHSkeletalBone::Calf_R, FName("calf_r")},
    {EOHSkeletalBone::Foot_R, FName("foot_r")},
    {EOHSkeletalBone::Ball_R, FName("ball_r")}};

// ==========================================
// EOHSkeletalBone <-> EOHBodyZone
// ==========================================

// One-to-One: Bone → Zone
const TMap<EOHSkeletalBone, EOHBodyZone> BoneToZoneMap = {
    {EOHSkeletalBone::Spine_02, EOHBodyZone::Torso_Upper},
    {EOHSkeletalBone::Spine_03, EOHBodyZone::Torso_Upper},
    {EOHSkeletalBone::Pelvis, EOHBodyZone::Torso_Lower},
    {EOHSkeletalBone::Spine_01, EOHBodyZone::Torso_Lower},
    {EOHSkeletalBone::Neck_01, EOHBodyZone::Cranial},
    {EOHSkeletalBone::Head, EOHBodyZone::Cranial},

    // Left Arm
    {EOHSkeletalBone::Clavicle_L, EOHBodyZone::Arm_L_Upper},
    {EOHSkeletalBone::UpperArm_L, EOHBodyZone::Arm_L_Upper},
    {EOHSkeletalBone::LowerArm_L, EOHBodyZone::Arm_L_Lower},
    {EOHSkeletalBone::Hand_L, EOHBodyZone::Hand_L},

    // Left Fingers
    {EOHSkeletalBone::Thumb_01_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Thumb_02_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Thumb_03_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Index_01_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Index_02_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Index_03_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Middle_01_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Middle_02_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Middle_03_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Ring_01_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Ring_02_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Ring_03_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Pinky_01_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Pinky_02_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Pinky_03_L, EOHBodyZone::Hand_L},

    // Right Arm
    {EOHSkeletalBone::Clavicle_R, EOHBodyZone::Arm_R_Upper},
    {EOHSkeletalBone::UpperArm_R, EOHBodyZone::Arm_R_Upper},
    {EOHSkeletalBone::LowerArm_R, EOHBodyZone::Arm_R_Lower},
    {EOHSkeletalBone::Hand_R, EOHBodyZone::Hand_R},

    // Right Fingers
    {EOHSkeletalBone::Thumb_01_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Thumb_02_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Thumb_03_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Index_01_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Index_02_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Index_03_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Middle_01_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Middle_02_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Middle_03_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Ring_01_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Ring_02_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Ring_03_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Pinky_01_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Pinky_02_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Pinky_03_R, EOHBodyZone::Hand_R},

    // Left Leg
    {EOHSkeletalBone::Thigh_L, EOHBodyZone::Leg_L_Upper},
    {EOHSkeletalBone::Calf_L, EOHBodyZone::Leg_L_Lower},
    {EOHSkeletalBone::Foot_L, EOHBodyZone::Foot_L},
    {EOHSkeletalBone::Ball_L, EOHBodyZone::Foot_L},

    // Right Leg
    {EOHSkeletalBone::Thigh_R, EOHBodyZone::Leg_R_Upper},
    {EOHSkeletalBone::Calf_R, EOHBodyZone::Leg_R_Lower},
    {EOHSkeletalBone::Foot_R, EOHBodyZone::Foot_R},
    {EOHSkeletalBone::Ball_R, EOHBodyZone::Foot_R}};

// One-to-One: Zone → Bone (representative bone for each zone)
const TMap<EOHBodyZone, EOHSkeletalBone> ZoneToBoneMap = {
    {EOHBodyZone::Torso_Upper, EOHSkeletalBone::Spine_03},
    {EOHBodyZone::Torso_Lower, EOHSkeletalBone::Pelvis},
    {EOHBodyZone::Cranial, EOHSkeletalBone::Head},
    {EOHBodyZone::Arm_L_Upper, EOHSkeletalBone::UpperArm_L},
    {EOHBodyZone::Arm_L_Lower, EOHSkeletalBone::LowerArm_L},
    {EOHBodyZone::Hand_L, EOHSkeletalBone::Hand_L},
    {EOHBodyZone::Arm_R_Upper, EOHSkeletalBone::UpperArm_R},
    {EOHBodyZone::Arm_R_Lower, EOHSkeletalBone::LowerArm_R},
    {EOHBodyZone::Hand_R, EOHSkeletalBone::Hand_R},
    {EOHBodyZone::Leg_L_Upper, EOHSkeletalBone::Thigh_L},
    {EOHBodyZone::Leg_L_Lower, EOHSkeletalBone::Calf_L},
    {EOHBodyZone::Foot_L, EOHSkeletalBone::Foot_L},
    {EOHBodyZone::Leg_R_Upper, EOHSkeletalBone::Thigh_R},
    {EOHBodyZone::Leg_R_Lower, EOHSkeletalBone::Calf_R},
    {EOHBodyZone::Foot_R, EOHSkeletalBone::Foot_R}};

// One-to-Many: Bone → Zones (bones that span multiple zones)
const TMap<EOHSkeletalBone, TArray<EOHBodyZone>> BoneToZonesMap = {
    // Most bones map to single zone, but including for completeness
    {EOHSkeletalBone::Spine_02, {EOHBodyZone::Torso_Upper}},
    {EOHSkeletalBone::Spine_03, {EOHBodyZone::Torso_Upper}},
    {EOHSkeletalBone::Pelvis, {EOHBodyZone::Torso_Lower}},
    {EOHSkeletalBone::Spine_01, {EOHBodyZone::Torso_Lower}},
    {EOHSkeletalBone::Neck_01, {EOHBodyZone::Cranial}},
    {EOHSkeletalBone::Head, {EOHBodyZone::Cranial}},

    // Arms
    {EOHSkeletalBone::Clavicle_L, {EOHBodyZone::Arm_L_Upper}},
    {EOHSkeletalBone::UpperArm_L, {EOHBodyZone::Arm_L_Upper}},
    {EOHSkeletalBone::LowerArm_L, {EOHBodyZone::Arm_L_Lower}},
    {EOHSkeletalBone::Hand_L, {EOHBodyZone::Hand_L}},

    {EOHSkeletalBone::Clavicle_R, {EOHBodyZone::Arm_R_Upper}},
    {EOHSkeletalBone::UpperArm_R, {EOHBodyZone::Arm_R_Upper}},
    {EOHSkeletalBone::LowerArm_R, {EOHBodyZone::Arm_R_Lower}},
    {EOHSkeletalBone::Hand_R, {EOHBodyZone::Hand_R}},

    // Legs
    {EOHSkeletalBone::Thigh_L, {EOHBodyZone::Leg_L_Upper}},
    {EOHSkeletalBone::Calf_L, {EOHBodyZone::Leg_L_Lower}},
    {EOHSkeletalBone::Foot_L, {EOHBodyZone::Foot_L}},
    {EOHSkeletalBone::Ball_L, {EOHBodyZone::Foot_L}},

    {EOHSkeletalBone::Thigh_R, {EOHBodyZone::Leg_R_Upper}},
    {EOHSkeletalBone::Calf_R, {EOHBodyZone::Leg_R_Lower}},
    {EOHSkeletalBone::Foot_R, {EOHBodyZone::Foot_R}},
    {EOHSkeletalBone::Ball_R, {EOHBodyZone::Foot_R}}};

// One-to-Many: Zone → Bones
const TMap<EOHBodyZone, TArray<EOHSkeletalBone>> ZoneToBonesMap = {
    {EOHBodyZone::Torso_Upper,
     {EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03}},
    {EOHBodyZone::Torso_Lower,
     {EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01}},
    {EOHBodyZone::Cranial, {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},

    {EOHBodyZone::Arm_L_Upper,
     {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L}},
    {EOHBodyZone::Arm_L_Lower, {EOHSkeletalBone::LowerArm_L}},
    {EOHBodyZone::Hand_L,
     {EOHSkeletalBone::Hand_L, EOHSkeletalBone::Thumb_01_L,
      EOHSkeletalBone::Thumb_02_L, EOHSkeletalBone::Thumb_03_L,
      EOHSkeletalBone::Index_01_L, EOHSkeletalBone::Index_02_L,
      EOHSkeletalBone::Index_03_L, EOHSkeletalBone::Middle_01_L,
      EOHSkeletalBone::Middle_02_L, EOHSkeletalBone::Middle_03_L,
      EOHSkeletalBone::Ring_01_L, EOHSkeletalBone::Ring_02_L,
      EOHSkeletalBone::Ring_03_L, EOHSkeletalBone::Pinky_01_L,
      EOHSkeletalBone::Pinky_02_L, EOHSkeletalBone::Pinky_03_L}},

    {EOHBodyZone::Arm_R_Upper,
     {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R}},
    {EOHBodyZone::Arm_R_Lower, {EOHSkeletalBone::LowerArm_R}},
    {EOHBodyZone::Hand_R,
     {EOHSkeletalBone::Hand_R, EOHSkeletalBone::Thumb_01_R,
      EOHSkeletalBone::Thumb_02_R, EOHSkeletalBone::Thumb_03_R,
      EOHSkeletalBone::Index_01_R, EOHSkeletalBone::Index_02_R,
      EOHSkeletalBone::Index_03_R, EOHSkeletalBone::Middle_01_R,
      EOHSkeletalBone::Middle_02_R, EOHSkeletalBone::Middle_03_R,
      EOHSkeletalBone::Ring_01_R, EOHSkeletalBone::Ring_02_R,
      EOHSkeletalBone::Ring_03_R, EOHSkeletalBone::Pinky_01_R,
      EOHSkeletalBone::Pinky_02_R, EOHSkeletalBone::Pinky_03_R}},

    {EOHBodyZone::Leg_L_Upper, {EOHSkeletalBone::Thigh_L}},
    {EOHBodyZone::Leg_L_Lower, {EOHSkeletalBone::Calf_L}},
    {EOHBodyZone::Foot_L, {EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},

    {EOHBodyZone::Leg_R_Upper, {EOHSkeletalBone::Thigh_R}},
    {EOHBodyZone::Leg_R_Lower, {EOHSkeletalBone::Calf_R}},
    {EOHBodyZone::Foot_R, {EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}}};

// Primary: Bone → Zone (no fingers)
const TMap<EOHSkeletalBone, EOHBodyZone> PrimaryBoneToZoneMap = {
    {EOHSkeletalBone::Pelvis, EOHBodyZone::Torso_Lower},
    {EOHSkeletalBone::Spine_01, EOHBodyZone::Torso_Lower},
    {EOHSkeletalBone::Spine_02, EOHBodyZone::Torso_Upper},
    {EOHSkeletalBone::Spine_03, EOHBodyZone::Torso_Upper},
    {EOHSkeletalBone::Neck_01, EOHBodyZone::Cranial},
    {EOHSkeletalBone::Head, EOHBodyZone::Cranial},
    {EOHSkeletalBone::Clavicle_L, EOHBodyZone::Arm_L_Upper},
    {EOHSkeletalBone::UpperArm_L, EOHBodyZone::Arm_L_Upper},
    {EOHSkeletalBone::LowerArm_L, EOHBodyZone::Arm_L_Lower},
    {EOHSkeletalBone::Hand_L, EOHBodyZone::Hand_L},
    {EOHSkeletalBone::Clavicle_R, EOHBodyZone::Arm_R_Upper},
    {EOHSkeletalBone::UpperArm_R, EOHBodyZone::Arm_R_Upper},
    {EOHSkeletalBone::LowerArm_R, EOHBodyZone::Arm_R_Lower},
    {EOHSkeletalBone::Hand_R, EOHBodyZone::Hand_R},
    {EOHSkeletalBone::Thigh_L, EOHBodyZone::Leg_L_Upper},
    {EOHSkeletalBone::Calf_L, EOHBodyZone::Leg_L_Lower},
    {EOHSkeletalBone::Foot_L, EOHBodyZone::Foot_L},
    {EOHSkeletalBone::Ball_L, EOHBodyZone::Foot_L},
    {EOHSkeletalBone::Thigh_R, EOHBodyZone::Leg_R_Upper},
    {EOHSkeletalBone::Calf_R, EOHBodyZone::Leg_R_Lower},
    {EOHSkeletalBone::Foot_R, EOHBodyZone::Foot_R},
    {EOHSkeletalBone::Ball_R, EOHBodyZone::Foot_R}};

// Primary: Zone → Primary Bone
const TMap<EOHBodyZone, EOHSkeletalBone> ZoneToPrimaryBoneMap = {
    {EOHBodyZone::Torso_Upper, EOHSkeletalBone::Spine_03},
    {EOHBodyZone::Torso_Lower, EOHSkeletalBone::Pelvis},
    {EOHBodyZone::Cranial, EOHSkeletalBone::Head},
    {EOHBodyZone::Arm_L_Upper, EOHSkeletalBone::UpperArm_L},
    {EOHBodyZone::Arm_L_Lower, EOHSkeletalBone::LowerArm_L},
    {EOHBodyZone::Hand_L, EOHSkeletalBone::Hand_L},
    {EOHBodyZone::Arm_R_Upper, EOHSkeletalBone::UpperArm_R},
    {EOHBodyZone::Arm_R_Lower, EOHSkeletalBone::LowerArm_R},
    {EOHBodyZone::Hand_R, EOHSkeletalBone::Hand_R},
    {EOHBodyZone::Leg_L_Upper, EOHSkeletalBone::Thigh_L},
    {EOHBodyZone::Leg_L_Lower, EOHSkeletalBone::Calf_L},
    {EOHBodyZone::Foot_L, EOHSkeletalBone::Foot_L},
    {EOHBodyZone::Leg_R_Upper, EOHSkeletalBone::Thigh_R},
    {EOHBodyZone::Leg_R_Lower, EOHSkeletalBone::Calf_R},
    {EOHBodyZone::Foot_R, EOHSkeletalBone::Foot_R}};

// Primary: Bone → Zones (primary bones only)
const TMap<EOHSkeletalBone, TArray<EOHBodyZone>> PrimaryBoneToZonesMap = {
    {EOHSkeletalBone::Pelvis, {EOHBodyZone::Torso_Lower}},
    {EOHSkeletalBone::Spine_01, {EOHBodyZone::Torso_Lower}},
    {EOHSkeletalBone::Spine_02, {EOHBodyZone::Torso_Upper}},
    {EOHSkeletalBone::Spine_03, {EOHBodyZone::Torso_Upper}},
    {EOHSkeletalBone::Neck_01, {EOHBodyZone::Cranial}},
    {EOHSkeletalBone::Head, {EOHBodyZone::Cranial}},
    {EOHSkeletalBone::Clavicle_L, {EOHBodyZone::Arm_L_Upper}},
    {EOHSkeletalBone::UpperArm_L, {EOHBodyZone::Arm_L_Upper}},
    {EOHSkeletalBone::LowerArm_L, {EOHBodyZone::Arm_L_Lower}},
    {EOHSkeletalBone::Hand_L, {EOHBodyZone::Hand_L}},
    {EOHSkeletalBone::Clavicle_R, {EOHBodyZone::Arm_R_Upper}},
    {EOHSkeletalBone::UpperArm_R, {EOHBodyZone::Arm_R_Upper}},
    {EOHSkeletalBone::LowerArm_R, {EOHBodyZone::Arm_R_Lower}},
    {EOHSkeletalBone::Hand_R, {EOHBodyZone::Hand_R}},
    {EOHSkeletalBone::Thigh_L, {EOHBodyZone::Leg_L_Upper}},
    {EOHSkeletalBone::Calf_L, {EOHBodyZone::Leg_L_Lower}},
    {EOHSkeletalBone::Foot_L, {EOHBodyZone::Foot_L}},
    {EOHSkeletalBone::Ball_L, {EOHBodyZone::Foot_L}},
    {EOHSkeletalBone::Thigh_R, {EOHBodyZone::Leg_R_Upper}},
    {EOHSkeletalBone::Calf_R, {EOHBodyZone::Leg_R_Lower}},
    {EOHSkeletalBone::Foot_R, {EOHBodyZone::Foot_R}},
    {EOHSkeletalBone::Ball_R, {EOHBodyZone::Foot_R}}};

// Primary: Zone → Primary Bones
const TMap<EOHBodyZone, TArray<EOHSkeletalBone>> ZoneToPrimaryBonesMap = {
    {EOHBodyZone::Torso_Upper,
     {EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03}},
    {EOHBodyZone::Torso_Lower,
     {EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01}},
    {EOHBodyZone::Cranial, {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},
    {EOHBodyZone::Arm_L_Upper,
     {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L}},
    {EOHBodyZone::Arm_L_Lower, {EOHSkeletalBone::LowerArm_L}},
    {EOHBodyZone::Hand_L, {EOHSkeletalBone::Hand_L}},
    {EOHBodyZone::Arm_R_Upper,
     {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R}},
    {EOHBodyZone::Arm_R_Lower, {EOHSkeletalBone::LowerArm_R}},
    {EOHBodyZone::Hand_R, {EOHSkeletalBone::Hand_R}},
    {EOHBodyZone::Leg_L_Upper, {EOHSkeletalBone::Thigh_L}},
    {EOHBodyZone::Leg_L_Lower, {EOHSkeletalBone::Calf_L}},
    {EOHBodyZone::Foot_L, {EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},
    {EOHBodyZone::Leg_R_Upper, {EOHSkeletalBone::Thigh_R}},
    {EOHBodyZone::Leg_R_Lower, {EOHSkeletalBone::Calf_R}},
    {EOHBodyZone::Foot_R, {EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}}};

const TMap<EOHBodyZone, TArray<EOHBodyPart>> ZoneToBodyPartsMap = {
    {EOHBodyZone::Torso_Upper, {EOHBodyPart::Torso}},
    {EOHBodyZone::Torso_Lower, {EOHBodyPart::Pelvis}},
    {EOHBodyZone::Cranial, {EOHBodyPart::Head}},

    {EOHBodyZone::Arm_L_Upper, {EOHBodyPart::Arm_Left}},
    {EOHBodyZone::Arm_L_Lower, {EOHBodyPart::Arm_Left}},
    {EOHBodyZone::Hand_L, {EOHBodyPart::Arm_Left}},

    {EOHBodyZone::Arm_R_Upper, {EOHBodyPart::Arm_Right}},
    {EOHBodyZone::Arm_R_Lower, {EOHBodyPart::Arm_Right}},
    {EOHBodyZone::Hand_R, {EOHBodyPart::Arm_Right}},

    {EOHBodyZone::Leg_L_Upper, {EOHBodyPart::Leg_Left}},
    {EOHBodyZone::Leg_L_Lower, {EOHBodyPart::Leg_Left}},
    {EOHBodyZone::Foot_L, {EOHBodyPart::Leg_Left}},

    {EOHBodyZone::Leg_R_Upper, {EOHBodyPart::Leg_Right}},
    {EOHBodyZone::Leg_R_Lower, {EOHBodyPart::Leg_Right}},
    {EOHBodyZone::Foot_R, {EOHBodyPart::Leg_Right}}};

const TMap<EOHBodyPart, TArray<EOHBodyZone>> BodyPartToZonesMap = {
    {EOHBodyPart::Torso, {EOHBodyZone::Torso_Upper}},
    {EOHBodyPart::Pelvis, {EOHBodyZone::Torso_Lower}},
    {EOHBodyPart::Head, {EOHBodyZone::Cranial}},

    {EOHBodyPart::Arm_Left,
     {EOHBodyZone::Arm_L_Upper, EOHBodyZone::Arm_L_Lower, EOHBodyZone::Hand_L}},
    {EOHBodyPart::Arm_Right,
     {EOHBodyZone::Arm_R_Upper, EOHBodyZone::Arm_R_Lower, EOHBodyZone::Hand_R}},

    {EOHBodyPart::Leg_Left,
     {EOHBodyZone::Leg_L_Upper, EOHBodyZone::Leg_L_Lower, EOHBodyZone::Foot_L}},
    {EOHBodyPart::Leg_Right,
     {EOHBodyZone::Leg_R_Upper, EOHBodyZone::Leg_R_Lower,
      EOHBodyZone::Foot_R}}};

const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyZone>>
    FunctionalGroupToZonesMap = {
        {EOHFunctionalBoneGroup::Cranial, {EOHBodyZone::Cranial}},
        {EOHFunctionalBoneGroup::Core,
         {EOHBodyZone::Torso_Lower, EOHBodyZone::Torso_Upper}},

        {EOHFunctionalBoneGroup::LeftArm,
         {EOHBodyZone::Arm_L_Upper, EOHBodyZone::Arm_L_Lower}},
        {EOHFunctionalBoneGroup::RightArm,
         {EOHBodyZone::Arm_R_Upper, EOHBodyZone::Arm_R_Lower}},
        {EOHFunctionalBoneGroup::LeftHand, {EOHBodyZone::Hand_L}},
        {EOHFunctionalBoneGroup::RightHand, {EOHBodyZone::Hand_R}},

        {EOHFunctionalBoneGroup::LeftLeg,
         {EOHBodyZone::Leg_L_Upper, EOHBodyZone::Leg_L_Lower}},
        {EOHFunctionalBoneGroup::RightLeg,
         {EOHBodyZone::Leg_R_Upper, EOHBodyZone::Leg_R_Lower}},
        {EOHFunctionalBoneGroup::LeftFoot, {EOHBodyZone::Foot_L}},
        {EOHFunctionalBoneGroup::RightFoot, {EOHBodyZone::Foot_R}},

        {EOHFunctionalBoneGroup::UpperBody,
         {EOHBodyZone::Cranial, EOHBodyZone::Torso_Upper,
          EOHBodyZone::Arm_L_Upper, EOHBodyZone::Arm_R_Upper}},
        {EOHFunctionalBoneGroup::LowerBody,
         {EOHBodyZone::Torso_Lower, EOHBodyZone::Leg_L_Upper,
          EOHBodyZone::Leg_R_Upper}},
        {EOHFunctionalBoneGroup::LeftLimbs,
         {EOHBodyZone::Arm_L_Upper, EOHBodyZone::Leg_L_Upper}},
        {EOHFunctionalBoneGroup::RightLimbs,
         {EOHBodyZone::Arm_R_Upper, EOHBodyZone::Leg_R_Upper}},
        {EOHFunctionalBoneGroup::FullBody,
         {EOHBodyZone::Cranial, EOHBodyZone::Torso_Upper,
          EOHBodyZone::Torso_Lower, EOHBodyZone::Arm_L_Upper,
          EOHBodyZone::Arm_L_Lower, EOHBodyZone::Hand_L,
          EOHBodyZone::Arm_R_Upper, EOHBodyZone::Arm_R_Lower,
          EOHBodyZone::Hand_R, EOHBodyZone::Leg_L_Upper,
          EOHBodyZone::Leg_L_Lower, EOHBodyZone::Foot_L,
          EOHBodyZone::Leg_R_Upper, EOHBodyZone::Leg_R_Lower,
          EOHBodyZone::Foot_R}}};

const TMap<EOHFunctionalBoneGroup, TArray<EOHBodyPart>>
    FunctionalGroupToBodyPartsMap = {
        {EOHFunctionalBoneGroup::Cranial, {EOHBodyPart::Head}},
        {EOHFunctionalBoneGroup::Core,
         {EOHBodyPart::Pelvis, EOHBodyPart::Torso}},
        {EOHFunctionalBoneGroup::LeftArm, {EOHBodyPart::Arm_Left}},
        {EOHFunctionalBoneGroup::RightArm, {EOHBodyPart::Arm_Right}},
        {EOHFunctionalBoneGroup::LeftHand, {EOHBodyPart::Arm_Left}},
        {EOHFunctionalBoneGroup::RightHand, {EOHBodyPart::Arm_Right}},
        {EOHFunctionalBoneGroup::LeftLeg, {EOHBodyPart::Leg_Left}},
        {EOHFunctionalBoneGroup::RightLeg, {EOHBodyPart::Leg_Right}},
        {EOHFunctionalBoneGroup::LeftFoot, {EOHBodyPart::Leg_Left}},
        {EOHFunctionalBoneGroup::RightFoot, {EOHBodyPart::Leg_Right}},
        {EOHFunctionalBoneGroup::UpperBody,
         {EOHBodyPart::Head, EOHBodyPart::Torso, EOHBodyPart::Arm_Left,
          EOHBodyPart::Arm_Right}},
        {EOHFunctionalBoneGroup::LowerBody,
         {EOHBodyPart::Pelvis, EOHBodyPart::Leg_Left, EOHBodyPart::Leg_Right}},
        {EOHFunctionalBoneGroup::LeftLimbs,
         {EOHBodyPart::Arm_Left, EOHBodyPart::Leg_Left}},
        {EOHFunctionalBoneGroup::RightLimbs,
         {EOHBodyPart::Arm_Right, EOHBodyPart::Leg_Right}},
        {EOHFunctionalBoneGroup::FullBody,
         {EOHBodyPart::Pelvis, EOHBodyPart::Torso, EOHBodyPart::Head,
          EOHBodyPart::Arm_Left, EOHBodyPart::Arm_Right, EOHBodyPart::Leg_Left,
          EOHBodyPart::Leg_Right}}};
// ==========================================
// EOHSkeletalBone <-> EOHBodyPart
// ==========================================

// One-to-One: Bone → BodyPart
const TMap<EOHSkeletalBone, EOHBodyPart> BoneToBodyPartMap = {
    {EOHSkeletalBone::Spine_01, EOHBodyPart::Torso},
    {EOHSkeletalBone::Spine_02, EOHBodyPart::Torso},
    {EOHSkeletalBone::Spine_03, EOHBodyPart::Torso},
    {EOHSkeletalBone::Pelvis, EOHBodyPart::Pelvis},
    {EOHSkeletalBone::Neck_01, EOHBodyPart::Head},
    {EOHSkeletalBone::Head, EOHBodyPart::Head},

    // Left Arm
    {EOHSkeletalBone::Clavicle_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::UpperArm_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::LowerArm_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Hand_L, EOHBodyPart::Arm_Left},

    // Left Fingers
    {EOHSkeletalBone::Thumb_01_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Thumb_02_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Thumb_03_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Index_01_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Index_02_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Index_03_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Middle_01_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Middle_02_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Middle_03_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Ring_01_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Ring_02_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Ring_03_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Pinky_01_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Pinky_02_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Pinky_03_L, EOHBodyPart::Arm_Left},

    // Right Arm
    {EOHSkeletalBone::Clavicle_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::UpperArm_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::LowerArm_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Hand_R, EOHBodyPart::Arm_Right},

    // Right Fingers
    {EOHSkeletalBone::Thumb_01_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Thumb_02_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Thumb_03_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Index_01_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Index_02_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Index_03_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Middle_01_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Middle_02_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Middle_03_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Ring_01_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Ring_02_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Ring_03_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Pinky_01_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Pinky_02_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Pinky_03_R, EOHBodyPart::Arm_Right},

    // Left Leg
    {EOHSkeletalBone::Thigh_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Calf_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Foot_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Ball_L, EOHBodyPart::Leg_Left},

    // Right Leg
    {EOHSkeletalBone::Thigh_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Calf_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Foot_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Ball_R, EOHBodyPart::Leg_Right}};

// One-to-One: BodyPart → Bone (representative bone)
const TMap<EOHBodyPart, EOHSkeletalBone> BodyPartToBoneMap = {
    {EOHBodyPart::Pelvis, EOHSkeletalBone::Pelvis},
    {EOHBodyPart::Torso, EOHSkeletalBone::Spine_02},
    {EOHBodyPart::Head, EOHSkeletalBone::Head},
    {EOHBodyPart::Arm_Left, EOHSkeletalBone::UpperArm_L},
    {EOHBodyPart::Arm_Right, EOHSkeletalBone::UpperArm_R},
    {EOHBodyPart::Leg_Left, EOHSkeletalBone::Thigh_L},
    {EOHBodyPart::Leg_Right, EOHSkeletalBone::Thigh_R}};

// One-to-Many: Bone → BodyParts
const TMap<EOHSkeletalBone, TArray<EOHBodyPart>> BoneToBodyPartsMap = {
    // Core bones
    {EOHSkeletalBone::Pelvis, {EOHBodyPart::Pelvis}},
    {EOHSkeletalBone::Spine_01, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Spine_02, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Spine_03, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Neck_01, {EOHBodyPart::Head}},
    {EOHSkeletalBone::Head, {EOHBodyPart::Head}},

    // Arms - all map to single part
    {EOHSkeletalBone::Clavicle_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::UpperArm_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::LowerArm_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::Hand_L, {EOHBodyPart::Arm_Left}},

    {EOHSkeletalBone::Clavicle_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::UpperArm_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::LowerArm_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::Hand_R, {EOHBodyPart::Arm_Right}},

    // Legs
    {EOHSkeletalBone::Thigh_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Calf_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Foot_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Ball_L, {EOHBodyPart::Leg_Left}},

    {EOHSkeletalBone::Thigh_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Calf_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Foot_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Ball_R, {EOHBodyPart::Leg_Right}}};

// One-to-Many: BodyPart → Bones
const TMap<EOHBodyPart, TArray<EOHSkeletalBone>> BodyPartToBonesMap = {
    {EOHBodyPart::Torso,
     {EOHSkeletalBone::Spine_01, EOHSkeletalBone::Spine_02,
      EOHSkeletalBone::Spine_03}},
    {EOHBodyPart::Pelvis, {EOHSkeletalBone::Pelvis}},
    {EOHBodyPart::Head, {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},

    {EOHBodyPart::Arm_Left,
     {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
      EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,
      EOHSkeletalBone::Thumb_01_L, EOHSkeletalBone::Thumb_02_L,
      EOHSkeletalBone::Thumb_03_L, EOHSkeletalBone::Index_01_L,
      EOHSkeletalBone::Index_02_L, EOHSkeletalBone::Index_03_L,
      EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Middle_02_L,
      EOHSkeletalBone::Middle_03_L, EOHSkeletalBone::Ring_01_L,
      EOHSkeletalBone::Ring_02_L, EOHSkeletalBone::Ring_03_L,
      EOHSkeletalBone::Pinky_01_L, EOHSkeletalBone::Pinky_02_L,
      EOHSkeletalBone::Pinky_03_L}},

    {EOHBodyPart::Arm_Right,
     {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
      EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
      EOHSkeletalBone::Thumb_01_R, EOHSkeletalBone::Thumb_02_R,
      EOHSkeletalBone::Thumb_03_R, EOHSkeletalBone::Index_01_R,
      EOHSkeletalBone::Index_02_R, EOHSkeletalBone::Index_03_R,
      EOHSkeletalBone::Middle_01_R, EOHSkeletalBone::Middle_02_R,
      EOHSkeletalBone::Middle_03_R, EOHSkeletalBone::Ring_01_R,
      EOHSkeletalBone::Ring_02_R, EOHSkeletalBone::Ring_03_R,
      EOHSkeletalBone::Pinky_01_R, EOHSkeletalBone::Pinky_02_R,
      EOHSkeletalBone::Pinky_03_R}},

    {EOHBodyPart::Leg_Left,
     {EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L,
      EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},

    {EOHBodyPart::Leg_Right,
     {EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R,
      EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}}};

// Primary: Bone → BodyPart
const TMap<EOHSkeletalBone, EOHBodyPart> PrimaryBoneToBodyPartMap = {
    {EOHSkeletalBone::Pelvis, EOHBodyPart::Pelvis},
    {EOHSkeletalBone::Spine_01, EOHBodyPart::Torso},
    {EOHSkeletalBone::Spine_02, EOHBodyPart::Torso},
    {EOHSkeletalBone::Spine_03, EOHBodyPart::Torso},
    {EOHSkeletalBone::Neck_01, EOHBodyPart::Head},
    {EOHSkeletalBone::Head, EOHBodyPart::Head},
    {EOHSkeletalBone::Clavicle_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::UpperArm_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::LowerArm_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Hand_L, EOHBodyPart::Arm_Left},
    {EOHSkeletalBone::Clavicle_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::UpperArm_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::LowerArm_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Hand_R, EOHBodyPart::Arm_Right},
    {EOHSkeletalBone::Thigh_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Calf_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Foot_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Ball_L, EOHBodyPart::Leg_Left},
    {EOHSkeletalBone::Thigh_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Calf_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Foot_R, EOHBodyPart::Leg_Right},
    {EOHSkeletalBone::Ball_R, EOHBodyPart::Leg_Right}};

// Primary: BodyPart → Primary Bone
const TMap<EOHBodyPart, EOHSkeletalBone> BodyPartToPrimaryBoneMap = {
    {EOHBodyPart::Pelvis, EOHSkeletalBone::Pelvis},
    {EOHBodyPart::Torso, EOHSkeletalBone::Spine_02},
    {EOHBodyPart::Head, EOHSkeletalBone::Head},
    {EOHBodyPart::Arm_Left, EOHSkeletalBone::UpperArm_L},
    {EOHBodyPart::Arm_Right, EOHSkeletalBone::UpperArm_R},
    {EOHBodyPart::Leg_Left, EOHSkeletalBone::Thigh_L},
    {EOHBodyPart::Leg_Right, EOHSkeletalBone::Thigh_R}};

// Primary: Bone → BodyParts
const TMap<EOHSkeletalBone, TArray<EOHBodyPart>> PrimaryBoneToBodyPartsMap = {
    {EOHSkeletalBone::Pelvis, {EOHBodyPart::Pelvis}},
    {EOHSkeletalBone::Spine_01, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Spine_02, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Spine_03, {EOHBodyPart::Torso}},
    {EOHSkeletalBone::Neck_01, {EOHBodyPart::Head}},
    {EOHSkeletalBone::Head, {EOHBodyPart::Head}},
    {EOHSkeletalBone::Clavicle_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::UpperArm_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::LowerArm_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::Hand_L, {EOHBodyPart::Arm_Left}},
    {EOHSkeletalBone::Clavicle_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::UpperArm_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::LowerArm_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::Hand_R, {EOHBodyPart::Arm_Right}},
    {EOHSkeletalBone::Thigh_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Calf_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Foot_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Ball_L, {EOHBodyPart::Leg_Left}},
    {EOHSkeletalBone::Thigh_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Calf_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Foot_R, {EOHBodyPart::Leg_Right}},
    {EOHSkeletalBone::Ball_R, {EOHBodyPart::Leg_Right}}};

// Primary: BodyPart → Primary Bones
const TMap<EOHBodyPart, TArray<EOHSkeletalBone>> BodyPartToPrimaryBonesMap = {
    {EOHBodyPart::Pelvis, {EOHSkeletalBone::Pelvis}},
    {EOHBodyPart::Torso,
     {EOHSkeletalBone::Spine_01, EOHSkeletalBone::Spine_02,
      EOHSkeletalBone::Spine_03}},
    {EOHBodyPart::Head, {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},
    {EOHBodyPart::Arm_Left,
     {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
      EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L}},
    {EOHBodyPart::Arm_Right,
     {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
      EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R}},
    {EOHBodyPart::Leg_Left,
     {EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L,
      EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},
    {EOHBodyPart::Leg_Right,
     {EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R,
      EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}}};

// ==========================================
// EOHSkeletalBone <-> EOHFunctionalBoneGroup
// ==========================================

// One-to-One: Bone → FunctionalGroup (primary group for each bone)
const TMap<EOHSkeletalBone, EOHFunctionalBoneGroup> BoneToFunctionalGroupMap = {
    // Core
    {EOHSkeletalBone::Pelvis, EOHFunctionalBoneGroup::Core},
    {EOHSkeletalBone::Spine_01, EOHFunctionalBoneGroup::Core},
    {EOHSkeletalBone::Spine_02, EOHFunctionalBoneGroup::Core},
    {EOHSkeletalBone::Spine_03, EOHFunctionalBoneGroup::Core},

    // Cranial
    {EOHSkeletalBone::Neck_01, EOHFunctionalBoneGroup::Cranial},
    {EOHSkeletalBone::Head, EOHFunctionalBoneGroup::Cranial},

    // Left Arm
    {EOHSkeletalBone::Clavicle_L, EOHFunctionalBoneGroup::LeftArm},
    {EOHSkeletalBone::UpperArm_L, EOHFunctionalBoneGroup::LeftArm},
    {EOHSkeletalBone::LowerArm_L, EOHFunctionalBoneGroup::LeftArm},
    {EOHSkeletalBone::Hand_L, EOHFunctionalBoneGroup::LeftHand},

    // Left Fingers
    {EOHSkeletalBone::Thumb_01_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Thumb_02_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Thumb_03_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Index_01_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Index_02_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Index_03_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Middle_01_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Middle_02_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Middle_03_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Ring_01_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Ring_02_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Ring_03_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Pinky_01_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Pinky_02_L, EOHFunctionalBoneGroup::LeftHand},
    {EOHSkeletalBone::Pinky_03_L, EOHFunctionalBoneGroup::LeftHand},

    // Right Arm
    {EOHSkeletalBone::Clavicle_R, EOHFunctionalBoneGroup::RightArm},
    {EOHSkeletalBone::UpperArm_R, EOHFunctionalBoneGroup::RightArm},
    {EOHSkeletalBone::LowerArm_R, EOHFunctionalBoneGroup::RightArm},
    {EOHSkeletalBone::Hand_R, EOHFunctionalBoneGroup::RightHand},

    // Right Fingers
    {EOHSkeletalBone::Thumb_01_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Thumb_02_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Thumb_03_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Index_01_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Index_02_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Index_03_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Middle_01_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Middle_02_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Middle_03_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Ring_01_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Ring_02_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Ring_03_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Pinky_01_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Pinky_02_R, EOHFunctionalBoneGroup::RightHand},
    {EOHSkeletalBone::Pinky_03_R, EOHFunctionalBoneGroup::RightHand},

    // Left Leg
    {EOHSkeletalBone::Thigh_L, EOHFunctionalBoneGroup::LeftLeg},
    {EOHSkeletalBone::Calf_L, EOHFunctionalBoneGroup::LeftLeg},
    {EOHSkeletalBone::Foot_L, EOHFunctionalBoneGroup::LeftFoot},
    {EOHSkeletalBone::Ball_L, EOHFunctionalBoneGroup::LeftFoot},

    // Right Leg
    {EOHSkeletalBone::Thigh_R, EOHFunctionalBoneGroup::RightLeg},
    {EOHSkeletalBone::Calf_R, EOHFunctionalBoneGroup::RightLeg},
    {EOHSkeletalBone::Foot_R, EOHFunctionalBoneGroup::RightFoot},
    {EOHSkeletalBone::Ball_R, EOHFunctionalBoneGroup::RightFoot}};

// One-to-One: FunctionalGroup → Bone (representative bone)
const TMap<EOHFunctionalBoneGroup, EOHSkeletalBone> FunctionalGroupToBoneMap = {
    {EOHFunctionalBoneGroup::Core, EOHSkeletalBone::Spine_02},
    {EOHFunctionalBoneGroup::Cranial, EOHSkeletalBone::Head},
    {EOHFunctionalBoneGroup::LeftArm, EOHSkeletalBone::UpperArm_L},
    {EOHFunctionalBoneGroup::RightArm, EOHSkeletalBone::UpperArm_R},
    {EOHFunctionalBoneGroup::LeftHand, EOHSkeletalBone::Hand_L},
    {EOHFunctionalBoneGroup::RightHand, EOHSkeletalBone::Hand_R},
    {EOHFunctionalBoneGroup::Arms, EOHSkeletalBone::UpperArm_L},
    {EOHFunctionalBoneGroup::Hands, EOHSkeletalBone::Hand_L},
    {EOHFunctionalBoneGroup::LeftLeg, EOHSkeletalBone::Thigh_L},
    {EOHFunctionalBoneGroup::RightLeg, EOHSkeletalBone::Thigh_R},
    {EOHFunctionalBoneGroup::LeftFoot, EOHSkeletalBone::Foot_L},
    {EOHFunctionalBoneGroup::RightFoot, EOHSkeletalBone::Foot_R},
    {EOHFunctionalBoneGroup::Legs, EOHSkeletalBone::Thigh_L},
    {EOHFunctionalBoneGroup::Feet, EOHSkeletalBone::Foot_L},
    {EOHFunctionalBoneGroup::UpperBody, EOHSkeletalBone::Spine_03},
    {EOHFunctionalBoneGroup::LowerBody, EOHSkeletalBone::Pelvis},
    {EOHFunctionalBoneGroup::LeftLimbs, EOHSkeletalBone::UpperArm_L},
    {EOHFunctionalBoneGroup::RightLimbs, EOHSkeletalBone::UpperArm_R},
    {EOHFunctionalBoneGroup::FullBody, EOHSkeletalBone::Spine_02}};

// One-to-Many: Bone → FunctionalGroups
const TMap<EOHSkeletalBone, TArray<EOHFunctionalBoneGroup>>
    BoneToFunctionalGroupsMap = {
        // Core
        {EOHSkeletalBone::Pelvis,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_01,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_02,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_03,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Cranial
        {EOHSkeletalBone::Neck_01,
         {EOHFunctionalBoneGroup::Cranial, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Head,
         {EOHFunctionalBoneGroup::Cranial, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Left Arm
        {EOHSkeletalBone::Clavicle_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::UpperArm_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::LowerArm_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Left Hand
        {EOHSkeletalBone::Hand_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_01_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_02_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_03_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_01_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_02_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_03_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_01_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_02_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_03_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_01_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_02_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_03_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_01_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_02_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_03_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Right Arm
        {EOHSkeletalBone::Clavicle_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::UpperArm_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::LowerArm_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Right Hand
        {EOHSkeletalBone::Hand_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_01_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_02_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thumb_03_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_01_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_02_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Index_03_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_01_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_02_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Middle_03_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_01_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_02_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ring_03_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_01_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_02_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Pinky_03_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Left Leg
        {EOHSkeletalBone::Thigh_L,
         {EOHFunctionalBoneGroup::LeftLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Calf_L,
         {EOHFunctionalBoneGroup::LeftLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Right Leg
        {EOHSkeletalBone::Thigh_R,
         {EOHFunctionalBoneGroup::RightLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Calf_R,
         {EOHFunctionalBoneGroup::RightLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Left Foot
        {EOHSkeletalBone::Foot_L,
         {EOHFunctionalBoneGroup::LeftFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ball_L,
         {EOHFunctionalBoneGroup::LeftFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},

        // Right Foot
        {EOHSkeletalBone::Foot_R,
         {EOHFunctionalBoneGroup::RightFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ball_R,
         {EOHFunctionalBoneGroup::RightFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}}};

// --------------------------------------------------
// FunctionalGroup → Bones
// --------------------------------------------------

// All Functional Groups To All Bones Map
const TMap<EOHFunctionalBoneGroup, TArray<EOHSkeletalBone>>
    FunctionalGroupToBonesMap = {
        {EOHFunctionalBoneGroup::None, {}},
        {EOHFunctionalBoneGroup::Cranial,
         {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},
        {EOHFunctionalBoneGroup::Core,
         {EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01,
          EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03}},
        {EOHFunctionalBoneGroup::LeftArm,
         {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
          EOHSkeletalBone::LowerArm_L}},
        {EOHFunctionalBoneGroup::RightArm,
         {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
          EOHSkeletalBone::LowerArm_R}},
        {EOHFunctionalBoneGroup::LeftHand,
         {EOHSkeletalBone::Hand_L, EOHSkeletalBone::Thumb_01_L,
          EOHSkeletalBone::Thumb_02_L, EOHSkeletalBone::Thumb_03_L,
          EOHSkeletalBone::Index_01_L, EOHSkeletalBone::Index_02_L,
          EOHSkeletalBone::Index_03_L, EOHSkeletalBone::Middle_01_L,
          EOHSkeletalBone::Middle_02_L, EOHSkeletalBone::Middle_03_L,
          EOHSkeletalBone::Ring_01_L, EOHSkeletalBone::Ring_02_L,
          EOHSkeletalBone::Ring_03_L, EOHSkeletalBone::Pinky_01_L,
          EOHSkeletalBone::Pinky_02_L, EOHSkeletalBone::Pinky_03_L}},

        {EOHFunctionalBoneGroup::RightHand,
         {EOHSkeletalBone::Hand_R, EOHSkeletalBone::Thumb_01_R,
          EOHSkeletalBone::Thumb_02_R, EOHSkeletalBone::Thumb_03_R,
          EOHSkeletalBone::Index_01_R, EOHSkeletalBone::Index_02_R,
          EOHSkeletalBone::Index_03_R, EOHSkeletalBone::Middle_01_R,
          EOHSkeletalBone::Middle_02_R, EOHSkeletalBone::Middle_03_R,
          EOHSkeletalBone::Ring_01_R, EOHSkeletalBone::Ring_02_R,
          EOHSkeletalBone::Ring_03_R, EOHSkeletalBone::Pinky_01_R,
          EOHSkeletalBone::Pinky_02_R, EOHSkeletalBone::Pinky_03_R}},

        {EOHFunctionalBoneGroup::Arms,
         {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
          EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Clavicle_R,
          EOHSkeletalBone::UpperArm_R, EOHSkeletalBone::LowerArm_R}},

        {EOHFunctionalBoneGroup::Hands,
         {EOHSkeletalBone::Hand_L,      EOHSkeletalBone::Hand_R,
          EOHSkeletalBone::Thumb_01_L,  EOHSkeletalBone::Thumb_02_L,
          EOHSkeletalBone::Thumb_03_L,  EOHSkeletalBone::Index_01_L,
          EOHSkeletalBone::Index_02_L,  EOHSkeletalBone::Index_03_L,
          EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Middle_02_L,
          EOHSkeletalBone::Middle_03_L, EOHSkeletalBone::Ring_01_L,
          EOHSkeletalBone::Ring_02_L,   EOHSkeletalBone::Ring_03_L,
          EOHSkeletalBone::Pinky_01_L,  EOHSkeletalBone::Pinky_02_L,
          EOHSkeletalBone::Pinky_03_L,  EOHSkeletalBone::Thumb_01_R,
          EOHSkeletalBone::Thumb_02_R,  EOHSkeletalBone::Thumb_03_R,
          EOHSkeletalBone::Index_01_R,  EOHSkeletalBone::Index_02_R,
          EOHSkeletalBone::Index_03_R,  EOHSkeletalBone::Middle_01_R,
          EOHSkeletalBone::Middle_02_R, EOHSkeletalBone::Middle_03_R,
          EOHSkeletalBone::Ring_01_R,   EOHSkeletalBone::Ring_02_R,
          EOHSkeletalBone::Ring_03_R,   EOHSkeletalBone::Pinky_01_R,
          EOHSkeletalBone::Pinky_02_R,  EOHSkeletalBone::Pinky_03_R}},

        {EOHFunctionalBoneGroup::LeftLeg,
         {EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L}},
        {EOHFunctionalBoneGroup::RightLeg,
         {EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R}},
        {EOHFunctionalBoneGroup::Legs,
         {EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L,
          EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R}},
        {EOHFunctionalBoneGroup::LeftFoot,
         {EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},
        {EOHFunctionalBoneGroup::RightFoot,
         {EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}},
        {EOHFunctionalBoneGroup::Feet,
         {EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L,
          EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}},
        {EOHFunctionalBoneGroup::UpperBody,
         {
             EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01,
             EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03,
             EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head,
             EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
             EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,
             EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
             EOHSkeletalBone::LowerArm_R,
             EOHSkeletalBone::Hand_R // Add fingers if needed
         }},
        {EOHFunctionalBoneGroup::LowerBody,
         {EOHSkeletalBone::Pelvis, EOHSkeletalBone::Thigh_L,
          EOHSkeletalBone::Calf_L, EOHSkeletalBone::Foot_L,
          EOHSkeletalBone::Ball_L, EOHSkeletalBone::Thigh_R,
          EOHSkeletalBone::Calf_R, EOHSkeletalBone::Foot_R,
          EOHSkeletalBone::Ball_R}},
        {EOHFunctionalBoneGroup::LeftLimbs,
         {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
          EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,
          EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L,
          EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},
        {EOHFunctionalBoneGroup::RightLimbs,
         {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
          EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
          EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R,
          EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}},
        {EOHFunctionalBoneGroup::FullBody,
         {EOHSkeletalBone::Pelvis,     EOHSkeletalBone::Spine_01,
          EOHSkeletalBone::Spine_02,   EOHSkeletalBone::Spine_03,
          EOHSkeletalBone::Neck_01,    EOHSkeletalBone::Head,
          EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
          EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,
          EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
          EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,
          EOHSkeletalBone::Thigh_L,    EOHSkeletalBone::Calf_L,
          EOHSkeletalBone::Foot_L,     EOHSkeletalBone::Ball_L,
          EOHSkeletalBone::Thigh_R,    EOHSkeletalBone::Calf_R,
          EOHSkeletalBone::Foot_R,     EOHSkeletalBone::Ball_R}}};

const TMap<EOHSkeletalBone, TArray<EOHFunctionalBoneGroup>>
    PrimaryBoneToFunctionalGroupsMap = {
        {EOHSkeletalBone::Pelvis,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_01,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_02,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Spine_03,
         {EOHFunctionalBoneGroup::Core, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Neck_01,
         {EOHFunctionalBoneGroup::Cranial, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Head,
         {EOHFunctionalBoneGroup::Cranial, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Clavicle_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::UpperArm_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::LowerArm_L,
         {EOHFunctionalBoneGroup::LeftArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Hand_L,
         {EOHFunctionalBoneGroup::LeftHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Clavicle_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::UpperArm_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::LowerArm_R,
         {EOHFunctionalBoneGroup::RightArm, EOHFunctionalBoneGroup::Arms,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Hand_R,
         {EOHFunctionalBoneGroup::RightHand, EOHFunctionalBoneGroup::Hands,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::UpperBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thigh_L,
         {EOHFunctionalBoneGroup::LeftLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Calf_L,
         {EOHFunctionalBoneGroup::LeftLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Foot_L,
         {EOHFunctionalBoneGroup::LeftFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ball_L,
         {EOHFunctionalBoneGroup::LeftFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::LeftLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Thigh_R,
         {EOHFunctionalBoneGroup::RightLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Calf_R,
         {EOHFunctionalBoneGroup::RightLeg, EOHFunctionalBoneGroup::Legs,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Foot_R,
         {EOHFunctionalBoneGroup::RightFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}},
        {EOHSkeletalBone::Ball_R,
         {EOHFunctionalBoneGroup::RightFoot, EOHFunctionalBoneGroup::Feet,
          EOHFunctionalBoneGroup::RightLimbs, EOHFunctionalBoneGroup::LowerBody,
          EOHFunctionalBoneGroup::FullBody}}};

// Primary Bones To A Single Primary Functional Group
const TMap<EOHSkeletalBone, EOHFunctionalBoneGroup>
    PrimaryBoneToFunctionalGroupMap = {
        {EOHSkeletalBone::Pelvis, EOHFunctionalBoneGroup::Core},
        {EOHSkeletalBone::Spine_01, EOHFunctionalBoneGroup::Core},
        {EOHSkeletalBone::Spine_02, EOHFunctionalBoneGroup::Core},
        {EOHSkeletalBone::Spine_03, EOHFunctionalBoneGroup::Core},
        {EOHSkeletalBone::Neck_01, EOHFunctionalBoneGroup::Cranial},
        {EOHSkeletalBone::Head, EOHFunctionalBoneGroup::Cranial},
        {EOHSkeletalBone::Clavicle_L, EOHFunctionalBoneGroup::LeftArm},
        {EOHSkeletalBone::UpperArm_L, EOHFunctionalBoneGroup::LeftArm},
        {EOHSkeletalBone::LowerArm_L, EOHFunctionalBoneGroup::LeftArm},
        {EOHSkeletalBone::Hand_L, EOHFunctionalBoneGroup::LeftHand},
        {EOHSkeletalBone::Clavicle_R, EOHFunctionalBoneGroup::RightArm},
        {EOHSkeletalBone::UpperArm_R, EOHFunctionalBoneGroup::RightArm},
        {EOHSkeletalBone::LowerArm_R, EOHFunctionalBoneGroup::RightArm},
        {EOHSkeletalBone::Hand_R, EOHFunctionalBoneGroup::RightHand},
        {EOHSkeletalBone::Thigh_L, EOHFunctionalBoneGroup::LeftLeg},
        {EOHSkeletalBone::Calf_L, EOHFunctionalBoneGroup::LeftLeg},
        {EOHSkeletalBone::Foot_L, EOHFunctionalBoneGroup::LeftFoot},
        {EOHSkeletalBone::Ball_L, EOHFunctionalBoneGroup::LeftFoot},
        {EOHSkeletalBone::Thigh_R, EOHFunctionalBoneGroup::RightLeg},
        {EOHSkeletalBone::Calf_R, EOHFunctionalBoneGroup::RightLeg},
        {EOHSkeletalBone::Foot_R, EOHFunctionalBoneGroup::RightFoot},
        {EOHSkeletalBone::Ball_R, EOHFunctionalBoneGroup::RightFoot}};

// Functional Group To Primary Bones
const TMap<EOHFunctionalBoneGroup, TArray<EOHSkeletalBone>>
    FunctionalGroupToPrimaryBonesMap = {
        {EOHFunctionalBoneGroup::Core,
         {EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01,
          EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03}},

        {EOHFunctionalBoneGroup::Cranial,
         {EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head}},

        {EOHFunctionalBoneGroup::LeftArm,
         {EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
          EOHSkeletalBone::LowerArm_L}},

        {EOHFunctionalBoneGroup::RightArm,
         {EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
          EOHSkeletalBone::LowerArm_R}},

        {EOHFunctionalBoneGroup::LeftLeg,
         {EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L}},

        {EOHFunctionalBoneGroup::RightLeg,
         {EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R}},

        {EOHFunctionalBoneGroup::LeftFoot,
         {EOHSkeletalBone::Foot_L, EOHSkeletalBone::Ball_L}},

        {EOHFunctionalBoneGroup::RightFoot,
         {EOHSkeletalBone::Foot_R, EOHSkeletalBone::Ball_R}}};

const TArray<EOHSkeletalBone> PrimarySkeletalBones = {
    // Core
    EOHSkeletalBone::Pelvis, EOHSkeletalBone::Spine_01,
    EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_03,
    EOHSkeletalBone::Neck_01, EOHSkeletalBone::Head,

    // Left Arm
    EOHSkeletalBone::Clavicle_L, EOHSkeletalBone::UpperArm_L,
    EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::Hand_L,

    // Right Arm
    EOHSkeletalBone::Clavicle_R, EOHSkeletalBone::UpperArm_R,
    EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::Hand_R,

    // Left Leg
    EOHSkeletalBone::Thigh_L, EOHSkeletalBone::Calf_L, EOHSkeletalBone::Foot_L,

    // Right Leg
    EOHSkeletalBone::Thigh_R, EOHSkeletalBone::Calf_R, EOHSkeletalBone::Foot_R};

// All Finger Bones Definition
const TArray<EOHSkeletalBone> AllFingerBones = {
    // Left Hand Fingers
    EOHSkeletalBone::Thumb_01_L, EOHSkeletalBone::Thumb_02_L,
    EOHSkeletalBone::Thumb_03_L, EOHSkeletalBone::Index_01_L,
    EOHSkeletalBone::Index_02_L, EOHSkeletalBone::Index_03_L,
    EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Middle_02_L,
    EOHSkeletalBone::Middle_03_L, EOHSkeletalBone::Ring_01_L,
    EOHSkeletalBone::Ring_02_L, EOHSkeletalBone::Ring_03_L,
    EOHSkeletalBone::Pinky_01_L, EOHSkeletalBone::Pinky_02_L,
    EOHSkeletalBone::Pinky_03_L,

    // Right Hand Fingers
    EOHSkeletalBone::Thumb_01_R, EOHSkeletalBone::Thumb_02_R,
    EOHSkeletalBone::Thumb_03_R, EOHSkeletalBone::Index_01_R,
    EOHSkeletalBone::Index_02_R, EOHSkeletalBone::Index_03_R,
    EOHSkeletalBone::Middle_01_R, EOHSkeletalBone::Middle_02_R,
    EOHSkeletalBone::Middle_03_R, EOHSkeletalBone::Ring_01_R,
    EOHSkeletalBone::Ring_02_R, EOHSkeletalBone::Ring_03_R,
    EOHSkeletalBone::Pinky_01_R, EOHSkeletalBone::Pinky_02_R,
    EOHSkeletalBone::Pinky_03_R};

// End of EOHSkeletalBone enum definition

} // namespace OHSkeletalMappings

#if 0
	// ======================= Internal Resolver ==========================
	
	static TOptional<EOHSkeletalBone> TryResolveBoneWithFilter(
		const FString& RawInput,
		const TMap<FName, EOHSkeletalBone>& BoneMap,
		const USkeletalMeshComponent* Mesh,
		TFunctionRef<bool(FName)> BoneValidator,
		float MinScoreThreshold)
	{
		if (!Mesh || BoneMap.Num() == 0)
			return TOptional<EOHSkeletalBone>();

		TArray<FString> Candidates;
		Algo::Transform(BoneMap, Candidates, [](const TPair<FName, EOHSkeletalBone>& Pair) {
			return Pair.Key.ToString();
		});

		const FOHNameMatchResult Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(RawInput, Candidates);
		if (Match.Score < MinScoreThreshold)
			return TOptional<EOHSkeletalBone>();

		const FName ResolvedName(*Match.Candidate);

		if (Mesh->GetBoneIndex(ResolvedName) == INDEX_NONE)
			return TOptional<EOHSkeletalBone>();

		if (!BoneValidator(ResolvedName))
			return TOptional<EOHSkeletalBone>();

		if (const EOHSkeletalBone* EnumValue = BoneMap.Find(ResolvedName))
			return *EnumValue;

		return TOptional<EOHSkeletalBone>();
	}
	//=====================================================================
	EOHSkeletalBone OHSkeletalMappings::GetSkeletalBoneFromFName(FName BoneName)
	{
		if (const EOHSkeletalBone* Found = FNameToSkeletalBoneMap.Find(BoneName))
		{
			return *Found;
		}

		TArray<FName> KnownNames;
		FNameToSkeletalBoneMap.GenerateKeyArray(KnownNames);
		FString Suggestion = OHSkeletalMappings::SuggestClosestBoneName_Levenshtein(BoneName, KnownNames);
		UE_LOG(LogTemp, Warning, TEXT("Unknown bone name: %s. Did you mean: %s?"), *BoneName.ToString(), *Suggestion);
		return EOHSkeletalBone::None;
	}
	FName OHSkeletalMappings::GetFNameFromSkeletalBone(EOHSkeletalBone Bone)
	{
		if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
		{
			return *Found;
		}
		return NAME_None;
	}

	const TArray<EOHBodyZone>& OHSkeletalMappings::GetBodyZonesFromBone(EOHSkeletalBone Bone)
	{
		static const TArray<EOHBodyZone> Empty;
		const TArray<EOHBodyZone>* Found = BoneToZonesMap.Find(Bone);
		return Found ? *Found : Empty;
	}

	const TArray<EOHBodyPart>& OHSkeletalMappings::GetBodyPartsFromBone(EOHSkeletalBone Bone)
	{
		static const TArray<EOHBodyPart> Empty;
		static TArray<EOHBodyPart> SinglePart;
		if (const EOHBodyPart* Found = BoneToBodyPartMap.Find(Bone))
		{
			SinglePart.SetNumUninitialized(1);
			SinglePart[0] = *Found;
			return SinglePart;
		}
		return Empty;
	}

	const TArray<EOHFunctionalBoneGroup>& OHSkeletalMappings::GetFunctionalGroupsFromBone(EOHSkeletalBone Bone)
	{
		static const TArray<EOHFunctionalBoneGroup> Empty;
		const TArray<EOHFunctionalBoneGroup>* Found = BoneToFunctionalGroupsMap.Find(Bone);
		return Found ? *Found : Empty;
	}

	const TArray<EOHSkeletalBone>& OHSkeletalMappings::GetBonesFromZone(EOHBodyZone Zone)
	{
		static const TArray<EOHSkeletalBone> Empty;
		const TArray<EOHSkeletalBone>* Found = ZoneToBonesMap.Find(Zone);
		return Found ? *Found : Empty;
	}

	const TArray<EOHSkeletalBone>& OHSkeletalMappings::GetBonesFromBodyPart(EOHBodyPart Part)
	{
		static const TArray<EOHSkeletalBone> Empty;
		const TArray<EOHSkeletalBone>* Found = BodyPartToBonesMap.Find(Part);
		return Found ? *Found : Empty;
	}

	TArray<FName> OHSkeletalMappings::GetPrimaryBoneNamesFromBodyPart(EOHBodyPart Part)
	{
		TArray<FName> Out;
		if (const TArray<EOHSkeletalBone>* Bones = BodyPartToPrimaryBonesMap.Find(Part))
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromBodyPartPreferPrimary(EOHBodyPart Part)
	{
		TArray<FName> Out;

		const TArray<EOHSkeletalBone>* Bones = BodyPartToPrimaryBonesMap.Find(Part);
		if (!Bones || Bones->IsEmpty())
		{
			Bones = BodyPartToBonesMap.Find(Part);
		}

		if (Bones)
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}

		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetPrimaryBoneNamesFromBodyZone(EOHBodyZone Zone)
	{
		TArray<FName> Out;
		if (const TArray<EOHSkeletalBone>* Bones = ZoneToPrimaryBonesMap.Find(Zone))
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromBodyZonePreferPrimary(EOHBodyZone Zone)
	{
		const TArray<EOHSkeletalBone>* Bones = ZoneToPrimaryBonesMap.Find(Zone);
		if (!Bones || Bones->IsEmpty())
		{
			Bones = ZoneToBonesMap.Find(Zone);
		}

		TArray<FName> Out;
		if (Bones)
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetPrimaryBoneNamesFromFunctionalGroup(EOHFunctionalBoneGroup Group)
	{
		TArray<FName> Out;
		if (const TArray<EOHSkeletalBone>* Bones = FunctionalGroupToPrimaryBonesMap.Find(Group))
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromFunctionalGroupPreferPrimary(EOHFunctionalBoneGroup Group)
	{
		const TArray<EOHSkeletalBone>* Bones = FunctionalGroupToPrimaryBonesMap.Find(Group);
		if (!Bones || Bones->IsEmpty())
		{
			Bones = FunctionalGroupToBonesMap.Find(Group);
		}

		TArray<FName> Out;
		if (Bones)
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromMapPreferPrimary(
	TFunction<const TArray<EOHSkeletalBone>*()> GetPrimary,
	TFunction<const TArray<EOHSkeletalBone>*()> GetFallback
)
	{
		const TArray<EOHSkeletalBone>* Bones = GetPrimary();
		if (!Bones || Bones->IsEmpty())
		{
			Bones = GetFallback();
		}

		TArray<FName> Out;
		if (Bones)
		{
			for (EOHSkeletalBone Bone : *Bones)
			{
				if (const FName* Found = SkeletalBoneToFNameMap.Find(Bone))
				{
					Out.Add(*Found);
				}
			}
		}
		return Out;
	}

	bool OHSkeletalMappings::TryParseSkeletalBone(const FString& Name, EOHSkeletalBone& Out)
	{
		const UEnum* Enum = StaticEnum<EOHSkeletalBone>();
		if (!Enum) return false;

		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i) // exclude _MAX
		{
			if (Enum->GetNameStringByIndex(i).Equals(Name, ESearchCase::IgnoreCase))
			{
				Out = static_cast<EOHSkeletalBone>(Enum->GetValueByIndex(i));
				return true;
			}
		}

		// Levenshtein fallback
		int32 BestScore = MAX_int32;
		EOHSkeletalBone BestMatch = EOHSkeletalBone::None;
		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			const FString EnumName = Enum->GetNameStringByIndex(i);
			const int32 Score = LevenshteinDistance(Name.ToLower(), EnumName.ToLower());
			if (Score < BestScore)
			{
				BestScore = Score;
				BestMatch = static_cast<EOHSkeletalBone>(Enum->GetValueByIndex(i));
			}
		}
		if (BestMatch != EOHSkeletalBone::None)
		{
			Out = BestMatch;
			return true;
		}

		return false;
	}
	
	bool OHSkeletalMappings::TryParseBodyPart(const FString& Name, EOHBodyPart& Out, bool bUseLevenshteinFallback)
	{
		const UEnum* Enum = StaticEnum<EOHBodyPart>();
		if (!Enum) return false;

		const FString InputLower = Name.ToLower();

		// Direct match
		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			if (Enum->GetNameStringByIndex(i).ToLower() == InputLower)
			{
				Out = static_cast<EOHBodyPart>(Enum->GetValueByIndex(i));
				return true;
			}
		}

		if (!bUseLevenshteinFallback) return false;

		// Levenshtein fallback
		int32 BestScore = MAX_int32;
		EOHBodyPart BestMatch = EOHBodyPart::None;

		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			const FString EnumName = Enum->GetNameStringByIndex(i).ToLower();
			const int32 Score = LevenshteinDistance(InputLower, EnumName);
			if (Score < BestScore)
			{
				BestScore = Score;
				BestMatch = static_cast<EOHBodyPart>(Enum->GetValueByIndex(i));
			}
		}

		if (BestMatch != EOHBodyPart::None && BestScore < 4)
		{
			Out = BestMatch;
			return true;
		}

		return false;
	}

	bool OHSkeletalMappings::TryParseBodyZone(const FString& Name, EOHBodyZone& Out)
	{
		const UEnum* Enum = StaticEnum<EOHBodyZone>();
		if (!Enum) return false;

		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			if (Enum->GetNameStringByIndex(i).Equals(Name, ESearchCase::IgnoreCase))
			{
				Out = static_cast<EOHBodyZone>(Enum->GetValueByIndex(i));
				return true;
			}
		}

		int32 BestScore = MAX_int32;
		EOHBodyZone BestMatch = EOHBodyZone::None;
		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			const FString EnumName = Enum->GetNameStringByIndex(i);
			const int32 Score = LevenshteinDistance(Name.ToLower(), EnumName.ToLower());
			if (Score < BestScore)
			{
				BestScore = Score;
				BestMatch = static_cast<EOHBodyZone>(Enum->GetValueByIndex(i));
			}
		}
		if (BestMatch != EOHBodyZone::None)
		{
			Out = BestMatch;
			return true;
		}

		return false;
	}

	bool OHSkeletalMappings::TryParseFunctionalGroup(const FString& Name, EOHFunctionalBoneGroup& Out)
	{
		const UEnum* Enum = StaticEnum<EOHFunctionalBoneGroup>();
		if (!Enum) return false;

		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			if (Enum->GetNameStringByIndex(i).Equals(Name, ESearchCase::IgnoreCase))
			{
				Out = static_cast<EOHFunctionalBoneGroup>(Enum->GetValueByIndex(i));
				return true;
			}
		}

		int32 BestScore = MAX_int32;
		EOHFunctionalBoneGroup BestMatch = EOHFunctionalBoneGroup::None;
		for (int32 i = 0; i < Enum->NumEnums() - 1; ++i)
		{
			const FString EnumName = Enum->GetNameStringByIndex(i);
			const int32 Score = LevenshteinDistance(Name.ToLower(), EnumName.ToLower());
			if (Score < BestScore)
			{
				BestScore = Score;
				BestMatch = static_cast<EOHFunctionalBoneGroup>(Enum->GetValueByIndex(i));
			}
		}
		if (BestMatch != EOHFunctionalBoneGroup::None)
		{
			Out = BestMatch;
			return true;
		}

		return false;
	}

	
	const TArray<EOHSkeletalBone>& OHSkeletalMappings::GetBonesFromFunctionalGroup(EOHFunctionalBoneGroup Group)
	{
		static const TArray<EOHSkeletalBone> Empty;
		const TArray<EOHSkeletalBone>* Found = FunctionalGroupToBonesMap.Find(Group);
		return Found ? *Found : Empty;
	}

	const TArray<EOHBodyPart>& OHSkeletalMappings::GetBodyPartsFromZone(EOHBodyZone Zone)
	{
		static const TArray<EOHBodyPart> Empty;
		if (const TArray<EOHBodyPart>* Found = ZoneToBodyPartsMap.Find(Zone))
		{
			return *Found;
		}
		return Empty;
	}

	const TArray<EOHBodyZone>& OHSkeletalMappings::GetZonesFromBodyPart(EOHBodyPart Part)
	{
		static const TArray<EOHBodyZone> Empty;
		if (const TArray<EOHBodyZone>* Found = BodyPartToZonesMap.Find(Part))
		{
			return *Found;
		}
		return Empty;
	}

	const TArray<EOHBodyZone>& OHSkeletalMappings::GetZonesFromFunctionalGroup(EOHFunctionalBoneGroup Group)
	{
		static const TArray<EOHBodyZone> Empty;
		if (const TArray<EOHBodyZone>* Found = FunctionalGroupToZonesMap.Find(Group))
		{
			return *Found;
		}
		return Empty;
	}

	const TArray<EOHBodyPart>& OHSkeletalMappings::GetBodyPartsFromFunctionalGroup(EOHFunctionalBoneGroup Group)
	{
		static const TArray<EOHBodyPart> Empty;
		if (const TArray<EOHBodyPart>* Found = FunctionalGroupToBodyPartsMap.Find(Group))
		{
			return *Found;
		}
		return Empty;
	}

	TArray<EOHSkeletalBone> OHSkeletalMappings::GetBonesFromZones(const TArray<EOHBodyZone>& Zones)
	{
		TArray<EOHSkeletalBone> Out;
		for (EOHBodyZone Zone : Zones)
		{
			Out.Append(OHSkeletalMappings::GetBonesFromZone(Zone));
		}
		return Out;
	}

	TArray<EOHSkeletalBone> OHSkeletalMappings::GetBonesFromBodyParts(const TArray<EOHBodyPart>& Parts)
	{
		TArray<EOHSkeletalBone> Out;
		for (EOHBodyPart Part : Parts)
		{
			Out.Append(OHSkeletalMappings::GetBonesFromBodyPart(Part));
		}
		return Out;
	}

	TArray<EOHSkeletalBone> OHSkeletalMappings::GetBonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups)
	{
		TArray<EOHSkeletalBone> Out;
		for (EOHFunctionalBoneGroup Group : Groups)
		{
			Out.Append(OHSkeletalMappings::GetBonesFromFunctionalGroup(Group));
		}
		return Out;
	}

	TArray<EOHBodyZone> OHSkeletalMappings::GetZonesFromBodyParts(const TArray<EOHBodyPart>& Parts)
	{
		TSet<EOHBodyZone> Unique;
		for (EOHBodyPart Part : Parts)
		{
			Unique.Append(OHSkeletalMappings::GetZonesFromBodyPart(Part));
		}
		return Unique.Array();
	}

	TArray<EOHBodyZone> OHSkeletalMappings::GetZonesFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups)
	{
		TSet<EOHBodyZone> Unique;
		for (EOHFunctionalBoneGroup Group : Groups)
		{
			Unique.Append(OHSkeletalMappings::GetZonesFromFunctionalGroup(Group));
		}
		return Unique.Array();
	}

	TArray<EOHBodyPart> OHSkeletalMappings::GetBodyPartsFromZones(const TArray<EOHBodyZone>& Zones)
	{
		TSet<EOHBodyPart> Unique;
		for (EOHBodyZone Zone : Zones)
		{
			Unique.Append(OHSkeletalMappings::GetBodyPartsFromZone(Zone));
		}
		return Unique.Array();
	}

	TArray<EOHBodyPart> OHSkeletalMappings::GetBodyPartsFromFunctionalGroups(const TArray<EOHFunctionalBoneGroup>& Groups)
	{
		TSet<EOHBodyPart> Unique;
		for (EOHFunctionalBoneGroup Group : Groups)
		{
			Unique.Append(OHSkeletalMappings::GetBodyPartsFromFunctionalGroup(Group));
		}
		return Unique.Array();
	}

	FString OHSkeletalMappings::ToString(EOHBodyPart Part)
	{
		return StaticEnum<EOHBodyPart>()->GetNameStringByValue(static_cast<int64>(Part));
	}

	FString OHSkeletalMappings::ToString(EOHBodyZone Zone)
	{
		return StaticEnum<EOHBodyZone>()->GetNameStringByValue(static_cast<int64>(Zone));
	}

	FString OHSkeletalMappings::ToString(EOHFunctionalBoneGroup Group)
	{
		return StaticEnum<EOHFunctionalBoneGroup>()->GetNameStringByValue(static_cast<int64>(Group));
	}

	FString OHSkeletalMappings::ToString(EOHSkeletalBone Bone)
	{
		return StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromBodyPart(EOHBodyPart BodyPart)
	{
		TArray<FName> OutNames;

		const TArray<EOHSkeletalBone>* Bones = BodyPartToBonesMap.Find(BodyPart);
		if (!Bones)
		{
			return OutNames;
		}

		for (EOHSkeletalBone Bone : *Bones)
		{
			if (const FName* BoneName = SkeletalBoneToFNameMap.Find(Bone))
			{
				OutNames.Add(*BoneName);
			}
		}

		return OutNames;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromBodyPart(EOHBodyPart BodyPart, bool bUseLevenshteinFallback)
	{
		TArray<FName> OutNames;

		const TArray<EOHSkeletalBone>* Bones = BodyPartToBonesMap.Find(BodyPart);
		if (!Bones)
		{
			return OutNames;
		}

		TArray<FName> ExistingNames;
		SkeletalBoneToFNameMap.GenerateValueArray(ExistingNames);

		for (EOHSkeletalBone Bone : *Bones)
		{
			if (const FName* FoundName = SkeletalBoneToFNameMap.Find(Bone))
			{
				OutNames.Add(*FoundName);
			}
			else if (bUseLevenshteinFallback)
			{
				FString BoneStr = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
				FName Suggestion = FName(*SuggestClosestBoneName_Levenshtein(FName(*BoneStr), ExistingNames));
				if (Suggestion != NAME_None)
				{
					OutNames.Add(Suggestion);
					UE_LOG(LogTemp, Warning, TEXT("Levenshtein fallback used: %s → %s"),
						*BoneStr, *Suggestion.ToString());
				}
			}
		}

		return OutNames;
	}


	TArray<FName> OHSkeletalMappings::GetBoneNamesFromBodyZone(EOHBodyZone Zone, bool bUseLevenshteinFallback)
	{
		TArray<FName> OutNames;
		const TArray<EOHSkeletalBone>* Bones = ZoneToBonesMap.Find(Zone);
		if (!Bones) return OutNames;

		TArray<FName> ExistingNames;
		SkeletalBoneToFNameMap.GenerateValueArray(ExistingNames);

		for (EOHSkeletalBone Bone : *Bones)
		{
			if (const FName* Name = SkeletalBoneToFNameMap.Find(Bone))
			{
				OutNames.Add(*Name);
			}
			else if (bUseLevenshteinFallback)
			{
				FName Suggestion = FName(*SuggestClosestBoneName_Levenshtein(FName(*StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone))), ExistingNames));
				if (Suggestion != NAME_None)
				{
					OutNames.Add(Suggestion);
				}
			}
		}
		return OutNames;
	}

	TArray<FName> OHSkeletalMappings::GetBoneNamesFromFunctionalGroup(EOHFunctionalBoneGroup Group, bool bUseLevenshteinFallback)
	{
		TArray<FName> OutNames;
		const TArray<EOHSkeletalBone>* Bones = FunctionalGroupToBonesMap.Find(Group);
		if (!Bones) return OutNames;

		TArray<FName> ExistingNames;
		SkeletalBoneToFNameMap.GenerateValueArray(ExistingNames);

		for (EOHSkeletalBone Bone : *Bones)
		{
			if (const FName* Name = SkeletalBoneToFNameMap.Find(Bone))
			{
				OutNames.Add(*Name);
			}
			else if (bUseLevenshteinFallback)
			{
				FName Suggestion = FName(*SuggestClosestBoneName_Levenshtein(FName(*StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone))), ExistingNames));
				if (Suggestion != NAME_None)
				{
					OutNames.Add(Suggestion);
				}
			}
		}
		return OutNames;
	}

	FName OHSkeletalMappings::GetMostLikelyBoneNameFromGroup(EOHFunctionalBoneGroup Group, bool bVerbose)
	{
		TMap<EOHSkeletalBone, int32> ScoreMap;

		const FString GroupName = StaticEnum<EOHFunctionalBoneGroup>()->GetNameStringByValue(static_cast<int64>(Group));
		const TArray<EOHSkeletalBone>* AllBones = FunctionalGroupToBonesMap.Find(Group);

		if (const TArray<EOHSkeletalBone>* PrimaryBones = FunctionalGroupToPrimaryBonesMap.Find(Group))
		{
			TSet<EOHSkeletalBone> PrimarySet;
			for (EOHSkeletalBone Bone : *PrimaryBones)
			{
				ScoreMap.Add(Bone, 1000);  // primary gets high base score
				PrimarySet.Add(Bone);
			}
		}

		if (AllBones)
		{
			for (EOHSkeletalBone Bone : *AllBones)
			{
				int32 Score = ScoreMap.Contains(Bone) ? ScoreMap[Bone] : 0;

				// Add score based on name similarity
				const FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
				int32 LevenshteinScore = 100 - LevenshteinDistance(GroupName, BoneName);
				Score += LevenshteinScore;

				// Weight based on anatomical order
				if (BoneName.Contains(TEXT("Upper"))) Score += 25;
				if (BoneName.Contains(TEXT("Lower"))) Score += 20;
				if (BoneName.Contains(TEXT("Hand")))  Score += 15;
				if (BoneName.Contains(TEXT("Foot")))  Score += 15;
				if (BoneName.Contains(TEXT("Thigh"))) Score += 10;
				if (BoneName.Contains(TEXT("Calf")))  Score += 10;

				ScoreMap.Add(Bone, Score);
			}
		}

		if (ScoreMap.Num() == 0) return NAME_None;

		// Find highest score
		EOHSkeletalBone BestBone = EOHSkeletalBone::None;
		int32 BestScore = MIN_int32;

		for (const auto& Pair : ScoreMap)
		{
			if (Pair.Value > BestScore)
			{
				BestScore = Pair.Value;
				BestBone = Pair.Key;
			}
		}

		if (bVerbose)
		{
			UE_LOG(LogTemp, Log, TEXT("Most likely bone for group %s is %s with score %d"),
				   *GroupName, *StaticEnum<EOHSkeletalBone>()->GetNameStringByValue((int64)BestBone), BestScore);
		}

		const FName* BoneFName = SkeletalBoneToFNameMap.Find(BestBone);
		return BoneFName ? *BoneFName : NAME_None;
	}

	FName OHSkeletalMappings::GetMostLikelyBoneNameFromBodyPart(EOHBodyPart BodyPart, bool bVerbose)
	{
		TMap<EOHSkeletalBone, int32> ScoreMap;

		const FString PartName = StaticEnum<EOHBodyPart>()->GetNameStringByValue(static_cast<int64>(BodyPart));
		const TArray<EOHSkeletalBone>* AllBones = BodyPartToBonesMap.Find(BodyPart);

		if (const TArray<EOHSkeletalBone>* PrimaryBones = BodyPartToPrimaryBonesMap.Find(BodyPart))
		{
			TSet<EOHSkeletalBone> PrimarySet;
			for (EOHSkeletalBone Bone : *PrimaryBones)
			{
				ScoreMap.Add(Bone, 1000);  // high weight for primary
				PrimarySet.Add(Bone);
			}
		}

		if (AllBones)
		{
			for (EOHSkeletalBone Bone : *AllBones)
			{
				int32 Score = ScoreMap.Contains(Bone) ? ScoreMap[Bone] : 0;

				const FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
				int32 LevenshteinScore = 100 - LevenshteinDistance(PartName, BoneName);
				Score += LevenshteinScore;

				// Heuristics based on keywords
				if (BoneName.Contains(TEXT("Upper"))) Score += 25;
				if (BoneName.Contains(TEXT("Lower"))) Score += 20;
				if (BoneName.Contains(TEXT("Hand")))  Score += 15;
				if (BoneName.Contains(TEXT("Foot")))  Score += 15;
				if (BoneName.Contains(TEXT("Thigh"))) Score += 10;
				if (BoneName.Contains(TEXT("Calf")))  Score += 10;

				ScoreMap.Add(Bone, Score);
			}
		}

		if (ScoreMap.Num() == 0) return NAME_None;

		EOHSkeletalBone BestBone = EOHSkeletalBone::None;
		int32 BestScore = MIN_int32;

		for (const auto& Pair : ScoreMap)
		{
			if (Pair.Value > BestScore)
			{
				BestScore = Pair.Value;
				BestBone = Pair.Key;
			}
		}

		if (bVerbose)
		{
			UE_LOG(LogTemp, Log, TEXT("Most likely bone for body part %s is %s with score %d"),
				   *PartName, *StaticEnum<EOHSkeletalBone>()->GetNameStringByValue((int64)BestBone), BestScore);
		}

		const FName* BoneFName = SkeletalBoneToFNameMap.Find(BestBone);
		return BoneFName ? *BoneFName : NAME_None;
	}

	FName OHSkeletalMappings::GetMostLikelyBoneNameFromBodyZone(EOHBodyZone Zone, bool bVerbose)
	{
		TMap<EOHSkeletalBone, int32> ScoreMap;

		const FString ZoneName = StaticEnum<EOHBodyZone>()->GetNameStringByValue(static_cast<int64>(Zone));
		const TArray<EOHSkeletalBone>* AllBones = ZoneToBonesMap.Find(Zone);

		if (const TArray<EOHSkeletalBone>* PrimaryBones = ZoneToPrimaryBonesMap.Find(Zone))
		{
			TSet<EOHSkeletalBone> PrimarySet;
			for (EOHSkeletalBone Bone : *PrimaryBones)
			{
				ScoreMap.Add(Bone, 1000);
				PrimarySet.Add(Bone);
			}
		}

		if (AllBones)
		{
			for (EOHSkeletalBone Bone : *AllBones)
			{
				int32 Score = ScoreMap.Contains(Bone) ? ScoreMap[Bone] : 0;

				const FString BoneName = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
				Score += 100 - LevenshteinDistance(ZoneName, BoneName);

				if (BoneName.Contains(TEXT("Upper"))) Score += 25;
				if (BoneName.Contains(TEXT("Lower"))) Score += 20;
				if (BoneName.Contains(TEXT("Hand")))  Score += 15;
				if (BoneName.Contains(TEXT("Foot")))  Score += 15;
				if (BoneName.Contains(TEXT("Thigh"))) Score += 10;
				if (BoneName.Contains(TEXT("Calf")))  Score += 10;

				ScoreMap.Add(Bone, Score);
			}
		}

		if (ScoreMap.Num() == 0) return NAME_None;

		EOHSkeletalBone BestBone = EOHSkeletalBone::None;
		int32 BestScore = MIN_int32;

		for (const auto& Pair : ScoreMap)
		{
			if (Pair.Value > BestScore)
			{
				BestScore = Pair.Value;
				BestBone = Pair.Key;
			}
		}

		if (bVerbose)
		{
			UE_LOG(LogTemp, Log, TEXT("Most likely bone for zone %s is %s with score %d"),
				   *ZoneName, *StaticEnum<EOHSkeletalBone>()->GetNameStringByValue((int64)BestBone), BestScore);
		}

		const FName* BoneFName = SkeletalBoneToFNameMap.Find(BestBone);
		return BoneFName ? *BoneFName : NAME_None;
	}

	TArray<FName> OHSkeletalMappings::GetMostLikelyBoneNamesFromBodyParts(const TArray<EOHBodyPart>& Parts, bool bVerbose)
	{
		TArray<FName> Out;
		for (EOHBodyPart Part : Parts)
		{
			const FName Name = GetMostLikelyBoneNameFromBodyPart(Part, bVerbose);
			if (Name != NAME_None)
			{
				Out.Add(Name);
			}
		}
		return Out;
	}

	TArray<FName> OHSkeletalMappings::GetMostLikelyBoneNamesFromBodyZones(const TArray<EOHBodyZone>& Zones, bool bVerbose)
	{
		TArray<FName> Out;
		for (EOHBodyZone Zone : Zones)
		{
			const FName Name = GetMostLikelyBoneNameFromBodyZone(Zone, bVerbose);
			if (Name != NAME_None)
			{
				Out.Add(Name);
			}
		}
		return Out;
	}

	FName OHSkeletalMappings::GetMostLikelyBoneName(EOHBodyPart Part, EOHBodyZone Zone, EOHFunctionalBoneGroup Group, bool bVerbose)
	{
		TSet<EOHSkeletalBone> CandidateBones;

		if (const TArray<EOHSkeletalBone>* Bones = BodyPartToBonesMap.Find(Part))
		{
			CandidateBones.Append(*Bones);
		}
		if (const TArray<EOHSkeletalBone>* Bones = ZoneToBonesMap.Find(Zone))
		{
			CandidateBones.Append(*Bones);
		}
		if (const TArray<EOHSkeletalBone>* Bones = FunctionalGroupToBonesMap.Find(Group))
		{
			CandidateBones.Append(*Bones);
		}

		TMap<EOHSkeletalBone, int32> ScoreMap;
		FString PartStr = StaticEnum<EOHBodyPart>()->GetNameStringByValue(static_cast<int64>(Part));
		FString ZoneStr = StaticEnum<EOHBodyZone>()->GetNameStringByValue(static_cast<int64>(Zone));
		FString GroupStr = StaticEnum<EOHFunctionalBoneGroup>()->GetNameStringByValue(static_cast<int64>(Group));

		for (EOHSkeletalBone Bone : CandidateBones)
		{
			FString BoneStr = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
			int32 Score = 0;

			if (PrimaryBoneToBodyPartsMap.Find(Bone) && PrimaryBoneToBodyPartsMap[Bone].Contains(Part)) Score += 1000;
			if (ZoneToPrimaryBonesMap.Find(Zone) && ZoneToPrimaryBonesMap[Zone].Contains(Bone)) Score += 1000;
			if (FunctionalGroupToPrimaryBonesMap.Find(Group) && FunctionalGroupToPrimaryBonesMap[Group].Contains(Bone)) Score += 1000;

			Score += 100 - LevenshteinDistance(PartStr, BoneStr);
			Score += 100 - LevenshteinDistance(ZoneStr, BoneStr);
			Score += 100 - LevenshteinDistance(GroupStr, BoneStr);

			ScoreMap.Add(Bone, Score);
		}

		EOHSkeletalBone BestBone = EOHSkeletalBone::None;
		int32 BestScore = MIN_int32;

		for (auto& Pair : ScoreMap)
		{
			if (Pair.Value > BestScore)
			{
				BestScore = Pair.Value;
				BestBone = Pair.Key;
			}
		}

		if (bVerbose)
		{
			UE_LOG(LogTemp, Log, TEXT("Best guess: %s (score %d)"),
				   *StaticEnum<EOHSkeletalBone>()->GetNameStringByValue((int64)BestBone), BestScore);
		}

		return SkeletalBoneToFNameMap.Contains(BestBone) ? SkeletalBoneToFNameMap[BestBone] : NAME_None;
	}

	FName OHSkeletalMappings::GetPreferredBoneNameFromBodyPart(EOHBodyPart Part, EOHBodyZone Zone, EOHFunctionalBoneGroup Group, bool bVerbose)
	{
		TSet<EOHSkeletalBone> CandidateBones;

		if (const TArray<EOHSkeletalBone>* Bones = BodyPartToBonesMap.Find(Part))
		{
			CandidateBones.Append(*Bones);
		}
		if (const TArray<EOHSkeletalBone>* ZoneBones = ZoneToBonesMap.Find(Zone))
		{
			CandidateBones.Append(*ZoneBones);
		}
		if (const TArray<EOHSkeletalBone>* GroupBones = FunctionalGroupToBonesMap.Find(Group))
		{
			CandidateBones.Append(*GroupBones);
		}

		TMap<EOHSkeletalBone, int32> ScoreMap;
		const FString PartStr = StaticEnum<EOHBodyPart>()->GetNameStringByValue(static_cast<int64>(Part));
		const FString ZoneStr = StaticEnum<EOHBodyZone>()->GetNameStringByValue(static_cast<int64>(Zone));
		const FString GroupStr = StaticEnum<EOHFunctionalBoneGroup>()->GetNameStringByValue(static_cast<int64>(Group));

		for (EOHSkeletalBone Bone : CandidateBones)
		{
			FString BoneStr = StaticEnum<EOHSkeletalBone>()->GetNameStringByValue(static_cast<int64>(Bone));
			int32 Score = 0;

			// Prioritize Body Part
			if (BodyPartToPrimaryBonesMap.Contains(Part) && BodyPartToPrimaryBonesMap[Part].Contains(Bone))
				Score += 1500;
			if (BodyPartToBonesMap.Contains(Part) && BodyPartToBonesMap[Part].Contains(Bone))
				Score += 1000;

			// Bonus for matching primary relationships
			if (ZoneToPrimaryBonesMap.Contains(Zone) && ZoneToPrimaryBonesMap[Zone].Contains(Bone))
				Score += 750;
			if (FunctionalGroupToPrimaryBonesMap.Contains(Group) && FunctionalGroupToPrimaryBonesMap[Group].Contains(Bone))
				Score += 750;

			// Fuzzy similarity
			Score += 100 - LevenshteinDistance(PartStr, BoneStr);
			Score += 50 - LevenshteinDistance(ZoneStr, BoneStr);
			Score += 50 - LevenshteinDistance(GroupStr, BoneStr);

			ScoreMap.Add(Bone, Score);
		}

		EOHSkeletalBone BestBone = EOHSkeletalBone::None;
		int32 BestScore = MIN_int32;

		for (const auto& Pair : ScoreMap)
		{
			if (Pair.Value > BestScore)
			{
				BestScore = Pair.Value;
				BestBone = Pair.Key;
			}
		}

		if (bVerbose)
		{
			UE_LOG(LogTemp, Log, TEXT("Preferred Bone Guess for %s (Zone: %s, Group: %s) is %s (Score: %d)"),
				   *PartStr, *ZoneStr, *GroupStr,
				   *StaticEnum<EOHSkeletalBone>()->GetNameStringByValue((int64)BestBone), BestScore);
		}

		return SkeletalBoneToFNameMap.Contains(BestBone) ? SkeletalBoneToFNameMap[BestBone] : NAME_None;
	}

	
	// Utilities
	int32 OHSkeletalMappings::LevenshteinDistance(const FString& A, const FString& B)
	{
		const int32 LenA = A.Len();
		const int32 LenB = B.Len();

		TArray<int32> PrevRow, CurrRow;
		PrevRow.SetNumZeroed(LenB + 1);
		CurrRow.SetNumZeroed(LenB + 1);

		for (int32 j = 0; j <= LenB; ++j)
		{
			PrevRow[j] = j;
		}

		for (int32 i = 1; i <= LenA; ++i)
		{
			CurrRow[0] = i;
			for (int32 j = 1; j <= LenB; ++j)
			{
				const int32 Cost = A[i - 1] == B[j - 1] ? 0 : 1;
				CurrRow[j] = FMath::Min3(
					CurrRow[j - 1] + 1,
					PrevRow[j] + 1,
					PrevRow[j - 1] + Cost
				);
			}
			PrevRow = CurrRow;
		}
		return CurrRow[LenB];
	}

	FString OHSkeletalMappings::SuggestClosestBoneName_Levenshtein(const FName& MissingBoneName, const TArray<FName>& ExistingBoneNames)
	{
		int32 BestScore = MAX_int32;
		FName BestMatch = NAME_None;

		for (const FName& ExistingName : ExistingBoneNames)
		{
			int32 Score = LevenshteinDistance(MissingBoneName.ToString(), ExistingName.ToString());
			if (Score < BestScore)
			{
				BestScore = Score;
				BestMatch = ExistingName;
			}
		}

		return BestMatch != NAME_None ? BestMatch.ToString() : TEXT("No suggestion");
	}

	void OHSkeletalMappings::GetBonesMatching(
	TOptional<EOHBodyZone> Zone,
	TOptional<EOHBodyPart> Part,
	TOptional<EOHFunctionalBoneGroup> Group,
	bool bRequirePrimary,
	TArray<EOHSkeletalBone>& OutBones)
	{
		OutBones.Reset();

		for (int32 Index = 0; Index < static_cast<int32>(EOHSkeletalBone::LastBone); ++Index)
		{
			const EOHSkeletalBone Bone = static_cast<EOHSkeletalBone>(Index);

			if (bRequirePrimary && !GetPrimarySkeletalBones().Contains(Bone))
				continue;

			if (Zone.IsSet() && GetBodyZoneFromBone(Bone) != Zone.GetValue())
				continue;

			if (Part.IsSet() && GetBodyPartFromBone(Bone) != Part.GetValue())
				continue;

			if (Group.IsSet())
			{
				const TArray<EOHFunctionalBoneGroup>& Groups = GetFunctionalGroupsFromBone(Bone);
				if (!Groups.Contains(Group.GetValue()))
					continue;
			}

			OutBones.Add(Bone);
		}
	}

	bool OHSkeletalMappings::IsBoneValidForSimulation(FName Bone, const USkeletalMeshComponent* Mesh, const UOHPhysicsManager* Manager)
	{
		if (!Mesh) return false;

		if (Manager && Manager->SimExclusionBoneSet.Contains(Bone))
			return false;

		const int32 Index = Mesh->GetBoneIndex(Bone);
		if (Index == INDEX_NONE)
			return false;

		const FString BoneStr = Bone.ToString().ToLower();
		if (BoneStr.Contains(TEXT("twist")) ||
			BoneStr.Contains(TEXT("ik")) ||
			BoneStr.Contains(TEXT("attach")) ||
			BoneStr.Contains(TEXT("helper")) ||
			BoneStr.StartsWith(TEXT("vb_")) ||
			BoneStr.EndsWith(TEXT("_dummy")))
		{
			return false;
		}

		UPhysicsAsset* Asset = Mesh->GetPhysicsAsset();
		if (!Asset || Asset->FindBodyIndex(Bone) == INDEX_NONE)
			return false;

		if (FBodyInstance* Body = Mesh->GetBodyInstance(Bone))
		{
			return Body->IsValidBodyInstance() && Body->GetBodyMass() > KINDA_SMALL_NUMBER;
		}

		return false;
	}

	bool OHSkeletalMappings::IsBoneValidForTracking(FName Bone, const USkeletalMeshComponent* Mesh)
	{
		if (!Mesh) return false;

		const int32 Index = Mesh->GetBoneIndex(Bone);
		if (Index == INDEX_NONE)
			return false;

		const FString BoneStr = Bone.ToString().ToLower();
		if (BoneStr.Contains(TEXT("twist")) ||
			BoneStr.Contains(TEXT("ik")) ||
			BoneStr.Contains(TEXT("attach")) ||
			BoneStr.Contains(TEXT("helper")) ||
			BoneStr.StartsWith(TEXT("vb_")) ||
			BoneStr.EndsWith(TEXT("_dummy")))
		{
			return false;
		}

		return true;
	}

		EOHSkeletalBone OHSkeletalMappings::ResolveSkeletalBoneFromNameSmart_Trackable(
	FName RawName,
	const USkeletalMeshComponent* Mesh,
	float MinScoreThreshold)
	{
		const TOptional<EOHSkeletalBone> Result = TryResolveBoneWithFilter(
			RawName.ToString(),
			FNameToSkeletalBoneMap,
			Mesh,
			[=](FName Bone) {
				return IsBoneValidForTracking(Bone, Mesh);
			},
			MinScoreThreshold);

		return Result.Get(EOHSkeletalBone::None);
	}

	EOHSkeletalBone OHSkeletalMappings::ResolveSkeletalBoneFromNameSmart_Simulatable(
	FName RawName,
	const USkeletalMeshComponent* Mesh,
	const UOHPhysicsManager* PhysicsManager,
	float MinScoreThreshold)
	{
		const TOptional<EOHSkeletalBone> Result = TryResolveBoneWithFilter(
			RawName.ToString(),
			FNameToSkeletalBoneMap,
			Mesh,
			[=](FName Bone)
			{
				return IsBoneValidForSimulation(Bone, Mesh, PhysicsManager);
			},
			MinScoreThreshold);

		return Result.Get(EOHSkeletalBone::None);
	}
	
	EOHSkeletalBone OHSkeletalMappings::ResolveSkeletalBoneFromSocketSmart_Simulatable(
		FName SocketName,
		const USkeletalMeshComponent* Mesh,
		const UOHPhysicsManager* PhysicsManager,
		float MinScoreThreshold)
	{
		if (!Mesh) return EOHSkeletalBone::None;

		const FName Bone = Mesh->GetSocketBoneName(SocketName);
		if (Bone == NAME_None)
			return EOHSkeletalBone::None;

		// Run smart resolution on the socket's attached bone name
		return ResolveSkeletalBoneFromNameSmart_Simulatable(Bone, Mesh, PhysicsManager, MinScoreThreshold);
	}
	
	EOHBodyZone OHSkeletalMappings::ResolveBodyZoneFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold)
	{
		const EOHSkeletalBone Bone = ResolveSkeletalBoneFromNameSmart(RawName, Mesh, MinScoreThreshold);
		return BoneToZoneMap.FindRef(Bone);
	}

	EOHBodyPart OHSkeletalMappings::ResolveBodyPartFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold)
	{
		const EOHSkeletalBone Bone = OHSkeletalMappings::ResolveSkeletalBoneFromNameSmart(RawName, Mesh, MinScoreThreshold);
		return OHSkeletalMappings::BoneToBodyPartMap.FindRef(Bone);
	}

	EOHFunctionalBoneGroup OHSkeletalMappings::ResolveFunctionalGroupFromNameSmart(FName RawName, const USkeletalMeshComponent* Mesh, float MinScoreThreshold)
	{
		const EOHSkeletalBone Bone = OHSkeletalMappings::ResolveSkeletalBoneFromNameSmart(RawName, Mesh, MinScoreThreshold);
		return OHSkeletalMappings::BoneToFunctionalGroupMap.FindRef(Bone);
	
	}

#endif
