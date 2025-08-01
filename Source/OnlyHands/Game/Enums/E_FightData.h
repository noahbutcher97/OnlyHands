// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "E_FightData.generated.h"

UENUM(BlueprintType)
namespace Enm_Side {
enum Enm_Side {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Front UMETA(DisplayName = "Front"),
    ENM_Back UMETA(DisplayName = "Back"),
    ENM_Left UMETA(DisplayName = "Left"),
    ENM_Right UMETA(DisplayName = "Right"),
};
}

UENUM(BlueprintType)
namespace Enm_AttackSideBody {
enum Enm_AttackSideBody {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Up UMETA(DisplayName = "Up"),
    ENM_Down UMETA(DisplayName = "Down"),
    ENM_Left UMETA(DisplayName = "Left"),
    ENM_Right UMETA(DisplayName = "Right"),
};
}

UENUM(BlueprintType)
namespace Enm_SkillAbillity {
enum Enm_SkillAbillity {
    ENM_None UMETA(DisplayName = "None"),
    ENM_AddDamage UMETA(DisplayName = "AddDamage"),
    ENM_AddBody UMETA(DisplayName = "AddBody"),
    ENM_AddSpeed UMETA(DisplayName = "AddSpeed"),
    ENM_AddJump UMETA(DisplayName = "AddJump"),
    ENM_AddPower UMETA(DisplayName = "AddPower"),
};
}

UENUM(BlueprintType)
namespace Enm_IntensityState {
enum Enm_IntensityState {
    ENM_None UMETA(DisplayName = "None"),
    ENM_VeryLow UMETA(DisplayName = "VeryLow"),
    ENM_Low UMETA(DisplayName = "Low"),
    ENM_Medium UMETA(DisplayName = "Medium"),
    ENM_High UMETA(DisplayName = "High"),
    ENM_VeryHigh UMETA(DisplayName = "VeryHigh"),
    ENM_ExtremeHigh UMETA(DisplayName = "ExtremeHigh"),
};
}

// BODY STATE

UENUM(BlueprintType)
namespace Enm_StateType {
enum Enm_StateType {
    ENM_None UMETA(DisplayName = "None"),
    ENM_STAND UMETA(DisplayName = "Stand"),
    ENM_CROUCH UMETA(DisplayName = "Crouch"),
    ENM_AIR UMETA(DisplayName = "Air"),
};
}

UENUM(BlueprintType)
namespace Enm_MoveType {
enum Enm_MoveType {
    ENM_None UMETA(DisplayName = "None"),
    ENM_IDLE UMETA(DisplayName = "Idle"),
    ENM_ATTACK UMETA(DisplayName = "Attack"),
    ENM_HIT UMETA(DisplayName = "Hit"),
    ENM_WINNER UMETA(DisplayName = "Winner"),
    ENM_DEATH UMETA(DisplayName = "Death"),
    ENM_NULL UMETA(DisplayName = "Null"),
    ENM_DFENSE UMETA(DisplayName = "Defense")
};
}

// PHYSIC
UENUM(BlueprintType)
namespace Enm_PhysicState {
enum Enm_PhysicState {
    ENM_None UMETA(DisplayName = "None"),
    ENM_GROUND UMETA(DisplayName = "Ground"),
    ENM_CROUCH UMETA(DisplayName = "Crouch"),
    ENM_AIR UMETA(DisplayName = "Air"),
    ENM_FLY UMETA(DisplayName = "Fly"),
    ENM_NULL UMETA(DisplayName = "Null"),
};
}

// HIT
UENUM(BlueprintType)
namespace Enm_HitPossibility {
enum Enm_HitPossibility {
    ENM_None UMETA(DisplayName = "None"),
    ENM_S UMETA(DisplayName = "S"),
    ENM_C UMETA(DisplayName = "C"),
    ENM_A UMETA(DisplayName = "A"),
    ENM_SC UMETA(DisplayName = "SC"),
    ENM_SA UMETA(DisplayName = "SA"),
    ENM_SCA UMETA(DisplayName = "SCA"),
    ENM_AC UMETA(DisplayName = "AC"),
};
}
UENUM(BlueprintType)
namespace Enm_HitBodyParts {
enum Enm_HitBodyParts {
    ENM_None UMETA(DisplayName = "None"),
    // Head
    ENM_StraightHead UMETA(DisplayName = "StraightHead"),
    ENM_LeftHead UMETA(DisplayName = "LeftHead"),
    ENM_RightHead UMETA(DisplayName = "RightHead"),
    ENM_BottomHead UMETA(DisplayName = "BottomHead"),
    // Shoulder
    ENM_LeftShoulder UMETA(DisplayName = "LeftShoulder"),
    ENM_RightShoulder UMETA(DisplayName = "RightShoulder"),
    // Arm
    ENM_LeftArm UMETA(DisplayName = "LeftArm"),
    ENM_RightArm UMETA(DisplayName = "RightArm"),
    // Hand
    ENM_LeftHand UMETA(DisplayName = "LeftHand"),
    ENM_RightHand UMETA(DisplayName = "RightHand"),
    // Chess
    ENM_StraightChest UMETA(DisplayName = "StraightChest"),
    ENM_LeftChest UMETA(DisplayName = "LeftChest"),
    ENM_RightChest UMETA(DisplayName = "RightChest"),
    // Leg
    ENM_LeftLeg UMETA(DisplayName = "LeftLeg"),
    ENM_RightLeg UMETA(DisplayName = "LeftLeg"),
};
}

UENUM(BlueprintType)
namespace Enm_ComboCategory {
enum Enm_ComboCategory {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Start UMETA(DisplayName = "Start"),
    ENM_Midle UMETA(DisplayName = "Midle"),
    ENM_High UMETA(DisplayName = "High"),
    ENM_Finish UMETA(DisplayName = "Finish"),
};
}

UENUM(BlueprintType)
namespace Enm_ComboState {
enum Enm_ComboState {
    ENM_None UMETA(DisplayName = "None"),
    ENM_1_OFF UMETA(DisplayName = "1_OFF"),
    ENM_2_ON UMETA(DisplayName = "2_ON"),
    ENM_3_END UMETA(DisplayName = "3_END"),
};
}
UENUM(BlueprintType)
namespace Enm_FrameState {
enum Enm_ComEnm_FrameStateboState {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Startup UMETA(DisplayName = "Startup"),
    ENM_Activated UMETA(DisplayName = "Activated"),
    ENM_Recovery UMETA(DisplayName = "Recovery"),
};
}

UENUM(BlueprintType)
namespace Enm_AIMovements {
enum Enm_AIMovements {
    ENM_Walk UMETA(DisplayName = "Walk"),
    ENM_Doge UMETA(DisplayName = "Doge"),
    ENM_Jump UMETA(DisplayName = "Jump"),
    ENM_Dash UMETA(DisplayName = "Dash"),
    ENM_Roll UMETA(DisplayName = "Roll"),
    ENM_Slip UMETA(DisplayName = "Slip"),
    ENM_Step UMETA(DisplayName = "Step"),
};
}

UENUM(BlueprintType)
namespace Enm_AIDirectionMov {
enum Enm_AIDirectionMov {
    ENM_Forward UMETA(DisplayName = "Forward"),
    ENM_Backward UMETA(DisplayName = "Backward"),
    ENM_Left UMETA(DisplayName = "Left"),
    ENM_Right UMETA(DisplayName = "Right"),
    ENM_FrdLeft UMETA(DisplayName = "FrdLeft"),
    ENM_FrdRight UMETA(DisplayName = "FrdRight"),
    ENM_BckLeft UMETA(DisplayName = "BckLeft"),
    ENM_BckRight UMETA(DisplayName = "BckRight"),
    ENM_RightBack UMETA(DisplayName = "RightBack"),
};
}

UENUM(BlueprintType)
namespace Enm_AIDistance {
enum Enm_AIDistance {
    ENM_Closed UMETA(DisplayName = "Closed"),
    ENM_Midle UMETA(DisplayName = "Midle"),
    ENM_Far UMETA(DisplayName = "Far"),
};
}

// AI Setup //==================================

UENUM(BlueprintType)
namespace Enm_AI_MoveSpace {
enum Enm_AI_MoveSpace {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Closed UMETA(DisplayName = "Closed"),
    ENM_Medium UMETA(DisplayName = "Medium"),
    ENM_Far UMETA(DisplayName = "Far"),
};
}

UENUM(BlueprintType)
namespace Enm_AI_AttackStyle {
enum Enm_AI_AttackStyle {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Agressive UMETA(DisplayName = "Agressive"),
    ENM_Punish UMETA(DisplayName = "Punish"),
    ENM_RunAway UMETA(DisplayName = "RunAway"),
    ENM_Mimics UMETA(DisplayName = "Mimics"),
    ENM_Stance UMETA(DisplayName = "Stance"),
};
}

UENUM(BlueprintType)
namespace Enm_AI_Intesity {
enum Enm_AI_Intesity {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Low UMETA(DisplayName = "Low"),
    ENM_Mix UMETA(DisplayName = "Mix"),
    ENM_Intesity UMETA(DisplayName = "Intesity"),
};
}
UENUM(BlueprintType)
namespace Enm_AI_BodyPart {
enum Enm_AI_BodyPart {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Head UMETA(DisplayName = "Head"),
    ENM_MidChess UMETA(DisplayName = "MidChess"),
    ENM_LeftBody UMETA(DisplayName = "LeftBody"),
    ENM_RightBody UMETA(DisplayName = "RightBody"),
};
}

UENUM(BlueprintType)
namespace Enm_AttackLevel {
enum Enm_AttackLevel {
    ENM_None UMETA(DisplayName = "None"),
    ENM_LIGHT UMETA(DisplayName = "LIGHT"),
    ENM_NORMAL UMETA(DisplayName = "NORMAL"),
    ENM_HEAVY UMETA(DisplayName = "HEAVY"),
};
}

UENUM(BlueprintType)
namespace Enm_Defense_Types {
enum Enm_Defense_Types {
    ENM_None UMETA(DisplayName = "None"),
    ENM_PBlock UMETA(DisplayName = "PBlock"),
    ENM_PEvade UMETA(DisplayName = "PEvade"),
    ENM_PPivot UMETA(DisplayName = "PPivot"),

};
}

UENUM(BlueprintType)
namespace Enm_MomentumPoints {
enum Enm_MomentumPoints {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Light_AttackHit UMETA(DisplayName = "Light_AttackHit"),
    ENM_Medium_AttackHit UMETA(DisplayName = "Medium_AttackHit"),
    ENM_Heavy_AttackHit UMETA(DisplayName = "Heavy_AttackHit"),
    ENM_Combo_ChainPlus UMETA(DisplayName = "Combo_ChainPlus"),
    ENM_Critical_Hit UMETA(DisplayName = "Critical_Hit"),
    ENM_Perfect_Parry UMETA(DisplayName = "Perfect_Parry"),
    ENM_Perfect_Counter UMETA(DisplayName = "Perfect_Counter"),
    ENM_Kndown_StaggerInfli UMETA(DisplayName = "Kndown_StaggerInfli"),
    ENM_Perfect_Evasion UMETA(DisplayName = "Perfect_Evasion"),
    ENM_Light_HitTaken UMETA(DisplayName = "Light_HitTaken"),
    ENM_Medium_HitTaken UMETA(DisplayName = "Medium_HitTaken"),
    ENM_Heavy_HitTaken UMETA(DisplayName = "Heavy_HitTaken"),
    ENM_Perfect_Block UMETA(DisplayName = "Perfect_Block"),
    ENM_NOT_Perfect_Block UMETA(DisplayName = "NOT_Perfect_Block"),
    ENM_Whiffed_Heavy UMETA(DisplayName = "Whiffed_Heavy"),
    ENM_Staggered_KnockedDown UMETA(DisplayName = "Staggered_KnockedDown"),
    ENM_Passive_Idle UMETA(DisplayName = "Passive_Idle"),

};
}

UENUM(BlueprintType)
<<<<<<< HEAD
namespace Enm_Defense_Types {
enum Enm_Defense_Types {
    ENM_None UMETA(DisplayName = "None"),
    ENM_PBlock UMETA(DisplayName = "PBlock"),
    ENM_PEvade UMETA(DisplayName = "PEvade"),
    ENM_PPivot UMETA(DisplayName = "PPivot"),

};
}

UENUM(BlueprintType)
namespace Enm_MomentumPoints {
enum Enm_MomentumPoints {
    ENM_None UMETA(DisplayName = "None"),
    ENM_Light_AttackHit UMETA(DisplayName = "Light_AttackHit"),
    ENM_Medium_AttackHit UMETA(DisplayName = "Medium_AttackHit"),
    ENM_Heavy_AttackHit UMETA(DisplayName = "Heavy_AttackHit"),
    ENM_Combo_ChainPlus UMETA(DisplayName = "Combo_ChainPlus"),
    ENM_Critical_Hit UMETA(DisplayName = "Critical_Hit"),
    ENM_Perfect_Parry UMETA(DisplayName = "Perfect_Parry"),
    ENM_Perfect_Counter UMETA(DisplayName = "Perfect_Counter"),
    ENM_Kndown_StaggerInfli UMETA(DisplayName = "Kndown_StaggerInfli"),
    ENM_Perfect_Evasion UMETA(DisplayName = "Perfect_Evasion"),
    ENM_Light_HitTaken UMETA(DisplayName = "Light_HitTaken"),
    ENM_Medium_HitTaken UMETA(DisplayName = "Medium_HitTaken"),
    ENM_Heavy_HitTaken UMETA(DisplayName = "Heavy_HitTaken"),
    ENM_Perfect_Block UMETA(DisplayName = "Perfect_Block"),
    ENM_NOT_Perfect_Block UMETA(DisplayName = "NOT_Perfect_Block"),
    ENM_Whiffed_Heavy UMETA(DisplayName = "Whiffed_Heavy"),
    ENM_Staggered_KnockedDown UMETA(DisplayName = "Staggered_KnockedDown"),
    ENM_Passive_Idle UMETA(DisplayName = "Passive_Idle"),

};
}

UENUM(BlueprintType)
enum class Enm_AI_Type : uint8 {
    ENM_None UMETA(DisplayName = "None"), ENM_RushDown UMETA(DisplayName = "RushDown"),
    ENM_Shoto UMETA(DisplayName = "Shoto"), ENM_Zoning UMETA(DisplayName = "Zoning"),
    == == == = enum class Enm_AI_Type : uint8{
                 ENM_None UMETA(DisplayName = "None"),
                 ENM_RushDown UMETA(DisplayName = "RushDown"),
                 ENM_Shoto UMETA(DisplayName = "Shoto"),
                 ENM_Zoning UMETA(DisplayName = "Zoning"),
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
             };