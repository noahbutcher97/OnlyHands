// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "../Enums/E_FightData.h"
#include "Blueprint/UserWidget.h"
#include "GameplayTagContainer.h"
#include "S_FightStructure.generated.h"

class UAnimCompositeBase;
class USoundCue;
class UParticleSystem;
class UNiagaraSystem;
class ULevelSequence;
class APawn;
class UBlendSpace;

/**
 *
 */

USTRUCT(BlueprintType)
struct F_SkinThumbnail : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Default_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Player_Highlight_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Player_Locked_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> OpponentHighlight_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Opponent_Locked_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> In_Game_Image;
};

USTRUCT(BlueprintType)
struct F_FighAnimBP : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UAnimSequenceBase> Anm_Idle;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UAnimSequenceBase> Anm_Block;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UBlendSpace> Anm_Walk;
};

USTRUCT(BlueprintType)
struct F_AnimationSlot : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UAnimCompositeBase* AnimSequences;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, FName> AnimSequences_Path;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Animation_Slot;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_inTime;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_OutTime;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_PlayRate;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 Animation_Loop;
};

USTRUCT(BlueprintType)
struct F_GrabAnimationSlot : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Grab_Attack;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, TSoftObjectPtr<UAnimCompositeBase>> Grab_Animations;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Player_PoseFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Player_RotateFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Player_BlendMovementFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Player_BlendAnimationStart;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Enemy_PoseFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Enemy_RotateFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Enemy_BlendMovementFix;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Enemy_BlendAnimationStart;
};

// Animation
USTRUCT(BlueprintType)
struct F_AnimationRef : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Animation_Group;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Animation_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Cinematic_Attack;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UAnimCompositeBase> Anim_Ref;
};

// Parry
USTRUCT(BlueprintType)
struct F_Parry : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName PBlock_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName PEvade_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName PPivot_Name;
};

USTRUCT(BlueprintType)
struct F_RevideStats : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_AttackSideBody::Enm_AttackSideBody> Side_Attack;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 MaxTime_ToRevide;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<TEnumAsByte<Enm_AttackSideBody::Enm_AttackSideBody>> SideAttack_Week;
};

USTRUCT(BlueprintType)
struct F_SpecialAnimations : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UAnimCompositeBase> AnimSpecial;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Animation_Slot;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_inTime;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_OutTime;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Animation_PlayRate;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Delay_Finish;
};
// FrameData
USTRUCT(BlueprintType)
struct F_DataFrames : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString AttackName;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_AttackLevel::Enm_AttackLevel> AttackLevel;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_AttackSideBody::Enm_AttackSideBody> AttackSide;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float TotalFrames;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Startup;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Activated;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Recovery;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MintDistanceRoot;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartCancelWindow;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float NFramesCancelWindow;
    /*UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_RevideStats RevideMode;*/
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_AnimationRef Cinematic_Animation;
};

USTRUCT(BlueprintType)
struct F_SpecialAbility : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool ActivatedSpecialAbility;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_SkillAbillity::Enm_SkillAbillity> AbilityState;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ValueUse;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float TimeEnd;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName NiagaraSystem_Ability;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UNiagaraSystem> P_AbilityNiagaraSystem;
};

USTRUCT(BlueprintType)
struct F_StateType : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_StateType::Enm_StateType> StateType;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_MoveType::Enm_MoveType> MoveType;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_PhysicState::Enm_PhysicState> PhysicState;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Power;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Juggle;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Control;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Priority;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool FaceEnemy;
};

USTRUCT(BlueprintType)
struct F_HitStun : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool HitStunActivated;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float HitStunTime;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float HitStunIntesityScale;
};

USTRUCT(BlueprintType)
struct F_HitSFX : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bSphereCollision_ImpactPoint;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName SocketName;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bParticle_ImpactPoint;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Particle_Scale;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Particle_Location;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName ParticleSystems_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UParticleSystem> ParticleSystems;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName NiagaraSystem_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UNiagaraSystem> P_NiagaraSystem;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName SoundsFX_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<USoundCue> P_SoundsFX;
};

USTRUCT(BlueprintType)
struct F_HitDef : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_MomentumPoints::Enm_MomentumPoints> AddPointsMovementum;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_MomentumPoints::Enm_MomentumPoints> ReducePointsMovementum;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<TEnumAsByte<Enm_AttackSideBody::Enm_AttackSideBody>> Side_Need_Defense;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TEnumAsByte<Enm_HitBodyParts::Enm_HitBodyParts> Hit_Body_Part;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Damage;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float BlockDamage;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ShakeCameraHit;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OnHit;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OnBlock;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_AnimationRef Hit_AnmRef;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_AnimationRef Death_AnmRef;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_HitStun HitStun;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_HitSFX HitEffects;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_Parry Parry_System;
};

// PlayerData

USTRUCT(BlueprintType)
struct F_StateDef : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_DataFrames FrameData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_StateType StateType;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<int32, F_HitDef> HitDef;
};

USTRUCT(BlueprintType)
struct F_States : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, F_StateDef> States;
};

USTRUCT(BlueprintType)
struct F_CostumePerfil : public FTableRowBase {
    GENERATED_BODY()
    // data
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ID;

    // money
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float crystals;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float stars;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float coins;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float level;
};

USTRUCT(BlueprintType)
struct F_SkinPerfil : public FTableRowBase {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Active;
    // Name
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Skin_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Skin_Subname;
    // Class
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName FighClass;
    // Skin
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftClassPtr<AActor> Skin_Player;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftClassPtr<AActor> Skin_NPC;
    // Image
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_SkinThumbnail Thumbnails;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Full_Body_Image;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<UTexture2D> Load_Screen_Image;

    // Data
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float level;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float life;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float attack;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float speed;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float defence;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float special;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<int32, FString> special_slots;
};

USTRUCT(BlueprintType)
struct F_SkinGroup : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, F_SkinPerfil> SkinGroup;
};

USTRUCT(BlueprintType)
struct F_SeasonGroup : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, F_SkinGroup> Skins;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, UDataTable*> FightClass;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, FString> Levels;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, TSoftObjectPtr<USoundCue>> Music;
};

USTRUCT(BlueprintType)
struct F_FightPerfil : public FTableRowBase {
    GENERATED_BODY()
    // data
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Attack;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Health;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Speed;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Defense;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Special;
};

USTRUCT(BlueprintType)
struct F_PlayerValues_Scale : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Attack;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Health;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Speed;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Defense;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Special;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D RotationRate;
};

USTRUCT(BlueprintType)
struct F_FightSequence_Position : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool World_PLocation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PLocation_Cinematic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Rotation_PLocation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator PRotation_Cinematic;
};

USTRUCT(BlueprintType)
struct F_FightLevelSequence : public FTableRowBase {
    GENERATED_BODY()
    // Cinematic

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<ULevelSequence> Player_Cinematic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftObjectPtr<ULevelSequence> NPC_Cinematic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Start_CameraTarget;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Start_Hidden_Rival;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Hud_RemoveLifeBar;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Hud_Show_Title;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSoftClassPtr<UUserWidget> Title_Cinematic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FString> HudTitleText_Cinematic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Custom_PLocation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_FightSequence_Position P_Position;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Custom_RivalLocation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_FightSequence_Position Rival_Position;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool End_Stop;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool End_Show_Rival;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool End_Stop_Animation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool End_BlockAllCinematics;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool End_NotRemoveTitle;
};

USTRUCT(BlueprintType)
struct F_FightClassPerfil : public FTableRowBase {
    GENERATED_BODY()
    // States
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, UDataTable*> DT_States;
    // Cinematic
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, F_FightLevelSequence> Cinematic;
    // SpecialAnimations
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, F_SpecialAnimations> Special_Animations;

    // AI
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Behavior_Type;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName Enemy_Type;
    // SKILL
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Skill_Name;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_FighAnimBP AnimBp;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, TSoftObjectPtr<UAnimCompositeBase>> AI_MovementList;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, TSoftObjectPtr<UAnimCompositeBase>> AI_MovementAttacks;
    // data
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_FightPerfil PlayerPerfil;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    F_PlayerValues_Scale GlobalPlayerValues;
};

USTRUCT(BlueprintType)
<<<<<<< HEAD
struct F_AI_Data : public FTableRowBase {
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    Enm_AI_Type AiType;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 AI_Level;
};

USTRUCT(BlueprintType)
struct F_LoadCharacter : public FTableRowBase {
    == == == = struct F_AI_Data : public FTableRowBase {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        GENERATED_BODY()
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        Enm_AI_Type AiType;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        int32 AI_Level;
    };

    USTRUCT(BlueprintType)
    struct F_LoadCharacter : public FTableRowBase {
        GENERATED_BODY()
        // Data
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_CostumePerfil PlayerData;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TMap<FName, FString> Left_Char;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_Data AI_Left;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
<<<<<<< HEAD
        TMap<FName, FString> Right_Char;
        == == == = TMap<FName, FString> Right_Char;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_Data AI_Right;
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_Data AI_Right;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TSoftObjectPtr<USoundCue> Music;
    };

    USTRUCT(BlueprintType)
    struct F_GameSeasons : public FTableRowBase {
        GENERATED_BODY()
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_SeasonGroup ListPlayer;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_PlayerValues_Scale PlayerValues_Scale;
    };

    // AI//

    USTRUCT(BlueprintType)
    struct F_AI_GetMovementPoints : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Closed;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Medium;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Far;
    };

    USTRUCT(BlueprintType)
    struct F_AI_AttackPoints : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Agressive;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Punish;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float RunAway;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Mimics;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Stance;
    };

    USTRUCT(BlueprintType)
    struct F_AI_StatetypePoints : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Idle;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Attack;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Block;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Counter;
    };

    USTRUCT(BlueprintType)
    struct F_AI_BodyPartsPoints : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float Head;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float MidChess;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float LeftBody;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float RightBody;
    };

    USTRUCT(BlueprintType)
    struct F_AI_Results : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_MoveSpace::Enm_AI_MoveSpace> DistanceResult;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_AttackStyle::Enm_AI_AttackStyle> AttackStyleResult;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_Intesity::Enm_AI_Intesity> AttackInstensity;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_Intesity::Enm_AI_Intesity> BlockInstensity;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_Intesity::Enm_AI_Intesity> CounterInstensity;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_BodyPart::Enm_AI_BodyPart> First_BodyPrefence;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TEnumAsByte<Enm_AI_BodyPart::Enm_AI_BodyPart> Second_BodyPrefence;
    };

    USTRUCT(BlueprintType)
    struct F_AI_GetValues : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName Name;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_GetMovementPoints GetMovementPoints;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_AttackPoints AttackPoints;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_StatetypePoints StatetypePoints;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_BodyPartsPoints BodyPartsPoints;
    };

    USTRUCT(BlueprintType)
    struct F_AI_BrainData : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName PlayerName;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        TMap<FName, F_AI_GetValues> RivalFightStylList;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        F_AI_Results RivalResult;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName Movement_Strategy;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName Attack_Strategy;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FName Defense_Strategy;
    };

    USTRUCT(BlueprintType)
    struct FInputWindow : public FTableRowBase {
        GENERATED_BODY()

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float StartTime;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float EndTime;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        FLinearColor WindowColor;

        UPROPERTY(EditAnywhere, BlueprintReadWrite)
<<<<<<< HEAD
        FName WindowName;
    };

    USTRUCT(BlueprintType)
    struct F_AI_MovementsTypes : public FTableRowBase {
        == == == = FName WindowName;
    };

    USTRUCT(BlueprintType)
    struct F_AI_MovementsTypes : public FTableRowBase {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
        GENERATED_BODY()
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float WalkForward;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float WalkAround;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float WalkBack;
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        float DashForward;
    };

    USTRUCT(BlueprintType)
<<<<<<< HEAD
    struct F_AI_AttackTypes : public FTableRowBase {
        GENERATED_BODY()
        UPROPERTY(EditAnywhere, BlueprintReadWrite)
        == == == = struct F_AI_AttackTypes : public FTableRowBase {
            GENERATED_BODY()
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
            float LightAttack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float MediumAttack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float HighAttack;

            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float SmartAttack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float DoubleAttack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float ChainAttack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float ChainVariation;
<<<<<<< HEAD
        };

        USTRUCT(BlueprintType)
        struct F_AI_DefenseTypes : public FTableRowBase {
            GENERATED_BODY()
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            == == == =
        };

        USTRUCT(BlueprintType)
        struct F_AI_DefenseTypes : public FTableRowBase {
            GENERATED_BODY()
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
            float Block;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float PerfectBlock;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float DashBack;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float DashLeft_Right;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float PerfectEvade;
            UPROPERTY(EditAnywhere, BlueprintReadWrite)
            float PerfectPivot;
        };

        USTRUCT(BlueprintType)
<<<<<<< HEAD
        struct F_AI_Action : public FTableRowBase {
            == == == = struct F_AI_Action : public FTableRowBase {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                GENERATED_BODY()
                UPROPERTY(EditAnywhere, BlueprintReadWrite)
                float AI_Agressive;
                UPROPERTY(EditAnywhere, BlueprintReadWrite)
                F_AI_MovementsTypes AI_MovementTypes;
                UPROPERTY(EditAnywhere, BlueprintReadWrite)
                F_AI_AttackTypes AI_AttackTypes;
                UPROPERTY(EditAnywhere, BlueprintReadWrite)
                F_AI_DefenseTypes AI_DefenseTypes;
<<<<<<< HEAD
                == == == =

>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
            };
