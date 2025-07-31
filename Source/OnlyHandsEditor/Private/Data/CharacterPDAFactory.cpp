#if WITH_EDITOR

#include "Data/CharacterPDAFactory.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "ObjectTools.h"
#include "EditorAssetLibrary.h"
#include "Data/PDA/CharacterData/PDA_CharacterBase.h"
#include "Data/PDA/CharacterData/PDA_CharMeshData.h"
#include "Data/PDA/CharacterData/PDA_CharUIData.h"
#include "Data/PDA/CharacterData/PDA_CharGameplayData.h"
#include "Data/PDA/CharacterData/PDA_CharCinematicData.h"

void UCharacterPDAFactory::GenerateFromDataTable()
{
    if (!CharacterDataTable) return;

    const FString TargetPath = OutputDirectory.Path;

    TArray<FName> RowNames = CharacterDataTable->GetRowNames();
    for (const FName& RowName : RowNames)
    {
        const FCharacterDataTableRow* Row = CharacterDataTable->FindRow<FCharacterDataTableRow>(RowName, TEXT(""));

        if (!Row) continue;

        const FString CharName = Row->CharacterID.ToString();
        const FString LookName = Row->LookID.ToString();
		FString CurrentPath = TargetPath + "/" + CharName + "/" + LookName;
        const FString BaseName = CharName + "_" + LookName;

        // - Check for existing assets in directory - 

        TArray<FString> AssetPaths = UEditorAssetLibrary::ListAssets(CurrentPath);

        if (!AssetPaths.IsEmpty())
        {
            UE_LOG(LogTemp, Error, TEXT("PDA's already generated for %s"), *BaseName);
            continue;
        }

        
//		FString CharacterBase;
//        for (const FString& AssetPath : AssetPaths)
//        {
//            UObject* LoadedAsset = UEditorAssetLibrary::LoadAsset(AssetPath);
//            if (!LoadedAsset) continue;
//
//            if (LoadedAsset->IsA<UPDA_CharacterBase>())
//            {
//                CharacterBase = AssetPath; // Store path of character base to delete last
//            }
//            else if
//			(LoadedAsset->IsA<UPDA_CharMeshData>() ||
//			LoadedAsset->IsA<UPDA_CharUIData>() ||
//			LoadedAsset->IsA<UPDA_CharGameplayData>() ||
//			LoadedAsset->IsA<UPDA_CharCinematicData>())
//            {
//                UEditorAssetLibrary::DeleteAsset(AssetPath);
//            }
//        }
//        if (!CharacterBase.IsEmpty())
//            UEditorAssetLibrary::DeleteAsset(CharacterBase);


        // Asset creation helper
        auto CreateAsset = [&](UClass* AssetClass, const FString& DataClassTag) -> UObject*
        {
            FString AssetName = NamePrefix + DataClassTag + "_" + BaseName;
            FString FullPath = CurrentPath + "/" + AssetName;
            return FAssetToolsModule::GetModule().Get().CreateAsset(AssetName, CurrentPath, AssetClass, nullptr);
        };

        // Create and populate MeshData
        auto* MeshData = Cast<UPDA_CharMeshData>(CreateAsset(UPDA_CharMeshData::StaticClass(), TEXT("CharacterSK")));
        MeshData->BaseMesh = Row->BaseMesh;
        MeshData->AttachedMeshes = Row->AttachedMeshes;

        // UI Data
        auto* UIData = Cast<UPDA_CharUIData>(CreateAsset(UPDA_CharUIData::StaticClass(), TEXT("CharacterUI")));
        UIData->Portrait = Row->Portrait;
        UIData->Thumbnail = Row->Thumbnail;

        // Gameplay Data
        auto* Gameplay = Cast<UPDA_CharGameplayData>(CreateAsset(UPDA_CharGameplayData::StaticClass(), TEXT("CharacterGameplay")));
        Gameplay->Archetype = Row->Archetype;
        Gameplay->Taunts = Row->Taunts;

        // Cinematic Data
        auto* Cine = Cast<UPDA_CharCinematicData>(CreateAsset(UPDA_CharCinematicData::StaticClass(), TEXT("CharacterCinematic")));
        Cine->Intro = Row->Intro;
        Cine->WinPose = Row->WinPose;

        // CharacterBase
        auto* CharBase = Cast<UPDA_CharacterBase>(CreateAsset(UPDA_CharacterBase::StaticClass(), TEXT("CharacterBase")));
        CharBase->SKMeshData = MeshData;
        CharBase->UIData = UIData;
        CharBase->GameplayData = Gameplay;
        CharBase->CinematicData = Cine;

        CharBase->MarkPackageDirty();
    }
}

#endif
