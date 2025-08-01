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

<<<<<<< HEAD void UCharacterPDAFactory::GenerateFromDataTable() { if (!CharacterDataTable) return; == == == = void UCharacterPDAFactory::GenerateFromDataTable() { if (!CharacterDataTable) return;
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

    const FString TargetPath = OutputDirectory.Path;

TArray<FName> RowNames = CharacterDataTable->GetRowNames();
<<<<<<< HEAD
for (const FName& RowName : RowNames) {
    const FCharacterDataTableRow* Row = CharacterDataTable->FindRow<FCharacterDataTableRow>(RowName, TEXT(""));

    if (!Row)
        continue;

    const FString CharName = Row->CharacterID.ToString();
    const FString LookName = Row->LookID.ToString();
    FString CurrentPath = TargetPath + "/" + CharName + "/" + LookName;
    const FString BaseName = CharName + "_" + LookName;

    // - Check for existing assets in directory -

    TArray<FString> AssetPaths = UEditorAssetLibrary::ListAssets(CurrentPath);

    if (!AssetPaths.IsEmpty()) {
        == == == = for (const FName& RowName : RowNames) {
            const FCharacterDataTableRow* Row = CharacterDataTable->FindRow<FCharacterDataTableRow>(RowName, TEXT(""));

            if (!Row)
                continue;

            const FString CharName = Row->CharacterID.ToString();
            const FString LookName = Row->LookID.ToString();
            FString CurrentPath = TargetPath + "/" + CharName + "/" + LookName;
            const FString BaseName = CharName + "_" + LookName;

            // - Check for existing assets in directory -

            TArray<FString> AssetPaths = UEditorAssetLibrary::ListAssets(CurrentPath);

            if (!AssetPaths.IsEmpty()) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                UE_LOG(LogTemp, Error, TEXT("PDA's already generated for %s"), *BaseName);
                continue;
            }

<<<<<<< HEAD
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
            auto CreateAsset = [&](UClass* AssetClass, const FString& DataClassTag) -> UObject* {
                == == == =

                             //		FString CharacterBase;
                             //        for (const FString& AssetPath : AssetPaths)
                             //        {
                             //            UObject* LoadedAsset = UEditorAssetLibrary::LoadAsset(AssetPath);
                             //            if (!LoadedAsset) continue;
                             //
                             //            if (LoadedAsset->IsA<UPDA_CharacterBase>())
                             //            {
                             //                CharacterBase = AssetPath; // Store path of character base to
                             //                delete last
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
                    auto CreateAsset = [&](UClass* AssetClass, const FString& DataClassTag) -> UObject* {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                    FString AssetName = NamePrefix + DataClassTag + "_" + BaseName;
                    FString FullPath = CurrentPath + "/" + AssetName;
                    return FAssetToolsModule::GetModule().Get().CreateAsset(AssetName, CurrentPath, AssetClass,
                                                                            nullptr);
                };

                // Create and populate MeshData
                auto* MeshData =
                    Cast<UPDA_CharMeshData>(CreateAsset(UPDA_CharMeshData::StaticClass(), TEXT("CharacterSK")));
                MeshData->BaseMesh = Row->BaseMesh;
                MeshData->AttachedMeshes = Row->AttachedMeshes;

                // UI Data
                auto* UIData = Cast<UPDA_CharUIData>(CreateAsset(UPDA_CharUIData::StaticClass(), TEXT("CharacterUI")));
                UIData->Portrait = Row->Portrait;
                UIData->Thumbnail = Row->Thumbnail;

                // Gameplay Data
<<<<<<< HEAD
                auto* Gameplay = Cast<UPDA_CharGameplayData>(
                    CreateAsset(UPDA_CharGameplayData::StaticClass(), TEXT("CharacterGameplay")));
                == == == = auto* Gameplay = Cast<UPDA_CharGameplayData>(
                             CreateAsset(UPDA_CharGameplayData::StaticClass(), TEXT("CharacterGameplay")));
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                Gameplay->Archetype = Row->Archetype;
                Gameplay->Taunts = Row->Taunts;

                // Cinematic Data
<<<<<<< HEAD
                auto* Cine = Cast<UPDA_CharCinematicData>(
                    CreateAsset(UPDA_CharCinematicData::StaticClass(), TEXT("CharacterCinematic")));
                == == == = auto* Cine = Cast<UPDA_CharCinematicData>(
                             CreateAsset(UPDA_CharCinematicData::StaticClass(), TEXT("CharacterCinematic")));
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                Cine->Intro = Row->Intro;
                Cine->WinPose = Row->WinPose;

                // CharacterBase
<<<<<<< HEAD
                auto* CharBase =
                    Cast<UPDA_CharacterBase>(CreateAsset(UPDA_CharacterBase::StaticClass(), TEXT("CharacterBase")));
                == == ==
                    = auto* CharBase =
                        Cast<UPDA_CharacterBase>(CreateAsset(UPDA_CharacterBase::StaticClass(), TEXT("CharacterBase")));
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                CharBase->SKMeshData = MeshData;
                CharBase->UIData = UIData;
                CharBase->GameplayData = Gameplay;
                CharBase->CinematicData = Cine;

                CharBase->MarkPackageDirty();
            }
        }

#endif
