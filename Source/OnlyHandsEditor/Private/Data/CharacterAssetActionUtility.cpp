
#if WITH_EDITOR

#include "Data/CharacterAssetActionUtility.h"
#include "EditorUtilitySubsystem.h"
#include "EditorAssetLibrary.h"
#include "AssetRegistry/AssetData.h"
#include "Data/CharacterPDAFactory.h"
#include "Data/PDA/CharacterData/CharacterDataTableRow.h"
#include "Engine/DataTable.h"
#include "EditorUtilityLibrary.h"

<<<<<<< HEAD UCharacterAssetActionUtility::UCharacterAssetActionUtility() { SupportedClasses.Add(UDataTable::StaticClass());
}

void UCharacterAssetActionUtility::GeneratePDAsFromDataTables() {
    // Get selected assets
    TArray<UObject*> SelectedAssets = UEditorUtilityLibrary::GetSelectedAssets();

    for (UObject* Asset : SelectedAssets) {
        if (UDataTable* DataTable = Cast<UDataTable>(Asset)) {
            // Optional: check struct type
            if (DataTable->GetRowStruct() == FCharacterDataTableRow::StaticStruct()) {
                == == == = UCharacterAssetActionUtility::UCharacterAssetActionUtility() {
                    SupportedClasses.Add(UDataTable::StaticClass());
                }

                void UCharacterAssetActionUtility::GeneratePDAsFromDataTables() {
                    // Get selected assets
                    TArray<UObject*> SelectedAssets = UEditorUtilityLibrary::GetSelectedAssets();

                    for (UObject* Asset : SelectedAssets) {
                        if (UDataTable* DataTable = Cast<UDataTable>(Asset)) {
                            // Optional: check struct type
                            if (DataTable->GetRowStruct() == FCharacterDataTableRow::StaticStruct()) {
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                                // Create a factory instance
                                UCharacterPDAFactory* Factory = NewObject<UCharacterPDAFactory>();
                                Factory->CharacterDataTable = DataTable;
                                Factory->OutputDirectory = CharacterDataRootDir;
                                Factory->GenerateFromDataTable();

                                UE_LOG(LogTemp, Log, TEXT("PDAs generated for DataTable: %s"), *DataTable->GetName());
                            }
<<<<<<< HEAD
                            // if (DataTable->GetRowStruct() == FArchetypeDataTableRow::StaticStruct()) TODO
                            else {
                                UE_LOG(LogTemp, Warning, TEXT("Skipping DataTable '%s': incompatible row struct."),
                                       *DataTable->GetName());
                            }
                            == == ==
                                =
                                    // if (DataTable->GetRowStruct() == FArchetypeDataTableRow::StaticStruct()) TODO
                                else {
                                UE_LOG(LogTemp, Warning, TEXT("Skipping DataTable '%s': incompatible row struct."),
                                       *DataTable->GetName());
                            }
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
                        }
                    }
                }

#endif
