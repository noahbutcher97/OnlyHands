
#if WITH_EDITOR

#include "Data/CharacterAssetActionUtility.h"
#include "EditorUtilitySubsystem.h"
#include "EditorAssetLibrary.h"
#include "AssetRegistry/AssetData.h"
#include "Data/CharacterPDAFactory.h"
#include "Data/PDA/CharacterData/CharacterDataTableRow.h"
#include "Engine/DataTable.h"
#include "EditorUtilityLibrary.h"

UCharacterAssetActionUtility::UCharacterAssetActionUtility()
{
    SupportedClasses.Add(UDataTable::StaticClass());
}

void UCharacterAssetActionUtility::GeneratePDAsFromDataTables()
{
    // Get selected assets
    TArray<UObject*> SelectedAssets = UEditorUtilityLibrary::GetSelectedAssets();

    for (UObject* Asset : SelectedAssets)
    {
        if (UDataTable* DataTable = Cast<UDataTable>(Asset))
        {
            // Optional: check struct type
            if (DataTable->GetRowStruct() == FCharacterDataTableRow::StaticStruct())
            {
                // Create a factory instance
                UCharacterPDAFactory* Factory = NewObject<UCharacterPDAFactory>();
                Factory->CharacterDataTable = DataTable;
                Factory->OutputDirectory = CharacterDataRootDir;
                Factory->GenerateFromDataTable();

                UE_LOG(LogTemp, Log, TEXT("PDAs generated for DataTable: %s"), *DataTable->GetName());
            }
            //if (DataTable->GetRowStruct() == FArchetypeDataTableRow::StaticStruct()) TODO
            else
            {
				UE_LOG(LogTemp, Warning, TEXT("Skipping DataTable '%s': incompatible row struct."), *DataTable->GetName());
			}
        }
    }
}

#endif
