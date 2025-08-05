#pragma once

#if WITH_EDITOR

#include "CoreMinimal.h"
#include "Blutility/Classes/AssetActionUtility.h"
#include "CharacterAssetActionUtility.generated.h"

UCLASS(Blueprintable)
class ONLYHANDSEDITOR_API UCharacterAssetActionUtility : public UAssetActionUtility {
    GENERATED_BODY()

  public:
    UPROPERTY(EditAnywhere, Category = "Data Paths", meta = (RelativeToGameContentDir))
    FDirectoryPath CharacterDataRootDir;

    UPROPERTY(EditAnywhere, Category = "Data Paths", meta = (RelativeToGameContentDir))
    FDirectoryPath ArenaDataRootDir;

    UPROPERTY(EditAnywhere, Category = "Data Paths", meta = (RelativeToGameContentDir))
    FDirectoryPath ArchetypeDataRootDir;

    UCharacterAssetActionUtility();

    /** Function will appear in right-click menu when valid assets are selected */
    UFUNCTION(CallInEditor, Category = "Generate PDA")
    void GeneratePDAsFromDataTables();

    // protected:
    //
    //     /** Limit what types this utility applies to */
    //     UPROPERTY(EditDefaultsOnly, Category = "Asset Filtering")
    //     TArray<TSoftClassPtr<UObject>> SupportedClasses = { UDataTable::StaticClass() };
};

#endif
