#pragma once

#if WITH_EDITOR 

#include "CoreMinimal.h"
#include "Data/PDA/CharacterData/CharacterDataTableRow.h"
#include "UObject/Object.h"
#include "EditorUtilityObject.h"
#include "CharacterPDAFactory.generated.h"

class UDataTable;

UCLASS(Blueprintable)
class ONLYHANDSEDITOR_API UCharacterPDAFactory : public UObject
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Character PDA Generation")
    void GenerateFromDataTable();

#if WITH_EDITORONLY_DATA
    UPROPERTY(EditAnywhere, Category = "Source")
    UDataTable* CharacterDataTable;

    UPROPERTY(EditAnywhere, Category = "Output")
    FDirectoryPath OutputDirectory;

    UPROPERTY(EditAnywhere, Category = "Settings")
    FString NamePrefix = TEXT("PDA_");
#endif
};

#endif
