#pragma once

<<<<<<< HEAD
#if WITH_EDITOR
    == == ==
    =
#if WITH_EDITOR
        >>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9

#include "CoreMinimal.h"
#include "Data/PDA/CharacterData/CharacterDataTableRow.h"
#include "UObject/Object.h"
#include "EditorUtilityObject.h"
#include "CharacterPDAFactory.generated.h"

        class UDataTable;

UCLASS(Blueprintable)
<<<<<<< HEAD
class ONLYHANDSEDITOR_API UCharacterPDAFactory : public UObject {
    GENERATED_BODY()

  public:
    == == == = class ONLYHANDSEDITOR_API UCharacterPDAFactory : public UObject {
        GENERATED_BODY()

      public:
>>>>>>> 0627b7d296554ee97d27b39fb5f7c959d6da32c9
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
