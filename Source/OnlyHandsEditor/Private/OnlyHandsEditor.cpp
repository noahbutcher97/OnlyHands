#include "OnlyHandsEditor.h"
#include "PropertyEditorModule.h"
#include "Modules/ModuleManager.h"
#include "PropertyCustomization/OHHapticParameterCurveCustomization.h" // Use the new class name!

DEFINE_LOG_CATEGORY(OnlyHandsEditor);

#define LOCTEXT_NAMESPACE "FOnlyHandsEditor"

static const FName HapticCurveName("FOHHapticParameterCurve");

void FOnlyHandsEditor::StartupModule() {
    UE_LOG(OnlyHandsEditor, Warning, TEXT("OnlyHandsEditor module has been loaded"));

    FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
    PropertyModule.RegisterCustomPropertyTypeLayout(
        HapticCurveName,
        FOnGetPropertyTypeCustomizationInstance::CreateStatic(&OHHapticParameterCurveCustomization::MakeInstance));
}

void FOnlyHandsEditor::ShutdownModule() {
    UE_LOG(OnlyHandsEditor, Warning, TEXT("OnlyHandsEditor module has been unloaded"));

    if (FModuleManager::Get().IsModuleLoaded("PropertyEditor")) {
        FPropertyEditorModule& PropertyModule =
            FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
        PropertyModule.UnregisterCustomPropertyTypeLayout(HapticCurveName);
    }
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FOnlyHandsEditor, OnlyHandsEditor)