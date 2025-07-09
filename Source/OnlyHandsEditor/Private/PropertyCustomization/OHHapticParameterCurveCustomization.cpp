#include "PropertyCustomization/OHHapticParameterCurveCustomization.h"
#include "Curves/RichCurve.h"
#include "DetailWidgetRow.h"
#include "IDetailChildrenBuilder.h"
#include "Misc/NotifyHook.h"
#include "PropertyHandle.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"

// Include your struct from runtime module:
#include "Data/Struct/OHHapticsStructs.h"

TSharedRef<IPropertyTypeCustomization>
OHHapticParameterCurveCustomization::MakeInstance() {
  return MakeShareable(new OHHapticParameterCurveCustomization());
}

void OHHapticParameterCurveCustomization::CustomizeHeader(
    TSharedRef<IPropertyHandle> StructPropertyHandle,
    FDetailWidgetRow &HeaderRow,
    IPropertyTypeCustomizationUtils &StructCustomizationUtils) {
  HeaderRow.NameContent()[StructPropertyHandle->CreatePropertyNameWidget()]
      .ValueContent()
      .MinDesiredWidth(300.f)[
          // Inline curve editing (advanced) can go here.
          SNew(STextBlock)
              .Text(FText::FromString(
                  "Edit points in the array below. (Advanced inline curve "
                  "editor can be added later)"))];
}

void OHHapticParameterCurveCustomization::CustomizeChildren(
    TSharedRef<IPropertyHandle> StructPropertyHandle,
    IDetailChildrenBuilder &StructBuilder,
    IPropertyTypeCustomizationUtils &StructCustomizationUtils) {
  StructBuilder.AddProperty(
      StructPropertyHandle->GetChildHandle("Times").ToSharedRef());
  StructBuilder.AddProperty(
      StructPropertyHandle->GetChildHandle("Values").ToSharedRef());
}