#pragma once

#include "IPropertyTypeCustomization.h"

class OHHapticParameterCurveCustomization : public IPropertyTypeCustomization {
public:
  static TSharedRef<IPropertyTypeCustomization> MakeInstance();

  virtual void CustomizeHeader(
      TSharedRef<IPropertyHandle> StructPropertyHandle,
      class FDetailWidgetRow &HeaderRow,
      IPropertyTypeCustomizationUtils &StructCustomizationUtils) override;

  virtual void CustomizeChildren(
      TSharedRef<IPropertyHandle> StructPropertyHandle,
      class IDetailChildrenBuilder &StructBuilder,
      IPropertyTypeCustomizationUtils &StructCustomizationUtils) override;
};