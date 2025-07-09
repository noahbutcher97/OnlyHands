// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "FunctionLibrary/OHGraphUtils.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "OHHelperUtils.generated.h"
#include "Utilities/OHSafeMapUtils.h"

/**
 * General-purpose utility function library for OnlyHands
 *
 * This library provides:
 * - General utility functions for working with enums and basic data types
 * - Simple single-bone classification and property queries
 * - Blueprint wrappers for inline helpers from EOHPhysicsEnums.h
 *
 * Use this library for:
 * - Simple, non-complex utility operations
 * - Basic bone classification (is it a finger? is it on the left side?)
 * - Generic helpers that work across different systems
 *
 * For advanced skeletal operations, bone hierarchies, or physics calculations,
 * use OHSkeletalPhysicsUtils instead.
 */
UCLASS()
class ONLYHANDS_API UOHHelperUtils : public UBlueprintFunctionLibrary {
  GENERATED_BODY()

public:
  /**
   * Generic helper to get the display name from any enum
   */
  template <typename TEnum>
  static FORCEINLINE FName GetEnumDisplayName(TEnum EnumValue) {
    static_assert(TIsEnum<TEnum>::Value,
                  "GetEnumDisplayName can only be used with enum types");

    const UEnum *EnumPtr = StaticEnum<TEnum>();
    if (!EnumPtr)
      return NAME_None;
    return FName(
        *EnumPtr->GetDisplayNameTextByValue(static_cast<int64>(EnumValue))
             .ToString());
  }
  /**
   * Get the string representation of any enum value
   */
  template <typename TEnum>
  static FORCEINLINE FString GetEnumValueAsString(TEnum EnumValue) {
    static_assert(TIsEnum<TEnum>::Value,
                  "GetEnumValueAsString can only be used with enum types");

    const UEnum *EnumPtr = StaticEnum<TEnum>();
    if (!EnumPtr)
      return TEXT("Invalid");

    // Get the FULL name (e.g., "EnumName::Value")
    FString FullName =
        EnumPtr->GetNameByValue(static_cast<int64>(EnumValue)).ToString();

    // Optionally strip enum prefix, leaving just the value
    FString CleanName;
    FullName.Split(TEXT("::"), nullptr, &CleanName, ESearchCase::IgnoreCase,
                   ESearchDir::FromEnd);

    return CleanName;
  }
  /**
   * Convert a string to an enum value
   */
  template <typename TEnum>
  static FORCEINLINE TEnum GetEnumValueFromString(const FString &EnumName,
                                                  TEnum DefaultValue) {
    static_assert(TIsEnum<TEnum>::Value,
                  "GetEnumValueFromString can only be used with enum types");

    const UEnum *EnumPtr = StaticEnum<TEnum>();
    if (!EnumPtr)
      return DefaultValue;

    int64 EnumValue = EnumPtr->GetValueByName(FName(*EnumName));
    return EnumValue != INDEX_NONE ? static_cast<TEnum>(EnumValue)
                                   : DefaultValue;
  }

  /**
   * Get the string representation of a BodyPart
   */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|Utils|Enum")
  static FString GetBodyBodyPartAsString(const EOHBodyPart BodyPart) {
    return GetEnumValueAsString(BodyPart);
  }

  /**
   * Check if a bone is a finger bone (wrapper for IsFingerBone in
   * EOHPhysicsEnums.h)
   */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|Utils|Bones")
  static bool IsFingerBone(const EOHSkeletalBone Bone) {
    // This calls the FORCEINLINE function from EOHPhysicsEnums.h
    return ::IsFingerBone(Bone);
  }

  /**
   * Get the body BodyPart for a bone (wrapper for GetBodyPartForBone in
   * EOHPhysicsEnums.h)
   */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|Utils|Bones")
  static EOHBodyPart GetBodyPartFromBone(const EOHSkeletalBone Bone) {
    // This calls the FORCEINLINE function from EOHPhysicsEnums.h
    return GetBodyPartForBone(Bone);
  }

  /**
   * Check if a bone is on the specified side of the body
   */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|Utils|Bones")
  static bool IsBoneOnSide(const EOHSkeletalBone Bone, const bool bLeftSide) {
    // Call the appropriate FORCEINLINE helper from EOHPhysicsEnums.h
    return bLeftSide ? IsLeftSideBone(Bone) : IsRightSideBone(Bone);
  }

  /**
   * Check if a bone is part of the arm or hand
   */
  UFUNCTION(BlueprintPure, Category = "OnlyHands|Utils|Bones")
  static bool IsArmOrHandBone(const EOHSkeletalBone Bone) {
    // Combine results from FORCEINLINE helpers
    return IsArmBone(Bone) || IsFingerBone(Bone);
  }
};
