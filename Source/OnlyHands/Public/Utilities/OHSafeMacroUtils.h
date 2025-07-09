#pragma once
#include "CoreMinimal.h"
#include "OHSafeMapUtils.h"

#define DEFINE_OH_MUTATION_WITH_CALLBACK_ACCESSORS(                            \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceLambda,       \
    CallbackLambda)                                                            \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
                                                                               \
public:                                                                        \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    return TryApplyRuntimeMutation<FName, ValueType, StructType>(              \
        this->GetIdentifier(), NewValue, &StructType::Cached##FieldName,       \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = ResolveInstanceLambda(Self)) {                  \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut, CallbackLambda);                   \
  }

#define DEFINE_OH_MUTATION_NO_DIRTY_ACCESSORS(                                 \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceLambda)       \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
                                                                               \
public:                                                                        \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    return TryApplyRuntimeMutation<FName, ValueType, StructType>(              \
        this->GetIdentifier(), NewValue, &StructType::Cached##FieldName,       \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = ResolveInstanceLambda(Self)) {                  \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut);                                   \
  }

#define DEFINE_OH_DIRTY_ACCESSORS(StructType, FieldName)                       \
private:                                                                       \
  UPROPERTY(Transient)                                                         \
  bool b##FieldName##Dirty = false;                                            \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }

#define DEFINE_OH_ANALYSIS_FIELD_ACCESSORS(ValueType, FieldName, DefaultValue) \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType FieldName = DefaultValue;                                          \
                                                                               \
public:                                                                        \
  FORCEINLINE const ValueType &Get##FieldName() const { return FieldName; }    \
                                                                               \
protected:                                                                     \
  FORCEINLINE ValueType &GetMutable##FieldName() { return FieldName; }

#define DEFINE_OH_BUFFER_ACCESSORS(ValueType, FieldName)                       \
private:                                                                       \
  UPROPERTY()                                                                  \
  TArray<ValueType> FieldName;                                                 \
                                                                               \
public:                                                                        \
  FORCEINLINE const TArray<ValueType> &Get##FieldName() const {                \
    return FieldName;                                                          \
  }                                                                            \
  FORCEINLINE int32 Get##FieldName##Count() const { return FieldName.Num(); }  \
  FORCEINLINE void Reset##FieldName() { FieldName.Reset(); }                   \
  FORCEINLINE ValueType Get##FieldName##At(int32 Index) const {                \
    return FieldName.IsValidIndex(Index) ? FieldName[Index] : ValueType();     \
  }                                                                            \
                                                                               \
protected:                                                                     \
  FORCEINLINE TArray<ValueType> &GetMutable##FieldName() { return FieldName; }

#define DEFINE_OH_LINKED_GRAPH_EDGE_ACCESSORS(EdgeType, FieldName)             \
private:                                                                       \
  UPROPERTY()                                                                  \
  TArray<EdgeType> FieldName;                                                  \
                                                                               \
public:                                                                        \
  FORCEINLINE const TArray<EdgeType> &Get##FieldName() const {                 \
    return FieldName;                                                          \
  }                                                                            \
  FORCEINLINE void Add##FieldName(const EdgeType &Edge) {                      \
    FieldName.Add(Edge);                                                       \
  }                                                                            \
  FORCEINLINE bool Has##FieldName() const { return FieldName.Num() > 0; }      \
                                                                               \
protected:                                                                     \
  FORCEINLINE TArray<EdgeType> &GetMutable##FieldName() { return FieldName; }

#define DEFINE_OH_VALIDATE_RUNTIME_OBJECT(ResolveFn, DebugLabel)               \
  FORCEINLINE bool HasValidRuntimeObject() const {                             \
    auto *Ptr = ResolveFn();                                                   \
    if (!Ptr) {                                                                \
      UE_LOG(LogTemp, Warning, TEXT("Missing runtime object for: %s"),         \
             *DebugLabel);                                                     \
      return false;                                                            \
    }                                                                          \
    return true;                                                               \
  }

#define DEFINE_OH_GRAPH_MAP_ACCESSORS(KeyType, ValueType, FieldName)           \
private:                                                                       \
  UPROPERTY()                                                                  \
  TMap<KeyType, ValueType> FieldName;                                          \
                                                                               \
public:                                                                        \
  FORCEINLINE const TMap<KeyType, ValueType> &Get##FieldName() const {         \
    return FieldName;                                                          \
  }                                                                            \
  FORCEINLINE TMap<KeyType, ValueType> &GetMutable##FieldName() {              \
    return FieldName;                                                          \
  }                                                                            \
  FORCEINLINE bool Has##FieldName(KeyType Key) const {                         \
    return FieldName.Contains(Key);                                            \
  }                                                                            \
  FORCEINLINE ValueType *Find##FieldName(KeyType Key) {                        \
    return FieldName.Find(Key);                                                \
  }

#define DEFINE_OH_SMART_POINTER_ACCESSORS_CUSTOM(                              \
    Type, FieldName, GetterName, SetterName, FallbackLambda, bLazyAssign)      \
private:                                                                       \
  Type FieldName = nullptr;                                                    \
                                                                               \
public:                                                                        \
  FORCEINLINE void SetterName(Type InPtr) { FieldName = InPtr; }               \
  FORCEINLINE Type GetterName() {                                              \
    if (!FieldName) {                                                          \
      Type Resolved = FallbackLambda();                                        \
      if (bLazyAssign) {                                                       \
        FieldName = Resolved;                                                  \
      }                                                                        \
      return Resolved;                                                         \
    }                                                                          \
    return FieldName;                                                          \
  }                                                                            \
  FORCEINLINE Type GetterName() const {                                        \
    if (!FieldName) {                                                          \
      return FallbackLambda();                                                 \
    }                                                                          \
    return FieldName;                                                          \
  }

// Use when no dirty flag or notification is needed, but equality check should
// skip redundant work
#define DEFINE_OH_MUTATION_NO_DIRTY_ACCESSORS_WITH_COMPARE(                    \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceLambda)       \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
                                                                               \
public:                                                                        \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    if (Cached##FieldName == NewValue)                                         \
      return EOHMutationResult::SkippedRedundant;                              \
    return TryApplyRuntimeMutation<FName, ValueType, StructType>(              \
        this->GetIdentifier(), NewValue, &StructType::Cached##FieldName,       \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = ResolveInstanceLambda(Self)) {                  \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut);                                   \
  }

// Adds dirty tracking + optional notifier, skips if already equal
#define DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                  \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceLambda,       \
    NotifierLambda)                                                            \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
  UPROPERTY(Transient)                                                         \
  bool b##FieldName##Dirty = false;                                            \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }                                                                            \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    if (Cached##FieldName == NewValue)                                         \
      return EOHMutationResult::SkippedRedundant;                              \
    return TryApplyRuntimeMutation<FName, ValueType, StructType>(              \
        this->GetIdentifier(), NewValue, &StructType::Cached##FieldName,       \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = ResolveInstanceLambda(Self)) {                  \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut,                                    \
        [this]() {                                                             \
          this->Set##FieldName##Dirty();                                       \
          NotifierLambda();                                                    \
        });                                                                    \
  }

#define DEFINE_OH_MUTATORS_ALL_BONE_FIELDS()                                   \
  DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                        \
      FOHBoneData, float, LinearDamping, TEXT("Linear Damping"),               \
      [](FOHBoneData &Self) { return Self.GetBodyInstance(); }, []() {})       \
  DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                        \
      FOHBoneData, float, AngularDamping, TEXT("Angular Damping"),             \
      [](FOHBoneData &Self) { return Self.GetBodyInstance(); }, []() {})       \
  DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                        \
      FOHBoneData, float, PhysicsBlendWeight, TEXT("Physics Blend Weight"),    \
      [](FOHBoneData &Self) { return Self.GetBodyInstance(); }, []() {})       \
  DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                        \
      FOHBoneData, float, CachedBodyMass, TEXT("Cached Mass"),                 \
      [](FOHBoneData &Self) { return Self.GetBodyInstance(); }, []() {})

// -----------------------------
// 🔁 Basic mutation macro with dirty flag
// Used for scalar or primitive fields (float, int32, bool)
// -----------------------------
#define DEFINE_OH_MUTATION_AND_DIRTY_ACCESSORS(                                \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceExpr)         \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
  UPROPERTY(Transient)                                                         \
  bool b##FieldName##Dirty = false;                                            \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }                                                                            \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    auto Resolve = ResolveInstanceExpr;                                        \
    return TryApplyRuntimeMutation<ValueType, StructType>(                     \
        GetIdentifier(), NewValue, &StructType::Cached##FieldName,             \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = Resolve()) {                                    \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut,                                    \
        [this]() { Set##FieldName##Dirty(); });                                \
  }

// -----------------------------
// 🔁 Vector field mutation with dirty flag
// For FVector, FQuat, FTransform-style data
// -----------------------------
#define DEFINE_OH_MUTATION_VECTOR_ACCESSORS(StructType, FieldName, DebugLabel, \
                                            ResolveInstanceExpr)               \
  DEFINE_OH_MUTATION_AND_DIRTY_ACCESSORS(StructType, FVector, FieldName,       \
                                         DebugLabel, ResolveInstanceExpr)

// -----------------------------
// 🧩 Enum field mutation
// For cases like EAngularDriveMode, EPhysicsType, etc.
// -----------------------------
#define DEFINE_OH_MUTATION_ENUM_ACCESSORS(StructType, EnumType, FieldName,     \
                                          DebugLabel, ResolveInstanceExpr)     \
  DEFINE_OH_MUTATION_AND_DIRTY_ACCESSORS(StructType, EnumType, FieldName,      \
                                         DebugLabel, ResolveInstanceExpr)

// -----------------------------
// ⚙️ Mutation macro for constraint instance fields
// Used inside FOHConstraintInstanceData
// -----------------------------
#define DEFINE_OH_CONSTRAINT_MUTATION_ACCESSORS(                               \
    StructType, ValueType, FieldName, DebugLabel, ResolveConstraintExpr)       \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
  UPROPERTY(Transient)                                                         \
  bool b##FieldName##Dirty = false;                                            \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }                                                                            \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    auto Resolve = ResolveConstraintExpr;                                      \
    return TryApplyRuntimeMutation<ValueType, StructType>(                     \
        GetIdentifier(), NewValue, &StructType::Cached##FieldName,             \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Constraint = Resolve()) {                                  \
            Constraint->FieldName = V;                                         \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut,                                    \
        [this]() { Set##FieldName##Dirty(); });                                \
  }

// -----------------------------
// 🪡 Adds support for marking a central dirty system
// Recommended for use with PhysicsManager node invalidation
// -----------------------------
#define DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS(                               \
    StructType, ValueType, FieldName, DebugLabel, ResolveExpr, NotifyFn)       \
private:                                                                       \
  UPROPERTY()                                                                  \
  ValueType Cached##FieldName;                                                 \
  UPROPERTY(Transient)                                                         \
  bool b##FieldName##Dirty = false;                                            \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }                                                                            \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    auto Resolve = ResolveExpr;                                                \
    return TryApplyRuntimeMutation<ValueType, StructType>(                     \
        GetIdentifier(), NewValue, &StructType::Cached##FieldName,             \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = Resolve()) {                                    \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut,                                    \
        [this]() {                                                             \
          Set##FieldName##Dirty();                                             \
          NotifyFn();                                                          \
        });                                                                    \
  }

// -----------------------------
// 🕵️ Getter-only for cached physics data (useful for analytics)
// -----------------------------
#define DEFINE_OH_READONLY_ACCESSOR(ValueType, FieldName)                      \
  FORCEINLINE ValueType Get##FieldName() const { return Cached##FieldName; }

#define DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE_AND_CALLBACK(     \
    StructType, ValueType, FieldName, DebugLabel, ResolveInstanceLambda,       \
    NotifierLambda)                                                            \
private:                                                                       \
  UPROPERTY() ValueType Cached##FieldName;                                     \
  UPROPERTY(Transient) bool b##FieldName##Dirty = false;                       \
                                                                               \
public:                                                                        \
  FORCEINLINE bool Is##FieldName##Dirty() const {                              \
    return b##FieldName##Dirty;                                                \
  }                                                                            \
  FORCEINLINE void Set##FieldName##Dirty(bool bDirty = true) {                 \
    b##FieldName##Dirty = bDirty;                                              \
  }                                                                            \
  EOHMutationResult Set##FieldName(const ValueType &NewValue,                  \
                                   bool bStrict = false,                       \
                                   int32 *OutMut = nullptr) {                  \
    if (Cached##FieldName == NewValue)                                         \
      return EOHMutationResult::SkippedRedundant;                              \
    return TryApplyRuntimeMutation<FName, ValueType, StructType>(              \
        GetIdentifier(), NewValue, &StructType::Cached##FieldName,             \
        [](StructType &Self, const ValueType &V) -> bool {                     \
          if (auto *Instance = ResolveInstanceLambda(Self)) {                  \
            Instance->FieldName = V;                                           \
            return true;                                                       \
          }                                                                    \
          return false;                                                        \
        },                                                                     \
        *this, DebugLabel, bStrict, OutMut,                                    \
        [this]() {                                                             \
          this->Set##FieldName##Dirty();                                       \
          NotifierLambda();                                                    \
        });                                                                    \
  }

// Read-only debug/analysis field
#define DEFINE_OH_ANALYSIS_FIELD(StructName, FieldType, FieldName,             \
                                 DefaultValue)                                 \
  UPROPERTY(VisibleAnywhere, Category = "OnlyHands|Debug")                     \
  FieldType FieldName = DefaultValue;                                          \
  DEFINE_OH_READONLY_ACCESSOR(FieldType, FieldName)

// Declares a constraint tuning field, with default value + mutation accessors
#define DEFINE_OH_CONSTRAINT_FIELD_WITH_ACCESSORS(                             \
    StructName, FieldType, FieldName, DefaultValue, DisplayName,               \
    ConstraintAccessor)                                                        \
  UPROPERTY(EditAnywhere, Category = "OnlyHands|Constraint")                   \
  FieldType FieldName = DefaultValue;                                          \
  DEFINE_OH_CONSTRAINT_MUTATION_ACCESSORS(StructName, FieldType, FieldName,    \
                                          DisplayName, ConstraintAccessor)

// Declares a read-only diagnostic field with a simple getter
#define DEFINE_OH_ANALYSIS_FIELD_WITH_ACCESSOR(FieldType, FieldName,           \
                                               DefaultValue)                   \
  UPROPERTY(VisibleAnywhere, Category = "OnlyHands|Debug")                     \
  FieldType FieldName = DefaultValue;                                          \
  DEFINE_OH_READONLY_ACCESSOR(FieldType, FieldName)

// Full constraint field with mutation accessors
#define DEFINE_OH_CONSTRAINT_FIELD_WITH_ACCESSORS(                             \
    StructName, FieldType, FieldName, DefaultValue, DisplayName,               \
    ConstraintAccessor)                                                        \
  UPROPERTY(EditAnywhere, Category = "OnlyHands|Constraint")                   \
  FieldType FieldName = DefaultValue;                                          \
  DEFINE_OH_CONSTRAINT_MUTATION_ACCESSORS(StructName, FieldType, FieldName,    \
                                          DisplayName, ConstraintAccessor)

// Simple editable scalar (non-constraint)
#define DEFINE_OH_EDITABLE_SCALAR_WITH_ACCESSORS(                              \
    StructName, FieldType, FieldName, DefaultValue, DisplayName)               \
  UPROPERTY(EditAnywhere, Category = "OnlyHands|Tuning")                       \
  FieldType FieldName = DefaultValue;                                          \
  DEFINE_OH_MUTATION_AND_NOTIFY_ACCESSORS_WITH_COMPARE(                        \
      StructName, FieldType, FieldName, DisplayName, nullptr, [] {})