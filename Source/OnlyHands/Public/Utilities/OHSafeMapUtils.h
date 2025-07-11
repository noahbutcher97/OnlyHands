#pragma once

#include "CoreMinimal.h"
#include "Algo/Accumulate.h"
#include "Algo/Sort.h"
#include "Algo/Transform.h"
#include "Data/Enum/EOHPhysicsEnums.h"
#include "Templates/Function.h"
DECLARE_LOG_CATEGORY_EXTERN(LogSafeMapUtils, Log, All);

/**
 * OHSafeMapUtils
 *
 * Type-agnostic template utilities to prevent data desync, dangling keys, invalid values,
 * and maintain structural integrity across maps, arrays, and key references.
 */
namespace OHSafeMapUtils {

template <typename KeyType, typename ValueType>
FORCEINLINE const ValueType* GetReference(const TMap<KeyType, ValueType>& Map, const KeyType& Key) {
    return Map.Find(Key);
}

template <typename KeyType, typename ValueType>
FORCEINLINE ValueType* GetMutableReference(TMap<KeyType, ValueType>& Map, const KeyType& Key) {
    return Map.Find(Key);
}

template <typename KeyType, typename ValueType>
FORCEINLINE const ValueType& GetReferenceChecked(const TMap<KeyType, ValueType>& Map, const KeyType& Key,
                                                 const TCHAR* Context = TEXT("GetReferenceChecked")) {
    if (!Map.Contains(Key)) {
        UE_LOG(LogSafeMapUtils, Error, TEXT("[%s] Key missing in map: %s"), Context,
               *TTypeToString<KeyType>::ToString(Key));
        checkNoEntry(); // Fatal for logic errors.
    }
    return Map.FindChecked(Key);
}

template <typename KeyType, typename ValueType>
FORCEINLINE ValueType& GetMutableReferenceChecked(TMap<KeyType, ValueType>& Map, const KeyType& Key,
                                                  const TCHAR* Context = TEXT("GetMutableReferenceChecked")) {
    if (!Map.Contains(Key)) {
        UE_LOG(LogSafeMapUtils, Error, TEXT("[%s] Key missing in map: %s"), Context,
               *TTypeToString<KeyType>::ToString(Key));
        checkNoEntry();
    }
    return Map.FindChecked(Key);
}

template <typename TContext, typename TValue, typename TStruct, typename TField = TValue>
EOHMutationResult TryApplyRuntimeMutation(
    const TContext& ContextID, const TValue& NewValue, TField TStruct::* CachedFieldPtr,
    TFunctionRef<bool(TStruct&, const TValue&)> ApplyToLiveInstanceFn, TStruct& TargetStruct,
    const FString& DebugLabel = TEXT("UnnamedField"), bool bStrict = false, int32* OutMutationCounter = nullptr,
    TFunctionRef<void()> OnMutation = []() {}) {
    // Validate pointer
    if (!CachedFieldPtr) {
        UE_LOG(LogTemp, Error, TEXT("Mutation failed: Null cached field ptr for [%s]"), *DebugLabel);
        return EOHMutationResult::Failure;
    }

    // Compare old and new
    const TValue& OldValue = TargetStruct.*CachedFieldPtr;
    const bool bSame = OldValue == NewValue;

    // Always update struct field
    TargetStruct.*CachedFieldPtr = NewValue;

    // Attempt live runtime update
    const bool bApplied = ApplyToLiveInstanceFn(TargetStruct, NewValue);

    if (!bSame || bStrict) {
        if (OutMutationCounter)
            (*OutMutationCounter)++;
        OnMutation();
    }

    if (bApplied) {
        UE_LOG(LogTemp, VeryVerbose, TEXT("Mutation [%s] applied to live instance (%s)."), *DebugLabel,
               *ContextID.ToString());
        return EOHMutationResult::SuccessDirect;
    } else if (!bSame || bStrict) {
        if (bStrict) {
            UE_LOG(LogTemp, Error, TEXT("Strict mutation failed: [%s] on [%s]"), *DebugLabel, *ContextID.ToString());
            ensureMsgf(false, TEXT("Strict mutation failure [%s]"), *DebugLabel);
            return EOHMutationResult::Failure;
        } else {
            UE_LOG(LogTemp, Warning, TEXT("Fallback mutation: [%s] cached only (no live instance)."), *DebugLabel);
            return EOHMutationResult::SuccessFallback;
        }
    }

    return EOHMutationResult::NoChange;
}
#pragma region PhysicsSystemTargeted

template <typename K, typename V, typename KeyContainerType>
FORCEINLINE bool AssertMapContainsAllKeys(const TMap<K, V>& Map, const KeyContainerType& RequiredKeys,
                                          const TCHAR* Context = TEXT("AssertMapContainsAllKeys")) {
    bool bAllFound = true;
    for (const K& Key : RequiredKeys) {
        if (!Map.Contains(Key)) {
            UE_LOG(LogSafeMapUtils, Warning, TEXT("[%s] Missing key: %s"), Context, *TTypeToString<K>::ToString(Key));
            bAllFound = false;
        }
    }
    return bAllFound;
}

template <typename K, typename V>
FORCEINLINE bool EnsureMapIsBijective(const TMap<K, V>& Map, const TCHAR* Context = TEXT("EnsureMapIsBijective")) {
    TSet<V> SeenValues;
    bool bBijective = true;

    for (const auto& Pair : Map) {
        if (SeenValues.Contains(Pair.Value)) {
            UE_LOG(LogSafeMapUtils, Error, TEXT("[%s] Non-bijective mapping detected for key: %s, value: %s"), Context,
                   *TTypeToString<K>::ToString(Pair.Key), *TTypeToString<V>::ToString(Pair.Value));
            bBijective = false;
        } else {
            SeenValues.Add(Pair.Value);
        }
    }
    return bBijective;
}

template <typename TKey, typename TEdge, typename TArrayWrapper, typename TArrayWrapper2>
void BuildGraphHierarchyAndConstraintMap(const TArray<TEdge>& Edges, TMap<TKey, TArrayWrapper>& ParentToChildrenMap,
                                         TMap<TKey, TArrayWrapper2>& BoneToConstraintsMap,
                                         TFunctionRef<TKey(const TEdge&)> GetParent,
                                         TFunctionRef<TKey(const TEdge&)> GetChild) {
    for (const TEdge& Edge : Edges) {
        const TKey Parent = GetParent(Edge);
        const TKey Child = GetChild(Edge);

        // Update hierarchy map
        TArrayWrapper& Children = ParentToChildrenMap.FindOrAdd(Parent);
        Children.Bones.Add(Child);

        // Update constraint map
        BoneToConstraintsMap.FindOrAdd(Parent).Constraints.Add(Edge);
        BoneToConstraintsMap.FindOrAdd(Child).Constraints.Add(Edge);
    }
}

template <typename K, typename V> TMap<K, V> GraphDiff(const TMap<K, V>& A, const TMap<K, V>& B) {
    TMap<K, V> Result;
    for (const auto& Pair : A) {
        if (const V* InB = B.Find(Pair.Key)) {
            if (!(*InB == Pair.Value)) {
                Result.Add(Pair.Key, Pair.Value);
            }
        } else {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename K, typename V, typename Extractor>
FORCEINLINE float ComputeMapMean(const TMap<K, V>& Map, Extractor&& Extract) {
    if (Map.Num() == 0) {
        return 0.f;
    }
    float Sum = 0.f;
    for (const auto& Pair : Map) {
        Sum += Extract(Pair.Value);
    }
    return Sum / static_cast<float>(Map.Num());
}

template <typename K, typename V>
FORCEINLINE float ComputeMapMean(const TMap<K, V>& Map, TFunctionRef<float(const V&)> Extract) {
    if (Map.Num() == 0) {
        return 0.f;
    }
    float Sum = 0.f;
    for (const auto& Pair : Map) {
        Sum += Extract(Pair.Value);
    }
    return Sum / static_cast<float>(Map.Num());
}

template <typename K, typename V>
TMap<int32, TArray<K>> BucketizeMap(const TMap<K, V>& Map, TFunctionRef<float(const V&)> Extract, float BucketSize) {
    TMap<int32, TArray<K>> Buckets;
    for (const auto& Pair : Map) {
        int32 Bucket = FMath::FloorToInt(Extract(Pair.Value) / BucketSize);
        Buckets.FindOrAdd(Bucket).Add(Pair.Key);
    }
    return Buckets;
}

template <typename K, typename V, typename Extractor>
FORCEINLINE float ComputeMapVariance(const TMap<K, V>& Map, Extractor&& Extract) {
    const int32 N = Map.Num();
    if (N == 0) {
        return 0.f;
    }
    const float Mean = ComputeMapMean(Map, Extract);
    float Accum = 0.f;
    for (const auto& Pair : Map) {
        const float Val = Extract(Pair.Value);
        Accum += FMath::Square(Val - Mean);
    }
    return Accum / static_cast<float>(N);
}

template <typename K, typename V>
FORCEINLINE float ComputeMapVariance(const TMap<K, V>& Map, TFunctionRef<float(const V&)> Extract) {
    const int32 N = Map.Num();
    if (N == 0) {
        return 0.f;
    }
    const float Mean = ComputeMapMean(Map, Extract);
    float Accum = 0.f;
    for (const auto& Pair : Map) {
        const float Val = Extract(Pair.Value);
        Accum += FMath::Square(Val - Mean);
    }
    return Accum / static_cast<float>(N);
}

template <typename K, typename V, typename Extractor>
FORCEINLINE TMap<K, float> ComputeZScores(const TMap<K, V>& Map, Extractor&& Extract) {
    TMap<K, float> Result;
    if (Map.Num() == 0) {
        return Result;
    }
    const float Mean = ComputeMapMean(Map, Extract);
    const float Variance = ComputeMapVariance(Map, Extract);
    const float StdDev = FMath::Sqrt(Variance);

    for (const auto& Pair : Map) {
        const float Z = (StdDev > 0.f) ? (Extract(Pair.Value) - Mean) / StdDev : 0.f;
        Result.Add(Pair.Key, Z);
    }
    return Result;
}

template <typename K, typename V>
FORCEINLINE TMap<K, float> ComputeZScores(const TMap<K, V>& Map, TFunctionRef<float(const V&)> Extract) {
    TMap<K, float> Result;
    if (Map.Num() == 0) {
        return Result;
    }
    const float Mean = ComputeMapMean(Map, Extract);
    const float Variance = ComputeMapVariance(Map, Extract);
    const float StdDev = FMath::Sqrt(Variance);

    for (const auto& Pair : Map) {
        const float Z = (StdDev > 0.f) ? (Extract(Pair.Value) - Mean) / StdDev : 0.f;
        Result.Add(Pair.Key, Z);
    }
    return Result;
}

template <typename K> TArray<TArray<K>> CompressLinearChains(const TMap<K, TArray<K>>& Graph) {
    TSet<K> Visited;
    TArray<TArray<K>> Chains;

    for (const auto& Pair : Graph) {
        const K& Start = Pair.Key;
        if (Visited.Contains(Start)) {
            continue;
        }

        TArray<K> Chain;
        K Current = Start;

        while (true) {
            Visited.Add(Current);
            Chain.Add(Current);

            const TArray<K>* Neighbors = Graph.Find(Current);
            if (!Neighbors || Neighbors->Num() != 1) {
                break;
            }

            const K& Next = (*Neighbors)[0];
            if (Visited.Contains(Next)) {
                break;
            }

            Current = Next;
        }

        if (Chain.Num() > 1) {
            Chains.Add(Chain);
        }
    }
    return Chains;
}

template <typename K, typename V, typename ResolverType>
FORCEINLINE TMap<K, V> MergeMapsWithConflictResolution(const TMap<K, V>& A, const TMap<K, V>& B,
                                                       ResolverType&& Resolver) {
    TMap<K, V> Result = A;
    for (const auto& Pair : B) {
        if (V* Existing = Result.Find(Pair.Key)) {
            *Existing = Resolver(*Existing, Pair.Value);
        } else {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename K, typename V>
FORCEINLINE TMap<K, V> MergeMapsWithConflictResolution(const TMap<K, V>& A, const TMap<K, V>& B,
                                                       TFunctionRef<V(const V&, const V&)> Resolver) {
    TMap<K, V> Result = A;
    for (const auto& Pair : B) {
        if (V* Existing = Result.Find(Pair.Key)) {
            *Existing = Resolver(*Existing, Pair.Value);
        } else {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename K, typename V> bool AreGraphsIsomorphic(const TMap<K, V>& A, const TMap<K, V>& B) {
    if (A.Num() != B.Num()) {
        return false;
    }

    for (const auto& Pair : A) {
        const V* InB = B.Find(Pair.Key);
        if (!InB || !(*InB == Pair.Value)) {
            return false;
        }
    }
    return true;
}

template <typename T, typename PredicateType>
FORCEINLINE TArray<T> MakeFilterIterator(const TArray<T>& Input, PredicateType&& Predicate, int32 MaxItems = -1) {
    TArray<T> Result;
    for (const T& Item : Input) {
        if (Predicate(Item)) {
            Result.Add(Item);
            if (MaxItems > 0 && Result.Num() >= MaxItems) {
                break;
            }
        }
    }
    return Result;
}

template <typename T>
FORCEINLINE TArray<T> MakeFilterIterator(const TArray<T>& Input, TFunctionRef<bool(const T&)> Predicate,
                                         int32 MaxItems = -1) {
    TArray<T> Result;
    for (const T& Item : Input) {
        if (Predicate(Item)) {
            Result.Add(Item);
            if (MaxItems > 0 && Result.Num() >= MaxItems) {
                break;
            }
        }
    }
    return Result;
}

template <typename K> bool AssertNoCycles(const TMap<K, TArray<K>>& Graph, FString Label = TEXT("Graph")) {
    TSet<K> Visited, Stack;

    TFunction<bool(const K&)> Recurse;
    Recurse = [&](const K& Node) -> bool {
        if (Stack.Contains(Node)) {
            UE_LOG(LogTemp, Error, TEXT("[%s] Cycle detected at: %s"), *Label, *Node.ToString());
            return false;
        }
        if (Visited.Contains(Node)) {
            return true;
        }

        Visited.Add(Node);
        Stack.Add(Node);

        const TArray<K>* Neighbors = Graph.Find(Node);
        if (Neighbors) {
            for (const K& N : *Neighbors) {
                if (!Recurse(N)) {
                    return false;
                }
            }
        }
        Stack.Remove(Node);
        return true;
    };

    for (const auto& Pair : Graph) {
        if (!Recurse(Pair.Key)) {
            return false;
        }
    }
    return true;
}

template <typename K> TSet<K> GetSubtreeFromNode(const K& Start, const TMap<K, TArray<K>>& Graph) {
    TSet<K> Result;
    TQueue<K> Queue;
    Queue.Enqueue(Start);

    while (!Queue.IsEmpty()) {
        K Current;
        Queue.Dequeue(Current);
        if (!Result.Add(Current)) {
            continue;
        }

        if (const TArray<K>* Children = Graph.Find(Current)) {
            for (const K& C : *Children) {
                if (!Result.Contains(C)) {
                    Queue.Enqueue(C);
                }
            }
        }
    }
    return Result;
}

template <typename T> struct TPipe {
    T Data;

    explicit TPipe(T InData) : Data(MoveTemp(InData)) {}

    template <typename Func> auto Map(Func F) const {
        using ResultT = decltype(F(*Data.begin()));
        TArray<ResultT> Result;
        Result.Reserve(Data.Num());
        for (const auto& Item : Data) {
            Result.Add(F(Item));
        }
        return TPipe<TArray<ResultT>>(MoveTemp(Result));
    }

    template <typename Pred> auto Filter(Pred P) const {
        TArray<typename T::ElementType> Result;
        for (const auto& Item : Data) {
            if (P(Item)) {
                Result.Add(Item);
            }
        }
        return TPipe<decltype(Result)>(MoveTemp(Result));
    }

    template <typename Reducer, typename OutT> OutT Reduce(Reducer R, OutT Init) const {
        for (const auto& Item : Data) {
            Init = R(Init, Item);
        }
        return Init;
    }

    const T& Get() const {
        return Data;
    }
    T&& Move() {
        return MoveTemp(Data);
    }
};

template <typename T> TPipe<T> MakePipe(T Input) {
    return TPipe<T>(MoveTemp(Input));
}

template <typename K, typename V> struct TPipeMap {
    TMap<K, V> Data;

    explicit TPipeMap(TMap<K, V> InData) : Data(MoveTemp(InData)) {}

    // Generic version
    template <typename VOut, typename Transformer> TPipeMap<K, VOut> MapValues(Transformer&& Transform) const {
        TMap<K, VOut> Result;
        for (const auto& Pair : Data) {
            Result.Add(Pair.Key, Transform(Pair.Value));
        }
        return TPipeMap<K, VOut>(MoveTemp(Result));
    }

    // TFunctionRef overload
    template <typename VOut> TPipeMap<K, VOut> MapValues(TFunctionRef<VOut(const V&)> Transform) const {
        TMap<K, VOut> Result;
        for (const auto& Pair : Data) {
            Result.Add(Pair.Key, Transform(Pair.Value));
        }
        return TPipeMap<K, VOut>(MoveTemp(Result));
    }

    // Generic version
    template <typename Predicate> TPipeMap<K, V> FilterPairs(Predicate&& Pred) const {
        TMap<K, V> Result;
        for (const auto& Pair : Data) {
            if (Pred(Pair.Key, Pair.Value)) {
                Result.Add(Pair.Key, Pair.Value);
            }
        }
        return TPipeMap<K, V>(MoveTemp(Result));
    }

    // TFunctionRef overload
    TPipeMap<K, V> FilterPairs(TFunctionRef<bool(const K&, const V&)> Pred) const {
        TMap<K, V> Result;
        for (const auto& Pair : Data) {
            if (Pred(Pair.Key, Pair.Value)) {
                Result.Add(Pair.Key, Pair.Value);
            }
        }
        return TPipeMap<K, V>(MoveTemp(Result));
    }

    // Generic version
    template <typename OutT, typename ReducerType> OutT Reduce(ReducerType&& Reducer, OutT Init) const {
        for (const auto& Pair : Data) {
            Init = Reducer(Init, Pair.Key, Pair.Value);
        }
        return Init;
    }

    // TFunctionRef overload
    template <typename OutT> OutT Reduce(TFunctionRef<OutT(const OutT&, const K&, const V&)> Reducer, OutT Init) const {
        for (const auto& Pair : Data) {
            Init = Reducer(Init, Pair.Key, Pair.Value);
        }
        return Init;
    }

    const TMap<K, V>& Get() const {
        return Data;
    }
    TMap<K, V>&& Move() {
        return MoveTemp(Data);
    }
};

template <typename K, typename V> TPipeMap<K, V> MakeMapPipe(TMap<K, V> Input) {
    return TPipeMap<K, V>(MoveTemp(Input));
}

template <typename K> bool ValidateParentLinkage(const TMap<K, TArray<K>>& ParentToChildren) {
    TSet<K> AllChildren;
    for (const auto& Pair : ParentToChildren) {
        AllChildren.Append(Pair.Value);
    }
    for (const K& Child : AllChildren) {
        if (!ParentToChildren.Contains(Child)) {
            UE_LOG(LogSafeMapUtils, Warning, TEXT("Child %s has no parent entry."), *TTypeToString<K>::ToString(Child));
            return false;
        }
    }
    return true;
}

template <typename K, typename V, typename Getter>
FORCEINLINE bool ValidateBoneMassRange(const TMap<K, V>& BoneMap, Getter&& GetMass, float Min = 0.01f,
                                       float Max = 1000.f) {
    for (const auto& Pair : BoneMap) {
        const float M = GetMass(Pair.Value);
        if (M < Min || M > Max) {
            UE_LOG(LogSafeMapUtils, Warning, TEXT("Bone %s has invalid mass: %f"),
                   *TTypeToString<K>::ToString(Pair.Key), M);
            return false;
        }
    }
    return true;
}

template <typename K, typename V>
FORCEINLINE bool ValidateBoneMassRange(const TMap<K, V>& BoneMap, TFunctionRef<float(const V&)> GetMass,
                                       float Min = 0.01f, float Max = 1000.f) {
    for (const auto& Pair : BoneMap) {
        const float M = GetMass(Pair.Value);
        if (M < Min || M > Max) {
            UE_LOG(LogSafeMapUtils, Warning, TEXT("Bone %s has invalid mass: %f"),
                   *TTypeToString<K>::ToString(Pair.Key), M);
            return false;
        }
    }
    return true;
}

template <typename K, typename Predicate>
FORCEINLINE TArray<TArray<K>> FindChainsMatching(const TMap<K, TArray<K>>& Graph, Predicate&& IsChainRoot) {
    TArray<TArray<K>> Chains;
    TSet<K> Visited;

    for (const auto& Pair : Graph) {
        if (!IsChainRoot(Pair.Key)) {
            continue;
        }
        if (Visited.Contains(Pair.Key)) {
            continue;
        }

        TArray<K> Chain;
        K Current = Pair.Key;

        while (!Visited.Contains(Current)) {
            Chain.Add(Current);
            Visited.Add(Current);

            const TArray<K>* Children = Graph.Find(Current);
            if (!Children || Children->Num() != 1) {
                break;
            }

            Current = (*Children)[0];
        }

        if (Chain.Num() > 1) {
            Chains.Add(Chain);
        }
    }
    return Chains;
}

template <typename K>
FORCEINLINE TArray<TArray<K>> FindChainsMatching(const TMap<K, TArray<K>>& Graph,
                                                 TFunctionRef<bool(const K&)> IsChainRoot) {
    TArray<TArray<K>> Chains;
    TSet<K> Visited;

    for (const auto& Pair : Graph) {
        if (!IsChainRoot(Pair.Key)) {
            continue;
        }
        if (Visited.Contains(Pair.Key)) {
            continue;
        }

        TArray<K> Chain;
        K Current = Pair.Key;

        while (!Visited.Contains(Current)) {
            Chain.Add(Current);
            Visited.Add(Current);

            const TArray<K>* Children = Graph.Find(Current);
            if (!Children || Children->Num() != 1) {
                break;
            }

            Current = (*Children)[0];
        }

        if (Chain.Num() > 1) {
            Chains.Add(Chain);
        }
    }
    return Chains;
}

template <typename K> bool IsLinearChain(const TMap<K, TArray<K>>& Graph, const K& Root) {
    int32 NodeCount = 0;
    K Current = Root;

    while (true) {
        NodeCount++;
        const TArray<K>* Children = Graph.Find(Current);
        if (!Children || Children->Num() == 0) {
            break;
        }
        if (Children->Num() > 1) {
            return false;
        }
        Current = (*Children)[0];
    }
    return NodeCount > 1;
}

template <typename TConstraint, typename TBone, typename GetAType, typename GetBType>
FORCEINLINE TSet<FName> FindUnconstrainedBones(const TMap<FName, TBone>& BoneMap,
                                               const TArray<TConstraint>& Constraints, GetAType&& GetA,
                                               GetBType&& GetB) {
    TSet<FName> Constrained;
    for (const TConstraint& C : Constraints) {
        Constrained.Add(GetA(C));
        Constrained.Add(GetB(C));
    }

    TSet<FName> AllBones;
    BoneMap.GetKeys(AllBones);
    return AllBones.Difference(Constrained);
}

template <typename TConstraint, typename TBone>
FORCEINLINE TSet<FName>
FindUnconstrainedBones(const TMap<FName, TBone>& BoneMap, const TArray<TConstraint>& Constraints,
                       TFunctionRef<FName(const TConstraint&)> GetA, TFunctionRef<FName(const TConstraint&)> GetB) {
    TSet<FName> Constrained;
    for (const TConstraint& C : Constraints) {
        Constrained.Add(GetA(C));
        Constrained.Add(GetB(C));
    }

    TSet<FName> AllBones;
    BoneMap.GetKeys(AllBones);
    return AllBones.Difference(Constrained);
}

template <typename TConstraint, typename TBone, typename GetParentType, typename GetChildType, typename GetMassType>
FORCEINLINE TArray<TConstraint> ValidateConstraintMassBalance(const TArray<TConstraint>& Constraints,
                                                              const TMap<FName, TBone>& BoneMap,
                                                              GetParentType&& GetParent, GetChildType&& GetChild,
                                                              GetMassType&& GetMass, float RatioThreshold = 2.5f) {
    TArray<TConstraint> Violations;
    for (const TConstraint& C : Constraints) {
        const FName A = GetParent(C), B = GetChild(C);
        const TBone *BA = BoneMap.Find(A), *BB = BoneMap.Find(B);
        if (!BA || !BB) {
            continue;
        }

        float MA = GetMass(*BA), MB = GetMass(*BB);
        if (FMath::IsNearlyZero(MA) || FMath::IsNearlyZero(MB)) {
            continue;
        }

        float Ratio = FMath::Max(MA, MB) / FMath::Min(MA, MB);
        if (Ratio > RatioThreshold) {
            Violations.Add(C);
        }
    }
    return Violations;
}

template <typename TConstraint, typename TBone>
FORCEINLINE TArray<TConstraint>
ValidateConstraintMassBalance(const TArray<TConstraint>& Constraints, const TMap<FName, TBone>& BoneMap,
                              TFunctionRef<FName(const TConstraint&)> GetParent,
                              TFunctionRef<FName(const TConstraint&)> GetChild,
                              TFunctionRef<float(const TBone&)> GetMass, float RatioThreshold = 2.5f) {
    TArray<TConstraint> Violations;
    for (const TConstraint& C : Constraints) {
        const FName A = GetParent(C), B = GetChild(C);
        const TBone *BA = BoneMap.Find(A), *BB = BoneMap.Find(B);
        if (!BA || !BB) {
            continue;
        }

        float MA = GetMass(*BA), MB = GetMass(*BB);
        if (FMath::IsNearlyZero(MA) || FMath::IsNearlyZero(MB)) {
            continue;
        }

        float Ratio = FMath::Max(MA, MB) / FMath::Min(MA, MB);
        if (Ratio > RatioThreshold) {
            Violations.Add(C);
        }
    }
    return Violations;
}

template <typename TConstraint, typename GetAType, typename GetBType>
FORCEINLINE TArray<TConstraint> DetectUnidirectionalConstraints(const TArray<TConstraint>& Constraints, GetAType&& GetA,
                                                                GetBType&& GetB) {
    TSet<FString> SeenPairs;
    TArray<TConstraint> Unidirectional;

    for (const TConstraint& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);
        FString Key = A.ToString() + "|" + B.ToString();
        FString ReverseKey = B.ToString() + "|" + A.ToString();

        if (!SeenPairs.Contains(ReverseKey)) {
            Unidirectional.Add(C);
        }
        SeenPairs.Add(Key);
    }
    return Unidirectional;
}

template <typename TConstraint>
FORCEINLINE TArray<TConstraint> DetectUnidirectionalConstraints(const TArray<TConstraint>& Constraints,
                                                                TFunctionRef<FName(const TConstraint&)> GetA,
                                                                TFunctionRef<FName(const TConstraint&)> GetB) {
    TSet<FString> SeenPairs;
    TArray<TConstraint> Unidirectional;

    for (const TConstraint& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);
        FString Key = A.ToString() + "|" + B.ToString();
        FString ReverseKey = B.ToString() + "|" + A.ToString();

        if (!SeenPairs.Contains(ReverseKey)) {
            Unidirectional.Add(C);
        }
        SeenPairs.Add(Key);
    }
    return Unidirectional;
}

template <typename TConstraint, typename GetAType, typename GetBType>
FORCEINLINE TMap<FName, TArray<FName>> BuildBoneConstraintAdjacencyGraph(const TArray<TConstraint>& Constraints,
                                                                         GetAType&& GetA, GetBType&& GetB) {
    TMap<FName, TArray<FName>> Graph;
    for (const TConstraint& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);

        Graph.FindOrAdd(A).AddUnique(B);
        Graph.FindOrAdd(B).AddUnique(A);
    }
    return Graph;
}

template <typename TConstraint>
FORCEINLINE TMap<FName, TArray<FName>> BuildBoneConstraintAdjacencyGraph(const TArray<TConstraint>& Constraints,
                                                                         TFunctionRef<FName(const TConstraint&)> GetA,
                                                                         TFunctionRef<FName(const TConstraint&)> GetB) {
    TMap<FName, TArray<FName>> Graph;
    for (const TConstraint& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);

        Graph.FindOrAdd(A).AddUnique(B);
        Graph.FindOrAdd(B).AddUnique(A);
    }
    return Graph;
}

template <typename TConstraint, typename GetAType, typename GetBType>
FORCEINLINE TMap<FName, int32> MeasureConstraintDensityPerBone(const TArray<TConstraint>& Constraints, GetAType&& GetA,
                                                               GetBType&& GetB) {
    TMap<FName, int32> Counts;
    for (const TConstraint& C : Constraints) {
        ++Counts.FindOrAdd(GetA(C));
        ++Counts.FindOrAdd(GetB(C));
    }
    return Counts;
}

template <typename TConstraint>
FORCEINLINE TMap<FName, int32> MeasureConstraintDensityPerBone(const TArray<TConstraint>& Constraints,
                                                               TFunctionRef<FName(const TConstraint&)> GetA,
                                                               TFunctionRef<FName(const TConstraint&)> GetB) {
    TMap<FName, int32> Counts;
    for (const TConstraint& C : Constraints) {
        ++Counts.FindOrAdd(GetA(C));
        ++Counts.FindOrAdd(GetB(C));
    }
    return Counts;
}

template <typename T, typename GetParentType, typename GetChildType>
FORCEINLINE TMap<FName, TArray<FName>> BuildParentToChildrenMap(const TArray<T>& Constraints, GetParentType&& GetParent,
                                                                GetChildType&& GetChild) {
    TMap<FName, TArray<FName>> Map;
    for (const T& C : Constraints) {
        Map.FindOrAdd(GetParent(C)).AddUnique(GetChild(C));
    }
    return Map;
}

template <typename T>
FORCEINLINE TMap<FName, TArray<FName>> BuildParentToChildrenMap(const TArray<T>& Constraints,
                                                                TFunctionRef<FName(const T&)> GetParent,
                                                                TFunctionRef<FName(const T&)> GetChild) {
    TMap<FName, TArray<FName>> Map;
    for (const T& C : Constraints) {
        Map.FindOrAdd(GetParent(C)).AddUnique(GetChild(C));
    }
    return Map;
}

template <typename T, typename GetParentType, typename GetChildType>
FORCEINLINE TMap<FName, FName> BuildChildToParentMap(const TArray<T>& Constraints, GetParentType&& GetParent,
                                                     GetChildType&& GetChild) {
    TMap<FName, FName> Map;
    for (const T& C : Constraints) {
        Map.Add(GetChild(C), GetParent(C));
    }
    return Map;
}

template <typename T>
FORCEINLINE TMap<FName, FName> BuildChildToParentMap(const TArray<T>& Constraints,
                                                     TFunctionRef<FName(const T&)> GetParent,
                                                     TFunctionRef<FName(const T&)> GetChild) {
    TMap<FName, FName> Map;
    for (const T& C : Constraints) {
        Map.Add(GetChild(C), GetParent(C));
    }
    return Map;
}

template <typename T, typename GetAType, typename GetBType>
FORCEINLINE TMap<FName, TArray<T>> BuildBoneToConstraintsMap(const TArray<T>& Constraints, GetAType&& GetA,
                                                             GetBType&& GetB) {
    TMap<FName, TArray<T>> Map;
    for (const T& C : Constraints) {
        Map.FindOrAdd(GetA(C)).Add(C);
        Map.FindOrAdd(GetB(C)).Add(C);
    }
    return Map;
}

template <typename T>
FORCEINLINE TMap<FName, TArray<T>> BuildBoneToConstraintsMap(const TArray<T>& Constraints,
                                                             TFunctionRef<FName(const T&)> GetA,
                                                             TFunctionRef<FName(const T&)> GetB) {
    TMap<FName, TArray<T>> Map;
    for (const T& C : Constraints) {
        Map.FindOrAdd(GetA(C)).Add(C);
        Map.FindOrAdd(GetB(C)).Add(C);
    }
    return Map;
}

template <typename TNode, typename TContainer>
TArray<TNode> FindAllRootsInMap(const TMap<TNode, TContainer>& ParentToChildren) {
    TSet<TNode> AllParents;
    TSet<TNode> AllChildren;

    for (const auto& Pair : ParentToChildren) {
        AllParents.Add(Pair.Key);

        for (const TNode& Child : Pair.Value) {
            AllChildren.Add(Child);
        }
    }

    TArray<TNode> Roots;
    for (const TNode& P : AllParents) {
        if (!AllChildren.Contains(P)) {
            Roots.Add(P);
        }
    }

    return Roots;
}

template <typename K> TSet<K> FindAllRoots(const TMap<K, TArray<K>>& Adjacency) {
    TSet<K> AllKeys, AllChildren;
    for (const auto& Pair : Adjacency) {
        AllKeys.Add(Pair.Key);
        AllChildren.Append(Pair.Value);
    }
    return AllKeys.Difference(AllChildren);
}

// Returns nodes that never appear as parents
template <typename TNode, typename TContainer>
TArray<TNode> FindAllLeavesInMap(const TMap<TNode, TContainer>& ParentToChildren) {
    TSet<TNode> AllChildren;
    for (const auto& Pair : ParentToChildren) {
        for (const TNode& Child : Pair.Value) {
            AllChildren.Add(Child);
        }
    }

    TArray<TNode> Leaves;
    for (const TNode& C : AllChildren) {
        if (!ParentToChildren.Contains(C)) {
            Leaves.Add(C);
        }
    }
    return Leaves;
}

template <typename K> TSet<K> FindAllLeaves(const TMap<K, TArray<K>>& Adjacency) {
    TSet<K> All, Children;
    for (const auto& Pair : Adjacency) {
        All.Add(Pair.Key);
        Children.Append(Pair.Value);
    }
    return All.Difference(Children);
}

// Finds nodes with no incoming or outgoing links
template <typename TNode, typename TContainer>
TArray<TNode> FindIsolatedNodesInMap(const TMap<TNode, TContainer>& ParentToChildren, const TSet<TNode>& AllNodes) {
    TSet<TNode> Connected;
    for (const auto& Pair : ParentToChildren) {
        Connected.Add(Pair.Key);
        for (const TNode& Child : Pair.Value) {
            Connected.Add(Child);
        }
    }

    TArray<TNode> Isolated;
    for (const TNode& Node : AllNodes) {
        if (!Connected.Contains(Node)) {
            Isolated.Add(Node);
        }
    }
    return Isolated;
}

// Builds a reverse graph: Child → Parents[]
template <typename TNode, typename TContainer>
TMap<TNode, TArray<TNode>> InvertMapToMultiMap(const TMap<TNode, TContainer>& ParentToChildren) {
    TMap<TNode, TArray<TNode>> Result;
    for (const auto& Pair : ParentToChildren) {
        for (const TNode& Child : Pair.Value) {
            Result.FindOrAdd(Child).Add(Pair.Key);
        }
    }
    return Result;
}

// Collects all parent nodes from a reversed graph
template <typename TNode> TSet<TNode> ExtractParentsFromChildMap(const TMap<TNode, TArray<TNode>>& ChildToParents) {
    TSet<TNode> Parents;
    for (const auto& Pair : ChildToParents) {
        for (const TNode& P : Pair.Value) {
            Parents.Add(P);
        }
    }
    return Parents;
}

template <typename TKey, typename TValue, typename RawType, typename ExtractFnType>
FORCEINLINE TMap<TKey, RawType> ExtractRawMapFromWrapper(const TMap<TKey, TValue>& Input, ExtractFnType&& ExtractFn) {
    TMap<TKey, RawType> Out;
    for (const auto& Pair : Input) {
        Out.Add(Pair.Key, ExtractFn(Pair.Value));
    }
    return Out;
}

template <typename TKey, typename TValue, typename RawType>
FORCEINLINE TMap<TKey, RawType> ExtractRawMapFromWrapper(const TMap<TKey, TValue>& Input,
                                                         TFunctionRef<RawType(const TValue&)> ExtractFn) {
    TMap<TKey, RawType> Out;
    for (const auto& Pair : Input) {
        Out.Add(Pair.Key, ExtractFn(Pair.Value));
    }
    return Out;
}

template <typename TKey, typename TValue, typename PredicateType>
FORCEINLINE bool AnyEntryFailsPredicate(const TMap<TKey, TValue>& Map, PredicateType&& Predicate) {
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Key, Pair.Value)) {
            return true;
        }
    }
    return false;
}

template <typename TKey, typename TValue>
FORCEINLINE bool AnyEntryFailsPredicate(const TMap<TKey, TValue>& Map,
                                        TFunctionRef<bool(const TKey&, const TValue&)> Predicate) {
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Key, Pair.Value)) {
            return true;
        }
    }
    return false;
}

template <typename TKey, typename TValue, typename PredicateType>
FORCEINLINE bool AllEntriesPassPredicate(const TMap<TKey, TValue>& Map, PredicateType&& Predicate) {
    for (const auto& Pair : Map) {
        if (!Predicate(Pair.Key, Pair.Value)) {
            return false;
        }
    }
    return true;
}

template <typename TKey, typename TValue>
FORCEINLINE bool AllEntriesPassPredicate(const TMap<TKey, TValue>& Map,
                                         TFunctionRef<bool(const TKey&, const TValue&)> Predicate) {
    for (const auto& Pair : Map) {
        if (!Predicate(Pair.Key, Pair.Value)) {
            return false;
        }
    }
    return true;
}

template <typename TKey, typename TValue, typename PredicateType>
FORCEINLINE TOptional<TKey> FindFirstFailingEntry(const TMap<TKey, TValue>& Map, PredicateType&& Predicate) {
    for (const auto& Pair : Map) {
        if (!Predicate(Pair.Key, Pair.Value)) {
            return Pair.Key;
        }
    }
    return TOptional<TKey>{};
}

template <typename TKey, typename TValue>
FORCEINLINE TOptional<TKey> FindFirstFailingEntry(const TMap<TKey, TValue>& Map,
                                                  TFunctionRef<bool(const TKey&, const TValue&)> Predicate) {
    for (const auto& Pair : Map) {
        if (!Predicate(Pair.Key, Pair.Value)) {
            return Pair.Key;
        }
    }
    return TOptional<TKey>{};
}

template <typename TElement, typename TKey, typename GetKeyType, typename DescribeValueType>
FORCEINLINE TMap<TKey, FString> MapToReport(const TArray<TElement>& Array, GetKeyType&& GetKey,
                                            DescribeValueType&& DescribeValue) {
    TMap<TKey, FString> Report;
    for (const TElement& Element : Array) {
        Report.Add(GetKey(Element), DescribeValue(Element));
    }
    return Report;
}

template <typename TElement, typename TKey>
FORCEINLINE TMap<TKey, FString> MapToReport(const TArray<TElement>& Array, TFunctionRef<TKey(const TElement&)> GetKey,
                                            TFunctionRef<FString(const TElement&)> DescribeValue) {
    TMap<TKey, FString> Report;
    for (const TElement& Element : Array) {
        Report.Add(GetKey(Element), DescribeValue(Element));
    }
    return Report;
}

#pragma region Visualization

template <typename T, typename GetLocationAType, typename GetLocationBType>
FORCEINLINE void DrawConstraintConnections(const TArray<T>& Constraints, GetLocationAType&& GetLocationA,
                                           GetLocationBType&& GetLocationB, const FColor& Color = FColor::Red,
                                           float Thickness = 2.f, float Duration = 0.f) {
    for (const T& C : Constraints) {
        DrawDebugLine(GWorld, GetLocationA(C), GetLocationB(C), Color, false, Duration, 0, Thickness);
    }
}

template <typename T>
FORCEINLINE void DrawConstraintConnections(const TArray<T>& Constraints, TFunctionRef<FVector(const T&)> GetLocationA,
                                           TFunctionRef<FVector(const T&)> GetLocationB,
                                           const FColor& Color = FColor::Red, float Thickness = 2.f,
                                           float Duration = 0.f) {
    for (const T& C : Constraints) {
        DrawDebugLine(GWorld, GetLocationA(C), GetLocationB(C), Color, false, Duration, 0, Thickness);
    }
}

template <typename T, typename GetLocationType>
FORCEINLINE void DrawBonePoints(const TMap<FName, T>& BoneMap, GetLocationType&& GetLocation,
                                const FColor& Color = FColor::Blue, float Radius = 3.f, float Duration = 0.f) {
    for (const auto& Pair : BoneMap) {
        DrawDebugSphere(GWorld, GetLocation(Pair.Value), Radius, 8, Color, false, Duration);
    }
}

template <typename T>
FORCEINLINE void DrawBonePoints(const TMap<FName, T>& BoneMap, TFunctionRef<FVector(const T&)> GetLocation,
                                const FColor& Color = FColor::Blue, float Radius = 3.f, float Duration = 0.f) {
    for (const auto& Pair : BoneMap) {
        DrawDebugSphere(GWorld, GetLocation(Pair.Value), Radius, 8, Color, false, Duration);
    }
}

template <typename T, typename GetLocationType>
FORCEINLINE void DrawBoneLabels(const TMap<FName, T>& BoneMap, GetLocationType&& GetLocation,
                                const FColor& Color = FColor::White, float Duration = 0.f) {
    for (const auto& Pair : BoneMap) {
        DrawDebugString(GWorld, GetLocation(Pair.Value) + FVector(0, 0, 5), Pair.Key.ToString(), nullptr, Color,
                        Duration, false);
    }
}

template <typename T>
FORCEINLINE void DrawBoneLabels(const TMap<FName, T>& BoneMap, TFunctionRef<FVector(const T&)> GetLocation,
                                const FColor& Color = FColor::White, float Duration = 0.f) {
    for (const auto& Pair : BoneMap) {
        DrawDebugString(GWorld, GetLocation(Pair.Value) + FVector(0, 0, 5), Pair.Key.ToString(), nullptr, Color,
                        Duration, false);
    }
}

template <typename T>
void DrawBoneHierarchyLinks(const TMap<FName, TArray<FName>>& Hierarchy, const TMap<FName, FVector>& BonePositions,
                            const FColor& Color = FColor::Green, float Thickness = 1.f, float Duration = 0.f) {
    for (const auto& Pair : Hierarchy) {
        const FVector* Start = BonePositions.Find(Pair.Key);
        if (!Start) {
            continue;
        }

        for (const FName& Child : Pair.Value) {
            if (const FVector* End = BonePositions.Find(Child)) {
                DrawDebugLine(GWorld, *Start, *End, Color, false, Duration, 0, Thickness);
            }
        }
    }
}

template <typename T, typename GetFromType, typename GetToType>
FORCEINLINE void DrawDirectionalConstraintArrows(const TArray<T>& Constraints, GetFromType&& GetFrom, GetToType&& GetTo,
                                                 const FColor& Color = FColor::Yellow, float ArrowSize = 10.f,
                                                 float Thickness = 1.5f, float Duration = 0.f) {
    for (const T& C : Constraints) {
        const FVector Start = GetFrom(C);
        const FVector End = GetTo(C);
        DrawDebugDirectionalArrow(GWorld, Start, End, ArrowSize, Color, false, Duration, 0, Thickness);
    }
}

template <typename T>
FORCEINLINE void DrawDirectionalConstraintArrows(const TArray<T>& Constraints, TFunctionRef<FVector(const T&)> GetFrom,
                                                 TFunctionRef<FVector(const T&)> GetTo,
                                                 const FColor& Color = FColor::Yellow, float ArrowSize = 10.f,
                                                 float Thickness = 1.5f, float Duration = 0.f) {
    for (const T& C : Constraints) {
        const FVector Start = GetFrom(C);
        const FVector End = GetTo(C);
        DrawDebugDirectionalArrow(GWorld, Start, End, ArrowSize, Color, false, Duration, 0, Thickness);
    }
}

template <typename T, typename TBone, typename GetAType, typename GetBType, typename GetMassType, typename GetPosType>
FORCEINLINE void DrawConstraintMassRatioText(const TArray<T>& Constraints, const TMap<FName, TBone>& BoneMap,
                                             GetAType&& GetA, GetBType&& GetB, GetMassType&& GetMass,
                                             GetPosType&& GetPos, const FColor& Color = FColor::White,
                                             float Duration = 0.f) {
    for (const T& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);
        const TBone* BA = BoneMap.Find(A);
        const TBone* BB = BoneMap.Find(B);
        if (!BA || !BB) {
            continue;
        }

        float MA = GetMass(*BA);
        float MB = GetMass(*BB);
        if (MA <= 0 || MB <= 0) {
            continue;
        }

        float Ratio = MA / MB;
        FVector Mid = (GetPos(A) + GetPos(B)) * 0.5f;

        DrawDebugString(GWorld, Mid, FString::Printf(TEXT("%.2f"), Ratio), nullptr, Color, Duration, false);
    }
}

template <typename T, typename TBone>
FORCEINLINE void DrawConstraintMassRatioText(const TArray<T>& Constraints, const TMap<FName, TBone>& BoneMap,
                                             TFunctionRef<FName(const T&)> GetA, TFunctionRef<FName(const T&)> GetB,
                                             TFunctionRef<float(const TBone&)> GetMass,
                                             TFunctionRef<FVector(const FName&)> GetPos,
                                             const FColor& Color = FColor::White, float Duration = 0.f) {
    for (const T& C : Constraints) {
        FName A = GetA(C);
        FName B = GetB(C);
        const TBone* BA = BoneMap.Find(A);
        const TBone* BB = BoneMap.Find(B);
        if (!BA || !BB) {
            continue;
        }

        float MA = GetMass(*BA);
        float MB = GetMass(*BB);
        if (MA <= 0 || MB <= 0) {
            continue;
        }

        float Ratio = MA / MB;
        FVector Mid = (GetPos(A) + GetPos(B)) * 0.5f;

        DrawDebugString(GWorld, Mid, FString::Printf(TEXT("%.2f"), Ratio), nullptr, Color, Duration, false);
    }
}

template <typename T, typename GetPositionType>
FORCEINLINE void DrawRootIndicators(const TSet<FName>& Roots, const TMap<FName, T>& BoneMap,
                                    GetPositionType&& GetPosition, const FColor& Color = FColor::Magenta,
                                    float Radius = 6.f, float Duration = 0.f) {
    for (const FName& Root : Roots) {
        if (const T* Bone = BoneMap.Find(Root)) {
            DrawDebugSphere(GWorld, GetPosition(*Bone), Radius, 8, Color, false, Duration);
        }
    }
}

template <typename T>
FORCEINLINE void DrawRootIndicators(const TSet<FName>& Roots, const TMap<FName, T>& BoneMap,
                                    TFunctionRef<FVector(const T&)> GetPosition, const FColor& Color = FColor::Magenta,
                                    float Radius = 6.f, float Duration = 0.f) {
    for (const FName& Root : Roots) {
        if (const T* Bone = BoneMap.Find(Root)) {
            DrawDebugSphere(GWorld, GetPosition(*Bone), Radius, 8, Color, false, Duration);
        }
    }
}

template <typename T>
void DrawSubtreeHighlight(const TSet<FName>& Subtree, const TMap<FName, T>& BoneMap,
                          TFunctionRef<FVector(const T&)> GetPosition, const FColor& Color = FColor::Cyan,
                          float Radius = 4.f, float Duration = 0.f) {
    for (const FName& Bone : Subtree) {
        if (const T* Entry = BoneMap.Find(Bone)) {
            DrawDebugSphere(GWorld, GetPosition(*Entry), Radius, 6, Color, false, Duration);
        }
    }
}

template <typename T, typename GetAType, typename GetBType>
FORCEINLINE void DrawConstraintEndpoints(const TArray<T>& Constraints, GetAType&& GetA, GetBType&& GetB,
                                         const FColor& ColorA = FColor::Red, const FColor& ColorB = FColor::Blue,
                                         float Radius = 3.f, float Duration = 0.f) {
    for (const T& C : Constraints) {
        DrawDebugSphere(GWorld, GetA(C), Radius, 8, ColorA, false, Duration);
        DrawDebugSphere(GWorld, GetB(C), Radius, 8, ColorB, false, Duration);
    }
}

template <typename T>
FORCEINLINE void DrawConstraintEndpoints(const TArray<T>& Constraints, TFunctionRef<FVector(const T&)> GetA,
                                         TFunctionRef<FVector(const T&)> GetB, const FColor& ColorA = FColor::Red,
                                         const FColor& ColorB = FColor::Blue, float Radius = 3.f,
                                         float Duration = 0.f) {
    for (const T& C : Constraints) {
        DrawDebugSphere(GWorld, GetA(C), Radius, 8, ColorA, false, Duration);
        DrawDebugSphere(GWorld, GetB(C), Radius, 8, ColorB, false, Duration);
    }
}

template <typename K, typename GetLocationType>
FORCEINLINE void DrawGraphCyclePaths(const TMap<K, TArray<K>>& Graph, GetLocationType&& GetLocation,
                                     const FColor& Color = FColor::Orange, float Thickness = 2.f,
                                     float Duration = 0.f) {
    TSet<K> Visited, Stack;
    TArray<K> Path;

    TFunction<bool(const K&)> Recurse;
    Recurse = [&](const K& Node) -> bool {
        if (Stack.Contains(Node)) {
            int32 StartIndex = Path.Find(Node);
            if (StartIndex != INDEX_NONE) {
                for (int32 i = StartIndex; i < Path.Num() - 1; ++i) {
                    DrawDebugLine(GWorld, GetLocation(Path[i]), GetLocation(Path[i + 1]), Color, false, Duration, 0,
                                  Thickness);
                }
            }
            return true;
        }
        if (Visited.Contains(Node)) {
            return false;
        }

        Visited.Add(Node);
        Stack.Add(Node);
        Path.Add(Node);

        if (const TArray<K>* Neighbors = Graph.Find(Node)) {
            for (const K& N : *Neighbors) {
                if (Recurse(N)) {
                    return true;
                }
            }
        }

        Stack.Remove(Node);
        Path.Pop();
        return false;
    };

    for (const auto& Pair : Graph) {
        if (!Visited.Contains(Pair.Key)) {
            Path.Reset();
            Recurse(Pair.Key);
        }
    }
}

template <typename K>
FORCEINLINE void DrawGraphCyclePaths(const TMap<K, TArray<K>>& Graph, TFunctionRef<FVector(const K&)> GetLocation,
                                     const FColor& Color = FColor::Orange, float Thickness = 2.f,
                                     float Duration = 0.f) {
    TSet<K> Visited, Stack;
    TArray<K> Path;

    TFunction<bool(const K&)> Recurse;
    Recurse = [&](const K& Node) -> bool {
        if (Stack.Contains(Node)) {
            int32 StartIndex = Path.Find(Node);
            if (StartIndex != INDEX_NONE) {
                for (int32 i = StartIndex; i < Path.Num() - 1; ++i) {
                    DrawDebugLine(GWorld, GetLocation(Path[i]), GetLocation(Path[i + 1]), Color, false, Duration, 0,
                                  Thickness);
                }
            }
            return true;
        }
        if (Visited.Contains(Node)) {
            return false;
        }

        Visited.Add(Node);
        Stack.Add(Node);
        Path.Add(Node);

        if (const TArray<K>* Neighbors = Graph.Find(Node)) {
            for (const K& N : *Neighbors) {
                if (Recurse(N)) {
                    return true;
                }
            }
        }

        Stack.Remove(Node);
        Path.Pop();
        return false;
    };

    for (const auto& Pair : Graph) {
        if (!Visited.Contains(Pair.Key)) {
            Path.Reset();
            Recurse(Pair.Key);
        }
    }
}

template <typename K, typename GetLocationType>
FORCEINLINE void DrawGraphIslands(const TMap<K, TArray<K>>& Graph, GetLocationType&& GetLocation, float Radius = 5.f,
                                  float Duration = 0.f) {
    TSet<K> Visited;
    TArray<FColor> Palette = {FColor::Red,     FColor::Green,  FColor::Blue,   FColor::Cyan,
                              FColor::Magenta, FColor::Yellow, FColor::Orange, FColor::Purple};

    for (const auto& Pair : Graph) {
        const K& Start = Pair.Key;
        if (Visited.Contains(Start)) {
            continue;
        }

        TSet<K> Island;
        TQueue<K> Queue;
        Queue.Enqueue(Start);

        while (!Queue.IsEmpty()) {
            K Node;
            Queue.Dequeue(Node);
            if (!Island.Add(Node)) {
                continue;
            }

            if (const TArray<K>* Neighbors = Graph.Find(Node)) {
                for (const K& N : *Neighbors) {
                    if (!Island.Contains(N)) {
                        Queue.Enqueue(N);
                    }
                }
            }
        }

        const FColor Color = Palette[Visited.Num() % Palette.Num()];
        for (const K& Node : Island) {
            DrawDebugSphere(GWorld, GetLocation(Node), Radius, 6, Color, false, Duration);
        }

        Visited.Append(Island);
    }
}

template <typename K>
FORCEINLINE void DrawGraphIslands(const TMap<K, TArray<K>>& Graph, TFunctionRef<FVector(const K&)> GetLocation,
                                  float Radius = 5.f, float Duration = 0.f) {
    TSet<K> Visited;
    TArray<FColor> Palette = {FColor::Red,     FColor::Green,  FColor::Blue,   FColor::Cyan,
                              FColor::Magenta, FColor::Yellow, FColor::Orange, FColor::Purple};

    for (const auto& Pair : Graph) {
        const K& Start = Pair.Key;
        if (Visited.Contains(Start)) {
            continue;
        }

        TSet<K> Island;
        TQueue<K> Queue;
        Queue.Enqueue(Start);

        while (!Queue.IsEmpty()) {
            K Node;
            Queue.Dequeue(Node);
            if (!Island.Add(Node)) {
                continue;
            }

            if (const TArray<K>* Neighbors = Graph.Find(Node)) {
                for (const K& N : *Neighbors) {
                    if (!Island.Contains(N)) {
                        Queue.Enqueue(N);
                    }
                }
            }
        }

        const FColor Color = Palette[Visited.Num() % Palette.Num()];
        for (const K& Node : Island) {
            DrawDebugSphere(GWorld, GetLocation(Node), Radius, 6, Color, false, Duration);
        }

        Visited.Append(Island);
    }
}

#pragma endregion

template <typename K, typename V, typename ToStringType>
FORCEINLINE void DebugLogMap(const TMap<K, V>& Map, const FString& Label, ToStringType&& ToString) {
    UE_LOG(LogTemp, Log, TEXT("--- %s ---"), *Label);
    for (const auto& Pair : Map) {
        UE_LOG(LogTemp, Log, TEXT("%s -> %s"), *Pair.Key.ToString(), *ToString(Pair.Value));
    }
}

template <typename K, typename V>
FORCEINLINE void DebugLogMap(const TMap<K, V>& Map, const FString& Label, TFunctionRef<FString(const V&)> ToString) {
    UE_LOG(LogTemp, Log, TEXT("--- %s ---"), *Label);
    for (const auto& Pair : Map) {
        UE_LOG(LogTemp, Log, TEXT("%s -> %s"), *Pair.Key.ToString(), *ToString(Pair.Value));
    }
}

#pragma endregion

// ============================================================================
//  Map Transformation / Projection Utilities
// ============================================================================

/**
 * Transforms all keys in a map using a provided transformation function.
 * Keeps the original values unchanged.
 *
 * Example: rename enum keys to string keys or apply normalization.
 */
template <typename TKeyIn, typename TKeyOut, typename TValue, typename KeyTransformType>
FORCEINLINE TMap<TKeyOut, TValue> TransformMapKeys(const TMap<TKeyIn, TValue>& Input, KeyTransformType&& KeyTransform) {
    TMap<TKeyOut, TValue> Result;
    for (const auto& Pair : Input) {
        Result.Add(KeyTransform(Pair.Key), Pair.Value);
    }
    return Result;
}

template <typename TKeyIn, typename TKeyOut, typename TValue>
FORCEINLINE TMap<TKeyOut, TValue> TransformMapKeys(const TMap<TKeyIn, TValue>& Input,
                                                   TFunction<TKeyOut(const TKeyIn&)> KeyTransform) {
    TMap<TKeyOut, TValue> Result;
    for (const auto& Pair : Input) {
        Result.Add(KeyTransform(Pair.Key), Pair.Value);
    }
    return Result;
}

/**
 * Extracts all keys from a map into a TArray.
 * Useful for building search sets or looping independently over keys.
 */
template <typename TKey, typename TValue> FORCEINLINE TArray<TKey> ExtractMapKeys(const TMap<TKey, TValue>& Map) {
    TArray<TKey> Keys;
    Keys.Reserve(Map.Num());
    for (const auto& Pair : Map) {
        Keys.Add(Pair.Key);
    }
    return Keys;
}

/**
 * Extracts all values from a map into a TArray.
 * Useful for collecting data or computing statistics from map contents.
 */
template <typename TKey, typename TValue> TArray<TValue> ExtractMapValues(const TMap<TKey, TValue>& Map) {
    TArray<TValue> Values;
    Values.Reserve(Map.Num());
    for (const auto& Pair : Map) {
        Values.Add(Pair.Value);
    }
    return Values;
}

/**
 * Checks if any of the given keys exist in the map.
 * Use to short-circuit operations or validate relevance.
 */
template <typename TKey, typename TValue, typename KeyContainerType>
FORCEINLINE bool ContainsAnyKeys(const TMap<TKey, TValue>& Map, const KeyContainerType& Keys) {
    for (const TKey& Key : Keys) {
        if (Map.Contains(Key)) {
            return true;
        }
    }
    return false;
}

/**
 * Removes a batch of keys from a map.
 * Safely skips any keys that don't exist.
 */
template <typename TKey, typename TValue, typename KeyArrayType>
FORCEINLINE int32 RemoveKeys(TMap<TKey, TValue>& Map, const KeyArrayType& Keys) {
    int32 NumRemoved = 0;
    for (const TKey& Key : Keys) {
        if (Map.Remove(Key) > 0) {
            ++NumRemoved;
        }
    }
    return NumRemoved;
}

/**
 * Creates a new map where each key is remapped using a provided transform function.
 * Values remain unchanged.
 * Useful for adapting formats or normalizing map key structures.
 */
template <typename TKeyOld, typename TKeyNew, typename TValue, typename KeyMapperType>
FORCEINLINE TMap<TKeyNew, TValue> RemapKeys(const TMap<TKeyOld, TValue>& Map, KeyMapperType&& KeyMapper) {
    TMap<TKeyNew, TValue> Result;
    for (const auto& Pair : Map) {
        Result.Add(KeyMapper(Pair.Key), Pair.Value);
    }
    return Result;
}

template <typename TKeyOld, typename TKeyNew, typename TValue>
FORCEINLINE TMap<TKeyNew, TValue> RemapKeys(const TMap<TKeyOld, TValue>& Map,
                                            TFunction<TKeyNew(const TKeyOld&)> KeyMapper) {
    TMap<TKeyNew, TValue> Result;
    for (const auto& Pair : Map) {
        Result.Add(KeyMapper(Pair.Key), Pair.Value);
    }
    return Result;
}

/**
 * Aggregates all key-value pairs in a map into a single result.
 * A generalized reduction (fold) operation.
 */
template <typename TKey, typename TValue, typename TResult, typename ReducerType>
FORCEINLINE TResult ReduceMap(const TMap<TKey, TValue>& Map, TResult InitialValue, ReducerType&& Reducer) {
    TResult Result = InitialValue;
    for (const auto& Pair : Map) {
        Result = Reducer(Result, Pair.Key, Pair.Value);
    }
    return Result;
}

template <typename TKey, typename TValue, typename TResult>
FORCEINLINE TResult ReduceMap(const TMap<TKey, TValue>& Map, TResult InitialValue,
                              TFunction<TResult(const TResult&, const TKey&, const TValue&)> Reducer) {
    TResult Result = InitialValue;
    for (const auto& Pair : Map) {
        Result = Reducer(Result, Pair.Key, Pair.Value);
    }
    return Result;
}

/**
 * Returns the first key-value pair in the map where a predicate is true.
 * Use this to perform fast validation or lightweight lookups.
 */
template <typename TKey, typename TValue, typename PredicateType>
FORCEINLINE TOptional<TPair<TKey, TValue>> FindFirstMatch(const TMap<TKey, TValue>& Map, PredicateType&& Predicate) {
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Key, Pair.Value)) {
            return Pair;
        }
    }
    return {};
}

template <typename TKey, typename TValue>
FORCEINLINE TOptional<TPair<TKey, TValue>> FindFirstMatch(const TMap<TKey, TValue>& Map,
                                                          TFunction<bool(const TKey&, const TValue&)> Predicate) {
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Key, Pair.Value)) {
            return Pair;
        }
    }
    return {};
}

/**
 * Maps each value of a map into an output array using a transform function.
 * Keys are ignored — this is for value projection only.
 */
template <typename TKey, typename TValue, typename TOut, typename TransformType>
FORCEINLINE TArray<TOut> MapValuesToArray(const TMap<TKey, TValue>& Map, TransformType&& Transform) {
    TArray<TOut> Out;
    Out.Reserve(Map.Num());
    for (const auto& Pair : Map) {
        Out.Add(Transform(Pair.Value));
    }
    return Out;
}

template <typename TKey, typename TValue, typename TOut>
FORCEINLINE TArray<TOut> MapValuesToArray(const TMap<TKey, TValue>& Map, TFunction<TOut(const TValue&)> Transform) {
    TArray<TOut> Out;
    Out.Reserve(Map.Num());
    for (const auto& Pair : Map) {
        Out.Add(Transform(Pair.Value));
    }
    return Out;
}

/**
 * Returns the intersection of keys between two maps.
 * Useful for syncing two systems or checking shared control structures.
 */
template <typename TKey, typename TValueA, typename TValueB>
TSet<TKey> IntersectKeySets(const TMap<TKey, TValueA>& A, const TMap<TKey, TValueB>& B) {
    TSet<TKey> KeysA, KeysB;
    A.GetKeys(KeysA);
    B.GetKeys(KeysB);
    return KeysA.Intersect(KeysB);
}

template <typename TKey, typename TValue, typename ValueSimilarityFuncType>
FORCEINLINE float ScoreMapSimilarity(const TMap<TKey, TValue>& A, const TMap<TKey, TValue>& B,
                                     ValueSimilarityFuncType&& ValueSimilarityFunc) {
    if (A.Num() == 0 && B.Num() == 0) {
        return 1.0f;
    }
    if (A.Num() == 0 || B.Num() == 0) {
        return 0.0f;
    }

    int32 MatchingKeys = 0;
    float TotalScore = 0.0f;

    for (const auto& Pair : A) {
        if (const TValue* BValue = B.Find(Pair.Key)) {
            ++MatchingKeys;
            TotalScore += ValueSimilarityFunc(Pair.Value, *BValue);
        }
    }

    const float MaxPossible = static_cast<float>(FMath::Min(A.Num(), B.Num()));
    return MaxPossible > 0 ? TotalScore / MaxPossible : 0.f;
}

template <typename TKey, typename TValue>
FORCEINLINE float ScoreMapSimilarity(const TMap<TKey, TValue>& A, const TMap<TKey, TValue>& B,
                                     TFunction<float(const TValue&, const TValue&)> ValueSimilarityFunc) {
    if (A.Num() == 0 && B.Num() == 0) {
        return 1.0f;
    }
    if (A.Num() == 0 || B.Num() == 0) {
        return 0.0f;
    }

    int32 MatchingKeys = 0;
    float TotalScore = 0.0f;

    for (const auto& Pair : A) {
        if (const TValue* BValue = B.Find(Pair.Key)) {
            ++MatchingKeys;
            TotalScore += ValueSimilarityFunc(Pair.Value, *BValue);
        }
    }

    const float MaxPossible = static_cast<float>(FMath::Min(A.Num(), B.Num()));
    return MaxPossible > 0 ? TotalScore / MaxPossible : 0.f;
}

template <typename TKey, typename TValue, typename MetricFuncType>
FORCEINLINE void RemoveMapOutliers(TMap<TKey, TValue>& Map, MetricFuncType&& MetricFunc, float ThresholdZScore = 2.0f) {
    TArray<float> Metrics;
    for (const auto& Pair : Map) {
        Metrics.Add(MetricFunc(Pair.Value));
    }

    if (Metrics.Num() < 2) {
        return;
    }

    const float Mean = Algo::Accumulate(Metrics, 0.0f) / Metrics.Num();
    const float Variance =
        Algo::Accumulate(Metrics, 0.0f, [=](float Acc, float V) { return Acc + FMath::Square(V - Mean); }) /
        Metrics.Num();

    const float StdDev = FMath::Sqrt(Variance);

    for (auto It = Map.CreateIterator(); It; ++It) {
        const float Z = FMath::Abs((MetricFunc(It->Value) - Mean) / (StdDev + KINDA_SMALL_NUMBER));
        if (Z > ThresholdZScore) {
            It.RemoveCurrent();
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void RemoveMapOutliers(TMap<TKey, TValue>& Map, TFunction<float(const TValue&)> MetricFunc,
                                   float ThresholdZScore = 2.0f) {
    TArray<float> Metrics;
    for (const auto& Pair : Map) {
        Metrics.Add(MetricFunc(Pair.Value));
    }

    if (Metrics.Num() < 2) {
        return;
    }

    const float Mean = Algo::Accumulate(Metrics, 0.0f) / Metrics.Num();
    const float Variance =
        Algo::Accumulate(Metrics, 0.0f, [=](float Acc, float V) { return Acc + FMath::Square(V - Mean); }) /
        Metrics.Num();

    const float StdDev = FMath::Sqrt(Variance);

    for (auto It = Map.CreateIterator(); It; ++It) {
        const float Z = FMath::Abs((MetricFunc(It->Value) - Mean) / (StdDev + KINDA_SMALL_NUMBER));
        if (Z > ThresholdZScore) {
            It.RemoveCurrent();
        }
    }
}

template <typename TKey, typename TValue, typename ExtractFuncType, typename ApplyFuncType>
FORCEINLINE void NormalizeMapValues(TMap<TKey, TValue>& Map, ExtractFuncType&& ExtractFunc, ApplyFuncType&& ApplyFunc) {
    if (Map.Num() == 0) {
        return;
    }

    float Min = TNumericLimits<float>::Max();
    float Max = TNumericLimits<float>::Lowest();

    for (const auto& Pair : Map) {
        const float Value = ExtractFunc(Pair.Value);
        Min = FMath::Min(Min, Value);
        Max = FMath::Max(Max, Value);
    }

    const float Range = FMath::Max(Max - Min, KINDA_SMALL_NUMBER);

    for (auto& Pair : Map) {
        const float Value = ExtractFunc(Pair.Value);
        const float Normalized = (Value - Min) / Range;
        ApplyFunc(Pair.Value, Normalized);
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void NormalizeMapValues(TMap<TKey, TValue>& Map, TFunction<float(const TValue&)> ExtractFunc,
                                    TFunction<void(TValue&, float)> ApplyFunc) {
    if (Map.Num() == 0) {
        return;
    }

    float Min = TNumericLimits<float>::Max();
    float Max = TNumericLimits<float>::Lowest();

    for (const auto& Pair : Map) {
        const float Value = ExtractFunc(Pair.Value);
        Min = FMath::Min(Min, Value);
        Max = FMath::Max(Max, Value);
    }

    const float Range = FMath::Max(Max - Min, KINDA_SMALL_NUMBER);

    for (auto& Pair : Map) {
        const float Value = ExtractFunc(Pair.Value);
        const float Normalized = (Value - Min) / Range;
        ApplyFunc(Pair.Value, Normalized);
    }
}

template <typename TKey, typename TValue> float EntropyOfMapKeys(const TMap<TKey, TValue>& Map) {
    if (Map.Num() == 0) {
        return 0.f;
    }

    TMap<TKey, int32> Frequency;
    for (const auto& Pair : Map) {
        ++Frequency.FindOrAdd(Pair.Key);
    }

    const float Total = static_cast<float>(Map.Num());
    float Entropy = 0.f;

    for (const auto& Pair : Frequency) {
        const float P = Pair.Value / Total;
        Entropy -= P * FMath::Log2(P);
    }
    return Entropy;
}

template <typename TKey, typename TValue, typename WeightFuncType>
FORCEINLINE TOptional<TValue> SelectWeightedRandom(const TMap<TKey, TValue>& Map, WeightFuncType&& WeightFunc) {
    if (Map.Num() == 0) {
        return {};
    }

    float TotalWeight = 0.f;
    for (const auto& Pair : Map) {
        TotalWeight += WeightFunc(Pair.Value);
    }

    if (TotalWeight <= 0.f) {
        return {};
    }

    const float Threshold = FMath::FRandRange(0.f, TotalWeight);
    float Accum = 0.f;

    for (const auto& Pair : Map) {
        Accum += WeightFunc(Pair.Value);
        if (Accum >= Threshold) {
            return Pair.Value;
        }
    }

    return {};
}

template <typename TKey, typename TValue>
FORCEINLINE TOptional<TValue> SelectWeightedRandom(const TMap<TKey, TValue>& Map,
                                                   TFunction<float(const TValue&)> WeightFunc) {
    if (Map.Num() == 0) {
        return {};
    }

    float TotalWeight = 0.f;
    for (const auto& Pair : Map) {
        TotalWeight += WeightFunc(Pair.Value);
    }

    if (TotalWeight <= 0.f) {
        return {};
    }

    const float Threshold = FMath::FRandRange(0.f, TotalWeight);
    float Accum = 0.f;

    for (const auto& Pair : Map) {
        Accum += WeightFunc(Pair.Value);
        if (Accum >= Threshold) {
            return Pair.Value;
        }
    }

    return {};
}

template <typename TKey, typename TValue, typename IsConflictType, typename MergeFuncType>
FORCEINLINE void ResolveConflictingEntries(TMap<TKey, TValue>& Map, IsConflictType&& IsConflict,
                                           MergeFuncType&& MergeFunc) {
    TSet<TKey> Keys = Map.GetKeys();
    for (const TKey& Key : Keys) {
        for (const TKey& OtherKey : Keys) {
            if (Key != OtherKey && Map.Contains(Key) && Map.Contains(OtherKey)) {
                TValue& A = Map[Key];
                TValue& B = Map[OtherKey];
                if (IsConflict(A, B)) {
                    Map[Key] = MergeFunc(A, B);
                    Map.Remove(OtherKey);
                }
            }
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void ResolveConflictingEntries(TMap<TKey, TValue>& Map,
                                           TFunction<bool(const TValue&, const TValue&)> IsConflict,
                                           TFunction<TValue(const TValue&, const TValue&)> MergeFunc) {
    TSet<TKey> Keys = Map.GetKeys();
    for (const TKey& Key : Keys) {
        for (const TKey& OtherKey : Keys) {
            if (Key != OtherKey && Map.Contains(Key) && Map.Contains(OtherKey)) {
                TValue& A = Map[Key];
                TValue& B = Map[OtherKey];
                if (IsConflict(A, B)) {
                    Map[Key] = MergeFunc(A, B);
                    Map.Remove(OtherKey);
                }
            }
        }
    }
}

template <typename TKey, typename TValue, typename ConflictResolverType>
FORCEINLINE TMap<TKey, TValue> MergeMapsWithResolution(const TMap<TKey, TValue>& A, const TMap<TKey, TValue>& B,
                                                       ConflictResolverType&& ConflictResolver) {
    TMap<TKey, TValue> Result = A;
    for (const auto& Pair : B) {
        if (Result.Contains(Pair.Key)) {
            Result[Pair.Key] = ConflictResolver(Result[Pair.Key], Pair.Value);
        } else {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename TKey, typename TValue>
FORCEINLINE TMap<TKey, TValue>
MergeMapsWithResolution(const TMap<TKey, TValue>& A, const TMap<TKey, TValue>& B,
                        TFunction<TValue(const TValue&, const TValue&)> ConflictResolver) {
    TMap<TKey, TValue> Result = A;
    for (const auto& Pair : B) {
        if (Result.Contains(Pair.Key)) {
            Result[Pair.Key] = ConflictResolver(Result[Pair.Key], Pair.Value);
        } else {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename TKey, typename TValue, typename TGroupKey, typename ClassifierType>
FORCEINLINE TMap<TGroupKey, TArray<TValue>> MapGroupBy(const TMap<TKey, TValue>& Map, ClassifierType&& Classifier) {
    TMap<TGroupKey, TArray<TValue>> Grouped;

    for (const auto& Pair : Map) {
        TGroupKey Key = Classifier(Pair.Key, Pair.Value);
        Grouped.FindOrAdd(Key).Add(Pair.Value);
    }
    return Grouped;
}

template <typename TKey, typename TValue, typename TGroupKey>
FORCEINLINE TMap<TGroupKey, TArray<TValue>> MapGroupBy(const TMap<TKey, TValue>& Map,
                                                       TFunction<TGroupKey(const TKey&, const TValue&)> Classifier) {
    TMap<TGroupKey, TArray<TValue>> Grouped;

    for (const auto& Pair : Map) {
        TGroupKey Key = Classifier(Pair.Key, Pair.Value);
        Grouped.FindOrAdd(Key).Add(Pair.Value);
    }
    return Grouped;
}

template <typename TKey, typename TValue, typename IsStableType>
FORCEINLINE TArray<TKey> DetectStableValues(const TMap<TKey, TValue>& Current, const TMap<TKey, TValue>& Previous,
                                            IsStableType&& IsStable) {
    TArray<TKey> StableKeys;
    for (const auto& Pair : Current) {
        const TValue* Prev = Previous.Find(Pair.Key);
        if (Prev && IsStable(Pair.Value, *Prev)) {
            StableKeys.Add(Pair.Key);
        }
    }
    return StableKeys;
}

template <typename TKey, typename TValue>
FORCEINLINE TArray<TKey> DetectStableValues(const TMap<TKey, TValue>& Current, const TMap<TKey, TValue>& Previous,
                                            TFunction<bool(const TValue&, const TValue&)> IsStable) {
    TArray<TKey> StableKeys;
    for (const auto& Pair : Current) {
        const TValue* Prev = Previous.Find(Pair.Key);
        if (Prev && IsStable(Pair.Value, *Prev)) {
            StableKeys.Add(Pair.Key);
        }
    }
    return StableKeys;
}

// ============================================================================
// Resolver & Heuristic Repair Algorithms (31–35)
// ============================================================================

/**
 * Auto-fills missing keys in a map by applying an inference function
 * based on other values or metadata in the existing map.
 *
 * Useful for filling out incomplete datasets (e.g. bone poses, input maps).
 */
template <typename TKey, typename TValue, typename InferFuncType>
FORCEINLINE void AutoFillMissingKeysWithInference(TMap<TKey, TValue>& Map, const TSet<TKey>& RequiredKeys,
                                                  InferFuncType&& InferFunc) {
    for (const TKey& Key : RequiredKeys) {
        if (!Map.Contains(Key)) {
            if (TOptional<TValue> Inferred = InferFunc(Key, Map)) {
                Map.Add(Key, Inferred.GetValue());
            }
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void AutoFillMissingKeysWithInference(
    TMap<TKey, TValue>& Map, const TSet<TKey>& RequiredKeys,
    TFunction<TOptional<TValue>(const TKey& MissingKey, const TMap<TKey, TValue>& Existing)> InferFunc) {
    for (const TKey& Key : RequiredKeys) {
        if (!Map.Contains(Key)) {
            if (TOptional<TValue> Inferred = InferFunc(Key, Map)) {
                Map.Add(Key, Inferred.GetValue());
            }
        }
    }
}

template <typename TSource, typename TKey, typename TValue, typename GetKeyType, typename MakeValueType>
FORCEINLINE TMap<TKey, TValue> BuildTypedMap(const TArray<TSource*>& SourceArray, GetKeyType&& GetKey,
                                             MakeValueType&& MakeValue) {
    TMap<TKey, TValue> Out;
    for (const TSource* Source : SourceArray) {
        if (!Source) {
            continue;
        }
        Out.Add(GetKey(Source), MakeValue(Source));
    }
    return Out;
}

template <typename TSource, typename TKey, typename TValue>
FORCEINLINE TMap<TKey, TValue> BuildTypedMap(const TArray<TSource*>& SourceArray,
                                             TFunctionRef<TKey(const TSource*)> GetKey,
                                             TFunctionRef<TValue(const TSource*)> MakeValue) {
    TMap<TKey, TValue> Out;
    for (const TSource* Source : SourceArray) {
        if (!Source) {
            continue;
        }
        Out.Add(GetKey(Source), MakeValue(Source));
    }
    return Out;
}

/**
 * Finds duplicate or semantically equivalent values in a map and replaces
 * them with a canonical, merged, or optimized version.
 *
 * Can reduce memory cost or stabilize behavior across redundant mappings.
 */
template <typename TKey, typename TValue, typename AreEquivalentType, typename CanonicalizerType>
FORCEINLINE void ResolveDuplicateValuesToCanonical(TMap<TKey, TValue>& Map, AreEquivalentType&& AreEquivalent,
                                                   CanonicalizerType&& Canonicalizer) {
    TArray<TValue> Unique;
    TMap<TValue, TValue> CanonicalMap;

    for (const auto& Pair : Map) {
        bool Found = false;
        for (const TValue& Existing : Unique) {
            if (AreEquivalent(Pair.Value, Existing)) {
                CanonicalMap.Add(Pair.Value, Existing);
                Found = true;
                break;
            }
        }
        if (!Found) {
            Unique.Add(Pair.Value);
        }
    }

    for (const TValue& Val : Unique) {
        TArray<TValue> Equivalents;
        for (const auto& P : Map) {
            if (AreEquivalent(P.Value, Val)) {
                Equivalents.Add(P.Value);
            }
        }
        TValue Canonical = Canonicalizer(Equivalents);
        for (auto& Pair : Map) {
            if (AreEquivalent(Pair.Value, Val)) {
                Pair.Value = Canonical;
            }
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void ResolveDuplicateValuesToCanonical(TMap<TKey, TValue>& Map,
                                                   TFunction<bool(const TValue&, const TValue&)> AreEquivalent,
                                                   TFunction<TValue(const TArray<TValue>&)> Canonicalizer) {
    TArray<TValue> Unique;
    TMap<TValue, TValue> CanonicalMap;

    for (const auto& Pair : Map) {
        bool Found = false;
        for (const TValue& Existing : Unique) {
            if (AreEquivalent(Pair.Value, Existing)) {
                CanonicalMap.Add(Pair.Value, Existing);
                Found = true;
                break;
            }
        }
        if (!Found) {
            Unique.Add(Pair.Value);
        }
    }

    for (const TValue& Val : Unique) {
        TArray<TValue> Equivalents;
        for (const auto& P : Map) {
            if (AreEquivalent(P.Value, Val)) {
                Equivalents.Add(P.Value);
            }
        }
        TValue Canonical = Canonicalizer(Equivalents);
        for (auto& Pair : Map) {
            if (AreEquivalent(Pair.Value, Val)) {
                Pair.Value = Canonical;
            }
        }
    }
}

/**
 * Adds default values to a map where keys match a known naming or structural pattern.
 * This is useful for procedural systems where structured identifiers like "Bone_01" exist.
 */
template <typename TKey, typename TValue, typename MatchesPatternType, typename GenerateDefaultType>
FORCEINLINE void FillMapDefaultsByPatternMatching(TMap<TKey, TValue>& Map, const TSet<TKey>& RequiredKeys,
                                                  MatchesPatternType&& MatchesPattern,
                                                  GenerateDefaultType&& GenerateDefault) {
    for (const TKey& Key : RequiredKeys) {
        if (!Map.Contains(Key) && MatchesPattern(Key)) {
            Map.Add(Key, GenerateDefault(Key));
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void FillMapDefaultsByPatternMatching(TMap<TKey, TValue>& Map, const TSet<TKey>& RequiredKeys,
                                                  TFunction<bool(const TKey&)> MatchesPattern,
                                                  TFunction<TValue(const TKey&)> GenerateDefault) {
    for (const TKey& Key : RequiredKeys) {
        if (!Map.Contains(Key) && MatchesPattern(Key)) {
            Map.Add(Key, GenerateDefault(Key));
        }
    }
}

/**
 * For maps where a key maps to multiple possible values, this function selects
 * the best one using a scoring heuristic and discards the rest.
 *
 * Useful for resolving IK targets, animation states, UI bindings, etc.
 */
template <typename TKey, typename TValue, typename ScoreFuncType>
FORCEINLINE void ResolveAmbiguousMappings(TMap<TKey, TArray<TValue>>& AmbiguousMap, ScoreFuncType&& ScoreFunc) {
    for (auto& Pair : AmbiguousMap) {
        if (Pair.Value.Num() <= 1) {
            continue;
        }

        TValue* Best = nullptr;
        float BestScore = -FLT_MAX;

        for (TValue& Candidate : Pair.Value) {
            float Score = ScoreFunc(Pair.Key, Candidate);
            if (Score > BestScore) {
                BestScore = Score;
                Best = &Candidate;
            }
        }

        if (Best) {
            Pair.Value = {*Best}; // Reduce to the best match
        }
    }
}

template <typename TKey, typename TValue>
FORCEINLINE void ResolveAmbiguousMappings(TMap<TKey, TArray<TValue>>& AmbiguousMap,
                                          TFunction<float(const TKey&, const TValue&)> ScoreFunc) {
    for (auto& Pair : AmbiguousMap) {
        if (Pair.Value.Num() <= 1) {
            continue;
        }

        TValue* Best = nullptr;
        float BestScore = -FLT_MAX;

        for (TValue& Candidate : Pair.Value) {
            float Score = ScoreFunc(Pair.Key, Candidate);
            if (Score > BestScore) {
                BestScore = Score;
                Best = &Candidate;
            }
        }

        if (Best) {
            Pair.Value = {*Best}; // Reduce to the best match
        }
    }
}

/**
 * Backfills missing keys in a target map using values from a reference map.
 * This is useful when restoring sync from a canonical data source.
 */
template <typename TKey, typename TValue>
void BackfillMissingMapValuesFromReferenceMap(TMap<TKey, TValue>& Target, const TMap<TKey, TValue>& Reference) {
    for (const auto& Pair : Reference) {
        if (!Target.Contains(Pair.Key)) {
            Target.Add(Pair.Key, Pair.Value);
        }
    }
}

// ============================================================================
// 🛡 Pointer & Object Safety Utilities (36–40)
// ============================================================================

/**
 * Removes map entries whose values are null or represent invalid UObjects.
 * Helps prevent crashes during iteration, especially in GC-managed systems.
 */
template <typename TKey, typename TObject> void RemoveNullOrInvalidUObjects(TMap<TKey, TObject*>& Map) {
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (It->Value == nullptr || !IsValid(It->Value)) {
            It.RemoveCurrent();
        }
    }
}

/**
 * Returns a filtered copy of the input map where all values are valid (non-null, non-destroyed).
 * This protects against dereferencing or operating on stale object references.
 */
template <typename TKey, typename TValue> TMap<TKey, TValue*> FilterMapToValidPointers(const TMap<TKey, TValue*>& Map) {
    TMap<TKey, TValue*> Result;
    for (const auto& Pair : Map) {
        if (IsValid(Pair.Value)) {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

/**
 * Creates a copy of a map by dereferencing only valid pointers.
 * Skips null pointers to avoid crashes.
 */
template <typename TKey, typename TObject>
TMap<TKey, TObject> DereferenceMapWithCheck(const TMap<TKey, TObject*>& Source) {
    TMap<TKey, TObject> Output;
    for (const auto& Pair : Source) {
        if (Pair.Value) {
            Output.Add(Pair.Key, *Pair.Value);
        }
    }
    return Output;
}

/**
 * Applies a callback to each valid object pointer in a map without modifying it.
 * Ensures safe iteration and access, skipping invalid objects.
 */
template <typename TKey, typename TObject, typename CallbackType>
FORCEINLINE void SafeApplyToLiveObjects(const TMap<TKey, TObject*>& Map, CallbackType&& Callback) {
    for (const auto& Pair : Map) {
        if (IsValid(Pair.Value)) {
            Callback(Pair.Key, Pair.Value);
        }
    }
}

template <typename TKey, typename TObject>
FORCEINLINE void SafeApplyToLiveObjects(const TMap<TKey, TObject*>& Map,
                                        TFunctionRef<void(const TKey&, TObject*)> Callback) {
    for (const auto& Pair : Map) {
        if (IsValid(Pair.Value)) {
            Callback(Pair.Key, Pair.Value);
        }
    }
}

/**
 * Removes entries from an actor pointer map where the actor is destroyed or invalid.
 * Protects systems from accessing pending-kill actors (e.g., during GC or cleanup).
 */
template <typename TKey> void RemoveStaleReferencesToDestroyedActors(TMap<TKey, AActor*>& Map) {
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (!IsValid(It->Value) || It->Value->IsPendingKill()) {
            It.RemoveCurrent();
        }
    }
}

// ============================================================================
//  Defensive Runtime Cleanup & Type Safety (41–45)
// ============================================================================

/**
 * Removes entries where the key and value are the same.
 * This is useful in self-referencing maps to avoid recursion or feedback loops.
 */
template <typename TKey> void RemoveSelfReferences(TMap<TKey, TKey>& Map) {
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (It->Key == It->Value) {
            It.RemoveCurrent();
        }
    }
}

/**
 * Deduplicates values in pointer maps, keeping only the first unique reference.
 * Ensures multiple keys don't point to the exact same object unless intentional.
 */
template <typename TKey, typename TValue> void PurgeDuplicatePointerValues(TMap<TKey, TValue*>& Map) {
    TSet<TValue*> Seen;
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (!Seen.Add(It->Value)) {
            It.RemoveCurrent(); // already seen this pointer
        }
    }
}

/**
 * Validates that all UObject* values in the map are of a required class type (or derived from it).
 * Useful for enforcing component types or interface compliance.
 */
template <typename TKey> void ValidateMapObjectTypes(TMap<TKey, UObject*>& Map, UClass* RequiredClass) {
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (!IsValid(It->Value) || !It->Value->IsA(RequiredClass)) {
            It.RemoveCurrent();
        }
    }
}

/**
 * Removes references to objects in worlds that are no longer loaded or valid.
 * Prevents use-after-destroy on level transitions or seamless travel.
 */
template <typename TKey, typename TObject> void RemoveObjectsInUnloadedWorlds(TMap<TKey, TObject*>& Map) {
    for (auto It = Map.CreateIterator(); It; ++It) {
        if (!IsValid(It->Value) || It->Value->GetWorld() == nullptr) {
            It.RemoveCurrent();
        }
    }
}

/**
 * Removes any components that are detached, invalid, or whose owning actor is no longer valid.
 * Protects against runtime errors from stale UI, scene, or actor-bound systems.
 */
template <typename TKey, typename TComponent> void RemoveInvalidComponentReferences(TMap<TKey, TComponent*>& Map) {
    static_assert(TIsDerivedFrom<TComponent, UActorComponent>::IsDerived, "Must be a UActorComponent-derived type");

    for (auto It = Map.CreateIterator(); It; ++It) {
        TComponent* Component = It->Value;
        if (!IsValid(Component) || !IsValid(Component->GetOwner())) {
            It.RemoveCurrent();
        }
    }
}

// ============================================================================
//  Enum Indirection & Semantic Mapping Utilities (56–60)
// ============================================================================

/**
 * Detects enum values assigned to multiple semantic groups, which may indicate
 * an ambiguous or conflicting classification.
 *
 * Example: "LeftArm" enum being part of both "UpperLimb" and "Spine".
 */
template <typename TEnum> TSet<TEnum> DetectConflictingEnumAssignments(const TMap<TEnum, TSet<FName>>& EnumToGroupMap) {
    TMap<FName, TSet<TEnum>> GroupMemberships;
    TSet<TEnum> Conflicted;

    for (const auto& Pair : EnumToGroupMap) {
        for (const FName& Group : Pair.Value) {
            GroupMemberships.FindOrAdd(Group).Add(Pair.Key);
        }
    }

    TMap<TEnum, int32> MembershipCount;
    for (const auto& Group : GroupMemberships) {
        for (const TEnum& Value : Group.Value) {
            int32& Count = MembershipCount.FindOrAdd(Value);
            if (++Count > 1) {
                Conflicted.Add(Value);
            }
        }
    }

    return Conflicted;
}

/**
 * Validates that there are no recursive or cyclic enum mapping chains.
 * For example, A→B→C→A is invalid and would be detected.
 *
 * Outputs the first cycle found in OutCycle.
 */
template <typename TEnum>
bool ValidateEnumClosureChains(const TMap<TEnum, TEnum>& IndirectionMap, TArray<TEnum>& OutCycle) {
    TSet<TEnum> Visited;

    for (const auto& Pair : IndirectionMap) {
        if (Visited.Contains(Pair.Key)) {
            continue; // Skip keys already checked
        }

        TSet<TEnum> Chain;
        TEnum Current = Pair.Key;

        while (IndirectionMap.Contains(Current)) {
            if (!Chain.Add(Current)) {
                // Cycle detected — copy the chain path to OutCycle
                OutCycle = Chain.Array();
                return false;
            }
            Visited.Add(Current);
            Current = IndirectionMap[Current];
        }
    }

    return true;
}

/**
 * Returns the chain of semantic remapping from a starting enum
 * to its final resolved semantic identity.
 *
 * Use to debug or verify enum resolution flows.
 */
template <typename TEnum> TArray<TEnum> TraceEnumMappingPath(const TMap<TEnum, TEnum>& IndirectionMap, TEnum Start) {
    TArray<TEnum> Path;
    TSet<TEnum> Visited;

    TEnum Current = Start;
    while (IndirectionMap.Contains(Current)) {
        if (!Visited.Add(Current)) {
            break; // cycle or loop
        }

        Path.Add(Current);
        Current = IndirectionMap[Current];
    }

    Path.Add(Current); // Final-resolved enum
    return Path;
}

/**
 * Generates the reverse map of a semantic remapping structure.
 * E.g., if A→Group1, B→Group1, returns Group1 → {A, B}.
 */
template <typename TEnum> TMap<TEnum, TArray<TEnum>> GenerateInverseEnumMap(const TMap<TEnum, TEnum>& ForwardMap) {
    TMap<TEnum, TArray<TEnum>> Inverse;
    for (const auto& Pair : ForwardMap) {
        Inverse.FindOrAdd(Pair.Value).Add(Pair.Key);
    }
    return Inverse;
}

/**
 * Checks which enum values from an expected set are missing from the mapped set.
 * Useful for validating full coverage of a skeletal rig or input schema.
 */
template <typename TEnum>
TSet<TEnum> ValidateCompleteEnumCoverage(const TSet<TEnum>& AllExpectedEnums, const TSet<TEnum>& UsedEnums) {
    TSet<TEnum> Missing = AllExpectedEnums.Difference(UsedEnums);
    return Missing;
}

// ============================================================================
// 🧠 Semantic Indirection Debug & Consistency Validators (61–65)
// ============================================================================

/**
 * Detects enums that are used as remap targets but never as remap sources.
 * These are potentially unused semantic categories or structural leftovers.
 */
template <typename TEnum> TSet<TEnum> DetectUnreachableSemanticTargets(const TMap<TEnum, TEnum>& IndirectionMap) {
    TSet<TEnum> AllTargets;
    TSet<TEnum> AllSources;

    for (const auto& Pair : IndirectionMap) {
        AllSources.Add(Pair.Key);
        AllTargets.Add(Pair.Value);
    }

    return AllTargets.Difference(AllSources);
}

/**
 * Detects enums that map to themselves (A → A), which is usually unintentional
 * and may cause incorrect indirection logic.
 */
template <typename TEnum> TSet<TEnum> DetectSelfReferentialEnums(const TMap<TEnum, TEnum>& IndirectionMap) {
    TSet<TEnum> SelfReferencing;
    for (const auto& Pair : IndirectionMap) {
        if (Pair.Key == Pair.Value) {
            SelfReferencing.Add(Pair.Key);
        }
    }
    return SelfReferencing;
}

/**
 * Identifies target enums that are referenced by multiple different source enums.
 * This can indicate semantic overlaps or mapping collisions.
 */
template <typename TEnum>
TMap<TEnum, TArray<TEnum>> DetectShadowedEnumMappings(const TMap<TEnum, TEnum>& IndirectionMap) {
    TMap<TEnum, TArray<TEnum>> TargetToSources;

    for (const auto& Pair : IndirectionMap) {
        TargetToSources.FindOrAdd(Pair.Value).Add(Pair.Key);
    }

    TMap<TEnum, TArray<TEnum>> Shadowed;
    for (const auto& Pair : TargetToSources) {
        if (Pair.Value.Num() > 1) {
            Shadowed.Add(Pair.Key, Pair.Value);
        }
    }

    return Shadowed;
}

/**
 * Resolves the final destination of each enum through all indirection chains.
 * This flattens the structure and resolves nested mappings into single-layer output.
 */
template <typename TEnum> TMap<TEnum, TEnum> FlattenEnumIndirection(const TMap<TEnum, TEnum>& IndirectionMap) {
    TMap<TEnum, TEnum> Flat;

    for (const auto& Pair : IndirectionMap) {
        TEnum Current = Pair.Value;
        TSet<TEnum> Visited;

        while (IndirectionMap.Contains(Current)) {
            if (!Visited.Add(Current)) {
                break; // cycle
            }
            Current = IndirectionMap[Current];
        }

        Flat.Add(Pair.Key, Current);
    }

    return Flat;
}

/**
 * Verifies that every source enum maps to a unique target (injective mapping).
 * This is critical when 1:1 semantic relationships are required.
 */
template <typename TEnum> bool VerifyMappingInjectivity(const TMap<TEnum, TEnum>& IndirectionMap) {
    TSet<TEnum> Targets;
    for (const auto& Pair : IndirectionMap) {
        if (!Targets.Add(Pair.Value)) {
            return false; // multiple sources mapped to same target
        }
    }
    return true;
}

// ============================================================================
// 🧾 Struct ↔ Object State Diffing, Syncing & Introspection (71–75)
// ============================================================================

/**
 * Returns a diff report between a struct and an object, listing all differing fields
 * with their struct and object values.
 */
template <typename TStruct, typename TObject>
TMap<FName, FString> StructObjectDiffReport(const TStruct& Struct, const TObject* Object) {
    TMap<FName, FString> Report;

    if (!IsValid(Object)) {
        return Report;
    }

    for (TFieldIterator<FProperty> PropIt(TStruct::StaticStruct()); PropIt; ++PropIt) {
        const FProperty* StructProp = *PropIt;
        FProperty* ObjectProp = FindFProperty<FProperty>(Object->GetClass(), StructProp->GetFName());

        if (ObjectProp && StructProp->SameType(ObjectProp)) {
            const void* StructValue = StructProp->ContainerPtrToValuePtr<const void>(&Struct);
            const void* ObjectValue = ObjectProp->ContainerPtrToValuePtr<const void>(Object);

            if (!StructProp->Identical(StructValue, ObjectValue)) {
                FString StructStr, ObjectStr;
                StructProp->ExportTextItem_Direct(StructStr, StructValue, nullptr, nullptr, PPF_None);
                ObjectProp->ExportTextItem_Direct(ObjectStr, ObjectValue, nullptr, nullptr, PPF_None);

                Report.Add(StructProp->GetFName(),
                           FString::Printf(TEXT("Struct: %s | Object: %s"), *StructStr, *ObjectStr));
            }
        }
    }
    return Report;
}

/**
 * Checks if a struct is in its default-initialized state.
 * Useful for detecting uninitialized or reset state containers.
 */
template <typename TStruct> bool IsStructDefault(const TStruct& Struct) {
    TStruct DefaultStruct;
    return TStruct::StaticStruct()->CompareScriptStruct(&Struct, &DefaultStruct, PPF_None) == false;
}

/**
 * Converts all fields of a struct into a human-readable key-value map.
 * Ideal for debug UI or developer logging.
 */
template <typename TStruct> TMap<FName, FString> StructToStringMap(const TStruct& Struct) {
    TMap<FName, FString> OutMap;

    for (TFieldIterator<FProperty> PropIt(TStruct::StaticStruct()); PropIt; ++PropIt) {
        const FProperty* Prop = *PropIt;
        const void* Value = Prop->ContainerPtrToValuePtr<const void>(&Struct);
        FString Exported;
        Prop->ExportTextItem_Direct(Exported, Value, nullptr, nullptr, PPF_None);
        OutMap.Add(Prop->GetFName(), Exported);
    }
    return OutMap;
}

/**
 * Selectively applies specific fields from a struct to an object, based on a provided update mask.
 * Useful for partial patching during gameplay, hot reload, or rollback.
 */
template <typename TStruct, typename TObject>
void MergeStructIntoObjectWithMask(const TStruct& Struct, TObject* Object, const TSet<FName>& FieldsToUpdate) {
    if (!IsValid(Object)) {
        return;
    }

    for (TFieldIterator<FProperty> PropIt(TStruct::StaticStruct()); PropIt; ++PropIt) {
        const FProperty* StructProp = *PropIt;
        if (!FieldsToUpdate.Contains(StructProp->GetFName())) {
            continue;
        }

        FProperty* ObjectProp = FindFProperty<FProperty>(Object->GetClass(), StructProp->GetFName());

        if (ObjectProp && StructProp->SameType(ObjectProp)) {
            const void* Src = StructProp->ContainerPtrToValuePtr<const void>(&Struct);
            void* Dst = ObjectProp->ContainerPtrToValuePtr<void>(Object);
            ObjectProp->CopyCompleteValue(Dst, Src);
        }
    }
}

/**
 * Serializes a struct into a clean, readable, sorted multiline string for logs or developer output.
 */
template <typename TStruct> FString SerializeStructForDebug(const TStruct& Struct) {
    TArray<FString> Lines;

    for (TFieldIterator<FProperty> PropIt(TStruct::StaticStruct()); PropIt; ++PropIt) {
        const FProperty* Prop = *PropIt;
        const void* Value = Prop->ContainerPtrToValuePtr<const void>(&Struct);
        FString Str;
        Prop->ExportTextItem_Direct(Str, Value, nullptr, nullptr, PPF_None);
        Lines.Add(FString::Printf(TEXT("%s = %s"), *Prop->GetName(), *Str));
    }

    Lines.Sort();
    return FString::Join(Lines, TEXT("\n"));
}

// ============================================================================
// 🔁 Graph Traversal, Loop Detection & Connectivity Analysis (86–90)
// ============================================================================

/**
 * Performs a safe breadth-first traversal of a node graph using a connection lambda.
 * Returns all reachable nodes from the starting point.
 */
template <typename TNode, typename GetConnectedNodesType>
FORCEINLINE TSet<TNode> TraverseConnectedGraph(TNode Start, GetConnectedNodesType&& GetConnectedNodes,
                                               int32 MaxDepth = 100) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);

    int32 Depth = 0;
    while (!Queue.IsEmpty() && Depth++ < MaxDepth) {
        TNode Current;
        Queue.Dequeue(Current);

        if (Visited.Contains(Current)) {
            continue;
        }
        Visited.Add(Current);

        for (const TNode& Neighbor : GetConnectedNodes(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return Visited;
}

template <typename TNode>
FORCEINLINE TSet<TNode> TraverseConnectedGraph(TNode Start, TFunctionRef<TArray<TNode>(const TNode&)> GetConnectedNodes,
                                               int32 MaxDepth = 100) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);

    int32 Depth = 0;
    while (!Queue.IsEmpty() && Depth++ < MaxDepth) {
        TNode Current;
        Queue.Dequeue(Current);

        if (Visited.Contains(Current)) {
            continue;
        }
        Visited.Add(Current);

        for (const TNode& Neighbor : GetConnectedNodes(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return Visited;
}

/**
 * Recursively detects cycles in a relational graph. Returns true if a cycle is found.
 * Outputs the path that triggered the cycle if applicable.
 */
template <typename TNode, typename GetConnectedNodesType>
bool DetectCyclesInGraph(TNode Start, GetConnectedNodesType&& GetConnectedNodes, TArray<TNode>& OutCycle,
                         int32 MaxDepth = -1) {
    TSet<TNode> Visited;
    TSet<TNode> Path;
    TMap<TNode, TNode> ParentMap;
    TArray<TPair<TNode, int32>> Stack;

    Stack.Add({Start, 0});
    Path.Add(Start);

    while (Stack.Num() > 0) {
        TPair<TNode, int32> Pair = Stack.Last();
        Stack.Pop();
        TNode Current = Pair.Key;
        int32 Depth = Pair.Value;

        if (MaxDepth >= 0 && Depth > MaxDepth) {
            Path.Remove(Current);
            continue;
        }

        bool bFoundUnvisited = false;
        for (const TNode& Neighbor : GetConnectedNodes(Current)) {
            if (!Visited.Contains(Neighbor)) {
                if (Path.Contains(Neighbor)) {
                    // Cycle detected
                    OutCycle.Reset();
                    TNode Trace = Current;
                    while (Trace != Neighbor && ParentMap.Contains(Trace)) {
                        OutCycle.Add(Trace);
                        Trace = ParentMap[Trace];
                    }
                    OutCycle.Add(Neighbor);
                    Algo::Reverse(OutCycle);
                    return true;
                }
                ParentMap.Add(Neighbor, Current);
                Stack.Add({Neighbor, Depth + 1});
                Path.Add(Neighbor);
                bFoundUnvisited = true;
                break;
            }
        }

        if (!bFoundUnvisited) {
            Visited.Add(Current);
            Path.Remove(Current);
        }
    }

    return false;
}

template <typename TNode>
bool DetectCyclesInGraph(TNode Start, TFunctionRef<TArray<TNode>(const TNode&)> GetConnectedNodes,
                         TArray<TNode>& OutCycle, int32 MaxDepth = -1) {
    TSet<TNode> Visited;
    TSet<TNode> Path;
    TMap<TNode, TNode> ParentMap;
    TArray<TPair<TNode, int32>> Stack;

    Stack.Add({Start, 0});
    Path.Add(Start);

    while (Stack.Num() > 0) {
        TPair<TNode, int32> Pair = Stack.Last();
        Stack.Pop();
        TNode Current = Pair.Key;
        int32 Depth = Pair.Value;

        if (MaxDepth >= 0 && Depth > MaxDepth) {
            Path.Remove(Current);
            continue;
        }

        bool bFoundUnvisited = false;
        for (const TNode& Neighbor : GetConnectedNodes(Current)) {
            if (!Visited.Contains(Neighbor)) {
                if (Path.Contains(Neighbor)) {
                    // Cycle detected
                    OutCycle.Reset();
                    TNode Trace = Current;
                    while (Trace != Neighbor && ParentMap.Contains(Trace)) {
                        OutCycle.Add(Trace);
                        Trace = ParentMap[Trace];
                    }
                    OutCycle.Add(Neighbor);
                    Algo::Reverse(OutCycle);
                    return true;
                }
                ParentMap.Add(Neighbor, Current);
                Stack.Add({Neighbor, Depth + 1});
                Path.Add(Neighbor);
                bFoundUnvisited = true;
                break;
            }
        }

        if (!bFoundUnvisited) {
            Visited.Add(Current);
            Path.Remove(Current);
        }
    }

    return false;
}

/**
 * Sorts a directed acyclic graph (DAG) into a valid execution or evaluation order.
 * Ignores cycles and treats them as already visited.
 */
template <typename TNode, typename GetDependenciesType>
TArray<TNode> FlattenGraphToLinearPath(const TArray<TNode>& Nodes, GetDependenciesType&& GetDependencies) {
    TArray<TNode> Result;
    TSet<TNode> Visited;

    TFunction<void(const TNode&)> Visit;
    Visit = [&](const TNode& Node) {
        if (Visited.Contains(Node)) {
            return;
        }
        Visited.Add(Node);

        for (const TNode& Dep : GetDependencies(Node)) {
            Visit(Dep);
        }
        Result.Add(Node);
    };

    for (const TNode& Node : Nodes) {
        Visit(Node);
    }

    return Result;
}

template <typename TNode>
TArray<TNode> FlattenGraphToLinearPath(const TArray<TNode>& Nodes,
                                       TFunctionRef<TArray<TNode>(const TNode&)> GetDependencies) {
    TArray<TNode> Result;
    TSet<TNode> Visited;

    TFunction<void(const TNode&)> Visit;
    Visit = [&](const TNode& Node) {
        if (Visited.Contains(Node)) {
            return;
        }
        Visited.Add(Node);

        for (const TNode& Dep : GetDependencies(Node)) {
            Visit(Dep);
        }
        Result.Add(Node);
    };

    for (const TNode& Node : Nodes) {
        Visit(Node);
    }

    return Result;
}

/**
 * Validates that a graph contains no unreachable nodes from the provided root.
 * Returns true only if all expected nodes are reachable.
 */
template <typename TNode, typename GetConnectedType>
bool ValidateGraphConnectivity(const TNode& Root, const TSet<TNode>& AllNodes, GetConnectedType&& GetConnected) {
    TSet<TNode> Reachable = TraverseConnectedGraph(Root, GetConnected);
    return Reachable.Includes(AllNodes);
}

template <typename TNode>
bool ValidateGraphConnectivity(const TNode& Root, const TSet<TNode>& AllNodes,
                               TFunctionRef<TArray<TNode>(const TNode&)> GetConnected) {
    TSet<TNode> Reachable = TraverseConnectedGraph(Root, GetConnected);
    return Reachable.Includes(AllNodes);
}

/**
 * Traces a lineage path from a given node up to the root using a single-parent lookup.
 * Returns the ordered path from child to ancestor.
 */
template <typename TNode, typename GetParentType>
TArray<TNode> TracePathToRoot(const TNode& Start, GetParentType&& GetParent) {
    TArray<TNode> Path;
    TSet<TNode> Seen;
    TNode Current = Start;

    while (Current.IsValid() && !Seen.Contains(Current)) {
        Path.Add(Current);
        Seen.Add(Current);

        TOptional<TNode> Next = GetParent(Current);
        if (!Next.IsSet()) {
            break;
        }

        Current = Next.GetValue();
    }

    return Path;
}

template <typename TNode>
TArray<TNode> TracePathToRoot(const TNode& Start, TFunctionRef<TOptional<TNode>(const TNode&)> GetParent) {
    TArray<TNode> Path;
    TSet<TNode> Seen;
    TNode Current = Start;

    while (Current.IsValid() && !Seen.Contains(Current)) {
        Path.Add(Current);
        Seen.Add(Current);

        TOptional<TNode> Next = GetParent(Current);
        if (!Next.IsSet()) {
            break;
        }

        Current = Next.GetValue();
    }

    return Path;
}

// ============================================================================
// 🧠 Advanced Graph Traversal & Enum-Based Network Utilities (91–100)
// ============================================================================

/**
 * Given a full set of nodes and a dependency function, computes the minimal
 * set of nodes from which all others are reachable.
 * Great for identifying seed/entry nodes for initialization or logic flow.
 */
template <typename TNode, typename GetDependenciesType>
TSet<TNode> CollapseGraphToMinimalRoots(const TSet<TNode>& AllNodes, GetDependenciesType&& GetDependencies) {
    TMap<TNode, TSet<TNode>> ReachMap;
    TSet<TNode> Roots;

    for (const TNode& Node : AllNodes) {
        ReachMap.Add(Node, TraverseConnectedGraph(Node, GetDependencies));
    }

    for (const auto& Pair : ReachMap) {
        bool IsRedundant = false;
        for (const auto& Other : ReachMap) {
            if (Other.Key != Pair.Key && Other.Value.Includes(Pair.Value)) {
                IsRedundant = true;
                break;
            }
        }
        if (!IsRedundant) {
            Roots.Add(Pair.Key);
        }
    }

    return Roots;
}

template <typename TNode>
TSet<TNode> CollapseGraphToMinimalRoots(const TSet<TNode>& AllNodes,
                                        TFunctionRef<TArray<TNode>(const TNode&)> GetDependencies) {
    TMap<TNode, TSet<TNode>> ReachMap;
    TSet<TNode> Roots;

    for (const TNode& Node : AllNodes) {
        ReachMap.Add(Node, TraverseConnectedGraph(Node, GetDependencies));
    }

    for (const auto& Pair : ReachMap) {
        bool IsRedundant = false;
        for (const auto& Other : ReachMap) {
            if (Other.Key != Pair.Key && Other.Value.Includes(Pair.Value)) {
                IsRedundant = true;
                break;
            }
        }
        if (!IsRedundant) {
            Roots.Add(Pair.Key);
        }
    }

    return Roots;
}

/**
 * Recursively traces upstream influences (parents) of a node, collecting all
 * unique transitive contributors. Excellent for causal graph tracing.
 */
template <typename TNode, typename GetParentsType>
TSet<TNode> TraceGraphInfluenceSources(const TNode& Target, GetParentsType&& GetParents) {
    TSet<TNode> Sources;
    TQueue<TNode> Queue;
    Queue.Enqueue(Target);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        for (const TNode& Parent : GetParents(Current)) {
            if (Sources.Add(Parent)) {
                Queue.Enqueue(Parent);
            }
        }
    }

    return Sources;
}

template <typename TNode>
TSet<TNode> TraceGraphInfluenceSources(const TNode& Target, TFunctionRef<TArray<TNode>(const TNode&)> GetParents) {
    TSet<TNode> Sources;
    TQueue<TNode> Queue;
    Queue.Enqueue(Target);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        for (const TNode& Parent : GetParents(Current)) {
            if (Sources.Add(Parent)) {
                Queue.Enqueue(Parent);
            }
        }
    }

    return Sources;
}

/**
 * Decomposes a graph into its disjoint connected components.
 * Each component is a distinct set of connected nodes.
 */
template <typename TNode, typename GetConnectedType>
TArray<TSet<TNode>> BuildGraphComponentSets(const TSet<TNode>& AllNodes, GetConnectedType&& GetConnected) {
    TSet<TNode> Unvisited = AllNodes;
    TArray<TSet<TNode>> Components;

    while (Unvisited.Num() > 0) {
        TNode Start = *Unvisited.CreateIterator();
        TSet<TNode> Component = TraverseConnectedGraph(Start, GetConnected);
        Components.Add(Component);
        Unvisited = Unvisited.Difference(Component);
    }

    return Components;
}

template <typename TNode>
TArray<TSet<TNode>> BuildGraphComponentSets(const TSet<TNode>& AllNodes,
                                            TFunctionRef<TArray<TNode>(const TNode&)> GetConnected) {
    TSet<TNode> Unvisited = AllNodes;
    TArray<TSet<TNode>> Components;

    while (Unvisited.Num() > 0) {
        TNode Start = *Unvisited.CreateIterator();
        TSet<TNode> Component = TraverseConnectedGraph(Start, GetConnected);
        Components.Add(Component);
        Unvisited = Unvisited.Difference(Component);
    }

    return Components;
}

/**
 * Performs a breadth-first search to return the shortest path
 * from Start to Goal node, by edge count.
 */
template <typename TNode, typename GetNeighborsType>
TArray<TNode> FindShortestPathInGraph(const TNode& Start, const TNode& Goal, GetNeighborsType&& GetNeighbors) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);
    CameFrom.Add(Start, Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        if (Current == Goal) {
            TArray<TNode> Path;
            TNode Step = Goal;
            while (Step != Start) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Start, 0);
            return Path;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!CameFrom.Contains(Neighbor)) {
                CameFrom.Add(Neighbor, Current);
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return {};
}

template <typename TNode>
TArray<TNode> FindShortestPathInGraph(const TNode& Start, const TNode& Goal,
                                      TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);
    CameFrom.Add(Start, Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        if (Current == Goal) {
            TArray<TNode> Path;
            TNode Step = Goal;
            while (Step != Start) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Start, 0);
            return Path;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!CameFrom.Contains(Neighbor)) {
                CameFrom.Add(Neighbor, Current);
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return {};
}

/**
 * Given a map of direct edges, computes the full transitive closure
 * of all indirect connections for each node.
 */
template <typename TNode>
TMap<TNode, TSet<TNode>> ResolveTransitiveClosureMap(const TMap<TNode, TArray<TNode>>& DirectConnections) {
    TMap<TNode, TSet<TNode>> Closure;

    for (const auto& Pair : DirectConnections) {
        TNode Key = Pair.Key;
        TSet<TNode> Reached = TraverseConnectedGraph(Key, [&](const TNode& Node) {
            const TArray<TNode>* Neighbors = DirectConnections.Find(Node);
            return Neighbors ? *Neighbors : TArray<TNode>{};
        });
        Reached.Remove(Key);
        Closure.Add(Key, Reached);
    }

    return Closure;
}

/**
 * Resolves the final destination of an enum remap chain via map lookup.
 * Cycle-safe; stops if it revisits any node.
 */
template <typename TEnum> TEnum ResolveEnumChain(const TMap<TEnum, TEnum>& EnumMap, TEnum Start) {
    TSet<TEnum> Visited;
    TEnum Current = Start;

    while (EnumMap.Contains(Current)) {
        if (!Visited.Add(Current)) {
            break;
        }
        Current = EnumMap[Current];
    }

    return Current;
}

/**
 * Converts an array of transition-style structs into a directed enum graph.
 * Useful for animation states, role transitions, phase maps, etc.
 */
template <typename TEnum, typename TStruct, typename GetFromType, typename GetToType>
FORCEINLINE TMap<TEnum, TArray<TEnum>> BuildEnumToStructGraph(const TArray<TStruct>& Transitions, GetFromType&& GetFrom,
                                                              GetToType&& GetTo) {
    TMap<TEnum, TArray<TEnum>> Graph;

    for (const TStruct& Entry : Transitions) {
        Graph.FindOrAdd(GetFrom(Entry)).Add(GetTo(Entry));
    }

    return Graph;
}

template <typename TEnum, typename TStruct>
FORCEINLINE TMap<TEnum, TArray<TEnum>> BuildEnumToStructGraph(const TArray<TStruct>& Transitions,
                                                              TFunctionRef<TEnum(const TStruct&)> GetFrom,
                                                              TFunctionRef<TEnum(const TStruct&)> GetTo) {
    TMap<TEnum, TArray<TEnum>> Graph;

    for (const TStruct& Entry : Transitions) {
        Graph.FindOrAdd(GetFrom(Entry)).Add(GetTo(Entry));
    }

    return Graph;
}

/**
 * Scans a graph of enum links for any enum values not present in a known set.
 * Helps catch typos, corruption, invalid graph inputs.
 */
template <typename TEnum>
TArray<TEnum> DetectInvalidEnumLinksInGraph(const TMap<TEnum, TArray<TEnum>>& Graph, const TSet<TEnum>& ValidEnums) {
    TArray<TEnum> Invalid;

    for (const auto& Pair : Graph) {
        if (!ValidEnums.Contains(Pair.Key)) {
            Invalid.Add(Pair.Key);
        }

        for (const TEnum& Target : Pair.Value) {
            if (!ValidEnums.Contains(Target)) {
                Invalid.Add(Target);
            }
        }
    }

    return Invalid;
}

/**
 * Checks whether a path exists between two enum values in a directed enum graph.
 */
template <typename TEnum>
bool ValidateEnumPathExists(const TEnum& Start, const TEnum& Goal, const TMap<TEnum, TArray<TEnum>>& Graph) {
    TSet<TEnum> Visited;
    TQueue<TEnum> Queue;
    Queue.Enqueue(Start);

    while (!Queue.IsEmpty()) {
        TEnum Current;
        Queue.Dequeue(Current);

        if (Current == Goal) {
            return true;
        }
        if (!Visited.Add(Current)) {
            continue;
        }

        const TArray<TEnum>* Neighbors = Graph.Find(Current);
        if (Neighbors) {
            for (const TEnum& N : *Neighbors) {
                if (!Visited.Contains(N)) {
                    Queue.Enqueue(N);
                }
            }
        }
    }

    return false;
}

/**
 * Traces and returns the shortest enum path between two nodes in a graph.
 * Useful for visualizing transitions, debugging misroutes, etc.
 */
template <typename TEnum>
TArray<TEnum> TraceEnumPath(const TEnum& Start, const TEnum& Goal, const TMap<TEnum, TArray<TEnum>>& Graph) {
    TMap<TEnum, TEnum> CameFrom;
    TQueue<TEnum> Queue;
    Queue.Enqueue(Start);
    CameFrom.Add(Start, Start);

    while (!Queue.IsEmpty()) {
        TEnum Current;
        Queue.Dequeue(Current);

        if (Current == Goal) {
            TArray<TEnum> Path;
            TEnum Step = Goal;
            while (Step != Start) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Start, 0);
            return Path;
        }

        const TArray<TEnum>* Neighbors = Graph.Find(Current);
        if (Neighbors) {
            for (const TEnum& N : *Neighbors) {
                if (!CameFrom.Contains(N)) {
                    CameFrom.Add(N, Current);
                    Queue.Enqueue(N);
                }
            }
        }
    }

    return {};
}

// ============================================================================
//  Node Graph Algorithms: Traversal, Recursion, Validation (101–110)
// ============================================================================

/**
 * Performs a depth-first traversal of a graph, applying a Visit lambda to each unique node.
 */
template <typename TNode, typename GetNeighborsType, typename VisitType>
void DepthFirstVisit(const TNode& Start, GetNeighborsType&& GetNeighbors, VisitType&& Visit) {
    TSet<TNode> Visited;
    TFunction<void(const TNode&)> Recurse;
    Recurse = [&](const TNode& Node) {
        if (Visited.Contains(Node)) {
            return;
        }

        Visited.Add(Node);
        Visit(Node);

        for (const TNode& Neighbor : GetNeighbors(Node)) {
            Recurse(Neighbor);
        }
    };
    Recurse(Start);
}

template <typename TNode>
void DepthFirstVisit(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                     TFunctionRef<void(const TNode&)> Visit) {
    TSet<TNode> Visited;
    TFunction<void(const TNode&)> Recurse;
    Recurse = [&](const TNode& Node) {
        if (Visited.Contains(Node)) {
            return;
        }

        Visited.Add(Node);
        Visit(Node);

        for (const TNode& Neighbor : GetNeighbors(Node)) {
            Recurse(Neighbor);
        }
    };
    Recurse(Start);
}

/**
 * Performs a breadth-first traversal of a graph, visiting each node level-wise.
 */
template <typename TNode, typename GetNeighborsType, typename VisitType>
void BreadthFirstVisit(const TNode& Start, GetNeighborsType&& GetNeighbors, VisitType&& Visit) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Visit(Current);
        for (const TNode& N : GetNeighbors(Current)) {
            if (!Visited.Contains(N)) {
                Queue.Enqueue(N);
            }
        }
    }
}

template <typename TNode>
void BreadthFirstVisit(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                       TFunctionRef<void(const TNode&)> Visit) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Visit(Current);
        for (const TNode& N : GetNeighbors(Current)) {
            if (!Visited.Contains(N)) {
                Queue.Enqueue(N);
            }
        }
    }
}

/**
 * Attempts to sort a DAG topologically. Returns false if a cycle is detected.
 */
template <typename TNode, typename GetDependenciesType>
bool TopologicalSort(const TSet<TNode>& AllNodes, GetDependenciesType&& GetDependencies, TArray<TNode>& OutSorted) {
    TMap<TNode, int32> InDegree;
    TMap<TNode, TArray<TNode>> ReverseEdges;

    for (const TNode& Node : AllNodes) {
        TArray<TNode> Deps = GetDependencies(Node);
        InDegree.FindOrAdd(Node);
        for (const TNode& Dep : Deps) {
            ++InDegree.FindOrAdd(Dep);
            ReverseEdges.FindOrAdd(Dep).Add(Node);
        }
    }

    TQueue<TNode> Queue;
    for (const auto& Pair : InDegree) {
        if (Pair.Value == 0) {
            Queue.Enqueue(Pair.Key);
        }
    }

    while (!Queue.IsEmpty()) {
        TNode Node;
        Queue.Dequeue(Node);
        OutSorted.Add(Node);

        for (const TNode& Dep : ReverseEdges.FindRef(Node)) {
            if (--InDegree.FindChecked(Dep) == 0) {
                Queue.Enqueue(Dep);
            }
        }
    }

    return OutSorted.Num() == AllNodes.Num();
}

template <typename TNode>
bool TopologicalSort(const TSet<TNode>& AllNodes, TFunctionRef<TArray<TNode>(const TNode&)> GetDependencies,
                     TArray<TNode>& OutSorted) {
    TMap<TNode, int32> InDegree;
    TMap<TNode, TArray<TNode>> ReverseEdges;

    for (const TNode& Node : AllNodes) {
        TArray<TNode> Deps = GetDependencies(Node);
        InDegree.FindOrAdd(Node);
        for (const TNode& Dep : Deps) {
            ++InDegree.FindOrAdd(Dep);
            ReverseEdges.FindOrAdd(Dep).Add(Node);
        }
    }

    TQueue<TNode> Queue;
    for (const auto& Pair : InDegree) {
        if (Pair.Value == 0) {
            Queue.Enqueue(Pair.Key);
        }
    }

    while (!Queue.IsEmpty()) {
        TNode Node;
        Queue.Dequeue(Node);
        OutSorted.Add(Node);

        for (const TNode& Dep : ReverseEdges.FindRef(Node)) {
            if (--InDegree.FindChecked(Dep) == 0) {
                Queue.Enqueue(Dep);
            }
        }
    }

    return OutSorted.Num() == AllNodes.Num();
}

/**
 * Runs Dijkstra's algorithm to compute shortest distances from a start node.
 */
template <typename TNode, typename GetNeighborsType, typename GetEdgeCostType>
TMap<TNode, float> DijkstraShortestPaths(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                         GetEdgeCostType&& GetEdgeCost) {
    TMap<TNode, float> Dist;
    TSet<TNode> Visited;
    TQueue<TPair<float, TNode>> Queue;

    Dist.Add(Start, 0.f);
    Queue.Enqueue({0.f, Start});

    while (!Queue.IsEmpty()) {
        TPair<float, TNode> Pair;
        Queue.Dequeue(Pair);
        TNode Current = Pair.Value;

        if (Visited.Contains(Current)) {
            continue;
        }
        Visited.Add(Current);

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            float NewDist = Dist[Current] + GetEdgeCost(Current, Neighbor);
            if (!Dist.Contains(Neighbor) || NewDist < Dist[Neighbor]) {
                Dist.Add(Neighbor, NewDist);
                Queue.Enqueue({NewDist, Neighbor});
            }
        }
    }

    return Dist;
}

template <typename TNode>
TMap<TNode, float> DijkstraShortestPaths(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                         TFunctionRef<float(const TNode&, const TNode&)> GetEdgeCost) {
    TMap<TNode, float> Dist;
    TSet<TNode> Visited;
    TQueue<TPair<float, TNode>> Queue;

    Dist.Add(Start, 0.f);
    Queue.Enqueue({0.f, Start});

    while (!Queue.IsEmpty()) {
        TPair<float, TNode> Pair;
        Queue.Dequeue(Pair);
        TNode Current = Pair.Value;

        if (Visited.Contains(Current)) {
            continue;
        }
        Visited.Add(Current);

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            float NewDist = Dist[Current] + GetEdgeCost(Current, Neighbor);
            if (!Dist.Contains(Neighbor) || NewDist < Dist[Neighbor]) {
                Dist.Add(Neighbor, NewDist);
                Queue.Enqueue({NewDist, Neighbor});
            }
        }
    }

    return Dist;
}

/**
 * Recursively finds all valid paths between two nodes, up to a given depth.
 */
template <typename TNode, typename GetNeighborsType>
void EnumerateAllPaths(const TNode& Start, const TNode& Goal, GetNeighborsType&& GetNeighbors,
                       TArray<TArray<TNode>>& OutPaths, int32 MaxDepth = 10) {
    TArray<TNode> CurrentPath;
    TSet<TNode> Seen;

    TFunction<void(const TNode&, int32)> Recurse;
    Recurse = [&](const TNode& Node, int32 Depth) {
        if (Depth > MaxDepth || Seen.Contains(Node)) {
            return;
        }

        CurrentPath.Add(Node);
        Seen.Add(Node);

        if (Node == Goal) {
            OutPaths.Add(CurrentPath);
        } else {
            for (const TNode& Neighbor : GetNeighbors(Node)) {
                Recurse(Neighbor, Depth + 1);
            }
        }

        CurrentPath.Pop();
        Seen.Remove(Node);
    };

    Recurse(Start, 0);
}

template <typename TNode>
void EnumerateAllPaths(const TNode& Start, const TNode& Goal, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                       TArray<TArray<TNode>>& OutPaths, int32 MaxDepth = 10) {
    TArray<TNode> CurrentPath;
    TSet<TNode> Seen;

    TFunction<void(const TNode&, int32)> Recurse;
    Recurse = [&](const TNode& Node, int32 Depth) {
        if (Depth > MaxDepth || Seen.Contains(Node)) {
            return;
        }

        CurrentPath.Add(Node);
        Seen.Add(Node);

        if (Node == Goal) {
            OutPaths.Add(CurrentPath);
        } else {
            for (const TNode& Neighbor : GetNeighbors(Node)) {
                Recurse(Neighbor, Depth + 1);
            }
        }

        CurrentPath.Pop();
        Seen.Remove(Node);
    };

    Recurse(Start, 0);
}

/**
 * Traverses until a node satisfying a goal condition is reached. Returns the first path to that goal.
 */
template <typename TNode, typename GetNeighborsType, typename IsGoalType>
TOptional<TArray<TNode>> TraverseUntilGoal(const TNode& Start, GetNeighborsType&& GetNeighbors, IsGoalType&& IsGoal) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    TSet<TNode> Visited;

    Queue.Enqueue(Start);
    CameFrom.Add(Start, Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        if (!Visited.Add(Current)) {
            continue;
        }
        if (IsGoal(Current)) {
            TArray<TNode> Path;
            TNode Step = Current;
            while (Step != Start) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Start, 0);
            return Path;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!CameFrom.Contains(Neighbor)) {
                CameFrom.Add(Neighbor, Current);
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return {};
}

template <typename TNode>
TOptional<TArray<TNode>> TraverseUntilGoal(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                           TFunctionRef<bool(const TNode&)> IsGoal) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    TSet<TNode> Visited;

    Queue.Enqueue(Start);
    CameFrom.Add(Start, Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        if (!Visited.Add(Current)) {
            continue;
        }
        if (IsGoal(Current)) {
            TArray<TNode> Path;
            TNode Step = Current;
            while (Step != Start) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Start, 0);
            return Path;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!CameFrom.Contains(Neighbor)) {
                CameFrom.Add(Neighbor, Current);
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return {};
}

/**
 * Performs a breadth-first traversal from the Start node,
 * using GetNeighbors to get adjacent nodes,
 * calls Visitor on each visited node,
 * stops when MaxNodesToVisit is reached (-1 means no limit).
 *
 * @param Start Starting node to begin traversal.
 * @param GetNeighbors Function returning adjacent nodes for a given node.
 * @param Visitor Function called on each visited node.
 * @param MaxNodesToVisit Max number of nodes to visit, -1 for unlimited.
 * @return Set of visited nodes.
 */
// 1. Generic callable for GetNeighbors and Visitor (most flexible)
template <typename TNode, typename GetNeighborsType, typename VisitorType>
FORCEINLINE TSet<TNode> TraverseWithNodeLimit(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                              VisitorType&& Visitor, int32 MaxNodesToVisit = -1) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;

    Queue.Enqueue(Start);
    int32 VisitedCount = 0;

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        if (Visited.Contains(Current)) {
            continue;
        }

        Visited.Add(Current);
        Visitor(Current);

        VisitedCount++;
        if (MaxNodesToVisit > 0 && VisitedCount >= MaxNodesToVisit) {
            break;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return Visited;
}

// 2. TFunctionRef for GetNeighbors and Visitor (explicit TFunctionRef overload)
template <typename TNode>
FORCEINLINE TSet<TNode> TraverseWithNodeLimit(const TNode& Start,
                                              TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                              TFunctionRef<void(const TNode&)> Visitor, int32 MaxNodesToVisit = -1) {
    return TraverseWithNodeLimit<TNode, TFunctionRef<TArray<TNode>(const TNode&)>, TFunctionRef<void(const TNode&)>>(
        Start, GetNeighbors, Visitor, MaxNodesToVisit);
}

// 3. Generic callable for GetNeighbors only, no visitor (convenience overload)
template <typename TNode, typename GetNeighborsType>
FORCEINLINE TSet<TNode> TraverseWithNodeLimit(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                              int32 MaxNodesToVisit = -1) {
    return TraverseWithNodeLimit<TNode>(
        Start, Forward<GetNeighborsType>(GetNeighbors), [](const TNode&) {}, // No-op visitor
        MaxNodesToVisit);
}

// 4. TFunctionRef for GetNeighbors only, no visitor (symmetry convenience overload)
template <typename TNode>
FORCEINLINE TSet<TNode> TraverseWithNodeLimit(const TNode& Start,
                                              TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                              int32 MaxNodesToVisit = -1) {
    return TraverseWithNodeLimit<TNode>(
        Start, GetNeighbors, [](const TNode&) {}, // No-op visitor
        MaxNodesToVisit);
}

template <typename TNode>
void TraverseWithNodeLimit_VisitorOnly(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                       TFunctionRef<void(const TNode&)> Visitor, int32 MaxNodesToVisit = -1) {
    (void)TraverseWithNodeLimit<TNode>(Start, GetNeighbors, Visitor, MaxNodesToVisit);
}

/**
 * Performs a breadth-first traversal from the Start node,
 * using GetNeighbors to get adjacent nodes,
 * calls Visitor on each visited node,
 * stops when MaxNodesToVisit is reached (-1 means no limit).
 *
 * @param Start Starting node to begin traversal.
 * @param GetNeighbors Function returning adjacent nodes for a given node.
 * @param Visitor Function called on each visited node.
 * @param ShouldStop Function returning true if traversal should stop.
 * @param MaxNodesToVisit Max number of nodes to visit, -1 for unlimited.
 * @return Set of visited nodes.
 */
// 1. Generic callable version for all callables
template <typename TNode, typename GetNeighborsType, typename VisitorType, typename ShouldStopType>
FORCEINLINE TSet<TNode> TraverseWithNodeLimitUntil(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                                   VisitorType&& Visitor, ShouldStopType&& ShouldStop,
                                                   int32 MaxNodesToVisit = -1) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Start);

    int32 VisitedCount = 0;

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (Visited.Contains(Current)) {
            continue;
        }

        Visited.Add(Current);
        Visitor(Current);
        if (ShouldStop(Current)) {
            break;
        }

        if (MaxNodesToVisit > 0 && ++VisitedCount >= MaxNodesToVisit) {
            break;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return Visited;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
FORCEINLINE TSet<TNode>
TraverseWithNodeLimitUntil(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                           TFunctionRef<void(const TNode&)> Visitor, TFunctionRef<bool(const TNode&)> ShouldStop,
                           int32 MaxNodesToVisit = -1) {
    return TraverseWithNodeLimitUntil<TNode, TFunctionRef<TArray<TNode>(const TNode&)>,
                                      TFunctionRef<void(const TNode&)>, TFunctionRef<bool(const TNode&)>>(
        Start, GetNeighbors, Visitor, ShouldStop, MaxNodesToVisit);
}

// 1. Generic callable version for GetNeighbors, Visitor, and GetKey
template <typename TNode, typename TKey, typename GetNeighborsType, typename VisitorType, typename GetKeyType>
FORCEINLINE TSet<TKey> TraverseWithProjection(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                              VisitorType&& Visitor, GetKeyType&& GetKey, int32 MaxNodesToVisit = -1) {
    TSet<TKey> VisitedKeys;
    TQueue<TNode> Queue;

    Queue.Enqueue(Start);
    int32 Count = 0;

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        const TKey Key = GetKey(Current);
        if (!VisitedKeys.Add(Key)) {
            continue;
        }

        Visitor(Current);
        if (MaxNodesToVisit > 0 && ++Count >= MaxNodesToVisit) {
            break;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!VisitedKeys.Contains(GetKey(Neighbor))) {
                Queue.Enqueue(Neighbor);
            }
        }
    }
    return VisitedKeys;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode, typename TKey>
FORCEINLINE TSet<TKey> TraverseWithProjection(const TNode& Start,
                                              TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                              TFunctionRef<void(const TNode&)> Visitor,
                                              TFunctionRef<TKey(const TNode&)> GetKey, int32 MaxNodesToVisit = -1) {
    return TraverseWithProjection<TNode, TKey, TFunctionRef<TArray<TNode>(const TNode&)>,
                                  TFunctionRef<void(const TNode&)>, TFunctionRef<TKey(const TNode&)>>(
        Start, GetNeighbors, Visitor, GetKey, MaxNodesToVisit);
}

// 1. Generic callable version for GetNeighbors and Transform
template <typename TNode, typename TResult, typename GetNeighborsType, typename TransformType>
FORCEINLINE TArray<TResult> TraverseAndMap(const TNode& Start, GetNeighborsType&& GetNeighbors,
                                           TransformType&& Transform, int32 MaxNodesToVisit = -1) {
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    TArray<TResult> Result;

    Queue.Enqueue(Start);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Result.Add(Transform(Current));
        if (MaxNodesToVisit > 0 && Visited.Num() >= MaxNodesToVisit) {
            break;
        }

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }
    return Result;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode, typename TResult>
FORCEINLINE TArray<TResult> TraverseAndMap(const TNode& Start, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                           TFunctionRef<TResult(const TNode&)> Transform, int32 MaxNodesToVisit = -1) {
    return TraverseAndMap<TNode, TResult, TFunctionRef<TArray<TNode>(const TNode&)>,
                          TFunctionRef<TResult(const TNode&)>>(Start, GetNeighbors, Transform, MaxNodesToVisit);
}

template <typename TNode> struct TAStarNodeData {
    // Cost from start node to this node
    float GCost = TNumericLimits<float>::Max();

    // Estimated total cost: GCost + heuristic
    float FCost = TNumericLimits<float>::Max();

    // Parent node in the path
    TNode Parent;

    // Flags
    bool bHasParent = false;
    bool bInOpenSet = false;
    bool bInClosedSet = false;

    // Optional cached edge cost from parent to this node
    float CachedEdgeCost = TNumericLimits<float>::Max();

    // Optional: iteration stamp for search reset optimization
    int32 VisitIteration = -1;

    TAStarNodeData() = default;

    void Reset() {
        GCost = TNumericLimits<float>::Max();
        FCost = TNumericLimits<float>::Max();
        bHasParent = false;
        bInOpenSet = false;
        bInClosedSet = false;
        CachedEdgeCost = TNumericLimits<float>::Max();
        VisitIteration = -1;
    }
};

/**
 * Extracts a subgraph from a root node to a limited depth, with optional loop awareness.
 */
// 1. Generic callable version for GetNeighbors
template <typename TNode, typename GetNeighborsType>
FORCEINLINE TSet<TNode> LoopAwareSubgraphExtract(const TNode& Root, GetNeighborsType&& GetNeighbors, int32 MaxDepth,
                                                 bool bIncludeLoopEntrances = true) {
    TSet<TNode> Subgraph;
    TSet<TNode> Path;
    TFunction<void(const TNode&, int32)> Visit;

    Visit = [&](const TNode& Node, int32 Depth) {
        if (Depth > MaxDepth || Path.Contains(Node)) {
            if (bIncludeLoopEntrances) {
                Subgraph.Add(Node);
            }
            return;
        }

        Path.Add(Node);
        Subgraph.Add(Node);

        for (const TNode& N : GetNeighbors(Node)) {
            Visit(N, Depth + 1);
        }

        Path.Remove(Node);
    };

    Visit(Root, 0);
    return Subgraph;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
FORCEINLINE TSet<TNode> LoopAwareSubgraphExtract(const TNode& Root,
                                                 TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors, int32 MaxDepth,
                                                 bool bIncludeLoopEntrances = true) {
    return LoopAwareSubgraphExtract<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(Root, GetNeighbors, MaxDepth,
                                                                                      bIncludeLoopEntrances);
}

// 1. Generic callable version for GetChildren and CloneNode
template <typename TNode, typename GetChildrenType, typename CloneNodeType>
void CloneSubgraph(const TNode& Root, GetChildrenType&& GetChildren, CloneNodeType&& CloneNode,
                   TSet<TNode>& OutVisited) {
    if (!OutVisited.Add(Root)) {
        return;
    }
    CloneNode(Root);

    for (const TNode& Child : GetChildren(Root)) {
        CloneSubgraph(Child, Forward<GetChildrenType>(GetChildren), Forward<CloneNodeType>(CloneNode), OutVisited);
    }
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
void CloneSubgraph(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetChildren,
                   TFunctionRef<void(const TNode&)> CloneNode, TSet<TNode>& OutVisited) {
    CloneSubgraph<TNode, TFunctionRef<TArray<TNode>(const TNode&)>, TFunctionRef<void(const TNode&)>>(
        Root, GetChildren, CloneNode, OutVisited);
}

/**
 * Validates that every path from a root to a leaf node satisfies a constraint function.
 */
// 1. Generic callable version for GetChildren and IsPathValid
template <typename TNode, typename GetChildrenType, typename IsPathValidType>
bool ValidateAllPathsSatisfyConstraint(const TNode& Root, GetChildrenType&& GetChildren, IsPathValidType&& IsPathValid,
                                       int32 MaxDepth = 12) {
    TArray<TNode> CurrentPath;
    TSet<TNode> Seen;

    TFunction<bool(const TNode&, int32)> Recurse;
    Recurse = [&](const TNode& Node, int32 Depth) -> bool {
        if (Depth > MaxDepth || Seen.Contains(Node)) {
            return true;
        }

        CurrentPath.Add(Node);
        Seen.Add(Node);

        const TArray<TNode>& Children = GetChildren(Node);
        if (Children.Num() == 0) {
            if (!IsPathValid(CurrentPath)) {
                return false;
            }
        } else {
            for (const TNode& Child : Children) {
                if (!Recurse(Child, Depth + 1)) {
                    return false;
                }
            }
        }

        CurrentPath.Pop();
        Seen.Remove(Node);
        return true;
    };

    return Recurse(Root, 0);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
bool ValidateAllPathsSatisfyConstraint(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetChildren,
                                       TFunctionRef<bool(const TArray<TNode>&)> IsPathValid, int32 MaxDepth = 12) {
    return ValidateAllPathsSatisfyConstraint<TNode, TFunctionRef<TArray<TNode>(const TNode&)>,
                                             TFunctionRef<bool(const TArray<TNode>&)>>(Root, GetChildren, IsPathValid,
                                                                                       MaxDepth);
}

// ============================================================================
// Editor-Compatible Circularity Detection (116–120)
// ============================================================================

/**
 * Detects any circular link in a struct-based graph using depth-first search.
 */
// 1. Generic callable version for GetLinks
template <typename TStruct, typename GetLinksType>
bool DetectCycleInStructGraph(const TStruct& Root, GetLinksType&& GetLinks) {
    TSet<TStruct> Visited;
    TSet<TStruct> Path;

    TFunction<bool(const TStruct&)> Recurse;
    Recurse = [&](const TStruct& Node) -> bool {
        if (Path.Contains(Node)) {
            return true; // cycle detected
        }
        if (Visited.Contains(Node)) {
            return false;
        }

        Visited.Add(Node);
        Path.Add(Node);

        for (const TStruct& Next : GetLinks(Node)) {
            if (Recurse(Next)) {
                return true;
            }
        }

        Path.Remove(Node);
        return false;
    };

    return Recurse(Root);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TStruct>
bool DetectCycleInStructGraph(const TStruct& Root, TFunctionRef<TArray<TStruct>(const TStruct&)> GetLinks) {
    return DetectCycleInStructGraph<TStruct, TFunctionRef<TArray<TStruct>(const TStruct&)>>(Root, GetLinks);
}

/**
 * Finds and returns all distinct cycles in a graph as node paths.
 */
// 1. Generic callable version for GetLinks
template <typename TStruct, typename GetLinksType>
TArray<TArray<TStruct>> ReportCyclesInGraph(const TStruct& Root, GetLinksType&& GetLinks) {
    TArray<TStruct> CurrentPath;
    TSet<TStruct> Visited;
    TArray<TArray<TStruct>> Cycles;

    TFunction<void(const TStruct&)> Recurse;
    Recurse = [&](const TStruct& Node) {
        if (CurrentPath.Contains(Node)) {
            int32 Index = CurrentPath.IndexOfByKey(Node);
            TArray<TStruct> Cycle = CurrentPath.Mid(Index);
            Cycle.Add(Node);
            Cycles.Add(Cycle);
            return;
        }

        if (!Visited.Add(Node)) {
            return;
        }

        CurrentPath.Add(Node);
        for (const TStruct& Next : GetLinks(Node)) {
            Recurse(Next);
        }
        CurrentPath.Pop();
    };

    Recurse(Root);
    return Cycles;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TStruct>
TArray<TArray<TStruct>> ReportCyclesInGraph(const TStruct& Root,
                                            TFunctionRef<TArray<TStruct>(const TStruct&)> GetLinks) {
    return ReportCyclesInGraph<TStruct, TFunctionRef<TArray<TStruct>(const TStruct&)>>(Root, GetLinks);
}

/**
 * Detects cycles in a UObject-based data asset graph, safe for editor use.
 */
// 1. Generic callable version for GetLinks
template <typename TAsset, typename GetLinksType>
bool DetectCycleInDataAssetGraph(const TAsset* Root, GetLinksType&& GetLinks) {
    TSet<const TAsset*> Visited;
    TSet<const TAsset*> Path;

    TFunction<bool(const TAsset*)> Recurse;
    Recurse = [&](const TAsset* Node) -> bool {
        if (!Node) {
            return false;
        }
        if (Path.Contains(Node)) {
            return true;
        }
        if (Visited.Contains(Node)) {
            return false;
        }

        Visited.Add(Node);
        Path.Add(Node);

        for (const TAsset* Next : GetLinks(Node)) {
            if (Recurse(Next)) {
                return true;
            }
        }

        Path.Remove(Node);
        return false;
    };

    return Recurse(Root);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TAsset>
bool DetectCycleInDataAssetGraph(const TAsset* Root, TFunctionRef<TArray<TAsset*>(const TAsset*)> GetLinks) {
    return DetectCycleInDataAssetGraph<TAsset, TFunctionRef<TArray<TAsset*>(const TAsset*)>>(Root, GetLinks);
}

/**
 * Validates that a graph of enum→enum links contains no loops.
 */
template <typename TEnum> bool ValidateEnumGraphHasNoLoops(const TMap<TEnum, TArray<TEnum>>& Graph) {
    TSet<TEnum> Visited;
    TSet<TEnum> Path;

    TFunction<bool(const TEnum&)> Recurse;
    Recurse = [&](const TEnum& Node) -> bool {
        if (Path.Contains(Node)) {
            return true;
        }
        if (Visited.Contains(Node)) {
            return false;
        }

        Visited.Add(Node);
        Path.Add(Node);

        const TArray<TEnum>* Children = Graph.Find(Node);
        if (Children) {
            for (const TEnum& Child : *Children) {
                if (Recurse(Child)) {
                    return true;
                }
            }
        }

        Path.Remove(Node);
        return false;
    };

    for (const auto& Pair : Graph) {
        if (Recurse(Pair.Key)) {
            return false;
        }
    }

    return true;
}

/**
 * Performs defensive cycle detection using a max-depth limit to avoid infinite loops.
 */
// 1. Generic callable version for GetLinks
template <typename TNode, typename GetLinksType>
bool DetectCycleWithMaxDepth(const TNode& Root, GetLinksType&& GetLinks, int32 MaxDepth = 32) {
    TSet<TNode> Path;
    TFunction<bool(const TNode&, int32)> Recurse;

    Recurse = [&](const TNode& Node, int32 Depth) -> bool {
        if (Depth > MaxDepth) {
            return true; // fallback: assume loop
        }
        if (Path.Contains(Node)) {
            return true;
        }

        Path.Add(Node);
        for (const TNode& N : GetLinks(Node)) {
            if (Recurse(N, Depth + 1)) {
                return true;
            }
        }
        Path.Remove(Node);
        return false;
    };

    return Recurse(Root, 0);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
bool DetectCycleWithMaxDepth(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks,
                             int32 MaxDepth = 32) {
    return DetectCycleWithMaxDepth<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(Root, GetLinks, MaxDepth);
}

// ============================================================================
//  Partial Graph Rollback Support (121–125)
// ============================================================================

/**
 * Reverts a graph to nodes reachable from one or more checkpoint roots.
 */
// 1. Generic callable version for GetNeighbors
template <typename TNode, typename GetNeighborsType>
TSet<TNode> RevertGraphToCheckpoint(const TSet<TNode>& AllNodes, const TSet<TNode>& CheckpointRoots,
                                    GetNeighborsType&& GetNeighbors) {
    TSet<TNode> Reachable;
    TQueue<TNode> Queue;

    for (const TNode& Root : CheckpointRoots) {
        Queue.Enqueue(Root);
        Reachable.Add(Root);
    }

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (Reachable.Add(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return AllNodes.Intersect(Reachable);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TSet<TNode> RevertGraphToCheckpoint(const TSet<TNode>& AllNodes, const TSet<TNode>& CheckpointRoots,
                                    TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors) {
    return RevertGraphToCheckpoint<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(AllNodes, CheckpointRoots,
                                                                                     GetNeighbors);
}

/**
 * Prunes nodes from the graph that cannot be reached from valid root nodes.
 */
// 1. Generic callable version for GetLinks
template <typename TNode, typename GetLinksType>
TSet<TNode> PruneGraphFromInvalidNodes(const TSet<TNode>& AllNodes, const TSet<TNode>& ValidRoots,
                                       GetLinksType&& GetLinks) {
    TSet<TNode> ValidSet;
    TQueue<TNode> Queue;

    for (const TNode& Root : ValidRoots) {
        Queue.Enqueue(Root);
        ValidSet.Add(Root);
    }

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        for (const TNode& Link : GetLinks(Current)) {
            if (ValidSet.Add(Link)) {
                Queue.Enqueue(Link);
            }
        }
    }

    return AllNodes.Intersect(ValidSet);
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TSet<TNode> PruneGraphFromInvalidNodes(const TSet<TNode>& AllNodes, const TSet<TNode>& ValidRoots,
                                       TFunctionRef<TArray<TNode>(const TNode&)> GetLinks) {
    return PruneGraphFromInvalidNodes<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(AllNodes, ValidRoots, GetLinks);
}

// 1. Generic callable version for CopyFn and ValidPredicate
template <typename TData, typename TSource, typename CopyFnType, typename ValidPredicateType>
void CopyIfValid(TData& Target, const TSource& Source, CopyFnType&& CopyFn, ValidPredicateType&& ValidPredicate) {
    if (ValidPredicate(Source)) {
        CopyFn(Target, Source);
    }
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TData, typename TSource>
void CopyIfValid(TData& Target, const TSource& Source, TFunctionRef<void(TData&, const TSource&)> CopyFn,
                 TFunctionRef<bool(const TSource&)> ValidPredicate) {
    CopyIfValid<TData, TSource, TFunctionRef<void(TData&, const TSource&)>, TFunctionRef<bool(const TSource&)>>(
        Target, Source, CopyFn, ValidPredicate);
}

// 1. Generic callable version for Initializer
template <typename TData, typename TSource, typename InitializerType>
TArray<TData> InitializeObjectsFromSourceArray(const TArray<TSource*>& Sources, InitializerType&& Initializer) {
    TArray<TData> Result;
    for (const TSource* Source : Sources) {
        if (!Source) {
            continue;
        }

        TData Data;
        if (Initializer(Data, Source)) {
            Result.Add(Data);
        }
    }
    return Result;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TData, typename TSource>
TArray<TData> InitializeObjectsFromSourceArray(const TArray<TSource*>& Sources,
                                               TFunctionRef<bool(TData&, const TSource*)> Initializer) {
    return InitializeObjectsFromSourceArray<TData, TSource, TFunctionRef<bool(TData&, const TSource*)>>(Sources,
                                                                                                        Initializer);
}

// Generic callable version - accept callables by const reference (or value)
template <typename TKey, typename TValue, typename TElement, typename GetKeyAType, typename GetKeyBType,
          typename ValidateValueType, typename ExtraConstraintCheckType>
void RemoveIfAnyMissingOrInvalid(
    TArray<TElement>& Array, const TMap<TKey, TValue>& ReferenceMap, const GetKeyAType& GetKeyA,
    const GetKeyBType& GetKeyB, const ValidateValueType& ValidateValue,
    const ExtraConstraintCheckType& ExtraConstraintCheck = [](const TElement&) { return true; }) {
    Array.RemoveAll([&](const TElement& Element) {
        const TValue* A = ReferenceMap.Find(GetKeyA(Element));
        const TValue* B = ReferenceMap.Find(GetKeyB(Element));

        const bool bValidBones = A && B && ValidateValue(*A) && ValidateValue(*B);
        const bool bValidConstraint = ExtraConstraintCheck(Element);

        return !bValidBones || !bValidConstraint;
    });
}

// TFunctionRef explicit overload
template <typename TKey, typename TValue, typename TElement>
void RemoveIfAnyMissingOrInvalid(
    TArray<TElement>& Array, const TMap<TKey, TValue>& ReferenceMap, TFunctionRef<TKey(const TElement&)> GetKeyA,
    TFunctionRef<TKey(const TElement&)> GetKeyB, TFunctionRef<bool(const TValue&)> ValidateValue,
    TFunctionRef<bool(const TElement&)> ExtraConstraintCheck = [](const TElement&) { return true; }) {
    RemoveIfAnyMissingOrInvalid<TKey, TValue, TElement, TFunctionRef<TKey(const TElement&)>,
                                TFunctionRef<TKey(const TElement&)>, TFunctionRef<bool(const TValue&)>,
                                TFunctionRef<bool(const TElement&)>>(Array, ReferenceMap, GetKeyA, GetKeyB,
                                                                     ValidateValue, ExtraConstraintCheck);
}

// 1. Overload for TFunctionRef parameters (most specific)
template <typename TKey, typename TValue, typename TElement>
void RemoveIfInvalidByKey(TArray<TElement>& Array, const TMap<TKey, TValue>& Map,
                          const TFunctionRef<TKey(const TElement&)>& GetKey,
                          const TFunctionRef<bool(const TValue&)>& Validate) {
    Array.RemoveAll([&](const TElement& Elem) {
        const TValue* Val = Map.Find(GetKey(Elem));
        return !Val || !Validate(*Val);
    });
}

// 2. Generic callable overload by const reference (lambdas, functors)
template <typename TKey, typename TValue, typename TElement, typename GetKeyType, typename ValidateType>
void RemoveIfInvalidByKey(TArray<TElement>& Array, const TMap<TKey, TValue>& Map, const GetKeyType& GetKey,
                          const ValidateType& Validate) {
    Array.RemoveAll([&](const TElement& Elem) {
        const TValue* Val = Map.Find(GetKey(Elem));
        return !Val || !Validate(*Val);
    });
}

// 3. Perfect forwarding overload for rvalue callables (optional)
template <typename TKey, typename TValue, typename TElement, typename GetKeyType, typename ValidateType>
void RemoveIfInvalidByKey(TArray<TElement>& Array, const TMap<TKey, TValue>& Map, GetKeyType&& GetKey,
                          ValidateType&& Validate) {
    RemoveIfInvalidByKey<TKey, TValue, TElement>(Array, Map,
                                                 static_cast<const std::remove_reference_t<GetKeyType>&>(GetKey),
                                                 static_cast<const std::remove_reference_t<ValidateType>&>(Validate));
}

/**
 * Captures all nodes reachable from a root for rollback or snapshot.
 */
// 1. Generic callable version for GetLinks
template <typename TNode, typename GetLinksType>
TSet<TNode> CaptureGraphSnapshot(const TNode& Root, GetLinksType&& GetLinks) {
    TSet<TNode> Snapshot;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);
    Snapshot.Add(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);

        for (const TNode& Link : GetLinks(Current)) {
            if (!Snapshot.Contains(Link)) {
                Snapshot.Add(Link);
                Queue.Enqueue(Link);
            }
        }
    }

    return Snapshot;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TSet<TNode> CaptureGraphSnapshot(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks) {
    return CaptureGraphSnapshot<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(Root, GetLinks);
}

/**
 * Computes added and removed nodes between two graph states.
 */
template <typename TNode>
void DiffGraphStates(const TSet<TNode>& OldState, const TSet<TNode>& NewState, TSet<TNode>& OutRemoved,
                     TSet<TNode>& OutAdded) {
    OutRemoved = OldState.Difference(NewState);
    OutAdded = NewState.Difference(OldState);
}

/**
 * Restores a previous graph snapshot by removing any non-snapshot nodes.
 */
template <typename TNode>
TSet<TNode> RestoreGraphFromSnapshot(const TSet<TNode>& CurrentState, const TSet<TNode>& Snapshot) {
    return CurrentState.Intersect(Snapshot);
}

// ============================================================================
//  Multisource Rollback Arbitration (126–130)
// ============================================================================

/**
 * Assigns a float credibility score to each rollback source node.
 */
// 1. Generic callable version for ScoreFunc
template <typename TNode, typename ScoreFuncType>
TMap<TNode, float> ScoreRollbackSources(const TSet<TNode>& Sources, ScoreFuncType&& ScoreFunc) {
    TMap<TNode, float> Scores;
    for (const TNode& Node : Sources) {
        Scores.Add(Node, ScoreFunc(Node));
    }
    return Scores;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TMap<TNode, float> ScoreRollbackSources(const TSet<TNode>& Sources, TFunctionRef<float(const TNode&)> ScoreFunc) {
    return ScoreRollbackSources<TNode, TFunctionRef<float(const TNode&)>>(Sources, ScoreFunc);
}

/**
 * Selects the highest-confidence rollback source node.
 */
// 1. Generic callable version for ScoreFunc
template <typename TNode, typename ScoreFuncType>
TOptional<TNode> SelectBestRollbackSource(const TSet<TNode>& Candidates, ScoreFuncType&& ScoreFunc) {
    TOptional<TNode> Best;
    float MaxScore = -FLT_MAX;

    for (const TNode& Node : Candidates) {
        float Score = ScoreFunc(Node);
        if (!Best.IsSet() || Score > MaxScore) {
            Best = Node;
            MaxScore = Score;
        }
    }

    return Best;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TOptional<TNode> SelectBestRollbackSource(const TSet<TNode>& Candidates, TFunctionRef<float(const TNode&)> ScoreFunc) {
    return SelectBestRollbackSource<TNode, TFunctionRef<float(const TNode&)>>(Candidates, ScoreFunc);
}

/**
 * Combines multiple rollback snapshots with optional consensus enforcement.
 */
template <typename TNode>
TSet<TNode> MergeGraphSnapshots(const TArray<TSet<TNode>>& Snapshots, bool bRequireConsensus = false) {
    if (Snapshots.Num() == 0) {
        return {};
    }

    if (!bRequireConsensus) {
        TSet<TNode> Merged;
        for (const TSet<TNode>& Snapshot : Snapshots) {
            Merged.Append(Snapshot);
        }
        return Merged;
    }
    TSet<TNode> Intersect = Snapshots[0];
    for (int32 i = 1; i < Snapshots.Num(); ++i) {
        Intersect = Intersect.Intersect(Snapshots[i]);
    }
    return Intersect;
}

/**
 * Returns node appearance counts across rollback snapshots.
 */
template <typename TNode> TMap<TNode, int32> IsolateRollbackDivergences(const TArray<TSet<TNode>>& Snapshots) {
    TMap<TNode, int32> OccurrenceCount;

    for (const TSet<TNode>& Snapshot : Snapshots) {
        for (const TNode& Node : Snapshot) {
            ++OccurrenceCount.FindOrAdd(Node);
        }
    }

    return OccurrenceCount;
}

/**
 * Builds a rollback graph consisting only of nodes seen in N+ sources.
 */
template <typename TNode>
TSet<TNode> BuildConsensusRollbackGraph(const TArray<TSet<TNode>>& Snapshots, int32 QuorumThreshold) {
    TMap<TNode, int32> Occurrence = IsolateRollbackDivergences(Snapshots);
    TSet<TNode> Consensus;

    for (const auto& Pair : Occurrence) {
        if (Pair.Value >= QuorumThreshold) {
            Consensus.Add(Pair.Key);
        }
    }

    return Consensus;
}

// ============================================================================
// Fully Navigable Graph Framework (131–140)
// ============================================================================

/**
 * Builds a one-way forward graph from any root using neighbor logic.
 */
// 1. Generic callable version for GetNeighbors
template <typename TNode, typename GetNeighborsType>
TMap<TNode, TArray<TNode>> BuildForwardGraph(const TNode& Root, GetNeighborsType&& GetNeighbors) {
    TMap<TNode, TArray<TNode>> Graph;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;

    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        TArray<TNode> Neighbors = GetNeighbors(Current);
        Graph.Add(Current, Neighbors);

        for (const TNode& N : Neighbors) {
            if (!Visited.Contains(N)) {
                Queue.Enqueue(N);
            }
        }
    }

    return Graph;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TMap<TNode, TArray<TNode>> BuildForwardGraph(const TNode& Root,
                                             TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors) {
    return BuildForwardGraph<TNode, TFunctionRef<TArray<TNode>(const TNode&)>>(Root, GetNeighbors);
}

/**
 * Generates the reverse mapping of all node edges (for backtracking).
 */
// 1. Generic adjacency container type version
template <typename TNode, typename AdjacencyContainer = TArray<TNode>>
TMap<TNode, AdjacencyContainer> BuildReverseGraph(const TMap<TNode, AdjacencyContainer>& ForwardGraph) {
    TMap<TNode, AdjacencyContainer> ReverseGraph;

    for (const auto& Pair : ForwardGraph) {
        const TNode& From = Pair.Key;
        for (const TNode& To : Pair.Value) {
            ReverseGraph.FindOrAdd(To).Add(From);
        }
    }

    return ReverseGraph;
}

// 2. Explicit TArray adjacency overload for backward compatibility
template <typename TNode> TMap<TNode, TArray<TNode>> BuildReverseGraph(const TMap<TNode, TArray<TNode>>& ForwardGraph) {
    return BuildReverseGraph<TNode, TArray<TNode>>(ForwardGraph);
}

/**
 * Traces a path from root to a given target node in the reverse graph.
 */
template <typename TNode>
TOptional<TArray<TNode>> FindPathToNode(const TNode& Target, const TMap<TNode, TArray<TNode>>& ReverseGraph,
                                        const TNode& Root) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    TSet<TNode> Visited;

    Queue.Enqueue(Target);
    CameFrom.Add(Target, Target);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        if (Current == Root) {
            TArray<TNode> Path;
            TNode Step = Target;
            while (Step != Root) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Root, 0);
            return Path;
        }

        const TArray<TNode>* Parents = ReverseGraph.Find(Current);
        if (Parents) {
            for (const TNode& Parent : *Parents) {
                if (!CameFrom.Contains(Parent)) {
                    CameFrom.Add(Parent, Current);
                    Queue.Enqueue(Parent);
                }
            }
        }
    }

    return {};
}

// 1. Generic callable version for GetNeighbors
template <typename TKey, typename TValue, typename GetNeighborsType>
TMap<TKey, TValue> ExtractSubgraph(const TKey& Start, const TMap<TKey, TValue>& FullGraph,
                                   GetNeighborsType&& GetNeighbors, int32 MaxNodes = -1) {
    TSet<TKey> Reachable = TraverseWithNodeLimit<TKey>(
        Start, GetNeighbors, [](const TKey&) {}, // No-op visitor
        MaxNodes);

    TMap<TKey, TValue> Result;
    for (const TKey& Key : Reachable) {
        if (const TValue* Val = FullGraph.Find(Key)) {
            Result.Add(Key, *Val);
        }
    }
    return Result;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TKey, typename TValue>
TMap<TKey, TValue> ExtractSubgraph(const TKey& Start, const TMap<TKey, TValue>& FullGraph,
                                   TFunctionRef<TArray<TKey>(const TKey&)> GetNeighbors, int32 MaxNodes = -1) {
    return ExtractSubgraph<TKey, TValue, TFunctionRef<TArray<TKey>(const TKey&)>>(Start, FullGraph, GetNeighbors,
                                                                                  MaxNodes);
}

/**
 * Extracts all nodes between a root and target node.
 */
// 1. Generic callable version for ReverseGraph lookup
template <typename TNode, typename ReverseGraphType>
TSet<TNode> ExtractSubgraphToNode(const TNode& Target, const ReverseGraphType& ReverseGraph, const TNode& Root) {
    TSet<TNode> Subgraph;
    TFunction<void(const TNode&)> Recurse;
    Recurse = [&](const TNode& Node) {
        if (!Subgraph.Add(Node)) {
            return;
        }
        if (Node == Root) {
            return;
        }

        const auto* Parents = ReverseGraph.Find(Node);
        if (Parents) {
            for (const TNode& P : *Parents) {
                Recurse(P);
            }
        }
    };
    Recurse(Target);
    return Subgraph;
}

// 2. Explicit overload for TMap<TNode, TArray<TNode>> for backward compatibility
template <typename TNode>
TSet<TNode> ExtractSubgraphToNode(const TNode& Target, const TMap<TNode, TArray<TNode>>& ReverseGraph,
                                  const TNode& Root) {
    return ExtractSubgraphToNode<TNode, TMap<TNode, TArray<TNode>>>(Target, ReverseGraph, Root);
}

/**
 * Searches forward through a graph for a matching predicate node.
 */
// 1. Generic callable version for GetChildren and Predicate
template <typename TNode, typename GetChildrenType, typename PredicateType>
TOptional<TArray<TNode>> SearchGraphByPredicate(const TNode& Root, GetChildrenType&& GetChildren,
                                                PredicateType&& Predicate) {
    TMap<TNode, TNode> CameFrom;
    TQueue<TNode> Queue;
    TSet<TNode> Visited;

    Queue.Enqueue(Root);
    CameFrom.Add(Root, Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        if (Predicate(Current)) {
            TArray<TNode> Path;
            TNode Step = Current;
            while (Step != Root) {
                Path.Insert(Step, 0);
                Step = CameFrom[Step];
            }
            Path.Insert(Root, 0);
            return Path;
        }

        for (const TNode& Child : GetChildren(Current)) {
            if (!CameFrom.Contains(Child)) {
                CameFrom.Add(Child, Current);
                Queue.Enqueue(Child);
            }
        }
    }

    return {};
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode>
TOptional<TArray<TNode>> SearchGraphByPredicate(const TNode& Root,
                                                TFunctionRef<TArray<TNode>(const TNode&)> GetChildren,
                                                TFunctionRef<bool(const TNode&)> Predicate) {
    return SearchGraphByPredicate<TNode, TFunctionRef<TArray<TNode>(const TNode&)>, TFunctionRef<bool(const TNode&)>>(
        Root, GetChildren, Predicate);
}

// ============================================================================
//  Search Indexing and Data Extraction (141–145)
// ============================================================================

/**
 * Traverses a graph and indexes nodes by a key derived from each node.
 */
// 1. Generic callable version for GetNeighbors and ExtractKey
template <typename TNode, typename TKey, typename GetNeighborsType, typename ExtractKeyType>
TMap<TKey, TArray<TNode>> IndexGraphByKey(const TNode& Root, GetNeighborsType&& GetNeighbors,
                                          ExtractKeyType&& ExtractKey) {
    TMap<TKey, TArray<TNode>> Index;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Index.FindOrAdd(ExtractKey(Current)).Add(Current);

        for (const TNode& Neighbor : GetNeighbors(Current)) {
            if (!Visited.Contains(Neighbor)) {
                Queue.Enqueue(Neighbor);
            }
        }
    }

    return Index;
}

// 2. TFunctionRef explicit overload for backward compatibility
template <typename TNode, typename TKey>
TMap<TKey, TArray<TNode>> IndexGraphByKey(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetNeighbors,
                                          TFunctionRef<TKey(const TNode&)> ExtractKey) {
    return IndexGraphByKey<TNode, TKey, TFunctionRef<TArray<TNode>(const TNode&)>, TFunctionRef<TKey(const TNode&)>>(
        Root, GetNeighbors, ExtractKey);
}

/**
 * Collects all nodes in the graph that match a given predicate.
 */
template <typename TNode>
TArray<TNode> CollectAllMatchingInGraph(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks,
                                        TFunctionRef<bool(const TNode&)> Predicate) {
    TArray<TNode> Matches;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        if (Predicate(Current)) {
            Matches.Add(Current);
        }

        for (const TNode& Next : GetLinks(Current)) {
            if (!Visited.Contains(Next)) {
                Queue.Enqueue(Next);
            }
        }
    }

    return Matches;
}

/**
 * Extracts specific field values from all reachable nodes in the graph.
 */
template <typename TNode, typename TValue>
TArray<TValue> ExtractFieldValuesFromGraph(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks,
                                           TFunctionRef<TValue(const TNode&)> Extract) {
    TArray<TValue> Out;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Out.Add(Extract(Current));

        for (const TNode& Next : GetLinks(Current)) {
            if (!Visited.Contains(Next)) {
                Queue.Enqueue(Next);
            }
        }
    }

    return Out;
}

/**
 * Maps a graph to a flattened list of transformed struct data.
 */
template <typename TNode, typename TStruct>
TArray<TStruct> MapGraphToStructs(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks,
                                  TFunctionRef<TStruct(const TNode&)> Project) {
    TArray<TStruct> Results;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Results.Add(Project(Current));

        for (const TNode& Next : GetLinks(Current)) {
            if (!Visited.Contains(Next)) {
                Queue.Enqueue(Next);
            }
        }
    }

    return Results;
}

/**
 * Builds a node-to-value cache of extracted fields during traversal.
 */
template <typename TNode, typename TValue>
TMap<TNode, TValue> BuildGraphValueCache(const TNode& Root, TFunctionRef<TArray<TNode>(const TNode&)> GetLinks,
                                         TFunctionRef<TValue(const TNode&)> Extract) {
    TMap<TNode, TValue> Cache;
    TSet<TNode> Visited;
    TQueue<TNode> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty()) {
        TNode Current;
        Queue.Dequeue(Current);
        if (!Visited.Add(Current)) {
            continue;
        }

        Cache.Add(Current, Extract(Current));

        for (const TNode& Next : GetLinks(Current)) {
            if (!Visited.Contains(Next)) {
                Queue.Enqueue(Next);
            }
        }
    }

    return Cache;
}

template <typename TKey, typename TValue, typename TConflictResolver>
bool TryInsertOrResolve(TMap<TKey, TValue>& Map, const TKey& Key, const TValue& Value,
                        TConflictResolver&& ConflictResolver) {
    if (Map.Contains(Key)) {
        TValue& Existing = Map[Key];
        Existing = ConflictResolver(Existing, Value);
        return false; // conflict handled via resolution
    }
    Map.Add(Key, Value);
    return true; // added cleanly
}

template <typename TKey, typename TValue, typename TConflictPredicate>
bool TryInsertIfNoConflict(TMap<TKey, TValue>& Map, const TKey& Key, const TValue& Value,
                           TConflictPredicate&& ConflictPredicate) {
    if (Map.Contains(Key)) {
        const TValue& Existing = Map[Key];
        if (ConflictPredicate(Existing, Value)) {
            return false; // conflict detected, reject
        }
        return true; // exists but not a conflict — safe to keep existing
    }
    Map.Add(Key, Value);
    return true; // inserted
}

template <typename TKey, typename TValue, typename TValidator>
int32 InsertAllValidated(TMap<TKey, TValue>& Map, const TMap<TKey, TValue>& Source, TValidator&& Validator) {
    int32 Count = 0;
    for (const auto& Pair : Source) {
        if (Validator(Pair.Value) && !Map.Contains(Pair.Key)) {
            Map.Add(Pair.Key, Pair.Value);
            ++Count;
        }
    }
    return Count;
}

template <typename TKey, typename TValue, typename TConflictPredicate>
int32 InsertAllIfNoConflict(TMap<TKey, TValue>& Map, const TMap<TKey, TValue>& Source,
                            TConflictPredicate&& ConflictPredicate) {
    int32 Count = 0;
    for (const auto& Pair : Source) {
        if (!Map.Contains(Pair.Key)) {
            Map.Add(Pair.Key, Pair.Value);
            ++Count;
        } else {
            const TValue& Existing = Map[Pair.Key];
            if (!ConflictPredicate(Existing, Pair.Value)) {
                ++Count; // counted as a non-conflict skip
            }
        }
    }
    return Count;
}

template <typename TKey, typename TValue, typename TResolver>
int32 ResolveAllConflicts(TMap<TKey, TValue>& Map, const TMap<TKey, TValue>& Source, TResolver&& Resolver) {
    int32 Resolved = 0;
    for (const auto& Pair : Source) {
        if (Map.Contains(Pair.Key)) {
            TValue& Existing = Map[Pair.Key];
            Existing = Resolver(Existing, Pair.Value);
            ++Resolved;
        }
    }
    return Resolved;
}

/**
 * Filters keys from a map where the associated value passes a predicate.
 */
template <typename TKey, typename TValue, typename TPredicate>
TArray<TKey> FilterKeysByValue(const TMap<TKey, TValue>& Map, TPredicate&& Predicate) {
    TArray<TKey> Keys;
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Value)) {
            Keys.Add(Pair.Key);
        }
    }
    return Keys;
}

template <typename K, typename V>
TMap<K, V> FilterMapValues(const TMap<K, V>& Source, TFunctionRef<bool(const V&)> Predicate) {
    TMap<K, V> Result;
    for (const TPair<K, V>& Pair : Source) {
        if (Predicate(Pair.Value)) {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

template <typename T, typename TPredicate> TArray<T> FilterArray(const TArray<T>& Input, TPredicate&& Predicate) {
    TArray<T> Output;
    for (const T& Item : Input) {
        if (Predicate(Item)) {
            Output.Add(Item);
        }
    }
    return Output;
}

namespace OHSafeMapUtils {
template <typename T> TArray<T> FilterArray(const TArray<T>& Source, TFunctionRef<bool(const T&)> Predicate) {
    TArray<T> Result;
    for (const T& Item : Source) {
        if (Predicate(Item)) {
            Result.Add(Item);
        }
    }
    return Result;
}
} // namespace OHSafeMapUtils

template <typename TKey, typename TValue, typename TPredicate>
TArray<TValue> FilterMapValues(const TMap<TKey, TValue>& Map, TPredicate&& Predicate) {
    TArray<TValue> Results;
    for (const auto& Pair : Map) {
        if (Predicate(Pair.Value)) {
            Results.Add(Pair.Value);
        }
    }
    return Results;
}

template <typename T> TArray<T> FilterArray(const TArray<T>& Source, TFunctionRef<bool(const T&)> Predicate) {
    TArray<T> Result;
    for (const T& Item : Source) {
        if (Predicate(Item)) {
            Result.Add(Item);
        }
    }
    return Result;
}

template <typename T, typename TTransform> TSet<T> MapArrayToSet(const TArray<T>& Input, TTransform&& Func) {
    TSet<T> Result;
    for (const T& Item : Input) {
        Result.Add(Func(Item));
    }
    return Result;
}

template <typename T, typename S> TSet<S> MapArrayToSet(const TArray<T>& Source, TFunctionRef<S(const T&)> Projector) {
    TSet<S> Result;
    for (const T& Item : Source) {
        Result.Add(Projector(Item));
    }
    return Result;
}

// With BreakOnFalse
template <typename K, typename V>
TMap<K, V> FilterMapValuesBreakable(const TMap<K, V>& Source, TFunctionRef<bool(const V&)> Predicate) {
    TMap<K, V> Result;
    for (const TPair<K, V>& Pair : Source) {
        if (!Predicate(Pair.Value)) {
            break;
        }

        Result.Add(Pair.Key, Pair.Value);
    }
    return Result;
}

// With a context object
template <typename K, typename V, typename C>
TMap<K, V> FilterMapValuesWithContext(const TMap<K, V>& Source, const C& Context,
                                      TFunctionRef<bool(const V&, const C&)> Predicate) {
    TMap<K, V> Result;
    for (const TPair<K, V>& Pair : Source) {
        if (Predicate(Pair.Value, Context)) {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}

// Suppress null projections
template <typename T, typename S>
TSet<S> MapArrayToSetSuppressNull(const TArray<T>& Source, TFunctionRef<TOptional<S>(const T&)> Projector) {
    TSet<S> Result;
    for (const T& Item : Source) {
        TOptional<S> Mapped = Projector(Item);
        if (Mapped.IsSet()) {
            Result.Add(Mapped.GetValue());
        }
    }
    return Result;
}

// With context
template <typename T, typename S, typename C>
TSet<S> MapArrayToSetWithContext(const TArray<T>& Source, const C& Context,
                                 TFunctionRef<S(const T&, const C&)> Projector) {
    TSet<S> Result;
    for (const T& Item : Source) {
        Result.Add(Projector(Item, Context));
    }
    return Result;
}

template <typename K, typename V, typename S>
TSet<S> MapMapToSet(const TMap<K, V>& Source, TFunctionRef<S(const V&)> Projector) {
    TSet<S> Result;
    for (const auto& Pair : Source) {
        Result.Add(Projector(Pair.Value));
    }
    return Result;
}

// With context
template <typename K, typename V, typename S, typename C>
TSet<S> MapMapToSetWithContext(const TMap<K, V>& Source, const C& Context,
                               TFunctionRef<S(const V&, const C&)> Projector) {
    TSet<S> Result;
    for (const auto& Pair : Source) {
        Result.Add(Projector(Pair.Value, Context));
    }
    return Result;
}

// Suppress null/invalid
template <typename K, typename V, typename S>
TSet<S> MapMapToSetSuppressNull(const TMap<K, V>& Source, TFunctionRef<TOptional<S>(const V&)> Projector) {
    TSet<S> Result;
    for (const auto& Pair : Source) {
        TOptional<S> Mapped = Projector(Pair.Value);
        if (Mapped.IsSet()) {
            Result.Add(Mapped.GetValue());
        }
    }
    return Result;
}

// Filter by index only
template <typename T> TArray<T> FilterArrayByIndex(const TArray<T>& Source, TFunctionRef<bool(int32)> IndexPredicate) {
    TArray<T> Result;
    for (int32 i = 0; i < Source.Num(); ++i) {
        if (IndexPredicate(i)) {
            Result.Add(Source[i]);
        }
    }
    return Result;
}

// Filter by value and index
template <typename T>
TArray<T> FilterArrayByIndexValue(const TArray<T>& Source, TFunctionRef<bool(const T&, int32)> Predicate) {
    TArray<T> Result;
    for (int32 i = 0; i < Source.Num(); ++i) {
        if (Predicate(Source[i], i)) {
            Result.Add(Source[i]);
        }
    }
    return Result;
}

template <typename T, typename S> TArray<S> MapSetToArray(const TSet<T>& Source, TFunctionRef<S(const T&)> Projector) {
    TArray<S> Result;
    for (const T& Item : Source) {
        Result.Add(Projector(Item));
    }
    return Result;
}

template <typename T, typename S, typename C>
TArray<S> MapSetToArrayWithContext(const TSet<T>& Source, const C& Context,
                                   TFunctionRef<S(const T&, const C&)> Projector) {
    TArray<S> Result;
    for (const T& Item : Source) {
        Result.Add(Projector(Item, Context));
    }
    return Result;
}

template <typename T> void SortArrayStableByKey(TArray<T>& Array, TFunctionRef<bool(const T&, const T&)> Comparator) {
    Array.Sort(Comparator); // Stable in UE5
}

template <typename T, typename K>
void SortArrayStableByField(TArray<T>& Array, TFunctionRef<K(const T&)> KeyFunc, bool bAscending = true) {
    Array.Sort([&](const T& A, const T& B) {
        const K KeyA = KeyFunc(A);
        const K KeyB = KeyFunc(B);
        return bAscending ? KeyA < KeyB : KeyB < KeyA;
    });
}

// Filter → Map
template <typename T, typename S>
TArray<S> FilterThenMap(const TArray<T>& Source, TFunctionRef<bool(const T&)> Filter,
                        TFunctionRef<S(const T&)> Project) {
    TArray<S> Result;
    for (const T& Item : Source) {
        if (Filter(Item)) {
            Result.Add(Project(Item));
        }
    }
    return Result;
}

// Map → Filter
template <typename T, typename S>
TArray<S> MapThenFilter(const TArray<T>& Source, TFunctionRef<S(const T&)> Project,
                        TFunctionRef<bool(const S&)> Filter) {
    TArray<S> Result;
    for (const T& Item : Source) {
        const S Transformed = Project(Item);
        if (Filter(Transformed)) {
            Result.Add(Transformed);
        }
    }
    return Result;
}

// Filter/Map on Set → Array
template <typename T, typename S>
TArray<S> FilterMapSetToArray(const TSet<T>& Source, TFunctionRef<bool(const T&)> Filter,
                              TFunctionRef<S(const T&)> Project) {
    TArray<S> Result;
    for (const T& Item : Source) {
        if (Filter(Item)) {
            Result.Add(Project(Item));
        }
    }
    return Result;
}

// Filter → Map (with index)
template <typename T, typename S>
TArray<S> FilterThenMapIndexed(const TArray<T>& Source, TFunctionRef<bool(const T&, int32)> Filter,
                               TFunctionRef<S(const T&, int32)> Project) {
    TArray<S> Result;
    for (int32 i = 0; i < Source.Num(); ++i) {
        if (Filter(Source[i], i)) {
            Result.Add(Project(Source[i], i));
        }
    }
    return Result;
}

// Map → Filter (with index)
template <typename T, typename S>
TArray<S> MapThenFilterIndexed(const TArray<T>& Source, TFunctionRef<S(const T&, int32)> Project,
                               TFunctionRef<bool(const S&, int32)> Filter) {
    TArray<S> Result;
    for (int32 i = 0; i < Source.Num(); ++i) {
        const S Mapped = Project(Source[i], i);
        if (Filter(Mapped, i)) {
            Result.Add(Mapped);
        }
    }
    return Result;
}

// Map → Sort by key
template <typename T, typename S, typename K>
TArray<S> MapThenSort(const TArray<T>& Source, TFunctionRef<S(const T&)> Project, TFunctionRef<K(const S&)> KeyFunc,
                      bool bAscending = true) {
    TArray<S> Result;
    for (const T& Item : Source) {
        Result.Add(Project(Item));
    }
    Result.Sort([&](const S& A, const S& B) {
        const K KeyA = KeyFunc(A);
        const K KeyB = KeyFunc(B);
        return bAscending ? KeyA < KeyB : KeyB < KeyA;
    });
    return Result;
}

// Index-aware Map → Sort
template <typename T, typename S, typename K>
TArray<S> MapThenSortIndexed(const TArray<T>& Source, TFunctionRef<S(const T&, int32)> Project,
                             TFunctionRef<K(const S&)> KeyFunc, bool bAscending = true) {
    TArray<S> Result;
    for (int32 i = 0; i < Source.Num(); ++i) {
        Result.Add(Project(Source[i], i));
    }
    Result.Sort([&](const S& A, const S& B) {
        const K KeyA = KeyFunc(A);
        const K KeyB = KeyFunc(B);
        return bAscending ? KeyA < KeyB : KeyB < KeyA;
    });
    return Result;
}

template <typename T, typename S> TSet<S> MapUnique(const TArray<T>& Source, TFunctionRef<S(const T&)> Projector) {
    TSet<S> Result;
    for (const T& Item : Source) {
        Result.Add(Projector(Item));
    }
    return Result;
}

template <typename T> TMap<int32, int32> StableIndexRemap(const TArray<T>& Original, const TArray<T>& Transformed) {
    TMap<int32, int32> IndexMap;

    for (int32 OldIndex = 0; OldIndex < Original.Num(); ++OldIndex) {
        const T& OldItem = Original[OldIndex];

        for (int32 NewIndex = 0; NewIndex < Transformed.Num(); ++NewIndex) {
            if (Transformed[NewIndex] == OldItem) {
                IndexMap.Add(OldIndex, NewIndex);
                break;
            }
        }
    }

    return IndexMap;
}

template <typename T>
void ReverseArrayWithIndexMap(const TArray<T>& Source, TArray<T>& OutReversed, TMap<int32, int32>& OutIndexRemap) {
    const int32 Count = Source.Num();
    OutReversed.Reserve(Count);
    OutIndexRemap.Reserve(Count);

    for (int32 i = Count - 1; i >= 0; --i) {
        OutReversed.Add(Source[i]);
        OutIndexRemap.Add(i, Count - 1 - i);
    }
}

inline int32 WrapCircular(int32 Index, int32 Count) {
    return (Index % Count + Count) % Count;
}

inline TMap<int32, int32> CircularIndexMap(int32 Count, int32 Offset) {
    TMap<int32, int32> Result;
    for (int32 i = 0; i < Count; ++i) {
        Result.Add(i, WrapCircular(i + Offset, Count));
    }
    return Result;
}

template <typename K, typename V> TMultiMap<V, K> InvertMap(const TMap<K, V>& Source) {
    TMultiMap<V, K> Inverted;
    for (const TPair<K, V>& Pair : Source) {
        Inverted.Add(Pair.Value, Pair.Key);
    }
    return Inverted;
}

template <typename A, typename B>
bool ValidateRemapConsistency(const TMap<A, B>& Forward, const TMultiMap<B, A>& Reverse) {
    for (const TPair<A, B>& Pair : Forward) {
        const A* BackMapped = Reverse.Find(Pair.Value);
        if (!BackMapped || *BackMapped != Pair.Key) {
            return false;
        }
    }
    return true;
}

template <typename T> TArray<T> RemapArrayUsingMap(const TArray<T>& Source, const TMap<int32, int32>& Remap) {
    TArray<T> Result;
    Result.SetNum(Source.Num());

    for (const TPair<int32, int32>& Pair : Remap) {
        if (Source.IsValidIndex(Pair.Key) && Result.IsValidIndex(Pair.Value)) {
            Result[Pair.Value] = Source[Pair.Key];
        }
    }

    return Result;
}

template <typename T>
bool VerifyGraphMappingCompleteness(const TSet<T>& AllNodes, const TMap<T, T>& Mapping, bool bAllowSelfMap = false) {
    TSet<T> MappedTargets;
    for (const TPair<T, T>& Pair : Mapping) {
        if (!AllNodes.Contains(Pair.Key) || !AllNodes.Contains(Pair.Value)) {
            return false;
        }

        if (!bAllowSelfMap && Pair.Key == Pair.Value) {
            return false;
        }

        MappedTargets.Add(Pair.Value);
    }

    return MappedTargets.Num() == Mapping.Num();
}

template <typename TItem, typename TPriority> class TPriorityQueue {
    struct FElement {
        TItem Item;
        TPriority Priority;

        bool operator<(const FElement& Other) const {
            // Min-heap: smaller priority = higher priority
            return Priority > Other.Priority;
        }
    };

    TArray<FElement> Heap;
    TMap<TItem, int32> ItemIndexMap; // Tracks position of items in the heap

    void SwapElements(int32 IndexA, int32 IndexB) {
        Swap(Heap[IndexA], Heap[IndexB]);
        ItemIndexMap[Heap[IndexA].Item] = IndexA;
        ItemIndexMap[Heap[IndexB].Item] = IndexB;
    }

    void HeapifyUp(int32 Index) {
        while (Index > 0) {
            int32 ParentIndex = (Index - 1) / 2;
            if (!(Heap[Index] < Heap[ParentIndex])) {
                break;
            }
            SwapElements(Index, ParentIndex);
            Index = ParentIndex;
        }
    }

    void HeapifyDown(int32 Index) {
        const int32 Count = Heap.Num();
        while (true) {
            int32 LeftChild = 2 * Index + 1;
            int32 RightChild = LeftChild + 1;
            int32 Smallest = Index;

            if (LeftChild < Count && Heap[LeftChild] < Heap[Smallest]) {
                Smallest = LeftChild;
            }

            if (RightChild < Count && Heap[RightChild] < Heap[Smallest]) {
                Smallest = RightChild;
            }

            if (Smallest == Index) {
                break;
            }

            SwapElements(Index, Smallest);
            Index = Smallest;
        }
    }

  public:
    bool IsEmpty() const {
        return Heap.Num() == 0;
    }

    bool Contains(const TItem& Item) const {
        return ItemIndexMap.Contains(Item);
    }

    void Enqueue(const TItem& Item, TPriority Priority) {
        if (Contains(Item)) {
            // Update priority if better (lower)
            int32 Index = ItemIndexMap[Item];
            if (Heap[Index].Priority > Priority) {
                Heap[Index].Priority = Priority;
                HeapifyUp(Index);
            }
        } else {
            Heap.Add({Item, Priority});
            int32 NewIndex = Heap.Num() - 1;
            ItemIndexMap.Add(Item, NewIndex);
            HeapifyUp(NewIndex);
        }
    }

    bool Dequeue(TItem& OutItem) {
        if (IsEmpty()) {
            return false;
        }

        OutItem = Heap[0].Item;
        ItemIndexMap.Remove(OutItem);

        if (Heap.Num() == 1) {
            Heap.RemoveAt(0);
            return true;
        }

        Heap[0] = Heap.Last();
        ItemIndexMap[Heap[0].Item] = 0;
        Heap.RemoveAt(Heap.Num() - 1);
        HeapifyDown(0);

        return true;
    }
};

/**
 * Highly flexible and extensible graph traits interface for A* pathfinding.
 *
 * @tparam TNode    Node identifier type.
 * @tparam TContext Optional context type for dynamic graph info; default is void.
 */
template <typename TNode, typename TContext = void> struct TAStarGraphTraits {
    // Virtual destructor for safe polymorphism
    virtual ~TAStarGraphTraits() = default;

    /**
     * Get neighbors of a given node.
     * Can optionally use Context to provide cached or async data.
     */
    virtual TArray<TNode> GetNeighbors(const TNode& Node, const TContext* Context = nullptr) const = 0;

    /**
     * Get cost from NodeA to NodeB.
     * Can use Context to modify cost dynamically.
     */
    virtual float GetCost(const TNode& NodeA, const TNode& NodeB, const TContext* Context = nullptr) const = 0;

    /**
     * Heuristic estimates cost from Node to Goal.
     * Should be admissible (never overestimate).
     * Can use Context to adjust heuristic dynamically.
     */
    virtual float Heuristic(const TNode& Node, const TNode& Goal, const TContext* Context = nullptr) const = 0;

    /**
     * Check if a neighbor is valid to traverse from a given node.
     * Allows skipping invalid or blocked edges.
     * Returns true by default (all neighbors are valid).
     */
    virtual bool IsValidNeighbor(const TNode& FromNode, const TNode& Neighbor,
                                 const TContext* Context = nullptr) const {
        return true;
    }

    /**
     * Optional hook called when a node is added to the open set.
     * Useful for debugging or instrumentation.
     */
    virtual void OnNodeOpened(const TNode& Node, const TContext* Context = nullptr) const {}

    /**
     * Optional hook called when a node is moved from the open set to the closed set.
     * Useful for debugging or instrumentation.
     */
    virtual void OnNodeClosed(const TNode& Node, const TContext* Context = nullptr) const {}

    /**
     * Optional hook called when the path is successfully found.
     * Allows custom processing or logging.
     */
    virtual void OnPathFound(const TArray<TNode>& Path, const TContext* Context = nullptr) const {}
};

template <typename TNode, typename TContext = void>
class TGenericGraphAStarTraits : public TAStarGraphTraits<TNode, TContext> {
  public:
    // Function types now accept context pointer (can be nullptr if unused)
    using FGetNeighborsFunc = TFunction<TArray<TNode>(const TNode&, const TContext*)>;
    using FGetCostFunc = TFunction<float(const TNode&, const TNode&, const TContext*)>;
    using FHeuristicFunc = TFunction<float(const TNode&, const TNode&, const TContext*)>;
    using FIsValidNeighborFunc = TFunction<bool(const TNode&, const TNode&, const TContext*)>;

  private:
    FGetNeighborsFunc GetNeighborsFunc;
    FGetCostFunc GetCostFunc;
    FHeuristicFunc HeuristicFunc;
    FIsValidNeighborFunc IsValidNeighborFunc;

  public:
    TGenericGraphAStarTraits(
        FGetNeighborsFunc InGetNeighbors, FGetCostFunc InGetCost, FHeuristicFunc InHeuristic,
        FIsValidNeighborFunc InIsValidNeighbor = [](const TNode&, const TNode&, const TContext*) { return true; })
        : GetNeighborsFunc(InGetNeighbors), GetCostFunc(InGetCost), HeuristicFunc(InHeuristic),
          IsValidNeighborFunc(InIsValidNeighbor) {}

    virtual TArray<TNode> GetNeighbors(const TNode& Node, const TContext* Context = nullptr) const override {
        return GetNeighborsFunc(Node, Context);
    }

    virtual float GetCost(const TNode& NodeA, const TNode& NodeB, const TContext* Context = nullptr) const override {
        return GetCostFunc(NodeA, NodeB, Context);
    }

    virtual float Heuristic(const TNode& Node, const TNode& Goal, const TContext* Context = nullptr) const override {
        return HeuristicFunc(Node, Goal, Context);
    }

    virtual bool IsValidNeighbor(const TNode& FromNode, const TNode& Neighbor,
                                 const TContext* Context = nullptr) const override {
        return IsValidNeighborFunc(FromNode, Neighbor, Context);
    }

    virtual void OnNodeOpened(const TNode& Node, const TContext* Context = nullptr) const override {}

    virtual void OnNodeClosed(const TNode& Node, const TContext* Context = nullptr) const override {}

    virtual void OnPathFound(const TArray<TNode>& Path, const TContext* Context = nullptr) const override {}
};

/**
 * Extended generic A* search engine template supporting:
 * - Multi-goal pathfinding,
 * - Runtime-switchable heuristic,
 * - Dynamic cost penalties,
 * - Incremental (stepwise) search,
 * - Context support and status callbacks.
 *
 * @tparam TNode Node type.
 * @tparam TContext Context type passed to traits (default void).
 * @tparam TKeyFuncs Key func for hashing (default Unreal default).
 * @tparam TEqualityComparer Equality comparer for TNode (default Unreal default).
 */
template <typename TNode, typename TContext = void,
          typename TKeyFuncs = TDefaultMapHashableKeyFuncs<TNode, void, false>,
          typename TEqualityComparer = TEqualTo<TNode>>
class TAStarSearch {
  public:
    enum class ESearchStatus : uint8 { NotStarted, InProgress, Found, Aborted, NotFound };

  private:
    const TAStarGraphTraits<TNode, TContext>& GraphTraits;
    const TContext* Context;
    int32 MaxNodesProcessed = 10000;

    TFunction<float(const TNode&, const TNode&)> RuntimeHeuristic;
    TFunction<float(const TNode&, const TNode&)> CostModifier;
    TFunction<void(ESearchStatus)> StatusCallback;

    TPriorityQueue<TNode, float> OpenSet;
    TSet<TNode, FDefaultSetAllocator, TKeyFuncs> OpenSetTracker;
    TSet<TNode, FDefaultSetAllocator, TKeyFuncs> ClosedSet;
    TMap<TNode, TAStarNodeData<TNode>, FDefaultSetAllocator, TKeyFuncs> NodeDataMap;

    TEqualityComparer EqualityComparer;

    TSet<TNode, FDefaultSetAllocator, TKeyFuncs> Goals;
    TNode ReachedGoal;
    bool bHasReachedGoal = false;

    ESearchStatus CurrentStatus = ESearchStatus::NotStarted;

    int32 ProcessedCount = 0;

  public:
    explicit TAStarSearch(const TAStarGraphTraits<TNode, TContext>& InTraits, const TContext* InContext = nullptr,
                          int32 InMaxNodesProcessed = 10000)
        : GraphTraits(InTraits), Context(InContext), MaxNodesProcessed(InMaxNodesProcessed) {}

    void SetHeuristicFunction(TFunction<float(const TNode&, const TNode&)> InHeuristic) {
        RuntimeHeuristic = MoveTemp(InHeuristic);
    }

    void SetCostModifierFunction(TFunction<float(const TNode&, const TNode&)> InCostModifier) {
        CostModifier = MoveTemp(InCostModifier);
    }

    void SetStatusCallback(TFunction<void(ESearchStatus)> InCallback) {
        StatusCallback = MoveTemp(InCallback);
    }

    ESearchStatus GetStatus() const {
        return CurrentStatus;
    }

    // Initialize or reset search with start and goal set
    void InitializeSearch(const TNode& Start, const TSet<TNode, FDefaultSetAllocator, TKeyFuncs>& InGoals) {
        check(InGoals.Num() > 0);

        ResetSearch();

        Goals = InGoals;
        bHasReachedGoal = false;
        ReachedGoal = TNode();

        TAStarNodeData<TNode>& StartData = NodeDataMap.FindOrAdd(Start);
        StartData.GCost = 0.f;
        StartData.FCost = ComputeMinHeuristic(Start, Goals);

        OpenSet.Enqueue(Start, StartData.FCost);
        OpenSetTracker.Add(Start);
        GraphTraits.OnNodeOpened(Start, Context);

        CurrentStatus = ESearchStatus::InProgress;
        NotifyStatus(CurrentStatus);
    }

    // Incremental search step: process up to MaxIterations nodes; returns false if search is finished
    bool StepSearch(int32 MaxIterations) {
        if (CurrentStatus != ESearchStatus::InProgress) {
            return false;
        }

        int32 Iterations = 0;

        while (!OpenSet.IsEmpty() && Iterations < MaxIterations) {
            if (++ProcessedCount > MaxNodesProcessed) {
                CurrentStatus = ESearchStatus::Aborted;
                NotifyStatus(CurrentStatus);
                return false;
            }

            TNode Current;
            OpenSet.Dequeue(Current);
            OpenSetTracker.Remove(Current);
            ClosedSet.Add(Current);
            GraphTraits.OnNodeClosed(Current, Context);

            if (Goals.Contains(Current)) {
                bHasReachedGoal = true;
                ReachedGoal = Current;
                CurrentStatus = ESearchStatus::Found;
                NotifyStatus(CurrentStatus);
                return false;
            }

            for (const TNode& Neighbor : GraphTraits.GetNeighbors(Current, Context)) {
                if (!GraphTraits.IsValidNeighbor(Current, Neighbor, Context)) {
                    continue;
                }

                if (ClosedSet.Contains(Neighbor)) {
                    continue;
                }

                float TentativeG = NodeDataMap[Current].GCost + ComputeCost(Current, Neighbor);

                TAStarNodeData<TNode>& NeighborData = NodeDataMap.FindOrAdd(Neighbor);

                if (TentativeG < NeighborData.GCost) {
                    NeighborData.GCost = TentativeG;
                    NeighborData.FCost = TentativeG + ComputeMinHeuristic(Neighbor, Goals);
                    NeighborData.Parent = Current;
                    NeighborData.bHasParent = true;

                    if (!OpenSetTracker.Contains(Neighbor)) {
                        OpenSet.Enqueue(Neighbor, NeighborData.FCost);
                        OpenSetTracker.Add(Neighbor);
                        GraphTraits.OnNodeOpened(Neighbor, Context);
                    }
                }
            }

            ++Iterations;
        }

        if (OpenSet.IsEmpty() && !bHasReachedGoal) {
            CurrentStatus = ESearchStatus::NotFound;
            NotifyStatus(CurrentStatus);
            return false;
        }

        return true;
    }

    // Convenience synchronous search for single goal
    bool FindPath(const TNode& Start, const TNode& Goal, TArray<TNode>& OutPath) {
        TSet<TNode, FDefaultSetAllocator, TKeyFuncs> GoalsSet = {Goal};
        return FindPath(Start, GoalsSet, OutPath);
    }

    // Synchronous multi-goal search for compatibility (calls StepSearch internally)
    bool FindPath(const TNode& Start, const TSet<TNode, FDefaultSetAllocator, TKeyFuncs>& InGoals,
                  TArray<TNode>& OutPath, TNode* OutReachedGoal = nullptr) {
        InitializeSearch(Start, InGoals);

        while (StepSearch(1000)) {
        } // Large step count for synchronous search

        if (bHasReachedGoal) {
            ReconstructPath(OutPath);
            if (OutReachedGoal) {
                *OutReachedGoal = ReachedGoal;
            }
            GraphTraits.OnPathFound(OutPath, Context);
            return true;
        }

        OutPath.Empty();
        return false;
    }

    // Returns true if search is done (success, failure, or aborted)
    bool IsSearchComplete() const {
        return CurrentStatus == ESearchStatus::Found || CurrentStatus == ESearchStatus::Aborted ||
               CurrentStatus == ESearchStatus::NotFound;
    }

    // Retrieve path if found after incremental or synchronous search
    bool GetPath(TArray<TNode>& OutPath) const {
        if (!bHasReachedGoal) {
            return false;
        }

        ReconstructPath(OutPath);
        return true;
    }

  private:
    void ResetSearch() {
        OpenSet = {};
        OpenSetTracker.Empty();
        ClosedSet.Empty();
        NodeDataMap.Empty();
        Goals.Empty();
        bHasReachedGoal = false;
        ReachedGoal = TNode();
        CurrentStatus = ESearchStatus::NotStarted;
        ProcessedCount = 0;
    }

    void NotifyStatus(ESearchStatus Status) const {
        if (StatusCallback) {
            StatusCallback(Status);
        }
    }

    float ComputeMinHeuristic(const TNode& Node, const TSet<TNode, FDefaultSetAllocator, TKeyFuncs>& InGoals) const {
        if (InGoals.Num() == 1) {
            return ComputeHeuristic(Node, *InGoals.CreateConstIterator());
        }

        float MinHeuristic = TNumericLimits<float>::Max();
        for (const TNode& Goal : InGoals) {
            float H = ComputeHeuristic(Node, Goal);
            if (H < MinHeuristic) {
                MinHeuristic = H;
            }
        }
        return MinHeuristic;
    }

    float ComputeHeuristic(const TNode& Node, const TNode& Goal) const {
        if (RuntimeHeuristic) {
            return RuntimeHeuristic(Node, Goal);
        }
        return GraphTraits.Heuristic(Node, Goal, Context);
    }

    float ComputeCost(const TNode& From, const TNode& To) const {
        float BaseCost = GraphTraits.GetCost(From, To, Context);
        if (CostModifier) {
            return BaseCost + CostModifier(From, To);
        }
        return BaseCost;
    }

    void ReconstructPath(const TNode& Start, const TNode& Goal, TArray<TNode>& OutPath) const {
        OutPath.Empty();

        TNode Current = Goal;
        while (!EqualityComparer()(Current, Start)) {
            OutPath.Insert(Current, 0);
            const TAStarNodeData<TNode>* Data = NodeDataMap.Find(Current);
            check(Data && Data->bHasParent);
            Current = Data->Parent;
        }
        OutPath.Insert(Start, 0);
    }

    // Helper for incremental search to reconstruct path from stored ReachedGoal
    void ReconstructPath(TArray<TNode>& OutPath) const {
        ReconstructPath(*Goals.CreateConstIterator(), ReachedGoal, OutPath);
    }
};

template <typename TElement, typename TKey>
TMap<TKey, TArray<TElement>> GroupBy(const TArray<TElement>& Items, TFunctionRef<TKey(const TElement&)> GetKey) {
    TMap<TKey, TArray<TElement>> Result;
    for (const TElement& Item : Items) {
        Result.FindOrAdd(GetKey(Item)).Add(Item);
    }
    return Result;
}

template <typename K1, typename V, typename K2>
TMap<K2, V> TransformKeys(const TMap<K1, V>& Input, TFunctionRef<K2(const K1&)> KeyTransform) {
    TMap<K2, V> Result;
    for (const auto& Pair : Input) {
        Result.Add(KeyTransform(Pair.Key), Pair.Value);
    }
    return Result;
}

template <typename TElement, typename TResult>
TResult Aggregate(const TArray<TElement>& Items, TResult Initial,
                  TFunctionRef<TResult(const TResult&, const TElement&)> Reducer) {
    TResult Result = Initial;
    for (const TElement& Item : Items) {
        Result = Reducer(Result, Item);
    }
    return Result;
}

template <typename K, typename V>
TMap<K, V> FilterMap(const TMap<K, V>& Input, TFunctionRef<bool(const K&, const V&)> Predicate) {
    TMap<K, V> Result;
    for (const auto& Pair : Input) {
        if (Predicate(Pair.Key, Pair.Value)) {
            Result.Add(Pair.Key, Pair.Value);
        }
    }
    return Result;
}
} // namespace OHSafeMapUtils
