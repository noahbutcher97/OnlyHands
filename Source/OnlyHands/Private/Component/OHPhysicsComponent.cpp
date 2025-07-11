#include "Component/OHPhysicsComponent.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Chaos/PhysicsObject.h"
#include "Chaos/ImplicitObject.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PhysicsEngine/PhysicsHandleComponent.h"
#include "Engine/EngineTypes.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Physics/PhysicsInterfaceUtils.h"
#include "Physics/PhysicsInterfaceDeclares.h"
#include "Physics/PhysicsFiltering.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "Chaos/PBDRigidParticles.h"
#include "FunctionLibrary/OHAlgoUtils.h" // For fuzzy and matching
#include "Animation/Skeleton.h"
#include "DrawDebugHelpers.h"
#include "PackageTools.h"
#include "PhysicsAssetUtils.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Data/OHPhysicalAnimationProfileDataAsset.h"

// --------- CONSTRUCTOR ---------
UOHPhysicsComponent::UOHPhysicsComponent() {
    PrimaryComponentTick.bCanEverTick = true;
}

// --------- LIFECYCLE ---------

void UOHPhysicsComponent::InitializeComponent() {
    Super::InitializeComponent();
    SetupPhysicsAnimationSystem();
}

void UOHPhysicsComponent::OnUnregister() {
    TeardownPhysicsAnimationSystem();
    Super::OnUnregister();
}

void UOHPhysicsComponent::BeginPlay() {
    Super::BeginPlay();

    // Find skeletal mesh (assumes single mesh per actor; adapt as needed)
    SkeletalMeshComponent = GetOwner() ? GetOwner()->FindComponentByClass<USkeletalMeshComponent>() : nullptr;

    InitializeRuntimeChains();
}

void UOHPhysicsComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    // Cleanup
    for (auto& Chain : RuntimeChains) {
        for (auto& BoneDrive : Chain.BoneDrives) {
            DestroyBoneDriveInstance(BoneDrive);
        }
    }
    RuntimeChains.Empty();
    Super::EndPlay(EndPlayReason);
}

void UOHPhysicsComponent::SetupPhysicsAnimationSystem() {
    // --- Find or Set Skeletal Mesh Component ---
    if (!SkeletalMeshComponent) {
        SkeletalMeshComponent = GetOwner() ? GetOwner()->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
    }
    if (!SkeletalMeshComponent) {
        UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] No SkeletalMeshComponent found on %s!"), *GetName());
        return;
    }

    // --- Find or Set Physics Asset ---
    PhysicsAsset = SkeletalMeshComponent->GetPhysicsAsset();
    if (!PhysicsAsset) {
        if (DefaultPhysicsAsset) {
            PhysicsAsset = DefaultPhysicsAsset;
            SkeletalMeshComponent->SetPhysicsAsset(DefaultPhysicsAsset, false);
            UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] Used DefaultPhysicsAsset for %s."), *GetName());
        }
        /*#if WITH_EDITOR
                        else
                        {
                                // Try to auto-generate a PhysicsAsset in editor
                                PhysicsAsset =
        GeneratePhysicsAssetForMesh(SkeletalMeshComponent->GetSkeletalMeshAsset()); if (PhysicsAsset)
                                {
                                        SkeletalMeshComponent->SetPhysicsAsset(PhysicsAsset, false);
                                        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] Auto-generated PhysicsAsset
        for %s."), *GetName());
                                }
                        }
        #endif*/
    }
    if (!PhysicsAsset) {
        UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] No valid PhysicsAsset for %s! Setup aborted."), *GetName());
        return;
    }

    // --- Auto-populate chains from skeleton/physics asset if empty ---
    if (Chains.Num() == 0) {
        AutoPopulateChainsFromSkeleton();
        UE_LOG(LogTemp, Log, TEXT("[OHPhysicsComponent] Auto-populated chains for %s."), *GetName());
    }

    // --- Setup Editor Visualization (optional) ---
#if WITH_EDITOR
    SetupEditorVisualization();
#endif

    // --- Validate runtime physics state against asset template ---
    ValidateRuntimePhysicsAgainstTemplate();
    FixAllMissingPhysicsForComponent(SkeletalMeshComponent, PhysicsAsset, this);
    ValidateRuntimePhysicsAgainstTemplate(); // Confirm fixed!
}

void UOHPhysicsComponent::TeardownPhysicsAnimationSystem() {
    // Remove runtime constraints and actors
    for (FPhysicalAnimationChainRuntime& Chain : RuntimeChains) {
        for (FPhysicalAnimationBoneDriveInstance& Drive : Chain.BoneDrives) {
            DestroyBoneDriveInstance(Drive);
        }
    }
    RuntimeChains.Empty();

#if WITH_EDITOR
    CleanupEditorVisualization();
#endif

    // Optionally clear references
    PhysicsAsset = nullptr;
}

void UOHPhysicsComponent::FixAllMissingPhysicsForComponent(USkeletalMeshComponent* SkeletalMeshComponent,
                                                           UPhysicsAsset* PhysicsAsset, UObject* WorldContext) {
    if (!SkeletalMeshComponent || !PhysicsAsset)
        return;

    int32 BodiesCreated = 0;
    int32 ConstraintsCreated = 0;
    int32 ParentConstraintsCreated = 0;

    // 1. Fix all missing bodies
    for (const USkeletalBodySetup* BodySetup : PhysicsAsset->SkeletalBodySetups) {
        FName BoneName = BodySetup->BoneName;
        FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            if (UOHPhysicsComponent::EnsureBodyForBone_Static(BoneName, SkeletalMeshComponent, PhysicsAsset))
                ++BodiesCreated;
        }
    }

    // 2. Fix all missing asset constraints
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : PhysicsAsset->ConstraintSetup) {
        const FConstraintInstance& Instance = ConstraintTemplate->DefaultInstance;
        FName Bone1 = Instance.ConstraintBone1;
        FName Bone2 = Instance.ConstraintBone2;

        if (UOHPhysicsComponent::EnsureBodiesAndConstraint_Static(Bone1, Bone2, SkeletalMeshComponent, PhysicsAsset))
            ++ConstraintsCreated;
    }

    // 3. (Optional) Strict: add missing parent-child constraints for all bones
    const FReferenceSkeleton& RefSkel = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
    for (int32 BoneIdx = 0; BoneIdx < RefSkel.GetNum(); ++BoneIdx) {
        FName BoneName = RefSkel.GetBoneName(BoneIdx);
        int32 ParentIdx = RefSkel.GetParentIndex(BoneIdx);
        if (ParentIdx != INDEX_NONE) {
            FName ParentName = RefSkel.GetBoneName(ParentIdx);
            if (UOHPhysicsComponent::EnsureBodiesAndConstraint_Static(BoneName, ParentName, SkeletalMeshComponent,
                                                                      PhysicsAsset))
                ++ParentConstraintsCreated;
        }
    }

    // 4. On-screen summary
    FString LogMsg = FString::Printf(TEXT("[OHPhysicsComponent] Batch Repair Complete: %d bodies, %d asset "
                                          "constraints, %d parent constraints created."),
                                     BodiesCreated, ConstraintsCreated, ParentConstraintsCreated);

    UE_LOG(LogTemp, Warning, TEXT("%s"), *LogMsg);
    if (GEngine) {
        GEngine->AddOnScreenDebugMessage(-1, 7.0f, FColor::Green, LogMsg);
    }
}

FBodyInstance* UOHPhysicsComponent::EnsureBodyForBone_Static(FName BoneName,
                                                             USkeletalMeshComponent* SkeletalMeshComponent,
                                                             UPhysicsAsset* PhysicsAsset) {
#if WITH_EDITOR
    int32 BodyIdx = PhysicsAsset->FindBodyIndex(BoneName);
    if (BodyIdx == INDEX_NONE) {
        FPhysAssetCreateParams Params;
        // int32 NewBodyIdx = FPhysicsAssetUtils::CreateNewBody(PhysicsAsset, BoneName, Params);
        PhysicsAsset->UpdateBodySetupIndexMap();
        PhysicsAsset->UpdateBoundsBodiesArray();
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] (Static) Created new body for bone: %s in PhysicsAsset."),
               *BoneName.ToString());
        // BodyIdx = NewBodyIdx;
    }
#endif
    FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        SkeletalMeshComponent->RecreatePhysicsState();
        Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            UE_LOG(LogTemp, Error,
                   TEXT("[OHPhysicsComponent] (Static) Failed to create runtime BodyInstance for bone: %s"),
                   *BoneName.ToString());
            return nullptr;
        }
    }
    return Body;
}

FConstraintInstance* UOHPhysicsComponent::EnsureConstraintBetweenBones_Static(
    FName Bone1, FName Bone2, USkeletalMeshComponent* SkeletalMeshComponent, UPhysicsAsset* PhysicsAsset) {
#if WITH_EDITOR
    int32 FoundIdx = INDEX_NONE;
    for (int32 Idx = 0; Idx < PhysicsAsset->ConstraintSetup.Num(); ++Idx) {
        const UPhysicsConstraintTemplate* Constraint = PhysicsAsset->ConstraintSetup[Idx];
        const FConstraintInstance& AssetInst = Constraint->DefaultInstance;
        if ((AssetInst.ConstraintBone1 == Bone1 && AssetInst.ConstraintBone2 == Bone2) ||
            (AssetInst.ConstraintBone1 == Bone2 && AssetInst.ConstraintBone2 == Bone1)) {
            FoundIdx = Idx;
            break;
        }
    }
    if (FoundIdx == INDEX_NONE) {
        FString ConstraintName = FString::Printf(TEXT("Constraint_%s_%s"), *Bone1.ToString(), *Bone2.ToString());
        /*  int32 NewIdx = FPhysicsAssetUtils::CreateNewConstraint(PhysicsAsset, FName(*ConstraintName));
                  if (NewIdx != INDEX_NONE)
                  {
                          UPhysicsConstraintTemplate* NewConstraint = PhysicsAsset->ConstraintSetup[NewIdx];
             NewConstraint->DefaultInstance.ConstraintBone1 = Bone1;
             NewConstraint->DefaultInstance.ConstraintBone2 = Bone2;
              UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] (Static) Created new constraint between %s and %s."),
          *Bone1.ToString(), *Bone2.ToString()); FoundIdx = NewIdx;
          }*/
    }
#endif
    FConstraintInstance* RuntimeConstraint = nullptr;
    for (FConstraintInstance* C : SkeletalMeshComponent->Constraints) {
        if ((C->ConstraintBone1 == Bone1 && C->ConstraintBone2 == Bone2) ||
            (C->ConstraintBone1 == Bone2 && C->ConstraintBone2 == Bone1)) {
            RuntimeConstraint = C;
            break;
        }
    }
    if (!RuntimeConstraint) {
        SkeletalMeshComponent->RecreatePhysicsState();
        for (FConstraintInstance* C : SkeletalMeshComponent->Constraints) {
            if ((C->ConstraintBone1 == Bone1 && C->ConstraintBone2 == Bone2) ||
                (C->ConstraintBone1 == Bone2 && C->ConstraintBone2 == Bone1)) {
                RuntimeConstraint = C;
                break;
            }
        }
        if (!RuntimeConstraint) {
            UE_LOG(LogTemp, Error,
                   TEXT("[OHPhysicsComponent] (Static) Failed to create runtime ConstraintInstance between %s and %s!"),
                   *Bone1.ToString(), *Bone2.ToString());
        }
    }
    return RuntimeConstraint;
}

FConstraintInstance* UOHPhysicsComponent::EnsureBodiesAndConstraint_Static(
    FName Bone1, FName Bone2, USkeletalMeshComponent* SkeletalMeshComponent, UPhysicsAsset* PhysicsAsset) {
    FBodyInstance* BodyA = EnsureBodyForBone_Static(Bone1, SkeletalMeshComponent, PhysicsAsset);
    FBodyInstance* BodyB = EnsureBodyForBone_Static(Bone2, SkeletalMeshComponent, PhysicsAsset);
    if (!BodyA || !BodyB)
        return nullptr;
    FConstraintInstance* Constraint =
        EnsureConstraintBetweenBones_Static(Bone1, Bone2, SkeletalMeshComponent, PhysicsAsset);
    return Constraint;
}

void UOHPhysicsComponent::ValidateRuntimePhysicsAgainstTemplate() {
    if (!SkeletalMeshComponent || !PhysicsAsset)
        return;

    // --- Validate bodies ---
    for (const USkeletalBodySetup* BodySetup : PhysicsAsset->SkeletalBodySetups) {
        FName BoneName = BodySetup->BoneName;
        FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] MISSING OR INVALID BODY for bone: %s on %s."),
                   *BoneName.ToString(), *GetName());
        }
    }

    // --- Validate constraints ---
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : PhysicsAsset->ConstraintSetup) {
        const FConstraintInstance& Instance = ConstraintTemplate->DefaultInstance;
        FName JointName = Instance.JointName;
        FName Bone1 = Instance.ConstraintBone1;
        FName Bone2 = Instance.ConstraintBone2;

        int32 Bone1Idx = SkeletalMeshComponent->GetBoneIndex(Bone1);
        int32 Bone2Idx = SkeletalMeshComponent->GetBoneIndex(Bone2);

        // 1. Are both bones present in the skeleton?
        if (Bone1Idx == INDEX_NONE) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] Constraint [%s]: Bone1 (%s) NOT found in skeletal mesh!"),
                   *JointName.ToString(), *Bone1.ToString());
            continue;
        }
        if (Bone2Idx == INDEX_NONE) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] Constraint [%s]: Bone2 (%s) NOT found in skeletal mesh!"),
                   *JointName.ToString(), *Bone2.ToString());
            continue;
        }

        // 2. Are both bones present in the physics simulation (i.e., have valid BodyInstances)?
        FBodyInstance* Body1 = SkeletalMeshComponent->GetBodyInstance(Bone1);
        FBodyInstance* Body2 = SkeletalMeshComponent->GetBodyInstance(Bone2);
        if (!Body1 || !Body1->IsValidBodyInstance()) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] Constraint [%s]: Bone1 (%s) has NO valid BodyInstance!"),
                   *JointName.ToString(), *Bone1.ToString());
            continue;
        }
        if (!Body2 || !Body2->IsValidBodyInstance()) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] Constraint [%s]: Bone2 (%s) has NO valid BodyInstance!"),
                   *JointName.ToString(), *Bone2.ToString());
            continue;
        }

        // 3. (Optional) Check if the runtime constraint exists in the SkeletalMeshComponent
        bool bFoundConstraint = false;
        for (const FConstraintInstance* RuntimeConstraint : SkeletalMeshComponent->Constraints) {
            if (!RuntimeConstraint)
                continue;
            if ((RuntimeConstraint->ConstraintBone1 == Bone1 && RuntimeConstraint->ConstraintBone2 == Bone2) ||
                (RuntimeConstraint->ConstraintBone1 == Bone2 && RuntimeConstraint->ConstraintBone2 == Bone1)) {
                bFoundConstraint = true;
                break;
            }
        }
        if (!bFoundConstraint) {
            UE_LOG(
                LogTemp, Warning,
                TEXT("[OHPhysicsComponent] Constraint [%s] between %s <-> %s: NO matching runtime constraint found!"),
                *JointName.ToString(), *Bone1.ToString(), *Bone2.ToString());
            // Optionally, call a helper to create/repair the constraint!
        }
    }

    // 4. (Optional) Check for any runtime bodies or constraints NOT present in the PhysicsAsset (out-of-sync issues)
    for (const FConstraintInstance* RuntimeConstraint : SkeletalMeshComponent->Constraints) {
        if (!RuntimeConstraint)
            continue;
        bool bAssetHasConstraint = false;
        for (const UPhysicsConstraintTemplate* ConstraintTemplate : PhysicsAsset->ConstraintSetup) {
            const FConstraintInstance& AssetInst = ConstraintTemplate->DefaultInstance;
            if ((AssetInst.ConstraintBone1 == RuntimeConstraint->ConstraintBone1 &&
                 AssetInst.ConstraintBone2 == RuntimeConstraint->ConstraintBone2) ||
                (AssetInst.ConstraintBone1 == RuntimeConstraint->ConstraintBone2 &&
                 AssetInst.ConstraintBone2 == RuntimeConstraint->ConstraintBone1)) {
                bAssetHasConstraint = true;
                break;
            }
        }
        if (!bAssetHasConstraint) {
            UE_LOG(LogTemp, Warning,
                   TEXT("[OHPhysicsComponent] RUNTIME constraint between %s <-> %s does NOT exist in PhysicsAsset!"),
                   *RuntimeConstraint->ConstraintBone1.ToString(), *RuntimeConstraint->ConstraintBone2.ToString());
        }
    }
}

FBodyInstance* UOHPhysicsComponent::EnsureBodyForBone(FName BoneName) {
    if (!PhysicsAsset || !SkeletalMeshComponent)
        return nullptr;

    // Check for body in asset
    int32 BodyIdx = PhysicsAsset->FindBodyIndex(BoneName);
    if (BodyIdx == INDEX_NONE) {
#if WITH_EDITOR
        // Add body to asset!
        FPhysAssetCreateParams Params;
        // (Customize Params as needed, e.g. geometry type, etc)
        //	int32 NewBodyIdx = FPhysicsAssetUtils::CreateNewBody(PhysicsAsset, BoneName, Params);
        PhysicsAsset->UpdateBodySetupIndexMap();
        PhysicsAsset->UpdateBoundsBodiesArray();
        UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] Created new body for bone: %s in PhysicsAsset."),
               *BoneName.ToString());
        //	BodyIdx = NewBodyIdx;
#endif
    }

    // Try to find runtime body instance in mesh
    FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
    if (!Body || !Body->IsValidBodyInstance()) {
        // The asset now has a body, but it might not be instanced yet (after reinit)
        SkeletalMeshComponent->RecreatePhysicsState();
        Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
        if (!Body || !Body->IsValidBodyInstance()) {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] Failed to create runtime BodyInstance for bone: %s"),
                   *BoneName.ToString());
            return nullptr;
        }
    }
    return Body;
}

FConstraintInstance* UOHPhysicsComponent::EnsureConstraintBetweenBones(FName Bone1, FName Bone2) {
    if (!PhysicsAsset || !SkeletalMeshComponent)
        return nullptr;

    // Search for constraint in asset (in either direction)
    int32 FoundIdx = INDEX_NONE;
    for (int32 Idx = 0; Idx < PhysicsAsset->ConstraintSetup.Num(); ++Idx) {
        const UPhysicsConstraintTemplate* Constraint = PhysicsAsset->ConstraintSetup[Idx];
        const FConstraintInstance& AssetInst = Constraint->DefaultInstance;
        if ((AssetInst.ConstraintBone1 == Bone1 && AssetInst.ConstraintBone2 == Bone2) ||
            (AssetInst.ConstraintBone1 == Bone2 && AssetInst.ConstraintBone2 == Bone1)) {
            FoundIdx = Idx;
            break;
        }
    }

    if (FoundIdx == INDEX_NONE) {
#if WITH_EDITOR
        // Create constraint asset entry!
        FString ConstraintName = FString::Printf(TEXT("Constraint_%s_%s"), *Bone1.ToString(), *Bone2.ToString());
        /* int32 NewIdx = FPhysicsAssetUtils::CreateNewConstraint(PhysicsAsset, FName(*ConstraintName));
         if (NewIdx != INDEX_NONE)
         {
             UPhysicsConstraintTemplate* NewConstraint = PhysicsAsset->ConstraintSetup[NewIdx];
             NewConstraint->DefaultInstance.ConstraintBone1 = Bone1;
             NewConstraint->DefaultInstance.ConstraintBone2 = Bone2;
             // Optionally set constraint profile properties here!
             UE_LOG(LogTemp, Warning, TEXT("[OHPhysicsComponent] Created new constraint between %s and %s."),
         *Bone1.ToString(), *Bone2.ToString()); FoundIdx = NewIdx;
         }*/
#endif
    }

    // Try to find or create runtime constraint
    FConstraintInstance* RuntimeConstraint = nullptr;
    for (FConstraintInstance* C : SkeletalMeshComponent->Constraints) {
        if ((C->ConstraintBone1 == Bone1 && C->ConstraintBone2 == Bone2) ||
            (C->ConstraintBone1 == Bone2 && C->ConstraintBone2 == Bone1)) {
            RuntimeConstraint = C;
            break;
        }
    }

    if (!RuntimeConstraint) {
        // Runtime constraints are generated after physics reinit
        SkeletalMeshComponent->RecreatePhysicsState();
        for (FConstraintInstance* C : SkeletalMeshComponent->Constraints) {
            if ((C->ConstraintBone1 == Bone1 && C->ConstraintBone2 == Bone2) ||
                (C->ConstraintBone1 == Bone2 && C->ConstraintBone2 == Bone1)) {
                RuntimeConstraint = C;
                break;
            }
        }
        if (!RuntimeConstraint) {
            UE_LOG(LogTemp, Error,
                   TEXT("[OHPhysicsComponent] Failed to create runtime ConstraintInstance between %s and %s!"),
                   *Bone1.ToString(), *Bone2.ToString());
        }
    }
    return RuntimeConstraint;
}

FConstraintInstance* UOHPhysicsComponent::EnsureBodiesAndConstraint(FName Bone1, FName Bone2) {
    // Ensure both bodies
    FBodyInstance* BodyA = EnsureBodyForBone(Bone1);
    FBodyInstance* BodyB = EnsureBodyForBone(Bone2);
    if (!BodyA || !BodyB)
        return nullptr;

    // Ensure constraint
    FConstraintInstance* Constraint = EnsureConstraintBetweenBones(Bone1, Bone2);
    return Constraint;
}

#if WITH_EDITOR
void UOHPhysicsComponent::SetupEditorVisualization() {
    if (!PhysicsAsset || !SkeletalMeshComponent)
        return;

    UWorld* World = GetWorld();
    if (!World)
        return;

    // Draw bodies
    for (const USkeletalBodySetup* BodySetup : PhysicsAsset->SkeletalBodySetups) {
        int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(BodySetup->BoneName);
        if (BoneIdx == INDEX_NONE)
            continue;
        FTransform TM = SkeletalMeshComponent->GetBoneTransform(BoneIdx);
        FVector Loc = TM.GetLocation();

        DrawDebugBox(World, Loc, FVector(2, 2, 2), TM.GetRotation(), FColor::Cyan, false, 15.f, 0, 0.75f);
        DrawDebugString(World, Loc, BodySetup->BoneName.ToString(), nullptr, FColor::Cyan, 15.f, true);
    }

    // Draw constraint arrows
    for (const UPhysicsConstraintTemplate* ConstraintTemplate : PhysicsAsset->ConstraintSetup) {
        const FConstraintInstance& Instance = ConstraintTemplate->DefaultInstance;
        int32 IdxA = SkeletalMeshComponent->GetBoneIndex(Instance.ConstraintBone1);
        int32 IdxB = SkeletalMeshComponent->GetBoneIndex(Instance.ConstraintBone2);
        if (IdxA == INDEX_NONE || IdxB == INDEX_NONE)
            continue;

        FVector A = SkeletalMeshComponent->GetBoneTransform(IdxA).GetLocation();
        FVector B = SkeletalMeshComponent->GetBoneTransform(IdxB).GetLocation();
        DrawDebugDirectionalArrow(World, A, B, 12.0f, FColor::Yellow, false, 15.f, 0, 2.f);
    }
}

void UOHPhysicsComponent::CleanupEditorVisualization() {
    // Optionally: DrawDebug functions with limited duration will auto-cleanup
    // If you use persistent debug objects or widgets, destroy them here
}
#endif

// --------- CHAIN INITIALIZATION ---------
void UOHPhysicsComponent::InitializeRuntimeChains() {
    RuntimeChains.Empty();
    if (!SkeletalMeshComponent)
        return;

    for (const auto& ChainSetting : Chains) {
        FPhysicalAnimationChainRuntime ChainRuntime;
        ChainRuntime.Settings = ChainSetting;
        ChainRuntime.RuntimeBlendAlpha = ChainSetting.BlendAlpha;
        ChainRuntime.bActive = ChainSetting.bEnabled;

        TArray<int32> BoneIndices = GetBoneIndicesInChain(ChainSetting.StartBone, ChainSetting.EndBone);

        for (int32 BoneIndex : BoneIndices) {
            FPhysicalAnimationBoneDriveInstance BoneDrive;
            BoneDrive.BoneName = SkeletalMeshComponent->GetBoneName(BoneIndex);
            BoneDrive.CurrentDriveSettings = ChainSetting.DriveSettings;
            BoneDrive.BlendAlpha = ChainSetting.BlendAlpha;
            BoneDrive.bActive = ChainSetting.bEnabled;

            // Optionally: Setup constraint and target actor here (or defer to Tick)
            // For now, we defer creation until Tick when drive is enabled

            ChainRuntime.BoneDrives.Add(BoneDrive);
        }

        RuntimeChains.Add(ChainRuntime);
    }
}

// --------- PER-TICK UPDATE ---------
void UOHPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                        FActorComponentTickFunction* ThisTickFunction) {
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UpdatePhysicalAnimation(DeltaTime);

    if (bDebugDraw) {
        DrawDebugInfo();
    }
}

void UOHPhysicsComponent::UpdatePhysicalAnimation(float DeltaTime) {
    if (!SkeletalMeshComponent)
        return;

    for (FPhysicalAnimationChainRuntime& Chain : RuntimeChains) {
        // Blending logic
        float TargetAlpha = Chain.Settings.bEnabled ? 1.f : 0.f;
        Chain.RuntimeBlendAlpha = FMath::FInterpTo(Chain.RuntimeBlendAlpha, TargetAlpha, DeltaTime, 6.f);

        for (FPhysicalAnimationBoneDriveInstance& BoneDrive : Chain.BoneDrives) {
            BoneDrive.BlendAlpha = Chain.RuntimeBlendAlpha;
            BoneDrive.bActive = (Chain.RuntimeBlendAlpha > KINDA_SMALL_NUMBER);

            // Setup/teardown constraint as enabled/disabled
            if (BoneDrive.bActive && !BoneDrive.ConstraintInstance) {
                const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneDrive.BoneName);
                const FTransform AnimTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);

                CreateBoneDriveInstance(BoneDrive, BoneDrive.CurrentDriveSettings, AnimTM);
            } else if (!BoneDrive.bActive && BoneDrive.ConstraintInstance) {
                // Destroy target/constraint
                DestroyBoneDriveInstance(BoneDrive);
                continue;
            }

            // If not active, skip
            if (!BoneDrive.bActive || !BoneDrive.ConstraintInstance)
                continue;

            // Move target actor and update constraint drive (main magic)
            const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneDrive.BoneName);
            const FTransform AnimTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);

            UpdateBoneDriveInstance(BoneDrive, AnimTM, Chain.Settings.DriveSettings, BoneDrive.BlendAlpha);
        }
    }
}

// --------- ENABLE/DISABLE CHAINS ---------
void UOHPhysicsComponent::SetChainEnabled(FName ChainName, bool bEnabled) {
    for (auto& Chain : RuntimeChains) {
        if (Chain.Settings.ChainName == ChainName) {
            Chain.Settings.bEnabled = bEnabled;
            break;
        }
    }
}

void UOHPhysicsComponent::SetChainBlendAlpha(FName ChainName, float BlendAlpha) {
    for (auto& Chain : RuntimeChains) {
        if (Chain.Settings.ChainName == ChainName) {
            Chain.Settings.BlendAlpha = FMath::Clamp(BlendAlpha, 0.f, 1.f);
            break;
        }
    }
}

void UOHPhysicsComponent::SetChainDriveSettings(FName ChainName, const FPhysicalAnimationDriveSettings& NewSettings) {
    for (auto& Chain : RuntimeChains) {
        if (Chain.Settings.ChainName == ChainName) {
            Chain.Settings.DriveSettings = NewSettings;
            break;
        }
    }
}

void UOHPhysicsComponent::AutoPopulateDefaultUE4Chains() {
    Chains.Empty();

    // These are the standard UE4 Mannequin bone names (Epic Skeleton)
    Chains.Add({"Spine", "spine_01", "spine_03", DefaultSpineDrive, 1.f, true});
    Chains.Add({"LeftArm", "upperarm_l", "hand_l", DefaultArmDrive, 1.f, true});
    Chains.Add({"RightArm", "upperarm_r", "hand_r", DefaultArmDrive, 1.f, true});
    Chains.Add({"LeftLeg", "thigh_l", "foot_l", DefaultLegDrive, 1.f, true});
    Chains.Add({"RightLeg", "thigh_r", "foot_r", DefaultLegDrive, 1.f, true});
    Chains.Add({"Head", "neck_01", "head", DefaultHeadDrive, 1.f, true});

    InitializeRuntimeChains();
}

void UOHPhysicsComponent::AutoPopulateDefaultUE5Chains() {
    Chains.Empty();

    // UE5 Mannequin (Manny/Quinn) - different bone names, especially for arms/legs
    Chains.Add({"Spine", "spine_01", "spine_05", DefaultSpineDrive, 1.f, true});
    Chains.Add({"LeftArm", "upperarm_l", "hand_l", DefaultArmDrive, 1.f, true});
    Chains.Add({"RightArm", "upperarm_r", "hand_r", DefaultArmDrive, 1.f, true});
    Chains.Add({"LeftLeg", "thigh_l", "foot_l", DefaultLegDrive, 1.f, true});
    Chains.Add({"RightLeg", "thigh_r", "foot_r", DefaultLegDrive, 1.f, true});
    Chains.Add({"Head", "neck_01", "head", DefaultHeadDrive, 1.f, true});

    // If you want to support fingers, toes, etc, just add more chains here using their actual bone names.

    InitializeRuntimeChains();
}

void UOHPhysicsComponent::AutoPopulateChainsForCurrentMesh() {
    if (!SkeletalMeshComponent) {
        UE_LOG(LogTemp, Warning, TEXT("No SkeletalMeshComponent assigned!"));
        return;
    }

    // Gather all bone names as strings
    TArray<FName> BoneNames;
    const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
    for (int32 i = 0; i < RefSkeleton.GetNum(); ++i) {
        BoneNames.Add(RefSkeleton.GetBoneName(i));
    }
    TArray<FString> BoneStrs;
    for (const FName& N : BoneNames) {
        BoneStrs.Add(N.ToString());
    }
    // Heuristic: UE5 Mannequin signature bones
    const TArray<FString> UE5SignatureBones = {
        TEXT("spine_05"), TEXT("clavicle_l"), TEXT("upperarm_l"), TEXT("lowerarm_l"), TEXT("hand_l"),
        TEXT("thigh_l"),  TEXT("calf_l"),     TEXT("foot_l"),     TEXT("neck_01"),    TEXT("head")};
    const TArray<FString> UE4SignatureBones = {
        TEXT("spine_03"), TEXT("clavicle_l"), TEXT("upperarm_l"), TEXT("lowerarm_l"), TEXT("hand_l"),
        TEXT("thigh_l"),  TEXT("calf_l"),     TEXT("foot_l"),     TEXT("neck_01"),    TEXT("head")};

    // Fuzzy match signature bones
    int32 UE5Match = 0, UE4Match = 0;
    for (const FString& Sig : UE5SignatureBones) {
        // Use your fuzzy matcher
        auto Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(Sig, BoneStrs);
        if (Match.Score > 0.95f)
            UE5Match++;
    }
    for (const FString& Sig : UE4SignatureBones) {
        auto Match = UOHAlgoUtils::FindBestNameMatchAutoStrategy(Sig, BoneStrs);
        if (Match.Score > 0.95f)
            UE4Match++;
    }

    // Decide skeleton type
    FString ChosenType = (UE5Match > 6) ? TEXT("UE5") : (UE4Match > 6 ? TEXT("UE4") : TEXT("UE4"));
    UE_LOG(LogTemp, Log, TEXT("AutoPopulate: Matches UE5: %d, UE4: %d. Choosing %s Mannequin config."), UE5Match,
           UE4Match, *ChosenType);

    if (ChosenType == TEXT("UE5")) {
        AutoPopulateDefaultUE5Chains();
    } else {
        AutoPopulateDefaultUE4Chains();
    }
}

// --------- BONE INDEX UTILITY ---------
TArray<int32> UOHPhysicsComponent::GetBoneIndicesInChain(FName StartBone, FName EndBone) const {
    TArray<int32> Indices;
    if (!SkeletalMeshComponent)
        return Indices;

    const FReferenceSkeleton& RefSkel = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
    const int32 StartIdx = RefSkel.FindBoneIndex(StartBone);
    const int32 EndIdx = RefSkel.FindBoneIndex(EndBone);
    if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE)
        return Indices;

    // This basic version walks parent->child in a straight line
    int32 CurIdx = StartIdx;
    Indices.Add(CurIdx);
    while (CurIdx != EndIdx) {
        const int32 NumChildren = RefSkel.GetNum();
        bool bFoundChild = false;
        for (int32 i = 0; i < NumChildren; ++i) {
            if (RefSkel.GetParentIndex(i) == CurIdx) {
                Indices.Add(i);
                CurIdx = i;
                bFoundChild = true;
                break;
            }
        }
        if (!bFoundChild)
            break; // Could not walk further
    }
    return Indices;
}

void UOHPhysicsComponent::CreateBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive,
                                                  const FPhysicalAnimationDriveSettings& DriveSettings,
                                                  const FTransform& InitialTargetTM) {
    if (!SkeletalMeshComponent)
        return;

    // Get Body Instance for the bone
    FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneDrive.BoneName);
    if (!Body || !Body->IsValidBodyInstance() || Body->ActorHandle == nullptr)
        return;

    // --- Clean up any existing instance ---
    DestroyBoneDriveInstance(BoneDrive);

    // --- 1. Create Kinematic Target Actor ---
    FPhysicsActorHandle TargetActor;
    {
        FActorCreationParams Params;
        Params.bSimulatePhysics = false;
        Params.bQueryOnly = false;
        Params.Scene = Body->GetPhysicsScene();
        Params.InitialTM = InitialTargetTM;

        FPhysicsInterface::CreateActor(Params, TargetActor);

        // Chaos requires some geometry, so we use a tiny sphere
        auto Sphere = MakeUnique<Chaos::FImplicitSphere3>(FVector::ZeroVector, 0.01f);
        TargetActor->GetGameThreadAPI().SetGeometry(MoveTemp(Sphere));
        TargetActor->GetGameThreadAPI().SetUserData(nullptr);

        TArray<FPhysicsActorHandle> ActorHandles{TargetActor};
        Params.Scene->AddActorsToScene_AssumesLocked(ActorHandles, false);
    }
    BoneDrive.TargetActor = TargetActor;

    // --- 2. Create & Configure Constraint Instance ---
    FConstraintInstance* ConstraintInstance = new FConstraintInstance();

    // Physical Animation default: all free
    auto& Profile = ConstraintInstance->ProfileInstance;
    Profile.LinearLimit.XMotion = LCM_Free;
    Profile.LinearLimit.YMotion = LCM_Free;
    Profile.LinearLimit.ZMotion = LCM_Free;
    Profile.ConeLimit.Swing1Motion = ACM_Free;
    Profile.ConeLimit.Swing2Motion = ACM_Free;
    Profile.TwistLimit.TwistMotion = ACM_Free;

    Profile.LinearDrive.XDrive.bEnablePositionDrive = !DriveSettings.bIsLocalSimulation;
    Profile.LinearDrive.XDrive.bEnableVelocityDrive = !DriveSettings.bIsLocalSimulation;
    Profile.LinearDrive.YDrive.bEnablePositionDrive = !DriveSettings.bIsLocalSimulation;
    Profile.LinearDrive.YDrive.bEnableVelocityDrive = !DriveSettings.bIsLocalSimulation;
    Profile.LinearDrive.ZDrive.bEnablePositionDrive = !DriveSettings.bIsLocalSimulation;
    Profile.LinearDrive.ZDrive.bEnableVelocityDrive = !DriveSettings.bIsLocalSimulation;

    Profile.AngularDrive.SlerpDrive.bEnablePositionDrive = true;
    Profile.AngularDrive.SlerpDrive.bEnableVelocityDrive = true;
    Profile.AngularDrive.AngularDriveMode = EAngularDriveMode::SLERP;

    ConstraintInstance->SetRefFrame(EConstraintFrame::Frame1, FTransform::Identity);
    ConstraintInstance->SetRefFrame(EConstraintFrame::Frame2, FTransform::Identity);

    // --- 3. Init constraint (attach simulated bone to target actor) ---
    ConstraintInstance->InitConstraint_AssumesLocked(Body->ActorHandle, TargetActor, 1.f);

    BoneDrive.ConstraintInstance = ConstraintInstance;

    // --- 4. Wake up the body, just in case ---
    Body->WakeInstance();
}

void UOHPhysicsComponent::DestroyBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive) {
    if (BoneDrive.ConstraintInstance) {
        BoneDrive.ConstraintInstance->TermConstraint();
        delete BoneDrive.ConstraintInstance;
        BoneDrive.ConstraintInstance = nullptr;
    }

    if (BoneDrive.TargetActor) {
        if (FChaosScene* PhysScene = FChaosEngineInterface::GetCurrentScene(BoneDrive.TargetActor)) {
            FPhysInterface_Chaos::ReleaseActor(BoneDrive.TargetActor, PhysScene);
        }
        BoneDrive.TargetActor = nullptr;
    }
}

void UOHPhysicsComponent::UpdateBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive,
                                                  const FTransform& TargetTM,
                                                  const FPhysicalAnimationDriveSettings& DriveSettings,
                                                  float BlendAlpha) {
    // Defensive check
    if (!BoneDrive.ConstraintInstance || !BoneDrive.TargetActor)
        return;

    // -- 1. Move the kinematic target actor to the new animation pose --
    FPhysicsInterface::SetKinematicTarget_AssumesLocked(BoneDrive.TargetActor, TargetTM);

    // -- 2. Update drive strengths for blending --
    const float Blend = FMath::Clamp(BlendAlpha, 0.f, 1.f);
    const float OrientationStrength = DriveSettings.OrientationStrength * Blend;
    const float AngularVelocityStrength = DriveSettings.AngularVelocityStrength * Blend;
    const float MaxAngularForce = DriveSettings.MaxAngularForce * Blend;

    const float PositionStrength = DriveSettings.PositionStrength * Blend;
    const float VelocityStrength = DriveSettings.VelocityStrength * Blend;
    const float MaxLinearForce = DriveSettings.MaxLinearForce * Blend;

    // -- 3. Set drive params on constraint --
    BoneDrive.ConstraintInstance->SetAngularDriveParams(OrientationStrength, AngularVelocityStrength, MaxAngularForce);

    if (DriveSettings.bIsLocalSimulation) {
        // Only angular drive for local
        BoneDrive.ConstraintInstance->SetLinearDriveParams(0.f, 0.f, 0.f);
    } else {
        BoneDrive.ConstraintInstance->SetLinearDriveParams(PositionStrength, VelocityStrength, MaxLinearForce);
    }
}

bool UOHPhysicsComponent::IsChainStable(const FPhysicalAnimationChainRuntime& Chain) const {
    for (const FPhysicalAnimationBoneDriveInstance& BoneDrive : Chain.BoneDrives) {
        FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneDrive.BoneName);
        if (!Body)
            continue;
        const FVector LinVel = Body->GetUnrealWorldVelocity();
        const FVector AngVel = Body->GetUnrealWorldAngularVelocityInRadians();
        if (LinVel.Size() > StabilitySettings.MaxAllowedLinearVelocity)
            return false;
        if (AngVel.Size() > StabilitySettings.MaxAllowedAngularVelocity)
            return false;

        const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneDrive.BoneName);
        FTransform AnimTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
        const FVector PosErr = Body->GetUnrealWorldTransform().GetLocation() - AnimTM.GetLocation();
        if (PosErr.Size() > StabilitySettings.MaxAllowedDistanceFromTarget)
            return false;
    }
    return true;
}

void UOHPhysicsComponent::RecoverChain(FPhysicalAnimationChainRuntime& Chain) {
    for (FPhysicalAnimationBoneDriveInstance& BoneDrive : Chain.BoneDrives) {
        FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneDrive.BoneName);
        if (Body) {
            const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneDrive.BoneName);
            FTransform AnimTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
            Body->SetBodyTransform(AnimTM, ETeleportType::TeleportPhysics);
            Body->SetLinearVelocity(FVector::ZeroVector, false);
            Body->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        }
        DestroyBoneDriveInstance(BoneDrive);
    }
}

/*
TArray<int32> UOHPhysicsComponent::GetBoneIndicesInChain(FName StartBone, FName EndBone) const
{
        TArray<int32> Indices;
        if (!SkeletalMeshComponent) return Indices;

        const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();

        int32 StartIdx = RefSkeleton.FindBoneIndex(StartBone);
        int32 EndIdx = RefSkeleton.FindBoneIndex(EndBone);
        if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE) return Indices;

        // Traverse from start to end in skeleton hierarchy
        int32 CurrentIdx = StartIdx;
        while (CurrentIdx != INDEX_NONE)
        {
                Indices.Add(CurrentIdx);
                if (CurrentIdx == EndIdx)
                        break;
                CurrentIdx = RefSkeleton.GetChildBoneIndex(CurrentIdx, 0); // May need more logic for complex branches
        }

        return Indices;
}
*/

void UOHPhysicsComponent::EnablePhysicsOnChain(FName ChainName, bool bEnable) {
    if (!SkeletalMeshComponent)
        return;

    for (FPhysicalAnimationChainSettings& Chain : Chains) {
        if (Chain.ChainName == ChainName) {
            Chain.bEnabled = bEnable;

            // For every bone in this chain, enable or disable simulation
            TArray<int32> BoneIndices = GetBoneIndicesInChain(Chain.StartBone, Chain.EndBone);

            for (int32 BoneIdx : BoneIndices) {
                if (BoneIdx == INDEX_NONE)
                    continue;
                FName BoneName = SkeletalMeshComponent->GetBoneName(BoneIdx);

                FBodyInstance* Body = SkeletalMeshComponent->GetBodyInstance(BoneName);
                if (Body) {
                    if (bEnable) {
                        Body->SetInstanceSimulatePhysics(true);
                        Body->WakeInstance();
                    } else {
                        Body->SetInstanceSimulatePhysics(false);
                        // Optional: Snap back to animation pose instantly
                        int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
                        if (BoneIndex != INDEX_NONE) {
                            FTransform AnimTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
                            Body->SetBodyTransform(AnimTM, ETeleportType::TeleportPhysics);
                        }
                    }
                }
            }

            UE_LOG(LogTemp, Log, TEXT("[OHPhysicsComponent] Physics %s for chain '%s'"),
                   bEnable ? TEXT("ENABLED") : TEXT("DISABLED"), *ChainName.ToString());
            break;
        }
    }
}

void UOHPhysicsComponent::SetDriveProfileForChain(FName ChainName, UOHPhysicalAnimationProfileDataAsset* Profile) {
    if (!Profile)
        return;
    if (!SkeletalMeshComponent)
        return;

    for (FPhysicalAnimationChainSettings& Chain : Chains) {
        if (Chain.ChainName == ChainName) {
            Chain.DriveSettings = Profile->DriveSettings;

            // Apply to all bones in chain if needed
            // If you store runtime drive instances, update their settings here!

            UE_LOG(LogTemp, Log, TEXT("[OHPhysicsComponent] Applied profile '%s' to chain '%s'"),
                   *Profile->ProfileName.ToString(), *ChainName.ToString());
            break;
        }
    }
}

void UOHPhysicsComponent::SetDebugDrawEnabled(bool bEnable) {
    bDebugDraw = bEnable;
}

void UOHPhysicsComponent::DrawDebugInfo() {
    if (!SkeletalMeshComponent)
        return;
    UWorld* World = GetWorld();
    if (!World)
        return;

    // Color palette for different chains
    static const TArray<FColor> ChainColors = {FColor::Red,     FColor::Green,  FColor::Blue,   FColor::Cyan,
                                               FColor::Magenta, FColor::Yellow, FColor::Orange, FColor::Turquoise};

    int ColorIdx = 0;
    for (const auto& Chain : RuntimeChains) {
        const FColor ChainColor = ChainColors[ColorIdx++ % ChainColors.Num()];

        for (const auto& BoneDrive : Chain.BoneDrives) {
            const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneDrive.BoneName);
            if (BoneIndex == INDEX_NONE)
                continue;

            FTransform BoneTM = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
            FVector BoneLoc = BoneTM.GetLocation();

            // Draw driven bone position
            DrawDebugSphere(World, BoneLoc, 2.0f, 12, ChainColor, false, -1.f, 0, 1.5f);

            // If you have target kinematic actors, draw their positions and error lines
            if (BoneDrive.TargetActor) {
                // Note: This is the world location of your kinematic target actor (animation pose)
                FVector TargetLoc = BoneDrive.TargetActor->GetGameThreadAPI().X();

                DrawDebugSphere(World, TargetLoc, 2.5f, 12, FColor::White, false, -1.f, 0, 1.0f);
                DrawDebugLine(World, BoneLoc, TargetLoc, FColor::Purple, false, -1.f, 0, 0.5f);

                // Draw error magnitude text at mid-point
                FVector Mid = (BoneLoc + TargetLoc) * 0.5f;
                float Error = FVector::Dist(BoneLoc, TargetLoc);
                FString Info = FString::Printf(TEXT("%s | Blend: %.2f | Error: %.1f"),
                                               *Chain.Settings.ChainName.ToString(), BoneDrive.BlendAlpha, Error);
                DrawDebugString(World, Mid, Info, nullptr, FColor::White, 0.f, true);
            } else {
                // Show bone name if no target
                DrawDebugString(World, BoneLoc, BoneDrive.BoneName.ToString(), nullptr, ChainColor, 0.f, true);
            }
        }
    }
}

void UOHPhysicsComponent::PreviewChainsInEditor() {
#if WITH_EDITOR
    if (!SkeletalMeshComponent)
        return;
    UWorld* World = GetWorld();
    if (!World)
        return;

    static const TArray<FColor> ChainColors = {FColor::Red,     FColor::Green,  FColor::Blue,   FColor::Cyan,
                                               FColor::Magenta, FColor::Yellow, FColor::Orange, FColor::Turquoise};
    int ColorIdx = 0;

    for (const FPhysicalAnimationChainSettings& Chain : Chains) {
        const FColor ChainColor = ChainColors[ColorIdx++ % ChainColors.Num()];
        TArray<int32> BoneIndices = GetBoneIndicesInChain(Chain.StartBone, Chain.EndBone);

        FVector LastLoc = FVector::ZeroVector;
        for (int32 i = 0; i < BoneIndices.Num(); ++i) {
            int32 BoneIdx = BoneIndices[i];
            if (BoneIdx == INDEX_NONE)
                continue;

            FTransform BoneTM = SkeletalMeshComponent->GetBoneTransform(BoneIdx);
            FVector BoneLoc = BoneTM.GetLocation();

            DrawDebugSphere(World, BoneLoc, 2.0f, 12, ChainColor, false, 10.0f, 0, 1.0f);

            if (i > 0) {
                DrawDebugLine(World, LastLoc, BoneLoc, ChainColor, false, 10.0f, 0, 0.75f);
            }
            LastLoc = BoneLoc;
        }

        // Draw chain name at midpoint (if any bones in chain)
        if (BoneIndices.Num() >= 2) {
            FVector StartLoc = SkeletalMeshComponent->GetBoneTransform(BoneIndices[0]).GetLocation();
            FVector EndLoc = SkeletalMeshComponent->GetBoneTransform(BoneIndices.Last()).GetLocation();
            FVector Mid = (StartLoc + EndLoc) * 0.5f;
            DrawDebugString(World, Mid, Chain.ChainName.ToString(), nullptr, ChainColor, 10.0f, true);
        }
    }
#endif
}
void UOHPhysicsComponent::AutoPopulateChainsFromSkeleton() {
    if (!SkeletalMeshComponent)
        return;

    const FReferenceSkeleton& RefSkel = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();

    // Step 1: Collect all bone indices that have a physics body
    TSet<int32> BodyBoneIndices;
    for (const FBodyInstance* Body : SkeletalMeshComponent->Bodies) {
        if (Body && Body->IsValidBodyInstance()) {
            int32 BoneIdx = RefSkel.FindBoneIndex(Body->BodySetup->BoneName);
            if (BoneIdx != INDEX_NONE)
                BodyBoneIndices.Add(BoneIdx);
        }
    }

    Chains.Empty();

    // Step 2: For each root (bone with no parent with a body), walk to each leaf
    for (int32 BoneIdx : BodyBoneIndices) {
        int32 ParentIdx = RefSkel.GetParentIndex(BoneIdx);
        if (ParentIdx == INDEX_NONE || !BodyBoneIndices.Contains(ParentIdx)) {
            // Walk this chain
            TArray<int32> ChainIndices;
            int32 CurrentIdx = BoneIdx;
            ChainIndices.Add(CurrentIdx);

            // Step down hierarchy as far as possible
            while (true) {
                int32 ChildWithBody = INDEX_NONE;
                for (int32 ChildIdx = 0; ChildIdx < RefSkel.GetNum(); ++ChildIdx) {
                    if (RefSkel.GetParentIndex(ChildIdx) == CurrentIdx && BodyBoneIndices.Contains(ChildIdx)) {
                        ChildWithBody = ChildIdx;
                        break; // Only follow the first child (linear chains)
                    }
                }
                if (ChildWithBody != INDEX_NONE) {
                    CurrentIdx = ChildWithBody;
                    ChainIndices.Add(CurrentIdx);
                } else
                    break;
            }

            if (ChainIndices.Num() > 1) {
                // Step 3: Use start/end bones for chain, assign a name
                FName StartBone = RefSkel.GetBoneName(ChainIndices[0]);
                FName EndBone = RefSkel.GetBoneName(ChainIndices.Last());

                FString AutoName = FString::Printf(TEXT("Chain_%s_%s"), *StartBone.ToString(), *EndBone.ToString());

                // Optional: Try to detect limb type by name for default drive
                FPhysicalAnimationDriveSettings Defaults = GetDefaultDriveForChain(StartBone, EndBone);

                Chains.Add(FPhysicalAnimationChainSettings{FName(AutoName), StartBone, EndBone, Defaults, 1.f, true});
            }
        }
    }
}

void UOHPhysicsComponent::VisualizeChainsInEditor() {
#if WITH_EDITOR
    if (!SkeletalMeshComponent)
        return;

    const FReferenceSkeleton& RefSkel = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
    UWorld* World = GetWorld();
    if (!World)
        return;

    static const TArray<FColor> ChainColors = {FColor::Red,     FColor::Green,  FColor::Blue,   FColor::Cyan,
                                               FColor::Magenta, FColor::Yellow, FColor::Orange, FColor::Turquoise,
                                               FColor::Emerald, FColor::Purple, FColor::Silver, FColor::Black,
                                               FColor::White};

    int ColorIdx = 0;
    for (const FPhysicalAnimationChainSettings& Chain : Chains) {
        FColor ChainColor = ChainColors[ColorIdx++ % ChainColors.Num()];

        int32 StartIdx = RefSkel.FindBoneIndex(Chain.StartBone);
        int32 EndIdx = RefSkel.FindBoneIndex(Chain.EndBone);
        if (StartIdx == INDEX_NONE || EndIdx == INDEX_NONE)
            continue;

        // Walk the chain from start to end
        int32 CurrIdx = StartIdx;
        FVector LastLoc = SkeletalMeshComponent->GetBoneTransform(CurrIdx).GetLocation();

        // Always show chain name at midpoint
        FVector EndLoc = SkeletalMeshComponent->GetBoneTransform(EndIdx).GetLocation();
        FVector Mid = (LastLoc + EndLoc) * 0.5f;
        DrawDebugString(World, Mid, Chain.ChainName.ToString(), nullptr, ChainColor, 10.f, true);

        while (true) {
            int32 NextIdx = INDEX_NONE;
            for (int32 i = 0; i < RefSkel.GetNum(); ++i) {
                if (RefSkel.GetParentIndex(i) == CurrIdx) {
                    NextIdx = i;
                    break; // Only first child
                }
            }
            if (NextIdx == INDEX_NONE || NextIdx == EndIdx)
                break;

            FVector NextLoc = SkeletalMeshComponent->GetBoneTransform(NextIdx).GetLocation();
            DrawDebugDirectionalArrow(World, LastLoc, NextLoc, 10.0f, ChainColor, false, 10.f, 0, 2.f);
            DrawDebugSphere(World, LastLoc, 2.5f, 12, ChainColor, false, 10.f, 0, 0.75f);
            LastLoc = NextLoc;
            CurrIdx = NextIdx;
        }
        // Draw final segment to EndIdx
        if (CurrIdx != EndIdx) {
            FVector FinalLoc = SkeletalMeshComponent->GetBoneTransform(EndIdx).GetLocation();
            DrawDebugDirectionalArrow(World, LastLoc, FinalLoc, 10.0f, ChainColor, false, 10.f, 0, 2.f);
            DrawDebugSphere(World, FinalLoc, 3.f, 14, ChainColor, false, 10.f, 0, 1.0f);
        }
    }
#endif
}

FPhysicalAnimationDriveSettings UOHPhysicsComponent::GetDefaultDriveForChain(FName StartBone, FName EndBone) {
    // Example: set strong for spine, soft for arms/legs, etc. Customize as needed!
    FString Name = EndBone.ToString().ToLower();
    if (Name.Contains(TEXT("thigh")) || Name.Contains(TEXT("calf")) || Name.Contains(TEXT("foot")))
        return DefaultLegDrive;
    if (Name.Contains(TEXT("upperarm")) || Name.Contains(TEXT("forearm")) || Name.Contains(TEXT("hand")))
        return DefaultArmDrive;
    if (Name.Contains(TEXT("spine")) || Name.Contains(TEXT("pelvis")))
        return DefaultSpineDrive;
    if (Name.Contains(TEXT("head")) || Name.Contains(TEXT("neck")))
        return DefaultHeadDrive;
    // Fallback: default or stiff
    return DefaultGeneralDrive;
}
#if 0

#include "Component/OHPhysicsComponent.h"
#include "TimerManager.h"
#include "GameFramework/Actor.h"
#include "Interface/IOHPhysicsBehaviorStrategy.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"
#include "Kismet/KismetMathLibrary.h"
UOHPhysicsComponent::UOHPhysicsComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

// Current
void UOHPhysicsComponent::BeginPlay()
{
    Super::BeginPlay();
    bInitialized = false; // Defer
    GetWorld()->GetTimerManager().SetTimerForNextTick(this, &UOHPhysicsComponent::SafeInitializePhysics);
}

// Current
void UOHPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    SimulationTick(DeltaTime);
    if (bNeedsProxyRebuild)
    {
        ApplyAutoProxyFollowChains();
        bNeedsProxyRebuild = false;
    }

}

// ----------- Initialization  ----------- //
// Current
void UOHPhysicsComponent::InitializeMeshReferences()
{
    if (Mesh && PhysicalAnimation)
        return;

    Mesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
    PhysicalAnimation = GetOwner()->FindComponentByClass<UPhysicalAnimationComponent>();

    if (!Mesh || !PhysicalAnimation)
    {
        UE_LOG(LogTemp, Error, TEXT("OHPhysicsComponent: Mesh or PhysicalAnimationComponent missing on %s"), *GetOwner()->GetName());
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("OHPhysicsComponent: Mesh and PhysicalAnimationComponent references initialized."));
}

// Current
void UOHPhysicsComponent::InitializePhysicsComponent()
{
    InitializeMeshReferences();

    if (!Mesh || !PhysicalAnimation)
        return;

    PhysicalAnimation->SetSkeletalMeshComponent(Mesh);
    Mesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    bInitialized = true;

    UE_LOG(LogTemp, Log, TEXT("OHPhysicsComponent: Physics component initialized."));
}

// Current
void UOHPhysicsComponent::InitializeBoneTracking()
{
    if (!Mesh)
    {
        Mesh = GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
        if (!Mesh)
        {
            UE_LOG(LogTemp, Error, TEXT("[OHPhysicsComponent] No skeletal mesh found."));
            return;
        }
    }

    BoneContextMap.InitializeFromMesh(Mesh);
    MarkProxyChainsDirty();
    
    TMap<EOHSkeletalBone, FOHBoneState>& MutableStates = BoneContextMap.GetMutableBoneStates();

    for (const EOHSkeletalBone EnumBone : BoneContextMap.ReferenceMap.GetMappedBones())
    {
        FOHBoneState BoneState;
        BoneState.Role = EOHBoneSimulationRole::AnimationOnly;
        BoneState.SimulationState = EOHBoneSimulationState::Kinematic;
        const FBoneContainer* Container = GetBoneContainer();
        BoneState.ProxyFollowSourceBone = BoneContextMap.GetReferenceMap().GetParentBoneEnum(EnumBone, *Container);

        MutableStates.Add(EnumBone, BoneState);
    }

    bIsInitialized = true;

    UE_LOG(LogTemp, Log, TEXT("[OHPhysicsComponent] Bone tracking initialized (%d bones)."),
        BoneContextMap.GetBoneStates().Num());
}
// Current
void UOHPhysicsComponent::InitializeComponents()
{
    if (bInitialized) return;

    InitializeMeshReferences();
    InitializePhysicsComponent();
}

void UOHPhysicsComponent::SafeInitializePhysics()
{
    InitializeMeshReferences();
    InitializePhysicsComponent();

    BoneContextMap.InitializeFromMesh(GetSkelMesh());
    MarkProxyChainsDirty();
    InitializeBoneTracking();                             
    InitializeProxyChains();                               

    ApplyPreconfiguredProfiles();
    bInitialized = true;
}




// Current
void UOHPhysicsComponent::ApplyPreconfiguredProfiles()
{
    for (const FOHPreconfiguredBoneProfile& Entry : PreconfiguredBones)
    {
        FPhysicalAnimationData Resolved = ResolveProfileOrData(Entry.ProfileName, Entry.OverrideData);
        PhysicsProfileMap.SetBoneProfile(Entry.Bone, Resolved);
        MarkProxyChainsDirty();
    }

    for (const FOHPreconfiguredBodyPartProfile& Entry : PreconfiguredParts)
    {
        FPhysicalAnimationData Resolved = ResolveProfileOrData(Entry.ProfileName, Entry.OverrideData);
        PhysicsProfileMap.SetBodyPartProfile(Entry.BodyPart, Resolved);
    }

    for (const FOHPreconfiguredRegionProfile& Entry : PreconfiguredRegions)
    {
        FPhysicalAnimationData Resolved = ResolveProfileOrData(Entry.ProfileName, Entry.OverrideData);
        PhysicsProfileMap.SetRegionProfile(Entry.Region, Resolved);
        MarkProxyChainsDirty();
    }

    UE_LOG(LogTemp, Log, TEXT("OHPhysicsComponent: Preconfigured profiles applied to PhysicsProfileMap."));
}






void UOHPhysicsComponent::SetBoneProfile(EOHSkeletalBone Bone, const FPhysicalAnimationData& Data, bool bApplyImmediately)
{
    PhysicsProfileMap.SetBoneProfile(Bone, Data);
    MarkProxyChainsDirty();
    FOHBoneState& State = BoneStates.FindOrAdd(Bone);

    State.ActiveProfileName = NAME_None; // Assume direct data, not named
    State.ResolvedProfileData = Data;

    if (bApplyImmediately && bPhysicsActive)
    {
        // Set full sim intent; let Finalize() apply it
        State.Role = EOHBoneSimulationRole::Simulated;
        State.SimulationState = EOHBoneSimulationState::Simulating;
        State.PhysicsBlendWeight = 1.0f;
    }

    UE_LOG(LogTemp, Log, TEXT("Set physics profile for bone: %s (applyImmediately: %s)"),
        *GetBoneNameFromEnum(Bone).ToString(),
        bApplyImmediately ? TEXT("✓") : TEXT("✗"));
}



void UOHPhysicsComponent::RemoveBoneProfile(EOHSkeletalBone Bone, bool bRemoveFromSimulation)
{
    PhysicsProfileMap.RemoveBoneProfile(Bone);

    if (!BoneStates.Contains(Bone))
        return;

    FOHBoneState& State = BoneStates[Bone];
    State.ActiveProfileName = NAME_None;
    State.ResolvedProfileData = FPhysicalAnimationData{};

    if (bRemoveFromSimulation)
    {
        State.Role = EOHBoneSimulationRole::AnimationOnly;
        State.SimulationState = EOHBoneSimulationState::Kinematic;
        State.PhysicsBlendWeight = 0.0f;
    }

    UE_LOG(LogTemp, Log, TEXT("Removed profile from bone %s. RemoveFromSimulation: %s"),
        *UEnum::GetValueAsString(Bone),
        bRemoveFromSimulation ? TEXT("✓") : TEXT("✗"));
}

void UOHPhysicsComponent::SetBodyPartProfile(EOHBodyPart BodyPart, const FPhysicalAnimationData& Data, bool bApplyImmediately)
{
    PhysicsProfileMap.SetBodyPartProfile(BodyPart, Data);

    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::BP_GetBonesFromBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        FOHBoneState& State = BoneStates.FindOrAdd(Bone);
        State.ActiveProfileName = NAME_None;
        State.ResolvedProfileData = Data;

        if (bApplyImmediately && bPhysicsActive)
        {
            State.Role = EOHBoneSimulationRole::Simulated;
            State.SimulationState = EOHBoneSimulationState::Simulating;
            State.PhysicsBlendWeight = 1.0f;
        }

        UE_LOG(LogTemp, Log, TEXT("Set physics profile for bone: %s (body part: %s, applyImmediately: %s)"),
            *GetBoneNameFromEnum(Bone).ToString(),
            *UEnum::GetValueAsString(BodyPart),
            bApplyImmediately ? TEXT("✓") : TEXT("✗"));
    }
}

void UOHPhysicsComponent::RemoveBodyPartProfile(EOHBodyPart BodyPart, bool bRemoveFromSimulation, bool bForceRecursive)
{
    PhysicsProfileMap.RemoveBodyPartProfile(BodyPart);

    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::BP_GetBonesFromBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        // Skip if a more specific bone override exists, and recursion is not forced
        if (!bForceRecursive && PhysicsProfileMap.HasBoneProfile(Bone))
            continue;

        if (!BoneStates.Contains(Bone))
            continue;

        FOHBoneState& State = BoneStates[Bone];
        State.ActiveProfileName = NAME_None;
        State.ResolvedProfileData = FPhysicalAnimationData{};

        if (bRemoveFromSimulation)
        {
            State.Role = EOHBoneSimulationRole::AnimationOnly;
            State.SimulationState = EOHBoneSimulationState::Kinematic;
            State.PhysicsBlendWeight = 0.0f;
        }

        UE_LOG(LogTemp, Log, TEXT("Removed profile from bone: %s (body part: %s, forceRecursive: %s, removed from sim: %s)"),
            *GetBoneNameFromEnum(Bone).ToString(),
            *UEnum::GetValueAsString(BodyPart),
            bForceRecursive ? TEXT("✓") : TEXT("✗"),
            bRemoveFromSimulation ? TEXT("✓") : TEXT("✗"));
    }
}

void UOHPhysicsComponent::SetRegionProfile(EOHBodyRegion Region, const FPhysicalAnimationData& Data, bool bApplyImmediately)
{
    PhysicsProfileMap.SetRegionProfile(Region, Data);
    MarkProxyChainsDirty();

    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::GetBonesInRegion(Region);

    for (EOHSkeletalBone Bone : Bones)
    {
        FOHBoneState& State = BoneStates.FindOrAdd(Bone);
        State.ActiveProfileName = NAME_None;
        State.ResolvedProfileData = Data;

        if (bApplyImmediately && bPhysicsActive)
        {
            State.Role = EOHBoneSimulationRole::Simulated;
            State.SimulationState = EOHBoneSimulationState::Simulating;
            State.PhysicsBlendWeight = 1.0f;
        }

        UE_LOG(LogTemp, Log, TEXT("Set physics profile for bone: %s (region: %s, applyImmediately: %s)"),
            *GetBoneNameFromEnum(Bone).ToString(),
            *UEnum::GetValueAsString(Region),
            bApplyImmediately ? TEXT("✓") : TEXT("✗"));
    }
}
void UOHPhysicsComponent::RemoveRegionProfile(EOHBodyRegion Region, bool bRemoveFromSimulation, bool bForceRecursive)
{
    PhysicsProfileMap.RemoveRegionProfile(Region);

    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::GetBonesInRegion(Region);

    for (EOHSkeletalBone Bone : Bones)
    {
        // Respect override priority unless forced
        if (!bForceRecursive &&
            (PhysicsProfileMap.HasBoneProfile(Bone) ||
             PhysicsProfileMap.HasBodyPartProfile(GetBodyPartForBone(Bone))))
        {
            continue;
        }

        if (!BoneStates.Contains(Bone))
            continue;

        FOHBoneState& State = BoneStates[Bone];
        State.ActiveProfileName = NAME_None;
        State.ResolvedProfileData = FPhysicalAnimationData{};

        if (bRemoveFromSimulation)
        {
            State.Role = EOHBoneSimulationRole::AnimationOnly;
            State.SimulationState = EOHBoneSimulationState::Kinematic;
            State.PhysicsBlendWeight = 0.0f;
        }

        UE_LOG(LogTemp, Log, TEXT("Removed profile from bone: %s (region: %s, forceRecursive: %s, removed from sim: %s)"),
            *GetBoneNameFromEnum(Bone).ToString(),
            *UEnum::GetValueAsString(Region),
            bForceRecursive ? TEXT("✓") : TEXT("✗"),
            bRemoveFromSimulation ? TEXT("✓") : TEXT("✗"));
    }
}

FPhysicalAnimationData UOHPhysicsComponent::ResolveProfileOrData(FName ProfileName, const FPhysicalAnimationData& FallbackData) const
{
    if (const FPhysicalAnimationData* Found = NamedProfileMap.Find(ProfileName))
    {
        return *Found;
    }
    return FallbackData;
}

void UOHPhysicsComponent::SetBoneProfileByName(EOHSkeletalBone Bone, FName ProfileName, bool bApplyImmediately)
{
    FPhysicalAnimationData Data = ResolveProfileOrData(ProfileName, FPhysicalAnimationData{});
    SetBoneProfile(Bone, Data, bApplyImmediately);
}

void UOHPhysicsComponent::SetBodyPartProfileByName(EOHBodyPart Part, FName ProfileName)
{
    FPhysicalAnimationData Data = ResolveProfileOrData(ProfileName, FPhysicalAnimationData{});
    SetBodyPartProfile(Part, Data);
}

void UOHPhysicsComponent::SetRegionProfileByName(EOHBodyRegion Region, FName ProfileName)
{
    FPhysicalAnimationData Data = ResolveProfileOrData(ProfileName, FPhysicalAnimationData{});
    SetRegionProfile(Region, Data);
    MarkProxyChainsDirty();
}



TArray<FName> UOHPhysicsComponent::GetAvailableProfileNames() const
{
    TArray<FName> Keys;
    NamedProfileMap.GetKeys(Keys);
    return Keys;
}


// ---------------- Update Functions ---------------- //



// Current
void UOHPhysicsComponent::UpdateBlendOperations(float DeltaTime)
{
	if (!Mesh || !PhysicalAnimation) return;

	const double CurrentTime = FPlatformTime::Seconds();

	for (int32 i = ActiveBlendOperations.Num() - 1; i >= 0; --i)
	{
		FBlendOperation& Op = ActiveBlendOperations[i];
		const EOHSkeletalBone BoneEnum = Op.Bone;

		FOHBoneState* State = BoneStates.Find(BoneEnum);
		if (!State || !UOHSkeletalPhysicsUtils::ShouldHaveBlendState(BoneEnum)) continue;

		const float ElapsedTime = CurrentTime - Op.StartTime;
		const float BlendAlpha = FMath::Clamp(Op.CalculateCurrentWeight(CurrentTime), 0.0f, 1.0f);
		const float StrengthMultiplier = Op.CalculateStrengthMultiplier(CurrentTime);
		const bool bFadingOut = FMath::IsNearlyZero(Op.TargetWeight);

		// --- Stage 1: Runtime Blend Progress
		State->ProxyBlendAlpha = BlendAlpha;
		State->PhysicsBlendWeight = BlendAlpha;
		State->BlendType = Op.BlendType;

		// --- Stage 2: Runtime State Resolution
		if (BlendAlpha >= 1.f - KINDA_SMALL_NUMBER)
		{
			State->SimulationState = EOHBoneSimulationState::Simulating;
			State->Role = EOHBoneSimulationRole::Simulated;
		}
		else if (BlendAlpha <= KINDA_SMALL_NUMBER)
		{
			State->SimulationState = EOHBoneSimulationState::Kinematic;
			State->Role = EOHBoneSimulationRole::AnimationOnly;
		}
		else
		{
			State->SimulationState = EOHBoneSimulationState::Blending;
			State->Role = EOHBoneSimulationRole::Simulated;
		}

		// --- Stage 3: Resolve Force Profile
		FPhysicalAnimationData BaseProfile = PhysicsProfileMap.GetResolvedProfile(BoneEnum);
		const float ForceScale = (StrengthMultiplier >= 0.f) ? StrengthMultiplier : 1.0f;
		BaseProfile.OrientationStrength *= BlendAlpha * ForceScale;
		BaseProfile.PositionStrength *= BlendAlpha * ForceScale;
		BaseProfile.VelocityStrength *= BlendAlpha * ForceScale;
		BaseProfile.AngularVelocityStrength *= BlendAlpha * ForceScale;
		BaseProfile.MaxLinearForce *= BlendAlpha * ForceScale;
		BaseProfile.MaxAngularForce *= BlendAlpha * ForceScale;

		State->ResolvedProfileData = BaseProfile;

#if !UE_BUILD_SHIPPING
		if (bPrintBoneDebug && ActiveBehaviorStrategy.IsValid())
		{
			PrintBoneDebug(BoneEnum, *State, Op, ActiveBehaviorStrategy.Get(), ElapsedTime);
		}
#endif

		// --- Stage 4: Finalization
		if (Op.IsComplete(CurrentTime))
		{
		 
			State->BlendType = EOHBlendType::None;

			if (Op.bDeactivateOnComplete || Op.bRemoveProfileAfter)
			{
				State->ActiveProfileName = NAME_None;
				PhysicsProfileMap.RemoveBoneProfile(BoneEnum);
			}

			ActiveBlendOperations.RemoveAt(i);
		    MarkProxyChainsDirty();
		}
		else
		{
			Op.UpdateState(CurrentTime);
		}
	}
}

void UOHPhysicsComponent::TickProxyFollow(float DeltaTime)
{
    if (!Mesh || !BoneContextMap.IsInitialized())
        return;

    const TMap<EOHSkeletalBone, FOHBoneState>& States = BoneContextMap.GetBoneStates();
    for (const TPair<EOHSkeletalBone, FOHBoneState>& Pair : States)
    {
        const EOHSkeletalBone Bone = Pair.Key;
        const FOHBoneState& State = Pair.Value;

        if (State.ProxyFollowSourceBone == EOHSkeletalBone::None)
            continue;

        const FTransform Target = BoneContextMap.GetProxySourceTransform(Bone);
        const FTransform Current = BoneContextMap.GetBoneTransform(Bone);
        const float Alpha = FMath::Clamp(State.ProxyBlendAlpha, 0.f, 1.f);

        const FTransform Blended = UKismetMathLibrary::TLerp(Current, Target, Alpha);

        if (FOHBoneState* MutableState = BoneContextMap.FindState(Bone))
        {
            MutableState->PreviousWorldTransform = MutableState->CurrentWorldTransform;
            MutableState->CurrentWorldTransform = Blended;
        }
    }
}
#if 0
void UOHPhysicsComponent::UpdateBlendOperations(float DeltaTime)
{
    if (!Mesh || !PhysicalAnimation) return;

    const double CurrentTime = FPlatformTime::Seconds();

    for (int32 i = ActiveBlendOperations.Num() - 1; i >= 0; --i)
    {
        FBlendOperation& Op = ActiveBlendOperations[i];
        const EOHSkeletalBone BoneEnum = Op.BoneEnum;

        // --- Tracking Validity Check ---
        if (!BoneStates.Contains(BoneEnum)) continue;
        FOHBoneState& State = BoneStates[BoneEnum];

        // --- Physics Simulation Gating ---
        if (!UOHSkeletalPhysicsUtils::ShouldHaveBlendState(BoneEnum))
        {
            continue; // BoneEnum is tracked but not simulation-eligible
        }

        const float ElapsedTime = CurrentTime - Op.StartTime;
        const float CurrentWeight = Op.CalculateCurrentWeight(CurrentTime);
        const float StrengthMultiplier = Op.CalculateStrengthMultiplier(CurrentTime);

        // --- Stage 1: Visual Pre-Blend (Proxy)
        bool bEnablePAC = false;

        if (Op.BlendType == EOHBlendType::FadeIn)
        {
            if (ElapsedTime >= PhysicsHandoffDelay)
            {
                bEnablePAC = true;
            }
        }
        else if (Op.BlendType == EOHBlendType::FadeOut)
        {
            bEnablePAC = true; // stays active during fade out
        }

        // --- Stage 2: Physics Blend Activation
        State.PhysicsBlendWeight = bEnablePAC ? 1.0f : 0.0f;

        // --- Stage 3: Resolve force profile + apply strength multiplier
        if (StrengthMultiplier >= 0.f)
        {
            FPhysicalAnimationData Profile = PhysicsProfileMap.GetResolvedProfile(BoneEnum);

            Profile.OrientationStrength *= StrengthMultiplier;
            Profile.PositionStrength *= StrengthMultiplier;
            Profile.VelocityStrength *= StrengthMultiplier;
            Profile.AngularVelocityStrength *= StrengthMultiplier;
            Profile.MaxLinearForce *= StrengthMultiplier;
            Profile.MaxAngularForce *= StrengthMultiplier;

            State.ResolvedProfileData = Profile;

#if !UE_BUILD_SHIPPING
            if (Profile.OrientationStrength <= 0.f)
            {
                UE_LOG(LogTemp, Warning, TEXT("UpdateBlendOperations: Resolved profile for bone %s has zero strength after scaling — PAC drive may not be applied."),
                    *UEnum::GetValueAsString(BoneEnum));
            }
#endif
        }

        // --- Stage 3.5: Debug display
        if (bPrintBoneDebug && ActiveBehaviorStrategy.IsValid())
        {
            PrintBoneDebug(BoneEnum, State, Op, ActiveBehaviorStrategy.Get(), ElapsedTime);
        }

        // --- Stage 4: Finalization and Cleanup
        if (Op.IsComplete(CurrentTime))
        {
            const bool bFadingOut = FMath::IsNearlyZero(Op.TargetWeight);

            State.SimulationState = bFadingOut
                ? EOHBoneSimulationState::Kinematic
                : EOHBoneSimulationState::Simulating;

            State.Role = bFadingOut
                ? EOHBoneSimulationRole::AnimationOnly
                : EOHBoneSimulationRole::Simulated;

            State.BlendType = EOHBlendType::None;

            if (Op.bDeactivateOnComplete || Op.bRemoveProfileAfter)
            {
                State.ActiveProfileName = NAME_None;
                PhysicsProfileMap.RemoveBoneProfile(BoneEnum);
            }

            ActiveBlendOperations.RemoveAt(i);
        }
        else
        {
            Op.UpdateState(CurrentTime);
        }
    }
}
#endif



// Current
void UOHPhysicsComponent::UpdateBoneTracking(float DeltaTime)
{
    if (!Mesh) return;

    const FOHBoneState* PelvisState = BoneStates.Find(EOHSkeletalBone::Pelvis);
    const FTransform PelvisDelta = (PelvisState)
        ? PelvisState->CurrentWorldTransform.GetRelativeTransform(PelvisState->PreviousWorldTransform)
        : FTransform::Identity;

    for (auto& Pair : BoneStates)
    {
        EOHSkeletalBone BoneEnum = Pair.Key;
        FOHBoneState& State = Pair.Value;

        const FName BoneName = GetBoneNameFromEnum(BoneEnum);

        // Step 1: Update previous and current world transform
        State.PreviousWorldTransform = State.CurrentWorldTransform;
        State.CurrentWorldTransform = Mesh->GetSocketTransform(BoneName, ERelativeTransformSpace::RTS_World);

        // Step 2: Motion correction-aware velocity/acceleration tracking
        const FTransform Correction = ResolveMotionCorrectionTransform(
            State.bOverrideMotionCorrectionSource ? State.MotionCorrectionSource : DefaultMotionCorrection,
            State.BoneEnum);

        State.UpdateMotionState(DeltaTime, Correction);

        // Step 3: Proxy Follow Blending — blend to animation pose if enabled
        if (State.ProxyFollowSourceBone != EOHSkeletalBone::None && State.ProxyBlendAlpha > 0.f)
        {
            const FName SourceBoneName = GetBoneNameFromEnum(State.ProxyFollowSourceBone);

            if (Mesh->DoesSocketExist(SourceBoneName))
            {
                const FTransform AnimPose = Mesh->GetSocketTransform(SourceBoneName, ERelativeTransformSpace::RTS_World);
                State.CurrentWorldTransform.Blend(State.CurrentWorldTransform, AnimPose, State.ProxyBlendAlpha);
            }
        }
    }
}

void UOHPhysicsComponent::EnableProxyFollowForBodyPart(EOHBodyPart BodyPart, float BlendAlpha)
{
    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::GetBonesInBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        if (!BoneStates.Contains(Bone)) continue;

        FOHBoneState& State = BoneStates[Bone];
        State.ProxyFollowSourceBone = Bone; // self-follow mode
        State.ProxyBlendAlpha = FMath::Clamp(BlendAlpha, 0.0f, 1.0f);
    }

    UE_LOG(LogTemp, Log, TEXT("Proxy Follow ENABLED for %s with BlendAlpha %.2f"),
        *UEnum::GetValueAsString(BodyPart), BlendAlpha);
}

void UOHPhysicsComponent::DisableProxyFollowForBodyPart(EOHBodyPart BodyPart)
{
    const TArray<EOHSkeletalBone> Bones = UOHSkeletalPhysicsUtils::GetBonesInBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        if (!BoneStates.Contains(Bone)) continue;

        FOHBoneState& State = BoneStates[Bone];
        State.ProxyFollowSourceBone = EOHSkeletalBone::None;
        State.ProxyBlendAlpha = 0.f;
    }

    UE_LOG(LogTemp, Log, TEXT("Proxy Follow DISABLED for %s"),
        *UEnum::GetValueAsString(BodyPart));
}


// --------------- Start Blend Operations --------------- //

// Current
void UOHPhysicsComponent::StartBlendOperationsForBones(
    const TArray<EOHSkeletalBone>& Bones,
    EOHBlendType BlendType,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren // <-- NEW
)
{
    const double CurrentTime = FPlatformTime::Seconds();

    for (EOHSkeletalBone Bone : Bones)
    {
        // Apply profile if provided
        if (!OptionalProfileName.IsNone())
        {
            const FPhysicalAnimationData Resolved = ResolveProfileOrData(OptionalProfileName, FPhysicalAnimationData{});
            PhysicsProfileMap.SetBoneProfile(Bone, Resolved);
            MarkProxyChainsDirty();
        }

        // Initial weight depends on blend direction
        const float CurrentWeight = (BlendType == EOHBlendType::FadeIn) ? 0.0f : 1.0f;

        FBlendOperation Op = (BlendType == EOHBlendType::FadeIn)
            ? FBlendOperation::CreateFadeIn(Bone, Duration, CurrentWeight, OptionalProfileName)
            : FBlendOperation::CreateFadeOut(Bone, Duration, CurrentWeight, OptionalProfileName);

        Op.StartTime = CurrentTime;
        Op.bDeactivateOnComplete = bRemoveProfileAfter;

        // 💡 NEW: Propagation logic
        if (BlendType == EOHBlendType::FadeIn && bPropagateToChildren)
        {
            Op.bPropagateAttachedRoleToChildren = true;
        }

        StartBlendOperation(Op);
    }

    UE_LOG(LogTemp, Log, TEXT("StartBlendOperationsForBones: %s %d bones over %.2fs — Profile: %s, Propagate: %s"),
        *UEnum::GetValueAsString(BlendType),
        Bones.Num(),
        Duration,
        *OptionalProfileName.ToString(),
        bPropagateToChildren ? TEXT("✓") : TEXT("✗"));
}

// Current
void UOHPhysicsComponent::StartBlendOperationsForBodyPart(
    EOHBodyPart BodyPart,
    EOHBlendType BlendType,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren // <-- NEW
)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);
    StartBlendOperationsForBones(Bones, BlendType, Duration, OptionalProfileName, bRemoveProfileAfter, bPropagateToChildren);
}

// Current
void UOHPhysicsComponent::StartBlendOperationsForRegion(
    EOHBodyRegion Region,
    EOHBlendType BlendType,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren // <-- NEW
)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);
    StartBlendOperationsForBones(Bones, BlendType, Duration, OptionalProfileName, bRemoveProfileAfter, bPropagateToChildren);
}

// ---------------- Cancel Blend Operations  ---------------- //

// Current
void UOHPhysicsComponent::CancelBlendOperationsForBones(const TArray<EOHSkeletalBone>& Bones)
{
    for (EOHSkeletalBone Bone : Bones)
    {
        ActiveBlendOperations.RemoveAll([&](const FBlendOperation& Op) {
            return Op.Bone == Bone;
        });
    }

    UE_LOG(LogTemp, Log, TEXT("Cancelled %d bone blend operations."), Bones.Num());
}

// Current
void UOHPhysicsComponent::CancelBlendOperationsForBodyPart(EOHBodyPart BodyPart)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);
    CancelBlendOperationsForBones(Bones);
}

// Current
void UOHPhysicsComponent::CancelBlendOperationsForRegion(EOHBodyRegion Region)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);
    CancelBlendOperationsForBones(Bones);
}


// ------------------ Schedule Blend Operations  ------------------ //

// Current
void UOHPhysicsComponent::ScheduleBlendOperationsForBones(
    const TArray<EOHSkeletalBone>& Bones,
    EOHBlendType BlendType,
    float DelaySeconds,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren
)
{
    for (EOHSkeletalBone Bone : Bones)
    {
        FTimerHandle& Handle = ScheduledBlendTimers.FindOrAdd(Bone);

        GetWorld()->GetTimerManager().SetTimer(
            Handle,
            FTimerDelegate::CreateLambda([=]()
            {
                const FOHBoneState* State = BoneStates.Find(Bone);
                if (!State) return;

                const float CurrentWeight = State->PhysicsBlendWeight;
                const float SafeDuration = FMath::Max(Duration, 0.01f);

                FBlendOperation Op = (BlendType == EOHBlendType::FadeIn)
                    ? FBlendOperation::CreateFadeIn(Bone, SafeDuration, CurrentWeight, OptionalProfileName)
                    : FBlendOperation::CreateFadeOut(Bone, SafeDuration, CurrentWeight, OptionalProfileName);

                Op.bDeactivateOnComplete = (BlendType == EOHBlendType::FadeOut);
                Op.bRemoveProfileAfter = bRemoveProfileAfter;
                Op.bPropagateAttachedRoleToChildren = bPropagateToChildren;

                ClearActiveBlendOperation(Bone); // Clear any pre-existing blend before applying new one

                StartBlendOperation(Op);
                ScheduledBlendTimers.Remove(Bone);
            }),
            DelaySeconds, false
        );
    }
}

// Current
void UOHPhysicsComponent::ScheduleBlendOperationsForBodyPart(
    EOHBodyPart BodyPart,
    EOHBlendType BlendType,
    float DelaySeconds,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren
)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);
    ScheduleBlendOperationsForBones(Bones, BlendType, DelaySeconds, Duration, OptionalProfileName, bRemoveProfileAfter, bPropagateToChildren);
}

// Current
void UOHPhysicsComponent::ScheduleBlendOperationsForRegion(
    EOHBodyRegion Region,
    EOHBlendType BlendType,
    float DelaySeconds,
    float Duration,
    FName OptionalProfileName,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren
)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);
    ScheduleBlendOperationsForBones(Bones, BlendType, DelaySeconds, Duration, OptionalProfileName, bRemoveProfileAfter, bPropagateToChildren);
}

// ------------------- Cancel Scheduled Blend Operations  ------------------- //

// Current
void UOHPhysicsComponent::CancelScheduledBlendOperationsForBones(const TArray<EOHSkeletalBone>& Bones)
{
    for (EOHSkeletalBone Bone : Bones)
    {
        if (ScheduledBlendTimers.Contains(Bone))
        {
            FTimerHandle& Timer = ScheduledBlendTimers[Bone];
            if (GetWorld())
            {
                GetWorld()->GetTimerManager().ClearTimer(Timer);
            }
            ScheduledBlendTimers.Remove(Bone);
        }
    }

    UE_LOG(LogTemp, Log, TEXT("Cancelled scheduled blend operations for %d bones."), Bones.Num());
}

// Current
void UOHPhysicsComponent::CancelScheduledBlendOperationsForBodyPart(EOHBodyPart BodyPart)
{
    CancelScheduledBlendOperationsForBones(GetBonesInBodyPart(BodyPart));
}

// Current
void UOHPhysicsComponent::CancelScheduledBlendOperationsForRegion(EOHBodyRegion Region)
{
    CancelScheduledBlendOperationsForBones(GetBonesInBodyRegion(Region));
}


// ---------------------------- Schedule Timed Blend Operation ------------------------- //

void UOHPhysicsComponent::ScheduleBoneForTimedFade(
    EOHSkeletalBone Bone,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    const float SafeFadeIn = FMath::Max(FadeInTime, 0.01f);
    const float SafeFadeOut = FMath::Max(FadeOutTime, 0.01f);

    // Schedule Fade In
    ScheduleBlendOperationsForBones({ Bone }, EOHBlendType::FadeIn, 0.0f, SafeFadeIn, OptionalProfileName, false, true);

    // Schedule Fade Out after FadeIn + Hold
    ScheduleBlendOperationsForBones({ Bone }, EOHBlendType::FadeOut, SafeFadeIn + HoldTime, SafeFadeOut, OptionalProfileName, bRemoveProfileAfter, true);
}


void UOHPhysicsComponent::ScheduleBodyPartForTimedFade(
    EOHBodyPart BodyPart,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);
    const float SafeFadeIn = FMath::Max(FadeInTime, 0.01f);
    const float SafeFadeOut = FMath::Max(FadeOutTime, 0.01f);

    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeIn, 0.0f, SafeFadeIn, OptionalProfileName, false, true);
    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeOut, SafeFadeIn + HoldTime, SafeFadeOut, OptionalProfileName, bRemoveProfileAfter, true);
}


void UOHPhysicsComponent::ScheduleRegionForTimedFade(
    EOHBodyRegion Region,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);
    const float SafeFadeIn = FMath::Max(FadeInTime, 0.01f);
    const float SafeFadeOut = FMath::Max(FadeOutTime, 0.01f);

    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeIn, 0.0f, SafeFadeIn, OptionalProfileName, false, true);
    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeOut, SafeFadeIn + HoldTime, SafeFadeOut, OptionalProfileName, bRemoveProfileAfter, true);
}


void UOHPhysicsComponent::ScheduleBonesForTimedFade(
    const TArray<EOHSkeletalBone>& Bones,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    const float SafeFadeIn = FMath::Max(FadeInTime, 0.01f);
    const float SafeFadeOut = FMath::Max(FadeOutTime, 0.01f);

    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeIn, 0.0f, SafeFadeIn, OptionalProfileName, false, true);
    ScheduleBlendOperationsForBones(Bones, EOHBlendType::FadeOut, SafeFadeIn + HoldTime, SafeFadeOut, OptionalProfileName, bRemoveProfileAfter, true);
}


void UOHPhysicsComponent::ScheduleBodyPartsForTimedFade(
    const TArray<EOHBodyPart>& BodyParts,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    for (EOHBodyPart Part : BodyParts)
    {
        ScheduleBodyPartForTimedFade(Part, FadeInTime, HoldTime, FadeOutTime, OptionalProfileName, bRemoveProfileAfter);
    }
}

void UOHPhysicsComponent::ScheduleRegionsForTimedFade(
    const TArray<EOHBodyRegion>& Regions,
    float FadeInTime,
    float HoldTime,
    float FadeOutTime,
    FName OptionalProfileName,
    bool bRemoveProfileAfter)
{
    for (EOHBodyRegion Region : Regions)
    {
        ScheduleRegionForTimedFade(Region, FadeInTime, HoldTime, FadeOutTime, OptionalProfileName, bRemoveProfileAfter);
    }
}


// ------------------ Blend Operations ------------------ //

// Current
void UOHPhysicsComponent::ScheduleBlendOut(EOHSkeletalBone Bone, float DelaySeconds, float BlendOutDuration, bool bRemoveProfile)
{
    const FOHBoneState* State = BoneStates.Find(Bone);
    const float CurrentWeight = State ? State->PhysicsBlendWeight : 1.0f;

    FBlendOperation Op = FBlendOperation::CreateFadeOut(Bone, BlendOutDuration, CurrentWeight);
    Op.bDeactivateOnComplete = bRemoveProfile;

    FTimerHandle Handle;
    GetWorld()->GetTimerManager().SetTimer(Handle, [this, Op]()
    {
        ClearActiveBlendOperation(Op.Bone); // Defensive clear before starting a delayed operation
        this->StartBlendOperation(Op);
    }, DelaySeconds, false);
}

// Current
void UOHPhysicsComponent::ClearScheduledBlend(EOHSkeletalBone Bone)
{
    if (ScheduledBlendTimers.Contains(Bone))
    {
        FTimerHandle& Timer = ScheduledBlendTimers[Bone];
        if (GetWorld())
        {
            GetWorld()->GetTimerManager().ClearTimer(Timer);
        }
        ScheduledBlendTimers.Remove(Bone);
    }
}

// Current
void UOHPhysicsComponent::ClearActiveBlendOperation(EOHSkeletalBone Bone)
{
    ActiveBlendOperations.RemoveAll([&](const FBlendOperation& Op) {
        return Op.Bone == Bone;
    });
}

// Current
bool UOHPhysicsComponent::HasActiveBlendForBone(EOHSkeletalBone Bone) const {
    return ActiveBlendOperations.ContainsByPredicate([&](const FBlendOperation& Op) {
        return Op.Bone == Bone;
    });
}

const FBlendOperation* UOHPhysicsComponent::FindActiveBlendForBone(EOHSkeletalBone Bone) const
{
    return ActiveBlendOperations.FindByPredicate([&](const FBlendOperation& Op)
    {
        return Op.Bone == Bone;
    });
}

// Current
void UOHPhysicsComponent::SyncBoneStateFromBlendOperation(const FBlendOperation& Op)
{
    FOHBoneState& State = BoneStates.FindOrAdd(Op.Bone);

    // --- Role & State ---
    State.Role = (Op.TargetWeight > 0.01f)
        ? EOHBoneSimulationRole::Simulated
        : EOHBoneSimulationRole::AnimationOnly;

    const float ClampedWeight = FMath::Clamp(Op.StartWeight, 0.f, 1.f);
    State.PhysicsBlendWeight = ClampedWeight;
    State.ProxyBlendAlpha = ClampedWeight;
    State.BlendType = Op.BlendType;

    if (ClampedWeight >= 1.f - KINDA_SMALL_NUMBER)
    {
        State.SimulationState = EOHBoneSimulationState::Simulating;
    }
    else if (ClampedWeight <= KINDA_SMALL_NUMBER)
    {
        State.SimulationState = EOHBoneSimulationState::Kinematic;
    }
    else
    {
        State.SimulationState = EOHBoneSimulationState::Blending;
    }

    // --- Optional profile reference
    if (!Op.ProfileNameOverride.IsNone())
    {
        State.ActiveProfileName = Op.ProfileNameOverride;
    }

    // --- Optional proxy tracking
    if (Op.ProxyFollowSourceBone != EOHSkeletalBone::None)
    {
        State.ProxyFollowSourceBone = Op.ProxyFollowSourceBone;
    }

    // --- Cache resolved profile
    State.ResolvedProfileData = PhysicsProfileMap.GetResolvedProfile(Op.Bone);

    UE_LOG(LogTemp, Verbose, TEXT("Synced BoneState for %s → SimState=%s, BlendType=%s, Weight=%.2f"),
        *GetBoneNameFromEnum(Op.Bone).ToString(),
        *UEnum::GetValueAsString(State.SimulationState),
        *UEnum::GetValueAsString(State.BlendType),
        ClampedWeight);

#if WITH_EDITOR
    ValidateBoneState(Op.Bone);
#endif
}



// -------------------- Validate BoneEnum State  -------------------- //

// Current
void UOHPhysicsComponent::ValidateBoneStateIntegrity()
{
    if (!Mesh) return;

    for (auto& Pair : BoneStates)
    {
        const EOHSkeletalBone BoneEnum = Pair.Key;
        FOHBoneState& State = Pair.Value;
        const FName BoneName = GetBoneNameFromEnum(BoneEnum);

        bool bStateChanged = false;

        // Sanity check: blending with no direction
        if (State.SimulationState == EOHBoneSimulationState::Blending && State.BlendType == EOHBlendType::None)
        {
            UE_LOG(LogTemp, Warning, TEXT("ValidateBoneState: %s is 'Blending' but has BlendType 'None'. Fixing to FadeOut."),
                *BoneName.ToString());
            State.BlendType = EOHBlendType::FadeOut;
            bStateChanged = true;
        }

        // Simulating with no profile name set
        if (State.Role == EOHBoneSimulationRole::Simulated && State.ActiveProfileName.IsNone())
        {
            UE_LOG(LogTemp, Warning, TEXT("ValidateBoneState: %s is Simulated but has no ActiveProfileName."),
                *BoneName.ToString());

            // Try to find one from PhysicsProfileMap
            if (HasValidProfile(BoneEnum))
            {
                State.ActiveProfileName = NAME_None; // intentionally empty, but logged
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("ValidateBoneState: %s has no associated profile! Consider fallback or removal."),
                    *BoneName.ToString());
            }

            bStateChanged = true;
        }

        // Blending state weight out of bounds
        if (IsBlendWeightInvalid(State.PhysicsBlendWeight))
        {
            State.PhysicsBlendWeight = FMath::Clamp(State.PhysicsBlendWeight, 0.0f, 1.0f);
            UE_LOG(LogTemp, Warning, TEXT("ValidateBoneState: %s had out-of-bounds blend weight. Clamped to %.2f"),
                *BoneName.ToString(), State.PhysicsBlendWeight);
            bStateChanged = true;
        }

        if (bStateChanged)
        {
            UE_LOG(LogTemp, Log, TEXT("ValidateBoneState: Corrected inconsistencies for bone %s"), *BoneName.ToString());
        }
    }

    UE_LOG(LogTemp, Log, TEXT("ValidateBoneStateIntegrity: Completed validation of %d bones."), BoneStates.Num());
}

// Current
void UOHPhysicsComponent::ValidateBoneState(EOHSkeletalBone Bone) const
{
    const FName BoneName = GetBoneNameFromEnum(Bone);

    const FOHBoneState* State = BoneStates.Find(Bone);
    const FBodyInstance* Body = Mesh ? Mesh->GetBodyInstance(BoneName) : nullptr;

    if (!State)
    {
        UE_LOG(LogTemp, Warning, TEXT("[VALIDATION] BoneState missing for %s"), *BoneName.ToString());
        return;
    }

    // Check physical simulation match
    bool bIsSimulating = Body && Body->IsInstanceSimulatingPhysics();

    switch (State->SimulationState)
    {
    case EOHBoneSimulationState::Simulating:
        if (!bIsSimulating)
        {
            UE_LOG(LogTemp, Error, TEXT("[VALIDATION] %s: State=Simulating but physics is OFF"), *BoneName.ToString());
        }
        break;

    case EOHBoneSimulationState::Kinematic:
        if (bIsSimulating)
        {
            UE_LOG(LogTemp, Error, TEXT("[VALIDATION] %s: State=Kinematic but physics is ON"), *BoneName.ToString());
        }
        break;

    case EOHBoneSimulationState::Blending:
        // Acceptable during transitions
        break;

    default:
        UE_LOG(LogTemp, Warning, TEXT("[VALIDATION] %s: Unknown SimulationState."), *BoneName.ToString());
    }

    // Check if blend weight is in expected range
        if (IsBlendWeightInvalid(State->PhysicsBlendWeight))
    {
        UE_LOG(LogTemp, Error, TEXT("[VALIDATION] %s: Invalid blend weight %.2f"), *BoneName.ToString(), State->PhysicsBlendWeight);
    }
}

// Current
bool UOHPhysicsComponent::HasValidProfile(EOHSkeletalBone Bone) const
{
    return PhysicsProfileMap.BoneProfiles.Contains(Bone);
}

// Current
bool UOHPhysicsComponent::IsBlendWeightInvalid(float Weight)
{
    return Weight < 0.0f || Weight > 1.0f;
}

// Current
#if WITH_EDITOR
void UOHPhysicsComponent::ValidateAllBoneStates() const
{
    UE_LOG(LogTemp, Log, TEXT("[VALIDATION] Running full bone state audit..."));

    for (const auto& Pair : BoneStates)
    {
        ValidateBoneState(Pair.Key);
    }

    UE_LOG(LogTemp, Log, TEXT("[VALIDATION] Audit complete."));
}
#endif

// Current
//#if !UE_BUILD_SHIPPING
void UOHPhysicsComponent::DisplayBoneValidationError(EOHSkeletalBone Bone, const FString& Message)
{
    if (!Mesh) return;

    const FName BoneName = GetBoneNameFromEnum(Bone);
    const FVector WorldLocation = Mesh->GetSocketLocation(BoneName);

    // Draw floating text in viewport
    DrawDebugString(
        GetWorld(),
        WorldLocation + FVector(0, 0, 20), // Offset above bone
        Message,
        nullptr,
        FColor::Red,
        5.0f,
        true
    );

    // Optional: log to screen for quick attention
    GEngine->AddOnScreenDebugMessage(
        -1, 5.0f, FColor::Red,
        FString::Printf(TEXT("Validation Error on %s: %s"), *BoneName.ToString(), *Message)
    );
}






void UOHPhysicsComponent::ResetPropagationFlags()
{
    for (auto& Pair : BoneStates)
    {
        FOHBoneState& State = Pair.Value;
        State.bPropagateAttachedRoleDownward = false;

        if (State.Role == EOHBoneSimulationRole::AttachedToSimulated)
        {
            State.Role = EOHBoneSimulationRole::AnimationOnly;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("UOHPhysicsComponent: Cleared attached roles and propagation flags."));
}

void UOHPhysicsComponent::ResetAllSimulationState()
{
    if (!Mesh || !PhysicalAnimation) return;

    // Clear all active blend operations
    ActiveBlendOperations.Reset();
    ScheduledBlendTimers.Empty();

    // Clear per-bone state
    for (auto& Pair : BoneStates)
    {
        const EOHSkeletalBone Bone = Pair.Key;
        FOHBoneState& State = Pair.Value;

        // Reset all roles and profiles
        ClearBoneState(Bone);

        // Remove physics blend and disable simulation
        const FName BoneName = GetBoneNameFromEnum(Bone);
        FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);

        if (Body)
        {
            Mesh->SetAllBodiesBelowPhysicsBlendWeight(BoneName, 0.0f, false, false);
            Body->SetInstanceSimulatePhysics(false);
            Body->PutInstanceToSleep();
        }

        // Remove any physical animation drives
        PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, FPhysicalAnimationData());
    }

    // Clear auto-remove flags
    BodyPartsToRemoveAfterBlend.Empty();
    RegionsToRemoveAfterBlend.Empty();

    // Optional: force update roles again if needed
    ResolveSimulationRoles(); // <-- not required, but safe to do if you intend to reinitialize next frame

    bPhysicsActive = false;
}


void UOHPhysicsComponent::ResolveSimulationRoles()
{
    // Clear propagation flags
    for (auto& Pair : BoneStates)
    {
        Pair.Value.bPropagateAttachedRoleDownward = false;
    }

    // Loop through all active FadeIn blends with propagation enabled
    for (const FBlendOperation& Op : ActiveBlendOperations)
    {
        if (Op.BlendType != EOHBlendType::FadeIn || !Op.bPropagateAttachedRoleToChildren)
            continue;

        const EOHSkeletalBone RootBone = Op.Bone;
        const TArray<EOHSkeletalBone> ChildBones = UOHSkeletalPhysicsUtils::GetBoneChainBelow(RootBone);

        for (EOHSkeletalBone Child : ChildBones)
        {
            if (Child == RootBone)
                continue;

            FOHBoneState& ChildState = BoneStates.FindOrAdd(Child);

            // Skip if already being explicitly blended or simulated
            const bool bAlreadyBlending = ActiveBlendOperations.ContainsByPredicate([Child](const FBlendOperation& Other)
            {
                return Other.Bone == Child;
            });

            if (bAlreadyBlending || ChildState.Role == EOHBoneSimulationRole::Simulated)
                continue;

            // Propagate attached role intent
            ChildState.Role = EOHBoneSimulationRole::AttachedToSimulated;
            ChildState.SimulationState = EOHBoneSimulationState::Blending;
            ChildState.bPropagateAttachedRoleDownward = true;

            // Start a soft FadeIn blend operation from current weight (typically 0)
            const float StartWeight = FMath::Clamp(ChildState.PhysicsBlendWeight, 0.0f, 1.0f);
            const float FadeDuration = 0.1f; // fast ramp up just to avoid snap

            FBlendOperation FadeInOp = FBlendOperation::CreateFadeIn(Child, FadeDuration, StartWeight);
            FadeInOp.bDeactivateOnComplete = false;
            FadeInOp.bRemoveProfileAfter = false;
            FadeInOp.bPropagateAttachedRoleToChildren = false; // avoid recursion

            StartBlendOperation(FadeInOp); // propagate gentle tracking without PAC force
        }
    }
}

void UOHPhysicsComponent::FinalizeBoneSimulationStates()
{
    if (!Mesh || !PhysicalAnimation) return;

    // Ensure children inherit any necessary attached-to-simulated roles
    ResolveSimulationRoles();

    for (auto& Pair : BoneStates)
    {
        const EOHSkeletalBone Bone = Pair.Key;
        FOHBoneState& State = Pair.Value;

        const FName BoneName = GetBoneNameFromEnum(Bone);
        FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);
        if (!Body) continue;

        const bool bShouldSimulate = State.Role == EOHBoneSimulationRole::Simulated;
        const bool bAttachedToSim = State.Role == EOHBoneSimulationRole::AttachedToSimulated;

        // Clamp and validate blend weight
        State.PhysicsBlendWeight = FMath::Clamp(State.PhysicsBlendWeight, 0.f, 1.f);

        // Apply physical animation settings (respects blend weight internally)
        PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, State.ResolvedProfileData);

        // Set blend weight into skeletal mesh
        Mesh->SetAllBodiesBelowPhysicsBlendWeight(BoneName, State.PhysicsBlendWeight, false, false);

        // Determine sim state
        if (bShouldSimulate)
        {
            if (!Body->IsInstanceSimulatingPhysics())
            {
                Body->SetInstanceSimulatePhysics(true);
                Body->WakeInstance();
            }
        }
        else if (bAttachedToSim)
        {
            // Attached bones follow parent but don't simulate
            Body->SetInstanceSimulatePhysics(false);
            Body->PutInstanceToSleep();
        }
        else
        {
            // Fully animation-driven
            Body->SetInstanceSimulatePhysics(false);
            Body->PutInstanceToSleep();
        }

        // Optional: runtime damping overrides from behavior strategy
        if (ActiveBehaviorStrategy.IsValid())
        {
            FOHResolvedPhysicsTargets StrategyTargets = ActiveBehaviorStrategy->Resolve(State, Bone);
            Body->LinearDamping  = StrategyTargets.TargetLinearDamping;
            Body->AngularDamping = StrategyTargets.TargetAngularDamping;
        }
    }
}
void UOHPhysicsComponent::FinalizeCompletedBlendOperations()
{
	const double CurrentTime = FPlatformTime::Seconds();
	TArray<int32> CompletedOpIds;

	for (int32 i = 0; i < ActiveBlendOperations.Num(); ++i)
	{
		FBlendOperation& Op = ActiveBlendOperations[i];
		const EOHSkeletalBone Bone = Op.Bone;

		Op.UpdateState(CurrentTime);

		if (FOHBoneState* State = BoneStates.Find(Bone))
		{
			const float CurrentWeight = Op.CalculateCurrentWeight(CurrentTime);
			State->PhysicsBlendWeight = FMath::Clamp(CurrentWeight, 0.0f, 1.0f);
			State->SimulationState = Op.CurrentState;

			// Mark as complete only if interpolation has reached the end
			if (Op.IsComplete(CurrentTime))
			{
				State->BlendType = EOHBlendType::None;

				const bool bIsFadeOut = Op.BlendType == EOHBlendType::FadeOut;
				const bool bWeightReachedZero = FMath::IsNearlyZero(CurrentWeight, 0.01f);

				if (bIsFadeOut && bWeightReachedZero && Op.bDeactivateOnComplete)
				{
					ClearBoneState(Bone);
				}

				if (Op.bRemoveProfileAfter)
				{
					// Only remove profile if blend has safely ended
					PhysicsProfileMap.RemoveBoneProfile(Bone);
				}

				CompletedOpIds.Add(Op.BlendOperationId);
			}
		}
	}

	ActiveBlendOperations.RemoveAll([&](const FBlendOperation& Op)
	{
		return CompletedOpIds.Contains(Op.BlendOperationId);
	});
}

bool UOHPhysicsComponent::IsBoneFollowingSimulatedParent(EOHSkeletalBone Bone) const
{
    const FOHBoneState* State = BoneStates.Find(Bone);
    if (!State || State->Role == EOHBoneSimulationRole::Simulated)
        return false;

    EOHSkeletalBone Current = Bone;
    while (Current != EOHSkeletalBone::None)
    {
        EOHSkeletalBone Parent = UOHSkeletalPhysicsUtils::GetParentBone(Current);
        const FOHBoneState* ParentState = BoneStates.Find(Parent);

        if (!ParentState)
            return false;

        if (ParentState->Role == EOHBoneSimulationRole::Simulated &&
            ParentState->bPropagateAttachedRoleDownward)
        {
            return true;
        }

        Current = Parent;
    }

    return false;
}

float UOHPhysicsComponent::GetCurrentBlendWeight(EOHSkeletalBone Bone) const
{
    if (const FOHBoneState* State = BoneStates.Find(Bone))
    {
        return State->PhysicsBlendWeight;
    }
    return 0.0f;
}

void UOHPhysicsComponent::DebugPrintBoneRoleState(EOHSkeletalBone Bone) const
{
    const FOHBoneState* State = BoneStates.Find(Bone);
    if (!State)
    {
        UE_LOG(LogTemp, Warning, TEXT("DebugPrint: BoneEnum %s has no state."), *GetBoneNameFromEnum(Bone).ToString());
        return;
    }

    FString RoleStr = UEnum::GetValueAsString(State->Role);
    FString StateStr = UEnum::GetValueAsString(State->SimulationState);
    bool bFollows = IsBoneFollowingSimulatedParent(Bone);
    FString FollowStr = bFollows ? TEXT("✓") : TEXT("✗");

    UE_LOG(LogTemp, Log, TEXT("BoneEnum: %s | Role: %s | SimulationState: %s | Weight: %.2f | Follows Sim Parent: %s"),
        *GetBoneNameFromEnum(Bone).ToString(),
        *RoleStr,
        *StateStr,
        State->PhysicsBlendWeight,
        *FollowStr);
}

//#endif


FOHBoneState UOHPhysicsComponent::GetBoneState(EOHSkeletalBone Bone) const
{
    if (const FOHBoneState* State = BoneStates.Find(Bone))
    {
        return *State;
    }
    return FOHBoneState();
}



// ------------------------------- PELVIS MOTION DELTA ------------------------ //
FTransform UOHPhysicsComponent::GetPelvisMotionDelta() const
{
    const FOHBoneState* PelvisState = BoneStates.Find(EOHSkeletalBone::Pelvis);

    if (PelvisState)
    {
        return PelvisState->CurrentWorldTransform.GetRelativeTransform(PelvisState->PreviousWorldTransform);
    }

    return FTransform::Identity;
}

FTransform UOHPhysicsComponent::ResolveMotionCorrectionTransform(EOHMotionCorrectionSource Source, EOHSkeletalBone Bone) const
{
    const FOHBoneState* RootState   = BoneStates.Find(EOHSkeletalBone::Root);
    const FOHBoneState* PelvisState = BoneStates.Find(EOHSkeletalBone::Pelvis);
    const FOHBoneState* ParentState = BoneStates.Find(UOHSkeletalPhysicsUtils::GetParentBone(Bone));
    const FOHBoneState* ChildState  = nullptr;

    if (Source == EOHMotionCorrectionSource::Child)
    {
        const TArray<EOHSkeletalBone> Children = UOHSkeletalPhysicsUtils::GetChildBones(Bone);
        if (Children.Num() > 0)
        {
            ChildState = BoneStates.Find(Children[0]);
        }
    }

    switch (Source)
    {
    case EOHMotionCorrectionSource::Root:
        return RootState ? RootState->CurrentWorldTransform.GetRelativeTransform(RootState->PreviousWorldTransform) : FTransform::Identity;

    case EOHMotionCorrectionSource::Pelvis:
        return PelvisState ? PelvisState->CurrentWorldTransform.GetRelativeTransform(PelvisState->PreviousWorldTransform) : FTransform::Identity;

    case EOHMotionCorrectionSource::PelvisAndRoot:
        if (PelvisState && RootState)
        {
            const FTransform PelvisDelta = PelvisState->CurrentWorldTransform.GetRelativeTransform(PelvisState->PreviousWorldTransform);
            const FTransform RootDelta   = RootState->CurrentWorldTransform.GetRelativeTransform(RootState->PreviousWorldTransform);
            return PelvisDelta * RootDelta;
        }
        return FTransform::Identity;

    case EOHMotionCorrectionSource::Parent:
        return ParentState ? ParentState->CurrentWorldTransform.GetRelativeTransform(ParentState->PreviousWorldTransform) : FTransform::Identity;

    case EOHMotionCorrectionSource::Child:
        return ChildState ? ChildState->CurrentWorldTransform.GetRelativeTransform(ChildState->PreviousWorldTransform) : FTransform::Identity;

    case EOHMotionCorrectionSource::Custom:
    case EOHMotionCorrectionSource::None:
    default:
        return FTransform::Identity;
    }
}

FTransform UOHPhysicsComponent::GetEffectiveCorrectionTransform(
    EOHMotionCorrectionSource Source,
    EOHSkeletalBone Bone,
    const FTransform& CustomTransform) const
{
    return (Source == EOHMotionCorrectionSource::Custom)
        ? CustomTransform
        : ResolveMotionCorrectionTransform(Source, Bone);
}


FVector UOHPhysicsComponent::GetCorrectedLinearVelocity(EOHSkeletalBone Bone, EOHMotionCorrectionSource Source, FTransform CustomTransform)
{
    const FTransform Correction = GetEffectiveCorrectionTransform(Source, Bone, CustomTransform);
    const FOHBoneState* State = BoneStates.Find(Bone);
    return State ? State->GetCorrectedLinearVelocity(Correction) : FVector::ZeroVector;
}

FVector UOHPhysicsComponent::GetCorrectedLinearAcceleration(EOHSkeletalBone Bone, EOHMotionCorrectionSource Source, FTransform CustomTransform)
{
    const FTransform Correction = GetEffectiveCorrectionTransform(Source, Bone, CustomTransform);
    const FOHBoneState* State = BoneStates.Find(Bone);
    return State ? State->GetCorrectedLinearAcceleration(Correction) : FVector::ZeroVector;
}

FVector UOHPhysicsComponent::GetCorrectedAngularVelocity(EOHSkeletalBone Bone, EOHMotionCorrectionSource Source, FTransform CustomTransform)
{
    const FTransform Correction = GetEffectiveCorrectionTransform(Source, Bone, CustomTransform);
    const FOHBoneState* State = BoneStates.Find(Bone);
    return State ? State->GetCorrectedAngularVelocity(Correction) : FVector::ZeroVector;
}

FVector UOHPhysicsComponent::GetCorrectedAngularAcceleration(EOHSkeletalBone Bone, EOHMotionCorrectionSource Source, FTransform CustomTransform)
{
    const FTransform Correction = GetEffectiveCorrectionTransform(Source, Bone, CustomTransform);
    const FOHBoneState* State = BoneStates.Find(Bone);
    return State ? State->GetCorrectedAngularAcceleration(Correction) : FVector::ZeroVector;
}

FTransform UOHPhysicsComponent::GetCorrectedBoneTransform(
    EOHSkeletalBone Bone,
    bool bUsePreviousFrame,
    EOHMotionCorrectionSource Source,
    FTransform CustomTransform)
{
    const FTransform Correction = GetEffectiveCorrectionTransform(Source, Bone, CustomTransform);
    const FOHBoneState* State = BoneStates.Find(Bone);
    if (!State) return FTransform::Identity;

    return bUsePreviousFrame
        ? State->GetPreviousTransformCorrected(Correction)
        : State->GetCurrentTransformCorrected(Correction);
}



void UOHPhysicsComponent::StartBlendOperation(const FBlendOperation& Operation)
{
	if (!Mesh || !PhysicalAnimation) return;

	const FName BoneName = GetBoneNameFromEnum(Operation.Bone);
	const double CurrentTime = FPlatformTime::Seconds();

	ClearActiveBlendOperation(Operation.Bone); // Ensure no duplicates

	// --- Resolve Profile ---
	FPhysicalAnimationData ResolvedProfile;
	if (!Operation.ProfileNameOverride.IsNone())
	{
		if (NamedProfileMap.Contains(Operation.ProfileNameOverride))
		{
			ResolvedProfile = NamedProfileMap[Operation.ProfileNameOverride];
			PhysicsProfileMap.SetBoneProfile(Operation.Bone, ResolvedProfile);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("StartBlendOperation: Missing profile %s for bone %s"),
				*Operation.ProfileNameOverride.ToString(), *BoneName.ToString());
		}
	}
	else
	{
		ResolvedProfile = PhysicsProfileMap.GetResolvedProfile(Operation.Bone);
	}

	PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, ResolvedProfile);

	// --- Enable Physics on Body Instance ---
	if (FBodyInstance* Body = Mesh->GetBodyInstance(BoneName))
	{
		Body->SetInstanceSimulatePhysics(true);

		if (Operation.StartWeight > 0.95f)
		{
			Body->WakeInstance();
		}
	}

	// --- Blend Weight Application ---
	const bool bApplyStartWeight =
		(Operation.BlendType == EOHBlendType::FadeIn) ||
		(Operation.BlendType == EOHBlendType::FadeOut && Operation.StartWeight > 0.05f);

	if (bApplyStartWeight && !ActiveBlendOperations.ContainsByPredicate(
		[&](const FBlendOperation& Existing) { return Existing.Bone == Operation.Bone; }))
	{
		Mesh->SetAllBodiesBelowPhysicsBlendWeight(BoneName, Operation.StartWeight, false, false);
	}

	// --- Sync FOHBoneState from blend ---
	SyncBoneStateFromBlendOperation(Operation);

	// --- Role Propagation ---
	if (Operation.BlendType == EOHBlendType::FadeIn && Operation.bPropagateAttachedRoleToChildren)
	{
		FOHBoneState& RootState = BoneStates.FindOrAdd(Operation.Bone);
		RootState.bPropagateAttachedRoleDownward = true;
	}

	// --- Force Cleanup If FadeOut + Profile Active ---
	if (Operation.BlendType == EOHBlendType::FadeOut &&
		Operation.bDeactivateOnComplete &&
		!Operation.bRemoveProfileAfter)
	{
		const_cast<FBlendOperation&>(Operation).bRemoveProfileAfter = true;
	}

	ActiveBlendOperations.Add(Operation);

	UE_LOG(LogTemp, Log, TEXT("StartBlendOperation: %s → %.2f → %.2f (%.2fs) | Profile: %s"),
		*BoneName.ToString(),
		Operation.StartWeight,
		Operation.TargetWeight,
		Operation.Duration,
		*Operation.ProfileNameOverride.ToString());
}

void UOHPhysicsComponent::StartBlendOperation(
    EOHSkeletalBone Bone,
    float Duration,
    bool bFadeIn,
    float CurrentWeight,
    FName ProfileNameOverride,
    UCurveFloat* BlendCurve,
    float StrengthOverride,
    UCurveFloat* StrengthCurve,
    bool bRemoveProfileAfter,
    bool bPropagateToChildren
)
{
    if (!BoneContextMap.HasValidBoneState(Bone)) return;

    // Auto-fill current weight if not specified
    if (CurrentWeight < 0.f)
    {
        const FOHBoneState* State = BoneContextMap.FindState(Bone);
        if (State)
        {
            CurrentWeight = State->PhysicsBlendWeight;
        }
    }

    FBlendOperation Operation = bFadeIn
        ? FBlendOperation::CreateFadeIn(Bone, Duration, CurrentWeight, ProfileNameOverride)
        : FBlendOperation::CreateFadeOut(Bone, Duration, CurrentWeight, ProfileNameOverride);

    Operation.BlendCurve = BlendCurve;
    Operation.StrengthOverride = StrengthOverride;
    Operation.StrengthCurve = StrengthCurve;
    Operation.bRemoveProfileAfter = bRemoveProfileAfter;
    Operation.bPropagateAttachedRoleToChildren = bPropagateToChildren;

    // Route to core
    StartBlendOperation(Operation);
}

// Internal Logic ---------------------------------->

FOHBoneState& UOHPhysicsComponent::AddOrGetBoneState(EOHSkeletalBone Bone)
{
    FOHBoneState& State = BoneStates.FindOrAdd(Bone);
    ENSURE_BONESTATE_KEY(State, Bone);
    return State;
}

void UOHPhysicsComponent::ClearBoneState(EOHSkeletalBone Bone)
{
    if (FOHBoneState* State = BoneStates.Find(Bone))
    {
        State->Role = EOHBoneSimulationRole::AnimationOnly;
        State->SimulationState = EOHBoneSimulationState::Kinematic;
        State->PhysicsBlendWeight = 0.0f;
        State->ActiveProfileName = NAME_None;
        State->ResolvedProfileData = FPhysicalAnimationData();
        State->bPropagateAttachedRoleDownward = false;
    }
}

#if !UE_BUILD_SHIPPING
void UOHPhysicsComponent::ValidateSimulationRoleGraph() const
{
    for (const auto& Pair : BoneStates)
    {
        const EOHSkeletalBone Bone = Pair.Key;
        const FOHBoneState& State = Pair.Value;

        if (State.Role == EOHBoneSimulationRole::Simulated && State.PhysicsBlendWeight < 0.9f)
        {
            UE_LOG(LogTemp, Warning, TEXT("BoneEnum %s marked Simulated but blend weight is %.2f"),
                *UEnum::GetValueAsString(Bone), State.PhysicsBlendWeight);
        }

        if (State.Role == EOHBoneSimulationRole::AttachedToSimulated)
        {
            const EOHSkeletalBone Parent = UOHSkeletalPhysicsUtils::GetParentBone(Bone);
            const FOHBoneState* ParentState = BoneStates.Find(Parent);

            if (!ParentState || ParentState->Role != EOHBoneSimulationRole::Simulated)
            {
                UE_LOG(LogTemp, Warning, TEXT("BoneEnum %s is AttachedToSimulated but parent %s is not Simulated"),
                    *UEnum::GetValueAsString(Bone),
                    *UEnum::GetValueAsString(Parent));
            }
        }
    }
}
#endif

EOHBoneSimulationRole UOHPhysicsComponent::GetBoneSimulationRole(EOHSkeletalBone Bone) const
{
    if (const FOHBoneState* State = BoneStates.Find(Bone))
    {
        return State->Role;
    }
    return EOHBoneSimulationRole::AnimationOnly;
}




// -------------------- MAIN BLENDING OPERATIONS ------------------------------ //

#if 0
void UOHPhysicsComponent::StartBoneBlend(EOHSkeletalBone BoneEnum, EOHBlendType BlendDirection, FName ProfileName)
{
    if (!Mesh || !PhysicalAnimation) return;

    FOHBoneState& State = BoneStates.FindOrAdd(BoneEnum);
    State.SimulationState = EOHBoneSimulationState::Blending;
    State.BlendType = BlendDirection;
    State.PhysicsBlendWeight = FMath::Clamp(State.PhysicsBlendWeight, 0.f, 1.f);

    const FName BoneName = GetBoneNameFromEnum(BoneEnum);

    if (BlendDirection == EOHBlendType::FadeIn)
    {
        State.Role = EOHBoneSimulationRole::Simulated;

        if (!ProfileName.IsNone())
        {
            State.ActiveProfileName = ProfileName;

            if (NamedProfileMap.Contains(ProfileName))
            {
                const FPhysicalAnimationData& Data = NamedProfileMap[ProfileName];
                PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, Data);
            }
        }

        Mesh->SetAllBodiesBelowPhysicsBlendWeight(BoneName, 0.f, false, false);
    }
    else if (BlendDirection == EOHBlendType::FadeOut)
    {
        // Role remains Simulated until blend finishes
    }
}
#endif

// ------------------------ BLEND IN ------------------------------ //
// Current
void UOHPhysicsComponent::FadeInBonePhysics(EOHSkeletalBone Bone, FName ProfileName, float Duration)
{
	const float SafeDuration = FMath::Max(Duration, 0.01f);
	const FOHBoneState* State = BoneStates.Find(Bone);
	if (!State) return;

	const float CurrentWeight = State->PhysicsBlendWeight;

	FBlendOperation Op = FBlendOperation::CreateFadeIn(Bone, SafeDuration, CurrentWeight, ProfileName);
	StartBlendOperation(Op);
}


// Current
void UOHPhysicsComponent::FadeInBodyPartPhysics(EOHBodyPart BodyPart, FName ProfileName, float Duration)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        FadeInBonePhysics(Bone, ProfileName, Duration);
    }
}


// Current
void UOHPhysicsComponent::FadeInRegionPhysics(EOHBodyRegion Region, FName ProfileName, float Duration)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);

    for (EOHSkeletalBone Bone : Bones)
    {
        FadeInBonePhysics(Bone, ProfileName, Duration);
    }
}

// ----------------- BLEND OUT ---------------------- //

// Current
void UOHPhysicsComponent::FadeOutBonePhysics(EOHSkeletalBone Bone, float Duration, bool bRemoveProfileAfter)
{
    const float SafeDuration = FMath::Max(Duration, 0.01f);
    const FOHBoneState* State = BoneStates.Find(Bone);
    if (!State) return;

    const float CurrentWeight = State->PhysicsBlendWeight;

    FBlendOperation Op = FBlendOperation::CreateFadeOut(Bone, SafeDuration, CurrentWeight);
    Op.bDeactivateOnComplete = true;
    Op.bRemoveProfileAfter = bRemoveProfileAfter;

    StartBlendOperation(Op);
}


// Current
void UOHPhysicsComponent::FadeOutBodyPartPhysics(EOHBodyPart BodyPart, float Duration, bool bRemoveProfileAfter)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyPart(BodyPart);

    for (EOHSkeletalBone Bone : Bones)
    {
        FadeOutBonePhysics(Bone, Duration, bRemoveProfileAfter);
    }
}


// Current
void UOHPhysicsComponent::FadeOutRegionPhysics(EOHBodyRegion Region, float Duration, bool bRemoveProfileAfter)
{
    const TArray<EOHSkeletalBone> Bones = GetBonesInBodyRegion(Region);

    for (EOHSkeletalBone Bone : Bones)
    {
        FadeOutBonePhysics(Bone, Duration, bRemoveProfileAfter);
    }
}

void UOHPhysicsComponent::ApplySimStateForBone(EOHSkeletalBone Bone, const FOHBoneState& State)
{
    if (!Mesh || !PhysicalAnimation) return;

    const FName BoneName = GetBoneNameFromEnum(Bone);
    FBodyInstance* Body = Mesh->GetBodyInstance(BoneName);
    if (!Body) return;

    const float BlendWeight = FMath::Clamp(State.PhysicsBlendWeight, 0.0f, 1.0f);
    const bool bShouldSimulate = UOHSkeletalPhysicsUtils::ShouldSimulate(State);

    // Step 1: Physics state transition
    if (bShouldSimulate)
    {
        if (!Body->IsInstanceSimulatingPhysics())
        {
            Body->SetInstanceSimulatePhysics(true);
            Body->WakeInstance();
        }
    }
    else
    {
        if (Body->IsInstanceSimulatingPhysics())
        {
            Body->SetInstanceSimulatePhysics(false);
            Body->PutInstanceToSleep();
        }
    }

    // Step 2: Safe blend weight application (guards snapping)
    if (BlendWeight > 0.0f)
    {
        Mesh->SetAllBodiesBelowPhysicsBlendWeight(BoneName, BlendWeight, false, false);
    }

    // Step 3: PAC drive logic (only if blend weight is meaningful and role allows it)
    const bool bAllowDrive = State.Role != EOHBoneSimulationRole::AttachedToSimulated && BlendWeight > 0.05f;

    if (bAllowDrive && UOHSkeletalPhysicsUtils::ShouldApplyPACDrive(State))
    {
        PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, State.ResolvedProfileData);
    }
    else
    {
        FPhysicalAnimationData EmptyDrive;
        PhysicalAnimation->ApplyPhysicalAnimationSettings(BoneName, EmptyDrive);
    }

    // Step 4: Optional strategy-based damping override
    if (ActiveBehaviorStrategy.IsValid())
    {
        FOHResolvedPhysicsTargets Strategy = ActiveBehaviorStrategy->Resolve(State, Bone);
        Body->LinearDamping = Strategy.TargetLinearDamping;
        Body->AngularDamping = Strategy.TargetAngularDamping;
    }
}
// --------------------- Helper Functions ---------------------- //

bool UOHPhysicsComponent::ShouldPropagateSimStateFromParent(EOHSkeletalBone Bone) const
{
    // Only propagate if parent is Simulated and this bone is AttachedToSimulated
    const FOHBoneState* State = BoneStates.Find(Bone);
    if (!State || State->Role != EOHBoneSimulationRole::AttachedToSimulated)
        return false;

    EOHSkeletalBone Parent = UOHSkeletalPhysicsUtils::GetParentBone(Bone);
    const FOHBoneState* ParentState = BoneStates.Find(Parent);
    return ParentState && ParentState->Role == EOHBoneSimulationRole::Simulated;
}

void UOHPhysicsComponent::SetGlobalPhysicalAnimationStrength(float NewMultiplier)
{
    GlobalStrengthMultiplier = FMath::Clamp(NewMultiplier, 0.0f, 10.0f); // Adjust upper bound if needed

    if (PhysicalAnimation)
    {
        PhysicalAnimation->SetStrengthMultiplyer(GlobalStrengthMultiplier);
        UE_LOG(LogTemp, Log, TEXT("Set global physical animation strength to %.2f"), GlobalStrengthMultiplier);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("SetGlobalPhysicalAnimationStrength called but PhysicalAnimationComponent is null."));
    }
}



// ---------------------- DEBUG ------------------ //


void UOHPhysicsComponent::PrintBoneDebug(
    EOHSkeletalBone Bone,
    const FOHBoneState& State,
    const FBlendOperation& Op,
    const IOHPhysicsBehaviorStrategy* Strategy,
    float ElapsedTime
)
{
    if (!Mesh || !GEngine) return;

    const FName BoneName = GetBoneNameFromEnum(Bone);

    if (!Mesh->DoesSocketExist(BoneName)) return;

    // --- Visual labels
    const FString BoneLabel = UEnum::GetValueAsString(Bone);
    const FString Stage = (State.PhysicsBlendWeight >= 1.0f) ? TEXT("Stage 2") : TEXT("Stage 1");
    const FString StrategyName = Strategy ? Strategy->GetStrategyName() : TEXT("None");

    const FString Msg = FString::Printf(
        TEXT("[%s] %s | PAC=%.2f | ProxyAlpha=%.2f | Elapsed=%.2fs | Strategy=%s\nOri=%.1f Pos=%.1f Vel=%.1f AngVel=%.1f"),
        *BoneLabel,
        *Stage,
        State.PhysicsBlendWeight,
        State.ProxyBlendAlpha,
        ElapsedTime,
        *StrategyName,
        State.ResolvedProfileData.OrientationStrength,
        State.ResolvedProfileData.PositionStrength,
        State.ResolvedProfileData.VelocityStrength,
        State.ResolvedProfileData.AngularVelocityStrength
    );

    // --- On-screen HUD message
    GEngine->AddOnScreenDebugMessage(
        (int32)Bone, // unique ID per bone
        0.016f,
        FColor::Cyan,
        Msg
    );

    // --- Draw debug line from simulated to animated position
    const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
    FTransform SimulatedTransform = Mesh->GetBoneTransform(BoneIndex);
    SimulatedTransform = SimulatedTransform * Mesh->GetComponentTransform(); // convert to world space
    const FVector SimPos = SimulatedTransform.GetLocation();

    const FVector AnimPos = Mesh->GetSocketTransform(BoneName, RTS_World).GetLocation();
    const FColor LineColor = FLinearColor::LerpUsingHSV(FLinearColor::Blue, FLinearColor::Red, State.ProxyBlendAlpha).ToFColor(true);

    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            DrawDebugLine(
                World,
                SimPos,
                AnimPos,
                LineColor,
                false,
                0.016f,
                0,
                2.0f
            );
        }
    }
}

void UOHPhysicsComponent::DebugBoneState(EOHSkeletalBone Bone)
{
    if (!BoneStates.Contains(Bone) || !ActiveBehaviorStrategy.IsValid()) return;

    const FOHBoneState& State = BoneStates[Bone];

    // Optional: pass dummy blend op or create real one
    FBlendOperation DummyOp;
    DummyOp.Bone = Bone;
    DummyOp.StartTime = FPlatformTime::Seconds();
    DummyOp.Duration = 1.0f;

    PrintBoneDebug(Bone, State, DummyOp, ActiveBehaviorStrategy.Get(), 0.f);
}

void UOHPhysicsComponent::PrepareBoneSimulationData(float DeltaTime)
{
    if (!bIsInitialized || DeltaTime <= 0.f) return;

    for (TPair<EOHSkeletalBone, FOHBoneState>& Elem : BoneStates)
    {
        FOHBoneState& State = Elem.Value;

        // Skip bones that are not actively simulating or blending
        if (!State.IsSimulating() && !State.IsBlending())
        {
            continue;
        }

        // Update motion vectors (vel, accel)
        State.UpdateMotionState(DeltaTime);

        // Smooth those values (you can expose smoothing alpha as a component field if desired)
        const float SmoothingAlpha = 0.15f;
        State.UpdateSmoothedValues(SmoothingAlpha);

        // Optional: Blend damping strength toward targets
        State.LerpTowardsDampingTargets(DeltaTime);
    }
}

void UOHPhysicsComponent::ApplyBoneSimulationTick(float DeltaTime)
{
    if (DeltaTime <= 0.f || BoneStates.Num() == 0) return;

    for (TPair<EOHSkeletalBone, FOHBoneState>& Elem : BoneStates)
    {
        const EOHSkeletalBone BoneEnum = Elem.Key;
        FOHBoneState& State = Elem.Value;

        if (!State.IsSimulating()) continue;

        const FTransform CurrentTransform = BoneContextMap.GetBoneTransform(BoneEnum);
        const FTransform ProxyTarget = BoneContextMap.GetProxySourceTransform(BoneEnum);

        // Simple linear interpolation toward proxy
        const float Alpha = State.ProxyBlendAlpha;
        FTransform BlendedTransform;
        BlendedTransform.Blend(CurrentTransform, ProxyTarget, Alpha);

        State.CurrentWorldTransform = BlendedTransform;
    }
}

void UOHPhysicsComponent::SetProxyToParentIfUnassigned()
{
	USkeletalMeshComponent* SkelMesh = GetSkelMesh();
	if (!SkelMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("[OnlyHands] SkelMesh not found in component."));
		return;
	}

	UAnimInstance* AnimInstance = SkelMesh->GetAnimInstance();
	if (!AnimInstance)
	{
		UE_LOG(LogTemp, Warning, TEXT("[OnlyHands] AnimInstance not valid."));
		return;
	}

	const FBoneContainer& BoneContainer = AnimInstance->GetRequiredBones();

	for (TPair<EOHSkeletalBone, FOHBoneState>& Pair : BoneContextMap.GetMutableBoneStates())
	{
		EOHSkeletalBone Bone = Pair.Key;
		FOHBoneState& State = Pair.Value;

		if (State.ProxyFollowSourceBone == EOHSkeletalBone::None)
		{
			EOHSkeletalBone ParentBone = BoneContextMap.GetReferenceMap().GetParentBoneEnum(Bone, BoneContainer);
			if (ParentBone != EOHSkeletalBone::None &&
				BoneContextMap.GetReferenceMap().HasValidReference(ParentBone))
			{
				State.SetProxySource(ParentBone);
			}
		}
	}
}

void UOHPhysicsComponent::SimulationTick(float DeltaTime)
{
    if (!BoneContextMap.IsInitialized()) return;

    const FBoneContainer* BoneContainer = GetBoneContainer();
    if (!BoneContainer) return;

    for (TPair<EOHSkeletalBone, FOHBoneState>& Pair : BoneContextMap.GetMutableBoneStates())
    {
        EOHSkeletalBone Bone = Pair.Key;
        FOHBoneState& State = Pair.Value;

        if (!State.IsInitialized()) continue;

        // Proxy Follow: override transforms if source is set
        if (State.ProxyFollowSourceBone != EOHSkeletalBone::None)
        {
            const FTransform ProxyTransform = BoneContextMap.GetProxySourceTransform(Bone);
            State.SetTransforms(ProxyTransform); // Applies to both current and previous
        }

        // Update motion state + smoothing
        State.UpdateMotionState(DeltaTime);
        State.UpdateSmoothedValues();

        // Damping interpolation (simple lerp for now)
        State.LerpTowardsDampingTargets(0.1f);
    }
}

USkeletalMeshComponent* UOHPhysicsComponent::GetSkelMesh() const
{
    return Cast<USkeletalMeshComponent>(GetOwner()->GetComponentByClass(USkeletalMeshComponent::StaticClass()));
}

void UOHPhysicsComponent::InitializeProxyChains()
{
    const FBoneContainer* BoneContainer = GetBoneContainer();
    if (!BoneContainer)
    {
        UE_LOG(LogTemp, Warning, TEXT("[OnlyHands] Cannot initialize proxy chains: BoneContainer invalid."));
        return;
    }

    for (TPair<EOHSkeletalBone, FOHBoneState>& Pair : BoneContextMap.GetMutableBoneStates())
    {
        EOHSkeletalBone Bone = Pair.Key;
        FOHBoneState& State = Pair.Value;

        if (State.ProxyFollowSourceBone != EOHSkeletalBone::None)
            continue;

        EOHSkeletalBone Current = Bone;
        while (Current != EOHSkeletalBone::None)
        {
            const EOHSkeletalBone Parent = BoneContextMap.GetReferenceMap().GetParentBoneEnum(Current, *BoneContainer);
            if (Parent == EOHSkeletalBone::None) break;

            if (BoneContextMap.GetReferenceMap().HasValidReference(Parent))
            {
                State.SetProxySource(Parent);
                break;
            }

            Current = Parent;
        }
    }
}

void UOHPhysicsComponent::ApplyAutoProxyFollowChains()
{
	if (!BoneContextMap.IsInitialized()) return;

	const FBoneContainer* BoneContainer = GetBoneContainer();
	if (!BoneContainer || !BoneContainer->IsValid()) return;

	const TMap<EOHSkeletalBone, FOHBoneState>& States = BoneContextMap.GetBoneStates();
	TMap<EOHSkeletalBone, EOHSkeletalBone> ParentMap;

	// --- Step 1: Build skeletal hierarchy map
	for (EOHSkeletalBone Bone : BoneContextMap.GetReferenceMap().GetMappedBones())
	{
		const FCompactPoseBoneIndex PoseIndex = BoneContextMap.GetReferenceMap().GetPoseIndex(Bone);
		if (PoseIndex == INDEX_NONE) continue;

		FCompactPoseBoneIndex ParentIndex = BoneContainer->GetParentBoneIndex(PoseIndex);
		if (ParentIndex == INDEX_NONE) continue;

		const FName ParentName = BoneContainer->GetReferenceSkeleton().GetBoneName(
			BoneContainer->MakeMeshPoseIndex(ParentIndex).GetInt()
		);

		const EOHSkeletalBone ParentEnum = GetEnumFromBoneName(ParentName);
		if (ParentEnum != EOHSkeletalBone::None)
		{
			ParentMap.Add(Bone, ParentEnum);
		}
	}

	// --- Step 2: Load region-aware overrides (if enabled)
	TMap<EOHSkeletalBone, EOHSkeletalBone> RegionOverrides =
		bApplyRegionAwareProxyChains ? GetRegionAwareProxyOverrides() : TMap<EOHSkeletalBone, EOHSkeletalBone>();

	// --- Step 3: Walk each bone and assign best proxy source
	for (TPair<EOHSkeletalBone, FOHBoneState>& Pair : BoneContextMap.GetMutableBoneStates())
	{
		const EOHSkeletalBone Bone = Pair.Key;
		FOHBoneState& State = Pair.Value;

		// Already simulating — skip proxy assignment
		if (State.IsSimulating()) continue;

		EOHSkeletalBone ProxySource = EOHSkeletalBone::None;

		// Priority 1: Use explicit region-aware override
		if (RegionOverrides.Contains(Bone))
		{
			ProxySource = RegionOverrides[Bone];
		}
		else
		{
			// Priority 2: Walk skeletal hierarchy to find simulated parent
			EOHSkeletalBone Current = ParentMap.Contains(Bone) ? ParentMap[Bone] : EOHSkeletalBone::None;
			while (Current != EOHSkeletalBone::None)
			{
				const FOHBoneState* ParentState = States.Find(Current);
				if (ParentState && ParentState->IsSimulating())
				{
					ProxySource = Current;
					break;
				}
				Current = ParentMap.Contains(Current) ? ParentMap[Current] : EOHSkeletalBone::None;
			}
		}

		// Apply proxy source if found
		if (ProxySource != EOHSkeletalBone::None)
		{
			State.SetProxySource(ProxySource, 1.f);

#if WITH_EDITOR
			if (bEnableDebugDraw)
			{
				const FString DebugStr = FString::Printf(TEXT("[ProxyChain] %s → %s"),
					*UEnum::GetValueAsString(Bone),
					*UEnum::GetValueAsString(ProxySource));
				UE_LOG(LogTemp, Verbose, TEXT("%s"), *DebugStr);
			}
#endif
		}
	}
}

TMap<EOHSkeletalBone, EOHSkeletalBone> UOHPhysicsComponent::GetRegionAwareProxyOverrides() const
{
    TMap<EOHSkeletalBone, EOHSkeletalBone> Overrides;

    // Spine
    Overrides.Add(EOHSkeletalBone::Spine_01, EOHSkeletalBone::Pelvis);
    Overrides.Add(EOHSkeletalBone::Spine_02, EOHSkeletalBone::Spine_01);
    Overrides.Add(EOHSkeletalBone::Spine_03, EOHSkeletalBone::Spine_02);
    Overrides.Add(EOHSkeletalBone::Neck_01, EOHSkeletalBone::Spine_03);
    Overrides.Add(EOHSkeletalBone::Head, EOHSkeletalBone::Neck_01);

    // Left Arm
    Overrides.Add(EOHSkeletalBone::UpperArm_L, EOHSkeletalBone::Clavicle_L);
    Overrides.Add(EOHSkeletalBone::LowerArm_L, EOHSkeletalBone::UpperArm_L);
    Overrides.Add(EOHSkeletalBone::Hand_L, EOHSkeletalBone::LowerArm_L);

    // Right Arm
    Overrides.Add(EOHSkeletalBone::UpperArm_R, EOHSkeletalBone::Clavicle_R);
    Overrides.Add(EOHSkeletalBone::LowerArm_R, EOHSkeletalBone::UpperArm_R);
    Overrides.Add(EOHSkeletalBone::Hand_R, EOHSkeletalBone::LowerArm_R);

    // Fingers — L
    Overrides.Add(EOHSkeletalBone::Thumb_01_L, EOHSkeletalBone::Hand_L);
    Overrides.Add(EOHSkeletalBone::Index_01_L, EOHSkeletalBone::Hand_L);
    Overrides.Add(EOHSkeletalBone::Middle_01_L, EOHSkeletalBone::Hand_L);
    Overrides.Add(EOHSkeletalBone::Ring_01_L, EOHSkeletalBone::Hand_L);
    Overrides.Add(EOHSkeletalBone::Pinky_01_L, EOHSkeletalBone::Hand_L);

    // Fingers — R
    Overrides.Add(EOHSkeletalBone::Thumb_01_R, EOHSkeletalBone::Hand_R);
    Overrides.Add(EOHSkeletalBone::Index_01_R, EOHSkeletalBone::Hand_R);
    Overrides.Add(EOHSkeletalBone::Middle_01_R, EOHSkeletalBone::Hand_R);
    Overrides.Add(EOHSkeletalBone::Ring_01_R, EOHSkeletalBone::Hand_R);
    Overrides.Add(EOHSkeletalBone::Pinky_01_R, EOHSkeletalBone::Hand_R);

    return Overrides;
}
#endif
