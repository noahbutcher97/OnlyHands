
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "OHPhysicsComponent.generated.h"
/*
// --------- DRIVE SETTINGS (like Epic's, but ready for expansion) ---------
USTRUCT(BlueprintType)
struct FPhysicalAnimationDriveSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    bool bIsLocalSimulation = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float OrientationStrength = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float AngularVelocityStrength = 100.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float PositionStrength = 500.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float VelocityStrength = 50.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float MaxLinearForce = 20000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float MaxAngularForce = 8000.f;
};

// --------- BONE DRIVE RUNTIME DATA ---------
USTRUCT()
struct FPhysicalAnimationBoneDriveInstance
{
    GENERATED_BODY()

    UPROPERTY()
    FName BoneName;

    // The kinematic target actor handle
    FPhysicsActorHandle TargetActor;

    // The constraint instance between physics bone and kinematic target
    FConstraintInstance* ConstraintInstance = nullptr;

    UPROPERTY()
    FPhysicalAnimationDriveSettings CurrentDriveSettings;

    UPROPERTY()
    float BlendAlpha = 1.f;

    UPROPERTY()
    bool bActive = false;

    FPhysicalAnimationBoneDriveInstance()
        : TargetActor()
        , ConstraintInstance(nullptr)
        , BlendAlpha(1.f)
        , bActive(false)
    {}
};

// --------- USER-DEFINED CHAIN SETTINGS ---------
USTRUCT(BlueprintType)
struct FPhysicalAnimationChainSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    FName ChainName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    FName StartBone;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    FName EndBone;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    FPhysicalAnimationDriveSettings DriveSettings;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    float BlendAlpha = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    bool bEnabled = true;
};

// --------- RUNTIME CHAIN STATE ---------
USTRUCT()
struct FPhysicalAnimationChainRuntime
{
    GENERATED_BODY()

    UPROPERTY() FName ChainName;

    UPROPERTY()
    FPhysicalAnimationChainSettings Settings;

    UPROPERTY()
    TArray<FPhysicalAnimationBoneDriveInstance> BoneDrives;

    UPROPERTY()
    float RuntimeBlendAlpha = 1.f;

    UPROPERTY()
    bool bActive = true;
};

// --------- STABILITY/SANITY CHECK SETTINGS (Optional/Advanced) ---------
USTRUCT(BlueprintType)
struct FPhysicalAnimationStabilitySettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stability")
    float MaxAllowedLinearVelocity = 3000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stability")
    float MaxAllowedAngularVelocity = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stability")
    float MaxAllowedDistanceFromTarget = 100.f;
};

// --------- DELEGATES ---------
UDELEGATE(BlueprintAuthorityOnly)
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPhysicsInstability, FName, ChainName);
// --------- MAIN COMPONENT ---------
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ONLYHANDS_API UOHPhysicsComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UOHPhysicsComponent();

    // Chain definitions
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation")
    TArray<FPhysicalAnimationChainSettings> Chains;

    // Instantly apply drive to a bone/chain, or remove it
    UFUNCTION(CallInEditor, BlueprintCallable, Category="Physical Animation")
    void EnablePhysicsOnChain(FName ChainName, bool bEnable);

    UFUNCTION(CallInEditor, BlueprintCallable, Category="Physical Animation")
    void SetDriveProfileForChain(FName ChainName, UOHPhysicalAnimationProfileDataAsset* Profile);

    UFUNCTION(CallInEditor, BlueprintCallable, Category="Physical Animation")
    void PreviewChainsInEditor();

    FPhysicalAnimationDriveSettings GetDefaultDriveForChain(FName StartBone, FName EndBone);

    // --------- Global or Per-Chain Stability Settings ---------
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physical Animation")
    FPhysicalAnimationStabilitySettings StabilitySettings;

    // --------- Events ---------
    UPROPERTY(BlueprintAssignable)
    FOnPhysicsInstability OnPhysicsInstabilityDetected;

    // --------- API (For C++ and Blueprint) ---------
    UFUNCTION(BlueprintCallable, Category = "Physical Animation")
    void SetChainEnabled(FName ChainName, bool bEnabled);

    UFUNCTION(BlueprintCallable, Category = "Physical Animation")
    void SetChainBlendAlpha(FName ChainName, float BlendAlpha);

    UFUNCTION(BlueprintCallable, Category = "Physical Animation")
    void SetChainDriveSettings(FName ChainName, const FPhysicalAnimationDriveSettings& NewSettings);

    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Physical Animation|Setup")
    void AutoPopulateDefaultUE4Chains();

    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Physical Animation|Setup")
    void AutoPopulateDefaultUE5Chains();

    UFUNCTION(CallInEditor, BlueprintCallable, Category="Physical Animation|Setup")
    void AutoPopulateChainsForCurrentMesh();

    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Physical Animation|Setup")
    void AutoPopulateChainsFromSkeleton();

    UFUNCTION(CallInEditor, BlueprintCallable, Category="Physical Animation")
    void VisualizeChainsInEditor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
    bool bDebugDraw = false;

    UFUNCTION(BlueprintCallable, Category="Debug")
    void SetDebugDrawEnabled(bool bEnable);

    UFUNCTION(BlueprintCallable, Category="Debug")
    void DrawDebugInfo();


    // --- Properties ---

    UPROPERTY(EditAnywhere, Category="Physical Animation")
    UPhysicsAsset* DefaultPhysicsAsset = nullptr; // Fallback if mesh has no asset

    UPhysicsAsset* PhysicsAsset = nullptr; // Runtime reference (private)

    // --- Methods ---
    virtual void InitializeComponent() override;
    virtual void OnUnregister() override;

    void SetupPhysicsAnimationSystem();
    void TeardownPhysicsAnimationSystem();



#if WITH_EDITOR

    void ValidateRuntimePhysicsAgainstTemplate();
    FBodyInstance* EnsureBodyForBone(FName BoneName);
    FConstraintInstance* EnsureConstraintBetweenBones(FName Bone1, FName Bone2);
    FConstraintInstance* EnsureBodiesAndConstraint(FName Bone1, FName Bone2);
#endif

#if WITH_EDITOR

    static void FixAllMissingPhysicsForComponent(
       USkeletalMeshComponent* SkeletalMeshComponent,
       UPhysicsAsset* PhysicsAsset,
       UObject* WorldContext = nullptr // For debug drawing if needed
   );
    static FBodyInstance* EnsureBodyForBone_Static(FName BoneName, USkeletalMeshComponent* SkeletalMeshComponent,
                                                   UPhysicsAsset* PhysicsAsset);
    static FConstraintInstance* EnsureConstraintBetweenBones_Static(FName Bone1, FName Bone2,
                                                                    USkeletalMeshComponent* SkeletalMeshComponent,
                                                                    UPhysicsAsset* PhysicsAsset);
    static FConstraintInstance* EnsureBodiesAndConstraint_Static(FName Bone1, FName Bone2,
                                                                 USkeletalMeshComponent* SkeletalMeshComponent,
                                                                 UPhysicsAsset* PhysicsAsset);
#endif

#if WITH_EDITOR
    void SetupEditorVisualization();
    void CleanupEditorVisualization();
#endif


    // --------- Core Overrides ---------
protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
override;


    // Recommended: Expose default drive profiles for easy tuning in editor!
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation|Setup")
    FPhysicalAnimationDriveSettings DefaultSpineDrive;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation|Setup")
    FPhysicalAnimationDriveSettings DefaultArmDrive;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation|Setup")
    FPhysicalAnimationDriveSettings DefaultLegDrive;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation|Setup")
    FPhysicalAnimationDriveSettings DefaultHeadDrive;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physical Animation|Setup")
    FPhysicalAnimationDriveSettings DefaultGeneralDrive;


    // --------- Internal Helpers & Data ---------
private:
    // The controlled skeletal mesh
    UPROPERTY()
    USkeletalMeshComponent* SkeletalMeshComponent = nullptr;

    // Internal runtime data for chains (mirrors user Chains)
    UPROPERTY()
    TArray<FPhysicalAnimationChainRuntime> RuntimeChains;

    // Find or create bone drive instances as needed
    void InitializeRuntimeChains();

    // Update targets, constraints, and blending each tick
    void UpdatePhysicalAnimation(float DeltaTime);

    // Watchdog for instability
    bool IsChainStable(const FPhysicalAnimationChainRuntime& Chain) const;
    void RecoverChain(FPhysicalAnimationChainRuntime& Chain);

    // Utility to gather bone indices in chain (parent to child, inclusive)
    TArray<int32> GetBoneIndicesInChain(FName StartBone, FName EndBone) const;

    // Create/destroy constraints and kinematic targets
    void CreateBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive, const FPhysicalAnimationDriveSettings&
DriveSettings, const FTransform& InitialTargetTM); static void
DestroyBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive);

    // Move target actor and update constraint drives
    static void UpdateBoneDriveInstance(FPhysicalAnimationBoneDriveInstance& BoneDrive, const FTransform& TargetTM,
const FPhysicalAnimationDriveSettings& DriveSettings, float BlendAlpha);





};*/