#include "Animation/OHAnimInstance.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "TimerManager.h"
#include "Component/OHPhysicsManager.h"
#include "FunctionLibrary/OHSkeletalPhysicsUtils.h"

UOHAnimInstance::UOHAnimInstance()
    : PreviewSimulatedVelocity(FVector(100.f, 0.f, 0.f)), PreviewOffsetDirection(FVector(1.f, 0.f, 0.f)),
      ExternalLinearVelocity(FVector::ZeroVector), ExternalPredictedTarget(FVector::ZeroVector),
      bIsCompressionActive(false), bHasDynamicPhysics(false), OverrideResetDelay(0.3f) {}

void UOHAnimInstance::NativeInitializeAnimation() {
    Super::NativeInitializeAnimation();

    // Avoid CDO and compile-time context
    if (HasAnyFlags(RF_ClassDefaultObject)) {
        return;
    }

    APawn* PawnOwner = TryGetPawnOwner();
    if (!PawnOwner) {
        return;
    }

    if (PhysicsManager == nullptr) {
        PhysicsManager = PawnOwner->FindComponentByClass<UOHPhysicsManager>();

        if (PhysicsManager != nullptr) {
            UE_LOG(LogTemp, Log, TEXT("[OHAnimInstance] PhysicsComponent successfully bound in InitializeAnimation."));
        }
    }
}

void UOHAnimInstance::SetCompressionEffectorTarget(const FVector WorldTarget) {
    ArmCompressionNode.EffectorTarget = WorldTarget;
    ArmCompressionNode.bUseEffectorOverride = true;
    bIsCompressionActive = true;
}

void UOHAnimInstance::ClearCompressionEffector() {
    ArmCompressionNode.bUseEffectorOverride = false;
    bIsCompressionActive = false;
}

void UOHAnimInstance::SetCompressionAlpha(float Alpha) {
    ArmCompressionNode.CompressionBlendAlpha = FMath::Clamp(Alpha, 0.f, 1.f);
}

void UOHAnimInstance::SetMirror(bool bMirror) {
    ArmCompressionNode.bMirror = bMirror;
}

void UOHAnimInstance::SetEffectorInBoneSpace(FName BoneName, FVector LocalOffset) {
    if (const USkeletalMeshComponent* SkelMesh = GetSkelMeshComponent()) {
        const FTransform BoneTransform = SkelMesh->GetSocketTransform(BoneName, RTS_Component);
        const FVector WorldEffector = BoneTransform.TransformPosition(LocalOffset);
        SetCompressionEffectorTarget(WorldEffector);
    }
}

void UOHAnimInstance::DriveCompressionFromHit(FVector HitLocation, float Impulse) {
    if (Impulse <= 0.f)
        return;

    const float ScaledAlpha = FMath::Clamp(Impulse / 500.f, 0.f, 1.f);
    SetCompressionAlpha(ScaledAlpha);
    SetCompressionEffectorTarget(HitLocation);

    if (UWorld* World = GetWorld()) {
        World->GetTimerManager().ClearTimer(CompressionResetTimer);
        World->GetTimerManager().SetTimer(CompressionResetTimer, this, &UOHAnimInstance::ClearCompressionEffector,
                                          OverrideResetDelay, false);
    }
}

void UOHAnimInstance::SetEffectorBlendAlpha(float Alpha) {
    ArmCompressionNode.EffectorBlendAlpha = FMath::Clamp(Alpha, 0.f, 1.f);
}

void UOHAnimInstance::SetEffectorInterpSpeed(float Speed) {
    ArmCompressionNode.EffectorInterpSpeed = FMath::Max(0.f, Speed);
}

void UOHAnimInstance::SetEffectorPredictionOffset(float Distance) {
    ArmCompressionNode.PredictiveOffset = Distance;
}

void UOHAnimInstance::SetEffectorByContext(const FVector& WorldTarget, float Force, bool bForceOverride) {
    if (Force >= ArmCompressionNode.CompressionThreshold || bForceOverride) {
        SetCompressionEffectorTarget(WorldTarget);
        SetCompressionAlpha(FMath::Clamp(Force / 500.f, 0.f, 1.f));
    }
}

void UOHAnimInstance::NativeUpdateAnimation(float DeltaSeconds) {
    Super::NativeUpdateAnimation(DeltaSeconds);

#if WITH_EDITOR
    if (IsRunningCommandlet()) {
        return;
    }
#endif

    if (HasAnyFlags(RF_ClassDefaultObject)) {
        return;
    }

    const APawn* PawnOwner = TryGetPawnOwner();
    if (!PawnOwner) {
        return;
    }

    USkeletalMeshComponent* SkelMesh = GetSkelMeshComponent();
    if (!SkelMesh || !SkelMesh->IsRegistered()) {
        return;
    }

    // === CRITICAL: EXPLICITLY HANDLE EDITOR PREVIEW ===
#if WITH_EDITOR
    const bool bSimulatePreview = !IsRunningGame();
#else
    constexpr bool bSimulatePreview = false;
#endif

    // === SAFE VARIABLE INITIALIZATION ===
    // Initialize essential external values even if physics component isn't ready
    if (bSimulatePreview || PhysicsManager == nullptr) {
        // CRUCIAL: Set safe defaults that won't cause crashes
        ExternalLinearVelocity =
            PreviewSimulatedVelocity.IsZero() ? FVector(100.f, 0.f, 0.f) : PreviewSimulatedVelocity;

        // Set a sensible predicted target that can't be null
        const FVector SafeTarget =
            SkelMesh ? SkelMesh->GetComponentLocation() + PreviewOffsetDirection.GetSafeNormal() * 50.f
                     : SkelMesh->GetComponentLocation() + FVector(100.f, 0.f, 0.f);

        ExternalPredictedTarget = SafeTarget;

        // Only proceed with physics in runtime
        if (bSimulatePreview) {
            return;
        }
    }

    // === Physics Integration (when available) ===
    if (PhysicsManager == nullptr) {
        PhysicsManager = PawnOwner->FindComponentByClass<UOHPhysicsManager>();
        if (PhysicsManager != nullptr) {
            UE_LOG(LogTemp, Log, TEXT("[OHAnimInstance] PhysicsComponent bound at runtime."));
        }
    }
}

/*
   // === Runtime Path: Pull from physics ===
    if (PhysicsComponent != nullptr)
    {
        ExternalLinearVelocity = PhysicsComponent->GetCorrectedLinearVelocity(
            EOHSkeletalBone::Hand_R,
            EOHMotionCorrectionSource::PelvisAndRoot,
            FTransform::Identity
        );

        ExternalPredictedTarget = UOHSkeletalPhysicsUtils::GetBoneWorldTransformSafe(
            SkelMesh, EOHSkeletalBone::Hand_R).GetLocation();
    }
}
*/
