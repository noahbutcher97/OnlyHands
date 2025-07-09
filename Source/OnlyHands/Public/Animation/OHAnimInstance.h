#pragma once

#include "Animation/AnimInstance.h"
#include "Animation/Nodes/AnimNode_ArmCompressionResponse.h"
#include "Animation/OHAnimInstance_Base.h"
#include "CoreMinimal.h"
#include "OHAnimInstance.generated.h"

class UOHPhysicsComponent;

/**
 * Animation instance for procedural arm compression using 2-bone IK.
 * Integrates with UOHPhysicsComponent if present. Exposes control for runtime
 * IK shaping.
 */
UCLASS()
class ONLYHANDS_API UOHAnimInstance : public UAnimInstance {
  GENERATED_BODY()

public:
  UOHAnimInstance();

  // === Core Animation Lifecycle ===
  virtual void NativeInitializeAnimation() override;
  virtual void NativeUpdateAnimation(float DeltaSeconds) override;

  /** Override the IK effector target (e.g. from collision hit location) */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetCompressionEffectorTarget(FVector WorldTarget);

  /** Reset to velocity-driven compression logic */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void ClearCompressionEffector();

  /** Set blend strength for compression override [0..1] */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetCompressionAlpha(float Alpha);

  /** Flip elbow direction (mirroring for left/right) */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetMirror(bool bMirror);

  /** Drive effector target from a given bone's space (e.g. for missed punches
   * or fallback) */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetEffectorInBoneSpace(FName BoneName, FVector LocalOffset);

  /** Procedurally drive IK from a hit result or force event */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void DriveCompressionFromHit(FVector HitLocation, float Impulse = 1.0f);

  /** Blend between velocity and override effector targets */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetEffectorBlendAlpha(float Alpha);

  /** Damping/smoothing for effector motion */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetEffectorInterpSpeed(float Speed);

  /** Forward-project effector along velocity vector */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetEffectorPredictionOffset(float Distance);

  /** Contextual helper — override effector only if force exceeds threshold */
  UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  void SetEffectorByContext(const FVector &WorldTarget, float Force = 1.0f,
                            bool bForceOverride = false);

  /** Poll physics component and enable compression if high-impulse hit occurred
   */
  // UFUNCTION(BlueprintCallable, Category = "Arm Compression IK")
  // void EnableCompressionFromPhysicsIfCollisionDetected(float ForceThreshold =
  // 300.f);

  /** Check if override is currently active */
  UFUNCTION(BlueprintPure, Category = "Arm Compression IK")
  bool IsCompressionActive() const { return bIsCompressionActive; }

  // Optional override for preview motion injection
  UPROPERTY(EditAnywhere, Category = "OnlyHands|Preview")
  FVector PreviewSimulatedVelocity = FVector(100.f, 0.f, 0.f);

  UPROPERTY(EditAnywhere, Category = "OnlyHands|Preview")
  FVector PreviewOffsetDirection = FVector(1.f, 0.f, 0.f);

  // Motion data passed to IK nodes
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "OnlyHands|IK",
            meta = (PinShownByDefault))
  FVector ExternalLinearVelocity = FVector::ZeroVector;

  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "OnlyHands|IK",
            meta = (PinShownByDefault))
  FVector ExternalPredictedTarget = FVector::ZeroVector;

protected:
  /** The node driving IK, declared in AnimGraph */
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AnimGraph",
            meta = (AllowPrivateAccess = "true"))
  FAnimNode_ArmCompressionResponse ArmCompressionNode;

  /** Whether a compression override is active */
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Compression")
  bool bIsCompressionActive = false;

  /** Whether a valid dynamic physics component was detected */
  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
  bool bHasDynamicPhysics = false;

  UFUNCTION(BlueprintPure, Category = "Arm Compression IK")
  const FAnimNode_ArmCompressionResponse &GetCompressionNode() const {
    return ArmCompressionNode;
  }

  // UFUNCTION(BlueprintPure, Category = "Arm Compression IK")
  // FVector GetCurrentVelocity() const;

  UPROPERTY(EditAnywhere, Category = "Compression")
  float OverrideResetDelay = 0.3f;

  // === Cached physics reference (runtime only) ===
  UPROPERTY(Transient)
  TObjectPtr<UOHPhysicsManager> PhysicsManager = nullptr;

  UFUNCTION(BlueprintPure, Category = "Physics")
  UOHPhysicsManager *GetPhysicsManager() const { return PhysicsManager; }

private:
  /** Internal timer for clearing effector override after a short delay */
  FTimerHandle CompressionResetTimer;
};