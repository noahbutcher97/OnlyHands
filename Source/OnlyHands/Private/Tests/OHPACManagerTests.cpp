#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Component/OHPACManager.h"

using namespace UE;

static void ApplyTestProfile(FConstraintInstance& CI, const FPhysicalAnimationData& Profile,
                             EAngularDriveMode::Type Mode) {
    CI.SetAngularDriveMode(Mode);
    CI.SetLinearPositionDrive(true, true, true);
    CI.SetLinearVelocityDrive(true, true, true);
    CI.SetLinearDriveParams(Profile.PositionStrength, Profile.VelocityStrength, 1000.f);

    if (Mode == EAngularDriveMode::SLERP) {
        CI.ProfileInstance.AngularDrive.SlerpDrive.bEnablePositionDrive = true;
        CI.ProfileInstance.AngularDrive.SlerpDrive.bEnableVelocityDrive = true;
        CI.SetAngularOrientationDrive(true, true);
        CI.SetAngularVelocityDrive(true, true);
        CI.SetAngularDriveParams(Profile.OrientationStrength, Profile.AngularVelocityStrength, 1000.f);
    } else {
        CI.ProfileInstance.AngularDrive.SwingDrive.bEnablePositionDrive = true;
        CI.ProfileInstance.AngularDrive.SwingDrive.bEnableVelocityDrive = true;
        CI.ProfileInstance.AngularDrive.TwistDrive.bEnablePositionDrive = true;
        CI.ProfileInstance.AngularDrive.TwistDrive.bEnableVelocityDrive = true;
        CI.SetAngularOrientationDrive(true, true);
        CI.SetAngularVelocityDrive(true, true);
        CI.ProfileInstance.AngularDrive.SwingDrive.Stiffness = Profile.OrientationStrength;
        CI.ProfileInstance.AngularDrive.TwistDrive.Stiffness = Profile.OrientationStrength;
        CI.ProfileInstance.AngularDrive.SwingDrive.Damping = Profile.AngularVelocityStrength;
        CI.ProfileInstance.AngularDrive.TwistDrive.Damping = Profile.AngularVelocityStrength;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReversePACProfileSlerpTest, "OnlyHands.PAC.ReverseProfileMatchesApplied.Slerp",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FReversePACProfileSlerpTest::RunTest(const FString& Parameters) {
    FPhysicalAnimationData Profile;
    Profile.PositionStrength = 1000.f;
    Profile.VelocityStrength = 100.f;
    Profile.OrientationStrength = 15000.f;
    Profile.AngularVelocityStrength = 1000.f;
    Profile.bIsLocalSimulation = true;

    FConstraintInstance CI;
    ApplyTestProfile(CI, Profile, EAngularDriveMode::SLERP);

    FOHConstraintDriveData Data = UOHPACManager::ExtractConstraintDriveData(&CI);
    UOHPACManager* Manager = NewObject<UOHPACManager>();
    FPhysicalAnimationData Result = Manager->ReversePACProfileFromDrives(Data, TEXT("Test"));

    TestTrue(TEXT("Pos"), FMath::IsNearlyEqual(Result.PositionStrength, Profile.PositionStrength));
    TestTrue(TEXT("Vel"), FMath::IsNearlyEqual(Result.VelocityStrength, Profile.VelocityStrength));
    TestTrue(TEXT("Ori"), FMath::IsNearlyEqual(Result.OrientationStrength, Profile.OrientationStrength));
    TestTrue(TEXT("Ang"), FMath::IsNearlyEqual(Result.AngularVelocityStrength, Profile.AngularVelocityStrength));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReversePACProfileSwingTwistTest,
                                 "OnlyHands.PAC.ReverseProfileMatchesApplied.SwingTwist",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FReversePACProfileSwingTwistTest::RunTest(const FString& Parameters) {
    FPhysicalAnimationData Profile;
    Profile.PositionStrength = 2000.f;
    Profile.VelocityStrength = 150.f;
    Profile.OrientationStrength = 20000.f;
    Profile.AngularVelocityStrength = 1500.f;
    Profile.bIsLocalSimulation = true;

    FConstraintInstance CI;
    ApplyTestProfile(CI, Profile, EAngularDriveMode::TwistAndSwing);

    FOHConstraintDriveData Data = UOHPACManager::ExtractConstraintDriveData(&CI);
    UOHPACManager* Manager = NewObject<UOHPACManager>();
    FPhysicalAnimationData Result = Manager->ReversePACProfileFromDrives(Data, TEXT("Test"));

    TestTrue(TEXT("Pos"), FMath::IsNearlyEqual(Result.PositionStrength, Profile.PositionStrength));
    TestTrue(TEXT("Vel"), FMath::IsNearlyEqual(Result.VelocityStrength, Profile.VelocityStrength));
    TestTrue(TEXT("Ori"), FMath::IsNearlyEqual(Result.OrientationStrength, Profile.OrientationStrength));
    TestTrue(TEXT("Ang"), FMath::IsNearlyEqual(Result.AngularVelocityStrength, Profile.AngularVelocityStrength));
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
