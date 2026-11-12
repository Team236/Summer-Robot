package com.team236.lib.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class FuelPhysicsSimManager {
    private static FuelPhysicsSim mFuelSim;
    private static RobotContainer mRobotContainer;

    private static final Timer mOuttakeTimer = new Timer();

    public static void configureFuelSim(RobotContainer container) {
        mRobotContainer = container;

        mFuelSim =
                FuelPhysicsSim.configureFuelSim(
                        Constants.TestbedConstants.kBumperWidthInches,
                        Constants.TestbedConstants.kBumperLengthInches,
                        Constants.TestbedConstants.kBumperHeightInches,
                        () -> container.getSimulatedRobotState().getLatestFieldToRobot(),
                        () -> container.getSimulatedRobotState().getSimChassisSpeeds());

        mFuelSim.addIntakeZone(
                () ->
                        container
                                .getRobotState()
                                .getArmPose()
                                .transformBy(Constants.ArmConstants.kArmPivotToGrabber),
                Units.inchesToMeters(7.0),
                Units.inchesToMeters(7.0),
                () -> {
                    if (container.getRobotState().getIntakePossession()
                            || !container.getArmIntake().canIntake()
                            || !(container
                                            .getRobotState()
                                            .getArmPose()
                                            .transformBy(Constants.ArmConstants.kArmPivotToGrabber)
                                            .getZ()
                                    < Units.inchesToMeters(
                                            Constants.FieldDimensions.kFuelDiameter))) {
                        return false;
                    }
                    return true;
                },
                () -> container.getRobotState().setIntakePossession(true));
    }

    public static void tick() {
        if (mFuelSim == null) {
            return;
        }
        if (mRobotContainer.getRobotState().getIntakePossession()
                && mRobotContainer.getArmIntake().isOuttaking()) {
            mOuttakeTimer.start();
            if (mOuttakeTimer.hasElapsed(
                    Constants.ArmConstants.ArmIntakeConstants.kOuttakeDelaySeconds)) {
                launchFuel();
                mOuttakeTimer.stop();
                mOuttakeTimer.reset();
            }
        } else {
            mOuttakeTimer.stop();
            mOuttakeTimer.reset();
        }
        mFuelSim.tick();
    }

    public static void launchFuel() {
        if (mFuelSim == null) {
            return;
        }

        Optional<Pose3d> fuelPose =
                RobotVizHelpers.getGrabberBallFieldPose(mRobotContainer.getRobotState());
        if (fuelPose.isEmpty()) {
            return;
        }

        var speeds = mRobotContainer.getSimulatedRobotState().getSimChassisSpeeds();

        mFuelSim.launchBall(
                fuelPose.get().getTranslation(),
                new Translation3d(
                                Constants.ArmConstants.ArmIntakeConstants
                                        .kSimOuttakeVelocityMetersPerSec,
                                0.0,
                                0.0)
                        .rotateBy(fuelPose.get().getRotation())
                        .plus(
                                new Translation3d(
                                        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0)),
                Constants.ArmConstants.ArmIntakeConstants.kSimOuttakeRPM);
        mRobotContainer.getRobotState().setIntakePossession(false);
    }
}
