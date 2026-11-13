package com.team236.lib.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class FuelPhysicsSimManager {
    private static FuelPhysicsSim mFuelSim;
    private static RobotContainer mRobotContainer;

    private static final Timer mOuttakeTimer = new Timer();
    private static final double mBaseRads =
            (Units.degreesToRadians(
                    Constants.ArmConstants.ArmPivotConstants.kArmPivotBaseAngleDegrees));

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

        Optional<Pose3d> fuelPose =
                RobotVizHelpers.getGrabberBallFieldPose(mRobotContainer.getRobotState());
        if (fuelPose.isEmpty()) {
            return;
        }

        Translation3d ejectionVelField = computeLaunchVelocityVector();
        Logger.recordOutput("Sim/LaunchVelocityVector", ejectionVelField);
    }

    private static Translation3d computeLaunchVelocityVector() {
        var speeds = mRobotContainer.getSimulatedRobotState().getSimChassisSpeeds();
        double armPivotRads = mRobotContainer.getRobotState().getArmPivotAngleRadians();
        double elevationAngleRadUnits = mBaseRads + armPivotRads;
        double vxRobot =
                Constants.ArmConstants.ArmIntakeConstants.kSimOuttakeVelocityMetersPerSec
                        * Math.sin(elevationAngleRadUnits);
        double vzRobot =
                Constants.ArmConstants.ArmIntakeConstants.kSimOuttakeVelocityMetersPerSec
                        * Math.cos(elevationAngleRadUnits);

        var robotPose = mRobotContainer.getRobotState().getLatestFieldToRobot();
        double robotYawRads =
                robotPose != null ? robotPose.getValue().getRotation().getRadians() : 0.0;

        return new Translation3d(vxRobot, 0.0, vzRobot)
                .rotateBy(new Rotation3d(0.0, 0.0, robotYawRads))
                .plus(new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0));
    }

    public static void launchFuel() {
        if (mFuelSim == null || mRobotContainer == null) {
            return;
        }

        Optional<Pose3d> fuelPose =
                RobotVizHelpers.getGrabberBallFieldPose(mRobotContainer.getRobotState());
        if (fuelPose.isEmpty()) {
            return;
        }

        Translation3d ejectionVelField = computeLaunchVelocityVector();

        mFuelSim.launchBall(
                fuelPose.get().getTranslation(),
                ejectionVelField,
                Constants.ArmConstants.ArmIntakeConstants.kSimOuttakeRPM);
        mRobotContainer.getRobotState().setIntakePossession(false);
    }
}
