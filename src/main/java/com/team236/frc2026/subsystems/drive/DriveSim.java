package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import com.team236.frc2026.SimulatedRobotState;
import com.team236.lib.simulation.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveSim} class extends {@link DriveHardware} to provide simulation-specific
 * functionality for the swerve drive system.
 */
public class DriveSim extends DriveHardware {

    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private SimulatedRobotState mSimRobotState = null;
    private Notifier mSimNotifier = null;
    private double mLastSimTime;
    private MapleSimSwerveDrivetrain mMapleSimSwerveDrivetrain = null;
    private Pose2d mLastConsumedPose = null;

    private Consumer<SwerveDriveState> mSimTelemetryConsumer =
            swerveDriveState -> {
                // Protect at init
                if (mSimRobotState == null) {
                    return;
                }

                if (Constants.kUseMapleSim && mMapleSimSwerveDrivetrain != null) {
                    swerveDriveState.Pose =
                            mMapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                }
                mSimRobotState.addFieldToRobot(swerveDriveState.Pose);
                telemetryConsumer.accept(swerveDriveState);
            };

    public DriveSim(
            RobotState robotState,
            SimulatedRobotState simRobotState,
            SwerveDrivetrainConstants driveTrainConstants,
            @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
        super(
                robotState,
                driveTrainConstants,
                Constants.kUseMapleSim
                        ? MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
                        : modules);
        this.mSimRobotState = simRobotState;

        // Rewrite the telemetry consumer with a consumer for sim
        registerTelemetry(mSimTelemetryConsumer);
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.kUseMapleSim) {
            mMapleSimSwerveDrivetrain =
                    new MapleSimSwerveDrivetrain(
                            Units.Seconds.of(kSimLoopPeriod),
                            Units.Pounds.of(Constants.SimulationConstants.kRobotWeightPounds),
                            Units.Inches.of(Constants.SimulationConstants.kBumperWidthInches),
                            Units.Inches.of(Constants.SimulationConstants.kBumperLengthInches),
                            DCMotor.getKrakenX60(1),
                            DCMotor.getKrakenX60(1),
                            1.2,
                            getModuleLocations(),
                            getPigeon2(),
                            getModules(),
                            SimTunerConstants.FrontLeft,
                            SimTunerConstants.FrontRight,
                            SimTunerConstants.BackLeft,
                            SimTunerConstants.BackRight);
            mSimNotifier = new Notifier(mMapleSimSwerveDrivetrain::update);
        } else {
            mLastSimTime = Utils.getCurrentTimeSeconds();
            mSimNotifier =
                    new Notifier(
                            () -> {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - mLastSimTime;
                                mLastSimTime = currentTime;
                                updateSimState(deltaTime, RobotController.getBatteryVoltage());
                            });
        }
        mSimNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (Constants.kUseMapleSim && mMapleSimSwerveDrivetrain != null) {
            mMapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        super.readInputs(inputs);

        // Handle the viz
        var pose = mSimRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", mSimRobotState.getLatestFieldToRobot());
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mMapleSimSwerveDrivetrain;
    }
}
