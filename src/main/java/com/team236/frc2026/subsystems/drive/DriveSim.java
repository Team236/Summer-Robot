package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import com.team236.frc2026.simulation.SimulatedRobotState;
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
 * The {@code DriveSim} extends {@code DriveHardware} and simulates the swerve drivetrain hardware,
 * tracks the simulated field pose, and manages the execution of the physics engine loop.
 */
public class DriveSim extends DriveHardware {
    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private SimulatedRobotState mSimRobotState = null;
    private Notifier mSimNotifier = null;
    private double mLastSimTime;
    private MapleSimSwerveDrivetrain mMapleSimSwerveDrivetrain = null;
    private Pose2d mLastConsumedPose = null;

    private final Consumer<SwerveDriveState> mSimTelemetryConsumer =
            swerveDriveState -> {
                if (mSimRobotState == null) {
                    return;
                }

                if (Constants.kUseMapleSim && mMapleSimSwerveDrivetrain != null) {
                    mSimRobotState.addFieldToRobot(
                            mMapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
                } else {
                    mSimRobotState.addFieldToRobot(swerveDriveState.Pose);
                }
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

        registerTelemetry(mSimTelemetryConsumer);
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.kUseMapleSim) {
            mMapleSimSwerveDrivetrain =
                    new MapleSimSwerveDrivetrain(
                            Units.Seconds.of(kSimLoopPeriod),
                            Units.Pounds.of(Constants.TestbedConstants.kRobotWeightPounds),
                            Units.Inches.of(Constants.TestbedConstants.kBumperWidthInches),
                            Units.Inches.of(Constants.TestbedConstants.kBumperLengthInches),
                            DCMotor.getKrakenX60(Constants.TestbedConstants.kDriveMotorCount),
                            DCMotor.getKrakenX60(Constants.TestbedConstants.kDriveMotorCount),
                            Constants.TestbedConstants.kWheelCoefficientOfFriction,
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

        var pose = mSimRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", mSimRobotState.getLatestFieldToRobot());
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrivetrain() {
        return mMapleSimSwerveDrivetrain;
    }
}
