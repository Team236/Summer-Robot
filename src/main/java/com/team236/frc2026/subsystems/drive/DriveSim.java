package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import com.team236.frc2026.simulation.SimulatedRobotState;
import com.team236.frc2026.utils.simulations.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveIOSim} class extends {@link DriveHardware} to provide simulation-specific
 * functionality for the swerve drive system.
 */
public class DriveSim extends DriveHardware {

    private SimulatedRobotState simRobotState = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;
    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    Pose2d lastConsumedPose = null;
    Consumer<SwerveDriveState> simTelemetryConsumer =
            swerveDriveState -> {
                // Protect at init
                if (simRobotState == null) {
                    return;
                }

                if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
                    swerveDriveState.Pose =
                            mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                }
                simRobotState.addFieldToRobot(swerveDriveState.Pose);
                // We don't have telemetryConsumer_ from DriveIOHardware here since it was private or non-existent in DriveHardware
                // but we can just use the telemetry logic from DriveHardware
            };

    public DriveSim(
            RobotState robotState,
            SimulatedRobotState simRobotState,
            SwerveDrivetrainConstants driveTrainConstants,
            @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
        super(robotState, driveTrainConstants, 
              Constants.useMapleSim ? MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules) : modules);
        this.simRobotState = simRobotState;

        // Rewrite the telemetry consumer with a consumer for sim
        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.useMapleSim) {
            mapleSimSwerveDrivetrain =
                    new MapleSimSwerveDrivetrain(
                            Units.Seconds.of(kSimLoopPeriod),
                            Units.Pounds.of(Constants.DriveConstants.kRobotWeightPounds),
                            Units.Inches.of(Constants.DriveConstants.kBumperLengthInches),
                            Units.Inches.of(Constants.DriveConstants.kBumperWidthInches),
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            Constants.DriveConstants.kWheelCoefficientOfFriction,
                            getModuleLocations(),
                            getPigeon2(),
                            getModules(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()[0],
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()[1],
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()[2],
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()[3]);
            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier =
                    new Notifier(
                            () -> {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - lastSimTime;
                                lastSimTime = currentTime;
                                updateSimState(deltaTime, RobotController.getBatteryVoltage());
                            });
        }
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void seedFieldCentric() {
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(new Pose2d());
            Timer.delay(0.05);
        }
        super.seedFieldCentric();
    }

    @Override
    public void resetGyro() {
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(new Pose2d());
            Timer.delay(0.05);
        }
        super.resetGyro();
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        super.readInputs(inputs);

        // Handle the viz
        var pose = simRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", simRobotState.getLatestFieldToRobot());
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }
}
