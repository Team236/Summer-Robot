package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.frc2026.RobotState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveHardware} class controls the drivetrain at the hardware level. It implements
 * {@code DriveIO} and uses CTRE's SwerveDrivetrain.
 */
public class DriveHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {

    // Constants & Tuning gains
    private static final double kOdometryFrequencyHz = 250.0;
    private static final int kHighTelemetryFrequencyHz = 250;
    private static final int kLowTelemetryFrequencyHz = 100;
    private static final int kOdometryThreadPriority = 99;
    private static final double kRotationToDegrees = 360.0;
    private static final String[] kModuleNames = {"Drive/FR", "Drive/FL", "Drive/BL", "Drive/BR"};

    // Thread-safe cache for telemetry data
    private AtomicReference<SwerveDriveState> mTelemetryCache = new AtomicReference<>();

    private final StatusSignal<AngularVelocity> mAngularPitchVelocity;
    private final StatusSignal<AngularVelocity> mAngularRollVelocity;
    private final StatusSignal<AngularVelocity> mAngularYawVelocity;
    private final StatusSignal<Angle> mRoll;
    private final StatusSignal<Angle> mPitch;
    private final StatusSignal<LinearAcceleration> mAccelerationX;
    private final StatusSignal<LinearAcceleration> mAccelerationY;

    // Creation of drivetrain
    public DriveHardware(
            RobotState robotState,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                driveTrainConstants,
                kOdometryFrequencyHz,
                modules);

        mAngularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        mAngularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        mAngularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        mRoll = getPigeon2().getRoll();
        mPitch = getPigeon2().getPitch();
        mAccelerationX = getPigeon2().getAccelerationX();
        mAccelerationY = getPigeon2().getAccelerationY();

        configureSignalUpdateFrequencies();

        this.getOdometryThread().setThreadPriority(kOdometryThreadPriority);
        registerTelemetry(state -> mTelemetryCache.set(state.clone()));
    }

    // Sets polling rate for CTRE hardware
    private void configureSignalUpdateFrequencies() {
        BaseStatusSignal.setUpdateFrequencyForAll(kHighTelemetryFrequencyHz, mAngularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                kLowTelemetryFrequencyHz,
                mAngularPitchVelocity,
                mAngularRollVelocity,
                mRoll,
                mPitch,
                mAccelerationX,
                mAccelerationY);
    }

    // Interface methods
    @Override
    public void readInputs(DriveIOTelemetry ioInputs) {
        var state = mTelemetryCache.get();
        if (state == null) return;

        ioInputs.updateFromState(state);
        ioInputs.gyroAngle = ioInputs.Pose.getRotation().getDegrees();

        BaseStatusSignal.refreshAll(
                mAngularYawVelocity,
                mAngularPitchVelocity,
                mAngularRollVelocity,
                mRoll,
                mPitch,
                mAccelerationX,
                mAccelerationY);

        // Updating localization + other systems with this data
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        if (driveState.ModuleStates == null) return;
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(
                    kModuleNames[i] + " Absolute Encoder Angle",
                    getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble()
                            * kRotationToDegrees);
            Logger.recordOutput(
                    kModuleNames[i] + " Steering Angle", driveState.ModuleStates[i].angle);
            Logger.recordOutput(
                    kModuleNames[i] + " Target Steering Angle", driveState.ModuleTargets[i].angle);
            Logger.recordOutput(
                    kModuleNames[i] + " Drive Velocity",
                    driveState.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(
                    kModuleNames[i] + " Target Drive Velocity",
                    driveState.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }

    @Override
    public void resetGyro() {
        super.seedFieldCentric();
    }
}
