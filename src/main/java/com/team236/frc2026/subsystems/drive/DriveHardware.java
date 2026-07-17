package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class DriveHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {

    // Tread safe cache for telemetry data
    AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;

    // Creation of drive train
    public DriveHardware(
            RobotState robotState,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, 250.0, modules);

        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        // Sets thread speed in parent class
        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                angularPitchVelocity,
                angularRollVelocity,
                roll,
                pitch,
                accelerationX,
                accelerationY);

        this.getOdometryThread().setThreadPriority(99);
    }

    // Interface methods
    public void readInputs(DriveIOTelemetry inputs) {}

    public void logModules(SwerveDriveState driveState) {}

    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }
}
