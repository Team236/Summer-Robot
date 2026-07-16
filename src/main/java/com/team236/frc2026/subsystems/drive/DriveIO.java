package com.team236.frc2026.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveIO {

    @AutoLog
    class SwerveDriveInputs extends SwerveDriveState {
        public double gyroAngle = 0.0;

        SwerveDriveInputs() {
            this.Pose = new Pose2d(); // Later abstract this out
        }

        public void setFromState (SwerveDriveState currentState) {
            this.Pose = currentState.Pose;
            this.SuccessfulDaqs = currentState.SuccessfulDaqs;
            this.FailedDaqs = currentState.FailedDaqs;
            this.ModuleStates = currentState.ModuleStates;
            this.ModuleTargets = currentState.ModuleTargets;
            this.OdometryPeriod = currentState.OdometryPeriod;
            this.Speeds = currentState.Speeds;
        }
    }

    // Interface methods that must be implimented by DriveHarware

    void getInputs(SwerveDriveInputs inputs);

    void logModules(SwerveDriveState driveState);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);
}
