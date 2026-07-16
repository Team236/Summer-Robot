package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * DriveIO interface sets up the input and output systems for the drive train. Contains methods to
 * control and monitor drive train.
 */
public interface DriveIO {

    @AutoLog
    class SwerveDriveTelemetry extends SwerveDriveState {
        public double gyroAngle = 0.0;

        SwerveDriveTelemetry() {
            this.Pose = new Pose2d(); // Later abstract this out to helper class
        }

        // Update current SwerveDriveTelemetry's variables with new data (Own through inheritance)
        public void updateFromState(SwerveDriveState currentState) {
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

    void readInputs(SwerveDriveTelemetry inputs);

    void logModules(SwerveDriveState driveState);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);
}
