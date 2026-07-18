package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code DriveIO} interface sets up the input and output systems for the drivetrain. It
 * contains methods to control and monitor the drivetrain.
 */
public interface DriveIO {

    @AutoLog
    class DriveIOTelemetry extends SwerveDriveState {
        public double gyroAngle = 0.0;

        DriveIOTelemetry() {
            this.Pose = new Pose2d(); // Later abstract this out to helper class
        }

        // Update current DriveIOTelemetry's variables with new data (Inherited from SwerveDriveState)
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

    // Interface methods that must be implemented by DriveHardware

    void readInputs(DriveIOTelemetry inputs);

    void logModules(SwerveDriveState driveState);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void resetGyro();
}