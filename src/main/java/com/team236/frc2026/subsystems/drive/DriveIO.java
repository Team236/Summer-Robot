package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.lib.math.GeometryHelpers;
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
    class DriveIOInputs extends SwerveDriveState {
        public double gyroAngle = 0.0;
        public double yawVelocity;
        public double pitchVelocity;
        public double rollVelocity;
        public double pitch;
        public double roll;

        DriveIOInputs() {
            this.Pose = GeometryHelpers.kPose2dZero;
        }

        // Update current DriveIOInputs's variables with new data
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

    void readInputs(DriveIOInputs ioInputs);

    void logModules(SwerveDriveState driveState);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void resetGyro();
}
