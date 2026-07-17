package com.team236.frc2026.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.lib.time.RobotTime;
import org.littletonrobotics.junction.Logger;

/** The {@code DriveSubsystem} class controlls the swerve drivetrain, manual and autonomous. */
public class DriveSubsystem extends SubsystemBase {
    DriveIO io;

    DriveIOTelemetryAutoLogged inputs = new DriveIOTelemetryAutoLogged();

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        // Logger.processInputs("DriveInputs", inputs);
        io.logModules(inputs);

    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }
}
