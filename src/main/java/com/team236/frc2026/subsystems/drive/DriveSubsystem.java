package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code DriveSubsystem} class controls the swerve drivetrain for both manual and autonomous
 * operation.
 */
public class DriveSubsystem extends SubsystemBase {
    private final DriveIO mIo;
    private final DriveIOTelemetryAutoLogged mInputs = new DriveIOTelemetryAutoLogged();

    public DriveSubsystem(DriveIO io) {
        mIo = io;
    }

    @Override
    public void periodic() {
        mIo.readInputs(mInputs);
        // Logger.processInputs("DriveInputs", mInputs);
        mIo.logModules(mInputs);
    }

    public void setControl(SwerveRequest request) {
        mIo.setControl(request);
    }

    public void resetGyro() {
        mIo.resetGyro();
    }
}
