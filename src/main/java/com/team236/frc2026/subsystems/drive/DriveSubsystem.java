package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveSubsystem} class controls the swerve drivetrain for both manual and autonomous
 * operation.
 */
public class DriveSubsystem extends SubsystemBase {
    private final DriveIO mIo;
    private final DriveIOInputsAutoLogged mInputs = new DriveIOInputsAutoLogged();

    public DriveSubsystem(DriveIO io) {
        mIo = io;
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();

        mIo.readInputs(mInputs);
        Logger.processInputs("DriveInputs", mInputs);
        mIo.logModules(mInputs);
        logDriveSubsystem(timestamp);
    }

    public void setControl(SwerveRequest request) {
        mIo.setControl(request);
    }

    public void resetGyro() {
        mIo.resetGyro();
    }

    public void logDriveSubsystem(double timestamp) {
        Logger.recordOutput(
                "Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput(
                "Drive/currentCommand",
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }
}
