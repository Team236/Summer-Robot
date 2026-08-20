package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.frc2026.subsystems.drive.DriveIO.DriveIOInputs;
import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.lib.limelight.LimelightHelpers;
import com.team236.lib.simulation.MapleSimSwerveDrivetrain;
import com.team236.lib.time.RobotTime;

import edu.wpi.first.math.geometry.Pose2d;
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

    public void setControl(SwerveRequest request) {
        mIo.setControl(request);
    }

    public void resetOdometry(Pose2d pose) {
        mIo.resetOdometry(pose);
    }

    public void logDriveSubsystem(double timestamp) {
        Logger.recordOutput(
                "Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput(
                "Drive/currentCommand",
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();

        mIo.readInputs(mInputs);
        updateLimelightGyroData(mInputs);
        Logger.processInputs("DriveInputs", mInputs);

        mIo.logModules(mInputs);
        logDriveSubsystem(timestamp);
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        mIo.addVisionMeasurement(visionFieldPoseEstimate);
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (mIo instanceof DriveSim) {
            return ((DriveSim) mIo).getMapleSimDrive();
        }
        return null;
    }


    private void updateLimelightGyroData(DriveIOInputs ioInputs) {
        LimelightHelpers.SetRobotOrientation(
                "limelight",
                ioInputs.gyroAngle,
                ioInputs.yawVelocity,
                ioInputs.pitch,
                ioInputs.pitchVelocity,
                ioInputs.roll,
                ioInputs.rollVelocity);
    }
}
