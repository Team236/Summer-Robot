package com.team236.frc2026.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import com.team236.frc2026.RobotState;
import com.team236.frc2026.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class TeleopSwerveDrive extends Command {
    private final RobotState mRobotState;
    private final RobotContainer mRobotContainer;
    protected DriveSubsystem mDrivetrain;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    private final SwerveRequest.FieldCentric driveOpenLoop =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Constants.DriveConstants.kMaxDriveSpeed
                                    * Constants.DriveConstants.kOpenLoopDeadband)
                    .withRotationalDeadband(
                            Constants.DriveConstants.kMaxRotationalRate
                                    * Constants.DriveConstants.kOpenLoopDeadband)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public TeleopSwerveDrive(
            DriveSubsystem drivetrain,
            RobotState robotState,
            RobotContainer robotContainer,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier turn) {

        mDrivetrain = drivetrain;
        mRobotState = robotState;
        mRobotContainer = robotContainer;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * Constants.DriveConstants.kMaxDriveSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * Constants.DriveConstants.kMaxDriveSpeed;
        double turnRate = mTurnSupplier.getAsDouble();

        double throttleFieldRelative = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldRelative = mRobotState.isRedAlliance() ? -strafe : strafe;

        mDrivetrain.setControl(
                driveOpenLoop
                        .withVelocityX(throttleFieldRelative)
                        .withVelocityY(strafeFieldRelative)
                        .withRotationalRate(
                                turnRate * Constants.DriveConstants.kMaxRotationalRate));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
