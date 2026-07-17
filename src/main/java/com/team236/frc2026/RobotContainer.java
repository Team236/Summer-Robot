package com.team236.frc2026;

import com.team236.frc2026.commands.TeleopSwerveDrive;
import com.team236.frc2026.subsystems.drive.DriveHardware;
import com.team236.frc2026.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {

    public static final XboxController driverController =
            new XboxController(Constants.Controller.kMainController);

    private DriveSubsystem buildDriveSubsystem() {
        return new DriveSubsystem(
                new DriveHardware(
                        mRobotState,
                        Constants.DriveConstants.kDrivetrain.getDrivetrainConstants(),
                        Constants.DriveConstants.kDrivetrain.getModuleConstants()));
    }

    private final DriveSubsystem mDriveSubsystem = buildDriveSubsystem();
    private final RobotState mRobotState = new RobotState();

    private final TeleopSwerveDrive driveCommand =
            (new TeleopSwerveDrive(
                    mDriveSubsystem,
                    mRobotState,
                    this,
                    () -> driverController.getLeftX(),
                    () -> driverController.getLeftY(),
                    () -> driverController.getRightX()));
}
