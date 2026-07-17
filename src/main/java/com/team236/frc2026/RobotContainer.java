package com.team236.frc2026;

import com.team236.frc2026.commands.TeleopSwerveDrive;
import com.team236.frc2026.subsystems.drive.DriveHardware;
import com.team236.frc2026.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final RobotState mRobotState = new RobotState();

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

    private final TeleopSwerveDrive mDriveCommand =
            (new TeleopSwerveDrive(
                    mDriveSubsystem,
                    mRobotState,
                    this,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX()));

    private void configureBindings() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);

        new JoystickButton(driverController, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(mDriveSubsystem::resetGyro, mDriveSubsystem));
    }

    RobotContainer() {
        configureBindings();
    }
}
