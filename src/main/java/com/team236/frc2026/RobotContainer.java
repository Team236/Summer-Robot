package com.team236.frc2026;

import com.team236.frc2026.commands.TeleopSwerveDrive;
import com.team236.frc2026.subsystems.drive.DriveHardware;
import com.team236.frc2026.subsystems.drive.DriveSubsystem;
import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.frc2026.subsystems.vision.VisionHardwareLimelight;
import com.team236.frc2026.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.Consumer;

/**
 * The {@code RobotContainer} class holds robot subsystems, commands, and operator interface
 * bindings. It acts as the primary structure for dependency injection and button mapping.
 */
public class RobotContainer {

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate estimate) {
                    mDriveSubsystem.addVisionMeasurement(estimate);
                }
            };

    private final RobotState mRobotState = new RobotState(visionEstimateConsumer);
    private final XboxController mDriverController =
            new XboxController(Constants.Controller.kMainController);

    private final DriveSubsystem mDriveSubsystem = buildDriveSubsystem();
    private final VisionSubsystem mVisionSubsystem =
            new VisionSubsystem(new VisionHardwareLimelight(mRobotState), mRobotState);

    private final TeleopSwerveDrive mDriveCommand =
            new TeleopSwerveDrive(
                    mDriveSubsystem,
                    mRobotState,
                    this,
                    () -> -mDriverController.getLeftY(),
                    () -> -mDriverController.getLeftX(),
                    () -> -mDriverController.getRightX());

    public RobotContainer() {
        configureBindings();
    }

    private DriveSubsystem buildDriveSubsystem() {
        return new DriveSubsystem(
                new DriveHardware(
                        mRobotState,
                        Constants.DriveConstants.kDrivetrain.getDrivetrainConstants(),
                        Constants.DriveConstants.kDrivetrain.getModuleConstants()));
    }

    private void configureBindings() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);

        new JoystickButton(mDriverController, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(mDriveSubsystem::resetGyro, mDriveSubsystem));
    }
}
