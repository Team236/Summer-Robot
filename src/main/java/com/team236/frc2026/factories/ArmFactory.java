package com.team236.frc2026.factories;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The {@code ArmFactory} provides static command factories for the arm pivot subsystem. Commands
 * here drive the arm to its predefined stow and intake positions using Motion Magic, or allow
 * manual duty cycle control.
 */
public class ArmFactory {

    /**
     * Moves the arm pivot to the stow position.
     *
     * @return A command that runs Motion Magic to the stow position.
     */
    public static Command stowArm(RobotContainer container) {
        return container
                .getArmPivot()
                .motionMagicSetpointCommand(() -> Constants.ArmConstants.kArmPivotStowPositionUnits)
                .withName("Arm Stow");
    }

    /**
     * Moves the arm pivot to the intake position.
     *
     * @return A command that runs Motion Magic to the intake position.
     */
    public static Command intakeArm(RobotContainer container) {
        return container
                .getArmPivot()
                .motionMagicSetpointCommand(
                        () -> Constants.ArmConstants.kArmPivotIntakePositionUnits)
                .withName("Arm Intake");
    }

    /**
     * Manually drives the arm upward using open-loop duty cycle.
     *
     * @return A command that applies positive duty cycle while held, stops on release.
     */
    public static Command manualUp(RobotContainer container) {
        return container
                .getArmPivot()
                .dutyCycleCommand(() -> Constants.ArmConstants.kArmPivotManualDutyCycle)
                .withName("Arm Manual Up");
    }

    /**
     * Manually drives the arm downward using open-loop duty cycle.
     *
     * @return A command that applies negative duty cycle while held, stops on release.
     */
    public static Command manualDown(RobotContainer container) {
        return container
                .getArmPivot()
                .dutyCycleCommand(() -> -Constants.ArmConstants.kArmPivotManualDutyCycle)
                .withName("Arm Manual Down");
    }
}
