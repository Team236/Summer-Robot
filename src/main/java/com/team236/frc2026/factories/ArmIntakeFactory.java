package com.team236.frc2026.factories;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The {@code ArmFactory} provides static command factories for the arm pivot subsystem. Commands
 * here drive the arm to its predefined stow and intake positions using Motion Magic, or allow
 * manual duty cycle control.
 */
public class ArmIntakeFactory {

    /**
     * Manually drives the arm upward using open-loop duty cycle.
     *
     * @return A command that applies positive duty cycle while held, stops on release.
     */
    public static Command manualIntake(RobotContainer container) {
        return container
                .getArmIntake()
                .voltageCommand(
                        () -> Constants.ArmConstants.ArmIntakeConstants.kArmIntakeManualVoltage)
                .withName("Arm Intake");
    }

    /**
     * Manually drives the arm downward using open-loop duty cycle.
     *
     * @return A command that applies negative duty cycle while held, stops on release.
     */
    public static Command manualOuttake(RobotContainer container) {
        return container
                .getArmIntake()
                .voltageCommand(
                        () -> -Constants.ArmConstants.ArmIntakeConstants.kArmIntakeManualVoltage)
                .withName("Arm Outtake");
    }
}
