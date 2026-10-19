package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code IButtonControlBoard} defines the standard interface for reading button inputs from a
 * input device.
 */
public interface IButtonControlBoard {
    Trigger getResetGyro();

    Trigger getIntakeMode();

    Trigger getScoringMode();

    Trigger getManualMode();

    Trigger getManualArmPivotExtend();

    Trigger getManualArmPivotRetract();
}
