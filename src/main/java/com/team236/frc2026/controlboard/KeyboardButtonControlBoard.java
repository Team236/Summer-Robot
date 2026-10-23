package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardButtonControlBoard implements IButtonControlBoard {
    private static KeyboardButtonControlBoard mInstance = null;

    private final CommandGenericHID mController = new CommandGenericHID(0);

    private KeyboardButtonControlBoard() {}

    public static KeyboardButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new KeyboardButtonControlBoard();
        }
        return mInstance;
    }

    @Override
    public Trigger getResetGyro() {
        return mController.button(1);
    }

    @Override
    public Trigger getIntakeMode() {
        return mController.button(2);
    }

    @Override
    public Trigger getScoringMode() {
        return mController.button(3);
    }

    @Override
    public Trigger getManualMode() {
        return mController.button(4);
    }

    @Override
    public Trigger getManualArmPivotExtend() {
        return mController.button(5);
    }

    @Override
    public Trigger getManualArmPivotRetract() {
        return mController.button(6);
    }
}
