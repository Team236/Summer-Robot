package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The {@code KeyboardDriveControlBoard} reads keyboard inputs from an computer and applies scaling
 * to feed drivetrain translation and rotation. (Axis(0): w-s, Axis(1): a-d, Axis(2): q-e)
 */
public class KeyboardDriveControlBoard implements IDriveControlBoard {
    private static KeyboardDriveControlBoard mInstance = null;

    private final GenericHID mController = new GenericHID(0);

    private KeyboardDriveControlBoard() {}

    public static KeyboardDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new KeyboardDriveControlBoard();
        }
        return mInstance;
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(mController.getRawAxis(0)), 1.5))
                * Math.signum(mController.getRawAxis(0));
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(mController.getRawAxis(1)), 1.5))
                * Math.signum(mController.getRawAxis(1));
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(mController.getRawAxis(2)), 2.0))
                * Math.signum(mController.getRawAxis(2));
    }

    @Override
    public double getRotationY() {
        return (Math.pow(Math.abs(mController.getRawAxis(2)), 2.0))
                * Math.signum(mController.getRawAxis(2));
    }
}
