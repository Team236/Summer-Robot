package com.team236.frc2026.controlboard;

/**
 * The {@code KeyboardDriveControlBoard} reads keyboard inputs from an computer and applies scaling
 * to feed drivetrain translation and rotation.
 */
public class KeyboardDriveControlBoard implements IDriveControlBoard {
    private static KeyboardDriveControlBoard mInstance = null;

    private KeyboardDriveControlBoard() {}

    public static KeyboardDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new KeyboardDriveControlBoard();
        }
        return mInstance;
    }

    @Override
    public double getThrottle() {
        return 0.0;
    }

    @Override
    public double getStrafe() {
        return 0.0;
    }

    @Override
    public double getRotation() {
        return 0.0;
    }

    @Override
    public double getRotationY() {
        return 0.0;
    }
}
