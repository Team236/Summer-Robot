package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code ControlBoard} acts as a centralized singleton that combines both drive and button
 * control interfaces into a single access point.
 */
public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard mInstance = null;

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        // mDriveControlBoard = XboxDriveControlBoard.getInstance();
        mDriveControlBoard = KeyboardDriveControlBoard.getInstance();
        mButtonControlBoard = XboxButtonControlBoard.getInstance();
    }

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return mDriveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return mDriveControlBoard.getRotation();
    }

    @Override
    public double getRotationY() {
        return mDriveControlBoard.getRotationY();
    }

    @Override
    public Trigger getResetGyro() {
        return mButtonControlBoard.getResetGyro();
    }

    @Override
    public Trigger getToggleMode() {
        return mButtonControlBoard.getToggleMode();
    }
}
