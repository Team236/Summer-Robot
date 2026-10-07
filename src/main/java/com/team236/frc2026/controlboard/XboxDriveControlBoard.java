package com.team236.frc2026.controlboard;

import com.team236.frc2026.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The {@code XboxDriveControlBoard} reads joystick inputs from an Xbox controller 
 * and applies scaling to feed drivetrain translation and rotation.
 */
public class XboxDriveControlBoard implements IDriveControlBoard {
    private static XboxDriveControlBoard mInstance = null;
    
    private final CommandXboxController mController;

    private XboxDriveControlBoard() {
        mController = new CommandXboxController(Constants.Controller.kMainController);
    }

    public static XboxDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new XboxDriveControlBoard();
        }
        return mInstance;
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(mController.getLeftY()), 1.5))
                * Math.signum(mController.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(mController.getLeftX()), 1.5))
                * Math.signum(mController.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(mController.getRightX()), 2.0))
                * Math.signum(mController.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(mController.getRightY()), 2.0))
                * Math.signum(mController.getRightY());
    }
}