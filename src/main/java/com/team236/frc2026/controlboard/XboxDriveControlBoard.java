package com.team236.frc2026.controlboard;

import com.team236.frc2026.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxDriveControlBoard implements IDriveControlBoard {
    private static XboxDriveControlBoard instance = null;

    public static XboxDriveControlBoard getInstance() {
        if (instance == null) {
            instance = new XboxDriveControlBoard();
        }

        return instance;
    }

    private final CommandXboxController controller;

    private XboxDriveControlBoard() {
        controller = new CommandXboxController(Constants.Controller.kMainController);
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5))
                * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5))
                * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0))
                * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0))
                * Math.signum(controller.getRightY());
    }
}
