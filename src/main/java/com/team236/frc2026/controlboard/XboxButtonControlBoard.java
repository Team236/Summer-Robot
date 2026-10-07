package com.team236.frc2026.controlboard;

import com.team236.frc2026.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxButtonControlBoard implements IButtonControlBoard {
    private static XboxButtonControlBoard instance = null;

    public static XboxButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new XboxButtonControlBoard();
        }
        return instance;
    }

    private final CommandXboxController controller;

    private XboxButtonControlBoard() {
        controller = new CommandXboxController(Constants.Controller.kMainController);
    }

    @Override
    public Trigger getResetGyro() {
        return controller.y();
    }

    @Override
    public Trigger getToggleMode() {
        return controller.b();
    }
}
