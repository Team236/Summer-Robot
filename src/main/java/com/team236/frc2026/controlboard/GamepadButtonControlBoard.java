package com.team236.frc2026.controlboard;

import com.team236.frc2026.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard instance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadButtonControlBoard();
        }
        return instance;
    }

    private final CommandXboxController controller;

    private GamepadButtonControlBoard() {
        controller = new CommandXboxController(Constants.Controller.kOperatorController);
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
