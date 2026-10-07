package com.team236.frc2026.controlboard;

import com.team236.frc2026.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code XboxButtonControlBoard} reads button inputs from an Xbox controller 
 * and maps them to specific triggers for robot functionality.
 */
public class XboxButtonControlBoard implements IButtonControlBoard {
    private static XboxButtonControlBoard mInstance = null;
    
    private final CommandXboxController mController;

    private XboxButtonControlBoard() {
        mController = new CommandXboxController(Constants.Controller.kMainController);
    }

    public static XboxButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new XboxButtonControlBoard();
        }
        return mInstance;
    }

    @Override
    public Trigger getResetGyro() {
        return mController.y();
    }

    @Override
    public Trigger getToggleMode() {
        return mController.b();
    }
}