package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IButtonControlBoard {
    Trigger getResetGyro();

    Trigger getToggleMode();
}
