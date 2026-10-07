package com.team236.frc2026.controlboard;

/**
 * The {@code IDriveControlBoard} defines the standard interface for reading drive inputs from a
 * input device.
 */
public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    double getRotationY();
}
