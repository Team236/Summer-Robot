package com.team236.lib.subsystems.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team236.lib.subsystems.cancoder.CanDeviceId;

/**
 * The {@code MotorSubsystemConfig} stores hardware identifiers, tuning gains, and configuration
 * profiles required to initialize a motor subsystem.
 */
public class MotorSubsystemConfig {
    public String name = "UNNAMED";
    public CanDeviceId talonCANId;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    public double unitToRotorRatio = 1.0;
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;
}
