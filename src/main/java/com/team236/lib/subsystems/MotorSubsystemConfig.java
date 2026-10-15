package com.team236.lib.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MotorSubsystemConfig {
    public String name = "UNNAMED";
    public CANDeviceId talonCANId;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    public double unitToRotorRatio = 1.0;
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;
}
