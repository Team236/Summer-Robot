package com.team236.lib.subsystems.motors;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code MotorInputs} acts as a data transfer object (DTO) that holds standard telemetry and
 * state data for a motor to be processed by AdvantageKit.
 */
@AutoLog
public class MotorInputs {
    public double velocityUnitsPerSecond = 0.0;
    public double unitPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double rawRotorPosition = 0.0;
    public double currentTemp = 0.0;
}
